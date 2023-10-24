import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped 
from nav_msgs.msg import Odometry
import math
# from std_msgs.msg import String, Bool, Float64, Int8
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix, Imu
# import tf_transformations
# from tf_transformations import euler_from_quaternion
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
import haversine as hs
from haversine import Unit


#declare parameters
F_linear_drive_pub_topic = 'force/linear_drive'
angle_subscript_topic = 'Localization/Odom'
position_subscript_topic = 'Localization/GPS'
goal_postion_subscript_topic = 'goal_point'
auto_mode_subscript_topic = 'auto_mode'
publish_rate = 10.0		#Hz
# F_linear_fix = 0.8
F_linear_fix = 1.2
goal_tolerance = 0.3        #m

# set original point of the global coordinate, this should be identical for all robots and should be close to the start point of the robot (within 500 m)
lat_0 = 30.537253708625634
lon_0 = -96.42643216988164

# lat/lon to meter linear converter
lat_to_m = hs.haversine((lat_0, lon_0), (lat_0 + 0.001, lon_0), unit=Unit.METERS)*1000.0
lon_to_m = hs.haversine((lat_0, lon_0), (lat_0, lon_0 + 0.001), unit=Unit.METERS)*1000.0
m_to_lat = 1/lat_to_m
m_to_lon = 1/lon_to_m

dt = 1/publish_rate
    

class Linear_force(Node):
    AUTO_MODE = 0       # 0: goal point mode, 1: field auto scan mode, should be 0 as default
    F_x = 0.0
    F_y = 0.0
    F_linear_angle = 0.0
    vehicle_angle = 0.0
    vehicle_lat = 0.0
    vehicle_lon = 0.0
    goal_lat = 0.0
    goal_lon = 0.0
    goal_point_flag = False
    localization_fix = False
    d_x = 0.0
    d_y = 0.0
    lat_c0 = 30.537253708625634
    lon_c0 = -96.42643216988164
    lat_c1 = 30.536830918924576
    lon_c1 = -96.4259455772973
    lat_c2 = 30.536753039194434
    lon_c2 = -96.4260354332024
    # lat_c3 = 30.53716125128544
    # lon_c3 = -96.4265536739527
    row_width = 1.5     #m
    row_number = 2
    scout_turn_path = 0     #0: stop, 1: scout row, 2: turn to next row, must be set to 1 when receiving the start flag
    row_count = 0
    P_turn = np.array([0.0, 0.0])
    # angular_vel_last = 0.0
    cmd_last = 10

    def __init__(self):
        super().__init__("Linear_force")	#node name

        self.pubF = self.create_publisher(Vector3Stamped, F_linear_drive_pub_topic, 10)
        self.angle_subscription = self.create_subscription(Odometry, angle_subscript_topic, self.angle_update, 10)
        self.postion_subscription = self.create_subscription(NavSatFix, position_subscript_topic, self.postion_update, 10)
        self.goal_point_subscription = self.create_subscription(NavSatFix, goal_postion_subscript_topic, self.goal_update, 10)
        self.auto_mode_subscription = self.create_subscription(Float64MultiArray, auto_mode_subscript_topic, self.auto_mode_update, 10)
        # self.imu_subscription = self.create_subscription(Imu, "Teensy/IMU", self.imu, 10)

        self.timer = self.create_timer(dt, self.F_pub)  #period time in sec, function name

        self.scout_orientation = np.arctan2((self.lat_c1 - self.lat_c0) * lat_to_m, (self.lon_c1 - self.lon_c0) * lon_to_m)
        self.change_row_orientation = np.arctan2((self.lat_c2 - self.lat_c1) * lat_to_m, (self.lon_c2 - self.lon_c1) * lon_to_m)
        print("Scout orientation: " + str(self.scout_orientation))
        print("Turn orientation: " + str(self.change_row_orientation))

    def F_pub(self):
        if self.goal_point_flag == True and self.localization_fix == True:
            self.F_x = F_linear_fix * np.cos(self.F_linear_angle - self.vehicle_angle)
            self.F_y = F_linear_fix * np.sin(self.F_linear_angle - self.vehicle_angle)
            F = Vector3Stamped()
            F.header.stamp = self.get_clock().now().to_msg()
            F.vector.x = self.F_x
            F.vector.y = self.F_y
            self.pubF.publish(F)
            self.localization_fix = False
        
        elif self.AUTO_MODE == 1 and self.localization_fix == True and self.scout_turn_path != 0:
            self.F_x = F_linear_fix * np.cos(self.F_linear_angle - self.vehicle_angle)
            self.F_y = F_linear_fix * np.sin(self.F_linear_angle - self.vehicle_angle)
            F = Vector3Stamped()
            F.header.stamp = self.get_clock().now().to_msg()
            F.vector.x = self.F_x
            F.vector.y = self.F_y
            self.pubF.publish(F)
            self.localization_fix = False
            if self.row_number == self.row_count:
                self.scout_turn_path = 0
                self.AUTO_MODE = 0
                print("Finish scouting!")
                self.row_count = 0
        else:
            F = Vector3Stamped()
            F.header.stamp = self.get_clock().now().to_msg()
            F.vector.x = 0.0
            F.vector.y = 0.0
            self.pubF.publish(F)

    def angle_update(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        ypr = self.quaternion_to_ypr(x, y, z, w)
        self.vehicle_angle = ypr[0]
        

    def postion_update(self, msg):
        self.localization_fix = True
        self.vehicle_lat = msg.latitude
        self.vehicle_lon = msg.longitude
        if self.goal_point_flag == True:
            self.d_x = (self.goal_lon - self.vehicle_lon) * lon_to_m
            self.d_y = (self.goal_lat - self.vehicle_lat) * lat_to_m
            if np.absolute(self.d_x) < goal_tolerance and np.absolute(self.d_y) < goal_tolerance:
                self.goal_point_flag = False
                print("Reach goal point!")
            else:
                self.F_linear_angle = np.arctan2(self.d_y, self.d_x)
        
        if self.AUTO_MODE == 1:
            P = np.array([(self.vehicle_lon - lon_0)* lon_to_m, (self.vehicle_lat - lat_0)* lat_to_m])      #UGV point
            if self.scout_turn_path == 1:
                if np.mod(self.row_count, 2) == 0:
                    P0 = np.array([(self.lon_c0 - lon_0)* lon_to_m, (self.lat_c0 - lat_0)* lat_to_m])
                    P1 = np.array([(self.lon_c1 - lon_0)* lon_to_m, (self.lat_c1 - lat_0)* lat_to_m])
                    P2 = np.array([(self.lon_c2 - lon_0)* lon_to_m, (self.lat_c2 - lat_0)* lat_to_m])
                    if self.reach_line(P0, P1, P2, P) == False: 
                        self.F_linear_angle = self.scout_orientation
                        # print("Scouting!")
                    else:       # Reach first row end
                        self.row_count += 1
                        self.scout_turn_path = 2
                        self.P_turn = P
                        print("Turning!")
                else:
                    P0 = np.array([(self.lon_c1 - lon_0)* lon_to_m, (self.lat_c1 - lat_0)* lat_to_m])
                    P1 = np.array([(self.lon_c0 - lon_0)* lon_to_m, (self.lat_c0 - lat_0)* lat_to_m])
                    P2 = np.array([(self.lon_c0 - lon_0)* lon_to_m + np.cos(self.change_row_orientation), (self.lat_c0 - lat_0)* lat_to_m + np.sin(self.change_row_orientation)])
                    if self.reach_line(P0, P1, P2, P) == False: 
                        self.F_linear_angle = self.wrapToPi(self.scout_orientation + np.pi)
                        # print("Scouting!")
                    else:       # Reach first row end
                        self.row_count += 1
                        self.scout_turn_path = 2
                        self.P_turn = P
                        print("Turning!")
            elif self.scout_turn_path == 2:
                P0 = self.P_turn
                P1 = self.P_turn + np.array([self.row_width * np.cos(self.change_row_orientation), self.row_width * np.sin(self.change_row_orientation)])
                P2 = P1 + np.array([np.cos(self.scout_orientation), np.sin(self.scout_orientation)])
                if self.reach_line(P0, P1, P2, P) == False: 
                    self.F_linear_angle = self.change_row_orientation
                else:       # Enter next row
                    self.scout_turn_path = 1
                    print("Enter next row!")
                    
    
    def goal_update(self, msg):
        if self.AUTO_MODE == 0:
            if msg.latitude != 0.0 or msg.longitude != 0.0:
                self.goal_point_flag = True
                self.goal_lat = msg.latitude
                self.goal_lon = msg.longitude
            else:
                self.goal_point_flag = False

    def auto_mode_update(self, msg):
        cmd = int(msg.data[0])
        if cmd == 0:
            self.AUTO_MODE = cmd
            self.cmd_last = cmd
            print("Goal point mode!")
        elif cmd == 1:
            if self.cmd_last!= cmd:
                print("Field scouting mode!")
                self.AUTO_MODE = cmd
                self.lat_c0 = msg.data[1]
                self.lon_c0 = msg.data[2]
                self.lat_c1 = msg.data[3]
                self.lon_c1 = msg.data[4]
                self.lat_c2 = msg.data[5]
                self.lon_c2 = msg.data[6]
                self.row_width = msg.data[7]
                # self.row_number = int(msg.data[8])
                self.scout_orientation = np.arctan2((self.lat_c1 - self.lat_c0) * lat_to_m, (self.lon_c1 - self.lon_c0) * lon_to_m)
                self.change_row_orientation = np.arctan2((self.lat_c2 - self.lat_c1) * lat_to_m, (self.lon_c2 - self.lon_c1) * lon_to_m)
                print("Scout orientation: " + str(self.scout_orientation))
                print("Turn orientation: " + str(self.change_row_orientation))
                self.scout_turn_path = 1
                self.cmd_last = cmd
    # def imu(self, msg):
    #     x = msg.orientation.x
    #     y = msg.orientation.y
    #     z = msg.orientation.z
    #     w = msg.orientation.w
    #     ypr = self.quaternion_to_ypr(x, y, z, w)
    #     self.vehicle_angle = ypr[0]
    #     self.F_x = F_linear_fix * np.cos(F_linear_angle - self.vehicle_angle)
    #     self.F_y = F_linear_fix * np.sin(F_linear_angle - self.vehicle_angle)

    def wrapToPi(self, angle):
        # takes an angle as input and calculates its equivalent value within the range of -pi (exclusive) to pi 
        wrapped_angle = angle % (2 * math.pi)
        if wrapped_angle > math.pi:
            wrapped_angle -= 2 * math.pi
        return wrapped_angle
    
    def quaternion_to_ypr(self, x, y, z, w):
        # Convert quaternion components to YPR angles
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw, pitch, roll
    
    def reach_line(self, P0, P1, P2, P):    
        x_vl = P2[0] - P1[0]
        y_vl = P2[1] - P1[1]
        x_vp = P[0] - P1[0] 
        y_vp = P[1] - P1[1]

        x_vp_0 = P0[0] - P1[0]
        y_vp_0 = P0[1] - P1[1]
        if y_vl < 0:
            x_vl = -x_vl
            y_vl = -y_vl
        if self.have_same_sign(x_vp * y_vl - x_vl * y_vp, x_vp_0 * y_vl - x_vl * y_vp_0) == True: 
            return False
        else:
            return True
    
    def have_same_sign(self, a, b):
        if (a >= 0 and b >= 0) or (a < 0 and b < 0):
            return True
        else:
            return False
		

def main():
	rclpy.init()
	my_pub = Linear_force()

	try:
		rclpy.spin(my_pub)
	except KeyboardInterrupt:
		my_pub.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
