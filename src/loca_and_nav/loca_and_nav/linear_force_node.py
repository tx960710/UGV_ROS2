import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped 
from nav_msgs.msg import Odometry
import math
# from std_msgs.msg import String, Bool, Float64, Int8
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
publish_rate = 10.0		#Hz
F_linear_fix = 0.8
goal_tolerance = 0.3        #m

# set original point of the global coordinate, this should be identical for all robots and should be close to the start point of the robot (within 500 m)
lat_0 = 30.5326515
lon_0 = -96.4216962

# lat/lon to meter linear converter
lat_to_m = hs.haversine((lat_0, lon_0), (lat_0 + 0.001, lon_0), unit=Unit.METERS)*1000.0
lon_to_m = hs.haversine((lat_0, lon_0), (lat_0, lon_0 + 0.001), unit=Unit.METERS)*1000.0
m_to_lat = 1/lat_to_m
m_to_lon = 1/lon_to_m

dt = 1/publish_rate
    

class Linear_force(Node):
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
    # angular_vel_last = 0.0

    def __init__(self):
        super().__init__("Linear_force")	#node name

        self.pubF = self.create_publisher(Vector3Stamped, F_linear_drive_pub_topic, 10)
        self.angle_subscription = self.create_subscription(Odometry, angle_subscript_topic, self.angle_update, 10)
        self.postion_subscription = self.create_subscription(NavSatFix, position_subscript_topic, self.postion_update, 10)
        self.goal_point_subscription = self.create_subscription(NavSatFix, goal_postion_subscript_topic, self.goal_update, 10)
        # self.imu_subscription = self.create_subscription(Imu, "Teensy/IMU", self.imu, 10)


        self.timer = self.create_timer(dt, self.F_pub)  #period time in sec, function name

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
    
    def goal_update(self, msg):
        self.goal_point_flag = True
        self.goal_lat = msg.latitude
        self.goal_lon = msg.longitude


    # def imu(self, msg):
    #     x = msg.orientation.x
    #     y = msg.orientation.y
    #     z = msg.orientation.z
    #     w = msg.orientation.w
    #     ypr = self.quaternion_to_ypr(x, y, z, w)
    #     self.vehicle_angle = ypr[0]
    #     self.F_x = F_linear_fix * np.cos(F_linear_angle - self.vehicle_angle)
    #     self.F_y = F_linear_fix * np.sin(F_linear_angle - self.vehicle_angle)

    # def wrapToPi(self, angle):
    #     # takes an angle as input and calculates its equivalent value within the range of -pi (exclusive) to pi 
    #     wrapped_angle = angle % (2 * math.pi)
    #     if wrapped_angle > math.pi:
    #         wrapped_angle -= 2 * math.pi
    #     return wrapped_angle
    
    def quaternion_to_ypr(self, x, y, z, w):
        # Convert quaternion components to YPR angles
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw, pitch, roll
		

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
