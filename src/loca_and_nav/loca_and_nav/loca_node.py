import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64, Int8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
import tf_transformations
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import haversine as hs
from haversine import Unit
import math


#declare parameters
# compass_var = (math.pi*1.5/180)**2
# sigma_a = 10    # m/s^2
# sigma_phi_dot = [10, 0.01, 0.01, 0.01]
vel_var = 0.25

odom_subscript_topic = 'Roboclaw/Odom'
gps_subscript_topic = 'Teensy/GPS'
gps2_subscript_topic = 'ReachM2/GPS'
imu_subscript_topic = 'Teensy/IMU'
publish_rate = 10		#Hz
fix_datum = False
datum_lat = 30.0
datum_lon = -90.0
datum_yaw = 0
pub_odom_tf = True

# set original point of the global coordinate, this should be identical for all robots and should be close to the start point of the robot (within 500 m)
# Farm
# lat_0 = 30.537253708625634
# lon_0 = -96.42643216988164

# TAMU CS
lat_0 = 30.6126599
lon_0 = -96.3431303

# lat/lon to meter linear converter
lat_to_m = hs.haversine((lat_0, lon_0), (lat_0 + 0.001, lon_0), unit=Unit.METERS)*1000.0
lon_to_m = hs.haversine((lat_0, lon_0), (lat_0, lon_0 + 0.001), unit=Unit.METERS)*1000.0
m_to_lat = 1/lat_to_m
m_to_lon = 1/lon_to_m

x_0 = (datum_lon - lon_0)*lon_to_m
y_0 = (datum_lat - lat_0)*lat_to_m
x_init = np.matrix([[x_0], 
                    [y_0], 
                    [0],
                    [datum_yaw]])
P_init = np.matrix([[0, 0, 0, 0], 
                    [0, 0, 0, 0], 
                    [0, 0, 0, 0],
                    [0, 0, 0, 0]])
    
    

class Localization(Node):
	
    x_k = x_init
    P_k = P_init
    pos_init_status = False
    angle_init_status = False

    t_k = 0.0
    t_k_1 = 0.0

    def __init__(self):
        super().__init__("Localization")	#node name

        self.pubOdom = self.create_publisher(Odometry, 'Localization/Odom', 10)
        self.pubGPS = self.create_publisher(NavSatFix, 'Localization/GPS', 10)
        #define tf broadcaster for Odom
        self.OdomTFBroadcaster = TransformBroadcaster(self)

        self.odom_vel_subscription = self.create_subscription(Odometry, odom_subscript_topic, self.vel_update, 10)
        self.gps_subscription = self.create_subscription(NavSatFix, gps_subscript_topic, self.gps_update, 10)
        self.gps2_subscription = self.create_subscription(NavSatFix, gps2_subscript_topic, self.gps2_update, 10)
        self.imu_subscription = self.create_subscription(Imu, imu_subscript_topic, self.imu_update, 10)

        self.timer = self.create_timer(1/publish_rate, self.loca_pub)  #period time in sec, function name

        if fix_datum == True:
            self.x_k = x_init
            self.P_k = P_init
            self.pos_init_status = True
            self.angle_init_status = True
        else:
            self.pos_init_status = False
            self.angle_init_status = False

    def loca_pub(self):
        if self.pos_init_status == True and self.angle_init_status == True:
            # publish the position and filtered GPS here
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x_k[0, 0]
            odom.pose.pose.position.y = self.x_k[1, 0]
            odom.pose.covariance[0] = self.P_k[0, 0]
            odom.pose.covariance[1] = self.P_k[0, 1]
            odom.pose.covariance[6] = self.P_k[1, 0]
            odom.pose.covariance[7] = self.P_k[1, 1]
            q = tf_transformations.quaternion_from_euler(0, 0, self.x_k[3, 0])
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            odom.pose.covariance[35] = self.P_k[3, 3]
            odom.twist.twist.linear.x = self.x_k[2, 0]
            odom.twist.covariance[0] = self.P_k[2, 2]
            self.pubOdom.publish(odom)

            filtered_gps = NavSatFix()
            filtered_gps.header.stamp = self.get_clock().now().to_msg()
            filtered_gps.header.frame_id = "filtered_gps"
            filtered_gps.status.status = 0
            filtered_gps.latitude = self.x_k[1, 0] * m_to_lat + lat_0
            filtered_gps.longitude = self.x_k[0, 0] * m_to_lon + lon_0
            filtered_gps.position_covariance[0] = self.P_k[0, 0]
            filtered_gps.position_covariance[1] = self.P_k[0, 1]
            filtered_gps.position_covariance[3] = self.P_k[1, 0]
            filtered_gps.position_covariance[4] = self.P_k[1, 1]
            self.pubGPS.publish(filtered_gps)


            if pub_odom_tf == True:
                t = TransformStamped()
                t.header.stamp = odom.header.stamp
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = self.x_k[0, 0]
                t.transform.translation.y = self.x_k[1, 0]
                t.transform.translation.z = 0.0
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                # Send the transformation
                self.OdomTFBroadcaster.sendTransform(t)


    def imu_update(self, msg):
        self.t_k = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = self.t_k - self.t_k_1
        self.t_k_1 = self.t_k
        if self.angle_init_status == False:
            if msg.orientation_covariance[8] < math.pi * 0.1:
            # if True:
                angle = self.quaternion_to_ypr(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)        #yaw pitch roll 
                yaw = angle[0]   
                self.x_k[3, 0] = yaw
                self.P_k[3, 3] = msg.orientation_covariance[8]
                self.angle_init_status = True
                print('The initial angle was updated by the IMU successfully.')
        elif self.pos_init_status == True:
            # Update prediction
            a_k = 0
            vel = self.x_k[2, 0]
            phi = self.x_k[3,0]
            phi_dot_k = msg.angular_velocity.z
            prediction = np.matrix([[dt*vel*math.cos(phi)], 
                                    [dt*vel*math.sin(phi)], 
                                    [dt*a_k],
                                    [dt*phi_dot_k]])
            Jacobia_f_x = np.matrix([[1, 0, dt*math.cos(phi), -dt*vel*math.sin(phi)], 
                                     [0, 1, dt*math.sin(phi), dt*vel*math.cos(phi)], 
                                     [0, 0, 1, 0],
                                     [0, 0, 0, 1]])
            Q = np.matrix([[0, 0, 0, 0], 
                           [0, 0, 0, 0], 
                           [0, 0, (dt*msg.linear_acceleration_covariance[0])**2, 0],
                           [0, 0, 0, (dt*msg.angular_velocity_covariance[8])**2]])
                        #    [0, 0, 0, (dt*0.01)**2]])
            self.x_k = self.x_k + prediction
            self.x_k[3,0] = self.wrapToPi(self.x_k[3,0])
            self.P_k = Jacobia_f_x*self.P_k*Jacobia_f_x.T + Q

            # Compass LKF 
            if msg.orientation_covariance[8] < math.pi * 0.1:
                H = np. matrix([0, 0, 0, 1])
                R = msg.orientation_covariance[8]
                angle = self.quaternion_to_ypr(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)        #yaw pitch roll
                y = angle[0] 
                y_tilde = y - self.x_k[3,0]
                y_tilde = self.wrapToPi(y_tilde)
                S = self.P_k[3, 3] + R
                K = self.P_k*H.T/S

                self.x_k += K*y_tilde
                self.x_k[3,0] = self.wrapToPi(self.x_k[3,0])
                self.P_k = (np.eye(4) - K*H)*self.P_k
                # print('The compass angle is:', y)

            

    def vel_update(self, msg):
        # Update velocity LKF from encoder here
        H = np. matrix([0, 0, 1, 0])
        R = vel_var
        y = msg.twist.twist.linear.x
        y_tilde = y - self.x_k[2,0]
        S = self.P_k[2, 2] + R
        K = self.P_k*H.T/S

        self.x_k += K*y_tilde
        self.P_k = (np.eye(4) - K*H)*self.P_k
    
    def gps_update(self, msg):
        if self.pos_init_status == False:
            if msg.latitude != 0.0 or msg.longitude != 0.0:
                self.x_k[0, 0] = (msg.longitude - lon_0)*lon_to_m
                self.x_k[1, 0] = (msg.latitude - lat_0)*lat_to_m
                self.P_k[0, 0] = msg.position_covariance[0]
                self.P_k[1, 1] = msg.position_covariance[4]
                self.pos_init_status = True
                print('The initial position was updated by the GPS successfully.')
        elif self.angle_init_status == True:
            # Update GPS LKF here
            H = np. matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])
            R = np. matrix([[msg.position_covariance[0], 0],
                            [0, msg.position_covariance[4]]])
            # R = np. matrix([[6.25, 0],
            #                 [0, 6.25]])
            y = np. matrix([[(msg.longitude - lon_0)*lon_to_m],
                            [(msg.latitude - lat_0)*lat_to_m]])
            y_tilde = y - H*self.x_k
            S = H * self.P_k * H.T + R
            K = self.P_k * H.T * S.I

            self.x_k += K*y_tilde
            self.P_k = (np.eye(4) - K*H)*self.P_k   

    def gps2_update(self, msg):   
        H = np. matrix([[1, 0, 0, 0],
                        [0, 1, 0, 0]])
        R = np. matrix([[msg.position_covariance[0], 0],
                        [0, msg.position_covariance[4]]])
        y = np. matrix([[(msg.longitude - lon_0)*lon_to_m],
                        [(msg.latitude - lat_0)*lat_to_m]])
        y_tilde = y - H*self.x_k
        S = H * self.P_k * H.T + R
        K = self.P_k * H.T * S.I

        self.x_k += K*y_tilde
        self.P_k = (np.eye(4) - K*H)*self.P_k

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
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw, pitch, roll
		

def main():
	rclpy.init()
	my_pub = Localization()

	try:
		rclpy.spin(my_pub)
	except KeyboardInterrupt:
		my_pub.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
