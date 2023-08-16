import rclpy
from .roboclaw_3 import Roboclaw
import struct
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64, Int8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster

address = 0x80

wheel_distance_default = 0.2		#m
wheel_radius_default = 0.1			#m
qpps_default = 2632					#counts
CPR_default = 5264					#counts per revolution
serial_port_default = '/dev/RoboclawCOM'
baudrate_default = 115200
subscript_topic_default = 'cmd_vel'
communication_rate_default = 10		#Hz

class Roboclaw_Driver(Node):
	CPS_left = 0
	CPS_right = 0
	CPS_left_k_1 = 0
	CPS_right_k_1 = 0
	WD = 0

	def __init__(self):
		super().__init__("Roboclaw_Driver")	#node name

		#declare parameters
		self.declare_parameter('Wheel_distance', wheel_distance_default)
		self.declare_parameter('Wheel_radius', wheel_radius_default)
		self.declare_parameter('QPPS', qpps_default)
		self.declare_parameter('CPR', CPR_default)
		self.declare_parameter('Serial_port', serial_port_default)
		self.declare_parameter('Baudrate', baudrate_default)
		self.declare_parameter('Subscript_topic', subscript_topic_default)
		self.declare_parameter('Communication_rate', communication_rate_default)
		
		#get values from parameters
		self.wheel_distance = self.get_parameter('Wheel_distance').get_parameter_value().double_value
		self.wheel_radius = self.get_parameter('Wheel_radius').get_parameter_value().double_value
		self.qpps = self.get_parameter('QPPS').get_parameter_value().integer_value
		self.CPR = self.get_parameter('CPR').get_parameter_value().integer_value
		self.serial_port = self.get_parameter('Serial_port').get_parameter_value().string_value
		self.baudrate_value = self.get_parameter('Baudrate').get_parameter_value().integer_value
		self.subscript_topic = self.get_parameter('Subscript_topic').get_parameter_value().string_value
		self.communication_rate = self.get_parameter('Communication_rate').get_parameter_value().integer_value

		self.speed2CPS = self.CPR/(2*np.pi*self.wheel_radius)
		self.CPS2speed = 1/self.speed2CPS

		self.cur_x = 0.0
		self.cur_y = 0.0
		self.cur_theta = 0.0
		self.last_enc_L = 0
		self.last_enc_R = 0

		#declare topics
		#self.pubGPS = self.create_publisher(NavSatFix, GPS_topic_name, 10)   #msg type, topic name, queue size
		self.pubOdom = self.create_publisher(Odometry, 'Roboclaw/Odom', 10)
		self.cmd_vel_subscription = self.create_subscription(Twist, self.subscript_topic, self.cmd_vel_callback, 10)
		self.timer = self.create_timer(1/self.communication_rate, self.roboclaw_talk)  #period time in sec, function name

		#define tf broadcaster for Odom
		#self.OdomTFBroadcaster = TransformBroadcaster(self)

		#roboclaw serial initialize
		self.get_logger().info('Connecting to the port...')
		self.rc = Roboclaw(self.serial_port,self.baudrate_value)
		self.rc.Open()
		self.get_logger().info('Connect to the port successfully.')
		self.EncM1_init = self.rc.ReadEncM1(address)[1]
		self.EncM2_init = self.rc.ReadEncM2(address)[1]

	def roboclaw_talk(self):
		if self.WD == 0:
			self.rc.SpeedM1(address,0)
			self.rc.SpeedM2(address,0)
		else:
			if self.CPS_left != self.CPS_left_k_1 or self.CPS_right != self.CPS_right_k_1:
				if np.absolute(self.CPS_left) < self.qpps:
					self.rc.SpeedM2(address,int(self.CPS_left))
				elif self.CPS_left > 0:
					self.rc.SpeedM2(address,self.qpps)
				else:
					self.rc.SpeedM2(address,-self.qpps)

				if np.absolute(self.CPS_right) < self.qpps:
					self.rc.SpeedM1(address,int(self.CPS_right))
				elif self.CPS_right > 0:
					self.rc.SpeedM1(address,self.qpps)
				else:
					self.rc.SpeedM1(address,-self.qpps)
			self.CPS_left_k_1 = self.CPS_left
			self.CPS_right_k_1 = self.CPS_right
			self.WD = self.WD - 1

		enc_L = self.rc.ReadEncM2(address)[1] - self.EncM2_init
		enc_R = self.rc.ReadEncM1(address)[1] - self.EncM1_init
		enc_speed_L = self.rc.ReadSpeedM2(address)[1]
		enc_speed_R = self.rc.ReadSpeedM1(address)[1]
		left_ticks = enc_L - self.last_enc_L
		right_ticks = enc_R - self.last_enc_R
		self.last_enc_L = enc_L
		self.last_enc_R = enc_R			

		#odom publisher write here:
		if np.absolute(left_ticks) < 500000 and np.absolute(right_ticks) < 500000:				
			left_distance = left_ticks * self.CPS2speed
			right_distance = right_ticks * self.CPS2speed
			dist = (left_distance + right_distance)/2.0
			left_speed = enc_speed_L * self.CPS2speed
			right_speed = enc_speed_R * self.CPS2speed
			vel_body = (left_speed +  right_speed)/ 2.0
			vel_theta = (right_speed - left_speed) / self.wheel_distance
			
			if left_ticks == right_ticks:
				d_theta = 0.0
				self.cur_x += dist * np.cos(self.cur_theta)
				self.cur_y += dist * np.sin(self.cur_theta)
			else:
				d_theta = (right_distance - left_distance) / self.wheel_distance
				r = dist / d_theta
				self.cur_x += r * (np.sin(d_theta + self.cur_theta) - np.sin(self.cur_theta))
				self.cur_y -= r * (np.cos(d_theta + self.cur_theta) - np.cos(self.cur_theta))
				self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

			q = tf_transformations.quaternion_from_euler(0, 0, self.cur_theta)								
			odom = Odometry()
			odom.header.stamp = self.get_clock().now().to_msg()
			odom.header.frame_id = "odom"
			odom.child_frame_id = 'base_link'
			odom.pose.pose.position.x = self.cur_x
			odom.pose.pose.position.y = self.cur_y
			odom.pose.pose.position.z = 0.0
			odom.pose.pose.orientation.x = q[0]
			odom.pose.pose.orientation.y = q[1]
			odom.pose.pose.orientation.z = q[2]
			odom.pose.pose.orientation.w = q[3]
			odom.twist.twist.linear.x = vel_body
			odom.twist.twist.angular.z = vel_theta
			self.pubOdom.publish(odom)

			#Test tf2, if you don't need tf2 just comment the following code
			# t = TransformStamped()
			# t.header.stamp = odom.header.stamp
			# t.header.frame_id = 'odom'
			# t.child_frame_id = 'base_link'
			# t.transform.translation.x = self.cur_x
			# t.transform.translation.y = self.cur_y
			# t.transform.translation.z = 0.0
			# t.transform.rotation.x = q[0]
			# t.transform.rotation.y = q[1]
			# t.transform.rotation.z = q[2]
			# t.transform.rotation.w = q[3]
			# # Send the transformation			
			# self.OdomTFBroadcaster.sendTransform(t)


	def cmd_vel_callback(self, vel_msg):
		linear_speed = vel_msg.linear.x
		angular_speed = vel_msg.angular.z

		#safety wathchdog, if there is no cmd_vel for 1 s, the motors will be stopped
		self.WD = self.communication_rate + 1

		#speed-transfer equation
		speed_right = (angular_speed * self.wheel_distance)/2 + linear_speed
		speed_left = linear_speed * 2 - speed_right
		self.CPS_left = speed_left * self.speed2CPS
		self.CPS_right = speed_right * self.speed2CPS

	def normalize_angle(self, angle):
		while angle > np.pi:
			angle -= 2.0 * np.pi
		while angle < -np.pi:
			angle += 2.0 * np.pi
		return angle

def main():
	rclpy.init()
	my_pub = Roboclaw_Driver()

	try:
		rclpy.spin(my_pub)
	except KeyboardInterrupt:
		my_pub.rc.SpeedM1(address,0)
		my_pub.rc.SpeedM2(address,0)
		my_pub.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
