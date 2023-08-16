import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image

# from std_msgs.msg import String, Bool, Float64, Int8
# from sensor_msgs.msg import NavSatFix, Imu
# import tf_transformations
# from tf_transformations import euler_from_quaternion
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# import haversine as hs
# from haversine import Unit
# import math


#declare parameters
F_obstacle_pub_topic = 'force/obstacle'
lidar_scan_subscript_topic = 'scan'
depth_subscript_topic = "camera/aligned_depth_to_color/image_raw"
enable_camera = False
publish_rate = 10.0		#Hz
K_e = 0.8              #K_e = k * rou * q'
effective_distance = 1.5        #m
vehcle_width = 0.426        #m
vehcle_length = 0.455       #m
lidar_position_x = 0.09     #m
lidar_position_y = 0.0     #m

# camera info
depthScale = 0.001      #m
image_width = 640
image_height = 480
cx = 317.3588562011719
cy = 237.2884521484375
fx = 380.35723876953125
fy = 380.35723876953125
min_obstacle_height = 0.05      #m
max_obstacle_height = 0.4       #m

dt = 1/publish_rate
fx_inv = 1 / fx
fy_inv = 1 / fy
K_e_camera = K_e * fx_inv * fy_inv / (max_obstacle_height - min_obstacle_height)    

class Obstacle_Force(Node):

    F_x_lidar = 0.0
    F_y_lidar = 0.0
    lidar_flag = False
    depth_camera_flag = False
    F_x_cam = 0.0
    F_y_cam = 0.0

    def __init__(self):
        super().__init__("Obstacle_Force")	#node name

        self.pub_F = self.create_publisher(Vector3Stamped, F_obstacle_pub_topic, 10)

        self.lidar_scan_subscription = self.create_subscription(LaserScan, lidar_scan_subscript_topic, self.F_lidar_obstacle, 10)
        if enable_camera == True:
            self.depth_cam_subscription = self.create_subscription(Image, depth_subscript_topic, self.depth_reading, 10)

        self.timer = self.create_timer(dt, self.force_pub)       #period time in sec, function name

    def force_pub(self):
        Force = Vector3Stamped()
        Force.header.stamp = self.get_clock().now().to_msg()
        if self.lidar_flag == True:
            self.lidar_flag = False
            Force.vector.x = self.F_x_lidar
            Force.vector.y = self.F_y_lidar
            if enable_camera == True and self.depth_camera_flag == True:
                self.depth_camera_flag == False
                Force.vector.x += self.F_x_cam
                Force.vector.y += self.F_y_cam
            self.pub_F.publish(Force)

        self.F_x_lidar = 0.0
        self.F_y_lidar = 0.0
        self.F_x_cam = 0.0
        self.F_y_cam = 0.0

    def F_lidar_obstacle(self, msg):
        self.lidar_flag = True
        angle = msg.angle_min
        count = 0
        len_count = len(msg.ranges)
        while count < len_count:
            if msg.ranges[count] < effective_distance:
                if msg.ranges[count] > self.find_boundary_distance(vehcle_width, vehcle_length, lidar_position_x, lidar_position_y, angle):
                    F = -K_e * msg.angle_increment / msg.ranges[count]
                    self.F_x_lidar += F * np.cos(angle)
                    self.F_y_lidar += F * np.sin(angle)

            angle += msg.angle_increment
            count += 1
    
    def depth_reading(self, msg):
        self.depth_camera_flag = True
        raw_bytes_data = np.array(msg.data, dtype=np.uint8).tobytes()
        depth_data = np.frombuffer(raw_bytes_data, dtype=np.uint16) * depthScale
        for v in range(image_height):
            # u = i % image_width     # remainder
            # v = i // image_width    # quotient
            for u in range(image_width):
                x = depth_data[u + v * image_width]
                y = - (u - cx) * x * fx_inv
                z = - (v - cy) * x * fy_inv
                if z > min_obstacle_height and z < max_obstacle_height:
                    d = np.sqrt(x**2 + y**2)
                    if d < effective_distance and d > 0.0:
                        angle = np.arctan2(y, x) 
                        F = -K_e_camera * (np.cos(angle))**2
                        self.F_x_cam += F * np.cos(angle)
                        self.F_y_cam += F * np.sin(angle)


    def find_boundary_distance(self, w, l, x, y, angle):
        d = 0.0
        if np.arctan2(-(w/2+y), l/2-x) < angle and angle <= np.arctan2(w/2-y, l/2-x):
            if self.depth_camera_flag == False:
                d = (l/2 - x) / np.cos(angle)
        elif np.arctan2(w/2-y, l/2-x) < angle and angle <= np.arctan2(w/2-y, -(l/2+x)):
            d = (w/2 - y) / np.sin(angle)
        elif np.arctan2(w/2-y, l/2-x) < angle and angle <= np.pi:
            d = -(l/2 + x) / np.cos(angle)
        elif -np.pi < angle and angle <= np.arctan2(-(w/2+y), -(l/2+x)):
            d = -(l/2 + x) / np.cos(angle) 
        elif np.arctan2(-(w/2+y), -(l/2+x)) < angle and angle <= np.arctan2(-(w/2+y), l/2-x):
            d = -(w/2 + y) / np.sin(angle) 
        return d

def main():
	rclpy.init()
	my_pub = Obstacle_Force()

	try:
		rclpy.spin(my_pub)
	except KeyboardInterrupt:
		my_pub.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
