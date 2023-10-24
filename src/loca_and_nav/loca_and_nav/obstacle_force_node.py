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
enable_camera = True
publish_rate = 10.0		#Hz
K_e = 0.8              #K_e = k * rou * q'
lidar_effective_distance = 1.5        #m
camera_effective_distance = 0.5         #m
vehcle_width = 0.426        #m
vehcle_length = 0.455       #m
lidar_position_x = 0.09     #m
lidar_position_y = 0.0     #m
obstacle_point_num = 360
decay_constant = 2.0

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
camera_height = 0.12        #m

obstacle_point_resolution = np.pi*2 / obstacle_point_num
dt = 1/publish_rate
fx_inv = 1 / fx
fy_inv = 1 / fy
K_e_camera = K_e * fx_inv * fy_inv / (max_obstacle_height - min_obstacle_height)    

class Obstacle_Force(Node):

    # F_x_lidar = 0.0
    # F_y_lidar = 0.0
    lidar_flag = False
    depth_camera_flag = False
    # F_x_cam = 0.0
    # F_y_cam = 0.0
    obstacle_distance_array = np.full((obstacle_point_num, 1), np.inf)       #it is a np.array
    obstacle_distance_cam_array = np.full((obstacle_point_num, 1), np.inf)

    def __init__(self):
        super().__init__("Obstacle_Force")	#node name

        self.pub_F = self.create_publisher(Vector3Stamped, F_obstacle_pub_topic, 10)

        self.lidar_scan_subscription = self.create_subscription(LaserScan, lidar_scan_subscript_topic, self.F_lidar_obstacle, 10)
        self.depth_cam_subscription = self.create_subscription(Image, depth_subscript_topic, self.depth_reading, 10)

        self.timer = self.create_timer(dt, self.force_pub)       #period time in sec, function name

    # Main loop
    def force_pub(self):
        Force = Vector3Stamped()
        Force.header.stamp = self.get_clock().now().to_msg()
        if enable_camera == True:
            if self.depth_camera_flag == True:
                self.depth_camera_flag = False
            else:
                for i in range(obstacle_point_num):
                    decay_distance = self.obstacle_distance_cam_array[i] * decay_constant
                    if decay_distance < self.obstacle_distance_array[i]:   
                        self.obstacle_distance_array[i] = decay_distance
        if self.lidar_flag == True:
            self.lidar_flag = False
            F_x = 0.0
            F_y = 0.0
            for i in range(obstacle_point_num):
                F = -K_e * obstacle_point_resolution / self.obstacle_distance_array[i]
                F = F[0]     # convert 1x1 numpy array to single value
                angle = i * obstacle_point_resolution - np.pi
                F_x += F * np.cos(angle)
                F_y += F * np.sin(angle)
            Force.vector.x = F_x
            Force.vector.y = F_y
            self.pub_F.publish(Force)

        self.obstacle_distance_array = np.full((obstacle_point_num, 1), np.inf)


    # Update lidar data to obstacle array
    def F_lidar_obstacle(self, msg):
        self.lidar_flag = True
        angle = msg.angle_min
        count = 0
        len_count = len(msg.ranges)
        while count < len_count:
            if msg.ranges[count] < lidar_effective_distance:      # msg.ranges[count] is the distance
                if msg.ranges[count] > self.find_boundary_distance(vehcle_width, vehcle_length, lidar_position_x, lidar_position_y, angle):
                    # F = -K_e * msg.angle_increment / msg.ranges[count]
                    # self.F_x_lidar += F * np.cos(angle)
                    # self.F_y_lidar += F * np.sin(angle)
                    angle_num = np.divmod(angle + np.pi, obstacle_point_resolution)
                    # print(angle_num)
                    if int(angle_num[0]) < obstacle_point_num:
                        if msg.ranges[count] < self.obstacle_distance_array[int(angle_num[0])]:
                            self.obstacle_distance_array[int(angle_num[0])] = msg.ranges[count]

            angle += msg.angle_increment
            count += 1
    
    # Update depth camera data to obstacle array
    def depth_reading(self, msg):
        if enable_camera == True:
            self.depth_camera_flag = True
            self.obstacle_distance_cam_array = np.full((obstacle_point_num, 1), np.inf)
            raw_bytes_data = np.array(msg.data, dtype=np.uint8).tobytes()
            depth_data = np.frombuffer(raw_bytes_data, dtype=np.uint16) * depthScale
            for v in range(0, image_height, 10):
                # u = i % image_width     # remainder
                # v = i // image_width    # quotient
                for u in range(0, image_width, 5):
                    x = depth_data[u + v * image_width]
                    y = - (u - cx) * x * fx_inv
                    z = - (v - cy) * x * fy_inv
                    if z + camera_height > min_obstacle_height and z + camera_height < max_obstacle_height:
                        distance = np.sqrt(x**2 + y**2)
                        if distance < camera_effective_distance and distance > 0.0:
                            angle = np.arctan2(y, x) 
                            angle_num = np.divmod(angle + np.pi, obstacle_point_resolution)
                            if int(angle_num[0]) < obstacle_point_num:
                                if distance < self.obstacle_distance_array[int(angle_num[0])]:   
                                    self.obstacle_distance_array[int(angle_num[0])] = distance      # Update obstacle array
                                    self.obstacle_distance_cam_array[int(angle_num[0])] = distance      # store obstacle from depth_cam


    # Calculate the distance between boundary and lidar
    def find_boundary_distance(self, w, l, x, y, angle):
        d = 0.0
        if np.arctan2(-(w/2+y), l/2-x) < angle and angle <= np.arctan2(w/2-y, l/2-x):
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
