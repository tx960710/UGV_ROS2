import rclpy
import numpy as np
from rclpy.node import Node
# from geometry_msgs.msg import Twist, Vector3Stamped 
# from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

#temp
K_e = 0.8              #K_e = k * rou * q'
effective_distance = 1.5
#temp

depth_subscript_topic = "camera/aligned_depth_to_color/image_raw"
depthScale = 0.001      #m
image_width = 640
image_height = 480
cx = 325.5
cy = 253.5
fx = 518.0
fy = 519.0
min_obstacle_height = 0.03      #m
max_obstacle_height = 0.5       #m

publish_rate = 1.0		#Hz

dt = 1/publish_rate
fx_inv = 1 / fx
fy_inv = 1 / fy
K_e_camera = K_e * fx_inv * fy_inv / (max_obstacle_height - min_obstacle_height)

class Depth_Detect(Node):
    depth_camera_flag = False
    F_x_cam = 0.0
    F_y_cam = 0.0

    def __init__(self):
        super().__init__("Depth_Detect")	#node name
        self.depth_cam_subscription = self.create_subscription(Image, depth_subscript_topic, self.depth_reading, 10)
        # self.motor_speed_subscription = self.create_subscription(Odometry, motor_speed_subscript_topic, self.motor_speed_update, 10)

        self.timer = self.create_timer(dt, self.check)       #period time in sec, function name

    def check(self):
        if self.depth_camera_flag == True:
            print([self.F_x_cam, self.F_y_cam])
            self.depth_camera_flag = False
        self.F_x_cam = 0
        self.F_y_cam = 0
    
    def depth_reading(self, msg):
        self.depth_camera_flag = True
        raw_bytes_data = np.array(msg.data, dtype=np.uint8).tobytes()
        depth_data = np.frombuffer(raw_bytes_data, dtype=np.uint16) * depthScale
        for i in range(len(depth_data)):
            u = i % image_width     # remainder
            v = i // image_width    # quotient
            x = depth_data[i]
            y = - (u - cx) * x * fx_inv
            z = - (v - cy) * x * fy_inv
            if z > min_obstacle_height and z < max_obstacle_height:
                d = np.sqrt(x**2 + y**2)
                if d < effective_distance and d > 0.0:
                    angle = np.arctan2(y, x) 
                    F = -K_e_camera * (np.cos(angle))**2
                    self.F_x_cam += F * np.cos(angle)
                    self.F_y_cam += F * np.sin(angle)
                    


def main():
	rclpy.init()
	my_pub = Depth_Detect()

	try:
		rclpy.spin(my_pub)
	except KeyboardInterrupt:
		my_pub.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
