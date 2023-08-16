# This is the newer, faster version of the camera subscriber / websocket publisher.  This version does not require saving images every iteration, but publishes them directly.  This is 3 times faster than the previous version.
# Last edited: Feb. 17, 2023 by Annalisa Jarecki

import base64
#import logging
#import time
#import roslibpy

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = CvBridge()

# Configure logging
#fmt = '%(asctime)s %(levelname)8s: %(message)s'
#logging.basicConfig(format=fmt, level=logging.INFO)
#log = logging.getLogger(__name__)

# Identify where the websocket is located (host IP and port)
#client = roslibpy.Ros(host='localhost', port=9090)
#client.run()

# Identify the topic name that will be published to the websocket and the message type
# publisher = roslibpy.Topic(client, '/ra_camera', 'sensor_msgs/CompressedImage')



class CamSubWsPub(Node):

    def __init__(self):
        super().__init__('cam_sub_ws_pub')
        
        self.cam_publisher = self.create_publisher(CompressedImage, 'ra_camera', 10)
        
        # When a message is heard from the ROS2 topic below, it calls the listener_callback function
        # This is where you list what topic to listen to for the image
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #tic = time.time()
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg)
        cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
        is_success, im_buf_arr = cv2.imencode(".jpg", cv2_img)
        # convert image to bytes
        byte_im = im_buf_arr
        # encode image
        encoded = base64.b64encode(byte_im).decode('ascii')
        # publish encoded jpeg to the websocket
        # self.cam_publisher.publish(dict(format='jpeg', data=encoded))
        
        img_msg = CompressedImage()
        img_msg.header.frame_id = "camera_color_optical_frame"
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.format = "jpeg"
        img_msg.data = np.array(byte_im, dtype="uint8").tostring()
        self.cam_publisher.publish(img_msg)
        
        # if you want to save a copy of the jpeg image to your computer, uncomment the line below
        #cv2.imwrite('cam-sub-ws-pub_img.jpeg', cv2_img)
        #toc = time.time()
        # print(toc-tic)
        #time.sleep(0.2)
        
        
def main(args=None):
    rclpy.init(args=args)
    
    camsubwspub = CamSubWsPub()
    rclpy.spin(camsubwspub)
    
    #publisher.unadvertise()
    #client.terminate()



if __name__ == '__main__':
    main()

