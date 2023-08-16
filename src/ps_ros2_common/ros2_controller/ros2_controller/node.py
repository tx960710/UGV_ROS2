import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

max_linear_speed_default = 0.1      #m/s
max_angular_speed_default = 0.5     #rad/s
publish_topic_default = 'cmd_vel'

class ControllerPublisher(Node):

    linear_speed = 0.0
    angular_speed = 0.0
    current_speed = 0.0
    current_angular_speed = 0.0
    delta_v = 0.0
    delta_w = 0.0
    last_joy_7 = 0

    def __init__(self):
        super().__init__('controller_node')

        #declare parameters
        self.declare_parameter('Max_linear_speed', max_linear_speed_default)
        self.declare_parameter('Max_angular_speed', max_angular_speed_default)
        self.declare_parameter('Publish_topic', publish_topic_default)

        #get values from parameters
        self.max_linear_speed = self.get_parameter('Max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('Max_angular_speed').get_parameter_value().double_value
        self.publish_topic = self.get_parameter('Publish_topic').get_parameter_value().string_value

        #declare topics
        self.vel_pub = self.create_publisher(Twist, self.publish_topic, 10)
        timer_period = 0.1  # seconds, 10 Hz
        self.controller_subscription = self.create_subscription(Joy, 'joy', self.controller_callback, 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.current_speed = self.max_linear_speed*0.5
        self.current_angular_speed = self.max_angular_speed*0.5 
        self.delta_v = self.max_linear_speed*0.1
        self.delta_w = self.max_angular_speed*0.1

    def controller_callback(self, joy_msg):
        # Up button
        if joy_msg.axes[7] == 1 and self.last_joy_7 == 0:
            self.current_speed += self.delta_v
            self.current_angular_speed += self.delta_w            
        # Down button
        elif joy_msg.axes[7] == -1 and self.last_joy_7 == 0:
            self.current_speed -= self.delta_v
            self.current_angular_speed -= self.delta_w
        

        # Speed limits    
        if self.current_speed > self.max_linear_speed:
            self.current_speed = self.max_linear_speed
            self.current_angular_speed = self.max_angular_speed
        elif self.current_speed < self.delta_v:
            self.current_speed = self.delta_v
            self.current_angular_speed = self.delta_w

        if self.last_joy_7 != joy_msg.axes[7] and self.last_joy_7 != 0:
            print("Current linear and angular speed is %s m/s and %s rad/s" %(self.current_speed, self.current_angular_speed))

        self.linear_speed = float(self.current_speed * joy_msg.axes[1])       #Joystick left y
        self.angular_speed = float(self.current_angular_speed * joy_msg.axes[3])     #Joystick right x	

        self.last_joy_7 = joy_msg.axes[7]

    def timer_callback(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.linear_speed
        vel_msg.angular.z = self.angular_speed
        self.vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init()
    my_pub = ControllerPublisher()

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()