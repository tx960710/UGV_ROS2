from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_roboclaw',
            # namespace='Robot1',
            executable='roboclaw',
            name='roboclaw_node',           
            parameters=[
                {"Serial_port": '/dev/RoboclawCOM'},
                {"Baudrate": 115200},
                {"Subscript_topic": 'cmd_vel'},
                {"Communication_rate": 10},     #Hz
                #Aion R1
                {"Wheel_distance": 0.6},        #m
                {"Wheel_radius": 0.078},        #m
                {"QPPS": 2800},                 #counts
                {"CPR": 1120},                  #counts per revolution
                
                #CLAW
                #{"Wheel_distance": 1.0},        #m
                #{"Wheel_radius": 0.1},          #m
                #{"QPPS": 2632},                 #counts
                #{"CPR": 5264},                  #counts per revolution
            ]
        ),

    ])
