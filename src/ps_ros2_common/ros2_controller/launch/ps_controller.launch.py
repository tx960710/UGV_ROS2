from launch import LaunchDescription
from launch.actions import Shutdown

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch file."""

    joy_node = Node(
        package='joy',
        executable='joy_node',
        on_exit=Shutdown(),
    )

    record_joy = Node(
        package='ps_ros2_common',
        executable='joy_test',
        # output='screen',
        on_exit=Shutdown(),
    )

    ps_controller_node = Node(
            package='ros2_controller',
            executable='ros2_controller',
            output='screen',
            emulate_tty=True,
            parameters=[
                # {"Publish_topic": 'turtle1/cmd_vel'},
                # {"Max_linear_speed": 2.0},        #m/s
                # {"Max_angular_speed": 3.0 }          #rad/s
                
                # #Aion R1
                # {"Publish_topic": '/cmd_vel'},
                # {"Max_linear_speed": 1.225},        #m/s
                # {"Max_angular_speed": 4.0833}          #rad/s

                #Aion R1 slow
                # {"Publish_topic": '/cmd_vel'},
                # {"Max_linear_speed": 0.5},        #m/s
                # {"Max_angular_speed": 0.5}          #rad/s

                #CLAW
                {"Publish_topic": '/cmd_vel'},
                {"Max_linear_speed": 0.31416},        #m/s
                {"Max_angular_speed": 0.62832}          #rad/s

            ]
        )

    return LaunchDescription([
        joy_node,
        record_joy,
        ps_controller_node
    ])
