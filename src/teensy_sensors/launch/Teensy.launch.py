from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="teensy_sensors",
            # namespace='Robot1',
            executable="teensy_sensors",
            name="IMU_GPS",
        ),
    ])
