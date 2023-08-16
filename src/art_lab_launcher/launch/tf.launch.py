from launch import LaunchDescription
from launch_ros.actions import Node

input_namespace = "/R1"

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", input_namespace + "/base_link_frame", input_namespace + "/imu_frame"],
        ),
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "map", input_namespace + "/odom_frame"],
        ),
    ])