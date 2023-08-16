import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch file."""
    # ns = 'Robot1'
    ns = ''
    nsf = ''

    urdf_file_name = 'R1.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('robot_description'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_description_node = Node(
        namespace = ns,
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output = 'screen',
        arguments=[urdf],
        parameters=[
                {"frame_prefix": nsf}
        ]
    )


    return LaunchDescription([
        robot_description_node,
    ])
