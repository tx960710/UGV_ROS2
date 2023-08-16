from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():             
    obstacle_force = Node(
                   package="loca_and_nav",
                   executable="obstacle_force"
               )
    
    linear_force = Node(
                   package="loca_and_nav",
                   executable="linear_force"
               )
    
    speed_pub = Node(
                   package="loca_and_nav",
                   executable="speed_pub"
               )
     
    
    return LaunchDescription([ 
        obstacle_force,
        linear_force,
        speed_pub
    ])
