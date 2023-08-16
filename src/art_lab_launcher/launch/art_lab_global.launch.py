from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robotDescription = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('robot_description')),
         '/robot_description.launch.py'])
      )

    teensySensors = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('teensy_sensors')),
         '/Teensy.launch.py'])
      )

    roboclaw = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ros2_roboclaw')),
         '/ros2_roboclaw.launch.py'])
      ) 

    # ekf = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([os.path.join(
    #      get_package_share_directory('robot_localization'), 'launch'),
    #      '/ekf.launch.py'])
    #   ) 
    
    ekf = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('robot_localization'), 'launch'),
         '/dual_ekf_navsat_example.launch.py'])
      )   
    
    return LaunchDescription([ 
        robotDescription,
        teensySensors,
        roboclaw,
        ekf
    ])