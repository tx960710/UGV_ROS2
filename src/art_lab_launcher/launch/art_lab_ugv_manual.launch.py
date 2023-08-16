from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


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

    #ekf = IncludeLaunchDescription(
    #  PythonLaunchDescriptionSource([os.path.join(
    #     get_package_share_directory('robot_localization'), 'launch'),
    #     '/ekf.launch.py'])
    #  ) 
    
    # ekf = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([os.path.join(
    #      get_package_share_directory('robot_localization'), 'launch'),
    #      '/dual_ekf_navsat_example.launch.py'])
    #   )   

    lidar = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rplidar_ros'), 'launch'),
         '/rplidar.launch.py'])
      ) 

    #slam = IncludeLaunchDescription(
    #  PythonLaunchDescriptionSource([os.path.join(
    #     get_package_share_directory('slam_toolbox'), 'launch'),
    #     '/offline_launch.py'])
    #     #launch_arguments={
    #     #   'base_frame': 'base_link',
    #     #   'transform_publish_period': '0.1'
    #     #}.items()
    #  )
      
    #nav2 = IncludeLaunchDescription(
    #  PythonLaunchDescriptionSource([os.path.join(
    #     get_package_share_directory('nav2_bringup'), 'launch'),
    #     '/navigation_launch.py']),
    #     launch_arguments={
    #        'use_sim_time': 'False',
    #        #'params_file': '~/Workspaces/art_lab_test_ws/src/art_lab_launcher/params/nav2_params.yaml'
    #     }.items()
    #  )
      
    #K. Lee's code
    realsense_cam = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('realsense_ros2_camera'), 'launch'),
         '/ros2_intel_realsense.launch.py'])
      ) 
    
    # Realsense official code
    #realsense_cam = IncludeLaunchDescription(
    #  PythonLaunchDescriptionSource([os.path.join(
    #    get_package_share_directory('realsense2_camera'), 'launch'),
    #     '/rs_launch.py'])
    #  )  
    
    Reach_M2 = Node(
                   package="Reach_Device",
                   executable="ReachM2"
               )
               
    compressed_image = Node(
                   package="image_processing",
                   executable="compressed_image"
               )
    
    web_socket = IncludeLaunchDescription(
      XMLLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rosbridge_server'), 'launch'),
         '/rosbridge_websocket_launch.xml'])
      ) 
    
    localization_ekf = Node(
                   package="loca_and_nav",
                   executable="localization",
                   output='screen',
                   emulate_tty=True
               )
     
    
    return LaunchDescription([ 
        robotDescription,
        teensySensors,
        roboclaw,
        # ekf,
        lidar,
        # slam,
        # nav2,
        realsense_cam,
        # Reach_M2,
        compressed_image,
        localization_ekf,
        web_socket
    ])
