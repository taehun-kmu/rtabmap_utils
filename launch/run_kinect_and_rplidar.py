#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory
    rtabmap_utils_dir = get_package_share_directory('rtabmap_utils')
    rplidar_ros_dir = get_package_share_directory('rplidar_ros')
    
    # ROS2에서는 kinect2_bridge 대신 k4a_ros2 패키지를 사용합니다
    # 이 패키지는 Azure Kinect를 위한 ROS2 드라이버입니다
    # 만약 Kinect2를 계속 사용하려면 freenect2_camera 패키지를 사용할 수 있습니다
    k4a_ros2_dir = get_package_share_directory('k4a_ros2')
    
    # Kinect2 ROS2 Driver
    kinect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(k4a_ros2_dir, 'launch', 'driver.launch.py')
        ),
        launch_arguments={
            'publish_tf': 'true',
        }.items(),
    )
    
    # RPLIDAR A2 Driver
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_ros_dir, 'launch', 'rplidar.launch.py')
        ),
    )
    
    # Static Transforms between the sensors
    static_tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_utils_dir, 'launch', 'static_tfs_kinect2_rplidar.py')
        ),
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(kinect_launch)
    ld.add_action(rplidar_launch)
    ld.add_action(static_tfs_launch)
    
    return ld 