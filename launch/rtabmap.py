#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('rtabmap_utils')
    
    # Create the launch configuration variables
    resolution = LaunchConfiguration('resolution')
    frame_id = LaunchConfiguration('frame_id')
    
    # Declare the launch arguments
    declare_resolution_cmd = DeclareLaunchArgument(
        'resolution',
        default_value='qhd',
        description='Image resolution of the Kinect to process in rtabmap: sd, qhd, hd')
        
    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Fixed frame id')
    
    # SLAM Toolbox node (ROS2 replacement for Hector SLAM)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'base_frame': 'laser',
            'odom_frame': 'odom',
            'use_sim_time': False,
            'publish_map_odom_transform': False,
            'scan_topic': '/scan',
        }],
    )
    
    # RTAB-Map node
    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'subscribe_depth': True,
            'frame_id': frame_id,
            'subscribe_scan': True,
            'cloud_decimation': 2,
            'cloud_max_depth': 5.0,
            'cloud_voxel_size': 0.01,
            'map_cleanup': False,
            'approx_sync': True,
            'Reg/Strategy': '1',  # 0=Visual, 1=ICP, 2=Visual+ICP
            'Vis/MaxDepth': '8.0',
            'Vis/InlierDistance': '0.1',
            'Optimizer/Slam2D': 'true',
            'Reg/Force3DoF': 'true',
        }],
        remappings=[
            ('rgb/image', ['/kinect2/', resolution, '/image_color_rect']),
            ('depth/image', ['/kinect2/', resolution, '/image_depth_rect']),
            ('rgb/camera_info', ['/kinect2/', resolution, '/camera_info']),
            ('scan', '/scan'),
            ('odom', '/scanmatch_odom'),
        ],
        arguments=['--delete_db_on_start'],
    )
    
    # RViz node
    rviz_config_file = os.path.join(package_dir, 'rviz_configs', 'kinect_rtabmap_with_hector.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
    )
    
    # Point cloud node
    points_xyzrgb_node = Node(
        package='rtabmap_ros',
        executable='point_cloud_xyzrgb',
        name='points_xyzrgb',
        parameters=[{
            'voxel_size': 0.01,
        }],
        remappings=[
            ('rgb/image', 'data_odom_sync/image'),
            ('depth/image', 'data_odom_sync/depth'),
            ('rgb/camera_info', 'data_odom_sync/camera_info'),
            ('cloud', 'voxel_cloud'),
        ],
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_resolution_cmd)
    ld.add_action(declare_frame_id_cmd)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rtabmap_node)
    ld.add_action(rviz_node)
    ld.add_action(points_xyzrgb_node)
    
    return ld 