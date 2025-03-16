#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('rtabmap_utils')
    
    # Create the launch configuration variables
    model = LaunchConfiguration('model')
    
    # Declare the launch arguments
    declare_model_cmd = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(package_dir, 'urdf', 'pushcart.urdf'),
        description='Path to the URDF model file')
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': {'file': model},
            'publish_frequency': 15.0,
        }],
    )
    
    # Static transform publisher node
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link',
        arguments=['-0.3', '0.0', '-0.425', '0.0', '0.0', '0.0', 'laser', 'base_link'],
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_model_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(static_tf_node)
    
    return ld 