#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Static transform publisher node for kinect2 to rplidar
    static_tf_kinect2_rplidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_kinect2_rplidar',
        arguments=['0.125', '0.09', '-0.08', '-1.5', '0.0', '-1.34', 'laser', 'kinect2_link'],
    )
    
    # Static transform publisher node for kinect2 inverted link
    static_tf_kinect2_inverted_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_kinect2_inverted_link',
        arguments=['0.0', '0.0', '0.0', '1.57', '-1.57', '0.0', 'kinect2_link', 'kinect2_laser_link'],
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(static_tf_kinect2_rplidar_node)
    ld.add_action(static_tf_kinect2_inverted_link_node)
    
    return ld 