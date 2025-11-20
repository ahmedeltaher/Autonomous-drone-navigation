#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='false'),
        
        Node(
            package='lidar_slam_2d',
            executable='scan_matcher_node.py',
            name='scan_matcher_node',
            output='screen',
            parameters=[{
                'map_resolution': 0.05,
                'map_size_x': 100.0,
                'map_size_y': 100.0,
            }]
        ),
        
        Node(
            package='lidar_slam_2d',
            executable='map_saver_node.py',
            name='map_saver_node',
            output='screen'
        ),
    ])
