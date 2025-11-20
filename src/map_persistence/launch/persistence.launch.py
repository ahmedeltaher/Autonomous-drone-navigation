#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_persistence',
            executable='map_manager_node.py',
            name='map_manager_node',
            output='screen',
            parameters=[{
                'map_directory': 'maps',
                'auto_save': True,
                'save_interval': 60.0,
            }]
        ),
    ])
