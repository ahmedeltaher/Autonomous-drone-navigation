#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_file',
            default_value='',
            description='Path to mission YAML file'
        ),
        
        DeclareLaunchArgument(
            'auto_start',
            default_value='false',
            description='Auto-start mission on launch'
        ),
        
        Node(
            package='waypoint_navigation',
            executable='waypoint_manager_node.py',
            name='waypoint_manager_node',
            output='screen',
            parameters=[{
                'mission_file': LaunchConfiguration('mission_file'),
                'waypoint_tolerance': 0.3,
                'auto_start': LaunchConfiguration('auto_start'),
                'loop_mission': False,
            }]
        ),
    ])
