#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamic_filter',
            executable='dynamic_filter_node.py',
            name='dynamic_filter_node',
            output='screen',
            parameters=[{
                'history_length': 10,
                'velocity_threshold': 0.2,
                'confidence_threshold': 0.7,
            }]
        ),
    ])
