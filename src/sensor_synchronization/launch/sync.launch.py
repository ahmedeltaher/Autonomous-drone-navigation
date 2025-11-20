#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_synchronization',
            executable='sync_node.py',
            name='sensor_sync_node',
            output='screen',
            parameters=[{
                'sync_tolerance_ms': 50.0,
                'queue_size': 10,
            }]
        ),
        
        Node(
            package='sensor_synchronization',
            executable='time_calibration_node.py',
            name='time_calibration_node',
            output='screen'
        ),
    ])
