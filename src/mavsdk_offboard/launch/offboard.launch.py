#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavsdk_offboard',
            executable='offboard_control_node.py',
            name='offboard_control_node',
            output='screen',
            parameters=[{
                'setpoint_rate': 20.0,
                'auto_arm': False,
                'auto_offboard': False,
                'connection_timeout': 5.0,
            }]
        ),
    ])
