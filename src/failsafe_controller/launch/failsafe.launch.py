#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='failsafe_controller',
            executable='failsafe_node.py',
            name='failsafe_node',
            output='screen',
            parameters=[{
                'odom_timeout': 2.0,
                'min_update_rate': 5.0,
                'position_jump_threshold': 5.0,
                'warning_time': 3.0,
                'emergency_descent_rate': 0.5,
                'landing_height': 0.3,
                'auto_arm_disarm': True,
            }]
        ),
    ])
