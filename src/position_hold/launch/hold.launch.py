#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='position_hold',
            executable='position_hold_node.py',
            name='position_hold_node',
            output='screen',
            parameters=[{
                'kp_xy': 1.0,
                'ki_xy': 0.1,
                'kd_xy': 0.5,
                'kp_z': 1.5,
                'ki_z': 0.2,
                'kd_z': 0.8,
                'max_velocity_xy': 2.0,
                'max_velocity_z': 1.0,
                'max_tilt_angle': 0.5,
                'control_rate': 50.0,
                'hover_throttle': 0.5,
            }]
        ),
    ])
