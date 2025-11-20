#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory_tracker',
            executable='pure_pursuit_node.py',
            name='pure_pursuit_node',
            output='screen',
            parameters=[{
                'lookahead_distance': 1.0,
                'lookahead_gain': 0.5,
                'min_lookahead': 0.5,
                'max_lookahead': 3.0,
                'linear_velocity': 0.5,
                'max_angular_velocity': 1.0,
                'goal_tolerance': 0.2,
                'path_tolerance': 0.5,
            }]
        ),
    ])
