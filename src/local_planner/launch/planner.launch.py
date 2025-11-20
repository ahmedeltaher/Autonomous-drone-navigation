#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='local_planner',
            executable='dwa_planner_node.py',
            name='dwa_planner_node',
            output='screen',
            parameters=[{
                'max_speed': 0.5,
                'min_speed': -0.1,
                'max_yaw_rate': 1.0,
                'max_accel': 0.2,
                'max_dyaw_rate': 1.0,
                'v_resolution': 0.05,
                'yaw_rate_resolution': 0.1,
                'predict_time': 2.0,
                'dt': 0.1,
                'heading_weight': 1.0,
                'clearance_weight': 0.5,
                'velocity_weight': 0.3,
                'robot_radius': 0.3,
                'obstacle_margin': 0.2,
            }]
        ),
    ])
