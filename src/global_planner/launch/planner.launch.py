#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='global_planner',
            executable='astar_planner_node.py',
            name='astar_planner_node',
            output='screen',
            parameters=[{
                'allow_diagonal': True,
                'cost_diagonal': 1.414,
                'cost_straight': 1.0,
                'occupied_threshold': 50,
                'path_smoothing': True,
            }]
        ),
    ])
