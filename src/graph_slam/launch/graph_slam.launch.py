#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='graph_slam',
            executable='pose_graph_node.py',
            name='pose_graph_node',
            output='screen',
            parameters=[{
                'keyframe_distance': 0.5,
                'loop_closure_distance': 2.0,
                'optimize_frequency': 1.0,
            }]
        ),
        
        Node(
            package='graph_slam',
            executable='loop_closure_node.py',
            name='loop_closure_node',
            output='screen',
            parameters=[{
                'scan_history_size': 100,
                'loop_threshold': 0.8,
            }]
        ),
    ])
