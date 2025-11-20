#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_pose_estimator',
            executable='vision_pose_node.py',
            name='vision_pose_node',
            output='screen',
            parameters=[{
                'publish_rate': 30.0,
                'frame_id': 'map',
                'child_frame_id': 'base_link',
                'use_odom': True,
                'use_pose': False,
                'position_covariance': 0.01,
                'orientation_covariance': 0.01,
            }]
        ),
    ])
