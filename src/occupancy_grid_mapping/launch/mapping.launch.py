#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='occupancy_grid_mapping',
            executable='occupancy_grid_node.py',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{
                'resolution': 0.05,
                'width': 200,
                'height': 200,
                'origin_x': -5.0,
                'origin_y': -5.0,
                'prob_occ': 0.7,
                'prob_free': 0.3,
            }]
        ),
        
        Node(
            package='occupancy_grid_mapping',
            executable='map_merger_node.py',
            name='map_merger_node',
            output='screen',
            parameters=[{
                'merge_method': 'average',
            }]
        ),
    ])
