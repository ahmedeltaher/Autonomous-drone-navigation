#!/usr/bin/env python3
"""
Launch file for Optical Flow Integration system

This launch file starts the optical flow node and optionally the simulator
for testing without hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for optical flow integration."""
    
    # Declare launch arguments
    declare_sim_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Use simulated optical flow data instead of real hardware'
    )
    
    declare_update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='20.0',
        description='Optical flow update rate in Hz'
    )
    
    declare_focal_length_arg = DeclareLaunchArgument(
        'focal_length_px',
        default_value='16.0',
        description='Focal length in pixels (PMW3901: 16.0, PX4FLOW: varies)'
    )
    
    declare_min_height_arg = DeclareLaunchArgument(
        'min_height',
        default_value='0.1',
        description='Minimum valid height in meters'
    )
    
    declare_max_height_arg = DeclareLaunchArgument(
        'max_height',
        default_value='5.0',
        description='Maximum valid height in meters'
    )
    
    declare_gyro_comp_arg = DeclareLaunchArgument(
        'use_gyro_compensation',
        default_value='true',
        description='Enable gyroscope compensation for rotation'
    )
    
    declare_publish_odom_arg = DeclareLaunchArgument(
        'publish_odometry',
        default_value='true',
        description='Publish full odometry estimate'
    )
    
    # Optical Flow Node
    optical_flow_node = Node(
        package='optical_flow_integration',
        executable='optical_flow_node.py',
        name='optical_flow_node',
        output='screen',
        parameters=[{
            'update_rate': LaunchConfiguration('update_rate'),
            'focal_length_px': LaunchConfiguration('focal_length_px'),
            'min_height': LaunchConfiguration('min_height'),
            'max_height': LaunchConfiguration('max_height'),
            'use_gyro_compensation': LaunchConfiguration('use_gyro_compensation'),
            'publish_odometry': LaunchConfiguration('publish_odometry'),
        }],
        remappings=[
            ('/optical_flow/velocity', '/sensor_fusion/optical_flow_velocity'),
        ]
    )
    
    # Optical Flow Simulator (conditional)
    optical_flow_simulator = Node(
        package='optical_flow_integration',
        executable='optical_flow_simulator.py',
        name='optical_flow_simulator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('simulation')),
        parameters=[{
            'publish_rate': LaunchConfiguration('update_rate'),
            'focal_length_px': LaunchConfiguration('focal_length_px'),
            'simulated_height': 1.5,
            'height_noise_std': 0.02,
            'flow_noise_std': 0.01,
        }]
    )
    
    return LaunchDescription([
        declare_sim_arg,
        declare_update_rate_arg,
        declare_focal_length_arg,
        declare_min_height_arg,
        declare_max_height_arg,
        declare_gyro_comp_arg,
        declare_publish_odom_arg,
        optical_flow_node,
        optical_flow_simulator,
    ])
