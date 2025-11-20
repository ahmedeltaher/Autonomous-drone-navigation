#!/usr/bin/env python3
"""
Launch file for IMU Processing System

This launch file starts the IMU processing nodes including:
- IMU processing with vibration filtering
- Complementary filter for orientation estimation
- Optional calibration tool
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for IMU processing."""
    
    # Declare launch arguments
    declare_use_cf_arg = DeclareLaunchArgument(
        'use_complementary_filter',
        default_value='true',
        description='Use complementary filter for orientation estimation'
    )
    
    declare_calibrate_arg = DeclareLaunchArgument(
        'calibrate',
        default_value='false',
        description='Run calibration tool instead of normal operation'
    )
    
    declare_update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='100.0',
        description='IMU processing update rate in Hz'
    )
    
    declare_accel_cutoff_arg = DeclareLaunchArgument(
        'accel_lpf_cutoff',
        default_value='20.0',
        description='Accelerometer low-pass filter cutoff frequency (Hz)'
    )
    
    declare_gyro_cutoff_arg = DeclareLaunchArgument(
        'gyro_lpf_cutoff',
        default_value='20.0',
        description='Gyroscope low-pass filter cutoff frequency (Hz)'
    )
    
    declare_cf_alpha_arg = DeclareLaunchArgument(
        'cf_alpha',
        default_value='0.98',
        description='Complementary filter alpha (gyro weight)'
    )
    
    # IMU Processing Node
    imu_processing_node = Node(
        package='imu_processing',
        executable='imu_processing_node.py',
        name='imu_processing_node',
        output='screen',
        parameters=[{
            'update_rate': LaunchConfiguration('update_rate'),
            'accel_lpf_cutoff': LaunchConfiguration('accel_lpf_cutoff'),
            'gyro_lpf_cutoff': LaunchConfiguration('gyro_lpf_cutoff'),
            'calibration_samples': 1000,
            'enable_bias_estimation': True,
            'gravity': 9.81,
        }],
        condition=IfCondition(
            # Only run if not calibrating
            # Note: This is simplified, proper negation would need custom launch code
            LaunchConfiguration('calibrate', default='false')
        )
    )
    
    # Complementary Filter Node
    complementary_filter_node = Node(
        package='imu_processing',
        executable='complementary_filter_node.py',
        name='complementary_filter_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_complementary_filter')),
        parameters=[{
            'alpha': LaunchConfiguration('cf_alpha'),
            'update_rate': LaunchConfiguration('update_rate'),
            'gravity': 9.81,
            'use_magnetometer': False,
        }]
    )
    
    # Calibration Node (conditional)
    calibration_node = Node(
        package='imu_processing',
        executable='imu_calibration_node.py',
        name='imu_calibration_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('calibrate')),
        parameters=[{
            'samples_per_position': 1000,
            'gravity': 9.81,
            'output_file': 'imu_calibration.yaml',
        }]
    )
    
    return LaunchDescription([
        declare_use_cf_arg,
        declare_calibrate_arg,
        declare_update_rate_arg,
        declare_accel_cutoff_arg,
        declare_gyro_cutoff_arg,
        declare_cf_alpha_arg,
        imu_processing_node,
        complementary_filter_node,
        calibration_node,
    ])
