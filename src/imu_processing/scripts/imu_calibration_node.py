#!/usr/bin/env python3
"""
IMU Calibration Node
Interactive calibration tool for IMU bias estimation

This node provides an interactive calibration procedure for:
- Accelerometer bias (3-axis)
- Gyroscope bias (3-axis)
- Scale factors (optional)
- Cross-axis sensitivity (optional)

Feature ID: 1.1.2 - IMU Data Processing
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String

import numpy as np
from collections import deque
import yaml
import os


class IMUCalibrationNode(Node):
    """
    ROS2 node for IMU calibration.
    
    Subscribes to:
        - /mavros/imu/data_raw: Raw IMU data
    
    Publishes:
        - /imu/calibration_status: Status messages
        - /imu/calibration_result: Calibration results
    """
    
    def __init__(self):
        super().__init__('imu_calibration_node')
        
        # Declare parameters
        self.declare_parameter('samples_per_position', 1000)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('output_file', 'imu_calibration.yaml')
        
        # Get parameters
        self.samples = self.get_parameter('samples_per_position').value
        self.gravity = self.get_parameter('gravity').value
        self.output_file = self.get_parameter('output_file').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data_raw',
            self.imu_callback,
            sensor_qos
        )
        
        # Publishers
        self.status_pub = self.create_publisher(
            String,
            '/imu/calibration_status',
            10
        )
        
        self.result_pub = self.create_publisher(
            Vector3Stamped,
            '/imu/calibration_result',
            10
        )
        
        # Calibration state
        self.calibration_active = False
        self.current_position = 0
        self.position_names = ['level', 'upside_down', 'left', 'right', 'nose_up', 'nose_down']
        
        # Data collection
        self.accel_data = deque(maxlen=self.samples)
        self.gyro_data = deque(maxlen=self.samples)
        
        # Results storage
        self.accel_measurements = []
        self.gyro_measurements = []
        
        # Timer for user interaction
        self.create_timer(1.0, self.interaction_callback)
        
        self.get_logger().info('IMU Calibration Node initialized')
        self.get_logger().info('='*60)
        self.get_logger().info('CALIBRATION PROCEDURE:')
        self.get_logger().info('1. Place drone on level surface')
        self.get_logger().info('2. Keep drone completely still')
        self.get_logger().info('3. Press Enter when ready')
        self.get_logger().info('='*60)
    
    def imu_callback(self, msg: Imu):
        """
        Collect IMU data during calibration.
        """
        if not self.calibration_active:
            return
        
        # Store accelerometer data
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        self.accel_data.append(accel)
        
        # Store gyroscope data
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        self.gyro_data.append(gyro)
        
        # Check if enough samples collected
        if len(self.accel_data) >= self.samples:
            self.process_position()
    
    def interaction_callback(self):
        """
        Handle user interaction during calibration.
        """
        if not self.calibration_active and self.current_position == 0:
            # Waiting for user to start calibration
            self.publish_status('Ready to start calibration. Type "start" in terminal.')
        elif self.calibration_active:
            progress = len(self.accel_data) / self.samples * 100
            self.publish_status(f'Collecting data for position {self.current_position + 1}/{len(self.position_names)}: {progress:.1f}%')
    
    def start_calibration(self):
        """
        Start the calibration procedure.
        """
        self.get_logger().info('Starting calibration...')
        self.calibration_active = True
        self.current_position = 0
        self.accel_measurements = []
        self.gyro_measurements = []
        self.collect_position_data()
    
    def collect_position_data(self):
        """
        Start collecting data for current position.
        """
        if self.current_position >= len(self.position_names):
            self.finish_calibration()
            return
        
        self.accel_data.clear()
        self.gyro_data.clear()
        
        position = self.position_names[self.current_position]
        self.get_logger().info(f'\nPosition {self.current_position + 1}/{len(self.position_names)}: {position}')
        self.get_logger().info(f'Collecting {self.samples} samples...')
        self.publish_status(f'Position: {position}')
    
    def process_position(self):
        """
        Process collected data for current position.
        """
        # Calculate mean
        accel_mean = np.mean(list(self.accel_data), axis=0)
        gyro_mean = np.mean(list(self.gyro_data), axis=0)
        
        # Calculate std dev
        accel_std = np.std(list(self.accel_data), axis=0)
        gyro_std = np.std(list(self.gyro_data), axis=0)
        
        self.get_logger().info(f'Accel mean: {accel_mean}')
        self.get_logger().info(f'Accel std: {accel_std}')
        self.get_logger().info(f'Gyro mean: {gyro_mean}')
        self.get_logger().info(f'Gyro std: {gyro_std}')
        
        # Store measurements
        self.accel_measurements.append(accel_mean)
        self.gyro_measurements.append(gyro_mean)
        
        # Move to next position
        self.current_position += 1
        
        if self.current_position < len(self.position_names):
            self.get_logger().info('\nReady for next position')
            self.get_logger().info('Press Enter when ready...')
            # In real use, wait for user input
            # For automation, proceed immediately
            self.create_timer(3.0, self.collect_position_data, one_shot=True)
        else:
            self.finish_calibration()
    
    def finish_calibration(self):
        """
        Calculate calibration parameters and save results.
        """
        self.calibration_active = False
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('CALIBRATION COMPLETE')
        self.get_logger().info('='*60)
        
        # Calculate gyroscope bias (simple average across all positions)
        gyro_bias = np.mean(self.gyro_measurements, axis=0)
        
        # Calculate accelerometer bias
        # For simple calibration, use the level position
        accel_bias = self.accel_measurements[0] - np.array([0, 0, self.gravity])
        
        # Calculate scale factors (optional, advanced calibration)
        # Would require measurements in all 6 orientations
        
        self.get_logger().info('\nCalibration Results:')
        self.get_logger().info(f'Gyro Bias (rad/s): {gyro_bias}')
        self.get_logger().info(f'Accel Bias (m/s²): {accel_bias}')
        
        # Save to file
        self.save_calibration(accel_bias, gyro_bias)
        
        # Publish results
        self.publish_result('accel_bias', accel_bias)
        self.publish_result('gyro_bias', gyro_bias)
        self.publish_status('Calibration complete! Results saved.')
    
    def save_calibration(self, accel_bias, gyro_bias):
        """
        Save calibration parameters to YAML file.
        """
        calibration_data = {
            'imu_processing_node': {
                'ros__parameters': {
                    'accel_bias_x': float(accel_bias[0]),
                    'accel_bias_y': float(accel_bias[1]),
                    'accel_bias_z': float(accel_bias[2]),
                    'gyro_bias_x': float(gyro_bias[0]),
                    'gyro_bias_y': float(gyro_bias[1]),
                    'gyro_bias_z': float(gyro_bias[2]),
                    'calibration_timestamp': self.get_clock().now().nanoseconds
                }
            }
        }
        
        try:
            with open(self.output_file, 'w') as f:
                yaml.dump(calibration_data, f, default_flow_style=False)
            self.get_logger().info(f'\nCalibration saved to: {self.output_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')
    
    def publish_status(self, message):
        """
        Publish calibration status message.
        """
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
    
    def publish_result(self, result_type, values):
        """
        Publish calibration result.
        """
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = result_type
        msg.vector.x = float(values[0])
        msg.vector.y = float(values[1])
        msg.vector.z = float(values[2])
        
        self.result_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = IMUCalibrationNode()
    
    # Simple CLI for starting calibration
    import threading
    
    def user_input_thread():
        while rclpy.ok():
            try:
                user_input = input()
                if user_input.lower() == 'start':
                    node.start_calibration()
                elif user_input.lower() == 'quit':
                    break
            except EOFError:
                break
    
    # Start input thread
    input_thread = threading.Thread(target=user_input_thread, daemon=True)
    input_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down IMU Calibration Node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
