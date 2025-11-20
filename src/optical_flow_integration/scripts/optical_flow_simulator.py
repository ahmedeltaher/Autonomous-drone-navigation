#!/usr/bin/env python3
"""
Optical Flow Simulator Node
Simulates optical flow sensor data for testing without hardware

This node generates synthetic optical flow data based on commanded velocities,
useful for development and testing of the optical flow integration system.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range
from mavros_msgs.msg import OpticalFlowRad
from std_msgs.msg import Header

import numpy as np
import math


class OpticalFlowSimulator(Node):
    """
    Simulates optical flow sensor (PX4FLOW/PMW3901) behavior.
    
    Subscribes to:
        - /cmd_vel: Commanded velocity for simulation
    
    Publishes:
        - /mavros/optical_flow/raw/optical_flow_rad: Simulated optical flow
        - /mavros/distance_sensor/hrlv_ez4_pub: Simulated range sensor
    """
    
    def __init__(self):
        super().__init__('optical_flow_simulator')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('focal_length_px', 16.0)  # pixels
        self.declare_parameter('sensor_size_m', 0.0001)  # meters
        self.declare_parameter('simulated_height', 1.5)  # meters
        self.declare_parameter('height_noise_std', 0.02)  # meters
        self.declare_parameter('flow_noise_std', 0.01)  # rad/s
        self.declare_parameter('quality_threshold', 200)  # 0-255
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.focal_length = self.get_parameter('focal_length_px').value
        self.sensor_size = self.get_parameter('sensor_size_m').value
        self.height = self.get_parameter('simulated_height').value
        self.height_noise = self.get_parameter('height_noise_std').value
        self.flow_noise = self.get_parameter('flow_noise_std').value
        self.quality = self.get_parameter('quality_threshold').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers
        self.flow_pub = self.create_publisher(
            OpticalFlowRad,
            '/mavros/optical_flow/raw/optical_flow_rad',
            sensor_qos
        )
        
        self.range_pub = self.create_publisher(
            Range,
            '/mavros/distance_sensor/hrlv_ez4_pub',
            sensor_qos
        )
        
        # State
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.last_publish_time = self.get_clock().now()
        
        # Timer for publishing
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_simulated_data
        )
        
        self.get_logger().info('Optical Flow Simulator initialized')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Simulated height: {self.height} m')
    
    def cmd_vel_callback(self, msg: TwistStamped):
        """
        Update current velocity from commanded velocity.
        """
        self.current_velocity[0] = msg.twist.linear.x
        self.current_velocity[1] = msg.twist.linear.y
        self.current_velocity[2] = msg.twist.linear.z
    
    def publish_simulated_data(self):
        """
        Publish simulated optical flow and range data.
        """
        now = self.get_clock().now()
        dt = (now - self.last_publish_time).nanoseconds / 1e9
        self.last_publish_time = now
        
        # Simulate optical flow
        self.publish_optical_flow(now, dt)
        
        # Simulate range sensor
        self.publish_range(now)
    
    def publish_optical_flow(self, timestamp, dt):
        """
        Generate and publish simulated optical flow data.
        
        Inverse of velocity estimation:
        ω = (v * f) / h
        """
        msg = OpticalFlowRad()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = 'optical_flow_sim'
        
        # Calculate focal length in physical units
        focal_length_m = self.focal_length * self.sensor_size
        
        # Calculate optical flow rates from velocity
        # ω = (v * f) / h
        if self.height > 0.1:
            flow_rate_x = (self.current_velocity[0] * focal_length_m) / self.height
            flow_rate_y = (self.current_velocity[1] * focal_length_m) / self.height
        else:
            flow_rate_x = 0.0
            flow_rate_y = 0.0
        
        # Add noise
        flow_rate_x += np.random.normal(0, self.flow_noise)
        flow_rate_y += np.random.normal(0, self.flow_noise)
        
        # Integration time in microseconds
        integration_time_us = int(dt * 1e6)
        msg.integration_time_us = integration_time_us
        
        # Integrated flow (flow_rate * time)
        msg.integrated_x = flow_rate_x * dt
        msg.integrated_y = flow_rate_y * dt
        
        # Simulate gyro data (assuming zero rotation for simplicity)
        msg.integrated_xgyro = 0.0
        msg.integrated_ygyro = 0.0
        msg.integrated_zgyro = 0.0
        
        # Temperature (simulated)
        msg.temperature = 25
        
        # Quality (simulated based on velocity magnitude)
        velocity_mag = np.linalg.norm(self.current_velocity[:2])
        if velocity_mag < 0.1:
            msg.quality = self.quality
        else:
            msg.quality = max(0, self.quality - int(velocity_mag * 20))
        
        # Time since last sonar update
        msg.time_delta_distance_us = integration_time_us
        
        # Distance (simulated height)
        msg.distance = self.height
        
        self.flow_pub.publish(msg)
    
    def publish_range(self, timestamp):
        """
        Generate and publish simulated range sensor data.
        """
        msg = Range()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = 'range_sensor_sim'
        
        # Sensor specifications (typical for lidar-lite)
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.02  # ~1 degree
        msg.min_range = 0.05  # 5 cm
        msg.max_range = 40.0  # 40 m
        
        # Add noise to height measurement
        noisy_height = self.height + np.random.normal(0, self.height_noise)
        noisy_height = max(msg.min_range, min(msg.max_range, noisy_height))
        
        msg.range = noisy_height
        
        self.range_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = OpticalFlowSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down Optical Flow Simulator')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
