#!/usr/bin/env python3
"""
Optical Flow Integration Node
Real-time velocity estimation from downward-facing camera

This node integrates optical flow data from PX4FLOW or PMW3901 sensors
with IMU data to provide accurate velocity estimation for GPS-denied navigation.

Feature ID: 1.1.1 - Optical Flow Integration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3
from sensor_msgs.msg import Range
from mavros_msgs.msg import OpticalFlowRad
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

import numpy as np
import math


class OpticalFlowNode(Node):
    """
    ROS2 node for optical flow-based velocity estimation.
    
    Subscribes to:
        - /mavros/optical_flow/raw/optical_flow_rad: Raw optical flow data
        - /mavros/distance_sensor/hrlv_ez4_pub: Range sensor data for height
        - /mavros/imu/data: IMU data for orientation
    
    Publishes:
        - /optical_flow/velocity: Estimated velocity with covariance
        - /optical_flow/odometry: Full odometry estimate
    """
    
    def __init__(self):
        super().__init__('optical_flow_node')
        
        # Declare parameters
        self.declare_parameter('update_rate', 20.0)  # Hz
        self.declare_parameter('focal_length_px', 16.0)  # pixels (for PMW3901)
        self.declare_parameter('sensor_size_m', 0.0001)  # meters
        self.declare_parameter('min_height', 0.1)  # meters
        self.declare_parameter('max_height', 5.0)  # meters
        self.declare_parameter('velocity_covariance', 0.1)  # m/s
        self.declare_parameter('use_gyro_compensation', True)
        self.declare_parameter('publish_odometry', True)
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.focal_length = self.get_parameter('focal_length_px').value
        self.sensor_size = self.get_parameter('sensor_size_m').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.velocity_cov = self.get_parameter('velocity_covariance').value
        self.use_gyro_comp = self.get_parameter('use_gyro_compensation').value
        self.publish_odom = self.get_parameter('publish_odometry').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.flow_sub = self.create_subscription(
            OpticalFlowRad,
            '/mavros/optical_flow/raw/optical_flow_rad',
            self.flow_callback,
            sensor_qos
        )
        
        self.range_sub = self.create_subscription(
            Range,
            '/mavros/distance_sensor/hrlv_ez4_pub',
            self.range_callback,
            sensor_qos
        )
        
        # Publishers
        self.velocity_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/optical_flow/velocity',
            10
        )
        
        if self.publish_odom:
            self.odom_pub = self.create_publisher(
                Odometry,
                '/optical_flow/odometry',
                10
            )
        
        # State variables
        self.current_height = 1.0  # meters
        self.last_flow_time = None
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z
        self.velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vz
        
        # Statistics
        self.flow_count = 0
        self.range_count = 0
        
        # Timer for status reporting
        self.create_timer(5.0, self.status_callback)
        
        self.get_logger().info('Optical Flow Node initialized')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Focal length: {self.focal_length} px')
        self.get_logger().info(f'Height range: {self.min_height} - {self.max_height} m')
    
    def flow_callback(self, msg: OpticalFlowRad):
        """
        Process optical flow data and estimate velocity.
        
        Optical flow equation:
        v = (ω * h) / f
        where:
        - v: velocity (m/s)
        - ω: angular velocity from optical flow (rad/s)
        - h: height above ground (m)
        - f: focal length in physical units (m/px)
        """
        self.flow_count += 1
        
        # Check if height is valid
        if self.current_height < self.min_height or self.current_height > self.max_height:
            self.get_logger().warn(
                f'Height {self.current_height:.2f}m out of range [{self.min_height}, {self.max_height}]',
                throttle_duration_sec=2.0
            )
            return
        
        # Extract flow rates (rad/s)
        flow_x = msg.integrated_x  # Angular velocity in x
        flow_y = msg.integrated_y  # Angular velocity in y
        integration_time = msg.integration_time_us / 1e6  # Convert to seconds
        
        if integration_time <= 0:
            return
        
        # Convert integrated flow to flow rate
        flow_rate_x = flow_x / integration_time
        flow_rate_y = flow_y / integration_time
        
        # Gyro compensation (if enabled and available)
        if self.use_gyro_comp:
            flow_rate_x -= msg.integrated_xgyro / integration_time
            flow_rate_y -= msg.integrated_ygyro / integration_time
        
        # Calculate velocity using flow equation
        # v = (flow_rate * height) / focal_length_physical
        focal_length_m = self.focal_length * self.sensor_size
        
        vx = (flow_rate_x * self.current_height) / focal_length_m
        vy = (flow_rate_y * self.current_height) / focal_length_m
        
        # Update velocity state
        self.velocity = np.array([vx, vy, 0.0])
        
        # Publish velocity with covariance
        self.publish_velocity(msg.header.stamp)
        
        # Update position estimate (dead reckoning)
        if self.last_flow_time is not None:
            dt = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) - self.last_flow_time
            if dt > 0 and dt < 1.0:  # Sanity check
                self.position += self.velocity * dt
        
        self.last_flow_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Publish odometry if enabled
        if self.publish_odom:
            self.publish_odometry(msg.header.stamp)
    
    def range_callback(self, msg: Range):
        """
        Update current height from range sensor.
        """
        self.range_count += 1
        
        if msg.range >= msg.min_range and msg.range <= msg.max_range:
            self.current_height = msg.range
        else:
            self.get_logger().warn(
                f'Range {msg.range:.2f}m out of sensor limits [{msg.min_range}, {msg.max_range}]',
                throttle_duration_sec=2.0
            )
    
    def publish_velocity(self, timestamp):
        """
        Publish velocity estimate with covariance.
        """
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'optical_flow'
        
        # Set linear velocity
        msg.twist.twist.linear.x = self.velocity[0]
        msg.twist.twist.linear.y = self.velocity[1]
        msg.twist.twist.linear.z = self.velocity[2]
        
        # Set covariance (6x6 matrix, row-major)
        # Only populate linear velocity covariance (first 3x3 block)
        cov = np.zeros(36)
        cov[0] = self.velocity_cov ** 2  # var(vx)
        cov[7] = self.velocity_cov ** 2  # var(vy)
        cov[14] = (self.velocity_cov * 2) ** 2  # var(vz) - higher uncertainty
        msg.twist.covariance = cov.tolist()
        
        self.velocity_pub.publish(msg)
    
    def publish_odometry(self, timestamp):
        """
        Publish full odometry estimate (position + velocity).
        """
        msg = Odometry()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Set position
        msg.pose.pose.position.x = self.position[0]
        msg.pose.pose.position.y = self.position[1]
        msg.pose.pose.position.z = self.position[2]
        
        # Set orientation (quaternion - identity for now)
        msg.pose.pose.orientation.w = 1.0
        
        # Set velocity
        msg.twist.twist.linear.x = self.velocity[0]
        msg.twist.twist.linear.y = self.velocity[1]
        msg.twist.twist.linear.z = self.velocity[2]
        
        # Set covariance for pose (6x6)
        pose_cov = np.zeros(36)
        # Position covariance increases with time (dead reckoning)
        pose_cov[0] = (0.1 + 0.01 * abs(self.position[0])) ** 2
        pose_cov[7] = (0.1 + 0.01 * abs(self.position[1])) ** 2
        pose_cov[14] = 0.01 ** 2  # z position (from range sensor)
        msg.pose.covariance = pose_cov.tolist()
        
        # Set velocity covariance (6x6)
        twist_cov = np.zeros(36)
        twist_cov[0] = self.velocity_cov ** 2
        twist_cov[7] = self.velocity_cov ** 2
        twist_cov[14] = (self.velocity_cov * 2) ** 2
        msg.twist.covariance = twist_cov.tolist()
        
        self.odom_pub.publish(msg)
    
    def status_callback(self):
        """
        Periodic status reporting.
        """
        self.get_logger().info(
            f'Status - Flow msgs: {self.flow_count}, Range msgs: {self.range_count}, '
            f'Height: {self.current_height:.2f}m, '
            f'Velocity: [{self.velocity[0]:.2f}, {self.velocity[1]:.2f}] m/s'
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = OpticalFlowNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down Optical Flow Node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
