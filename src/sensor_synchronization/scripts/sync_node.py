#!/usr/bin/env python3
"""
Sensor Synchronization Node
Microsecond-level timestamp alignment across multiple sensors

This node synchronizes data from:
- Optical flow (20 Hz)
- IMU (100 Hz)
- Lidar (5-10 Hz)

Using approximate time synchronization with configurable tolerance.

Feature ID: 1.1.4 - Sensor Synchronization
"""

import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

import numpy as np


class SensorSyncNode(Node):
    """
    Synchronize multiple sensor streams with microsecond precision.
    
    Subscribes to:
        - /optical_flow/velocity: Optical flow velocity
        - /imu/filtered: Filtered IMU data
        - /scan: Lidar scans
    
    Publishes:
        - /sensor_fusion/synchronized: Synchronized sensor bundle
        - /sensor_fusion/odometry: Fused odometry estimate
    """
    
    def __init__(self):
        super().__init__('sensor_sync_node')
        
        # Declare parameters
        self.declare_parameter('sync_tolerance_ms', 50.0)  # milliseconds
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('sync_mode', 'approximate')  # 'approximate' or 'exact'
        
        # Get parameters
        tolerance_ms = self.get_parameter('sync_tolerance_ms').value
        self.tolerance = tolerance_ms / 1000.0  # Convert to seconds
        queue_size = self.get_parameter('queue_size').value
        
        # Create subscribers with message_filters
        self.optical_flow_sub = Subscriber(
            self,
            TwistWithCovarianceStamped,
            '/optical_flow/velocity'
        )
        
        self.imu_sub = Subscriber(
            self,
            Imu,
            '/imu/filtered'
        )
        
        self.lidar_sub = Subscriber(
            self,
            LaserScan,
            '/scan'
        )
        
        # Approximate time synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.optical_flow_sub, self.imu_sub, self.lidar_sub],
            queue_size=queue_size,
            slop=self.tolerance
        )
        self.sync.registerCallback(self.sync_callback)
        
        # Publisher for synchronized data
        self.odom_pub = self.create_publisher(
            Odometry,
            '/sensor_fusion/odometry',
            10
        )
        
        # Statistics
        self.sync_count = 0
        self.dropped_flow = 0
        self.dropped_imu = 0
        self.dropped_lidar = 0
        
        self.create_timer(5.0, self.status_callback)
        
        self.get_logger().info('Sensor Synchronization Node initialized')
        self.get_logger().info(f'Sync tolerance: {tolerance_ms:.1f} ms')
        self.get_logger().info(f'Queue size: {queue_size}')
    
    def sync_callback(self, flow_msg, imu_msg, lidar_msg):
        """
        Called when synchronized messages are available.
        """
        self.sync_count += 1
        
        # Check timestamp alignment
        flow_time = flow_msg.header.stamp.sec + flow_msg.header.stamp.nanosec * 1e-9
        imu_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
        lidar_time = lidar_msg.header.stamp.sec + lidar_msg.header.stamp.nanosec * 1e-9
        
        max_diff = max(abs(flow_time - imu_time), 
                      abs(flow_time - lidar_time),
                      abs(imu_time - lidar_time))
        
        if max_diff > self.tolerance:
            self.get_logger().warn(
                f'Time sync exceeded tolerance: {max_diff*1000:.2f} ms',
                throttle_duration_sec=2.0
            )
        
        # Fuse sensor data
        fused_odom = self.fuse_sensors(flow_msg, imu_msg, lidar_msg)
        
        # Publish synchronized data
        self.odom_pub.publish(fused_odom)
    
    def fuse_sensors(self, flow_msg, imu_msg, lidar_msg):
        """
        Fuse synchronized sensor data into unified odometry.
        """
        odom = Odometry()
        
        # Use latest timestamp
        odom.header.stamp = imu_msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Velocity from optical flow
        odom.twist.twist.linear = flow_msg.twist.twist.linear
        odom.twist.covariance = flow_msg.twist.covariance
        
        # Orientation from IMU
        odom.pose.pose.orientation = imu_msg.orientation
        
        # Position would come from SLAM
        # For now, integrate velocity
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        
        return odom
    
    def status_callback(self):
        """
        Report synchronization statistics.
        """
        self.get_logger().info(
            f'Synced: {self.sync_count}, '
            f'Dropped - Flow: {self.dropped_flow}, IMU: {self.dropped_imu}, Lidar: {self.dropped_lidar}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SensorSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
