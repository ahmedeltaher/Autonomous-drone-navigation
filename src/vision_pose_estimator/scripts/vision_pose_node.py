#!/usr/bin/env python3
"""
Vision Pose Estimation Node
Inject SLAM pose into PX4 EKF2 for GPS-denied navigation

Epic 4: Flight Controller Integration
Feature: Vision Pose Estimation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import VisionPositionEstimate
import math
from tf2_ros import TransformBroadcaster
import numpy as np


class VisionPoseNode(Node):
    def __init__(self):
        super().__init__('vision_pose_node')
        
        # Parameters
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('use_odom', True)
        self.declare_parameter('use_pose', False)
        self.declare_parameter('position_covariance', 0.01)  # m^2
        self.declare_parameter('orientation_covariance', 0.01)  # rad^2
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.use_odom = self.get_parameter('use_odom').value
        self.use_pose = self.get_parameter('use_pose').value
        self.pos_cov = self.get_parameter('position_covariance').value
        self.ori_cov = self.get_parameter('orientation_covariance').value
        
        # State
        self.latest_pose = None
        self.latest_timestamp = None
        
        # Subscribers
        if self.use_odom:
            self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                10
            )
            self.get_logger().info('Subscribed to /odom')
        
        if self.use_pose:
            self.create_subscription(
                PoseStamped,
                '/slam/pose',
                self.pose_callback,
                10
            )
            self.get_logger().info('Subscribed to /slam/pose')
        
        # Publishers
        self.vision_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )
        
        self.vision_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/mavros/vision_pose/pose_cov',
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishing timer
        self.dt = 1.0 / self.publish_rate
        self.create_timer(self.dt, self.publish_vision_pose)
        
        self.get_logger().info('Vision Pose Estimator Node initialized')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Frame: {self.frame_id} -> {self.child_frame_id}')
    
    def odom_callback(self, msg):
        """Receive odometry from SLAM."""
        self.latest_pose = msg.pose.pose
        self.latest_timestamp = msg.header.stamp
    
    def pose_callback(self, msg):
        """Receive pose from SLAM."""
        self.latest_pose = msg.pose
        self.latest_timestamp = msg.header.stamp
    
    def publish_vision_pose(self):
        """Publish vision pose to MAVROS for EKF2 integration."""
        if self.latest_pose is None:
            return
        
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose = self.latest_pose
        
        # Publish to MAVROS
        self.vision_pub.publish(pose_msg)
        
        # Create PoseWithCovarianceStamped for better EKF2 integration
        pose_cov_msg = PoseWithCovarianceStamped()
        pose_cov_msg.header = pose_msg.header
        pose_cov_msg.pose.pose = self.latest_pose
        
        # Set covariance matrix (6x6: x, y, z, roll, pitch, yaw)
        covariance = np.zeros(36)
        # Position covariance (x, y, z)
        covariance[0] = self.pos_cov   # x
        covariance[7] = self.pos_cov   # y
        covariance[14] = self.pos_cov  # z
        # Orientation covariance (roll, pitch, yaw)
        covariance[21] = self.ori_cov  # roll
        covariance[28] = self.ori_cov  # pitch
        covariance[35] = self.ori_cov  # yaw
        
        pose_cov_msg.pose.covariance = covariance.tolist()
        
        # Publish covariance version
        self.vision_cov_pub.publish(pose_cov_msg)
        
        # Broadcast TF
        self.broadcast_transform()
    
    def broadcast_transform(self):
        """Broadcast TF transform for visualization."""
        if self.latest_pose is None:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        
        # Translation
        t.transform.translation.x = self.latest_pose.position.x
        t.transform.translation.y = self.latest_pose.position.y
        t.transform.translation.z = self.latest_pose.position.z
        
        # Rotation
        t.transform.rotation = self.latest_pose.orientation
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = VisionPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
