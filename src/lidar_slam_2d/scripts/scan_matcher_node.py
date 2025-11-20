#!/usr/bin/env python3
"""
Scan Matcher Node
2D scan matching for SLAM localization and mapping

This node provides scan-to-scan and scan-to-map matching for:
- Real-time localization
- Map building
- Loop closure detection
- Pose graph optimization

Feature ID: 1.1.3 - 2D Lidar SLAM
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from std_msgs.msg import Header

import numpy as np
import math
from collections import deque
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


class ScanMatcherNode(Node):
    """
    ROS2 node for 2D lidar scan matching.
    
    This node implements scan matching algorithms for SLAM:
    - ICP (Iterative Closest Point)
    - Scan-to-map correlation
    - Pose estimation and tracking
    
    Subscribes to:
        - /scan: LaserScan data from lidar
        - /odom: Odometry for initial guess
    
    Publishes:
        - /slam/pose: Estimated robot pose
        - /slam/map: Occupancy grid map
        - /slam/scan_matched: Matched scan for visualization
    """
    
    def __init__(self):
        super().__init__('scan_matcher_node')
        
        # Declare parameters
        self.declare_parameter('map_resolution', 0.05)  # meters per pixel
        self.declare_parameter('map_size_x', 100.0)  # meters
        self.declare_parameter('map_size_y', 100.0)  # meters
        self.declare_parameter('scan_matching_rate', 5.0)  # Hz
        self.declare_parameter('min_scan_range', 0.1)  # meters
        self.declare_parameter('max_scan_range', 30.0)  # meters
        self.declare_parameter('use_imu', True)  # Use IMU for rotation estimate
        self.declare_parameter('use_odom', True)  # Use wheel odom as initial guess
        
        # Get parameters
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_size_x = self.get_parameter('map_size_x').value
        self.map_size_y = self.get_parameter('map_size_y').value
        self.scan_rate = self.get_parameter('scan_matching_rate').value
        self.min_range = self.get_parameter('min_scan_range').value
        self.max_range = self.get_parameter('max_scan_range').value
        self.use_imu = self.get_parameter('use_imu').value
        self.use_odom = self.get_parameter('use_odom').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/optical_flow/odometry',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/slam/pose',
            10
        )
        
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/slam/map',
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize map
        self.map_width = int(self.map_size_x / self.map_resolution)
        self.map_height = int(self.map_size_y / self.map_resolution)
        self.occupancy_map = np.ones((self.map_height, self.map_width)) * -1  # Unknown
        
        # Robot state
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.robot_covariance = np.eye(3) * 0.01  # Initial uncertainty
        
        # Scan history for matching
        self.last_scan = None
        self.last_scan_time = None
        self.scan_count = 0
        
        # Odometry for initial guess
        self.last_odom = None
        
        # Statistics
        self.match_success_count = 0
        self.match_fail_count = 0
        
        # Timer for status
        self.create_timer(5.0, self.status_callback)
        
        self.get_logger().info('Scan Matcher Node initialized')
        self.get_logger().info(f'Map size: {self.map_size_x}x{self.map_size_y}m')
        self.get_logger().info(f'Map resolution: {self.map_resolution}m/pixel')
        self.get_logger().info(f'Scan matching rate: {self.scan_rate} Hz')
    
    def scan_callback(self, msg: LaserScan):
        """
        Process laser scan and perform scan matching.
        """
        self.scan_count += 1
        
        # Extract valid points
        points = self.extract_scan_points(msg)
        
        if len(points) < 10:  # Need minimum points for matching
            self.get_logger().warn('Too few valid scan points', throttle_duration_sec=5.0)
            return
        
        # Get initial pose guess from odometry
        pose_guess = self.get_pose_guess()
        
        # Perform scan matching
        if self.last_scan is not None:
            matched_pose, success = self.match_scans(points, self.last_scan, pose_guess)
            
            if success:
                self.robot_pose = matched_pose
                self.match_success_count += 1
                
                # Update map with new scan
                self.update_map(points, matched_pose)
            else:
                self.match_fail_count += 1
                self.get_logger().warn('Scan matching failed', throttle_duration_sec=2.0)
        
        # Store scan for next iteration
        self.last_scan = points
        self.last_scan_time = self.get_clock().now()
        
        # Publish pose and map
        self.publish_pose(msg.header.stamp)
        self.publish_map(msg.header.stamp)
        self.publish_tf(msg.header.stamp)
    
    def extract_scan_points(self, scan: LaserScan):
        """
        Convert laser scan to Cartesian points.
        """
        points = []
        
        angle = scan.angle_min
        for i, r in enumerate(scan.ranges):
            if r >= self.min_range and r <= self.max_range:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            
            angle += scan.angle_increment
        
        return np.array(points)
    
    def odom_callback(self, msg: Odometry):
        """
        Store odometry for initial pose guess.
        """
        self.last_odom = msg
    
    def get_pose_guess(self):
        """
        Get initial pose guess from odometry or constant velocity model.
        """
        if self.use_odom and self.last_odom is not None:
            # Use odometry as guess
            x = self.last_odom.pose.pose.position.x
            y = self.last_odom.pose.pose.position.y
            
            # Extract yaw from quaternion
            quat = self.last_odom.pose.pose.orientation
            theta = math.atan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            )
            
            return np.array([x, y, theta])
        else:
            # Use last known pose
            return self.robot_pose.copy()
    
    def match_scans(self, current_scan, reference_scan, initial_guess):
        """
        Match current scan to reference using ICP algorithm.
        
        Returns:
            matched_pose: Estimated pose [x, y, theta]
            success: Boolean indicating if matching succeeded
        """
        # Simple ICP implementation
        max_iterations = 50
        tolerance = 0.001
        
        pose = initial_guess.copy()
        
        for iteration in range(max_iterations):
            # Transform current scan to reference frame
            transformed_scan = self.transform_points(current_scan, pose)
            
            # Find correspondences (nearest neighbors)
            correspondences = self.find_correspondences(transformed_scan, reference_scan)
            
            if len(correspondences) < 5:
                return pose, False
            
            # Compute pose update
            pose_update = self.compute_pose_update(correspondences)
            
            # Update pose
            pose = self.compose_poses(pose, pose_update)
            
            # Check convergence
            if np.linalg.norm(pose_update[:2]) < tolerance:
                break
        
        # Check if matching is reasonable
        displacement = np.linalg.norm(pose[:2] - initial_guess[:2])
        if displacement > 1.0:  # More than 1m change is suspicious
            return initial_guess, False
        
        return pose, True
    
    def transform_points(self, points, pose):
        """
        Transform points by pose [x, y, theta].
        """
        x, y, theta = pose
        
        # Rotation matrix
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        
        # Transform each point
        transformed = np.zeros_like(points)
        transformed[:, 0] = cos_t * points[:, 0] - sin_t * points[:, 1] + x
        transformed[:, 1] = sin_t * points[:, 0] + cos_t * points[:, 1] + y
        
        return transformed
    
    def find_correspondences(self, scan1, scan2, max_dist=0.5):
        """
        Find nearest neighbor correspondences between scans.
        """
        correspondences = []
        
        for p1 in scan1:
            # Find nearest point in scan2
            distances = np.linalg.norm(scan2 - p1, axis=1)
            min_idx = np.argmin(distances)
            
            if distances[min_idx] < max_dist:
                correspondences.append((p1, scan2[min_idx]))
        
        return correspondences
    
    def compute_pose_update(self, correspondences):
        """
        Compute pose update from correspondences using least squares.
        """
        if len(correspondences) == 0:
            return np.zeros(3)
        
        # Extract source and target points
        src_points = np.array([c[0] for c in correspondences])
        tgt_points = np.array([c[1] for c in correspondences])
        
        # Compute centroids
        src_centroid = np.mean(src_points, axis=0)
        tgt_centroid = np.mean(tgt_points, axis=0)
        
        # Center the points
        src_centered = src_points - src_centroid
        tgt_centered = tgt_points - tgt_centroid
        
        # Compute rotation using SVD
        H = src_centered.T @ tgt_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Extract angle
        theta = math.atan2(R[1, 0], R[0, 0])
        
        # Compute translation
        t = tgt_centroid - R @ src_centroid
        
        return np.array([t[0], t[1], theta])
    
    def compose_poses(self, pose1, pose2):
        """
        Compose two poses: result = pose1 ⊕ pose2
        """
        x1, y1, theta1 = pose1
        x2, y2, theta2 = pose2
        
        cos_t1 = math.cos(theta1)
        sin_t1 = math.sin(theta1)
        
        x = x1 + cos_t1 * x2 - sin_t1 * y2
        y = y1 + sin_t1 * x2 + cos_t1 * y2
        theta = theta1 + theta2
        
        # Normalize angle
        theta = math.atan2(math.sin(theta), math.cos(theta))
        
        return np.array([x, y, theta])
    
    def update_map(self, points, pose):
        """
        Update occupancy grid with new scan.
        """
        # Transform points to map frame
        world_points = self.transform_points(points, pose)
        
        # Convert to map coordinates
        map_x = (world_points[:, 0] / self.map_resolution + self.map_width / 2).astype(int)
        map_y = (world_points[:, 1] / self.map_resolution + self.map_height / 2).astype(int)
        
        # Mark occupied cells
        for x, y in zip(map_x, map_y):
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                self.occupancy_map[y, x] = 100  # Occupied
        
        # Mark free cells using ray tracing
        robot_map_x = int(pose[0] / self.map_resolution + self.map_width / 2)
        robot_map_y = int(pose[1] / self.map_resolution + self.map_height / 2)
        
        for x, y in zip(map_x, map_y):
            self.trace_ray(robot_map_x, robot_map_y, x, y)
    
    def trace_ray(self, x0, y0, x1, y1):
        """
        Trace ray from (x0,y0) to (x1,y1) and mark cells as free.
        Bresenham's line algorithm.
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if self.occupancy_map[y, x] == -1:  # Only update unknown cells
                    self.occupancy_map[y, x] = 0  # Free
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def publish_pose(self, timestamp):
        """
        Publish estimated robot pose.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        
        # Set position
        msg.pose.pose.position.x = self.robot_pose[0]
        msg.pose.pose.position.y = self.robot_pose[1]
        msg.pose.pose.position.z = 0.0
        
        # Set orientation (convert theta to quaternion)
        theta = self.robot_pose[2]
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        
        # Set covariance (3x3 for x, y, theta -> 6x6 for pose)
        covariance = np.zeros(36)
        covariance[0] = self.robot_covariance[0, 0]  # x
        covariance[7] = self.robot_covariance[1, 1]  # y
        covariance[35] = self.robot_covariance[2, 2]  # theta
        msg.pose.covariance = covariance.tolist()
        
        self.pose_pub.publish(msg)
    
    def publish_map(self, timestamp):
        """
        Publish occupancy grid map.
        """
        msg = OccupancyGrid()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = -self.map_size_x / 2.0
        msg.info.origin.position.y = -self.map_size_y / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Flatten map and convert to int8
        msg.data = self.occupancy_map.flatten().astype(np.int8).tolist()
        
        self.map_pub.publish(msg)
    
    def publish_tf(self, timestamp):
        """
        Publish TF transform from map to base_link.
        """
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.robot_pose[0]
        t.transform.translation.y = self.robot_pose[1]
        t.transform.translation.z = 0.0
        
        theta = self.robot_pose[2]
        t.transform.rotation.w = math.cos(theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
    
    def status_callback(self):
        """
        Periodic status reporting.
        """
        total_matches = self.match_success_count + self.match_fail_count
        success_rate = 0.0
        if total_matches > 0:
            success_rate = self.match_success_count / total_matches * 100
        
        self.get_logger().info(
            f'Status - Scans: {self.scan_count}, '
            f'Match success: {self.match_success_count}/{total_matches} ({success_rate:.1f}%), '
            f'Pose: [{self.robot_pose[0]:.2f}, {self.robot_pose[1]:.2f}, {math.degrees(self.robot_pose[2]):.1f}°]'
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = ScanMatcherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down Scan Matcher Node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
