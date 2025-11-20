#!/usr/bin/env python3
"""
Loop Closure Detection Node
Detects revisited locations using scan matching

Feature ID: 1.1.6 - Graph-Based SLAM
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
import numpy as np
import math


class LoopClosureNode(Node):
    def __init__(self):
        super().__init__('loop_closure_node')
        
        self.declare_parameter('scan_history_size', 100)
        self.declare_parameter('loop_threshold', 0.8)  # Match score threshold
        
        self.history_size = self.get_parameter('scan_history_size').value
        self.threshold = self.get_parameter('loop_threshold').value
        
        # Scan history
        self.scan_history = []
        self.pose_history = []
        
        # Subscribers
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/slam/pose',
            self.pose_callback,
            10
        )
        
        # Publishers
        self.loop_pub = self.create_publisher(Bool, '/loop_closure/detected', 10)
        
        self.current_pose = None
        self.loop_count = 0
        
        self.get_logger().info('Loop Closure Node initialized')
    
    def pose_callback(self, msg):
        """Store current pose."""
        self.current_pose = msg
    
    def scan_callback(self, msg):
        """Check for loop closures."""
        if self.current_pose is None:
            return
        
        current_scan = self.extract_features(msg)
        
        # Check against scan history
        for i, (hist_scan, hist_pose) in enumerate(zip(self.scan_history, self.pose_history)):
            if i < len(self.scan_history) - 20:  # Skip recent scans
                score = self.match_scans(current_scan, hist_scan)
                
                if score > self.threshold:
                    self.loop_count += 1
                    self.get_logger().info(f'Loop closure detected! Score: {score:.2f}')
                    
                    msg = Bool()
                    msg.data = True
                    self.loop_pub.publish(msg)
                    break
        
        # Add to history
        self.scan_history.append(current_scan)
        self.pose_history.append(self.current_pose)
        
        # Limit history size
        if len(self.scan_history) > self.history_size:
            self.scan_history.pop(0)
            self.pose_history.pop(0)
    
    def extract_features(self, scan):
        """Extract features from laser scan."""
        features = []
        
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                features.append([x, y])
            angle += scan.angle_increment
        
        return np.array(features)
    
    def match_scans(self, scan1, scan2):
        """Compute similarity score between scans."""
        if len(scan1) == 0 or len(scan2) == 0:
            return 0.0
        
        # Simple nearest neighbor matching
        matches = 0
        for p1 in scan1:
            distances = np.linalg.norm(scan2 - p1, axis=1)
            if np.min(distances) < 0.2:  # 20cm threshold
                matches += 1
        
        score = matches / len(scan1)
        return score


def main(args=None):
    rclpy.init(args=args)
    node = LoopClosureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
