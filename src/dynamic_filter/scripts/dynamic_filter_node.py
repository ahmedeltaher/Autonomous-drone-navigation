#!/usr/bin/env python3
"""
Dynamic Object Filtering Node
Remove moving obstacles from static occupancy grid

Feature ID: 1.1.8 - Dynamic Object Filtering
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
from collections import deque


class DynamicFilterNode(Node):
    def __init__(self):
        super().__init__('dynamic_filter_node')
        
        # Parameters
        self.declare_parameter('history_length', 10)  # Number of scans to track
        self.declare_parameter('velocity_threshold', 0.2)  # m/s
        self.declare_parameter('confidence_threshold', 0.7)  # 70% confidence
        
        self.history_len = self.get_parameter('history_length').value
        self.vel_thresh = self.get_parameter('velocity_threshold').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        
        # Tracking data
        self.scan_history = deque(maxlen=self.history_len)
        self.pose_history = deque(maxlen=self.history_len)
        
        # Current state
        self.current_pose = None
        self.static_map = None
        
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
        
        self.create_subscription(
            OccupancyGrid,
            '/occupancy_grid',
            self.map_callback,
            10
        )
        
        # Publishers
        self.filtered_map_pub = self.create_publisher(
            OccupancyGrid,
            '/static_map',
            10
        )
        
        self.dynamic_objects_pub = self.create_publisher(
            MarkerArray,
            '/dynamic_objects',
            10
        )
        
        # Processing timer
        self.create_timer(1.0, self.filter_dynamic_objects)
        
        self.get_logger().info('Dynamic Filter Node initialized')
        self.get_logger().info(f'History: {self.history_len}, Vel thresh: {self.vel_thresh} m/s')
    
    def pose_callback(self, msg):
        """Store current robot pose."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        quat = msg.pose.pose.orientation
        theta = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )
        
        self.current_pose = (x, y, theta)
    
    def scan_callback(self, msg):
        """Store scan and pose history."""
        if self.current_pose is None:
            return
        
        # Extract scan points
        points = []
        angle = msg.angle_min
        
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = self.current_pose[0] + r * math.cos(self.current_pose[2] + angle)
                y = self.current_pose[1] + r * math.sin(self.current_pose[2] + angle)
                points.append([x, y])
            angle += msg.angle_increment
        
        if len(points) > 0:
            self.scan_history.append(np.array(points))
            self.pose_history.append(self.current_pose)
    
    def map_callback(self, msg):
        """Store occupancy grid."""
        self.static_map = msg
    
    def filter_dynamic_objects(self):
        """Detect and filter dynamic objects."""
        if len(self.scan_history) < 2 or self.static_map is None:
            return
        
        # Detect dynamic objects
        dynamic_points = self.detect_dynamic_objects()
        
        if len(dynamic_points) == 0:
            # No dynamic objects, publish original map
            self.filtered_map_pub.publish(self.static_map)
            return
        
        # Create filtered map
        filtered_map = self.create_filtered_map(dynamic_points)
        
        # Publish filtered map
        self.filtered_map_pub.publish(filtered_map)
        
        # Visualize dynamic objects
        self.visualize_dynamic_objects(dynamic_points)
        
        self.get_logger().info(
            f'Filtered {len(dynamic_points)} dynamic objects',
            throttle_duration_sec=5.0
        )
    
    def detect_dynamic_objects(self):
        """Detect points that moved between scans."""
        if len(self.scan_history) < 2:
            return []
        
        dynamic_points = []
        
        # Compare consecutive scans
        for i in range(len(self.scan_history) - 1):
            scan1 = self.scan_history[i]
            scan2 = self.scan_history[i + 1]
            pose1 = self.pose_history[i]
            pose2 = self.pose_history[i + 1]
            
            # Time between scans (assume 0.1s)
            dt = 0.1
            
            # Find corresponding points
            for p2 in scan2:
                # Find nearest point in previous scan
                distances = np.linalg.norm(scan1 - p2, axis=1)
                min_dist = np.min(distances)
                
                # If point moved significantly
                if min_dist > self.vel_thresh * dt:
                    dynamic_points.append(p2)
        
        return dynamic_points
    
    def create_filtered_map(self, dynamic_points):
        """Remove dynamic objects from map."""
        filtered = OccupancyGrid()
        filtered.header = self.static_map.header
        filtered.info = self.static_map.info
        
        # Copy original map data
        map_array = np.array(self.static_map.data).reshape(
            self.static_map.info.height,
            self.static_map.info.width
        )
        
        # Remove dynamic objects
        for point in dynamic_points:
            # Convert to grid coordinates
            gx = int((point[0] - self.static_map.info.origin.position.x) / 
                    self.static_map.info.resolution)
            gy = int((point[1] - self.static_map.info.origin.position.y) / 
                    self.static_map.info.resolution)
            
            # Mark as unknown in a small radius
            radius = 2  # cells
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    x = gx + dx
                    y = gy + dy
                    if 0 <= x < self.static_map.info.width and \
                       0 <= y < self.static_map.info.height:
                        map_array[y, x] = -1  # Unknown
        
        filtered.data = map_array.flatten().tolist()
        return filtered
    
    def visualize_dynamic_objects(self, dynamic_points):
        """Publish markers for dynamic objects."""
        markers = MarkerArray()
        
        for i, point in enumerate(dynamic_points):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            markers.markers.append(marker)
        
        self.dynamic_objects_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
