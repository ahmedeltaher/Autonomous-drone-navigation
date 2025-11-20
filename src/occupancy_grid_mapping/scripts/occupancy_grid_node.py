#!/usr/bin/env python3
"""
Occupancy Grid Mapping Node
Probabilistic 2D/2.5D occupancy grid generation

Feature ID: 1.1.7 - Occupancy Grid Mapping
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')
        
        # Parameters
        self.declare_parameter('resolution', 0.05)  # 5cm per cell
        self.declare_parameter('width', 200)  # cells
        self.declare_parameter('height', 200)  # cells
        self.declare_parameter('origin_x', -5.0)  # meters
        self.declare_parameter('origin_y', -5.0)  # meters
        self.declare_parameter('prob_occ', 0.7)  # P(occupied | hit)
        self.declare_parameter('prob_free', 0.3)  # P(occupied | miss)
        
        self.resolution = self.get_parameter('resolution').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.prob_occ = self.get_parameter('prob_occ').value
        self.prob_free = self.get_parameter('prob_free').value
        
        # Log-odds parameters
        self.l_occ = self.prob_to_logodds(self.prob_occ)
        self.l_free = self.prob_to_logodds(self.prob_free)
        self.l_prior = 0.0  # Unknown = 50% probability
        
        # Initialize grid (log-odds)
        self.grid = np.zeros((self.height, self.width), dtype=np.float32)
        
        # Current pose
        self.robot_pose = None
        
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
        
        # Publisher
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/occupancy_grid',
            10
        )
        
        # Update timer
        self.create_timer(1.0, self.publish_map)
        
        self.update_count = 0
        self.get_logger().info('Occupancy Grid Mapping Node initialized')
        self.get_logger().info(f'Grid: {self.width}x{self.height} @ {self.resolution}m/cell')
    
    def pose_callback(self, msg):
        """Store current robot pose."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        quat = msg.pose.pose.orientation
        theta = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )
        
        self.robot_pose = (x, y, theta)
    
    def scan_callback(self, msg):
        """Update occupancy grid with laser scan."""
        if self.robot_pose is None:
            return
        
        x, y, theta = self.robot_pose
        
        # Convert robot pose to grid coordinates
        robot_grid_x = int((x - self.origin_x) / self.resolution)
        robot_grid_y = int((y - self.origin_y) / self.resolution)
        
        if not self.in_bounds(robot_grid_x, robot_grid_y):
            return
        
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Endpoint (occupied)
                end_x = x + r * math.cos(theta + angle)
                end_y = y + r * math.sin(theta + angle)
                
                end_grid_x = int((end_x - self.origin_x) / self.resolution)
                end_grid_y = int((end_y - self.origin_y) / self.resolution)
                
                # Ray trace
                cells = self.bresenham(robot_grid_x, robot_grid_y, 
                                      end_grid_x, end_grid_y)
                
                # Update cells along ray (free space)
                for cx, cy in cells[:-1]:
                    if self.in_bounds(cx, cy):
                        self.grid[cy, cx] += self.l_free - self.l_prior
                
                # Update endpoint (occupied)
                if self.in_bounds(end_grid_x, end_grid_y):
                    self.grid[end_grid_y, end_grid_x] += self.l_occ - self.l_prior
            
            angle += msg.angle_increment
        
        self.update_count += 1
    
    def bresenham(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for ray tracing."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            cells.append((x, y))
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return cells
    
    def in_bounds(self, x, y):
        """Check if coordinates are within grid bounds."""
        return 0 <= x < self.width and 0 <= y < self.height
    
    def prob_to_logodds(self, prob):
        """Convert probability to log-odds."""
        return math.log(prob / (1.0 - prob))
    
    def logodds_to_prob(self, logodds):
        """Convert log-odds to probability."""
        return 1.0 - 1.0 / (1.0 + math.exp(logodds))
    
    def publish_map(self):
        """Publish occupancy grid map."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Convert log-odds to probabilities, then to occupancy values (0-100)
        occupancy = np.zeros((self.height, self.width), dtype=np.int8)
        
        for y in range(self.height):
            for x in range(self.width):
                logodds = self.grid[y, x]
                prob = self.logodds_to_prob(logodds)
                
                if prob < 0.2:
                    occupancy[y, x] = 0  # Free
                elif prob > 0.8:
                    occupancy[y, x] = 100  # Occupied
                else:
                    occupancy[y, x] = -1  # Unknown
        
        msg.data = occupancy.flatten().tolist()
        
        self.map_pub.publish(msg)
        
        if self.update_count > 0:
            self.get_logger().info(
                f'Published map: {self.update_count} updates',
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
