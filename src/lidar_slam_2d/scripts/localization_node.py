#!/usr/bin/env python3
"""
Localization Node
Localize robot within existing map using Monte Carlo Localization (Particle Filter)

Feature ID: 1.1.3 - 2D Lidar SLAM
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
import numpy as np
import math


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        
        self.declare_parameter('num_particles', 500)
        self.num_particles = self.get_parameter('num_particles').value
        
        # Initialize particles randomly
        self.particles = np.random.randn(self.num_particles, 3)  # x, y, theta
        self.weights = np.ones(self.num_particles) / self.num_particles
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/slam/map', self.map_callback, 10
        )
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/localization/pose', 10
        )
        
        self.particles_pub = self.create_publisher(
            PoseArray, '/localization/particles', 10
        )
        
        self.map = None
        self.get_logger().info(f'Localization Node initialized with {self.num_particles} particles')
    
    def map_callback(self, msg):
        self.map = msg
    
    def scan_callback(self, msg):
        if self.map is None:
            return
        
        # Particle filter update
        self.motion_update()
        self.measurement_update(msg)
        self.resample()
        
        # Publish best estimate
        self.publish_pose(msg.header.stamp)
        self.publish_particles(msg.header.stamp)
    
    def motion_update(self):
        # Add motion noise
        self.particles += np.random.randn(self.num_particles, 3) * 0.1
    
    def measurement_update(self, scan):
        # Update weights based on scan likelihood
        for i in range(self.num_particles):
            self.weights[i] = self.compute_likelihood(self.particles[i], scan)
        
        # Normalize weights
        self.weights /= np.sum(self.weights)
    
    def compute_likelihood(self, particle, scan):
        # Simplified likelihood based on map correspondence
        return np.random.random()  # Placeholder
    
    def resample(self):
        # Systematic resampling
        indices = np.random.choice(
            self.num_particles, self.num_particles, p=self.weights
        )
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles
    
    def publish_pose(self, timestamp):
        # Weighted average of particles
        mean_pose = np.average(self.particles, axis=0, weights=self.weights)
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = mean_pose[0]
        msg.pose.pose.position.y = mean_pose[1]
        msg.pose.pose.orientation.z = math.sin(mean_pose[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(mean_pose[2] / 2.0)
        
        self.pose_pub.publish(msg)
    
    def publish_particles(self, timestamp):
        msg = PoseArray()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        
        for p in self.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.orientation.z = math.sin(p[2] / 2.0)
            pose.orientation.w = math.cos(p[2] / 2.0)
            msg.poses.append(pose)
        
        self.particles_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
