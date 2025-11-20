#!/usr/bin/env python3
"""
Pose Graph SLAM Node
Manages pose graph with loop closure and optimization

Feature ID: 1.1.6 - Graph-Based SLAM
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.optimize import least_squares
import math


class PoseGraphNode(Node):
    def __init__(self):
        super().__init__('pose_graph_node')
        
        self.declare_parameter('keyframe_distance', 0.5)  # meters
        self.declare_parameter('loop_closure_distance', 2.0)  # meters
        self.declare_parameter('optimize_frequency', 1.0)  # Hz
        
        self.kf_dist = self.get_parameter('keyframe_distance').value
        self.lc_dist = self.get_parameter('loop_closure_distance').value
        
        # Pose graph
        self.nodes = []  # List of poses [x, y, theta]
        self.edges = []  # List of constraints [(i, j, dx, dy, dtheta, info)]
        
        # Subscribers
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/slam/pose',
            self.pose_callback,
            10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/graph_slam/path', 10)
        self.graph_pub = self.create_publisher(MarkerArray, '/graph_slam/graph', 10)
        
        # Optimization timer
        self.create_timer(1.0 / self.get_parameter('optimize_frequency').value, 
                         self.optimize_graph)
        
        self.last_keyframe = None
        self.get_logger().info('Pose Graph Node initialized')
    
    def pose_callback(self, msg):
        """Add poses as graph nodes."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        theta = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )
        
        current_pose = np.array([x, y, theta])
        
        # Check if keyframe
        if self.should_add_keyframe(current_pose):
            self.add_keyframe(current_pose)
            self.check_loop_closures(len(self.nodes) - 1)
    
    def should_add_keyframe(self, pose):
        """Determine if pose should be added as keyframe."""
        if self.last_keyframe is None:
            return True
        
        dist = np.linalg.norm(pose[:2] - self.last_keyframe[:2])
        return dist > self.kf_dist
    
    def add_keyframe(self, pose):
        """Add new keyframe to graph."""
        idx = len(self.nodes)
        self.nodes.append(pose.copy())
        
        # Add odometry edge from previous node
        if idx > 0:
            prev_pose = self.nodes[idx - 1]
            dx = pose[0] - prev_pose[0]
            dy = pose[1] - prev_pose[1]
            dtheta = pose[2] - prev_pose[2]
            
            # Information matrix (inverse covariance)
            info = np.eye(3) * 10.0
            
            self.edges.append((idx - 1, idx, dx, dy, dtheta, info))
        
        self.last_keyframe = pose
        self.get_logger().info(f'Added keyframe {idx}')
    
    def check_loop_closures(self, current_idx):
        """Detect loop closures."""
        if current_idx < 10:  # Need minimum history
            return
        
        current_pose = self.nodes[current_idx]
        
        # Check against all previous poses (except recent ones)
        for i in range(current_idx - 10):
            candidate = self.nodes[i]
            dist = np.linalg.norm(current_pose[:2] - candidate[:2])
            
            if dist < self.lc_dist:
                # Loop closure detected
                dx = current_pose[0] - candidate[0]
                dy = current_pose[1] - candidate[1]
                dtheta = current_pose[2] - candidate[2]
                
                # Lower information for loop closures
                info = np.eye(3) * 5.0
                
                self.edges.append((i, current_idx, dx, dy, dtheta, info))
                self.get_logger().info(f'Loop closure: {i} <-> {current_idx}')
    
    def optimize_graph(self):
        """Optimize pose graph using least squares."""
        if len(self.nodes) < 2:
            return
        
        # Flatten poses for optimization
        x0 = np.array(self.nodes).flatten()
        
        # Optimize
        result = least_squares(self.error_function, x0, method='lm')
        
        # Update nodes
        optimized = result.x.reshape(-1, 3)
        self.nodes = optimized.tolist()
        
        # Publish results
        self.publish_path()
        self.publish_graph()
    
    def error_function(self, x):
        """Compute error for all edges."""
        poses = x.reshape(-1, 3)
        errors = []
        
        for i, j, dx, dy, dtheta, info in self.edges:
            pi = poses[i]
            pj = poses[j]
            
            # Predicted relative pose
            pred_dx = pj[0] - pi[0]
            pred_dy = pj[1] - pi[1]
            pred_dtheta = pj[2] - pi[2]
            
            # Error
            e = np.array([
                pred_dx - dx,
                pred_dy - dy,
                pred_dtheta - dtheta
            ])
            
            # Weighted error
            errors.extend(e * np.sqrt(np.diag(info)))
        
        return np.array(errors)
    
    def publish_path(self):
        """Publish optimized trajectory."""
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for pose in self.nodes:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = pose[0]
            p.pose.position.y = pose[1]
            p.pose.orientation.w = math.cos(pose[2] / 2.0)
            p.pose.orientation.z = math.sin(pose[2] / 2.0)
            msg.poses.append(p)
        
        self.path_pub.publish(msg)
    
    def publish_graph(self):
        """Visualize pose graph."""
        msg = MarkerArray()
        
        # Nodes
        nodes_marker = Marker()
        nodes_marker.header.frame_id = 'map'
        nodes_marker.header.stamp = self.get_clock().now().to_msg()
        nodes_marker.type = Marker.SPHERE_LIST
        nodes_marker.scale.x = nodes_marker.scale.y = nodes_marker.scale.z = 0.1
        nodes_marker.color.r = 1.0
        nodes_marker.color.a = 1.0
        
        for pose in self.nodes:
            from geometry_msgs.msg import Point
            p = Point()
            p.x, p.y = pose[0], pose[1]
            nodes_marker.points.append(p)
        
        msg.markers.append(nodes_marker)
        self.graph_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
