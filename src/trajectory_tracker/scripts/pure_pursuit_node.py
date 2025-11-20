#!/usr/bin/env python3
"""
Pure Pursuit Trajectory Tracker Node
Smooth path following with lookahead-based control

Epic 3: Autonomous Navigation Stack
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
import numpy as np
import math


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # Parameters
        self.declare_parameter('lookahead_distance', 1.0)  # meters
        self.declare_parameter('lookahead_gain', 0.5)  # velocity-based gain
        self.declare_parameter('min_lookahead', 0.5)  # meters
        self.declare_parameter('max_lookahead', 3.0)  # meters
        self.declare_parameter('linear_velocity', 0.5)  # m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s
        self.declare_parameter('goal_tolerance', 0.2)  # meters
        self.declare_parameter('path_tolerance', 0.5)  # meters
        
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.lookahead_gain = self.get_parameter('lookahead_gain').value
        self.min_lookahead = self.get_parameter('min_lookahead').value
        self.max_lookahead = self.get_parameter('max_lookahead').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.path_tol = self.get_parameter('path_tolerance').value
        
        # State
        self.current_pose = None
        self.path = None
        self.current_index = 0
        
        # Subscribers
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.create_subscription(
            Path,
            '/global_plan',
            self.path_callback,
            10
        )
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(Marker, '/pursuit_target', 10)
        
        # Control loop
        self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        self.get_logger().info('Pure Pursuit Tracker initialized')
        self.get_logger().info(f'Lookahead: {self.lookahead}m, Linear vel: {self.linear_vel}m/s')
    
    def odom_callback(self, msg):
        """Update current pose."""
        pose = msg.pose.pose
        self.current_pose = {
            'x': pose.position.x,
            'y': pose.position.y,
            'theta': self.quaternion_to_yaw(pose.orientation)
        }
    
    def path_callback(self, msg):
        """Receive new path to follow."""
        if len(msg.poses) == 0:
            return
        
        self.path = msg
        self.current_index = 0
        self.get_logger().info(f'New path received with {len(msg.poses)} points')
    
    def control_loop(self):
        """Main trajectory tracking control loop."""
        if self.current_pose is None or self.path is None:
            return
        
        # Check if goal reached
        if self.current_index >= len(self.path.poses):
            self.get_logger().info('Path completed!')
            self.stop_robot()
            self.path = None
            return
        
        # Get goal point
        goal = self.path.poses[-1]
        goal_dist = math.sqrt(
            (goal.pose.position.x - self.current_pose['x'])**2 +
            (goal.pose.position.y - self.current_pose['y'])**2
        )
        
        if goal_dist < self.goal_tol:
            self.get_logger().info('Goal reached!')
            self.stop_robot()
            self.path = None
            return
        
        # Find lookahead point
        lookahead_point = self.find_lookahead_point()
        
        if lookahead_point is None:
            self.get_logger().warn('No lookahead point found!')
            self.stop_robot()
            return
        
        # Calculate control commands
        linear_vel, angular_vel = self.pure_pursuit_control(lookahead_point)
        
        # Publish commands
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)
        
        # Visualize target
        self.visualize_target(lookahead_point)
    
    def find_lookahead_point(self):
        """Find point on path at lookahead distance."""
        # Adaptive lookahead based on velocity
        current_vel = abs(self.linear_vel)
        lookahead = np.clip(
            self.lookahead + self.lookahead_gain * current_vel,
            self.min_lookahead,
            self.max_lookahead
        )
        
        # Find closest point on path
        min_dist = float('inf')
        closest_idx = self.current_index
        
        for i in range(self.current_index, len(self.path.poses)):
            pose = self.path.poses[i]
            dist = math.sqrt(
                (pose.pose.position.x - self.current_pose['x'])**2 +
                (pose.pose.position.y - self.current_pose['y'])**2
            )
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        self.current_index = closest_idx
        
        # Find point at lookahead distance
        for i in range(closest_idx, len(self.path.poses)):
            pose = self.path.poses[i]
            dist = math.sqrt(
                (pose.pose.position.x - self.current_pose['x'])**2 +
                (pose.pose.position.y - self.current_pose['y'])**2
            )
            
            if dist >= lookahead:
                return {
                    'x': pose.pose.position.x,
                    'y': pose.pose.position.y
                }
        
        # Return goal if no point at lookahead distance
        goal = self.path.poses[-1]
        return {
            'x': goal.pose.position.x,
            'y': goal.pose.position.y
        }
    
    def pure_pursuit_control(self, target):
        """Calculate control commands using pure pursuit."""
        # Transform target to robot frame
        dx = target['x'] - self.current_pose['x']
        dy = target['y'] - self.current_pose['y']
        
        # Rotate to robot frame
        cos_theta = math.cos(-self.current_pose['theta'])
        sin_theta = math.sin(-self.current_pose['theta'])
        
        target_x = dx * cos_theta - dy * sin_theta
        target_y = dx * sin_theta + dy * cos_theta
        
        # Calculate curvature
        lookahead_dist = math.sqrt(target_x**2 + target_y**2)
        
        if lookahead_dist < 0.01:
            return 0.0, 0.0
        
        # Pure pursuit formula
        curvature = 2.0 * target_y / (lookahead_dist ** 2)
        
        # Calculate velocities
        linear_vel = self.linear_vel
        angular_vel = curvature * linear_vel
        
        # Limit angular velocity
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        # Reduce linear velocity for sharp turns
        if abs(angular_vel) > 0.5:
            linear_vel *= (1.0 - abs(angular_vel) / self.max_angular_vel)
        
        return linear_vel, angular_vel
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)
    
    def visualize_target(self, target):
        """Visualize lookahead target point."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = target['x']
        marker.pose.position.y = target['y']
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.target_pub.publish(marker)
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
