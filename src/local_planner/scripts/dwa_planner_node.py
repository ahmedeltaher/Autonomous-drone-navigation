#!/usr/bin/env python3
"""
Dynamic Window Approach (DWA) Local Planner Node
Real-time obstacle avoidance and path following

Epic 3: Autonomous Navigation Stack
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math


class DWAPlannerNode(Node):
    def __init__(self):
        super().__init__('dwa_planner_node')
        
        # DWA Parameters
        self.declare_parameter('max_speed', 0.5)  # m/s
        self.declare_parameter('min_speed', -0.1)  # m/s
        self.declare_parameter('max_yaw_rate', 1.0)  # rad/s
        self.declare_parameter('max_accel', 0.2)  # m/s^2
        self.declare_parameter('max_dyaw_rate', 1.0)  # rad/s^2
        self.declare_parameter('v_resolution', 0.05)  # m/s
        self.declare_parameter('yaw_rate_resolution', 0.1)  # rad/s
        self.declare_parameter('predict_time', 2.0)  # seconds
        self.declare_parameter('dt', 0.1)  # seconds
        
        # Scoring weights
        self.declare_parameter('heading_weight', 1.0)
        self.declare_parameter('clearance_weight', 0.5)
        self.declare_parameter('velocity_weight', 0.3)
        
        # Safety
        self.declare_parameter('robot_radius', 0.3)  # meters
        self.declare_parameter('obstacle_margin', 0.2)  # meters
        
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_dyaw_rate = self.get_parameter('max_dyaw_rate').value
        self.v_res = self.get_parameter('v_resolution').value
        self.yaw_res = self.get_parameter('yaw_rate_resolution').value
        self.predict_time = self.get_parameter('predict_time').value
        self.dt = self.get_parameter('dt').value
        
        self.heading_weight = self.get_parameter('heading_weight').value
        self.clearance_weight = self.get_parameter('clearance_weight').value
        self.velocity_weight = self.get_parameter('velocity_weight').value
        
        self.robot_radius = self.get_parameter('robot_radius').value
        self.obstacle_margin = self.get_parameter('obstacle_margin').value
        
        # State
        self.current_velocity = Twist()
        self.current_pose = None
        self.goal = None
        self.obstacles = []
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_pub = self.create_publisher(Path, '/local_plan', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)
        
        # Control loop
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('DWA Planner Node initialized')
        self.get_logger().info(f'Max speed: {self.max_speed} m/s, Max yaw rate: {self.max_yaw_rate} rad/s')
    
    def odom_callback(self, msg):
        """Update current pose and velocity."""
        self.current_pose = msg.pose.pose
        self.current_velocity.linear.x = msg.twist.twist.linear.x
        self.current_velocity.angular.z = msg.twist.twist.angular.z
    
    def scan_callback(self, msg):
        """Convert laser scan to obstacle points."""
        if self.current_pose is None:
            return
        
        obstacles = []
        angle = msg.angle_min
        
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Convert to global frame
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                obstacles.append([x, y])
            angle += msg.angle_increment
        
        self.obstacles = np.array(obstacles) if len(obstacles) > 0 else np.array([])
    
    def goal_callback(self, msg):
        """Set new goal."""
        self.goal = msg.pose
        self.get_logger().info(f'New goal: ({self.goal.position.x:.2f}, {self.goal.position.y:.2f})')
    
    def control_loop(self):
        """Main DWA control loop."""
        if self.current_pose is None or self.goal is None:
            return
        
        # Check if goal reached
        goal_dist = math.sqrt(
            (self.goal.position.x - self.current_pose.position.x)**2 +
            (self.goal.position.y - self.current_pose.position.y)**2
        )
        
        if goal_dist < 0.2:  # 20cm tolerance
            self.get_logger().info('Goal reached!')
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return
        
        # Run DWA
        best_v, best_w, best_traj = self.dwa()
        
        if best_traj is not None:
            # Publish command
            cmd = Twist()
            cmd.linear.x = best_v
            cmd.angular.z = best_w
            self.cmd_pub.publish(cmd)
            
            # Publish trajectory
            self.publish_trajectory(best_traj)
        else:
            # Stop if no safe trajectory
            self.get_logger().warn('No safe trajectory found!')
            cmd = Twist()
            self.cmd_pub.publish(cmd)
    
    def dwa(self):
        """Dynamic Window Approach algorithm."""
        # Calculate dynamic window
        dw = self.calculate_dynamic_window()
        
        best_score = -float('inf')
        best_v = 0.0
        best_w = 0.0
        best_traj = None
        
        # Sample velocities in dynamic window
        for v in np.arange(dw[0], dw[1], self.v_res):
            for w in np.arange(dw[2], dw[3], self.yaw_res):
                # Predict trajectory
                traj = self.predict_trajectory(v, w)
                
                # Check collision
                if self.check_collision(traj):
                    continue
                
                # Score trajectory
                score = self.score_trajectory(traj, v, w)
                
                if score > best_score:
                    best_score = score
                    best_v = v
                    best_w = w
                    best_traj = traj
        
        return best_v, best_w, best_traj
    
    def calculate_dynamic_window(self):
        """Calculate allowable velocities based on dynamics."""
        # Current velocities
        v = self.current_velocity.linear.x
        w = self.current_velocity.angular.z
        
        # Dynamic window from robot dynamics
        v_min = max(self.min_speed, v - self.max_accel * self.dt)
        v_max = min(self.max_speed, v + self.max_accel * self.dt)
        w_min = max(-self.max_yaw_rate, w - self.max_dyaw_rate * self.dt)
        w_max = min(self.max_yaw_rate, w + self.max_dyaw_rate * self.dt)
        
        return [v_min, v_max, w_min, w_max]
    
    def predict_trajectory(self, v, w):
        """Predict trajectory for given velocities."""
        traj = []
        x = 0.0
        y = 0.0
        theta = 0.0
        
        time = 0.0
        while time <= self.predict_time:
            traj.append([x, y, theta])
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += w * self.dt
            time += self.dt
        
        return np.array(traj)
    
    def check_collision(self, traj):
        """Check if trajectory collides with obstacles."""
        if len(self.obstacles) == 0:
            return False
        
        for point in traj:
            for obs in self.obstacles:
                dist = math.sqrt((point[0] - obs[0])**2 + (point[1] - obs[1])**2)
                if dist < (self.robot_radius + self.obstacle_margin):
                    return True
        
        return False
    
    def score_trajectory(self, traj, v, w):
        """Score trajectory based on heading, clearance, and velocity."""
        # Heading score (alignment with goal)
        final_pos = traj[-1]
        goal_angle = math.atan2(
            self.goal.position.y - self.current_pose.position.y,
            self.goal.position.x - self.current_pose.position.x
        )
        heading_score = math.pi - abs(goal_angle - final_pos[2])
        
        # Clearance score (distance to obstacles)
        clearance_score = float('inf')
        if len(self.obstacles) > 0:
            min_dist = float('inf')
            for point in traj:
                for obs in self.obstacles:
                    dist = math.sqrt((point[0] - obs[0])**2 + (point[1] - obs[1])**2)
                    min_dist = min(min_dist, dist)
            clearance_score = min_dist
        
        # Velocity score (prefer higher speeds)
        velocity_score = abs(v)
        
        # Weighted sum
        total_score = (
            self.heading_weight * heading_score +
            self.clearance_weight * clearance_score +
            self.velocity_weight * velocity_score
        )
        
        return total_score
    
    def publish_trajectory(self, traj):
        """Publish best trajectory for visualization."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'base_link'
        
        for point in traj:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            path.poses.append(pose)
        
        self.traj_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
