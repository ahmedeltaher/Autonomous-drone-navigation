#!/usr/bin/env python3
"""
Position Hold Node
Stable hovering without GPS using sensor fusion

Epic 3: Autonomous Navigation Stack
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
import math


class PositionHoldNode(Node):
    def __init__(self):
        super().__init__('position_hold_node')
        
        # PID Parameters
        self.declare_parameter('kp_xy', 1.0)  # Position proportional gain
        self.declare_parameter('ki_xy', 0.1)  # Position integral gain
        self.declare_parameter('kd_xy', 0.5)  # Position derivative gain
        self.declare_parameter('kp_z', 1.5)   # Altitude proportional gain
        self.declare_parameter('ki_z', 0.2)   # Altitude integral gain
        self.declare_parameter('kd_z', 0.8)   # Altitude derivative gain
        
        # Limits
        self.declare_parameter('max_velocity_xy', 2.0)  # m/s
        self.declare_parameter('max_velocity_z', 1.0)   # m/s
        self.declare_parameter('max_tilt_angle', 0.5)   # rad (~30 deg)
        
        # Control parameters
        self.declare_parameter('control_rate', 50.0)  # Hz
        self.declare_parameter('hover_throttle', 0.5)  # Baseline throttle
        
        self.kp_xy = self.get_parameter('kp_xy').value
        self.ki_xy = self.get_parameter('ki_xy').value
        self.kd_xy = self.get_parameter('kd_xy').value
        self.kp_z = self.get_parameter('kp_z').value
        self.ki_z = self.get_parameter('ki_z').value
        self.kd_z = self.get_parameter('kd_z').value
        
        self.max_vel_xy = self.get_parameter('max_velocity_xy').value
        self.max_vel_z = self.get_parameter('max_velocity_z').value
        self.max_tilt = self.get_parameter('max_tilt_angle').value
        
        self.control_rate = self.get_parameter('control_rate').value
        self.hover_throttle = self.get_parameter('hover_throttle').value
        
        # State
        self.current_pose = None
        self.target_pose = None
        self.current_velocity = np.zeros(3)
        self.current_orientation = None
        
        # PID state
        self.integral_xy = np.zeros(2)
        self.integral_z = 0.0
        self.prev_error_xy = np.zeros(2)
        self.prev_error_z = 0.0
        self.dt = 1.0 / self.control_rate
        
        # Mode
        self.hold_enabled = False
        
        # Subscribers
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.create_subscription(
            PoseStamped,
            '/hold_setpoint',
            self.setpoint_callback,
            10
        )
        
        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Publishers
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel_hold',
            10
        )
        
        # Control loop
        self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('Position Hold Node initialized')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
        self.get_logger().info(f'PID gains - XY: [{self.kp_xy}, {self.ki_xy}, {self.kd_xy}]')
        self.get_logger().info(f'PID gains - Z: [{self.kp_z}, {self.ki_z}, {self.kd_z}]')
    
    def odom_callback(self, msg):
        """Update current pose and velocity."""
        self.current_pose = msg.pose.pose.position
        
        # Update velocity
        self.current_velocity[0] = msg.twist.twist.linear.x
        self.current_velocity[1] = msg.twist.twist.linear.y
        self.current_velocity[2] = msg.twist.twist.linear.z
        
        # If no target set, use current position
        if self.target_pose is None:
            self.target_pose = np.array([
                self.current_pose.x,
                self.current_pose.y,
                self.current_pose.z
            ])
            self.hold_enabled = True
            self.get_logger().info(
                f'Hold position set: [{self.target_pose[0]:.2f}, '
                f'{self.target_pose[1]:.2f}, {self.target_pose[2]:.2f}]'
            )
    
    def setpoint_callback(self, msg):
        """Update target position."""
        self.target_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        self.hold_enabled = True
        
        # Reset integral terms
        self.integral_xy = np.zeros(2)
        self.integral_z = 0.0
        
        self.get_logger().info(
            f'New hold position: [{self.target_pose[0]:.2f}, '
            f'{self.target_pose[1]:.2f}, {self.target_pose[2]:.2f}]'
        )
    
    def imu_callback(self, msg):
        """Store current orientation."""
        self.current_orientation = msg.orientation
    
    def control_loop(self):
        """Main position hold control loop."""
        if not self.hold_enabled or self.current_pose is None:
            return
        
        # Calculate position error
        current_pos = np.array([
            self.current_pose.x,
            self.current_pose.y,
            self.current_pose.z
        ])
        
        error = self.target_pose - current_pos
        error_xy = error[:2]
        error_z = error[2]
        
        # PID control for XY
        self.integral_xy += error_xy * self.dt
        derivative_xy = (error_xy - self.prev_error_xy) / self.dt
        
        vel_cmd_xy = (
            self.kp_xy * error_xy +
            self.ki_xy * self.integral_xy +
            self.kd_xy * derivative_xy
        )
        
        # PID control for Z
        self.integral_z += error_z * self.dt
        derivative_z = (error_z - self.prev_error_z) / self.dt
        
        vel_cmd_z = (
            self.kp_z * error_z +
            self.ki_z * self.integral_z +
            self.kd_z * derivative_z
        )
        
        # Apply velocity limits
        vel_xy_mag = np.linalg.norm(vel_cmd_xy)
        if vel_xy_mag > self.max_vel_xy:
            vel_cmd_xy = vel_cmd_xy / vel_xy_mag * self.max_vel_xy
        
        vel_cmd_z = np.clip(vel_cmd_z, -self.max_vel_z, self.max_vel_z)
        
        # Update previous errors
        self.prev_error_xy = error_xy
        self.prev_error_z = error_z
        
        # Publish velocity command
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'map'
        
        cmd.twist.linear.x = vel_cmd_xy[0]
        cmd.twist.linear.y = vel_cmd_xy[1]
        cmd.twist.linear.z = vel_cmd_z
        
        self.cmd_pub.publish(cmd)
        
        # Log status
        pos_error = np.linalg.norm(error)
        if pos_error > 0.1:  # Only log if error > 10cm
            self.get_logger().info(
                f'Position error: {pos_error:.3f}m, '
                f'Vel cmd: [{vel_cmd_xy[0]:.2f}, {vel_cmd_xy[1]:.2f}, {vel_cmd_z:.2f}]',
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = PositionHoldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
