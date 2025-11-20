#!/usr/bin/env python3
"""
MAVSDK Offboard Control Node
Send position setpoints via MAVLink

Epic 4: Flight Controller Integration
Feature 1.1.16: MAVSDK Offboard Mode
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
import math


class OffboardControlNode(Node):
    def __init__(self):
        super().__init__('offboard_control_node')
        
        # Parameters
        self.declare_parameter('setpoint_rate', 20.0)  # Hz
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('auto_offboard', False)
        self.declare_parameter('connection_timeout', 5.0)  # seconds
        
        self.setpoint_rate = self.get_parameter('setpoint_rate').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.auto_offboard = self.get_parameter('auto_offboard').value
        self.connection_timeout = self.get_parameter('connection_timeout').value
        
        # State
        self.current_state = State()
        self.current_pose = None
        self.target_pose = None
        self.offboard_enabled = False
        self.armed = False
        
        # Subscribers
        self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.create_subscription(
            PoseStamped,
            '/offboard/target_pose',
            self.target_pose_callback,
            10
        )
        
        # Publishers
        self.setpoint_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )
        
        # Service clients
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        self.arm_client.wait_for_service(timeout_sec=self.connection_timeout)
        self.mode_client.wait_for_service(timeout_sec=self.connection_timeout)
        self.get_logger().info('MAVROS services available')
        
        # Setpoint publishing loop
        self.dt = 1.0 / self.setpoint_rate
        self.create_timer(self.dt, self.publish_setpoint)
        
        # Mode management timer
        self.create_timer(1.0, self.manage_mode)
        
        self.get_logger().info('MAVSDK Offboard Control Node initialized')
        self.get_logger().info(f'Setpoint rate: {self.setpoint_rate} Hz')
        self.get_logger().info(f'Auto-arm: {self.auto_arm}, Auto-offboard: {self.auto_offboard}')
    
    def state_callback(self, msg):
        """Update flight controller state."""
        self.current_state = msg
        self.offboard_enabled = (msg.mode == 'OFFBOARD')
        self.armed = msg.armed
    
    def odom_callback(self, msg):
        """Update current pose from odometry."""
        self.current_pose = msg.pose.pose
        
        # If no target set, hover at current position
        if self.target_pose is None and self.current_pose is not None:
            self.target_pose = PoseStamped()
            self.target_pose.pose = self.current_pose
            self.get_logger().info(
                f'Target set to current position: '
                f'({self.current_pose.position.x:.2f}, '
                f'{self.current_pose.position.y:.2f}, '
                f'{self.current_pose.position.z:.2f})'
            )
    
    def target_pose_callback(self, msg):
        """Receive new target pose."""
        self.target_pose = msg
        self.get_logger().info(
            f'New target: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})'
        )
    
    def publish_setpoint(self):
        """Publish position setpoint at high rate."""
        if self.target_pose is None:
            return
        
        # Create position target message
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # Position control
        target.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        
        # Set position
        target.position.x = self.target_pose.pose.position.x
        target.position.y = self.target_pose.pose.position.y
        target.position.z = self.target_pose.pose.position.z
        
        # Set yaw (from quaternion)
        yaw = self.quaternion_to_yaw(self.target_pose.pose.orientation)
        target.yaw = yaw
        
        self.setpoint_pub.publish(target)
    
    def manage_mode(self):
        """Manage offboard mode and arming."""
        if not self.current_state.connected:
            return
        
        # Auto-enable offboard mode
        if self.auto_offboard and not self.offboard_enabled:
            self.set_offboard_mode()
        
        # Auto-arm if in offboard mode
        if self.auto_arm and self.offboard_enabled and not self.armed:
            self.arm_vehicle()
    
    def set_offboard_mode(self):
        """Set vehicle to offboard mode."""
        request = SetMode.Request()
        request.custom_mode = 'OFFBOARD'
        
        future = self.mode_client.call_async(request)
        future.add_done_callback(self.mode_callback)
    
    def mode_callback(self, future):
        """Handle mode change response."""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('Offboard mode enabled')
            else:
                self.get_logger().warn('Failed to set offboard mode')
        except Exception as e:
            self.get_logger().error(f'Mode service call failed: {e}')
    
    def arm_vehicle(self):
        """Arm the vehicle."""
        request = CommandBool.Request()
        request.value = True
        
        future = self.arm_client.call_async(request)
        future.add_done_callback(self.arm_callback)
    
    def arm_callback(self, future):
        """Handle arming response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Vehicle armed')
            else:
                self.get_logger().warn('Failed to arm vehicle')
        except Exception as e:
            self.get_logger().error(f'Arming service call failed: {e}')
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
