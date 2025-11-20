#!/usr/bin/env python3
"""
Failsafe Controller Node
Automatic landing on SLAM failure for safety

Epic 4: Flight Controller Integration
Feature: Fail-Safe Logic
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from std_msgs.msg import Bool
from enum import Enum
import time


class FailsafeState(Enum):
    NORMAL = 0
    WARNING = 1
    EMERGENCY = 2
    LANDING = 3
    LANDED = 4


class FailsafeNode(Node):
    def __init__(self):
        super().__init__('failsafe_node')
        
        # Parameters
        self.declare_parameter('odom_timeout', 2.0)  # seconds
        self.declare_parameter('min_update_rate', 5.0)  # Hz
        self.declare_parameter('position_jump_threshold', 5.0)  # meters
        self.declare_parameter('warning_time', 3.0)  # seconds
        self.declare_parameter('emergency_descent_rate', 0.5)  # m/s
        self.declare_parameter('landing_height', 0.3)  # meters
        self.declare_parameter('auto_arm_disarm', True)
        
        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.min_rate = self.get_parameter('min_update_rate').value
        self.pos_jump_thresh = self.get_parameter('position_jump_threshold').value
        self.warning_time = self.get_parameter('warning_time').value
        self.descent_rate = self.get_parameter('emergency_descent_rate').value
        self.landing_height = self.get_parameter('landing_height').value
        self.auto_arm_disarm = self.get_parameter('auto_arm_disarm').value
        
        # State
        self.failsafe_state = FailsafeState.NORMAL
        self.last_odom_time = None
        self.last_position = None
        self.odom_count = 0
        self.warning_start_time = None
        self.flight_controller_state = State()
        self.current_altitude = 0.0
        
        # Subscribers
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        # Publishers
        self.failsafe_pub = self.create_publisher(
            Bool,
            '/failsafe/active',
            10
        )
        
        self.emergency_land_pub = self.create_publisher(
            PoseStamped,
            '/emergency/land_target',
            10
        )
        
        # Service clients
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        
        # Monitoring timer
        self.create_timer(0.1, self.monitor_slam_health)  # 10 Hz
        
        # Rate calculation timer
        self.create_timer(1.0, self.calculate_odom_rate)
        self.last_odom_count = 0
        self.current_rate = 0.0
        
        self.get_logger().info('Failsafe Controller initialized')
        self.get_logger().info(f'SLAM timeout: {self.odom_timeout}s')
        self.get_logger().info(f'Min update rate: {self.min_rate} Hz')
    
    def odom_callback(self, msg):
        """Monitor SLAM odometry."""
        current_time = self.get_clock().now()
        self.last_odom_time = current_time
        self.odom_count += 1
        
        # Check for position jumps
        current_pos = msg.pose.pose.position
        
        if self.last_position is not None:
            jump = (
                (current_pos.x - self.last_position.x)**2 +
                (current_pos.y - self.last_position.y)**2 +
                (current_pos.z - self.last_position.z)**2
            )**0.5
            
            if jump > self.pos_jump_thresh:
                self.get_logger().warn(
                    f'Position jump detected: {jump:.2f}m! SLAM may be unstable.'
                )
                self.trigger_warning()
        
        self.last_position = current_pos
        self.current_altitude = current_pos.z
    
    def state_callback(self, msg):
        """Update flight controller state."""
        self.flight_controller_state = msg
    
    def calculate_odom_rate(self):
        """Calculate SLAM update rate."""
        new_count = self.odom_count
        self.current_rate = new_count - self.last_odom_count
        self.last_odom_count = new_count
        
        # Check if rate is too low
        if self.current_rate < self.min_rate and self.current_rate > 0:
            self.get_logger().warn(
                f'SLAM update rate low: {self.current_rate:.1f} Hz (min: {self.min_rate} Hz)'
            )
    
    def monitor_slam_health(self):
        """Main monitoring loop for SLAM health."""
        if self.last_odom_time is None:
            return
        
        # Check timeout
        current_time = self.get_clock().now()
        time_since_odom = (current_time - self.last_odom_time).nanoseconds / 1e9
        
        if time_since_odom > self.odom_timeout:
            self.get_logger().error(
                f'SLAM FAILURE! No odometry for {time_since_odom:.2f}s'
            )
            self.trigger_emergency()
        elif time_since_odom > self.odom_timeout * 0.5:
            self.get_logger().warn(
                f'SLAM degraded! No odometry for {time_since_odom:.2f}s'
            )
            self.trigger_warning()
        else:
            # Check if we're in warning state and can recover
            if self.failsafe_state == FailsafeState.WARNING:
                self.get_logger().info('SLAM recovered. Returning to normal.')
                self.failsafe_state = FailsafeState.NORMAL
                self.warning_start_time = None
        
        # Check warning timeout
        if self.failsafe_state == FailsafeState.WARNING:
            if self.warning_start_time is not None:
                warning_duration = time.time() - self.warning_start_time
                if warning_duration > self.warning_time:
                    self.get_logger().error(
                        f'Warning timeout! Triggering emergency landing.'
                    )
                    self.trigger_emergency()
        
        # Execute emergency procedures
        if self.failsafe_state == FailsafeState.EMERGENCY:
            self.execute_emergency_landing()
        
        # Publish failsafe status
        failsafe_msg = Bool()
        failsafe_msg.data = (self.failsafe_state != FailsafeState.NORMAL)
        self.failsafe_pub.publish(failsafe_msg)
    
    def trigger_warning(self):
        """Trigger warning state."""
        if self.failsafe_state == FailsafeState.NORMAL:
            self.failsafe_state = FailsafeState.WARNING
            self.warning_start_time = time.time()
            self.get_logger().warn('FAILSAFE WARNING activated')
    
    def trigger_emergency(self):
        """Trigger emergency landing."""
        if self.failsafe_state not in [FailsafeState.EMERGENCY, FailsafeState.LANDING, FailsafeState.LANDED]:
            self.failsafe_state = FailsafeState.EMERGENCY
            self.get_logger().error('EMERGENCY FAILSAFE ACTIVATED - Initiating landing')
    
    def execute_emergency_landing(self):
        """Execute emergency landing procedure."""
        if self.failsafe_state == FailsafeState.LANDED:
            return
        
        # Check if already landed
        if self.current_altitude < self.landing_height:
            self.get_logger().info('Vehicle at ground level. Disarming.')
            if self.auto_arm_disarm:
                self.disarm_vehicle()
            self.failsafe_state = FailsafeState.LANDED
            return
        
        # Set to landing state
        if self.failsafe_state == FailsafeState.EMERGENCY:
            self.failsafe_state = FailsafeState.LANDING
            self.get_logger().info('Executing emergency landing...')
        
        # Command landing via MAVROS
        if not self.flight_controller_state.armed:
            self.get_logger().info('Vehicle not armed. Landing complete.')
            self.failsafe_state = FailsafeState.LANDED
            return
        
        # Try to set LAND mode
        self.set_land_mode()
    
    def set_land_mode(self):
        """Set flight controller to LAND mode."""
        if not self.mode_client.service_is_ready():
            return
        
        request = SetMode.Request()
        request.custom_mode = 'AUTO.LAND'
        
        future = self.mode_client.call_async(request)
        future.add_done_callback(self.land_mode_callback)
    
    def land_mode_callback(self, future):
        """Handle land mode response."""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('LAND mode activated')
            else:
                self.get_logger().warn('Failed to set LAND mode')
        except Exception as e:
            self.get_logger().error(f'Land mode service call failed: {e}')
    
    def disarm_vehicle(self):
        """Disarm the vehicle."""
        if not self.arm_client.service_is_ready():
            return
        
        request = CommandBool.Request()
        request.value = False
        
        future = self.arm_client.call_async(request)
        future.add_done_callback(self.disarm_callback)
    
    def disarm_callback(self, future):
        """Handle disarm response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Vehicle disarmed')
            else:
                self.get_logger().warn('Failed to disarm vehicle')
        except Exception as e:
            self.get_logger().error(f'Disarm service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FailsafeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
