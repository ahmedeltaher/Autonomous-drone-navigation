#!/usr/bin/env python3
"""
Complementary Filter Node
Fuses accelerometer and gyroscope data for improved orientation estimation

The complementary filter combines:
- High-frequency gyroscope data (short-term accuracy)
- Low-frequency accelerometer data (long-term accuracy, gravity reference)

This provides drift-free orientation estimation suitable for navigation.

Feature ID: 1.1.2 - IMU Data Processing
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from std_msgs.msg import Float64MultiArray

import numpy as np
import math


class ComplementaryFilterNode(Node):
    """
    ROS2 node implementing complementary filter for orientation estimation.
    
    Subscribes to:
        - /imu/filtered: Filtered IMU data
    
    Publishes:
        - /imu/orientation_cf: Complementary filter orientation
        - /imu/euler_angles: Roll, pitch, yaw in radians
        - /imu/attitude_correction: Accelerometer-based correction
    """
    
    def __init__(self):
        super().__init__('complementary_filter_node')
        
        # Declare parameters
        self.declare_parameter('alpha', 0.98)  # Complementary filter coefficient
        self.declare_parameter('update_rate', 100.0)  # Expected Hz
        self.declare_parameter('gravity', 9.81)  # m/s^2
        self.declare_parameter('use_magnetometer', False)  # For yaw correction
        
        # Get parameters
        self.alpha = self.get_parameter('alpha').value
        self.update_rate = self.get_parameter('update_rate').value
        self.gravity = self.get_parameter('gravity').value
        self.use_mag = self.get_parameter('use_magnetometer').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/filtered',
            self.imu_callback,
            sensor_qos
        )
        
        # Publishers
        self.orientation_pub = self.create_publisher(
            QuaternionStamped,
            '/imu/orientation_cf',
            10
        )
        
        self.euler_pub = self.create_publisher(
            Vector3Stamped,
            '/imu/euler_angles',
            10
        )
        
        self.correction_pub = self.create_publisher(
            Vector3Stamped,
            '/imu/attitude_correction',
            10
        )
        
        # State
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # quaternion [w, x, y, z]
        self.last_timestamp = None
        
        # Statistics
        self.message_count = 0
        
        # Timer for status
        self.create_timer(5.0, self.status_callback)
        
        self.get_logger().info('Complementary Filter Node initialized')
        self.get_logger().info(f'Filter alpha: {self.alpha} (gyro weight)')
        self.get_logger().info(f'Expected rate: {self.update_rate} Hz')
    
    def imu_callback(self, msg: Imu):
        """
        Process IMU data using complementary filter.
        """
        self.message_count += 1
        
        # Extract measurements
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Calculate dt
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_timestamp is None:
            self.last_timestamp = current_time
            return
        
        dt = current_time - self.last_timestamp
        if dt <= 0 or dt > 1.0:  # Sanity check
            self.last_timestamp = current_time
            return
        
        self.last_timestamp = current_time
        
        # Apply complementary filter
        self.complementary_filter(accel, gyro, dt)
        
        # Publish results
        self.publish_orientation(msg.header.stamp)
        self.publish_euler_angles(msg.header.stamp)
    
    def complementary_filter(self, accel, gyro, dt):
        """
        Complementary filter implementation.
        
        orientation = alpha * (orientation + gyro * dt) + (1 - alpha) * accel_orientation
        
        Where:
        - alpha: weight for gyroscope (high-pass)
        - (1-alpha): weight for accelerometer (low-pass)
        """
        # Step 1: Integrate gyroscope (high-frequency component)
        gyro_orientation = self.integrate_gyro(gyro, dt)
        
        # Step 2: Get orientation from accelerometer (low-frequency component)
        accel_orientation = self.orientation_from_accel(accel)
        
        # Step 3: Blend using complementary filter
        # Convert quaternions to avoid gimbal lock issues
        self.orientation = self.slerp(
            gyro_orientation,
            accel_orientation,
            1.0 - self.alpha
        )
        
        # Normalize
        norm = np.linalg.norm(self.orientation)
        if norm > 0:
            self.orientation = self.orientation / norm
    
    def integrate_gyro(self, gyro, dt):
        """
        Integrate gyroscope to predict orientation.
        """
        q = self.orientation.copy()
        wx, wy, wz = gyro
        
        # Quaternion derivative
        omega = np.array([
            [0, -wx, -wy, -wz],
            [wx, 0, wz, -wy],
            [wy, -wz, 0, wx],
            [wz, wy, -wx, 0]
        ])
        
        q_dot = 0.5 * omega @ q
        q_new = q + q_dot * dt
        
        # Normalize
        norm = np.linalg.norm(q_new)
        if norm > 0:
            return q_new / norm
        return q
    
    def orientation_from_accel(self, accel):
        """
        Calculate orientation (roll and pitch) from accelerometer.
        Assumes gravity is the only force (valid when not accelerating).
        """
        # Normalize accelerometer
        accel_norm = np.linalg.norm(accel)
        if accel_norm < 0.1:  # Avoid division by zero
            return self.orientation
        
        accel_normalized = accel / accel_norm
        
        # Calculate roll and pitch from gravity vector
        roll = math.atan2(accel_normalized[1], accel_normalized[2])
        pitch = math.atan2(-accel_normalized[0],
                          math.sqrt(accel_normalized[1]**2 + accel_normalized[2]**2))
        
        # Keep current yaw (accelerometer can't measure yaw)
        _, _, yaw = self.quaternion_to_euler(self.orientation)
        
        # Convert to quaternion
        return self.euler_to_quaternion(roll, pitch, yaw)
    
    def slerp(self, q1, q2, t):
        """
        Spherical linear interpolation between two quaternions.
        """
        # Ensure shortest path
        dot = np.dot(q1, q2)
        if dot < 0:
            q2 = -q2
            dot = -dot
        
        # If quaternions are very close, use linear interpolation
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        # Calculate angle between quaternions
        theta = math.acos(np.clip(dot, -1.0, 1.0))
        
        # SLERP formula
        sin_theta = math.sin(theta)
        w1 = math.sin((1.0 - t) * theta) / sin_theta
        w2 = math.sin(t * theta) / sin_theta
        
        return w1 * q1 + w2 * q2
    
    def quaternion_to_euler(self, q):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).
        """
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return np.array([w, x, y, z])
    
    def publish_orientation(self, timestamp):
        """
        Publish filtered orientation as quaternion.
        """
        msg = QuaternionStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'imu_link'
        msg.quaternion.w = float(self.orientation[0])
        msg.quaternion.x = float(self.orientation[1])
        msg.quaternion.y = float(self.orientation[2])
        msg.quaternion.z = float(self.orientation[3])
        
        self.orientation_pub.publish(msg)
    
    def publish_euler_angles(self, timestamp):
        """
        Publish Euler angles (roll, pitch, yaw).
        """
        roll, pitch, yaw = self.quaternion_to_euler(self.orientation)
        
        msg = Vector3Stamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'imu_link'
        msg.vector.x = roll
        msg.vector.y = pitch
        msg.vector.z = yaw
        
        self.euler_pub.publish(msg)
    
    def status_callback(self):
        """
        Periodic status reporting.
        """
        roll, pitch, yaw = self.quaternion_to_euler(self.orientation)
        self.get_logger().info(
            f'Status - Messages: {self.message_count}, '
            f'Orientation (deg): Roll={math.degrees(roll):.1f}, '
            f'Pitch={math.degrees(pitch):.1f}, Yaw={math.degrees(yaw):.1f}'
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = ComplementaryFilterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down Complementary Filter Node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
