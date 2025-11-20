#!/usr/bin/env python3
"""
IMU Processing Node
High-frequency pose estimation and vibration filtering

This node processes raw IMU data (accelerometer and gyroscope) to provide:
- Filtered acceleration and angular velocity
- Bias estimation and compensation
- Vibration filtering using low-pass filters
- High-frequency orientation estimation

Feature ID: 1.1.2 - IMU Data Processing
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from std_msgs.msg import Float64

import numpy as np
from scipy.signal import butter, lfilter
from collections import deque


class IMUProcessingNode(Node):
    """
    ROS2 node for IMU data processing with vibration filtering.
    
    Subscribes to:
        - /mavros/imu/data_raw: Raw IMU data from sensor
    
    Publishes:
        - /imu/filtered: Filtered IMU data
        - /imu/accel_filtered: Filtered acceleration
        - /imu/gyro_filtered: Filtered angular velocity
        - /imu/orientation: Estimated orientation
        - /imu/bias/accel: Accelerometer bias estimate
        - /imu/bias/gyro: Gyroscope bias estimate
    """
    
    def __init__(self):
        super().__init__('imu_processing_node')
        
        # Declare parameters
        self.declare_parameter('update_rate', 100.0)  # Hz
        self.declare_parameter('accel_lpf_cutoff', 20.0)  # Hz - low-pass filter for accel
        self.declare_parameter('gyro_lpf_cutoff', 20.0)  # Hz - low-pass filter for gyro
        self.declare_parameter('calibration_samples', 1000)  # Number of samples for bias calibration
        self.declare_parameter('enable_bias_estimation', True)
        self.declare_parameter('gravity', 9.81)  # m/s^2
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.accel_cutoff = self.get_parameter('accel_lpf_cutoff').value
        self.gyro_cutoff = self.get_parameter('gyro_lpf_cutoff').value
        self.calib_samples = self.get_parameter('calibration_samples').value
        self.enable_bias_est = self.get_parameter('enable_bias_estimation').value
        self.gravity = self.get_parameter('gravity').value
        
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
            '/mavros/imu/data_raw',
            self.imu_callback,
            sensor_qos
        )
        
        # Publishers
        self.filtered_imu_pub = self.create_publisher(
            Imu,
            '/imu/filtered',
            10
        )
        
        self.accel_pub = self.create_publisher(
            Vector3Stamped,
            '/imu/accel_filtered',
            10
        )
        
        self.gyro_pub = self.create_publisher(
            Vector3Stamped,
            '/imu/gyro_filtered',
            10
        )
        
        self.orientation_pub = self.create_publisher(
            QuaternionStamped,
            '/imu/orientation',
            10
        )
        
        self.accel_bias_pub = self.create_publisher(
            Vector3Stamped,
            '/imu/bias/accel',
            10
        )
        
        self.gyro_bias_pub = self.create_publisher(
            Vector3Stamped,
            '/imu/bias/gyro',
            10
        )
        
        # Initialize filters
        self.init_filters()
        
        # Bias estimation
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.calibration_data_accel = deque(maxlen=self.calib_samples)
        self.calibration_data_gyro = deque(maxlen=self.calib_samples)
        self.is_calibrated = False
        
        # State for integration
        self.last_timestamp = None
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # quaternion [w, x, y, z]
        
        # Statistics
        self.imu_count = 0
        self.last_rate_check = self.get_clock().now()
        self.messages_since_check = 0
        
        # Timer for status
        self.create_timer(5.0, self.status_callback)
        
        self.get_logger().info('IMU Processing Node initialized')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Accel LPF cutoff: {self.accel_cutoff} Hz')
        self.get_logger().info(f'Gyro LPF cutoff: {self.gyro_cutoff} Hz')
    
    def init_filters(self):
        """
        Initialize Butterworth low-pass filters for vibration filtering.
        """
        nyquist = self.update_rate / 2.0
        
        # Design filters (2nd order Butterworth)
        self.accel_b, self.accel_a = butter(
            2,
            self.accel_cutoff / nyquist,
            btype='low'
        )
        
        self.gyro_b, self.gyro_a = butter(
            2,
            self.gyro_cutoff / nyquist,
            btype='low'
        )
        
        # Filter state buffers (for lfilter zi)
        self.accel_filter_state = {
            'x': deque(maxlen=50),
            'y': deque(maxlen=50),
            'z': deque(maxlen=50)
        }
        
        self.gyro_filter_state = {
            'x': deque(maxlen=50),
            'y': deque(maxlen=50),
            'z': deque(maxlen=50)
        }
    
    def apply_lowpass_filter(self, value, axis, filter_state, b, a):
        """
        Apply low-pass filter to a single axis.
        """
        # Add new value
        filter_state[axis].append(value)
        
        # Need at least 3 samples for filtering
        if len(filter_state[axis]) < 3:
            return value
        
        # Apply filter
        filtered = lfilter(b, a, list(filter_state[axis]))
        
        return filtered[-1]
    
    def imu_callback(self, msg: Imu):
        """
        Process raw IMU data with vibration filtering and bias compensation.
        """
        self.imu_count += 1
        self.messages_since_check += 1
        
        # Extract raw measurements
        accel_raw = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        gyro_raw = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Calibration phase
        if not self.is_calibrated and self.enable_bias_est:
            self.calibration_data_accel.append(accel_raw)
            self.calibration_data_gyro.append(gyro_raw)
            
            if len(self.calibration_data_accel) >= self.calib_samples:
                self.compute_bias()
                self.is_calibrated = True
                self.get_logger().info('IMU calibration complete')
            else:
                # Still calibrating
                if self.imu_count % 100 == 0:
                    progress = len(self.calibration_data_accel) / self.calib_samples * 100
                    self.get_logger().info(f'Calibrating IMU: {progress:.1f}%')
                return
        
        # Apply bias compensation
        accel_corrected = accel_raw - self.accel_bias
        gyro_corrected = gyro_raw - self.gyro_bias
        
        # Apply low-pass filtering (vibration filtering)
        accel_filtered = np.array([
            self.apply_lowpass_filter(
                accel_corrected[0], 'x',
                self.accel_filter_state, self.accel_b, self.accel_a
            ),
            self.apply_lowpass_filter(
                accel_corrected[1], 'y',
                self.accel_filter_state, self.accel_b, self.accel_a
            ),
            self.apply_lowpass_filter(
                accel_corrected[2], 'z',
                self.accel_filter_state, self.accel_b, self.accel_a
            )
        ])
        
        gyro_filtered = np.array([
            self.apply_lowpass_filter(
                gyro_corrected[0], 'x',
                self.gyro_filter_state, self.gyro_b, self.gyro_a
            ),
            self.apply_lowpass_filter(
                gyro_corrected[1], 'y',
                self.gyro_filter_state, self.gyro_b, self.gyro_a
            ),
            self.apply_lowpass_filter(
                gyro_corrected[2], 'z',
                self.gyro_filter_state, self.gyro_b, self.gyro_a
            )
        ])
        
        # Update orientation estimate using gyroscope integration
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_timestamp is not None:
            dt = current_time - self.last_timestamp
            if dt > 0 and dt < 1.0:  # Sanity check
                self.integrate_gyroscope(gyro_filtered, dt)
        self.last_timestamp = current_time
        
        # Publish filtered data
        self.publish_filtered_imu(msg.header.stamp, accel_filtered, gyro_filtered)
        self.publish_accel(msg.header.stamp, accel_filtered)
        self.publish_gyro(msg.header.stamp, gyro_filtered)
        self.publish_orientation(msg.header.stamp)
        
        # Publish bias estimates periodically
        if self.imu_count % 100 == 0:
            self.publish_bias_estimates(msg.header.stamp)
    
    def compute_bias(self):
        """
        Compute accelerometer and gyroscope biases from calibration data.
        """
        # Convert to numpy array
        accel_data = np.array(list(self.calibration_data_accel))
        gyro_data = np.array(list(self.calibration_data_gyro))
        
        # Gyro bias is simply the mean (should be zero when stationary)
        self.gyro_bias = np.mean(gyro_data, axis=0)
        
        # Accel bias: mean minus gravity (assuming Z-axis up)
        accel_mean = np.mean(accel_data, axis=0)
        # Subtract gravity from Z-axis
        self.accel_bias = accel_mean - np.array([0.0, 0.0, self.gravity])
        
        self.get_logger().info(f'Accel bias: {self.accel_bias}')
        self.get_logger().info(f'Gyro bias: {self.gyro_bias}')
    
    def integrate_gyroscope(self, gyro, dt):
        """
        Integrate gyroscope measurements to update orientation estimate.
        Uses quaternion integration for numerical stability.
        """
        # Current quaternion
        q = self.orientation
        
        # Gyro measurements (body frame angular velocity)
        wx, wy, wz = gyro
        
        # Quaternion derivative
        omega = np.array([
            [0, -wx, -wy, -wz],
            [wx, 0, wz, -wy],
            [wy, -wz, 0, wx],
            [wz, wy, -wx, 0]
        ])
        
        # Update quaternion: q_new = q + 0.5 * omega * q * dt
        q_dot = 0.5 * omega @ q
        q_new = q + q_dot * dt
        
        # Normalize quaternion
        q_norm = np.linalg.norm(q_new)
        if q_norm > 0:
            self.orientation = q_new / q_norm
        else:
            self.orientation = np.array([1.0, 0.0, 0.0, 0.0])
    
    def publish_filtered_imu(self, timestamp, accel, gyro):
        """
        Publish complete filtered IMU message.
        """
        msg = Imu()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'imu_link'
        
        # Linear acceleration
        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])
        
        # Angular velocity
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        
        # Orientation (from integration)
        msg.orientation.w = float(self.orientation[0])
        msg.orientation.x = float(self.orientation[1])
        msg.orientation.y = float(self.orientation[2])
        msg.orientation.z = float(self.orientation[3])
        
        # Covariance (simplified)
        msg.linear_acceleration_covariance = [0.01] * 9
        msg.angular_velocity_covariance = [0.001] * 9
        msg.orientation_covariance = [0.01] * 9
        
        self.filtered_imu_pub.publish(msg)
    
    def publish_accel(self, timestamp, accel):
        """
        Publish filtered acceleration.
        """
        msg = Vector3Stamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'imu_link'
        msg.vector.x = float(accel[0])
        msg.vector.y = float(accel[1])
        msg.vector.z = float(accel[2])
        
        self.accel_pub.publish(msg)
    
    def publish_gyro(self, timestamp, gyro):
        """
        Publish filtered angular velocity.
        """
        msg = Vector3Stamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'imu_link'
        msg.vector.x = float(gyro[0])
        msg.vector.y = float(gyro[1])
        msg.vector.z = float(gyro[2])
        
        self.gyro_pub.publish(msg)
    
    def publish_orientation(self, timestamp):
        """
        Publish estimated orientation as quaternion.
        """
        msg = QuaternionStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'imu_link'
        msg.quaternion.w = float(self.orientation[0])
        msg.quaternion.x = float(self.orientation[1])
        msg.quaternion.y = float(self.orientation[2])
        msg.quaternion.z = float(self.orientation[3])
        
        self.orientation_pub.publish(msg)
    
    def publish_bias_estimates(self, timestamp):
        """
        Publish current bias estimates.
        """
        # Accelerometer bias
        accel_msg = Vector3Stamped()
        accel_msg.header.stamp = timestamp
        accel_msg.header.frame_id = 'imu_link'
        accel_msg.vector.x = float(self.accel_bias[0])
        accel_msg.vector.y = float(self.accel_bias[1])
        accel_msg.vector.z = float(self.accel_bias[2])
        self.accel_bias_pub.publish(accel_msg)
        
        # Gyroscope bias
        gyro_msg = Vector3Stamped()
        gyro_msg.header.stamp = timestamp
        gyro_msg.header.frame_id = 'imu_link'
        gyro_msg.vector.x = float(self.gyro_bias[0])
        gyro_msg.vector.y = float(self.gyro_bias[1])
        gyro_msg.vector.z = float(self.gyro_bias[2])
        self.gyro_bias_pub.publish(gyro_msg)
    
    def status_callback(self):
        """
        Periodic status reporting.
        """
        now = self.get_clock().now()
        dt = (now - self.last_rate_check).nanoseconds / 1e9
        
        if dt > 0:
            actual_rate = self.messages_since_check / dt
            self.get_logger().info(
                f'Status - IMU rate: {actual_rate:.1f} Hz, '
                f'Total messages: {self.imu_count}, '
                f'Calibrated: {self.is_calibrated}'
            )
        
        self.last_rate_check = now
        self.messages_since_check = 0


def main(args=None):
    rclpy.init(args=args)
    
    node = IMUProcessingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down IMU Processing Node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
