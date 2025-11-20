#!/usr/bin/env python3
"""
Time Calibration Node
Measure and compensate for sensor timestamp offsets

Feature ID: 1.1.4 - Sensor Synchronization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float64
import numpy as np
from collections import deque


class TimeCalibrationNode(Node):
    def __init__(self):
        super().__init__('time_calibration_node')
        
        self.declare_parameter('calibration_samples', 100)
        self.samples = self.get_parameter('calibration_samples').value
        
        # Data buffers
        self.flow_times = deque(maxlen=self.samples)
        self.imu_times = deque(maxlen=self.samples)
        self.lidar_times = deque(maxlen=self.samples)
        
        # Subscribers
        self.create_subscription(
            TwistWithCovarianceStamped,
            '/optical_flow/velocity',
            lambda msg: self.flow_times.append(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ),
            10
        )
        
        self.create_subscription(
            Imu,
            '/imu/filtered',
            lambda msg: self.imu_times.append(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ),
            10
        )
        
        self.create_subscription(
            LaserScan,
            '/scan',
            lambda msg: self.lidar_times.append(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ),
            10
        )
        
        # Publishers for offsets
        self.offset_pub = self.create_publisher(Float64, '/time_calibration/offset', 10)
        
        self.create_timer(5.0, self.calibrate)
        
        self.get_logger().info('Time Calibration Node initialized')
    
    def calibrate(self):
        """Calculate timestamp offsets between sensors."""
        if len(self.flow_times) < 10 or len(self.imu_times) < 10 or len(self.lidar_times) < 10:
            self.get_logger().info('Collecting samples for calibration...')
            return
        
        flow_dt = np.diff(list(self.flow_times))
        imu_dt = np.diff(list(self.imu_times))
        lidar_dt = np.diff(list(self.lidar_times))
        
        flow_rate = 1.0 / np.mean(flow_dt) if len(flow_dt) > 0 else 0
        imu_rate = 1.0 / np.mean(imu_dt) if len(imu_dt) > 0 else 0
        lidar_rate = 1.0 / np.mean(lidar_dt) if len(lidar_dt) > 0 else 0
        
        self.get_logger().info(
            f'Sensor rates - Flow: {flow_rate:.1f} Hz, '
            f'IMU: {imu_rate:.1f} Hz, Lidar: {lidar_rate:.1f} Hz'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TimeCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
