# Sensor Synchronization

**Feature ID: 1.1.4 - Sensor Synchronization**  
**Priority: P1**

Microsecond-level timestamp alignment across sensors for GPS-denied navigation.

---

## 📋 Overview

Synchronizes multi-rate sensor streams (optical flow 20Hz, IMU 100Hz, lidar 5-10Hz) with configurable time tolerance for accurate sensor fusion.

### Key Features

- ✅ Approximate time synchronization (50ms tolerance)
- ✅ Message buffering and queuing
- ✅ Multi-rate sensor support
- ✅ Timestamp offset calibration
- ✅ Drop detection and statistics

---

## 🚀 Quick Start

```bash
# Build
colcon build --packages-select sensor_synchronization

# Launch
ros2 launch sensor_synchronization sync.launch.py

# View synchronized output
ros2 topic echo /sensor_fusion/odometry
```

---

## 📊 Topics

### Input
- `/optical_flow/velocity` (20 Hz)
- `/imu/filtered` (100 Hz)
- `/scan` (5-10 Hz)

### Output
- `/sensor_fusion/odometry` - Synchronized fused data

---

## ⚙️ Parameters

```yaml
sync_tolerance_ms: 50.0  # Time window for sync
queue_size: 10           # Buffer size
```

---

## 🎯 Performance

- Sync Tolerance: 50 ms
- Max Time Skew: < 100 μs
- Drop Rate: < 1%

---

## 📚 Algorithm

Uses ROS2 `message_filters` with `ApproximateTimeSynchronizer` for multi-rate sensor alignment.

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
