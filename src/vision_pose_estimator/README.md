# Vision Pose Estimator

**Epic 4: Flight Controller Integration**  
**Feature**: Vision Pose Estimation  
**Priority**: P0

Inject SLAM pose into PX4 EKF2 for GPS-denied navigation.

---

## 📋 Overview

Vision-based pose estimation:
- SLAM pose integration
- EKF2 injection via MAVROS
- 30 Hz publishing rate
- Covariance support
- TF broadcasting

---

## 🚀 Quick Start

```bash
# Build
colcon build --packages-select vision_pose_estimator

# Launch SLAM first
ros2 launch graph_slam graph_slam.launch.py

# Launch vision pose estimator
ros2 launch vision_pose_estimator vision_pose.launch.py

# Verify pose publishing
ros2 topic echo /mavros/vision_pose/pose
```

---

## 📊 Topics

### Input
- `/odom` - SLAM odometry

### Output
- `/mavros/vision_pose/pose` - Vision pose
- `/mavros/vision_pose/pose_cov` - With covariance

---

## ⚙️ PX4 Configuration

Required PX4 parameters:

```bash
# Enable vision position fusion
param set EKF2_AID_MASK 24

# Vision position delay
param set EKF2_EVP_NOISE 0.1

# Vision angle delay
param set EKF2_EVA_NOISE 0.1
```

---

## 🎯 EKF2 Integration

**Covariance Matrix**:
- Position: 0.01 m²
- Orientation: 0.01 rad²

**Update Rate**: 30 Hz

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
