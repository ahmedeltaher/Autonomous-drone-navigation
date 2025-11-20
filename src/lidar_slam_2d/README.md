# 2D Lidar SLAM

**Feature ID: 1.1.3 - 2D Lidar SLAM**  
**Priority: P0**

Horizontal plane mapping and localization for GPS-denied indoor navigation.

---

## 📋 Overview

This package implements 2D SLAM using lidar for autonomous indoor navigation. It provides real-time mapping, localization, and occupancy grid generation.

### Key Features

- ✅ ICP-based scan matching
- ✅ Occupancy grid mapping (5cm resolution)
- ✅ Real-time pose estimation
- ✅ Map saving/loading
- ✅ Particle filter localization
- ✅ Integration with IMU and optical flow

---

## 🚀 Quick Start

### Build

```bash
colcon build --packages-select lidar_slam_2d
source install/setup.bash
```

### Launch SLAM

```bash
# Start SLAM
ros2 launch lidar_slam_2d slam.launch.py

# View map in RViz
rviz2
```

### Save Map

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

---

## 📊 Topics

### Input
- `/scan` - LaserScan from RPLidar
- `/optical_flow/odometry` - Odometry initial guess

### Output
- `/slam/pose` - Robot pose estimate
- `/slam/map` - Occupancy grid map

---

## ⚙️ Parameters

```yaml
map_resolution: 0.05    # 5cm per pixel
map_size_x: 100.0       # 100m x 100m map
scan_matching_rate: 5.0 # 5 Hz matching
```

---

## 🎯 Performance

- Map Resolution: 5cm
- Update Rate: 5 Hz
- Pose Accuracy: < 10cm
- Map Size: 100m x 100m

---

## 📚 Algorithm

**ICP Scan Matching**:
1. Extract scan points
2. Transform to map frame
3. Find correspondences
4. Optimize pose (SVD)
5. Update occupancy grid

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
