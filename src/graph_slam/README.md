# Graph-Based SLAM

**Feature ID: 1.1.6 - Graph-Based SLAM**  
**Priority: P0**

Loop closure detection and pose graph optimization for drift-free SLAM.

---

## 📋 Overview

Implements graph-based SLAM with:
- Keyframe-based pose graph
- Loop closure detection via scan matching
- Pose graph optimization using least squares

---

## 🚀 Quick Start

```bash
# Build
colcon build --packages-select graph_slam

# Launch
ros2 launch graph_slam graph_slam.launch.py
```

---

## 📊 Topics

### Input
- `/slam/pose` - Raw SLAM poses
- `/scan` - Lidar scans for loop closure

### Output
- `/graph_slam/path` - Optimized trajectory
- `/graph_slam/graph` - Pose graph visualization

---

## ⚙️ Parameters

```yaml
keyframe_distance: 0.5      # m between keyframes
loop_closure_distance: 2.0  # m for loop detection
optimize_frequency: 1.0     # Hz
```

---

## 🎯 Performance

- Keyframes: Every 0.5m
- Loop Closure: < 2m distance
- Optimization: 1 Hz

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
