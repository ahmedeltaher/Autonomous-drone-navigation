# Dynamic Object Filtering

**Feature ID: 1.1.8 - Dynamic Object Filtering**  
**Priority: P1**

Remove moving obstacles from static occupancy grid maps.

---

## 📋 Overview

Filters dynamic objects using:
- Temporal scan tracking
- Motion detection
- Static map generation

---

## 🚀 Quick Start

```bash
# Build
colcon build --packages-select dynamic_filter

# Launch
ros2 launch dynamic_filter filter.launch.py

# View filtered map
ros2 topic echo /static_map
```

---

## 📊 Topics

### Input
- `/scan` - Lidar scans
- `/slam/pose` - Robot pose
- `/occupancy_grid` - Raw map

### Output
- `/static_map` - Filtered map
- `/dynamic_objects` - Detected objects

---

## ⚙️ Parameters

```yaml
history_length: 10       # Scan history
velocity_threshold: 0.2  # m/s
```

---

## 🎯 Algorithm

1. Track scan history (10 frames)
2. Compare consecutive scans
3. Detect moved points (> 0.2 m/s)
4. Remove from static map

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
