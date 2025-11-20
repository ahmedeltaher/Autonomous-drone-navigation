# Occupancy Grid Mapping

**Feature ID: 1.1.7 - Occupancy Grid Mapping**  
**Priority: P0**

Probabilistic 2D/2.5D environment representation for navigation.

---

## 📋 Overview

Creates occupancy grid maps using:
- Log-odds probabilistic updates
- Bresenham ray tracing
- Multi-map merging

---

## 🚀 Quick Start

```bash
# Build
colcon build --packages-select occupancy_grid_mapping

# Launch
ros2 launch occupancy_grid_mapping mapping.launch.py

# View in RViz
rviz2
```

---

## 📊 Topics

### Input
- `/scan` - Lidar scans
- `/slam/pose` - Robot pose

### Output
- `/occupancy_grid` - Occupancy grid (0-100)
- `/merged_map` - Merged maps

---

## ⚙️ Parameters

```yaml
resolution: 0.05    # 5cm per cell
width: 200          # 10m x 10m map
prob_occ: 0.7       # Hit probability
prob_free: 0.3      # Miss probability
```

---

## 🎯 Algorithm

**Log-Odds Occupancy Mapping**:
1. Ray trace from robot to endpoint
2. Update free cells along ray
3. Update occupied cell at endpoint
4. Convert log-odds to probability

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
