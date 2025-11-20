# Global Path Planner

**Epic 3: Autonomous Navigation Stack**  
**Priority: P0**

A* algorithm for optimal path planning on occupancy grids.

---

## 📋 Overview

Global path planner with:
- A* search algorithm
- 8-connected grid search
- Path smoothing
- Optimal path guarantee

---

## 🚀 Quick Start

```bash
# Build
colcon build --packages-select global_planner

# Launch
ros2 launch global_planner planner.launch.py

# Send goal
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{...}"
```

---

## 📊 Topics

### Input
- `/static_map` - Occupancy grid
- `/robot_pose` - Current pose
- `/goal_pose` - Target pose

### Output
- `/global_plan` - Planned path

---

## ⚙️ Parameters

```yaml
allow_diagonal: true     # 8-connected
cost_diagonal: 1.414     # sqrt(2)
occupied_threshold: 50   # 0-100
path_smoothing: true     # Remove waypoints
```

---

## 🎯 Algorithm

**A* Search**:
```
f(n) = g(n) + h(n)
```
- g(n): Cost from start
- h(n): Heuristic to goal (Euclidean)

**Path Smoothing**:
- Line-of-sight optimization
- Removes unnecessary waypoints

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
