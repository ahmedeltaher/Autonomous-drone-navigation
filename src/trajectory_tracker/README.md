# Trajectory Tracker

**Epic 3: Autonomous Navigation Stack**  
**Priority: P0**

Pure Pursuit controller for smooth path following.

---

## 📋 Overview

Trajectory tracking with:
- Pure pursuit algorithm
- Adaptive lookahead distance
- Smooth path following
- Goal detection

---

## 🚀 Quick Start

```bash
# Build
colcon build --packages-select trajectory_tracker

# Launch
ros2 launch trajectory_tracker tracker.launch.py

# Follow global plan
ros2 topic echo /cmd_vel
```

---

## 📊 Topics

### Input
- `/odom` - Robot odometry
- `/global_plan` - Path to follow

### Output
- `/cmd_vel` - Velocity commands
- `/pursuit_target` - Lookahead point

---

## ⚙️ Parameters

```yaml
lookahead_distance: 1.0  # meters
linear_velocity: 0.5     # m/s
goal_tolerance: 0.2      # meters
```

---

## 🎯 Algorithm

**Pure Pursuit**:
```
κ = 2y / L²
ω = κ * v
```
- κ: curvature
- y: lateral offset
- L: lookahead distance

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
