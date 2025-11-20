# Waypoint Navigation

**Feature**: 1.1.14 - Waypoint Navigation  
**Priority**: P0

Execute pre-defined indoor waypoint missions.

---

## 📋 Overview

Mission-based waypoint navigation:
- Load waypoints from YAML
- Sequential execution
- Progress tracking
- Visual markers
- Loop missions (optional)

---

## 🚀 Quick Start

```bash
# Build
colcon build --packages-select waypoint_navigation

# Launch with mission file
ros2 launch waypoint_navigation waypoint.launch.py \
  mission_file:=/path/to/mission.yaml \
  auto_start:=true

# Monitor progress
ros2 topic echo /waypoint_markers
```

---

## 📊 Topics

### Input
- `/odom` - Robot odometry

### Output
- `/goal_pose` - Current waypoint goal
- `/waypoint_markers` - Visualization

---

## 📝 Mission File Format

```yaml
mission_name: "My Mission"
description: "Indoor patrol route"

waypoints:
  - x: 0.0
    y: 0.0
    z: 0.0
    name: "Start"
  
  - x: 5.0
    y: 3.0
    z: 0.0
    name: "Room A"
```

---

## ⚙️ Parameters

```yaml
waypoint_tolerance: 0.3  # meters
auto_start: false        # Start on launch
loop_mission: false      # Repeat mission
```

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
