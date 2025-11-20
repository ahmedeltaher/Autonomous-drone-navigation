# MAVSDK Offboard Control

**Epic 4: Flight Controller Integration**  
**Feature**: 1.1.16 - MAVSDK Offboard Mode  
**Priority**: P0

Send position setpoints via MAVLink for flight control.

---

## 📋 Overview

Offboard control with:
- MAVLink position setpoints
- 20 Hz streaming (PX4 requirement)
- Offboard mode management
- Auto-arm capability
- Safety interlocks

---

## 🚀 Quick Start

```bash
# Build
colcon build --packages-select mavsdk_offboard

# Launch MAVROS first
ros2 launch mavros px4.launch fcu_url:=udp://:14540@

# Launch offboard control
ros2 launch mavsdk_offboard offboard.launch.py

# Send target position
ros2 topic pub /offboard/target_pose geometry_msgs/msg/PoseStamped "{...}"
```

---

## 📊 Topics

### Input
- `/mavros/state` - Flight controller state
- `/odom` - Robot odometry
- `/offboard/target_pose` - Target position

### Output
- `/mavros/setpoint_raw/local` - Position setpoints

---

## ⚙️ Parameters

```yaml
setpoint_rate: 20.0    # Hz (>2Hz required)
auto_arm: false        # Auto-arm vehicle
auto_offboard: false   # Auto-enable offboard
```

---

## 🛡️ Safety

**Requirements**:
- Setpoint rate >2Hz
- MAVROS connection
- Offboard mode enabled
- Vehicle armed

**Interlocks**:
- Connection check
- Mode verification
- Arming confirmation

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
