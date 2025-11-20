# Position Hold Mode

**Epic 3: Autonomous Navigation Stack**  
**Priority: P0**

Stable hovering without GPS using sensor fusion and PID control.

---

## 📋 Overview

Position hold controller with:
- 3D PID control (XY + Z)
- Sensor fusion (IMU + Optical Flow + SLAM)
- Drift compensation
- 50 Hz control rate

---

## 🚀 Quick Start

```bash
# Build
colcon build --packages-select position_hold

# Launch
ros2 launch position_hold hold.launch.py

# Set hold position
ros2 topic pub /hold_setpoint geometry_msgs/msg/PoseStamped "{...}"
```

---

## 📊 Topics

### Input
- `/odom` - Odometry (position + velocity)
- `/imu/data` - IMU for orientation
- `/hold_setpoint` - Target position

### Output
- `/cmd_vel_hold` - Velocity commands

---

## ⚙️ Parameters

```yaml
kp_xy: 1.0           # XY proportional
ki_xy: 0.1           # XY integral
kd_xy: 0.5           # XY derivative
max_velocity_xy: 2.0 # m/s
control_rate: 50.0   # Hz
```

---

## 🎯 PID Control

**Position Error**:
```
e = target - current
```

**Control Output**:
```
u = Kp*e + Ki*∫e + Kd*de/dt
```

**Prevents GPS drift!**

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
