# Failsafe Controller

**Epic 4: Flight Controller Integration**  
**Feature**: Fail-Safe Logic  
**Priority**: P0

Automatic landing on SLAM failure for safety.

---

## 📋 Overview

Intelligent failsafe system:
- SLAM health monitoring
- Automatic emergency landing
- Position jump detection
- Multi-stage warning system
- Graceful recovery

---

## 🚀 Quick Start

```bash
# Build
colcon build --packages-select failsafe_controller

# Launch
ros2 launch failsafe_controller failsafe.launch.py

# Monitor status
ros2 topic echo /failsafe/active
```

---

## 📊 Topics

### Input
- `/odom` - SLAM odometry
- `/mavros/state` - Flight controller state

### Output
- `/failsafe/active` - Failsafe status (Bool)
- `/emergency/land_target` - Landing target

---

## 🔄 State Machine

```
NORMAL → WARNING → EMERGENCY → LANDING → LANDED
  ↑         ↓
  └─────────┘ (Recovery)
```

**States**:
- NORMAL: All systems OK
- WARNING: SLAM degraded (3s grace period)
- EMERGENCY: SLAM failed
- LANDING: Executing landing
- LANDED: On ground, disarmed

---

## ⚙️ Safety Checks

1. **Timeout**: No odometry >2s
2. **Update Rate**: <5 Hz
3. **Position Jump**: >5m sudden change
4. **Warning Period**: 3s grace time

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
