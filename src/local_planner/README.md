# Local Path Planner

**Epic 3: Autonomous Navigation Stack**  
**Priority: P0**

Dynamic Window Approach (DWA) for real-time obstacle avoidance.

---

## 📋 Overview

DWA local planner with:
- Dynamic velocity sampling
- Trajectory prediction
- Multi-criteria scoring
- Real-time collision avoidance

---

## 🚀 Quick Start

```bash
# Build
colcon build --packages-select local_planner

# Launch
ros2 launch local_planner planner.launch.py

# Send goal
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{...}"
```

---

## 📊 Topics

### Input
- `/odom` - Robot odometry
- `/scan` - Lidar scan
- `/goal_pose` - Navigation goal

### Output
- `/cmd_vel` - Velocity commands
- `/local_plan` - Planned trajectory

---

## ⚙️ Parameters

```yaml
max_speed: 0.5        # m/s
max_yaw_rate: 1.0     # rad/s
predict_time: 2.0     # seconds
robot_radius: 0.3     # meters
```

---

## 🎯 Algorithm

**DWA Steps**:
1. Calculate dynamic window
2. Sample velocity pairs (v, ω)
3. Predict trajectories
4. Check collisions
5. Score trajectories
6. Select best velocity

**Scoring**:
- Heading: Goal alignment
- Clearance: Obstacle distance
- Velocity: Forward speed

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20
