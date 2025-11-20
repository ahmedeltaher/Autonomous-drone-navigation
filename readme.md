# GPS-Denied Navigation System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![PX4](https://img.shields.io/badge/PX4-v1.14-green.svg)](https://px4.io/)
[![Python](https://img.shields.io/badge/Python-3.8+-yellow.svg)](https://www.python.org/)
[![Difficulty](https://img.shields.io/badge/Difficulty-High-red.svg)]()

**Autonomous drone navigation using optical flow, IMU, and lidar when GPS is unavailable.**

---

## 📋 Overview

This project enables drones to navigate autonomously in GPS-denied environments with centimeter-level accuracy using multi-sensor fusion and SLAM algorithms. The system is designed for warehouse inspection, search & rescue, and mapping missions.

---

## 🎯 Features Checklist

- [x] Multi-sensor data acquisition (Optical Flow, IMU, Lidar)
- [x] Real-time SLAM with occupancy grid mapping
- [x] Local & global path planning with obstacle avoidance
- [x] PX4 offboard integration via MAVSDK
- [x] Vision-based position hold and waypoint navigation
- [x] Multi-session mapping (Map Persistence)
- [x] Dynamic object filtering
- [x] Vision pose estimation for EKF2
- [x] Fail-safe automatic landing
- [x] Trajectory tracking with Pure Pursuit

---

## 🏗️ Feature Breakdown

### Epic 1: Multi-Sensor Data Acquisition & Fusion
| Feature ID | Feature Name | Description | Priority |
|------------|--------------|-------------|----------|
| 1.1.1 | Optical Flow Integration | Real-time velocity estimation from downward-facing camera | P0 |
| 1.1.2 | IMU Data Processing | High-frequency pose estimation and vibration filtering | P0 |
| 1.1.3 | 2D Lidar SLAM | Horizontal plane mapping and localization | P0 |
| 1.1.4 | Sensor Synchronization | Microsecond-level timestamp alignment across sensors | P1 |

### Epic 2: Real-Time SLAM Engine
| Feature ID | Feature Name | Description | Priority |
|------------|--------------|-------------|----------|
| 1.1.6 | Graph-Based SLAM | Loop closure and pose graph optimization | P0 |
| 1.1.7 | Occupancy Grid Mapping | 2D/2.5D environment representation | P0 |
| 1.1.8 | Dynamic Object Filtering | Remove moving obstacles from static map | P1 |
| 1.1.9 | Map Persistence | Save/load maps for repeated missions | P1 |

### Epic 3: Autonomous Navigation Stack
| Feature ID | Feature Name | Description | Priority |
|------------|--------------|-------------|----------|
| 1.1.11 | Local Path Planner | Dynamic window approach for real-time avoidance | P0 |
| 1.1.12 | Global Path Planner | A* algorithm on occupancy grid | P0 |
| 1.1.13 | Position Hold Mode | Stable hover without GPS drift | P0 |
| 1.1.14 | Waypoint Navigation | Execute pre-defined waypoints | P1 |

### Epic 4: Flight Controller Integration
| Feature ID | Feature Name | Description | Priority |
|------------|--------------|-------------|----------|
| 1.1.16 | MAVSDK Offboard Mode | Send position setpoints via MAVLink | P0 |
| 1.1.17 | Vision Pose Estimation | Inject SLAM pose into EKF2 | P0 |
| 1.1.18 | Fail-Safe Logic | Automatic landing on SLAM failure | P0 |

---
## 🔧 Technical Architecture

![GPS-Denied Navigation System](GPS-Denied%20Indoor%20Navigation%20System.drawio.png)

---

## 🔗 System Integration

### Complete Data Flow Pipeline

```
1. SENSOR ACQUISITION (Epic 1)
   ├─ optical_flow_integration → Visual velocity
   ├─ imu_processing → Orientation & acceleration
   └─ lidar_slam_2d → Laser scans
          ↓
2. SENSOR FUSION (Epic 1)
   └─ sensor_synchronization → Time-aligned data
          ↓
3. LOCALIZATION & MAPPING (Epic 2)
   ├─ graph_slam → Pose estimation with loop closure
   ├─ occupancy_grid_mapping → Environment representation
   ├─ dynamic_filter → Remove moving objects
   └─ map_persistence → Save/load maps
          ↓
4. PATH PLANNING (Epic 3)
   ├─ global_planner (A*) → Optimal waypoint paths
   └─ local_planner (DWA) → Real-time obstacle avoidance
          ↓
5. TRAJECTORY EXECUTION (Epic 3)
   ├─ trajectory_tracker (Pure Pursuit) → Smooth path following
   ├─ position_hold (PID) → Stable hovering
   └─ waypoint_navigation → Mission execution
          ↓
6. FLIGHT CONTROL (Epic 4)
   ├─ vision_pose_estimator → SLAM → EKF2 fusion
   ├─ mavsdk_offboard → Position setpoints via MAVLink
   └─ failsafe_controller → Emergency landing on failure
          ↓
7. VEHICLE EXECUTION
   └─ PX4 Flight Controller → Motor commands
```

### Feature Integration Matrix

| Feature | Inputs | Outputs | Integrates With |
|---------|--------|---------|-----------------|
| **optical_flow_integration** | Camera frames | `/optical_flow/velocity` | sensor_synchronization |
| **imu_processing** | IMU raw data | `/imu/data` | sensor_synchronization, graph_slam |
| **lidar_slam_2d** | Laser scans | `/scan` | sensor_synchronization, graph_slam |
| **sensor_synchronization** | All sensor topics | `/fused/odom` | graph_slam |
| **graph_slam** | Fused odometry | `/slam/pose`, `/odom` | occupancy_grid_mapping, vision_pose_estimator |
| **occupancy_grid_mapping** | SLAM pose + scans | `/map` | global_planner, local_planner |
| **dynamic_filter** | `/map` + velocities | `/static_map` | map_persistence |
| **map_persistence** | Filtered map | Saved maps | occupancy_grid_mapping |
| **global_planner** | `/static_map`, goal | `/global_plan` | trajectory_tracker |
| **local_planner** | `/map`, `/odom` | `/local_plan` | trajectory_tracker |
| **trajectory_tracker** | Plans + odometry | `/cmd_vel` | mavsdk_offboard |
| **position_hold** | Target pose | `/cmd_vel_hold` | mavsdk_offboard |
| **waypoint_navigation** | Mission file | `/goal_pose` | global_planner |
| **vision_pose_estimator** | `/odom` | `/mavros/vision_pose/pose` | PX4 EKF2 |
| **mavsdk_offboard** | `/cmd_vel` topics | `/mavros/setpoint_raw/local` | PX4 |
| **failsafe_controller** | `/odom`, `/mavros/state` | `/failsafe/active`, LAND mode | PX4 |

### Critical Dependencies

**For Localization**:
- optical_flow_integration + imu_processing + lidar_slam_2d → sensor_synchronization → graph_slam

**For Navigation**:
- graph_slam → occupancy_grid_mapping → global_planner + local_planner → trajectory_tracker

**For Flight Control**:
- trajectory_tracker OR position_hold → mavsdk_offboard → PX4
- graph_slam → vision_pose_estimator → PX4 EKF2

**For Safety**:
- graph_slam → failsafe_controller → PX4 (emergency landing)

---

## ⚠️ Risk Mitigation

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| SLAM drift in featureless areas | High | Critical | Add AprilTag landmarks, fuse multiple sensor modalities |
| Computational overload | Medium | High | Offload to Coral TPU, optimize with C++ nodes |
| Magnetic interference | High | Medium | Use optical flow as primary yaw source, disable mag fusion |
| Lidar motion distortion | Medium | Medium | Implement motion compensation using IMU pre-integration |

---

## ✅ Definition of Done

- ✓ Drone hovers stably for 2 minutes in 5m×5m room (<20cm drift)
- ✓ Successfully navigates 4-waypoint mission with obstacle avoidance
- ✓ SLAM relocalizes after being picked up and moved (kidnapped robot problem)
- ✓ All safety tests pass in simulation + real-world validation
- ✓ Documentation includes calibration procedures and tuning guide

---

## 🚀 Quick Start

### Prerequisites
- PX4-compatible flight controller (Pixhawk 4, Cube Orange)
- Raspberry Pi 4 (4GB+)
- RPLidar A1/A2, Optical Flow sensor (PMW3901)
- ROS2 Humble, MAVSDK-Python

### Installation
```bash
# Navigate to project directory
cd "Autonomous Navigation & Control"

# Build all packages (16 ROS2 packages)
./setup_workspace.sh

# Source workspace
source install/setup.bash
```

### First Flight - Complete Launch Sequence

```bash
# 1. Launch sensor fusion (Terminal 1)
ros2 launch sensor_synchronization sync.launch.py

# 2. Launch SLAM system (Terminal 2)
ros2 launch graph_slam graph_slam.launch.py

# 3. Launch occupancy grid mapping (Terminal 3)
ros2 launch occupancy_grid_mapping mapping.launch.py

# 4. Launch MAVROS - connect to PX4 (Terminal 4)
ros2 launch mavros px4.launch fcu_url:=udp://:14540@

# 5. Launch vision pose estimator (Terminal 5)
ros2 launch vision_pose_estimator vision_pose.launch.py

# 6. Launch failsafe controller (Terminal 6)
ros2 launch failsafe_controller failsafe.launch.py

# 7. Launch navigation stack (Terminal 7)
ros2 launch global_planner planner.launch.py &
ros2 launch local_planner planner.launch.py &
ros2 launch trajectory_tracker tracker.launch.py

# 8. Launch offboard control (Terminal 8)
ros2 launch mavsdk_offboard offboard.launch.py

# 9. Execute waypoint mission (Terminal 9)
ros2 launch waypoint_navigation waypoint.launch.py \
  mission_file:=missions/example_mission.yaml \
  auto_start:=true
```

### Quick Launch - Position Hold Test

```bash
# Minimal setup for testing position hold
./setup_workspace.sh
source install/setup.bash

# Launch SLAM + Position Hold
ros2 launch graph_slam graph_slam.launch.py &
ros2 launch mavros px4.launch fcu_url:=udp://:14540@ &
ros2 launch vision_pose_estimator vision_pose.launch.py &
ros2 launch position_hold hold.launch.py &
ros2 launch failsafe_controller failsafe.launch.py
```

---

## 📦 Implemented Packages (16 Total)

### Epic 1: Multi-Sensor Data Acquisition & Fusion ✅

1. **optical_flow_integration** - Visual odometry from camera
   - 20 Hz velocity estimation
   - Covariance support
   - Simulation mode

2. **imu_processing** - High-rate orientation data
   - 100 Hz IMU processing
   - Complementary filter
   - Calibration support

3. **lidar_slam_2d** - 2D laser SLAM
   - Real-time scan matching
   - TF broadcasting
   - Loop closure detection

4. **sensor_synchronization** - Multi-sensor fusion
   - 50ms time window
   - Kalman filter fusion
   - Synchronized odometry

### Epic 2: Real-Time SLAM Engine ✅

5. **graph_slam** - Graph optimization SLAM
   - Pose graph optimization
   - Loop closure detection
   - G2O integration

6. **occupancy_grid_mapping** - Probabilistic mapping
   - Log-odds occupancy
   - Dynamic updates
   - Configurable resolution

7. **dynamic_filter** - Motion filtering
   - Velocity-based detection
   - Static map generation
   - Threshold tuning

8. **map_persistence** - Map save/load
   - YAML/PGM format
   - Multi-session support
   - Automatic saving

### Epic 3: Autonomous Navigation Stack ✅

9. **local_planner** - Dynamic Window Approach
   - Real-time obstacle avoidance
   - Velocity space sampling
   - Collision checking

10. **global_planner** - A* path planning
    - Optimal path finding
    - Occupancy grid search
    - Waypoint generation

11. **position_hold** - PID hovering
    - 3D position control (XY + Z)
    - 50 Hz control rate
    - Drift compensation

12. **trajectory_tracker** - Pure Pursuit
    - Smooth path following
    - Adaptive lookahead
    - Curvature-based steering

13. **waypoint_navigation** - Mission execution
    - YAML mission files
    - Sequential execution
    - Visual markers

### Epic 4: Flight Controller Integration ✅

14. **mavsdk_offboard** - MAVLink control
    - Position setpoint streaming (20 Hz)
    - Offboard mode management
    - Auto-arm capability

15. **vision_pose_estimator** - EKF2 integration
    - SLAM pose injection
    - 30 Hz publishing
    - Covariance support

16. **failsafe_controller** - Safety monitor
    - SLAM health checking
    - Automatic emergency landing
    - Multi-stage warnings

---

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 📚 References & Resources

- [PX4 Vision Pose Estimation](https://docs.px4.io/main/en/computer_vision/)
- [ROS2 SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [MAVSDK-Python](https://mavsdk.mavlink.io/main/en/python/)
- [Hector SLAM](http://wiki.ros.org/hector_slam)

---

**⚠️ Safety Notice:** Always test in simulation first. Use a safety pilot with manual override switch. Indoor flight approval required.
