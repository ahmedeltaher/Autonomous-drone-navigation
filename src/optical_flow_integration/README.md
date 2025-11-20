# Optical Flow Integration

**Feature ID: 1.1.1 - Optical Flow Integration**  
**Priority: P0**

Real-time velocity estimation from downward-facing camera for GPS-denied indoor navigation.

---

## 📋 Overview

This package implements optical flow-based velocity estimation for autonomous drone navigation in GPS-denied environments. It processes data from optical flow sensors (PX4FLOW or PMW3901) and range sensors to provide accurate velocity estimates with covariance information.

### Key Features

- ✅ Real-time velocity estimation at 20 Hz
- ✅ Support for PMW3901 and PX4FLOW sensors
- ✅ Gyroscope compensation for rotational motion
- ✅ Height-based velocity scaling
- ✅ Covariance estimation for sensor fusion
- ✅ Simulation mode for testing without hardware
- ✅ Dead reckoning position estimation

---

## 🎯 Technical Specifications

### Sensor Support

| Sensor | Focal Length (px) | Sensor Size (m) | Notes |
|--------|------------------|-----------------|-------|
| PMW3901 | 16.0 | 0.0001 | Recommended for indoor use |
| PX4FLOW | Varies | 0.000006 | Configurable, requires calibration |

### Performance Metrics

- **Update Rate**: 20 Hz
- **Velocity Accuracy**: ±0.1 m/s (at 1.5m height)
- **Height Range**: 0.1 - 5.0 meters
- **Latency**: < 50 ms

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    MAVROS / PX4                         │
│  (Optical Flow Sensor + Range Sensor + IMU)             │
└────────────┬────────────────────────────┬───────────────┘
             │                            │
             │ OpticalFlowRad            │ Range
             │                            │
┌────────────▼────────────────────────────▼───────────────┐
│              Optical Flow Node                          │
│  • Flow rate calculation                                │
│  • Gyro compensation                                    │
│  • Height-based velocity scaling                        │
│  • Covariance estimation                                │
└────────────┬────────────────────────────────────────────┘
             │
             │ TwistWithCovarianceStamped
             │ Odometry
             │
┌────────────▼────────────────────────────────────────────┐
│              Sensor Fusion / SLAM                       │
└─────────────────────────────────────────────────────────┘
```

---

## 🚀 Quick Start

### Prerequisites

```bash
# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Install dependencies
sudo apt install python3-numpy
```

### Build the Package

```bash
# Navigate to workspace root
cd ~/Autonomous\ Navigation\ \&\ Control

# Build the package
colcon build --packages-select optical_flow_integration

# Source the workspace
source install/setup.bash
```

### Hardware Setup

1. **Connect PMW3901 Optical Flow Sensor**
   - Connect to Pixhawk's I2C or UART port
   - Configure PX4 parameters:
     ```
     SENS_FLOW_ROT = 0  (No rotation)
     SENS_EN_PMW3901 = 1
     ```

2. **Connect Range Sensor (Lidar-Lite or similar)**
   - Connect to Pixhawk's I2C port
   - Configure PX4 parameters:
     ```
     SENS_EN_LL40LS = 1
     ```

3. **Mount Sensors on Drone**
   - Optical flow sensor: Downward-facing, centered under drone
   - Range sensor: Downward-facing
   - Ensure clear line of sight to ground

### Launch

#### With Real Hardware

```bash
# Launch optical flow node
ros2 launch optical_flow_integration optical_flow.launch.py

# View velocity output
ros2 topic echo /optical_flow/velocity
```

#### With Simulation

```bash
# Launch with simulator
ros2 launch optical_flow_integration optical_flow.launch.py simulation:=true

# In another terminal, send test velocities
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "header:
    stamp: {sec: 0, nanosec: 0}
    frame_id: 'base_link'
  twist:
    linear: {x: 0.5, y: 0.2, z: 0.0}
    angular: {x: 0.0, y: 0.0, z: 0.0}"
```

---

## 📊 Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mavros/optical_flow/raw/optical_flow_rad` | `mavros_msgs/OpticalFlowRad` | Raw optical flow data from sensor |
| `/mavros/distance_sensor/hrlv_ez4_pub` | `sensor_msgs/Range` | Range sensor (height) data |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/optical_flow/velocity` | `geometry_msgs/TwistWithCovarianceStamped` | Estimated velocity with covariance |
| `/optical_flow/odometry` | `nav_msgs/Odometry` | Full odometry estimate (position + velocity) |

---

## ⚙️ Parameters

### Optical Flow Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `update_rate` | double | 20.0 | Processing rate (Hz) |
| `focal_length_px` | double | 16.0 | Sensor focal length (pixels) |
| `sensor_size_m` | double | 0.0001 | Physical sensor size (meters) |
| `min_height` | double | 0.1 | Minimum valid height (meters) |
| `max_height` | double | 5.0 | Maximum valid height (meters) |
| `velocity_covariance` | double | 0.1 | Velocity standard deviation (m/s) |
| `use_gyro_compensation` | bool | true | Enable gyro compensation |
| `publish_odometry` | bool | true | Publish full odometry |

### Configuration File

Edit `config/optical_flow_params.yaml` to customize parameters:

```yaml
optical_flow_node:
  ros__parameters:
    update_rate: 20.0
    focal_length_px: 16.0
    min_height: 0.1
    max_height: 5.0
```

---

## 🧪 Testing

### Unit Tests

```bash
# Run tests
colcon test --packages-select optical_flow_integration

# View results
colcon test-result --all
```

### Integration Test

1. Launch simulator:
   ```bash
   ros2 launch optical_flow_integration optical_flow.launch.py simulation:=true
   ```

2. Verify topics:
   ```bash
   ros2 topic list | grep optical_flow
   ```

3. Check velocity output:
   ```bash
   ros2 topic echo /optical_flow/velocity
   ```

### Calibration

To calibrate the focal length for your specific sensor:

1. Fly at known height (e.g., 1.5m)
2. Move at known velocity (e.g., 0.5 m/s)
3. Record optical flow output
4. Calculate: `focal_length = (flow_rate * height) / velocity`
5. Update `focal_length_px` parameter

---

## 🔧 Troubleshooting

### No Velocity Output

**Symptom**: Node running but no velocity published

**Solutions**:
- Check MAVROS connection: `ros2 topic list | grep mavros`
- Verify optical flow data: `ros2 topic echo /mavros/optical_flow/raw/optical_flow_rad`
- Check height is in valid range (0.1 - 5.0 m)

### Incorrect Velocity Estimates

**Symptom**: Velocity estimates don't match actual movement

**Solutions**:
- Recalibrate focal length (see Calibration section)
- Verify sensor mounting (must be perpendicular to ground)
- Check for vibration damping
- Enable gyro compensation if rotating

### Quality Issues

**Symptom**: Low quality readings or erratic behavior

**Solutions**:
- Ensure textured surface below drone (not reflective/featureless)
- Maintain height between 0.5 - 3.0 m for best results
- Check sensor lens is clean
- Verify adequate lighting

---

## 📚 Theory of Operation

### Optical Flow Equation

The velocity estimation is based on the optical flow equation:

```
v = (ω × h) / f
```

Where:
- `v` = velocity (m/s)
- `ω` = angular velocity from optical flow (rad/s)
- `h` = height above ground (m)
- `f` = focal length in physical units (m)

### Covariance Model

Velocity uncertainty increases with:
1. Height (farther from ground = less accuracy)
2. Low texture surfaces
3. Vibrations
4. Sensor quality degradation

---

## 🎓 Related Documentation

- [PX4 Optical Flow Guide](https://docs.px4.io/main/en/sensor/optical_flow.html)
- [PMW3901 Datasheet](https://www.pixart.com/products-detail/10/PMW3901MB-TXQT)
- Sprint 1 User Story: Optical Flow Data Integration

---

## 📈 Future Enhancements

- [ ] Multi-rate Kalman filter for smoother estimates
- [ ] Adaptive covariance based on surface texture
- [ ] Support for 3D optical flow
- [ ] Integration with VIO (Visual Inertial Odometry)
- [ ] Quality-based sensor fusion weighting

---

## 🤝 Contributing

To extend this feature:

1. Add new sensor support in `optical_flow_node.py`
2. Update sensor specifications table in this README
3. Add configuration examples in `config/`
4. Update unit tests
5. Submit pull request with test results

---

## 📄 License

MIT License - See main project LICENSE file

---

**Status**: ✅ Implemented  
**Last Updated**: 2025-11-20  
**Maintainer**: Indoor Navigation Team
