#!/bin/bash
# Workspace Setup Script for GPS-Denied Indoor Navigation System
# This script sets up the ROS2 workspace and builds the optical flow integration package

set -e  # Exit on error

echo "=========================================="
echo "GPS-Denied Indoor Navigation System Setup"
echo "=========================================="
echo ""

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ROS2 not found. Please install ROS2 Humble first."
    echo "Visit: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo "✓ ROS2 found"

# Source ROS2 if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
fi

echo "✓ ROS2 Distribution: $ROS_DISTRO"
echo ""

# Check for required dependencies
echo "Checking dependencies..."

MISSING_DEPS=()

# Check for MAVROS
if ! ros2 pkg list | grep -q "mavros"; then
    MISSING_DEPS+=("ros-humble-mavros ros-humble-mavros-extras")
fi

# Check for Python numpy
if ! python3 -c "import numpy" 2>/dev/null; then
    MISSING_DEPS+=("python3-numpy")
fi

if [ ${#MISSING_DEPS[@]} -ne 0 ]; then
    echo "Missing dependencies detected:"
    for dep in "${MISSING_DEPS[@]}"; do
        echo "  - $dep"
    done
    echo ""
    read -p "Install missing dependencies? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt update
        sudo apt install -y "${MISSING_DEPS[@]}"
    else
        echo "WARNING: Build may fail without required dependencies"
    fi
fi

echo "✓ Dependencies checked"
echo ""

# Build the workspace
echo "Building workspace..."
echo "This may take a few minutes..."
echo ""

colcon build --packages-select optical_flow_integration imu_processing lidar_slam_2d sensor_synchronization graph_slam occupancy_grid_mapping dynamic_filter map_persistence local_planner global_planner position_hold trajectory_tracker waypoint_navigation mavsdk_offboard vision_pose_estimator failsafe_controller --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ Build successful!"
    echo "=========================================="
    echo ""
    echo "Next steps:"
    echo ""
    echo "1. Source the workspace:"
    echo "   source install/setup.bash"
    echo ""
    echo "2. Launch optical flow integration:"
    echo "   ros2 launch optical_flow_integration optical_flow.launch.py"
    echo ""
    echo "3. Launch IMU processing:"
    echo "   ros2 launch imu_processing imu_processing.launch.py"
    echo ""
    echo "4. Launch with simulation (optical flow):"
    echo "   ros2 launch optical_flow_integration optical_flow.launch.py simulation:=true"
    echo ""
    echo "5. Launch 2D Lidar SLAM:"
    echo "   ros2 launch lidar_slam_2d slam.launch.py"
    echo ""
    echo "6. Launch sensor synchronization:"
    echo "   ros2 launch sensor_synchronization sync.launch.py"
    echo ""
    echo "7. Launch graph-based SLAM:"
    echo "   ros2 launch graph_slam graph_slam.launch.py"
    echo ""
    echo "8. Launch occupancy grid mapping:"
    echo "   ros2 launch occupancy_grid_mapping mapping.launch.py"
    echo ""
    echo "9. Launch dynamic object filtering:"
    echo "   ros2 launch dynamic_filter filter.launch.py"
    echo ""
    echo "10. Launch map persistence:"
    echo "   ros2 launch map_persistence persistence.launch.py"
    echo ""
    echo "11. Launch local path planner:"
    echo "   ros2 launch local_planner planner.launch.py"
    echo ""
    echo "12. Launch global path planner:"
    echo "   ros2 launch global_planner planner.launch.py"
    echo ""
    echo "13. Launch position hold mode:"
    echo "   ros2 launch position_hold hold.launch.py"
    echo ""
    echo "14. Launch trajectory tracker:"
    echo "   ros2 launch trajectory_tracker tracker.launch.py"
    echo ""
    echo "15. Launch waypoint navigation:"
    echo "   ros2 launch waypoint_navigation waypoint.launch.py mission_file:=missions/example_mission.yaml"
    echo ""
    echo "16. Launch MAVSDK offboard control:"
    echo "   ros2 launch mavsdk_offboard offboard.launch.py"
    echo ""
    echo "17. Launch vision pose estimator:"
    echo "   ros2 launch vision_pose_estimator vision_pose.launch.py"
    echo ""
    echo "18. Launch failsafe controller:"
    echo "   ros2 launch failsafe_controller failsafe.launch.py"
    echo ""
    echo "19. Calibrate IMU:"
    echo "   ros2 launch imu_processing imu_processing.launch.py calibrate:=true"
    echo ""
    echo "For more information:"
    echo "  - Optical Flow: src/optical_flow_integration/README.md"
    echo "  - IMU Processing: src/imu_processing/README.md"
    echo "  - 2D Lidar SLAM: src/lidar_slam_2d/README.md"
    echo "  - Sensor Sync: src/sensor_synchronization/README.md"
    echo "  - Graph SLAM: src/graph_slam/README.md"
    echo "  - Occupancy Grid: src/occupancy_grid_mapping/README.md"
    echo "  - Dynamic Filter: src/dynamic_filter/README.md"
    echo "  - Map Persistence: src/map_persistence/README.md"
    echo "  - Local Planner: src/local_planner/README.md"
    echo "  - Global Planner: src/global_planner/README.md"
    echo "  - Position Hold: src/position_hold/README.md"
    echo "  - Trajectory Tracker: src/trajectory_tracker/README.md"
    echo "  - Waypoint Navigation: src/waypoint_navigation/README.md"
    echo "  - MAVSDK Offboard: src/mavsdk_offboard/README.md"
    echo "  - Vision Pose: src/vision_pose_estimator/README.md"
    echo "  - Failsafe Controller: src/failsafe_controller/README.md"
    echo ""
else
    echo ""
    echo "=========================================="
    echo "✗ Build failed"
    echo "=========================================="
    echo ""
    echo "Please check the error messages above and ensure all dependencies are installed."
    exit 1
fi
