#!/usr/bin/env python3
"""
Waypoint Manager Node
Executes pre-defined indoor waypoint missions

Feature 1.1.14: Waypoint Navigation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import math
from enum import Enum


class MissionState(Enum):
    IDLE = 0
    EXECUTING = 1
    COMPLETED = 2
    ABORTED = 3


class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')
        
        # Parameters
        self.declare_parameter('mission_file', '')
        self.declare_parameter('waypoint_tolerance', 0.3)  # meters
        self.declare_parameter('auto_start', False)
        self.declare_parameter('loop_mission', False)
        
        self.mission_file = self.get_parameter('mission_file').value
        self.waypoint_tol = self.get_parameter('waypoint_tolerance').value
        self.auto_start = self.get_parameter('auto_start').value
        self.loop_mission = self.get_parameter('loop_mission').value
        
        # State
        self.current_pose = None
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.mission_state = MissionState.IDLE
        
        # Subscribers
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            10
        )
        
        # Load mission
        if self.mission_file:
            self.load_mission(self.mission_file)
        
        # Mission loop
        self.create_timer(0.5, self.mission_loop)
        
        self.get_logger().info('Waypoint Manager Node initialized')
        if self.mission_file:
            self.get_logger().info(f'Loaded mission: {self.mission_file}')
            self.get_logger().info(f'Waypoints: {len(self.waypoints)}')
    
    def load_mission(self, filename):
        """Load waypoint mission from YAML file."""
        try:
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)
            
            if 'waypoints' not in data:
                self.get_logger().error('No waypoints found in mission file!')
                return
            
            self.waypoints = data['waypoints']
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
            
            # Visualize waypoints
            self.visualize_waypoints()
            
            # Auto-start if enabled
            if self.auto_start:
                self.start_mission()
        
        except Exception as e:
            self.get_logger().error(f'Failed to load mission: {e}')
    
    def odom_callback(self, msg):
        """Update current pose."""
        self.current_pose = msg.pose.pose
    
    def start_mission(self):
        """Start executing the mission."""
        if len(self.waypoints) == 0:
            self.get_logger().warn('No waypoints to execute!')
            return
        
        self.current_waypoint_idx = 0
        self.mission_state = MissionState.EXECUTING
        self.get_logger().info('Mission started!')
        
        # Send first waypoint
        self.send_current_waypoint()
    
    def mission_loop(self):
        """Main mission execution loop."""
        if self.mission_state != MissionState.EXECUTING:
            return
        
        if self.current_pose is None:
            return
        
        if self.current_waypoint_idx >= len(self.waypoints):
            self.mission_completed()
            return
        
        # Check if current waypoint reached
        if self.is_waypoint_reached():
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} reached!'
            )
            
            # Move to next waypoint
            self.current_waypoint_idx += 1
            
            if self.current_waypoint_idx < len(self.waypoints):
                self.send_current_waypoint()
            else:
                self.mission_completed()
    
    def send_current_waypoint(self):
        """Send current waypoint as goal."""
        if self.current_waypoint_idx >= len(self.waypoints):
            return
        
        wp = self.waypoints[self.current_waypoint_idx]
        
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        
        goal.pose.position.x = float(wp['x'])
        goal.pose.position.y = float(wp['y'])
        goal.pose.position.z = float(wp.get('z', 0.0))
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        
        self.get_logger().info(
            f'Sending waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}: '
            f'({wp["x"]:.2f}, {wp["y"]:.2f}, {wp.get("z", 0.0):.2f})'
        )
    
    def is_waypoint_reached(self):
        """Check if current waypoint is reached."""
        if self.current_waypoint_idx >= len(self.waypoints):
            return False
        
        wp = self.waypoints[self.current_waypoint_idx]
        
        dist = math.sqrt(
            (self.current_pose.position.x - wp['x'])**2 +
            (self.current_pose.position.y - wp['y'])**2 +
            (self.current_pose.position.z - wp.get('z', 0.0))**2
        )
        
        return dist < self.waypoint_tol
    
    def mission_completed(self):
        """Handle mission completion."""
        self.get_logger().info('Mission completed!')
        
        if self.loop_mission:
            self.get_logger().info('Looping mission...')
            self.start_mission()
        else:
            self.mission_state = MissionState.COMPLETED
    
    def visualize_waypoints(self):
        """Visualize all waypoints as markers."""
        marker_array = MarkerArray()
        
        for i, wp in enumerate(self.waypoints):
            # Waypoint sphere
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(wp['x'])
            marker.pose.position.y = float(wp['y'])
            marker.pose.position.z = float(wp.get('z', 0.0))
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            
            # Waypoint text
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose = marker.pose
            text_marker.pose.position.z += 0.5
            
            text_marker.scale.z = 0.3
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f'WP{i+1}'
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
