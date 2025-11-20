#!/usr/bin/env python3
"""
A* Global Path Planner Node
Plans optimal paths on occupancy grid maps

Epic 3: Autonomous Navigation Stack
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import heapq
import math


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner_node')
        
        # Parameters
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('cost_diagonal', 1.414)  # sqrt(2)
        self.declare_parameter('cost_straight', 1.0)
        self.declare_parameter('occupied_threshold', 50)  # 0-100
        self.declare_parameter('path_smoothing', True)
        
        self.allow_diagonal = self.get_parameter('allow_diagonal').value
        self.cost_diagonal = self.get_parameter('cost_diagonal').value
        self.cost_straight = self.get_parameter('cost_straight').value
        self.occupied_thresh = self.get_parameter('occupied_threshold').value
        self.smooth_path = self.get_parameter('path_smoothing').value
        
        # State
        self.map = None
        self.start_pose = None
        self.goal_pose = None
        
        # Subscribers
        self.create_subscription(
            OccupancyGrid,
            '/static_map',
            self.map_callback,
            10
        )
        
        self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.start_callback,
            10
        )
        
        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/astar_explored', 10)
        
        self.get_logger().info('A* Planner Node initialized')
        self.get_logger().info(f'Diagonal movement: {self.allow_diagonal}')
    
    def map_callback(self, msg):
        """Store occupancy grid map."""
        self.map = msg
    
    def start_callback(self, msg):
        """Update robot start pose."""
        self.start_pose = msg.pose
    
    def goal_callback(self, msg):
        """Receive goal and plan path."""
        self.goal_pose = msg.pose
        self.get_logger().info(
            f'New goal: ({self.goal_pose.position.x:.2f}, '
            f'{self.goal_pose.position.y:.2f})'
        )
        
        if self.map is None:
            self.get_logger().error('No map available!')
            return
        
        if self.start_pose is None:
            self.get_logger().error('No start pose available!')
            return
        
        # Plan path
        path = self.plan_path()
        
        if path is not None:
            self.get_logger().info(f'Path found with {len(path.poses)} waypoints')
            self.path_pub.publish(path)
        else:
            self.get_logger().error('No path found!')
    
    def plan_path(self):
        """Plan path using A* algorithm."""
        # Convert poses to grid coordinates
        start_grid = self.world_to_grid(
            self.start_pose.position.x,
            self.start_pose.position.y
        )
        goal_grid = self.world_to_grid(
            self.goal_pose.position.x,
            self.goal_pose.position.y
        )
        
        if not self.is_valid(start_grid[0], start_grid[1]):
            self.get_logger().error('Start position invalid!')
            return None
        
        if not self.is_valid(goal_grid[0], goal_grid[1]):
            self.get_logger().error('Goal position invalid!')
            return None
        
        # Run A*
        path_grid = self.astar(start_grid, goal_grid)
        
        if path_grid is None:
            return None
        
        # Smooth path if enabled
        if self.smooth_path:
            path_grid = self.smooth_path_fn(path_grid)
        
        # Convert to Path message
        path = self.grid_path_to_msg(path_grid)
        return path
    
    def astar(self, start, goal):
        """A* pathfinding algorithm."""
        # Priority queue: (f_score, counter, node)
        counter = 0
        open_set = [(0, counter, start)]
        counter += 1
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        open_set_hash = {start}
        
        while open_set:
            current = heapq.heappop(open_set)[2]
            open_set_hash.remove(current)
            
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + self.cost(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    
                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
                        counter += 1
                        open_set_hash.add(neighbor)
        
        return None  # No path found
    
    def get_neighbors(self, node):
        """Get valid neighbors of a node."""
        x, y = node
        neighbors = []
        
        # 4-connected
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
        # 8-connected (add diagonals)
        if self.allow_diagonal:
            directions += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if self.is_valid(nx, ny):
                neighbors.append((nx, ny))
        
        return neighbors
    
    def is_valid(self, x, y):
        """Check if grid cell is valid and not occupied."""
        if x < 0 or x >= self.map.info.width:
            return False
        if y < 0 or y >= self.map.info.height:
            return False
        
        idx = y * self.map.info.width + x
        cell_value = self.map.data[idx]
        
        # -1 = unknown, 0 = free, 100 = occupied
        if cell_value == -1 or cell_value >= self.occupied_thresh:
            return False
        
        return True
    
    def cost(self, node1, node2):
        """Cost between two adjacent nodes."""
        dx = abs(node1[0] - node2[0])
        dy = abs(node1[1] - node2[1])
        
        if dx + dy == 2:  # Diagonal
            return self.cost_diagonal
        else:  # Straight
            return self.cost_straight
    
    def heuristic(self, node, goal):
        """Heuristic function (Euclidean distance)."""
        return math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from start to goal."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def smooth_path_fn(self, path):
        """Smooth path by removing unnecessary waypoints."""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # Look ahead as far as possible
            j = len(path) - 1
            while j > i + 1:
                if self.has_line_of_sight(path[i], path[j]):
                    break
                j -= 1
            
            smoothed.append(path[j])
            i = j
        
        return smoothed
    
    def has_line_of_sight(self, node1, node2):
        """Check if there's a clear line of sight between two nodes."""
        x0, y0 = node1
        x1, y1 = node2
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if not self.is_valid(x, y):
                return False
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return True
    
    def world_to_grid(self, wx, wy):
        """Convert world coordinates to grid coordinates."""
        gx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        gy = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        return (gx, gy)
    
    def grid_to_world(self, gx, gy):
        """Convert grid coordinates to world coordinates."""
        wx = gx * self.map.info.resolution + self.map.info.origin.position.x
        wy = gy * self.map.info.resolution + self.map.info.origin.position.y
        return (wx, wy)
    
    def grid_path_to_msg(self, path_grid):
        """Convert grid path to Path message."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        
        for node in path_grid:
            wx, wy = self.grid_to_world(node[0], node[1])
            
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path.poses.append(pose)
        
        return path


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
