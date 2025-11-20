#!/usr/bin/env python3
"""
Map Manager Node
Save and load occupancy grid maps for repeated missions

Feature ID: 1.1.9 - Map Persistence
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger, Empty
import numpy as np
import yaml
import os
from datetime import datetime


class MapManagerNode(Node):
    def __init__(self):
        super().__init__('map_manager_node')
        
        # Parameters
        self.declare_parameter('map_directory', 'maps')
        self.declare_parameter('auto_save', True)
        self.declare_parameter('save_interval', 60.0)  # seconds
        
        self.map_dir = self.get_parameter('map_directory').value
        self.auto_save = self.get_parameter('auto_save').value
        self.save_interval = self.get_parameter('save_interval').value
        
        # Create maps directory
        os.makedirs(self.map_dir, exist_ok=True)
        
        # Current map
        self.current_map = None
        self.map_name = f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        # Subscribers
        self.create_subscription(
            OccupancyGrid,
            '/static_map',
            self.map_callback,
            10
        )
        
        # Publishers
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/loaded_map',
            10
        )
        
        # Services
        self.save_srv = self.create_service(
            Trigger,
            '/save_map',
            self.save_map_service
        )
        
        self.load_srv = self.create_service(
            Trigger,
            '/load_map',
            self.load_map_service
        )
        
        # Auto-save timer
        if self.auto_save:
            self.create_timer(self.save_interval, self.auto_save_map)
        
        self.get_logger().info('Map Manager Node initialized')
        self.get_logger().info(f'Map directory: {self.map_dir}')
        self.get_logger().info(f'Auto-save: {self.auto_save} (every {self.save_interval}s)')
    
    def map_callback(self, msg):
        """Store incoming map."""
        self.current_map = msg
    
    def save_map_service(self, request, response):
        """Service to save current map."""
        if self.current_map is None:
            response.success = False
            response.message = 'No map available to save'
            return response
        
        try:
            self.save_map(self.map_name)
            response.success = True
            response.message = f'Map saved: {self.map_name}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed to save map: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def load_map_service(self, request, response):
        """Service to load map."""
        try:
            # Find latest map
            map_files = [f for f in os.listdir(self.map_dir) if f.endswith('.npy')]
            if not map_files:
                response.success = False
                response.message = 'No maps found'
                return response
            
            latest_map = sorted(map_files)[-1]
            map_name = latest_map[:-4]  # Remove .npy
            
            loaded_map = self.load_map(map_name)
            self.map_pub.publish(loaded_map)
            
            response.success = True
            response.message = f'Map loaded: {map_name}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed to load map: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def auto_save_map(self):
        """Auto-save map periodically."""
        if self.current_map is not None:
            try:
                self.save_map(self.map_name)
                self.get_logger().info(
                    f'Auto-saved map: {self.map_name}',
                    throttle_duration_sec=60.0
                )
            except Exception as e:
                self.get_logger().error(f'Auto-save failed: {str(e)}')
    
    def save_map(self, name):
        """Save map to disk."""
        # Convert map data to numpy array
        map_array = np.array(self.current_map.data).reshape(
            self.current_map.info.height,
            self.current_map.info.width
        )
        
        # Save map data
        np.save(os.path.join(self.map_dir, f'{name}.npy'), map_array)
        
        # Save metadata
        metadata = {
            'name': name,
            'timestamp': datetime.now().isoformat(),
            'resolution': float(self.current_map.info.resolution),
            'width': int(self.current_map.info.width),
            'height': int(self.current_map.info.height),
            'origin': {
                'x': float(self.current_map.info.origin.position.x),
                'y': float(self.current_map.info.origin.position.y),
                'z': float(self.current_map.info.origin.position.z),
            }
        }
        
        with open(os.path.join(self.map_dir, f'{name}.yaml'), 'w') as f:
            yaml.dump(metadata, f)
    
    def load_map(self, name):
        """Load map from disk."""
        # Load map data
        map_array = np.load(os.path.join(self.map_dir, f'{name}.npy'))
        
        # Load metadata
        with open(os.path.join(self.map_dir, f'{name}.yaml'), 'r') as f:
            metadata = yaml.safe_load(f)
        
        # Create OccupancyGrid message
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info.resolution = metadata['resolution']
        msg.info.width = metadata['width']
        msg.info.height = metadata['height']
        msg.info.origin.position.x = metadata['origin']['x']
        msg.info.origin.position.y = metadata['origin']['y']
        msg.info.origin.position.z = metadata['origin']['z']
        msg.info.origin.orientation.w = 1.0
        
        msg.data = map_array.flatten().tolist()
        
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MapManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
