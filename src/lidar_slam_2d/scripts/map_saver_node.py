#!/usr/bin/env python3
"""
Map Saver Node
Save and load occupancy grid maps

Feature ID: 1.1.3 - 2D Lidar SLAM
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
import numpy as np
import yaml
import os


class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        
        self.declare_parameter('map_filename', 'indoor_map')
        self.map_filename = self.get_parameter('map_filename').value
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/slam/map',
            self.map_callback,
            10
        )
        
        self.save_srv = self.create_service(
            Trigger,
            '/save_map',
            self.save_map_callback
        )
        
        self.current_map = None
        self.get_logger().info('Map Saver Node initialized')
    
    def map_callback(self, msg):
        self.current_map = msg
    
    def save_map_callback(self, request, response):
        if self.current_map is None:
            response.success = False
            response.message = 'No map available'
            return response
        
        try:
            # Save map data
            map_data = np.array(self.current_map.data).reshape(
                self.current_map.info.height,
                self.current_map.info.width
            )
            
            np.save(f'{self.map_filename}.npy', map_data)
            
            # Save metadata
            metadata = {
                'resolution': self.current_map.info.resolution,
                'width': self.current_map.info.width,
                'height': self.current_map.info.height,
                'origin': {
                    'x': self.current_map.info.origin.position.x,
                    'y': self.current_map.info.origin.position.y,
                }
            }
            
            with open(f'{self.map_filename}.yaml', 'w') as f:
                yaml.dump(metadata, f)
            
            response.success = True
            response.message = f'Map saved to {self.map_filename}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed to save map: {str(e)}'
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
