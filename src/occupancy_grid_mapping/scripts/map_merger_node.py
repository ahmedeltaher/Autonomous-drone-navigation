#!/usr/bin/env python3
"""
Map Merger Node
Merge multiple occupancy grids for multi-session mapping

Feature ID: 1.1.7 - Occupancy Grid Mapping
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np


class MapMergerNode(Node):
    def __init__(self):
        super().__init__('map_merger_node')
        
        self.declare_parameter('merge_method', 'average')  # 'average', 'max', 'min'
        self.method = self.get_parameter('merge_method').value
        
        # Store maps
        self.maps = {}
        
        # Subscribers
        self.create_subscription(
            OccupancyGrid,
            '/occupancy_grid',
            lambda msg: self.map_callback(msg, 'grid1'),
            10
        )
        
        self.create_subscription(
            OccupancyGrid,
            '/slam/map',
            lambda msg: self.map_callback(msg, 'grid2'),
            10
        )
        
        # Publisher
        self.merged_pub = self.create_publisher(
            OccupancyGrid,
            '/merged_map',
            10
        )
        
        # Merge timer
        self.create_timer(2.0, self.merge_maps)
        
        self.get_logger().info(f'Map Merger Node initialized (method: {self.method})')
    
    def map_callback(self, msg, source):
        """Store incoming maps."""
        self.maps[source] = msg
    
    def merge_maps(self):
        """Merge all available maps."""
        if len(self.maps) < 2:
            return
        
        # Use first map as reference
        ref_map = list(self.maps.values())[0]
        
        # Create merged grid
        merged = OccupancyGrid()
        merged.header = ref_map.header
        merged.info = ref_map.info
        
        # Merge data
        grids = [np.array(m.data).reshape(m.info.height, m.info.width) 
                for m in self.maps.values()]
        
        if self.method == 'average':
            merged_data = np.mean(grids, axis=0)
        elif self.method == 'max':
            merged_data = np.max(grids, axis=0)
        elif self.method == 'min':
            merged_data = np.min(grids, axis=0)
        else:
            merged_data = grids[0]
        
        merged.data = merged_data.flatten().astype(np.int8).tolist()
        
        self.merged_pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = MapMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
