#!/usr/bin/env python3
"""Simple map publisher node"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np
import os
from pathlib import Path

try:
    from PIL import Image
except ImportError:
    Image = None


class SimpleMapPublisher(Node):
    def __init__(self):
        super().__init__('simple_map_publisher')
        
        # Declare parameters
        self.declare_parameter('map_yaml', '')
        
        map_yaml = self.get_parameter('map_yaml').value
        
        if not map_yaml or not os.path.exists(map_yaml):
            self.get_logger().error(f"Map YAML file not found: {map_yaml}")
            raise RuntimeError(f"Map YAML file not found: {map_yaml}")
        
        # Parse YAML
        import yaml
        with open(map_yaml, 'r') as f:
            map_config = yaml.safe_load(f)
        
        # Get map image path
        map_dir = os.path.dirname(map_yaml)
        image_file = os.path.join(map_dir, map_config['image'])
        
        if not os.path.exists(image_file):
            self.get_logger().error(f"Map image file not found: {image_file}")
            raise RuntimeError(f"Map image file not found: {image_file}")
        
        # Load image
        if Image is None:
            self.get_logger().error("PIL not available, cannot load image")
            raise RuntimeError("PIL required for map loading")
        
        img = Image.open(image_file)
        img_array = np.array(img)
        
        # Convert to occupancy grid
        resolution = map_config.get('resolution', 0.05)
        origin_x = map_config['origin'][0]
        origin_y = map_config['origin'][1]
        negate = map_config.get('negate', 0)
        occupied_thresh = map_config.get('occupied_thresh', 0.65)
        free_thresh = map_config.get('free_thresh', 0.25)
        
        # Normalize image to 0-255
        if img_array.dtype != np.uint8:
            img_array = (img_array * 255).astype(np.uint8)
        
        if negate:
            img_array = 255 - img_array
        
        # Convert to occupancy grid data (0-100, 100=occupied, 0=free, -1=unknown)
        img_normalized = img_array.astype(np.float32) / 255.0
        occupancy_data = np.where(img_normalized > occupied_thresh, 100,
                                  np.where(img_normalized < free_thresh, 0, -1)).astype(np.int8)
        
        # Flatten and flip if needed
        occupancy_list = occupancy_data.flatten().tolist()
        
        # Create OccupancyGrid message
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.info.resolution = resolution
        msg.info.width = img_array.shape[1]
        msg.info.height = img_array.shape[0]
        msg.info.origin = Pose()
        msg.info.origin.position.x = float(origin_x)
        msg.info.origin.position.y = float(origin_y)
        msg.info.origin.orientation.w = 1.0
        
        msg.data = occupancy_list
        
        self.msg = msg
        self.pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        self.get_logger().info(f"Published map: {img_array.shape[1]}x{img_array.shape[0]}, "
                              f"resolution={resolution}, origin=({origin_x}, {origin_y})")
        
        # Publish continuously at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_callback)
    
    def publish_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
