#!/usr/bin/env python3
"""
Table Detector Node: Analyzes costmap to detect tables using connected component analysis.
Subscribes to /map, detects obstacles, filters by size, and publishes table positions.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import cv2
from typing import List, Tuple, Dict
import yaml


class TableInfo:
    """Structure to hold table information"""
    def __init__(self, id: str, center: Tuple[float, float], 
                 corners: List[Tuple[float, float]], 
                 width: float, height: float):
        self.id = id
        self.center = center
        self.corners = corners
        self.width = width
        self.height = height


class TableDetector(Node):
    """
    Detects tables from occupancy grid map using connected component analysis.
    
    Subscribes to:
        - /map (nav_msgs/OccupancyGrid): Costmap data
    
    Publishes:
        - /detected_tables (geometry_msgs/PoseArray): Centers of detected tables
        - /table_markers (visualization_msgs/MarkerArray): Visualization of table boundaries
    
    Parameters:
        - occupied_threshold: Occupancy value threshold for obstacles (default: 65)
        - min_table_area: Minimum area in m^2 for table detection (default: 0.15)
        - max_table_area: Maximum area in m^2 for table detection (default: 1.0)
        - save_to_yaml: Whether to save detected tables to YAML file (default: True)
        - yaml_output_path: Path to save YAML file (default: tables_detected.yaml)
    """
    
    def __init__(self):
        super().__init__('table_detector')
        self.get_logger().info("Table Detector Node Started.")
        
        # Declare parameters
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('min_table_area', 0.15)  # m^2
        self.declare_parameter('max_table_area', 1.0)   # m^2
        self.declare_parameter('save_to_yaml', True)
        self.declare_parameter('yaml_output_path', 
                     '/home/raaf/PDM_tiago_ws/src/maps/tables_detected.yaml')
        self.declare_parameter('erode_iterations', 0)  # default off; set >0 to trim inflated edges
        
        # Get parameters
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.min_table_area = self.get_parameter('min_table_area').value
        self.max_table_area = self.get_parameter('max_table_area').value
        self.save_to_yaml = self.get_parameter('save_to_yaml').value
        self.yaml_output_path = self.get_parameter('yaml_output_path').value
        self.erode_iterations = int(self.get_parameter('erode_iterations').value)
        
        # State
        self.map_data = None
        self.tables: List[TableInfo] = []
        
        # QoS for map (latched)
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, latched_qos)
        
        # Publishers
        self.table_pub = self.create_publisher(PoseArray, '/detected_tables', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/table_markers', 10)
        
        self.get_logger().info("Waiting for map data...")
    
    def map_callback(self, msg: OccupancyGrid):
        """Process incoming map and detect tables"""
        if self.map_data is not None:
            return  # Only process map once
        
        self.get_logger().info("Map received, analyzing for tables...")
        self.map_data = msg
        
        # Detect tables
        self.tables = self.detect_tables(msg)
        
        if len(self.tables) > 0:
            self.get_logger().info(f"Detected {len(self.tables)} table(s)")
            
            # Log table information
            for table in self.tables:
                self.get_logger().info(
                    f"  {table.id}: center=({table.center[0]:.2f}, {table.center[1]:.2f}), "
                    f"size={table.width:.2f}x{table.height:.2f}m"
                )
            
            # Publish results
            self.publish_tables()
            self.publish_markers()
            
            # Save to YAML if requested
            if self.save_to_yaml:
                self.save_tables_to_yaml()
        else:
            self.get_logger().warn("No tables detected in the map")
    
    def detect_tables(self, map_msg: OccupancyGrid) -> List[TableInfo]:
        """
        Detect tables using connected component analysis
        
        Args:
            map_msg: OccupancyGrid message
            
        Returns:
            List of TableInfo objects
        """
        # Convert occupancy grid to numpy array
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        
        # Reshape data to 2D array
        grid = np.array(map_msg.data).reshape((height, width))
        
        # Create binary image: 255 for occupied, 0 for free
        binary = np.zeros((height, width), dtype=np.uint8)
        binary[grid >= self.occupied_threshold] = 255

        # Optional erosion to counter inflation artifacts
        if self.erode_iterations > 0:
            kernel = np.ones((3, 3), np.uint8)
            binary = cv2.erode(binary, kernel, iterations=self.erode_iterations)
        
        # Find connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            binary, connectivity=8
        )
        
        tables = []
        
        # Process each component (skip label 0 which is background)
        for i in range(1, num_labels):
            # Get component properties
            area_pixels = stats[i, cv2.CC_STAT_AREA]
            area_m2 = float(area_pixels * (resolution ** 2))
            
            # Filter by area
            if area_m2 < self.min_table_area or area_m2 > self.max_table_area:
                continue
            
            # Get bounding box in pixels
            x_px = stats[i, cv2.CC_STAT_LEFT]
            y_px = stats[i, cv2.CC_STAT_TOP]
            w_px = stats[i, cv2.CC_STAT_WIDTH]
            h_px = stats[i, cv2.CC_STAT_HEIGHT]
            
            # Convert centroid to map coordinates
            cent_x_px = centroids[i, 0]
            cent_y_px = centroids[i, 1]
            
            # Convert from image coords (row, col) to map coords (x, y)
            # Image: y increases downward, x increases rightward
            # Map: x increases rightward, y increases upward
            center_x = float(origin_x + cent_x_px * resolution)
            center_y = float(origin_y + (height - cent_y_px) * resolution)
            
            # Calculate bounding box dimensions in meters
            width_m = float(w_px * resolution)
            height_m = float(h_px * resolution)
            
            # Calculate corners (assuming axis-aligned rectangle)
            # Bottom-left corner
            bl_x = float(origin_x + x_px * resolution)
            bl_y = float(origin_y + (height - (y_px + h_px)) * resolution)
            
            # Calculate all four corners (counter-clockwise from bottom-left)
            corners = [
                (bl_x, bl_y),                           # bottom-left
                (bl_x + width_m, bl_y),                 # bottom-right
                (bl_x + width_m, bl_y + height_m),      # top-right
                (bl_x, bl_y + height_m)                 # top-left
            ]
            
            # Create table info
            table = TableInfo(
                id=f"table_{i}",
                center=(center_x, center_y),
                corners=corners,
                width=width_m,
                height=height_m
            )
            
            tables.append(table)
            
        return tables
    
    def publish_tables(self):
        """Publish detected table centers as PoseArray"""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'
        
        for table in self.tables:
            pose = Pose()
            pose.position.x = table.center[0]
            pose.position.y = table.center[1]
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        self.table_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(self.tables)} table centers")
    
    def publish_markers(self):
        """Publish visualization markers for table boundaries"""
        marker_array = MarkerArray()
        
        for idx, table in enumerate(self.tables):
            # Create a line strip marker for table boundary
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'table_boundaries'
            marker.id = idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Set marker properties
            marker.scale.x = 0.05  # Line width
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            # Add corner points (close the loop)
            for corner in table.corners:
                point = Point()
                point.x = corner[0]
                point.y = corner[1]
                point.z = 0.0
                marker.points.append(point)
            
            # Close the loop
            point = Point()
            point.x = table.corners[0][0]
            point.y = table.corners[0][1]
            point.z = 0.0
            marker.points.append(point)
            
            marker_array.markers.append(marker)
            
            # Add text label at center
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'table_labels'
            text_marker.id = idx + 1000  # Offset ID to avoid conflicts
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = table.center[0]
            text_marker.pose.position.y = table.center[1]
            text_marker.pose.position.z = 0.5
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = table.id
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published markers for {len(self.tables)} tables")
    
    def save_tables_to_yaml(self):
        """Save detected tables to YAML file"""
        try:
            tables_dict = {
                'tables': []
            }
            
            for table in self.tables:
                table_dict = {
                    'id': table.id,
                    'center': [float(round(table.center[0], 5)), float(round(table.center[1], 5))],
                    'width': float(round(table.width, 5)),
                    'height': float(round(table.height, 5)),
                    'corners': [
                        [float(round(c[0], 5)), float(round(c[1], 5))] for c in table.corners
                    ]
                }
                tables_dict['tables'].append(table_dict)
            
            # Add metadata
            tables_dict['metadata'] = {
                'detection_method': 'connected_components',
                'min_area_m2': self.min_table_area,
                'max_area_m2': self.max_table_area,
                'occupied_threshold': self.occupied_threshold
            }
            
            with open(self.yaml_output_path, 'w') as f:
                yaml.safe_dump(tables_dict, f, default_flow_style=False, sort_keys=False)
            
            self.get_logger().info(f"Saved table data to {self.yaml_output_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save YAML: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TableDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
