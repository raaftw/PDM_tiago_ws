import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

# Input: OccupancyGrid map, Output: Path
from nav_msgs.msg import OccupancyGrid, Path

# Path: Array of PoseStamped
from geometry_msgs.msg import PoseStamped, Pose, Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header

import heapq
import math
import random
from typing import List, Tuple, Optional 

### TODO: Update topic names and Frame Name

class TreeNode:
    """Node in the RRT* tree."""
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0
        self.children = []
    
    def distance_to(self, other: 'TreeNode') -> float:
        """Calculate Euclidean distance to another node."""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

class RRTStarPlanner(Node):
    def __init__(self):
        super().__init__('rrt_star_planner_node')
        self.get_logger().info("RRT* Global Planner Node Started.")

        # --- A. Store Map Data ---
        # The map data (resolution, origin, width, height, and grid data)
        self.map_data = None
        self.current_robot_pose = None
        self.start_pose = None
        self.goal_pose = None

        # Subscriber for the static map topic with transient_local durability
        # (map_server publishes with latched durability, which requires transient_local QoS to receive)
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        # Subscriber for the robot's current pose (e.g., AMCL)
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        # Allow manual start pose from RViz: 2D Pose Estimate publishes /initialpose (PoseWithCovarianceStamped)
        self.initialpose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )
        
        # --- B. Publishers and Subscribers for Planning ---
        # Subscriber for the goal pose (published by RViz on the '/goal_pose' topic)
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Publish path with transient_local durability for RViz compatibility
        path_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.path_publisher = self.create_publisher(
            Path,
            '/reference_path',
            path_qos
        )
        
        self.get_logger().info("Waiting for map data on /map topic...")
        
        # RRT* parameters - reduced for faster planning
        self.max_iterations = 500  # Reduced from 5000 for faster planning
        self.step_size = 1.0  # Increased from 0.5 for larger steps
        self.goal_sample_rate = 0.2  # 20% of iterations try to reach goal
        self.rewire_radius = 3.0  # Increased from 2.0 for better connectivity
        self.tree = None

    def map_callback(self, msg: OccupancyGrid):
        """Stores the map data and its metadata (resolution, origin) once."""
        self.get_logger().info(f"[DEBUG] map_callback triggered. Map already set: {self.map_data is not None}")
        if self.map_data is None:
            self.map_data = msg
            self.get_logger().info(f"Map received and stored. Resolution: {msg.info.resolution} m/cell, Size: {msg.info.width}x{msg.info.height}")
        else:
            self.get_logger().info(f"Map already stored, ignoring update.")
    
    def pose_callback(self, msg: PoseStamped):
        """Updates the current robot pose."""
        self.current_robot_pose = msg

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """Store manual start pose from RViz (converted to PoseStamped)."""
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.start_pose = ps
        self.get_logger().info("Start pose set from /initialpose")

    def goal_callback(self, goal_pose_msg: PoseStamped):
        """Called when a new goal is received from RViz."""
        self.goal_pose = goal_pose_msg
        if self.map_data is None:
            self.get_logger().warn("Cannot plan: Map data not yet received.")
            return

        # Determine start pose: prefer manual start from /initialpose; fallback to /amcl_pose
        start_pose = self.start_pose if self.start_pose is not None else self.current_robot_pose
        if start_pose is None:
            self.get_logger().warn("Cannot plan: No start pose. Click '2D Pose Estimate' in RViz or provide /amcl_pose.")
            return

        self.get_logger().info("New goal received. Starting RRT* planning...")
        
        # Run RRT* to generate the path
        self.get_logger().info(f"[DEBUG] Calling _rrt_star_search from start {start_pose.pose.position.x:.2f}, {start_pose.pose.position.y:.2f} to goal {goal_pose_msg.pose.position.x:.2f}, {goal_pose_msg.pose.position.y:.2f}")
        try:
            path_waypoints_world = self._rrt_star_search(start_pose, goal_pose_msg)
        except Exception as e:
            self.get_logger().error(f"Exception during RRT* search: {str(e)}")
            return
        
        if path_waypoints_world is None:
            self.get_logger().warn("RRT* search failed: No path found")
            return
        
        self.get_logger().info(f"[DEBUG] Path found with {len(path_waypoints_world)} waypoints, creating message...")
        try:
            path_msg = self._create_path_message(path_waypoints_world)
            self.path_publisher.publish(path_msg)
            self.get_logger().info(f"Path published with {len(path_waypoints_world)} waypoints")
        except Exception as e:
            self.get_logger().error(f"Exception while publishing path: {str(e)}")
    
    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices."""
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        resolution = self.map_data.info.resolution
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        return grid_x, grid_y
    
    def _grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates."""
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        resolution = self.map_data.info.resolution
        
        x = origin_x + (grid_x + 0.5) * resolution
        y = origin_y + (grid_y + 0.5) * resolution
        
        return x, y
    
    def _is_valid_cell(self, grid_x: int, grid_y: int) -> bool:
        """Check if a grid cell is valid and not occupied."""
        if grid_x < 0 or grid_x >= self.map_data.info.width:
            return False
        if grid_y < 0 or grid_y >= self.map_data.info.height:
            return False
        
        # Check occupancy (0 = free, 100 = occupied, -1 = unknown)
        index = grid_y * self.map_data.info.width + grid_x
        occupancy = self.map_data.data[index]
        
        # Treat unknown as occupied; consider cells >= 65 as occupied
        if occupancy < 0:
            return False
        return occupancy < 65
    
    def _heuristic(self, grid_x1: int, grid_y1: int, grid_x2: int, grid_y2: int) -> float:
        """Euclidean distance heuristic."""
        dx = grid_x1 - grid_x2
        dy = grid_y1 - grid_y2
        return math.sqrt(dx*dx + dy*dy)
    
    def _is_collision_free(self, x1: float, y1: float, x2: float, y2: float, samples: int = 0) -> bool:
        """Check if a line segment from (x1, y1) to (x2, y2) is collision-free.
        Adaptively sample every map cell along the segment, including endpoints."""
        # Determine adaptive steps based on map resolution
        resolution = self.map_data.info.resolution
        distance = math.hypot(x2 - x1, y2 - y1)
        steps = max(1, int(math.ceil(distance / resolution)))
        for i in range(steps + 1):  # include endpoints
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            grid_x, grid_y = self._world_to_grid(x, y)
            if not self._is_valid_cell(grid_x, grid_y):
                return False
        return True
    
    def _rrt_star_search(self, start_pose: PoseStamped, goal_pose: PoseStamped) -> Optional[List[Pose]]:
        """Perform RRT* search to find an optimal path from start to goal."""
        # Convert world coordinates to nodes
        start_x = start_pose.pose.position.x
        start_y = start_pose.pose.position.y
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        
        # Validate start and goal
        start_grid_x, start_grid_y = self._world_to_grid(start_x, start_y)
        goal_grid_x, goal_grid_y = self._world_to_grid(goal_x, goal_y)
        
        if not self._is_valid_cell(start_grid_x, start_grid_y):
            self.get_logger().warn(f"Start position ({start_grid_x}, {start_grid_y}) is occupied")
            return None
        if not self._is_valid_cell(goal_grid_x, goal_grid_y):
            self.get_logger().warn(f"Goal position ({goal_grid_x}, {goal_grid_y}) is occupied")
            return None
        
        # Initialize tree
        start_node = TreeNode(start_x, start_y)
        start_node.cost = 0.0
        self.tree = [start_node]
        goal_node = None
        best_path_cost = float('inf')
        
        # Get map bounds
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        resolution = self.map_data.info.resolution
        map_width = self.map_data.info.width * resolution
        map_height = self.map_data.info.height * resolution
        
        # RRT* main loop
        for iteration in range(self.max_iterations):
            # Sample random point or goal
            if random.random() < self.goal_sample_rate:
                random_node = TreeNode(goal_x, goal_y)
            else:
                rand_x = origin_x + random.random() * map_width
                rand_y = origin_y + random.random() * map_height
                random_node = TreeNode(rand_x, rand_y)
            
            # Find nearest node in tree
            nearest_node = min(self.tree, key=lambda n: n.distance_to(random_node))
            
            # Extend towards random node
            distance = nearest_node.distance_to(random_node)
            if distance < self.step_size:
                new_node = random_node
            else:
                # Step in direction of random node
                angle = math.atan2(random_node.y - nearest_node.y, random_node.x - nearest_node.x)
                new_node = TreeNode(
                    nearest_node.x + self.step_size * math.cos(angle),
                    nearest_node.y + self.step_size * math.sin(angle)
                )
            
            # Check collision for line from nearest to new
            if not self._is_collision_free(nearest_node.x, nearest_node.y, new_node.x, new_node.y):
                continue
            
            # Find neighbors within rewire radius
            neighbors = [n for n in self.tree if n.distance_to(new_node) < self.rewire_radius]
            
            # Find best parent
            best_parent = nearest_node
            best_cost = nearest_node.cost + nearest_node.distance_to(new_node)
            
            for neighbor in neighbors:
                if self._is_collision_free(neighbor.x, neighbor.y, new_node.x, new_node.y):
                    cost = neighbor.cost + neighbor.distance_to(new_node)
                    if cost < best_cost:
                        best_cost = cost
                        best_parent = neighbor
            
            # Add new node to tree
            new_node.parent = best_parent
            new_node.cost = best_cost
            best_parent.children.append(new_node)
            self.tree.append(new_node)
            
            # Rewire neighbors
            for neighbor in neighbors:
                if neighbor == best_parent:
                    continue
                new_cost = best_cost + new_node.distance_to(neighbor)
                if new_cost < neighbor.cost and self._is_collision_free(new_node.x, new_node.y, neighbor.x, neighbor.y):
                    neighbor.parent.children.remove(neighbor)
                    neighbor.parent = new_node
                    neighbor.cost = new_cost
                    new_node.children.append(neighbor)
            
            # Check if goal is reached
            if new_node.distance_to(TreeNode(goal_x, goal_y)) < self.step_size:
                if self._is_collision_free(new_node.x, new_node.y, goal_x, goal_y):
                    goal_node = TreeNode(goal_x, goal_y)
                    goal_node.parent = new_node
                    goal_node.cost = new_node.cost + new_node.distance_to(goal_node)
                    new_node.children.append(goal_node)
                    self.tree.append(goal_node)
                    
                    if goal_node.cost < best_path_cost:
                        best_path_cost = goal_node.cost
        
        # Extract path from tree
        if goal_node is None:
            self.get_logger().warn("RRT* failed to find path to goal")
            return None
        
        path = []
        current = goal_node
        while current is not None:
            pose = Pose(position=Point(x=current.x, y=current.y, z=0.0))
            path.append(pose)
            current = current.parent
        
        path.reverse()
        return path
    
    
    def _create_path_message(self, poses: List[Pose]) -> Path:
        """Convert a list of poses to a nav_msgs/Path message."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for pose in poses:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose = pose
            path_msg.poses.append(pose_stamped)
        
        return path_msg


def main(args=None):
    rclpy.init(args=args)
    rrt_star_planner = RRTStarPlanner()
    rclpy.spin(rrt_star_planner)
    rrt_star_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()