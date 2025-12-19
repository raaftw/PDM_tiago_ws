import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, PoseWithCovarianceStamped
import math
import random
from typing import List, Tuple, Optional

class TreeNode:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.parent = None 
        self.cost = 0.0
    
    # Use squared distance for faster nearest neighbor search
    def distance_sq_to(self, other: 'TreeNode') -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        return dx*dx + dy*dy

    # Use real distance for cost and steps calculations
    def distance_to(self, other: 'TreeNode') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

class RRTStarPlanner(Node):
    """
    RRT* Global Planner Node: receives map and start/goal poses, computes a global path using RRT* algorithm.
    This algorithm only runs when a new goal is received.

    Input topics: 
    - /map : map data (OccupancyGrid)
    - /mobile_base_controller/odom or /initialpose : start pose (given by Gazebo or clicked in rviz)
    - /goal_pose : goal pose (clicked by user in rviz)

    Output topics:
    - /reference_path: computed global path (Path)
    """
    def __init__(self):
        super().__init__('rrt_star_planner_node')
        self.get_logger().info("RRT* Global Planner Started.")

        # Declare ROS params
        self.declare_parameter('max_iterations', 1000)
        self.declare_parameter('step_size', 1.0)
        self.declare_parameter('goal_sample_rate', 0.3)
        self.declare_parameter('rewire_radius', 3.0)
        self.declare_parameter('robot_radius', 0.27)  # half of 54cm base
        self.declare_parameter('safety_margin', 0.30)  # 30cm clearance

        self.map_data = None
        self.start_pose = None
        self.inflated_map = None  # Cache inflated map for speed
        
        # Latched QoS for map and path
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, latched_qos)
        self.create_subscription(Odometry, '/mobile_base_controller/odom', self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_publisher = self.create_publisher(Path, '/reference_path', latched_qos)
        
        self.current_odom = None
        self.get_logger().info("Waiting for map...")

    def map_callback(self, msg: OccupancyGrid):
        if self.map_data is None:
            self.map_data = msg
            self._inflate_map()
            self.get_logger().info("Map received and inflated.")

    def odom_callback(self, msg):
        self.current_odom = msg
        self.get_logger().debug("Current odom position updated.")

    def goal_callback(self, goal_msg: PoseStamped):
        if self.map_data is None:
            self.get_logger().warn("Map not received yet.")
            return

        if self.current_odom is None:
            self.get_logger().warn("Current odom position not received yet.")
            return

        start_ps = PoseStamped()
        start_ps.header = self.current_odom.header
        start_ps.pose = self.current_odom.pose.pose

        self.get_logger().info("New goal received. Starting RRT* from current position...")
        path = self._rrt_star_search(start_ps, goal_msg)
        
        if path:
            self.publish_path(path)
            self.get_logger().info(f"Path found with {len(path)} waypoints.")
        else:
            self.get_logger().warn("No path found.")

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        res = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        return int((x - origin_x) / res), int((y - origin_y) / res)

    def _inflate_map(self):
        """
        Pre-compute inflated map once. Much faster than checking circle on every call.
        Inflates obstacles by robot_radius + safety_margin.
        """
        robot_radius = self.get_parameter('robot_radius').value
        safety_margin = self.get_parameter('safety_margin').value
        total_radius = robot_radius + safety_margin
        
        res = self.map_data.info.resolution
        inflate_cells = int(math.ceil(total_radius / res))
        
        width = self.map_data.info.width
        height = self.map_data.info.height
        
        # Copy original map
        self.inflated_map = list(self.map_data.data)
        
        # Pre-compute circular kernel indices
        kernel = []
        for dx in range(-inflate_cells, inflate_cells + 1):
            for dy in range(-inflate_cells, inflate_cells + 1):
                dist = math.sqrt(dx*dx + dy*dy) * res
                if dist <= total_radius:
                    kernel.append((dx, dy))
        
        # Inflate obstacles
        for gy in range(height):
            for gx in range(width):
                index = gy * width + gx
                if self.map_data.data[index] >= 30:  # Occupied cell
                    # Inflate around this obstacle
                    for dx, dy in kernel:
                        ngx, ngy = gx + dx, gy + dy
                        if 0 <= ngx < width and 0 <= ngy < height:
                            nindex = ngy * width + ngx
                            self.inflated_map[nindex] = 100  # Mark as occupied
        
        self.get_logger().info(f"Map inflated with {total_radius:.2f}m radius ({inflate_cells} cells)")
    
    def _is_valid_cell(self, x: float, y: float) -> bool:
        """
        Check if position (x, y) is valid using pre-inflated map (much faster).
        """
        gx, gy = self._world_to_grid(x, y)
        if gx < 0 or gx >= self.map_data.info.width or \
           gy < 0 or gy >= self.map_data.info.height:
            return False
        
        index = gy * self.map_data.info.width + gx
        return self.inflated_map[index] < 30

    def _check_line(self, n1, n2) -> bool:
        # FIX 4: Use REAL distance here to calculate steps
        dist = n1.distance_to(n2) 
        res = self.map_data.info.resolution
        steps = max(1, int(math.ceil(dist / res)))
        
        for i in range(steps + 1):
            t = i / steps
            x = n1.x + t * (n2.x - n1.x)
            y = n1.y + t * (n2.y - n1.y)
            if not self._is_valid_cell(x, y):
                return False
        return True

    def _rrt_star_search(self, start_pose, goal_pose) -> Optional[List[Pose]]:
        max_iterations = self.get_parameter('max_iterations').value
        step_size = self.get_parameter('step_size').value
        goal_sample_rate = self.get_parameter('goal_sample_rate').value
        rewire_radius = self.get_parameter('rewire_radius').value
        
        # Pre-calculate squared values for comparisons
        step_size_sq = step_size * step_size
        rewire_radius_sq = rewire_radius * rewire_radius

        start_x, start_y = start_pose.pose.position.x, start_pose.pose.position.y
        goal_x, goal_y = goal_pose.pose.position.x, goal_pose.pose.position.y
        
        if not self._is_valid_cell(start_x, start_y) or not self._is_valid_cell(goal_x, goal_y):
            self.get_logger().warn("Start or Goal is occupied!")
            return None
        
        start_node = TreeNode(start_x, start_y)
        self.tree = [start_node]
        final_goal_node = None
        
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        map_w = self.map_data.info.width * self.map_data.info.resolution
        map_h = self.map_data.info.height * self.map_data.info.resolution
        
        for _ in range(max_iterations):
            # 1. Sample
            if random.random() < goal_sample_rate:
                rand_node = TreeNode(goal_x, goal_y)
            else:
                rand_node = TreeNode(origin_x + random.random() * map_w, 
                                     origin_y + random.random() * map_h)
            
            # 2. Nearest (Use Squared - Fast)
            nearest = min(self.tree, key=lambda n: n.distance_sq_to(rand_node))
            
            # 3. Steer
            dist_sq = nearest.distance_sq_to(rand_node)
            if dist_sq < step_size_sq:
                new_node = rand_node
            else:
                angle = math.atan2(rand_node.y - nearest.y, rand_node.x - nearest.x)
                new_node = TreeNode(nearest.x + step_size * math.cos(angle),
                                    nearest.y + step_size * math.sin(angle))
            
            # 4. Collision Check
            if not self._check_line(nearest, new_node):
                continue
            
            # 5. Connect
            # Use Squared for finding neighbors (Fast)
            neighbors = [n for n in self.tree if n.distance_sq_to(new_node) < rewire_radius_sq]
            best_parent = nearest
            
            # FIX 5: Use Real Distance (sqrt) for Cost Calculation!
            min_cost = nearest.cost + nearest.distance_to(new_node)
            
            for nb in neighbors:
                if self._check_line(nb, new_node):
                    # FIX 6: Cost must be linear distance
                    cost = nb.cost + nb.distance_to(new_node)
                    if cost < min_cost:
                        min_cost = cost
                        best_parent = nb
            
            new_node.parent = best_parent
            new_node.cost = min_cost
            self.tree.append(new_node)
            
            # 6. Rewire
            for nb in neighbors:
                if nb == best_parent: continue
                
                # FIX 7: Cost must be linear distance
                new_cost = new_node.cost + new_node.distance_to(nb)
                
                if new_cost < nb.cost and self._check_line(new_node, nb):
                    nb.parent = new_node
                    nb.cost = new_cost
            
            # 7. Check Goal
            if new_node.distance_sq_to(TreeNode(goal_x, goal_y)) < step_size_sq:
                if self._check_line(new_node, TreeNode(goal_x, goal_y)):
                    goal_node = TreeNode(goal_x, goal_y)
                    goal_node.parent = new_node
                    goal_node.cost = new_node.cost + new_node.distance_to(goal_node)
                    
                    if final_goal_node is None or goal_node.cost < final_goal_node.cost:
                        final_goal_node = goal_node

        if final_goal_node:
            path = []
            curr = final_goal_node
            while curr:
                path.append(Pose(position=Point(x=curr.x, y=curr.y, z=0.0)))
                curr = curr.parent
            path.reverse()
            return path
        return None

    def publish_path(self, poses: List[Pose]):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for p in poses:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = p
            msg.poses.append(ps)
        self.path_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RRTStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()