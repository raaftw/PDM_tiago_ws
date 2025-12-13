import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
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
    
    # FIX 1: Explicitly name this "Squared" so we don't use it by accident
    def distance_sq_to(self, other: 'TreeNode') -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        return dx*dx + dy*dy

    # FIX 2: We still need Real distance for Cost and Steps calculations
    def distance_to(self, other: 'TreeNode') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

class RRTStarPlanner(Node):
    def __init__(self):
        super().__init__('rrt_star_planner_node')
        self.get_logger().info("RRT* Global Planner Started.")

        self.declare_parameter('max_iterations', 1000)
        self.declare_parameter('step_size', 1.0)
        self.declare_parameter('goal_sample_rate', 0.3)
        self.declare_parameter('rewire_radius', 3.0)

        self.map_data = None
        self.start_pose = None
        
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, latched_qos)
        self.create_subscription(PoseStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_publisher = self.create_publisher(Path, '/global_path', latched_qos)
        
        self.get_logger().info("Waiting for map...")

    def map_callback(self, msg: OccupancyGrid):
        if self.map_data is None:
            self.map_data = msg
            self.get_logger().info("Map received.")

    def pose_callback(self, msg: PoseStamped):
        self.current_robot_pose = msg

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.start_pose = ps
        self.get_logger().info("Start pose set manually.")

    def goal_callback(self, goal_msg: PoseStamped):
        if self.map_data is None:
            self.get_logger().warn("Map not received yet.")
            return

        start = self.start_pose if self.start_pose else self.current_robot_pose
        if start is None:
            self.get_logger().warn("No start pose yet.")
            return

        self.get_logger().info("Starting RRT* Global Planner...")
        path = self._rrt_star_search(start, goal_msg)
        
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

    def _is_valid_cell(self, x: float, y: float) -> bool:
        gx, gy = self._world_to_grid(x, y)
        if gx < 0 or gx >= self.map_data.info.width or \
           gy < 0 or gy >= self.map_data.info.height:
            return False
        
        index = gy * self.map_data.info.width + gx
        occupancy = self.map_data.data[index]
        
        # FIX 3: Map data is 0-100. '30' is a standard threshold.
        return occupancy < 30

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