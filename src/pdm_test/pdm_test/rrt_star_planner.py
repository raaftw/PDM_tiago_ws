#!/usr/bin/env python3
import math
import random
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


class MapData:
    def __init__(self, msg: OccupancyGrid, occ_threshold: int = 50, clearance_m: float = 0.3):
        self.info = msg.info
        self.res = self.info.resolution
        self.ox = self.info.origin.position.x
        self.oy = self.info.origin.position.y
        self.width = self.info.width
        self.height = self.info.height
        self.occ_threshold = occ_threshold
        self.clearance_m = clearance_m

        data = list(msg.data)
        total = self.width * self.height
        occ = [False] * total

        # Mark unknown or below threshold as occupied
        for idx, v in enumerate(data):
            if v < 0 or v < self.occ_threshold:
                occ[idx] = True

        # Inflate obstacles by clearance radius
        r_cells = int(math.ceil(self.clearance_m / self.res)) if self.clearance_m > 0 else 0
        if r_cells > 0:
            inflated = occ[:]
            r2 = r_cells * r_cells
            w, h = self.width, self.height
            for y in range(h):
                row_offset = y * w
                for x in range(w):
                    if not occ[row_offset + x]:
                        continue
                    y_min = max(0, y - r_cells)
                    y_max = min(h - 1, y + r_cells)
                    for ny in range(y_min, y_max + 1):
                        dy = ny - y
                        dy2 = dy * dy
                        row_off_n = ny * w
                        x_min = max(0, x - r_cells)
                        x_max = min(w - 1, x + r_cells)
                        for nx in range(x_min, x_max + 1):
                            dx = nx - x
                            if dx * dx + dy2 <= r2:
                                inflated[row_off_n + nx] = True
            occ = inflated

        self.occ = occ

    def world_to_map(self, x: float, y: float):
        mx = int((x - self.ox) / self.res)
        my = int((y - self.oy) / self.res)
        return mx, my

    def is_free(self, x: float, y: float):
        mx, my = self.world_to_map(x, y)
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return False
        idx = my * self.width + mx
        return not self.occ[idx]

    def is_segment_free(self, x1, y1, x2, y2, step: float):
        dx = x2 - x1
        dy = y2 - y1
        dist = math.hypot(dx, dy)
        # Use a fine step to avoid skipping thin obstacles
        step = min(step, self.res * 0.5)
        n = max(int(dist / step), 1)
        for i in range(n + 1):
            t = i / n
            x = x1 + t * dx
            y = y1 + t * dy
            if not self.is_free(x, y):
                return False
        return True

    def sample_free(self):
        for _ in range(100):
            x = random.uniform(self.ox, self.ox + self.width * self.res)
            y = random.uniform(self.oy, self.oy + self.height * self.res)
            if self.is_free(x, y):
                return x, y
        return None


class RRTStarPlanner(Node):
    def __init__(self):
        super().__init__('rrt_star_planner')

        # Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('occ_threshold', 50)
        self.declare_parameter('clearance_m', 0.3)
        self.declare_parameter('step_size', 0.5)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('max_iters', 2000)
        self.declare_parameter('rewire_radius', 1.0)

        self.map_topic = self.get_parameter('map_topic').value
        self.path_topic = self.get_parameter('path_topic').value

        self.map_data: Optional[MapData] = None
        self.has_planned = False

        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('RRT* planner ready, waiting for map...')

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = MapData(
            msg,
            occ_threshold=int(self.get_parameter('occ_threshold').value),
            clearance_m=float(self.get_parameter('clearance_m').value),
        )
        if not hasattr(self, '_map_logged'):
            self.get_logger().info(
                f"Map received: {msg.info.width}x{msg.info.height}, res={msg.info.resolution}"
            )
            self._map_logged = True

    def timer_callback(self):
        if self.map_data is None or self.has_planned:
            return

        start = (0.0, 0.0)
        goal = (
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
        )

        if not self.plan_and_publish(start, goal):
            self.get_logger().warn('RRT* failed to find a path.')
        else:
            self.get_logger().info('RRT* path published.')

        self.has_planned = True

    # ---------------- Planning ----------------

    def nearest(self, nodes: List[Tuple[float, float]], q: Tuple[float, float]):
        return min(range(len(nodes)), key=lambda i: (nodes[i][0]-q[0])**2 + (nodes[i][1]-q[1])**2)

    def steer(self, from_pt, to_pt, step_size):
        fx, fy = from_pt
        tx, ty = to_pt
        dx, dy = tx - fx, ty - fy
        dist = math.hypot(dx, dy)
        if dist <= step_size:
            return to_pt
        scale = step_size / dist
        return fx + dx * scale, fy + dy * scale

    def distance(self, a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    def plan_and_publish(self, start, goal) -> bool:
        m = self.map_data
        step_size = float(self.get_parameter('step_size').value)
        collision_step = min(self.map_data.res * 0.5, step_size * 0.25)
        goal_tol = float(self.get_parameter('goal_tolerance').value)
        max_iters = int(self.get_parameter('max_iters').value)
        rewire_r = float(self.get_parameter('rewire_radius').value)

        if not m.is_free(*start):
            self.get_logger().warn('Start in obstacle.')
            return False
        if not m.is_free(*goal):
            self.get_logger().warn('Goal in obstacle.')
            return False

        nodes: List[Tuple[float, float]] = [start]
        parents: List[int] = [-1]
        cost: List[float] = [0.0]

        best_goal_idx = None
        best_goal_cost = float('inf')

        for it in range(max_iters):
            sample = m.sample_free()
            if sample is None:
                continue

            nearest_idx = self.nearest(nodes, sample)
            new_pt = self.steer(nodes[nearest_idx], sample, step_size)

            if not m.is_segment_free(*nodes[nearest_idx], *new_pt, step=collision_step):
                continue

            new_cost = cost[nearest_idx] + self.distance(nodes[nearest_idx], new_pt)

            # choose better parent among nearby nodes
            neighbors = [i for i, n in enumerate(nodes) if self.distance(n, new_pt) <= rewire_r]
            best_parent = nearest_idx
            best_parent_cost = new_cost
            for nb in neighbors:
                c = cost[nb] + self.distance(nodes[nb], new_pt)
                if c < best_parent_cost and m.is_segment_free(*nodes[nb], *new_pt, step=collision_step):
                    best_parent = nb
                    best_parent_cost = c

            nodes.append(new_pt)
            parents.append(best_parent)
            cost.append(best_parent_cost)
            new_idx = len(nodes) - 1

            # rewire neighbors to new node if better
            for nb in neighbors:
                alt = cost[new_idx] + self.distance(nodes[new_idx], nodes[nb])
                if alt < cost[nb] and m.is_segment_free(*nodes[new_idx], *nodes[nb], step=collision_step):
                    parents[nb] = new_idx
                    cost[nb] = alt

            # check goal
            if self.distance(new_pt, goal) <= goal_tol and m.is_segment_free(*new_pt, *goal, step=collision_step):
                goal_cost = cost[new_idx] + self.distance(new_pt, goal)
                if goal_cost < best_goal_cost:
                    best_goal_cost = goal_cost
                    best_goal_idx = new_idx

            if best_goal_idx is not None and it > max_iters * 0.2:
                break

        if best_goal_idx is None:
            self.get_logger().warn('No goal reached within iterations.')
            return False

        # build path back
        path_pts = [(goal[0], goal[1])]
        idx = best_goal_idx
        while idx != -1:
            path_pts.append(nodes[idx])
            idx = parents[idx]
        path_pts.reverse()

        self.publish_path(path_pts)
        return True

    # ---------------- Publishing ----------------

    def publish_path(self, pts: List[tuple]):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        q = Quaternion()
        q.w = 1.0

        for (x, y) in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = Pose()
            ps.pose.position = Point(x=x, y=y, z=0.0)
            ps.pose.orientation = q
            msg.poses.append(ps)

        self.path_pub.publish(msg)
        self.get_logger().info(f'Published RRT* path with {len(pts)} points.')


def main(args=None):
    rclpy.init(args=args)
    node = RRTStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
