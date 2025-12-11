#!/usr/bin/env python3
import math
from typing import Optional, List

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header


# ---------------- Map Helper ----------------

class MapData:
    def __init__(self, msg: OccupancyGrid, occ_threshold: int = 50):
        self.msg = msg
        self.info = msg.info
        self.res = self.info.resolution
        self.ox = self.info.origin.position.x
        self.oy = self.info.origin.position.y
        self.width = self.info.width
        self.height = self.info.height
        self.occ_threshold = occ_threshold

        # copy map data
        self.data = list(msg.data)

    def world_to_map(self, x: float, y: float):
        mx = int((x - self.ox) / self.res)
        my = int((y - self.oy) / self.res)
        return mx, my

    def cell_value(self, x: float, y: float):
        mx, my = self.world_to_map(x, y)
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return None
        idx = my * self.width + mx
        v = self.data[idx]
        if v < 0:
            return v
        return v

    def is_free(self, x: float, y: float):
        v = self.cell_value(x, y)
        if v is None:
            return False
        if v < 0:
            return False  # unknown treated as obstacle
        # Map uses high=free (e.g., 100) and low=obstacle (e.g., 0)
        return v >= self.occ_threshold

    def is_segment_free(self, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        dist = math.hypot(dx, dy)
        step = self.res * 0.5
        n = max(int(dist / step), 1)
        for i in range(n + 1):
            t = i / n
            x = x1 + t * dx
            y = y1 + t * dy
            if not self.is_free(x, y):
                return False
        return True


# ---------------- Planner Node ----------------

class StraightLinePlannerNode(Node):
    def __init__(self):
        super().__init__("straight_line_planner")

        # Declare parameters
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("path_topic", "/global_path")
        # Robot start fixed at (0,0)
        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("occ_threshold", 50)

        # Get parameters
        self.map_topic = self.get_parameter("map_topic").value
        self.path_topic = self.get_parameter("path_topic").value

        self.map_data: Optional[MapData] = None
        self.has_planned = False

        # ROS interfaces
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 10
        )
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("Straight-line planner ready, waiting for map...")

    # ---------------- Callbacks ----------------

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = MapData(
            msg,
            occ_threshold=int(self.get_parameter("occ_threshold").value),
        )
        if not hasattr(self, '_map_logged'):
            self.get_logger().info(
                f"Map received: {msg.info.width}x{msg.info.height}, res={msg.info.resolution}"
            )
            self._map_logged = True

    def timer_callback(self):
        if self.map_data is None:
            return
        if self.has_planned:
            return

        sx = 0.0
        sy = 0.0
        gx = self.get_parameter("goal_x").value
        gy = self.get_parameter("goal_y").value
        start = (sx, sy)
        goal = (gx, gy)

        self.get_logger().info(f"Planning straight line from {start} → {goal}")

        if not self.plan_and_publish(start, goal):
            self.get_logger().warn("Could not compute a collision-free straight line path.")

        self.has_planned = True

    # ---------------- Planning & Publishing ----------------

    def plan_and_publish(self, start, goal) -> bool:
        sx, sy = start
        gx, gy = goal

        m = self.map_data

        # Collision checks with logging of sampled occupancy
        start_val = m.cell_value(sx, sy)
        goal_val = m.cell_value(gx, gy)
        if start_val is None:
            self.get_logger().warn("Start is out of map bounds.")
            return False
        if goal_val is None:
            self.get_logger().warn("Goal is out of map bounds.")
            return False
        if start_val < 0:
            self.get_logger().warn(f"Start is unknown (val={start_val}).")
            return False
        if goal_val < 0:
            self.get_logger().warn(f"Goal is unknown (val={goal_val}).")
            return False
        if start_val < m.occ_threshold:
            self.get_logger().warn(f"Start is in an obstacle (val={start_val}, threshold={m.occ_threshold}).")
            return False
        if goal_val < m.occ_threshold:
            self.get_logger().warn(f"Goal is in an obstacle (val={goal_val}, threshold={m.occ_threshold}).")
            return False

        if not m.is_segment_free(sx, sy, gx, gy):
            self.get_logger().warn("Straight segment intersects obstacle.")
            return False

        # Create smooth line with sampled points
        dist = math.hypot(gx - sx, gy - sy)
        step = m.res * 2.0
        n = max(int(dist / step), 2)

        points = []
        for i in range(n + 1):
            t = i / n
            x = sx + t * (gx - sx)
            y = sy + t * (gy - sy)
            points.append((x, y))

        self.publish_path(points)
        return True

    def publish_path(self, pts: List[tuple]):
        msg = Path()
        msg.header.frame_id = "map"
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
        self.get_logger().info(f"Published straight path with {len(pts)} points.")


def main(args=None):
    rclpy.init(args=args)
    node = StraightLinePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
