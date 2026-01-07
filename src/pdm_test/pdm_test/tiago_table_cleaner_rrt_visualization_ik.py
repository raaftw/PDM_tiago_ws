#!/usr/bin/env python3
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration as DurationMsg

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState

from .rrt_connect_2 import RRTConnect
from .collision_checker import JOINT_LIMITS


ARM_JOINTS = [
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
]


class TiagoTableCleanerRRTVisualizationIK(Node):
    def __init__(self):
        super().__init__("tiago_table_cleaner_rrt_visualization_ik")

        # --- Publishers ---
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )

        # RViz marker topic
        self.marker_pub = self.create_publisher(
            MarkerArray, "/rrt_debug_markers", 10
        )

        # --- Joint state subscriber ---
        self._latest_joint_state: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        # --- MoveIt service clients ---
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")

        # --- Service trigger ---
        self.create_service(Trigger, "/clean_table", self._srv_clean_table)

        # --- MoveIt config (matches your working CLI IK/FK tests) ---
        self.moveit_group_name = "arm"
        self.moveit_ee_link = "gripper_tool_link" #"arm_7_link" #"arm_tool_link"
        self.planning_frame = "base_link"

        # --- Poses ---
        self.preclean_q: List[float] = [1.0, 1.0, 0.0, 2.2, -1.2, 0.7, 0.5]
        self.rest_q: List[float] = [0.50, -1.34, -0.48, 1.94, -1.49, 1.37, 0.00]

        # Cartesian waypoints (base_link)
        self.wipe_z = 0.51        # 1 cm above tabletop (0.50)
        self.edge_margin = 0.05   # 5 cm inside edges
        '''
        x0, x1 = 0.50 + self.edge_margin, 1.00 - self.edge_margin
        y0, y1 = -0.25 + self.edge_margin, 0.25 - self.edge_margin
        z = self.wipe_z

        # One square loop (closed)
        self.cartesian_waypoints = [
            (x0, y0, z),
            (x1, y0, z),
            (x1, y1, z),
            (x0, y1, z),
            (x0, y0, z),
        ]
        '''
        self.cartesian_waypoints: List[Tuple[float, float, float]] = [
            (0.6, -0.1, 0.75),
            (0.8, -0.1, 0.72),
            (0.8, 0.2, 0.72),
            (0.6, 0.2, 0.72),
            (0.6, -0.1, 0.75),
        ]
        '''
        self.cartesian_waypoints: List[Tuple[float, float, float]] = [
            (0.6, 0.85, 0.0),
            (0.7, 0.82, 0.0),
            (0.7, 0.82, 0.2),
            (0.6, 0.82, 0.2),
            (0.6, 0.82, 0.0),
        ]
        '''
        # --- Pipeline flags/state ---
        self._pipeline_running = False
        self._trigger_requested = False

        # Async IK chain state
        self._ik_index: int = 0
        self._ik_joint_waypoints: List[List[float]] = []
        self._q_start: Optional[List[float]] = None

        # --- Deterministic RRT visualization storage (joint-space edges) ---
        self._rrt_edges_q: List[Tuple[List[float], List[float]]] = []

        # FK cache (helps a bit during drawing)
        self._fk_cache: Dict[str, Point] = {}

        # --- Planner (RRT-Connect) with edge callback ---
        self.planner = RRTConnect(
            is_state_valid=self.is_state_valid,
            is_edge_valid=self.is_edge_valid,
            step_size=0.2,
            max_iters=2000,
            goal_bias=0.5,
            on_add_edge=self._on_rrt_edge_added,   # rrt_connect_2.py supports this :contentReference[oaicite:1]{index=1}
        )

        # Timer to check trigger
        self.create_timer(0.1, self._timer_check_trigger)

        # Wait for services
        self._wait_for_services()

        self.get_logger().info(
            "READY. RViz: add MarkerArray topic /rrt_debug_markers, Fixed Frame base_link. "
            "Call /clean_table to run."
        )

    # ------------------- ROS plumbing -------------------

    def _wait_for_services(self) -> None:
        self.get_logger().info("Waiting for /compute_ik ...")
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/compute_ik not available")

        self.get_logger().info("Waiting for /compute_fk ...")
        if not self.fk_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/compute_fk not available")

    def _js_cb(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _srv_clean_table(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if self._pipeline_running:
            res.success = False
            res.message = "Pipeline already running."
            return res

        self._trigger_requested = True
        res.success = True
        res.message = "Triggered."
        self.get_logger().info("Received /clean_table trigger.")
        return res

    def _timer_check_trigger(self) -> None:
        if self._trigger_requested and not self._pipeline_running:
            self._trigger_requested = False
            self._start_pipeline()

    # ------------------- Joint helpers -------------------

    def get_current_q(self) -> Optional[List[float]]:
        if self._latest_joint_state is None:
            return None
        name_to_pos = dict(zip(self._latest_joint_state.name, self._latest_joint_state.position))
        try:
            return [float(name_to_pos[j]) for j in ARM_JOINTS]
        except KeyError:
            return None

    def is_state_valid(self, q: List[float]) -> bool:
        return all(lo <= qi <= hi for qi, (lo, hi) in zip(q, JOINT_LIMITS))

    def is_edge_valid(self, q1: List[float], q2: List[float]) -> bool:
        if not (self.is_state_valid(q1) and self.is_state_valid(q2)):
            return False
        mid = [(a + b) * 0.5 for a, b in zip(q1, q2)]
        return self.is_state_valid(mid)

    # ------------------- Pipeline (async IK -> RRT -> publish) -------------------

    def _start_pipeline(self) -> None:
        self._pipeline_running = True
        self.get_logger().info("Starting pipeline...")

        self._q_start = self.get_current_q()
        if self._q_start is None:
            self.get_logger().error("No joint state. Aborting.")
            self._pipeline_running = False
            return

        if not (self.ik_client.service_is_ready() and self.fk_client.service_is_ready()):
            self.get_logger().error("IK/FK services not ready. Aborting.")
            self._pipeline_running = False
            return

        self._clear_markers()
        self._publish_heartbeat_marker()

        # reset IK and RRT debug storage
        self._ik_joint_waypoints = []
        self._ik_index = 0
        self._rrt_edges_q = []
        self._fk_cache = {}

        self._call_ik_for_current_waypoint()

    # ------------------- IK chain (callback-based) -------------------

    def _call_ik_for_current_waypoint(self) -> None:
        if self._ik_index >= len(self.cartesian_waypoints):
            self.get_logger().info("All IK waypoints solved. Starting RRT planning...")
            self._run_rrt_and_publish()
            return

        x, y, z = self.cartesian_waypoints[self._ik_index]
        self.get_logger().info(f"Calling IK waypoint {self._ik_index+1}: ({x:.3f}, {y:.3f}, {z:.3f})")

        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.moveit_group_name
        req.ik_request.pose_stamped = pose
        req.ik_request.ik_link_name = self.moveit_ee_link

        # seed
        seed_q = self.get_current_q()
        if seed_q is not None:
            seed_js = JointState()
            seed_js.name = ARM_JOINTS
            seed_js.position = seed_q
            rs = RobotState()
            rs.joint_state = seed_js
            req.ik_request.robot_state = rs

        future = self.ik_client.call_async(req)
        future.add_done_callback(self._make_ik_done_cb(self._ik_index))

    def _make_ik_done_cb(self, index: int):
        def _cb(future):
            try:
                res = future.result()
            except Exception as e:
                self.get_logger().error(f"IK exception at waypoint {index+1}: {e}")
                self._pipeline_running = False
                return

            if res is None or res.error_code.val != res.error_code.SUCCESS:
                code = None if res is None else res.error_code.val
                self.get_logger().error(f"IK failed at waypoint {index+1}, error_code={code}")
                self._pipeline_running = False
                return

            js = res.solution.joint_state
            name_to_pos = dict(zip(js.name, js.position))
            try:
                q = [float(name_to_pos[j]) for j in ARM_JOINTS]
            except KeyError as e:
                self.get_logger().error(f"IK missing joint {e} at waypoint {index+1}")
                self._pipeline_running = False
                return

            self._ik_joint_waypoints.append(q)
            self.get_logger().info(f"IK OK waypoint {index+1}")

            self._ik_index += 1
            self._call_ik_for_current_waypoint()

        return _cb

    # ------------------- RRT + Trajectory publishing -------------------

    def _run_rrt_and_publish(self) -> None:
        if self._q_start is None:
            self._pipeline_running = False
            return

        joint_targets: List[List[float]] = [self.preclean_q]
        joint_targets.extend(self._ik_joint_waypoints)
        joint_targets.append(self.preclean_q)
        joint_targets.append(self.rest_q)

        full_path: List[List[float]] = []
        current = self._q_start

        for seg_idx, goal in enumerate(joint_targets, start=1):
            self.get_logger().info(f"RRT segment {seg_idx}/{len(joint_targets)}...")
            path = self.planner.plan(current, goal)
            if not path:
                self.get_logger().error(f"RRT failed at segment {seg_idx}.")
                self._pipeline_running = False
                return

            if full_path:
                full_path.extend(path[1:])
            else:
                full_path.extend(path)

            current = goal

        self.get_logger().info(
            f"RRT complete. Joint states: {len(full_path)}. Collected edges: {len(self._rrt_edges_q)}"
        )

        # Deterministic visualization (no async FK dependency)
        self._publish_rrt_markers_from_edges(self._rrt_edges_q, max_edges=800)

        # publish trajectory
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        t = 0.0
        for q in full_path:
            t += 0.6
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.time_from_start = DurationMsg(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            traj.points.append(pt)

        self.arm_pub.publish(traj)
        self.get_logger().info("Trajectory published.")
        self._pipeline_running = False

    # ------------------- RRT edge callback (store only, super reliable) -------------------

    def _on_rrt_edge_added(self, q_from, q_to) -> None:
        # store edges in joint space, visualize later
        self._rrt_edges_q.append((list(q_from), list(q_to)))

    # ------------------- FK + visualization (sync, after planning) -------------------

    def _q_key(self, q: List[float]) -> str:
        return ",".join(f"{v:.4f}" for v in q)

    def _fk_point_sync(self, q: List[float]) -> Optional[Point]:
        k = self._q_key(q)
        if k in self._fk_cache:
            return self._fk_cache[k]

        req = GetPositionFK.Request()
        req.header.frame_id = self.planning_frame
        req.fk_link_names = [self.moveit_ee_link]

        js = JointState()
        js.name = ARM_JOINTS
        js.position = q
        rs = RobotState()
        rs.joint_state = js
        req.robot_state = rs

        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.6)
        if not future.done():
            return None

        res = future.result()
        if res is None or res.error_code.val != res.error_code.SUCCESS or not res.pose_stamped:
            return None

        pose = res.pose_stamped[0].pose
        p = Point(x=pose.position.x, y=pose.position.y, z=pose.position.z)
        self._fk_cache[k] = p
        return p

    def _publish_rrt_markers_from_edges(self, edges_q: List[Tuple[List[float], List[float]]], max_edges: int = 800) -> None:
        # Limit edges so RViz doesn't get flooded
        edges_q = edges_q[:max_edges]

        node_pts: List[Point] = []
        edge_pts: List[Point] = []

        for qf, qt in edges_q:
            pf = self._fk_point_sync(qf)
            pt = self._fk_point_sync(qt)
            if pf is None or pt is None:
                continue
            node_pts.append(pt)
            edge_pts.append(pf)
            edge_pts.append(pt)

        arr = MarkerArray()

        nodes = Marker()
        nodes.header.frame_id = self.planning_frame
        nodes.header.stamp = self.get_clock().now().to_msg()
        nodes.ns = "rrt_nodes"
        nodes.id = 0
        nodes.type = Marker.SPHERE_LIST
        nodes.action = Marker.ADD
        nodes.scale.x = 0.02
        nodes.scale.y = 0.02
        nodes.scale.z = 0.02
        nodes.color = ColorRGBA(r=0.1, g=0.6, b=1.0, a=0.9)
        nodes.points = node_pts
        arr.markers.append(nodes)

        edges = Marker()
        edges.header.frame_id = self.planning_frame
        edges.header.stamp = self.get_clock().now().to_msg()
        edges.ns = "rrt_edges"
        edges.id = 1
        edges.type = Marker.LINE_LIST
        edges.action = Marker.ADD
        edges.scale.x = 0.005
        edges.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
        edges.points = edge_pts
        arr.markers.append(edges)

        self.marker_pub.publish(arr)
        self.get_logger().info(f"Published RRT markers: nodes={len(node_pts)}, edge_points={len(edge_pts)}")

    # ------------------- Marker helpers -------------------

    def _clear_markers(self) -> None:
        arr = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        m.header.frame_id = self.planning_frame
        m.header.stamp = self.get_clock().now().to_msg()
        arr.markers.append(m)
        self.marker_pub.publish(arr)

    def _publish_heartbeat_marker(self) -> None:
        """A marker that ALWAYS shows if RViz is configured correctly."""
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.planning_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "debug"
        m.id = 999
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = 0.6
        m.pose.position.y = 0.0
        m.pose.position.z = 0.85
        m.pose.orientation.w = 1.0
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.scale.z = 0.05
        m.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)  # magenta
        arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = TiagoTableCleanerRRTVisualizationIK()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

