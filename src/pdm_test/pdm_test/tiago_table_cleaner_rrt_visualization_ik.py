#!/usr/bin/env python3
from __future__ import annotations

import time
import json
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from builtin_interfaces.msg import Duration as DurationMsg

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String

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

        # Callback group (IK/FK + services + timers)
        self.cb_group = ReentrantCallbackGroup()

        # Publishers
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )

        # RViz marker topic
        self.marker_pub = self.create_publisher(
            MarkerArray, "/rrt_debug_markers", 10
        )

        # Metrics topic (JSON)
        self.metrics_pub = self.create_publisher(
            String, "/arm_planner_metrics", 10
        )

        # Cumulative success stats across /clean_table calls
        self._runs_total = 0
        self._runs_success = 0

        self.down_orientation = None

        # Joint state subscriber 
        self._latest_joint_state: Optional[JointState] = None
        self.create_subscription(
            JointState, "/joint_states", self._js_cb, 10, callback_group=self.cb_group
        )

        # MoveIt service clients
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=self.cb_group
        )
        self.fk_client = self.create_client(
            GetPositionFK, "/compute_fk", callback_group=self.cb_group
        )

        # Service trigger
        self.create_service(
            Trigger, "/clean_table", self._srv_clean_table, callback_group=self.cb_group
        )

        # MoveIt config 
        self.moveit_group_name = "arm"
        self.moveit_ee_link = "gripper_tool_link"
        self.planning_frame = "base_link"

        # Poses
        self.preclean_q: List[float] = [0.25, 1.0, 0.0, 2.2, -1.2, 0.7, 0.5]
        self.rest_q: List[float] = [0.50, -1.34, -0.48, 1.94, -1.49, 1.37, 0.00]

        # Cartesian waypoints (with respect to base_link) - zig-zag
        self.cartesian_waypoints: List[Tuple[float, float, float]] = [
            (0.5, -0.2, 0.7),
            (0.75, -0.1, 0.7),
            (0.5,  0.0, 0.7),
            (0.75,  0.1, 0.7),
            (0.5,  0.2, 0.7),
        ]

        # Table collision box (base_link frame)
        self.table_box = {
            "x": (0.5, 1.1),
            "y": (-0.3, 0.3),
            "z": (0.45, 0.6),
        }

        # Pipeline flags/state
        self._pipeline_running = False
        self._trigger_requested = False

        # Async IK chain state
        self._ik_index: int = 0
        self._ik_joint_waypoints: List[List[float]] = []
        self._q_start: Optional[List[float]] = None

        # Deterministic RRT visualization (joint-space edges)
        self._rrt_edges_q: List[Tuple[List[float], List[float]]] = []

        
        self._fk_cache: Dict[str, Point] = {}

        # Planner (RRT-Connect) with edge callback
        self.planner = RRTConnect(
            is_state_valid=self.is_state_valid,
            is_edge_valid=self.is_edge_valid,
            step_size=0.2,
            max_iters=2000,
            goal_bias=0.5,
            on_add_edge=self._on_rrt_edge_added,
        )

        # Timer to check trigger
        self.create_timer(
            0.1, self._timer_check_trigger, callback_group=self.cb_group
        )

        
        self._wait_for_services()
        self._init_down_orientation_from_preclean()

        self.get_logger().info(
            "READY. RViz: add MarkerArray topic /rrt_debug_markers, Fixed Frame base_link. "
            "Call /clean_table to run."
        )

    
    def _reset_pipeline(self) -> None:
        self._pipeline_running = False
        self._ik_index = 0
        self._ik_joint_waypoints = []
        self._rrt_edges_q = []
        self._fk_cache = {}
        self.get_logger().info("Pipeline state reset; ready for next /clean_table.")

    # Metrics helpers 
    
    def _path_length_jointspace(self, path: List[List[float]]) -> float:
        
        if not path or len(path) < 2:
            return 0.0
        total = 0.0
        for a, b in zip(path[:-1], path[1:]):
            total += self.planner._dist(a, b)
        return float(total)

    def _straight_line_distance_jointspace(self, q_start: List[float], q_goal: List[float]) -> float:
        
        return float(self.planner._dist(q_start, q_goal))

    def _smoothness_second_difference(self, path: List[List[float]]) -> float:
        
        if not path or len(path) < 3:
            return 0.0
        total = 0.0
        for qm1, q0, qp1 in zip(path[:-2], path[1:-1], path[2:]):
            dd = [(a - 2.0 * b + c) for a, b, c in zip(qp1, q0, qm1)]
            total += (sum(x * x for x in dd) ** 0.5)
        return float(total)

    def _publish_metrics(
        self,
        success: bool,
        planning_time_s: float,
        nodes_added: int,
        path_length: float,
        straight_line_distance: float,
        extra_length: float,
        detour_ratio: float,
        smoothness: float,
        segments_planned: int,
        segments_total: int,
    ) -> None:
        
        if success:
            self._runs_success += 1

        msg = {
            "timestamp_ns": int(self.get_clock().now().nanoseconds),
            "success": bool(success),
            "planning_time_s": float(planning_time_s),
            "nodes_added": int(nodes_added),
            "path_length": float(path_length),
            "straight_line_distance": float(straight_line_distance),
            "extra_length": float(extra_length),
            "detour_ratio": float(detour_ratio),
            "smoothness": float(smoothness),
            "segments_planned": int(segments_planned),
            "segments_total": int(segments_total),
            "runs_total": int(self._runs_total),
            "runs_success": int(self._runs_success),
            "success_rate": float(self._runs_success / self._runs_total) if self._runs_total > 0 else 0.0,
        }

        out = String()
        out.data = json.dumps(msg)
        self.metrics_pub.publish(out)

        self.get_logger().info(
            f"[ARM_METRICS] success={success} time={planning_time_s:.3f}s "
            f"nodes={nodes_added} pathLen={path_length:.3f} straight={straight_line_distance:.3f} "
            f"extra={extra_length:.3f} ratio={detour_ratio:.3f} smooth={smoothness:.3f} "
            f"segments={segments_planned}/{segments_total} "
            f"cumSuccess={self._runs_success}/{self._runs_total} "
            f"rate={msg['success_rate']:.3f}"
        )

    # ROS plumbing 

    def _init_down_orientation_from_preclean(self) -> None:
        
        req = GetPositionFK.Request()
        req.header.frame_id = self.planning_frame
        req.fk_link_names = [self.moveit_ee_link]

        js = JointState()
        js.name = ARM_JOINTS
        js.position = self.preclean_q
        rs = RobotState()
        rs.joint_state = js
        req.robot_state = rs

        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if not future.done():
            self.get_logger().error("FK for preclean_q timed out; using identity orientation.")
            return

        res = future.result()
        if res is None or res.error_code.val != res.error_code.SUCCESS or not res.pose_stamped:
            self.get_logger().error("FK for preclean_q failed; using identity orientation.")
            return

        pose = res.pose_stamped[0].pose
        self.down_orientation = pose.orientation
        self.get_logger().info(
            f"Initialized down_orientation from preclean_q: "
            f"({self.down_orientation.x:.3f}, {self.down_orientation.y:.3f}, "
            f"{self.down_orientation.z:.3f}, {self.down_orientation.w:.3f})"
        )

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

    # Joint helpers 

    def get_current_q(self) -> Optional[List[float]]:
        if self._latest_joint_state is None:
            return None
        name_to_pos = dict(zip(self._latest_joint_state.name, self._latest_joint_state.position))
        try:
            return [float(name_to_pos[j]) for j in ARM_JOINTS]
        except KeyError:
            return None

    def is_state_valid(self, q: List[float]) -> bool:
        if not all(lo <= qi <= hi for qi, (lo, hi) in zip(q, JOINT_LIMITS)):
            return False

        p = self._fk_point_sync(q)
        if p is None:
            return False

        xb, yb, zb = self.table_box["x"], self.table_box["y"], self.table_box["z"]
        if xb[0] <= p.x <= xb[1] and yb[0] <= p.y <= yb[1] and zb[0] <= p.z <= zb[1]:
            return False

        return True

    def is_edge_valid(self, q1: List[float], q2: List[float]) -> bool:
        samples = 10
        for i in range(samples + 1):
            alpha = i / samples
            q = [(1 - alpha) * a + alpha * b for a, b in zip(q1, q2)]
            if not self.is_state_valid(q):
                return False
        return True

    # Pipeline (IK -> RRT -> publish)
    def _start_pipeline(self) -> None:
        self._pipeline_running = True
        self.get_logger().info("Starting pipeline...")

        self._q_start = self.get_current_q()
        if self._q_start is None:
            self.get_logger().error("No joint state. Aborting.")
            self._runs_total += 1
            self._publish_metrics(
                success=False,
                planning_time_s=0.0,
                nodes_added=0,
                path_length=0.0,
                straight_line_distance=0.0,
                extra_length=0.0,
                detour_ratio=0.0,
                smoothness=0.0,
                segments_planned=0,
                segments_total=0,
            )
            self._reset_pipeline()
            return

        if not (self.ik_client.service_is_ready() and self.fk_client.service_is_ready()):
            self.get_logger().error("IK/FK services not ready. Aborting.")
            self._runs_total += 1
            self._publish_metrics(
                success=False,
                planning_time_s=0.0,
                nodes_added=0,
                path_length=0.0,
                straight_line_distance=0.0,
                extra_length=0.0,
                detour_ratio=0.0,
                smoothness=0.0,
                segments_planned=0,
                segments_total=0,
            )
            self._reset_pipeline()
            return

        self._clear_markers()

        self._ik_joint_waypoints = []
        self._ik_index = 0
        self._rrt_edges_q = []
        self._fk_cache = {}

        self._call_ik_for_current_waypoint()

    # IK chain

    def _call_ik_for_current_waypoint(self) -> None:
        if self._ik_index >= len(self.cartesian_waypoints):
            if not self._ik_joint_waypoints:
                self.get_logger().error("All IK waypoints failed. Resetting pipeline.")
                self._runs_total += 1
                self._publish_metrics(
                    success=False,
                    planning_time_s=0.0,
                    nodes_added=0,
                    path_length=0.0,
                    straight_line_distance=0.0,
                    extra_length=0.0,
                    detour_ratio=0.0,
                    smoothness=0.0,
                    segments_planned=0,
                    segments_total=0,
                )
                self._reset_pipeline()
                return

            self.get_logger().info("All IK waypoints solved. Starting RRT planning...")
            self._run_rrt_and_publish()
            return

        x, y, z = self.cartesian_waypoints[self._ik_index]
        self.get_logger().info(
            f"Calling IK waypoint {self._ik_index+1}: ({x:.3f}, {y:.3f}, {z:.3f})"
        )

        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        if self.down_orientation is not None:
            pose.pose.orientation = self.down_orientation
        else:
            pose.pose.orientation.w = 1.0

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.moveit_group_name
        req.ik_request.pose_stamped = pose
        req.ik_request.ik_link_name = self.moveit_ee_link

        if self._ik_joint_waypoints:
            seed_q = self._ik_joint_waypoints[-1]
        else:
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
                self._runs_total += 1
                self._publish_metrics(
                    success=False,
                    planning_time_s=0.0,
                    nodes_added=0,
                    path_length=0.0,
                    straight_line_distance=0.0,
                    extra_length=0.0,
                    detour_ratio=0.0,
                    smoothness=0.0,
                    segments_planned=0,
                    segments_total=0,
                )
                self._reset_pipeline()
                return

            if res is None or res.error_code.val != res.error_code.SUCCESS:
                code = None if res is None else res.error_code.val
                self.get_logger().warn(
                    f"IK failed at waypoint {index+1}, error_code={code}. Skipping this waypoint."
                )
                self._ik_index += 1
                self._call_ik_for_current_waypoint()
                return

            js = res.solution.joint_state
            name_to_pos = dict(zip(js.name, js.position))
            try:
                q = [float(name_to_pos[j]) for j in ARM_JOINTS]
            except KeyError as e:
                self.get_logger().error(f"IK missing joint {e} at waypoint {index+1}")
                self._runs_total += 1
                self._publish_metrics(
                    success=False,
                    planning_time_s=0.0,
                    nodes_added=0,
                    path_length=0.0,
                    straight_line_distance=0.0,
                    extra_length=0.0,
                    detour_ratio=0.0,
                    smoothness=0.0,
                    segments_planned=0,
                    segments_total=0,
                )
                self._reset_pipeline()
                return

            self._ik_joint_waypoints.append(q)
            self.get_logger().info(f"IK OK waypoint {index+1}")

            self._ik_index += 1
            self._call_ik_for_current_waypoint()

        return _cb

    #  RRT & Trajectory publishing

    def _run_rrt_and_publish(self) -> None:
        if self._q_start is None:
            self._reset_pipeline()
            return

        self._runs_total += 1

        if not self._ik_joint_waypoints:
            self.get_logger().error("No valid IK waypoints. Resetting pipeline.")
            self._publish_metrics(
                success=False,
                planning_time_s=0.0,
                nodes_added=0,
                path_length=0.0,
                straight_line_distance=0.0,
                extra_length=0.0,
                detour_ratio=0.0,
                smoothness=0.0,
                segments_planned=0,
                segments_total=0,
            )
            self._reset_pipeline()
            return

        joint_targets: List[List[float]] = [self.preclean_q]
        joint_targets.extend(self._ik_joint_waypoints)
        joint_targets.append(self.preclean_q)
        joint_targets.append(self.rest_q)

        segments_total = len(joint_targets)
        segments_planned = 0

        total_nodes_added = 0
        total_path_length = 0.0
        total_straight = 0.0
        total_smoothness = 0.0

        t0 = time.perf_counter()

        full_path: List[List[float]] = []
        current = self._q_start

        for seg_idx, goal in enumerate(joint_targets, start=1):
            self.get_logger().info(f"RRT segment {seg_idx}/{segments_total}...")

            seg_straight = self._straight_line_distance_jointspace(current, goal)
            seg_path = self.planner.plan(current, goal)

            total_nodes_added += int(self.planner.stats.get("nodes_added", 0))

            if not seg_path:
                planning_time = time.perf_counter() - t0

                extra_length = total_path_length - total_straight
                detour_ratio = (total_path_length / total_straight) if total_straight > 1e-9 else 0.0

                self.get_logger().error(f"RRT failed at segment {seg_idx}. Resetting pipeline.")
                self._publish_metrics(
                    success=False,
                    planning_time_s=planning_time,
                    nodes_added=total_nodes_added,
                    path_length=total_path_length,
                    straight_line_distance=total_straight,
                    extra_length=extra_length,
                    detour_ratio=detour_ratio,
                    smoothness=total_smoothness,
                    segments_planned=segments_planned,
                    segments_total=segments_total,
                )
                self._reset_pipeline()
                return

            segments_planned += 1

            total_straight += seg_straight
            total_path_length += self._path_length_jointspace(seg_path)
            total_smoothness += self._smoothness_second_difference(seg_path)

            if full_path:
                full_path.extend(seg_path[1:])
            else:
                full_path.extend(seg_path)

            current = goal

        planning_time = time.perf_counter() - t0

        extra_length = total_path_length - total_straight
        detour_ratio = (total_path_length / total_straight) if total_straight > 1e-9 else 0.0

        self._publish_metrics(
            success=True,
            planning_time_s=planning_time,
            nodes_added=total_nodes_added,
            path_length=total_path_length,
            straight_line_distance=total_straight,
            extra_length=extra_length,
            detour_ratio=detour_ratio,
            smoothness=total_smoothness,
            segments_planned=segments_planned,
            segments_total=segments_total,
        )

        self.get_logger().info(
            f"RRT complete. Joint states: {len(full_path)}. Collected edges: {len(self._rrt_edges_q)}"
        )

        self._publish_rrt_markers_from_edges(self._rrt_edges_q, max_edges=800)

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        t = 0.0
        for q in full_path:
            t += 0.3
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.time_from_start = DurationMsg(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            traj.points.append(pt)

        self.arm_pub.publish(traj)
        self.get_logger().info("Trajectory published.")
        self._reset_pipeline()

    # RRT edge callback

    def _on_rrt_edge_added(self, q_from, q_to) -> None:
        self._rrt_edges_q.append((list(q_from), list(q_to)))

    # FK & visualization

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

    def _publish_rrt_markers_from_edges(
        self,
        edges_q: List[Tuple[List[float], List[float]]],
        max_edges: int = 800,
    ) -> None:
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
        self.get_logger().info(
            f"Published RRT markers: nodes={len(node_pts)}, edge_points={len(edge_pts)}"
        )

    # Marker helpers 

    def _clear_markers(self) -> None:
        arr = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        m.header.frame_id = self.planning_frame
        m.header.stamp = self.get_clock().now().to_msg()
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

