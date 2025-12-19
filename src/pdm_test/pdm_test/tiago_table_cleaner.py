#!/usr/bin/env python3
"""
Upgraded TiagoTableCleaner:
- Service /clean_table executes: current → pre-clean → wipe raster → pre-clean → rest
- Uses RRT-Connect for all joint-space transitions
- Uses MoveIt IK for wipe waypoints
- FK clearance enforced on edges (throttled sampling)
"""

from __future__ import annotations
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

from moveit_msgs.srv import GetPositionFK, GetPositionIK

from .rrt_connect import RRTConnect
from .collision_checker import JOINT_LIMITS, TABLE

ARM_JOINTS = [
    "arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint",
    "arm_5_joint","arm_6_joint","arm_7_joint",
]
EEF_LINK = "gripper_tool_link"
PLANNING_GROUP = "arm"

class TiagoTableCleaner(Node):

    def __init__(self):
        super().__init__("tiago_table_cleaner")

        # Publishers
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.marker_pub = self.create_publisher(Marker, "eef_path", 10)

        # Joint state subscriber
        self._latest_joint_state: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        # Services
        self.create_service(Trigger, "/clean_table", self._srv_clean_table)

        # MoveIt FK/IK clients
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /compute_fk...")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /compute_ik...")

        # Planner: lightweight state validity, FK clearance on edges
        self.planner = RRTConnect(
            is_state_valid=self.is_state_valid,
            is_edge_valid=self.is_edge_valid,
            step_size=0.2,
            max_iters=2000,
            goal_bias=0.5,
        )

        # Key poses
        self.rest_q = [0.50, -1.34, -0.48, 1.94, -1.49, 1.37, 0.0]
        self.preclean_q = [1.2, 1.0, 0.0, 2.2, -1.2, 0.7, 0.5]

        self.get_logger().info("TiagoTableCleaner ready.")

    # -------------------------
    # Callbacks & state
    # -------------------------
    def _js_cb(self, msg: JointState):
        self._latest_joint_state = msg

    def get_current_q(self) -> Optional[List[float]]:
        if self._latest_joint_state is None:
            return None
        name_to_pos = dict(zip(self._latest_joint_state.name, self._latest_joint_state.position))
        try:
            return [float(name_to_pos[j]) for j in ARM_JOINTS]
        except KeyError:
            return None

    # -------------------------
    # MoveIt FK/IK
    # -------------------------
    def fk_gripper(self, q: List[float]) -> Tuple[float, float, float]:
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = [EEF_LINK]
        js = JointState()
        js.name = ARM_JOINTS
        js.position = q
        req.robot_state.joint_state = js
        future = self.fk_client.call_async(req)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
        result = future.result()
        if not result or not result.pose_stamped:
            raise RuntimeError("FK failed")
        p = result.pose_stamped[0].pose.position
        return p.x, p.y, p.z

    def ik_gripper(self, target_xyz: Tuple[float, float, float], q_seed: Optional[List[float]]) -> Optional[List[float]]:
        req = GetPositionIK.Request()
        req.ik_request.group_name = PLANNING_GROUP
        req.ik_request.ik_link_name = EEF_LINK
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position.x = target_xyz[0]
        req.ik_request.pose_stamped.pose.position.y = target_xyz[1]
        req.ik_request.pose_stamped.pose.position.z = target_xyz[2]
        req.ik_request.pose_stamped.pose.orientation.w = 1.0
        req.ik_request.attempts = 5
        req.ik_request.timeout.nanosec = int(0.2e9)
        if q_seed:
            js = JointState()
            js.name = ARM_JOINTS
            js.position = q_seed
            req.ik_request.robot_state.joint_state = js
        future = self.ik_client.call_async(req)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
        result = future.result()
        if not result or not result.solution.joint_state.name:
            return None
        name_to_pos = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
        q = [float(name_to_pos[j]) for j in ARM_JOINTS]
        return q

    # -------------------------
    # Validity & edge checks
    # -------------------------
    def is_state_valid(self, q: List[float]) -> bool:
        for qi, (lo, hi) in zip(q, JOINT_LIMITS):
            if qi < lo or qi > hi:
                return False
        return True

    def is_edge_valid(self, q1: List[float], q2: List[float]) -> bool:
        # Throttled FK clearance: check only 3 samples along edge
        for t in [0.0, 0.5, 1.0]:
            q = [(1 - t) * a + t * b for a, b in zip(q1, q2)]
            try:
                _, _, z = self.fk_gripper(q)
            except Exception as e:
                self.get_logger().debug(f"FK error on edge: {e}")
                return False
            if z < (TABLE.top_z + TABLE.clearance):
                return False
        return True

    # -------------------------
    # Visualization
    # -------------------------
    def publish_marker(self, x, y, z, i):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "eef_path"
        m.id = i
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.scale.x = m.scale.y = m.scale.z = 0.02
        m.color.a = 1.0
        m.color.g = 1.0
        self.marker_pub.publish(m)

    # -------------------------
    # Wipe raster
    # -------------------------
    def make_wipe_raster(self) -> List[Tuple[float, float, float]]:
        z = TABLE.top_z + TABLE.clearance
        x0, x1 = TABLE.x_min + TABLE.inset, TABLE.x_max - TABLE.inset
        y0, y1 = TABLE.y_min + TABLE.inset, TABLE.y_max - TABLE.inset
        lines = 4
        waypoints: List[Tuple[float, float, float]] = []
        for k in range(lines):
            y = y0 + (y1 - y0) * (k / (lines - 1))
            if k % 2 == 0:
                waypoints.append((x0, y, z))
                waypoints.append((x1, y, z))
            else:
                waypoints.append((x1, y, z))
                waypoints.append((x0, y, z))
        return waypoints

    # -------------------------
    # Service
    # -------------------------
    def _srv_clean_table(self, _, res):
        self.get_logger().info("Entered /clean_table pipeline")
        q_start = self.get_current_q()
        if q_start is None:
            res.success = False
            res.message = "No joint state"
            return res

        # 1) Current → pre-clean
        path1 = self.planner.plan(q_start, self.preclean_q)
        if not path1:
            res.success = False
            res.message = "Planning to pre-clean failed"
            return res

        # 2) Wipe raster
    # -------------------------
    # Service
    # -------------------------
    def _srv_clean_table(self, _, res):
        self.get_logger().info("Entered /clean_table pipeline")
        q_start = self.get_current_q()
        if q_start is None:
            res.success = False
            res.message = "No joint state"
            return res

        # 1) Current → pre-clean
        path1 = self.planner.plan(q_start, self.preclean_q)
        if not path1:
            res.success = False
            res.message = "Planning to pre-clean failed"
            return res

        # 2) Wipe raster
        raster = self.make_wipe_raster()
        path2: List[List[float]] = []
        q_curr = path1[-1]
        i_marker = 0
        for wp in raster:
            q_goal = self.ik_gripper(wp, q_seed=q_curr)
            if q_goal is None:
                res.success = False
                res.message = "IK failed for wipe waypoint"
                return res
            seg = self.planner.plan(q_curr, q_goal)
            if not seg:
                res.success = False
                res.message = "Planning failed inside wipe raster"
                return res
            path2 += seg[1:]  # avoid duplicate start
            q_curr = seg[-1]
            x, y, z = self.fk_gripper(q_curr)
            self.publish_marker(x, y, z, i_marker)
            i_marker += 1

        # 3) Back to pre-clean
        path3 = self.planner.plan(q_curr, self.preclean_q)
        if not path3:
            res.success = False
            res.message = "Planning back to pre-clean failed"
            return res

        # 4) Pre-clean → rest
        path4 = self.planner.plan(path3[-1], self.rest_q)
        if not path4:
            res.success = False
            res.message = "Planning to rest failed"
            return res

        # Build and publish trajectory
        full_path = path1 + path2 + path3[1:] + path4[1:]
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        t = 0.0
        for q in full_path:
            t += 0.6
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            traj.points.append(pt)

        self.get_logger().info(f"Publishing trajectory with {len(traj.points)} points")
        self.arm_pub.publish(traj)
        res.success = True
        res.message = "Pre-clean, wipe, pre-clean, rest executed"
        return res


def main():
    rclpy.init()
    node = TiagoTableCleaner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

