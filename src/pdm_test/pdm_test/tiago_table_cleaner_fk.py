#!/usr/bin/env python3
"""
Step-2 FK test node with hard FK timeouts, probe logging, and fallback
- Plans current → pre-clean
- Validity: joint limits + FK clearance (endpoint only if FK available)
- If FK times out, switches to joint-limits-only so you still get motion
"""

from __future__ import annotations
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionFK

from .rrt_connect import RRTConnect
from .collision_checker import JOINT_LIMITS

ARM_JOINTS = [
    "arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint",
    "arm_5_joint","arm_6_joint","arm_7_joint",
]

CANDIDATE_EEF_LINKS = [
    "gripper_tool_link",
    "arm_tool_link",
    "gripper_tip_link",
    "end_effector_link",
]

class TiagoTableCleanerFK(Node):
    def __init__(self):
        super().__init__("tiago_table_cleaner_fk")

        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self._latest_joint_state: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_service(Trigger, "/clean_table", self._srv_clean_table)

        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /compute_fk...")

        # Planner: we’ll set is_edge_valid dynamically (FK or no-FK)
        self.use_fk = False
        self.clearance_min_z: Optional[float] = None
        self.eef_link: Optional[str] = None

        self.planner = RRTConnect(
            is_state_valid=self.is_state_valid,
            is_edge_valid=self.is_edge_valid,
            step_size=0.2,
            max_iters=2000,
            goal_bias=0.5,
        )

        self.preclean_q = [1.2, 1.0, 0.0, 2.2, -1.2, 0.7, 0.5]

        self.get_logger().info("TiagoTableCleanerFK ready.")

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

    # ---------- FK helpers with hard timeout ----------
    def fk_once(self, q: List[float], link_name: str, timeout_sec: float = 0.5) -> Tuple[float, float, float]:
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = [link_name]
        js = JointState()
        js.name = ARM_JOINTS
        js.position = q
        req.robot_state.joint_state = js

        self.get_logger().info(f"FK call start: link={link_name}")
        future = self.fk_client.call_async(req)
        done = rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not done:
            raise TimeoutError(f"FK timeout for link {link_name} after {timeout_sec}s")
        result = future.result()
        if not result or not result.pose_stamped:
            raise RuntimeError(f"FK failed for link {link_name}")
        p = result.pose_stamped[0].pose.position
        self.get_logger().info(f"FK call end: link={link_name}, z={p.z:.3f}")
        return p.x, p.y, p.z

    def fk_gripper(self, q: List[float]) -> Tuple[float, float, float]:
        if not self.eef_link:
            raise RuntimeError("EEF link not selected yet")
        return self.fk_once(q, self.eef_link)

    # ---------- Validity ----------
    def is_state_valid(self, q: List[float]) -> bool:
        return all(lo <= qi <= hi for qi, (lo, hi) in zip(q, JOINT_LIMITS))

    def is_edge_valid(self, q1: List[float], q2: List[float]) -> bool:
        if not self.use_fk or self.clearance_min_z is None:
            return True  # no FK clearance: joint limits only
        # Endpoint-only FK clearance
        for t in (0.0, 1.0):
            q = [(1 - t) * a + t * b for a, b in zip(q1, q2)]
            try:
                _, _, z = self.fk_gripper(q)
            except Exception as e:
                self.get_logger().warn(f"FK error on edge: {e}")
                return False
            if z < self.clearance_min_z:
                return False
        return True

    # ---------- Service ----------
    def _srv_clean_table(self, _, res):
        self.get_logger().info("Entered /clean_table (FK clearance test)")
        q_start = self.get_current_q()
        if q_start is None:
            res.success = False
            res.message = "No joint state"
            return res

        # Try FK probes with hard timeout; if all fail, disable FK and proceed
        z_scores = []
        for link in CANDIDATE_EEF_LINKS:
            try:
                x, y, z = self.fk_once(self.preclean_q, link, timeout_sec=0.5)
                z_scores.append((z, link))
            except Exception as e:
                self.get_logger().warn(f"FK probe failed for {link}: {e}")

        if z_scores:
            z_scores.sort(reverse=True)
            self.eef_link = z_scores[0][1]
            self.use_fk = True
            self.get_logger().info(f"Selected EEF link: {self.eef_link} (z={z_scores[0][0]:.3f})")
            # Calibrate clearance: 3 cm below goal Z
            try:
                _, _, z_goal = self.fk_gripper(self.preclean_q)
                self.clearance_min_z = z_goal - 0.03
                self.get_logger().info(f"Calibrated clearance_min_z={self.clearance_min_z:.3f}")
            except Exception as e:
                self.get_logger().warn(f"FK failed during calibration: {e}; disabling FK")
                self.use_fk = False
                self.clearance_min_z = None
        else:
            self.get_logger().warn("All FK probes timed out/failed. Proceeding without FK clearance.")
            self.use_fk = False
            self.clearance_min_z = None

        # Plan current → pre-clean
        path = self.planner.plan(q_start, self.preclean_q)
        if not path:
            res.success = False
            res.message = "Planning to pre-clean failed"
            return res

        # Build and publish trajectory
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        t = 0.0
        for q in path:
            t += 0.6
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            traj.points.append(pt)

        self.get_logger().info(f"Publishing trajectory with {len(traj.points)} points")
        self.arm_pub.publish(traj)
        res.success = True
        res.message = "Moved to pre-clean pose"
        return res


def main():
    rclpy.init()
    node = TiagoTableCleanerFK()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

