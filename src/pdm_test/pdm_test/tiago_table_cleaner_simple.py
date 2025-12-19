#!/usr/bin/env python3
"""
Minimal test: plan from current joint state to pre-clean pose
Publishes trajectory directly to /arm_controller/joint_trajectory
"""

from __future__ import annotations
from typing import List, Optional

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .rrt_connect import RRTConnect
from .collision_checker import JOINT_LIMITS


ARM_JOINTS = [
    "arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint",
    "arm_5_joint","arm_6_joint","arm_7_joint",
]

class TiagoTableCleanerSimple(Node):

    def __init__(self):
        super().__init__("tiago_table_cleaner_simple")

        # Publisher
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

        # Joint state subscriber
        self._latest_joint_state: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        # Service
        self.create_service(Trigger, "/clean_table", self._srv_clean_table)

        # Planner (joint limits only for validity)
        self.planner = RRTConnect(
            is_state_valid=self.is_state_valid,
            is_edge_valid=lambda q1,q2: True,  # skip FK clearance for speed
            step_size=0.2,
            max_iters=2000,
            goal_bias=0.5,
        )

        # Pre-clean pose
        self.preclean_q = [1.2, 1.0, 0.0, 2.2, -1.2, 0.7, 0.5]

        self.get_logger().info("TiagoTableCleanerSimple ready.")

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

    def is_state_valid(self, q: List[float]) -> bool:
        for qi, (lo, hi) in zip(q, JOINT_LIMITS):
            if qi < lo or qi > hi:
                return False
        return True

    def _srv_clean_table(self, _, res):
        self.get_logger().info("Entered /clean_table (simple current→pre-clean test)")
        q_start = self.get_current_q()
        if q_start is None:
            res.success = False
            res.message = "No joint state"
            return res

        path = self.planner.plan(q_start, self.preclean_q)
        if not path:
            res.success = False
            res.message = "Planning to pre-clean failed"
            return res

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
    node = TiagoTableCleanerSimple()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
