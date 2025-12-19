#!/usr/bin/env python3
"""
Step-2 test node (RRT-only, no FK/IK):
- Plans current → pre-clean using RRT-Connect
- Validity: joint limits only
- Publishes trajectory to arm controller
"""

from __future__ import annotations
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
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

class TiagoTableCleanerRRT(Node):

    def __init__(self):
        super().__init__("tiago_table_cleaner_rrt")

        # Publisher
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

        # Joint state subscriber
        self._latest_joint_state: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        # Service
        self.create_service(Trigger, "/clean_table", self._srv_clean_table)

        # Planner
        self.planner = RRTConnect(
            is_state_valid=self.is_state_valid,
            is_edge_valid=self.is_edge_valid,
            step_size=0.2,
            max_iters=2000,
            goal_bias=0.5,
        )

        # Target pose
        self.preclean_q = [1.0, 1.0, 0.0, 2.2, -1.2, 0.7, 0.5]
        self.wipe_front = [1.2, 0.4, -0.0, 0.8, -1.2, 0.7, 0.5]
        self.wipe_left = [1.5, 0.4, -0.0, 0.8, -1.2, 0.7, 0.5]
        self.wipe_back = [1.5, 0.8, 0.0, 1.5, -1.2, 0.7, 0.5]
        self.rest_q = [0.50, -1.34, -0.48, 1.94, -1.49, 1.37, 0.00]

        self.get_logger().info("TiagoTableCleanerRRT ready (RRT-only).")

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

    # ---------- Validity ----------
    def is_state_valid(self, q: List[float]) -> bool:
        return all(lo <= qi <= hi for qi, (lo, hi) in zip(q, JOINT_LIMITS))

    def is_edge_valid(self, q1: List[float], q2: List[float]) -> bool:
        # With FK removed, just check joint limits at endpoints
        return self.is_state_valid(q1) and self.is_state_valid(q2)

    # ---------- Service ----------
    def _srv_clean_table(self, _, res):
        self.get_logger().info("Entered /clean_table (RRT-only)")
        q_start = self.get_current_q()
        if q_start is None:
            res.success = False
            res.message = "No joint state"
            return res
            
        # Define waypoints 
        waypoints = [ self.preclean_q, self.wipe_front, self.wipe_left, self.wipe_back, self.preclean_q, self.rest_q, ]

        full_path = [] 
        current = q_start 
        for goal in waypoints: 
            path_segment = self.planner.plan(current, goal) 
            if not path_segment: 
                res.success = False 
                res.message = f"Planning failed to {goal}" 
                return res
            # Concatenate (skip duplicate start) 
            if full_path: 
                full_path += path_segment[1:] 
            else: 
                full_path += path_segment 
            current = goal 
            
        # Publish trajectory 
        traj = JointTrajectory() 
        traj.joint_names = ARM_JOINTS 
        t = 0.0 
        for q in full_path: 
            t += 0.6 
            pt = JointTrajectoryPoint() 
            pt.positions = q 
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9)) 
            traj.points.append(pt) 
            
        self.get_logger().info(f"Publishing trajectory with {len(traj.points)} points (multi-waypoint wipe)") 
        self.arm_pub.publish(traj) 
        
        res.success = True 
        res.message = "Completed wipe sequence" 
        return res



def main():
    rclpy.init()
    node = TiagoTableCleanerRRT()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
