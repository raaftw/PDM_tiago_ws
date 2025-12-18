#!/usr/bin/env python3
"""
Step-2 FK test node (local FK, no MoveIt):
- Plans current → pre-clean using RRT-Connect
- Validity: joint limits + local FK clearance (endpoint-only)
- Loads URDF once and computes FK to chosen EEF link
"""

from __future__ import annotations
from typing import List, Optional

import pathlib

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .rrt_connect import RRTConnect
from .collision_checker import JOINT_LIMITS

from .fk_from_urdf import fk_tiago_arm

ARM_JOINTS = [
    "arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint",
    "arm_5_joint","arm_6_joint","arm_7_joint",
]

class TiagoTableCleanerLocalFK(Node):

    def __init__(self):
        super().__init__("tiago_table_cleaner_local_fk")

        # Publish trajectories
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

        # Subscribe to joint states
        self._latest_joint_state: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        # Service entry
        self.create_service(Trigger, "/clean_table", self._srv_clean_table)
'''
        # Load URDF text (point this to your expanded URDF file)
        # If you have the expanded URDF on parameter /robot_description, you can also load from there.
        urdf_path = pathlib.Path.home() / "PDM_tiago_ws" / "src" / "tiago_description" / "expanded_tiago.urdf"
        try:
            self.urdf_text = urdf_path.read_text()
            self.get_logger().info(f"Loaded URDF from {urdf_path}")
        except Exception as e:
            # Fallback: if you store a local copy, set the path here
            self.get_logger().warn(f"Failed to read URDF at {urdf_path}: {e}")
            self.urdf_text = ""
'''
        # Load URDF text from ROS parameter /robot_description
        self.declare_parameter("robot_description", "")
        self.urdf_text = self.get_parameter("robot_description").value

        if not self.urdf_text:
            self.get_logger().warn("URDF text is empty. Make sure /robot_description is set by your launch file.")
        else:
            self.get_logger().info("Loaded URDF from /robot_description parameter")

        # Base and EEF link names
        self.base_link = "base_link"           # confirm in your setup (could also be torso_lift_link if you want)
        self.eef_link = "hand_tool_link"    # replace with your actual gripper tip link name

        # Planner
        self.planner = RRTConnect(
            is_state_valid=self.is_state_valid,
            is_edge_valid=self.is_edge_valid,
            step_size=0.2,
            max_iters=2000,
            goal_bias=0.5,
        )

        # Poses
        self.preclean_q = [1.2, 1.0, 0.0, 2.2, -1.2, 0.7, 0.5]
        self.wipe_q = [0.8, 0.5, -0.2, 2.0, -1.0, 0.6, 0.3] 

        # Clearance threshold (auto-calibrated from goal FK)
        self.clearance_min_z: Optional[float] = None

        self.get_logger().info("TiagoTableCleanerLocalFK ready.")

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

    # ---------- Local FK ----------
    def fk_gripper(self, q: List[float]):
        if not self.urdf_text:
            raise RuntimeError("URDF text not loaded")
        x, y, z = fk_tiago_arm(
            urdf_text=self.urdf_text,
            base_link=self.base_link,
            eef_link=self.eef_link,
            arm_joint_order=ARM_JOINTS,
            q=q,
        )
        return x, y, z

    # ---------- Validity ----------
    def is_state_valid(self, q: List[float]) -> bool:
        return all(lo <= qi <= hi for qi, (lo, hi) in zip(q, JOINT_LIMITS))

    def is_edge_valid(self, q1: List[float], q2: List[float]) -> bool:
        if self.clearance_min_z is None:
            return True  # not calibrated yet
        for t in (0.0, 1.0):
            q = [(1 - t) * a + t * b for a, b in zip(q1, q2)]
            try:
                _, _, z = self.fk_gripper(q)
            except Exception as e:
                self.get_logger().warn(f"Local FK error: {e}")
                return False
            if z < self.clearance_min_z:
                return False
        return True

    # ---------- Service ----------
    def _srv_clean_table(self, _, res):
        self.get_logger().info("Entered /clean_table (multi-pose RRT test)")
        q_start = self.get_current_q()
        if q_start is None:
            res.success = False
            res.message = "No joint state"
            return res
    
        # Plan current → pre-clean
        path1 = self.planner.plan(q_start, self.preclean_q)
        if not path1:
            res.success = False
            res.message = "Planning to pre-clean failed"
            return res
    
        # Plan pre-clean → wipe
        path2 = self.planner.plan(self.preclean_q, self.wipe_q)
        if not path2:
            res.success = False
            res.message = "Planning to wipe failed"
            return res
    
        # Concatenate paths (skip duplicate start of path2)
        full_path = path1 + path2[1:]
    
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
    
        self.get_logger().info(f"Publishing trajectory with {len(traj.points)} points (multi-pose)")
        self.arm_pub.publish(traj)
        res.success = True    
        res.message = "Moved through pre-clean and wipe poses"    
        return res



def main():
    rclpy.init()
    node = TiagoTableCleanerLocalFK()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
