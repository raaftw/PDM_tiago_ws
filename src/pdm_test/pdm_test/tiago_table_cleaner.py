#!/usr/bin/env python3
"""
tiago_table_cleaner.py

Services:
- /go_to_rest_pose (Trigger)
- /clean_table (Trigger)

Uses:
- Your RRT-Connect planner
- Simple state validity + edge checking
- FollowJointTrajectory for execution
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from .collision_checker import is_state_valid, is_edge_valid, TABLE
from .kinematics import fk_end_effector, ik_solve_xyz
from .rrt_connect import RRTConnect


ARM_JOINTS = [
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
]


class TiagoTableCleaner(Node):
    def __init__(self):
        super().__init__("tiago_table_cleaner")

        # Action client for arm execution
        self.arm_action = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )

        # Latest joint state
        self._latest_joint_state: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        # Services
        self.create_service(Trigger, "go_to_rest_pose", self._srv_go_to_rest)
        self.create_service(Trigger, "clean_table", self._srv_clean_table)

        # Planner
        self.planner = RRTConnect(
            is_state_valid=lambda q: is_state_valid(q, fk_end_effector),
            is_edge_valid=lambda q1, q2: is_edge_valid(q1, q2, fk_end_effector),
            step_size=0.2,
            max_iters=8000,
            goal_bias=0.10,
        )

        # Rest pose (use your captured values)
        self.rest_q = [
            0.5000004697,
            -1.3399972337,
            -0.4799966989,
            1.9399756239,
            -1.4900028450,
            1.3700203543,
            0.0,
        ]

        # Pre-clean pose (tune later)
        self.preclean_q = [
            0.2,
            -1.0,
            -0.2,
            1.0,
            -1.0,
            0.8,
            0.0,
        ]

        # Wipe parameters (1m x 1m top surface)
        self.wipe_size = 1.0
        self.wipe_step = 0.25
        self.wipe_z = TABLE.top_z  # plane constraint uses top_z

        self.get_logger().info("TiagoTableCleaner ready. Services: /go_to_rest_pose, /clean_table")

    def _js_cb(self, msg: JointState):
        self._latest_joint_state = msg

    def _get_current_arm_q(self) -> Optional[List[float]]:
        if self._latest_joint_state is None:
            return None
        name_to_pos = {n: p for n, p in zip(self._latest_joint_state.name, self._latest_joint_state.position)}
        try:
            return [float(name_to_pos[j]) for j in ARM_JOINTS]
        except KeyError:
            return None

    # -------------------------
    # Services
    # -------------------------
    def _srv_go_to_rest(self, req, res):
        ok = self._execute_joint_goal(self.rest_q, duration_sec=3.0)
        res.success = ok
        res.message = "Moved to rest pose." if ok else "Failed to move to rest pose."
        return res

    def _srv_clean_table(self, req, res):
        q_start = self._get_current_arm_q()
        if q_start is None:
            res.success = False
            res.message = "No /joint_states for arm joints yet."
            return res

        # Plan rest -> preclean
        self.get_logger().info("Planning: current -> preclean (RRT-Connect)")
        path = self.planner.plan(q_start, self.preclean_q)
        if path is None:
            res.success = False
            res.message = "Failed to plan current->preclean"
            return res

        if not self._execute_path(path, sec_per_waypoint=0.5):
            res.success = False
            res.message = "Failed executing current->preclean"
            return res

        # Wipe waypoints
        waypoints_xyz = self._generate_zigzag_xyz()

        q_curr = self._get_current_arm_q() or self.preclean_q

        for idx, target_xyz in enumerate(waypoints_xyz):
            ik_q = ik_solve_xyz(target_xyz, q_seed=q_curr)
            if ik_q is None:
                self.get_logger().warn(f"IK failed at waypoint {idx}: {target_xyz}")
                res.success = False
                res.message = f"IK failed at waypoint {idx}"
                return res

            q_goal = ik_q.tolist()

            self.get_logger().info(f"Planning wipe segment {idx+1}/{len(waypoints_xyz)}")
            seg_path = self.planner.plan(q_curr, q_goal)
            if seg_path is None:
                res.success = False
                res.message = f"Failed to plan wipe segment {idx}"
                return res

            if not self._execute_path(seg_path, sec_per_waypoint=0.4):
                res.success = False
                res.message = f"Failed executing wipe segment {idx}"
                return res

            q_curr = q_goal

        # Return to rest
        self.get_logger().info("Planning: end -> rest (RRT-Connect)")
        back_path = self.planner.plan(q_curr, self.rest_q)
        if back_path is None:
            res.success = False
            res.message = "Failed to plan end->rest"
            return res

        if not self._execute_path(back_path, sec_per_waypoint=0.5):
            res.success = False
            res.message = "Failed executing end->rest"
            return res

        res.success = True
        res.message = "Cleaning completed."
        return res

    # -------------------------
    # Wipe grid
    # -------------------------
    def _generate_zigzag_xyz(self) -> List[Tuple[float, float, float]]:
        pts: List[Tuple[float, float, float]] = []
        x0, x1 = TABLE.x_min, TABLE.x_max
        y0, y1 = TABLE.y_min, TABLE.y_max

        rows = int(round(self.wipe_size / self.wipe_step))
        rows = max(1, rows)

        for i in range(rows + 1):
            y = y0 + i * self.wipe_step
            if y > y1:
                y = y1

            if i % 2 == 0:
                pts.append((x0, y, self.wipe_z))
                pts.append((x1, y, self.wipe_z))
            else:
                pts.append((x1, y, self.wipe_z))
                pts.append((x0, y, self.wipe_z))

        return pts

    # -------------------------
    # Execution helpers
    # -------------------------
    def _execute_joint_goal(self, q_goal: List[float], duration_sec: float = 3.0) -> bool:
        if not self.arm_action.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Action server /arm_controller/follow_joint_trajectory not available.")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q_goal]
        pt.time_from_start.sec = int(duration_sec)
        pt.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        goal.trajectory.points = [pt]

        send_future = self.arm_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def _execute_path(self, path: List[List[float]], sec_per_waypoint: float = 0.4) -> bool:
        if not self.arm_action.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Action server /arm_controller/follow_joint_trajectory not available.")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS

        points: List[JointTrajectoryPoint] = []
        t = 0.0
        for q in path:
            t += sec_per_waypoint
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            points.append(pt)

        goal.trajectory.points = points

        send_future = self.arm_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = TiagoTableCleaner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

