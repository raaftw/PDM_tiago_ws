#!/usr/bin/env python3

from __future__ import annotations
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from std_srvs.srv import Trigger

from .rrt_connect import RRTConnect
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


class TiagoTableCleanerRRTIK(Node):
    def __init__(self):
        super().__init__("tiago_table_cleaner_rrt_ik")

        # Publisher to arm controller
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )

        # Joint state subscriber
        self._latest_joint_state: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

        # MoveIt IK client
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.get_logger().info("Waiting for /compute_ik service...")
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("MoveIt IK service /compute_ik not available.")
        else:
            self.get_logger().info("Connected to /compute_ik.")

        # RRT planner
        self.planner = RRTConnect(
            is_state_valid=self.is_state_valid,
            is_edge_valid=self.is_edge_valid,
            step_size=0.2,
            max_iters=2000,
            goal_bias=0.5,
        )

        # Cartesian waypoints in base_link
        self.cartesian_waypoints: List[Tuple[float, float, float]] = [
            (0.6, 0.0, 0.85),
            (0.7, 0.0, 0.82),
            (0.7, 0.2, 0.82),
            (0.6, 0.2, 0.82),
            (0.6, 0.0, 0.85),
        ]

        # Poses
        self.preclean_q: List[float] = [1.0, 1.0, 0.0, 2.2, -1.2, 0.7, 0.5]
        self.rest_q: List[float] = [0.50, -1.34, -0.48, 1.94, -1.49, 1.37, 0.00]

        # MoveIt config
        self.moveit_group_name = "arm"
        self.moveit_ee_link = "arm_tool_link"
        self.planning_frame = "base_link"

        # State for pipeline control
        self._pipeline_running: bool = False
        self._trigger_requested: bool = False

        # State for IK sequence
        self._ik_index: int = 0
        self._ik_joint_waypoints: List[List[float]] = []
        self._q_start: Optional[List[float]] = None

        # Service to trigger cleaning
        self._srv = self.create_service(
            Trigger, "/clean_table", self._srv_clean_table
        )

        # Timer to check triggers and start pipeline
        self.create_timer(0.1, self._timer_check_trigger)

        self.get_logger().info(
            "TiagoTableCleanerRRTIK ready. Call /clean_table to start cleaning."
        )

    # ---------------- Joint state handling ----------------

    def _joint_state_cb(self, msg: JointState):
        self._latest_joint_state = msg

    def get_current_q(self) -> Optional[List[float]]:
        if self._latest_joint_state is None:
            return None
        name_to_pos = dict(zip(self._latest_joint_state.name,
                               self._latest_joint_state.position))
        try:
            return [float(name_to_pos[j]) for j in ARM_JOINTS]
        except KeyError:
            return None

    # ---------------- Validity checks for RRT ----------------

    def is_state_valid(self, q: List[float]) -> bool:
        return all(lo <= qi <= hi for qi, (lo, hi) in zip(q, JOINT_LIMITS))

    def is_edge_valid(self, q1: List[float], q2: List[float]) -> bool:
        if not (self.is_state_valid(q1) and self.is_state_valid(q2)):
            return False
        mid = [(a + b) * 0.5 for a, b in zip(q1, q2)]
        return self.is_state_valid(mid)

    # ---------------- Service callback ----------------

    def _srv_clean_table(self, request: Trigger.Request, response: Trigger.Response):
        if self._pipeline_running:
            response.success = False
            response.message = "Cleaning pipeline already running."
            return response

        self._trigger_requested = True
        response.success = True
        response.message = "Cleaning pipeline triggered."
        self.get_logger().info("Received /clean_table request. Triggering pipeline...")
        return response

    # ---------------- Trigger timer ----------------

    def _timer_check_trigger(self):
        if self._trigger_requested and not self._pipeline_running:
            self._trigger_requested = False
            self._start_pipeline()

    # ---------------- Pipeline start ----------------

    def _start_pipeline(self):
        self.get_logger().info("Starting IK + RRT cleaning pipeline...")
        self._pipeline_running = True

        # Get current joint state as starting configuration
        self._q_start = self.get_current_q()
        if self._q_start is None:
            self.get_logger().error("No current joint state available; aborting.")
            self._pipeline_running = False
            return

        if not self.ik_client.service_is_ready():
            self.get_logger().error("IK service /compute_ik not ready; aborting.")
            self._pipeline_running = False
            return

        # Reset IK state and start with waypoint 0
        self._ik_joint_waypoints = []
        self._ik_index = 0

        # First motion: current → preclean_q will be handled in RRT phase
        self._call_ik_for_current_waypoint()

    # ---------------- IK sequence (sequential, callback-based) ----------------

    def _call_ik_for_current_waypoint(self):
        """Send IK request for waypoint at self._ik_index."""
        if self._ik_index >= len(self.cartesian_waypoints):
            # All waypoints done → append preclean + rest, then RRT
            self.get_logger().info("All IK waypoints solved. Adding preclean & rest.")
            self._ik_joint_waypoints.append(self.preclean_q)
            self._ik_joint_waypoints.append(self.rest_q)
            self._run_rrt_and_publish()
            return

        x, y, z = self.cartesian_waypoints[self._ik_index]
        self.get_logger().info(
            f"Calling IK for waypoint {self._ik_index + 1} "
            f"at ({x:.3f}, {y:.3f}, {z:.3f})"
        )

        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.moveit_group_name
        req.ik_request.pose_stamped = pose
        req.ik_request.ik_link_name = self.moveit_ee_link

        # Seed RobotState with current arm joints only
        seed_q = self.get_current_q()
        if seed_q is not None:
            seed_js = JointState()
            seed_js.name = ARM_JOINTS
            seed_js.position = seed_q
            rs = RobotState()
            rs.joint_state = seed_js
            req.ik_request.robot_state = rs

        future = self.ik_client.call_async(req)
        # Bind the current index to the callback
        future.add_done_callback(self._make_ik_done_cb(self._ik_index))

    def _make_ik_done_cb(self, index: int):
        """Capture index for the IK done callback."""

        def _ik_done_cb(future):
            x, y, z = self.cartesian_waypoints[index]
            try:
                result = future.result()
            except Exception as e:
                self.get_logger().error(f"IK exception at waypoint {index + 1}: {e}")
                self._pipeline_running = False
                return

            if result is None:
                self.get_logger().error(
                    f"IK returned no result for waypoint {index + 1} "
                    f"({x:.3f}, {y:.3f}, {z:.3f})"
                )
                self._pipeline_running = False
                return

            if result.error_code.val != result.error_code.SUCCESS:
                self.get_logger().error(
                    f"IK failed for waypoint {index + 1} "
                    f"({x:.3f}, {y:.3f}, {z:.3f}), "
                    f"error_code={result.error_code.val}"
                )
                self._pipeline_running = False
                return

            js = result.solution.joint_state
            name_to_pos = dict(zip(js.name, js.position))
            try:
                q = [float(name_to_pos[j]) for j in ARM_JOINTS]
            except KeyError as e:
                self.get_logger().error(
                    f"Missing joint {e} in IK result at waypoint {index + 1}"
                )
                self._pipeline_running = False
                return

            self.get_logger().info(
                f"IK OK for waypoint {index + 1} → "
                f"[{', '.join(f'{v:+.3f}' for v in q)}]"
            )
            self._ik_joint_waypoints.append(q)

            # Move to next waypoint
            self._ik_index += 1
            self._call_ik_for_current_waypoint()

        return _ik_done_cb

    # ---------------- RRT + trajectory publishing ----------------

    def _run_rrt_and_publish(self):
        """Run RRT over:
           current → preclean_q → IK waypoints → preclean_q → rest_q
        """
        if self._q_start is None:
            self.get_logger().error("No start configuration for RRT; aborting.")
            self._pipeline_running = False
            return

        # Build the full list of joint-space targets
        # Start with preclean_q before IK path
        joint_targets: List[List[float]] = [self.preclean_q]
        # Then IK waypoints (already in self._ik_joint_waypoints except final two)
        # At this point _ik_joint_waypoints = [IK1, IK2, ..., IKN, preclean_q, rest_q]
        if len(self._ik_joint_waypoints) < 2:
            self.get_logger().error("Not enough joint waypoints collected; aborting.")
            self._pipeline_running = False
            return

        ik_only = self._ik_joint_waypoints[:-2]  # exclude last preclean & rest
        joint_targets.extend(ik_only)
        # Then preclean_q again and rest_q
        joint_targets.append(self.preclean_q)
        joint_targets.append(self.rest_q)

        self.get_logger().info("Starting RRT planning between joint waypoints...")

        full_path: List[List[float]] = []
        current = self._q_start

        for idx, goal in enumerate(joint_targets, start=1):
            self.get_logger().info(
                f"Planning segment {idx}: "
                f"from [{', '.join(f'{v:+.3f}' for v in current)}] "
                f"to   [{', '.join(f'{v:+.3f}' for v in goal)}]"
            )
            path = self.planner.plan(current, goal)
            if not path:
                self.get_logger().error(f"RRT failed at segment {idx}; aborting.")
                self._pipeline_running = False
                return

            if full_path:
                full_path.extend(path[1:])
            else:
                full_path.extend(path)

            current = goal

        self.get_logger().info(
            f"RRT produced a path with {len(full_path)} joint states."
        )

        # Build JointTrajectory
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS

        t = 0.0
        for q in full_path:
            t += 0.6  # seconds per step
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1.0) * 1e9),
            )
            traj.points.append(pt)

        self.get_logger().info(
            f"Publishing trajectory with {len(traj.points)} points to "
            f"/arm_controller/joint_trajectory."
        )
        self.arm_pub.publish(traj)
        self._pipeline_running = False
        self.get_logger().info("Cleaning pipeline finished.")

def main(args=None):
    rclpy.init(args=args)
    node = TiagoTableCleanerRRTIK()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

