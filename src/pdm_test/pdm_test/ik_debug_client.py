#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK


class IKDebugClient(Node):
    def __init__(self):
        super().__init__("ik_debug_client")

        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("IK service /compute_ik not available")
            rclpy.shutdown()
            return

        # Call IK once after startup
        self.timer = self.create_timer(2.0, self.call_once)

    def call_once(self):
        self.timer.cancel()
        self.get_logger().info("Calling IK once from standalone client")

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.6
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.8
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.pose_stamped = pose
        req.ik_request.ik_link_name = "arm_7_link"

        self.get_logger().info(f"IK REQUEST (standalone):\n{req}")

        future = self.cli.call_async(req)

        def done_cb(fut):
            res = fut.result()
            self.get_logger().info(f"IK RESPONSE (standalone):\n{res}")
            rclpy.shutdown()

        future.add_done_callback(done_cb)


def main():
    rclpy.init()
    node = IKDebugClient()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

