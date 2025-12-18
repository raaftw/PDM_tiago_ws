#!/usr/bin/env python3
"""
Ground Truth Republisher Node

This node subscribes to /ground_truth_odom (nav_msgs/Odometry) from Gazebo
and republishes it as /ground_truth_pose (geometry_msgs/PoseStamped) for
visualization in RViz.

This is necessary because RViz's Pose display only accepts PoseStamped messages,
not Odometry messages.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class GroundTruthRepublisher(Node):
    """
    Subscribes to ground truth odometry and republishes as PoseStamped.
    """

    def __init__(self):
        super().__init__('ground_truth_republisher')
        
        # Publisher for ground truth pose
        self.pose_pub = self.create_publisher(PoseStamped, '/ground_truth_pose', 10)
        
        # Subscriber to ground truth odometry from Gazebo
        self.create_subscription(Odometry, '/ground_truth_odom', self._odom_callback, 10)
        
        self.get_logger().info('Ground truth republisher started')
        self.get_logger().info('  Subscribing to: /ground_truth_odom')
        self.get_logger().info('  Publishing to: /ground_truth_pose')

    def _odom_callback(self, odom_msg: Odometry):
        """
        Convert Odometry to PoseStamped and republish.
        
        Args:
            odom_msg: Odometry message from /ground_truth_odom
        """
        pose_msg = PoseStamped()
        
        # Copy header (frame_id and timestamp)
        pose_msg.header = odom_msg.header
        
        # Copy pose from odometry
        pose_msg.pose = odom_msg.pose.pose
        
        # Publish
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
