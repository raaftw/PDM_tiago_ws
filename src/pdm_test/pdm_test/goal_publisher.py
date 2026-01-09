#!/usr/bin/env python3
"""
Automatic goal publisher for benchmarking.
Publishes a goal after startup delay to ensure consistent testing coordinates.

For MPC: publishes geometry_msgs/PoseStamped to /goal_pose
For Nav2: sends goal via NavigateToPose action
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        # Predefined goal locations (in front of tables - 4 corners)
        self.goal_locations = {
            'center': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'corner_1': {'x': 2.5, 'y': 1.8, 'theta': 1.57},      # Front-right
            'corner_2': {'x': 2.5, 'y': -1.8, 'theta': -1.57},    # Back-right
            'corner_3': {'x': -2.5, 'y': 1.8, 'theta': 1.57},     # Front-left
            'corner_4': {'x': -2.5, 'y': -1.8, 'theta': -1.57},   # Back-left
        }
        
        # Parameters
        self.declare_parameter('mode', 'mpc')  # 'mpc' or 'nav2'
        self.declare_parameter('goal_location', 'center')
        self.declare_parameter('goal_x', '')  # Empty means use goal_location
        self.declare_parameter('goal_y', '')
        self.declare_parameter('goal_theta', '')
        self.declare_parameter('startup_delay', 20.0)
        self.declare_parameter('frame_id', 'map')
        
        self.mode = self.get_parameter('mode').value
        goal_location = self.get_parameter('goal_location').value
        goal_x_str = self.get_parameter('goal_x').value
        goal_y_str = self.get_parameter('goal_y').value
        goal_theta_str = self.get_parameter('goal_theta').value
        self.startup_delay = self.get_parameter('startup_delay').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Determine goal coordinates
        if goal_x_str and goal_y_str and goal_theta_str:
            # Use explicit coordinates if provided
            try:
                self.goal_x = float(goal_x_str)
                self.goal_y = float(goal_y_str)
                self.goal_theta = float(goal_theta_str)
                self.get_logger().info(f'Using explicit goal coordinates: x={self.goal_x}, y={self.goal_y}, theta={self.goal_theta}')
            except ValueError:
                self.get_logger().error(f'Invalid goal coordinates: x={goal_x_str}, y={goal_y_str}, theta={goal_theta_str}')
                return
        else:
            # Use predefined location
            if goal_location not in self.goal_locations:
                self.get_logger().error(f'Invalid goal_location: {goal_location}. Valid: {list(self.goal_locations.keys())}')
                return
            coords = self.goal_locations[goal_location]
            self.goal_x = coords['x']
            self.goal_y = coords['y']
            self.goal_theta = coords['theta']
            self.get_logger().info(f'Using predefined location "{goal_location}": x={self.goal_x}, y={self.goal_y}, theta={self.goal_theta}')
        
        self.get_logger().info(f'Goal Publisher initialized: mode={self.mode}, goal=({self.goal_x}, {self.goal_y}, {self.goal_theta}), delay={self.startup_delay}s')
        
        # Both MPC and Nav2 use /goal_pose topic
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Wait for startup, then publish goal once
        self.timer = self.create_timer(self.startup_delay, self.publish_goal_once)
    
    def publish_goal_once(self):
        """Publish goal after startup delay."""
        self.timer.cancel()  # Only publish once
        
        self.get_logger().info(f'Publishing goal now: mode={self.mode}, x={self.goal_x}, y={self.goal_y}, theta={self.goal_theta}')
        
        # Both modes use the same method now
        self.publish_goal()
    
    def publish_goal(self):
        """Publish PoseStamped to /goal_pose (works for both MPC and Nav2)."""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.frame_id
        
        goal_msg.pose.position.x = self.goal_x
        goal_msg.pose.position.y = self.goal_y
        goal_msg.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        qz = math.sin(self.goal_theta / 2.0)
        qw = math.cos(self.goal_theta / 2.0)
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw
        
        # Check if anyone is listening
        num_subscribers = self.goal_pub.get_subscription_count()
        self.get_logger().info(f'Publishing to /goal_pose (subscribers: {num_subscribers})')
        self.get_logger().info(f'Goal message: position=({self.goal_x}, {self.goal_y}), yaw={self.goal_theta} rad ({math.degrees(self.goal_theta):.1f}°), quat=({goal_msg.pose.orientation.x}, {goal_msg.pose.orientation.y}, {goal_msg.pose.orientation.z}, {goal_msg.pose.orientation.w})')
        
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Goal published to /goal_pose: x={self.goal_x}, y={self.goal_y}, theta={self.goal_theta} (mode={self.mode})')


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
