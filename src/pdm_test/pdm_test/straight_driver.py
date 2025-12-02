import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class StraightDriver(Node):
    """Simple node that publishes a constant forward+angular velocity on /cmd_vel to drive in circles."""

    def __init__(self):
        super().__init__('straight_driver')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_cb)  # 10 Hz
        self.duration_s = float(self.declare_parameter('duration', 60.0).value)
        # linear_speed controls forward motion, angular_speed controls rotation (radians/sec)
        self.linear_speed = float(self.declare_parameter('linear_speed', 0.2).value)
        self.angular_speed = float(self.declare_parameter('angular_speed', 0.5).value)
        self._start_time = self.get_clock().now()
        self.get_logger().info(
            f'StraightDriver (circle) running: linear={self.linear_speed} m/s angular={self.angular_speed} rad/s for {self.duration_s}s'
        )

    def timer_cb(self):
        # stop after duration
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        msg = Twist()
        if elapsed < self.duration_s:
            msg.linear.x = float(self.linear_speed)
            msg.angular.z = float(self.angular_speed)
        else:
            # send zero to stop
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StraightDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down StraightDriver')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
