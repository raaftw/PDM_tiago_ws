import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math
from typing import List


class MinObstacleDistanceNode(Node):
    def __init__(self):
        super().__init__('metrics_min_distance')
        # Allow multiple potential scan topics to support different stacks
        default_topics = ['/scan_raw', '/scan', '/front_laser/scan']
        self.scan_topics: List[str] = self.declare_parameter('scan_topics', default_topics).get_parameter_value().string_array_value or default_topics

        self.pub = self.create_publisher(Float32, '/metrics/min_obstacle_distance', 10)
        self._subs = []
        for t in self.scan_topics:
            try:
                sub = self.create_subscription(LaserScan, t, self._scan_cb, 10)
                self._subs.append(sub)
                self.get_logger().info(f'Subscribed to LaserScan topic: {t}')
            except Exception as e:
                self.get_logger().warn(f'Failed to subscribe to {t}: {e}')

    def _scan_cb(self, msg: LaserScan):
        # Compute minimal valid distance in the scan
        min_d = None
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and r >= msg.range_min and r <= msg.range_max and r > 0.01:
                # Optionally restrict FOV to front; here we use full scan for safety margin
                if min_d is None or r < min_d:
                    min_d = r
            angle += msg.angle_increment

        if min_d is None:
            return

        out = Float32()
        out.data = float(min_d)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MinObstacleDistanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
