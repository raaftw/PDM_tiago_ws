import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray


class ObstaclePublisher(Node):
    """Convert an occupancy grid into obstacle triplets (x, y, r)."""

    def __init__(self):
        super().__init__('obstacle_publisher')

        self.occ_threshold = int(self.declare_parameter('occ_threshold', 50).value)
        self.stride = int(self.declare_parameter('stride', 2).value)
        self.frame_id = str(self.declare_parameter('frame_id', 'map').value)
        self.min_obstacle_radius = float(self.declare_parameter('min_obstacle_radius', 0.15).value)
        self.map_topic = str(self.declare_parameter('map_topic', '/map').value)
        self.output_topic = str(self.declare_parameter('output_topic', '/obstacles').value)

        # Map server publishes with transient local durability; match QoS to receive latched map.
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.obstacle_pub = self.create_publisher(Float32MultiArray, self.output_topic, 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, map_qos)

        self._last_msg = None
        self.create_timer(1.0, self._republish_if_available)

        self.get_logger().info(
            f'obstacle_publisher listening to {self.map_topic}, publishing {self.output_topic}, '
            f'threshold={self.occ_threshold}, stride={self.stride}'
        )

    def _map_cb(self, msg: OccupancyGrid):
        obstacles_msg = self._grid_to_obstacles(msg)
        if obstacles_msg is None:
            return

        self._last_msg = obstacles_msg
        self.obstacle_pub.publish(obstacles_msg)
        count = len(obstacles_msg.data) // 3
        self.get_logger().info(f'Published {count} obstacles')

    def _republish_if_available(self):
        if self._last_msg is not None:
            self.obstacle_pub.publish(self._last_msg)

    def _grid_to_obstacles(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        if width * height != len(msg.data):
            self.get_logger().warn('OccupancyGrid size mismatch; skipping map')
            return None

        stride = max(1, self.stride)
        threshold = self.occ_threshold

        yaw = self._quaternion_to_yaw(origin.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        ox = origin.position.x
        oy = origin.position.y

        obstacles = []
        radius = max(self.min_obstacle_radius, 0.5 * resolution * stride)

        for iy in range(0, height, stride):
            row_offset = iy * width
            for ix in range(0, width, stride):
                idx = row_offset + ix
                cell = msg.data[idx]

                # Ignore unknown (-1) and free space below threshold
                if cell < 0 or cell < threshold:
                    continue

                # Cell center in map frame, accounting for map origin yaw
                x_cell = (ix + 0.5) * resolution
                y_cell = (iy + 0.5) * resolution
                x_world = ox + cos_yaw * x_cell - sin_yaw * y_cell
                y_world = oy + sin_yaw * x_cell + cos_yaw * y_cell

                obstacles.extend([x_world, y_world, radius])

        if not obstacles:
            self.get_logger().warn('No occupied cells above threshold; not publishing obstacles')
            return None

        msg_out = Float32MultiArray()
        msg_out.data = obstacles
        return msg_out

    @staticmethod
    def _quaternion_to_yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
