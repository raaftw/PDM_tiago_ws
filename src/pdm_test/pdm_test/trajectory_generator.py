import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np
import math
from pdm_test.utils import PathGenerator, resample_path

class TrajectoryGenerator(Node):
    """
    Generate and publish a reference trajectory as nav_msgs/Path.

    Parameters:
      - path_type: 'line' | 'circle'
      - start: [x, y] for line
      - goal: [x, y] for line
      - center, radius: for circle
      - num_points: number of path waypoints
      - publish_rate: Hz to republish the path
      - frame_id: TF frame for poses (e.g. 'map' or 'odom')
    """

    def __init__(self):
        super().__init__('trajectory_generator')

        # Ros parameters (default values)
        self.path_type = self.declare_parameter('path_type', 'line').value
        self.num_points = int(self.declare_parameter('num_points', 100).value)
        self.publish_rate = float(self.declare_parameter('publish_rate', 1.0).value)

        self.start_x = float(self.declare_parameter('start_x', 0.0).value)
        self.start_y = float(self.declare_parameter('start_y', 0.0).value)
        self.goal_x = float(self.declare_parameter('goal_x', 5.0).value)
        self.goal_y = float(self.declare_parameter('goal_y', 0.0).value)

        self.circle_center_x = float(self.declare_parameter('circle_center_x', 0.0).value)
        self.circle_center_y = float(self.declare_parameter('circle_center_y', 0.0).value)
        self.circle_radius = float(self.declare_parameter('circle_radius', 1.0).value)
        
        self.start_angle = float(self.declare_parameter('start_angle', 0.0).value)
        self.direction = self.declare_parameter('direction', 'ccw').value
        self.resample_to = self.declare_parameter('resample_to', -1).value

        self.start = (self.start_x, self.start_y)
        self.goal = (self.goal_x, self.goal_y)
        self.center = (self.circle_center_x, self.circle_center_y)
        self.radius = self.circle_radius

        # TODO: Change to map frame, when available
        self.frame_id = self.declare_parameter('frame_id', 'odom').value


        # Create publisher of the path
        self.path_pub = self.create_publisher(Path, 'reference_path', 10)

        # Create timer
        self.create_timer(1.0 / float(self.publish_rate), self._timer_cb)
        

    def _timer_cb(self):
        """
        Periodic callback:
         - build path points: call self._build_path()
         - convert points to nav_msgs/Path (PoseStamped per point)
         - publish the Path
        """
        
        # Get the points of the path
        path_points = self._build_path() 

        # Create path message from the points
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id

        for point in path_points:

            # Create PoseStamped for each point
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            if path_points.shape[1] > 2:
                theta = point[2]
            else:
                theta = 0.0

            pose.pose.orientation = self._theta_to_quaternion(theta)
            path_msg.poses.append(pose)

        # Publish the path
        self.path_pub.publish(path_msg)
        self.get_logger().debug(f'Published /reference_path with {path_msg.poses.__len__()} poses')

    def _build_path(self) -> 'np.ndarray':
        """
        Construct the path as a (N,3) numpy array, path_type can be 'line' or 'circle'.
        
        Return:
          - (N,3) array of [x,y,theta]
        """

        # Use utils method to generate the base xy path
        if self.path_type.lower() == 'line':
            xy = PathGenerator.straight_line(self.start, self.goal, num_points=self.num_points)
        elif self.path_type.lower() == 'circle':
            xy = PathGenerator.circle(self.center, self.radius, num_points=self.num_points,
                                    start_angle=float(self.start_angle),
                                    direction=self.direction)
        else:
            xy = np.array([[0.0, 0.0]])

        # Log the generated path for debugging
        self.get_logger().debug(f'Path points: {xy}')
        self.get_logger().info(f'Generated {self.path_type} path with {len(xy)} points: start={xy[0]}, end={xy[-1]}')

        # Attach orientations (theta) so downstream code has (x,y,theta)
        path_with_theta = PathGenerator.add_orientation_to_path(xy)

        # Optionally resample to a different resolution
        if hasattr(self, 'resample_to') and self.resample_to > 0:
            path_with_theta = resample_path(path_with_theta, int(self.resample_to))
            
        return path_with_theta 


    def _theta_to_quaternion(self, theta: float) -> 'Quaternion':
        """
        Convert yaw (theta) to geometry_msgs/Quaternion.
        """
        
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(theta / 2.0)
        q.w = math.cos(theta / 2.0)
        return q


def main(args=None):
    """
    Start the TrajectoryGenerator node.
    """
    
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()