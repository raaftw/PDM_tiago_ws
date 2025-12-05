import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

class MpcController(Node):
    """
    Subscribe to /odom and /reference_path, compute control, publish /cmd_vel.

    Parameters to declare:
      - control_rate (Hz)
      - mpc_horizon (int)
      - dt (float)
      - max_v, max_omega (limits)
      - optional gains (k_v, k_theta) for a fallback controller
    """

    def __init__(self):
        super().__init__('mpc_controller')

        # Get parameters and store locally
        self.control_rate = float(self.declare_parameter('control_rate', 10.0).value)
        self.mpc_horizon = int(self.declare_parameter('mpc_horizon', 10).value)
        self.k_heading = float(self.declare_parameter('k_heading', 2.0).value)
        self.v_const = float(self.declare_parameter('v_const', 1.0).value)
        self.dt = float(self.declare_parameter('dt', 0.1).value)
        self.max_v = float(self.declare_parameter('max_v', 5.0).value)
        self.max_omega = float(self.declare_parameter('max_omega', 5.0).value)

        # Create publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscriptions to odom and path (global planner)
        self.create_subscription(Odometry, '/mobile_base_controller/odom', self._odom_cb, 10)  # to get current state (gazebo odometry)
        self.create_subscription(Path, 'reference_path', self._path_cb, 10)  # to get the reference path

        # Create control timer
        self.create_timer(1.0 / self.control_rate, self._control_timer_cb)

        self.current_state = None   # numpy array [x,y,theta]
        self.reference_path = None  # numpy array (N,3)

    def _odom_cb(self, msg: 'Odometry'):
        """
        Odometry callback:
         - extract x, y from msg.pose.pose.position
         - extract yaw from quaternion in msg.pose.pose.orientation
         - store as numpy array self.current_state = np.array([x, y, yaw])
        """
        
        self.current_state = np.array([msg.pose.pose.position.x,
                                        msg.pose.pose.position.y,
                                        self._quaternion_to_theta(msg.pose.pose.orientation)])

    def _quaternion_to_theta(self, q: 'Quaternion') -> float:
        """
        Convert geometry_msgs/Quaternion to yaw angle theta in radians.
        """
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        return theta
        

    def _path_cb(self, msg: 'Path'):
        """
        Path callback:
         - convert nav_msgs/Path (PoseStamped[]) to a numpy array
           of shape (N,3) where each row is [x,y,theta]
         - store in self.reference_path
        """
        
        self.reference_path = self._path_to_numpy(msg)


    def _control_timer_cb(self):
      """
      Periodic control loop call back, compute and publish control twist msg.
      """

      # Publish zero if no state or path yet
      if self.current_state is None or self.reference_path is None:
          zero_msg = Twist()
          self.cmd_pub.publish(zero_msg)
          return

      v, omega = self.compute_control(self.current_state, self.reference_path)

      self.get_logger().info(f'State: x={self.current_state[0]:.2f}, y={self.current_state[1]:.2f}, theta={self.current_state[2]:.2f}')
    

      # #  Saturate controls to avoid exceeding robot limits
      # v = np.clip(v, -self.max_v, self.max_v)
      # omega = np.clip(omega, -self.max_omega, self.max_omega)

      # Create and publish twist message to cmd_vel
      twist_msg = Twist()
      twist_msg.linear.x = v
      twist_msg.angular.z = omega

      self.get_logger().debug(f'Publishing v={v:.3f}, omega={omega:.3f}')


      self.cmd_pub.publish(twist_msg)


    def compute_control(self, state: 'np.ndarray', ref_path: 'np.ndarray') -> tuple:
        """
        Compute control (v, omega) to follow ref_path from current state. This is just a proportional controller on the heading and constant v.


        Later replace this method with MPC:
         - setup optimization over horizon using model predictions
         - solve for control sequence and return first control
        """
        
        # Baseline controller (just P control to follow the path)
        position = state[0:2]

        # Find nearest point on reference path
        dists = np.linalg.norm(ref_path[:, 0:2] - position, axis=1)
        nearest_idx = np.argmin(dists)
        nearest_point = ref_path[nearest_idx]

        # Get desired heading at nearest point on ref path
        desired_theta = nearest_point[2]

        # Compute heading error
        heading_error = self._wrap_to_pi(desired_theta - state[2])

        # P controller for heading
        k_heading = 2.0  # gain for heading
        omega = self.k_heading * heading_error  

        v = self.v_const  # constant forward speed (from parameter)

        return v, omega

    def _wrap_to_pi(self, angle: float) -> float:
        """
        Robust angle normalization.
        Recommended implementation:
          return (angle + np.pi) % (2.0 * np.pi) - np.pi
        or using arctan2:
          return np.arctan2(np.sin(angle), np.cos(angle))
        """
        
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def _path_to_numpy(self, path_msg: 'Path') -> 'np.ndarray':
        """
        Convert nav_msgs/Path to numpy array (N,3) of [x,y,theta].
        For each PoseStamped:
          - x = pose.position.x
          - y = pose.position.y
          - theta = yaw from pose.orientation quaternion
        """
        
        points = []
        for pose_stamped in path_msg.poses:
            
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y

            theta = self._quaternion_to_theta(pose_stamped.pose.orientation)
            points.append([x, y, theta])

        return np.array(points)

# main entrypoint for console_scripts
def main(args=None):
    """
    Start the MpcController node.
    """
    rclpy.init(args=args)
    node = MpcController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()