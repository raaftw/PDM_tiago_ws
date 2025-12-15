import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
from pdm_test.models.tiago_diff_drive_model import TiagoDifferentialDriveModel
from scipy.optimize import minimize
from std_msgs.msg import Float32MultiArray


class MpcController(Node):
    """
    Subscribe to /odom and /reference_path, compute control, publish /cmd_vel.

    Parameters to declare:
      - control_rate (Hz)
      - mpc_horizon (int)
      - dt (float)
      - max_v, max_omega (limits)
      - controller_type: "dummy" or "mpc"
      - dummy controller gains (v_const, k_heading) 
    """

    def __init__(self):
        super().__init__('mpc_controller')

        # Get parameters and store locally
        self.control_rate = float(self.declare_parameter('control_rate', 10.0).value)
        self.mpc_horizon = int(self.declare_parameter('mpc_horizon', 10).value)
        self.controller_type = str(self.declare_parameter('controller_type', 'mpc').value)
        self.k_heading = float(self.declare_parameter('k_heading', 2.0).value)
        self.v_const = float(self.declare_parameter('v_const', 1.0).value)
        self.dt = float(self.declare_parameter('dt', 0.1).value)
        self.max_v = float(self.declare_parameter('max_v', 5.0).value)
        self.v_min = float(self.declare_parameter('v_min', -0.5).value)
        self.max_omega = float(self.declare_parameter('max_omega', 5.0).value)

        # MPC-specific parameters
        self.Q_x = float(self.declare_parameter('Q_x', 10.0).value)
        self.Q_y = float(self.declare_parameter('Q_y', 10.0).value)
        self.Q_theta = float(self.declare_parameter('Q_theta', 5.0).value)
        self.R_v = float(self.declare_parameter('R_v', 0.1).value)       
        self.R_omega = float(self.declare_parameter('R_omega', 0.1).value)

        self.W_obstacle = float(self.declare_parameter('W_obstacle', 10.0).value)
        self.d_safe = float(self.declare_parameter('d_safe', 0.5).value)
        self.robot_radius = float(self.declare_parameter('robot_radius', 0.2).value)
        self.obstacle_source = str(self.declare_parameter('obstacle_source', 'map').value)

        self.v_ref = float(self.declare_parameter('v_ref', 0.7).value)  # MPC reference velocity

        self.optimizer_maxiter = int(self.declare_parameter('optimizer_maxiter', 60).value)

        self.robot_model = TiagoDifferentialDriveModel(dt=self.dt)

        # Obstacles (triplets: x, y, r) in controller frame (e.g., 'odom')
        self.obstacles = []
        self.create_subscription(Float32MultiArray, '/obstacles', self._obstacles_cb, 10)

        # Create publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscriptions to odom and path (global planner)
        self.create_subscription(Odometry, '/mobile_base_controller/odom', self._odom_cb, 10)  # to get current state (gazebo odometry)
        self.create_subscription(Path, '/reference_path', self._path_cb, 10)  # to get the reference path

        # Create control timer
        self.create_timer(1.0 / self.control_rate, self._control_timer_cb)

        self.current_state = None   # numpy array [x,y,theta]
        self.reference_path = None  # numpy array (N,3)
        self.prev_v = 0.0  # For acceleration limiting
         
        self.get_logger().info(f'Local planner initialized with type: {self.controller_type}')


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

    def _obstacles_cb(self, msg: Float32MultiArray):
      data = list(msg.data)
      if not data or (len(data) % 3) != 0:
          self.get_logger().warn('Received /obstacles with invalid length; expected triplets (x,y,r)')
          return
      self.obstacles = [(data[i], data[i+1], data[i+2]) for i in range(0, len(data), 3)]

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
      v = np.clip(v, self.v_min, self.max_v)
      omega = np.clip(omega, -self.max_omega, self.max_omega)

      # Create and publish twist message to cmd_vel
      twist_msg = Twist()
      twist_msg.linear.x = v
      twist_msg.angular.z = omega

      self.get_logger().debug(f'Publishing v={v:.3f}, omega={omega:.3f}')

      self.cmd_pub.publish(twist_msg)


    def compute_control(self, state: 'np.ndarray', ref_path: 'np.ndarray') -> tuple:
      """
      Main dispatcher: choose controller based on parameter.
      
      Returns: (v, omega)
      """
      if self.controller_type == 'dummy':
          return self._compute_dummy_control(state, ref_path)
      elif self.controller_type == 'mpc':
          return self._compute_mpc_control(state, ref_path)
      elif self.controller_type == 'mpc_avoid':
          return self._compute_control_mpc_avoid(self.current_state, self.reference_path)
      else:
          self.get_logger().warn(f'Unknown controller_type: {self.controller_type}, using dummy')
          return self._compute_dummy_control(state, ref_path)


   



    def _compute_dummy_control(self, state: 'np.ndarray', ref_path: 'np.ndarray') -> tuple:
        """
        Baseline proportional controller on heading + constant velocity.
        This is your current dummy controller.
        """
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
        omega = self.k_heading * heading_error  

        v = self.v_const  # constant forward speed (from parameter)

        return v, omega

    def _extract_reference_trajectory(self, current_state, ref_path):
      """Extract next N points from global path."""
      current_pos = current_state[0:2]
      distances = np.linalg.norm(ref_path[:, 0:2] - current_pos, axis=1)
      nearest_idx = np.argmin(distances)
      
      end_idx = min(nearest_idx + self.mpc_horizon, len(ref_path))
      x_ref_traj = ref_path[nearest_idx:end_idx]
      
      # Pad if needed
      if len(x_ref_traj) < self.mpc_horizon:
          last_point = ref_path[-1]
          padding = np.tile(last_point, (self.mpc_horizon - len(x_ref_traj), 1))
          x_ref_traj = np.vstack([x_ref_traj, padding])
      
      return x_ref_traj


    # def _compute_cost(self, current_state, u_seq, x_ref_traj):
    #   """Compute MPC cost WITHOUT terminal cost."""
      
    #   N = self.mpc_horizon
    #   cost = 0.0
      
    #   # Simulate full trajectory using robot model
    #   x_traj = self.robot_model.simulate_trajectory(current_state, u_seq)
      
    #   for k in range(N):
    #       # Get predicted state at step k+1
    #       x_next = x_traj[k + 1]
          
    #       # Tracking error against reference
    #       x_ref_k = x_ref_traj[k]
    #       error_state = x_next - x_ref_k
    #       error_state[2] = self._wrap_to_pi(error_state[2])
          
    #       # State tracking cost
    #       state_cost = (self.Q_x * error_state[0]**2 + 
    #                     self.Q_y * error_state[1]**2 + 
    #                     self.Q_theta * error_state[2]**2)
          
    #       # Control effort cost
    #       v_k, omega_k = u_seq[k]
    #       control_cost = self.R_v * v_k**2 + self.R_omega * omega_k**2
          
    #       cost += state_cost + control_cost
      
    #   return cost

    # THIS USED TO WORK (SO KEEP IT)
    # def _compute_mpc_control(self, state, ref_path):
    #   """MPC controller: solve optimization, return first control."""
      
    #   # Extract reference trajectory
    #   x_ref_traj = self._extract_reference_trajectory(state, ref_path)
      
    #   # Initial guess: constant forward motion
    #   u_init = np.zeros((self.mpc_horizon, 2))
    #   u_init[:, 0] = self.v_ref
    #   u_init_flat = u_init.flatten()
      
    #   # Objective function
    #   def objective(u_flat):
    #       u_seq = u_flat.reshape((self.mpc_horizon, 2))
    #       return self._compute_cost(state, u_seq, x_ref_traj)
      
    #   # Define bounds for each control
    #   bounds = []
    #   for k in range(self.mpc_horizon):
    #       bounds.append((self.v_min, self.max_v))           # v bounds
    #       bounds.append((-self.max_omega, self.max_omega))  # ω bounds
      
    #   # Solve
    #   result = minimize(
    #       objective,
    #       u_init_flat,
    #       method='SLSQP',
    #       bounds=bounds,
    #       options={'maxiter': 50}
    #   )
      
    #   u_opt = result.x.reshape((self.mpc_horizon, 2))
      
    #   # Return first control input only (receding horizon)
    #   return u_opt[0, 0], u_opt[0, 1]

    def _compute_mpc_control(self, state, ref_path):
      """
      MPC controller WITHOUT obstacle avoidance (for testing/baseline).
      Optimizes both v and omega over the horizon to track reference path.
      """
      if ref_path is None:
          return 0.0, 0.0
      
      H = self.mpc_horizon
      targets = self._extract_reference_trajectory(state, ref_path)
      
      # Decision variables: [v_0, omega_0, v_1, omega_1, ..., v_{H-1}, omega_{H-1}]
      u0 = np.zeros(2 * H)
      u0[::2] = self.v_ref  # Initialize v values to v_ref
      
      def total_cost(u_flat):
          x, y, th = state
          cost = 0.0
          
          for k in range(H):
              v_k = float(u_flat[2*k])
              omega_k = float(u_flat[2*k + 1])
              
              # Dynamics (same as mpc_avoid)
              x += v_k * math.cos(th) * self.dt
              y += v_k * math.sin(th) * self.dt
              th = self._wrap_to_pi(th + omega_k * self.dt)
              
              # Tracking cost
              dx, dy = x - targets[k,0], y - targets[k,1]
              dth = self._wrap_to_pi(th - targets[k,2])
              cost += self.Q_x*dx*dx + self.Q_y*dy*dy + self.Q_theta*dth*dth
              
              # Control effort
              cost += self.R_v * (v_k - self.v_ref)**2 + self.R_omega * omega_k**2
          
          # NO obstacle cost (difference from mpc_avoid)
          return cost
      
      # Bounds: alternating v and omega
      bounds = [(self.v_min, self.max_v), (-self.max_omega, self.max_omega)] * H
      
      res = minimize(total_cost, u0, bounds=bounds, method='SLSQP', options={'maxiter': self.optimizer_maxiter})
      
      if res.success:
          v_cmd = float(res.x[0])
          omega_cmd = float(res.x[1])
      else:
          v_cmd, omega_cmd = 0.0, 0.0
      
      return v_cmd, omega_cmd


        
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


    def _compute_control_mpc_avoid(self, state, path_np):
      if path_np is None:
          return 0.0, 0.0
      if not self.obstacles:
          # No obstacles detected, use standard MPC
          return self._compute_mpc_control(state, path_np)     
       
      H = self.mpc_horizon
      targets = self._extract_reference_trajectory(state, path_np)
      
      # Decision variables: [v_0, omega_0, v_1, omega_1, ..., v_{H-1}, omega_{H-1}]
      u0 = np.zeros(2 * H)
      u0[::2] = self.v_ref  # Initialize v values to v_ref
      
      def total_cost(u_flat):
          x, y, th = state
          traj_xy = []
          cost = 0.0
          
          for k in range(H):
              v_k = float(u_flat[2*k])
              omega_k = float(u_flat[2*k + 1])
              
              # Dynamics
              x += v_k * math.cos(th) * self.dt
              y += v_k * math.sin(th) * self.dt
              th = self._wrap_to_pi(th + omega_k * self.dt)
              traj_xy.append((x, y))
              
              # Tracking cost
              dx, dy = x - targets[k,0], y - targets[k,1]
              dth = self._wrap_to_pi(th - targets[k,2])
              cost += self.Q_x*dx*dx + self.Q_y*dy*dy + self.Q_theta*dth*dth
              
              # Control effort
              # cost += self.R_v * v_k**2 + self.R_omega * omega_k**2
              cost += self.R_v * (v_k - self.v_ref)**2 + self.R_omega * omega_k**2
          
          # Obstacle cost
          cost += self._obstacle_cost(traj_xy)
          return cost
      
      # Bounds: alternating v and omega
      bounds = [(self.v_min, self.max_v), (-self.max_omega, self.max_omega)] * H
      
      res = minimize(total_cost, u0, bounds=bounds, method='SLSQP', options={'maxiter': self.optimizer_maxiter})
      
      if res.success:
          v_cmd = float(res.x[0])
          omega_cmd = float(res.x[1])
      else:
          v_cmd, omega_cmd = 0.0, 0.0
      
      return v_cmd, omega_cmd
    
    def _obstacle_cost(self, traj_xy):
      cost = 0.0
      d_safe = self.d_safe
      rr = self.robot_radius

      for x, y in traj_xy:
          
          for xo, yo, ro in self.obstacles:
              
              dist = math.hypot(x - xo, y - yo)
              min_clear = d_safe + rr + ro

              if dist < min_clear:
                  cost += (min_clear - dist) ** 2

      return self.W_obstacle * cost

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