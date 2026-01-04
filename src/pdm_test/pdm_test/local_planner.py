import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
from pdm_test.models.tiago_diff_drive_model import TiagoDifferentialDriveModel
from scipy.optimize import minimize
from std_msgs.msg import LaserScan
import cvxpy as cp


class MpcController(Node):

    def __init__(self):
        super().__init__('mpc_controller')

        self.dt = float(self.declare_parameter('dt', 0.1).value)
        
        # MPC-specific parameters
        self.Q_x = float(self.declare_parameter('Q_x', 5.0).value)
        self.Q_y = float(self.declare_parameter('Q_y', 5.0).value)
        self.Q_theta = float(self.declare_parameter('Q_theta', 0.0).value)

        self.R_v = float(self.declare_parameter('R_v', 0.5).value)       
        self.R_omega = float(self.declare_parameter('R_omega', 0.2).value)

        self.Q_f_x = float(self.declare_parameter('Q_f_x', 500.0).value)
        self.Q_f_y = float(self.declare_parameter('Q_f_y', 500.0).value)
        self.Q_f_theta = float(self.declare_parameter('Q_f_theta', 0.0).value)

        self.mpc_horizon = int(self.declare_parameter('mpc_horizon', 30).value)
        self.optimizer_maxiter = int(self.declare_parameter('optimizer_maxiter', 100).value)

        # Tiago constraints
        tiago_model = TiagoDifferentialDriveModel(dt=self.dt)
        self.max_v = float(self.declare_parameter('max_v', tiago_model.v_max).value)
        self.v_min = float(self.declare_parameter('v_min', tiago_model.v_min).value)
        self.max_omega = float(self.declare_parameter('max_omega', tiago_model.omega_max).value)
        self.min_omega = float(self.declare_parameter('min_omega', tiago_model.omega_min).value)

        # Create publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscriptions to odom and path (global planner) and laser scan (obstacle avoidance)
        self.create_subscription(Odometry, '/ground_truth_odom', self._odom_cb, 10)  # to get current state (gazebo ground truth)
        self.create_subscription(Path, '/reference_path', self._path_cb, 10)  # to get the reference path
        self.create_subscription(LaserScan, '/scan_raw', self._laser_scan_cb, 10)  # to get laser scan data for obstacle avoidance

        # Create control timer
        self.create_timer(self.dt, self._control_timer_cb)

        self.current_state = None   # numpy array [x,y,theta]
        self.reference_path = None  # numpy array (N,3)
         
        self.get_logger().info(f'Local planner initialized. MPC horizon: {self.mpc_horizon}, dt: {self.dt}s')



    # ------------------------ CALLBACKS -----------------------
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

        # Saturate controls to avoid exceeding robot limits
        v = np.clip(v, self.v_min, self.max_v)
        omega = np.clip(omega, -self.max_omega, self.max_omega)

        # Create and publish twist message to cmd_vel
        twist_msg = Twist()
        twist_msg.linear.x = v
        twist_msg.angular.z = omega

        self.get_logger().debug(f'Publishing v={v:.3f}, omega={omega:.3f}')

        self.cmd_pub.publish(twist_msg)

    def _odom_cb(self, msg: Odometry):
        """
        Odometry callback to update current state.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.current_state = np.array([x, y, theta])

    def _path_cb(self, msg: Path):
        """
        Path callback to update reference global path.
        """
        path_points = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y

            # Extract yaw from quaternion
            q = pose_stamped.pose.orientation
            theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            path_points.append([x, y, theta])

        self.reference_path = np.array(path_points)


    def _laser_scan_cb(self, msg: LaserScan):
        """
        Laser scan callback - convert to Cartesian, transform to map frame.
        """
        if self.current_state is None:
            return  # Can't transform without current pose
        
        scan_points = []
        angle = msg.angle_min
        
        # Get robot's current pose
        robot_x = self.current_state[0]
        robot_y = self.current_state[1]
        robot_theta = self.current_state[2]
        
        for r in msg.ranges:
            if r > 0.3 and msg.range_min < r < msg.range_max:
                # Point in robot frame
                px_robot = r * np.cos(angle)
                py_robot = r * np.sin(angle)
                
                # Transform to map frame
                cos_theta = np.cos(robot_theta)
                sin_theta = np.sin(robot_theta)
                
                px_map = robot_x + cos_theta * px_robot - sin_theta * py_robot
                py_map = robot_y + sin_theta * px_robot + cos_theta * py_robot
                
                scan_points.append((px_map, py_map))
            
            angle += msg.angle_increment
        
        self._scan_points_buffer = scan_points

    # ------------------------ MPC CONTROL -----------------------

    def compute_control(self, state: 'np.ndarray', ref_path: 'np.ndarray') -> tuple:
        """
        Main dispatcher: choose controller based on parameter.

        Returns: (v, omega)
        """
        return self.compute_mpc_control(state, ref_path)


    def compute_mpc_control(self, state, ref_path):
        """
        Solve MPC using CasADi + IPOPT (proper nonlinear MPC solver).
        """
        import casadi as ca
        
        if ref_path is None or len(ref_path) < self.mpc_horizon:
            return 0.0, 0.0
        
        # Extract reference trajectory
        distances = np.linalg.norm(ref_path[:, :2] - state[:2], axis=1)
        closest_idx = np.argmin(distances)
        start_idx = max(0, closest_idx)
        end_idx = min(len(ref_path), start_idx + self.mpc_horizon)
        ref_traj = ref_path[start_idx:end_idx]
        
        if len(ref_traj) < self.mpc_horizon:
            last_point = ref_path[-1]
            padding = np.tile(last_point, (self.mpc_horizon - len(ref_traj), 1))
            ref_traj = np.vstack([ref_traj, padding])
        
        N = self.mpc_horizon

        # DEBUG: Check scan points
        scan_points = self._scan_points_buffer if hasattr(self, '_scan_points_buffer') else []
        if len(scan_points) > 0:
            self.get_logger().info(f'Obstacles detected: {len(scan_points)} points')
            closest_obstacle = min(scan_points, key=lambda p: np.sqrt((p[0]-state[0])**2 + (p[1]-state[1])**2))
            dist_to_closest = np.sqrt((closest_obstacle[0]-state[0])**2 + (closest_obstacle[1]-state[1])**2)
            self.get_logger().info(f'Closest obstacle: {dist_to_closest:.2f}m away at ({closest_obstacle[0]:.2f}, {closest_obstacle[1]:.2f})')
        else:
            self.get_logger().warn('No obstacles detected in scan!')
        
        # ==================== CASADI OPTIMIZATION ====================
        opti = ca.Opti()
        
        # Variables
        X = opti.variable(3, N + 1)  # States: x, y, theta
        U = opti.variable(2, N)      # Controls: v, omega
        
        # Cost function
        cost = 0
        for k in range(N):
            state_error = X[:, k] - ref_traj[k, :].reshape(3, 1)
            cost += self.Q_x * state_error[0]**2
            cost += self.Q_y * state_error[1]**2
            cost += self.Q_theta * state_error[2]**2
            cost += self.R_v * U[0, k]**2
            cost += self.R_omega * U[1, k]**2
        
        # Terminal cost
        final_error = X[:, N] - ref_traj[-1, :].reshape(3, 1)
        cost += self.Q_f_x * final_error[0]**2
        cost += self.Q_f_y * final_error[1]**2
        cost += self.Q_f_theta * final_error[2]**2
        
        opti.minimize(cost)
        
        # ==================== CONSTRAINTS ====================
        
        # Initial state
        opti.subject_to(X[:, 0] == state.reshape(3, 1))
        
        # Dynamics constraints
        for k in range(N):
            x_k = X[0, k]
            y_k = X[1, k]
            theta_k = X[2, k]
            v_k = U[0, k]
            omega_k = U[1, k]
            
            x_next = x_k + self.dt * v_k * ca.cos(theta_k)
            y_next = y_k + self.dt * v_k * ca.sin(theta_k)
            theta_next = theta_k + self.dt * omega_k
            
            opti.subject_to(X[0, k+1] == x_next)
            opti.subject_to(X[1, k+1] == y_next)
            opti.subject_to(X[2, k+1] == theta_next)
        
        # Control bounds
        opti.subject_to(U[0, :] >= self.v_min)
        opti.subject_to(U[0, :] <= self.max_v)
        opti.subject_to(U[1, :] >= -self.max_omega)
        opti.subject_to(U[1, :] <= self.max_omega)
        
        # Obstacle avoidance
        d_safe = 0.35
        scan_points = self._scan_points_buffer if hasattr(self, '_scan_points_buffer') else []
        
        if len(scan_points) > 0:
            for k in range(N + 1):
                for px, py in scan_points:
                    dist = ca.sqrt((X[0, k] - px)**2 + (X[1, k] - py)**2)
                    opti.subject_to(dist >= d_safe)
        
        # ==================== SOLVE ====================
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 200
        }
        opti.solver('ipopt', opts)
        
        try:
            sol = opti.solve()
            
            v_opt = float(sol.value(U[0, 0]))
            omega_opt = float(sol.value(U[1, 0]))
            
            return v_opt, omega_opt
        
        except Exception as e:
            self.get_logger().error(f'CasADi solve failed: {str(e)}')
            return 0.0, 0.0




def main(args=None):
    """
    Start the MpcController node.
    """
    rclpy.init(args=args)
    node = MpcController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()