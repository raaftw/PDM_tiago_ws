import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
from pdm_test.models.tiago_diff_drive_model import TiagoDifferentialDriveModel
from scipy.optimize import minimize
from sensor_msgs.msg import LaserScan
import time


class MpcController(Node):

    def __init__(self):
        super().__init__('mpc_controller')

        self.dt = float(self.declare_parameter('dt', 0.1).value)
        
        # MPC-specific parameters
        self.Q_x = float(self.declare_parameter('Q_x', 60.0).value)
        self.Q_y = float(self.declare_parameter('Q_y', 60.0).value)
        self.Q_theta = float(self.declare_parameter('Q_theta', 0.0).value)

        self.R_v = float(self.declare_parameter('R_v', 0.4).value)       
        self.R_omega = float(self.declare_parameter('R_omega', 0.2).value)

        self.Q_f_x = float(self.declare_parameter('Q_f_x', 50.0).value)
        self.Q_f_y = float(self.declare_parameter('Q_f_y', 50.0).value)
        self.Q_f_theta = float(self.declare_parameter('Q_f_theta', 0.0).value)

        self.mpc_horizon = int(self.declare_parameter('mpc_horizon', 15).value)
        self.optimizer_maxiter = int(self.declare_parameter('optimizer_maxiter', 200).value)

        self._scan_points_buffer = []

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

        self.get_logger().info(f'AAA State: x={self.current_state[0]:.2f}, y={self.current_state[1]:.2f}, theta={self.current_state[2]:.2f}')

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
        path_points = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            q = pose_stamped.pose.orientation
            theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            path_points.append([x, y, theta])

        path_array = np.array(path_points)
        
        # First interpolate to get a smoother path
        if len(path_array) > 1:
            path_array = self._interpolate_path(path_array, points_per_segment=3)
        
        # Then pad to exactly mpc_horizon waypoints
        if len(path_array) < self.mpc_horizon:
            last_point = path_array[-1]
            padding = np.tile(last_point, (self.mpc_horizon - len(path_array), 1))
            path_array = np.vstack([path_array, padding])
        
        self.reference_path = path_array[:self.mpc_horizon]  # Trim if too long

    def _interpolate_path(self, path, points_per_segment=5):
        """Interpolate path to create more waypoints between RRT* waypoints."""
        if len(path) < 2:
            return path
        
        interpolated = [path[0]]
        
        for i in range(len(path) - 1):
            current = path[i]
            next_point = path[i + 1]
            
            for j in range(1, points_per_segment):
                t = j / points_per_segment
                interp_point = current + t * (next_point - current)
                interpolated.append(interp_point)
            
            interpolated.append(next_point)
        
        return np.array(interpolated)


    def _laser_scan_cb(self, msg: LaserScan):
        """
        Laser scan callback - convert to Cartesian, transform to map frame.
        """
        
        self.get_logger().debug(f'Laser scan received: {len(msg.ranges)} ranges, range_min={msg.range_min:.2f}, range_max={msg.range_max:.2f}')
        
        if self.current_state is None:
            self.get_logger().debug('Cannot process laser scan: current_state is None')
            return  # Can't transform without current pose
        
        scan_points = []
        angle = msg.angle_min
        valid_count = 0
        
        # Get robot's current pose
        robot_x = self.current_state[0]
        robot_y = self.current_state[1]
        robot_theta = self.current_state[2]
        
        for r in msg.ranges:
            
            if r > 0.27 and r >= msg.range_min and r <= msg.range_max:
                # Point in robot frame
                px_robot = r * np.cos(angle)
                py_robot = r * np.sin(angle)
                
                # Transform to map frame
                cos_theta = np.cos(robot_theta)
                sin_theta = np.sin(robot_theta)
                
                px_map = robot_x + cos_theta * px_robot - sin_theta * py_robot
                py_map = robot_y + sin_theta * px_robot + cos_theta * py_robot
                
                scan_points.append((px_map, py_map))
                valid_count += 1
            
            angle += msg.angle_increment
        
        self._scan_points_buffer = scan_points
        self.get_logger().debug(f'Laser scan processed: {valid_count} valid points converted to obstacles')

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
            self.get_logger().warn(f'Path too short: len={len(ref_path) if ref_path is not None else "None"}, need {self.mpc_horizon}')
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
            self.get_logger().debug(f'Obstacles detected: {len(scan_points)} points')
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

            # cost += self.R_v * U[0, k]**2

            backward_penalty = 100.0  # Factor to penalize backward motion
            cost += self.R_v * U[0, k]**2  # Base velocity cost
            cost += 10.0 * (U[0, k] - 0.5)**2  # penalize deviation from v_ref

            # cost += backward_penalty * ca.fmax(0, -U[0, k])**2  # Extra penalty when v < 0

            cost += self.R_omega * U[1, k]**2
        
        # Heading alignment cost (so it doesnt drive backwards)
        for k in range(N-1):
            dx = ref_traj[k+1,0] - ref_traj[k,0]
            dy = ref_traj[k+1,1] - ref_traj[k,1]
            ref_heading = ca.atan2(dy, dx + 1e-6)
            heading_error = X[2,k] - ref_heading
            cost += 8.0 * heading_error**2 

        # Smoothness cost (to avoid wobbling)
        for k in range(N - 1):
            v_change = U[0, k+1] - U[0, k]
            omega_change = U[1, k+1] - U[1, k]
            cost += 0.0 * v_change**2         # tune 0.5–2.0
            cost += 2.0 * omega_change**2     # tune 2.0–6.0

        # Terminal cost
        final_error = X[:, N] - ref_traj[-1, :].reshape(3, 1)
        cost += self.Q_f_x * final_error[0]**2
        cost += self.Q_f_y * final_error[1]**2
        
        
        
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

        d_safe = 0.35  # safety distance in meters
        d_preferred = 0.6  # preferred distance in meters
        scan_points = self._scan_points_buffer if hasattr(self, '_scan_points_buffer') else []
        

        if len(scan_points) > 0:
            # Find top closest obstacles (not just 1)
            distances_to_robot = [(np.sqrt((p[0]-state[0])**2 + (p[1]-state[1])**2), p) for p in scan_points]
            distances_to_robot.sort(key=lambda x: x[0])
            closest_n_obstacles = [p for _, p in distances_to_robot[:3]]  # Top 3 closest
            
            obstacle_penalty_weight = 30.0  # tune: 5–20

            for k in range(N + 1):
                min_dist = None
                for px, py in closest_n_obstacles:
                    dist = ca.sqrt((X[0, k] - px)**2 + (X[1, k] - py)**2)
                    if min_dist is None:
                        min_dist = dist
                    else:
                        min_dist = ca.fmin(min_dist, dist)
                
                if min_dist is not None:
                    opti.subject_to(min_dist >= d_safe)

                    # Soft penalty for preferred distance
                    violation = ca.fmax(0, d_preferred - min_dist)
                    cost += obstacle_penalty_weight * violation**2
        
        

        # ==================== SOLVE ====================

        opti.minimize(cost)

        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': self.optimizer_maxiter
        }
        opti.solver('ipopt', opts)
        

        try:
            t0 = time.time()
            sol = opti.solve()
            solve_ms = (time.time() - t0) * 1000.0

            v_opt = float(sol.value(U[0, 0]))
            omega_opt = float(sol.value(U[1, 0]))

            self.get_logger().info(f'MPC solved in {solve_ms:.1f} ms: v={v_opt:.3f}, omega={omega_opt:.3f}')
            return v_opt, omega_opt
        except Exception as e:
            self.get_logger().info(f'CasADi solve failed: {str(e)}')
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