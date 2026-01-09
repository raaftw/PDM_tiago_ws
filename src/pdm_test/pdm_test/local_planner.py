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
from enum import Enum
from std_srvs.srv import Trigger

class ControllerState(Enum):
    """State machine for MPC controller."""
    ACTIVE = 0          # Normal path tracking
    STUCK = 1           # Robot stuck (failure timeout)
    AT_GOAL = 2         # Robot at goal, ready for hand motion
    WAITING = 3         # Waiting for hand motion to complete


class MpcController(Node):

    def __init__(self):
        super().__init__('mpc_controller')

        self.dt = float(self.declare_parameter('dt', 0.1).value)
        
        # MPC-specific parameters
        self.Q_x = float(self.declare_parameter('Q_x', 35.0).value)
        self.Q_y = float(self.declare_parameter('Q_y', 35.0).value)

        self.R_v = float(self.declare_parameter('R_v', 0.8).value)
        self.R_omega = float(self.declare_parameter('R_omega', 0.02).value)

        self.Q_f_x = float(self.declare_parameter('Q_f_x', 80.0).value)
        self.Q_f_y = float(self.declare_parameter('Q_f_y', 80.0).value)

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
        self.exiting = False

        self.current_state = None   # numpy array [x,y,theta]
        self.reference_path = None  # numpy array (N,3)
         
        self.get_logger().info(f'Local planner initialized. MPC horizon: {self.mpc_horizon}, dt: {self.dt}s')


        # Simple goal brake
        self.brake_weight = float(self.declare_parameter('brake_weight', 25.0).value)
        self.brake_radius = float(self.declare_parameter('brake_radius', 0.4).value)



        # ============ STATE MACHINE TO TRACK STATE OF CONTROLLER ============

        # Failure tracking
        self.consecutive_failures = 0
        self.consecutive_zeros = 0
        self.failure_timeout_steps = 15  # 15 * 0.1s = 1.5s
        
        # State machine
        self.controller_state = ControllerState.ACTIVE
        
        # Goal tracking
        self.goal_pose = None  # Will store goal when path received
        self.goal_distance_threshold = 0.5  # meters
        
        # At-goal heading 
        self.target_heading_goal = 1.0  # Global frame 
        
        # Service client for hand motion 
        self.hand_motion_client = self.create_client(Trigger, '/clean_table')
        self.hand_motion_called = False



    def _control_timer_cb(self):
        """
        Periodic control loop callback.
        Implements state machine: ACTIVE -> STUCK -> AT_GOAL
        """

        # Publish zero if no state or path yet
        if self.current_state is None or self.reference_path is None:
            zero_msg = Twist()
            self.cmd_pub.publish(zero_msg)
            return
        
        # If motion finished, hold zero and stop
        if self.exiting:
            twist_msg = Twist()  # zeros
            self.cmd_pub.publish(twist_msg)
            return
        
        if self.controller_state == ControllerState.WAITING:
            twist_msg = Twist()
            self.cmd_pub.publish(twist_msg)
            return

        # ============ STATE MACHINE LOGIC ============
        
        if self.controller_state == ControllerState.ACTIVE:
            # Try to compute MPC control
            v, omega = self.compute_control(self.current_state, self.reference_path)
            
            # Track failures (Phase 1)
            if math.isnan(v) or math.isnan(omega):
                # MPC solver failed
                self.consecutive_failures += 1
                self.consecutive_zeros = 0
            elif abs(v) < 5e-2 and abs(omega) < 0.16:
                # MPC output is zero (might be stuck)
                self.consecutive_zeros += 1
                self.consecutive_failures = 0
            else:
                # Normal control output
                self.consecutive_failures = 0
                self.consecutive_zeros = 0
            
            # Check if stuck (Phase 1: timeout)
            if self.consecutive_failures > self.failure_timeout_steps or self.consecutive_zeros > self.failure_timeout_steps * 2:
                self.get_logger().warn(f'STUCK detected: failures={self.consecutive_failures}, zeros={self.consecutive_zeros}')
                self.controller_state = ControllerState.STUCK
                v, omega = 0.0, 0.0  # Stop immediately
            
        elif self.controller_state == ControllerState.STUCK:
            # Determine if at goal or need to replan (Phase 2)
            if self.goal_pose is None:
                self.get_logger().warn('STUCK but goal_pose is None, stopping')
                v, omega = 0.0, 0.0
            else:
                # Calculate distance to goal
                dist_to_goal = np.linalg.norm(self.current_state[:2] - self.goal_pose[:2])
                self.get_logger().info(f'STUCK state: distance_to_goal={dist_to_goal:.2f}m (threshold={self.goal_distance_threshold}m)')
                
                if dist_to_goal < self.goal_distance_threshold:
                    # At goal: transition to AT_GOAL
                    self.get_logger().info('Transitioning to AT_GOAL state')
                    self.controller_state = ControllerState.AT_GOAL
                    v, omega = 0.0, 0.0  # Stop
                else:
                    # Far from goal: for now, stay stopped (Phase 4 will add replanning)
                    self.get_logger().info('Stuck far from goal - replanning not yet implemented')
                    v, omega = 0.0, 0.0
        
        elif self.controller_state == ControllerState.AT_GOAL:
            # At goal: turn to target heading and call hand motion (Phase 3)
            if not self.hand_motion_called:
                # First, turn to target heading
                current_heading = self.current_state[2]
                heading_error = self._normalize_angle(self.target_heading_goal - current_heading)
                
                # Use simple proportional control to turn
                # If heading_error is large, apply omega; otherwise call hand motion
                heading_threshold = 0.05  # radians 
                
                if abs(heading_error) > heading_threshold:
                    # Still turning
                    kp_heading = 10.0  # Proportional gain for heading
                    omega = np.clip(kp_heading * heading_error, -self.max_omega, self.max_omega)
                    v = 0.0
                    self.get_logger().info(f'Turning to heading: error={heading_error:.3f}, omega={omega:.3f}')
                else:
                    # Heading aligned: call hand motion service
                    self.get_logger().info('Heading aligned! Calling hand motion service...')
                    self._call_hand_motion()
                    self.hand_motion_called = True
                    # self.exiting = True
                    v, omega = 0.0, 0.0
            else:
                # Hand motion already called, stay stopped
                v, omega = 0.0, 0.0

        # Saturate controls to avoid exceeding robot limits
        v = np.clip(v, self.v_min, self.max_v)
        omega = np.clip(omega, -self.max_omega, self.max_omega)

        # Log state
        dist_to_goal = np.linalg.norm(self.current_state[:2] - self.goal_pose[:2])
        self.get_logger().info(f'Distance to goal: {dist_to_goal:.2f}, State: x={self.current_state[0]:.2f}, y={self.current_state[1]:.2f}, theta={self.current_state[2]:.2f}, FSM={self.controller_state.name}')

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
        Path callback - extract reference trajectory and store goal.
        Resets state machine when new path arrives.
        """
        path_points = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            q = pose_stamped.pose.orientation
            theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            path_points.append([x, y, theta])

        path_array = np.array(path_points)
        
        # Store goal pose (last point in path)
        if len(path_array) > 0:
            self.goal_pose = path_array[-1]  # [x, y, theta]
            self.get_logger().info(f'Goal set to: x={self.goal_pose[0]:.2f}, y={self.goal_pose[1]:.2f}, theta={self.goal_pose[2]:.2f}')
        
        # Reset state machine when new path arrives
        self.controller_state = ControllerState.ACTIVE
        self.consecutive_failures = 0
        self.consecutive_zeros = 0
        self.hand_motion_called = False
        
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
        
        # Field of view: in robot frame (front half)
        fov_min = -np.pi / 4 
        fov_max = np.pi / 4
        
        for r in msg.ranges:
            
            if r > 0.23 and r >= msg.range_min and r <= msg.range_max:
                # Check if this point is in the front FOV
                if fov_min <= angle <= fov_max:
                    px_robot = r * np.cos(angle)
                    py_robot = r * np.sin(angle)
                    
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

        # Simple goal proximity brake: penalize speed when close
        if getattr(self, 'goal_pose', None) is not None:
            dist_goal = float(np.linalg.norm(state[:2] - self.goal_pose[:2]))
            goal_target = self.goal_pose.reshape(3, 1)
        else:
            dist_goal = float(np.linalg.norm(state[:2] - ref_traj[-1, :2]))
            goal_target = ref_traj[-1, :].reshape(3, 1)

        # Brake factor: ramps from 0 (far) to strong (close)
        brake_radius = 0.4  # meters
        if dist_goal < brake_radius:
            brake_factor = (brake_radius - dist_goal) / brake_radius  # 0→1 as you get closer
        else:
            brake_factor = 0.0

        # Add speed penalty near goal
        brake_weight = 25.0  # tune 15–40
        for k in range(N):
            cost += brake_weight * brake_factor * U[0, k]**2



        for k in range(N):
            state_error = X[:, k] - ref_traj[k, :].reshape(3, 1)
            cost += self.Q_x * state_error[0]**2
            cost += self.Q_y * state_error[1]**2

            # Base effort costs with omega relaxed near goal
            cost += self.R_v * U[0, k]**2
            cost += self.R_omega * U[1, k]**2

            # Goal brake: penalize speed when close
            cost += self.brake_weight * brake_factor * U[0, k]**2

            # Strong penalty for backward motion (v < 0)
            backward_penalty_weight = 4000.0
            neg_v = ca.fmax(0, -U[0, k])  # ReLU(-v)
            cost += backward_penalty_weight * neg_v**2



        w_heading = 5.0  # tune 5–12
        window = min(5, N)  # penalize heading for first 5 steps

        for k in range(window):
            lookahead_idx = min(k + 2, N - 1)
            dx = ref_traj[lookahead_idx, 0] - X[0, k]
            dy = ref_traj[lookahead_idx, 1] - X[1, k]
            
            norm = ca.sqrt(dx*dx + dy*dy) + 1e-9
            ux = dx / norm
            uy = dy / norm
            
            hx = ca.cos(X[2, k])
            hy = ca.sin(X[2, k])
            
            align_err = 1.0 - (hx*ux + hy*uy)
            cost += w_heading * align_err**2


        # Smoothness cost (to avoid wobbling)
        for k in range(N - 1):
            v_change = U[0, k+1] - U[0, k]
            omega_change = U[1, k+1] - U[1, k]
            cost += 0.0 * v_change**2         
            cost += 1.5 * omega_change**2     


        # Terminal cost toward actual goal
        final_error = X[:, N] - goal_target
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
        opti.subject_to(U[0, :] >= self.v_min)
        opti.subject_to(U[1, :] >= -self.max_omega)
        opti.subject_to(U[1, :] <= self.max_omega)

        d_safe = 0.35  # safety distance in meters
        d_preferred = 0.7  # preferred distance in meters
        scan_points = self._scan_points_buffer if hasattr(self, '_scan_points_buffer') else []
        

        if len(scan_points) > 0:
            # Find top closest obstacles (not just 1)
            distances_to_robot = [(np.sqrt((p[0]-state[0])**2 + (p[1]-state[1])**2), p) for p in scan_points]
            distances_to_robot.sort(key=lambda x: x[0])
            closest_n_obstacles = [p for _, p in distances_to_robot[:3]]  
            
            obstacle_penalty_weight = 60.0

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

            heading_err_val = float(sol.value(align_err))
            self.get_logger().info(f'Heading align error: {heading_err_val:.3f}')

            self.get_logger().info(f'MPC solved in {solve_ms:.1f} ms: v={v_opt:.3f}, omega={omega_opt:.3f}')
            return v_opt, omega_opt
        except Exception as e:
            self.get_logger().info(f'CasADi solve failed: {str(e)}')
            return 0.0, 0.0



    def _normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi].
        """
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def _call_hand_motion(self):
        """Call the /clean_table Trigger service for hand motion and then exit."""
        # Wait briefly for the service to be ready
        if not self.hand_motion_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/clean_table service not ready after 5s, skipping hand motion')
            self._shutdown_node()
            return

        request = Trigger.Request()
        try:
            future = self.hand_motion_client.call_async(request)
            future.add_done_callback(self._on_hand_motion_done)
            self.get_logger().info('Hand motion service called successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to call hand motion service: {str(e)}')
            self._shutdown_node()

    def _on_hand_motion_done(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Hand motion finished: success={result.success}, msg="{result.message}"')
        except Exception as e:
            self.get_logger().warn(f'Hand motion future error: {e}')
        self._shutdown_node()

    def _shutdown_node(self):
        """Stop timer, destroy node, and shutdown rclpy."""
        self.get_logger().info('Shutting down MPC node after hand motion.')
        try:
            self.control_timer.cancel()
        except Exception:
            pass
        try:
            self.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main(args=None):
    """
    Start the MpcController node.
    """
    rclpy.init(args=args)
    node = MpcController()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():  # only shut down if not already done
            rclpy.shutdown()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()