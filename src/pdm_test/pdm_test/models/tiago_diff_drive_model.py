import numpy as np
from typing import Tuple

class TiagoDifferentialDriveModel:
    """Kinematic model for TIAGo differential drive motion (base).
    
    State: [x, y, theta]
    Control: [v, omega]
    """
    
    def __init__(self, dt: float = 0.1, 
                    v_min: float = 0.0, v_max: float = 0.6, 
                    omega_min: float = -0.5, omega_max: float = 0.5):
        """dt is the time step in seconds"""
        # Store dt, state_dim=3, control_dim=2

        self.dt = dt
        self.state_dim = 3
        self.control_dim = 2

        self.v_min, self.v_max = v_min, v_max
        self.omega_min, self.omega_max = omega_min, omega_max


    def continuous_dynamics(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        """Return [dx/dt, dy/dt, dtheta/dt]
        
        Equations:
        dx/dt = v * cos(theta)
        dy/dt = v * sin(theta)
        dtheta/dt = omega
        """

        # Unpack state and control
        v = control[0]
        omega = control[1]
        theta = state[2]

        # Apply kinematic model equations
        x_dot = v * np.cos(theta)
        y_dot = v * np.sin(theta)
        theta_dot = omega

        return np.array([x_dot, y_dot, theta_dot])
    
    def discrete_step(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        """Predict next state: x_next = x + dx/dt * dt"""

        # Get derivative of the state from continuous_dynamics
        derivative = self.continuous_dynamics(state, control)

        # new x = x_prev + x_dot * dt
        next_state = state + derivative * self.dt

        # Normalize theta so it stays within -pi and pi
        next_state[2] = self._normalize_angle(next_state[2])

        return next_state
    
    def simulate_trajectory(self, initial_state: np.ndarray, control_sequence: np.ndarray) -> np.ndarray:
        """Given sequence of controls (np array), return full trajectory"""
        
        # Start with initial state
        state = initial_state
        trajectory = [state.copy()]

        # Loop through each control, call discrete_step and append the new states to the generated trajectory
        for control in control_sequence:

            state = self.discrete_step(state, control)
            trajectory.append(state.copy())
        
        return np.array(trajectory)
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Keep angle in [-π, π]"""

        while angle > np.pi: # angle > pi : subtract 2 pi
            angle -= 2 * np.pi
        while angle < -np.pi: # angle < pi : add 2 pi
            angle += 2 * np.pi
        return angle

