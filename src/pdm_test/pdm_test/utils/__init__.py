import numpy as np
from typing import Tuple

class PathGenerator:
    """Generate reference paths for MPC tracking"""
    
    @staticmethod
    def straight_line(start: Tuple[float, float], goal: Tuple[float, float], num_points: int = 100) -> np.ndarray:
        """Generate straight line between start and goal (Tuple of x,y)
        
        Returns array of shape (num_points, 2) with [x, y]
        """
        
        path = np.zeros((num_points, 2))

        path[:, 0] = np.linspace(start[0], goal[0], num_points)  # x values
        path[:, 1] = np.linspace(start[1], goal[1], num_points)  # y values

        return path
    
    @staticmethod
    def circle(center: Tuple[float, float], radius: float, num_points: int = 100, start_angle: float = 0.0, direction: str = 'ccw') -> np.ndarray:
        """Generate circular path given center, radius, start angle and direction
        
        x = center_x + radius * cos(angle)
        y = center_y + radius * sin(angle)
        
        Returns array of shape (num_points, 2) with [x, y]
        """
        
        path = np.zeros((num_points, 2))
        
        angles = np.linspace(start_angle, start_angle + (2 * np.pi if direction == 'ccw' else -2 * np.pi), num_points)
        
        path[:, 0] = center[0] + radius * np.cos(angles)  # x values
        path[:, 1] = center[1] + radius * np.sin(angles)  # y values

        return path
    
    @staticmethod
    def add_orientation_to_path(path: np.ndarray) -> np.ndarray:
        """Add theta (heading) to path
        
        For each point, calculate angle to next point using atan2
        
        Returns array of shape (num_points, 3) with [x, y, theta]
        """
        
        num_points = path.shape[0]
        path_with_theta = np.zeros((num_points, 3))
        path_with_theta[:, :2] = path
        
        for i in range(num_points):

            if i < num_points - 1:
                dx = path[i + 1, 0] - path[i, 0]
                dy = path[i + 1, 1] - path[i, 1]
            else:
                dx = path[i, 0] - path[i - 1, 0]
                dy = path[i, 1] - path[i - 1, 1]
            
            theta = np.arctan2(dy, dx)
            path_with_theta[i, 2] = theta
        
        return path_with_theta

def resample_path(path: np.ndarray, new_num_points: int) -> np.ndarray:
    """Resample path to different number of points using np.interp"""

    original_num_points = path.shape[0]
    original_indices = np.linspace(0, 1, original_num_points)
    new_indices = np.linspace(0, 1, new_num_points)

    resampled_path = np.zeros((new_num_points, path.shape[1]))

    for dim in range(path.shape[1]):
        resampled_path[:, dim] = np.interp(new_indices, original_indices, path[:, dim])

    return resampled_path