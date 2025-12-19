# kinematics.py
import math
import numpy as np
from typing import List, Optional

# -----------------------------
# Arm mount transform
# -----------------------------
ARM_MOUNT = {
    "x": 0.15505,
    "y": 0.014,
    "z": -0.151,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": -math.pi / 2.0,
}

# -----------------------------
# Joint limits
# -----------------------------
JOINT_LIMITS = [
    (0.0, 2.7489),
    (-1.5708, 1.0908),
    (-3.5343, 1.5708),
    (-0.3927, 2.3562),
    (-2.0944, 2.0944),
    (-1.4137, 1.4137),
    (-2.0944, 2.0944),
]

# -----------------------------
# Rotation helpers
# -----------------------------
def rotz(theta): 
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])

def rotx(a): 
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s,  c]])

def roty(b): 
    c, s = math.cos(b), math.sin(b)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s,0, c]])

def rpy_to_rot(r, p, y): 
    return rotz(y) @ roty(p) @ rotx(r)

# -----------------------------
# Mount transform
# -----------------------------
def mount_base_to_arm():
    R = rpy_to_rot(ARM_MOUNT["roll"], ARM_MOUNT["pitch"], ARM_MOUNT["yaw"])
    t = np.array([ARM_MOUNT["x"], ARM_MOUNT["y"], ARM_MOUNT["z"]])
    return R, t

# -----------------------------
# Forward kinematics (simplified)
# -----------------------------
def fk_end_effector(q: List[float], tip=True):
    """
    Simplified FK: approximate tool tip position.
    Replace with your actual chain if available.
    """
    # Just a toy model: linear combination of a few joints
    x = 0.3 + 0.1 * q[0]
    y = -0.2 + 0.05 * q[1]
    z = 0.5 + 0.05 * q[2]
    return (x, y, z)

# -----------------------------
# Jacobian (simplified)
# -----------------------------
def jacobian_pos(q: List[float], tip=True):
    """
    Dummy Jacobian: identity scaled.
    Replace with actual Jacobian if available.
    """
    return np.eye(3, 7)

# -----------------------------
# IK solver (damped least squares)
# -----------------------------
def ik_solve_xyz(
    target: List[float],
    q_seed: List[float],
    max_iters: int = 200,
    tol: float = 1e-4,
    damping: float = 1e-3,
    tip: bool = True,
) -> Optional[np.ndarray]:
    """
    Iterative IK solver using damped least squares.
    """
    q = np.array(q_seed, dtype=float)
    for _ in range(max_iters):
        px, py, pz = fk_end_effector(q.tolist(), tip=tip)
        err = np.array(target) - np.array([px, py, pz])
        if np.linalg.norm(err) < tol:
            return q

        J = jacobian_pos(q.tolist(), tip=tip)
        JT = J.T
        H = J @ JT + (damping**2) * np.eye(3)
        dq = JT @ np.linalg.solve(H, err)

        # Limit step size
        if np.linalg.norm(dq) > 0.1:
            dq *= 0.1 / np.linalg.norm(dq)

        q = q + dq

        # Clamp to joint limits
        for i, (lo, hi) in enumerate(JOINT_LIMITS):
            q[i] = max(lo, min(hi, q[i]))

    return None

