"""
kinematics.py

Simplified kinematics for TIAGo arm planning (course project).
Provides:
- fk_end_effector(q): (x, y, z)
- ik_solve_xyz(target_xyz, q_seed): numerical IK using finite differences

NOTE:
This is not exact TIAGo URDF FK. It's a consistent model to:
- support collision constraints (table plane)
- support IK to map wipe points to joint goals
"""

from __future__ import annotations

from typing import Sequence, Tuple, Optional
import numpy as np
import math


# Approximate link lengths (meters) for a planar reach model
L1 = 0.35
L2 = 0.30
L3 = 0.25

# Base height and vertical "lift" controlled by q4
BASE_Z = 0.65
Z_AMP = 0.20  # z in [BASE_Z - 0.2, BASE_Z + 0.2]


def fk_end_effector(q: Sequence[float]) -> Tuple[float, float, float]:
    """
    Simplified FK:
    - q1,q2,q3 control planar reach (x,y)
    - q4 controls z (sine mapping)
    - remaining joints don't affect xyz in this simplified model
    """
    q1, q2, q3, q4, q5, q6, q7 = q

    # Planar 3-link chain
    th1 = q1
    th2 = q1 + q2
    th3 = q1 + q2 + q3

    x = L1 * math.cos(th1) + L2 * math.cos(th2) + L3 * math.cos(th3)
    y = L1 * math.sin(th1) + L2 * math.sin(th2) + L3 * math.sin(th3)

    z = BASE_Z + Z_AMP * math.sin(q4)

    return (x, y, z)


def _numeric_jacobian(q: np.ndarray, eps: float = 1e-4) -> np.ndarray:
    """
    Finite-difference Jacobian for xyz wrt joints (7x -> 3).
    """
    f0 = np.array(fk_end_effector(q.tolist()), dtype=float)
    J = np.zeros((3, 7), dtype=float)

    for i in range(7):
        qp = q.copy()
        qp[i] += eps
        fp = np.array(fk_end_effector(qp.tolist()), dtype=float)
        J[:, i] = (fp - f0) / eps

    return J


def ik_solve_xyz(
    target_xyz: Tuple[float, float, float],
    q_seed: Sequence[float],
    max_iters: int = 200,
    tol: float = 1e-3,
    damping: float = 1e-2,
) -> Optional[np.ndarray]:
    """
    Numerical IK (damped least squares) to match end-effector xyz.

    Returns:
        np.ndarray shape (7,) if success, else None
    """
    q = np.array(q_seed, dtype=float)
    target = np.array(target_xyz, dtype=float)

    for _ in range(max_iters):
        cur = np.array(fk_end_effector(q.tolist()), dtype=float)
        err = target - cur
        if np.linalg.norm(err) < tol:
            return q

        J = _numeric_jacobian(q)
        # Damped least squares: dq = J^T (J J^T + λI)^-1 err
        JJt = J @ J.T
        dq = J.T @ np.linalg.solve(JJt + damping * np.eye(3), err)

        # step limit for stability
        step = np.clip(dq, -0.2, 0.2)
        q = q + step

    return None

