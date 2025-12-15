"""
collision_checker.py

Environment + constraints + validity checks for RRT-Connect.

We model a fixed, axis-aligned 1m x 1m x 1m table in base_link frame.
We enforce:
- Joint limits
- End-effector (wiper tip) stays above table top (simple plane constraint)
- Edge checking: all interpolated states between q1 and q2 must be valid
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence, Tuple, List
import math


# ----------------------------
# TABLE MODEL (FIXED)
# ----------------------------
@dataclass(frozen=True)
class TableModel:
    # Table is axis-aligned in base_link frame
    center_x: float = 0.8      # example: in front of robot
    center_y: float = 0.0
    center_z: float = 0.5      # center height
    size_x: float = 1.0
    size_y: float = 1.0
    size_z: float = 1.0
    clearance: float = 0.01    # 1 cm safety margin

    @property
    def top_z(self) -> float:
        return self.center_z + self.size_z / 2.0

    @property
    def x_min(self) -> float:
        return self.center_x - self.size_x / 2.0

    @property
    def x_max(self) -> float:
        return self.center_x + self.size_x / 2.0

    @property
    def y_min(self) -> float:
        return self.center_y - self.size_y / 2.0

    @property
    def y_max(self) -> float:
        return self.center_y + self.size_y / 2.0


TABLE = TableModel()

# ----------------------------
# JOINT LIMITS (coarse, safe)
# ----------------------------
# arm_1 ... arm_7 joints
JOINT_LIMITS: List[Tuple[float, float]] = [
    (-2.8,  2.8),   # arm_1
    (-1.5,  1.5),   # arm_2
    (-2.8,  2.8),   # arm_3
    (-1.5,  1.5),   # arm_4
    (-2.8,  2.8),   # arm_5
    (-2.0,  2.0),   # arm_6
    (-2.8,  2.8),   # arm_7
]


# ----------------------------
# VALIDITY CHECKS
# ----------------------------
def within_joint_limits(q: Sequence[float]) -> bool:
    if len(q) != 7:
        return False
    for qi, (lo, hi) in zip(q, JOINT_LIMITS):
        if qi < lo or qi > hi:
            return False
    return True


def table_plane_constraint(eef_xyz: Tuple[float, float, float], table: TableModel = TABLE) -> bool:
    """
    Simple collision constraint: end-effector z must be above table top.
    """
    _, _, z = eef_xyz
    return z >= (table.top_z - table.clearance)


def is_state_valid(q: Sequence[float], fk_func: Callable[[Sequence[float]], Tuple[float, float, float]]) -> bool:
    """
    Configuration validity:
    - joint limits
    - end-effector above table top plane
    """
    if not within_joint_limits(q):
        return False
    eef_xyz = fk_func(q)
    if not table_plane_constraint(eef_xyz):
        return False
    return True


def is_edge_valid(
    q1: Sequence[float],
    q2: Sequence[float],
    fk_func: Callable[[Sequence[float]], Tuple[float, float, float]],
    resolution: float = 0.05,
) -> bool:
    """
    Edge validity by interpolating in joint space.
    resolution: max per-joint step used for interpolation.
    """
    max_delta = max(abs(a - b) for a, b in zip(q1, q2))
    steps = max(2, int(max_delta / resolution) + 1)

    for i in range(steps + 1):
        t = i / steps
        q = [(1.0 - t) * a + t * b for a, b in zip(q1, q2)]
        if not is_state_valid(q, fk_func):
            return False
    return True

