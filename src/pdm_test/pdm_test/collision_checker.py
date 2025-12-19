# collision_checker.py
from dataclasses import dataclass
from typing import Sequence, List

# -----------------------------
# Table model
# -----------------------------
@dataclass(frozen=True)
class TableModel:
    center_x: float = 0.75
    center_y: float = 0.0
    center_z: float = 0.25
    size_x: float = 0.5
    size_y: float = 0.5
    size_z: float = 0.5
    clearance: float = 0.02     # strict margin above top surface
    inset: float = 0.03         # inset from table edges for wiping paths

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

# -----------------------------
# Joint limits (Tiago arm)
# -----------------------------
JOINT_LIMITS: List[tuple] = [
    (0.0, 2.7489),
    (-1.5708, 1.0908),
    (-3.5343, 1.5708),
    (-0.3927, 2.3562),
    (-2.0944, 2.0944),
    (-1.4137, 1.4137),
    (-2.0944, 2.0944),
]

def within_joint_limits(q: Sequence[float]) -> bool:
    return all(lo <= qi <= hi for qi, (lo, hi) in zip(q, JOINT_LIMITS))

