"""
rrt_connect.py

RRT-Connect in joint space (7D).
Uses:
- is_state_valid(q)
- is_edge_valid(q1,q2)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence
import random
import math

from .collision_checker import JOINT_LIMITS


@dataclass
class Node:
    q: List[float]
    parent: Optional["Node"] = None


class RRTConnect:
    def __init__(
        self,
        is_state_valid: Callable[[Sequence[float]], bool],
        is_edge_valid: Callable[[Sequence[float], Sequence[float]], bool],
        step_size: float = 0.2,
        max_iters: int = 8000,
        goal_bias: float = 0.05,
    ):
        self.is_state_valid = is_state_valid
        self.is_edge_valid = is_edge_valid
        self.step_size = step_size
        self.max_iters = max_iters
        self.goal_bias = goal_bias

    def _sample(self, q_goal: Sequence[float]) -> List[float]:
        # small goal bias helps convergence
        if random.random() < self.goal_bias:
            return list(q_goal)
        return [random.uniform(lo, hi) for (lo, hi) in JOINT_LIMITS]

    @staticmethod
    def _dist(q1: Sequence[float], q2: Sequence[float]) -> float:
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(q1, q2)))

    def _nearest(self, tree: List[Node], q: Sequence[float]) -> Node:
        return min(tree, key=lambda n: self._dist(n.q, q))

    def _steer(self, q_from: Sequence[float], q_to: Sequence[float]) -> List[float]:
        d = self._dist(q_from, q_to)
        if d <= self.step_size:
            return list(q_to)
        return [a + (b - a) * (self.step_size / d) for a, b in zip(q_from, q_to)]

    def _extend(self, tree: List[Node], q_target: Sequence[float]) -> Optional[Node]:
        n_near = self._nearest(tree, q_target)
        q_new = self._steer(n_near.q, q_target)

        if not self.is_state_valid(q_new):
            return None
        if not self.is_edge_valid(n_near.q, q_new):
            return None

        n_new = Node(q=q_new, parent=n_near)
        tree.append(n_new)
        return n_new

    def _connect(self, tree: List[Node], q_target: Sequence[float]) -> Optional[Node]:
        """
        Try to connect tree to q_target by repeated extends.
        Return the last node if it got close enough.
        """
        while True:
            n_new = self._extend(tree, q_target)
            if n_new is None:
                return None
            if self._dist(n_new.q, q_target) < self.step_size:
                return n_new

    @staticmethod
    def _trace_path(n: Node) -> List[List[float]]:
        path = []
        cur = n
        while cur is not None:
            path.append(cur.q)
            cur = cur.parent
        path.reverse()
        return path

    def plan(self, q_start: Sequence[float], q_goal: Sequence[float]) -> Optional[List[List[float]]]:
        if not self.is_state_valid(q_start):
            return None
        if not self.is_state_valid(q_goal):
            return None

        Ta: List[Node] = [Node(list(q_start))]
        Tb: List[Node] = [Node(list(q_goal))]

        for _ in range(self.max_iters):
            q_rand = self._sample(q_goal)
            na = self._extend(Ta, q_rand)
            if na is not None:
                nb = self._connect(Tb, na.q)
                if nb is not None:
                    # Paths meet: Ta root -> na, Tb root -> nb
                    path_a = self._trace_path(na)
                    path_b = self._trace_path(nb)
                    path_b.reverse()  # nb -> ... -> goal
                    return path_a + path_b[1:]  # avoid duplicate join state

            Ta, Tb = Tb, Ta

        return None

