import random
import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

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
        max_iters: int = 2000,
        goal_bias: float = 0.5,
        on_add_edge: Optional[Callable[[Sequence[float], Sequence[float]], None]] = None,
    ):
        self.is_state_valid = is_state_valid
        self.is_edge_valid = is_edge_valid
        self.step_size = step_size
        self.max_iters = max_iters
        self.goal_bias = goal_bias
        self.on_add_edge = on_add_edge

    def _sample(self, q_goal: Sequence[float]) -> List[float]:
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
        alpha = self.step_size / max(d, 1e-9)
        return [a + (b - a) * alpha for a, b in zip(q_from, q_to)]

    def _extend(self, tree: List[Node], q_target: Sequence[float]) -> Optional[Node]:
        n_near = self._nearest(tree, q_target)
        q_new = self._steer(n_near.q, q_target)

        if not self.is_state_valid(q_new):
            return None
        if not self.is_edge_valid(n_near.q, q_new):
            return None

        n_new = Node(q=q_new, parent=n_near)
        tree.append(n_new)

        # <-- Visualization hook
        if self.on_add_edge is not None:
            try:
                self.on_add_edge(n_near.q, q_new)
            except Exception:
                # Never let visualization crash planning
                pass

        return n_new

    def _connect(self, tree: List[Node], q_target: Sequence[float]) -> Optional[Node]:
        while True:
            n_new = self._extend(tree, q_target)
            if n_new is None:
                return None
            if self._dist(n_new.q, q_target) < self.step_size:
                return n_new

    @staticmethod
    def _trace_path(n: Node) -> List[List[float]]:
        path: List[List[float]] = []
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
                    path_a = self._trace_path(na)
                    path_b = self._trace_path(nb)
                    path_b.reverse()
                    return path_a + path_b[1:]
            Ta, Tb = Tb, Ta

        return None

