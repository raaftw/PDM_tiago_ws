#!/usr/bin/env python3
"""
FK from URDF:
- Parses joints, builds chain base_link -> eef_link
- Computes FK pose for given joint angles
- Supports revolute/continuous and prismatic joints

Usage:
    x, y, z = fk_tiago_arm(urdf_text, base_link, eef_link, arm_joint_order, q)
"""

from __future__ import annotations
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np


# -----------------------------
# Basic transforms
# -----------------------------
def trans_xyz(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = np.array([x, y, z], dtype=float)
    return T

def rot_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ], dtype=float)
    T = np.eye(4)
    T[:3, :3] = R
    return T

def rot_axis(axis: np.ndarray, theta: float) -> np.ndarray:
    ax = np.array(axis, dtype=float)
    n = np.linalg.norm(ax)
    if n < 1e-12:
        ax = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        ax = ax / n
    x, y, z = ax
    c, s = math.cos(theta), math.sin(theta)
    C = 1.0 - c
    R = np.array([
        [c + x*x*C,     x*y*C - z*s,  x*z*C + y*s],
        [y*x*C + z*s,   c + y*y*C,    y*z*C - x*s],
        [z*x*C - y*s,   z*y*C + x*s,  c + z*z*C],
    ], dtype=float)
    T = np.eye(4)
    T[:3, :3] = R
    return T


# -----------------------------
# URDF parsing
# -----------------------------
@dataclass
class JointURDF:
    name: str
    parent: str
    child: str
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    axis: Tuple[float, float, float]
    joint_type: str  # 'revolute', 'continuous', 'prismatic', 'fixed'

def parse_triplet(s: Optional[str]) -> Tuple[float, float, float]:
    if not s:
        return (0.0, 0.0, 0.0)
    vals = [float(v) for v in s.split()]
    # URDF uses radians in rpy and meters in xyz
    if len(vals) != 3:
        return (0.0, 0.0, 0.0)
    return (vals[0], vals[1], vals[2])

def load_urdf_joints(urdf_text: str) -> Dict[str, JointURDF]:
    root = ET.fromstring(urdf_text)
    joints: Dict[str, JointURDF] = {}
    for j in root.findall("joint"):
        name = j.attrib.get("name", "")
        parent = j.find("parent").attrib["link"]
        child = j.find("child").attrib["link"]
        origin = j.find("origin")
        xyz = parse_triplet(origin.attrib.get("xyz")) if origin is not None else (0.0, 0.0, 0.0)
        rpy = parse_triplet(origin.attrib.get("rpy")) if origin is not None else (0.0, 0.0, 0.0)
        axis_el = j.find("axis")
        axis = parse_triplet(axis_el.attrib.get("xyz")) if axis_el is not None else (0.0, 0.0, 0.0)
        joint_type = j.attrib.get("type", "fixed")
        joints[name] = JointURDF(
            name=name,
            parent=parent,
            child=child,
            origin_xyz=xyz,
            origin_rpy=rpy,
            axis=axis,
            joint_type=joint_type,
        )
    return joints


# -----------------------------
# Chain construction
# -----------------------------
def build_chain(joints: Dict[str, JointURDF], base_link: str, eef_link: str) -> List[JointURDF]:
    # parent_link -> outgoing joints
    adj: Dict[str, List[JointURDF]] = {}
    for j in joints.values():
        adj.setdefault(j.parent, []).append(j)

    chain: List[JointURDF] = []
    visited = set()

    def dfs(link: str) -> bool:
        if link == eef_link:
            return True
        visited.add(link)
        for j in adj.get(link, []):
            if j.child in visited:
                continue
            chain.append(j)
            if dfs(j.child):
                return True
            chain.pop()
        return False

    if not dfs(base_link):
        raise RuntimeError(f"Could not build chain from {base_link} to {eef_link}")
    return chain


# -----------------------------
# FK computation
# -----------------------------
def fk_compose(chain: List[JointURDF], q_map: Dict[str, float]) -> np.ndarray:
    T = np.eye(4)
    for j in chain:
        # fixed origin of the joint frame relative to parent link
        ox, oy, oz = j.origin_xyz
        rr, rp, ry = j.origin_rpy
        T = T @ trans_xyz(ox, oy, oz) @ rot_rpy(rr, rp, ry)

        if j.joint_type in ("revolute", "continuous"):
            theta = float(q_map.get(j.name, 0.0))
            T = T @ rot_axis(np.array(j.axis, dtype=float), theta)
        elif j.joint_type == "prismatic":
            d = float(q_map.get(j.name, 0.0))
            ax = np.array(j.axis, dtype=float)
            n = np.linalg.norm(ax)
            if n < 1e-12:
                ax = np.array([0.0, 0.0, 1.0], dtype=float)
            else:
                ax = ax / n
            T = T @ trans_xyz(*(ax * d))
        elif j.joint_type == "fixed":
            # nothing to add beyond origin
            pass
        else:
            # unknown treated as fixed
            pass
    return T

def fk_position(chain: List[JointURDF], q_map: Dict[str, float]) -> Tuple[float, float, float]:
    T = fk_compose(chain, q_map)
    pos = T[:3, 3]
    return float(pos[0]), float(pos[1]), float(pos[2])


# -----------------------------
# Convenience wrapper
# -----------------------------
def fk_tiago_arm(urdf_text: str,
                 base_link: str,
                 eef_link: str,
                 arm_joint_order: List[str],
                 q: List[float]) -> Tuple[float, float, float]:
    joints = load_urdf_joints(urdf_text)
    chain = build_chain(joints, base_link=base_link, eef_link=eef_link)
    q_map = {name: float(val) for name, val in zip(arm_joint_order, q)}
    return fk_position(chain, q_map)
