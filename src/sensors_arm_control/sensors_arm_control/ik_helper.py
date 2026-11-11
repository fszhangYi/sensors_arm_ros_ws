"""Thin IK helper using hik-sensors CS66 kinematics (Python 3.8)."""

from __future__ import annotations

import math
import os
import sys
from typing import List, Optional

import numpy as np


def _ensure_sensors_path() -> None:
    sensors_src = os.environ.get("SENSORS_SRC", "/home/ubuntu/autodl-tmp/sensors/src")
    if sensors_src not in sys.path:
        sys.path.insert(0, sensors_src)


def xyzrpy_to_T(xyzrpy: List[float]) -> np.ndarray:
    """xyz (m) + rpy (rad) → 4x4."""
    if len(xyzrpy) < 3:
        raise ValueError("xyzrpy needs at least xyz")
    x, y, z = float(xyzrpy[0]), float(xyzrpy[1]), float(xyzrpy[2])
    r = float(xyzrpy[3]) if len(xyzrpy) > 3 else 0.0
    p = float(xyzrpy[4]) if len(xyzrpy) > 4 else 0.0
    yaw = float(xyzrpy[5]) if len(xyzrpy) > 5 else 0.0
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def solve_arm_ik(arm_id: str, xyzrpy: List[float], seed_rad: Optional[List[float]] = None) -> List[float]:
    _ensure_sensors_path()
    seed = list(seed_rad or [0.0] * 6)[:6]
    seed_deg = np.rad2deg(np.asarray(seed, dtype=float))
    T = xyzrpy_to_T(xyzrpy)

    if arm_id == "arm-c":
        from sensors.cs66_kin_g26562 import ik_flange  # type: ignore
        q_deg = ik_flange(
            T,
            seed_deg,
            return_details=False,
            position_tolerance_m=1e-3,
            orientation_tolerance_rad=5e-2,
        )
    elif arm_id == "arm-b":
        from sensors.cs66_kinematics import ik_flange  # type: ignore
        q_deg = ik_flange(
            T,
            seed_deg,
            return_details=False,
            position_tolerance_m=1e-3,
            orientation_tolerance_rad=5e-2,
        )
    else:
        # EC616: no guaranteed kinematics on this host — return seed
        return seed

    q = [float(math.radians(float(v))) for v in np.asarray(q_deg, dtype=float).reshape(-1)[:6]]
    return q
