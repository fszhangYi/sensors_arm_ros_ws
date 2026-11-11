"""Seven-segment absolute joint ramp (ported for Python 3.8)."""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, List, Optional


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def seven_segment_alpha(t: float, T: float, jerk_frac: float = 0.10, accel_frac: float = 0.15) -> float:
    """Normalized progress alpha in [0,1] with jerk/accel segments."""
    if T <= 1e-9:
        return 1.0
    u = _clamp(t / T, 0.0, 1.0)
    j = _clamp(jerk_frac, 0.0, 0.2)
    a = _clamp(accel_frac, 0.0, 0.3)
    # Piecewise quintic-ish blend via smoothstep windows
    # Simple symmetric S-curve fallback that respects end conditions
    if u < j:
        s = u / max(j, 1e-9)
        return 0.5 * s * s * (j / max(j + a, 1e-9))
    if u > 1.0 - j:
        s = (1.0 - u) / max(j, 1e-9)
        return 1.0 - 0.5 * s * s * (j / max(j + a, 1e-9))
    # Cosine ease in middle
    return 0.5 - 0.5 * math.cos(math.pi * u)


class AbsRampController:
    def __init__(
        self,
        *,
        t_min_s: float = 0.1,
        t_max_s: float = 30.0,
        v_norm_rad_s: float = 0.02,
        jerk_seg_frac: float = 0.10,
        accel_seg_frac: float = 0.15,
        ramp_hz: float = 20.0,
    ) -> None:
        self.t_min_s = t_min_s
        self.t_max_s = t_max_s
        self.v_norm_rad_s = v_norm_rad_s
        self.jerk_seg_frac = jerk_seg_frac
        self.accel_seg_frac = accel_seg_frac
        self.ramp_hz = ramp_hz
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._active = False
        self._message = ""
        self._arm_id = ""

    def status(self):
        with self._lock:
            return {
                "enabled": True,
                "active": self._active,
                "message": self._message,
                "arm_id": self._arm_id,
            }

    def cancel(self, message: str = "cancelled"):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        with self._lock:
            self._active = False
            self._message = message
        return {"ok": True, "message": message, **self.status()}

    def start(
        self,
        arm_id: str,
        q0: List[float],
        q1: List[float],
        set_joints: Callable[[List[float]], None],
        speed_scale: float = 1.0,
    ):
        self.cancel(message="restart")
        self._stop.clear()
        d = max(abs(a - b) for a, b in zip(q0, q1)) if q0 and q1 else 0.0
        v = max(self.v_norm_rad_s * max(speed_scale, 1e-6), 1e-6)
        T = _clamp(d / v, self.t_min_s, self.t_max_s)

        def _run():
            with self._lock:
                self._active = True
                self._arm_id = arm_id
                self._message = "ramping"
            t0 = time.time()
            dt = 1.0 / max(self.ramp_hz, 1.0)
            while not self._stop.is_set():
                t = time.time() - t0
                alpha = seven_segment_alpha(t, T, self.jerk_seg_frac, self.accel_seg_frac)
                q = [a + alpha * (b - a) for a, b in zip(q0, q1)]
                set_joints(q)
                if t >= T:
                    set_joints(list(q1))
                    break
                time.sleep(dt)
            with self._lock:
                self._active = False
                self._message = "done" if not self._stop.is_set() else self._message

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()
        return {"ok": True, "T": T, **self.status()}
