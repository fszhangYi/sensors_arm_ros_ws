"""ROS-backed orchestrator surface matching sensors-arm viz API needs."""

from __future__ import annotations

import base64
import copy
import json
import math
import os
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from sensors_arm_web.rosbridge import RosbridgeClient


def _active_camera_mode(cam: dict) -> dict:
    """Pick the FOV mode the web UI is currently driving."""
    modes = list(cam.get("modes") or [])
    mid = str(cam.get("active_mode_id") or cam.get("mode_id") or "")
    if mid:
        for m in modes:
            if str((m or {}).get("id") or "") == mid:
                return dict(m or {})
    for m in modes:
        if (m or {}).get("selected") or (m or {}).get("active"):
            return dict(m or {})
    return dict(modes[0] or {}) if modes else {}


def _vec3(val, default=(0.0, 0.0, 0.0)) -> list:
    try:
        if val is None:
            return [float(default[0]), float(default[1]), float(default[2])]
        seq = list(val)
        if len(seq) < 3:
            return [float(default[0]), float(default[1]), float(default[2])]
        return [float(seq[0]), float(seq[1]), float(seq[2])]
    except Exception:
        return [float(default[0]), float(default[1]), float(default[2])]


def _mode_to_gazebo_req(arm_id: str, cam: dict) -> dict:
    """Map web camera/mode fields → SetGazeboCamera request fields."""
    mode = _active_camera_mode(cam)
    width = int(mode.get("width") or cam.get("width") or 0)
    height = int(mode.get("height") or cam.get("height") or 0)
    h_deg = float(mode.get("H_deg") or mode.get("h_deg") or cam.get("H_deg") or cam.get("h_deg") or 0.0)
    v_deg = float(mode.get("V_deg") or mode.get("v_deg") or cam.get("V_deg") or cam.get("v_deg") or 0.0)
    fx = mode.get("fx", cam.get("fx"))
    if h_deg <= 0.0 and fx not in (None, "") and width > 0:
        try:
            h_deg = 2.0 * math.degrees(math.atan(0.5 * float(width) / float(fx)))
        except Exception:
            h_deg = 0.0
    update_rate = float(cam.get("update_rate") or mode.get("update_rate") or 0.0)

    # Extrinsics: YAML/web lens is mm in tool frame; Gazebo service wants meters.
    units = str(cam.get("units") or "mm").lower()
    lens_raw = _vec3(cam.get("lens"), (0.0, 0.0, 0.0))
    scale = 0.001 if units in ("mm", "millimeter", "millimetre") else 1.0
    lens_m = [x * scale for x in lens_raw]
    look = _vec3(cam.get("look"), (0.0, 0.0, 1.0))
    right = _vec3(cam.get("right"), (1.0, 0.0, 0.0))
    up = _vec3(cam.get("up"), (0.0, 1.0, 0.0))
    apply_extrinsics = any(abs(x) > 1e-12 for x in lens_raw) or any(
        abs(x) > 1e-12 for x in look
    )

    return {
        "arm_id": arm_id,
        "mode_id": str(mode.get("id") or cam.get("mode_id") or ""),
        "width": max(0, width),
        "height": max(0, height),
        "h_deg": float(h_deg),
        "v_deg": float(v_deg),
        "update_rate": float(update_rate),
        "apply_extrinsics": bool(apply_extrinsics),
        "lens_m": lens_m,
        "look": look,
        "right": right,
        "up": up,
    }


def _load_yaml(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def _decode_rosbridge_image(msg: dict) -> Optional[Any]:
    """Decode sensor_msgs/Image from rosbridge JSON → HxWx3 RGB uint8 ndarray."""
    try:
        import numpy as np
    except Exception:
        return None
    enc = str(msg.get("encoding") or "")
    if enc not in ("rgb8", "bgr8"):
        return None
    w = int(msg.get("width") or 0)
    h = int(msg.get("height") or 0)
    raw = msg.get("data")
    if w <= 0 or h <= 0 or raw in (None, ""):
        return None
    try:
        if isinstance(raw, str):
            buf = base64.b64decode(raw)
        elif isinstance(raw, (bytes, bytearray)):
            buf = bytes(raw)
        elif isinstance(raw, list):
            buf = bytes(raw)
        else:
            return None
        arr = np.frombuffer(buf, dtype=np.uint8).reshape((h, w, 3))
        if enc == "bgr8":
            arr = arr[:, :, ::-1].copy()
        else:
            arr = arr.copy()
        return arr
    except Exception:
        return None


def _encode_rgb_frames_mp4(frames: List[Any], fps: float = 5.0) -> bytes:
    """Encode RGB frames to MP4 bytes (mp4v)."""
    import cv2  # type: ignore
    import numpy as np

    if not frames:
        raise RuntimeError("no camera frames captured")
    h, w = int(frames[0].shape[0]), int(frames[0].shape[1])
    fd, path = tempfile.mkstemp(suffix=".mp4")
    os.close(fd)
    try:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(path, fourcc, float(fps), (w, h))
        if not writer.isOpened():
            raise RuntimeError("OpenCV VideoWriter failed to open")
        for fr in frames:
            img = np.asarray(fr)
            if img.shape[0] != h or img.shape[1] != w:
                img = cv2.resize(img, (w, h))
            writer.write(img[:, :, ::-1])  # RGB → BGR
        writer.release()
        with open(path, "rb") as f:
            return f.read()
    finally:
        try:
            os.unlink(path)
        except Exception:
            pass


class Hub:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._snap: Optional[dict] = None
        self._seq = 0

    def publish(self, payload: dict) -> None:
        with self._lock:
            self._seq += 1
            data = dict(payload)
            data["_hub_seq"] = self._seq
            self._snap = data

    def snapshot(self) -> Optional[dict]:
        with self._lock:
            return copy.deepcopy(self._snap) if self._snap else None


class RosOrchestrator:
    """Subset of sensors-arm Orchestrator used by the Web UI, backed by ROS2."""

    def __init__(
        self,
        *,
        default_yaml: str,
        models_catalog: str,
        fov_yaml: str,
        rosbridge_url: str,
        mode: str = "sim",
    ) -> None:
        self.cfg_path = default_yaml
        self.models_path = models_catalog
        self.fov_yaml = fov_yaml
        self.mode = mode
        self.cfg = _load_yaml(default_yaml)
        self.models = (_load_yaml(models_catalog).get("models") or {})
        self.fov_cfg = _load_yaml(fov_yaml) if fov_yaml and os.path.isfile(fov_yaml) else {}
        self._camera_override: Dict[str, dict] = {}
        self._homes: Dict[str, List[float]] = {}
        self._connected = set()
        self._armed: Dict[str, bool] = {}
        self._joints: Dict[str, List[float]] = {}
        self._max_delta: Dict[str, float] = {}
        self._ros = RosbridgeClient(rosbridge_url)
        self.hub = Hub()
        self._loop_stop = threading.Event()
        self._viz_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._abs_ramp_status = {"enabled": True, "active": False, "message": "", "arm_id": ""}
        self._teleop = {"enabled": False}
        self._js_latest: Dict[str, Any] = {}
        # One long-lived asyncio loop for rosbridge (avoids connect/teardown per HTTP call).
        import asyncio

        self._aio_loop = asyncio.new_event_loop()
        self._aio_thread = threading.Thread(
            target=self._aio_loop.run_forever, name="ros-aio", daemon=True
        )
        self._aio_thread.start()

        for arm in self.cfg.get("arms") or []:
            aid = arm["id"]
            home = list(arm.get("home_joints_rad") or [0.0] * 6)
            if aid == "arm-c" and len(home) == 6:
                home = home + [0.0]
            self._homes[aid] = home
            self._joints[aid] = list(home)
            self._armed[aid] = False
            self._max_delta[aid] = 15.0

        # Prefer FOV YAML samples/modes when present
        if self.fov_cfg:
            for arm in self.cfg.get("arms") or []:
                if arm.get("id") == self.fov_cfg.get("arm_id", "arm-c"):
                    cam = arm.setdefault("camera", {})
                    for k in ("lens", "look", "right", "up", "modes", "samples", "camera_model", "units", "frame"):
                        if k in self.fov_cfg:
                            cam[k] = copy.deepcopy(self.fov_cfg[k])
                    cam["enabled"] = True

    def start(self) -> None:
        self._loop_stop.clear()
        self._viz_thread = threading.Thread(target=self._viz_loop, name="ros-viz", daemon=True)
        self._viz_thread.start()
        # Best-effort rosbridge subscribe in background
        threading.Thread(target=self._ros_bootstrap, name="ros-boot", daemon=True).start()

    def stop(self) -> None:
        self._loop_stop.set()
        import asyncio

        try:
            fut = asyncio.run_coroutine_threadsafe(self._ros.close(), self._aio_loop)
            fut.result(timeout=5.0)
        except Exception:
            pass
        try:
            self._aio_loop.call_soon_threadsafe(self._aio_loop.stop)
        except Exception:
            pass

    def _ros_bootstrap(self) -> None:
        import asyncio

        async def _run():
            try:
                await self._ros.connect(timeout=10.0)
                await self._ros.subscribe(
                    "/joint_states",
                    "sensor_msgs/JointState",
                    self._on_joint_states,
                )
            except Exception:
                pass

        try:
            asyncio.run_coroutine_threadsafe(_run(), self._aio_loop).result(timeout=15.0)
        except Exception:
            pass

    def _on_joint_states(self, msg: dict) -> None:
        names = list(msg.get("name") or [])
        pos = list(msg.get("position") or [])
        by_name = {str(n): float(p) for n, p in zip(names, pos)}
        with self._lock:
            self._js_latest = {"name": names, "position": pos, "t": time.time()}
            # Map onto active cell heuristically
            for aid, home in self._homes.items():
                if len(pos) >= 6 and len(names) >= 6:
                    # Prefer arm-c joint naming
                    if any(n.startswith("joint") for n in names) and aid in ("arm-b", "arm-c"):
                        q = [float(x) for x in pos[: len(home)]]
                        while len(q) < len(home):
                            q.append(home[len(q)])
                        # Prefer named gripper so 7-DoF stays in sync with Gazebo.
                        if "gripper_opening" in by_name:
                            if len(q) >= 7:
                                q[6] = by_name["gripper_opening"]
                            elif aid == "arm-c":
                                q = q[:6] + [by_name["gripper_opening"]]
                        self._joints[aid] = q
                        if aid == "arm-c" and len(self._homes.get(aid) or []) < 7:
                            self._homes[aid] = list(q if len(q) >= 7 else (q[:6] + [0.0]))

    def _call_sync(self, coro_factory, timeout: float = 30.0):
        import asyncio

        try:
            fut = asyncio.run_coroutine_threadsafe(coro_factory(), self._aio_loop)
            return fut.result(timeout=timeout)
        except Exception as exc:
            raise RuntimeError(str(exc)) from exc

    def _arm_cmd_ros(self, arm_id: str, action: str, **kwargs) -> dict:
        joints = kwargs.get("joints") or []
        jog = kwargs.get("jog_delta") or []
        speed = float(kwargs.get("speed_scale") or 1.0)
        cancel = bool(kwargs.get("cancel_abs_ramp") or False)

        def _factory():
            return self._ros.call_service(
                "/arm/command",
                "sensors_arm_msgs/ArmCommand",
                {
                    "command": {
                        "arm_id": arm_id,
                        "action": action,
                        "joints": [float(x) for x in joints],
                        "jog_delta": [float(x) for x in jog],
                        "speed_scale": speed,
                        "cancel_abs_ramp": cancel,
                    }
                },
            )

        try:
            values = self._call_sync(_factory)
            ok = bool(values.get("ok", False))
            status = {}
            try:
                status = json.loads(values.get("status_json") or "{}")
            except Exception:
                status = {}
            if ok and action == "connect":
                self._connected.add(arm_id)
            if ok and action == "disconnect":
                self._connected.discard(arm_id)
                self._armed[arm_id] = False
            if ok and action == "arm":
                self._armed[arm_id] = True
            if ok and action in ("disarm", "estop"):
                self._armed[arm_id] = False
            if status.get("joints"):
                self._joints[arm_id] = list(status["joints"])
            return {"ok": ok, "arm_id": arm_id, "message": values.get("message", ""), "status": status}
        except Exception as exc:
            # Sim fallback when ROS not up yet
            if self.mode == "sim" and action in ("connect", "disconnect", "arm", "disarm", "home", "jog", "joints", "abs_joints", "select"):
                return self._sim_fallback(arm_id, action, **kwargs)
            return {"ok": False, "arm_id": arm_id, "error": str(exc)}

    def _sim_fallback(self, arm_id: str, action: str, **kwargs) -> dict:
        if action == "connect":
            self._connected.add(arm_id)
            return {"ok": True, "arm_id": arm_id, "connected": True, "message": "sim connect"}
        if action == "disconnect":
            self._connected.discard(arm_id)
            self._armed[arm_id] = False
            return {"ok": True, "arm_id": arm_id, "connected": False}
        if action == "arm":
            self._armed[arm_id] = True
            return {"ok": True, "arm_id": arm_id}
        if action in ("disarm", "estop"):
            self._armed[arm_id] = False
            return {"ok": True, "arm_id": arm_id}
        if action == "home":
            self._joints[arm_id] = list(self._homes[arm_id])
            return {"ok": True, "arm_id": arm_id, "joints_rad": list(self._joints[arm_id])}
        if action in ("joints", "abs_joints"):
            q = list(kwargs.get("joints") or [])
            if q:
                home = self._homes[arm_id]
                if len(q) == 6 and len(home) == 7:
                    q = q + [self._joints[arm_id][6]]
                self._joints[arm_id] = [float(x) for x in q]
            return {"ok": True, "arm_id": arm_id, "joints_rad": list(self._joints[arm_id])}
        if action == "jog":
            delta = list(kwargs.get("jog_delta") or [])
            q = list(self._joints[arm_id])
            for i, d in enumerate(delta):
                if i < len(q):
                    q[i] += float(d)
            self._joints[arm_id] = q
            return {"ok": True, "arm_id": arm_id, "joints_rad": q}
        return {"ok": True, "arm_id": arm_id}

    # --- API surface ---

    def arms_descriptor(self) -> List[dict]:
        out = []
        for arm in self.cfg.get("arms") or []:
            mid = arm.get("model")
            model = dict(self.models.get(mid) or {})
            aid = arm["id"]
            cam = self.arm_camera_get(aid)
            out.append({
                "id": aid,
                "label": arm.get("label") or aid,
                "model": mid,
                "brand": model.get("brand"),
                "product": model.get("product"),
                "num_joints": model.get("num_joints", 6),
                "urdf": model.get("urdf"),
                "joint_names": model.get("joint_names"),
                "joint_offset_rad": model.get("joint_offset_rad"),
                "joint_signs": model.get("joint_signs"),
                "mirror_y": model.get("mirror_y"),
                "gripper_rpy": model.get("gripper_rpy"),
                "backend": model.get("backend"),
                "kinematics": model.get("kinematics"),
                "kinematics_profile": model.get("kinematics_profile"),
                "kinematics_supported": bool(model.get("kinematics")),
                "read_agent": arm.get("read_agent"),
                "write_agent": arm.get("write_agent"),
                "home_joints_rad": list(self._homes.get(aid) or []),
                "tcp_xyz": list(arm.get("tcp_xyz") or [0, 0, 0.18]),
                "connected": aid in self._connected,
                "robot_ip": None,
                "robot_tcp_port": None,
                "endpoint": None,
                "max_delta_deg": self._max_delta.get(aid, 15.0),
                "camera": cam,
            })
        return out

    def arm_camera_get(self, arm_id: str) -> dict:
        arm = self._arm(arm_id)
        cam = copy.deepcopy(arm.get("camera") or {"enabled": False})
        ov = self._camera_override.get(arm_id) or {}
        cam.update(ov)
        cam["arm_id"] = arm_id
        cam["enabled"] = bool(cam.get("enabled"))
        return cam

    def arm_camera_set(self, arm_id: str, patch: dict) -> dict:
        arm = self._arm(arm_id)
        if not (arm.get("camera") or {}).get("enabled"):
            return {"ok": False, "arm_id": arm_id, "error": "this cell has no camera (camera.enabled=false)"}
        clean = {k: v for k, v in (patch or {}).items() if k != "arm_id"}
        clean["enabled"] = True
        cur = dict(self._camera_override.get(arm_id) or {})
        cur.update(clean)
        self._camera_override[arm_id] = cur
        cam = self.arm_camera_get(arm_id)
        gazebo = self._apply_camera_to_gazebo(arm_id, cam)
        out = {"ok": True, "arm_id": arm_id, "camera": cam}
        if gazebo:
            out["gazebo"] = gazebo
        return out

    def _apply_camera_to_gazebo(self, arm_id: str, cam: Optional[dict] = None) -> dict:
        """Push active web FOV mode into live Gazebo CameraSensor (best-effort)."""
        if self.mode == "hardware":
            return {"ok": False, "skipped": True, "error": "hardware mode"}
        cam = cam or self.arm_camera_get(arm_id)
        req = _mode_to_gazebo_req(arm_id, cam)
        if req["h_deg"] <= 0.0 and req["width"] <= 0 and not req.get("apply_extrinsics"):
            return {"ok": False, "skipped": True, "error": "no H_deg/width/extrinsics in camera mode"}

        def _factory():
            return self._ros.call_service(
                "/fov/set_gazebo_camera",
                "sensors_arm_msgs/SetGazeboCamera",
                req,
            )

        try:
            values = self._call_sync(_factory, timeout=10.0)
            applied = {}
            try:
                applied = json.loads(values.get("applied_json") or "{}")
            except Exception:
                applied = {}
            return {
                "ok": bool(values.get("ok", False)),
                "error": values.get("error") or "",
                "applied": applied,
                "request": req,
            }
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc), "request": req}

    def _arm(self, arm_id: str) -> dict:
        for arm in self.cfg.get("arms") or []:
            if arm["id"] == arm_id:
                return arm
        raise KeyError(arm_id)

    def status(self) -> dict:
        return {
            "app": "sensors-arm-ros2",
            "config": {"site": self.cfg.get("site"), "path": self.cfg_path},
            "arms": self.arms_descriptor(),
            "agents": {},
            "sensors_site": self.cfg.get("site"),
            "sensors_dry_run": self.mode != "hardware",
            "arm_abs_ramp": dict(self._abs_ramp_status),
            "teleop_stream": dict(self._teleop),
            "connected_arms": sorted(self._connected),
        }

    def arm_connect(self, arm_id: str) -> dict:
        return self._arm_cmd_ros(arm_id, "connect")

    def arm_disconnect(self, arm_id: str) -> dict:
        return self._arm_cmd_ros(arm_id, "disconnect")

    def arm_command(
        self,
        arm_id: str,
        *,
        arm: Optional[bool] = None,
        disarm: Optional[bool] = None,
        stop: Optional[bool] = None,
        joints_rad: Optional[List[float]] = None,
        jog_joint: Optional[int] = None,
        delta_deg: Optional[float] = None,
        delta_rad: Optional[float] = None,
        cancel_abs_ramp: bool = False,
        timing: Optional[str] = None,
        duration_s: Optional[float] = None,
        **_kwargs,
    ) -> dict:
        if cancel_abs_ramp:
            return self._arm_cmd_ros(arm_id, "cancel_ramp", cancel_abs_ramp=True)
        if stop:
            return self._arm_cmd_ros(arm_id, "estop")
        if disarm:
            return self._arm_cmd_ros(arm_id, "disarm")
        if arm:
            return self._arm_cmd_ros(arm_id, "arm")
        if jog_joint is not None:
            import math
            d = delta_rad if delta_rad is not None else math.radians(float(delta_deg or 0.0))
            delta = [0.0] * 7
            delta[int(jog_joint)] = float(d)
            return self._arm_cmd_ros(arm_id, "jog", jog_delta=delta)
        if joints_rad is not None:
            action = "abs_joints" if timing or duration_s else "joints"
            return self._arm_cmd_ros(arm_id, action, joints=list(joints_rad), speed_scale=1.0)
        return {"ok": False, "arm_id": arm_id, "error": "empty command"}

    def arm_home_get(self, arm_id: str) -> dict:
        return {"ok": True, "arm_id": arm_id, "joints_rad": list(self._homes[arm_id])}

    def arm_home_set(self, arm_id: str, joints_rad: Optional[List[float]] = None) -> dict:
        if joints_rad:
            self._homes[arm_id] = list(joints_rad)
        else:
            self._homes[arm_id] = list(self._joints[arm_id])
        return self.arm_home_get(arm_id)

    def arm_home_go(self, arm_id: str, **_kwargs) -> dict:
        return self._arm_cmd_ros(arm_id, "home")

    def arm_ik(
        self,
        arm_id: str,
        xyzrpy: List[float],
        seed_joints_rad: Optional[List[float]] = None,
        **_kwargs,
    ) -> dict:
        def _factory():
            return self._ros.call_service(
                "/arm/ik",
                "sensors_arm_msgs/ArmIk",
                {
                    "arm_id": arm_id,
                    "xyzrpy": [float(x) for x in xyzrpy],
                },
            )

        try:
            values = self._call_sync(_factory)
            return {
                "ok": bool(values.get("ok")),
                "arm_id": arm_id,
                "joints_rad": list(values.get("joints") or []),
                "message": values.get("message", ""),
            }
        except Exception as exc:
            return {"ok": False, "arm_id": arm_id, "error": str(exc)}

    def arm_max_delta_get(self, arm_id: str) -> dict:
        return {"ok": True, "arm_id": arm_id, "max_delta_deg": self._max_delta.get(arm_id, 15.0)}

    def arm_max_delta_set(self, arm_id: str, max_delta_deg: float) -> dict:
        self._max_delta[arm_id] = float(max_delta_deg)
        return self.arm_max_delta_get(arm_id)

    def teleop_stream_status(self, arm_id: Optional[str] = None) -> dict:
        return dict(self._teleop)

    def teleop_stream_set(self, arm_id: str, enabled: bool = False, joints_rad=None, hz: float = 50.0) -> dict:
        self._teleop = {"enabled": bool(enabled), "arm_id": arm_id, "hz": hz}
        if joints_rad is not None:
            self._arm_cmd_ros(arm_id, "joints", joints=list(joints_rad))
        return self.teleop_stream_status(arm_id)

    def teleop_stream_setpoint(self, arm_id: str, joints_rad: List[float]) -> dict:
        return self._arm_cmd_ros(arm_id, "joints", joints=list(joints_rad))

    def teleop_stream_stop(self, arm_id: str, message: str = "") -> dict:
        self._teleop = {"enabled": False, "arm_id": arm_id, "message": message}
        return self.teleop_stream_status(arm_id)

    def arm_fov_analyze(self, arm_id: str, override: Optional[dict] = None) -> dict:
        if not self.arm_camera_get(arm_id).get("enabled"):
            return {"ok": False, "arm_id": arm_id, "error": "this cell has no camera (camera.enabled=false)"}
        gazebo_apply: dict = {}
        if override:
            set_out = self.arm_camera_set(arm_id, override)
            gazebo_apply = dict(set_out.get("gazebo") or {})
        else:
            gazebo_apply = self._apply_camera_to_gazebo(arm_id)
        cam = self.arm_camera_get(arm_id)

        # Prefer ROS FovValidate service
        mode = _active_camera_mode(cam)
        mode_id = str(mode.get("id") or "")
        if not mode_id and override and override.get("modes"):
            mode_id = str((override["modes"][0] or {}).get("id") or "")
        elif not mode_id and cam.get("modes"):
            mode_id = str(cam["modes"][0].get("id") or "")

        def _factory():
            return self._ros.call_service(
                "/fov/validate",
                "sensors_arm_msgs/FovValidate",
                {
                    "arm_id": arm_id,
                    "mode_id": mode_id,
                    "width": 0,
                    "height": 0,
                    "fx": 0.0,
                    "fy": 0.0,
                    "h_deg": 0.0,
                    "v_deg": 0.0,
                    "use_image_occlusion": False,
                    "export_zip": False,
                },
            )

        try:
            values = self._call_sync(_factory, timeout=60.0)
            if values.get("ok"):
                result = json.loads(values.get("result_json") or "{}")
                return {
                    "ok": True,
                    "arm_id": arm_id,
                    "label": self._arm(arm_id).get("label") or arm_id,
                    "result": result,
                    "camera": cam,
                    "gazebo": gazebo_apply,
                }
            return {
                "ok": False,
                "arm_id": arm_id,
                "error": values.get("error") or "fov failed",
                "gazebo": gazebo_apply,
            }
        except Exception:
            # Offline projection fallback (same math as sensors_arm_fov)
            out = self._fov_offline(arm_id, cam)
            out["gazebo"] = gazebo_apply
            return out

    def _fov_offline(self, arm_id: str, cam: dict) -> dict:
        try:
            from sensors_arm_fov.projection import analyze_projection
        except Exception:
            # Path fallback when not sourced
            import sys
            cand = Path(__file__).resolve().parents[2] / "sensors_arm_fov"
            if cand.is_dir() and str(cand) not in sys.path:
                sys.path.insert(0, str(cand.parent))
            from sensors_arm_fov.projection import analyze_projection  # type: ignore

        work = dict(cam)
        for k in ("lens", "look", "right", "up", "modes", "samples"):
            if k not in work and k in self.fov_cfg:
                work[k] = copy.deepcopy(self.fov_cfg[k])
        result = analyze_projection(work)
        return {
            "ok": True,
            "arm_id": arm_id,
            "label": self._arm(arm_id).get("label") or arm_id,
            "result": result,
            "camera": work,
            "offline": True,
        }

    def arm_fov_export(self, arm_id: str, override: Optional[dict] = None) -> dict:
        """Record wrist-cam video of gripper tips repeatedly opening/closing (no stills)."""
        if not self.arm_camera_get(arm_id).get("enabled"):
            return {"ok": False, "arm_id": arm_id, "error": "this cell has no camera (camera.enabled=false)"}
        gazebo_apply: dict = {}
        if override:
            set_out = self.arm_camera_set(arm_id, override)
            gazebo_apply = dict(set_out.get("gazebo") or {})
        else:
            gazebo_apply = self._apply_camera_to_gazebo(arm_id)

        from datetime import datetime, timezone

        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        filename = "gripper_open_close_{}_{}.mp4".format(arm_id, stamp)
        try:
            # Brief settle so Gazebo CameraSensor / CameraInfo pick up new FOV.
            time.sleep(0.35)
            blob, meta = self._record_gripper_open_close_video(arm_id)
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "arm_id": arm_id, "error": str(exc), "gazebo": gazebo_apply}
        if isinstance(meta, dict):
            meta = dict(meta)
            meta["gazebo"] = gazebo_apply
        return {
            "ok": True,
            "arm_id": arm_id,
            "filename": filename,
            "zip_bytes": blob,
            "media_type": "video/mp4",
            "has_gazebo_figures": False,
            "has_video": True,
            "result": meta,
            "gazebo": gazebo_apply,
            "warning": "" if gazebo_apply.get("ok") else str(gazebo_apply.get("error") or ""),
        }

    def _record_gripper_open_close_video(
        self,
        arm_id: str,
        *,
        cycles: int = 3,
        open_m: float = 0.033,
        step_s: float = 0.28,
    ) -> Tuple[bytes, dict]:
        """Drive gripper open/close while capturing /cell_c/d405/color/image_raw."""
        import asyncio

        image_topic = str(
            self.fov_cfg.get("image_topic") or "/cell_c/d405/color/image_raw"
        )
        with self._lock:
            q = list(self._joints.get(arm_id) or self._homes.get(arm_id) or [])
        if len(q) < 6:
            raise RuntimeError("need current arm joints before recording")
        q6 = [float(x) for x in q[:6]]
        opening0 = float(q[6]) if len(q) >= 7 else 0.0

        frames: List[Any] = []
        frame_lock = threading.Lock()
        recording = {"on": False}

        def _on_image(msg: dict) -> None:
            if not recording["on"]:
                return
            img = _decode_rosbridge_image(msg)
            if img is None:
                return
            with frame_lock:
                frames.append(img)

        async def _ensure_sub() -> None:
            await self._ros.connect(timeout=10.0)
            await self._ros.subscribe(image_topic, "sensor_msgs/Image", _on_image)

        asyncio.run_coroutine_threadsafe(_ensure_sub(), self._aio_loop).result(timeout=15.0)
        # Wait for at least one frame before animating.
        recording["on"] = True
        t0 = time.time()
        while time.time() - t0 < 5.0:
            with frame_lock:
                n0 = len(frames)
            if n0 > 0:
                break
            time.sleep(0.1)
        with frame_lock:
            if not frames:
                recording["on"] = False
                raise RuntimeError("camera image not ready (%s)" % image_topic)

        # Triangle wave on gripper_opening: closed ↔ open, several cycles.
        steps = [0.0, 0.5 * open_m, open_m, 0.5 * open_m, 0.0]
        try:
            for _ in range(max(1, int(cycles))):
                for opening in steps:
                    self._arm_cmd_ros(
                        arm_id, "joints", joints=list(q6) + [float(opening)]
                    )
                    time.sleep(float(step_s))
            time.sleep(0.4)
        finally:
            recording["on"] = False
            # Restore prior opening.
            try:
                self._arm_cmd_ros(arm_id, "joints", joints=list(q6) + [opening0])
            except Exception:
                pass

        with frame_lock:
            captured = list(frames)
        if len(captured) < 3:
            raise RuntimeError("too few frames captured (%d)" % len(captured))

        # Aim ~camera rate (Gazebo D405 often ~5 Hz).
        fps = max(3.0, min(15.0, len(captured) / max(0.5, cycles * len(steps) * step_s)))
        blob = _encode_rgb_frames_mp4(captured, fps=fps)
        meta = {
            "method": "gripper_open_close_video",
            "image_topic": image_topic,
            "cycles": int(cycles),
            "open_m": float(open_m),
            "frames": len(captured),
            "fps": float(fps),
            "size_bytes": len(blob),
        }
        return blob, meta

    def arm_fov_sync_stl(self, arm_id: str, write_yaml: bool = True) -> dict:
        return {"ok": True, "arm_id": arm_id, "message": "ROS path uses YAML extrinsics; STL sync skipped"}

    def request_shutdown(self) -> dict:
        self.stop()
        return {"ok": True}

    def _viz_payload(self) -> dict:
        frames = []
        for aid in sorted(self._connected):
            q = list(self._joints.get(aid) or [])
            frames.append({
                "agent_id": aid + "-read",
                "sensor_id": aid,
                "t_wall": time.time(),
                "seq": int(time.time() * 1000) % 1000000,
                "payload": {"joints_rad": q, "armed": self._armed.get(aid, False)},
            })
        return {
            "schema": "sensors_arm.viz.v1",
            "site": self.cfg.get("site"),
            "t_wall": time.time(),
            "arms": self.arms_descriptor(),
            "rates": {"viz_hz_target": 15.0, "agents": {}},
            "frames": frames,
            "dry_run": self.mode != "hardware",
            "arm_abs_ramp": dict(self._abs_ramp_status),
            "teleop_stream": dict(self._teleop),
            "connected_arms": sorted(self._connected),
        }

    def _viz_loop(self) -> None:
        while not self._loop_stop.wait(1.0 / 15.0):
            try:
                self.hub.publish(self._viz_payload())
            except Exception:
                pass
