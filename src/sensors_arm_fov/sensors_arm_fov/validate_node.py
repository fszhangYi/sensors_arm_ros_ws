#!/usr/bin/env python3
"""FOV validate node: CameraInfo (+ optional image) projection service."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Optional

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from sensors_arm_fov.projection import analyze_projection, apply_image_occlusion, build_export_zip
from sensors_arm_msgs.srv import FovValidate

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore


def _camera_qos():
    """Match gazebo_ros_camera (RELIABLE); fall back to sensor BEST_EFFORT."""
    try:
        from rclpy.qos import QoSHistoryPolicy, QoSProfile, ReliabilityPolicy

        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
    except Exception:
        return qos_profile_sensor_data


class FovValidateNode(Node):
    def __init__(self) -> None:
        super().__init__("sensors_arm_fov_validate")
        self.declare_parameter("config_file", "")
        self.declare_parameter("camera_info_topic", "/cell_c/d405/color/camera_info")
        self.declare_parameter("image_topic", "/cell_c/d405/color/image_raw")
        self.declare_parameter("optical_frame", "wrist_cam_d405_optical_frame")
        self.declare_parameter("tool_frame", "gripper_centor_link")
        self.declare_parameter("export_image_wait_s", 8.0)

        cfg_path = self.get_parameter("config_file").get_parameter_value().string_value
        self._cfg: Dict[str, Any] = {}
        if cfg_path:
            self._cfg = self._load_yaml(cfg_path)
        self._lock = threading.Lock()
        self._camera_info: Optional[CameraInfo] = None
        self._image: Optional[np.ndarray] = None
        self._image_stamp_s: float = 0.0
        self._cb = ReentrantCallbackGroup()
        self._export_wait_s = float(
            self.get_parameter("export_image_wait_s").get_parameter_value().double_value
        )

        info_topic = (
            self._cfg.get("camera_info_topic")
            or self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        image_topic = (
            self._cfg.get("image_topic")
            or self.get_parameter("image_topic").get_parameter_value().string_value
        )
        cam_qos = _camera_qos()
        self.create_subscription(CameraInfo, info_topic, self._on_info, cam_qos, callback_group=self._cb)
        self.create_subscription(Image, image_topic, self._on_image, cam_qos, callback_group=self._cb)
        self._srv = self.create_service(
            FovValidate, "fov/validate", self._on_validate, callback_group=self._cb
        )
        self.get_logger().info(
            "FOV validate ready (info=%s image=%s cfg_keys=%s)"
            % (info_topic, image_topic, list(self._cfg.keys()))
        )

    @staticmethod
    def _load_yaml(path: str) -> Dict[str, Any]:
        if yaml is None:
            raise RuntimeError("PyYAML required")
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        return dict(data)

    def _on_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._camera_info = msg

    def _on_image(self, msg: Image) -> None:
        # Support rgb8 / bgr8
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            img = arr.reshape((msg.height, msg.width, 3))
        except ValueError:
            return
        if msg.encoding == "bgr8":
            img = img[:, :, ::-1].copy()
        with self._lock:
            self._image = img
            self._image_stamp_s = time.time()

    def _wait_image(self, timeout_s: float) -> Optional[np.ndarray]:
        """Block briefly until a Gazebo camera frame is cached (export path)."""
        deadline = time.time() + max(0.0, float(timeout_s))
        while True:
            with self._lock:
                if self._image is not None:
                    return self._image.copy()
            if time.time() >= deadline:
                return None
            time.sleep(0.05)

    def _mode_from_request(self, req: FovValidate.Request) -> Optional[Dict[str, Any]]:
        modes = list(self._cfg.get("modes") or [])
        mid = (req.mode_id or "").strip()
        base = None
        if mid:
            for m in modes:
                if m.get("id") == mid:
                    base = dict(m)
                    break
            if base is None:
                base = {"id": mid}
        else:
            base = dict(modes[0]) if modes else {"id": "HD87", "width": 1280, "height": 720, "H_deg": 87.0, "V_deg": 58.0}

        if req.width > 0:
            base["width"] = int(req.width)
        if req.height > 0:
            base["height"] = int(req.height)
        if req.fx > 0:
            base["fx"] = float(req.fx)
        if req.fy > 0:
            base["fy"] = float(req.fy)
        if req.h_deg > 0:
            base["H_deg"] = float(req.h_deg)
        if req.v_deg > 0:
            base["V_deg"] = float(req.v_deg)

        # Prefer live CameraInfo optics when request did not set H/V/fx
        with self._lock:
            info = self._camera_info
        if info is not None and req.h_deg <= 0 and req.fx <= 0:
            base["width"] = int(info.width)
            base["height"] = int(info.height)
            if len(info.k) >= 5:
                base["fx"] = float(info.k[0])
                base["fy"] = float(info.k[4])
                base.pop("H_deg", None)
                base.pop("V_deg", None)
        return base

    def _on_validate(
        self, req: FovValidate.Request, res: FovValidate.Response
    ) -> FovValidate.Response:
        try:
            if not self._cfg.get("samples"):
                res.ok = False
                res.error = "FOV config missing samples; set config_file"
                return res
            mode = self._mode_from_request(req)
            # Samples + lens/look/right are all flange/tool-frame (gripper_centor_link).
            # Independent of fingertip opening; wrist cam is also fixed on the flange.
            result = analyze_projection(self._cfg, mode_overrides=mode)
            result["pose_source"] = "flange_tool_frame"
            result["tool_frame"] = str(
                self._cfg.get("tool_frame") or "gripper_centor_link"
            )

            primary = (mode or {}).get("id") or result.get("recommend_mode")
            if req.use_image_occlusion and primary:
                with self._lock:
                    img = None if self._image is None else self._image.copy()
                if img is not None:
                    result = apply_image_occlusion(result, img, primary)

            res.ok = True
            res.error = ""
            res.recommend_mode = str(result.get("recommend_mode") or "")
            res.result_json = json.dumps(result, ensure_ascii=False)
            if req.export_zip:
                # Export must wait for a live Gazebo frame; otherwise zip has only JSON/md.
                img = self._wait_image(self._export_wait_s)
                if img is None:
                    with self._lock:
                        img = None if self._image is None else self._image.copy()
                primary = (mode or {}).get("id") or result.get("recommend_mode")
                zbytes = build_export_zip(
                    result,
                    self._cfg,
                    arm_id=req.arm_id or self._cfg.get("arm_id", "arm-c"),
                    image_rgb=img,
                    primary_mode=str(primary) if primary else None,
                )
                # ROS2 uint8[] accepts bytes
                res.zip_bytes = zbytes
                res.zip_filename = "fov_export.zip"
                if img is None:
                    self.get_logger().warn(
                        "FOV export zip has no Gazebo frame yet (image topic empty after %.1fs)"
                        % self._export_wait_s
                    )
                    res.error = "exported without Gazebo frame (camera image not ready)"
                else:
                    self.get_logger().info(
                        "FOV export zip includes Gazebo frame %dx%d"
                        % (int(img.shape[1]), int(img.shape[0]))
                    )
            else:
                res.zip_bytes = b""
                res.zip_filename = ""
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("FOV validate failed: %s" % exc)
            res.ok = False
            res.error = str(exc)
            res.result_json = "{}"
            res.zip_bytes = b""
            res.zip_filename = ""
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FovValidateNode()
    # Multi-threaded: TF + camera subscriptions must progress while /fov/validate runs.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
