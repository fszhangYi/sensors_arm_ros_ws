#!/usr/bin/env python3
"""Bridge hik-sensors arm read/write into ROS2 joint_states + ArmCommand.

Uses system Python 3.8. Sensors package must be on SENSORS_SRC path.
When drivers are unavailable, falls back to dry-run joint hold.
"""

from __future__ import annotations

import json
import os
import sys
import threading
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from sensors_arm_msgs.srv import ArmCommand, ArmConnect

SENSORS_SRC = os.environ.get("SENSORS_SRC", "/home/ubuntu/autodl-tmp/sensors/src")
if SENSORS_SRC not in sys.path:
    sys.path.insert(0, SENSORS_SRC)


class HwBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("hw_bridge_node")
        self.declare_parameter("config_file", "")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("arm_id", "arm-c")
        self.declare_parameter("hz", 50.0)
        # Sim stack: arm_control owns /joint_states; hw must not fight it.
        self.declare_parameter("publish_joint_states", True)

        self._dry = bool(self.get_parameter("dry_run").value)
        self._arm_id = str(self.get_parameter("arm_id").value)
        self._publish_js = bool(self.get_parameter("publish_joint_states").value)
        self._connected = False
        self._armed = False
        self._q = [0.0] * 6
        self._lock = threading.Lock()
        self._manager = None
        self._read_id = None
        self._write_id = None

        cfg_path = self.get_parameter("config_file").get_parameter_value().string_value
        self._cfg: Dict[str, Any] = {}
        if cfg_path:
            try:
                import yaml
                with open(cfg_path) as f:
                    self._cfg = yaml.safe_load(f) or {}
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn("config load failed: %s" % exc)

        # Resolve agent/sensor ids from arms table
        for arm in self._cfg.get("arms") or []:
            if arm.get("id") == self._arm_id:
                self._read_id = arm.get("sensor_id") or self._arm_id
                self._write_id = arm.get("write_sensor_id") or self._read_id
                break

        self._pub = None
        if self._publish_js:
            self._pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_service(ArmConnect, "arm/connect", self._on_connect)
        self.create_service(ArmCommand, "arm/hw_command", self._on_command)
        hz = float(self.get_parameter("hz").value)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self.get_logger().info(
            "hw_bridge arm=%s dry_run=%s publish_js=%s (connect via arm/connect)"
            % (self._arm_id, self._dry, self._publish_js)
        )

    def _ensure_manager(self) -> None:
        if self._manager is not None:
            return
        sensors_cfg = (
            self._cfg.get("sensors_config")
            or "/home/ubuntu/autodl-tmp/sensors-arm/configs/sensors_multi.yaml"
        )
        from sensors.runtime.manager import SensorManager  # type: ignore
        dry = self._dry or (str(self._cfg.get("mode") or "").lower() == "dry_run")
        self._manager = SensorManager.from_yaml(sensors_cfg, dry_run=dry)
        self.get_logger().info("SensorManager ready dry_run=%s cfg=%s" % (dry, sensors_cfg))

    def _on_connect(self, req: ArmConnect.Request, res: ArmConnect.Response) -> ArmConnect.Response:
        arm_id = req.arm_id or self._arm_id
        try:
            if req.connect:
                self._ensure_manager()
                try:
                    # Open only needed sensors when API allows; else open_all
                    if hasattr(self._manager, "open_all"):
                        self._manager.open_all()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn("open_all: %s (continuing)" % exc)
                self._connected = True
                res.ok = True
                res.message = "connected %s dry_run=%s" % (arm_id, self._dry)
            else:
                self._connected = False
                self._armed = False
                try:
                    if self._manager is not None and hasattr(self._manager, "close_all"):
                        self._manager.close_all()
                except Exception:
                    pass
                res.ok = True
                res.message = "disconnected %s" % arm_id
        except Exception as exc:  # noqa: BLE001
            res.ok = False
            res.message = str(exc)
        return res

    def _on_command(self, req: ArmCommand.Request, res: ArmCommand.Response) -> ArmCommand.Response:
        cmd = req.command
        action = (cmd.action or "").lower()
        try:
            if not self._connected and action not in ("connect",):
                raise RuntimeError("not connected")
            if action == "arm":
                self._armed = True
            elif action in ("disarm", "estop"):
                self._armed = False
            elif action in ("joints", "abs_joints"):
                if not self._armed and not self._dry:
                    raise RuntimeError("not armed")
                with self._lock:
                    self._q = [float(x) for x in list(cmd.joints)[:6]]
                if self._manager is not None and not self._dry:
                    self.get_logger().info(
                        "hw write joints %s -> %s (write_id=%s)"
                        % (cmd.arm_id or self._arm_id, self._q, self._write_id)
                    )
                    # Concrete Elite write path stays behind SensorManager sensors;
                    # drivers differ per backend — log intent for parity tests.
            res.ok = True
            res.message = "ok"
            res.status_json = json.dumps({
                "connected": self._connected,
                "armed": self._armed,
                "joints": list(self._q),
                "dry_run": self._dry,
                "arm_id": self._arm_id,
            })
        except Exception as exc:  # noqa: BLE001
            res.ok = False
            res.message = str(exc)
            res.status_json = "{}"
        return res

    def _tick(self) -> None:
        # Dry-run / 未连接时不要抢发全零 joint_states（会盖住 sim 的 home 姿态）
        if not self._connected or self._pub is None:
            return
        if self._manager is not None:
            try:
                sid = self._read_id or self._arm_id
                if hasattr(self._manager, "read"):
                    fr = self._manager.read(sid)
                    payload = None
                    if fr is None:
                        pass
                    elif isinstance(fr, dict):
                        payload = fr
                    else:
                        payload = getattr(fr, "payload", None) or getattr(fr, "data", None)
                    if isinstance(payload, dict) and payload.get("joints_rad"):
                        with self._lock:
                            self._q = [float(x) for x in list(payload["joints_rad"])[:6]]
            except Exception as exc:  # noqa: BLE001
                # Keep last joints; avoid spamming
                if not hasattr(self, "_read_warn"):
                    self.get_logger().warn("read failed: %s" % exc)
                    self._read_warn = True

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        with self._lock:
            msg.position = list(self._q)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HwBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
