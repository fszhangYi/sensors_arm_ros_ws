#!/usr/bin/env python3
"""Arm command / IK / home services for sim (and relay to hw topics)."""

from __future__ import annotations

import json
import shutil
import subprocess
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState

from sensors_arm_control.abs_ramp import AbsRampController
from sensors_arm_msgs.srv import ArmCommand, ArmIk


DEFAULT_CELLS = {
    "arm-a": {
        "label": "Cell A · EC616",
        "joint_names": ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"],
        "home": [0.0, -0.4, 0.8, 0.0, 1.2, 0.0],
        "camera_enabled": False,
    },
    "arm-b": {
        "label": "Cell B · CS66",
        "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        "home": [0.01044, -1.64208, 2.55021, -2.47928, 1.57057, -0.01733],
        "camera_enabled": False,
    },
    "arm-c": {
        "label": "Cell C · CS66 G26562",
        "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_opening"],
        "home": [2.8695, -1.9539, 1.8019, -1.6618, 4.7475, -0.0275, 0.0],
        "camera_enabled": True,
        # Gazebo entity name when this cell is spawned alone.
        "gazebo_model": "cell_c",
    },
}


class ArmControlNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_control_node")
        self.declare_parameter("active_arm", "arm-c")
        self.declare_parameter("mode", "sim")  # sim | hardware
        self.declare_parameter("sync_gazebo", False)
        self.declare_parameter("gazebo_model_name", "")
        self.declare_parameter("gazebo_sync_hz", 5.0)
        self._mode = self.get_parameter("mode").value
        self._cells = {k: dict(v) for k, v in DEFAULT_CELLS.items()}
        self._q: Dict[str, List[float]] = {k: list(v["home"]) for k, v in self._cells.items()}
        self._armed: Dict[str, bool] = {k: False for k in self._cells}
        self._connected: Dict[str, bool] = {k: (self._mode == "sim") for k in self._cells}
        self._lock = threading.Lock()
        self._ramp = AbsRampController()
        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self._active = self.get_parameter("active_arm").value
        self.create_service(ArmCommand, "arm/command", self._on_command)
        self.create_service(ArmIk, "arm/ik", self._on_ik)
        self.create_timer(0.02, self._publish_active)

        self._sync_gazebo = bool(self.get_parameter("sync_gazebo").value) and self._mode == "sim"
        self._gz_model_override = str(self.get_parameter("gazebo_model_name").value or "").strip()
        self._gz_bin = shutil.which("gz") or "gz"
        self._gz_dirty = False
        self._gz_gen = 0
        self._gz_logged_ready = False
        self._gz_fail_logs = 0
        self._gz_last_flush_s = 0.0
        if self._sync_gazebo:
            hz = max(0.5, float(self.get_parameter("gazebo_sync_hz").value))
            # Drive Classic Gazebo via `gz joint` (ROS set_model_configuration RPC
            # does not complete on this host). Wall clock avoids sim-time stalls.
            self.create_timer(
                1.0 / hz,
                self._flush_gazebo_if_dirty,
                clock=Clock(clock_type=ClockType.SYSTEM_TIME),
            )
            self._gz_dirty = True

        self.get_logger().info(
            "arm_control_node mode=%s active=%s sync_gazebo=%s gz=%s"
            % (self._mode, self._active, self._sync_gazebo, self._gz_bin)
        )

    def _publish_active(self) -> None:
        arm_id = self._active
        cell = self._cells[arm_id]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            msg.name = list(cell["joint_names"])
            msg.position = list(self._q[arm_id])
        self._pub.publish(msg)

    def _gazebo_model_name(self, arm_id: str) -> str:
        if self._gz_model_override:
            return self._gz_model_override
        return str(self._cells[arm_id].get("gazebo_model") or "")

    @staticmethod
    def _gazebo_joint_command(names: List[str], positions: List[float]) -> Tuple[List[str], List[float]]:
        """Expand mimic gripper finger so Gazebo links stay mirrored."""
        out_n = list(names)
        out_p = [float(x) for x in positions]
        if "gripper_opening" in out_n and "gripper_finger_r" not in out_n:
            i = out_n.index("gripper_opening")
            out_n.append("gripper_finger_r")
            out_p.append(out_p[i])
        return out_n, out_p

    def _flush_gazebo_if_dirty(self) -> None:
        if not self._sync_gazebo:
            return
        with self._lock:
            if not self._gz_dirty:
                return
        self._flush_gazebo()

    def _flush_gazebo(self) -> None:
        if not self._sync_gazebo:
            return
        with self._lock:
            arm_id = self._active
            names = list(self._cells[arm_id]["joint_names"])
            positions = list(self._q[arm_id])
            self._gz_dirty = False
            self._gz_gen += 1
            gen = self._gz_gen
        model = self._gazebo_model_name(arm_id)
        if not model:
            return
        names, positions = self._gazebo_joint_command(names, positions)
        threading.Thread(
            target=self._gz_joint_worker,
            args=(gen, model, names, positions),
            name="gz-joint",
            daemon=True,
        ).start()

    def _gz_model_exists(self, model: str) -> bool:
        try:
            proc = subprocess.run(
                [self._gz_bin, "model", "-m", model, "-i"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2.0,
                check=False,
            )
            return proc.returncode == 0
        except Exception:
            return False

    def _gz_world_pause(self, pause: bool) -> None:
        """Freeze/unfreeze physics so joints stop hunting after a pose set."""
        try:
            subprocess.run(
                [
                    self._gz_bin, "world",
                    "-w", "cell_c_fov",
                    "-p", "1" if pause else "0",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2.0,
                check=False,
            )
        except Exception:
            pass

    def _gz_joint_worker(
        self,
        gen: int,
        model: str,
        names: List[str],
        positions: List[float],
    ) -> None:
        if not self._gz_model_exists(model):
            with self._lock:
                self._gz_dirty = True
            return
        # Unpause → set targets once → brief settle → pause (stops continuous PID motion).
        self._gz_world_pause(False)
        procs = []
        ok = 0
        try:
            for jn, jp in zip(names, positions):
                if gen != self._gz_gen:
                    break
                cmd = [
                    self._gz_bin, "joint",
                    "-m", model,
                    "-j", jn,
                    "--pos-t", "%.6f" % float(jp),
                    "--pos-p", "50",
                    "--pos-i", "0",
                    "--pos-d", "5",
                ]
                procs.append(subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                ))
            ok = 0
            for proc in procs:
                if gen != self._gz_gen:
                    proc.kill()
                    continue
                try:
                    if proc.wait(timeout=3.0) == 0:
                        ok += 1
                except subprocess.TimeoutExpired:
                    proc.kill()
            if gen == self._gz_gen and ok > 0:
                time.sleep(0.35)
                # Drop continuous effort so joints don't keep twitching.
                for jn in names:
                    if gen != self._gz_gen:
                        break
                    subprocess.run(
                        [self._gz_bin, "joint", "-m", model, "-j", jn, "-f", "0"],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=2.0,
                        check=False,
                    )
                if gen == self._gz_gen:
                    self._gz_world_pause(True)
        except Exception as exc:  # noqa: BLE001
            self._gz_fail_logs += 1
            if self._gz_fail_logs <= 3:
                self.get_logger().warn("gz joint failed: %s" % exc)
            with self._lock:
                self._gz_dirty = True
            return
        if gen != self._gz_gen:
            return
        if ok == 0:
            with self._lock:
                self._gz_dirty = True
            return
        if not self._gz_logged_ready:
            self._gz_logged_ready = True
            self.get_logger().info(
                "Gazebo joints set then paused (model=%s, n=%d)" % (model, ok)
            )

    def _status(self, arm_id: str) -> dict:
        cell = self._cells[arm_id]
        with self._lock:
            return {
                "arm_id": arm_id,
                "label": cell["label"],
                "connected": self._connected[arm_id],
                "armed": self._armed[arm_id],
                "joints": list(self._q[arm_id]),
                "joint_names": list(cell["joint_names"]),
                "camera_enabled": cell["camera_enabled"],
                "mode": self._mode,
                "abs_ramp": self._ramp.status(),
            }

    def _on_command(self, req: ArmCommand.Request, res: ArmCommand.Response) -> ArmCommand.Response:
        cmd = req.command
        arm_id = cmd.arm_id or self._active
        if arm_id not in self._cells:
            res.ok = False
            res.message = "unknown arm_id"
            res.status_json = "{}"
            return res
        action = (cmd.action or "").lower()
        try:
            if action == "connect":
                self._connected[arm_id] = True
            elif action == "disconnect":
                self._connected[arm_id] = False
                self._armed[arm_id] = False
            elif action == "arm":
                if not self._connected[arm_id]:
                    raise RuntimeError("not connected")
                self._armed[arm_id] = True
            elif action in ("disarm", "estop"):
                self._armed[arm_id] = False
                self._ramp.cancel(message=action)
            elif action == "cancel_ramp" or cmd.cancel_abs_ramp:
                self._ramp.cancel()
            elif action == "home":
                self._active = arm_id
                q1 = list(self._cells[arm_id]["home"])
                with self._lock:
                    q0 = list(self._q[arm_id])
                self._ramp.start(arm_id, q0, q1, lambda q: self._set_q(arm_id, q), speed_scale=1.0)
            elif action == "joints":
                self._active = arm_id
                q1 = list(cmd.joints)
                if len(q1) != len(self._cells[arm_id]["joint_names"]):
                    # allow 6-dof without gripper
                    names = self._cells[arm_id]["joint_names"]
                    if len(q1) == 6 and len(names) == 7:
                        with self._lock:
                            q1 = list(q1) + [self._q[arm_id][6]]
                    else:
                        raise RuntimeError("joints length mismatch")
                if not self._armed[arm_id] and self._mode != "sim":
                    raise RuntimeError("not armed")
                with self._lock:
                    q0 = list(self._q[arm_id])
                if abs(float(cmd.speed_scale or 1.0)) >= 0.99 and len(q1) == len(q0):
                    # treat as abs ramp when speed_scale provided via abs path
                    pass
                self._set_q(arm_id, q1)
            elif action == "abs_joints":
                self._active = arm_id
                q1 = list(cmd.joints)
                names = self._cells[arm_id]["joint_names"]
                if len(q1) == 6 and len(names) == 7:
                    with self._lock:
                        q1 = list(q1) + [self._q[arm_id][6]]
                with self._lock:
                    q0 = list(self._q[arm_id])
                self._ramp.start(
                    arm_id, q0, q1, lambda q: self._set_q(arm_id, q),
                    speed_scale=float(cmd.speed_scale or 1.0),
                )
            elif action == "jog":
                self._active = arm_id
                delta = list(cmd.jog_delta)
                with self._lock:
                    q = list(self._q[arm_id])
                for i, d in enumerate(delta):
                    if i < len(q):
                        q[i] += float(d)
                self._set_q(arm_id, q)
            elif action == "select":
                self._active = arm_id
                if self._sync_gazebo:
                    with self._lock:
                        self._gz_dirty = True
            else:
                raise RuntimeError("unknown action: %s" % action)
            res.ok = True
            res.message = "ok"
            res.status_json = json.dumps(self._status(arm_id))
        except Exception as exc:  # noqa: BLE001
            res.ok = False
            res.message = str(exc)
            res.status_json = json.dumps(self._status(arm_id))
        return res

    def _set_q(self, arm_id: str, q: List[float]) -> None:
        with self._lock:
            self._q[arm_id] = [float(x) for x in q]
            do_flush = self._sync_gazebo and arm_id == self._active
            if do_flush:
                self._gz_dirty = True
        if not do_flush:
            return
        # Throttle gz storms during abs_ramp; timer retries leftover dirty.
        now = time.monotonic()
        if now - self._gz_last_flush_s >= 0.12:
            self._gz_last_flush_s = now
            self._flush_gazebo()

    def _on_ik(self, req: ArmIk.Request, res: ArmIk.Response) -> ArmIk.Response:
        """Best-effort IK: try sensors kinematics; else reject."""
        arm_id = req.arm_id or self._active
        try:
            joints = self._solve_ik(arm_id, list(req.xyzrpy))
            res.ok = True
            res.message = "ok"
            res.joints = joints
        except Exception as exc:  # noqa: BLE001
            res.ok = False
            res.message = str(exc)
            res.joints = []
        return res

    def _solve_ik(self, arm_id: str, xyzrpy: List[float]) -> List[float]:
        from sensors_arm_control.ik_helper import solve_arm_ik

        with self._lock:
            seed = list(self._q[arm_id][:6])
        joints6 = solve_arm_ik(arm_id, xyzrpy, seed_rad=seed)
        names = self._cells[arm_id]["joint_names"]
        if len(names) == 7:
            with self._lock:
                return list(joints6) + [float(self._q[arm_id][6])]
        return list(joints6)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
