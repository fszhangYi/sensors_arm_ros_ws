#!/usr/bin/env python3
"""Publish /joint_states (Gazebo kinematic plugin latches them)."""

from __future__ import annotations

import threading
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SimJointDriver(Node):
    def __init__(self) -> None:
        super().__init__("sim_joint_driver")
        self.declare_parameter("joints", [""])
        self.declare_parameter("positions", [0.0])
        self.declare_parameter("rate_hz", 50.0)

        joints = list(self.get_parameter("joints").value)
        positions = [float(x) for x in self.get_parameter("positions").value]
        if len(joints) != len(positions):
            raise RuntimeError("joints/positions length mismatch")
        pairs = [(j, p) for j, p in zip(joints, positions) if j]
        self._names = [j for j, _ in pairs]
        self._pos = [p for _, p in pairs]
        self._lock = threading.Lock()
        self._pub = self.create_publisher(JointState, "joint_states", 10)
        hz = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self.get_logger().info("sim_joint_driver joints=%s" % self._names)

    def set_positions(self, positions: List[float]) -> None:
        with self._lock:
            if len(positions) != len(self._pos):
                raise ValueError("bad positions length")
            self._pos = [float(x) for x in positions]

    def get_positions(self) -> List[float]:
        with self._lock:
            return list(self._pos)

    def _tick(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            msg.name = list(self._names)
            msg.position = list(self._pos)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimJointDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
