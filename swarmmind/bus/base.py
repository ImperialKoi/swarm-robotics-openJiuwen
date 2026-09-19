"""The transport seam.

Nodes are transport-agnostic: they publish and subscribe by topic name and never import
a transport. ``LocalBus`` is the default and needs nothing installed; ``Ros2Bus``
(D-stretch) is a drop-in that the same nodes run on unchanged, with identical topic
names and payload shapes.

This is what makes "ROS2-compatible node graph on a pluggable transport" an accurate
description rather than a marketing line. Do not import rclpy outside bus/ros2.py.
"""

from __future__ import annotations

from typing import Any, Protocol


class Bus(Protocol):
    def publish(self, topic: str, msg: Any) -> None: ...
    def subscribe(self, topic: str, callback) -> None: ...
    def latest(self, topic: str) -> Any | None: ...
