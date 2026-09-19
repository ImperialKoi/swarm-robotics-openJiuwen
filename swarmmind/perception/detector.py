"""Detector interface. Pixels in, victim reports out.

This is the boundary the project's honesty claim rests on: everything above it sees
``Detection`` objects and never the simulator's victim list. Both the classical detector
and the trained CNN implement it, so swapping them changes nothing upstream.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class Detection:
    robot: int
    pos: tuple[float, float]     # world position implied by image coords + camera geometry
    range_m: float
    bearing: float               # body-frame, radians
    conf: float


class Detector(Protocol):
    name: str

    def detect(self, world, rig, frames: np.ndarray, idx: np.ndarray) -> list[Detection]:
        """``frames`` is (K, H, W, 3) uint8 for the robots named by ``idx``."""
        ...
