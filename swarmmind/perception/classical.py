"""Classical victim detector: colour-band discrimination + block pooling.

**This ships by default.** Pure numpy, no ML runtime, no install, deterministic. It is
also the baseline the trained CNN must beat at the gate (TECHNICAL.md section 8) --
same rule as everywhere else in this project.

It is a genuine detector, not a lookup: it works on noisy pixels, it is fooled by
warm-tinted rubble, and it misses victims that are small, distant, or half-buried.
"""

from __future__ import annotations

import numpy as np

from .detector import Detection
from .raster import COLOR_HAZARD, COLOR_VICTIM


class ClassicalVictimDetector:
    name = "classical"

    def __init__(self, *, block: int = 6, min_fill: float = 0.34, max_wide: float = 0.55,
                 warm_lo: int = 118, warm_hi: int = 205, red_lo: int = 168) -> None:
        self.block = block
        self.min_fill = min_fill
        #: Reject a warm block whose whole neighbourhood is also warm. A victim is a
        #: compact blob; a warm-rubble field is broad. Colour alone cannot separate
        #: them -- they overlap by design -- so the classical detector uses extent.
        #: This is the cue that produces its characteristic errors: a victim lying in
        #: the middle of warm rubble is missed, and a small isolated warm patch is
        #: reported as a casualty.
        self.max_wide = max_wide
        # Warmth band (R - B). A victim sits between warm rubble below and the hazard
        # above; both bounds matter. Hazard is *warmer* than a victim, so an upper
        # bound is what stops the swarm reporting the fire as a casualty.
        self.warm_lo, self.warm_hi = warm_lo, warm_hi
        self.red_lo = red_lo

    def detect(self, world, rig, frames: np.ndarray, idx: np.ndarray) -> list[Detection]:
        if len(idx) == 0:
            return []
        f = frames.astype(np.int16)
        warmth = f[..., 0] - f[..., 2]
        mask = (warmth >= self.warm_lo) & (warmth <= self.warm_hi) & (f[..., 0] >= self.red_lo)

        b = self.block
        k, h, w = mask.shape
        hh, ww = h // b, w // b
        pooled = mask[:, : hh * b, : ww * b].reshape(k, hh, b, ww, b).sum(axis=(2, 4))
        fill = pooled / float(b * b)

        # 3x3 neighbourhood mean of the block fills, edge-replicated.
        pad = np.pad(fill, ((0, 0), (1, 1), (1, 1)), mode="edge")
        wide = sum(
            pad[:, dy : dy + hh, dx : dx + ww] for dy in range(3) for dx in range(3)
        ) / 9.0

        hits = np.argwhere((fill >= self.min_fill) & (wide <= self.max_wide))

        out: list[Detection] = []
        for ki, by, bx in hits:
            row = (by + 0.5) * b
            col = (bx + 0.5) * b
            robot = int(idx[ki])
            x, y, d, bearing = rig.to_world(world, robot, row, col)
            out.append(
                Detection(
                    robot=robot,
                    pos=(float(x), float(y)),
                    range_m=float(d),
                    bearing=float(bearing),
                    conf=float(min(1.0, fill[ki, by, bx])),
                )
            )
        return out


__all__ = ["ClassicalVictimDetector", "COLOR_VICTIM", "COLOR_HAZARD"]
