"""The appearance raster: what the world physically looks like.

Rendered once per tick and shared by every camera (perception/camera.py). This is the
last layer that is allowed to know ground truth -- everything above the detector sees
only pixels.

**Victims deliberately have no dedicated channel.** A channel that says "victim here"
reduces detection to a threshold and makes the computer-vision claim hollow. Materials
share appearance instead: a fraction of rubble is warm-tinted and genuinely confusable
with a victim once sensor noise is applied, so false positives are real rather than
decorative.
"""

from __future__ import annotations

import numpy as np

from ..sim import grid

# Base material colours, BGR-agnostic (treated as RGB throughout).
COLOR_FLOOR = np.array([42, 44, 50], dtype=np.int16)
COLOR_WALL = np.array([150, 150, 156], dtype=np.int16)
COLOR_RUBBLE = np.array([95, 82, 72], dtype=np.int16)
#: Warm rubble: the deliberate confuser. Its warmth (R-B = 128) sits *inside* the
#: detector's victim band on purpose. At R-B = 106 it fell below the band and could
#: never fire, so every "false positive" the system reported would have been a lie.
#: Discriminating it from a victim requires shape, not colour -- which is exactly the
#: cue a trained detector should beat the classical one on.
COLOR_RUBBLE_WARM = np.array([186, 104, 58], dtype=np.int16)
COLOR_VICTIM = np.array([224, 124, 58], dtype=np.int16)
COLOR_HAZARD = np.array([255, 92, 28], dtype=np.int16)
COLOR_ROBOT = np.array([68, 132, 204], dtype=np.int16)

#: Fraction of rubble cells rendered in the confusable warm tint.
WARM_RUBBLE_FRACTION = 0.30

#: Per-cell static texture amplitude, so flat materials are not perfectly uniform.
TEXTURE_AMPLITUDE = 14


class AppearanceRaster:
    """Static terrain rendered once; dynamic elements stamped each tick."""

    def __init__(self, world) -> None:
        self.shape = world.shape
        self.cell = world.cell
        self._static = self._render_terrain(world)
        self._frame = self._static.copy()
        #: Disc offset templates by radius in cells, so stamping does not reallocate.
        self._discs: dict[int, tuple[np.ndarray, np.ndarray]] = {}

    # ------------------------------------------------------------------ terrain

    def _render_terrain(self, world) -> np.ndarray:
        rng = world.rng["map"]
        h, w = self.shape
        img = np.empty((h, w, 3), dtype=np.int16)
        img[:] = COLOR_FLOOR

        img[world.occ == grid.WALL] = COLOR_WALL

        rubble = world.occ == grid.RUBBLE
        img[rubble] = COLOR_RUBBLE
        # Warm rubble is chosen per-cell from the map stream, so it is part of the map
        # and identical across runs on the same seed.
        warm = rubble & (rng.random(self.shape) < WARM_RUBBLE_FRACTION)
        img[warm] = COLOR_RUBBLE_WARM

        texture = rng.integers(
            -TEXTURE_AMPLITUDE, TEXTURE_AMPLITUDE + 1, size=(h, w, 1), dtype=np.int16
        )
        np.add(img, texture, out=img)
        np.clip(img, 0, 255, out=img)
        return img.astype(np.uint8)

    # ------------------------------------------------------------------ per tick

    def render(self, world) -> np.ndarray:
        """Current appearance of the whole world, (H, W, 3) uint8."""
        np.copyto(self._frame, self._static)

        if world.hazard.active and world.hazard.radius > 0:
            self._stamp_disc(world.hazard.centre, world.hazard.radius, COLOR_HAZARD)

        # Robots, stamped as one batched scatter rather than a Python loop -- at 512
        # robots and 5 Hz a loop here costs ~8 ms a pass for no reason. Carried victims
        # move with their carrier and are drawn by the victim pass below, so a loaded
        # carrier reads as robot-plus-victim rather than as a bare robot.
        alive = np.nonzero(world.status <= 1)[0]
        if len(alive):
            self._stamp_discs_batch(world.pos[alive], max(1, int(round(
                max(float(world.radius.max()), self.cell * 0.6) / self.cell))))

        for v in world.victims:
            if v.state == 4:                      # RESCUED: removed from the field
                continue
            # A buried victim shows as a SMALL patch of full victim colour -- a hand
            # out of the rubble -- growing as debris is cleared.
            #
            # Blending toward rubble instead was the obvious choice and it is wrong: a
            # fully buried victim then renders as pure rubble, is invisible to every
            # camera, and can never be found -- so it can never be dug out either. Size,
            # not colour, is what a digger's progress changes.
            frac = 1.0 - float(np.clip(v.debris_remaining, 0.0, 1.0))
            self._stamp_disc(v.pos, self.cell * (0.6 + 0.9 * frac), COLOR_VICTIM)

        return self._frame

    def _stamp_discs_batch(self, centres: np.ndarray, r_cells: int) -> None:
        tpl = self._discs.get(r_cells)
        if tpl is None:
            tpl = grid.disc_template(float(r_cells))
            self._discs[r_cells] = tpl
        dy, dx = tpl
        h, w = self.shape
        cix, ciy = grid.world_to_cell(centres[:, 0], centres[:, 1], self.cell, self.shape)
        iy = np.clip(ciy[:, None] + dy[None, :], 0, h - 1)
        ix = np.clip(cix[:, None] + dx[None, :], 0, w - 1)
        self._frame[iy, ix] = COLOR_ROBOT

    def _stamp_disc(self, centre, radius_m: float, colour) -> None:
        r_cells = max(1, int(round(radius_m / self.cell)))
        tpl = self._discs.get(r_cells)
        if tpl is None:
            tpl = grid.disc_template(float(r_cells))
            self._discs[r_cells] = tpl
        dy, dx = tpl
        h, w = self.shape
        cix, ciy = grid.world_to_cell(
            np.asarray(centre[0]), np.asarray(centre[1]), self.cell, self.shape
        )
        iy = np.clip(int(ciy) + dy, 0, h - 1)
        ix = np.clip(int(cix) + dx, 0, w - 1)
        self._frame[iy, ix] = colour
