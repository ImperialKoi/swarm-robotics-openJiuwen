"""Batched egocentric cameras.

Rendering 512 separate camera views is infeasible on an M1 -- 512 robots at 5 Hz is
2,560 renders a second. The world is 2.5D top-down, so a view is an affine
crop-and-rotate **sample of the one shared raster**: a batched gather, not a render.
Cost scales with ``robots x H x W``, not with world size.

Occlusion is computed here, and that is simulating optics rather than shortcutting
perception: a real camera physically cannot see through a wall. What the detector does
with the resulting pixels is the part that must be real.
"""

from __future__ import annotations

import numpy as np

from ..sim import grid


class CameraRig:
    """One camera per robot, captured as a single batched operation."""

    def __init__(self, world, height: int = 48, width: int = 48,
                 fov_deg: float = 90.0, noise_sigma: float = 9.0,
                 near_clip: float | None = None) -> None:
        self.h, self.w = height, width
        self.fov = np.deg2rad(fov_deg)
        self.tan_half = float(np.tan(self.fov / 2.0))
        self.noise_sigma = noise_sigma
        self.cell = world.cell
        self.shape = world.shape

        # Skip the robot's own chassis. Without this the near rows photograph the
        # robot's own body disc, which wastes ~10% of every frame on a constant.
        self.near_clip = float(
            near_clip if near_clip is not None else 2.2 * float(world.radius.max())
        )

        # Local image template, identical for every robot and scaled by its own range.
        # Column = bearing. **Row 0 is the FARTHEST**, row H-1 the nearest, so a frame
        # reads like a forward-looking camera: ground near the robot at the bottom.
        f = 1.0 - (np.arange(height, dtype=np.float32) + 0.5) / height      # (H,) 1 -> 0
        u = ((np.arange(width, dtype=np.float32) + 0.5) / width - 0.5) * 2  # (W,) in [-1,1]
        self._near_frac = None      # set per robot in capture (range differs per robot)
        self._f = f[:, None]                       # (H,1) far..near interpolation
        self._d = self._f                          # kept for lateral scaling below
        self._u = u[None, :]                       # (1,W) lateral fraction
        #: Lateral offset as a fraction of range: u * d * tan(fov/2). Constant per column
        #: divided by d, which is why each column is a single fixed bearing.
        self._lat = self._u * self._d * self.tan_half
        self.column_bearing = np.arctan2(u * self.tan_half, 1.0).astype(np.float32)  # (W,)

    # ------------------------------------------------------------------ capture

    def capture(self, world, raster_img: np.ndarray,
                idx: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Frames and their per-pixel world cells.

        Returns ``(frames, cells)`` where ``frames`` is (K, H, W, 3) uint8 and ``cells``
        is (K, H, W, 2) int32 of (iy, ix) -- the caller uses the latter to mark explored
        space from what was actually seen, rather than from a geometric disc.
        """
        if idx is None:
            idx = np.arange(world.n)
        k = len(idx)
        if k == 0:
            z = np.zeros((0, self.h, self.w, 3), dtype=np.uint8)
            return z, np.zeros((0, self.h, self.w, 2), dtype=np.int32)

        rng_m = world.sensor_radius[idx].astype(np.float32)[:, None, None]
        theta = world.theta[idx].astype(np.float32)[:, None, None]
        ct, st = np.cos(theta), np.sin(theta)

        near = np.minimum(self.near_clip, rng_m * 0.5)
        fwd = near + self._f[None] * (rng_m - near)     # (K,H,W) metres ahead
        lat = self._u[None] * fwd * self.tan_half       # (K,H,W) metres left
        wx = world.pos[idx, 0].astype(np.float32)[:, None, None] + ct * fwd - st * lat
        wy = world.pos[idx, 1].astype(np.float32)[:, None, None] + st * fwd + ct * lat

        h, w = self.shape
        ix = (wx / self.cell).astype(np.int32)
        iy = (wy / self.cell).astype(np.int32)
        inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        np.clip(ix, 0, w - 1, out=ix)
        np.clip(iy, 0, h - 1, out=iy)

        # --- occlusion -------------------------------------------------------------
        # Each image column is one fixed bearing, so a running count of walls down the
        # column is exactly a per-ray march. A cell containing the wall is itself
        # visible; anything behind it is not.
        # Rows run far -> near, so a wall occludes rows with a SMALLER index (farther).
        # Accumulate from the near end (last row) back toward the far end.
        is_wall = world.occ[iy, ix] == grid.WALL
        near_first = np.cumsum(is_wall[:, ::-1, :], axis=1)[:, ::-1, :]
        behind_wall = (near_first - is_wall) > 0
        visible = inside & ~behind_wall

        frames = raster_img[iy, ix]                      # (K,H,W,3) uint8
        frames = np.where(visible[..., None], frames, 0).astype(np.int16)

        # --- sensor noise ----------------------------------------------------------
        # Per-CHANNEL, drawn per robot per pixel so two robots looking at the same cell
        # disagree. Per-channel matters: luminance-only noise broadcast across RGB
        # leaves the R-B warmth signal the detector keys on completely unperturbed, so
        # there is no colour confusion at all and every "false positive" would be a lie.
        # Uniform integers rather than gaussians -- same effect on discrimination,
        # roughly 3x cheaper at 512 x 48 x 48 x 3 draws per pass.
        if self.noise_sigma > 0:
            a = max(1, int(round(self.noise_sigma * 1.7)))
            # Own stream: drawing camera noise from the shared "noise" generator would
            # make every other subsystem's trajectory depend on whether perception
            # ran, and on its frame size.
            frames += world.rng["perception"].integers(
                -a, a + 1, size=(k, self.h, self.w, 3), dtype=np.int16
            )
        np.clip(frames, 0, 255, out=frames)

        cells = np.stack([iy, ix], axis=-1)
        cells[~visible] = -1
        return frames.astype(np.uint8), cells

    # ------------------------------------------------------------------ geometry

    def to_world(self, world, robot: int, row: np.ndarray, col: np.ndarray):
        """Invert image coordinates back to world positions for a robot's detections."""
        rng_m = float(world.sensor_radius[robot])
        near = min(self.near_clip, rng_m * 0.5)
        f = 1.0 - (np.asarray(row, dtype=np.float64) + 0.5) / self.h
        d = near + f * (rng_m - near)
        u = ((np.asarray(col, dtype=np.float64) + 0.5) / self.w - 0.5) * 2.0
        lat = u * d * self.tan_half
        th = float(world.theta[robot])
        x = world.pos[robot, 0] + np.cos(th) * d - np.sin(th) * lat
        y = world.pos[robot, 1] + np.sin(th) * d + np.cos(th) * lat
        return x, y, d, np.arctan2(u * self.tan_half, 1.0)
