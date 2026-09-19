"""Top-down mission views as RGB arrays.

Two views, which are exactly the dashboard's fog/god-view toggle (MVP spec element 6):

- **swarm view** -- what the swarm actually knows. Unexplored space is dark, and victims
  appear only where the swarm has *reported* one, including phantoms.
- **god view** -- ground truth, for the operator only.

Rendering both side by side is the single most useful debugging artefact for perception:
a detector failure shows up instantly as reports that do not line up with reality.
"""

from __future__ import annotations

import numpy as np

from ..perception.raster import AppearanceRaster
from ..sim import grid
from ..sim.robot import DESTROYED, LANE_INDEX, OUT_OF_COMMS

LANE_COLOR = {
    "none": np.array([90, 200, 255], np.uint8),      # scout   -- cyan
    "scoop": np.array([255, 190, 60], np.uint8),     # digger  -- amber
    "gripper": np.array([120, 255, 140], np.uint8),  # carrier -- green
    "antenna": np.array([215, 130, 255], np.uint8),  # relay   -- violet
}
COLOR_DEAD = np.array([90, 40, 40], np.uint8)
COLOR_VICTIM_TRUE = np.array([255, 90, 90], np.uint8)
COLOR_REPORT = np.array([255, 255, 255], np.uint8)
FOG_DIM = 0.18


class MissionRenderer:
    def __init__(self, world) -> None:
        self.raster = AppearanceRaster(world)
        self.shape = world.shape
        self.cell = world.cell

    def views(self, world, reports=None) -> tuple[np.ndarray, np.ndarray]:
        """(swarm_view, god_view), both (H, W, 3) uint8."""
        god = self.raster.render(world).astype(np.float32)

        swarm = god * FOG_DIM
        seen = world.explored
        swarm[seen] = god[seen]

        god = god.astype(np.uint8)
        swarm = swarm.astype(np.uint8)

        # Ground truth victims: operator view only.
        for v in world.victims:
            if v.state != 4:
                self._dot(god, v.pos, 2, COLOR_VICTIM_TRUE)

        # What the swarm believes -- may include phantoms, may miss real victims.
        for r in reports or []:
            self._ring(swarm, r, 3, COLOR_REPORT)

        for img in (swarm, god):
            self._robots(img, world)
        return swarm, god

    def _robots(self, img, world) -> None:
        from ..sim.robot import LANES

        for lane in LANES:
            for status, colour in ((None, LANE_COLOR[lane]), (DESTROYED, COLOR_DEAD)):
                m = world.actuator == LANE_INDEX[lane]
                m &= (world.status >= DESTROYED) if status else (world.status <= OUT_OF_COMMS)
                if not m.any():
                    continue
                self._dots(img, world.pos[m], 1, colour)

    def _dots(self, img, centres, r_cells, colour) -> None:
        dy, dx = grid.disc_template(float(r_cells))
        h, w = self.shape
        cix, ciy = grid.world_to_cell(centres[:, 0], centres[:, 1], self.cell, self.shape)
        iy = np.clip(ciy[:, None] + dy[None, :], 0, h - 1)
        ix = np.clip(cix[:, None] + dx[None, :], 0, w - 1)
        img[iy, ix] = colour

    def _dot(self, img, centre, r_cells, colour) -> None:
        self._dots(img, np.asarray(centre, dtype=float)[None, :], r_cells, colour)

    def _ring(self, img, centre, r_cells, colour) -> None:
        h, w = self.shape
        cix, ciy = grid.world_to_cell(
            np.asarray(centre[0]), np.asarray(centre[1]), self.cell, self.shape
        )
        dy, dx = grid.disc_template(float(r_cells))
        keep = (dy**2 + dx**2) >= (r_cells - 1) ** 2
        iy = np.clip(int(ciy) + dy[keep], 0, h - 1)
        ix = np.clip(int(cix) + dx[keep], 0, w - 1)
        img[iy, ix] = colour


def contact_sheet(frames: np.ndarray, cols: int = 8, pad: int = 2) -> np.ndarray:
    """Tile camera frames into one image: what the robots are actually looking at."""
    k, h, w, _ = frames.shape
    rows = (k + cols - 1) // cols
    sheet = np.zeros((rows * (h + pad) + pad, cols * (w + pad) + pad, 3), np.uint8)
    sheet[:] = 24
    for i in range(k):
        r, c = divmod(i, cols)
        y, x = pad + r * (h + pad), pad + c * (w + pad)
        sheet[y : y + h, x : x + w] = frames[i]
    return sheet
