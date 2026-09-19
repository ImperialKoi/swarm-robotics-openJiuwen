"""Exact routing to collection points, on ground wide enough to drive.

**The defect this fixes.** Loaded carriers were holding casualties for minutes a short
walk from a reachable zone. Measured on demo seeds 42-45: 39 casualties were still being
carried at the buzzer, picked up at a median t=296 with a median 124 s left, against a
median ~31 s of route at loaded speed -- 34 of the 39 would have been delivered at route
speed. Traced to one carrier on seed 42: the coarse flow field told it to go due east from
a concave pocket whose east and north cells were rock. Repulsion pushed it back, the queue
behind it pushed it in, and the swept-circle override zeroed it for 300 of its last 400
ticks, 12 m from the zone.

M-60 fixed phantom edges *between* coarse cells. This is what remains inside one: a coarse
cell has one descent direction, and it ignores the fine-grid rock a robot may be standing
next to. It bites hardest on the carry leg because every loaded carrier converges on the
same twelve points.

**Why a fine field is affordable here and nowhere else.** A fine distance field costs
~22 ms (M-4), which is why the auction never builds one per task. Collection points never
move, so one multi-source field per chassis -- seeded from every zone's delivery disc at
once -- covers every delivery for the whole mission: four fields, built once, under a
second.

**Why the ground is eroded first.** The first version routed over raw passability and
lost 13 rescues on seed 44: the exact shortest path threaded a one-cell gap, and a robot
steered by force fields cannot hold a heading straight enough to pass through one -- walls
on both sides push it back and the override stops it every tick. Routing over cells whose
eight neighbours are all passable keeps every route at least three cells wide. Where the
eroded field has no answer (a robot standing in a narrow spot, or a zone reachable only
through a squeeze), the coarse field is used exactly as before.

Measured, demo seeds 42-45, Tier 3 off: rescued **75.25 -> 82.75**, up on 4 of 4 seeds
(68->73, 88->96, 77->80, 68->82); found +2.25, lost -1.75.
"""

from __future__ import annotations

import numpy as np

from ..sim import grid

_DX = np.array([1, 1, 0, -1, -1, -1, 0, 1], dtype=np.int32)
_DY = np.array([0, 1, 1, 1, 0, -1, -1, -1], dtype=np.int32)
UNIT = np.stack([_DX, _DY], axis=1).astype(np.float64)
UNIT /= np.linalg.norm(UNIT, axis=1, keepdims=True)

#: Zones are delivered to anywhere within `extraction_radius`; seeding slightly inside it
#: means a robot that reaches the seeded disc is certainly inside the real one.
DISC_FRAC = 0.8


def _shift(a: np.ndarray, dx: int, dy: int) -> np.ndarray:
    """`out[y, x] = a[y - dy, x - dx]`, False outside."""
    out = np.zeros_like(a)
    h, w = a.shape
    out[max(0, dy):h + min(0, dy), max(0, dx):w + min(0, dx)] = \
        a[max(0, -dy):h + min(0, -dy), max(0, -dx):w + min(0, -dx)]
    return out


def erode8(passable: np.ndarray) -> np.ndarray:
    """Cells whose eight neighbours are all passable."""
    out = passable.copy()
    for dx, dy in zip(_DX, _DY, strict=True):
        out &= _shift(passable, int(dx), int(dy))
    return out


def fine_field(passable: np.ndarray, sources: np.ndarray) -> np.ndarray:
    """Wavefront distance in cells from `sources`, 8-connected **without corner cutting**:
    a diagonal step is legal only when both orthogonal cells beside it are open, because a
    body cannot slide between two blocked corners (`World.step` moves axis by axis)."""
    dist = np.full(passable.shape, np.inf, dtype=np.float32)
    front = sources & passable
    dist[front] = 0.0
    legal = []
    for dx, dy in zip(_DX, _DY, strict=True):
        if dx and dy:
            legal.append(_shift(passable, -int(dx), 0) & _shift(passable, 0, -int(dy)))
        else:
            legal.append(np.ones_like(passable))
    d = 0.0
    while front.any():
        d += 1.0
        nxt = np.zeros_like(passable)
        for k in range(8):
            nxt |= _shift(front & legal[k], int(_DX[k]), int(_DY[k]))
        nxt &= passable & np.isinf(dist)
        dist[nxt] = d
        front = nxt
    return dist


def direction_grid(passable: np.ndarray, dist: np.ndarray) -> np.ndarray:
    """Steepest-descent step index per cell (int8), -1 where nothing improves."""
    h, w = passable.shape
    iy, ix = np.mgrid[0:h, 0:w]
    vals = np.full((h, w, 8), np.inf, dtype=np.float32)
    for k in range(8):
        nx = (ix + _DX[k]).clip(0, w - 1)
        ny = (iy + _DY[k]).clip(0, h - 1)
        diag = bool(_DX[k] and _DY[k])
        # A hair more for diagonals, so ties go straight rather than zig-zagging.
        v = dist[ny, nx] + np.float32(0.001 * (2.0 if diag else 1.0))
        if diag:
            v = np.where(passable[iy, nx] & passable[ny, ix], v, np.inf)
        vals[:, :, k] = v
    vals = np.where(vals < dist[:, :, None], vals, np.inf)
    best = np.argmin(vals, axis=-1)
    good = np.isfinite(np.take_along_axis(vals, best[:, :, None], axis=-1))[:, :, 0]
    return np.where(good, best, -1).astype(np.int8)


class ZoneRouting:
    """Per-chassis direction grids to the nearest collection point."""

    def __init__(self, world) -> None:
        self.cell, self.shape = world.cell, world.shape
        zones = [world.scn.base, *world.scn.extraction_zones]
        gx, gy = grid.cell_centres(world.shape, world.cell)
        r = world.scn.extraction_radius * DISC_FRAC
        src = np.zeros(world.shape, dtype=bool)
        for zx, zy in zones:
            src |= (gx - zx) ** 2 + (gy - zy) ** 2 <= r * r
        self.dirs = []
        for c in range(len(world.chassis_passable)):
            wide = erode8(world.chassis_passable[c])
            self.dirs.append(direction_grid(wide, fine_field(wide, src)))
        # Goals arrive snapped to the cell grid by `SkillExecutor.goals`; match that.
        self.keys = {(round(float(x) / world.cell) * world.cell,
                      round(float(y) / world.cell) * world.cell) for x, y in zones}

    def is_zone(self, gx: float, gy: float) -> bool:
        return (gx, gy) in self.keys

    def step(self, chassis: int, x, y) -> tuple[np.ndarray, np.ndarray]:
        """(unit direction, valid mask) at each position."""
        ix, iy = grid.world_to_cell(np.asarray(x), np.asarray(y), self.cell, self.shape)
        s = self.dirs[int(chassis)][iy, ix]
        ok = s >= 0
        return np.where(ok[:, None], UNIT[np.maximum(s, 0)], 0.0), ok
