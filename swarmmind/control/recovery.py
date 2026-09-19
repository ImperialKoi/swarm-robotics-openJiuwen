"""Bounded fine-grid detours for robots that stop making physical progress.

The auction keeps its shared coarse fields. Recovery builds at most one small patch
per tick, shared by goal, chassis and start block, and keeps at most 64 patches.
Only terrain is read: no casualty, hazard-truth or perception data enters routing.
"""

from __future__ import annotations

from collections import OrderedDict

import numpy as np

from ..sim import grid
from .zone_routing import _DX, _DY, direction_grid, erode8, fine_field

STALL_SECONDS = 6.0
PROGRESS_METRES = 1.0


class RecoveryFields:
    def __init__(self, world, nav):
        self.cell, self.shape = world.cell, world.shape
        self.nav = nav
        self.passable = world.chassis_passable
        self.wide = [erode8(p) for p in self.passable]
        self.cache = OrderedDict()
        self.built_at = -1
        self.builds = 0

    def directions(self, world, chassis, goal, positions):
        result = np.zeros_like(positions)
        valid = np.zeros(len(positions), dtype=bool)
        # Eight-metre blocks share a 32-metre patch, independent of fine cell size.
        block = max(1, int(np.ceil(8 / self.cell)))
        ix, iy = grid.world_to_cell(*positions.T, self.cell, self.shape)
        blocks = np.column_stack([ix // block, iy // block])
        for bx, by in np.unique(blocks, axis=0):
            key = (int(chassis), float(goal[0]), float(goal[1]), int(bx), int(by))
            patch = self.cache.get(key)
            if patch is None:
                if self.built_at == world.tick:
                    continue  # amortise work even when a whole crowd gets stuck
                self.built_at = world.tick
                patch = self._build(chassis, goal, int(bx), int(by), block)
                self.cache[key] = patch
                self.builds += 1
                if len(self.cache) > 64:
                    self.cache.popitem(last=False)
            self.cache.move_to_end(key)
            x0, y0, steps = patch
            rows = np.flatnonzero((blocks[:, 0] == bx) & (blocks[:, 1] == by))
            step = steps[iy[rows] - y0, ix[rows] - x0]
            good = step >= 0
            rows, step = rows[good], step[good]
            waypoint = np.column_stack([ix[rows] + _DX[step] + .5,
                                        iy[rows] + _DY[step] + .5]) * self.cell
            delta = waypoint - positions[rows]
            result[rows] = delta / np.maximum(np.linalg.norm(delta, axis=1, keepdims=True), 1e-9)
            valid[rows] = True
        return result, valid

    def _build(self, chassis, goal, bx, by, block):
        margin = max(2, int(np.ceil(12 / self.cell)))
        x0, y0 = max(0, bx * block - margin), max(0, by * block - margin)
        x1 = min(self.shape[1], (bx + 1) * block + margin)
        y1 = min(self.shape[0], (by + 1) * block + margin)
        raw = self.passable[chassis, y0:y1, x0:x1]
        wide = self.wide[chassis][y0:y1, x0:x1]
        yy, xx = np.indices(raw.shape)
        wx, wy = (xx + x0 + .5) * self.cell, (yy + y0 + .5) * self.cell
        nav = self.nav.nav[chassis]
        coarse = nav.distance_at(nav.field(*goal), wx, wy)
        gx, gy = int(goal[0] / self.cell) - x0, int(goal[1] / self.cell) - y0
        source = np.zeros_like(raw)
        if 0 <= gx < raw.shape[1] and 0 <= gy < raw.shape[0] and raw[gy, gx]:
            source[gy, gx] = True
        else:
            # Exit toward lower coarse distance, through a physically connected
            # patch. A blocked direct edge now has to take the real local detour.
            rim = np.zeros_like(raw)
            rim[[0, -1], :] = True
            rim[:, [0, -1]] = True
            candidates = rim & wide & (coarse < 1e9)
            if candidates.any():
                source = candidates & (coarse == coarse[candidates].min())
        # Wide routes first. The raw fallback covers the short approach out of a
        # narrow starting cell; neither field cuts diagonally through blocked corners.
        wide_steps = direction_grid(wide, fine_field(wide, source))
        raw_steps = direction_grid(raw, fine_field(raw, source))
        return x0, y0, np.where(wide_steps >= 0, wide_steps, raw_steps)
