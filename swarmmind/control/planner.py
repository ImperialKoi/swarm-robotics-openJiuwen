"""Navigation: coarse flow fields, A*, and frontier detection.

The central scaling decision of Tier 1 lives here. A* per robot does not scale: 512
robots replanning every 2 s is ~256 searches/second over 66k cells. Instead, robots
navigate by descending a **shared distance field computed once per goal** on a
4x-downsampled grid -- the same field the auction reads for bidding (MEASUREMENTS.md
M-4). Cost is per *goal*, not per robot, and since ``passable`` never changes during a
mission the fields cache for the whole run.

A* survives for single-robot use: tests, debugging, and any one-off unique goal. It is
never called per-robot per-tick.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass

import numpy as np

from ..sim import grid

#: 8-neighbour offsets, in a fixed order. Never iterate a set here -- tie-breaking in
#: gradient descent must be reproducible (CLAUDE.md invariant #5).
_DX = np.array([1, 1, 0, -1, -1, -1, 0, 1], dtype=np.int32)
_DY = np.array([0, 1, 1, 1, 0, -1, -1, -1], dtype=np.int32)
_DNORM = np.sqrt(_DX.astype(np.float32) ** 2 + _DY.astype(np.float32) ** 2)
# The field builder orders its edges differently from the steering tie-break order.
_EDGE_INDEX = np.array([grid.COARSE_DIRS.index((int(dx), int(dy)))
                        for dx, dy in zip(_DX, _DY, strict=True)])

#: The eight step directions as unit vectors, precomputed.
#:
#: Built by exactly the expression ``descend`` used to evaluate per robot per tick
#: (``d.astype(float64) / linalg.norm(d)``), so a table lookup returns bit-identical
#: values -- no offset in the last place, and therefore no change to the seed-42
#: scorecard hash. No row is (0, 0), so the norm is never below 1 and the old
#: ``np.maximum(n, 1e-9)`` guard was always a no-op.
_D_UNIT = np.stack([_DX, _DY], axis=1).astype(np.float64)
_D_UNIT /= np.linalg.norm(_D_UNIT, axis=1, keepdims=True)

#: Stored in the per-goal direction grid for "nothing here improves on standing still".
_NO_STEP = np.int8(-1)

#: Distance stand-in for unreachable coarse cells. Large enough to never win an argmin,
#: finite so arithmetic on it does not produce NaN.
UNREACHABLE = np.float32(1e9)


class NavFields:
    """Cache of coarse distance fields, keyed by goal cell.

    One entry serves every robot heading to that goal, and serves the auction's bid
    distance for the same target. Fields are valid for the whole mission because
    ``passable`` is static.
    """

    def __init__(self, passable: np.ndarray, cell: float, factor: int = 4,
                 max_entries: int = 512, threshold: float = 0.5,
                 edge_aware: bool = True, edge_steering: bool = False) -> None:
        self.factor = factor
        self.cell = cell
        self.coarse_cell = cell * factor
        self.coarse = grid.downsample(passable, factor, threshold)
        #: Which coarse steps have a real fine-grid link. Without this the wavefront
        #: routes through corridors that exist only on the routing grid -- 4.4-6.1% of
        #: all coarse edges on the demo map (POSSIBLE_BUG2, M-60). Built once per
        #: chassis; `passable` never changes for the mission.
        self.edges = grid.coarse_edge_masks(passable, factor) if edge_aware else None
        # Experimental: field-consistent steering removes blocked steps, but reduced
        # rescues on the small scripted fixture. Requires demo-map evaluation (M-84).
        self.edge_steering = edge_steering
        self.shape = self.coarse.shape
        self._cache: dict[tuple[int, int], np.ndarray] = {}
        #: Steepest-descent direction index per coarse cell, one grid per goal, built
        #: lazily by `descend_to`. int8 -- a quarter the size of the float32 field it
        #: accompanies, and only goals that are actually *navigated to* ever get one
        #: (the auction's distance queries do not build these).
        self._dirs: dict[tuple[int, int], np.ndarray] = {}
        self._order: list[tuple[int, int]] = []
        self._max = max_entries
        #: Resolved goal coordinates -> cache key, so the 20 Hz control loop stops
        #: redoing `to_coarse` + `_nearest_passable` for goals that change at 1 Hz.
        self._goal_key: dict[tuple[float, float], tuple[int, int]] = {}
        self.hits = 0
        self.misses = 0

    # --- coordinates --------------------------------------------------------------

    def to_coarse(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        # `ndarray.clip`, not `np.clip`. The module-level function dispatches through
        # `fromnumeric._wrapfunc` into `_methods._clip`, which builds two `np.finfo`
        # objects on every call to find the dtype bounds. Profiling one 60 s episode
        # counted 162k clip calls and 308k finfo constructions -- about a fifth of the
        # episode -- on arrays of a dozen elements, where that overhead is the entire
        # cost. The method form is the same operation without the detour.
        h, w = self.shape
        ix = (np.asarray(x) / self.coarse_cell).astype(np.int32)
        iy = (np.asarray(y) / self.coarse_cell).astype(np.int32)
        return ix.clip(0, w - 1), iy.clip(0, h - 1)

    def _nearest_passable(self, ix: int, iy: int) -> tuple[int, int]:
        """Snap a goal that landed on an impassable coarse cell to the closest passable
        one. A goal inside rubble is common -- a victim can be under it."""
        if self.coarse[iy, ix]:
            return ix, iy
        piy, pix = np.nonzero(self.coarse)
        if len(pix) == 0:
            raise RuntimeError("no passable coarse cells; check map generation")
        k = int(np.argmin((pix - ix) ** 2 + (piy - iy) ** 2))
        return int(pix[k]), int(piy[k])

    # --- fields -------------------------------------------------------------------

    def _resolve(self, gx: float, gy: float) -> tuple[int, int]:
        """Goal coordinates -> cache key, memoised.

        Goals are reissued by the auction at 1 Hz but re-resolved by Tier 1 at 20 Hz, so
        this ran twenty times more often than it changed. Keyed on the exact float pair:
        an identical goal hits, and a goal that moves at all simply misses and takes the
        old path, so the memo can never return a key for the wrong position.
        """
        ck = (float(gx), float(gy))
        key = self._goal_key.get(ck)
        if key is None:
            ix, iy = self.to_coarse(np.asarray(gx), np.asarray(gy))
            key = self._nearest_passable(int(ix), int(iy))
            # Bounded: goal coordinates are floats and a drifting target would otherwise
            # accumulate an entry per tick for the whole mission. Dropping the lot is
            # fine -- it is a pure memo and refills in one pass.
            if len(self._goal_key) >= 4 * self._max:
                self._goal_key.clear()
            self._goal_key[ck] = key
        return key

    def _field_for(self, key: tuple[int, int]) -> np.ndarray:
        cached = self._cache.get(key)
        if cached is not None:
            self.hits += 1
            return cached
        self.misses += 1
        f = grid.distance_field(self.coarse, key, self.edges).astype(np.float32)
        f[~np.isfinite(f)] = UNREACHABLE
        self._cache[key] = f
        self._order.append(key)
        if len(self._order) > self._max:
            evicted = self._order.pop(0)
            del self._cache[evicted]
            # The direction grid is derived from the field; outliving it would leak.
            self._dirs.pop(evicted, None)
        return f

    def field(self, gx: float, gy: float) -> np.ndarray:
        """Distance field (in coarse cells) to the goal, cached."""
        return self._field_for(self._resolve(gx, gy))

    def distance_at(self, field: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Metres from each position to the field's goal. O(1) per robot."""
        ix, iy = self.to_coarse(x, y)
        return field[iy, ix] * self.coarse_cell

    def descend(self, field: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Unit direction of steepest descent at each position, shape (N, 2).

        Zero where the robot already stands on the goal cell or nothing improves.
        """
        ix, iy = self.to_coarse(x, y)
        h, w = self.shape
        nix = np.clip(ix[:, None] + _DX[None, :], 0, w - 1)
        niy = np.clip(iy[:, None] + _DY[None, :], 0, h - 1)
        vals = field[niy, nix]
        # Cost to *step* there, so diagonals are not preferred purely for being diagonal.
        vals = vals + _DNORM[None, :] * 0.001
        here = field[iy, ix][:, None]
        if self.edge_steering and self.edges is not None:
            allowed = self.edges[_EDGE_INDEX[None, :], iy[:, None], ix[:, None]]
            vals = np.where(allowed, vals, np.float32(np.inf))
        vals = np.where(vals < here, vals, np.float32(np.inf))
        best = np.argmin(vals, axis=1)
        improves = np.isfinite(np.take_along_axis(vals, best[:, None], axis=1)).ravel()
        d = np.stack([_DX[best], _DY[best]], axis=1).astype(np.float64)
        n = np.linalg.norm(d, axis=1, keepdims=True)
        return np.where(improves[:, None] & (n[:, 0] > 0)[:, None], d / np.maximum(n, 1e-9), 0.0)

    # --- precomputed descent ----------------------------------------------------------

    def _direction_grid(self, key: tuple[int, int]) -> np.ndarray:
        """Steepest-descent step index for *every* coarse cell of one goal's field.

        ``descend`` recomputed this per robot per tick, but the field it reads is static
        for the whole mission -- ``passable`` never changes and the cache never
        invalidates -- so the answer at a given cell is static too. Profiling found
        ``descend`` being called 13 times a tick on arrays of roughly one robot each:
        all of numpy's dispatch cost, none of its vectorisation benefit. Hoisting the
        work to once per goal turns the per-tick job into a single gather.

        The arithmetic is deliberately identical to ``descend``'s, evaluated over a
        mesh instead of a robot subset, so ``argmin`` sees the same values in the same
        order and breaks ties the same way.
        """
        d = self._dirs.get(key)
        if d is not None:
            return d
        field = self._field_for(key)
        h, w = self.shape
        iy, ix = np.mgrid[0:h, 0:w]
        nix = (ix[:, :, None] + _DX).clip(0, w - 1)
        niy = (iy[:, :, None] + _DY).clip(0, h - 1)
        vals = field[niy, nix] + _DNORM * 0.001
        if self.edge_steering and self.edges is not None:
            # A neighbouring cell can be closer by a route *around* a wall. Its lower
            # distance does not make the direct edge to it traversable.
            allowed = self.edges[_EDGE_INDEX].transpose(1, 2, 0)
            vals = np.where(allowed, vals, np.float32(np.inf))
        vals = np.where(vals < field[:, :, None], vals, np.float32(np.inf))
        best = np.argmin(vals, axis=-1)
        improves = np.isfinite(np.take_along_axis(vals, best[:, :, None], axis=-1))[:, :, 0]
        d = np.where(improves, best, _NO_STEP).astype(np.int8)
        self._dirs[key] = d
        return d

    def descend_to(self, gx: float, gy: float,
                   x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """``descend`` against the goal at (gx, gy), via the precomputed grid.

        Equivalent to ``descend(field(gx, gy), x, y)`` and asserted so in
        ``tests/test_planner.py`` -- ``descend`` is kept as the reference definition.
        """
        b = self._direction_grid(self._resolve(gx, gy))
        ix, iy = self.to_coarse(x, y)
        step = b[iy, ix]
        # -1 would index the last row of the table, so the mask is load-bearing.
        return np.where((step >= 0)[:, None], _D_UNIT[step], 0.0)


class NavSet:
    """One NavFields per chassis.

    Routing has to be per-locomotion or the flow fields lie: a wheeled unit told to
    descend a field built on legged passability will walk confidently into a river and
    stop at the bank. Fields are cached per chassis and, since passability is static for
    the mission, never invalidate -- so this costs memory, not recomputation.
    """

    def __init__(self, world, factor: int = 4, threshold: float = 0.5,
                 edge_aware: bool = True, edge_steering: bool = False) -> None:
        from ..sim.robot import CHASSIS

        self.chassis = CHASSIS
        # Flight crosses disconnected ground regions. Keep the world's rotor mask
        # for landing safety; only its route/bid fields use open airspace.
        airspace = np.ones(world.shape, dtype=bool)
        # A rotor can cross terrain but not the map's rendered escarpment.  Use the
        # same two-cell safety corridor as chassis passability, so a flow field never
        # offers an edge-hugging route that collision must subsequently reject.
        from ..sim.world import EDGE_CLEARANCE_CELLS
        edge = EDGE_CLEARANCE_CELLS
        airspace[:edge, :] = False
        airspace[-edge:, :] = False
        airspace[:, :edge] = False
        airspace[:, -edge:] = False
        self.nav = [
            NavFields(airspace if chassis == "rotor" else world.chassis_passable[i],
                      world.cell, factor, threshold=threshold,
                      edge_aware=edge_aware, edge_steering=edge_steering)
            for i, chassis in enumerate(CHASSIS)
        ]
        self.factor = factor
        self.coarse_cell = self.nav[0].coarse_cell
        #: Exact routing to collection points (`control/zone_routing.py`). `None` is the
        #: coarse field everywhere, as shipped.
        self.zones = None

    def enable_zone_routing(self, world) -> None:
        from .zone_routing import ZoneRouting

        self.zones = ZoneRouting(world)

    def field(self, chassis: int, gx: float, gy: float) -> np.ndarray:
        return self.nav[int(chassis)].field(gx, gy)

    def distance_at(self, chassis: int, field, x, y):
        return self.nav[int(chassis)].distance_at(field, x, y)

    def descend(self, chassis: int, field, x, y):
        return self.nav[int(chassis)].descend(field, x, y)

    def descend_to(self, chassis: int, gx: float, gy: float, x, y):
        coarse = self.nav[int(chassis)].descend_to(gx, gy, x, y)
        if self.zones is None or not self.zones.is_zone(gx, gy):
            return coarse
        fine, ok = self.zones.step(chassis, x, y)
        return np.where(ok[:, None], fine, coarse)

    def coarse_of(self, chassis: int) -> np.ndarray:
        return self.nav[int(chassis)].coarse

    @property
    def misses(self) -> int:
        return sum(n.misses for n in self.nav)

    @property
    def hits(self) -> int:
        return sum(n.hits for n in self.nav)


# --- A*, for single goals only --------------------------------------------------------


def astar(passable: np.ndarray, start: tuple[int, int], goal: tuple[int, int]
          ) -> list[tuple[int, int]] | None:
    """8-connected A* on the fine grid. Returns [(ix, iy), ...] or None.

    NOT for the hot loop -- use NavFields. This exists for tests, debugging, and any
    genuinely one-off path.
    """
    h, w = passable.shape
    sx, sy = start
    gx, gy = goal
    if not (passable[sy, sx] and passable[gy, gx]):
        return None
    if start == goal:
        return [start]

    def hcost(x: int, y: int) -> float:
        dx, dy = abs(x - gx), abs(y - gy)
        return (dx + dy) + (np.sqrt(2.0) - 2.0) * min(dx, dy)

    open_heap = [(hcost(sx, sy), 0.0, sx, sy)]
    came: dict[tuple[int, int], tuple[int, int]] = {}
    gscore = {start: 0.0}
    closed = np.zeros_like(passable)

    while open_heap:
        _, g, x, y = heapq.heappop(open_heap)
        if (x, y) == goal:
            path = [(x, y)]
            while (x, y) in came:
                x, y = came[(x, y)]
                path.append((x, y))
            return path[::-1]
        if closed[y, x]:
            continue
        closed[y, x] = True
        for k in range(8):
            nx, ny = x + int(_DX[k]), y + int(_DY[k])
            if not (0 <= nx < w and 0 <= ny < h) or not passable[ny, nx] or closed[ny, nx]:
                continue
            ng = g + float(_DNORM[k])
            if ng < gscore.get((nx, ny), np.inf):
                gscore[(nx, ny)] = ng
                came[(nx, ny)] = (x, y)
                heapq.heappush(open_heap, (ng + hcost(nx, ny), ng, nx, ny))
    return None


# --- frontier -------------------------------------------------------------------------


@dataclass(frozen=True)
class FrontierTarget:
    pos: tuple[float, float]
    size: int
    sector: int


def frontier_mask(explored: np.ndarray, passable: np.ndarray) -> np.ndarray:
    """**Unexplored** passable cells adjacent to explored ones.

    The frontier must lie on the unknown side of the boundary. Defining it as the
    explored side makes every explore task complete the instant it is issued -- the
    target cell is already explored by construction -- so robots receive a task, retire
    it, and never move.
    """
    known = explored & passable
    return passable & ~explored & grid.dilate8(known)


def frontier_targets(world, max_targets: int = 48, factor: int = 8) -> list[FrontierTarget]:
    """Cluster frontier cells into representative goals, largest cluster first.

    Clustering is a fixed grid binning rather than a flood fill: it is O(cells), needs
    no tie-breaking, and is therefore deterministic. Ties on size break on cell index
    so the ordering is reproducible (CLAUDE.md invariant #5).
    """
    fm = frontier_mask(world.explored, world.passable & (world.water == 0))
    iy, ix = np.nonzero(fm)
    if len(ix) == 0:
        return []
    by = iy // factor
    bx = ix // factor
    nbx = (world.shape[1] + factor - 1) // factor
    key = by * nbx + bx
    counts = np.bincount(key)
    sums_x = np.bincount(key, weights=ix.astype(np.float64))
    sums_y = np.bincount(key, weights=iy.astype(np.float64))
    nz = np.nonzero(counts)[0]
    # -count first, then key: deterministic total order.
    order = sorted(nz.tolist(), key=lambda k: (-int(counts[k]), int(k)))[:max_targets]

    out = []
    for k in order:
        # A centroid of bank cells can lie in the river (or inside a building).
        # Choose an actual frontier member nearest it instead.
        members = np.flatnonzero(key == k)
        pick = members[np.argmin((ix[members] - sums_x[k] / counts[k]) ** 2
                                 + (iy[members] - sums_y[k] / counts[k]) ** 2)]
        cx, cy = (ix[pick] + 0.5) * world.cell, (iy[pick] + 0.5) * world.cell
        sec = int(world.sector_of_cell[iy[pick], ix[pick]])
        out.append(FrontierTarget(pos=(float(cx), float(cy)), size=int(counts[k]), sector=sec))
    return out
