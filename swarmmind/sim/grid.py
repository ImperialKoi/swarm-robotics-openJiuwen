"""Occupancy grid: generation, connectivity, line of sight, distance fields.

Everything here is vectorised and deterministic. No Python loop may run per-robot
per-tick (CLAUDE.md scaling rules); the loops that do exist run at 5 Hz or 1 Hz over
whole arrays.

Indexing convention throughout: ``grid[iy, ix]``, world ``(x, y)`` -> ``(ix, iy)``.
"""

from __future__ import annotations

import numpy as np

FREE = np.uint8(0)
WALL = np.uint8(1)
RUBBLE = np.uint8(2)  # passable, traversed at RUBBLE_SPEED_FACTOR

RUBBLE_SPEED_FACTOR = 0.5

#: Fractions along the robot->cell segment sampled for line of sight. Eight samples
#: over a <=9 m sensor radius means a sample roughly every 2 cells: coarse enough to
#: be cheap, fine enough that no wall thinner than the sample spacing exists (all
#: walls are at least one cell and generated as blobs, never as 1-cell diagonals).
_LOS_SAMPLES = np.linspace(0.12, 0.94, 8, dtype=np.float32)


def world_to_cell(x: np.ndarray, y: np.ndarray, cell: float, shape: tuple[int, int]):
    """Vectorised world -> grid index, clipped to bounds.

    `ndarray.clip`, not `np.clip`. This is M-38's finding in the hottest function in the
    codebase: the module-level `np.clip` detours through `_wrapfunc` -> `_methods._clip`
    and constructs two `np.finfo` objects **per call**. Profiled at t=240 on the demo
    scenario, `world_to_cell` was 633,156 calls and 12.2 s of a 27.4 s window, of which
    8.4 s was that detour and 2,964,952 of the calls were `np.finfo.__init__`.

    M-38 fixed the same mistake in `control/planner.py` and this call site was missed.
    """
    h, w = shape
    ix = (np.asarray(x) / cell).astype(np.int32).clip(0, w - 1)
    iy = (np.asarray(y) / cell).astype(np.int32).clip(0, h - 1)
    return ix, iy


def cell_centres(shape: tuple[int, int], cell: float):
    """(xs, ys) world coordinates of every cell centre, shaped like the grid."""
    h, w = shape
    xs = (np.arange(w, dtype=np.float32) + 0.5) * cell
    ys = (np.arange(h, dtype=np.float32) + 0.5) * cell
    return np.meshgrid(xs, ys)


def generate(
    rng: np.random.Generator,
    *,
    width_m: float,
    height_m: float,
    cell: float,
    n_clusters: int,
    cluster_radius_m: tuple[float, float],
    rubble_fraction: float,
    keepout: list[tuple[float, float, float]],
) -> np.ndarray:
    """Generate a rubble field: blob obstacles, a wall border, and clear keepout discs.

    ``keepout`` is a list of ``(x, y, radius)`` discs kept free -- base and extraction
    zones, so a mission can never start or end inside a wall.
    """
    w = int(round(width_m / cell))
    h = int(round(height_m / cell))
    occ = np.full((h, w), FREE, dtype=np.uint8)

    gx, gy = cell_centres((h, w), cell)
    cx = rng.uniform(0.0, width_m, n_clusters)
    cy = rng.uniform(0.0, height_m, n_clusters)
    cr = rng.uniform(cluster_radius_m[0], cluster_radius_m[1], n_clusters)
    # Rubble (passable) vs solid wall, decided per cluster.
    is_rubble = rng.random(n_clusters) < rubble_fraction

    for i in range(n_clusters):
        # Squashed ellipse so clusters do not all read as circles.
        ax = cr[i] * rng.uniform(0.7, 1.4)
        ay = cr[i] * rng.uniform(0.7, 1.4)
        blob = (((gx - cx[i]) / ax) ** 2 + ((gy - cy[i]) / ay) ** 2) <= 1.0
        occ[blob] = RUBBLE if is_rubble[i] else WALL

    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = WALL

    for kx, ky, kr in keepout:
        occ[((gx - kx) ** 2 + (gy - ky) ** 2) <= kr * kr] = FREE
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = WALL
    return occ


def road_network(points: list[tuple[float, float]]) -> list[tuple[int, int]]:
    """Minimum spanning tree over the fixed points: the edges to build roads along.

    A complete graph is the obvious thing and it is wrong. Twelve collection points give
    66 corridors, each regrading the terrain it crosses, so intersections become steps
    where a later road overwrites an earlier one -- measured max slope on a road was
    1.17, three times a wheeled unit's limit, and the network was impassable to the very
    chassis it exists for. A spanning tree connects everything with 11 edges and barely
    crosses itself.
    """
    if len(points) < 2:
        return []
    pts = np.asarray(points, dtype=np.float64)
    inside = [0]
    outside = list(range(1, len(points)))
    edges: list[tuple[int, int]] = []
    while outside:
        best = None
        for i in inside:
            d = np.linalg.norm(pts[outside] - pts[i], axis=1)
            k = int(np.argmin(d))
            if best is None or d[k] < best[0]:
                best = (float(d[k]), i, outside[k])
        _, a, b = best
        edges.append((a, b))
        inside.append(b)
        outside.remove(b)
    return edges


def carve_roads(occ: np.ndarray, cell: float, points: list[tuple[float, float]],
                edges: list[tuple[int, int]], half_width_m: float = 1.6) -> np.ndarray:
    """Clear straight corridors between the mission's fixed points.

    Obstacle clusters are placed without regard to the base, and on some seeds they ring
    it: seed 44 left **753 passable cells out of 153,600** once `ensure_connected` walled
    off everything the base could not reach. Roads between the collection points
    guarantee those points share a component, and a disaster zone having roads between
    its staging areas is hardly a stretch.
    """
    h, w = occ.shape
    out = occ.copy()
    r = max(1, int(round(half_width_m / cell)))
    dy, dx = np.mgrid[-r:r + 1, -r:r + 1]
    disc = (dy * dy + dx * dx) <= r * r
    dy, dx = dy[disc], dx[disc]

    for i, j in edges:
        ax, ay = points[i]
        bx, by = points[j]
        steps = max(2, int(np.hypot(bx - ax, by - ay) / cell))
        t = np.linspace(0.0, 1.0, steps)
        ix, iy = world_to_cell(ax + (bx - ax) * t, ay + (by - ay) * t, cell, occ.shape)
        yy = np.clip(iy[:, None] + dy[None, :], 0, h - 1)
        xx = np.clip(ix[:, None] + dx[None, :], 0, w - 1)
        out[yy, xx] = FREE
    return out


def ensure_connected(occ: np.ndarray, seed_cell: tuple[int, int]) -> np.ndarray:
    """Wall off any passable region unreachable from ``seed_cell`` (ix, iy).

    Guarantees every free cell in the returned grid is reachable, so victim placement,
    frontier selection and distance fields can never target an unreachable pocket.
    Filling pockets is preferred over regenerating the map: it is deterministic and
    cannot loop.
    """
    passable = occ != WALL
    reached = _flood(passable, seed_cell)
    occ = occ.copy()
    occ[passable & ~reached] = WALL
    return occ


def _flood(passable: np.ndarray, seed_cell: tuple[int, int]) -> np.ndarray:
    ix, iy = seed_cell
    if not passable[iy, ix]:
        raise ValueError(f"flood seed cell ({ix}, {iy}) is not passable")
    reached = np.zeros_like(passable)
    reached[iy, ix] = True
    frontier = reached.copy()
    while frontier.any():
        grown = dilate8(frontier) & passable & ~reached
        reached |= grown
        frontier = grown
    return reached


def dilate8(mask: np.ndarray) -> np.ndarray:
    """8-connected dilation by one cell, via shifted ORs. No SciPy dependency."""
    out = mask.copy()
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    out[1:, 1:] |= mask[:-1, :-1]
    out[1:, :-1] |= mask[:-1, 1:]
    out[:-1, 1:] |= mask[1:, :-1]
    out[:-1, :-1] |= mask[1:, 1:]
    return out


def _shift(a: np.ndarray, dx: int, dy: int) -> np.ndarray:
    """`a` translated by (dx, dy) with False padding -- never wrapping."""
    out = np.zeros_like(a)
    h, w = a.shape
    ys = slice(max(0, dy), h + min(0, dy))
    xs = slice(max(0, dx), w + min(0, dx))
    sys_ = slice(max(0, -dy), h + min(0, -dy))
    sxs = slice(max(0, -dx), w + min(0, -dx))
    out[ys, xs] = a[sys_, sxs]
    return out


def distance_field(passable: np.ndarray, target_cell: tuple[int, int],
                   edges: np.ndarray | None = None) -> np.ndarray:
    """Obstacle-aware distance from ``target_cell`` to every cell, in *cells*.

    8-connected uniform-cost wavefront BFS: one vectorised dilation per distance level,
    ~O(grid diameter) numpy operations total (a few ms on a 192x128 grid). This is a
    Chebyshev metric -- a pure diagonal costs 1 per step rather than sqrt(2), so it
    under-estimates diagonal travel by up to ~30%.

    That is deliberate and acceptable: this field feeds *bid ranking* (section 5.2),
    not path execution. Tier 1 plans real paths with A*. If evaluation shows bid quality
    suffering, replace this with a bucketed 2:3 chamfer wavefront -- same signature.
    """
    ix, iy = target_cell
    dist = np.full(passable.shape, np.inf, dtype=np.float32)
    if not passable[iy, ix]:
        return dist
    dist[iy, ix] = 0.0
    frontier = np.zeros_like(passable)
    frontier[iy, ix] = True
    d = 0.0
    # The wavefront grows by one cell per level, so for most of a field only a small
    # window of the grid can change. Operating on the whole array every level is the
    # bulk of the cost on a 90 x 60 coarse grid with a ~150-level diameter. The window
    # is tracked exactly -- one cell of margin per level -- so the result is identical.
    H, W = passable.shape
    wy0, wy1, wx0, wx1 = iy, iy + 1, ix, ix + 1
    while frontier.any():
        d += 1.0
        wy0, wy1 = max(0, wy0 - 1), min(H, wy1 + 1)
        wx0, wx1 = max(0, wx0 - 1), min(W, wx1 + 1)
        win = (slice(wy0, wy1), slice(wx0, wx1))
        f_win = frontier[win]
        p_win = passable[win]
        if edges is None:
            n_win = dilate8(f_win) & p_win
        else:
            # Expand per direction, and only where a real fine-grid link exists. Plain
            # `dilate8` treats every adjacency between passable coarse cells as walkable,
            # which invents corridors through solid rock -- see `coarse_edge_masks`.
            n_win = np.zeros_like(f_win)
            h_, w_ = f_win.shape
            for k, (dx, dy) in enumerate(COARSE_DIRS):
                # Same shape as `dilate8`: in-place ORs into destination slices, with no
                # per-direction allocation. The only difference is that the source is
                # masked by whether this step exists on the fine grid.
                e_win = edges[k][win]
                dy0, dy1 = max(0, dy), h_ + min(0, dy)
                dx0, dx1 = max(0, dx), w_ + min(0, dx)
                sy0, sy1 = max(0, -dy), h_ + min(0, -dy)
                sx0, sx1 = max(0, -dx), w_ + min(0, -dx)
                n_win[dy0:dy1, dx0:dx1] |= (f_win[sy0:sy1, sx0:sx1]
                                            & e_win[sy0:sy1, sx0:sx1])
            n_win &= p_win
        n_win &= np.isinf(dist[win])
        dist[win][n_win] = d
        frontier = np.zeros_like(passable)
        frontier[win] = n_win
    return dist


def clearance_mask(occ: np.ndarray, cells: int = 1) -> np.ndarray:
    """Passable cells at least ``cells`` away from any WALL.

    Spawn points and any other "must not start stuck" placement come from here. A robot
    whose body circle overlaps a wall cannot move along that axis, so placing one at a
    free cell that merely *touches* a wall produces a robot that never leaves the spot.
    """
    blocked = occ == WALL
    for _ in range(cells):
        blocked = dilate8(blocked)
    return (occ != WALL) & ~blocked


def downsample(passable: np.ndarray, factor: int, threshold: float = 0.5) -> np.ndarray:
    """Coarse passability: a block is passable if this fraction of its cells are.

    "Any" would route robots into gaps too narrow to fit; "all" would wall off legitimate
    corridors. The threshold sets where between those the routing grid sits, and it is a
    real trade: too low and flow fields promise routes through ground the robot cannot
    actually thread, so it presses into terrain and the safety override stalls it.
    """
    h, w = passable.shape
    ph, pw = -h % factor, -w % factor
    if ph or pw:
        passable = np.pad(passable, ((0, ph), (0, pw)), constant_values=False)
    hh, ww = passable.shape[0] // factor, passable.shape[1] // factor
    blocks = passable.reshape(hh, factor, ww, factor)
    return blocks.mean(axis=(1, 3)) >= threshold


#: Neighbour offsets in the order `coarse_edge_masks` returns them.
COARSE_DIRS: tuple[tuple[int, int], ...] = (
    (-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1),
)


def coarse_edge_masks(fine: np.ndarray, factor: int) -> np.ndarray:
    """Which steps between adjacent coarse cells a robot can actually take.

    `downsample` calls a block passable when enough of it is, and the wavefront then
    assumes plain 8-connectivity between passable blocks. Both are individually
    reasonable and together they invent corridors: two blocks can each pass the majority
    test while their passable fine cells lie on opposite sides of solid rock, so the
    edge between them exists on the routing grid and nowhere in the world.

    Measured on the demo map, **4.4-6.1% of all coarse edges are phantom**, three
    quarters of them diagonal, and a robot routed onto one is held against the rock by
    its own goal term until the safety floor zeroes it (POSSIBLE_BUG2, M-60).

    Returns `(8, H, W)` in `COARSE_DIRS` order: `mask[d, cy, cx]` is True when a step
    from coarse `(cx, cy)` in direction `d` has at least one pair of 8-adjacent passable
    fine cells across the boundary. For a pure diagonal that is exactly one fine cell
    against one fine cell -- which is why diagonals dominate the phantom count.
    """
    h, w = fine.shape
    ph, pw = -h % factor, -w % factor
    if ph or pw:
        fine = np.pad(fine, ((0, ph), (0, pw)), constant_values=False)
    hh, ww = fine.shape[0] // factor, fine.shape[1] // factor
    f = factor
    blocks = fine.reshape(hh, f, ww, f)

    #: The fine cells on each face of every block: (hh, ww, f).
    north = blocks[:, 0, :, :]
    south = blocks[:, f - 1, :, :]
    west = blocks[:, :, :, 0].transpose(0, 2, 1)
    east = blocks[:, :, :, f - 1].transpose(0, 2, 1)

    def touch(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Any passable cell of `a` 8-adjacent to one of `b`, both (n, f) face strips."""
        shifted = b.copy()
        shifted[:, :-1] |= b[:, 1:]
        shifted[:, 1:] |= b[:, :-1]
        return (a & shifted).any(axis=1)

    out = np.zeros((len(COARSE_DIRS), hh, ww), dtype=bool)
    for d, (dx, dy) in enumerate(COARSE_DIRS):
        # source cells that have a neighbour in this direction at all
        ys = slice(max(0, -dy), hh - max(0, dy))
        xs = slice(max(0, -dx), ww - max(0, dx))
        nys = slice(max(0, dy), hh - max(0, -dy))
        nxs = slice(max(0, dx), ww - max(0, -dx))
        if dx and dy:
            # Only the two touching corner cells can ever be adjacent.
            a_y = f - 1 if dy > 0 else 0
            a_x = f - 1 if dx > 0 else 0
            b_y = 0 if dy > 0 else f - 1
            b_x = 0 if dx > 0 else f - 1
            a = blocks[ys, a_y, xs, a_x]
            b = blocks[nys, b_y, nxs, b_x]
            out[d][ys, xs] = a & b
        elif dx:
            a = (east if dx > 0 else west)[ys, xs]
            b = (west if dx > 0 else east)[nys, nxs]
            out[d][ys, xs] = touch(a.reshape(-1, f), b.reshape(-1, f)).reshape(a.shape[:2])
        else:
            a = (south if dy > 0 else north)[ys, xs]
            b = (north if dy > 0 else south)[nys, nxs]
            out[d][ys, xs] = touch(a.reshape(-1, f), b.reshape(-1, f)).reshape(a.shape[:2])
    return out


def distance_field_multi(passable: np.ndarray, sources: np.ndarray) -> np.ndarray:
    """Distance (in cells) from every cell to the nearest True in ``sources``.

    Same wavefront as ``distance_field`` seeded from many cells at once, so
    "how close is this robot to the fire it can see" costs one pass per cycle rather
    than one per hazard cell.
    """
    dist = np.full(passable.shape, np.inf, dtype=np.float32)
    frontier = sources & passable
    if not frontier.any():
        return dist
    dist[frontier] = 0.0
    d = 0.0
    while frontier.any():
        d += 1.0
        nxt = dilate8(frontier) & passable & np.isinf(dist)
        dist[nxt] = d
        frontier = nxt
    return dist


def disc_template(radius_cells: float) -> tuple[np.ndarray, np.ndarray]:
    """Offsets (dy, dx) of every cell whose centre lies within ``radius_cells``."""
    r = int(np.ceil(radius_cells))
    dy, dx = np.mgrid[-r : r + 1, -r : r + 1]
    keep = (dy * dy + dx * dx) <= radius_cells * radius_cells
    return dy[keep].astype(np.int32), dx[keep].astype(np.int32)


def visible_cells(
    occ: np.ndarray, ox: float, oy: float, tix: np.ndarray, tiy: np.ndarray, cell: float
) -> np.ndarray:
    """Boolean mask over candidate cells: which are in line of sight from (ox, oy).

    Samples ``_LOS_SAMPLES`` fractions along each segment and requires every sample to
    miss a WALL. RUBBLE does not block sight, only speed.
    """
    h, w = occ.shape
    tx = (tix.astype(np.float32) + 0.5) * cell
    ty = (tiy.astype(np.float32) + 0.5) * cell
    vis = np.ones(tix.shape, dtype=bool)
    for f in _LOS_SAMPLES:
        px = ox + f * (tx - ox)
        py = oy + f * (ty - oy)
        six = np.clip((px / cell).astype(np.int32), 0, w - 1)
        siy = np.clip((py / cell).astype(np.int32), 0, h - 1)
        vis &= occ[siy, six] != WALL
    return vis
