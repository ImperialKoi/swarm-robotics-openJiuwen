"""Terrain generation: rolling ground, hills, mountains, water-filled rivers and ditches.

**This is not decoration.** The heightfield used to be render-only; it now produces the
slope and water fields that decide which robots can go where, which is what makes a
heterogeneous swarm mean something. A scout that cannot ford a river and a legged unit
that can is a real difference in capability, and the auction has to route around it.

Still 2.5D: a heightfield with per-cell slope and water depth, not 3D physics. Robots
move in the plane and terrain gates where the plane is traversable.

Three things about the shape of this file are load-bearing:

* **Relief is built from three scales, not one.** Long-wavelength noise gives rolling
  ground, `hills` are the mid-scale bumps you can see from the operator's camera, and
  `mountains` are the few steep peaks that actually stop a chassis. An earlier version
  had only 30 m-wavelength noise plus three peaks and rendered as a *pancake with
  scratches on it* -- the octaves were grain, not landform.
* **Water is a surface elevation, not a depth stamp.** Every wet feature registers a
  `level` (absolute metres) and a `wet` mask; depth is `level - ground` computed once at
  the very end. That is why a river reads as *full of water* rather than as a dry
  trench, and why every later step that raises ground -- a ford, a zone apron, a road
  causeway -- drains itself correctly and for free.
* **Rivers are straight.** A random-walk centreline is not what a river looks like from
  above; it looks like a scribble. The naturalism lives in the cross-section (bed, bank,
  feathered shoulder) and in a water surface that runs downhill, not in the plan view.
"""

from __future__ import annotations

import numpy as np

from . import grid

#: Freeboard: how far a river's banks stand above its water surface. Small, because the
#: point of the change was rivers that look *full*, and a metre of dry bank inside the
#: channel is exactly the trench look it replaced.
BANK_M = 0.55

#: Road grade ceiling and the clearance a deck keeps over the water it bridges. The grade
#: sits well under a wheeled unit's 0.40 limit because `slope_of` reads a 2-cell central
#: difference, so a corridor at exactly the limit reads as over it at the shoulder.
ROAD_GRADE = 0.28
ROAD_FREEBOARD_M = 0.35

#: How a cutting or embankment meets natural ground, and how far out it may reach.
#:
#: Roads used to feather over a fixed 5 m shoulder. That is fine on flat ground and a
#: cliff on a hillside: where the corridor crosses a rise the deck is graded flat and the
#: ground 5 m away is 18 m higher, which reads as a 3.4 gradient -- steep enough that the
#: central-difference slope contaminates the carriageway itself and severs the very route
#: the road exists to guarantee. Grading the sides at a fixed batter instead makes the
#: shoulder as wide as the cut is deep.
ROAD_BATTER = 0.45
ROAD_BATTER_MAX_M = 24.0
#: How sharply a carriageway outvotes a neighbouring road's batter. 1 is a plain mean.
ROAD_DECK_POWER = 6.0
#: Weight given to natural ground as the prior in that mean. See the note where it is used.
ROAD_EPS = 1e-4

#: How deep a river is allowed to cut through ground that rises along its course. The
#: water surface runs downhill (a running minimum of the smoothed ground profile), and
#: without a cap a river crossing a mountain flank gouges a 20 m canyon.
MAX_CUT_M = 3.0

#: Landform octaves as (wavelength in cells, amplitude in metres).
#:
#: The amplitudes are set by the GRADIENT each contributes, not by how each looks: with
#: smoothstep interpolation an octave adds roughly 1.5 x amp/(scale*cell). These sum to
#: ~0.39 worst case and typically far less.
#:
#: The split between the long and short octaves is the whole point. The original
#: ((28,7),(13,3),(6,1.2),(3,0.4)) summed to ~1.27 -- past even a legged robot's limit
#: before a single mountain was placed -- and yet rendered flat, because 30 m is grain
#: rather than a hillside. Moving the amplitude into 150 m and 70 m triples the relief
#: you can see while more than halving the gradient that blocks a chassis.
#:
#: Scaled to 0.8x of the first version that got the look right, which was measured
#: (M-62) to cost 14 points of the casualties a wheeled unit can reach for 0.5 m of
#: hill-scale relief -- a bad trade in a scenario whose score is people carried out.
OCTAVES = ((150, 10.5), (70, 4.4), (32, 1.5), (14, 0.55), (6, 0.24))


def _smoothstep(t: np.ndarray) -> np.ndarray:
    return t * t * (3.0 - 2.0 * t)


def _value_noise(rng: np.random.Generator, shape: tuple[int, int],
                 octaves: tuple[tuple[int, float], ...]) -> np.ndarray:
    """Smoothstep-interpolated value noise. No dependency, deterministic per stream.

    Smoothstep rather than bilinear: linear interpolation of a lattice leaves visible
    diamond creases along the cell diagonals, which is the single biggest reason the old
    landform read as procedural. The price is a 1.5x steeper worst-case gradient per
    octave, which the amplitudes below already account for.
    """
    h, w = shape
    out = np.zeros(shape, dtype=np.float32)
    for scale, amp in octaves:
        gh = max(2, h // scale) + 1
        gw = max(2, w // scale) + 1
        small = rng.random((gh, gw)).astype(np.float32)
        ys = np.linspace(0, gh - 1, h)
        xs = np.linspace(0, gw - 1, w)
        y0 = np.clip(ys.astype(int), 0, gh - 2)
        x0 = np.clip(xs.astype(int), 0, gw - 2)
        fy = _smoothstep((ys - y0)[:, None])
        fx = _smoothstep((xs - x0)[None, :])
        a = small[y0][:, x0]
        b = small[y0][:, x0 + 1]
        c = small[y0 + 1][:, x0]
        d = small[y0 + 1][:, x0 + 1]
        out += ((a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy) * amp
    return out


def _box_blur(a: np.ndarray, r: int) -> np.ndarray:
    """Three separable box passes -- a gaussian of sigma ~= r cells, in O(n) and numpy only.

    Used to build the *smoothed landform* the zone aprons blend toward. Blending toward a
    lightly-blurred copy was one of the four wrong turns on terrain (blurring a trench
    keeps a trench); the thing that makes it work here is the radius. At sigma = 20 m a
    16 m-wide river channel contributes ~0.4 m of residual dip, which is nothing, while a
    60 m hill survives essentially intact.
    """
    if r < 1:
        return a.copy()
    out = a.astype(np.float32)
    k = 2 * r + 1
    for axis in (0, 1):
        for _ in range(3):
            pad = np.pad(out, [(r + 1, r) if i == axis else (0, 0) for i in range(2)],
                         mode="edge")
            cs = np.cumsum(pad, axis=axis, dtype=np.float32)
            lo = np.take(cs, np.arange(0, out.shape[axis]), axis=axis)
            hi = np.take(cs, np.arange(k, out.shape[axis] + k), axis=axis)
            out = (hi - lo) / k
    return out


def _segment_frame(gx: np.ndarray, gy: np.ndarray, a: np.ndarray, b: np.ndarray
                   ) -> tuple[np.ndarray, np.ndarray]:
    """(perpendicular distance, along-parameter in [0,1]) for a straight segment.

    Analytic, because the centreline is straight. The old per-step splat loop existed
    only to follow a wobbling path; without the wobble it is 300 iterations of numpy
    doing what two dot products do exactly.
    """
    seg = b - a
    length2 = float(seg @ seg)
    if length2 < 1e-9:
        return np.full(gx.shape, np.inf, np.float32), np.zeros(gx.shape, np.float32)
    rel_x, rel_y = gx - a[0], gy - a[1]
    t = np.clip((rel_x * seg[0] + rel_y * seg[1]) / length2, 0.0, 1.0)
    px = rel_x - t * seg[0]
    py = rel_y - t * seg[1]
    return np.sqrt(px * px + py * py).astype(np.float32), t.astype(np.float32)


def _sample_along(z: np.ndarray, cell: float, a: np.ndarray, b: np.ndarray, n: int
                  ) -> np.ndarray:
    """Ground height at ``n`` evenly-spaced points along the segment."""
    t = np.linspace(0.0, 1.0, n)
    xs = a[0] + (b[0] - a[0]) * t
    ys = a[1] + (b[1] - a[1]) * t
    ix, iy = grid.world_to_cell(xs, ys, cell, z.shape)
    return z[iy, ix].astype(np.float32)


def _smooth1d(h: np.ndarray, r: int, *, pin_ends: bool = False) -> np.ndarray:
    """Moving average. With ``pin_ends``, the result is corrected to meet ``h`` exactly
    at both endpoints, tapering the correction away over ``2r`` samples.

    Pinning is not cosmetic. Roads share their endpoints -- eleven edges over twelve
    collection points -- and each profile is smoothed over its own length, so an edge
    climbing away from a junction is biased upward at that junction while a flat edge
    leaving the same junction is not. On seed 51 the two decks out of base disagreed by
    1.5 m *at base*, and the corridor that lost the weighted vote stepped 2.4 m in one
    cell four metres from the staging area: the road out of base was severed for wheels
    by an averaging artefact, not by terrain.
    """
    if r < 1 or h.size < 3:
        return h.copy()
    r = min(r, h.size // 2)
    pad = np.pad(h, r, mode="edge")
    ker = np.ones(2 * r + 1, dtype=np.float32) / (2 * r + 1)
    out = np.convolve(pad, ker, mode="valid").astype(np.float32)
    if pin_ends:
        taper = np.clip(1.0 - np.arange(h.size, dtype=np.float32) / max(2 * r, 1),
                        0.0, 1.0)
        out += (h[0] - out[0]) * taper + (h[-1] - out[-1]) * taper[::-1]
    return out


def _slope_limit(h: np.ndarray, ds: float, g: float, *, raise_only: bool = False,
                 pin_ends: bool = False) -> np.ndarray:
    """A profile whose along-track gradient never exceeds ``g``, close to ``h``.

    Lower envelope of the cones ``h[j] + lim*|k-j|`` -- the min-plus distance transform,
    computed exactly by one forward and one backward pass. A pointwise minimum of
    Lipschitz functions is Lipschitz, so the result satisfies the bound by construction
    and needs no iteration. ``raise_only`` takes the mirror upper envelope instead, so a
    section lifted for clearance over water keeps its clearance and grows approach ramps
    rather than being clamped back down into the river.

    ``pin_ends`` holds the first and last samples fixed, by first clipping ``h`` into the
    cones reachable from both endpoints. Both properties are needed together and neither
    is optional:

    * Roads share their endpoints, so two decks leaving one collection point must leave
      it at the same height or the corridor that loses the weighted vote steps in one
      cell -- 2.4 m, four metres from the staging area, on seeds 44/51/55.
    * The obvious pinning -- iterating a two-sided clamp with the ends held -- does not
      converge. Where the ground climbs out of a pad faster than the grade allows, the
      forward pass pulls the profile down toward the pinned start and the backward pass
      pulls it back up toward the high tail, and it oscillates: measured on seed 51,
      six passes left a 3.60 m step against a 0.283 m limit. The envelope has a closed
      form and no fixed-point to miss.
    """
    n = h.size
    p = h.astype(np.float32).copy()
    if n < 2:
        return p
    lim = np.float32(g * ds)
    if pin_ends:
        k = np.arange(n, dtype=np.float32)
        rev = k[::-1]
        hi = np.minimum(h[0] + lim * k, h[-1] + lim * rev)
        lo = np.maximum(h[0] - lim * k, h[-1] - lim * rev)
        # `lo <= hi` unless the endpoints are further apart than the grade can span, which
        # a 105 m road at 0.28 cannot be; the minimum keeps `clip` well-defined anyway.
        p = np.clip(p, np.minimum(lo, hi), hi)
    if raise_only:
        for i in range(1, n):
            p[i] = max(p[i], p[i - 1] - lim)
        for i in range(n - 2, -1, -1):
            p[i] = max(p[i], p[i + 1] - lim)
    else:
        for i in range(1, n):
            p[i] = min(p[i], p[i - 1] + lim)
        for i in range(n - 2, -1, -1):
            p[i] = min(p[i], p[i + 1] + lim)
    return p


def _point_to_segment(px: float, py: float, a: np.ndarray, b: np.ndarray) -> float:
    seg = b - a
    length2 = float(seg @ seg)
    if length2 < 1e-9:
        return float(np.hypot(px - a[0], py - a[1]))
    t = np.clip(((px - a[0]) * seg[0] + (py - a[1]) * seg[1]) / length2, 0.0, 1.0)
    return float(np.hypot(px - (a[0] + t * seg[0]), py - (a[1] + t * seg[1])))


def _river_course(rng: np.random.Generator, index: int, width_m: float, height_m: float,
                  keepouts: list[tuple[float, float, float]], clearance: float,
                  tries: int = 48) -> tuple[np.ndarray, np.ndarray]:
    """Endpoints of a straight edge-to-edge course that misses the collection points.

    Two rules, both learned from looking at the map rather than at the code.

    *Alternating axis.* Drawing two independent pairs of edges gave seed 42 two
    near-parallel rivers lying on top of each other in the bottom third, which divides
    nothing. Crossing alternate axes is what makes rivers a routing problem.

    *Clearance.* A river through a collection point is drained and levelled by that
    point's apron, so it arrives on screen as a chain of disconnected ponds -- the exact
    look this whole change set out to remove. Siting the channel away from the depots
    instead of repairing it afterwards is also just what happens in the world.
    """
    horizontal = (index % 2 == 0)
    best: tuple[float, np.ndarray, np.ndarray] | None = None
    for _ in range(tries):
        if horizontal:
            a = np.array([-2.0, rng.uniform(0.08, 0.92) * height_m])
            b = np.array([width_m + 2.0, rng.uniform(0.08, 0.92) * height_m])
        else:
            a = np.array([rng.uniform(0.08, 0.92) * width_m, -2.0])
            b = np.array([rng.uniform(0.08, 0.92) * width_m, height_m + 2.0])
        gap = min((_point_to_segment(kx, ky, a, b) for kx, ky, _ in keepouts),
                  default=np.inf)
        if best is None or gap > best[0]:
            best = (gap, a, b)
        if gap >= clearance:
            break
    assert best is not None
    return best[1], best[2]


def generate(rng: np.random.Generator, occ: np.ndarray, cell: float, cfg,
             keepouts: list[tuple[float, float, float]] | None = None,
             base_cell: tuple[int, int] | None = None,
             passable: np.ndarray | None = None,
             max_slope: float = 0.58,
             roads: list[tuple[float, float]] | None = None,
             road_edges: list[tuple[int, int]] | None = None,
             road_half_width_m: float = 2.4
             ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(height, terrain, water) in metres.

    ``height`` includes rubble and wall bumps and is what gets rendered.
    ``terrain`` is the smooth landform, and is what slope is measured from -- taking the
    gradient of ``height`` instead would read every one-metre rubble block as a cliff and
    make the whole map impassable.
    ``water`` is depth, zero on dry land.
    """
    shape = occ.shape
    h, w = shape
    width_m, height_m = w * cell, h * cell

    # Rolling ground. See OCTAVES for why the amplitudes are what they are.
    z = _value_noise(rng, shape, OCTAVES)

    gx, gy = grid.cell_centres(shape, cell)

    # --- hills: many, rounded, mostly traversable ------------------------------------
    # The mid scale, and the feature that makes the map read as landscape rather than as
    # a plain with three lumps on it. Height comes from a steepness ratio for the same
    # reason mountains' does -- what matters is the gradient, and a gaussian's steepest
    # point is ~1.21 x (peak / radius). At 0.11-0.26 that is 0.13-0.31: under a wheeled
    # unit's 0.40 limit, so hills shape routes without severing them. Mountains are what
    # sever.
    for _ in range(cfg.hills):
        cx, cy = rng.uniform(0, width_m), rng.uniform(0, height_m)
        rad = rng.uniform(*cfg.hill_radius_m)
        peak = rad * rng.uniform(*cfg.hill_steepness)
        d2 = ((gx - cx) ** 2 + (gy - cy) ** 2) / (rad * rad)
        z += (peak * np.exp(-2.0 * d2)).astype(np.float32)

    # --- mountains: steep, tall, few -------------------------------------------------
    # Kept as separate fields so individual peaks can be attenuated later. A mountain
    # flank is *meant* to stop a tracked robot -- that is the locomotion axis doing its
    # job -- but a ring of them around base makes the map unrunnable, and flattening
    # everything to fix that throws away the differentiation entirely.
    # Height is derived from a STEEPNESS ratio, not set directly, because slope is what
    # gates traversal and absolute height does not. For a gaussian peak the maximum
    # gradient is 1.27 x (peak / radius) -- a tighter gaussian than a hill's 1.21 -- so a
    # ratio range spanning the chassis limits guarantees some flanks stop wheeled units
    # only and others stop tracked ones too.
    mountain_fields: list[np.ndarray] = []
    for _ in range(cfg.mountains):
        cx, cy = rng.uniform(0, width_m), rng.uniform(0, height_m)
        rad = rng.uniform(*cfg.mountain_radius_m)
        peak = rad * rng.uniform(*cfg.mountain_steepness)
        d = np.sqrt((gx - cx) ** 2 + (gy - cy) ** 2) / rad
        mountain_fields.append((peak * np.exp(-2.2 * d * d)).astype(np.float32))
    mountain_scale = np.ones(len(mountain_fields), dtype=np.float32)

    def _with_mountains(base: np.ndarray) -> np.ndarray:
        out = base.copy()
        for f, sc in zip(mountain_fields, mountain_scale, strict=True):
            out += f * sc
        return out

    base_z = z.copy()
    z = _with_mountains(base_z)

    # --- attenuate only the peaks that wall the base in -------------------------------
    if base_cell is not None and passable is not None and mountain_fields:
        bx, by = base_cell
        for _ in range(24):
            mask = passable & (slope_of(z, cell) <= max_slope)
            if mask[by, bx]:
                reach = grid._flood(mask, (bx, by))
                if reach.sum() >= 0.85 * mask.sum():
                    break
            else:
                reach = np.zeros_like(mask)
            # Blame the mountain contributing most to the barrier that is holding the
            # reachable region in, and shrink that one. Distant peaks keep their flanks.
            barrier = grid.dilate8(reach) & passable & ~mask if reach.any() else ~mask
            if not barrier.any():
                barrier = ~mask
            blame = [float((f * sc)[barrier].sum())
                     for f, sc in zip(mountain_fields, mountain_scale, strict=True)]
            worst = int(np.argmax(blame))
            if blame[worst] <= 0.0:
                break
            mountain_scale[worst] *= 0.7
            z = _with_mountains(base_z)

    z -= z.min()

    # Water is carried as a SURFACE ELEVATION plus a mask, never as a depth. Depth is
    # `level - ground`, evaluated once at the end of this function, so every later step
    # that raises ground -- fords, zone aprons, road causeways -- drains itself and
    # cannot leave water perched on top of a hill.
    #
    # `cap` is the other half of the contract: no feature may end up deeper than the
    # depth it was drawn with. Without it, any later step that *lowers* ground under
    # standing water deepens it without limit, and the combinations are not obvious --
    # a marsh whose basin a river then carves through produced a 12 m lake. Capping is
    # not a fudge: it says the feature has a bed at `level - cap`, which is what a
    # marsh_depth_m of 0.38 was always supposed to mean.
    wet = np.zeros(shape, dtype=bool)
    level = np.zeros(shape, dtype=np.float32)
    cap = np.zeros(shape, dtype=np.float32)

    def _flood(mask: np.ndarray, surface: np.ndarray, depth: float,
               override: bool = False) -> None:
        """Flood ``mask`` to ``surface``. Deeper water wins unless ``override``.

        Rivers override: a river cuts through whatever it crosses, so where a channel
        runs over a marsh the channel's surface is the one that counts.
        """
        take = mask if override else (mask & (~wet | (surface > level)))
        level[take] = surface[take]
        cap[take] = depth
        wet[mask] = True

    def _depth() -> np.ndarray:
        """Water depth as it stands right now: the one definition, used everywhere.

        `level` on its own is not the water surface. Where features overlap, a cell can
        keep the level of the deeper pool while its ground has since been cut away by
        something else, and `level - ground` is then metres of notional water that `cap`
        immediately truncates. Reading raw `level` as a surface is what made the road
        pass build an 11 m causeway to bridge 0.38 m of marsh, and the approach ramps it
        graded to reach that deck are what severed the road out of base.
        """
        return np.where(wet, np.clip(np.minimum(level - z, cap), 0.0, None), 0.0)

    # --- ditches: narrow, straight, shorter, dry --------------------------------------
    # A legged-only shortcut, not a barrier that cuts the map -- and straight, for the
    # same reason the rivers are. Cut before anything is flooded, so a ditch can never
    # drill a hole through the bed of a river that was already filled.
    for _ in range(cfg.ditches):
        start = np.array([rng.uniform(0, width_m), rng.uniform(0, height_m)])
        ang = rng.uniform(0, 2 * np.pi)
        length = rng.uniform(*cfg.ditch_length_m)
        end = np.clip(start + np.array([np.cos(ang), np.sin(ang)]) * length,
                      [0.0, 0.0], [width_m, height_m])
        hw = rng.uniform(*cfg.ditch_width_m) * 0.5
        depth = rng.uniform(*cfg.ditch_depth_m)
        d, _t = _segment_frame(gx, gy, start, end)
        z -= (depth * np.clip(1.0 - (d / hw) ** 2, 0.0, 1.0)).astype(np.float32)

    # --- marsh: broad, shallow, and the reason tracked units exist --------------------
    #
    # Without a barrier in the 0 to 0.45 m band, tracked robots are strictly dominated by
    # wheeled ones: same reach, 18% slower, no reason to build any. Marsh stops wheels
    # and nothing else, which is precisely the niche that makes the middle chassis worth
    # its slot.
    #
    # A pool sits LEVEL, and its level is the lowest ground in the basin plus the marsh's
    # depth -- not the basin's median, which on sloping ground put the surface metres
    # above the downhill side and produced a 14 m lake. Filling from the floor up bounds
    # the depth at `marsh_depth_m` by construction, whatever the ground under it does,
    # and gives a natural waterline where the level meets the bank.
    for _ in range(cfg.marshes):
        cx, cy = rng.uniform(0, width_m), rng.uniform(0, height_m)
        rad = rng.uniform(*cfg.marsh_radius_m)
        depth = rng.uniform(*cfg.marsh_depth_m)
        prof = np.clip(1.0 - ((gx - cx) ** 2 + (gy - cy) ** 2) / (rad * rad), 0.0, 1.0)
        basin = prof > 0.02
        if not basin.any():
            continue
        z -= (prof * depth * 0.55).astype(np.float32)   # a shallow basin, not a trench
        lvl = float(z[basin].min()) + depth
        _flood(basin & (z < lvl), np.full(shape, lvl, dtype=np.float32), depth)

    # --- rivers: straight, edge to edge, and full of water ---------------------------
    #
    # Straight on purpose. The old centreline was a random walk (`river_wobble`), which
    # from the operator's camera reads as a scribble rather than as a river; naturalism
    # comes from the cross-section and from a surface that runs downhill, not from the
    # plan view. Edge to edge so they actually divide the map.
    for river_i in range(cfg.rivers):
        hw0 = rng.uniform(*cfg.river_width_m) * 0.5
        a, b = _river_course(rng, river_i, width_m, height_m, keepouts or [],
                             clearance=hw0 * 2.6 + 18.0)
        depth = rng.uniform(*cfg.river_depth_m)
        d, t = _segment_frame(gx, gy, a, b)
        span = float(np.linalg.norm(b - a))
        n = max(8, int(span / cell))

        # Water surface: the smoothed ground along the course, forced downhill by a
        # running minimum so the river never flows uphill, and capped so it cuts at most
        # MAX_CUT_M through rising ground instead of gouging a canyon through a peak.
        prof0 = _smooth1d(_sample_along(z, cell, a, b, n), max(2, int(40.0 / cell)))
        if prof0[-1] > prof0[0]:
            surf1d = np.minimum.accumulate(prof0[::-1])[::-1]
        else:
            surf1d = np.minimum.accumulate(prof0)
        surf1d = np.maximum(surf1d, prof0 - MAX_CUT_M) - BANK_M

        idx = np.clip((t * (n - 1)).astype(np.int32), 0, n - 1)
        surf = surf1d[idx]

        # Width breathes gently along the course. The centreline stays dead straight --
        # this is the difference between "a river" and "a canal", and costs one sine.
        hw = hw0 * (1.0 + 0.22 * np.sin(t * rng.uniform(2.0, 4.5) * np.pi
                                        + rng.uniform(0.0, 6.28)))
        u = d / np.maximum(hw, 1e-3)

        # Cross-section, relative to the water surface:
        #   u <= 1      parabolic bed, `depth` at the centre, waterline at the rim
        #   1 < u <= 1.4  bank climbing BANK_M out of the water
        #   u > 1.4     bank top, feathered back into natural ground by u = 2.3
        bed = -depth * np.clip(1.0 - u * u, 0.0, 1.0)
        bank = BANK_M * np.clip((u - 1.0) / 0.4, 0.0, 1.0)
        target = surf + np.where(u <= 1.0, bed, bank)
        blend = np.clip((2.3 - u) / 0.9, 0.0, 1.0)
        z = np.where(u <= 2.3, z * (1.0 - blend) + target * blend, z).astype(np.float32)

        # The channel is full to `surf`; a river overrides whatever it crosses, because
        # that is what a river does to a marsh.
        _flood(u <= 1.0, surf, depth, override=True)

        # Fords. A river deep enough to stop a tracked robot severs the map -- 7 of 20
        # seeds produced a base cut off from most of its own ground. Real terrain has
        # crossings, and having to route to one is the interesting behaviour; being
        # unable to cross at all is just a broken scenario. A ford is a gaussian rise in
        # the ground; the water over it drains by itself, because depth is derived. Its
        # radius is the river's own half-width, which puts the steepest part of the rise
        # at ~1.21 x amp/radius -- about 0.24, so a wheeled unit can use the crossing.
        for f in range(cfg.river_fords):
            ct = (f + 0.5) / max(1, cfg.river_fords)
            cx = a[0] + (b[0] - a[0]) * ct
            cy = a[1] + (b[1] - a[1]) * ct
            fr = max(hw0, 5.0)
            amp = depth + BANK_M + 0.30
            z += (amp * np.exp(-((gx - cx) ** 2 + (gy - cy) ** 2) / (fr * fr))
                  ).astype(np.float32)

    # --- flatten around base and the collection points --------------------------------
    # Landform is generated without regard to where the mission needs flat, dry ground,
    # so a river can be carved straight through the base. On one test seed that left
    # *zero* reachable cells: the staging area was cut off from the entire map and the
    # mission was unwinnable before it started.
    #
    # Two failed attempts are worth recording. Blending toward a *lightly* blurred copy
    # fills nothing: a river carved through the base leaves a 2 m trench, a small blur of
    # a trench is still a trench, and draining it merely produced a dry ravine with
    # vertical walls -- base slope 3.5, impassable to everything. Filling to a level over
    # a narrow disc is no better: it puts a cliff at the disc's own rim wherever the
    # surroundings differ in height.
    #
    # What works is two stages against a HEAVILY smoothed landform. Stage one blends
    # toward `z_soft` (sigma ~ 20 m), which erases channels and grain but keeps the hill
    # the zone sits on; stage two levels a small pad inside that. Because stage two
    # blends toward a field that is already smooth, the rim gradient it can introduce is
    # bounded by the smooth field's own gradient -- no cliff is reachable.
    #
    # **The apron radius matters more than anything else in this file.** It used to be
    # 60 m around each of twelve fixed points on a 360 x 240 m map, level-filled: the
    # discs covered ~90% of the map and overlapped, so *that*, not the octaves, is why
    # the shipped map rendered as a pancake. Measured again at 43 m it still softened
    # 55% of the map -- enough to drain most of both rivers, because a channel blended
    # toward a 20 m blur of itself is a channel no longer. At 26 m (69 m for base) it
    # touches 29% and levels far less of that, which is the difference between "the
    # collection points are usable" and "the terrain has been deleted around them".
    #
    # Base is the exception and gets a much wider pad: 512 robots need ~36 m of staging
    # room before they gridlock against their own separation radius
    # (tests/test_terrain.py::test_staging_area_scales_with_the_swarm).
    base_xy = (None if base_cell is None
               else ((base_cell[0] + 0.5) * cell, (base_cell[1] + 0.5) * cell))
    z_soft = _box_blur(z, max(4, int(round(20.0 / cell))))
    for kx, ky, kr in keepouts or []:
        pad = max(kr, 10.0)
        if base_xy is not None and abs(kx - base_xy[0]) < cell and abs(ky - base_xy[1]) < cell:
            pad *= 2.6
        d2 = (gx - kx) ** 2 + (gy - ky) ** 2
        soft = np.clip(1.0 - d2 / (pad * 2.2) ** 2, 0.0, 1.0) ** 1.5
        z = (z * (1.0 - soft) + z_soft * soft).astype(np.float32)
        inside = d2 <= (pad * 0.8) ** 2
        if not inside.any():
            continue
        flat = np.clip(1.0 - d2 / (pad * 1.5) ** 2, 0.0, 1.0) ** 2.0
        z = (z * (1.0 - flat) + float(np.median(z[inside])) * flat).astype(np.float32)
        wet &= d2 > (pad * 1.1) ** 2

    # --- grade and drain the roads ----------------------------------------------------
    #
    # Clearing obstacles along a corridor does not make a road: on the shipped seed the
    # corridors still crossed marsh and hillside, so a wheeled unit reached 2% of the map
    # while a legged one reached 98%. A road is by definition traversable, so each one is
    # regraded and drained. This is also what gives the wheeled chassis its domain --
    # fastest on the network, useless off it.
    #
    # The profile FOLLOWS the ground (smoothed, then slope-limited) instead of ramping
    # straight between the endpoints. On the old flat map the two were the same thing;
    # with hills on it, a straight ramp is a cutting with vertical sides wherever the
    # ground rises over it, which looks like a canyon and drives like one. Water is
    # bridged rather than deleted: the reference profile sits ROAD_FREEBOARD_M above the
    # surface anywhere the corridor is wet, so a road crossing a river is a causeway.
    if roads and road_edges:
        depth_now = _depth()
        under_water = depth_now > 0.0
        surface = z + depth_now
        z_ref = np.where(under_water, surface + ROAD_FREEBOARD_M, z).astype(np.float32)
        # Corridors are accumulated and applied ONCE, rather than one `np.where` per edge
        # overwriting the last. Eleven edges meeting at twelve points overlap near every
        # junction, and on a map with relief the two profiles crossing there disagree by
        # metres -- written sequentially that disagreement lands as a one-cell cliff at
        # each corridor's rim. Measured on seed 51: 214 such spikes, up to 5.2 gradient,
        # two of them on the only road out of base, which is what put wheeled units on
        # 2% of the map. Blending the corridors by weight makes the result continuous by
        # construction, because every term in it is.
        acc_w = np.zeros(shape, dtype=np.float32)
        acc_h = np.zeros(shape, dtype=np.float32)
        acc_r = np.zeros(shape, dtype=np.float32)
        carriageway = np.zeros(shape, dtype=bool)
        for i, j in road_edges:
            a = np.array(roads[i], dtype=np.float64)
            b = np.array(roads[j], dtype=np.float64)
            span = float(np.linalg.norm(b - a))
            if span < 1e-3:
                continue
            n = max(8, int(span / cell))
            ds = span / (n - 1)
            prof = _smooth1d(_sample_along(z_ref, cell, a, b, n),
                             max(2, int(12.0 / cell)), pin_ends=True)
            prof = _slope_limit(prof, ds, ROAD_GRADE, pin_ends=True)
            # The causeway is not optional and cannot be left to the smoothed profile:
            # smoothing averages a river crossing against the banks either side and pulls
            # the deck back under the water, which is a bridge a wheeled unit drowns on.
            # Lift the crossing, then re-establish the gradient by raising the approaches
            # rather than by dropping the deck.
            crossing = _sample_along(under_water.astype(np.float32),
                                     cell, a, b, n) > 0.5
            if crossing.any():
                lvl = _sample_along(surface, cell, a, b, n)
                prof = np.where(crossing, np.maximum(prof, lvl + ROAD_FREEBOARD_M), prof)
                prof = _slope_limit(prof, ds, ROAD_GRADE, raise_only=True,
                                    pin_ends=True)

            d, t = _segment_frame(gx, gy, a, b)
            ramp = prof[np.clip((t * (n - 1)).astype(np.int32), 0, n - 1)]
            # Flat across the carriageway and a cell beyond it, so the central-difference
            # slope on the road reads the road and not its shoulder; then a batter as
            # wide as the cut is deep.
            flat_to = road_half_width_m + cell
            outer = flat_to + np.clip(np.abs(ramp - z) / ROAD_BATTER, 2.0,
                                      ROAD_BATTER_MAX_M)
            near = d <= outer
            if not near.any():
                continue
            wgt = _smoothstep(np.clip((outer - d) / (outer - flat_to), 0.0, 1.0))
            # Two accumulators, on purpose. `acc_w` says how much road is here at all and
            # feathers the outermost batter into natural ground. The height itself is a
            # sharply weighted vote (`** ROAD_DECK_POWER`) so that a carriageway beats the
            # batter of a road crossing near it: under a plain weighted mean, a 27 m-wide
            # cutting alongside dragged its neighbour's deck up and down with it and put
            # 1.56-gradient cells on the only road out of base.
            hard = wgt ** ROAD_DECK_POWER
            acc_r += hard * ramp
            acc_h += hard
            acc_w += wgt
            carriageway |= d <= road_half_width_m * 1.2
        blend = np.clip(acc_w, 0.0, 1.0)
        # Natural ground is the prior in the weighted mean, not a floor on the
        # denominator. Clamping the denominator instead (`acc_r / max(acc_h, 1e-6)`)
        # returns a *fraction* of the right height out where the weights are tiny, so the
        # far edge of every batter was pulled down by ~9% of its own elevation -- a 2.4 m
        # trench ringing each corridor on a 27 m-high hillside, which is what was still
        # severing the road out of base on seeds 44, 51 and 55. As `acc_h` goes to zero
        # this form goes to `z`, so the whole expression goes to `z`, which is the answer.
        ramp = (acc_r + z * ROAD_EPS) / (acc_h + ROAD_EPS)
        z = (z * (1.0 - blend) + ramp * blend).astype(np.float32)
        wet &= ~carriageway

    # Renormalise ground AND every water surface together. Shifting one without the
    # other silently drains or floods the whole map: ditches cut below zero, so the
    # final shift is ~2 m, which was enough to empty every marsh on one seed.
    floor = float(z.min())
    z -= floor
    level -= floor
    terrain = z.copy()

    # Depth, at last: the water that a surface elevation implies over the final ground,
    # never deeper than the feature that put it there.
    water = _depth()
    water[water < 0.02] = 0.0
    water = water.astype(np.float32)

    # --- rubble and structure on top -------------------------------------------------
    full = z.copy()
    rub = occ == grid.RUBBLE
    full[rub] += 0.55 + rng.random(shape).astype(np.float32)[rub] * 0.9
    wall = occ == grid.WALL
    full[wall] += 0.9 + rng.random(shape).astype(np.float32)[wall] * 1.3

    return full.astype(np.float32), terrain.astype(np.float32), water


def slope_of(terrain: np.ndarray, cell: float) -> np.ndarray:
    """Gradient magnitude of the landform: rise over run, dimensionless."""
    dzdy, dzdx = np.gradient(terrain, cell)
    return np.sqrt(dzdx * dzdx + dzdy * dzdy).astype(np.float32)
