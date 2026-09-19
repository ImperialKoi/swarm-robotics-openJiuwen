"""Construction-only casualty placement using the simulator's physical terrain.

No display scenery or hidden hints are exposed to the swarm. A wall margin is a
sheltered site; passable rubble is a plausible excavation site. Neither implies
a simulated building interior or an underground cavity.
"""

from __future__ import annotations

import numpy as np

from . import grid


def cover_masks(occ: np.ndarray, cell: float, radius_m: float
                ) -> tuple[np.ndarray, np.ndarray]:
    """Cells within a metric disc of solid obstacles / passable debris.

    The artificial map border is not shelter. Pad with empty space instead of
    wrapping or clipping, which would turn the map edges into preferred sites.
    """
    walls = occ == grid.WALL
    walls[[0, -1], :] = False
    walls[:, [0, -1]] = False
    sources = np.stack([walls, occ == grid.RUBBLE])
    dy, dx = grid.disc_template(radius_m / cell)
    pad = int(np.ceil(radius_m / cell))
    padded = np.pad(sources, ((0, 0), (pad, pad), (pad, pad)))
    near = np.zeros_like(sources)
    h, w = occ.shape
    for oy, ox in zip(dy, dx, strict=True):
        near |= padded[:, pad + oy:pad + oy + h, pad + ox:pad + ox + w]
    return near[0], near[1]


def select_sites(
    xy: np.ndarray, order: np.ndarray, rubble: np.ndarray,
    near_wall: np.ndarray, near_rubble: np.ndarray, *, count: int,
    buried_count: int, cover_fraction: float, separation: float,
) -> list[int]:
    """Keep the weighted draw within each habitat, with deterministic fallbacks.

    Buried casualties first use rubble, then debris margins, then wall margins.
    Other covered casualties prefer walls (which really occlude camera sight).
    An exposed minority prefers open ground. Every pool falls back to the other
    safe candidates on sparse maps; clearance/reachability never get relaxed.
    The first ``buried_count`` returned sites require excavation.
    """
    covered = near_wall | near_rubble

    def ranked(*pools: np.ndarray) -> np.ndarray:
        # Stable category sorting preserves the distance-weighted random order.
        priority = np.full(len(xy), len(pools))
        for rank, pool in reversed(list(enumerate(pools))):
            priority[pool] = rank
        return order[np.argsort(priority[order], kind="stable")]

    burial_order = ranked(rubble, near_rubble, near_wall)
    shelter_order = ranked(near_wall, near_rubble)
    exposed_order = ranked(~covered)
    covered_count = max(buried_count, round(count * cover_fraction))
    available = np.ones(len(xy), dtype=bool)
    chosen: list[int] = []
    for candidates, target in ((burial_order, buried_count),
                               (shelter_order, covered_count),
                               (exposed_order, count)):
        for k in candidates:
            if len(chosen) >= target:
                break
            if available[k]:
                chosen.append(int(k))
                available &= np.sum((xy - xy[k]) ** 2, axis=1) >= separation ** 2
                available[k] = False  # even when separation has relaxed to zero
    return chosen
