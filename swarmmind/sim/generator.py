"""Procedural scenarios, so the commander learns to search *a* map rather than *the* map.

Everything trained so far has seen one scenario at three seeds. That is enough to memorise
a rubble layout and nothing else, and it is the weakness behind the gate's held-out seeds:
they vary the terrain but not the *problem* -- same size, same casualty count, same hazard
clock, same swarm. A commander tuned on that has learned where the casualties usually are.

This draws whole scenarios: map dimensions, obstacle density, terrain character, casualty
count and burial rate, hazard timing and growth, and swarm size. The commander sees a
different search problem every episode and cannot memorise any of them.

**Every generated scenario must be winnable**, or the reward signal is noise. `sample()`
draws, builds the world, checks it, and redraws on failure -- a map whose casualties sit
behind a river no chassis can ford teaches the commander only that the task is hopeless.
"""

from __future__ import annotations

import dataclasses

import numpy as np

from .scenario import Scenario

#: Ranges each knob is drawn from. Centred near the shipped demo so a generated scenario
#: is a sibling of it, not a different game.
RANGES: dict[str, tuple[float, float]] = {
    "width_m": (240.0, 560.0),
    "aspect": (0.55, 0.85),          # height = width x aspect
    "rubble_fraction": (0.20, 0.42),
    "victims_per_10k_m2": (5.0, 11.0),
    "buried_frac": (0.20, 0.55),
    "hazard_ignite_t": (60.0, 200.0),
    "hazard_growth": (0.10, 0.55),
    "robots_per_10k_m2": (35.0, 65.0),
    "hills": (8.0, 28.0),
    "mountains": (0.0, 5.0),
    "rivers": (0.0, 3.0),
    "marshes": (0.0, 6.0),
}

#: Lane proportions, held fixed. These were measured (M-33) and are not the commander's
#: problem to rediscover -- what varies is the map, not the fleet's composition.
LANE_MIX = {"none": 0.46, "scoop": 0.23, "gripper": 0.19, "antenna": 0.12}


def _draw(rng: np.random.Generator, name: str) -> float:
    lo, hi = RANGES[name]
    return float(rng.uniform(lo, hi))


def _lane_counts(total: int) -> dict[str, int]:
    counts = {k: max(1, int(round(total * v))) for k, v in LANE_MIX.items()}
    counts["none"] += total - sum(counts.values())      # absorb rounding into scouts
    return counts


def draw(rng: np.random.Generator, base: Scenario | None = None) -> Scenario:
    """One scenario, drawn but not yet checked. Use `sample()` unless you want the raw draw."""
    base = base or Scenario.load("demo")
    w = round(_draw(rng, "width_m") / 20.0) * 20.0
    h = round(w * _draw(rng, "aspect") / 20.0) * 20.0
    area = w * h / 10_000.0

    n_robots = int(round(area * _draw(rng, "robots_per_10k_m2") / 16.0)) * 16
    n_robots = int(np.clip(n_robots, 64, 900))
    n_victims = int(np.clip(round(area * _draw(rng, "victims_per_10k_m2")), 8, 200))
    buried = int(n_victims * _draw(rng, "buried_frac"))

    # Sector grid tracks map shape, so a sector stays roughly a sector-sized piece of
    # ground however the map is drawn. The commander reasons in sectors; letting them
    # stretch with the map would change what a directive means.
    cols = int(np.clip(round(w / 60.0), 3, 10))
    rows = int(np.clip(round(h / 55.0), 3, 8))

    mp = dataclasses.replace(
        base.map, width_m=w, height_m=h,
        sector_rows=rows, sector_cols=cols,
        rubble_fraction=_draw(rng, "rubble_fraction"),
        n_clusters=int(area * rng.uniform(15.0, 32.0)),
    )
    terrain = dataclasses.replace(
        base.terrain,
        hills=int(_draw(rng, "hills")),
        mountains=int(_draw(rng, "mountains")),
        rivers=int(_draw(rng, "rivers")),
        marshes=int(_draw(rng, "marshes")),
    )
    victims = dataclasses.replace(base.victims, count=n_victims, buried_count=buried)
    hazard = dataclasses.replace(
        base.hazard,
        ignite_t=_draw(rng, "hazard_ignite_t"),
        growth_rate=_draw(rng, "hazard_growth"),
        origin_sector=_pick_sector(rng, rows, cols),
    )
    return dataclasses.replace(
        base, name=f"gen-{int(rng.integers(0, 1_000_000)):06d}",
        map=mp, terrain=terrain, victims=victims, hazard=hazard,
        base=(w * 0.05, h * 0.07),
        extraction_zones=_zones(w, h),
        robots_per_lane=n_robots // 4,
        lane_counts=_lane_counts(n_robots),
        mission_duration_s=base.mission_duration_s,
    )


def _pick_sector(rng: np.random.Generator, rows: int, cols: int) -> str:
    return f"{chr(ord('A') + int(rng.integers(0, rows)))}{int(rng.integers(0, cols)) + 1}"


def _zones(w: float, h: float, spacing: float = 150.0) -> list[tuple[float, float]]:
    """A lattice of collection points, so carry distance scales with the map rather than
    exploding on a large draw."""
    nx = max(2, int(round(w / spacing)))
    ny = max(2, int(round(h / spacing)))
    return [(w * (i + 0.5) / nx, h * (j + 0.5) / ny)
            for j in range(ny) for i in range(nx)][1:]


def sample(seed: int, base: Scenario | None = None, tries: int = 12) -> Scenario:
    """A drawn scenario that is actually winnable.

    Rejection sampling, because a generator that emits unsolvable maps trains a commander
    to believe the task is hopeless. The checks are the cheap ones that catch the failures
    seen in practice: a map sealed off from its base, and casualties nothing can reach.
    """
    from .world import World

    rng = np.random.default_rng(seed)
    last = None
    for _ in range(tries):
        scn = draw(rng, base)
        try:
            w = World(scn, seed)
        except Exception:                                   # noqa: BLE001
            continue
        passable = float(w.passable.mean())
        if not (0.35 <= passable <= 0.95):
            last = f"passable {passable:.0%}"
            continue
        # Every casualty must stand on ground *something* can drive to.
        reach = w.chassis_passable.any(axis=0)
        ix = np.clip((np.array([v.pos[0] for v in w.victims]) / w.cell).astype(int),
                     0, w.shape[1] - 1)
        iy = np.clip((np.array([v.pos[1] for v in w.victims]) / w.cell).astype(int),
                     0, w.shape[0] - 1)
        if not reach[iy, ix].all():
            last = "a casualty is unreachable by every chassis"
            continue
        return scn
    raise RuntimeError(f"no winnable scenario in {tries} draws (last: {last})")
