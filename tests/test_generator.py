"""Procedural scenarios.

The generator exists so the commander learns to search *a* map rather than *the* map. Two
properties matter and both are easy to lose: the draws must actually differ, and every one
of them must be winnable. A generator that quietly emits hopeless maps teaches a policy
that the task is hopeless, and the failure looks like bad training rather than bad data.
"""

from __future__ import annotations

import numpy as np
import pytest

from swarmmind.sim.generator import LANE_MIX, RANGES, draw, sample
from swarmmind.sim.robot import LANES
from swarmmind.sim.world import World


def test_draws_actually_differ():
    """Same generator, different seeds -- if these collide the whole exercise is theatre."""
    scns = [draw(np.random.default_rng(s)) for s in range(12)]
    shapes = {(s.map.width_m, s.map.height_m) for s in scns}
    assert len(shapes) >= 8, f"only {len(shapes)} distinct map shapes in 12 draws"
    assert len({s.victims.count for s in scns}) >= 6
    assert len({s.n_robots for s in scns}) >= 5
    assert len({s.hazard.origin_sector for s in scns}) >= 4


def test_a_draw_is_reproducible():
    """Training runs have to be repeatable, so a seed must pin the whole scenario."""
    a = draw(np.random.default_rng(3))
    b = draw(np.random.default_rng(3))
    assert (a.map.width_m, a.map.height_m) == (b.map.width_m, b.map.height_m)
    assert a.victims.count == b.victims.count and a.n_robots == b.n_robots
    assert a.hazard.origin_sector == b.hazard.origin_sector


@pytest.mark.parametrize("seed", [0, 1, 2, 3])
def test_sampled_scenarios_are_winnable(seed):
    scn = sample(seed)
    w = World(scn, seed)
    assert 0.35 <= w.passable.mean() <= 0.95
    reach = w.chassis_passable.any(axis=0)
    ix = np.clip((np.array([v.pos[0] for v in w.victims]) / w.cell).astype(int),
                 0, w.shape[1] - 1)
    iy = np.clip((np.array([v.pos[1] for v in w.victims]) / w.cell).astype(int),
                 0, w.shape[0] - 1)
    assert reach[iy, ix].all(), "a casualty no chassis can reach"


@pytest.mark.parametrize("seed", [0, 2])
def test_every_lane_is_present_in_a_generated_swarm(seed):
    """A draw with no carriers cannot rescue anyone, and would score every policy zero."""
    scn = sample(seed)
    w = World(scn, seed)
    counts = w.lane_counts()
    assert set(counts) == set(LANES)
    assert all(v > 0 for v in counts.values()), counts
    assert counts["none"] == max(counts.values()), "scouts should dominate"


def test_lane_mix_sums_to_one():
    assert abs(sum(LANE_MIX.values()) - 1.0) < 1e-9


def test_ranges_are_ordered():
    for name, (lo, hi) in RANGES.items():
        assert hi > lo, f"{name} range is inverted"


def test_sector_grid_tracks_map_shape():
    """Sectors are the commander's unit of thought; they must stay a consistent size or a
    directive means something different on every map."""
    for seed in range(6):
        scn = draw(np.random.default_rng(seed))
        w_per = scn.map.width_m / scn.map.sector_cols
        h_per = scn.map.height_m / scn.map.sector_rows
        assert 40.0 <= w_per <= 95.0, f"sector width {w_per:.0f} m on seed {seed}"
        assert 40.0 <= h_per <= 95.0, f"sector height {h_per:.0f} m on seed {seed}"
