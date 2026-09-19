"""CLAUDE.md invariant #5. The rehearsed demo timings depend on this test.

The scorecard tests run the full shipping stack (Mission: Tier 1 + Tier 2). The array
test drives the world layer directly so a determinism break can be localised to the
sim rather than the controller.
"""

from __future__ import annotations

import numpy as np

from swarmmind.mission import run_mission
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World

SHORT = 30.0  # seconds of sim time; enough to exercise motion, fog, comms and victims


def test_same_seed_same_scorecard_hash():
    scn = Scenario.load("test")
    a = run_mission(scn, 42, max_time=SHORT)
    b = run_mission(scn, 42, max_time=SHORT)
    assert a.hash() == b.hash(), "seed 42 is not reproducible -- check for set/dict iteration"


def test_different_seeds_diverge():
    scn = Scenario.load("test")
    a = run_mission(scn, 42, max_time=SHORT)
    b = run_mission(scn, 43, max_time=SHORT)
    assert a.hash() != b.hash()


def test_world_state_identical_arrays():
    """Stronger than the hash: every array must match, not just the summary."""
    scn = Scenario.load("test")
    worlds = []
    for _ in range(2):
        w = World(scn, 42)
        rng = w.rng["noise"]
        from swarmmind.control.wander import wander

        for _ in range(int(SHORT * scn.rates.tick_hz)):
            v, omega = wander(w, rng)
            w.step(v, omega)
        worlds.append(w)
    a, b = worlds
    assert np.array_equal(a.pos, b.pos)
    assert np.array_equal(a.battery, b.battery)
    assert np.array_equal(a.explored, b.explored)
    assert np.array_equal(a.status, b.status)
    assert [v.state for v in a.victims] == [v.state for v in b.victims]
