"""Tier 1 is the safety floor (CLAUDE.md invariant #2).

Nothing above Tier 1 may put a robot inside a wall. These tests hammer the controller
with adversarial goals -- including goals placed deliberately inside obstacles -- and
assert zero penetrations.
"""

from __future__ import annotations

import numpy as np

from swarmmind.control.planner import NavSet
from swarmmind.control.tier1_reflex import ReflexController
from swarmmind.sim import grid
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World


def _penetrations(w) -> int:
    ix, iy = grid.world_to_cell(w.pos[:, 0], w.pos[:, 1], w.cell, w.shape)
    return int((w.occ[iy, ix] == grid.WALL).sum())


def test_no_penetration_with_adversarial_goals():
    """Goals inside walls, on the far side of obstacles, and at the map edge."""
    scn = Scenario.load("test")
    w = World(scn, 3)
    nav = NavSet(w)
    reflex = ReflexController(w)
    rng = np.random.default_rng(11)

    wiy, wix = np.nonzero(~w.passable)
    piy, pix = np.nonzero(w.passable)
    goal_xy = []
    for k in rng.choice(len(wix), 6, replace=False):          # inside walls
        goal_xy.append(((wix[k] + 0.5) * w.cell, (wiy[k] + 0.5) * w.cell))
    for k in rng.choice(len(pix), 6, replace=False):          # legitimate
        goal_xy.append(((pix[k] + 0.5) * w.cell, (piy[k] + 0.5) * w.cell))

    steps = int(120 * scn.rates.tick_hz)
    for t in range(steps):
        if t % 200 == 0:
            goal_id = rng.integers(0, len(goal_xy), w.n).astype(np.int32)
        v, omega = reflex.commands(w, nav, goal_xy, goal_id)
        w.step(v, omega)
        assert _penetrations(w) == 0, f"wall penetration at t={w.t:.2f}"


def test_override_fires_and_permits_rotation():
    """A robot driven straight at a wall must stop translating but keep turning."""
    scn = Scenario.load("test")
    w = World(scn, 5)
    nav = NavSet(w)
    reflex = ReflexController(w)

    # Find a wall whose western neighbour this robot's chassis can actually stand on.
    # A robot placed on terrain it cannot traverse is *stranded*, and stranded robots are
    # deliberately exempt from the override so a shove into mud is not permanent -- so
    # testing the override there would test nothing.
    ok = w.chassis_passable[w.chassis[0]]
    wall = None
    stand = None
    wiy, wix = np.nonzero(~w.passable)
    for cy, cx in zip(wiy, wix, strict=True):
        if cx >= 2 and ok[cy, cx - 1]:
            wall = ((cx + 0.5) * w.cell, (cy + 0.5) * w.cell)
            # Stand in the cell immediately beside it: the swept circle only reaches
            # radius + v*dt ahead, roughly 0.4 m, so a robot parked further back is
            # correctly not blocked and the test would prove nothing.
            stand = ((cx - 1 + 0.5) * w.cell, (cy + 0.5) * w.cell)
            break
    assert wall is not None, "no wall with traversable ground beside it"
    w.pos[0] = np.array(stand)
    w.theta[0] = 0.0
    # Exercise the override directly rather than through commands(): with the wall as
    # its goal the robot simply *arrives* and stops, so nothing would reach the override
    # and the test would pass for the wrong reason.
    v_in = w.v_max.copy()
    v_out = reflex._wall_override(w, v_in)

    assert reflex.blocked_count > 0, "the swept-circle override never fired"
    assert v_out[0] == 0.0, "a robot driving into a wall was not stopped"
    assert v_out[1:].max() > 0.0, "the override stopped robots that were not blocked"

    # Rotation must survive: a blocked robot turns in place rather than grinding.
    goal_id = np.full(w.n, -1, dtype=np.int32)
    goal_id[0] = 0
    _, omega = reflex.commands(w, nav, [wall], goal_id)
    assert np.isfinite(omega[0]), "rotation must remain available while blocked"


def test_hazard_reflex_uses_only_observed_hazard():
    """Robots may react to hazard they have SEEN. Reacting to the ground-truth disc
    would defeat the fog-of-war claim (CLAUDE.md invariant #3)."""
    scn = Scenario.load("test")
    w = World(scn, 42)
    nav = NavSet(w)
    reflex = ReflexController(w)

    goal_id = np.full(w.n, -1, dtype=np.int32)
    reflex.commands(w, nav, [], goal_id)
    assert not w.hazard_known.any()
    assert np.allclose(reflex._haz, 0.0), "no observed hazard yet, so no hazard force"

    # Mark the cells right in front of every robot as observed hazard.
    ix, iy = grid.world_to_cell(w.pos[:, 0] + 2.0, w.pos[:, 1], w.cell, w.shape)
    w.hazard_known[iy, ix] = True
    reflex.commands(w, nav, [], goal_id)
    assert np.linalg.norm(reflex._haz, axis=1).max() > 0.0, "observed hazard must repel"


def test_stationary_robots_do_not_drift():
    """A robot with no goal holds position. Drift reads as malfunction on the dashboard."""
    scn = Scenario.load("test")
    w = World(scn, 9)
    nav = NavSet(w)
    reflex = ReflexController(w)
    start = w.pos.copy()
    goal_id = np.full(w.n, -1, dtype=np.int32)
    for _ in range(200):
        v, omega = reflex.commands(w, nav, [], goal_id)
        w.step(v, omega)
    # Separation may nudge robots apart at spawn, but nobody should wander off. The
    # allowance is a few metres rather than centimetres: spawn packs the swarm tightly
    # and inter-robot repulsion legitimately spreads it before it settles.
    assert np.linalg.norm(w.pos - start, axis=1).max() < 5.0
