"""CLAUDE.md invariant #1: **Tier 2 never depends on Tier 3.**

This is the project's central claim, and it is the one that is easiest to lose by
accident -- not by writing `if directive:` into the auction, but by slowly tuning the
swarm until it only performs well while something is steering it. So the control
condition is run here, in full, with `/hivemind/directives` completely silent.

`hivemind=False` must not be a degraded mode. It must be a mission.
"""

from __future__ import annotations

import numpy as np

from swarmmind.mission import Mission
from swarmmind.perception.tracker import RESOLVED, Report
from swarmmind.sim import grid
from swarmmind.sim.robot import CHASSIS_INDEX, DESTROYED
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import CLEARED

HALF = 210.0  # half a test mission: long enough to allocate, execute and recover


def _mission(**kw):
    return Mission(Scenario.load("test"), 42, **kw)


def test_the_swarm_completes_a_mission_with_the_hivemind_silent():
    m = _mission(hivemind=False)
    card = m.run()
    assert m.hivemind is None
    assert card.directives_issued == 0 and card.directives_rejected == 0
    assert card.victims_found > 0, "no casualty was found without Tier 3"
    assert card.ground_explored_frac > 0.25, (
        f"only {card.ground_explored_frac:.1%} explored with the hivemind off -- "
        "Tier 2 has become dependent on Tier 3"
    )


def test_allocation_happens_without_any_directive():
    m = _mission(hivemind=False)
    for _ in range(400):
        m.tick()
    assigned = sum(1 for a in m.executor.assignment if a is not None)
    assert assigned > 0, "the auction allocated nothing with Tier 3 silent"
    assert not m.world.sector_abandoned.any()
    assert (m.world.sector_priority == 1).all(), "sector priority moved with no hivemind"


def test_self_healing_works_without_any_directive():
    """Destroy the robot holding a task; the swarm must re-offer it on its own.

    The task is a **planted extract** on a cleared casualty, not whatever happened to be
    in `assignment[0]`. That matters: an earlier version grabbed the first assignment it
    found and asserted the exact target was re-awarded within 3 s, which is only
    well-defined for a task that cannot evaporate. It picked an `investigate` whose
    contact was dismissed two seconds later -- nothing left to re-offer, and correctly so.
    A casualty lying cleared on the ground does not go away.
    """
    m = _mission(hivemind=False)
    w = m.world
    for _ in range(200):
        m.tick()

    v = w.victims[0]
    v.state, v.buried, v.debris_remaining = CLEARED, False, 0.0
    m.tracker.reports.append(Report(
        id="rep_selfheal", pos=v.pos.copy(), state=RESOLVED, victim=0,
        n_obs=9, conf_sum=9.0, first_t=w.t, last_t=w.t))

    # Let the auction award the extract, then find who took it.
    holder = -1
    deadline = w.t + 20.0
    while holder < 0 and w.t < deadline:
        m.tick()
        holder = next((i for i, a in enumerate(m.executor.assignment)
                       if a is not None and a.kind == "extract" and a.victim == 0), -1)
    assert holder >= 0, "no robot ever took the planted extract task"

    lost = m.executor.assignment[holder]
    w.status[holder] = DESTROYED

    # Run the full 3 s rather than breaking on the first robot to hold the work.
    #
    # A casualty carries a backup carrier from the moment it is offered
    # (`TaskGenerator.carriers_per_victim`), so "another robot has this task" is already
    # true when the holder dies. Breaking on it would leave this loop before the 2.0 s
    # heartbeat timeout has orphaned the dead robot, and the test would pass without
    # ever exercising the release path it exists to prove.
    took_over = -1
    deadline = w.t + 3.0
    while w.t < deadline:
        m.tick()
        took_over = next((i for i, a in enumerate(m.executor.assignment)
                          if i != holder and a is not None and a.kind == lost.kind
                          and a.victim == 0), -1)
    assert m.executor.assignment[holder] is None, "the dead robot still holds its task"
    assert took_over >= 0, (
        "a lost robot's task was never re-offered within 3 s -- this is the "
        "self-healing claim the whole demo rests on"
    )


def test_the_hivemind_is_a_modifier_not_a_requirement():
    """Both configurations must be missions. Neither may be a placeholder."""
    off = _mission(hivemind=False).run(max_time=HALF)
    on = _mission(scripted_hivemind=True).run(max_time=HALF)
    assert off.directives_issued == 0 and on.directives_issued > 0
    for card, label in ((off, "hivemind off"), (on, "hivemind on")):
        assert card.ground_explored_frac > 0.15, f"{label}: {card.ground_explored_frac:.1%}"
        assert card.victims_found > 0, f"{label}: found nothing"


def test_a_mission_is_deterministic_with_the_hivemind_running():
    """Invariant #6 has to survive Tier 3 or the rehearsed demo timings are worthless."""
    a = _mission(scripted_hivemind=True).run(max_time=HALF)
    b = _mission(scripted_hivemind=True).run(max_time=HALF)
    assert a.hash() == b.hash()
    assert a.directives_issued == b.directives_issued


def test_a_stale_directive_cannot_outlive_the_swarm_that_ignored_it():
    """End-to-end version of the expiry test: nothing stays closed at mission end."""
    m = _mission(scripted_hivemind=True)
    m.run(max_time=HALF)
    m.hivemind._expire.__self__.expiry = 0.0
    m.hivemind._expire(m.world)
    assert not m.world.sector_abandoned.any()
    assert (m.world.sector_priority == 1).all()


def test_the_demo_path_reports_directives_and_wall_clock():
    """`--demo` printed `0 issued, 0 rejected` for a working hivemind.

    `World.scorecard()` cannot know about Tier 3 or the clock, so a caller that prints it
    directly gets zeros in four fields. The realtime path did exactly that, and the
    giveaway -- `wall / rtf 0.00s / 0.00x` on the same line -- is easy to read past. This
    asserts the completion happens, so the demo cannot under-report its headline feature.
    """
    m = _mission(hivemind=True, scripted_hivemind=True)
    for _ in range(400):
        m.tick()

    raw = m.world.scorecard()
    assert raw.wall_seconds == 0.0 and raw.directives_issued == 0, (
        "the world's own scorecard is supposed to leave these to the Mission"
    )

    card = m.scorecard(12.5)
    assert card.wall_seconds == 12.5
    assert card.rtf > 0.0
    assert card.directives_issued == m.hivemind.stats["issued"], (
        "the demo path is not reporting the directives the hivemind actually issued"
    )


def test_the_realtime_path_constructs():
    """`--demo` previously broke because it had no other test coverage.

    `evolved=` was added to `FastSim.__init__` for the gate's control arm; `DemoSim`
    overrides `__init__`, did not take it, and every headless test kept passing because
    they all build a `FastSim`. The demo crashed on startup. One construction is enough
    to catch a signature drifting between a backend and its subclass.
    """
    m = Mission(Scenario.load("test"), 42, realtime=True, hivemind=False)
    m.tick()
    assert m.sim.rtf > 0.0
    assert m.world.t > 0.0

    control = Mission(Scenario.load("test"), 42, realtime=True, hivemind=False,
                      evolved=False)
    assert control.world.specs[0] != m.world.specs[0], (
        "the realtime path ignores `evolved=` -- the gate's control arm would be wrong"
    )


def test_a_rotor_over_dark_ground_lands_to_look():
    """M-71: rotors see nothing airborne, and were airborne 82.9% of their lives."""
    m = Mission(Scenario.load("test"), 42, hivemind=False)
    w = m.world
    rotor = np.nonzero(w.chassis == CHASSIS_INDEX["rotor"])[0]
    assert len(rotor), "fixture has no rotors"
    for _ in range(40):
        m.tick()
    ix, iy = grid.world_to_cell(w.pos[:, 0], w.pos[:, 1], w.cell, w.shape)
    dark_and_flying = w.airborne & ~w.explored[iy, ix]
    assert not dark_and_flying.any(), (
        f"{int(dark_and_flying.sum())} rotors are flying blind over unexplored ground")


def test_a_rotor_still_flies_over_ground_the_swarm_knows():
    """The lane's whole point is the 2x dash; landing everywhere would delete it."""
    m = Mission(Scenario.load("test"), 42, hivemind=False)
    w = m.world
    m.rotors_land_to_look = True
    for _ in range(40):
        m.tick()
    w.explored[:] = True                      # nothing left to look at
    for _ in range(4):
        m.tick()
    rotor = w.chassis == CHASSIS_INDEX["rotor"]
    assert (w.airborne & rotor).any(), "no rotor flies even over a fully explored map"
