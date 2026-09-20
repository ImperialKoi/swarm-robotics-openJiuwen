"""Tier 2: the auction, and the self-healing claim the whole demo rests on.

CLAUDE.md invariant #1: allocation, execution and recovery must all work with
/hivemind/directives completely silent. Nothing in this file involves an LLM.
"""

from __future__ import annotations

import numpy as np
import pytest

from swarmmind.control.planner import UNREACHABLE
from swarmmind.mission import Mission
from swarmmind.nodes.skill_executor import Assignment, capable, eligible
from swarmmind.nodes.tasks import RANK, OpenTask
from swarmmind.perception.tracker import RESOLVED, Report
from swarmmind.sim.robot import DESTROYED, LANE_INDEX, OUT_OF_COMMS
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import CLEARED


def _mission(alloc="auction", seed=42):
    return Mission(Scenario.load("test"), seed, allocator=alloc)


def _plant_cleared_victim(m, vi=0):
    """Put a known, cleared casualty on the board so an extract task exists at once."""
    w = m.world
    v = w.victims[vi]
    v.state, v.buried, v.debris_remaining = CLEARED, False, 0.0
    r = Report(id=f"rep_test_{vi}", pos=v.pos.copy(), state=RESOLVED, victim=vi,
               n_obs=9, conf_sum=9.0, first_t=w.t, last_t=w.t)
    m.tracker.reports.append(r)
    return v, r


# --- eligibility ----------------------------------------------------------------------


def test_capability_gating_is_physical():
    w = _mission().world
    assert (capable(w, "clear_debris") == (w.actuator == LANE_INDEX["scoop"])).all()
    assert (capable(w, "extract") == (w.actuator == LANE_INDEX["gripper"])).all()
    assert (capable(w, "relay") == (w.actuator == LANE_INDEX["antenna"])).all()
    assert capable(w, "explore").all()


@pytest.mark.parametrize("kind", ["explore", "investigate", "extract", "clear_debris"])
def test_relays_are_never_given_search_work(kind):
    """A relay that takes a frontier walks out past the chain it is holding up, and the
    swarm drops out of contact behind it -- a cascade that ended one test run with
    0 of 16 robots in comms."""
    w = _mission().world
    assert not eligible(w, kind)[w.actuator == LANE_INDEX["antenna"]].any()


def test_relay_tasks_still_go_to_relays():
    w = _mission().world
    assert (eligible(w, "relay") == (w.actuator == LANE_INDEX["antenna"])).all()


# --- bidding --------------------------------------------------------------------------


def test_award_goes_to_the_lowest_bidder():
    m = _mission()
    w = m.world
    v, _ = _plant_cleared_victim(m)
    # Pick a LEGGED gripper: casualties can sit on ground a wheeled unit cannot stand
    # on, and a bidder that cannot reach the target reads UNREACHABLE and correctly
    # loses. This test is about the bid comparison, not about traversability.
    from swarmmind.sim.robot import CHASSIS_INDEX

    grippers = np.nonzero(w.actuator == LANE_INDEX["gripper"])[0]
    w.chassis[grippers] = CHASSIS_INDEX["legged"]
    w.chassis_passable = w._build_chassis_passability()
    w.pos[grippers] = np.array(w.scn.base)
    w.pos[grippers[0]] = v.pos + np.array([1.5, 0.0])
    w.battery[:] = 1.0
    # Casualties are placed far from base, so the near gripper would be out of comms and
    # therefore not allowed to bid. Force full connectivity: this test is about the bid
    # comparison, not about the comms model.
    w.in_comms[:] = True

    m.allocator.step(w, m.executor, m.tracker)
    a = m.executor.assignment[int(grippers[0])]
    assert a is not None and a.kind == "extract" and a.victim == 0


def test_bidding_does_not_scale_with_robot_count():
    """One BFS per task target, cached for the mission -- not one A* per (robot, task).

    At 512 robots the pair-wise form is ~10k searches a second and the project does not
    scale. Guard it by counting field computations, which must track tasks, not robots.
    """
    m = _mission()
    w = m.world
    before = m.nav.misses
    for _ in range(int(3 * w.scn.rates.tick_hz)):
        m.tick()
    fields = m.nav.misses - before
    assert fields < 200, f"{fields} distance fields for a 3 s window -- bidding is per-robot"


# --- self-healing: the central claim ---------------------------------------------------


def test_destroyed_robot_task_is_reassigned_within_three_seconds():
    """**M1.** Heartbeat timeout (2.0 s) plus one auction cycle (1.0 s) = 3.0 s.

    No LLM is involved, and nothing special-cases the death: the robot stops
    heartbeating, its assignment is released, and the work reappears in the next
    cycle's task list because tasks are regenerated from world state.
    """
    m = _mission()
    w = m.world
    v, _ = _plant_cleared_victim(m)
    w.battery[:] = 1.0
    w.in_comms[:] = True
    m.allocator.step(w, m.executor, m.tracker)

    holder = next(i for i, a in enumerate(m.executor.assignment)
                  if a is not None and a.kind == "extract" and a.victim == 0)
    w._kill(holder, "test")
    assert w.status[holder] == DESTROYED

    # Run the whole 3 s window rather than stopping at the first sign of life.
    #
    # A casualty now carries a *backup* carrier from the moment it is offered
    # (`TaskGenerator.carriers_per_victim`), so "some other robot holds this work" is
    # true before the holder is even killed, and breaking on it would exit this loop
    # before the 2.0 s heartbeat timeout has had a chance to orphan the dead robot --
    # passing the test without ever exercising the self-healing path it exists to guard.
    t0 = w.t
    reassigned = -1
    while w.t - t0 <= 3.0:
        m.tick()
        for i, a in enumerate(m.executor.assignment):
            if i != holder and a is not None and a.victim == 0 and a.kind == "extract":
                reassigned = i
    assert reassigned >= 0, f"task never reassigned within 3 s (t={w.t - t0:.2f})"
    assert w.actuator[reassigned] == LANE_INDEX["gripper"]
    assert m.executor.assignment[holder] is None, "a destroyed robot must not hold work"

    kinds = [e["kind"] for e in m.events.events]
    assert "task_orphaned" in kinds and "task_awarded" in kinds


def test_out_of_comms_robot_keeps_working_but_its_task_is_re_offered():
    """Releasing an out-of-contact robot frees it to bid on something else -- and for a
    relay that means abandoning the post the rest of the swarm talks through."""
    m = _mission()
    w = m.world
    _plant_cleared_victim(m)
    w.in_comms[:] = True
    m.allocator.step(w, m.executor, m.tracker)
    holder = next(i for i, a in enumerate(m.executor.assignment) if a is not None)

    m.allocator.last_hb[holder] = w.t - 99.0     # silent, but alive
    m.allocator.step(w, m.executor, m.tracker)

    a = m.executor.assignment[holder]
    assert a is not None, "a live robot must keep executing while out of contact"
    assert a.orphaned is True, "the work must go back on the market"


def test_orphaned_assignment_releases_its_claim():
    m = _mission()
    w = m.world
    _plant_cleared_victim(m)
    w.in_comms[:] = True
    m.allocator.step(w, m.executor, m.tracker)
    holder = next(i for i, a in enumerate(m.executor.assignment)
                  if a is not None and a.victim == 0)

    tasks = m.allocator.gen.generate(w, m.executor, m.tracker)
    assert not any(t.victim == 0 for t in tasks), "claimed work must not be re-offered"

    m.executor.assignment[holder].orphaned = True
    tasks = m.allocator.gen.generate(w, m.executor, m.tracker)
    assert any(t.victim == 0 for t in tasks), "orphaned work must be re-offered"


# --- fault injection -------------------------------------------------------------------


def test_fault_is_state_triggered_not_wall_clock():
    """A fixed timestamp lands at an awkward moment if the run goes long or short."""
    m = _mission()
    w = m.world
    cfg = m.fault.cfg
    w.t = cfg.min_t + 1.0
    m.fault.step(w, m.executor, m._emit)
    assert m.fault.fired_at is None, "fired before the rescue precondition was met"

    for k in range(m.fault.cfg.min_rescued):
        w.victims[k].state = 4
    m.fault.step(w, m.executor, m._emit)
    assert m.fault.fired_at is not None


def test_fault_fires_once_and_destroys_exactly_one_robot():
    m = _mission()
    w = m.world
    w.t = m.fault.cfg.fallback_t + 1.0
    before = w.robots_lost
    for _ in range(5):
        m.fault.step(w, m.executor, m._emit)
    assert w.robots_lost == before + 1
    assert m.fault.victim_robot is not None


def test_fault_prefers_a_loaded_carrier():
    """Dropping the casualty makes the recovery visible rather than merely logged."""
    m = _mission()
    w = m.world
    i = int(np.nonzero(w.actuator == LANE_INDEX["gripper"])[0][0])
    w.carrying[i] = 0
    w.victims[0].state, w.victims[0].carrier = 3, i
    m.executor.assign(w, i, Assignment(task_id="x", kind="extract", victim=0,
                                       target=tuple(w.victims[0].pos)))
    w.t = m.fault.cfg.fallback_t + 1.0
    m.fault.step(w, m.executor, m._emit)
    assert m.fault.victim_robot == w.robot_ids[i]
    assert w.victims[0].state == CLEARED, "the casualty must be dropped, not deleted"


def test_one_casualty_is_only_ever_offered_once_per_cycle():
    """Duplicate reports must not multiply tasks -- the count is policy, not report count.

    Several reports can resolve onto one casualty; each used to become its own task.
    `claimed_v` dedups against *live assignments*, which none of them are yet in the
    cycle they are first offered -- so on seed 42 casualty v6 was awarded to three
    carriers at t=396, each ~255 m away, for one rescue.

    The fix is the `offered` set, and it is what this guards. A casualty is now
    deliberately offered `carriers_per_victim` times as insurance against a holder that
    stalls, so the invariant is no longer "exactly one" -- it is that the number is set
    by policy and does not grow with the number of corroborating reports. Three reports
    and thirty must produce the same tasks, exactly one of which may preempt.

    The casualty is put into `CLEARED` directly rather than waiting for the mission to
    produce one, so the test asserts every run instead of skipping when the fixture
    happens not to have dug anyone out yet.
    """
    from swarmmind.nodes.tasks import TaskGenerator

    m = Mission(Scenario.load("test"), 42, hivemind=False)
    for _ in range(200):
        m.tick()
    w = m.world
    v = w.victims[0]
    v.state, v.debris_remaining = CLEARED, 0.0

    # Three independent reports corroborating one casualty -- the normal case, and what
    # the tracker is for.
    counts = []
    for n_reports in (3, 30):
        m.tracker.reports = [r for r in m.tracker.reports if not r.id.startswith("dup")]
        m.tracker.reports.extend(
            Report(id=f"dup{k}", pos=np.asarray(v.pos, dtype=float),
                   state=RESOLVED, victim=0)
            for k in range(n_reports)
        )
        m.executor.assignment[:] = [None] * w.n

        gen = TaskGenerator()
        for_target = [t for t in gen.generate(w, m.executor, m.tracker) if t.victim == 0]
        counts.append(len(for_target))
        assert len(for_target) == gen.carriers_per_victim, (
            f"{len(for_target)} tasks for one casualty from {n_reports} reports -- "
            f"expected carriers_per_victim ({gen.carriers_per_victim})"
        )
        # Exactly one may preempt; the rest take idle robots or nobody.
        assert sum(1 for t in for_target if not t.backup) == 1, (
            "more than one preempting task for a single casualty -- carriers go in a pack"
        )
    assert counts[0] == counts[1], (
        f"task count grew with report count ({counts}) -- duplicate reports are "
        f"multiplying tasks again"
    )


def test_a_near_busy_robot_beats_a_far_free_one_for_a_casualty():
    """Preemption has to compete, not wait until nobody free can bid.

    The old rule ran preemption only when `_best_bidder` found nobody at all, so a free
    carrier on the far side of the map always beat a closer one that happened to be
    exploring -- measured at 258 m versus 4 m on seed 42.

    **The far robot has to be reachable, not merely standing on passable ground.** The
    previous fixture took the geometrically farthest cell of `chassis_passable`, which on
    the sedimentary-cliff map is across the river and disconnected: the nav field returns
    `UNREACHABLE`, `_best_bidder` answers `(-1, 0.0)`, and the comparison became
    `0.1 < 0.0` -- unfalsifiable, and about connectivity rather than preemption. Pick the
    farthest cell *by nav distance* and the two bids are comparable again.
    """
    m = Mission(Scenario.load("test"), 42, hivemind=False)
    for _ in range(200):
        m.tick()
    w, auc = m.world, m.allocator
    grippers = np.nonzero(eligible(w, "extract") & (w.status <= OUT_OF_COMMS) & w.in_comms)[0]
    if len(grippers) < 2:
        pytest.skip("fixture has too few carriers to distinguish near from far")

    near, far = int(grippers[0]), int(grippers[1])
    target = (float(w.pos[near, 0]), float(w.pos[near, 1]))

    # Farthest cell the far robot's own chassis can actually *drive* to, measured with
    # the same accessor the auction bids through -- `nav.field` is on the 4x-downsampled
    # navigation grid while `chassis_passable` is fine, so the two cannot be indexed
    # against each other and `distance_at` is what reconciles them.
    chassis = int(w.chassis[far])
    field = auc.nav.field(chassis, target[0], target[1])
    cells = np.argwhere(w.chassis_passable[chassis])
    xs = (cells[:, 1] + 0.5) * w.cell
    ys = (cells[:, 0] + 0.5) * w.cell
    d = auc.nav.distance_at(chassis, field, xs, ys)
    d = np.where(d >= UNREACHABLE * 0.5, -1.0, d)
    assert d.max() > 0, "no reachable ground at all; the fixture cannot pose the question"
    pick = int(np.argmax(d))
    w.pos[far] = (float(xs[pick]), float(ys[pick]))
    assert np.linalg.norm(w.pos[far] - np.asarray(target)) > 20.0, "near and far coincide"

    task = OpenTask("extract", target, RANK["extract"], victim=0, report="r0", value=3.0)

    # The near one is busy on lower-priority work; the far one is free.
    m.executor.assignment[:] = [None] * w.n
    m.executor.assign(w, near, Assignment(task_id="x", kind="explore",
                                          target=(float(w.pos[far, 0]), float(w.pos[far, 1]))))
    free = np.zeros(w.n, dtype=bool)
    free[far] = True

    j, pscore = auc._best_preemptable(w, m.executor, task)
    i, score = auc._best_bidder(w, task, free)
    assert j == near, "the busy robot standing on the casualty is not even a candidate"
    assert i == far, "the far carrier cannot reach the casualty; this tests nothing"

    # The predicate the allocator actually evaluates (auction.py, `step`). Asserting only
    # the ratio half silently passes or fails on the `i < 0` branch instead.
    preempts = j >= 0 and (i < 0 or pscore < score * auc.PREEMPT_RATIO)
    assert preempts, (
        f"preempting the near robot ({pscore:.1f}s) does not beat the far free one "
        f"({score:.1f}s) by the required margin -- the auction will send the far one"
    )


def test_a_carrier_mid_delivery_is_never_preempted():
    """A loaded carrier finishes its delivery, whatever else needs doing.

    The one interruption that loses work already done: the casualty goes back on the
    ground and the trip so far is spent for nothing. `_best_preemptable` skips any robot
    with `carrying >= 0` for exactly this reason, and it holds no matter how close that
    robot happens to be standing to the new task.
    """
    m = Mission(Scenario.load("test"), 42, hivemind=False)
    for _ in range(200):
        m.tick()
    w, auc = m.world, m.allocator
    grippers = np.nonzero(eligible(w, "extract") & (w.status <= OUT_OF_COMMS) & w.in_comms)[0]
    if len(grippers) < 1:
        pytest.skip("fixture has no carrier to load")

    carrier = int(grippers[0])
    target = (float(w.pos[carrier, 0]), float(w.pos[carrier, 1]))
    m.executor.assignment[:] = [None] * w.n
    m.executor.assign(w, carrier, Assignment(task_id="d", kind="deliver", target=target))
    task = OpenTask("extract", target, RANK["extract"], victim=0, report="r0", value=3.0)

    w.carrying[carrier] = -1
    j_free, _ = auc._best_preemptable(w, m.executor, task)
    w.carrying[carrier] = 0                      # now holding a casualty
    j_loaded, _ = auc._best_preemptable(w, m.executor, task)

    assert j_loaded != carrier, (
        "a carrier holding a casualty was offered up for preemption; it would drop the "
        "victim it is already delivering")
    assert j_free == carrier, (
        "the same carrier is not preemptable even when empty, so the loaded assertion "
        "above would pass for the wrong reason")

def test_a_robot_never_wins_a_goal_its_chassis_cannot_reach():
    """`UNREACHABLE` is 1e9 -- a finite sentinel, not infinity.

    Divided by `v_max` it makes a huge but *finite* bid, which `np.isfinite` accepts, so
    the robot wins any round where no reachable robot is free and then stands still for
    the rest of the mission holding a task it can never execute. Measured at t=120 on
    seed 42: 156 of the 326 robots parked at base were in exactly that state.

    The distance field is stubbed rather than hunting the fixture map for a genuinely
    disconnected target: what is under test is the bid, not the topology.
    """
    from swarmmind.control.planner import UNREACHABLE

    m = Mission(Scenario.load("test"), 42, hivemind=False)
    for _ in range(100):
        m.tick()
    w, auc = m.world, m.allocator
    free = (w.status <= OUT_OF_COMMS) & w.in_comms
    task = OpenTask("explore", (float(w.pos[0, 0]), float(w.pos[0, 1])), RANK["explore"])
    real = auc.nav.distance_at

    auc.nav.distance_at = lambda c, f, x, y: np.full(w.n, float(UNREACHABLE))
    try:
        i, _ = auc._best_bidder(w, task, free)
        assert i < 0, (
            f"{w.robot_ids[i]} won a task nothing can reach -- it will hold the "
            "assignment and never move"
        )
        # ...and the guard must not reject work that *is* reachable.
        auc.nav.distance_at = lambda c, f, x, y: np.full(w.n, 12.0)
        j, _ = auc._best_bidder(w, task, free)
        assert j >= 0, "the unreachable guard is rejecting reachable work"
    finally:
        auc.nav.distance_at = real
