"""Tier 2 execution invariants.

Every test here corresponds to a bug that actually happened. They are cheap
and they each cost real debugging time, so none of them are hypothetical.
"""

from __future__ import annotations

import numpy as np
import pytest

from swarmmind.control.heuristic import GreedyAssigner
from swarmmind.control.planner import NavSet
from swarmmind.control.tier1_reflex import ReflexParams
from swarmmind.mission import Mission
from swarmmind.nodes import skill_executor
from swarmmind.nodes.skill_executor import Assignment, SkillExecutor, capable
from swarmmind.nodes.tasks import hazard_preemptions, nearest_haven
from swarmmind.sim import grid
from swarmmind.sim.robot import LANE_INDEX, OUT_OF_COMMS
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import CLEARED, REACH_DIG, REACH_GRAB, World


@pytest.fixture
def w():
    return World(Scenario.load("test"), 42)


def test_interaction_reach_exceeds_stop_radius():
    """Tier 1 parks a robot within STOP_RADIUS of a goal snapped to the cell grid. If
    the world's interaction reach is smaller than that, carriers park just outside
    pickup range and no victim is ever rescued."""
    for scenario in ("test", "demo"):
        scn = Scenario.load(scenario)
        worst = max(SkillExecutor.STOP_RADIUS.values()) + scn.map.cell
        assert worst < REACH_GRAB, f"{scenario}: REACH_GRAB {REACH_GRAB} <= {worst}"
        assert worst < REACH_DIG, f"{scenario}: REACH_DIG {REACH_DIG} <= {worst}"


def test_capability_gating(w):
    assert capable(w, "explore").all()
    assert (capable(w, "clear_debris") == (w.actuator == LANE_INDEX["scoop"])).all()
    assert (capable(w, "extract") == (w.actuator == LANE_INDEX["gripper"])).all()
    assert (capable(w, "relay") == (w.actuator == LANE_INDEX["antenna"])).all()


def test_relay_holds_its_post(w):
    """A relay that reports done on arrival gets an explore task, walks off, and the
    comms chain collapses behind it -- taking the swarm out of contact."""
    ex = SkillExecutor(w)
    i = int(np.nonzero(w.actuator == LANE_INDEX["antenna"])[0][0])
    post = tuple(w.pos[i])
    ex.assign(w, i, Assignment(task_id="relay_1", kind="relay", target=post, assigned_at=w.t))
    for _ in range(5):
        w.t += 10.0
        ex.update(w, arrive_radius=1.5)
    assert ex.assignment[i] is not None, "an on-station relay must not be released"


def test_relay_abandons_a_burning_post(w):
    """Holding unconditionally is how relays burn: the hazard grows over the post and a
    task with no completion condition keeps the robot there until its battery is gone."""
    ex = SkillExecutor(w)
    i = int(np.nonzero(w.actuator == LANE_INDEX["antenna"])[0][0])
    post = tuple(w.pos[i])
    ex.assign(w, i, Assignment(task_id="relay_1", kind="relay", target=post, assigned_at=w.t))
    ix, iy = grid.world_to_cell(np.asarray(post[0]), np.asarray(post[1]), w.cell, w.shape)
    w.hazard_known[int(iy), int(ix)] = True
    ex.update(w, arrive_radius=1.5)
    assert ex.assignment[i] is None


def test_stalled_uses_geodesic_progress_not_euclidean(w):
    """A robot routing around an obstacle increases its straight-line distance. Judging
    that as a stall releases healthy tasks mid-journey."""
    ex = SkillExecutor(w)
    nav = NavSet(w)
    i = 0
    iy, ix = np.nonzero(w.passable)
    far = ((ix[-1] + 0.5) * w.cell, (iy[-1] + 0.5) * w.cell)
    a = Assignment(task_id="e1", kind="explore", target=far, assigned_at=w.t)
    ex.assign(w, i, a)
    ex._stalled(w, nav, i, a)
    first = a.last_dist
    assert np.isfinite(first)
    # Move the robot straight away from the goal in a line; geodesic distance rises,
    # so no progress is recorded, but the task is not released until the timeout.
    w.pos[i] = np.array(w.scn.base)
    w.t += 5.0
    assert ex._stalled(w, nav, i, a) is False, "5 s without progress is not yet a stall"
    w.t += 60.0
    assert ex._stalled(w, nav, i, a) is True


def test_arrived_robot_is_never_stalled(w):
    ex = SkillExecutor(w)
    nav = NavSet(w)
    i = 0
    a = Assignment(task_id="e1", kind="explore", target=tuple(w.pos[i]), assigned_at=w.t)
    ex.assign(w, i, a)
    w.t += 500.0
    assert ex._stalled(w, nav, i, a) is False


def test_out_of_comms_idle_robot_closes_on_the_nearest_link(w):
    """Without this reflex an out-of-comms robot can never be reassigned, sits forever,
    keeps its discoveries buffered, and the shared map stops growing -- deadlocking the
    whole swarm.

    It aims at the **nearest robot still in contact**, not at base. Aiming at base was
    the original reflex and it worked, expensively: on seed 42 it sent 30 robots a median
    281 m home to regain a link that was often tens of metres away, dragging them off the
    frontier where the unexplored ground is (M-53).
    """
    ex = SkillExecutor(w)
    i = 0
    w.in_comms[i] = False
    goal_xy, goal_id = ex.goals(w)
    assert goal_id[i] >= 0
    assert "out of contact" in ex.reason[i]

    goal = np.asarray(goal_xy[goal_id[i]])
    linked = np.nonzero(w.in_comms & (w.status <= OUT_OF_COMMS))[0]
    d = np.linalg.norm(w.pos[linked] - goal, axis=1)
    assert d.min() <= w.cell, "the goal is not on any robot that is actually in contact"

    # With nobody in contact there is no link to close on, and base is the only
    # rendezvous the swarm can agree on without coordinating.
    w.in_comms[:] = False
    ex._dirty = True
    goal_xy2, goal_id2 = ex.goals(w)
    assert goal_xy2[goal_id2[i]] == pytest.approx(
        (round(w.scn.base[0] / w.cell) * w.cell, round(w.scn.base[1] / w.cell) * w.cell)
    )


def test_comms_change_alone_refreshes_the_goal_cache(w):
    """`goals()` is cached behind a dirty flag that tracked assignments and carrying.

    Comms state became a goal *input* when recovery started aiming at the nearest link,
    so a change in it has to invalidate the cache on its own -- otherwise a robot that
    has just lost contact keeps the stale answer (no goal) until something unrelated
    happens to dirty the cache.
    """
    ex = SkillExecutor(w)
    ex.idle_explore = False        # isolate the recovery reflex from the drift fallback
    i = 0
    _, goal_id = ex.goals(w)
    assert goal_id[i] == -1, "with drift off, an idle robot in contact has no goal"
    assert ex._dirty is False, "the cache should be clean after a rebuild"

    w.in_comms[i] = False                    # the only thing that changes
    _, goal_id = ex.goals(w)
    assert goal_id[i] >= 0, (
        "losing contact did not invalidate the goal cache -- the recovery reflex is "
        "invisible until an unrelated assignment happens to dirty it"
    )


def test_an_idle_robot_in_contact_still_gets_somewhere_to_be(w):
    """No idle units. An unemployed robot drifts at unexplored ground, it does not park.

    The auction can only employ as many robots as it has tasks, and task supply is
    bounded by design, so at swarm scale there are always more free robots than tasks.
    Before this, `goals()` returned -1 for every one of them and they stood still --
    84 to 131 robots of 512 at every sample on seed 42.
    """
    ex = SkillExecutor(w)
    w.in_comms[:] = True
    w.explored[:] = False                     # nothing surveyed: every sector is dark

    _, goal_id = ex.goals(w)
    idle = [i for i in range(w.n) if ex.assignment[i] is None]
    assert idle, "fixture has no idle robots to test"
    assert all(goal_id[i] >= 0 for i in idle), (
        "an idle robot in contact was left with no goal -- it will stand still"
    )

    # Shared targets, not one per robot: Tier 1 builds one flow field per goal.
    assert len(set(int(goal_id[i]) for i in idle)) <= len(w.sector_ids)


def test_drift_stops_once_the_map_is_covered(w):
    """The fallback is bounded by `IDLE_EXPLORE_UNTIL`, not perpetual motion for its own
    sake: ground no chassis can stand in would otherwise be chased forever."""
    from swarmmind.nodes.skill_executor import IDLE_EXPLORE_UNTIL

    ex = SkillExecutor(w)
    w.in_comms[:] = True
    w.explored[:] = True                      # everything seen -> nothing left to drift at
    ex._dark_at = -1e9
    assert len(ex._dark_points(w)) == 0, (
        f"sectors at 100% still attract idle robots (threshold {IDLE_EXPLORE_UNTIL})"
    )


def test_goals_are_deduplicated(w):
    """One flow field per distinct goal is what makes Tier 1 affordable at 512 robots."""
    ex = SkillExecutor(w)
    target = tuple(w.pos[0] + np.array([5.0, 5.0]))
    for i in range(w.n):
        w.in_comms[i] = True
        ex.assign(w, i, Assignment(task_id=f"e{i}", kind="explore", target=target))
    goal_xy, goal_id = ex.goals(w)
    assert len(goal_xy) == 1
    assert (goal_id == 0).all()


def test_goal_cache_invalidates_on_pickup(w):
    """An extract goal switches from the victim to an extraction zone at pickup. A stale
    cache would keep steering the loaded carrier back at the victim it is holding."""
    ex = SkillExecutor(w)
    i = int(np.nonzero(w.actuator == LANE_INDEX["gripper"])[0][0])
    v = w.victims[0]
    v.state, v.buried, v.debris_remaining = CLEARED, False, 0.0
    ex.assign(w, i, Assignment(task_id="x1", kind="extract", target=tuple(v.pos), victim=0))
    before = ex.goals(w)[0][ex.goals(w)[1][i]]
    assert before == pytest.approx(
        (round(v.pos[0] / w.cell) * w.cell, round(v.pos[1] / w.cell) * w.cell)
    )
    w.carrying[i] = 0
    v.state, v.carrier = 3, i     # CARRIED
    after = ex.goals(w)[0][ex.goals(w)[1][i]]
    assert after != before, "goal must switch to an extraction zone once carrying"


def test_antenna_robots_are_not_given_search_work():
    """Letting relays take explore tasks makes them walk out past the chain they are
    holding up, and the swarm drops out of comms behind them."""
    scn = Scenario.load("test")
    w = World(scn, 42)
    ex = SkillExecutor(w)
    a = GreedyAssigner(w)
    for _ in range(40):
        a._next_t = 0.0
        a.step(w, ex, None)
    for i, asg in enumerate(ex.assignment):
        if asg is not None and w.actuator[i] == LANE_INDEX["antenna"]:
            assert asg.kind in ("relay", "retreat"), f"relay r{i} given {asg.kind}"


def test_robot_in_observed_hazard_is_pre_empted():
    """Tier 1 only sees ~6 m of hazard. Inside a front tens of metres across every probe
    reads hazardous and there is no usable gradient -- the robot needs a destination."""
    scn = Scenario.load("test")
    w = World(scn, 42)
    ex = SkillExecutor(w)
    i = 0
    ex.assign(w, i, Assignment(task_id="e1", kind="explore", target=tuple(w.pos[i] + 5.0)))
    ix, iy = grid.world_to_cell(w.pos[i, 0], w.pos[i, 1], w.cell, w.shape)
    w.hazard_known[int(iy), int(ix)] = True
    assert i in hazard_preemptions(w, ex)
    assert nearest_haven(w, i) is not None


def test_full_stack_rescues_and_finds():
    """End-to-end: the four-lane chain (scout finds, digger clears, carrier extracts,
    relay keeps them in contact) must actually complete on the test fixture."""
    m = Mission(Scenario.load("test"), 42, params=ReflexParams())
    sc = m.run()
    # Deliberately low. This fixture is 16 robots on a map that now has a river, a
    # mountain and ditches, and its job is to catch a break in the chain -- perception
    # to allocation to extraction -- not to certify performance. The tracker assertions
    # below are the real signal that each stage is doing something.
    assert sc.victims_found >= 2, f"only found {sc.victims_found}/8"
    assert sc.victims_rescued >= 1, "the extraction chain never completed"
    # Terrain now gates where each chassis can go, so full coverage of the fixture is
    # no longer expected -- and should not be. Ground only some robots can reach is
    # the locomotion axis working.
    assert sc.ground_explored_frac > 0.45

    # Thresholds are lower than they were against the ground-truth oracle, and that is
    # the point: casualties are now found by a detector that misses some and
    # hallucinates others, not by being told where they are.
    st = m.tracker.stats
    assert st["resolved"] > 0, "no report ever resolved onto a real casualty"
    assert st["dismissed"] > 0, "no phantom was ever dismissed -- perception is too easy"


# --------------------------------------------------------------- relay role ladder

def _relay_world():
    """A world where every antenna robot is idle and parked on base."""
    w = World(Scenario.load("test"), 42)
    relays = np.nonzero(w.actuator == LANE_INDEX["antenna"])[0]
    assert len(relays) >= 3, "fixture needs an antenna lane"
    w.pos[relays] = np.asarray(w.scn.base, dtype=float)
    w.status[:] = 0
    w.in_comms[:] = True
    return w, relays


def test_relay_rung1_bridges_short_of_the_cut_off_robot():
    """A cut-off robot outranks everything, and the relay stops inside its own net."""
    w, relays = _relay_world()
    victim = int(np.nonzero(w.actuator != LANE_INDEX["antenna"])[0][0])
    w.pos[victim] = np.asarray(w.scn.base, dtype=float) + [120.0, 0.0]
    w.in_comms[victim] = False

    orders = skill_executor._relay_orders(w, relays)
    assert len(orders) == 1, "one cut-off robot must consume exactly one relay"
    (gx, gy), why = next(iter(orders.values()))
    assert "cut-off" in why
    gap = np.linalg.norm(np.array([gx, gy]) - w.pos[victim])
    assert gap == pytest.approx(w.scn.comms.relay_radius * 0.8, rel=1e-6), (
        "the relay must stop at radius, not walk onto the robot and go dark too")


def test_relay_rung2_reinforces_a_link_at_the_edge_of_range():
    """With nobody cut off, the lane shores up the link that is about to break."""
    w, relays = _relay_world()
    base = np.asarray(w.scn.base, dtype=float)
    br = w.scn.comms.base_radius
    others = np.nonzero(w.actuator != LANE_INDEX["antenna"])[0]
    w.pos[others] = base                                   # everyone else is safe
    edge = int(others[0])
    w.pos[edge] = base + [br * 0.95, 0.0]                  # ... except this one

    orders = skill_executor._relay_orders(w, relays)
    assert len(orders) == 1
    (gx, gy), why = next(iter(orders.values()))
    assert "edge of range" in why
    d = np.linalg.norm(np.array([gx, gy]) - base)
    assert d < np.linalg.norm(w.pos[edge] - base), (
        "shoring up means standing between the anchor and the robot, not beyond it")


def test_relay_rung3_is_no_order_at_all():
    """A healthy net returns nothing, so surplus relays fall through to dark ground."""
    w, relays = _relay_world()
    w.pos[w.actuator != LANE_INDEX["antenna"]] = np.asarray(w.scn.base, dtype=float)
    assert skill_executor._relay_orders(w, relays) == {}


def test_relay_orders_never_pile_up():
    """The M-67 defect: independent nearest-lookup put 72 relays on one point."""
    w, relays = _relay_world()
    base = np.asarray(w.scn.base, dtype=float)
    cut = np.nonzero(w.actuator != LANE_INDEX["antenna"])[0][:3]
    for n, j in enumerate(cut):
        w.pos[j] = base + [100.0 + 10.0 * n, 20.0 * n]
        w.in_comms[j] = False

    orders = skill_executor._relay_orders(w, relays)
    assert len(orders) == len(cut), "one relay per cut-off robot, and no more"
    goals = {g for g, _ in orders.values()}
    assert len(goals) == len(cut), "every claiming relay gets a distinct goal"


def test_a_relay_going_to_recharge_does_not_claim_a_robot_it_will_not_reach():
    """The recharge branch outranks the lane's orders, so it must not consume a claim."""
    w = World(Scenario.load("test"), 42)
    relays = np.nonzero(w.actuator == LANE_INDEX["antenna"])[0]
    base = np.asarray(w.scn.base, dtype=float)
    w.pos[relays] = base
    w.status[:] = 0
    w.in_comms[:] = True
    w.battery[relays] = 1.0
    w.battery[relays[0]] = 0.1                     # this one is heading for the pad

    ex = SkillExecutor(w)
    for i in range(w.n):
        ex.assignment[i] = None
    cut = int(np.nonzero(w.actuator != LANE_INDEX["antenna"])[0][0])
    w.pos[cut] = base + [90.0, 0.0]
    w.in_comms[cut] = False
    ex.mark_dirty()

    goal_xy, goal_id = ex.goals(w)
    bridging = [int(i) for i in relays
                if goal_id[i] >= 0 and "cut-off" in ex.reason[i]]
    assert len(bridging) == 1, "the cut-off robot must still get exactly one relay"
    assert bridging[0] != int(relays[0]), "and not the one that left to recharge"


def test_a_load_bearing_relay_does_not_wander_off():
    """Rung 0: if a robot's only link is this relay, the relay is that robot's post."""
    w = World(Scenario.load("test"), 42)
    base = np.asarray(w.scn.base, dtype=float)
    br, rr = w.scn.comms.base_radius, w.scn.comms.relay_radius
    w.status[:] = 0
    w.in_comms[:] = True
    relays = np.nonzero(w.actuator == LANE_INDEX["antenna"])[0]
    others = np.nonzero(w.actuator != LANE_INDEX["antenna"])[0]
    w.pos[relays] = base
    w.pos[others] = base

    # One relay out on a limb, with one robot reaching base only through it.
    link, dep, cut = int(relays[0]), int(others[0]), int(others[1])
    w.pos[link] = base + [br + rr * 0.4, 0.0]
    w.pos[dep] = base + [br + rr * 0.8, 0.0]
    # ... and somebody cut off far away, so rung 1 has a competing job on offer.
    w.pos[cut] = base + [0.0, br + rr * 3.0]
    w.in_comms[cut] = False

    orders = skill_executor._relay_orders(w, relays)
    (gx, gy), why = orders[link]
    assert "only link" in why, f"the load-bearing relay was sent away: {why!r}"
    assert (gx, gy) == pytest.approx(tuple(w.pos[link])), "holding means not moving"
    assert any("cut-off" in w for _, w in orders.values()), (
        "some other relay must still take the rung 1 job")
