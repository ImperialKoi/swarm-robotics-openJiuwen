"""Rescue failures reproduced during the control-path audit."""

from __future__ import annotations

import numpy as np
import pytest

from swarmmind.control.planner import NavFields, NavSet
from swarmmind.mission import Mission
from swarmmind.nodes.skill_executor import NO_PROGRESS_AFTER, Assignment, SkillExecutor
from swarmmind.perception.tracker import CONFIRMED, RESOLVED, Report
from swarmmind.sim import grid
from swarmmind.sim.robot import ACTIVE, CHASSIS_INDEX, FAILED, LANE_INDEX
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import CARRIED, CLEARED, FOUND, REACH_DIG, World


@pytest.mark.parametrize("dx,dy", grid.COARSE_DIRS)
@pytest.mark.parametrize("cached", [False, True])
def test_steering_uses_only_edges_the_distance_field_can_traverse(dx, dy, cached):
    # Adjacent majority-passable blocks, with their shared face/corner blocked.
    # There is an open detour, so the field is finite on both sides of the wall.
    factor = 4
    fine = np.ones((5 * factor, 5 * factor), dtype=bool)
    source = (2, 2)
    x0, y0 = np.array(source) * factor
    xs = x0 + (factor - 1 if dx > 0 else 0) if dx else slice(x0, x0 + factor)
    ys = y0 + (factor - 1 if dy > 0 else 0) if dy else slice(y0, y0 + factor)
    fine[ys, xs] = False
    nav = NavFields(fine, cell=1.0, factor=factor, edge_steering=True)
    goal = ((source[0] + dx + 0.5) * factor, (source[1] + dy + 0.5) * factor)
    field = nav.field(*goal)
    assert field[source[1], source[0]] == 2, "the route must take the detour"
    x, y = (np.array([(v + 0.5) * factor]) for v in source)
    direction = nav.descend_to(*goal, x, y) if cached else nav.descend(field, x, y)
    step = tuple(np.sign(direction[0]).astype(int))
    assert step in grid.COARSE_DIRS, "a reachable cell must have a next step"
    assert nav.edges[grid.COARSE_DIRS.index(step), source[1], source[0]], (
        f"steering chose the blocked {step} edge instead of the open detour"
    )


@pytest.fixture
def rescue_world():
    w = World(Scenario.load("test"), 42)
    ex = SkillExecutor(w)
    ex.idle_explore = False
    ex.roaming_relays = False
    w.in_comms[:] = True
    i = int(np.flatnonzero(w.actuator == LANE_INDEX["gripper"])[0])
    return w, ex, i


def test_pickup_between_auctions_caches_a_reachable_delivery_goal(rescue_world):
    w, ex, i = rescue_world
    # Forward collection point is nearer, but cut off from the carrier by a river.
    # Base is farther away and reachable. Use real fields to exercise zone selection.
    w.chassis_passable[:] = True
    w.chassis_passable[:, :, 160:164] = False
    w.pos[i] = (76.0, 54.0)
    v = w.victims[0]
    v.pos = w.pos[i].copy()
    v.state = CLEARED
    nav = NavSet(w)
    ex.assign(w, i, Assignment("pickup", "extract", tuple(v.pos), victim=0))
    ex.goals(w, nav)
    w.carrying[i] = 0
    v.state, v.carrier = CARRIED, i
    ex.update(w, nav, full=False)
    xy, ids = ex.goals(w, nav)
    assert xy[ids[i]] == tuple(w.scn.base), "the fast update cached an unreachable zone"


@pytest.mark.parametrize("prior_kind", [None, "explore", "extract"])
@pytest.mark.parametrize("full", [False, True])
def test_opportunistic_pickup_delivers_the_casualty_actually_held(rescue_world, prior_kind, full):
    w, ex, i = rescue_world
    if prior_kind is not None:
        w.victims[1].state = CLEARED
        ex.assign(w, i, Assignment("previous", prior_kind, tuple(w.victims[1].pos),
                                  victim=1 if prior_kind == "extract" else None))
    ex.goals(w)
    w.carrying[i] = 0
    w.victims[0].state, w.victims[0].carrier = CARRIED, i
    ex.update(w, full=full)
    a = ex.assignment[i]
    assert a is not None and a.kind == "extract" and a.victim == 0
    xy, ids = ex.goals(w)
    assert xy[ids[i]] in [tuple(w.scn.base), *map(tuple, w.scn.extraction_zones)]
    assert not ex.free_mask(w)[i], "a loaded carrier must not bid on unrelated work"


def test_loaded_carrier_finishes_hazard_retreat_before_resuming_delivery(rescue_world):
    w, ex, i = rescue_world
    w.carrying[i] = 0
    w.victims[0].state, w.victims[0].carrier = CARRIED, i
    haven = tuple(w.pos[i] + [8.0, 0.0])
    ex.assign(w, i, Assignment("retreat", "retreat", haven))
    ex.update(w, full=False)
    assert ex.assignment[i].kind == "retreat"
    w.pos[i] = haven
    ex.update(w, full=False)
    assert ex.assignment[i] is None
    ex.update(w, full=False)
    assert ex.assignment[i].kind == "extract" and ex.assignment[i].victim == 0


@pytest.mark.parametrize("in_reach,airborne", [(False, False), (True, False), (True, True)])
def test_partial_debris_only_exempts_a_digger_that_is_actually_at_work(
        rescue_world, in_reach, airborne):
    w, ex, _ = rescue_world
    i = int(np.flatnonzero(w.actuator == LANE_INDEX["scoop"])[0])
    v = w.victims[0]
    v.state, v.buried, v.debris_remaining = FOUND, True, 0.5
    w.pos[i] = v.pos + [REACH_DIG * (0.9 if in_reach else 3.0), 0.0]
    if airborne:
        w.chassis[i] = CHASSIS_INDEX["rotor"]
        w.airborne[i] = True
    ex.assign(w, i, Assignment("dig", "clear_debris", tuple(v.pos), victim=0))
    ex.update(w)
    w.t += NO_PROGRESS_AFTER["clear_debris"] + 1.0
    ex.update(w)
    assert (ex.assignment[i] is not None) == (in_reach and not airborne)


def test_fault_publishes_one_destruction_event():
    m = Mission(Scenario.load("test"), 42, hivemind=False)
    w = m.world
    w.t = m.fault.cfg.fallback_t + 1.0
    m.fault.step(w, m.executor, m._emit)
    m.events.drain_world(w)
    events = [ev for ev in m.events.of_kind("robot_destroyed")
              if ev["robot"] == m.fault.victim_robot]
    assert len(events) == 1, "one failed robot must produce one death notification"
    assert "structural collapse" in events[0]["text"]


@pytest.mark.parametrize("airborne,in_comms", [(True, True), (False, False), (False, True)])
def test_contact_resolution_requires_a_ground_inspection_and_live_link(airborne, in_comms):
    m = Mission(Scenario.load("test"), 42, hivemind=False)
    w = m.world
    i = int(np.flatnonzero(w.chassis == CHASSIS_INDEX["rotor"])[0])
    w.status[:] = FAILED
    w.status[i] = ACTIVE
    w.pos[i] = w.victims[0].pos.copy()
    w.airborne[i], w.in_comms[i] = airborne, in_comms
    report = Report("inspected", w.victims[0].pos.copy(), state=CONFIRMED)
    m.tracker.reports.append(report)
    resolved = m.tracker.resolve(w)
    can_report = not airborne and in_comms
    assert bool(resolved) == can_report
    assert report.state == (RESOLVED if can_report else CONFIRMED)


def test_stationary_reconnected_robot_resolves_without_another_camera_frame(monkeypatch):
    m = Mission(Scenario.load("test"), 42, hivemind=False)
    w = m.world
    w.status[:] = FAILED
    w.status[0] = ACTIVE
    w.in_comms[0] = True
    w.airborne[:] = False
    w.pos[0] = w.victims[0].pos.copy()
    m._cam_never[:] = False
    m._last_cam_pos[:] = w.pos
    report = Report("reconnected", w.victims[0].pos.copy(), state=CONFIRMED)
    m.tracker.reports.append(report)

    def unexpected_frame(*args):
        pytest.fail("stationary bookkeeping must not require a new camera frame")

    monkeypatch.setattr(m.rig, "capture", unexpected_frame)
    m._perceive()
    assert report.state == RESOLVED
    assert w.victims[0].state == FOUND
    ix, iy = grid.world_to_cell(w.pos[0, 0], w.pos[0, 1], w.cell, w.shape)
    assert w.explored[iy, ix]
