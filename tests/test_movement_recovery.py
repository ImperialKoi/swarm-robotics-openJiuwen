"""Orders, river parking and delivery stalls reproduced on the small fixture."""

from dataclasses import replace

import numpy as np
import pytest

from swarmmind.control.planner import NavSet, frontier_targets
from swarmmind.control.recovery import STALL_SECONDS, RecoveryFields
from swarmmind.control.tier1_reflex import ReflexController
from swarmmind.nodes.auction import AuctionNode
from swarmmind.nodes.skill_executor import (
    DARK_REFRESH_S,
    Assignment,
    SkillExecutor,
    _drift_targets,
    _dry_standing_goal,
)
from swarmmind.nodes.tasks import OpenTask, snap_passable
from swarmmind.sim import grid
from swarmmind.sim.robot import CHASSIS_INDEX, FAILED, LANE_INDEX
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import CARRIED, RESCUED, World


@pytest.fixture
def world():
    w = World(Scenario.load("test"), 42)
    w.occ[:] = 0
    w.passable[:] = True
    w.chassis_passable[:] = True
    w.water[:] = 0
    w.hazard_known[:] = False
    w.in_comms[:] = True
    w.explored[:] = False
    w.sector_explored_pct[:] = 0
    return w


def test_idle_goal_refreshes_without_an_auction_award(world):
    ex = SkillExecutor(world)
    ex.roaming_relays = False
    xy, ids = ex.goals(world)
    assert ids[0] >= 0
    world.explored[:] = True
    world.t += DARK_REFRESH_S
    _, ids = ex.goals(world)
    assert ids[0] == -1, "idle robot kept walking to a stale, surveyed target"
    assert "awaiting assignment" in ex.reason[0]


def test_idle_orders_precede_distance_and_exclude_abandoned_sectors(world):
    pts = np.array([[12.5, 12.5], [72.5, 44.5]])
    world.pos[0] = pts[0]
    x, y = grid.world_to_cell(*pts.T, world.cell, world.shape)
    sectors = world.sector_of_cell[y, x]
    assert sectors[0] != sectors[1]
    world.sector_priority[sectors[1]] = 0
    dark = {int(world.chassis[0]): pts}
    assert _drift_targets(world, np.array([0]), dark)[0] == tuple(pts[1])
    world.sector_abandoned[sectors[1]] = True
    assert _drift_targets(world, np.array([0]), dark)[0] == tuple(pts[0])


def test_order_change_invalidates_idle_cache_and_preserves_assignment(world):
    ex = SkillExecutor(world)
    ex.roaming_relays = False
    assigned = (12.25, 12.25)
    ex.assign(world, 0, Assignment("existing", "explore", assigned))
    xy, ids = ex.goals(world)
    assert xy[ids[0]] == assigned
    world.sector_abandoned[:] = True
    xy, ids = ex.goals(world)
    assert xy[ids[0]] == assigned  # the auction handles explicit retreat/preemption
    assert ids[1] == -1


def test_negative_component_labels_do_not_make_unreachable_targets_reachable(world):
    pts = np.array([[72.5, 44.5]])
    c = int(world.chassis[0])
    labels = np.full(world.shape, -1)
    assert not _drift_targets(world, np.array([0]), {c: pts}, {c: labels})


def test_search_targets_and_relay_posts_are_dry_members_of_the_frontier(world):
    world.water[:, 20:26] = 1.0
    world.explored[10:30, 10:40] = True
    targets = frontier_targets(world)
    assert targets
    for t in targets:
        x, y = grid.world_to_cell(*t.pos, world.cell, world.shape)
        assert world.water[y, x] == 0
        assert not world.explored[y, x]
    target = snap_passable(world, (23 * world.cell, 20 * world.cell))
    assert target is not None
    x, y = grid.world_to_cell(*target, world.cell, world.shape)
    assert world.water[y, x] == 0
    for pts in SkillExecutor(world)._dark_points(world).values():
        x, y = grid.world_to_cell(*pts.T, world.cell, world.shape)
        assert (world.water[y, x] == 0).all()


def test_computed_relay_midpoint_moves_to_a_reachable_dry_bank(world):
    world.water[:, 30:40] = 1.0
    labels = {c: np.zeros(world.shape, dtype=int) for c in range(4)}
    goal = _dry_standing_goal(world, 0, (17.5, 14.0), labels)
    assert goal is not None
    x, y = grid.world_to_cell(*goal, world.cell, world.shape)
    assert world.water[y, x] == 0
    assert np.linalg.norm(np.array(goal) - (17.5, 14.0)) < 3


def test_idle_aircraft_over_water_can_choose_dry_ground_across_the_channel(world):
    c = CHASSIS_INDEX["rotor"]
    world.chassis[0] = c
    world.pos[0] = (17.5, 14.0)
    world.water[:, 30:40] = 1.0
    world.chassis_passable[c, :, 30:40] = False
    ex = SkillExecutor(world)
    target = _drift_targets(world, np.array([0]), {c: np.array([[22.25, 14.25]])},
                            ex._reach_labels(world))
    assert target[0] == (22.25, 14.25)


def test_high_priority_search_can_redirect_search_but_never_a_loaded_carrier(world):
    ex, nav = SkillExecutor(world), NavSet(world)
    auction = AuctionNode(world, nav)
    old, goal = (12.25, 12.25), (72.25, 44.25)
    x, y = grid.world_to_cell(*goal, world.cell, world.shape)
    world.sector_priority[world.sector_of_cell[y, x]] = 0
    world.status[1:] = FAILED
    ex.assign(world, 0, Assignment("old", "explore", old))
    task = OpenTask("explore", goal, 4.5)
    assert auction._best_preemptable(world, ex, task)[0] == 0
    world.carrying[0] = 0
    assert auction._best_preemptable(world, ex, task)[0] == -1
    world.carrying[0] = -1
    ex.assign(world, 0, Assignment("dig", "clear_debris", old, victim=0))
    assert auction._best_preemptable(world, ex, task)[0] == -1


def _pocket(world):
    # The coarse cell's eastward descent points into a U-shaped pocket. A real
    # route first goes west, then north round the walls, to a collection point.
    world.chassis_passable[:, 24:49, 36:38] = False
    world.chassis_passable[:, 24:26, 20:38] = False
    world.chassis_passable[:, 47:49, 20:38] = False
    world.occ[~world.chassis_passable[0]] = grid.WALL
    world.passable[:] = world.chassis_passable[0]
    return (17.0, 18.0), (24.0, 18.0)


def test_recovery_takes_the_physical_detour_and_limits_work_per_tick(world):
    start, goal = _pocket(world)
    nav = NavSet(world)
    recovery = RecoveryFields(world, nav)
    positions = np.array([start])
    direction, good = recovery.directions(world, 0, goal, positions)
    assert good[0]
    assert direction[0, 0] < 0, "must leave the pocket before heading toward the goal"
    recovery.directions(world, 1, goal, positions)
    assert recovery.builds == 1
    world.tick += 1
    recovery.directions(world, 1, goal, positions)
    assert recovery.builds == 2


def test_loaded_carrier_escapes_pocket_and_delivers_with_safety_enabled(world):
    start, goal = _pocket(world)
    world.scn = replace(world.scn, base=goal, extraction_zones=[], extraction_radius=2.0)
    world.status[:] = FAILED
    i = int(np.flatnonzero(world.actuator == LANE_INDEX["gripper"])[0])
    world.status[i] = 0
    world.chassis[i] = 0
    world.pos[i], world.theta[i] = start, 0
    world.carrying[i] = 0
    world.victims[0].state, world.victims[0].carrier = CARRIED, i
    ex, nav, reflex = SkillExecutor(world), NavSet(world), ReflexController(world)
    ex.assign(world, i, Assignment("deliver", "extract", start, victim=0))
    xy, ids = ex.goals(world, nav)
    reflex.commands(world, nav, xy, ids)
    # Reproduce the six seconds with no displacement that activate recovery.
    world.t = STALL_SECONDS + 1
    world.tick = round(world.t / world.dt)
    saw_recovery = False
    for _ in range(2400):
        xy, ids = ex.goals(world, nav)
        v, omega = reflex.commands(world, nav, xy, ids, ex.stop_radii(world, 1.5))
        saw_recovery |= bool(reflex.recovering[i])
        world.step(v, omega)
        x, y = grid.world_to_cell(*world.pos[i], world.cell, world.shape)
        assert world.chassis_passable[0, y, x], "recovery crossed a blocked wall"
        if world.victims[0].state == RESCUED:
            break
    assert saw_recovery
    assert world.victims[0].state == RESCUED, (world.pos[i], reflex._dir[i])


def test_holding_a_relay_post_is_not_a_navigation_stall(world):
    reflex, nav = ReflexController(world), NavSet(world)
    xy, ids = [tuple(world.pos[0])], np.full(world.n, -1)
    ids[0] = 0
    reflex.commands(world, nav, xy, ids)
    world.t += STALL_SECONDS * 2
    reflex.commands(world, nav, xy, ids)
    assert not reflex.recovering[0]


def test_demo_spawns_have_body_clearance_dry_ground_and_a_route_out():
    # Construction only: no demo mission ticks or training on this machine.
    w = World(Scenario.load("demo"), 42)
    nav = NavSet(w)
    x, y = grid.world_to_cell(*w.pos.T, w.cell, w.shape)
    assert (w.water[y, x] == 0).all()
    assert len(np.unique(np.column_stack([x, y]), axis=0)) == w.n
    assert w.can_land().all()
    for c in range(3):
        idx = np.flatnonzero(w.chassis == c)
        d = nav.distance_at(c, nav.field(c, *w.scn.base), *w.pos[idx].T)
        assert (d < 1e9).all()
    ex = SkillExecutor(w)
    _, ids = ex.goals(w, nav)
    assert (ids >= 0).all(), "a fresh deployment must give every robot somewhere to go"
