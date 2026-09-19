"""World invariants: connectivity, placement, physics safety, comms, the rescue chain."""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from swarmmind.control.wander import wander
from swarmmind.sim import grid
from swarmmind.sim.robot import DESTROYED, LANE_INDEX, LANES
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World


@pytest.fixture(scope="module")
def scn():
    return Scenario.load("test")


@pytest.fixture(scope="module")
def world(scn):
    return World(scn, 42)


def test_grid_shape_matches_scenario(world, scn):
    assert world.occ.shape == scn.grid_shape == (128, 192)


def test_every_free_cell_is_reachable_from_base(world):
    """ensure_connected must leave no unreachable pockets -- victims and frontier
    targets are sampled from free cells and an unreachable one would hang a robot."""
    assert np.isfinite(world.dist_from_base[world.passable]).all()


def test_map_is_not_mostly_wall(world):
    frac = world.passable.mean()
    assert 0.55 < frac < 0.98, f"passable fraction {frac:.2f} outside a usable range"


def test_all_four_lanes_present(world, scn):
    counts = world.lane_counts()
    assert set(counts) == set(LANES)
    assert all(c == scn.robots_per_lane for c in counts.values())
    assert world.n == scn.n_robots


def test_victims_placed_and_buried_quota(world, scn):
    assert len(world.victims) == scn.victims.count
    assert sum(v.buried for v in world.victims) == scn.victims.buried_count
    for v in world.victims:
        ix, iy = grid.world_to_cell(v.pos[0], v.pos[1], world.cell, world.shape)
        assert world.passable[int(iy), int(ix)], f"{v.id} placed inside a wall"


def test_every_casualty_is_reachable_and_workable(scn):
    """A casualty nobody can get to is worse than one nobody finds: the swarm sees it,
    dispatches a carrier, and the carrier never arrives -- which looks like the system
    working. `passable & finite dist_from_base` is not sufficient, because robots route
    on the 4x-downsampled nav grid, and because a body in a one-cell gap cannot be
    approached within REACH_GRAB."""
    from swarmmind.control.planner import NavFields

    w = World(scn, 42)
    nav = NavFields(w.passable, w.cell)
    field = nav.field(*scn.base)
    for v in w.victims:
        d = float(nav.distance_at(field, np.array([v.pos[0]]), np.array([v.pos[1]]))[0])
        assert np.isfinite(d) and d < 1e8, f"{v.id} has no route from base on the nav grid"
        ix, iy = grid.world_to_cell(v.pos[0], v.pos[1], w.cell, w.shape)
        assert grid.clearance_mask(w.occ, cells=1)[int(iy), int(ix)], (
            f"{v.id} is wedged against a wall; no carrier can reach REACH_GRAB of it"
        )


def test_victims_respect_min_separation(world, scn):
    pts = np.array([v.pos for v in world.victims])
    d = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
    np.fill_diagonal(d, np.inf)
    assert d.min() >= scn.victims.min_separation_m - 1e-6


def test_robots_never_penetrate_walls(scn):
    """Tier 1 is the safety floor (CLAUDE.md #2), but the collision response in World
    must hold even under the wander controller, which has no avoidance at all."""
    w = World(scn, 7)
    rng = w.rng["noise"]
    for _ in range(int(60 * scn.rates.tick_hz)):
        v, omega = wander(w, rng)
        w.step(v, omega)
        ix, iy = grid.world_to_cell(w.pos[:, 0], w.pos[:, 1], w.cell, w.shape)
        assert not (w.occ[iy, ix] == grid.WALL).any(), f"wall penetration at t={w.t:.2f}"


def test_hazard_ignites_on_schedule(scn):
    w = World(scn, 42)
    rng = w.rng["noise"]
    while w.t < scn.hazard.ignite_t - scn.dt:
        w.step(*wander(w, rng))
    assert not w.hazard.active
    w.step(*wander(w, rng))
    assert w.hazard.active
    assert any(e["kind"] == "hazard_ignited" for e in w.events)


def test_hazard_grows_and_drifts(scn):
    w = World(scn, 42)
    rng = w.rng["noise"]
    while w.t < scn.hazard.ignite_t + 60.0:
        w.step(*wander(w, rng))
    assert w.hazard.radius == pytest.approx(scn.hazard.radius0 + scn.hazard.growth_rate * 60.0, abs=0.1)
    assert np.linalg.norm(w.hazard.centre - w.hazard.origin) > 1.0


def test_robots_far_from_base_lose_comms(scn):
    """The relay lane is load-bearing: teleport a scout past every relay and it must
    drop out of the component rather than keep reporting."""
    w = World(scn, 42)
    scouts = np.nonzero(w.actuator == LANE_INDEX["none"])[0]
    i = int(scouts[0])
    w.pos[i] = np.array([scn.map.width_m - 3.0, scn.map.height_m - 3.0])
    # Park every relay at base so nothing extends the component outward.
    relays = np.nonzero(w.actuator == LANE_INDEX["antenna"])[0]
    w.pos[relays] = np.array(scn.base)
    w._update_comms()
    assert not w.in_comms[i]
    assert w.status[i] == 1  # OUT_OF_COMMS


def test_out_of_comms_observations_buffer_until_reconnect(scn):
    """A disconnected robot cannot tell anyone what it saw.

    Explored space now comes from camera frames, so this guards the same claim the old
    fog test did, against the mechanism that replaced it.
    """
    from swarmmind.mission import Mission

    m = Mission(scn, 42)
    w = m.world
    relays = np.nonzero(w.actuator == LANE_INDEX["antenna"])[0]
    w.pos[relays] = np.array(scn.base)
    i = int(np.nonzero(w.actuator == LANE_INDEX["none"])[0][0])
    w.pos[i] = np.array([scn.map.width_m - 6.0, scn.map.height_m - 6.0])
    w._update_comms()
    assert not w.in_comms[i]

    before = int(w.explored.sum())
    frames, cells = m.rig.capture(w, m.raster.render(w), np.array([i]))
    w.mark_seen(cells, np.array([i]))
    assert int(w.explored.sum()) == before, "an out-of-comms robot must not reveal the map"
    assert i in w._pending_cells and len(w._pending_cells[i]) > 0

    w.pos[i] = np.array(scn.base)
    w._update_comms()
    assert int(w.explored.sum()) > before, "reconnecting must flush buffered observations"
    assert i not in w._pending_cells
    assert i in w.reconnected, "perception must be told to flush this robot's sightings"
    assert any(e["kind"] == "robot_reconnected" for e in w.events)


def test_victim_discovery_is_not_an_oracle(scn):
    """The world must never mark a casualty found on its own.

    This used to be `dist <= sensor_radius` against ground-truth coordinates -- the
    robot was simply told. Discovery now belongs to perception, and the only way in is
    mark_found(), called when a detector report resolves onto a real person.
    """
    w = World(scn, 42)
    v = w.victims[0]
    w.pos[:] = v.pos
    for _ in range(40):
        w.step(np.zeros(w.n), np.zeros(w.n))
    assert v.state == 0, "proximity alone must not discover a casualty"
    assert w.victims_found == 0

    w.mark_found(0, 0)
    assert v.state == 1 and w.victims_found == 1


def test_carrier_death_drops_the_victim(scn):
    """Otherwise a victim vanishes with its carrier and the mission is unwinnable."""
    w = World(scn, 42)
    v = w.victims[0]
    v.state, v.buried, v.debris_remaining = 2, False, 0.0   # CLEARED
    i = int(np.nonzero(w.actuator == LANE_INDEX["gripper"])[0][0])
    w.pos[i] = v.pos.copy()
    w._update_victims()
    assert v.state == 3 and v.carrier == i                  # CARRIED

    w._kill(i, "test")
    assert w.status[i] == DESTROYED
    assert v.state == 2 and v.carrier == -1                 # back to CLEARED
    assert w.carrying[i] == -1


def test_demo_scenario_is_constructible_and_sane():
    """The demo scenario is too slow to run in the suite, but a config regression in it
    must not survive to demo day. Constructing it is cheap and catches every one that
    matters: unreachable regions, an over-dense map, victims in walls, bad lane counts."""
    scn = Scenario.load("demo")
    w = World(scn, 42)

    # Lane sizes are deliberately uneven (M-33): the relay network saturates at ~44
    # posts however many antennas exist, so an even split parked a quarter of the swarm
    # at spawn. What must hold is that the roster matches the declared counts and that
    # no lane is empty -- a swarm with no carriers cannot rescue anyone.
    assert w.n == scn.n_robots
    built = w.lane_counts()
    assert built == scn.lane_counts, f"roster {built} != declared {scn.lane_counts}"
    assert all(v > 0 for v in built.values()), f"an empty lane: {built}"
    assert built["none"] > built["antenna"], "more relays than scouts is upside down"
    assert len(scn.sector_ids()) == 48
    assert len(w.victims) == scn.victims.count
    assert sum(v.buried for v in w.victims) == scn.victims.buried_count

    frac = w.passable.mean()
    assert 0.55 < frac < 0.98, f"passable fraction {frac:.3f} -- retune map.n_clusters"
    assert np.isfinite(w.dist_from_base[w.passable]).all(), "unreachable region on the demo map"

    # No victim should be unreachable or buried inside a wall.
    for v in w.victims:
        ix, iy = grid.world_to_cell(v.pos[0], v.pos[1], w.cell, w.shape)
        assert np.isfinite(w.dist_from_base[int(iy), int(ix)]), f"{v.id} unreachable"

    # Every casualty must be recoverable inside the mission clock. Derive the bound from
    # the scenario rather than hardcoding metres: a magic number silently goes wrong the
    # moment the map is resized, which is exactly how a 161 m worst-case carry -- a 448 s
    # round trip on a 420 s run -- got into the 480 x 320 m version of this map.
    zones = [scn.base, *scn.extraction_zones]
    worst = max(
        min(float(np.hypot(v.pos[0] - zx, v.pos[1] - zy)) for zx, zy in zones)
        for v in w.victims
    )
    # Median carrier, not the slowest. Carriers are mixed across chassis and a legged
    # one is 20% slower by design; sizing the whole map on the slowest unit assumes the
    # furthest casualty always draws the worst-suited robot, which the auction exists to
    # prevent.
    loaded = float(np.median(w.v_max[w.actuator == LANE_INDEX["gripper"]])) * \
        scn.victims.carry_speed_factor
    round_trip = 2.0 * worst / loaded
    budget = 0.75 * scn.mission_duration_s
    assert round_trip < budget, (
        f"worst carry {worst:.0f} m = {round_trip:.0f} s round trip, over the "
        f"{budget:.0f} s budget -- add collection points or speed carriers up"
    )


def test_scorecard_fields_are_sane(world):
    sc = world.scorecard()
    assert sc.victims_total == len(world.victims)
    assert sc.robots_total == world.n
    assert 0.0 <= sc.ground_explored_frac <= 1.0
    assert len(sc.hash()) == 16


def test_the_hazard_grows_then_burns_out():
    """A fire that only ever grows is a countdown, not a hazard.

    Ground it reaches is lost for the rest of the mission, because `tasks.py` generates
    no work whose target is in known hazard -- so a casualty the fire caught is written
    off in silence. Burn-out is what gives that ground, and those casualties, back.
    """
    scn = Scenario.load("test")
    burning = dataclasses.replace(
        scn, hazard=dataclasses.replace(
            scn.hazard, ignite_t=10.0, radius0=5.0, growth_rate=1.0,
            peak_t=30.0, decay_rate=0.5),
    )
    w = World(burning, seed=42)

    def radius_at(t: float) -> float:
        w.t = t
        w._update_hazard()
        return w.hazard.radius

    assert radius_at(20.0) == pytest.approx(15.0)      # growing:  5 + 1.0 * 10
    assert radius_at(30.0) == pytest.approx(25.0)      # peak:     5 + 1.0 * 20
    assert radius_at(50.0) == pytest.approx(15.0)      # receding: 25 - 0.5 * 20
    assert radius_at(200.0) == 0.0, "a burnt-out fire must reach zero, not go negative"

    # The old behaviour has to remain available, and is what `test.yaml` still uses.
    forever = World(scn, seed=42)
    forever.t = 400.0
    forever._update_hazard()
    grew = forever.hazard.radius
    forever.t = 401.0
    forever._update_hazard()
    assert forever.hazard.radius > grew, "peak_t=None must keep growing indefinitely"


def test_a_robot_knows_the_ground_under_its_own_feet():
    """The camera near clip (1.03 m) exceeds the cell (1.0 m), so it never sees its own."""
    w = World(Scenario.load("test"), 42)
    w.status[:] = 0
    w.in_comms[:] = True
    w.explored[:] = False
    ix, iy = grid.world_to_cell(w.pos[:, 0], w.pos[:, 1], w.cell, w.shape)
    assert not w.explored[iy[0], ix[0]]
    w.mark_underfoot(np.arange(w.n))
    assert w.explored[iy[0], ix[0]], "standing on a cell did not reveal it"
    assert w.explored.sum() >= 1


def test_underfoot_respects_comms_like_the_camera_does():
    """An unreachable robot cannot report where it has been, either."""
    w = World(Scenario.load("test"), 42)
    w.status[:] = 0
    w.in_comms[:] = True
    w.in_comms[0] = False
    w.explored[:] = False
    ix, iy = grid.world_to_cell(w.pos[:, 0], w.pos[:, 1], w.cell, w.shape)
    w.mark_underfoot(np.array([0]))
    assert not w.explored[iy[0], ix[0]], "an out-of-comms robot wrote to the shared map"
    assert 0 in w._pending_cells, "and did not buffer it either"
    w._flush_buffers(0)
    assert w.explored[iy[0], ix[0]], "reconnecting did not deliver the buffered cell"
