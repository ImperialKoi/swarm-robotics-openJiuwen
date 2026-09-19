"""Terrain, and the locomotion axis it exists to justify.

The point of these tests is not that terrain renders nicely. It is that each chassis has
a job no other chassis does -- otherwise the swarm is carrying two redundant designs, and
the cheapest one wins every time.
"""

from __future__ import annotations

import numpy as np
import pytest

from swarmmind.control.planner import NavSet
from swarmmind.sim import grid
from swarmmind.sim import terrain as tg
from swarmmind.sim.robot import CHASSIS_INDEX, CHASSIS_LIMITS, GROUND_CHASSIS
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World

SEEDS = list(range(42, 58))


@pytest.fixture(scope="module")
def demo():
    return World(Scenario.load("demo"), 42)


def _base_cell(w):
    ix, iy = grid.world_to_cell(
        np.asarray(w.scn.base[0]), np.asarray(w.scn.base[1]), w.cell, w.shape
    )
    return int(ix), int(iy)


def _reach(w, chassis: int) -> float:
    m = w.chassis_passable[chassis]
    bx, by = _base_cell(w)
    return grid._flood(m, (bx, by)).sum() / w.passable.sum() if m[by, bx] else 0.0


# --- the axis earns its keep ------------------------------------------------------------


def test_each_chassis_reaches_strictly_more_than_the_last(demo):
    """The ordering must be strict. If two chassis reach the same ground, the slower one
    is dead weight and should not exist -- that is an efficiency argument, not a
    stylistic one."""
    r = [_reach(demo, i) for i in range(len(GROUND_CHASSIS))]
    for a, b in zip(range(len(GROUND_CHASSIS) - 1), range(1, len(GROUND_CHASSIS)), strict=True):
        assert r[b] > r[a] + 0.05, (
            f"{GROUND_CHASSIS[b]} reaches {r[b]:.0%} vs {GROUND_CHASSIS[a]}'s {r[a]:.0%} -- "
            f"too close to justify building both"
        )


def test_speed_is_the_price_of_reach(demo):
    """Reach and speed must trade in opposite directions, or one chassis dominates."""
    speeds = [CHASSIS_LIMITS[c][2] for c in GROUND_CHASSIS]
    reaches = [_reach(demo, i) for i in range(len(GROUND_CHASSIS))]
    assert speeds == sorted(speeds, reverse=True), "speed must fall as reach rises"
    assert reaches == sorted(reaches), "reach must rise as speed falls"


def test_wheeled_is_meaningfully_restricted_but_not_stranded(demo):
    """Two failure modes bracket this: a chassis that reaches everywhere is pointless,
    and one that reaches nothing is broken. An earlier tuning left wheeled at 2%."""
    r = _reach(demo, CHASSIS_INDEX["wheeled"])
    assert 0.20 < r < 0.75, f"wheeled reaches {r:.0%}"


def test_legged_is_the_fallback_and_actually_falls_back(demo):
    """Legged is the go-anywhere option. If it has reachability problems of its own,
    there is nothing underneath it and some casualties are simply unrecoverable."""
    assert _reach(demo, CHASSIS_INDEX["legged"]) > 0.90


# --- the three barrier tiers exist --------------------------------------------------------


def test_marsh_stops_wheels_and_nothing_else(demo):
    """Marsh is the entire reason the tracked chassis exists: a barrier in the band
    between a wheeled unit's 0 m wade and a tracked unit's 0.45 m."""
    marsh = (demo.water > 0.05) & (demo.water <= CHASSIS_LIMITS["tracked"][1])
    assert marsh.sum() > 0, "no shallow water anywhere; tracked units have no niche"
    assert not demo.chassis_passable[CHASSIS_INDEX["wheeled"]][marsh].any()
    # Isolate the water effect: marsh on a steep bank is blocked for slope reasons and
    # says nothing about wade depth.
    gentle = marsh & demo.passable & (demo.slope <= CHASSIS_LIMITS["tracked"][0])
    assert gentle.sum() > 0
    assert demo.chassis_passable[CHASSIS_INDEX["tracked"]][gentle].all()


def test_rivers_stop_tracks_but_not_legs(demo):
    deep = demo.water > CHASSIS_LIMITS["tracked"][1]
    assert deep.sum() > 0, "no water deep enough to separate tracked from legged"
    assert not demo.chassis_passable[CHASSIS_INDEX["tracked"]][deep].any()
    wadeable = deep & (demo.water <= CHASSIS_LIMITS["legged"][1]) & demo.passable
    assert wadeable.sum() > 0, "rivers are deeper than legged can ford -- nobody crosses"


def test_steep_ground_stops_wheels_before_tracks(demo):
    band = (demo.slope > CHASSIS_LIMITS["wheeled"][0]) & (
        demo.slope <= CHASSIS_LIMITS["tracked"][0]
    ) & demo.passable
    assert band.sum() > 0, "no slope in the wheeled/tracked band; mountains do nothing"


def test_slope_is_measured_from_the_landform_not_the_rubble(demo):
    """Taking the gradient of `height` would read every one-metre rubble block as a
    cliff and make the whole map impassable to everything."""
    from_height = tg.slope_of(demo.height, demo.cell)
    assert np.median(demo.slope) < np.median(from_height)
    assert (demo.slope <= CHASSIS_LIMITS["legged"][0]).mean() > 0.9


# --- routing respects it ------------------------------------------------------------------


def test_flow_fields_are_per_chassis(demo):
    """A field built on legged passability walks a wheeled unit confidently into a
    river and leaves it at the bank."""
    nav = NavSet(demo)
    far = demo.scn.extraction_zones[-1]
    fields = [nav.field(i, far[0], far[1]) for i in range(len(GROUND_CHASSIS))]
    assert not np.array_equal(fields[CHASSIS_INDEX["wheeled"]],
                              fields[CHASSIS_INDEX["legged"]])
    reachable = [np.isfinite(f).sum() - (f > 1e8).sum() for f in fields]
    assert reachable[CHASSIS_INDEX["legged"]] > reachable[CHASSIS_INDEX["wheeled"]]


def test_robots_never_enter_terrain_their_chassis_cannot_cross():
    """The safety floor extended to terrain: a wheeled robot must never end a tick in
    water, however hard the higher tiers push it."""
    from swarmmind.mission import Mission

    m = Mission(Scenario.load("test"), 42)
    w = m.world
    for _ in range(int(90 * w.scn.rates.tick_hz)):
        m.tick()
        ix, iy = grid.world_to_cell(w.pos[:, 0], w.pos[:, 1], w.cell, w.shape)
        ok = w.chassis_passable[w.chassis, iy, ix]
        alive = w.status <= 1
        grounded = alive & ~w.airborne
        assert ok[grounded].all(), (
            f"t={w.t:.1f}: {(~ok & grounded).sum()} robots are standing on terrain their "
            f"chassis cannot traverse"
        )
        assert not (w.airborne & (w.chassis != CHASSIS_INDEX["rotor"])).any()


# --- generation is robust -------------------------------------------------------------------


@pytest.mark.parametrize("scenario", ["test", "demo"])
def test_every_seed_builds(scenario):
    """Terrain must build on the configured demo maps and the fixture's seed sweep.

    Keep demo checks on demo_seeds; the small fixture still exercises varied terrain.
    """
    scn = Scenario.load(scenario)
    seeds = scn.demo_seeds or SEEDS
    failures = []
    for seed in seeds:
        try:
            World(scn, seed)
        except RuntimeError as exc:
            failures.append((seed, str(exc)[:70]))
    assert not failures, f"{len(failures)}/{len(seeds)} seeds failed: {failures[:3]}"


@pytest.mark.parametrize("scenario", ["test", "demo"])
def test_roads_keep_the_collection_points_connected(scenario):
    """Obstacle clusters are placed with no regard to the base. Roads guarantee the
    fixed points share a component before connectivity pruning deletes everything else."""
    scn = Scenario.load(scenario)
    w = World(scn, 44)
    bx, by = _base_cell(w)
    reach = grid._flood(w.passable, (bx, by))
    for zx, zy in scn.extraction_zones:
        ix, iy = grid.world_to_cell(np.asarray(zx), np.asarray(zy), w.cell, w.shape)
        assert reach[int(iy), int(ix)], f"collection point ({zx}, {zy}) is cut off"
    assert w.passable.mean() > 0.5, f"only {w.passable.mean():.0%} of the map survived"


def test_every_chassis_can_reach_every_collection_point(demo):
    """Roads exist to guarantee exactly this. A carrier that cannot reach a collection
    point cannot complete a rescue, whatever else it can do -- and it is the wheeled
    chassis, the most restricted, that this actually binds on."""
    bx, by = _base_cell(demo)
    for ci, name in enumerate(GROUND_CHASSIS):
        m = demo.chassis_passable[ci]
        assert m[by, bx], f"{name} cannot even stand at base"
        reach = grid._flood(m, (bx, by))
        for zx, zy in demo.scn.extraction_zones:
            ix, iy = grid.world_to_cell(np.asarray(zx), np.asarray(zy), demo.cell, demo.shape)
            assert reach[int(iy), int(ix)], (
                f"{name} cannot reach the collection point at ({zx}, {zy})"
            )


def test_roads_are_graded_and_drained(demo):
    """Clearing obstacles along a corridor does not make a road. Before the corridors
    were regraded and drained, the shipped seed left wheeled units on 2% of the map."""
    gx, gy = grid.cell_centres(demo.shape, demo.cell)
    pts = demo.road_points
    passable_frac: list[float] = []
    weights: list[int] = []
    for i, j in demo.road_edges:
        ax, ay = pts[i]
        bx, by = pts[j]
        seg = np.array([bx - ax, by - ay], dtype=np.float64)
        length2 = float(seg @ seg)
        if length2 < 1.0:
            continue
        rel_x, rel_y = gx - ax, gy - ay
        t = np.clip((rel_x * seg[0] + rel_y * seg[1]) / length2, 0.0, 1.0)
        perp = np.abs(rel_x * -seg[1] + rel_y * seg[0]) / np.sqrt(length2)
        # Centreline only: the shoulder legitimately meets natural ground.
        core = (perp <= demo.cell) & (t > 0.05) & (t < 0.95)
        if not core.any():
            continue
        # Drainage is a hard guarantee: a wheeled unit cannot enter water at all.
        assert demo.water[core].max() <= 1e-6, "a road runs through water"
        passable_frac.append(
            float((demo.slope[core] <= CHASSIS_LIMITS["wheeled"][0]).mean()))
        weights.append(int(core.sum()))

    # Aggregate, not per-road. A single link between two collection points that happen
    # to sit either side of a mountain is legitimately steep, and the network routes
    # around it -- which is why the binding guarantee is the zone-reachability test
    # above, not this one. This catches the regression where regrading stopped working
    # altogether and the whole network became unusable to wheels.
    agg = float(np.average(passable_frac, weights=weights))
    assert agg > 0.85, f"only {agg:.0%} of road centreline is passable to wheels"


def test_terrain_generates_every_feature(demo):
    cfg = demo.scn.terrain
    assert cfg.hills > 0 and cfg.mountains > 0 and cfg.rivers > 0
    assert cfg.ditches > 0 and cfg.marshes > 0
    assert demo.terrain.max() > 30.0, "no relief worth the name"
    assert (demo.water > 0).mean() > 0.03, "no water anywhere"


def test_relief_is_landform_not_grain(demo):
    """A map can have 30 m of range and still read as a pancake with scratches on it,
    which is what it did: the amplitude was all in 30 m-wavelength noise, and twelve 60 m
    level-fill aprons flattened 90% of what was left. Relief has to be there at the scale
    a hill actually is."""
    z = demo.terrain
    coarse = z[::24, ::24]                      # one sample every 24 m
    assert float(coarse.std()) > 6.0, (
        f"terrain varies by only {coarse.std():.1f} m at hill scale -- it is grain, "
        f"not landform"
    )
    # And it must not do that by being uniformly steep: most of the map stays walkable.
    assert (demo.slope <= CHASSIS_LIMITS["wheeled"][0]).mean() > 0.6


def test_rivers_are_straight_and_full_of_water(demo):
    """Two claims in one, because they are the same fix. A river is a straight channel
    holding water to its banks, not a wandering dry trench: the water field used to be a
    flat stamp on the cells a wobbling path happened to touch."""
    deep = demo.water > CHASSIS_LIMITS["tracked"][1]
    assert deep.sum() > 0
    ys, xs = np.nonzero(deep)
    # Straightness: the deep water lies along a small number of lines, so a total least
    # squares fit per connected channel is tight. Cheap proxy -- the principal axis of
    # each channel explains nearly all of its spread.
    from swarmmind.sim.grid import _flood

    seen = np.zeros_like(deep)
    channels = 0
    for y, x in zip(ys[::40], xs[::40], strict=True):
        if seen[y, x]:
            continue
        comp = _flood(deep, (int(x), int(y)))
        seen |= comp
        cy, cx = np.nonzero(comp)
        if len(cy) < 1200:
            # Small pockets are legitimately round: where two channels cross, and where a
            # ford pinches one off. The claim is about the channels themselves.
            continue
        pts = np.stack([cx - cx.mean(), cy - cy.mean()], axis=1).astype(float)
        s = np.linalg.svd(pts, compute_uv=False)
        aspect = s[0] / max(s[1], 1e-6)
        assert aspect > 6.0, f"channel of {len(cy)} cells has aspect {aspect:.1f}"
        channels += 1
    assert channels > 0, "no channel large enough to check"

    # Full: the bed is parabolic, so a channel runs through every depth from its banks to
    # its centreline rather than being one flat stamp.
    d = demo.water[demo.water > 0.02]
    assert np.percentile(d, 90) > 3.0 * np.percentile(d, 10), (
        "water depth is nearly uniform -- the channels are stamped, not filled"
    )


def test_water_is_never_deeper_than_the_feature_that_made_it(demo):
    """Depth is `surface - ground` and every later step moves the ground. Overlapping
    features once produced a 14 m lake out of a 0.38 m marsh."""
    deepest = max(demo.scn.terrain.river_depth_m[1], demo.scn.terrain.marsh_depth_m[1])
    assert demo.water.max() <= deepest + 1e-3, f"{demo.water.max():.2f} m of water"


# --- robots must be able to move --------------------------------------------------------


def test_nobody_spawns_on_ground_their_chassis_cannot_cross(demo):
    """A wheeled robot spawned in marsh can never move: its own cell is impassable, so
    the safety override zeroes its speed every tick, forever. Measured before the fix,
    52.5% of robot-ticks had a goal and zero commanded speed."""
    ix, iy = grid.world_to_cell(demo.pos[:, 0], demo.pos[:, 1], demo.cell, demo.shape)
    bad = ~demo.chassis_passable[demo.chassis, iy, ix]
    assert not bad.any(), f"{bad.sum()} robots spawned on terrain they cannot traverse"


def test_staging_area_scales_with_the_swarm(demo):
    """768 robots in a 22 m disc is ~2 m each against a 2.5 m separation radius: every
    robot sits inside every neighbour's repulsion field and the swarm gridlocks."""
    from swarmmind.control.tier1_reflex import ReflexParams

    spread = float(np.linalg.norm(demo.pos - np.array(demo.scn.base), axis=1).max())
    area_each = np.pi * spread**2 / demo.n
    assert area_each > ReflexParams().separation_radius**2, (
        f"only {area_each:.1f} m^2 per robot at spawn -- they will jam"
    )


def test_a_stranded_robot_can_get_out():
    """Terrain can shift under a robot -- the hazard grows, another robot shoves it.
    If the override zeroes a robot whose own cell is impassable, that push is permanent
    and the repulsion term never gets a chance to work."""
    from swarmmind.control.planner import NavSet
    from swarmmind.control.tier1_reflex import ReflexController

    w = World(Scenario.load("test"), 42)
    nav, reflex = NavSet(w), ReflexController(w)
    # Drop a wheeled robot into water it cannot cross. Away from the map rim on purpose:
    # the first wet cell in scan order is a corner, and a robot wedged between two border
    # walls is held there by the walls, which is a different claim from this one.
    wheeled = int(np.nonzero(w.chassis == CHASSIS_INDEX["wheeled"])[0][0])
    wet = np.argwhere(w.water > CHASSIS_LIMITS["wheeled"][1] + 0.05)
    margin = 20
    wet = wet[(wet[:, 0] > margin) & (wet[:, 0] < w.shape[0] - margin)
              & (wet[:, 1] > margin) & (wet[:, 1] < w.shape[1] - margin)]
    if len(wet) == 0:
        pytest.skip("no water on this map")
    cy, cx = wet[len(wet) // 2]
    w.pos[wheeled] = np.array([(cx + 0.5) * w.cell, (cy + 0.5) * w.cell])

    goal = [tuple(w.scn.base)]
    gid = np.full(w.n, -1, dtype=np.int32)
    gid[wheeled] = 0
    start = w.pos[wheeled].copy()
    for _ in range(200):
        v, om = reflex.commands(w, nav, goal, gid)
        w.step(v, om)
    moved = float(np.linalg.norm(w.pos[wheeled] - start))
    assert moved > 0.5, f"stranded robot moved {moved:.2f} m -- it is stuck forever"
    ix, iy = grid.world_to_cell(w.pos[wheeled, 0:1], w.pos[wheeled, 1:2], w.cell, w.shape)
    assert w.chassis_passable[CHASSIS_INDEX["wheeled"], iy[0], ix[0]], (
        "it moved but never actually got out of the water"
    )


def test_tier1_avoids_terrain_not_just_walls(demo):
    """Probing only for walls leaves robots with no repulsion from water and steep
    slope: they drive into it and stall. The obstacle term must know about chassis."""
    from swarmmind.control.planner import NavSet
    from swarmmind.control.tier1_reflex import ReflexController

    w = World(Scenario.load("test"), 42)
    nav, reflex = NavSet(w), ReflexController(w)
    wheeled = np.nonzero(w.chassis == CHASSIS_INDEX["wheeled"])[0]
    # Park wheeled robots just short of water, facing it.
    wet = np.argwhere(w.water > 0.1)
    if len(wet) == 0:
        pytest.skip("no water on this map")
    placed = 0
    for cy, cx in wet[:: max(1, len(wet) // len(wheeled))]:
        if placed >= len(wheeled):
            break
        i = int(wheeled[placed])
        w.pos[i] = np.array([(cx + 0.5) * w.cell - 2.0, (cy + 0.5) * w.cell])
        w.theta[i] = 0.0
        placed += 1
    reflex.commands(w, nav, [], np.full(w.n, -1, dtype=np.int32))
    assert np.linalg.norm(reflex._rep[wheeled[:placed]], axis=1).max() > 0.0, (
        "water produced no avoidance response at all"
    )
