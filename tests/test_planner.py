"""Navigation: flow fields, A*, frontier detection."""

from __future__ import annotations

import numpy as np
import pytest

from swarmmind.control.planner import NavFields, astar, frontier_mask, frontier_targets
from swarmmind.sim import grid
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World


@pytest.fixture(scope="module")
def world():
    return World(Scenario.load("test"), 42)


def test_downsample_is_majority_not_any(world):
    p = np.array([[True, True, False, False],
                  [True, True, False, False]])
    assert grid.downsample(p, 2).tolist() == [[True, False]]


def test_downsample_pads_non_divisible_shapes():
    p = np.ones((5, 7), dtype=bool)
    assert grid.downsample(p, 4).shape == (2, 2)


def test_field_decreases_toward_goal(world):
    nav = NavFields(world.passable, world.cell)
    gx, gy = world.scn.base
    f = nav.field(gx, gy)
    ix, iy = nav.to_coarse(np.asarray(gx), np.asarray(gy))
    assert f[int(iy), int(ix)] == 0.0
    reachable = f[nav.coarse] < 1e8
    assert reachable.mean() > 0.9, "most of the coarse map should reach base"


def test_descend_points_downhill(world):
    """The direction returned must actually reduce the field value."""
    nav = NavFields(world.passable, world.cell)
    f = nav.field(*world.scn.base)
    rng = np.random.default_rng(0)
    iy, ix = np.nonzero(nav.coarse & (f > 0) & (f < 1e8))
    k = rng.choice(len(ix), size=200, replace=False)
    x = (ix[k] + 0.5) * nav.coarse_cell
    y = (iy[k] + 0.5) * nav.coarse_cell
    d = nav.descend(f, x, y)
    assert np.all(np.linalg.norm(d, axis=1) > 0.5), "every non-goal cell must have a downhill step"
    here = f[iy[k], ix[k]]
    nx, ny = nav.to_coarse(x + d[:, 0] * nav.coarse_cell, y + d[:, 1] * nav.coarse_cell)
    assert np.all(f[ny, nx] < here)


@pytest.mark.parametrize("edge_steering", [False, True])
def test_descend_to_is_bit_identical_to_descend(world, edge_steering):
    """The precomputed direction grid must reproduce `descend` exactly, not merely well.

    `descend` stays the reference definition of the manoeuvre; `descend_to` is an
    optimisation that hoists it to once per goal. Anything less than bit equality here
    changes robot headings, and through them the seed-42 scorecard hash -- so this is
    `assert_array_equal`, not `allclose`.
    """
    nav = NavFields(world.passable, world.cell, edge_steering=edge_steering)
    rng = np.random.default_rng(7)
    h, w = nav.shape
    # Several goals, including one snapped out of a wall, and positions everywhere --
    # on the goal cell, in unreachable pockets and out past the map edge.
    giy, gix = np.nonzero(nav.coarse)
    goals = [world.scn.base]
    for k in rng.choice(len(gix), size=4, replace=False):
        goals.append(((gix[k] + 0.5) * nav.coarse_cell, (giy[k] + 0.5) * nav.coarse_cell))

    x = rng.uniform(-5.0, w * nav.coarse_cell + 5.0, size=400)
    y = rng.uniform(-5.0, h * nav.coarse_cell + 5.0, size=400)
    for gx, gy in goals:
        expect = nav.descend(nav.field(gx, gy), x, y)
        np.testing.assert_array_equal(nav.descend_to(gx, gy, x, y), expect)


def test_direction_grid_is_built_once_per_goal_and_evicted_with_its_field(world):
    """It is derived state: it must not be rebuilt per tick, nor outlive its field."""
    nav = NavFields(world.passable, world.cell, max_entries=2)
    x = np.array([10.0, 20.0])
    y = np.array([10.0, 20.0])
    for _ in range(5):
        nav.descend_to(*world.scn.base, x, y)
    assert len(nav._dirs) == 1, "one goal must not build five direction grids"

    # Overflow the field cache; the derived grids must go with the fields.
    giy, gix = np.nonzero(nav.coarse)
    for k in gix[:6]:
        nav.descend_to((k + 0.5) * nav.coarse_cell,
                       (giy[0] + 0.5) * nav.coarse_cell, x, y)
    assert len(nav._dirs) <= len(nav._cache) <= 2


def test_goal_key_memo_survives_a_moving_goal(world):
    """A goal that drifts must miss the memo rather than resolve to a stale cell."""
    nav = NavFields(world.passable, world.cell)
    bx, by = world.scn.base
    assert nav._resolve(bx, by) == nav._resolve(bx, by)
    far = nav._resolve(bx + 40.0, by + 20.0)
    assert far != nav._resolve(bx, by)


@pytest.mark.parametrize("enabled", [False, True])
def test_edge_steering_requires_explicit_cli_opt_in(monkeypatch, enabled):
    from swarmmind import cli
    from swarmmind.mission import Mission

    missions = []

    def capture(scn, seed, **kwargs):
        mission = Mission(scn, seed, **kwargs)
        missions.append(mission)
        return mission

    monkeypatch.setattr(cli, "Mission", capture)
    args = ["run", "--headless", "--scenario", "test", "--max-time", "0"]
    if enabled:
        args.append("--edge-steering")
    assert cli.main(args) == 0
    assert all(nav.edge_steering == enabled for nav in missions[0].nav.nav)


def test_field_cache_is_reused(world):
    nav = NavFields(world.passable, world.cell)
    nav.field(*world.scn.base)
    misses = nav.misses
    for _ in range(20):
        nav.field(*world.scn.base)
    assert nav.misses == misses and nav.hits >= 20


def test_goal_in_a_wall_snaps_to_passable(world):
    """A victim can be under rubble; a goal there must still produce a usable field."""
    nav = NavFields(world.passable, world.cell)
    iy, ix = np.nonzero(~nav.coarse)
    wx = (ix[0] + 0.5) * nav.coarse_cell
    wy = (iy[0] + 0.5) * nav.coarse_cell
    f = nav.field(wx, wy)
    assert (f == 0.0).sum() == 1, "snapped goal must still have exactly one zero cell"


def test_astar_finds_a_connected_path(world):
    base = grid.world_to_cell(
        np.asarray(world.scn.base[0]), np.asarray(world.scn.base[1]), world.cell, world.shape
    )
    start = (int(base[0]), int(base[1]))
    iy, ix = np.nonzero(world.passable)
    goal = (int(ix[len(ix) // 2]), int(iy[len(iy) // 2]))
    path = astar(world.passable, start, goal)
    assert path is not None and path[0] == start and path[-1] == goal
    for (ax, ay), (bx, by) in zip(path, path[1:], strict=False):
        assert max(abs(ax - bx), abs(ay - by)) == 1
        assert world.passable[by, bx]


def test_astar_returns_none_into_a_wall(world):
    iy, ix = np.nonzero(~world.passable)
    base = grid.world_to_cell(
        np.asarray(world.scn.base[0]), np.asarray(world.scn.base[1]), world.cell, world.shape
    )
    assert astar(world.passable, (int(base[0]), int(base[1])), (int(ix[0]), int(iy[0]))) is None


def test_frontier_lies_on_the_unknown_side():
    """Defining the frontier on the explored side makes every explore task complete the
    instant it is issued, and the swarm never moves. Guard the direction explicitly.

    Needs a mission that has actually looked at something: explored space now comes from
    camera frames, so a freshly constructed world knows nothing at all."""
    from swarmmind.mission import Mission

    m = Mission(Scenario.load("test"), 42)
    for _ in range(60):
        m.tick()
    world = m.world
    assert world.explored.any(), "cameras revealed nothing"
    fm = frontier_mask(world.explored, world.passable)
    assert fm.any()
    assert not (fm & world.explored).any(), "frontier cells must be UNexplored"
    assert (fm & world.passable).sum() == fm.sum(), "frontier cells must be passable"


def test_frontier_targets_are_ordered_and_deterministic():
    from swarmmind.mission import Mission

    m = Mission(Scenario.load("test"), 42)
    for _ in range(60):
        m.tick()
    world = m.world
    a = frontier_targets(world, max_targets=20)
    b = frontier_targets(world, max_targets=20)
    assert [t.pos for t in a] == [t.pos for t in b]
    assert [t.size for t in a] == sorted([t.size for t in a], reverse=True)
