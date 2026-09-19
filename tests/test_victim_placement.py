"""Concealment must not undo the rescue chain's reachability guarantees."""

from dataclasses import replace

import numpy as np
import pytest

from swarmmind.control.planner import NavFields
from swarmmind.sim import grid
from swarmmind.sim.placement import cover_masks
from swarmmind.sim.robot import CHASSIS_INDEX
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World


@pytest.mark.parametrize("seed", Scenario.load("demo").demo_seeds)
def test_demo_casualties_have_cover_and_a_rescue_approach(seed):
    world = World(Scenario.load("demo"), seed)
    cfg = world.scn.victims
    points = np.array([v.pos for v in world.victims])
    ix, iy = grid.world_to_cell(points[:, 0], points[:, 1], world.cell, world.shape)
    buried = np.array([v.buried for v in world.victims])
    wall, rubble = cover_masks(world.occ, world.cell, cfg.cover_radius_m)
    covered = (wall | rubble)[iy, ix]
    assert covered.sum() >= round(cfg.cover_fraction * cfg.count)
    assert (~covered).sum() > 0, "some casualties should still be exposed"
    assert rubble[iy[buried], ix[buried]].all(), "burial must be associated with debris"
    assert wall[iy[~buried], ix[~buried]].sum() > (~buried).sum() / 2
    assert (world.water[iy, ix] == 0).all(), "a wading route is not a place to lie"
    assert grid.clearance_mask(world.occ)[iy, ix].all()

    legged = world.chassis_passable[CHASSIS_INDEX["legged"]]
    bx, by = grid.world_to_cell(*world.scn.base, world.cell, world.shape)
    assert grid._flood(legged, (int(bx), int(by)))[iy, ix].all()
    nav = NavFields(legged, world.cell)
    distances = nav.distance_at(nav.field(*world.scn.base), points[:, 0], points[:, 1])
    assert (distances < 1e8).all(), "a fine-grid pocket alone is not an approach"
    assert len(points) == cfg.count and buried.sum() == cfg.buried_count
    pairwise = np.linalg.norm(points[:, None] - points[None, :], axis=2)
    np.fill_diagonal(pairwise, np.inf)
    assert pairwise.min() >= cfg.min_separation_m
    for x, y, radius in world.scn.keepouts():
        assert (np.linalg.norm(points - [x, y], axis=1) > radius * 1.5).all()


def test_empty_landscape_falls_back_without_using_the_map_border_as_cover():
    scenario = Scenario.load("test")
    scenario = replace(scenario, map=replace(scenario.map, n_clusters=0))
    world = World(scenario, 42)
    wall, rubble = cover_masks(world.occ, world.cell, scenario.victims.cover_radius_m)
    assert not wall.any() and not rubble.any()
    assert len(world.victims) == scenario.victims.count
    assert sum(v.buried for v in world.victims) == scenario.victims.buried_count


def test_impossible_spacing_relaxes_without_overlapping_or_losing_casualties():
    scenario = Scenario.load("test")
    scenario = replace(scenario, victims=replace(scenario.victims, min_separation_m=1000))
    world = World(scenario, 42)
    assert world.victim_separation == 0
    points = np.array([v.pos for v in world.victims])
    assert len(np.unique(points, axis=0)) == scenario.victims.count
    assert sum(v.buried for v in world.victims) == scenario.victims.buried_count


@pytest.mark.parametrize("cell", [0.5, 1.0])
def test_cover_radius_is_in_metres_and_does_not_wrap(cell):
    occ = np.zeros((40, 40), dtype=np.uint8)
    occ[20, 20] = grid.WALL
    occ[1, 1] = grid.RUBBLE
    wall, rubble = cover_masks(occ, cell, 3.0)
    assert wall[20, 20 + int(3 / cell)]
    assert not wall[20, 21 + int(3 / cell)]
    assert not wall[20 + int(3 / cell), 20 + int(3 / cell)]
    assert rubble[1, 1] and not rubble[-1, -1]


def test_placement_is_repeatable_without_changing_other_subsystems():
    scenario = Scenario.load("test")
    a, b = World(scenario, 42), World(scenario, 42)
    assert np.array_equal([v.pos for v in a.victims], [v.pos for v in b.victims])
    assert [v.buried for v in a.victims] == [v.buried for v in b.victims]
    exposed = World(replace(scenario, victims=replace(scenario.victims, cover_fraction=0)), 42)
    for name in ("occ", "water", "height", "pos", "theta"):
        assert np.array_equal(getattr(a, name), getattr(exposed, name))
    assert np.array_equal(a.hazard.origin, exposed.hazard.origin)
