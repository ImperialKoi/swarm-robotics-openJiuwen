"""Scale, drainage, reproducibility and collection access on the authored demo crop."""

import json
from pathlib import Path

import numpy as np
import pytest

from swarmmind.sim import grid
from swarmmind.sim.landscape import channel_fields
from swarmmind.sim.robot import CHASSIS_INDEX, GROUND_CHASSIS
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World
from swarmmind.viz.reference_landscape import biome_at


def test_exported_dashboard_layout_matches_scenario_metres():
    scn = Scenario.load("demo")
    layout = scn.terrain.reference
    assert json.loads(Path("godot/assets/landscapes/demo.json").read_text()) == layout
    assert layout["map_m"] == [scn.map.width_m, scn.map.height_m] == [320, 216]
    assert scn.map.width_m*scn.map.height_m == 360*240*.8
    assert scn.map.cell == 1  # Crop the map; do not shrink robots and features.
    assert 4 <= layout["road_width_m"] <= 6
    assert all(6 <= min(b[2:]) <= max(b[2:]) <= 10 for b in layout["buildings"])
    for x, y, width, height in layout["buildings"]:
        assert 0 < x < x+width < scn.map.width_m
        assert 0 < y < y+height < scn.map.height_m
        assert biome_at(layout, x+width/2, y+height/2) == "ruin"


def test_three_downhill_arms_share_one_confluence_and_water_level():
    scn = Scenario.load("demo")
    layout = scn.terrain.reference
    a, b, outlet = layout["channels"]
    assert a["points"][-1] == b["points"][-1] == outlet["points"][0]
    for channel in layout["channels"]:
        points = np.asarray(channel["points"])
        assert np.all(np.diff(points[:, 2]) < 0), "river flows uphill"
        assert 14 <= channel["width_m"] <= 22
    # Before the two dry road crossings, all three channels form one wet component.
    d, width, _, _ = channel_fields(scn.grid_shape, scn.map.cell, layout)
    wet = d < width
    x, y = np.asarray(a["points"][-1][:2], int)
    np.testing.assert_array_equal(grid._flood(wet, (x, y)), wet)
    assert wet[0].any() and wet[:, -1].any() and wet[-1].any()
    assert not wet[:, 0].any(), "a fourth river arm appeared"


@pytest.mark.parametrize("seed", Scenario.load("demo").demo_seeds)
def test_reference_crop_keeps_all_collection_points_reachable(seed):
    world = World(Scenario.load("demo"), seed)
    bx, by = grid.world_to_cell(*world.scn.base, world.cell, world.shape)
    for name in GROUND_CHASSIS:
        reachable = grid._flood(world.chassis_passable[CHASSIS_INDEX[name]], (int(bx), int(by)))
        for point in [world.scn.base, *world.scn.extraction_zones]:
            x, y = grid.world_to_cell(*point, world.cell, world.shape)
            assert reachable[y, x], (seed, name, point)
            assert world.water[y, x] == 0
    x, y = grid.world_to_cell(*world.pos.T, world.cell, world.shape)
    assert world.chassis_passable[world.chassis, y, x].all()
    for x, y, width, height in world.scn.terrain.reference["buildings"]:
        footprint = np.s_[y:y+height, x:x+width]
        assert (world.occ[footprint] == grid.WALL).all(), "a road or depot erased a house"
        assert world.water[footprint].max() == 0
        assert np.ptp(world.terrain[footprint]) < .05, "house foundation is not level"
    # No random tree or building footprint may obstruct the wet channel.
    assert not ((world.occ[1:-1, 1:-1] == grid.WALL)
                & (world.water[1:-1, 1:-1] > .02)).any()


def test_reference_crop_is_repeatable_without_changing_terrain_scale():
    first, second = (World(Scenario.load("demo"), 42) for _ in range(2))
    for name in ("occ", "height", "terrain", "water", "pos"):
        np.testing.assert_array_equal(getattr(first, name), getattr(second, name))
    assert 35 < first.terrain.max() < 65
