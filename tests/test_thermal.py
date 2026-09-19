"""Display heat respects occlusion, casualty state, and deterministic replay."""

import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from swarmmind.viz.thermal import in_sensor, intensity, screen_pass


def test_exposure_and_variation_never_invert_the_heat_order():
    exposed, buried = [], []
    for x in np.linspace(1, 40, 100):
        for time in (0, 1, 12.5):
            exposed.append(intensity(x, 9.7, 0, False, time))
            buried.append(intensity(x, 9.7, 0, True, time))
            assert intensity(x, 9.7, 2, True, time) == intensity(x, 9.7, 2, False, time)
    assert min(exposed) > max(buried)
    assert np.ptp(exposed) > .09
    assert intensity(2, 3, 0, True, 5) == intensity(2, 3, 0, True, 5)
    assert intensity(2, 3, 0, True, 5) != intensity(2, 3, 0, True, 6)
    for state in (3, 4):
        assert intensity(2, 3, state, True) == 0  # no stale bodies after pickup


def test_sensor_cone_range_and_occlusion():
    occ = np.zeros((40, 40), dtype=np.uint8)
    origin = np.array([4.5, 10.5])
    assert in_sensor(occ, 1, origin, 0, [12.5, 10.5])
    assert not in_sensor(occ, 1, origin, 0, [2.5, 10.5])
    assert not in_sensor(occ, 1, origin, 0, [5.5, 15.5])
    assert not in_sensor(occ, 1, origin, 0, [32.5, 10.5])
    occ[10, 8] = 1
    assert not in_sensor(occ, 1, origin, 0, [12.5, 10.5])
    occ[10, 8] = 2  # traversable rubble is not a solid wall
    assert in_sensor(occ, 1, origin, 0, [12.5, 10.5])


def test_screen_heat_is_brighter_than_background_and_replays():
    frame = np.array([[[255, .78*255, 255], [255, .34*255, 255], [150, 150, 150]]])
    result = screen_pass(frame, 10)
    luminance = result @ [.2126, .7152, .0722]
    assert luminance[0, 0] > luminance[0, 1] > luminance[0, 2]
    np.testing.assert_array_equal(result, screen_pass(frame, 10))


def test_heat_meshes_respect_depth_without_mutating_the_world():
    import copy

    from swarmmind.rng import STREAMS
    from swarmmind.sim.scenario import Scenario
    from swarmmind.sim.world import World
    from swarmmind.viz.render3d import Camera3D, Renderer3D
    from swarmmind.viz.thermal import draw_sources

    world = World(Scenario.load("test"), 42)
    world.occ[:] = 0
    world.height[:] = 0
    world.water[:] = 0
    world.victims = [world.victims[0]]
    world.victims[0].pos[:] = [14, 14]
    world.victims[0].state = 2
    world.pos[0] = [7, 14]
    world.theta[0] = 0
    rng_before = [copy.deepcopy(world.rng[name].bit_generator.state) for name in STREAMS]
    camera = Camera3D(np.array([7., 14., 2.8]), np.array([14., 14., .5]), fov_deg=48)
    renderer = Renderer3D(world, 160, 100)
    open_frame = np.zeros((100, 160, 3))
    draw_sources(renderer, world, camera, open_frame, np.full((100, 160), np.inf), 0)
    assert np.count_nonzero(open_frame) > 100
    occluded_frame = np.zeros_like(open_frame)
    # An opaque foreground surface at 1 m covers the body at 7 m.
    draw_sources(renderer, world, camera, occluded_frame, np.ones((100, 160)), 0)
    assert not np.any(occluded_frame)
    assert world.victims[0].state == 2
    assert [world.rng[name].bit_generator.state for name in STREAMS] == rng_before


def test_native_thermal_controls_and_visibility(tmp_path):
    native = shutil.which("godot") or "/Applications/Godot.app/Contents/MacOS/Godot"
    if not Path(native).is_file():
        pytest.skip("Godot unavailable; thermal reference tested separately")
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [native, "--headless", "--path", str(root / "godot"),
         "--log-file", str(tmp_path / "godot.log"), "--script", "res://tests/thermal_check.gd"],
        text=True, capture_output=True, timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "THERMAL_CHECK_OK" in result.stdout, result.stdout + result.stderr
    assert "SCRIPT ERROR" not in result.stderr, result.stderr
