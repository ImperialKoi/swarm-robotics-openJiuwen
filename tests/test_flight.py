"""Flight must cross ground obstacles, retain safety, and clear the displayed hills."""

import numpy as np
import pytest

from swarmmind.control.planner import NavSet
from swarmmind.control.tier1_reflex import ReflexController
from swarmmind.mission import Mission
from swarmmind.nodes.skill_executor import Assignment
from swarmmind.sim import grid
from swarmmind.sim.robot import ACTIVE, AIRBORNE_SPEED, CHASSIS_INDEX, FAILED, OUT_OF_COMMS
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World
from swarmmind.viz.terrain_surface import FLIGHT_CLEARANCE, FLIGHT_SLOPE, TerrainSurface


@pytest.fixture
def flight_world():
    w = World(Scenario.load("test"), 42)
    rotor = int(np.flatnonzero(w.chassis == CHASSIS_INDEX["rotor"])[0])
    ground = int(np.flatnonzero(w.chassis == CHASSIS_INDEX["wheeled"])[0])
    w.status[:] = FAILED
    w.status[[rotor, ground]] = ACTIVE
    w.occ[:] = grid.FREE
    w.chassis_passable[:] = True
    w.explored[:] = True
    w.pos[rotor], w.pos[ground] = (20.5, 30.5), (20.5, 40.5)
    w.theta[:] = 0
    return w, rotor, ground


@pytest.mark.parametrize("obstacle", ["wall", "water", "mountain", "rubble"])
def test_aircraft_cross_ground_barriers_at_flight_speed(flight_world, obstacle):
    w, rotor, ground = flight_world
    barrier = slice(int(21/w.cell), int(28/w.cell))
    if obstacle == "rubble":
        w.occ[:, barrier] = grid.RUBBLE
    else:
        w.chassis_passable[:, :, barrier] = False
        if obstacle == "wall":
            w.occ[:, barrier] = grid.WALL
        elif obstacle == "water":
            w.water[:, barrier] = 4.0
        else:
            w.slope[:, barrier] = 10.0
    nav = NavSet(w)
    reflex = ReflexController(w)
    goals = [(45.0, 30.5), (45.0, 40.5)]
    ids = np.full(w.n, -1)
    ids[[rotor, ground]] = [0, 1]
    w.airborne[rotor] = True
    start = w.pos.copy()
    for _ in range(50):
        v, omega = reflex.commands(w, nav, goals, ids)
        assert v[rotor] == pytest.approx(w.v_max[rotor])
        w.step(v, omega)
    assert w.pos[rotor, 0] == pytest.approx(start[rotor, 0] + 50*w.dt*w.v_max[rotor]*AIRBORNE_SPEED)
    assert w.pos[rotor, 0] > 28
    if obstacle != "rubble":
        assert w.pos[ground, 0] < 21, "ground safety floor must still stop the barrier"
    field = nav.field(CHASSIS_INDEX["rotor"], *goals[0])
    assert np.isfinite(nav.distance_at(CHASSIS_INDEX["rotor"], field, 20.5, 30.5))


def test_aircraft_ignore_ground_traffic_but_separate_from_other_aircraft(flight_world):
    w, rotor, ground = flight_world
    w.pos[ground] = w.pos[rotor] + [0.4, 0]
    w.airborne[rotor] = True
    reflex = ReflexController(w)
    reflex._separation(w)
    np.testing.assert_array_equal(reflex._sep[[rotor, ground]], 0)
    other = int(np.flatnonzero(w.chassis == CHASSIS_INDEX["rotor"])[1])
    w.status[other] = ACTIVE
    w.pos[other] = w.pos[ground]
    w.airborne[other] = True
    reflex._separation(w)
    assert reflex._sep[rotor, 0] < 0 < reflex._sep[other, 0]
    np.testing.assert_array_equal(reflex._sep[ground], 0)


def test_aircraft_cannot_leave_the_map_or_grant_flight_to_ground_units(flight_world):
    w, rotor, ground = flight_world
    w.pos[rotor] = (w.scn.map.width_m-w.radius[rotor]-.01, 30)
    w.airborne[[rotor, ground]] = True
    reflex = ReflexController(w)
    v = reflex._wall_override(w, w.v_max.copy())
    assert v[rotor] == 0
    before = w.pos[rotor].copy()
    w.step(w.v_max, np.zeros(w.n))
    np.testing.assert_array_equal(w.pos[rotor], before)
    assert not w.airborne[ground]
    w._kill(rotor, "test")
    assert not w.airborne[rotor]


def test_takeoff_precedes_steering_and_unsafe_ground_never_forces_a_landing():
    m = Mission(Scenario.load("test"), 42, hivemind=False)
    w = m.world
    i = int(np.flatnonzero(w.chassis == CHASSIS_INDEX["rotor"])[0])
    w.status[:] = FAILED
    w.status[i] = ACTIVE
    w.in_comms[:] = True
    w.pos[i], w.theta[i] = (20.5, 30.5), 0
    w.chassis_passable[:] = True
    w.chassis_passable[:, :, int(21/w.cell):int(28/w.cell)] = False
    w.explored[:] = True
    m.nav = NavSet(w)
    m.executor.idle_explore = False
    m.executor.roaming_relays = False
    m.executor.assign(w, i, Assignment("flight", "explore", (40.5, 30.5)))
    m._alloc_next = 100
    m.tick()
    assert w.airborne[i] and w.pos[i, 0] > 20.5
    # Blind over the water: remain aloft, do not sink or reveal it as a ground sensor.
    w.pos[i] = (24.5, 30.5)
    w.explored[:] = False
    m.tick()
    assert not w.can_land()[i] and w.airborne[i]
    assert not w.explored.any()


def test_flight_envelope_climbs_before_cliffs_and_clears_every_terrain_lod():
    heights = np.zeros((48, 64))
    heights[:, 32:40] = 30  # Vertical mountain face, deliberately harsher than the demo.
    surface = TerrainSurface(heights, np.zeros_like(heights), np.zeros_like(heights), 1)
    x = np.linspace(0, 64, 257)
    altitude = surface.flight_height_at_world(x, np.full_like(x, 24))
    assert altitude[0] < altitude[80] < altitude[128], "climb must begin before the mountain"
    assert np.max(np.abs(np.diff(altitude)/np.diff(x))) <= FLIGHT_SLOPE + 1e-8
    for stride in (1, 2, 4, 8):
        for offset in (-2.0, 0.0, 2.0):
            terrain = surface.height_at_world(x+offset, np.full_like(x, 24), stride)
            assert np.all(altitude >= terrain + FLIGHT_CLEARANCE - 1e-8)
    basis, origin = surface.robot_pose(24, 24, .7, "rotor", airborne=True)
    np.testing.assert_array_equal(basis[:, 2], [0, 0, 1])
    assert origin[2] > surface.robot_pose(24, 24, .7, "legged")[1][2] + 5
    assert surface.robot_pose(24, 24, .7, "rotor")[1][2] < origin[2]
    assert surface.robot_pose(24, 24, .7, "wheeled", airborne=True)[1][2] < origin[2]


def test_disconnected_rotor_can_take_off_after_its_own_ground_inspection():
    m = Mission(Scenario.load("test"), 42, hivemind=False)
    w = m.world
    i = int(np.flatnonzero(w.chassis == CHASSIS_INDEX["rotor"])[0])
    w.status[:] = FAILED
    w.status[i] = OUT_OF_COMMS
    w.in_comms[:] = False
    w.occ[:] = grid.FREE
    w.chassis_passable[:] = True
    w.explored[:] = False
    w.pos[i], w.theta[i] = (80.5, 40.5), 0
    m.nav = NavSet(w)
    m.executor.assign(w, i, Assignment("flight", "explore", (90.0, 40.5)))
    m._alloc_next = 100
    m.tick()  # The camera samples on this tick, but its observations must stay private.
    assert not w.in_comms[i] and not w.explored.any()
    assert i in w._pending_cells
    assert w.airborne[i] and w.pos[i, 0] > 80.5, "waiting for shared fog would strand the rotor"


def test_flight_cameras_and_distant_markers_follow_the_aircraft(flight_world):
    from swarmmind.viz.render3d import Renderer3D

    w, rotor, _ = flight_world
    r = Renderer3D(w, 640, 400)
    w.airborne[rotor] = True
    origin = r.unit_pose(w, rotor)[1]
    assert r.pov(w, rotor).pos[2] == pytest.approx(origin[2]+1.1)
    assert r.chase(w, rotor).target[2] == pytest.approx(origin[2]+r.CHASE_ANCHOR_Z)
    exclude = np.ones(w.n, dtype=bool)
    exclude[rotor] = False
    points, _ = r._robot_points(w, None, exclude)
    assert points[:, 2].min() == pytest.approx(origin[2]+.3, abs=1e-5)
    w.airborne[rotor] = False
    assert r.unit_pose(w, rotor)[1][2] < origin[2]-5


def test_native_flight_cameras_and_picking(tmp_path):
    import shutil
    import subprocess
    from pathlib import Path

    native = shutil.which("godot") or "/Applications/Godot.app/Contents/MacOS/Godot"
    if not Path(native).is_file():
        pytest.skip("Native Godot unavailable; offline flight geometry is tested separately")
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [native, "--headless", "--path", str(root / "godot"),
         "--log-file", str(tmp_path / "godot.log"), "--script", "res://tests/flight_check.gd"],
        text=True, capture_output=True, timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "FLIGHT_DASHBOARD_OK" in result.stdout, result.stdout + result.stderr
    assert "SCRIPT ERROR" not in result.stderr, result.stderr
