"""Display rigs must describe real activity and never change a mission."""

from __future__ import annotations

import copy

import numpy as np
import pytest

from swarmmind.contracts.schemas import ACTIVITY
from swarmmind.mission import Mission
from swarmmind.rng import STREAMS
from swarmmind.sim.robot import CHASSIS_INDEX, LANE_INDEX
from swarmmind.sim.scenario import Scenario
from swarmmind.viz.render3d import Camera3D, Renderer3D, raster_triangles, render_unit
from swarmmind.viz.units import (
    CARRY,
    DIG,
    DISABLED,
    RELAY,
    REST,
    SCAN,
    animation_state,
    build_model,
    joint_angle,
    posed_parts,
    valid_variants,
)


def test_dig_animation_requires_assignment_and_a_nearby_active_excavation():
    lane, chassis = LANE_INDEX["scoop"], CHASSIS_INDEX["tracked"]
    position = (10.0, 20.0)
    args = (lane, chassis, ACTIVITY["dig"], 0, position)
    assert animation_state(*args) == REST
    assert animation_state(*args, digs=((12.501, 20.0),)) == REST
    assert animation_state(*args, digs=((12.5, 20.0),)) == DIG
    assert animation_state(*args, digs=((3.0, 2.0), (10.0, 20.1))) == DIG
    assert animation_state(lane, chassis, ACTIVITY["explore"], 0, position,
                           digs=(position,)) == REST
    assert animation_state(LANE_INDEX["none"], chassis, ACTIVITY["dig"], 0, position,
                           digs=(position,)) == REST


def test_carrying_and_relay_post_animations_require_matching_role_and_activity():
    for status in (0, 1):  # Out-of-comms units still execute their local task.
        assert animation_state(LANE_INDEX["gripper"], 0, ACTIVITY["carry"],
                               status, (0, 0)) == CARRY
        assert animation_state(LANE_INDEX["gripper"], 0, ACTIVITY["explore"],
                               status, (0, 0)) == REST
        assert animation_state(LANE_INDEX["antenna"], 0, ACTIVITY["relay_post"],
                               status, (0, 0)) == RELAY
        assert animation_state(LANE_INDEX["antenna"], 0, ACTIVITY["carry"],
                               status, (0, 0)) == REST


@pytest.mark.parametrize("activity", (ACTIVITY["explore"], ACTIVITY["investigate"]))
def test_rotor_camera_never_claims_scanning_without_landed_telemetry(activity):
    assert animation_state(LANE_INDEX["none"], CHASSIS_INDEX["rotor"],
                           activity, 0, (0, 0)) == REST
    for chassis in ("wheeled", "tracked", "legged"):
        assert animation_state(LANE_INDEX["none"], CHASSIS_INDEX[chassis],
                               activity, 0, (0, 0)) == SCAN


@pytest.mark.parametrize("status", (2, 3))
def test_failed_or_destroyed_status_overrides_all_activity(status):
    for lane, activity in (("none", "explore"), ("scoop", "dig"),
                           ("gripper", "carry"), ("antenna", "relay_post")):
        assert animation_state(LANE_INDEX[lane], 0, ACTIVITY[activity], status,
                               (0, 0), digs=((0, 0),)) == DISABLED
        model = build_model(lane, "legged")
        for joint in model.joints:
            assert joint_angle(joint, 0, 0, DISABLED) == joint_angle(joint, 4.7, 2.1, DISABLED)
    model = build_model("antenna", "rotor")
    for joint in model.joints:
        assert joint_angle(joint, 0, 0, DISABLED) == joint_angle(joint, 4.7, 2.1, DISABLED)


@pytest.mark.parametrize("chassis", ("wheeled", "tracked", "legged"))
def test_locomotion_mechanisms_depend_on_distance_and_do_not_walk_while_parked(chassis):
    model = build_model("none", chassis)
    locomotion = [joint for joint in model.joints if joint.kind in ("wheel", "gait")]
    assert locomotion
    for joint in locomotion:
        angle = joint_angle(joint, 0.0, 0.07, REST)
        assert joint_angle(joint, 6.73, 0.07, REST) == angle
        assert joint_angle(joint, 6.73, 0.21, REST) != pytest.approx(angle)


@pytest.mark.parametrize(("lane", "chassis"), valid_variants())
def test_articulated_geometry_stays_above_ground_through_work_and_travel(lane, chassis):
    model = build_model(lane, chassis)
    work = {"none": SCAN, "scoop": DIG, "gripper": CARRY, "antenna": RELAY}[lane]
    distance = np.pi / 2 if chassis in ("wheeled", "tracked") else 1.0
    for state in (REST, work, DISABLED):
        for fraction in np.linspace(0, 1, 17):
            for part, vertices in posed_parts(model, time=4 * fraction,
                                              travel=distance * fraction, state=state):
                assert np.isfinite(vertices).all()
                assert vertices[:, 2].min() >= -.03, (
                    f"{model.name}/{part.name} penetrates the floor at state {state}, "
                    f"phase {fraction}: {vertices[:, 2].min():.4f} m")
                assert not part.vertices.flags.writeable
                assert not part.triangles.flags.writeable


def test_triangle_depth_occlusion_is_independent_of_submission_order():
    camera = Camera3D(np.array([0.0, -5.0, 0.0]), np.zeros(3))
    # Equal projected triangles at depths 4 and 6, both facing the camera.
    face = np.array([[-1.0, 0.0, -1.0], [1.0, 0.0, -1.0], [0.0, 0.0, 1.0]])
    near = face * .8 + (0.0, -1.0, 0.0)
    far = face * 1.2 + (0.0, 1.0, 0.0)
    triangles = np.array([[0, 1, 2]])
    outputs = []
    for sequence in ((near, far), (far, near)):
        image = np.zeros((48, 64, 3), dtype=np.float64)
        depth = np.full((48, 64), np.inf)
        for vertices in sequence:
            color = (200, 0, 0) if vertices is near else (0, 200, 0)
            raster_triangles(image, depth, camera, vertices, triangles, color)
        assert image[24, 32, 0] > 0 and image[24, 32, 1] == 0
        assert depth[24, 32] == pytest.approx(4)
        outputs.append(image)
    np.testing.assert_array_equal(*outputs)


def test_triangle_rasterizer_culls_backfaces_and_behind_camera():
    camera = Camera3D(np.array([0.0, -5.0, 0.0]), np.zeros(3))
    vertices = np.array([[-1.0, 0.0, -1.0], [1.0, 0.0, -1.0], [0.0, 0.0, 1.0]])
    image = np.zeros((24, 32, 3))
    depth = np.full((24, 32), np.inf)
    raster_triangles(image, depth, camera, vertices, np.array([[0, 2, 1]]), (255, 0, 0))
    raster_triangles(image, depth, camera, vertices - (0, 6, 0),
                     np.array([[0, 1, 2]]), (255, 0, 0))
    assert not image.any() and np.isinf(depth).all()


def test_studio_render_is_repeatable_and_preserves_cached_geometry():
    model = build_model("scoop", "tracked")
    originals = [(part.vertices.tobytes(), part.triangles.tobytes()) for part in model.parts]
    before = render_unit(model, time=.13, state=DIG, width=120, height=100)
    other_pose = render_unit(model, time=.72, state=DIG, width=120, height=100)
    after = render_unit(model, time=.13, state=DIG, width=120, height=100)
    assert before.dtype == np.uint8 and before.shape == (100, 120, 3)
    assert np.any(before != other_pose)
    np.testing.assert_array_equal(before, after)
    assert originals == [(part.vertices.tobytes(), part.triangles.tobytes()) for part in model.parts]


def test_scene_render_preserves_mission_arrays_rng_and_next_tick():
    mission = Mission(Scenario.load("test"), 42, hivemind=False)
    control = Mission(Scenario.load("test"), 42, hivemind=False)
    world = mission.world
    before = {name: value.copy() for name, value in vars(world).items()
              if isinstance(value, np.ndarray)}
    rng_before = {name: copy.deepcopy(world.rng[name].bit_generator.state) for name in STREAMS}
    card_before = world.scorecard().hash()
    renderer = Renderer3D(world, width=96, height=72)
    frame = renderer.render(world, renderer.chase(world, 0))
    assert frame.shape == (72, 96, 3)
    assert card_before == world.scorecard().hash()
    for name, values in before.items():
        np.testing.assert_array_equal(getattr(world, name), values, err_msg=name)
    for name in STREAMS:
        assert world.rng[name].bit_generator.state == rng_before[name], f"Rendering consumed {name} RNG"
    mission.tick()
    control.tick()
    assert mission.scorecard().hash() == control.scorecard().hash()
    np.testing.assert_array_equal(mission.world.pos, control.world.pos)
