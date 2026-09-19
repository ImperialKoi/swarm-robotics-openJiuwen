"""Verify animation parity, lazy assets, and native CPU culling/LOD when available."""

from __future__ import annotations

import json
import re
import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from swarmmind.contracts.schemas import ACTIVITY
from swarmmind.viz.units import (
    CARRY,
    DIG,
    DISABLED,
    LANE_RGB,
    RELAY,
    REST,
    SCAN,
    build_lod_model,
    build_model,
    joint_angle,
    posed_parts,
    rotation,
    valid_variants,
)

ROOT = Path(__file__).resolve().parents[1]
FLEET = (ROOT / "godot/scripts/unit_fleet.gd").read_text()
SHADER = (ROOT / "godot/shaders/units.gdshader").read_text()
MAIN = (ROOT / "godot/scripts/main.gd").read_text()
KINDS = json.loads(re.search(r"const JOINT_KINDS := (\[.*?\])", FLEET)[1])


def _expression(source, context):
    if "?" in source:
        condition, rest = source.split("?", 1)
        yes, no = rest.split(":", 1)
        return _expression(yes if _expression(condition, context) else no, context)
    return eval(source.strip(), {"__builtins__": {}}, context)


def _shader_angle(joint, time, travel, state):
    kind = KINDS.index(joint.kind)
    context = {"p": SimpleNamespace(x=kind, y=joint.amplitude, z=joint.frequency, w=joint.phase),
               "animation_time": time, "travel": travel, "state": state,
               "kind": kind, "sin": np.sin, "TAU": 2*np.pi}
    body = SHADER.split("float joint_angle(", 1)[1].split("void vertex()", 1)[0]
    if state == DISABLED:
        expression = re.search(r"if \(state == 5\) \{\s*return (.*?);", body)[1]
    else:
        context["wave"] = _expression(re.search(r"float wave = (.*?);", body)[1], context)
        cases = dict(re.findall(r"if \(kind == (\d+)\) \{ return (.*?); \}", body))
        expression = cases.get(str(kind), "0.0")
    return _expression(expression, context)


@pytest.mark.parametrize(("lane", "chassis"), valid_variants())
def test_shader_expressions_match_every_joint_in_every_display_state(lane, chassis):
    for joint in build_model(lane, chassis).joints:
        for state in (REST, SCAN, DIG, CARRY, RELAY, DISABLED):
            for time, travel in ((0.0, 0.0), (.173, -.46), (14.61, 10.72), (420.0, 813.2)):
                assert _shader_angle(joint, time, travel, state) == pytest.approx(
                    joint_angle(joint, time, travel, state), abs=1e-10)


def test_godot_coordinate_reflection_preserves_joint_rotation_and_outward_normals():
    # Execute the coordinate expression in the loader, so an axis reorder is caught.
    line = FLEET.split("static func _map_vector", 1)[1].split("\n\n", 1)[0].split("return Vector3(", 1)[1]
    expression = line.strip().removesuffix(")")
    def map_vector(v):
        return np.array(eval(f"({expression})", {"__builtins__": {}}, {"v": v, "float": float}))

    assert "axes[j] = -_map_vector" in FLEET
    for lane, chassis in valid_variants():
        model = build_model(lane, chassis)
        for joint in model.joints:
            angle = _shader_angle(joint, .713, .24, DIG)
            point = np.array([.71, -.24, .87])
            expected = map_vector(rotation(joint.axis, angle) @ point)
            actual = rotation(-map_vector(joint.axis), angle) @ map_vector(point)
            np.testing.assert_allclose(actual, expected, atol=1e-9)
        for part in model.parts:
            a, b, c = part.vertices[part.triangles[0]]
            expected = map_vector(np.cross(b-a, c-a))
            actual = np.cross(map_vector(c)-map_vector(a), map_vector(b)-map_vector(a))
            np.testing.assert_allclose(actual, expected, atol=1e-7)


def test_fleet_tables_match_frozen_contract_and_display_palette():
    codes = dict((key, int(value)) for key, value in re.findall(
        r'"(\w+)": (\d+)', FLEET.split("const WORK_ACTIVITY :=", 1)[1].split("}", 1)[0]))
    assert codes == {key: ACTIVITY[key] for key in codes}
    assert set(codes) == {"explore", "investigate", "dig", "carry", "relay_post"}
    palette = MAIN.split("const LANE_COLORS :=", 1)[1].split("]", 1)[0]
    colors = [tuple(map(float, x.split(","))) for x in re.findall(r"Color\((.*?)\)", palette)]
    assert colors == list(LANE_RGB.values())
    assert 'if lane == 0 and chassis != 3' in FLEET
    assert 'if status >= 2:' in FLEET
    assert 'if (state == 5)' in SHADER
    assert 'state != 3' in SHADER  # Payload remains hidden unless actually carrying.


def test_live_resources_fit_uniform_arrays_and_ship_inside_godot():
    catalog = json.loads((ROOT / "godot/assets/units/fleet.json").read_text())
    assert {(m["lane"], m["chassis"]) for m in catalog["models"]} == set(valid_variants())
    for model in catalog["models"]:
        assert len(model["joints"]) <= 32
        assert len({tuple(p["color"]) for p in model["parts"]}) <= 16
    for path in re.findall(r'"res://([^\"]+)"', FLEET):
        assert (ROOT / "godot" / path).is_file()
    for source in (FLEET, SHADER):
        assert not any(line.startswith("    ") for line in source.splitlines())
    assert 'var bots: UnitFleet' in MAIN
    assert 'view_mode == View.POV and _on_unit()' in MAIN
    assert 'INSTANCE_ID == hidden_instance' in SHADER


def test_long_missions_keep_wheel_and_gait_phase_precise_in_compatibility_mode():
    assert '_travel[i] = fposmod(_travel[i] + distance, period)' in FLEET
    for chassis in ("wheeled", "tracked", "legged"):
        for joint in build_model("none", chassis).joints:
            if joint.kind not in ("wheel", "gait"):
                continue
            period = 2*np.pi*joint.amplitude if joint.kind == "wheel" else 1/joint.frequency
            for travel in (1024.173, 4096.773, -2048.36):
                packed = float(np.float16(travel % period))
                actual = _shader_angle(joint, 10.0, packed, REST)
                expected = joint_angle(joint, 10.0, travel, REST)
                np.testing.assert_allclose(rotation(joint.axis, actual),
                                           rotation(joint.axis, expected), atol=.003)


@pytest.mark.parametrize(("lane", "chassis"), valid_variants())
def test_lod_budget_preserves_role_animation_and_ground_clearance(lane, chassis):
    high = build_model(lane, chassis)
    for lod, budget in ((1, 450), (2, 120)):
        model = build_lod_model(lane, chassis, lod)
        assert model.triangle_count <= budget < high.triangle_count
        assert {j.kind for j in model.joints} == {j.kind for j in high.joints}
        for part in model.parts:
            triangles = part.vertices[part.triangles]
            assert np.all(np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                                  triangles[:, 2] - triangles[:, 0]), axis=1) > 1e-8)
        work = {"none": SCAN, "scoop": DIG, "gripper": CARRY, "antenna": RELAY}[lane]
        for state in (REST, work, DISABLED):
            for phase in np.linspace(0, 1, 17):
                for _, vertices in posed_parts(model, time=4*phase, travel=phase, state=state):
                    assert vertices[:, 2].min() >= -.03
        for joint in model.joints:
            if joint.kind in ("wheel", "gait"):
                original = next(j for j in high.joints if j.kind == joint.kind)
                assert joint.amplitude == original.amplitude
                assert joint.frequency == original.frequency
            for state in (REST, work, DISABLED):
                assert _shader_angle(joint, .71, .19, state) == pytest.approx(
                    joint_angle(joint, .71, .19, state))


def test_native_visible_batches_lod_memory_and_telemetry(tmp_path):
    native = shutil.which("godot") or "/Applications/Godot.app/Contents/MacOS/Godot"
    if not Path(native).is_file():
        pytest.skip("Native Godot is not installed; source and asset checks still run")
    result = subprocess.run([native, "--headless", "--path", str(ROOT / "godot"),
                             "--log-file", str(tmp_path / "godot.log"),
                             "--script", "res://tests/unit_fleet_check.gd"],
                            text=True, capture_output=True, timeout=45)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "FLEET_LOD_CHECK_OK" in result.stdout, result.stdout + result.stderr
