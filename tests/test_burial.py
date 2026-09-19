"""Burial is visible geometry tied to real excavation, with no early truth reveal."""

import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from swarmmind.sim.robot import LANE_INDEX
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import CARRIED, CLEARED, FOUND, World
from swarmmind.viz.burial import (
    ASSETS,
    BURIED_LIFT,
    body_arrays,
    exported_rubble,
    needs_excavation,
    rubble_parts,
)


def test_buried_casualty_cannot_be_carried_until_real_excavation_completes():
    world = World(Scenario.load("test"), 42)
    victim = next(v for v in world.victims if v.buried)
    world.victims = [victim]
    victim.state = FOUND
    carrier = int(np.flatnonzero(world.actuator == LANE_INDEX["gripper"])[0])
    digger = int(np.flatnonzero(world.actuator == LANE_INDEX["scoop"])[0])
    world.pos[:] = -1000  # isolate the victim state machine from incidental nearby units
    world.pos[carrier] = victim.pos
    for _ in range(100):
        world._update_victims()
    assert victim.state == FOUND and victim.debris_remaining == 1
    assert world.carrying[carrier] == -1
    world.pos[digger] = victim.pos + [.5, 0]
    steps = int(np.ceil(1/(world.scn.victims.clear_rate*world.dt)))+2
    for _ in range(steps):
        world._update_victims()
        if victim.state == CLEARED:
            break
        assert world.carrying[carrier] == -1
    assert victim.state == CLEARED and victim.debris_remaining == 0
    assert victim.buried  # original condition survives clearing; it is not the live state
    assert not needs_excavation(victim.state, victim.buried)
    world._update_victims()
    assert victim.state == CARRIED and world.carrying[carrier] == 0


def test_rubble_core_physically_encloses_the_buried_body():
    core = rubble_parts()[0]
    faces = core.vertices[core.triangles]
    normals = np.cross(faces[:, 1]-faces[:, 0], faces[:, 2]-faces[:, 0])
    vertices = body_arrays()[0] + [0, 0, BURIED_LIFT]
    # Every body vertex lies behind every outer face, so depth-tested geometry
    # cannot show an exposed torso through this solid rubble cover.
    distances = np.einsum("fvc,fc->fv", vertices[None, :, :]-faces[:, :1, :], normals)
    assert distances.max() <= 1e-6
    assert vertices[:, 2].min() > core.vertices[:, 2].min()


def test_burial_export_matches_the_inspected_reference_geometry():
    exported = json.loads((ASSETS / "burial.json").read_text(encoding="utf-8"))
    assert exported == exported_rubble()
    assert len(exported["vertices"]) == len(exported["colors"])
    assert len(exported["vertices"]) // 3 < 300


def test_native_burial_state_and_visibility(tmp_path):
    native = shutil.which("godot") or "/Applications/Godot.app/Contents/MacOS/Godot"
    if not Path(native).is_file():
        pytest.skip("Native Godot unavailable; source geometry and rescue chain tested separately")
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [native, "--headless", "--path", str(root / "godot"),
         "--log-file", str(tmp_path / "godot.log"), "--script", "res://tests/burial_check.gd"],
        text=True, capture_output=True, timeout=30,
    )
    assert result.returncode == 0, result.stdout+result.stderr
    assert "BURIAL_CHECK_OK" in result.stdout, result.stdout+result.stderr
    assert "SCRIPT ERROR" not in result.stderr, result.stderr
