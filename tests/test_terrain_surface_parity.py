"""Numerical agreement between the offline landscape and the native dashboard."""

import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from swarmmind.viz.terrain_surface import TerrainSurface


def test_native_landscape_matches_reference(tmp_path):
    native = shutil.which("godot") or "/Applications/Godot.app/Contents/MacOS/Godot"
    if not Path(native).is_file():
        pytest.skip("Native Godot unavailable; terrain mathematics are tested separately")

    # Rock faces, a winding river, wet map edges, and a dry crossing exercise the
    # shared hash, erosion, region filtering and shoreline coordinates together.
    yy, xx = np.indices((13, 19))
    occ = ((xx + 2*yy) % 3).astype(np.uint8)
    land = 3 + .55*xx + 1.8*np.sin(xx*.7) + 1.2*np.cos(yy*.6)
    height = (land + np.where(occ == 1, 1.55, np.where(occ == 2, 1, 0))).astype(np.float32)
    water = np.where((xx > 3+yy//4) & (xx < 10+yy//4), .35 + .08*(yy % 4), 0)
    water[6] = 0
    water = water.astype(np.float32)
    cell = .7
    reference = TerrainSurface(height, occ, water, cell)
    fixture = {
        "height": height.ravel().tolist(), "occupancy": occ.ravel().tolist(),
        "water": water.ravel().tolist(), "width": 19, "depth": 13, "cell": cell,
    }
    (tmp_path / "fixture.json").write_text(json.dumps(fixture))
    (tmp_path / "project.godot").write_text("config_version=5\n")
    root = Path(__file__).resolve().parents[1]
    shutil.copyfile(root / "godot/scripts/terrain_surface.gd", tmp_path / "terrain_surface.gd")
    (tmp_path / "check.gd").write_text('''extends SceneTree
const Surface = preload("res://terrain_surface.gd")

func _initialize() -> void:
	var data: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://fixture.json"))
	var source := Surface.new()
	source.setup(PackedFloat32Array(data.height), PackedByteArray(data.occupancy),
		PackedFloat32Array(data.water), int(data.width), int(data.depth), float(data.cell))
	source.flight_height_at_world(0.0, 0.0)
	var result := {
		"flight_corners": Array(source.flight_corners),
		"corners": Array(source.corners), "water_corners": Array(source.water_corners),
		"water_depths": Array(source.water_depths), "minimum": source.minimum,
		"maximum": source.maximum, "water_xy": [], "colors": [], "normals": [],
		"land_vertices": [], "water_vertices": [],
	}
	for value in source.water_xy:
		result.water_xy.append([value.x, value.y])
	for value in source.colors:
		result.colors.append([value.r, value.g, value.b, value.a])
	for value in source.normals:
		result.normals.append([value.x, value.z, value.y])
	var land := source.build_mesh(0, 0, source.gw, source.gh, 4)
	for value in land.surface_get_arrays(0)[Mesh.ARRAY_VERTEX]:
		result.land_vertices.append([value.x, value.z, value.y])
	var river := source.build_water_mesh(0, 0, source.gw, source.gh)
	for value in river.surface_get_arrays(0)[Mesh.ARRAY_VERTEX]:
		result.water_vertices.append([value.x, value.z, value.y])
	var output := FileAccess.open("res://result.json", FileAccess.WRITE)
	output.store_string(JSON.stringify(result))
	output.close()
	print("TERRAIN_PARITY_OK")
	quit()
''')
    result = subprocess.run(
        [native, "--headless", "--path", str(tmp_path),
         "--log-file", str(tmp_path / "godot.log"), "--script", "res://check.gd"],
        text=True, capture_output=True, timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "TERRAIN_PARITY_OK" in result.stdout, result.stdout + result.stderr
    assert "SCRIPT ERROR" not in result.stderr, result.stderr
    actual = json.loads((tmp_path / "result.json").read_text())
    reference.flight_height_at_world(0.0, 0.0)
    expected = {
        "flight_corners": reference.flight_corners.ravel(),
        "corners": reference.corners.ravel(),
        "water_corners": reference.water_corners.ravel(),
        "water_depths": reference.water_depths.ravel(),
        "water_xy": reference.water_xy.reshape(-1, 2),
        "colors": reference.colors.reshape(-1, 4),
        "normals": reference.normals.reshape(-1, 3),
        "minimum": reference.minimum, "maximum": reference.maximum,
        "land_vertices": reference.tile_arrays(0, 0, 19, 13, 4).vertices,
        "water_vertices": reference.water_arrays(0, 0, 19, 13).vertices,
    }
    for name, values in expected.items():
        # Native packed vectors/colors are float32; the Python reference is float64.
        np.testing.assert_allclose(actual[name], values, rtol=0, atol=3e-6, err_msg=name)
