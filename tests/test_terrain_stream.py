"""Exercise the actual Godot streaming implementation when the engine is installed."""

import shutil
import subprocess
from pathlib import Path

import pytest


def test_native_terrain_stream_visibility_detail_and_ground_contact(tmp_path):
    native = shutil.which("godot") or "/Applications/Godot.app/Contents/MacOS/Godot"
    if not Path(native).is_file():
        pytest.skip("Native Godot unavailable; terrain mathematics are tested separately")
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [native, "--headless", "--path", str(root / "godot"),
         "--log-file", str(tmp_path / "godot.log"),
         "--script", "res://tests/terrain_stream_check.gd"],
        text=True, capture_output=True, timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "TERRAIN_STREAM_CHECK_OK" in result.stdout, result.stdout + result.stderr
