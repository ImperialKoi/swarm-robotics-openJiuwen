"""Read exported GLBs independently to check geometry, pivots and clip loops."""

import json
import struct

import numpy as np
import pytest

from swarmmind.viz.unit_export import export_catalog, export_glb, export_runtime_catalog
from swarmmind.viz.units import (
    CARRY,
    DIG,
    DISABLED,
    RELAY,
    REST,
    SCAN,
    build_lod_model,
    build_model,
    joint_angle,
    valid_variants,
)


def _read_glb(path):
    raw = path.read_bytes()
    magic, version, length = struct.unpack_from("<4sII", raw)
    assert (magic, version, length) == (b"glTF", 2, len(raw))
    json_length, json_kind = struct.unpack_from("<I4s", raw, 12)
    assert json_kind == b"JSON" and json_length % 4 == 0
    document = json.loads(raw[20:20 + json_length])
    binary_start = 20 + json_length
    binary_length, binary_kind = struct.unpack_from("<I4s", raw, binary_start)
    assert binary_kind == b"BIN\x00" and binary_length % 4 == 0
    binary = raw[binary_start + 8:]
    assert len(binary) == binary_length
    assert document["buffers"] == [{"byteLength": len(binary)}]
    assert "images" not in document and "textures" not in document
    return document, binary


def _accessor(document, binary, index):
    accessor = document["accessors"][index]
    view = document["bufferViews"][accessor["bufferView"]]
    assert view["buffer"] == 0 and view["byteOffset"] % 4 == 0
    assert view["byteOffset"] + view["byteLength"] <= len(binary)
    assert accessor["componentType"] == 5126
    width = {"SCALAR": 1, "VEC3": 3, "VEC4": 4}[accessor["type"]]
    assert accessor["count"] * width * 4 == view["byteLength"]
    values = np.frombuffer(binary, dtype="<f4", count=accessor["count"] * width,
                           offset=view["byteOffset"]).reshape(-1, width)
    assert np.isfinite(values).all()
    np.testing.assert_allclose(values.min(axis=0), accessor["min"])
    np.testing.assert_allclose(values.max(axis=0), accessor["max"])
    return values


@pytest.mark.parametrize(("lane", "chassis"), valid_variants())
def test_glb_geometry_matches_model_and_has_unit_flat_normals(tmp_path, lane, chassis):
    model = build_model(lane, chassis)
    doc, binary = _read_glb(export_glb(model, tmp_path / "unit.glb"))
    assert doc["asset"]["version"] == "2.0"
    assert len(doc["meshes"]) == len(model.parts)
    assert doc["nodes"][0]["children"] == list(range(1, len(model.joints) + 1))
    for part_index, part in enumerate(model.parts):
        mesh = doc["meshes"][part_index]
        assert mesh["name"] == part.name
        primitive = mesh["primitives"][0]
        assert primitive["mode"] == 4
        attributes = primitive["attributes"]
        positions = _accessor(doc, binary, attributes["POSITION"])
        normals = _accessor(doc, binary, attributes["NORMAL"])
        colors = _accessor(doc, binary, attributes["COLOR_0"])
        assert len(positions) == len(part.triangles) * 3
        parent = doc["nodes"][part.joint + 1]
        mesh_node = next(i for i, node in enumerate(doc["nodes"])
                         if node.get("mesh") == part_index)
        assert mesh_node in parent["children"]
        restored = positions + np.asarray(parent["translation"])
        expected = part.vertices[part.triangles].reshape(-1, 3)[:, [0, 2, 1]]
        expected = expected * (1.0, 1.0, -1.0)
        np.testing.assert_allclose(restored, expected, atol=1e-6)
        faces = positions.reshape(-1, 3, 3)
        geometric = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
        geometric /= np.linalg.norm(geometric, axis=1)[:, None]
        np.testing.assert_allclose(normals, np.repeat(geometric, 3, axis=0), atol=2e-6)
        np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1.0, atol=1e-6)
        srgb = np.asarray(part.color)
        linear = np.where(srgb <= .04045, srgb / 12.92, ((srgb + .055) / 1.055) ** 2.4)
        np.testing.assert_allclose(colors, np.broadcast_to(linear, colors.shape), atol=1e-6)


@pytest.mark.parametrize(("lane", "chassis"), valid_variants())
def test_glb_clips_loop_without_root_motion_and_animate_every_joint(tmp_path, lane, chassis):
    model = build_model(lane, chassis)
    doc, binary = _read_glb(export_glb(model, tmp_path / "unit.glb"))
    work = {"none": "scan", "scoop": "dig", "gripper": "carry", "antenna": "relay"}[lane]
    assert [clip["name"] for clip in doc["animations"]] == ["idle", "move", work, "disabled"]
    for clip in doc["animations"]:
        rotations = set()
        animated = False
        for channel in clip["channels"]:
            target = channel["target"]
            assert target["node"] != 0 and target["path"] in ("rotation", "scale")
            sampler = clip["samplers"][channel["sampler"]]
            times = _accessor(doc, binary, sampler["input"]).ravel()
            poses = _accessor(doc, binary, sampler["output"])
            assert len(times) == len(poses)
            assert times[0] == 0 and np.all(np.diff(times) > 0)
            assert times[-1] == pytest.approx(clip["extras"]["loop_seconds"])
            if target["path"] == "rotation":
                rotations.add(target["node"])
                np.testing.assert_allclose(np.linalg.norm(poses, axis=1), 1.0, atol=1e-6)
                assert abs(np.dot(poses[0], poses[-1])) == pytest.approx(1.0, abs=1e-6)
                assert np.all(np.sum(poses[:-1] * poses[1:], axis=1) > 0)
                animated |= not np.allclose(poses, poses[0])
                # A non-axis-aligned probe catches sign/handedness mistakes in
                # animated axes independently of the exporter's conversion.
                joint = model.joints[target["node"] - 1]
                state = {"idle": REST, "move": REST, "scan": SCAN, "dig": DIG,
                         "carry": CARRY, "relay": RELAY, "disabled": DISABLED}[clip["name"]]
                frame = len(times) // 3
                travel = clip["extras"]["travel_metres"] * times[frame] / times[-1]
                angle = joint_angle(joint, float(times[frame]), float(travel), state)
                axis = np.asarray(joint.axis, dtype=float)
                probe = np.array((0.31, -0.17, 0.23))
                if np.linalg.norm(axis) > 0:
                    axis /= np.linalg.norm(axis)
                    rotated = (probe * np.cos(angle) + np.cross(axis, probe) * np.sin(angle)
                               + axis * np.dot(axis, probe) * (1 - np.cos(angle)))
                else:
                    rotated = probe
                expected = rotated[[0, 2, 1]] * (1, 1, -1)
                q, w = poses[frame, :3], poses[frame, 3]
                point = probe[[0, 2, 1]] * (1, 1, -1)
                actual = point + 2 * np.cross(q, np.cross(q, point) + w * point)
                np.testing.assert_allclose(actual, expected, atol=2e-5)
            else:
                assert model.joints[target["node"] - 1].kind == "payload"
                np.testing.assert_array_equal(poses, 1.0 if clip["name"] == "carry" else 0.0)
        assert rotations == set(range(1, len(model.joints) + 1))
        if clip["name"] in ("move", "carry"):
            assert animated
        if clip["name"] == "disabled":
            assert not animated


def test_catalog_covers_supported_variants_and_export_is_reproducible(tmp_path):
    metadata = export_catalog(tmp_path)
    assert {(row["lane"], row["chassis"]) for row in metadata["models"]} == set(valid_variants())
    assert ("gripper", "rotor") not in valid_variants()
    assert metadata["total_bytes"] == sum((tmp_path / row["file"]).stat().st_size
                                          for row in metadata["models"])
    assert json.loads((tmp_path / "catalog.json").read_text()) == metadata
    assert metadata["runtime_file"] == "fleet.json"
    assert metadata["runtime_bytes"] == (tmp_path / "fleet.json").stat().st_size
    lane, chassis = valid_variants()[0]
    model = build_model(lane, chassis)
    original = (tmp_path / f"{lane}_{chassis}.glb").read_bytes()
    assert export_glb(model, tmp_path / "second.glb").read_bytes() == original


def test_runtime_catalog_keeps_indexed_source_geometry_and_joint_motion(tmp_path):
    path = export_runtime_catalog(tmp_path)
    original = path.read_bytes()
    assert len(original) < 5_000_000
    fleet = json.loads(original)
    assert [(row["lane"], row["chassis"]) for row in fleet["models"]] == list(valid_variants())
    for row in fleet["models"]:
        model = build_model(row["lane"], row["chassis"])
        assert row["name"] == model.name
        assert len(row["joints"]) == len(model.joints)
        for stored, joint in zip(row["joints"], model.joints, strict=True):
            assert stored["name"] == joint.name and stored["kind"] == joint.kind
            for field in ("pivot", "axis", "amplitude", "frequency", "phase"):
                assert np.isfinite(stored[field]).all()
                np.testing.assert_array_equal(stored[field], getattr(joint, field))
        assert len(row["parts"]) == len(model.parts)
        for stored, part in zip(row["parts"], model.parts, strict=True):
            assert stored["name"] == part.name
            vertices = np.asarray(stored["vertices"])
            triangles = np.asarray(stored["triangles"])
            assert np.isfinite(vertices).all() and vertices.shape == part.vertices.shape
            np.testing.assert_allclose(vertices, part.vertices, atol=5.1e-7, rtol=0)
            np.testing.assert_array_equal(triangles, part.triangles)
            assert triangles.min() >= 0 and triangles.max() < len(vertices)
            np.testing.assert_array_equal(stored["color"], part.color)
            assert np.isfinite(stored["color"]).all() and np.isfinite(stored["accent"])
            assert stored["accent"] == part.accent
            assert stored["joint"] == part.joint and 0 <= stored["joint"] < len(row["joints"])
    assert export_runtime_catalog(tmp_path).read_bytes() == original


def test_shipped_models_match_their_source(tmp_path):
    from pathlib import Path

    shipped = Path(__file__).resolve().parents[1] / "godot" / "assets" / "units"
    metadata = export_catalog(tmp_path)
    names = ["fleet.json", "catalog.json", "fleet_index.json",
             *[row["file"] for row in metadata["models"]],
             *[f"{lane}_{chassis}_lod{lod}.json" for lane, chassis in valid_variants()
               for lod in range(3)]]
    for name in names:
        assert (shipped / name).read_bytes() == (tmp_path / name).read_bytes(), (
            f"{name} is stale; regenerate with scripts/design_units.py")


def test_lazy_manifest_is_small_and_lods_are_independent_files(tmp_path):
    export_runtime_catalog(tmp_path)
    index = tmp_path / "fleet_index.json"
    assert index.stat().st_size < 10_000
    rows = json.loads(index.read_text())["models"]
    assert len(rows) == 15
    for row in rows:
        assert "parts" not in row and "joints" not in row
        assert len(row["lods"]) == 3
        for lod, entry in enumerate(row["lods"]):
            model = build_lod_model(row["lane"], row["chassis"], lod)
            geometry = json.loads((tmp_path / entry["file"]).read_text())
            assert sum(len(p["triangles"]) for p in geometry["parts"]) == model.triangle_count
            assert entry["triangles"] == model.triangle_count
            assert len(geometry["joints"]) == len(model.joints)
            assert len({tuple(p["color"]) for p in geometry["parts"]}) <= 16
            assert len(geometry["joints"]) <= 32
