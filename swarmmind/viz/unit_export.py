"""Dependency-free glTF 2.0 export of the display-only modular robot fleet.

The model of record uses +Z up and +X forward; GLB positions, normals, pivots,
and rotation axes use the proper rotation (x, z, -y), giving +Y up. Each mesh
keeps its part name and each moving assembly gets a separate rigid joint node.
This is ordinary glTF node animation, so it needs neither a skin nor a plugin.

Format reference: https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html
"""

from __future__ import annotations

import json
import math
import struct
from dataclasses import asdict
from fractions import Fraction
from pathlib import Path
from typing import Any

import numpy as np

from swarmmind.viz.units import (
    CARRY,
    DIG,
    DISABLED,
    RELAY,
    REST,
    SCAN,
    UnitModel,
    build_lod_model,
    build_model,
    joint_angle,
    joint_visible,
    valid_variants,
)

_WORK = {"none": ("scan", SCAN), "scoop": ("dig", DIG),
         "gripper": ("carry", CARRY), "antenna": ("relay", RELAY)}
_TIME_STATE = {"scan": SCAN, "dig": DIG, "relay": RELAY}


def _y_up(values: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    return values[..., [0, 2, 1]] * (1.0, 1.0, -1.0)


def _linear_color(color: tuple[float, float, float]) -> np.ndarray:
    srgb = np.asarray(color, dtype=np.float64)
    return np.where(srgb <= 0.04045, srgb / 12.92, ((srgb + 0.055) / 1.055) ** 2.4)


def _rotation(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    direction = _y_up(axis)
    length = np.linalg.norm(direction)
    if length == 0:
        if angle != 0:
            raise ValueError("An animated joint must have a nonzero rotation axis")
        return np.array((0.0, 0.0, 0.0, 1.0))
    return np.append(direction / length * math.sin(angle / 2), math.cos(angle / 2))


class _Buffer:
    """One aligned binary buffer, with a dedicated view for each accessor."""

    def __init__(self) -> None:
        self.data = bytearray()
        self.views: list[dict[str, Any]] = []
        self.accessors: list[dict[str, Any]] = []

    def add(self, values: np.ndarray, kind: str, *, vertex: bool = False) -> int:
        values = np.ascontiguousarray(values, dtype="<f4")
        if not np.isfinite(values).all() or not len(values):
            raise ValueError("glTF accessors require finite, nonempty data")
        self.data.extend(b"\x00" * (-len(self.data) % 4))
        view: dict[str, Any] = {
            "buffer": 0,
            "byteOffset": len(self.data),
            "byteLength": values.nbytes,
        }
        if vertex:
            view["target"] = 34962  # ARRAY_BUFFER
        index = len(self.accessors)
        flat = values.reshape(len(values), -1)
        self.accessors.append({
            "bufferView": len(self.views),
            "componentType": 5126,  # FLOAT
            "count": len(values),
            "type": kind,
            "min": flat.min(axis=0).tolist(),
            "max": flat.max(axis=0).tolist(),
        })
        self.views.append(view)
        self.data.extend(values.tobytes())
        return index


def _timing(model: UnitModel, state: int, moving: bool) -> tuple[float, float, int]:
    """Choose a shared time period and complete wheel/stride cycles.

    Time frequencies are rational cycles/s. Deriving their common period keeps
    the first and last poses identical without inserting a visible reset frame.
    Locomotion is driven by travelled metres, independently of the clip clock.
    """
    frequencies = [
        Fraction(str(joint.frequency)).limit_denominator(1000)
        for joint in model.joints
        if state != DISABLED and joint.frequency > 0
        and (joint.kind == "rotor" or _TIME_STATE.get(joint.kind) == state)
    ]
    duration = 2.0
    if frequencies:
        denominator = math.lcm(*(f.denominator for f in frequencies))
        numerator = math.gcd(*(f.numerator * (denominator // f.denominator)
                               for f in frequencies))
        period = denominator / numerator
        duration = period * max(1, math.ceil(2.0 / period))

    travel = 0.0
    if moving:
        radii = [abs(j.amplitude) for j in model.joints if j.kind == "wheel"]
        strides = [j.frequency for j in model.joints if j.kind == "gait"]
        if radii and strides:
            raise ValueError("A chassis cannot combine wheel and gait travel cycles")
        if radii:
            if min(radii) <= 0:
                raise ValueError("Wheel joints require a positive radius")
            ratios = [Fraction(str(r / radii[0])).limit_denominator(1000) for r in radii]
            travel = math.tau * radii[0] * math.lcm(*(r.numerator for r in ratios))
        elif strides:
            fractions = [Fraction(str(f)).limit_denominator(1000) for f in strides]
            denominator = math.lcm(*(f.denominator for f in fractions))
            numerator = math.gcd(*(f.numerator * (denominator // f.denominator)
                                   for f in fractions))
            travel = 2.0 * denominator / numerator

    # At least 16 samples/revolution prevents rotor aliasing or shortest-path
    # quaternion interpolation from making the propellers turn backwards.
    max_hz = max((float(f) for f in frequencies), default=0.0)
    samples = max(2, math.ceil(duration * max(24.0, max_hz * 16.0))) + 1
    return duration, travel, samples


def _animations(model: UnitModel, buffer: _Buffer) -> list[dict[str, Any]]:
    work_name, work_state = _WORK[model.lane]
    specs = (("idle", REST, False), ("move", REST, True),
             (work_name, work_state, work_state == CARRY), ("disabled", DISABLED, False))
    animations = []
    for name, state, moving in specs:
        duration, travel, count = _timing(model, state, moving)
        times = np.linspace(0.0, duration, count)
        time_index = buffer.add(times, "SCALAR")
        endpoints = buffer.add(np.array((0.0, duration)), "SCALAR")
        samplers, channels = [], []
        for index, joint in enumerate(model.joints):
            rotations = np.array([
                _rotation(joint.axis, joint_angle(joint, float(t), travel * t / duration, state))
                for t in times
            ])
            # q and -q represent the same orientation; keep successive keys in
            # the same hemisphere for importers which linearly blend rotations.
            for frame in range(1, count):
                if np.dot(rotations[frame - 1], rotations[frame]) < 0:
                    rotations[frame] *= -1
            stationary = np.allclose(rotations, rotations[0], atol=1e-8)
            output = buffer.add(rotations[[0, -1]] if stationary else rotations, "VEC4")
            samplers.append({"input": endpoints if stationary else time_index,
                             "output": output, "interpolation": "LINEAR"})
            channels.append({"sampler": len(samplers) - 1,
                             "target": {"node": index + 1, "path": "rotation"}})
            # glTF has no core visibility track. Zero scale hides the attached
            # casualty in other states; keying it in every clip prevents stale
            # payload visibility when switching away from carry.
            if joint.kind == "payload":
                scale = float(joint_visible(joint, state))
                scale_index = buffer.add(np.full((2, 3), scale), "VEC3")
                samplers.append({"input": endpoints, "output": scale_index,
                                 "interpolation": "STEP"})
                channels.append({"sampler": len(samplers) - 1,
                                 "target": {"node": index + 1, "path": "scale"}})
        animations.append({
            "name": name,
            "channels": channels,
            "samplers": samplers,
            "extras": {"loop": True, "loop_seconds": duration,
                       "in_place": True, "travel_metres": travel,
                       "description": "Display animation; does not change simulation physics."},
        })
    return animations


def export_glb(model: UnitModel, path: str | Path) -> Path:
    """Write an editable, self-contained animated GLB; return its path."""
    buffer = _Buffer()
    nodes: list[dict[str, Any]] = [{
        "name": model.name,
        "children": list(range(1, len(model.joints) + 1)),
        "extras": {"lane": model.lane, "chassis": model.chassis,
                   "forward": "+X", "up": "+Y", "display_only": True},
    }]
    for joint in model.joints:
        node: dict[str, Any] = {
            "name": joint.name,
            "translation": _y_up(joint.pivot).tolist(),
            "rotation": _rotation(joint.axis, joint_angle(joint, 0.0, 0.0, REST)).tolist(),
            "children": [],
            "extras": {"motion": joint.kind},
        }
        if not joint_visible(joint, REST):
            node["scale"] = [0.0, 0.0, 0.0]
        nodes.append(node)

    meshes = []
    for part in model.parts:
        joint = model.joints[part.joint]
        # Duplicate triangle corners so each face has one normal: a deliberate
        # low-poly surface, with no smoothing across panel or tread edges.
        positions = _y_up(part.vertices[part.triangles] - np.asarray(joint.pivot))
        normals = np.cross(positions[:, 1] - positions[:, 0],
                           positions[:, 2] - positions[:, 0])
        lengths = np.linalg.norm(normals, axis=1)
        if np.any(lengths < 1e-12):
            raise ValueError(f"Degenerate triangle in {model.name}/{part.name}")
        normals = np.repeat(normals / lengths[:, None], 3, axis=0)
        positions = positions.reshape(-1, 3)
        colors = np.tile(_linear_color(part.color), (len(positions), 1))
        attributes = {
            "POSITION": buffer.add(positions, "VEC3", vertex=True),
            "NORMAL": buffer.add(normals, "VEC3", vertex=True),
            "COLOR_0": buffer.add(colors, "VEC3", vertex=True),
        }
        mesh_index = len(meshes)
        meshes.append({"name": part.name,
                       "primitives": [{"attributes": attributes, "material": 0, "mode": 4}],
                       "extras": {"lane_accent": float(part.accent)}})
        nodes[part.joint + 1]["children"].append(len(nodes))
        nodes.append({"name": part.name, "mesh": mesh_index})

    animations = _animations(model, buffer)
    document = {
        "asset": {"version": "2.0", "generator": "SwarmMind modular unit exporter"},
        "scene": 0,
        "scenes": [{"name": model.name, "nodes": [0]}],
        "nodes": nodes,
        "meshes": meshes,
        "materials": [{"name": "Robot painted alloy",
                       "pbrMetallicRoughness": {"baseColorFactor": [1, 1, 1, 1],
                                                "metallicFactor": 0.15,
                                                "roughnessFactor": 0.72}}],
        "animations": animations,
        "accessors": buffer.accessors,
        "bufferViews": buffer.views,
        "buffers": [{"byteLength": len(buffer.data)}],
    }
    encoded = json.dumps(document, separators=(",", ":"), allow_nan=False).encode("utf-8")
    encoded += b" " * (-len(encoded) % 4)
    binary = bytes(buffer.data)
    binary += b"\x00" * (-len(binary) % 4)
    glb = (struct.pack("<4sII", b"glTF", 2, 12 + 8 + len(encoded) + 8 + len(binary))
           + struct.pack("<I4s", len(encoded), b"JSON") + encoded
           + struct.pack("<I4s", len(binary), b"BIN\x00") + binary)
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(glb)
    return destination


def _runtime_model(model: UnitModel) -> dict[str, Any]:
    return {
        "name": model.name, "lane": model.lane, "chassis": model.chassis,
        "joints": [asdict(joint) for joint in model.joints],
        "parts": [{
            "name": part.name,
            "vertices": np.round(np.asarray(part.vertices, dtype=np.float64), 6).tolist(),
            "triangles": part.triangles.tolist(),
            "color": list(part.color),
            "accent": float(part.accent),
            "joint": part.joint,
        } for part in model.parts],
    }


def export_runtime_catalog(directory: str | Path) -> Path:
    """Write indexed geometry and joint parameters for the batched dashboard.

    Unlike the GLBs, this keeps the source convention (+Z up, +X forward,
    counter-clockwise triangle winding). The dashboard performs its own axis
    conversion while packing the shared animated mesh. Positions are rounded
    to a micrometre to keep JSON compact without float32 expansion artefacts.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    models, index = [], []
    for lane, chassis in valid_variants():
        model = build_model(lane, chassis)
        models.append(_runtime_model(model))
        lods = []
        for lod in range(3):
            silhouette = build_lod_model(lane, chassis, lod)
            filename = f"{lane}_{chassis}_lod{lod}.json"
            (directory / filename).write_text(json.dumps(_runtime_model(silhouette),
                separators=(",", ":"), allow_nan=False) + "\n", encoding="utf-8")
            lods.append({"file": filename, "triangles": silhouette.triangle_count})
        period = next((math.tau*j.amplitude if j.kind == "wheel" else 1/j.frequency
                       for j in model.joints if j.kind in ("wheel", "gait")), 1.0)
        index.append({"name": model.name, "lane": lane, "chassis": chassis,
                      "travel_period": period, "radius": 3.0, "lods": lods})
    # Startup reads only this ~7 KB index; indexed source/rigs are loaded on demand
    # per visible variant and released along with the idle GPU batches.
    (directory / "fleet_index.json").write_text(json.dumps({"models": index},
        separators=(",", ":"), allow_nan=False) + "\n", encoding="utf-8")
    destination = directory / "fleet.json"
    destination.write_text(json.dumps({"models": models}, separators=(",", ":"),
                                      allow_nan=False) + "\n", encoding="utf-8")
    return destination


def export_catalog(directory: str | Path) -> dict[str, Any]:
    """Export all animated GLBs, the batched runtime fleet and a JSON manifest."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    records = []
    for lane, chassis in valid_variants():
        model = build_model(lane, chassis)
        path = export_glb(model, directory / f"{lane}_{chassis}.glb")
        records.append({
            "name": model.name, "lane": lane, "chassis": chassis, "file": path.name,
            "bytes": path.stat().st_size,
            "triangles": sum(len(part.triangles) for part in model.parts),
            "parts": len(model.parts), "joints": len(model.joints),
            "animations": ["idle", "move", _WORK[lane][0], "disabled"],
        })
    runtime_path = export_runtime_catalog(directory)
    metadata = {
        "format": "glTF 2.0 binary", "coordinates": "+Y up, +X forward",
        "display_only": True, "models": records,
        "total_bytes": sum(row["bytes"] for row in records),
        "total_triangles": sum(row["triangles"] for row in records),
        "runtime_file": runtime_path.name, "runtime_bytes": runtime_path.stat().st_size,
    }
    (directory / "catalog.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return metadata
