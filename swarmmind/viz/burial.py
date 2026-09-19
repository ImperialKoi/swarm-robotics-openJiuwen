"""Display-only rubble cover for casualties awaiting excavation.

The frozen truth row is [x, y, state, buried]. ``buried`` records the original
condition, so the state must also be checked: CLEARED casualties stay unburied.
These meshes never participate in navigation, perception or simulator updates.
"""

from __future__ import annotations

import json
import struct
from functools import lru_cache
from pathlib import Path
from typing import NamedTuple

import numpy as np

from .prop_stream import TRIANGLES, shaped

BODY_SCALE = 1.8
BURIED_LIFT = -.28
SURFACE_LIFT = .05
ASSETS = Path(__file__).resolve().parents[2] / "godot/assets/victims"


class BurialPart(NamedTuple):
    vertices: np.ndarray
    triangles: np.ndarray
    color: tuple[float, float, float]
    kind: str


def _mound():
    """An irregular twelve-sided pile; a buried lower skirt meets sloping ground."""
    angles = np.arange(12)*2*np.pi/12
    outline = np.column_stack([np.cos(angles), np.sin(angles)])
    outline *= (1+.035*np.sin(angles*3))[:, None]
    vertices = np.concatenate([
        np.column_stack([outline*[2.1, 1.85], np.full(12, -.60)]),
        np.column_stack([outline*[1.95, 1.7], np.full(12, .35)]),
        np.column_stack([outline*[1.5, 1.25], np.full(12, 1.06)]),
        [[0, 0, 1.18]],
    ])
    triangles = []
    for ring in (0, 12):
        for i in range(12):
            j = (i+1) % 12
            triangles.extend([[ring+i, ring+j, ring+12+i], [ring+j, ring+12+j, ring+12+i]])
    for i in range(12):
        triangles.append([24+i, 24+(i+1) % 12, 36])
    return BurialPart(vertices, np.asarray(triangles), (.43, .39, .31), "rubble_core")


def needs_excavation(state: int, buried: bool) -> bool:
    return buried and state < 2  # HIDDEN / FOUND, never CLEARED / CARRIED / RESCUED


def rubble_parts():
    """A solid rubble core over the body, collapsed slabs, masonry and a timber.

    The core encloses the scaled body bounds at BURIED_LIFT. A small sleeve at the
    edge represents the visible clue used by the existing perception raster.
    """
    parts = [
        shaped((-.18, .05, 1.14), (2.8, 1.0, .20), .21,
               (.62, .60, .53), "collapsed_slab", roll=.17, taper=.91),
        shaped((.35, -.46, .99), (1.75, .72, .23), -.32,
               (.51, .50, .45), "broken_slab", roll=-.22, taper=.85),
        shaped((-.08, .53, .90), (3.0, .17, .18), -.23,
               (.29, .22, .15), "timber", roll=.1),
        shaped((1.94, -.31, .11), (.32, .19, .17), .1,
               (.65, .24, .19), "exposed_sleeve", taper=.9),
    ]
    for i in range(9):
        angle = i * 2*np.pi/9
        parts.append(shaped((np.cos(angle)*1.85, np.sin(angle)*1.6, .20),
                            (.66+(i % 3)*.10, .57, .62), angle+.3,
                            (.48+(i % 3)*.035, .44, .36), "masonry", taper=.46, lean=.08))
    return [_mound(), *[BurialPart(p.vertices, TRIANGLES, p.color, p.kind) for p in parts]]


def exported_rubble() -> dict:
    """Baked, flat-shaded Y-up triangles; Godot loads the same original geometry."""
    vertices, colors = [], []
    light = np.array([.45, .35, .82])
    light /= np.linalg.norm(light)
    for part in rubble_parts():
        faces = part.vertices[part.triangles]
        normal = np.cross(faces[:, 1]-faces[:, 0], faces[:, 2]-faces[:, 0])
        normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-9)
        shade = .4+.6*np.clip(normal @ light, 0, 1)
        vertices.extend(faces[:, :, [0, 2, 1]].reshape(-1, 3).tolist())
        colors.extend(np.repeat(np.asarray(part.color)[None, :]*shade[:, None], 3, axis=0).tolist())
    return {"vertices": vertices, "colors": colors}


@lru_cache(maxsize=1)
def body_arrays():
    """Read the repository's small baked casualty GLB for faithful offline previews."""
    raw = (ASSETS / "CurledUpPerson.glb").read_bytes()
    size = struct.unpack_from("<I", raw, 12)[0]
    doc = json.loads(raw[20:20+size])
    binary = memoryview(raw)[28+size:]

    def accessor(index):
        item = doc["accessors"][index]
        view = doc["bufferViews"][item["bufferView"]]
        dtype = {5126: "<f4", 5123: "<u2", 5121: "u1"}[item["componentType"]]
        width = {"SCALAR": 1, "VEC3": 3, "VEC4": 4}[item["type"]]
        return np.frombuffer(binary, dtype=dtype, count=item["count"]*width,
                             offset=view.get("byteOffset", 0)+item.get("byteOffset", 0)).reshape(-1, width)

    primitive = doc["meshes"][0]["primitives"][0]
    vertices = accessor(primitive["attributes"]["POSITION"])[:, [0, 2, 1]] * BODY_SCALE
    triangles = accessor(primitive["indices"]).reshape(-1, 3)[:, [0, 2, 1]]
    colors = accessor(primitive["attributes"]["COLOR_0"])[:, :3] * [1, .55, .52]
    return vertices, triangles, colors
