"""Original articulated rescue fleet; display geometry, never simulation physics.

Metres, Z up, +X forward. Parts contain model-space vertices and one rigid joint.
The offline renderer, GLB exporter and batched Godot mesh use this same source.
Travel is measured distance, so a parked wheel or leg cannot keep walking.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from swarmmind.contracts.schemas import ACTIVITY
from swarmmind.sim.robot import CHASSIS, CHASSIS_BARRED, LANES

REST, SCAN, DIG, CARRY, RELAY, DISABLED = range(6)
LANE_RGB = {
    "none": (0.35, 0.78, 1.0),
    "scoop": (1.0, 0.75, 0.24),
    "gripper": (0.47, 1.0, 0.55),
    "antenna": (0.84, 0.51, 1.0),
}
ROLE_NAMES = {"none": "Scout", "scoop": "Digger", "gripper": "Carrier", "antenna": "Relay"}
INK = (0.075, 0.105, 0.13)
RUBBER = (0.105, 0.13, 0.145)
STEEL = (0.31, 0.39, 0.43)
SHELL = (0.78, 0.83, 0.82)
LENS = (0.15, 0.83, 0.94)


@dataclass(frozen=True)
class Joint:
    name: str
    pivot: tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    kind: str = "static"
    amplitude: float = 0.0
    frequency: float = 0.25
    phase: float = 0.0


@dataclass(frozen=True)
class Part:
    name: str
    vertices: np.ndarray
    triangles: np.ndarray
    color: tuple[float, float, float]
    accent: float = 0.0
    joint: int = 0


@dataclass(frozen=True)
class UnitModel:
    name: str
    lane: str
    chassis: str
    parts: tuple[Part, ...]
    joints: tuple[Joint, ...]

    @property
    def triangle_count(self) -> int:
        return sum(len(p.triangles) for p in self.parts)


def valid_variants() -> tuple[tuple[str, str], ...]:
    return tuple((lane, chassis) for lane in LANES for chassis in CHASSIS
                 if chassis not in CHASSIS_BARRED.get(lane, ()))


def joint_angle(joint: Joint, time: float, travel: float, state: int) -> float:
    """The animation specification, ported to the dashboard's vertex shader."""
    if state == DISABLED:
        return -joint.amplitude if joint.kind == "dig" else 0.0
    wave = np.sin(2.0 * np.pi * joint.frequency * time + joint.phase)
    if joint.kind == "wheel":
        return travel / joint.amplitude
    if joint.kind == "gait":
        return joint.amplitude * np.sin(2.0 * np.pi * joint.frequency * travel + joint.phase)
    if joint.kind == "rotor":
        return 2.0 * np.pi * joint.frequency * time + joint.phase
    if joint.kind == "scan":
        return joint.amplitude * wave if state == SCAN else 0.0
    if joint.kind == "dig":
        return -joint.amplitude * (0.5 + 0.5 * wave) if state == DIG else -joint.amplitude
    if joint.kind == "grip":
        return joint.amplitude if state == CARRY else 0.0
    if joint.kind == "relay":
        return joint.amplitude * wave if state == RELAY else 0.0
    return 0.0


def joint_visible(joint: Joint, state: int) -> bool:
    return joint.kind != "payload" or state == CARRY


def rotation(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def posed_parts(model: UnitModel, *, time: float = 0.0, travel: float = 0.0,
                state: int = REST):
    for part in model.parts:
        joint = model.joints[part.joint]
        if not joint_visible(joint, state):
            continue
        pivot = np.asarray(joint.pivot)
        rot = rotation(joint.axis, joint_angle(joint, time, travel, state))
        yield part, (part.vertices - pivot) @ rot.T + pivot


def animation_state(lane: int, chassis: int, activity: int, status: int,
                    position, digs=()) -> int:
    """Only animate work supported by existing dashboard telemetry.

    Dig assignments include transit. Excavation sites, already sent for dust, are the
    evidence that a bucket is working. Rotor flight is absent from the frozen row;
    keep those camera heads fixed instead of inventing airborne detection.
    """
    if status >= 2:
        return DISABLED
    if (lane == 1 and activity == ACTIVITY["dig"]
            and any((position[0] - d[0]) ** 2 + (position[1] - d[1]) ** 2 <= 2.5 ** 2
                    for d in digs)):
        return DIG
    if lane == 2 and activity == ACTIVITY["carry"]:
        return CARRY
    if lane == 3 and activity == ACTIVITY["relay_post"]:
        return RELAY
    if lane == 0 and chassis != 3 and activity in (ACTIVITY["explore"], ACTIVITY["investigate"]):
        return SCAN
    return REST


class _Builder:
    def __init__(self, lane, chassis):
        self.lane, self.chassis = lane, chassis
        self.parts: list[Part] = []
        self.joints = [Joint("chassis")]
        self.paint = LANE_RGB[lane]

    def joint(self, name, pivot, axis, kind, amplitude=0.0, frequency=0.25, phase=0.0):
        self.joints.append(Joint(name, tuple(pivot), tuple(axis), kind,
                                 amplitude, frequency, phase))
        return len(self.joints) - 1

    def mesh(self, name, vertices, faces, color, joint=0, accent=0.0):
        v = np.asarray(vertices, dtype=np.float32)
        f = np.asarray(faces, dtype=np.int32)
        v.setflags(write=False)
        f.setflags(write=False)
        self.parts.append(Part(name, v, f, color, accent, joint))

    def box(self, name, centre, size, color, joint=0, bevel=0.0, accent=0.0):
        """A chamfered eight-sided enclosure with bevelled top and bottom edges."""
        x, y, z = np.asarray(size) * 0.5
        cut = min(x, y) * 0.24
        ring = np.array([[-x + cut, -y], [x - cut, -y], [x, -y + cut],
                         [x, y - cut], [x - cut, y], [-x + cut, y],
                         [-x, y - cut], [-x, -y + cut]])
        inset = min(bevel, z * 0.6, x * 0.2, y * 0.2)
        vertices = []
        for height, scale in ((-z, 1.0 - inset / min(x, y)),
                              (-z + inset, 1.0), (z - inset, 1.0),
                              (z, 1.0 - inset / min(x, y))):
            vertices.extend([[px * scale, py * scale, height] for px, py in ring])
        faces = []
        # With zero bevel use only two rings, avoiding zero-area geometry.
        if inset == 0.0:
            vertices = vertices[:8] + vertices[24:]
        rings = len(vertices) // 8
        for level in range(rings - 1):
            for k in range(8):
                a, b = level * 8 + k, level * 8 + (k + 1) % 8
                faces.extend([(a, b, b + 8), (a, b + 8, a + 8)])
        for k in range(1, 7):
            faces.append((0, k + 1, k))
            top = (rings - 1) * 8
            faces.append((top, top + k, top + k + 1))
        self.mesh(name, np.asarray(vertices) + centre, faces, color, joint, accent)

    def cylinder(self, name, centre, radius, length, color, joint=0,
                 axis=(0, 0, 1), segments=12, accent=0.0):
        axis = np.asarray(axis, dtype=float)
        axis /= np.linalg.norm(axis)
        helper = np.array([1, 0, 0]) if abs(axis[0]) < 0.9 else np.array([0, 1, 0])
        u = np.cross(helper, axis)
        u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        a = np.arange(segments) * (2 * np.pi / segments)
        ring = radius * (np.cos(a)[:, None] * u + np.sin(a)[:, None] * v)
        vertices = np.concatenate([ring - axis * length / 2, ring + axis * length / 2])
        faces = []
        for k in range(segments):
            nxt = (k + 1) % segments
            faces.extend([(k, nxt, nxt + segments), (k, nxt + segments, k + segments)])
        for k in range(1, segments - 1):
            faces.extend([(0, k + 1, k), (segments, segments + k, segments + k + 1)])
        self.mesh(name, vertices + centre, faces, color, joint, accent)

    def beam(self, name, start, end, width, color, joint=0):
        a, b = np.asarray(start), np.asarray(end)
        self.cylinder(name, (a + b) / 2, width / 2, np.linalg.norm(b - a), color,
                      joint, axis=b - a, segments=6, accent=float(color == self.paint))

    def ring(self, name, centre, outer, inner, depth, color, joint=0):
        n = 16
        a = np.arange(n) * (2 * np.pi / n)
        vertices = [[r * np.cos(t), r * np.sin(t), z] for z, r in
                    ((-depth/2, outer), (depth/2, outer),
                     (-depth/2, inner), (depth/2, inner)) for t in a]
        faces = []
        for k in range(n):
            q = (k + 1) % n
            for a0, b0, c0, d0 in ((k, q, n+q, n+k),
                                   (2*n+q, 2*n+k, 3*n+k, 3*n+q),
                                   (n+k, n+q, 3*n+q, 3*n+k),
                                   (q, k, 2*n+k, 2*n+q)):
                faces.extend([(a0, b0, c0), (a0, c0, d0)])
        self.mesh(name, np.asarray(vertices) + centre, faces, color, joint)

    def wheel(self, x, y, *, radius=0.25, track=False):
        centre = (x, y, 0.28)
        joint = self.joint(f"{'sprocket' if track else 'wheel'}_{x}_{y}", centre,
                           (0, 1, 0), "wheel", radius)
        self.cylinder("rubber tyre", centre, radius, 0.19, RUBBER, joint, axis=(0, 1, 0))
        side = 1 if y > 0 else -1
        hub = (x, y + side * 0.102, 0.28)
        self.cylinder("alloy rim", hub, radius * 0.66, 0.02, STEEL, joint, axis=(0, 1, 0))
        self.cylinder("drive hub", (x, y + side * 0.12, 0.28), radius * 0.29, 0.025,
                      self.paint, joint, axis=(0, 1, 0), segments=8, accent=1)
        # Radial bars make rotation readable, even on a monochrome tyre.
        for k in range(6):
            a = k * np.pi / 3
            self.beam("spoke", (x + np.cos(a)*radius*.32, hub[1] + side*.016,
                                .28 + np.sin(a)*radius*.32),
                      (x + np.cos(a)*radius*.59, hub[1] + side*.016,
                       .28 + np.sin(a)*radius*.59), .035, SHELL, joint)

    def chassis_geometry(self):
        if self.chassis == "wheeled":
            for x in (-0.44, 0.44):
                for y in (-0.54, 0.54):
                    self.wheel(x, y)
            self.box("suspension tray", (0, 0, .36), (1.10, .92, .15), INK, bevel=.03)
        elif self.chassis == "tracked":
            for y in (-.54, .54):
                self.box("continuous track belt", (0, y, .28), (1.55, .25, .54),
                         RUBBER, bevel=.06)
                for x in (-.50, 0.0, .50):
                    self.wheel(x, y + np.sign(y)*.075, track=True)
                for x in np.linspace(-.58, .58, 9):
                    for z in (.035, .525):
                        self.box("tread pad", (x, y, z), (.07, .28, .035), STEEL)
                self.box("track fender", (0, y, .57), (1.5, .31, .085),
                         self.paint, bevel=.015, accent=1)
        elif self.chassis == "legged":
            for x in (-.43, .43):
                for side in (-1, 1):
                    hip = (x, side*.36, .64)
                    knee = (x + np.sign(x)*.15, side*.65, .35)
                    foot = (x, side*.77, .085)
                    phase = 0.0 if x * side > 0 else np.pi
                    j = self.joint(f"leg_{x}_{side}", hip, (0, 1, 0), "gait", .30, 1, phase)
                    self.cylinder("hip servo", hip, .105, .14, STEEL, j, axis=(0, 1, 0))
                    self.beam("upper leg", hip, knee, .16, self.paint, j)
                    self.cylinder("knee servo", knee, .085, .10, INK, j, axis=(0, 1, 0))
                    self.beam("shin", knee, foot, .11, SHELL, j)
                    self.box("terrain foot", foot, (.25, .22, .09), RUBBER, j, bevel=.02)
        else:
            for x in (-.66, .66):
                for y in (-.67, .67):
                    self.beam("rotor outrigger", (x*.4, y*.4, .66), (x, y, .94), .12, STEEL)
                    self.ring("rotor safety guard", (x, y, .96), .37, .32, .075, INK)
                    j = self.joint(f"rotor_{x}_{y}", (x, y, .99), (0, 0, 1), "rotor",
                                   frequency=4, phase=0.0 if x*y > 0 else np.pi/4)
                    self.box("rotor blade A", (x, y, .99), (.60, .045, .017), SHELL, j)
                    self.box("rotor blade B", (x, y, .99), (.045, .60, .017), SHELL, j)
                    self.cylinder("motor cap", (x, y, 1.02), .07, .08, self.paint,
                                  segments=8, accent=1)
            for y in (-.34, .34):
                for x in (-.4, .4):
                    self.beam("landing strut", (x, y, .50), (x, y*1.3, .13), .055, STEEL)
                self.box("landing skid", (0, y*1.3, .10), (1.15, .08, .08), INK, bevel=.02)

    def body(self):
        self.box("lower hull", (0, 0, .54), (1.23, .83, .26), INK, bevel=.05)
        self.box("ceramic upper armour", (0, 0, .76), (1.24, .88, .26), SHELL, bevel=.06)
        for y in (-.425, .425):
            self.box("lane shoulder", (0, y, .78), (1.00, .065, .17),
                     self.paint, bevel=.015, accent=1)
            for x in (-.32, -.19, -.06):
                self.box("cooling slot", (x, y*1.085, .78), (.065, .017, .075), INK)
        self.box("spine stripe", (-.12, 0, .902), (.72, .19, .026), self.paint, accent=1)
        self.box("rear battery", (-.48, 0, .98), (.26, .58, .16), INK, bevel=.035)
        for y in (-.28, .28):
            self.box("front running light", (.624, y, .76), (.02, .12, .075), LENS)
        self.box("front crash bar", (.62, 0, .52), (.11, .75, .11), STEEL, bevel=.02)

    def scout(self):
        j = self.joint("camera gimbal", (.19, 0, .99), (0, 0, 1), "scan", .30)
        self.cylinder("gimbal pedestal", (.19, 0, .95), .17, .12, STEEL)
        self.box("stereo camera housing", (.22, 0, 1.13), (.40, .61, .25), INK, j, bevel=.045)
        self.box("camera brow", (.21, 0, 1.265), (.43, .66, .055), self.paint, j, accent=1)
        for y in (-.18, .18):
            self.cylinder("lens bezel", (.43, y, 1.13), .093, .035, STEEL, j, axis=(1, 0, 0))
            self.cylinder("camera glass", (.455, y, 1.13), .070, .014, LENS, j, axis=(1, 0, 0))
            self.cylinder("lens aperture", (.465, y, 1.13), .030, .008, INK, j, axis=(1, 0, 0))
        self.beam("telemetry whip", (-.49, -.25, 1.06), (-.53, -.25, 1.43), .025, STEEL)

    def digger(self):
        j = self.joint("bucket linkage", (.30, 0, .90), (0, 1, 0), "dig", .38, .5)
        for y in (-.24, .24):
            self.beam("reinforced lift arm", (.30, y, .90), (.90, y, .68), .14, self.paint, j)
            self.beam("bucket arm", (.90, y, .68), (1.19, y, .31), .12, self.paint, j)
            self.beam("hydraulic sleeve", (.39, y*.75, .91), (.71, y*.75, .72), .09, INK, j)
            self.beam("hydraulic piston", (.71, y*.75, .72), (.95, y*.75, .55), .045, SHELL, j)
            self.cylinder("bucket pivot", (1.17, y, .34), .075, .09, STEEL, j, axis=(0, 1, 0))
        self.box("bucket floor", (1.38, 0, .16), (.57, .87, .09), STEEL, j, bevel=.02)
        self.box("bucket back", (1.13, 0, .30), (.08, .87, .31), self.paint, j, accent=1)
        for y in (-.43, .43):
            self.box("bucket cheek", (1.36, y, .29), (.51, .065, .29), self.paint, j, accent=1)
        for y in np.linspace(-.34, .34, 5):
            self.box("replaceable tooth", (1.70, y, .14), (.19, .085, .08), SHELL, j, bevel=.01)
        self.box("tool controller", (-.1, 0, 1.0), (.38, .46, .19), self.paint, bevel=.04, accent=1)

    def carrier(self):
        self.box("rescue deck", (0, 0, .98), (1.31, .76, .12), STEEL, bevel=.025)
        self.box("stretcher cushion", (.02, 0, 1.065), (1.12, .52, .085), INK, bevel=.03)
        for side in (-1, 1):
            y = side*.45
            self.beam("stretcher rail", (-.61, y, 1.05), (.65, y, 1.05), .065, SHELL)
            j = self.joint(f"padded gripper {side}", (.22, y, 1.03), (0, 0, 1),
                           "grip", -side*.35)
            self.box("gripper knuckle", (.27, y, 1.13), (.19, .16, .19), self.paint, j, accent=1)
            self.beam("lifting jaw", (.28, y, 1.12), (.78, side*.55, 1.12), .075, STEEL, j)
            self.box("soft jaw pad", (.78, side*.48, 1.12), (.12, .22, .14), INK, j, bevel=.02)
        self.box("rescue stripe", (-.54, 0, 1.115), (.08, .49, .018), self.paint, accent=1)
        j = self.joint("secured casualty", (0, 0, 0), (0, 0, 1), "payload")
        self.box("rescue blanket", (-.03, 0, 1.21), (.75, .43, .23), (0.89, .79, .53), j, bevel=.065)
        self.box("blanket foot pocket", (.44, 0, 1.17), (.26, .32, .17), SHELL, j, bevel=.045)
        self.box("head support", (-.51, 0, 1.21), (.22, .27, .20), SHELL, j, bevel=.05)
        for x in (-.22, .20):
            self.box("restraint strap", (x, 0, 1.334), (.065, .45, .018), self.paint, j, accent=1)

    def relay(self):
        self.box("radio equipment rack", (-.15, 0, 1.04), (.48, .62, .27), INK, bevel=.035)
        for y in (-.32, .32):
            self.box("radio panel", (-.15, y, 1.04), (.36, .035, .18), self.paint, accent=1)
        self.cylinder("mast sleeve", (-.15, 0, 1.36), .07, .57, STEEL)
        self.cylinder("telescopic mast", (-.15, 0, 1.73), .037, .40, SHELL)
        j = self.joint("antenna head", (-.15, 0, 1.83), (0, 0, 1), "relay", .42)
        self.box("panel antenna", (-.15, 0, 1.84), (.14, .62, .34), self.paint, j, bevel=.035, accent=1)
        self.box("antenna face", (-.067, 0, 1.84), (.025, .53, .27), SHELL, j, bevel=.006)
        for y in (-.18, 0.0, .18):
            self.box("antenna element", (-.05, y, 1.84), (.02, .035, .22), STEEL, j)
        self.beam("whip aerial", (-.15, .24, 2.0), (-.15, .24, 2.26), .023, STEEL, j)
        self.cylinder("mast beacon", (-.15, 0, 2.055), .056, .06, LENS, j, segments=8)


@lru_cache(maxsize=15)
def build_model(lane: str, chassis: str) -> UnitModel:
    if (lane, chassis) not in valid_variants():
        raise ValueError(f"unsupported rescue unit: {lane}/{chassis}")
    b = _Builder(lane, chassis)
    b.chassis_geometry()
    b.body()
    {"none": b.scout, "scoop": b.digger, "gripper": b.carrier, "antenna": b.relay}[lane]()
    return UnitModel(f"{ROLE_NAMES[lane].lower()}_{chassis}", lane, chassis,
                     tuple(b.parts), tuple(b.joints))


class _SilhouetteBuilder(_Builder):
    """Rigid, closed primitives for distant units; no tiny trim or bevels."""

    def box(self, name, centre, size, color, joint=0, bevel=0.0, accent=0.0):
        vertices = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                             [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]])
        vertices = vertices * np.asarray(size) / 2 + centre
        faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
                 (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
                 (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
        self.mesh(name, vertices, faces, color, joint, accent)

    def beam(self, name, start, end, width, color, joint=0):
        a, b = np.asarray(start), np.asarray(end)
        axis = b - a
        length = np.linalg.norm(axis)
        axis = axis / length
        helper = np.array([1, 0, 0]) if abs(axis[0]) < .9 else np.array([0, 1, 0])
        side = np.cross(axis, helper)
        side /= np.linalg.norm(side)
        up = np.cross(axis, side)
        self.box(name, (0, 0, 0), (length, width, width), color, joint,
                 accent=float(color == self.paint))
        part = self.parts.pop()
        vertices = part.vertices @ np.stack([axis, side, up]) + (a + b) / 2
        self.mesh(name, vertices, part.triangles, color, joint, part.accent)


@lru_cache(maxsize=45)
def build_lod_model(lane: str, chassis: str, lod: int) -> UnitModel:
    """Three articulated detail levels: original, medium (<=450), far (<=120).

    These are deliberate silhouettes, not arbitrary triangle removal: cameras,
    bucket, stretcher and mast remain distinct, and travel/work still drive joints.
    Metres, pivots and wheel/stride periods match the detailed rig.
    """
    if lod == 0:
        return build_model(lane, chassis)
    if lod not in (1, 2) or (lane, chassis) not in valid_variants():
        raise ValueError(f"unsupported rescue unit LOD: {lane}/{chassis}/{lod}")
    medium = lod == 1
    b = _SilhouetteBuilder(lane, chassis)
    if chassis == "wheeled":
        for x in (-.44, .44):
            for y in (-.54, .54):
                centre = (x, y, .28)
                j = b.joint(f"wheel_{x}_{y}", centre, (0, 1, 0), "wheel", .25)
                b.cylinder("rubber tyre", centre, .25, .19, RUBBER, j,
                           axis=(0, 1, 0), segments=6 if medium else 4)
                if medium:
                    b.box("drive hub", (x, y + np.sign(y)*.11, .28), (.25, .025, .075),
                          b.paint, j, accent=1)
    elif chassis == "tracked":
        for y in (-.54, .54):
            b.box("continuous track belt", (0, y, .28), (1.55, .25, .54), RUBBER)
            # Keep a moving tread cue on both versions; the cheap wheel uses
            # the same .25 m drive radius as the detailed rig.
            j = b.joint(f"sprocket_{y}", (.5, y, .28), (0, 1, 0), "wheel", .25)
            b.box("track drive", (.5, y + np.sign(y)*.13, .28), (.25, .025, .075),
                  STEEL, j)
            if medium:
                b.box("track fender", (0, y, .57), (1.5, .31, .085), b.paint, accent=1)
                for x in (-.5, 0, .5):
                    b.cylinder("track sprocket", (x, y, .28), .2, .29, STEEL,
                               axis=(0, 1, 0), segments=6)
    elif chassis == "legged":
        for x in (-.43, .43):
            for side in (-1, 1):
                hip, foot = (x, side*.36, .64), (x, side*.77, .085)
                phase = 0.0 if x*side > 0 else np.pi
                j = b.joint(f"leg_{x}_{side}", hip, (0, 1, 0), "gait", .30, 1, phase)
                if medium:
                    knee = (x + np.sign(x)*.15, side*.65, .35)
                    b.beam("upper leg", hip, knee, .13, b.paint, j)
                    b.beam("shin", knee, foot, .10, SHELL, j)
                    b.box("terrain foot", foot, (.25, .22, .09), RUBBER, j)
                else:
                    b.beam("terrain leg", hip, foot, .12, b.paint, j)
    else:
        for y in (-.67, .67):
            b.box("rotor crossarm", (0, y, .85), (1.42, .10, .12), STEEL)
            for x in (-.66, .66):
                j = b.joint(f"rotor_{x}_{y}", (x, y, .99), (0, 0, 1), "rotor",
                            frequency=4, phase=0.0 if x*y > 0 else np.pi/4)
                b.box("rotor blade", (x, y, .99), (.60, .06, .025), SHELL, j)
                if medium:
                    b.box("rotor cross blade", (x, y, .99), (.06, .60, .025), SHELL, j)
                    b.cylinder("motor cap", (x, y, .97), .085, .16, INK, segments=4)
        if medium:
            for y in (-.44, .44):
                b.box("landing skid", (0, y, .10), (1.15, .08, .08), INK)
    b.box("armoured hull", (0, 0, .66), (1.24, .88, .48), b.paint, accent=1)
    if medium:
        b.box("upper armour", (0, 0, .87), (.92, .72, .07), SHELL)
        b.box("rear battery", (-.48, 0, .98), (.26, .58, .16), INK)
    if lane == "none":
        j = b.joint("camera gimbal", (.19, 0, .99), (0, 0, 1), "scan", .30)
        b.box("stereo camera", (.22, 0, 1.13), (.40, .61, .25), INK, j)
        b.box("camera glass", (.43, 0, 1.13), (.025, .49, .13), LENS, j)
        if medium:
            b.box("camera brow", (.21, 0, 1.265), (.43, .66, .055), b.paint, j, accent=1)
    elif lane == "scoop":
        j = b.joint("bucket linkage", (.30, 0, .90), (0, 1, 0), "dig", .38, .5)
        b.beam("lift arm", (.30, 0, .90), (1.17, 0, .34), .20, b.paint, j)
        b.box("bucket floor", (1.38, 0, .16), (.57, .87, .09), STEEL, j)
        b.box("bucket back", (1.13, 0, .30), (.08, .87, .31), b.paint, j, accent=1)
        if medium:
            for y in (-.43, .43):
                b.box("bucket cheek", (1.36, y, .29), (.51, .065, .29), b.paint, j, accent=1)
            for y in (-.30, 0, .30):
                b.box("bucket tooth", (1.70, y, .14), (.19, .085, .08), SHELL, j)
    elif lane == "gripper":
        b.box("rescue deck", (0, 0, 1.01), (1.31, .76, .18), STEEL)
        for side in (-1, 1):
            j = b.joint(f"padded gripper {side}", (.22, side*.45, 1.03),
                        (0, 0, 1), "grip", -side*.35)
            b.box("lifting jaw", (.55, side*.49, 1.12), (.56, .09, .14), b.paint, j, accent=1)
        j = b.joint("secured casualty", (0, 0, 0), (0, 0, 1), "payload")
        b.box("rescue blanket", (-.03, 0, 1.21), (.99, .43, .23), (.89, .79, .53), j)
        if medium:
            b.box("head support", (-.51, 0, 1.21), (.22, .27, .20), SHELL, j)
    else:
        b.box("telescopic mast", (-.15, 0, 1.41), (.09, .09, 1.0), STEEL)
        j = b.joint("antenna head", (-.15, 0, 1.83), (0, 0, 1), "relay", .42)
        b.box("panel antenna", (-.15, 0, 1.84), (.14, .62, .34), b.paint, j, accent=1)
        if medium:
            b.box("radio equipment", (-.15, 0, 1.04), (.48, .62, .27), INK)
            b.box("antenna face", (-.067, 0, 1.84), (.025, .53, .27), SHELL, j)
    return UnitModel(f"{ROLE_NAMES[lane].lower()}_{chassis}_lod{lod}", lane, chassis,
                     tuple(b.parts), tuple(b.joints))
