"""Display particles: the model, written here so the dashboard's version can be checked.

Digging is the one stage of the rescue chain with nothing to show for itself. A scoop
robot clearing a slab and a scoop robot sitting idle are the same box at the same
coordinates for twenty seconds, so the lane that unburies 32 of the 80 casualties was
invisible on the map and legible only in the event feed.

**Godot cannot be run from the development environment**, so the maths lives here first
and `godot/scripts/main.gd` is a port of it: same integration, same per-kind constants,
same fade curve. `tests/test_bridge_protocol.py` fails the build if the two drift apart,
and `scripts/snapshot3d.py --fx` renders the field offline so a plume can actually be
looked at before it is written in GDScript.

Everything here is **display only**. Nothing in this module is read by the simulator, the
auction or any controller, and the field is driven by `world.digging` and `/swarm/events`
-- both of which describe things the swarm is already doing in plain sight, so drawing
them reveals nothing the operator could not see anyway.

The cost model is the reason the parameters look the way they do. One MultiMesh, one
draw call, a fixed pool, and a per-particle update cheap enough to run in GDScript every
frame: three vector operations and a multiply-add, no branches, no allocation. Anything
that needs a curve lookup or a sort does not belong in it.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: Pool ceiling, shared with the dashboard. **Measured, not chosen** (MEASUREMENTS.md
#: M-66): demo/seed 42 to t=320 peaks at 199 live with 4 simultaneous excavations and
#: never reaches the cap, and god-view embers add at most ~57 on top. 448 is a little
#: over 2x the realistic worst case. When the pool is full the oldest particle is
#: recycled, so an over-subscribed field degrades into a shorter-lived one, not a stall.
MAX_PARTICLES = 448

#: Particles emitted per second by one digger working a casualty. A dig lasts ~20 s at
#: `clear_rate`, so this is a continuous plume rather than a burst, and the pool has to
#: hold every simultaneous excavation on the map at once (up to 32 on the demo scenario).
#: 22/s at ~1.9 s of life is ~42 live particles per site. Demo/seed 42 peaks at 4
#: simultaneous excavations and 199 live particles including bursts (M-66).
DIG_RATE = 22.0

#: Extra emission per additional digger, as a fraction of `DIG_RATE`. Sub-linear on
#: purpose: `MAX_DIGGERS` caps the excavation rate at 3, and the plume tracks the rate,
#: so a fourth machine parked on the same hole must not make the dust grow.
DIG_PER_DIGGER = 0.45

#: One spawn in this many is a rubble chip rather than dust. Chips are what say *debris
#: is being moved*; dust alone reads as a smoke machine.
DIG_CHIP_EVERY = 4

#: Emission stops beyond this range from the camera. At 260 m of orbit a 0.4 m particle
#: is well under a pixel, so this is throwing away work that could not be seen -- it is
#: not a draw-distance compromise. Applied at spawn, so particles already alive finish
#: their lives normally and nothing pops.
CULL_M = 190.0

#: Hazard embers per second, per metre of hazard radius, and the ceiling on that rate.
#: The disc grows for most of the mission, so an unbounded rate would eventually spend
#: the whole pool on scenery. Embers are drawn **only on the god-view toggle**, exactly
#: like the hazard disc itself: the true fire is ground truth, and a second, prettier
#: leak of it is still a leak.
EMBER_PER_M = 0.6
EMBER_MAX_RATE = 26.0


@dataclass(frozen=True)
class Kind:
    """One particle species. Fields are in SI units and mirrored in main.gd."""

    name: str
    colour: tuple[float, float, float]
    alpha: float
    life: tuple[float, float]
    #: Vertical launch speed, m/s.
    up: tuple[float, float]
    #: Horizontal launch speed, m/s, in a uniformly random bearing.
    out: tuple[float, float]
    #: Downward acceleration. Dust is buoyant and hangs; a chip is a rock.
    gravity: float
    #: Velocity lost per second, as a fraction. Air resistance on something light.
    drag: float
    size: tuple[float, float]
    #: Metres of edge gained per second. Dust plumes expand as they rise; debris does not.
    grow: float
    #: Radius of the spawn scatter around the emission point.
    jitter: float


#: The species table. Order is the wire-independent display contract between this module
#: and main.gd -- the GDScript FX_KINDS dictionary carries the same names and numbers,
#: and test_bridge_protocol.py compares them field by field.
#:
#: Colours are from the **display** palette, not `perception/raster.py`. The detector's
#: world is dark and low-contrast by design; dust that read correctly to a conv net would
#: be invisible on a projector in a lit room.
KINDS: dict[str, Kind] = {
    # Excavation dust. Warm, pale, slow: the colour of pulverised concrete against the
    # DISPLAY_RUBBLE ground, light enough to read against both rubble and shadow.
    "dust": Kind("dust", (0.87, 0.83, 0.74), 0.62, (1.4, 2.4), (2.2, 4.5), (0.8, 2.2),
                 1.6, 1.1, (0.34, 0.62), 0.80, 0.70),
    # Rubble thrown clear of the hole. Ballistic, dark, and it does not expand.
    "chip": Kind("chip", (0.34, 0.28, 0.22), 0.95, (0.7, 1.2), (2.2, 4.2), (0.8, 2.6),
                 9.8, 0.15, (0.12, 0.22), 0.0, 0.35),
    # A robot dying. Smoke first, then the sparks below it.
    "smoke": Kind("smoke", (0.20, 0.19, 0.18), 0.70, (1.4, 2.4), (1.4, 3.0), (0.5, 1.8),
                  -0.6, 1.2, (0.45, 0.85), 1.30, 0.60),
    "spark": Kind("spark", (1.00, 0.56, 0.16), 1.00, (0.5, 1.0), (3.0, 7.0), (1.5, 4.5),
                  11.0, 0.20, (0.10, 0.18), 0.0, 0.30),
    # A casualty reaching an extraction zone. The one unambiguously good event in the
    # mission, and the only effect that rises without falling back. Matches the
    # `victim_rescued` colour in the event feed so the burst and the log line agree.
    "lift": Kind("lift", (0.47, 1.00, 0.55), 0.85, (1.1, 1.8), (2.6, 5.0), (0.3, 1.2),
                 -1.1, 0.9, (0.16, 0.30), 0.25, 0.90),
    # The slab coming off: one broad, slow puff, bigger and paler than working dust so
    # the end of a dig is distinguishable from the middle of one.
    "puff": Kind("puff", (0.92, 0.89, 0.81), 0.62, (1.3, 2.1), (0.6, 1.8), (1.4, 3.2),
                 1.4, 1.9, (0.45, 0.80), 1.10, 0.80),
    # Power drawn at the base pad. Cool blue-white against every other effect in the
    # table, which are all warm: nothing else on the map is this colour, so a charging
    # robot reads at a glance even in a crowd. Rises gently and briefly -- a top-up is
    # a pause, not an event, and it must not compete with a rescue burst.
    "charge": Kind("charge", (0.55, 0.82, 1.00), 0.90, (0.6, 1.0), (1.6, 3.2), (0.6, 1.6),
                   -0.9, 0.8, (0.10, 0.20), 0.30, 0.55),
    # Hazard embers. Gated on the god-view toggle exactly like the hazard disc itself --
    # the true fire is ground truth and the operator only sees it with the toggle on.
    "ember": Kind("ember", (1.00, 0.42, 0.13), 0.85, (1.6, 2.8), (2.0, 5.0), (0.2, 1.0),
                  -1.4, 0.7, (0.14, 0.28), 0.10, 1.20),
}

#: Burst sizes for the one-shot effects, keyed by `/swarm/events` kind. An event with no
#: entry gets no particles; test_bridge_protocol.py checks every kind named here is a
#: real event kind, so a rename cannot leave a burst wired to nothing.
BURSTS: dict[str, tuple[tuple[str, int], ...]] = {
    "victim_rescued": (("lift", 22), ("dust", 8)),
    "victim_cleared": (("puff", 14), ("chip", 6)),
    "robot_destroyed": (("smoke", 14), ("spark", 18)),
    "robot_recharged": (("charge", 12),),
}

_FIELDS = ("x", "y", "z", "vx", "vy", "vz", "age", "life", "size", "grow",
           "gravity", "drag", "r", "g", "b", "a")


def fade(u: np.ndarray | float):
    """Alpha envelope over normalised age. Quadratic ease-out.

    One curve for every kind, because the GDScript port evaluates it per particle per
    frame and a per-kind curve would mean a branch or a table lookup in the hot loop.
    `(1-u)^2` is two operations and reads as dust thinning rather than dust switching off.
    """
    f = 1.0 - u
    return f * f


class ParticleField:
    """A fixed pool of display particles, integrated with explicit Euler.

    Explicit Euler at 60 Hz over a 1 s life is visibly indistinguishable from anything
    better for ballistic dust, and it is what GDScript can afford. The arrays are
    parallel and flat for the same reason the robot state is (CLAUDE.md scaling rules):
    the update is vectorised here and a single tight loop there.
    """

    def __init__(self, seed: int = 42, capacity: int = MAX_PARTICLES) -> None:
        # Display RNG, seeded and entirely separate from the simulator's RngBook. A
        # rehearsed demo should look the same on the second run as the first, and nothing
        # drawn here may ever perturb the sim's streams (CLAUDE.md invariant 6).
        self.rng = np.random.default_rng(seed)
        self.cap = int(capacity)
        self.n = 0
        self._d = {f: np.zeros(self.cap, dtype=np.float64) for f in _FIELDS}
        self._carry: dict[tuple[float, float], float] = {}

    def __len__(self) -> int:
        return self.n

    # ------------------------------------------------------------------ emission

    def spawn(self, kind: str, x: float, y: float, z: float, count: int) -> int:
        """Add up to ``count`` particles of ``kind`` at a point. Returns how many landed."""
        k = KINDS[kind]
        count = int(count)
        if count <= 0:
            return 0
        # Recycle the oldest rather than dropping the newest: a full pool should thin the
        # whole field evenly, not freeze it at whatever happened to fill it first.
        free = self.cap - self.n
        if count > free:
            self._recycle(count - free)
        i0, i1 = self.n, self.n + count
        self.n = i1
        d, rng = self._d, self.rng
        sl = slice(i0, i1)

        bearing = rng.uniform(0.0, 2.0 * np.pi, count)
        jr = k.jitter * np.sqrt(rng.uniform(0.0, 1.0, count))
        d["x"][sl] = x + np.cos(bearing) * jr
        d["y"][sl] = y + np.sin(bearing) * jr
        d["z"][sl] = z + rng.uniform(0.0, 0.35, count)

        launch = rng.uniform(0.0, 2.0 * np.pi, count)
        out = rng.uniform(*k.out, count)
        d["vx"][sl] = np.cos(launch) * out
        d["vy"][sl] = np.sin(launch) * out
        d["vz"][sl] = rng.uniform(*k.up, count)

        d["age"][sl] = 0.0
        d["life"][sl] = rng.uniform(*k.life, count)
        d["size"][sl] = rng.uniform(*k.size, count)
        d["grow"][sl] = k.grow
        d["gravity"][sl] = k.gravity
        d["drag"][sl] = k.drag
        d["r"][sl], d["g"][sl], d["b"][sl] = k.colour
        d["a"][sl] = k.alpha
        return count

    def burst(self, event_kind: str, pos, z: float) -> int:
        """Fire the one-shot effect for a `/swarm/events` kind, if it has one."""
        made = 0
        for kind, count in BURSTS.get(event_kind, ()):
            made += self.spawn(kind, float(pos[0]), float(pos[1]), z, count)
        return made

    def emit_dig(self, x: float, y: float, z: float, diggers: int, dt: float) -> int:
        """Continuous plume for one excavation.

        Fractional particles are carried between calls per site, so a rate that works out
        to 1.4 particles a frame emits 1 and 2 alternately instead of always rounding to
        the same number and drifting off the intended rate.
        """
        rate = DIG_RATE * (1.0 + DIG_PER_DIGGER * (max(int(diggers), 1) - 1))
        key = (round(x, 1), round(y, 1))
        want = self._carry.get(key, 0.0) + rate * dt
        count = int(want)
        self._carry[key] = want - count
        if count <= 0:
            return 0
        chips = count // DIG_CHIP_EVERY
        return (self.spawn("dust", x, y, z, count - chips)
                + self.spawn("chip", x, y, z, chips))

    # ------------------------------------------------------------------ the step

    def step(self, dt: float) -> None:
        if self.n == 0:
            return
        d, sl = self._d, slice(0, self.n)
        d["age"][sl] += dt
        d["vz"][sl] -= d["gravity"][sl] * dt
        keep = np.maximum(0.0, 1.0 - d["drag"][sl] * dt)
        for v in ("vx", "vy", "vz"):
            d[v][sl] *= keep
        for p, v in (("x", "vx"), ("y", "vy"), ("z", "vz")):
            d[p][sl] += d[v][sl] * dt
        dead = np.nonzero(d["age"][sl] >= d["life"][sl])[0]
        if len(dead):
            self._drop(dead)

    def update(self, world, dt: float, *, camera=None) -> None:
        """One display frame: emit from `world.digging`, then integrate.

        The dig sites come from the simulator's own excavation predicate, so a plume
        exists exactly when debris is moving. `camera` is optional and only used for the
        `CULL_M` spawn cull; offline frames pass the renderer's camera position.
        """
        for x, y, n in world.digging:
            if camera is not None and (x - camera[0]) ** 2 + (y - camera[1]) ** 2 > CULL_M ** 2:
                continue
            self.emit_dig(x, y, self.ground_at(world, x, y), int(n), dt)
        self.step(dt)

    @staticmethod
    def ground_at(world, x: float, y: float) -> float:
        ix = int(np.clip(x / world.cell, 0, world.shape[1] - 1))
        iy = int(np.clip(y / world.cell, 0, world.shape[0] - 1))
        return float(world.height[iy, ix])

    # ------------------------------------------------------------------ readback

    def points(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Live particles as ``(positions, rgb 0-255, alpha, edge metres)``."""
        d, sl = self._d, slice(0, self.n)
        pos = np.stack([d["x"][sl], d["y"][sl], d["z"][sl]], axis=1).astype(np.float32)
        rgb = np.stack([d["r"][sl], d["g"][sl], d["b"][sl]], axis=1).astype(np.float32) * 255.0
        u = np.clip(d["age"][sl] / np.maximum(d["life"][sl], 1e-9), 0.0, 1.0)
        alpha = (d["a"][sl] * fade(u)).astype(np.float32)
        size = (d["size"][sl] + d["grow"][sl] * d["age"][sl]).astype(np.float32)
        return pos, rgb, alpha, np.maximum(size, 0.01)

    # ------------------------------------------------------------------ pool

    def _drop(self, idx: np.ndarray) -> None:
        """Swap-remove, which is why nothing here may depend on particle order."""
        keep = np.ones(self.n, dtype=bool)
        keep[idx] = False
        order = np.nonzero(keep)[0]
        for f in _FIELDS:
            self._d[f][: len(order)] = self._d[f][order]
        self.n = len(order)

    def _recycle(self, count: int) -> None:
        oldest = np.argsort(-self._d["age"][: self.n], kind="stable")[:count]
        self._drop(oldest)
