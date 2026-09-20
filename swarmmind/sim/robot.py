"""Robot bodies and the demo roster.

Bodies here are hand-set placeholders that satisfy the four-lane rescue chain. They are
replaced on D9 by elites selected out of the MAP-Elites archive
(``training/mapelites/select.py`` -> ``assets/scenarios/demo_roster.yaml``); this module
keeps the same ``RobotSpec`` shape so nothing downstream changes when that happens.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import yaml

LANES: tuple[str, ...] = ("none", "scoop", "gripper", "antenna")
LANE_INDEX = {a: i for i, a in enumerate(LANES)}

#: Locomotion, the third axis. Actuator decides *what* a robot can do; chassis decides
#: *where* it can go. Terrain is only meaningful if some of the swarm cannot cross it --
#: a river that everything fords is scenery.
CHASSIS: tuple[str, ...] = ("wheeled", "tracked", "legged", "rotor")
CHASSIS_INDEX = {c: i for i, c in enumerate(CHASSIS)}

#: (max traversable slope as rise/run, max water depth in metres, speed multiplier).
#: Fast and road-bound, through to slow and goes-anywhere.
CHASSIS_LIMITS: dict[str, tuple[float, float, float]] = {
    # 0.30 stranded wheeled units at 2% of the demo map -- a single bottleneck at the
    # base apron rim they could not climb, which is a broken robot rather than a
    # specialised one. Measured: 0.40 puts them at 60%.
    "wheeled": (0.40, 0.00, 1.18),
    "tracked": (0.58, 0.45, 1.00),
    # Legged is the goes-anywhere option, and its limits are set so it genuinely does.
    # At 0.95 it was itself getting cut off -- noise and mountain gradients stack, and a
    # chassis that is meant to be the fallback cannot have a reachability problem of its
    # own. Its cost is speed: 20% slower than tracked, 32% slower than wheeled. That is
    # the whole trade, and it is why all three are worth building.
    "legged": (1.60, 1.20, 0.80),
    # Rotor. The limits here are its *landed* limits -- where it can set down. Airborne
    # it ignores terrain entirely and moves at AIRBORNE_SPEED x its rating, but it sees
    # nothing while flying: fog only lifts and casualties are only detected once it is
    # on the ground. It is a spotter, not a survey drone, and it cannot lift a casualty.
    "rotor": (1.60, 0.00, 1.00),
}

#: The chassis that trade ground reach against ground speed. The rotor is deliberately
#: not among them: it is not a better legged unit, it is a different axis. It buys reach
#: by leaving the ground entirely and pays in blindness and in being unable to lift a
#: casualty, not in speed. Tests assert the ladder over these three only.
GROUND_CHASSIS: tuple[str, ...] = ("wheeled", "tracked", "legged")

#: Speed multiplier while airborne. The whole point of the lane: cross ground fast, land,
#: look, and hand the find to the diggers and carriers over the comms net.
AIRBORNE_SPEED = 2.0

#: Top reverse speed as a fraction of forward. Only the operator's manual override ever
#: reverses (control/manual.py) -- Tier 1 turns to face its goal and never commands a
#: negative speed -- so for every autonomous robot the old [0, v_max] clip is unchanged.
REVERSE_SPEED = 0.5

#: How much faster than its own rating a robot moves while the operator is driving it
#: (control/manual.py). A concession to the demo, not a property of the body: these are
#: 0.9-2.0 m/s machines on a 480x320 m map, and at the rated speed the digger a judge
#: takes the keys to crosses one sector in the time they have to watch it.
#:
#: Applied to exactly one robot at a time, only through the bridge, so no evaluation,
#: gate or seed-42 hash can see it -- `World.speed_boost` is all ones without a
#: dashboard attached. The safety floor is unaffected: `_wall_override` sweeps the
#: *commanded* distance, so a faster robot simply sweeps further, and at 20 Hz the
#: fastest boosted ground unit steps 0.20 m against a 0.5 m cell -- still two samples
#: inside every cell it enters.
#:
#: It stacks with `AIRBORNE_SPEED`, so a driven rotor in flight moves at 4x its rating.
#: That is deliberate: flying one across the map is the shot this is for.
OPERATOR_SPEED = 2.0

#: A rotor cannot carry a casualty, so it is never given the gripper actuator.
CHASSIS_BARRED: dict[str, frozenset[str]] = {"gripper": frozenset({"rotor"})}


def chassis_for(actuator: str, k: int) -> str:
    """Round-robin chassis for the k-th robot of a lane, skipping barred combinations."""
    allowed = [c for c in CHASSIS if c not in CHASSIS_BARRED.get(actuator, frozenset())]
    return allowed[k % len(allowed)]

# status codes, kept as small ints so robot status is a numpy array not a list of str
ACTIVE, OUT_OF_COMMS, FAILED, DESTROYED = 0, 1, 2, 3
STATUS_NAME = {ACTIVE: "active", OUT_OF_COMMS: "out_of_comms", FAILED: "failed", DESTROYED: "destroyed"}


@dataclass(frozen=True)
class RobotSpec:
    """One robot's physical parameters. Speed and size are coupled: heavier is slower."""

    id: str
    actuator: str
    chassis: str
    radius: float
    v_max: float
    omega_max: float
    sensor_radius: float
    battery_capacity: float


#: Placeholder lane archetypes: (radius, v_max, omega_max, sensor_radius, battery_cap).
#: The fast/light <-> slow/heavy tradeoff of the MVP spec's second archive axis, set by
#: hand until evolution measures it.
_ARCHETYPE: dict[str, tuple[float, float, float, float, float]] = {
    "none":    (0.22, 2.00, 3.5, 7.0, 1.00),   # scout   -- fast, light, far-seeing
    "antenna": (0.28, 1.40, 2.8, 5.0, 1.20),   # relay   -- middling, long endurance
    "gripper": (0.38, 1.20, 2.2, 4.5, 1.30),   # carrier -- slow, strong
    "scoop":   (0.45, 0.90, 2.0, 4.0, 1.40),   # digger  -- slowest, toughest
}


def default_roster(per_lane: int, rng: np.random.Generator,
                   counts: dict[str, int] | None = None) -> list[RobotSpec]:
    """Robots of each actuator lane, with deterministic per-robot jitter.

    ``counts`` gives per-lane sizes; without it every lane gets ``per_lane``. The even
    split is rarely the right shape -- the relay network saturates at a few dozen posts
    however many antennas exist, so the rest are 25% of the swarm standing at spawn.

    Jitter exists so the swarm does not move as four rigid blocks -- identical bodies
    produce identical bids, which produces visible lockstep and ties broken by id.
    """
    specs: list[RobotSpec] = []
    n = 0
    for actuator in LANES:
        radius, v_max, omega, sensor, batt = _ARCHETYPE[actuator]
        lane_n = per_lane if not counts else int(counts.get(actuator, 0))
        for k in range(lane_n):
            j = rng.uniform(0.92, 1.08, 5)
            # Chassis is mixed evenly within every lane, not tied to it: the point is
            # that a task beyond a river needs a carrier that can ford it, which only
            # bites if carriers differ among themselves.
            chassis = chassis_for(actuator, k)
            specs.append(
                RobotSpec(
                    id=f"r{n:02d}",
                    actuator=actuator,
                    chassis=chassis,
                    radius=float(radius * j[0]),
                    v_max=float(v_max * j[1] * CHASSIS_LIMITS[chassis][2]),
                    omega_max=float(omega * j[2]),
                    sensor_radius=float(sensor * j[3]),
                    battery_capacity=float(batt * j[4]),
                )
            )
            n += 1
    return specs


#: Where `training/mapelites/select.py` writes the evolved roster. Absent until a
#: MAP-Elites run has produced one, which is the normal state before D9 completes.
ROSTER_PATH = Path(__file__).resolve().parents[2] / "assets" / "scenarios" / (
    "demo_roster.yaml")


def load_roster(path: Path | None = None) -> list[RobotSpec] | None:
    """Read an evolved roster, or return None if there is not one.

    None is not a failure. The hand-set archetypes are the gate's baseline, so a
    checkout with no archive must behave exactly as it did before evolution existed.
    """
    path = Path(path) if path is not None else ROSTER_PATH
    if not path.exists():
        return None
    # Explicit encoding: this file is written by `training/mapelites/select.py`, which
    # may well be a different machine from the one reading it. Defaulting to the locale
    # encoding makes a roster evolved on one box unreadable on another.
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    rows = raw.get("robots") or []
    if not rows:
        return None
    return [RobotSpec(**{k: v for k, v in r.items() if k != "elite"}) for r in rows]


def evolved_roster(per_lane: int, rng: np.random.Generator,
                   path: Path | None = None,
                   counts: dict[str, int] | None = None,
                   use_evolved: bool = True) -> list[RobotSpec]:
    """The evolved roster if one exists, cut or cycled to `per_lane` per actuator.

    A roster stored at 8 robots per lane has to serve a 128-per-lane demo, so bodies are
    cycled rather than regenerated -- an evolved elite repeated is still an evolved
    elite, and it keeps the id scheme and lane balance that everything downstream
    assumes.
    """
    # `use_evolved=False` is how the gate gets a genuinely classical arm. Until D12 it
    # could not: `World` loaded the roster unconditionally, so the "classical baseline"
    # ran evolved *bodies* with a classical bid and the comparison measured something
    # other than what it claimed. A control arm nobody can construct is not a control.
    evolved = load_roster(path) if use_evolved else None
    if evolved is None:
        return default_roster(per_lane, rng, counts)

    by_lane: dict[str, list[RobotSpec]] = {a: [] for a in LANES}
    for spec in evolved:
        by_lane.setdefault(spec.actuator, []).append(spec)

    out: list[RobotSpec] = []
    n = 0
    for actuator in LANES:
        lane_n = per_lane if not counts else int(counts.get(actuator, 0))
        pool = by_lane.get(actuator) or []
        if not pool:
            # A lane evolution never filled falls back to its archetype rather than
            # being dropped -- a swarm with no carriers cannot rescue anyone.
            radius, v_max, omega, sensor, batt = _ARCHETYPE[actuator]
            for k in range(lane_n):
                chassis = chassis_for(actuator, k)
                j = rng.uniform(0.92, 1.08, 5)
                out.append(RobotSpec(
                    id=f"r{n:02d}", actuator=actuator, chassis=chassis,
                    radius=float(radius * j[0]),
                    v_max=float(v_max * j[1] * CHASSIS_LIMITS[chassis][2]),
                    omega_max=float(omega * j[2]),
                    sensor_radius=float(sensor * j[3]),
                    battery_capacity=float(batt * j[4]),
                ))
                n += 1
            continue
        for k in range(lane_n):
            src = pool[k % len(pool)]
            out.append(replace(src, id=f"r{n:02d}",
                               chassis=chassis_for(actuator, k)))
            n += 1
    return out
