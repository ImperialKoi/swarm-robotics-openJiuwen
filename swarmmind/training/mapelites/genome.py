"""The 15-gene genome: five numbers that build a robot, ten that decide how it behaves.

Everything is a real value in `[0,1]`, because that is what MAP-Elites mutates. Decoding
happens here and nowhere else, so an archive stored today still means the same thing when
it is read back on demo day.

**Why parameters instead of a network.** These ten behaviour genes *are* the Tier-2
policy. An interpretable vector evolves faster than an MLP at this budget, cannot produce
uninspectable garbage, and reads on a dashboard -- a judge can be shown that this scout
has `hazard_aversion` 0.2 and watch it cut the corner. `tier2_policy.py` keeps an
implementation slot for an MLP behind the same interface if it ever earns one.

**The midpoint is near the hand-tuned baseline, but it is not the baseline.** Every range
is centred so that an all-0.5 genome decodes to approximately the `_ARCHETYPE` and
`BidWeights` values already in use, so evolution starts from something sensible rather
than from noise. It is *not* equivalent, and the difference matters for the gate: the
gene-driven bid has four terms the classical bid does not have at all
(`frontier_gain`, `revisit_penalty`, `comms_tether`, `battery_reserve`), so a midpoint
genome is a richer policy that merely shares the classical coefficients. Measured on
`test`/seed 42 it already explores 47.7% against the classical 35.8%.

**So the gate's control arm is `genes=None`, not an all-0.5 genome.** `run_mission()`
passes no genome for exactly this reason. Comparing evolved against midpoint would be
measuring evolution against a policy that already contains most of the change, and would
flatter the result.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ...sim.robot import CHASSIS_LIMITS, LANES, RobotSpec

#: Order is the archive's contract. Appending is safe; reordering invalidates every
#: archive ever stored, so it must not happen after an archive is created.
GENE_NAMES: tuple[str, ...] = (
    # --- morphology ---------------------------------------------------------------
    "actuator_g",        # -> floor(g * 4) picks the lane
    "chassis_scale",     # body radius: small and nimble <-> large and tough
    "motor_power",       # top speed, and the energy it costs
    "sensor_scale",      # how far it sees
    "battery_scale",     # endurance
    # --- behaviour ----------------------------------------------------------------
    "frontier_gain",
    "distance_penalty",
    "hazard_aversion",
    "comms_tether",
    "revisit_penalty",
    "task_commitment",
    "formation_spacing",
    "wander_bias",
    "battery_reserve",
    "switch_cost",
)
N_GENES = len(GENE_NAMES)
GENE_INDEX = {g: i for i, g in enumerate(GENE_NAMES)}

#: (low, high) for each decoded quantity. Midpoint = the hand-set baseline.
RANGES: dict[str, tuple[float, float]] = {
    "chassis_scale":     (0.18, 0.50),
    "motor_power":       (0.70, 2.30),
    "sensor_scale":      (3.00, 8.00),
    "battery_scale":     (0.80, 1.60),
    "frontier_gain":     (0.00, 2.00),
    "distance_penalty":  (0.30, 2.00),
    "hazard_aversion":   (0.00, 90.0),   # midpoint 45 ~ BidWeights.hazard = 40
    "comms_tether":      (0.00, 40.0),
    "revisit_penalty":   (0.00, 1.50),
    "task_commitment":   (0.00, 12.0),
    "formation_spacing": (1.20, 4.00),   # -> ReflexParams.separation_radius (2.5)
    "wander_bias":       (0.00, 1.00),
    "battery_reserve":   (0.00, 0.60),
    "switch_cost":       (0.00, 20.0),
}

#: Heavier bodies are slower. Without this coupling every elite is simply "maximum motor,
#: maximum battery, maximum sensor" and the archive's second axis collapses to one bin --
#: there has to be a cost, or there is no trade-off to discover.
MASS_SPEED_PENALTY = 0.55


def _lerp(name: str, g: float) -> float:
    lo, hi = RANGES[name]
    return lo + (hi - lo) * float(np.clip(g, 0.0, 1.0))


@dataclass(frozen=True)
class BehaviorGenes:
    """The ten Tier-2 genes, decoded. Scalars for one robot; see `BehaviorParams`."""

    frontier_gain: float
    distance_penalty: float
    hazard_aversion: float
    comms_tether: float
    revisit_penalty: float
    task_commitment: float
    formation_spacing: float
    wander_bias: float
    battery_reserve: float
    switch_cost: float


def lane_of(genome) -> str:
    """`floor(g * 4)` -> lane. Clipped so a gene of exactly 1.0 is not a fifth lane."""
    g = float(np.clip(genome[GENE_INDEX["actuator_g"]], 0.0, 1.0))
    return LANES[min(int(g * len(LANES)), len(LANES) - 1)]


def decode_behavior(genome) -> BehaviorGenes:
    return BehaviorGenes(**{
        name: _lerp(name, genome[GENE_INDEX[name]])
        for name in BehaviorGenes.__dataclass_fields__
    })


def decode_spec(genome, robot_id: str, chassis: str) -> RobotSpec:
    """Morphology genes -> a `RobotSpec` the simulator can spawn.

    Chassis is *not* a gene. It is assigned by the roster, because locomotion is what
    makes terrain meaningful and evolution left to itself picks whichever chassis is
    cheapest on the training map and deletes the other two.
    """
    radius = _lerp("chassis_scale", genome[GENE_INDEX["chassis_scale"]])
    motor = _lerp("motor_power", genome[GENE_INDEX["motor_power"]])
    # Bigger body, slower robot -- and legged pays its own separate 20%.
    size_drag = 1.0 - MASS_SPEED_PENALTY * (radius - RANGES["chassis_scale"][0]) / (
        RANGES["chassis_scale"][1] - RANGES["chassis_scale"][0])
    return RobotSpec(
        id=robot_id,
        actuator=lane_of(genome),
        chassis=chassis,
        radius=radius,
        v_max=motor * size_drag * CHASSIS_LIMITS[chassis][2],
        omega_max=1.6 + 2.4 * (1.0 - (radius - RANGES["chassis_scale"][0]) / (
            RANGES["chassis_scale"][1] - RANGES["chassis_scale"][0])),
        sensor_radius=_lerp("sensor_scale", genome[GENE_INDEX["sensor_scale"]]),
        battery_capacity=_lerp("battery_scale", genome[GENE_INDEX["battery_scale"]]),
    )


def baseline() -> np.ndarray:
    """All-0.5: the hand-tuned heuristic, expressed as a genome.

    This is the origin evolution is measured against, and the seed the first generation
    mutates from.
    """
    return np.full(N_GENES, 0.5)


def random_genomes(rng: np.random.Generator, n: int) -> np.ndarray:
    return rng.random((n, N_GENES))


class BehaviorParams:
    """Per-robot behaviour genes as parallel arrays.

    A swarm is 768 robots with 768 different genomes, and the auction bids on all of them
    in one vectorised expression. Anything that forces a Python loop over robots here
    breaks the scaling rule that the whole simulator is built around, so every gene is an
    `(n,)` array and the bid stays one numpy expression.
    """

    __slots__ = tuple(BehaviorGenes.__dataclass_fields__)

    def __init__(self, genomes: np.ndarray) -> None:
        genomes = np.atleast_2d(np.asarray(genomes, dtype=float))
        for name in self.__slots__:
            lo, hi = RANGES[name]
            col = np.clip(genomes[:, GENE_INDEX[name]], 0.0, 1.0)
            setattr(self, name, lo + (hi - lo) * col)

    def __len__(self) -> int:
        return len(self.frontier_gain)

    @classmethod
    def uniform(cls, n: int, genome=None) -> BehaviorParams:
        """One genome applied to every robot -- the baseline swarm."""
        g = baseline() if genome is None else np.asarray(genome, dtype=float)
        return cls(np.tile(g, (n, 1)))
