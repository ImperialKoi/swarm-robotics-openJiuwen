"""Fitness and measures for one genome.

A candidate is evaluated **inside a working swarm**, not alone. A gripper genome is only
meaningful if there are scouts finding casualties and diggers uncovering them; scored in
isolation, the best carrier is whichever one wanders least. So the candidate's genome
fills its own lane and the rest of the swarm runs the classical baseline. What is being
measured is the candidate's contribution to a rescue chain that actually functions.

The two measures are what the archive is organised by, and only one of them is chosen:

* **Lane** comes from the genome (`floor(actuator_g * 4)`).
**Fitness is normalised per lane, and the raw scales are wildly different.** Raw relay
fitness is a robot count (~16); raw scout fitness is explored-fraction-per-joule (~0.8).
Left raw, the archive's `qd_score` and `obj_max` are just the antenna lane with noise on
top, and a run's headline numbers say nothing about the other three lanes. `LANE_REFERENCE`
divides each lane by a documented "this is good" value. It is a monotone transform, so
*selection within a lane is unchanged* -- what it fixes is cross-lane reporting.

**Fitness reads ground truth, and that is correct.** `_fitness` counts real casualties
delivered and real debris cleared, straight from the simulator. That does not violate
invariant #4: the invariant is about *swarm-side* code, and scoring is not swarm-side. The
robots being scored still see nothing but `Detection` objects -- a genome is rewarded for
casualties it actually rescued, not for casualties it believed it rescued, which is the
only way phantom reports get punished rather than paid for. Do not "fix" this to read the
report tracker; that would make a detector that hallucinates victims score well.

* **Mean speed is measured, not intended.** A genome can ask for a big motor and still
  crawl, because it is heavy, or because its own `formation_spacing` keeps it jammed
  against neighbours, or because it spends the episode stuck in marsh. Binning on the
  requested top speed would fill the archive with robots that are fast on paper. This is
  the axis the diversity claim rests on, so it has to be observed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ...mission import Mission
from ...sim.robot import CHASSIS, LANE_INDEX
from ...sim.scenario import Scenario
from .genome import BehaviorParams, baseline, decode_spec, lane_of

#: 60 s is long enough for a scout to clear real ground and a carrier to complete a
#: delivery, and short enough that 300k of them fit in an evening on eight cores.
EPISODE_S = 60.0

#: Training seeds. The gate re-evaluates on ten seeds never used here -- an elite that
#: only wins on these three has memorised three maps.
TRAIN_SEEDS: tuple[int, ...] = (11, 12, 13)

#: Out of comms for more than this fraction of the episode and a scout is penalised.
#: A scout that runs off alone finds plenty and reports none of it.
COMMS_FLOOR = 0.30
COMMS_PENALTY = 0.5

#: Per-lane "a good robot scores about this" divisors, so the four lanes are commensurable
#: and `qd_score` means something. Set from the first real archive (MEASUREMENTS.md M-31).
#: Dividing is monotone: within a lane, the ordering of elites is untouched.
LANE_REFERENCE: dict[str, float] = {
    "none": 0.9,      # explored fraction per joule
    "scoop": 12.0,    # debris cleared + 2 x casualties freed
    "gripper": 2.0,   # casualties delivered, plus a promptness bonus
    "antenna": 4.0,   # mean robots held in comms *by a relay*
}


@dataclass
class Evaluation:
    fitness: float
    lane: int
    mean_speed: float
    detail: dict


def _apply_morphology(world, genome, mine: np.ndarray) -> None:
    """Write the candidate's decoded body onto its lane, in place.

    The world reads `radius`, `v_max`, `omega_max`, `sensor_radius` and `battery_cap`
    live every tick and precomputes nothing from them, so overwriting after construction
    is equivalent to having spawned them that way. Chassis is left alone -- it is the
    roster's choice, not a gene, so each chassis keeps its share of the lane and a
    genome cannot quietly evolve the terrain problem away.
    """
    for i in np.nonzero(mine)[0]:
        spec = decode_spec(genome, world.robot_ids[i], CHASSIS[int(world.chassis[i])])
        world.radius[i] = spec.radius
        world.v_max[i] = spec.v_max
        world.omega_max[i] = spec.omega_max
        world.sensor_radius[i] = spec.sensor_radius
        world.battery_cap[i] = spec.battery_capacity


def _episode(scn: Scenario, genome, seed: int) -> tuple[Mission, np.ndarray]:
    """One 60 s FastSim episode. Tier 3 is off -- this measures Tier 2."""
    m = Mission(scn, seed, hivemind=False)
    w = m.world
    lane_i = LANE_INDEX[lane_of(genome)]
    mine = w.actuator == lane_i

    # Candidate genes on its own lane, baseline everywhere else.
    genomes = np.tile(baseline(), (w.n, 1))
    genomes[mine] = np.asarray(genome, dtype=float)
    genes = BehaviorParams(genomes)
    _apply_morphology(w, genome, mine)
    m.genes = genes
    m.reflex.set_genes(genes)
    m.allocator.genes = genes

    prev = w.pos.copy()
    path = np.zeros(w.n)
    in_comms_ticks = np.zeros(w.n)
    relay_ticks = np.zeros(w.n)
    ticks = 0
    while w.t < EPISODE_S and not w.done:
        m.tick()
        path += np.linalg.norm(w.pos - prev, axis=1)
        prev = w.pos.copy()
        in_comms_ticks += w.in_comms
        relay_ticks += w.comms_via_relay
        ticks += 1
    return m, np.stack([path, in_comms_ticks, np.full(w.n, max(ticks, 1)), relay_ticks])


def _fitness(m: Mission, lane: str, mine: np.ndarray, stats: np.ndarray) -> tuple[float, dict]:
    """Lane-specific. Each lane is asked for the thing that lane exists to do."""
    w = m.world
    path, comms_ticks, ticks, relay_ticks = stats
    energy = float(w.energy_used[mine].sum())
    comms_frac = float((comms_ticks[mine] / ticks[mine]).mean()) if mine.any() else 0.0

    if lane == "none":                        # scout: ground covered, per unit of energy
        covered = float(w.explored[w.passable].mean())
        f = covered / max(energy, 1e-6)
        if comms_frac < 1.0 - COMMS_FLOOR:
            f *= COMMS_PENALTY
        detail = {"explored_frac": covered, "energy": energy, "comms_frac": comms_frac}

    elif lane == "scoop":                     # digger: debris moved and casualties freed
        # Debris is a *continuous* quantity, not a count of finished digs. A digger that
        # got three casualties to 90% scores above one that touched nothing, and at a
        # ~1.4k-evaluation budget that difference is most of the gradient -- an integer
        # step function leaves nearly every early genome tied at zero.
        debris = float(sum(1.0 - v.debris_remaining for v in w.victims if v.buried))
        freed = sum(1 for v in w.victims if v.state >= 2)
        f = debris + 2.0 * float(freed)
        detail = {"debris_cleared": round(debris, 3), "victims_cleared": freed}

    elif lane == "gripper":                   # carrier: deliveries, and how fast
        delivered = w.victims_rescued
        mtr = w.scorecard().mean_time_to_rescue
        promptness = 0.0 if delivered == 0 else 0.3 * (1.0 - min(mtr, EPISODE_S) / EPISODE_S)
        f = float(delivered) + promptness
        detail = {"delivered": delivered, "mean_time_to_rescue": mtr}

    else:
        # Relay: mean number of robots held in comms *by a relay* -- connected, but not
        # within base range on their own. That is exactly the set which would drop out
        # if the relays were removed, so it is causal rather than a proxy, and it costs
        # nothing extra because the simulator already separates the two terms.
        #
        # The earlier measure counted every connected robot. On a 16-robot map everyone
        # is near base anyway, so every relay scored an identical 16.0 and the lane
        # learned nothing at all (MEASUREMENTS.md M-31).
        f = float(relay_ticks.sum() / max(ticks[0], 1))
        detail = {"mean_robots_held_by_relay": round(f, 3)}

    detail["energy"] = energy
    detail["raw_fitness"] = round(f, 4)
    return f / LANE_REFERENCE[lane], detail


def evaluate(genome, seeds: tuple[int, ...] = TRAIN_SEEDS,
             scenario: str = "test") -> Evaluation:
    """Mean fitness over `seeds`, plus the two archive measures.

    Averaging over several maps is what stops the archive filling with genomes that
    exploit one rubble layout. Three is the training budget; the gate uses ten it has
    never seen.
    """
    scn = Scenario.load(scenario)
    lane = lane_of(genome)
    lane_i = LANE_INDEX[lane]
    fits, speeds, details = [], [], []
    for seed in seeds:
        m, stats = _episode(scn, genome, seed)
        mine = m.world.actuator == lane_i
        f, detail = _fitness(m, lane, mine, stats)
        fits.append(f)
        # Measured, not requested: distance actually travelled over the episode.
        speeds.append(float(stats[0][mine].mean() / EPISODE_S) if mine.any() else 0.0)
        details.append(detail)
    return Evaluation(
        fitness=float(np.mean(fits)),
        lane=lane_i,
        mean_speed=float(np.mean(speeds)),
        detail={"per_seed": details, "fitness_per_seed": fits},
    )


def evaluate_batch(genomes, seeds: tuple[int, ...] = TRAIN_SEEDS,
                   scenario: str = "test") -> list[Evaluation]:
    return [evaluate(g, seeds, scenario) for g in np.asarray(genomes, dtype=float)]


__all__ = ["EPISODE_S", "TRAIN_SEEDS", "Evaluation", "evaluate", "evaluate_batch"]
