"""Scoring a commander.

A commander is judged on **whole missions across maps it has never seen**, because that is
the only thing it is for. Tier-2 genomes are scored per lane on one fixture; a territory
policy scored that way would learn the demo map's geometry and nothing transferable.

Episodes are deliberately small and short. A full demo mission is ~4 minutes of wall time,
which at any useful population size is days per generation. Training draws compact maps
from `sim.generator` and runs them for `EPISODE_S`; the *shape* of the problem -- fog,
comms, a spreading hazard, a rescue chain -- is identical, and it is the shape the policy
has to learn rather than the size.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ...mission import Mission
from ...nodes.command import CommandParams
from ...sim.generator import sample
from ...sim.world import RESCUED

#: Shorter than the demo's 420 s. Long enough for the hazard to ignite and for a rescue
#: chain to complete, short enough to run thousands of them. With TRAIN_MAX_ROBOTS this
#: puts one evaluation near 20 s, so a 9-candidate generation on 3 maps is ~2 minutes on
#: seven workers -- roughly 300 generations overnight rather than 60.
EPISODE_S = 150.0

#: Training maps. Held-out maps for the gate are drawn from a disjoint range.
#:
#: **Six, not three, because three was memorisable.** Run 2 gained +3.66 on its three
#: training maps while its held-out score moved 0.00 across four checks -- seven free
#: parameters against three fixed maps is enough to fit the maps rather than the problem,
#: which is the exact failure `command/gate.py` was written to catch ("a commander that
#: only wins on the three maps it trained against has memorised three maps").
#:
#: The cost is linear and real: doubling the maps halves the generations a fixed budget
#: buys, ~290 down to ~150 in ten hours. That trade is worth taking -- 290 generations of
#: memorising three maps is worth less than 150 that generalise.
#:
#: Rotating a different subset each generation was considered and rejected: `run.py`
#: keeps the best-so-far with `if scores[i] > best`, and that comparison is meaningless
#: once the maps underneath it change between generations.
TRAIN_MAP_SEEDS: tuple[int, ...] = (1001, 1002, 1003, 1004, 1005, 1006)
HELD_OUT_MAP_SEEDS: tuple[int, ...] = tuple(range(2001, 2011))

#: Keeps training scenarios compact. Applied on top of whatever the generator drew.
TRAIN_MAX_ROBOTS = 160

# --- the objective ---------------------------------------------------------------------
#
# **The commander is scored on rescuing everyone, not on rescuing whoever is cheapest.**
#
# Run 1 trained against `gate.mission_score`, which pays a flat 10 per rescue. A flat rate
# is not neutral: a casualty near base costs a fraction of the travel, dig and carry time
# of one at the map edge, so a fixed reward per head quietly instructs the commander to
# harvest the near ones and ignore the rest. It did exactly that -- 121 generations drove
# `w_unexplored` to 0.05 and `w_distance` to 1.64, a policy that values ground *by its
# closeness to base*.
#
# Two changes:
#
# * **A victim's worth rises with its distance from base**, from `VICTIM_NEAR_W` at base to
#   `VICTIM_NEAR_W + VICTIM_FAR_BONUS` at the furthest casualty on that map. Normalising
#   per map keeps maps of different sizes commensurable.
# * **Every victim not rescued is subtracted at that same weight.** Note what this does and
#   does not do: since
#       sum(rescued w) - sum(unrescued w) == 2*sum(rescued w) - sum(all w)
#   and `sum(all w)` is a constant for a given map, this is an *affine* transform of the
#   old objective. CMA-ES ranks candidates, so it **cannot change which policy the search
#   prefers**. It is kept because it makes the number honest to read -- a negative score
#   means the swarm left more value on the ground than it brought home -- and because it
#   would start to matter if candidates were ever compared across different map sets. The
#   distance weighting is the half that actually redirects the search.
#
# This deliberately does *not* touch `gate.mission_score`: that function is the shared
# yardstick for the MAP-Elites gate too, and a training reward and an acceptance metric
# are allowed to differ. The gate still judges on rescues, and must.

#: A casualty at base is worth this; one at the furthest point of the map, this plus the
#: bonus. 1.0/1.0 means the far edge is worth exactly twice the near edge -- a real
#: gradient without making near casualties worthless.
VICTIM_NEAR_W = 1.0
VICTIM_FAR_BONUS = 1.0

#: Term scales. The victim term is normalised to [-1, 1] and dominates on purpose: the
#: others are proxies for it and must never be worth more than the thing they proxy.
W_VICTIMS, W_FOUND, W_EXPLORED, W_LOST = 100.0, 10.0, 10.0, 5.0


def _victim_weights(world) -> np.ndarray:
    """Per-victim worth, rising with distance from base. Normalised per map."""
    base = np.asarray(world.scn.base, dtype=np.float64)
    pos = np.array([v.pos for v in world.victims], dtype=np.float64)
    if len(pos) == 0:
        return np.zeros(0)
    d = np.linalg.norm(pos - base, axis=1)
    return VICTIM_NEAR_W + VICTIM_FAR_BONUS * (d / max(float(d.max()), 1e-9))


def command_score(world, card) -> float:
    """The commander's training objective. See the note above.

    **Every term is scale-free**, which is the half of this that actually changes the
    search. The generator draws maps with different casualty counts, and an unnormalised
    sum makes a 60-casualty map contribute six times the magnitude of a 10-casualty one --
    so the mean over three maps was mostly the biggest map's opinion, and the other two
    were rounding error. Dividing by each map's own total makes the maps commensurable,
    which is what "rescue everyone" has to mean when "everyone" differs per map.
    """
    w = _victim_weights(world)
    if len(w) == 0:
        return 0.0
    rescued = np.array([v.state == RESCUED for v in world.victims], dtype=bool)
    # In [-1, 1]: +1 is everyone home, -1 is nobody, and the zero crossing is the point
    # where the weighted haul balances what was left behind.
    victims = float(w[rescued].sum() - w[~rescued].sum()) / float(w.sum())
    found = card.victims_found / max(card.victims_total, 1)
    lost = card.robots_lost / max(card.robots_total, 1)
    return (
        W_VICTIMS * victims
        + W_FOUND * found
        + W_EXPLORED * card.ground_explored_frac
        - W_LOST * lost
    )


@dataclass
class CommandEval:
    score: float
    rescued: float
    explored: float
    per_map: list[dict]


def _shrink(scn):
    """Cap swarm size so an episode is seconds, not minutes. Proportions are preserved."""
    import dataclasses

    if scn.n_robots <= TRAIN_MAX_ROBOTS:
        return scn
    f = TRAIN_MAX_ROBOTS / scn.n_robots
    counts = {k: max(4, int(v * f)) for k, v in scn.lane_counts.items()}
    return dataclasses.replace(scn, lane_counts=counts,
                               robots_per_lane=sum(counts.values()) // 4)


def evaluate(params, map_seeds=TRAIN_MAP_SEEDS) -> CommandEval:
    """Mean mission score over `map_seeds`, each a freshly generated map.

    Tier 3's advisor is off: this measures the *commander*, and mixing in a language model
    whose latency varies with machine load would put wall-clock noise into the objective.
    """
    p = params if isinstance(params, CommandParams) else CommandParams.from_vector(params)
    scores, detail = [], []
    for seed in map_seeds:
        scn = _shrink(sample(seed))
        m = Mission(scn, seed, hivemind=False, command=True)
        m.commander.p = p
        m.commander._cut(m.world)
        w = m.world
        while w.t < EPISODE_S and not w.done:
            m.tick()
        card = w.scorecard()
        scores.append(command_score(w, card))
        detail.append({
            "map": seed,
            "rescued": card.victims_rescued,
            "total": card.victims_total,
            "explored": round(card.ground_explored_frac, 4),
        })
    return CommandEval(
        score=float(np.mean(scores)),
        rescued=float(np.mean([d["rescued"] / max(d["total"], 1) for d in detail])),
        explored=float(np.mean([d["explored"] for d in detail])),
        per_map=detail,
    )


def baseline(map_seeds=TRAIN_MAP_SEEDS) -> CommandEval:
    """The hand-set commander -- what a trained one has to beat before it ships."""
    return evaluate(CommandParams(), map_seeds)
