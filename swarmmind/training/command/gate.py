"""Does the trained commander ship?

Held-out maps only. `evaluate.TRAIN_MAP_SEEDS` is (1001-1003) and this uses (2001-2010),
disjoint by construction and asserted in tests -- a commander that only wins on the three
maps it trained against has memorised three maps.

Three arms, because two is not enough to interpret:

* **none** -- no commander at all. `Mission(command=False)`, the pre-Tier-3a swarm.
* **heuristic** -- the hand-set `CommandParams`.
* **trained** -- whatever `runs/command/best.npz` holds.

The middle arm is the one people forget, and it is the one that matters: at the time of
writing the hand-set commander scores *worse* than no commander at all, so "trained beats
heuristic" would be a meaningless victory. Trained has to beat **none**.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from ...mission import Mission
from ...nodes.command import CommandParams
from ...sim.generator import sample
from ..gate import MARGIN, mission_score
from .evaluate import EPISODE_S, HELD_OUT_MAP_SEEDS, TRAIN_MAP_SEEDS, _shrink


def _run(map_seed: int, params: CommandParams | None) -> dict:
    scn = _shrink(sample(map_seed))
    m = Mission(scn, map_seed, hivemind=False, command=params is not None)
    if params is not None:
        m.commander.p = params
        m.commander._cut(m.world)
    w = m.world
    while w.t < EPISODE_S and not w.done:
        m.tick()
    card = w.scorecard()
    return {"map": map_seed, "score": mission_score(card),
            "rescued": card.victims_rescued, "total": card.victims_total,
            "explored": round(card.ground_explored_frac, 4)}


def arm(name: str, params: CommandParams | None, seeds) -> dict:
    runs = [_run(s, params) for s in seeds]
    return {
        "name": name,
        "score": float(np.mean([r["score"] for r in runs])),
        "rescued_frac": float(np.mean([r["rescued"] / max(r["total"], 1) for r in runs])),
        "explored": float(np.mean([r["explored"] for r in runs])),
        "per_map": runs,
    }


def load_trained(path: Path) -> CommandParams | None:
    f = Path(path) / "best.npz"
    if not f.exists():
        return None
    return CommandParams.from_vector(np.load(f)["params"])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="commander gate, held-out maps")
    ap.add_argument("--run", type=Path, default=Path("runs/command"))
    ap.add_argument("--maps", type=int, nargs="+", default=list(HELD_OUT_MAP_SEEDS))
    ap.add_argument("--report", type=Path, default=None)
    args = ap.parse_args(argv)

    assert not set(args.maps) & set(TRAIN_MAP_SEEDS), "gate maps overlap training maps"
    trained = load_trained(args.run)

    arms = [arm("none", None, args.maps), arm("heuristic", CommandParams(), args.maps)]
    if trained is not None:
        arms.append(arm("trained", trained, args.maps))
    else:
        print(f"  no trained commander at {args.run}/best.npz -- comparing the other two")

    print(f"\n  {len(args.maps)} held-out maps: {args.maps}")
    print(f"  {'arm':>10} {'score':>9} {'rescued':>9} {'explored':>9}")
    for a in arms:
        print(f"  {a['name']:>10} {a['score']:>9.2f} {a['rescued_frac']:>8.0%} "
              f"{a['explored']:>8.0%}")

    control = arms[0]["score"]
    verdict = "no trained commander to judge"
    ships = False
    if trained is not None:
        ratio = arms[-1]["score"] / control if control else float("inf")
        ships = ratio >= MARGIN
        verdict = (f"trained commander ships ({ratio:.2f}x no-commander)" if ships else
                   f"no commander ships; trained reached only {ratio:.2f}x, "
                   f"under the {MARGIN:.2f}x bar")
    print(f"\n  {verdict}")

    if args.report:
        args.report.write_text(json.dumps(
            {"arms": arms, "maps": args.maps, "ships": ships, "verdict": verdict},
            indent=2))
        print(f"  -> {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
