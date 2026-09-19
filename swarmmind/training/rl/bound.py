"""Is there anything to learn? Three arms on the demo map, before any long training run.

    python -m swarmmind.training.rl.bound --out bound.jsonl      # the demo's four maps

The project has now seen two trained components whose action had no headroom (M-74's
detector; the contact-inspection probe measured before this module existed). This is
the cheap question first: if the classical staging rule does not move rescues, a learned
version of it has very little to find, and the Kaggle hours belong elsewhere.

    shipped              the demo as it is on main
    routing              + exact routing to collection points (`control/zone_routing.py`)
    routing+staging      + the heuristic unit policy (`control/unit_policy.py`)

Results are appended one mission at a time, so a session that dies keeps what it finished.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

from ...control import unit_policy as up
from ...mission import Mission
from ...sim.scenario import Scenario
from ..gate import mission_score

#: Every arm is (zone routing, unit-policy mode, Tier 3). The first three answer "is there
#: anything to learn" with Tier 3 off, which is how every training arm is measured. The
#: `tier3*` arms answer a different question: **what the demo would actually run.** The gate
#: measures routing with Tier 3 off and Tier 3 without routing, so the combination -- the
#: configuration on stage if routing is adopted -- is unmeasured, and two changes that each
#: help alone have cancelled in this project before (M-66, M-72).
ARMS = {
    "shipped": dict(zone_routing=False, mode="default", hivemind=False),
    "routing": dict(zone_routing=True, mode="default", hivemind=False),
    "routing+staging": dict(zone_routing=True, mode="heuristic", hivemind=False),
    # --- what the demo runs: Tier 3 live (scripted rung, deterministic) ------------------
    "tier3": dict(zone_routing=False, mode="default", hivemind=True),
    "tier3+routing": dict(zone_routing=True, mode="default", hivemind=True),
    "tier3+routing+policy": dict(zone_routing=True, mode="learned", hivemind=True),
}


def _one(job):
    for v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[v] = "1"
    arm, seed, scenario, policy_path = job
    cfg = ARMS[arm]
    ctl = None
    if cfg["mode"] == "learned":
        from .ppo import UnitPolicy

        ctl = up.UnitController("learned", UnitPolicy.load_npz(policy_path))
    elif cfg["mode"] != "default":
        ctl = up.UnitController(cfg["mode"])
    # The scripted rung, not a model: deterministic, no network, no wall-clock dependence
    # (TECHNICAL §12), so this comparison is reproducible like every other row here.
    m = Mission(Scenario.load(scenario), seed, hivemind=cfg["hivemind"],
                scripted_hivemind=cfg["hivemind"], unit_policy=ctl,
                zone_routing=cfg["zone_routing"])
    card = m.run()
    return {"arm": arm, "seed": seed, "rescued": card.victims_rescued,
            "found": card.victims_found, "explored": card.ground_explored_frac,
            "lost": card.robots_lost, "score": round(mission_score(card), 3),
            "wall_s": card.wall_seconds}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="headroom bound for the unit policy")
    ap.add_argument("--seeds", type=int, nargs="+", default=None,
                    help="default: the scenario's demo_seeds (the maps the demo plays)")
    ap.add_argument("--arms", nargs="+", default=list(ARMS))
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--out", type=Path, default=Path("runs/rl/bound.jsonl"))
    ap.add_argument("--unit-policy", type=Path, default=Path("runs/rl/policy_best.npz"),
                    help="weights for any arm whose mode is 'learned'")
    args = ap.parse_args(argv)
    if any(ARMS[a]["mode"] == "learned" for a in args.arms) and not args.unit_policy.exists():
        raise SystemExit(f"no trained policy at {args.unit_policy}; drop the learned arm "
                         f"or attach the training output")
    if not args.seeds:
        args.seeds = list(Scenario.load(args.scenario).demo_seeds)
        if not args.seeds:
            raise SystemExit(f"scenario {args.scenario!r} declares no demo_seeds; pass --seeds")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    done = set()
    if args.out.exists():
        for line in args.out.read_text(encoding="utf-8").splitlines():
            r = json.loads(line)
            done.add((r["arm"], r["seed"]))
    jobs = [(a, s, args.scenario, str(args.unit_policy))
            for s in args.seeds for a in args.arms if (a, s) not in done]
    print(f"  {len(jobs)} missions to run ({len(done)} already recorded), "
          f"{args.workers} workers", flush=True)

    if args.workers > 1 and jobs:
        import multiprocessing as mp

        if getattr(sys.modules.get("__main__"), "__file__", None) is None:
            raise RuntimeError("run with `python -m`, not from a REPL (M-30)")
        pool = mp.get_context("spawn").Pool(args.workers)
        stream = pool.imap_unordered(_one, jobs)
    else:
        pool, stream = None, map(_one, jobs)
    try:
        for r in stream:
            with open(args.out, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(r) + "\n")
            print(json.dumps(r), flush=True)
    finally:
        if pool is not None:
            pool.close()
            pool.join()

    rows = [json.loads(x) for x in args.out.read_text(encoding="utf-8").splitlines()]
    by = defaultdict(dict)
    for r in rows:
        by[r["arm"]][r["seed"]] = r
    # Only the seeds asked for: an attached older bound.jsonl may carry other maps.
    seeds = (sorted(set(args.seeds) & set.intersection(*[set(v) for v in by.values()]))
             if by else [])
    print(f"\n  paired over seeds {seeds}")
    print(f"  {'arm':18s} {'rescued':>8s} {'found':>7s} {'lost':>6s} {'score':>8s}  per-seed rescued")
    for arm in args.arms:
        if arm not in by:
            continue
        rs = [by[arm][s] for s in seeds]
        print(f"  {arm:18s} {np.mean([r['rescued'] for r in rs]):8.2f} "
              f"{np.mean([r['found'] for r in rs]):7.2f} {np.mean([r['lost'] for r in rs]):6.2f} "
              f"{np.mean([r['score'] for r in rs]):8.2f}  {[r['rescued'] for r in rs]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
