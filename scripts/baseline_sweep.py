#!/usr/bin/env python
"""Multi-seed before-picture for the relay work, both Tier-3 arms.

    uv run python scripts/baseline_sweep.py
    uv run python scripts/baseline_sweep.py --seeds 42 43 44 45 --workers 4
    uv run python scripts/baseline_sweep.py --label after --out runs/sweep_after.json

Why this exists when `diagnose.py` already reports seed 42 in far more detail:

* **One seed cannot carry the decision.** M-36 is a recorded case of a correct-looking
  search fix measuring as a regression, and M-26's Tier-3 result was one seed wide and
  honestly labelled "promising, not demonstrated". A single before/after pair on seed 42
  would repeat that mistake with a different variable.
* **The Tier-3 arm is unresolved and must not move underneath the measurement.** M-26
  had the scripted rung ahead on seed 42; a later build is behind on the same seed
  (16 rescued on / 18 off). Running both arms separates a relay change from a Tier-3
  change instead of confounding them.
* **The recorded baselines are from another machine.** M-38/M-39's numbers do not
  reproduce here with no sim code changed in between, so the before-picture has to be
  taken on the machine the demo runs on.

Determinism is per-process and verified (seed 42 hashes `eb39e6a4ba22080b` on two
independent runs), so each cell is one run rather than an average of repeats. Both
`victims_found` and `comms_reach_m` are reported because M-39's chain --
rescued <- discovery <- explored <- comms reach -- means a relay fix should move reach
first and rescues last. A fix that moves rescues without moving reach did something else.
"""

from __future__ import annotations

import os

# Before numpy. Each cell is a whole mission in its own process; BLAS threads inside
# them only contend for the same cores the pool is already saturating.
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

import argparse  # noqa: E402
import dataclasses  # noqa: E402
import json  # noqa: E402
import multiprocessing as mp  # noqa: E402
import time  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

from swarmmind.mission import Mission  # noqa: E402
from swarmmind.sim.robot import LANE_INDEX, OUT_OF_COMMS  # noqa: E402
from swarmmind.sim.scenario import Scenario  # noqa: E402

DEFAULT_SEEDS = (42, 43, 44, 45)

#: The whole-system gate, PLAN.md §6.1. Delivery is averaged over the sweep because it is
#: the rescue chain's own number and is stable across seeds (57-61%); discovery is pinned
#: to one reference seed because it varies 25-36 across seeds and a single seed is
#: deterministic, so the bar can sit close without absorbing noise.
GATE_DELIVERY_PCT = 55.0
#: Of 110 casualties. Re-derived twice as the scenario moved: 30 of 120, then 40
#: of 80, now 55 of 110. Measured ~75 found on seed 42 with Tier 3 off, so the floor keeps
#: the same ~25% headroom (M-47, M-52). Rescale it whenever `victims.count` changes -- a
#: discovery floor in absolute casualties is meaningless against a different denominator.
GATE_DISCOVERY = 55
GATE_SEED = 42


def _mechanism(m: Mission) -> dict:
    """The two numbers M-39 says bind everything downstream, read at end of mission.

    Ground truth, deliberately: this is a measurement harness, not swarm-side code --
    the same exemption `scripts/diagnose.py` documents.
    """
    w = m.world
    alive = w.status <= OUT_OF_COMMS
    base = np.asarray(w.scn.base, dtype=float)
    linked = alive & w.in_comms
    reach = (float(np.linalg.norm(w.pos[linked] - base, axis=1).max())
             if linked.any() else 0.0)
    is_relay = alive & (w.actuator == LANE_INDEX["antenna"])
    on_post = sum(1 for i in np.nonzero(is_relay)[0]
                  if (a := m.executor.assignment[int(i)]) is not None and a.kind == "relay")
    return {"comms_reach_m": round(reach, 1),
            "relays_on_post": int(on_post),
            "relays_alive": int(is_relay.sum())}


def _cell(job: tuple[str, int, bool]) -> dict:
    scenario, seed, hivemind = job
    scn = Scenario.load(scenario)
    m = Mission(scn, seed, hivemind=hivemind)
    card = m.run()
    return {"seed": seed, "hivemind": hivemind, "hash": card.hash(),
            **dataclasses.asdict(card), **_mechanism(m)}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--seeds", type=int, nargs="+", default=list(DEFAULT_SEEDS))
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--label", default="before")
    ap.add_argument("--out", default="runs/sweep.json")
    args = ap.parse_args()

    jobs = [(args.scenario, s, hm) for s in args.seeds for hm in (False, True)]
    print(f"  {args.label}: {len(jobs)} missions, {args.workers} workers "
          f"-- seeds {args.seeds}, scenario {args.scenario}")

    t0 = time.perf_counter()
    ctx = mp.get_context("spawn")
    with ctx.Pool(args.workers) as pool:
        rows = pool.map(_cell, jobs)
    wall = time.perf_counter() - t0

    rows.sort(key=lambda r: (r["seed"], r["hivemind"]))
    print("\n  seed  tier3   resc  found  expl%   lost  reach_m  posts  hash")
    print("  " + "-" * 62)
    for r in rows:
        print(f"  {r['seed']:>4}  {'on ' if r['hivemind'] else 'off':>5}   "
              f"{r['victims_rescued']:>4}  {r['victims_found']:>5}  "
              f"{100 * r['ground_explored_frac']:>5.1f}  {r['robots_lost']:>5}  "
              f"{r['comms_reach_m']:>7.0f}  "
              f"{r['relays_on_post']:>2}/{r['relays_alive']:<2}  {r['hash']}")

    for arm in (False, True):
        a = [r for r in rows if r["hivemind"] is arm]
        n = len(a)
        resc = sum(r["victims_rescued"] for r in a)
        found = sum(r["victims_found"] for r in a)
        print(f"\n  tier3 {'on ' if arm else 'off'}  mean over {n} seeds:  "
              f"rescued {resc / n:.2f}   found {found / n:.2f}   "
              f"explored {100 * sum(r['ground_explored_frac'] for r in a) / n:.2f}%   "
              f"reach {sum(r['comms_reach_m'] for r in a) / n:.0f} m   "
              f"**delivery {100 * resc / max(1, found):.1f}%**")

    # The whole-system gate, PLAN.md §6.1. Reported here rather than left to be worked out
    # by hand, because a bar nobody evaluates is not a bar. Tier 3 off is the arm that
    # counts: invariant #1 says the swarm has to stand up with the hivemind silent.
    off = [r for r in rows if not r["hivemind"]]
    if args.scenario != "demo":
        # The bars are demo-scenario numbers. The `test` fixture is 16 robots and 8
        # casualties; scoring it against them reports FAIL for a working swarm.
        print(f"\n  gate not evaluated -- PLAN §6.1 is defined on `demo`, "
              f"this was `{args.scenario}`")
    elif off:
        f_tot = sum(r["victims_found"] for r in off)
        delivery = 100.0 * sum(r["victims_rescued"] for r in off) / max(1, f_tot)
        ref = next((r for r in off if r["seed"] == GATE_SEED), None)
        print("\n  gate (PLAN §6.1, tier 3 off)")
        print(f"    delivery  {delivery:5.1f}%  >= {GATE_DELIVERY_PCT}%      "
              f"{'PASS' if delivery >= GATE_DELIVERY_PCT else 'FAIL'}")
        if ref is not None:
            d = ref["victims_found"]
            print(f"    discovery {d:5d}    >= {GATE_DISCOVERY} of {ref['victims_total']} on seed {GATE_SEED}   "
                  f"{'PASS' if d >= GATE_DISCOVERY else 'FAIL'}")
        else:
            print(f"    discovery      seed {GATE_SEED} not in this sweep -- not evaluated")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(
        {"label": args.label, "scenario": args.scenario, "seeds": args.seeds,
         "wall_seconds": round(wall, 1), "rows": rows}, indent=2), encoding="utf-8")
    print(f"\n  {wall / 60:.1f} min -> {out}")


if __name__ == "__main__":
    main()
