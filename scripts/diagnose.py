#!/usr/bin/env python
"""Where does the rescue rate actually go? -- the measurement M-34/35/36 need re-taken.

    uv run python scripts/diagnose.py                     # demo, seed 42
    uv run python scripts/diagnose.py --scenario test --seed 11

M-35 established the chain that bounds the mission:

    rescued <- discovery <- explored fraction <- comms coverage <- relay deployment

and separately, delivery needs enough clock left to carry a casualty home. Every
intervention that skipped a link failed, twice measurably (M-36).

**Those numbers are stale.** They predate the rotor chassis, store-and-forward (M-37),
`posts_per_chain` 1 -> 8, and the evolved roster from MAP-Elites run 0. This script
re-measures every link in one pass so the next change is aimed at whichever one binds
*now* rather than whichever bound in an earlier build.

It reads ground truth deliberately -- it is a diagnostic, not swarm-side code, and
`scripts/` is outside the `GUARDED` trees in `tests/test_no_ground_truth_leak.py`. Death
causes come off `/swarm/events` rather than being recomputed, so they are exactly what the
dashboard would have shown.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from swarmmind.contracts import topics
from swarmmind.mission import Mission
from swarmmind.sim.robot import LANE_INDEX, OUT_OF_COMMS
from swarmmind.sim.scenario import Scenario


def _sample(m: Mission, deaths: dict[str, int]) -> dict:
    w = m.world
    alive = w.status <= OUT_OF_COMMS
    base = np.asarray(w.scn.base, dtype=float)

    # How far the *connected* component actually reaches. This is the number M-36 said
    # everything else dies against: ground beyond it can be driven over but not reported.
    linked = alive & w.in_comms
    reach = (float(np.linalg.norm(w.pos[linked] - base, axis=1).max())
             if linked.any() else 0.0)

    # Relays holding a post, against relays that exist. M-35 measured 14 of 192.
    is_relay = alive & (w.actuator == LANE_INDEX["antenna"])
    on_post = sum(1 for i in np.nonzero(is_relay)[0]
                  if (a := m.executor.assignment[int(i)]) is not None and a.kind == "relay")

    return {
        "t": round(float(w.t), 1),
        "explored_pct": round(100.0 * float(w.explored[w.passable].mean()), 2),
        "found": int(sum(1 for v in w.victims if v.state >= 1)),
        "rescued": int(w.victims_rescued),
        "alive": int(alive.sum()),
        "in_comms_pct": round(100.0 * float(w.in_comms[alive].mean()), 1) if alive.any() else 0.0,
        "via_relay": int(w.comms_via_relay.sum()),
        "comms_reach_m": round(reach, 1),
        "relays_on_post": on_post,
        "relays_alive": int(is_relay.sum()),
        "deaths": dict(deaths),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="diagnose the rescue-rate bottleneck")
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--every", type=float, default=30.0, help="sample period, sim seconds")
    ap.add_argument("--out", type=Path, default=Path("runs/diagnose.json"))
    args = ap.parse_args(argv)

    scn = Scenario.load(args.scenario)
    m = Mission(scn, args.seed, hivemind=False)
    w = m.world

    # Keyed by robot id, not counted per event: the scripted fault emits its own
    # `robot_destroyed` and *then* calls `world._kill`, which emits a second one for the
    # same robot. Counting events reports two deaths for one casualty (and the dashboard
    # shows that robot dying twice -- worth fixing at the source).
    cause_of: dict[str, str] = {}

    def on_event(ev):
        if ev.get("kind") != "robot_destroyed":
            return
        rid = ev.get("robot", "?")
        text = ev.get("text", "")
        if " destroyed by " in text:
            # world._kill's form: "<id> destroyed by <cause> in <sector>". Authoritative.
            cause_of[rid] = text.split(" destroyed by ")[1].split(" in ")[0]
        else:
            cause_of.setdefault(rid, "scripted fault")

    m.bus.subscribe(topics.SWARM_EVENTS, on_event)

    def deaths() -> dict[str, int]:
        out: dict[str, int] = {}
        for c in cause_of.values():
            out[c] = out.get(c, 0) + 1
        return out

    print(f"  {args.scenario} seed {args.seed}   {w.n} robots   "
          f"{len(w.victims)} casualties   {scn.mission_duration_s:.0f} s")
    print(f"  map {w.shape[1] * w.cell:.0f}x{w.shape[0] * w.cell:.0f} m   "
          f"base_radius {scn.comms.base_radius:.0f}   relay_radius {scn.comms.relay_radius:.0f}")
    hdr = (f"\n  {'t':>5} {'expl%':>6} {'found':>6} {'resc':>5} {'alive':>6} "
           f"{'comms%':>7} {'relay+':>7} {'reach_m':>8} {'posts':>10}")
    print(hdr)
    print("  " + "-" * (len(hdr) - 3))

    samples = [_sample(m, deaths())]
    next_t = args.every
    while not w.done and w.t < scn.mission_duration_s:
        m.tick()
        if w.t >= next_t:
            s = _sample(m, deaths())
            samples.append(s)
            next_t += args.every
            print(f"  {s['t']:>5.0f} {s['explored_pct']:>6.2f} {s['found']:>6} "
                  f"{s['rescued']:>5} {s['alive']:>6} {s['in_comms_pct']:>7.1f} "
                  f"{s['via_relay']:>7} {s['comms_reach_m']:>8.0f} "
                  f"{s['relays_on_post']:>5}/{s['relays_alive']:<4}", flush=True)

    card = m.sim.world.scorecard()
    final = samples[-1]

    # The arithmetic that decides whether the rescue chain or discovery is the ceiling:
    # rescuing *everything ever found* is an upper bound on any downstream improvement.
    total = max(len(w.victims), 1)
    print(f"\n  scorecard: rescued {card.victims_rescued}/{total}  "
          f"found {card.victims_found}/{total}  explored {card.ground_explored_frac:.1%}  "
          f"lost {card.robots_lost}/{w.n}  mttr {card.mean_time_to_rescue:.0f}s")
    print(f"  deaths by cause: {final['deaths'] or 'none'}")
    print(f"  comms reached {final['comms_reach_m']:.0f} m of a "
          f"{float(np.hypot(w.shape[1] * w.cell, w.shape[0] * w.cell)):.0f} m diagonal")
    print(f"  relays on post at end: {final['relays_on_post']}/{final['relays_alive']}")
    print(f"\n  CEILING: rescuing 100% of everything ever found = "
          f"{card.victims_found}/{total} = {card.victims_found / total:.1%}")
    print("  -> if that is under the gate, discovery is the constraint, not the "
          "rescue chain.")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({
        "scenario": args.scenario, "seed": args.seed,
        "n_robots": int(w.n), "n_victims": total,
        "samples": samples,
        "scorecard": {k: v for k, v in card.__dict__.items()},
    }, indent=2), encoding="utf-8")
    print(f"\n  -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
