#!/usr/bin/env python
"""Find the detector's sweet spot: recall on the hard casualties without drowning in rubble.

    uv run python scripts/sweep_detector.py                   # demo, seed 44
    uv run python scripts/sweep_detector.py --seed 42 --full   # then re-run winners live

**Why this is not just a parameter loop over missions.** A mission is ~4.5 minutes, and a
3x3x2 grid is 18 of them per seed. Instead every candidate detector runs on the *same*
camera frames as the shipped one, inside a single mission: the baseline detector drives
the simulation, so the trajectory is identical for every config, and the sweep measures
only what the configs disagree about. One mission prices the whole grid.

What it scores, per config, is the thing the tracker actually gates on -- not raw pixels:

  recall     casualties that would have become *promotable* (best_conf >= min_conf AND
             >= min_views distinct vantage points), replaying the tracker's own arithmetic
  phantoms   detections landing more than `merge_radius` from any real casualty, as a
             fraction of all detections -- every one of these costs an investigate trip
  buried     recall restricted to buried casualties, which is where the misses are:
             3 of the 4 that the shipped detector never fired on at all -- gate
             attribution on seed 44, 15 misses, recorded with this script's landing

The sweet spot is the knee of recall against phantoms. `--full` then re-runs the top
candidates as real missions, because frame-level recall is a prediction and `rescued` is
the only number that settles it.

Ground truth is read for scoring only, after the fact. `scripts/` is outside the GUARDED
trees in tests/test_no_ground_truth_leak.py; nothing here feeds the swarm.
"""

from __future__ import annotations

import argparse
import itertools

import numpy as np

from swarmmind.mission import Mission
from swarmmind.perception.classical import ClassicalVictimDetector
from swarmmind.perception.tracker import VictimReportTracker
from swarmmind.sim.scenario import Scenario

#: The grid. `block` is first on purpose: a buried casualty renders as a *small* patch
#: (M-7), and a smaller pooling block raises its fill without widening the colour band
#: at all -- it is the one knob that buys recall on small targets without buying rubble.
#: `max_wide` is the extent test that rejects a casualty lying inside warm rubble, which
#: is exactly the population being missed, so it is swept second. `min_fill` last: it is
#: the bluntest of the three and trades recall against phantoms almost one for one.
GRID = {
    "block": (6, 4),
    "max_wide": (0.55, 0.70),
    "min_fill": (0.34, 0.26, 0.20),
}


class Scorer:
    """Replays the tracker's promotion arithmetic for one detector config."""

    def __init__(self, tracker: VictimReportTracker, n_victims: int) -> None:
        self.tr = tracker
        self.best = np.zeros(n_victims)
        self.views: list[list[np.ndarray]] = [[] for _ in range(n_victims)]
        self.n_det = 0
        self.n_phantom = 0

    def add(self, world, dets, vpos: np.ndarray) -> None:
        for d in dets:
            self.n_det += 1
            dv = np.linalg.norm(vpos - np.asarray(d.pos), axis=1)
            k = int(np.argmin(dv))
            if dv[k] > self.tr.merge_radius:
                self.n_phantom += 1
                continue
            eff = float(d.conf) * self.tr._range_trust(float(d.range_m))
            self.best[k] = max(self.best[k], eff)
            obs = world.pos[d.robot]
            if all(float(np.linalg.norm(obs - u)) > self.tr.parallax
                   for u in self.views[k]):
                self.views[k].append(obs.copy())

    def promotable(self) -> np.ndarray:
        nv = np.array([len(v) for v in self.views])
        return (self.best >= self.tr.min_conf) & (nv >= self.tr.min_views)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="sweep the classical detector")
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--seed", type=int, default=44)
    ap.add_argument("--full", action="store_true",
                    help="re-run the top candidates as real missions (slow)")
    ap.add_argument("--top", type=int, default=3)
    args = ap.parse_args(argv)

    scn = Scenario.load(args.scenario)
    m = Mission(scn, args.seed, hivemind=False)
    w = m.world
    buried = np.array([v.buried for v in w.victims], dtype=bool)

    combos = [dict(zip(GRID, c, strict=True)) for c in itertools.product(*GRID.values())]
    cands = {tuple(sorted(c.items())): (ClassicalVictimDetector(**c),
                                        Scorer(m.tracker, len(w.victims)))
             for c in combos}
    print(f"  {args.scenario} seed {args.seed}: {len(w.victims)} casualties "
          f"({int(buried.sum())} buried), {len(cands)} detector configs on shared frames")

    shipped = m.detector.detect

    def detect(world, rig, frames, idx):
        # Ground truth moves as casualties are carried, so re-read it each pass.
        live = np.array([v.pos for v in world.victims], dtype=float)
        for det, sc in cands.values():
            sc.add(world, det.detect(world, rig, frames, idx), live)
        return shipped(world, rig, frames, idx)

    m.detector.detect = detect

    while not w.done and w.t < scn.mission_duration_s:
        m.tick()

    rows = []
    for key, (_, sc) in cands.items():
        p = sc.promotable()
        rows.append((dict(key), int(p.sum()), int((p & buried).sum()),
                     sc.n_phantom / max(1, sc.n_det), sc.n_det))
    rows.sort(key=lambda r: (-r[1], r[3]))

    print(f"\n  {'block':>5} {'max_wide':>9} {'min_fill':>9} {'promotable':>11} "
          f"{'of buried':>10} {'phantom%':>9} {'detections':>11}")
    for cfg, n, nb, ph, nd in rows:
        print(f"  {cfg['block']:>5} {cfg['max_wide']:>9.2f} {cfg['min_fill']:>9.2f} "
              f"{n:>7}/{len(w.victims):<3} {nb:>6}/{int(buried.sum()):<3} "
              f"{100*ph:>8.1f}% {nd:>11}")
    print("\n  Sweet spot = the knee: the last config that adds casualties faster than\n"
          "  it adds phantoms. Every phantom is an investigate trip a robot does not\n"
          "  spend searching, so recall bought past the knee is paid for in coverage.")

    if args.full:
        print(f"\n  re-running the top {args.top} as real missions "
              f"(~4.5 min each) -- frame recall is a prediction, `rescued` is the answer")
        for cfg, *_ in rows[: args.top]:
            mm = Mission(scn, args.seed, hivemind=False)
            mm.detector = ClassicalVictimDetector(**cfg)
            ww = mm.world
            while not ww.done and ww.t < scn.mission_duration_s:
                mm.tick()
            c = ww.scorecard()
            print(f"    {cfg}  found {c.victims_found:>3}  rescued {c.victims_rescued:>3}"
                  f"  explored {c.ground_explored_frac:.1%}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
