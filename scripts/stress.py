#!/usr/bin/env python
"""Robot-count sweep -- the measurement that sets N.

Method (MVP spec section 2): find the largest N whose DemoSim real-time factor holds at
or above 0.8x, then ship floor(0.8 * ceiling). Re-run with Godot and llama-server
resident; the ceiling under real memory pressure is the one that matters on 8 GB.

    uv run python scripts/stress.py
    uv run python scripts/stress.py --counts 16 32 64 128 --seconds 60
"""

from __future__ import annotations

import argparse
import dataclasses
import resource
import sys
import time

from swarmmind.mission import Mission
from swarmmind.sim.scenario import Scenario

DEFAULT_COUNTS = (16, 32, 48, 64, 96, 128, 192, 256)
RTF_FLOOR = 0.8


def _peak_rss_mb() -> float:
    """Peak RSS in MB. Units of ru_maxrss are platform-defined: bytes on macOS,
    kilobytes on Linux. Guessing from magnitude misreports small processes -- a 37 MB
    process on macOS looks like 38016 "MB" -- so branch on the platform."""
    r = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return r / (1024 * 1024) if sys.platform == "darwin" else r / 1024


def measure(scn: Scenario, n_total: int, seconds: float, seed: int, realtime: bool) -> dict:
    """Throughput at ``n_total`` robots, running the **real** stack.

    Measured through Mission, so the number includes Tier 1 steering, flow-field
    navigation, Tier 2 task execution and allocation -- not just world physics. A sweep
    against a placeholder controller measures the floor of cost and overstates the
    ceiling by several times.

    Measured on FastSim, not DemoSim: DemoSim *sleeps* to the wall clock, so timing it
    costs one second of real time per simulated second and can never report better than
    1.0x. The headroom number that matters is FastSim's -- DemoSim sustains real time
    exactly when FastSim's RTF exceeds 1.0. ``--realtime`` additionally runs DemoSim to
    check for scheduling jitter, which is the one thing FastSim cannot show.
    """
    scn = dataclasses.replace(scn, robots_per_lane=max(1, n_total // 4))
    steps = int(seconds * scn.rates.tick_hz)

    m = Mission(scn, seed)
    t0 = time.perf_counter()
    for _ in range(steps):
        m.tick()
    fast_wall = time.perf_counter() - t0

    demo_rtf = float("nan")
    if realtime:
        d = Mission(scn, seed, realtime=True)
        for _ in range(steps):
            d.tick()
        demo_rtf = d.sim.rtf

    return {
        "n": scn.n_robots,
        "fast_steps_per_s": steps / fast_wall,
        "fast_rtf": (steps / scn.rates.tick_hz) / fast_wall,
        "demo_rtf": demo_rtf,
        "ms_per_tick": 1000.0 * fast_wall / steps,
        "peak_rss_mb": _peak_rss_mb(),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--counts", type=int, nargs="+", default=list(DEFAULT_COUNTS))
    ap.add_argument("--seconds", type=float, default=150.0,
                    help="sim seconds per count. Must exceed hazard.ignite_t or the "
                         "sample misses the most expensive phase of the mission.")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--realtime", action="store_true",
                    help="also run DemoSim to check scheduling jitter (slow: 1 s wall per sim s)")
    args = ap.parse_args()

    scn = Scenario.load(args.scenario)
    print(f"{'N':>5} {'ms/tick':>9} {'RTF':>9} {'steps/s':>9} {'RSS MB':>8}")
    print("-" * 46)
    ceiling = 0
    for n in args.counts:
        r = measure(scn, n, args.seconds, args.seed, args.realtime)
        ok = r["fast_rtf"] >= RTF_FLOOR
        flag = "" if ok else "  <- below 0.8x"
        print(f"{r['n']:>5} {r['ms_per_tick']:>9.2f} {r['fast_rtf']:>9.1f} "
              f"{r['fast_steps_per_s']:>9.0f} {r['peak_rss_mb']:>8.0f}{flag}")
        if args.realtime:
            print(f"      realtime check: demo RTF {r['demo_rtf']:.3f}")
        if ok:
            ceiling = r["n"]

    if ceiling == max(args.counts):
        print(f"\nceiling NOT bracketed: the largest count tested ({ceiling}) still holds "
              f"{RTF_FLOOR}x.\nRe-run with higher --counts before trusting a number.")
    elif ceiling:
        ship = int(0.8 * ceiling)
        print(f"\nceiling (RTF >= {RTF_FLOOR}): {ceiling} robots")
        print(f"ship 80% of it: robots_per_lane = {max(1, ship // 4)}  ({(ship // 4) * 4} total)")
        print("Cap this by dashboard readability too -- above ~48 markers Godot needs MultiMesh.")
        print("\nNOTE: valid only for the controller and scenario swept. Re-run with the\n"
              "full auction and again with Godot and llama-server\n"
              "resident -- the ceiling under real memory pressure is the one that matters.")
    else:
        print("\nno tested count held 0.8x RTF; profile the tick before reducing N")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
