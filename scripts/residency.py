#!/usr/bin/env python
"""R1: does Godot + the simulator + llama-server fit in 8 GB, together?

    uv run python scripts/residency.py                 # samples until Ctrl-C
    uv run python scripts/residency.py --seconds 480   # a full demo run plus warm-up

Start this **fourth**, after llama-server, the sim and the Godot dashboard are all up.
It finds them by name, samples each process's physical footprint, and — the number that
actually decides the demo — watches how much swap **grows** while they run.

**Why physical footprint and not RSS.** M-29 measured the model at 829 MB physical and
89 MB RSS: the weights are mmap'd and Metal holds them outside conventional resident
memory, so `ps` understates it by 10x. `vmmap --summary` is the number that means
something on this machine.

**Why swap growth, not swap.** 8 GB with three resident processes can fit and still ruin
the demo: once macOS starts paging, `DemoSim` runs at wall-clock speed and misses ticks
visibly. But macOS keeps swap allocated from earlier activity — this machine sat at
1,868 MB used with nothing but an editor open — so the absolute figure says nothing. What
matters is the **delta from the moment sampling starts**. Pass condition: swap does not
grow during the run, and total footprint stays inside the ~4.7 GB budget in
TECHNICAL.md §9.

Risk register: PLAN.md §8 R1, the last demo-fatal risk still unmeasured.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import signal
import subprocess
import time
from pathlib import Path

#: (label, pattern matched against the full command line). Order is display order.
TARGETS = (
    ("llama-server", r"llama.*(serve|server)"),
    ("simulator", r"swarmmind\.cli"),
    ("response-team", r"swarmmind\.hivemind\.team\.worker"),
    ("godot", r"[Gg]odot"),
)

FOOTPRINT = re.compile(r"Physical footprint:\s+([0-9.]+)([KMG])")
SWAP = re.compile(r"used\s*=\s*([0-9.]+)([KMG])")


def _mb(value: str, unit: str) -> float:
    return float(value) * {"K": 1 / 1024, "M": 1.0, "G": 1024.0}[unit]


def _pids(pattern: str) -> list[int]:
    """Matching pids, excluding this process and the pgrep itself."""
    out = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
    return [int(p) for p in out.stdout.split()]


def _footprint_mb(pid: int) -> float | None:
    """Physical footprint of one process, or None if it has gone away."""
    out = subprocess.run(["vmmap", "--summary", str(pid)],
                         capture_output=True, text=True)
    m = FOOTPRINT.search(out.stdout)
    return _mb(*m.groups()) if m else None


def _swap_mb() -> float:
    out = subprocess.run(["sysctl", "vm.swapusage"], capture_output=True, text=True)
    m = SWAP.search(out.stdout)
    return _mb(*m.groups()) if m else 0.0


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
    ap.add_argument("--interval", type=float, default=5.0)
    ap.add_argument("--out", default="runs/residency.json")
    args = ap.parse_args()

    if not shutil.which("vmmap"):
        raise SystemExit("vmmap not found -- this measurement is macOS only")

    print(f"  sampling every {args.interval:.0f}s. Ctrl-C to stop and print the summary.\n")
    print("      t   " + "".join(f"{label:>14}" for label, _ in TARGETS)
          + f"{'total':>10}{'swap Δ':>10}")

    samples: list[dict] = []
    stop = False

    def _handle(_sig, _frm):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle)
    t0 = time.monotonic()

    while not stop:
        t = time.monotonic() - t0
        row: dict[str, float] = {}
        for label, pattern in TARGETS:
            mb = 0.0
            for pid in _pids(pattern):
                got = _footprint_mb(pid)
                if got:
                    mb += got
            row[label] = round(mb, 1)
        row["total"] = round(sum(row.values()), 1)
        swap_now = _swap_mb()
        nonlocal_baseline = samples[0]["swap_abs"] if samples else swap_now
        row["swap_abs"] = round(swap_now, 1)
        row["swap_delta"] = round(swap_now - nonlocal_baseline, 1)
        row["t"] = round(t, 1)
        samples.append(row)

        print(f"  {t:6.0f}   " + "".join(f"{row[label]:>13.0f}M" for label, _ in TARGETS)
              + f"{row['total']:>9.0f}M{row['swap_delta']:>+9.0f}M"
              + ("   <-- PAGING" if row["swap_delta"] > 0 else ""))

        if args.seconds and t >= args.seconds:
            break
        time.sleep(args.interval)

    if not samples:
        return
    print("\n  peaks")
    for label, _ in (*TARGETS, ("total", "")):
        peak = max(s[label] for s in samples)
        missing = " (never seen -- was it running?)" if peak == 0.0 else ""
        print(f"    {label:<14} {peak:8.0f} MB{missing}")
    swap_grew = max(s["swap_delta"] for s in samples)
    print(f"    {'swap growth':<14} {swap_grew:+8.0f} MB"
          + ("   ** the demo will stutter -- this is the R1 failure **" if swap_grew > 0
             else "   -- clean, never paged")
          + f"   (baseline {samples[0]['swap_abs']:.0f} MB already in use)")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"interval_s": args.interval, "samples": samples},
                              indent=2), encoding="utf-8")
    print(f"\n  {len(samples)} samples -> {out}")


if __name__ == "__main__":
    main()
