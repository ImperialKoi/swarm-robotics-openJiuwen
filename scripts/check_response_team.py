#!/usr/bin/env python
"""Real-model peer-revision check over a labeled synthetic observation fixture.

This does not run a mission or dispatch its synthetic observations to any robot.
Start scripts/serve_hivemind.sh first. No API or external network is used.
"""

import argparse
import time
from pathlib import Path

from swarmmind.hivemind.team.client import WorkerClient
from swarmmind.hivemind.team.config import ROOT, TeamConfig
from swarmmind.hivemind.team.trace import Trace, render_report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--python", default=str(ROOT / "integrations/workswarm/.venv/bin/python"))
    ap.add_argument("--runtime", choices=("workswarm", "local"), default="workswarm")
    ap.add_argument("--trace", default="runs/team/peer-check.jsonl")
    args = ap.parse_args()
    sectors = [{
        "id": sid, "explored": 1.0, "hazard": 0.0, "connected": 0,
        "contacts": 0, "resolved_reports": 0, "backlog": {}, "collection": False,
        "priority": 1, "abandoned": False,
    } for sid in ("A1", "B1", "C1", "D1")]
    sectors[0].update(contacts=1, backlog={"extract": 1})
    sectors[1].update(explored=0.2, connected=3)
    snapshot = {"id": "synthetic-peer-check", "sim_time": 60.0, "duration": 420,
                "sectors": sectors, "lanes": {
                    lane: {"connected": 1, "battery": 0.8}
                    for lane in ("camera", "digger", "gripper", "relay")},
                "events": [], "previous": {}}
    trace = Trace(args.trace, console=True)
    trace.record("started", runtime=args.runtime, input_kind="SYNTHETIC VERIFICATION FIXTURE",
               purpose="Unsupported rescue demand must be challenged by logistics; no dispatch.")
    client = WorkerClient(Path(args.python), args.runtime, TeamConfig.load(), trace)
    submitted = False
    result = None
    try:
        deadline = time.monotonic() + 115
        while time.monotonic() < deadline:
            for row in client.poll():
                if row["kind"] == "ready" and not submitted:
                    client.submit(snapshot)
                    submitted = True
                elif row["kind"] == "result":
                    result = row["result"]
                elif row["kind"] in {"fatal", "error"}:
                    raise RuntimeError(row.get("error"))
            if result is not None:
                break
            time.sleep(0.02)
        revised = any(r["event"] == "revised" for r in trace.records)
        passed = bool(result and (result.get("directive") or {}).get("sector") == "B1" and revised)
        trace.record("finished", passed=passed, result=result,
                   scope="Synthetic observations, real runtime and local model; not a mission.")
    finally:
        client.close()
        trace.close()
        render_report(args.trace)
    print(f"Peer-revision check: {'PASS' if passed else 'FAIL'}; {args.trace}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
