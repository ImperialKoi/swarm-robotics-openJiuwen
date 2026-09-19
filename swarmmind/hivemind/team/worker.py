"""Persistent isolated runtime. stdin/stdout carry only detached observations/results.

Launched with the simulator interpreter for custom orchestration, or the separate
WorkSwarm interpreter for native Leader/Teammate execution. No World is constructed.
"""

import argparse
import asyncio
import json
import sys

from .config import TeamConfig
from .model import LocalModel
from .workflow import collaborate

_request_id = ""


def send(kind, **payload):
    print(json.dumps({"kind": kind, "request": _request_id, **payload}), flush=True)


def active_record(kind, **payload):
    send("trace", event=kind, **payload)


async def execute(request, runtime):
    config = TeamConfig(**request["config"])
    if runtime == "workswarm":
        from .native import run_native

        return await run_native(request["snapshot"], config, active_record)
    model = LocalModel(config, active_record)

    async def decide(role, context, count):
        return await asyncio.to_thread(model.generate, role, context, count)

    return await collaborate(request["snapshot"], config, decide, active_record)


async def main():
    global _request_id
    ap = argparse.ArgumentParser()
    ap.add_argument("--runtime", choices=("local", "workswarm"), required=True)
    args = ap.parse_args()
    if args.runtime == "workswarm":
        import importlib.metadata

        if importlib.metadata.version("workswarm") != "0.2.6":
            raise RuntimeError("expected workswarm==0.2.6 in the isolated environment")
        # Import at startup so readiness means the engine can actually load.
        from .native import run_native  # noqa: F401
    send("ready", runtime=args.runtime)
    for line in sys.stdin:
        if len(line) > 131072:
            raise ValueError("request exceeds the observation budget")
        request = json.loads(line)
        _request_id = request["snapshot"]["id"]
        try:
            result = await execute(request, args.runtime)
            send("result", result=result)
        except Exception as exc:
            send("error", error=f"{type(exc).__name__}: {exc}"[:240])


if __name__ == "__main__":
    asyncio.run(main())
