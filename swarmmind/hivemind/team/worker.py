"""Persistent isolated runtime. stdin/stdout carry only detached observations/results.

Launched with the simulator interpreter for custom orchestration, or the separate
WorkSwarm interpreter for the official SwarmFlow engine. No World is constructed.
"""

import argparse
import asyncio
import json
import sys
from pathlib import Path

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
    model = LocalModel(config, active_record)
    if runtime == "workswarm":
        from openjiuwen.agent_teams.workflow.engine.facade import (
            AgentBackend,
            AgentResult,
            run_workflow,
        )

        class RescueBackend(AgentBackend):
            async def run(self, prompt, opts, schema_json):
                data = json.loads(prompt)
                before = model.tokens
                result = await asyncio.to_thread(model.generate, data["role"],
                                                  data["context"], data["count"])
                used = model.tokens - before
                self.budget.add(used)
                if self.workflow_budget is not None:
                    self.workflow_budget.add(used)
                return AgentResult(structured=result, tokens=used)

        result = await run_workflow(str(Path(__file__).with_name("swarmflow.py")),
                                    args=request, backend=RescueBackend(), cap=1,
                                    run_id=request["snapshot"]["id"])
    else:
        async def decide(role, context, count):
            return await asyncio.to_thread(model.generate, role, context, count)

        result = await collaborate(request["snapshot"], config, decide, active_record)
    return result


def main():
    global _request_id
    ap = argparse.ArgumentParser()
    ap.add_argument("--runtime", choices=("local", "workswarm"), required=True)
    args = ap.parse_args()
    if args.runtime == "workswarm":
        import importlib.metadata

        if importlib.metadata.version("workswarm") != "0.2.6":
            raise RuntimeError("expected workswarm==0.2.6 in the isolated environment")
        # Import at startup so readiness means the engine can actually load.
        from openjiuwen.agent_teams.workflow.engine import facade  # noqa: F401
    send("ready", runtime=args.runtime)
    for line in sys.stdin:
        if len(line) > 131072:
            raise ValueError("request exceeds the observation budget")
        request = json.loads(line)
        _request_id = request["snapshot"]["id"]
        try:
            result = asyncio.run(execute(request, args.runtime))
            send("result", result=result)
        except Exception as exc:
            send("error", error=f"{type(exc).__name__}: {exc}"[:240])


if __name__ == "__main__":
    # Ensure the workflow loader's absolute import sees this module's trace state.
    sys.modules["swarmmind.hivemind.team.worker"] = sys.modules[__name__]
    main()
