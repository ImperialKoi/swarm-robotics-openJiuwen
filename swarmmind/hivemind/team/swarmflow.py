"""Fixed WorkSwarm SwarmFlow script. Loaded by the official engine, not generated live."""

import json

from swarmmind.hivemind.team.config import TeamConfig
from swarmmind.hivemind.team.model import choice_schema
from swarmmind.hivemind.team.workflow import collaborate

META = {"name": "rescue-response", "description": "Propose, challenge, revise and verify a rescue plan",
        "phases": ["response"], "max_concurrency": 1}


async def run(args):
    from swarmflow import agent, phase

    phase("response")

    async def decide(role, context, count):
        return await agent(json.dumps({"role": role, "context": context, "count": count}),
                           label=role, schema=choice_schema(count),
                           options={"timeout": args["config"]["deadline_s"]})

    # The backend owns the role trace sink; only serializable observations are in args.
    from swarmmind.hivemind.team.worker import active_record

    return await collaborate(args["snapshot"], TeamConfig(**args["config"]), decide, active_record)
