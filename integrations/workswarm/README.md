# WorkSwarm integration

This is a customized application using the official WorkSwarm 0.2.6 SwarmFlow engine
and a restricted local-model backend. It is not a reusable Swarm Skill package.

- [Setup, workflow, demo and limitations](../../docs/MULTI_AGENT_DEMO.md)
- [Pinned optional environment](requirements.lock)
- [Fixed SwarmFlow workflow](../../swarmmind/hivemind/team/swarmflow.py)
- [Backend and worker](../../swarmmind/hivemind/team/worker.py)
- [Role tools](../../swarmmind/hivemind/team/tools.py)
- [Original research and plan](../../docs/SWARM_UPGRADE_PLAN.md)
- [Recorded live collaboration and failed diagnostic](examples/README.md)

Install with `./scripts/setup_response_team.sh` from the repository root. Do not install
this dependency set into the simulator environment. No WorkSwarm service needs to run
separately; `--response-team` starts and supervises its own isolated worker.
