# SwarmMind

Hierarchical hivemind-controlled robot swarm for simulated disaster search and rescue.
Built for Hack the North.

Three control tiers: a 20 Hz reflex layer, a 1 Hz decentralised task auction that self-heals
when robots die, and a locally-run LLM issuing sector-level strategy every 6 seconds under a JSON schema constraint.
The auction has no LLM in it, which is why the swarm keeps working when the hivemind goes offline.

An optional **three-agent response team** adds a rescue lead, logistics specialist and
safety reviewer using openJiuwen/WorkSwarm's native Leader/Teammate system and the local model. Peers can revise
or veto sector plans; validated orders affect the existing auction. The team has an
independent trace and graceful scripted fallback.
See [setup and complete demo instructions](docs/MULTI_AGENT_DEMO.md), including current
component status and measured limitations. Run it with `--demo --response-team` after setup.

- [CLAUDE.md](CLAUDE.md) — working rules and invariants
- [docs/PLAN.md](docs/PLAN.md) — scope, priorities, risk
- [docs/TECHNICAL.md](docs/TECHNICAL.md) — design of record
- [docs/DASHBOARD.md](docs/DASHBOARD.md) — **how to run the dashboard and read a run**
- [Animated rescue fleet](docs/units/index.html) — 15 original unit models and their animations; [design notes](docs/UNIT_MODELS.md)

```bash
uv sync
make check                                          # lint + tests + the tiny fixture smoke mission
uv run python -m swarmmind.cli run --headless --seed 42
```
