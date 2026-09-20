# SwarmMind

Hierarchical hivemind-controlled robot swarm for simulated disaster search and rescue.
Built for Hack the North.

Three control tiers: a 20 Hz reflex layer, a 1 Hz decentralised task auction that self-heals
when robots die, and an optional hosted model (OpenRouter) issuing sector-level strategy every 6 seconds under a JSON schema constraint.
The auction has no LLM in it, which is why the swarm keeps working when the hivemind goes offline.

An optional **three-agent response team** adds a rescue lead, logistics specialist and
safety reviewer using openJiuwen/WorkSwarm's native Leader/Teammate system and the same hosted model. Peers can revise
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

Model setup: copy `.env.example` to `.env` and set `OPENROUTER_API_KEY` (an OpenRouter
`sk-or-v1-...` key), then run
`uv run --env-file .env python -m swarmmind.cli run --demo --hivemind-allow-api`.
Both paths default to `openai/gpt-5.6-terra` via OpenRouter, chosen by measurement over 15
models (M-88); see the setup guide for the team's isolated runtime. `OPENROUTER_MODEL`
overrides the model for both paths.
`.env` is gitignored; no API key is stored in tracked files.

**Talk to the swarm.** `--voice` adds an operator voice channel: speak, and one model
call hears the question, reads the operator's fog-limited map and answers out loud, with
captions under the map and sector orders the swarm executes. The operator outranks Tier 3
and the response team on the sectors they name. `--team-scout` adds a fourth team role
that reads that map for the rescue lead.

```bash
uv sync --extra bridge --extra voice --extra dev --extra evo
uv run --env-file .env python -m swarmmind.cli run --demo --voice
uv run --env-file .env python -m swarmmind.cli run --demo --voice --response-team --team-scout
```

Both are optional and off by default, and neither claims a rescue-score improvement.
Details, latency and cost in [docs/OMNI_VOICE.md](docs/OMNI_VOICE.md).