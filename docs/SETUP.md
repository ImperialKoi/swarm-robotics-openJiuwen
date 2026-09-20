# Setup: clone to demo

Every step from a bare machine to the running demo, including the voice channel and the
multi-agent response team. Tested on macOS (Apple silicon).

If you only want to see the swarm run and have nothing installed, do **steps 1–4** and
stop; that gives you a complete headless mission with no API key, no network and no
dashboard.

---

## 0. Prerequisites

| | why | check |
|---|---|---|
| **[uv](https://docs.astral.sh/uv/)** | the project pins **Python 3.12**; system Python is too new for the dependencies | `uv --version` |
| **Godot 4.7.2** | the 3D dashboard. Optional — the simulation runs headless without it | `/Applications/Godot.app/Contents/MacOS/Godot --version` |
| **An OpenRouter key** | Tier 3, the voice channel and the response team. Optional — everything falls back to a scripted rung offline | — |
| **A microphone** | only for `--voice` | — |

Install uv if you need it:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**Do not `pip install` into the system interpreter.** `uv` creates and pins the 3.12
environment; nothing here should touch your system Python.

---

## 1. Clone

```bash
git clone https://github.com/ImperialKoi/swarm-robotics-openJiuwen.git
cd swarm-robotics-openJiuwen
```

## 2. Install dependencies

```bash
uv sync --extra bridge --extra voice --extra dev --extra evo
```

Each extra is separate on purpose — the base install is deliberately small:

| extra | brings | needed for |
|---|---|---|
| `bridge` | `websockets` | the Godot dashboard |
| `voice` | `sounddevice` | the microphone and playback |
| `dev` | `pytest`, `ruff` | the test suite |
| `evo` | `ribs` | MAP-Elites, and two test modules that import it |

> **`uv sync --extra voice` on its own removes `websockets` and breaks the dashboard.**
> `uv sync` replaces the whole extra set rather than adding to it, so always pass the
> full list above.

## 3. Verify the install

```bash
make check
```

Ruff, the full test suite, and a headless mission on the tiny fixture. This should be
green before you go further. It takes a few minutes; the suite runs a real mission.

## 4. Run a mission with no dashboard and no network

```bash
uv run python -m swarmmind.cli run --headless --scenario test --seed 42   # ~25 s
uv run python -m swarmmind.cli run --headless --seed 42                   # the demo map, ~6 min
```

Prints a scorecard and a hash. **Seed 42 is deterministic** — the same hash every run on
the same machine. No model is called on this path at all.

At this point the swarm works. Everything below adds interfaces on top of it.

---

## 5. Add the model key

```bash
cp .env.example .env
$EDITOR .env          # set OPENROUTER_API_KEY=sk-or-v1-...
```

`.env` is gitignored. Load it by running commands with `uv run --env-file .env ...` — it
is **not** picked up automatically.

Get a key at [openrouter.ai](https://openrouter.ai/keys). Roughly $0.005 per spoken turn
and $0.0029 per Tier 3 cycle, so a full demo costs cents.

## 6. Set up the Godot dashboard

```bash
/Applications/Godot.app/Contents/MacOS/Godot --headless --path godot --import
```

**Do not skip this on a fresh clone.** Godot's class cache is generated and gitignored, so
until the project is imported no `class_name` resolves — and the failure is reported
against an innocent file:

```
SCRIPT ERROR: Parse Error: Could not resolve class "SwarmDashboard"
```

Re-run it any time you add a script with a new `class_name`.

Then open the editor once: launch **Godot 4.7.2** → **Import** → select
`godot/project.godot` → **Open**.

## 7. Set up the multi-agent response team (optional)

```bash
./scripts/setup_response_team.sh
```

Builds a **separate** virtualenv at `integrations/workswarm/.venv` from a pinned lockfile.
The core environment never imports WorkSwarm. It prints
`WorkSwarm <version> native Leader/Teammate ready` when it worked.

---

## 8. Run the demo

Two terminals.

**Terminal 1 — the swarm:**

```bash
uv run --env-file .env python -m swarmmind.cli run --demo
```

Wait for `dashboard: ws://127.0.0.1:8765`.

**Terminal 2 — the dashboard:** open the Godot project and press **F5**. It reconnects
either way, so the order does not matter.

### The full build, both prize tracks at once

```bash
uv run --env-file .env python -m swarmmind.cli run --demo --voice \
    --response-team --team-scout --team-trace runs/team/live.jsonl
```

| flag | adds |
|---|---|
| `--voice` | the operator voice channel: hold **U**, speak, release |
| `--response-team` | the rescue lead / logistics / safety agents |
| `--team-scout` | a fourth agent that reads the operator's map for the lead |

Every one of these is **off by default** and additive. The mission runs identically
without them.

### Other useful runs

```bash
# no strategic layer at all -- the control condition for "Tier 2 does not depend on Tier 3"
uv run python -m swarmmind.cli run --demo --no-hivemind

# the tiny fixture, boots in a second, good for checking the dashboard
uv run python -m swarmmind.cli run --demo --scenario test
```

---

## 9. Drive the demo

| key | does |
|---|---|
| **`U` (hold)** | **talk to the swarm.** Release to send; captions appear under the map |
| `E` | eagle-eye, the whole map at once — **start here** |
| click | follow a robot |
| `F` | cycle POV → chase → orbit |
| `W` `A` `S` `D` | drive the followed unit |
| `Space` | its action — pick up, set down, take off, land |
| `G` | god view — removes the fog and shows the truth. For debugging, **not for judging** |
| `H` | simulated thermal vision for the followed unit |
| `Esc` | stop following |

Full key table in [DASHBOARD.md](DASHBOARD.md).

### Things to say

| say | what happens |
|---|---|
| *"How is the search going? Which areas still need coverage?"* | answers from the swarm's own belief; **nothing is retasked** |
| *"Sector A6 is a priority. Push rescue teams there now."* | applies the order, claims the sector, outranks Tier 3 |
| *"Abandon every single sector on the map."* | **refused by the feasibility filter, out loud, with the reason** |

That third one is the best thing to show a judge: a human's spoken order goes through the
same filter a model's does, and the refusal is spoken back.

---

## Troubleshooting

| symptom | cause |
|---|---|
| `Could not resolve class "SwarmDashboard"` | step 6 not run. Import the Godot project |
| `ModuleNotFoundError: websockets` | `uv sync` run without `--extra bridge`. Re-run the full command in step 2 |
| `ModuleNotFoundError: ribs` | same, missing `--extra evo` |
| `Set OPENROUTER_API_KEY to use the OpenRouter model` | `.env` missing, or the command was not run with `uv run --env-file .env` |
| Dashboard connects then drops | a frame exceeded Godot's 64 KB inbound buffer |
| Voice hears nothing | grant microphone permission; check `U` is reaching the sim — the badge under the map reads `LISTENING` while held |
| `cannot listen on 127.0.0.1:8765` | another instance is running. Stop it, or pass `--port 8766` and set `SWARMMIND_WS=ws://127.0.0.1:8766` for Godot |
| Very low frame rate at 512 robots | expected on 8 GB. Use `--scenario test`, or `--no-hivemind` to free ~830 MB |

## What runs where

Nothing trains on your machine. The hosted models are called over HTTPS for Tier 3, the
voice channel and the response team; everything else — physics, perception, the auction,
Tier 1 — runs locally. `--headless` calls no model at all.
