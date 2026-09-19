> Current model configuration: the response team (openJiuwen native Leader/Teammate)
> uses `openai/gpt-5.6-terra` through **OpenRouter** via `OPENROUTER_API_KEY`
> (`OPENROUTER_MODEL` overrides; per-model quirks live in `TUNING`, measured in M-88 --
> a model outside that table gets a cautious default). The regular hivemind uses the same endpoint with
> `--hivemind-allow-api`. Keep the key in the gitignored `.env` and run with
> `uv run --env-file .env ...`. This user-requested hosted configuration supersedes the
> local-Qwen setup below; network access is needed for model calls. Headless runs and
> scripted fallback remain offline. Historical Qwen measurements do not validate hosted
> behavior; M-87 and M-88 have the live OpenRouter checks.

# CLAUDE.md — SwarmMind

Hierarchical hivemind-controlled robot swarm for simulated disaster search and rescue.
Demo target: **Hack the North**.

Read [docs/PLAN.md](docs/PLAN.md) for scope, priorities and cut lines.
Read [docs/TECHNICAL.md](docs/TECHNICAL.md) for the design of record — schemas, algorithms, numbers.
**When PLAN/TECHNICAL and this file disagree, TECHNICAL wins for design and PLAN wins for scope.**

---

## Environment (this is not a normal dev box — read before installing anything)

- **Apple M1, 8 GB RAM, 8 cores.** No Docker, no Homebrew, no ROS2, no Godot, no Node.
- System Python is **3.14** — too new for torch / pyribs / stable-baselines3. The project pins **3.12 via `uv`**. Never `pip install` into the system interpreter.
- **All GPU work is Kaggle free tier** (CV detector, RAFT/DPO/GRPO). **MAP-Elites runs locally** on the 8 M1 cores - faster per-core than a Kaggle CPU session. Nothing trains in torch on this laptop.
- **Unit-policy RL trains on Kaggle CPU sessions, not on this machine** (owner's decision). This laptop runs unit tests and short smoke runs on `test.yaml` only; multi-mission jobs (bounds, training, gates) go to `swarmmind/training/notebooks/unit_policy.ipynb`. Measured on Kaggle: **~1,040 s per demo mission with 4 in parallel, 13.8 missions/hour** ([MEASUREMENTS.md M-76a](docs/MEASUREMENTS.md)).
- **Evaluation costs 2.5 s, not 3.3 ms.** The original MAP-Elites budget was out by ~200x ([MEASUREMENTS.md M-30](docs/MEASUREMENTS.md)): ~1,400 evaluations/hour on 7 workers. Size runs from that, and never quote the old 300k figure.
- **Never start `EvaluationPool` from a REPL or a heredoc.** `spawn` re-executes `__main__`; with no importable `__main__` every worker re-runs the caller and spawns its own. It is a fork bomb that looks like a hang. Use a real script or `python -m`.
- The 8 GB budget is real and tracked in [TECHNICAL.md §9](docs/TECHNICAL.md). Before adding a resident process, check it fits.

```bash
uv sync                                        # env
make check                                     # ruff + pytest + headless mission — must be green
uv run python -m swarmmind.cli run --headless --seed 42            # demo scenario, 512 robots, ~6 min (367 s, M-76)
uv run python -m swarmmind.cli run --headless --scenario test      # tiny fixture, ~1 s
uv run python scripts/stress.py                                    # robot-count sweep
make evolve                                    # MAP-Elites (uv sync --extra evo first)
uv run python -m swarmmind.training.mapelites.run --bench 20        # eval throughput
uv run python -m swarmmind.cli run --demo      # DemoSim + WS bridge for Godot
uv run python -m swarmmind.cli run --demo --no-hivemind   # must still work
uv run python -m swarmmind.cli run --headless --zone-routing      # carrier-trap fix, not the demo default
uv run python -m swarmmind.cli run --demo --unit-policy runs/rl/policy_best.npz  # kept, unused

uv run python scripts/kaggle_bundle.py         # zip for Kaggle -> runs/kaggle/swarmmind-rl-src.zip
# then on Kaggle: unit_policy.ipynb, JOB="bound" (~45 min) or JOB="train" (11 h, resumable)

./scripts/serve_hivemind.sh                    # local Tier 3 model, before --demo
# Not required: with nothing listening on 8080 the ladder drops to the scripted rung.
# `--headless` never uses a model at all, so the seed-42 hash stays reproducible.
```

### The demo plays four maps: seeds 42, 43, 44, 45

`assets/scenarios/demo.yaml` `demo_seeds` is the **only** definition; the gate
(`training/gate.py`), the unit-policy trainer and bound (`training/rl/`) and the Kaggle notebook
all read it, and `tests/test_demo_seeds.py` fails if any of them drift. Other seed ranges stay in
the code -- `HELD_OUT_SEEDS` 101-110 behind `gate.py --held-out`, MAP-Elites' 11-13 on the test
fixture -- but nothing for the demo trains, checks or gates on them, and `cli run --demo` prints a
note on any other seed.

**The consequence has to be said the same way every time:** anything trained on these seeds is
tuned *for these four maps*. A win on them is a win on the demo, not evidence it works on a map it
has not seen -- the commander's run 2 is the record of the difference (+3.66 on its training maps,
0.00 elsewhere). Say "tuned on the four demo maps".

---

## The six invariants

Breaking any of these is a regression regardless of how the demo looks.

1. **Tier 2 never depends on Tier 3.** The auction must allocate, execute and self-heal with `/hivemind/directives` completely silent. `tests/test_mission.py` runs with the hivemind disabled and must pass. This is the project's central claim — do not make a directive load-bearing.
2. **Tier 1 is the safety floor.** The swept-circle wall override in `control/tier1_reflex.py` is not bypassable by any higher tier, ever.
3. **Perception is real, and no swarm-side code reads ground truth.** Robots see through a detector running on their own occluded camera frame (`perception/`). Nothing above `perception/` may read victim positions, the hazard disc, or the appearance raster from the simulator - only `Detection` objects. The simulation owns optics (crop, rotate, occlude); the robot owns the detector.
4. **No swarm-side code reads `/world/ground_truth`.** It exists for the dashboard alone. `tests/test_no_ground_truth_leak.py` enforces this; if it fails, the fog-of-war claim is false and the demo is dishonest.
5. **`swarmmind/contracts/` is FROZEN.** Schemas and topic names are defined there and nowhere else. A change to a schema is a change to `contracts/`, the Godot parser, and the tests, in one commit.
6. **Determinism.** Seed 42 must produce a byte-identical scorecard hash. One seeded `Generator` per subsystem from `SeedSequence.spawn()`; never global `np.random` or bare `random`; **never iterate a `set` or unordered `dict` in the sim or auction loop** — sort by id. Rehearsed demo timings depend on this.

---

## Architecture in one screen

```
Godot 4 (native macOS) ──WebSocket 10Hz── bridge_node
                                              │
              Bus (LocalBus default · Ros2Bus drop-in, same topic names)
                                              │
   blackboard · auction · hivemind · hazard · events · fault_injector
                                              │
                     N × skill_executor (Tier 2) + reflex (Tier 1)
                                              │
              perception: camera (occluded egocentric crop) → detector
                                              │
                SimBackend: FastSim (training) | DemoSim (demo) | GazeboSim (stretch)
```

- **`FastSim` and `DemoSim` are the same `World` class**, differing only in clocking. Never let their physics diverge — that is what makes training transfer for free.
- ROS2 is a **transport adapter, not a dependency**. Nodes are transport-agnostic. Do not `import rclpy` outside `bus/ros2.py`.
- Godot talks **WebSocket**, not `godot_ros`. Do not reintroduce the GDExtension.
- **The dashboard is 3D; the simulation is 2.5D and stays that way.** `world.height` is render-only - navigation, bidding, collision and perception are all flat. 512 robots with 3D physics do not fit this machine. Keep saying *custom 2.5D kinematic simulator*.
- **Godot cannot be run from the dev environment.** Anything visual gets verified first with `swarmmind/viz/render3d.py`, which renders the same scene to PNG offline. Write the maths, look at the frame, then port it to GDScript. `tests/test_bridge_protocol.py` parses `main.gd` and fails the build if the wire format and the dashboard disagree.
- **Two palettes, on purpose.** `perception/raster.py` is what the detector sees - dark, low-contrast, warm rubble confusable with a casualty. `render3d.py` / `main.gd` use a separate display palette for human eyes. Do not unify them.
- **Nav2 is out** of the primary path. Tier 1 is our own A* + vector-field controller.

### Scaling rules (robot count is measured, not chosen)

`N` is **512** (128 per lane). The measured ceiling under the D1 placeholder controller was 3072 (docs/MEASUREMENTS.md M-1), but compute was never the binding constraint - mission size is, and the two must scale together. Re-measure after D2 and D3, which are far heavier than the D1 controller.

**Robot count and mission size are coupled.** A large swarm on a small map clears all fog in seconds, leaves the auction nothing to allocate, and makes the scripted failure invisible. If you change `robots_per_lane`, you are also changing `map.*`, `victims.count` and `hazard.growth_rate`. Four rules keep the scaling honest:

- **Robot state is parallel numpy arrays**, never a list of objects. **Nothing in the tick loop may iterate robots in Python** outside the 5 Hz sensor pass. A `for robot in robots` in the hot loop is a bug, not a style issue.
- **Auction bidding uses a BFS distance field per task target** on the *coarse* nav grid, cached for the whole mission (`passable` never changes) and read O(1) per robot. Never call A\* per (robot, task) pair - that is ~10k searches/second at 512 robots. A\* belongs in Tier 1 only.
- **The announcement cap must scale with free robots**, not be a flat number. A fixed cap of 24 throttled a 512-robot swarm to 24 new assignments per second and left the rest idle.
- **Orphaning re-offers work without taking it away** from a live robot. Releasing an out-of-contact relay frees it to bid on a frontier, it abandons its post, and the swarm behind it loses contact - a cascade that ended one run with 0 of 16 robots in comms.
- **Fog/LOS runs at 5 Hz** with a half-cell movement gate, vectorized over a disc template. Not per-cell Bresenham at 20 Hz.
- **Distance fields cost 22.4 ms each on the demo grid** (MEASUREMENTS.md M-4). One field per open task per cycle is ~4.5 s of compute per second at 200+ tasks. D3 must use a 4x-downsampled navigation grid, cap announced tasks at ~24 per cycle, and amortise field computation across the cycle's 20 ticks. All three, not one.

---

## Working rules

- **Measure before tuning `N`.** The stress sweep sets the *upper bound*; the shipped number is also capped by dashboard readability. Above ~48 markers Godot switches to `MultiMeshInstance2D`.
- **`make check` stays green from D3 onward.** A commit that breaks the headless mission gets reverted, not debugged later.
- **Build the classical heuristic first, always.** `control/heuristic.py` is both the fallback and the gate's baseline. Anything learned must beat it on the demo maps ([TECHNICAL.md §8](docs/TECHNICAL.md)) or it does not ship. A heuristic winning is a **result**, not a failure.
- **Bound the action before training it.** Measure what a perfect or classical version of the action buys first (`training/rl/bound.py`). Contact inspection was dropped this way: an oracle choosing only real casualties moved rescues +0.5 ([MEASUREMENTS.md M-76](docs/MEASUREMENTS.md)).
- **Everything trained is reached by a ladder, never a jump.** Hivemind: RAFT → DPO → GRPO, each banked before attempting the next. Never leave the repo in a state where the only tuned model is one that hasn't finished training.
- **Checkpoint Kaggle work from the first run.** `/kaggle/working` is not ambiently persistent — resume works by attaching the previous saved version's output as an input.
- Prefer **interpretable parameter vectors over neural nets** where they do the job (see the Tier-2 `BehaviorParams` genome). Faster to evolve, impossible to produce uninspectable garbage, and legible on the dashboard.
- Scenario constants live in `assets/scenarios/*.yaml`, never hardcoded in nodes. **`demo.yaml` is the demo; `test.yaml` is a deliberately tiny fixture so `make check` stays seconds.** Tests run against `test.yaml`, plus one cheap construct-and-assert test against `demo.yaml` so config regressions cannot reach demo day.
- **Record measurements in `docs/MEASUREMENTS.md`, never delete a row.** The trend across days is what shows whether a change regressed.
- New `/swarm/events` kinds need a dashboard handler in the same change, or they are invisible.

## Do not

- **Do not reopen Tier 3a (the commander) by tuning `CommandParams`.** It was trained three times and gated on ten held-out maps: **none 70.81 / heuristic 65.57 / trained 69.42 = 0.98x**, under the 1.05x bar. **No commander ships**, and `command=False` is already the default. The limit is *mechanism*, not parameters — fenced relay tasks, bearing-cut pie-slice wedges, a value function that clips to zero. Change one of those and re-gate, or leave it alone. Full record in [training_notes/](training_notes/SUMMARY.md), reasoning in [PLAN.md §7.6](docs/PLAN.md).
- Do not add Gazebo, ROS2, Nav2 or Docker to the critical path. `GazeboSim` is a time-boxed event-day stretch.
- Do not add speed/pause/2x controls. Cut deliberately, not deferred.
- Do not make the demo require network. The hivemind runs locally on `llama.cpp`; the API provider is rung 3 of a fallback ladder, behind `--hivemind-allow-api`.
- Do not add a resident process without checking the 8 GB budget.
- Do not silently widen scope. The cut lines in [PLAN.md §7.1](docs/PLAN.md) are ordered; follow them.
- **Do not train, check or gate anything for the demo on seeds outside `demo_seeds`**, and do not describe the unit policy as working on other maps. See "The demo plays four maps" above.
- **Do not turn the unit policy on for the demo.** `control/unit_policy.py` and `training/rl/` are kept and unused: gated at **1.04x** with Tier 3 off and **0.997x** with it live (M-76d, M-76e). It is a flag, off by default, and it stays that way unless something re-measures it.
- **Do not adopt `zone_routing` for the demo on the Kaggle numbers.** It is +15 rescues with Tier 3 silent (M-76d) and **unresolved with Tier 3 live**: +9.5 on Kaggle, −1.25 on the demo machine, the same code and seeds (M-76f). It stays off by default; the demo's rehearsed build is unchanged.
- **A before/after pair must be taken on the machine that will run it.** Platform float moves a single demo map by ~10 rescues here — the size of the effects being chased (M-40, M-76f).

---

## Language when describing this project

Accuracy matters more than impressiveness here — a judge who catches an overstatement discounts everything else.

| Say | Not |
|---|---|
| market-based single-round reverse auction | CBBA |
| proximity-based fog-of-war reveal | SLAM |
| ROS2-compatible node graph on a pluggable transport | built on ROS2 (unless `Ros2Bus` is actually running) |
| custom 2.5D kinematic simulator | Gazebo (unless `GazeboSim` is actually running) |
| RAFT-tuned / DPO-tuned / GRPO-tuned — whichever cleared the gate | fine-tuned with GRPO, aspirationally |
| a unit policy trained with PPO on the four demo maps (only if it cleared the gate) | an RL-trained swarm / learned behaviour that generalises |
| exact routing to collection points, a correctness fix | a learned or trained navigation |

`SHIPPING.md` (generated by `training/gate.py` on D12) is the source of truth for which components are trained vs. heuristic. Keep it accurate.

---

## LLM usage in this repo

- **CV detector (D11):** small conv net over 48x48x3 egocentric frames, trained on Kaggle GPU against simulator labels. Must beat `perception/classical.py` at the gate or the classical detector ships. Perception is real either way.
- **Hivemind (primary):** locally-served `Qwen2.5-1.5B-Instruct` GGUF via `llama-server` with JSON-schema-constrained decoding. Fine-tuned on this simulator.
- **Hivemind (API rung):** Anthropic SDK, model `claude-opus-5`, `output_config={"effort":"low","format":{...}}` for structured output, `thinking={"type":"adaptive"}`. **Assistant prefill returns 400 on Opus 5** — use structured outputs to constrain the JSON, never prefill. ~$0.60 per 7-minute run. **There is no API key on this machine, so this rung has never run** — `build_ladder` drops it silently and the code is unverified against the live API. Do not describe it as working.
- **The schema is the latency budget.** Generation dominates the 6 s cycle. Unconstrained, the 1.5B model took 10.6 s per call and every cycle fell through to the scripted rung while appearing to work. `maxLength` in `DIRECTIVE_SCHEMA` compiles to a GBNF constraint; loosening it is a latency change first ([MEASUREMENTS.md M-27](docs/MEASUREMENTS.md)).
- **`nodes/hivemind.py` must stay asynchronous.** One tick starts a request, a later tick collects it. Joining the worker thread — the obvious design, and one that passes every test that uses an instant provider — freezes `DemoSim` for seconds at a time, because it runs at wall-clock speed.
- Every hivemind output passes `hivemind/filter.py` before it can affect anything. Rejections are published and logged on purpose — the filter catching a bad directive is a good demo moment.
- Directives expire after 30 s. A stale `abandon` locking a third of the map is the most likely quiet failure mode.

---

## File pointers

| Need | Go to |
|---|---|
| Message shapes, topic names | [swarmmind/contracts/schemas.py](swarmmind/contracts/schemas.py) — **FROZEN** |
| Auction, bidding, self-healing | [swarmmind/nodes/auction.py](swarmmind/nodes/auction.py) · [TECHNICAL.md §5.2](docs/TECHNICAL.md) |
| Directive → behavior wiring | [swarmmind/nodes/hivemind.py](swarmmind/nodes/hivemind.py) · [TECHNICAL.md §5.3](docs/TECHNICAL.md) |
| Sim physics, hazard, comms | [swarmmind/sim/world.py](swarmmind/sim/world.py) · [TECHNICAL.md §3](docs/TECHNICAL.md) |
| Dashboard particle effects | [swarmmind/viz/particles.py](swarmmind/viz/particles.py) is the **model of record**; `FX_KINDS` in `main.gd` is a port of it and `tests/test_bridge_protocol.py` compares the two. Look at a plume with `snapshot3d.py --fx` before changing a number |
| Evolution | [swarmmind/training/mapelites/](swarmmind/training/mapelites/) · [TECHNICAL.md §6](docs/TECHNICAL.md) |
| Hivemind training | [swarmmind/training/llm/](swarmmind/training/llm/) · [TECHNICAL.md §7](docs/TECHNICAL.md) |
| Ship / don't-ship decision | [swarmmind/training/gate.py](swarmmind/training/gate.py) · [TECHNICAL.md §8](docs/TECHNICAL.md) |
| **What every training run did, and why it failed** | [training_notes/SUMMARY.md](training_notes/SUMMARY.md) — read before re-running anything |
| **Every problem hit so far, and what fixed it** | [docs/FIXES.md](docs/FIXES.md) — grouped by subsystem; read before re-fixing something |
| Why the rescue rate is ~11%, measured live | `uv run python scripts/diagnose.py` · [MEASUREMENTS.md M-34/35/36](docs/MEASUREMENTS.md) |
| Demo running order | [docs/RUNBOOK.md](docs/RUNBOOK.md) (written D13) |
| Unit policy (per-robot staging inside Tier 2) | [swarmmind/control/unit_policy.py](swarmmind/control/unit_policy.py) · [TECHNICAL.md §7a](docs/TECHNICAL.md) · [training_notes/run4-unit-policy.md](training_notes/run4-unit-policy.md) |
| Unit-policy training (BC -> PPO, numpy), bound, Kaggle notebook | [swarmmind/training/rl/](swarmmind/training/rl/) · [swarmmind/training/notebooks/unit_policy.ipynb](swarmmind/training/notebooks/unit_policy.ipynb) |
| Routing to collection points (carrier trap fix; kept, **off by default**) | [swarmmind/control/zone_routing.py](swarmmind/control/zone_routing.py) · [MEASUREMENTS.md M-76, M-76e, M-76f](docs/MEASUREMENTS.md) |
