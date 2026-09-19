# SwarmMind — Execution Plan

**Owner:** daniel · **Event:** Hack the North
**Target:** a 5–7 minute live demo of a hierarchical hivemind-controlled robot swarm doing disaster search and rescue.

This document covers scope and priorities. [TECHNICAL.md](TECHNICAL.md) is the *how*. [../CLAUDE.md](../CLAUDE.md) is the standing instruction file for coding agents.

**Owner-approved extension:** the optional multi-agent response application.
[SWARM_UPGRADE_PLAN.md](SWARM_UPGRADE_PLAN.md) records the approved scope;
[MULTI_AGENT_DEMO.md](MULTI_AGENT_DEMO.md) records the implementation, validation,
cut scope and remaining operator preparation. The owner subsequently requested the native
Leader/Teammate system; `--response-team` now selects that implementation. Existing
component defaults remain intact.

---

## 0. What changed from the original MVP spec, and why

The original spec was written before the hardware and resource constraints were known. Four facts reshape it:

| Fact | Consequence |
|---|---|
| Dev + demo machine is an **Apple M1, 8 GB RAM, 8 cores**. No Docker, no Homebrew, no ROS2, no Godot installed. | Gazebo + Nav2 + 20 robots is not reachable. A per-robot Nav2 stack is ~5 processes; 8 GB caps you near 4–6 robots at maybe 0.3–0.5× real-time, inside a Docker `linux/arm64` VM, with XQuartz for the GUI. |
| Only other compute is **Kaggle free tier** (~30 GPU-h/wk T4/P100, ~30 CPU-h/wk, 9 h GPU / 12 h CPU session cap). | All GPU training is Kaggle. All evolution is local CPU + Kaggle CPU. Nothing trains on the Mac in torch. |
| Single developer + coding agents. | The Section 14 RL layer must be built on a ladder where every rung is independently shippable. No all-or-nothing bets. |
| Installed Python is **3.14**. | torch / stable-baselines3 / pyribs do not have 3.14 wheels. The project pins **Python 3.12** via `uv`. |

### 0.1 Decision deltas (each is a deliberate change to the original spec)

**D1 — Simulation backbone becomes a swappable adapter; the headless sim is primary.**
One `SimBackend` interface with three implementations: `FastSim` (headless, as-fast-as-possible, used for all training), `DemoSim` (identical physics, clocked to wall time, used for the demo), and `GazeboSim` (optional, stretch, attempted at the event only). *FastSim and DemoSim are the same code with different clocking* — so there is no sim-to-sim gap between where policies train and where they run. This collapses most of §14.6's reality-gap risk down onto the optional Gazebo path only.

**D2 — ROS2 becomes a transport adapter, not a dependency.**
Node classes are transport-agnostic and talk over a `Bus` interface. `LocalBus` (in-process asyncio pub/sub) is the default and needs nothing installed. `Ros2Bus` (rclpy) is a drop-in that the same nodes run on unchanged, if and when ROS2 is available. **Topic names and message shapes are the exact ones in the original §3.2 / §5.** The architecture stays honestly ROS2-shaped; the demo does not depend on ROS2 booting.
*Judge-facing wording:* "ROS2-compatible node graph running over a pluggable transport; ROS2 backend included." Not "built on ROS2" unless the ROS2 backend is actually running at demo time.

**D3 — Nav2 is out of the primary path.**
Tier 1 becomes a ~300-line reactive controller (A* global path on the occupancy grid + vector-field local avoidance) that runs at 20 Hz for the whole swarm inside one Python process, fully vectorized. Nav2 comes back only under `GazeboSim`.

**D4 — Godot connects over WebSocket, not the `godot_ros` GDExtension.**
`godot_ros` needs ROS2 headers on the same host as Godot. Godot runs natively on macOS; ROS2 would be inside Docker. WebSocket (Godot 4's built-in `WebSocketPeer`) is therefore the *primary* transport, not the fallback the original §3.3 called it.

**D5 — The hivemind runs locally, not over an API.**
A GRPO-tuned **Qwen2.5-1.5B-Instruct** exported to Q4_K_M GGUF (~1.0 GB) served by `llama.cpp` on Metal. This deletes two of the original §10 risks outright (venue wifi, live API failure during judging) and makes the fine-tuning *load-bearing* rather than decorative — the thing on screen is the thing you trained. An Anthropic API provider stays available behind `--hivemind=api` for A/B comparison and as one rung of the fallback ladder.

**D6 — §14.3 MAPPO polish is demoted to stretch.**
Multi-agent PPO over 24 agents, on top of evolution *and* GRPO *and* a dashboard *and* an integration pass, on 8 GB, is where this plan breaks. MAP-Elites evolves interpretable behavior parameters directly (see [TECHNICAL.md §6](TECHNICAL.md)), which gets most of the specialization benefit at ~5% of the cost and produces an archive that is *more* legible on the dashboard, not less. MAPPO remains optional stretch scope.

**D7 — GRPO is reached by a ladder, not a jump.** See §5 below. This is the answer to "is there another option such as FreeSolo?"

**D9 — Robot perception is real computer vision, not a distance check.**
Until D3 victim discovery was `seen = dist <= sensor_radius` evaluated against ground-truth victim coordinates — the robot was *told* a victim was there. No image, no detector, no false positives. That made "the robots have computer vision" false, so it is now built properly:

- The world is rendered once per tick to an **appearance raster** (RGB-like, with noise), not a semantic map. Victims do not get their own channel; they must be *discriminated* from rubble that resembles them.
- Each robot's camera view is an **egocentric crop of that shared raster**, occluded by per-bearing raycasts. Rendering 512 separate views is infeasible on an M1; sampling one shared raster 512 times is a batched gather.
- A detector turns pixels into victim reports. **The classical detector ships first** (colour/shape discrimination + connected components, numpy only) and becomes the gate baseline; the CNN trains on Kaggle GPU and must beat it on held-out seeds or it does not ship.

Occlusion is computed geometrically and that is *not* a perception shortcut — a real camera physically cannot see through a wall. The simulation's job is to deliver the light that reaches the sensor; perception's job is what the detector makes of it.

**Consequence: victim discovery becomes probabilistic.** Missed detections and false positives are now possible, mission tuning shifts, and `/swarm/state` starts carrying detection confidence. This is a strictly better demo — it is what makes the hivemind's prioritisation a real decision — but it is not free.

**D10 — All GPU training runs on Kaggle; evolution stays local.**
MAP-Elites uses the 8 M1 cores (faster per-core than a Kaggle CPU session). The CV detector and the hivemind ladder both train on Kaggle GPU. Nothing trains in torch on the laptop.

**D8 — Robot count is measured, not chosen, and scaling to it changes three things.**
Per the original §2 method: sweep `N` until `DemoSim` real-time factor drops below 0.8x, then ship 80% of that ceiling. Planning default is **64**; measure before scaling because a large `N` invalidates three otherwise-reasonable designs:
1. **Bidding cannot call A\* per (robot, task) pair.** At 64 robots x 20 open tasks that is 1280 path searches per second. Replaced by a **BFS distance field per task target**, computed once per auction cycle (~2 ms each over a 192x128 grid) and read by every robot as an O(1) lookup. Cost becomes independent of `N`.
2. **Fog updates decimate.** Sensor/LOS runs at 5 Hz, and only for robots that have moved more than half a cell since their last update.
3. **The dashboard switches to `MultiMeshInstance2D`** above ~48 markers, and the hivemind prompt stays lane-aggregated (it already is) so token count does not grow with `N`.

The real ceiling is likely dashboard readability and prompt legibility rather than compute. If the sweep says 150 robots is free, take a number the audience can still parse - the measurement sets the *upper bound*, not the target.

### 0.2 Spec inconsistencies resolved

- Original §7 says "Eight elements required for MVP" then lists **twelve**; original §9 says "the four dashboard panels." Resolved: twelve items, tiered P0/P1/P2 in §4.3 below. P0 is the MVP bar.
- Original §2 ("Robot types: All 4") vs §4 ("no longer 4 fixed, hand-authored types"). Resolved: **four actuator lanes** (none / gripper / scoop / antenna) are fixed because the mission requires them; body and behavior *within* each lane are evolved.
- Original §14.4 says "Kaggle storage is persistent." Imprecise, and it will cost you a training run. `/kaggle/working` survives only as a *saved notebook version's output*. Resume works by attaching the previous version's output (or a private Dataset) as an input. Checkpoint logic is written this way from day one.

---

## 1. What is actually being built

A heterogeneous swarm of 512 robots searches a 320 m × 216 m disaster zone divided into 48 labeled sectors, finds and extracts 110 casualties, while a hazard spreads and one robot is destroyed mid-run. The owner-requested [Nepal confluence crop](NEPAL_TERRAIN.md) replaces the previous random terrain; historical mission results do not measure this new map.

> **Scenario numbers in §1–§3 were corrected to match `assets/scenarios/demo.yaml`, which is the only source of truth.** They had drifted twice (map 320×208 → 480×320 at D5d, casualties 8 → 80 → 120) and the document did not follow. That drift is not cosmetic: §6's 75% rescue target was set against the *first* of those scenarios and was still being quoted against the third. See §6.1.

Three control tiers, unchanged in spirit from the original spec:

- **Tier 1 — reflex, 20 Hz, no learning.** Collision avoidance and motor commands. Overrides everything above it. Nothing higher can make a robot hit a wall.
- **Tier 2 — tactical, 1 Hz, decentralized.** Market-based single-round reverse auction over tasks, with capability gating and heartbeat-driven task orphaning. *This is the entire self-healing mechanism and it contains no LLM.* Within an assigned task, behavior is driven by an evolved parameter vector tied to that robot's evolved body.
- **Tier 3 — strategic, every 6 s, a locally-run LLM** (base Qwen2.5-1.5B; the fine-tuning ladder in §5 was never run, and the gate ships the scripted rung — see `SHIPPING.md`).** Reads the blackboard, emits sector-level priorities and abandon directives with natural-language reasoning. Passes a feasibility filter before it can affect anything. **Never issues per-robot commands.**

The demo's central claim — *"the hivemind can go offline and the swarm keeps working"* — is true because Tier 2 has no dependency on Tier 3. That must stay true. Any change that makes Tier 2 need a directive to function is a regression, no matter how good it looks.

### 1.1 Why all four robot lanes are necessary (this was underspecified)

The original spec never says how the four types interlock, which makes "each type performs its unique skill" hard to satisfy honestly. The mission is designed so the rescue chain **requires** all four:

```
scout finds victim  →  (if buried) digger clears debris  →  carrier extracts to a safe zone
                    ↑
        all of it only registers on the blackboard if the robot is inside
        the comms component rooted at base — which relays extend
```

- **none (scout-like):** fast, light, no actuator. Explores. Reveals fog.
- **scoop (digger-like):** 32 of the 80 victims are buried and cannot be extracted until debris is cleared.
- **gripper (carrier-like):** victims must be physically carried to one of two extraction zones. Nothing else counts as a rescue.
- **antenna (relay-like):** a robot outside the comms component buffers its discoveries and they do not reach the blackboard or the hivemind. Relays extend the component. **This makes relays visibly matter**, and it is drawn on the dashboard as link lines plus greyed-out "out of contact" robots.

This is a strict improvement over the original spec.

---

## 2. Locked decisions

| Decision | Resolution |
|---|---|
| Perception | **Real CV.** Egocentric camera crops off a shared appearance raster, occluded; classical detector ships as baseline, CNN must beat it at the gate |
| Robot lanes | 4 actuator lanes; body + behavior evolved within each lane |
| Robot count | **768**, split **352 scouts / 176 diggers / 144 carriers / 96 relays** — the split is measured against comms coverage, not even (MEASUREMENTS.md M-33). Measured ceiling under the D1 controller was 3072 (M-1); mission size, not compute, is the binding constraint — see 0.1 D8 |
| Map | 320 m × 216 m, 1.0 m grid (69,120 cells), 48 sectors A1–F8; owner-requested Nepal confluence crop |
| Casualties | **120, 48 buried**, mild distance bias (`distance_weight_exp` 0.4, measured — M-17) |
| Mission length | 420 s hard cap |
| Hazard | Ignites t=90 s, `r(t) = 6.0 + 0.30·(t−90)` → ~105 m by t=420, drift 0.50 m/s from C4 |
| Fault injection | **State-triggered**, not wall-clock: first tick where `t ≥ 200 ∧ victims_rescued ≥ 10`; hard fallback at t=260. The threshold scales against what the swarm *achieves*, not against victim count — at 30 it was unreachable and the fallback fired every run ([M-46](MEASUREMENTS.md)) |
| Sim backbone | `FastSim`/`DemoSim` primary, `GazeboSim` optional stretch |
| Transport | `LocalBus` primary, `Ros2Bus` drop-in, WebSocket to Godot |
| Hivemind | Local GGUF (tuned) → local GGUF (base) → Anthropic API → scripted. Ladder, in that order |
| Dashboard | Godot 4, native macOS, 12 elements tiered P0/P1/P2 |
| Failure VFX | Particle burst + screen shake, one-shot on the `robot_failed` event |
| Audio | Cut |
| Speed/pause controls | Cut entirely (unchanged from original spec) |
| Multi-map | Deferred |
| **Demo maps** | **Four: seeds 42, 43, 44, 45**. `demo.yaml` `demo_seeds` is the only definition. Everything for the demo -- training, in-run checks, bounds, the gate -- runs on these four; other seed ranges stay in code, unused. Anything trained on them is **tuned for these maps**, and is described that way (§7.8) |
| Backup video | **Mandatory**, against the shipping build |
| RL | MAP-Elites **local** (8 M1 cores). CV detector + GRPO ladder on **Kaggle GPU**. MAPPO stretch. **Unit policy (§7.8): behaviour cloning → PPO on Kaggle CPU sessions**, never on the laptop (owner) |
| Everything ships behind a gate | Any trained component that fails §6 ships as its classical heuristic instead |

---

## 3. Mission design (concrete numbers)

These numbers live in `assets/scenarios/demo.yaml`. The Nepal crop uses reference-based dimensions; construction and routing checks are in [MEASUREMENTS.md](MEASUREMENTS.md), while full mission performance remains unmeasured on this terrain. Derivation in [TECHNICAL.md §3](TECHNICAL.md). The fast test suite uses `assets/scenarios/test.yaml` instead, which is deliberately tiny so `make check` stays seconds.

| Parameter | Value |
|---|---|
| Map | 320 × 216 m, cell **1.0 m** → 69,120 cells; reference-based confluence crop |
| Sectors | 48 (rows A–F × cols 1–8), each 40 × 36 m |
| Terrain | Two headwaters joining one outlet; 14–22 m channels, wooded shoulders, small terrace settlements and debris fans. Water and slope gate chassis access |
| Base / extraction | base `(30, 40)`; 11 further collection points on connected terrace roads |
| Robots | **512** — 171 / 117 / 96 / 128 by lane |
| Casualties | **110** total, **44 buried**, `distance_weight_exp` 0.4 (M-17, M-52) |
| Sim tick | 20 Hz · fog/LOS 5 Hz |
| Auction cycle | 1 Hz (announce → 0.3 s bid window → award) |
| Heartbeat / orphan timeout | 2 Hz / 2.0 s |
| Hivemind cadence / timeout | 6.0 s / 5.0 s hard (raised from 4.0 s — M-27) |
| Hazard | ignite t=90 s; peak r=31 m at t=240; burns out by t≈326; drift 0.335 m/s, origin C4 |
| Comms | base radius 40 m, relay radius 46 m; existing range retained for the new crop |
| Battery | 0.05 %/s idle, 0.20 %/s moving, 1.5 %/s inside hazard |
| Mission cap | 420 s — **pinned by the demo's own length, not tunable.** 420 s at 1× is the 7-minute run in §4 |

**Scorecard** (printed by every headless run, and the thing the gate and the reward function both read):
`victims_rescued / 110`, `victims_found / 110`, `ground_explored_frac`, `robots_lost`, `mean_time_to_rescue`, `energy_used`, `directives_issued / rejected`, `wall_seconds`, `rtf`.

---

## 4. Demo script (rehearsed against the real clock)

The demo follows these observable events.

| Beat | What the operator does | What must be on screen |
|---|---|---|
| **Cold open** | Nothing. Eagle-eye, fog on, sectors on. | Fully fogged 48-sector map, 512 robots massed at base, HUD `0/80`. |
| **First directive** | Nothing. | Reasoning feed prints the hivemind's opening explore priorities, with a `source: base-local` badge — **not** `tuned-local`; no model was trained, and `mission.py` correctly leaves `tuned=False`. Auto-pan pulses the prioritized sectors. |
| **Exploration** | Nothing. | Fog peels back sector by sector. Ticker fires constantly with Tier-2 `last_action_reason` lines. Victims start logging. Relay chain visibly extends; a robot that outruns the chain greys out and reconnects. |
| **Follow-cam** | Click a scout. | Camera follows it; side panel shows its live `last_action_reason`. Talk over this: "this is Tier 2 — no LLM in this loop." |
| **God view** | Toggle god-view for ~8 s, then back. | True victim pins and true hazard extent appear next to what the swarm actually knows. *This is the "the hivemind doesn't cheat" beat.* |
| **Hazard ignites** | Nothing. | Hazard blooms at C4 and starts drifting. *(See [M-46](MEASUREMENTS.md).)* |
| **Reprioritize** | Nothing. | Hivemind reasoning feed explicitly abandons the threatened sector. Robots inside it get `retreat` tasks and visibly leave. Auto-pan snaps to that sector. *This is the "watch the AI think" beat.* |
| **Failure** | Nothing (state-triggered). | Fire/smoke particle burst + screen shake on the doomed robot. HUD active-count ticks down. Within 1–3 s the event log shows `task_orphaned` → `task_awarded` to a different robot, and the ticker calls it out. *This is the "watch it self-heal" beat.* |
| **Hivemind offline** | Press `H`, count to ten, press `H` again. | Reasoning feed flips to a red `HIVEMIND OFFLINE` banner. Swarm keeps bidding, rescuing, self-healing. **Hold for ~10 s, no longer** - the point lands immediately and dead air costs more than the beat is worth. One narration line: "Tier 2 has no LLM in it." |
| **Endgame** | Nothing. | Remaining victims found, dug out, carried in. HUD climbs to final. |
| **Close** | Nothing. | Final scorecard overlay. |

**Narration hooks, one line each:** three tiers / auction is the self-healing / relays gate what the AI can even see / the model runs locally on this laptop with wifi off, schema-constrained so it can only say things the swarm can act on. **Not "fine-tuned"** — the ladder in §5 was never run, and §10 is explicit that an overstatement a judge catches discounts everything else.

### 4.3 Dashboard element tiers

**P0 — MVP bar, must ship by D5.** These are the demo.
1. Fog-of-war map · 2. Robot markers (lane color × status) · 3. LLM reasoning feed · 4. Event/timeline log · 5. Eagle-eye camera · 10. HUD stats bar

**P1 — the beats, must ship by D9.**
6. Fog/god-view toggle · 8. Victim ground-truth toggle · 9. Robot follow-cam + reason panel · 11. Tier-2 auto-caption ticker · Failure VFX (particles + screen shake)

**P2 — polish, ship if D13 has room.**
7. Sector boundary toggle · 12. Auto-highlight/pan · comms link lines · MAP-Elites archive heatmap panel · hivemind source badge

---

## 5. The RL ladder — answering "GRPO, or something like FreeSolo?"

> **Status: never run.** This section is the plan as written on D0 and is kept for the reasoning. No rung was climbed — `swarmmind/training/llm/` has never contained a file. The ladder sits at cut lines 5-7 in §7.1, the gate ships the scripted rung, and by D14 the remaining slack was spent on the CV detector instead (a smaller, self-contained job). Do not describe the hivemind as tuned.

**There is no "FreeSolo" in LLM post-training.** FreeSOLO is a 2022 unsupervised instance-segmentation paper — unrelated. What is real, and worth knowing, is a ladder of methods that all reuse *the same expensive part*: a rollout harness that scores a candidate directive by simulating forward from a swarm state. Building that harness once is ~80% of the work; which rung you climb after that is cheap to change.

| Rung | Method | What it does | Cost on a Kaggle T4 | Risk |
|---|---|---|---|---|
| 1 | **RAFT / RFT** (rejection-sampling fine-tune) | Sample G directives per state, keep the best by simulator score, plain SFT on the winners | ~2–3 h | Very low. Just SFT. |
| 2 | **DPO** on simulator-scored pairs | Best vs. worst of the same G becomes (chosen, rejected). Offline, no vLLM needed | ~2–3 h | Low. Very stable. |
| 3 | **GRPO** *(the target)* | Group-relative advantage over G sampled directives, online, RLVR-style verifiable reward | ~6–9 h | Moderate. Needs vLLM generation, reward tuning, tolerant of neither reward bugs nor identical-reward groups. |
| — | *also real, if GRPO misbehaves* | **RLOO** (simpler group baseline, TRL `RLOOTrainer`), **Dr. GRPO** (`loss_type="dr_grpo"`, removes length bias), **DAPO** tricks (clip-higher, dynamic sampling, token-level loss), **GSPO** (sequence-level ratios) | — | These are GRPO variants, not detours. |

**The plan: climb the ladder, banking each rung.** RAFT on D10 gives a shipped, tuned model by end of D10 no matter what happens after. DPO on D11 morning is a strict upgrade attempt. GRPO on D11 night is the headline. Each is evaluated against the same held-out gate (§6); whichever wins ships. This is the difference between "we fine-tuned it" as a fact and as a hope.

**Two implementation details that decide whether GRPO works at all:**
1. **Same seed for every candidate in a group.** Different seeds inject state-difficulty variance straight into the group-relative advantage and drown the signal.
2. **Curriculum on the reward.** Steps 0–200 score *only* JSON validity + feasibility-filter pass. Mission reward switches on after. Skipping this is the standard way GRPO runs burn six hours learning to emit brackets.

Full reward spec and dataset construction: [TECHNICAL.md §7](TECHNICAL.md).

---

## 6. The gate — what "good enough" means before anything ships

Checked independently per component. **Revised: on the four demo maps, seeds 42–45**
(`demo.yaml` `demo_seeds`), because those are the only maps the demo plays. The ten held-out
seeds 101–110 remain available (`gate.py --held-out`) and are no longer the default. Components
trained elsewhere (bodies: test fixture 11–13; detector: 1–8) have still never seen the demo maps;
**the unit policy is trained on them**, so its margin is a margin on the demo only.

| Component | Ships if | Otherwise |
|---|---|---|
| Evolved lane (per actuator lane) | mean lane-score ≥ **1.10 ×** the classical heuristic baseline **and** task success rate ≥ **0.60** | that lane ships the classical heuristic. Evolution is not required to win everywhere. |
| Hivemind (RAFT / DPO / GRPO checkpoint) | mean mission reward ≥ **1.05 ×** the prompted-base-model baseline, **and** feasibility-filter rejection rate ≤ **10%**, **and** p95 latency ≤ **4.0 s** locally | next rung down the ladder, ending at the prompted base model, ending at the scripted directives. |
| Zone routing (Tier 1 correctness fix, §7.8) | mission score ≥ **1.05 ×** coarse routing on the demo maps | coarse routing stays, as on `main` |
| Unit policy (PPO checkpoint, §7.8) | mission score ≥ **1.05 ×** the **better** of routing-alone and the heuristic staging rule, on the demo maps | the better of those two ships |
| Whole system — **behaviour** | `tests/test_mission.py` passes with Tier 3 silent: every lane performs its lane-unique skill, the failure event fires, the orphaned task is reassigned within 3 s | do not ship the change. |
| Whole system — **mission quality** (revised , §6.1) | On demo seeds 42–45, Tier 3 off: **delivery ≥ 55%** of casualties *found* are rescued, **and discovery ≥ 55 of 110** on seed 42 | do not ship the change. |

**The gate is a real gate.** Shipping a heuristic that beat the trained thing is a *result*, and it is a better demo answer than a trained thing that quietly underperforms. Say so to judges if asked.

### 6.1 Why the 75% rescue target was replaced (revised , D12)

**The old bar was ≥ 75% of casualties rescued. It is arithmetically unreachable on this
scenario and it was never revised when the scenario changed underneath it.**

Measured, four seeds, both Tier-3 arms, on the demo machine
([M-39](MEASUREMENTS.md), [M-40](MEASUREMENTS.md)):

| | before D12 | after D12 | after the D13 rescale |
|---|---:|---:|---:|
| casualties **found** | 28.50 / 120 | 29.75 / 120 | **51.25 / 80** |
| casualties **rescued** | 16.25 / 120 | 18.00 / 120 | **32.50 / 80** |
| **delivery** — rescued of found | 57.0% | 60.5% | **63.4%** |

**Only ~25% of casualties were ever found, so rescuing everything discovered was ~25%.**
No component-level fix changed that: D9–D12 attacked it from five directions (Tier 1
stalling, comms gating, late discovery, store-and-forward, the relay/frontier coupling)
and each found a different constraint underneath the last. The ceiling was a statement
about map size against a mission clock that §3 pins at 420 s because the *demo* is seven
minutes long.

> **Updated configuration.** The scenario was rescaled — 360 × 240 m, 512 robots, 80 casualties —
> and discovery went **24% → 64%**, rescues **15.6% → 40.6%** ([M-47](MEASUREMENTS.md)).
> The ceiling argument above is therefore about the *old* map. It is kept because the two
> numbers this gate is built from did not change: delivery held at ~63% across both
> scenarios, which is what makes it the right thing to gate on. 75% rescued is still not
> reachable — it would need ~75% discovery *and* near-perfect delivery — but it is now out
> by roughly 1.8x rather than 3x, and the honest reason to keep the split gate is that it
> measures the rescue chain rather than the map.

**And 75% is a stale constant.** It was set against a 320 × 208 m map with 8 casualties.
The scenario has since become 480 × 320 m with 120, and the target never moved with it —
the same failure as the flat 24-task announcement cap, the even 192/192/192/192 lane split
and the obstacle-cluster count, all of which are recorded in
[FIXES.md §1.7](FIXES.md). Revising it is catching the fourth instance of a known bug,
not moving a goalpost.

**What replaces it, and why these two numbers.**

- **Delivery — rescued ÷ found, bar 55%.** This is what the rescue chain actually claims:
  the auction, the four lanes, digging, carrying and self-healing. It is the number a
  regression in any of them moves. Measured band is 57–61% across four seeds and both
  Tier-3 arms, so a drop below 55% means something broke rather than drifted.
- **Discovery — found of 80, floor 40 on seed 42.** Without this, exploration could
  regress invisibly: find fewer casualties and still deliver 60% of them. A single seed is
  deterministic, so this bar has no noise to absorb and can sit close: measured 49 with
  Tier 3 off, 50 on, and the floor is 40. Discovery varies far more across seeds
  (49/60/54/42) than delivery does, which is why the floor is pinned to one reference seed
  rather than averaged. Re-derived at the D13 rescale; it was 30 of 120.
- **Both are reported absolutely on the scorecard.** The ratio is the gate; `rescued 21 /
  80` is what goes on screen and into `SHIPPING.md`.

**Where it is measured.** `tests/test_mission.py` runs the tiny `test` fixture in seconds
and guards invariant #1 and the 3-second self-heal — it is not, and should not become, a
mission-quality benchmark. Mission quality is measured on the demo scenario by
`scripts/baseline_sweep.py --seeds 42 43 44 45`, ~10 minutes on four workers, which is
also the before/after harness any future change to the swarm is measured with.

**What is said to judges.** *"It delivers about 60% of what it finds, in a zone
deliberately too large to search in seven minutes — which is why prioritising under
uncertainty is the problem worth solving."* Both halves of that sentence are measured, and
the second half is the reason the mission is interesting rather than an excuse for the
first. **Do not quote 75% anywhere; it never described this scenario.**

---

## 7. Priorities

### 7.1 Hard cut lines

If scope needs reducing, cut in this order — top first:
1. `GazeboSim` (already optional)
2. MAPPO polish (already stretch)
3. P2 dashboard elements
4. **The learned CV detector → ship the classical detector.** Perception stays real either way; only the "trained" label is lost
5. GRPO → ship the DPO or RAFT checkpoint
6. DPO → ship the RAFT checkpoint
7. RAFT → ship the prompted base model
8. Evolved policies → ship classical heuristics for all lanes
8a. **Unit policy (§7.8) → not shipped, already cut.** Gated at 1.04x / 0.997x live (M-76d, M-76e). The code is on `main` behind a flag that defaults off; cutting it costs nothing
9. Local model → ship the Anthropic API provider (accepts wifi risk)
10. **Never cut:** M1, M2, the auction's self-healing, real perception, the backup video.

---

---

## 7.5 Alternative commander architecture — evaluated, not shipped

The original architecture described a project where Tier 3 was a language model issuing sector
priorities. That is no longer what this is. The design changed:

* **The mind is a trained policy, not the LLM.** `nodes/command.py` is Tier 3a — it owns
  *territory*, decides which squadron searches where, and runs every cycle in
  microseconds. The language model became an **advisor**, consulted occasionally, out of
  the control loop. Its latency stops mattering, which removes the timeout problem
  entirely.
* **Squadrons** are the unit of command: 48 robots holding a contiguous wedge.
* **A fourth chassis, the rotor** — flies at 2x, blind until it lands, cannot carry.
* **Procedural maps** (`sim/generator.py`), so the commander learns to search *a* map
  rather than *the* map.


### Measured speed, for sizing what follows

| | rate | source |
|---|---|---|
| Tier-2 genome evaluation | 1,418/hour, 7 workers | M-30 |
| commander evaluation (3 generated maps, 180 s) | ~86 s each, ~290/hour on 7 workers | measured  |
| full demo mission, headless | ~4.5 min | M-24 |
| local model, one directive | 3.6 s | M-27 |

## 7.6 Tier 3a (the commander) is closed — measured, gated, not shipped

**Decided on the evidence, not on taste.** Three CMA-ES runs over
`CommandParams`; full record in [`training_notes/SUMMARY.md`](../training_notes/SUMMARY.md).

| arm | score | rescued | explored |
|---|---:|---:|---:|
| **none** | **70.81** | 52 / 768 | 28.3% |
| heuristic | 65.57 | 47 / 768 | 27.3% |
| trained | 69.42 | 52 / 768 | 28.5% |

Training was **not** wasted: the hand-set commander was *actively harmful* (0.93× — worse
than no commander at all) and training made it neutral (0.98×). But neutral does not ship.
The three-arm gate is why that can be stated honestly — "trained beats the heuristic by
5.9%" is true, sounds good, and means nothing.

**Why a territory policy could not win.** Tuning seven numbers cannot repair a mechanism,
and three faults were found by reading `nodes/command.py` rather than the training curve:

1. **Relay tasks are territorially fenced** (`auction.py:157`). A relay must often sit in a
   neighbouring squadron's wedge; fencing it strangles the comms chain everything depends
   on — the cascade CLAUDE.md documents.
2. **Wedges are bearing-cut pie slices.** A far slice covers enormous area, a near one
   almost none, with the same robots either way.
3. **The value function clips at zero**, so a large `w_distance` flattens the split
   entirely — run 1 learned to *stop partitioning*, the degenerate imitation of "no
   commander".

**The code stays.** `command=False` is the default and the gate keeps all three arms, so
the claim remains reproducible and `SHIPPING.md` can state it. Deleting it would delete
the evidence. Anyone reopening this must change one of the three mechanisms above and
re-gate — **not** search the same parameterisation harder.

**What this says about the demo narrative:** Tier 3a's contribution was always meant to be
*coordination*, not information — it reads exactly the same fused picture every unit does.
The swarm is decentralised, and that is now a measured claim rather than a fallback.

---

## 7.7 The rescue rate — the only remaining problem that can sink the demo

Measured , [M-39](MEASUREMENTS.md), demo seed 42 with the evolved roster:

    rescued 19/120 (15.8%)   found 30/120   explored 37.3%   lost 102/768   mttr 194 s

**Only 30 casualties are ever *found*, so rescuing everything discovered is 25%.** The
rescue chain is not the constraint; discovery is. Work aimed at extraction, allocation or
Tier 3 is aimed at the wrong link.

> The bar this was measured against — a flat ≥ 75% rescued — **was replaced on **
> after D12 confirmed the ceiling holds across seeds. See **§6.1**. The paragraphs below
> are the D11 record that led to that decision and are left as written.

### The binding mechanism

Three numbers plateau together at ~t=180: comms reach **~240 m** (of a 577 m diagonal),
explored **37.3%**, relays on post **~46 of 92**.

`TaskGenerator.relay_posts` builds each chain radially from base toward a **frontier
target**. The frontier is the edge of *explored* ground; explored ground ends at the comms
boundary, because `mark_seen` only writes to the shared map for in-contact robots. **So the
relay network can never be sent ahead of the swarm — each link waits for exploration that
is waiting for that link.**

This is a mechanism fault, not a tuning one. Raising `posts_per_chain` lengthens a chain
that still has nowhere to be sent. The fix is to place posts toward *unexplored* territory
beyond the frontier.

### Second finding: hazard is the dominant cause of loss

    hazard 84 · battery 17 · scripted fault 1   (102 of 768 lost)

82% of losses are the spreading fire, concentrated in the last third of the mission
(768 alive at t=240, 666 at t=420) — precisely when late discoveries would need carrying
home. Not yet separated: hazard reflex under-weighted vs `growth_rate: 0.30` simply being
too aggressive for a 420 s mission.

### Ordered next steps

1. ✅ **Break the relay/frontier coupling.** Done , [M-40](MEASUREMENTS.md). Chains
   are now aimed a bounded distance *past* the frontier; the value saturates, so the
   mechanism is bound by the 96-relay lane rather than by a tuned number.
2. ✅ **Re-measure.** Four seeds x both Tier-3 arms, before and after, on this machine —
   `scripts/baseline_sweep.py`, `runs/sweep_{before,after}.json`. M-36's warning held up:
   the two guard clauses that came with the fix measured as nothing on their own, and only
   the extension moved anything.
3. ✅ **Decide the target.** Done  — **option (a)**. The whole-system gate is now
   two numbers, delivery (rescued ÷ found, bar 55%) and discovery (found of 120, floor 25),
   both reported absolutely. Written up with the measurement behind it in **§6.1**.
   Option (b) — shrinking the map or casualty count until 75% is reachable — was rejected
   because mission length is pinned by the demo at 420 s, so rebalancing means removing
   the very thing that makes prioritisation a real decision. Option (c) was rejected
   because the swarm's actual result is worth reporting.

**D12 is therefore closed.** The remaining rescue-rate items (relay re-aiming, comms area
rather than reach, hazard growth rate) are in [FIXES.md §12](FIXES.md) and are worth a
point or two of a metric that is no longer the headline. **They are not the next work.**

### What the fix did not fix

M-40's discovery ceiling is the same ceiling M-39 measured. Comms reach rose 20% and bought
2.2 points of exploration, because reach is a *max* distance and coverage follows area. The
relay lane also commits fully within 30 s and never re-aims. Both are open, and both are
smaller than the target decision above.

### What is deliberately not on this list

`GazeboSim`, `Ros2Bus`, the browser fallback view, MAPPO, and a neural commander. The
neural commander is the rung above the evolved one and only earns a slot if the evolved
one clears the gate early — CLAUDE.md's ladder rule, applied to the thing we just built.

---

## 7.8 Unit policy and zone routing — branch `rl/unit-policy`

**The request:** RL-train the individual units, on Kaggle, using the ~29 hours available.
**The constraint:** `main` is the demo and is frozen for rehearsal on ; this work lives
on branch `rl/unit-policy` in a separate worktree, uncommitted, and swaps into the demo only if
it clears the gate *and* the owner decides to re-rehearse.

### What was measured before anything was trained (MEASUREMENTS.md M-76, M-76a)

| question | answer |
|---|---|
| Send searching robots to inspect unconfirmed contacts? | **No.** Crude rule −7.25 rescued; a ground-truth **oracle** choosing only real casualties +0.5. No headroom — dropped |
| Where does rescue-chain time go? | carrying 56%, **waiting for a carrier 33%**, waiting for a digger 11% |
| Why were 34 of 39 late casualties still being carried at the buzzer? | **A routing bug**, not a policy: the coarse flow field trapped loaded carriers against rock near collection points |
| Fix it? | `control/zone_routing.py`: +8.1 rescued over 8 seeds on Kaggle (1.097×), **+15 on 42–45** |
| Does heuristic staging help on top? | +0.9 rescued (1.008×): down on four seeds, up on four |

### Decisions

- **The demo plays four maps, 42–45** (owner, ; §2). Training, checks, bound and gate all
  run on those. This removes the held-out question for the demo by design: the policy is tuned
  for the maps it will be shown. It must be described that way.
- **Action space: per-robot staging for carriers and diggers** with nothing urgent (wait at a
  dig site or a confirmed contact), plus "dark ground", "hold", and the default, which is always
  offered and reproduces `main` byte for byte. The auction still allocates everything; a staged
  robot is withheld only from *search* work (TECHNICAL §7a).
- **Ladder:** heuristic staging rule (classical baseline) → behaviour cloning of it → PPO.
  Numpy, no torch. The bar is the better of routing-alone and the heuristic, never the weaker.
- **Compute:** Kaggle CPU sessions, ~13.8 demo missions/hour; one iteration = one episode per
  demo map (~17 min). ~11 h per session, resumed from the previous version's output.
- **Zone routing: kept, off by default, and the reason is measured (M-76f).** The fix is worth
  +7.5 to +15 rescues with Tier 3 silent on two machines, and in the demo's own configuration the
  demo machine says −1.25 where Kaggle says +9.5. The demo keeps its rehearsed build; no beat
  sheet or backup video changes.
- **The unit policy is kept and unused.** 1.04x at the gate, 0.997x live. The owner's call was to
  keep the code rather than delete it, so the result stays reproducible.

## 8. Risk register

| # | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| R1 | 8 GB RAM exhausted with Godot + Python + llama.cpp running together | Medium | Demo-fatal | Budgeted at ~4.7 GB total ([TECHNICAL.md §9](TECHNICAL.md)). Measured on D7. Fallback: drop model to Q4_0 or run hivemind at 8 s cadence. |
| R2 | Local 1.5B inference exceeds the 4 s budget on M1 | Medium | High | Cap `max_tokens` at 220, reasoning capped at two sentences, JSON-schema-constrained decoding, inference on a worker thread so the sim never blocks. Fallback: previous directive persists; ladder drops a rung. |
| R3 | GRPO fails to beat base on the gate | **Medium-high** | Low | *This is why the ladder exists.* RAFT is banked on D10. Worst case ships a strictly-better-than-base model anyway. |
| R4 | Kaggle session dies mid-run (9 h cap, or preemption) | High | Medium | Checkpoint every N steps and at 8 h wall clock; resume via output→input chaining. Written on D10, not after the first loss. |
| R5 | Evolution produces degenerate bodies (everything converges to one lane) | Medium | Low | Archive measures are `(actuator_lane, measured_speed)`; per-lane emitters guarantee coverage. Gate falls back per-lane. |
| R6 | Reward hacking — hivemind learns to game the metric | Medium | Medium | Reward includes a churn penalty and a feasibility term; gate uses held-out seeds; manually read 20 sampled directives per checkpoint. If the reasoning text stops matching the directives, that is the tell. |
| R7 | Determinism drifts (dict ordering, float accumulation) and rehearsed timings stop holding | Medium | High | `test_determinism.py` hashes the scorecard on seed 42 every run. Single seeded RNG per subsystem. No `set` iteration in the sim loop. |
| R8 | Godot ↔ Python WS backpressure at high `N` × 10 Hz | Low | Medium | ~200 B/robot/tick: 64 robots ≈ 13 KB/tick ≈ 130 KB/s; 256 robots ≈ 500 KB/s, still fine locally. Drop-oldest on the `state` queue (snapshots are idempotent), never-drop for events. If it ever bites, delta-encode `state`. |
| R9 | Integration between sim / nodes / dashboard slips | Medium | High | Contract frozen D7. Rough integration at M2 (D4), not at the end. Schema round-trip tests. |
| R10 | Live demo fails for non-code reasons | Low | Total | Final backup video D13. Three cold-boot rehearsals D14. Run with wifi off by design. |
| R11 | Scope creep from the original spec's stretch list | **High** | High | §7.1 cut lines are written down *before* the pressure arrives. |
| R12 | Gazebo attempted at the event and eats the 36 hours | Medium | Medium | `GazeboSim` is explicitly non-critical-path and time-boxed to 6 hours at the event. |

---

## 9. Success criteria

MVP is working when, in a single unattended run on seed 42:

- [ ] All four actuator lanes are present and each performs its lane-unique skill at least once (enforced by the rescue chain in §1.1, asserted in `test_mission.py`).
- [ ] Fog-of-war visibly clears as exploration happens.
- [ ] The hazard visibly spreads, and the reasoning feed contains a directive that explicitly reprioritizes or abandons because of it.
- [ ] The scripted failure fires, and the failed robot's task is reassigned to a different capable robot within 3 s, logged, with no manual intervention. **At 768 robots this needs the dashboard to pan to it** — one robot in 768 is not self-evident.
- [ ] The run completes in 5–7 minutes with no operator restart.
- [ ] The whole thing runs with **wifi off**.
- [ ] `SHIPPING.md` accurately states which components are trained and which are heuristic.

---

## 10. What gets said to judges (accuracy notes)

- It is a **market-based single-round reverse auction**, not CBBA. Do not say CBBA.
- Fog-of-war is **proximity-based reveal**, not SLAM.
- It is a **ROS2-compatible node graph on a pluggable transport**. Only say "running on ROS2" if `Ros2Bus` is the active transport at that moment.
- The physics is a **custom 2.5D kinematic simulator**, not Gazebo — unless `GazeboSim` is actually running.
- The hivemind is **not fine-tuned**. No rung of §5's ladder was ever run — `swarmmind/training/llm/` is empty and the GGUF is stock Qwen2.5-1.5B-Instruct. The true and still-impressive description is **locally served, wifi off, with JSON-schema-constrained decoding**; the dashboard badge reads `base-local` and is correct. "RAFT-tuned" would only have been a real answer if RAFT had run and cleared the gate.
- Tier 3, updated: **on the four demo maps the scripted rung clears the bar at 1.05x** (+3.75 rescued, M-76d), where on ten held-out seeds it measured 1.02x (M-45) and on some four-seed samples it measured a net cost (M-65). Say **"on the maps we demo it helps, and we have not shown it generalises"**, and say it is an advisor the swarm is free to ignore — the `H` beat proves that either way.
- If a lane ships its heuristic because evolution lost, **say so**. It is a stronger answer than the alternative and it is what the gate is for.

---

## 11. Immediate next actions (D0 → D1)

1. Install `uv` (`curl -LsSf https://astral.sh/uv/install.sh | sh`) — no Homebrew needed.
2. `uv python install 3.12`, `uv init`, pin deps.
3. Download Godot 4.4 stable `.dmg` from godotengine.org.
4. Build or download `llama.cpp` for macOS arm64 with Metal; pull `Qwen2.5-1.5B-Instruct` Q4_K_M GGUF into `assets/models/`.
5. `git init`, scaffold the tree from [TECHNICAL.md §2](TECHNICAL.md), commit these three docs.
6. Confirm a Kaggle account with phone verification (required for GPU) and note the weekly quota reset day.
