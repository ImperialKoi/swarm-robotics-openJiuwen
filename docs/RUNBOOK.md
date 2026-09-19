# RUNBOOK — demo day

**Current build:** use [MULTI_AGENT_DEMO.md](MULTI_AGENT_DEMO.md) for the current
build, response-team setup, live disable command and measured limitations. The current
scenario has 512 robots and **110 casualties**. The earlier timings and scores below
are historical; they have not been re-established for this scenario or the new team.

Measurement configuration: **after** the scenario rescale — 360 x 240 m, 512 robots, 80 casualties, 96 relays. Anything measured before that rescale does not apply.

---

## 1. Pre-flight — the machine, not the code

**This is the part that fails.** The code is deterministic; the laptop is not.

- [ ] **Cold boot.** Not a restart of the app — a reboot.
- [ ] **Quit everything else.** Browser, editor, Slack, Docker if it ever gets installed.
      Measured: with an editor and a browser open, the demo grew swap by **561 MB** and
      real-time factor decayed from 1.00x to 0.89x mid-run ([M-41 pre-flight
      note](MEASUREMENTS.md)). The three processes fit; the machine sharing them does not.
- [ ] **Wi-Fi off.** By design — nothing in the demo path needs the network, and turning
      it off in front of the audience is a better demonstration than saying so.
- [ ] `sysctl vm.swapusage` — note the baseline. It should be near zero after a reboot.
- [ ] Display scaling set and the Godot window sized *before* anyone is watching.
- [ ] Backup video open in a second Space, ready to switch to.

**The memory budget, measured with all three resident:**

| | peak |
|---|---:|
| llama-server | 984 MB (still climbing at t=139 — sample longer) |
| simulator | 224 MB |
| Godot | 1,527 MB — **4.4x the 350 MB budgeted in TECHNICAL §9** |
| total | 2,586 MB |

An **exported** Godot build rather than the editor is the untested lever if this bites.
Test it before demo day, not on it.

---

## 1a. Which map

**The demo plays four maps: seeds 42, 43, 44, 45** (`demo.yaml` `demo_seeds`).
Nothing has been rehearsed, tuned or gated on any other seed, and `cli run --demo` prints a note if
you start one. The beat sheet below is measured on **seed 42**; the other three are the fallbacks
if a map needs to change between runs. Pass `--seed` explicitly every time.

**What is on this build and what is not.** The demo runs coarse routing and no unit policy --
the build these timings were measured on. `control/zone_routing.py` and `control/unit_policy.py`
are in the tree and **both default off**: routing because its gain does not survive on this
machine with Tier 3 live (M-76f), the unit policy because it gated at 1.04x (`SHIPPING.md`).
Turning either on means re-running `scripts/beats.py`, re-timing this sheet and re-recording the
backup video before anyone narrates it.

## 2. Start order

Four terminals. Wait for each to settle before the next.

```bash
# 1. the model. Wait for "listening on 8080".
./scripts/serve_hivemind.sh

# 2. the simulator, holding at t=0 until the dashboard connects.
uv run python -m swarmmind.cli run --demo --wait

# 3. Godot: open the project, press F5.
#    The sim starts the moment it connects. Watch for "connected  demo  512 robots".

# 4. optional, only if you want the numbers afterwards
uv run python scripts/residency.py --seconds 480
```

**If the model server is not running, nothing breaks.** `build_ladder` skips the rung in
under a millisecond — connection refused is instant on loopback — and Tier 3 falls to the
scripted baseline. The demo runs. Do not let a failed model start turn into a scramble.

---

## 3. Historical beat sheet — seed 42, earlier scenario

| clock | beat | operator | what must be on screen |
|---|---|---|---|
| **0:00** | Cold open | nothing | Fully fogged map, 512 robots massed at base, HUD `0/80` |
| **0:00** | First directive | nothing | Reasoning feed prints within a tick — Tier 3 runs every 6 s from t=0 |
| **0:32** | First rescue | nothing | HUD ticks to `1/80`; ticker calls the extraction |
| **2:15** | First casualty dug out | nothing | `victim_cleared` in the log — the digger lane earning its slot |
| **1:00–1:30** | Exploration | **click a scout at ~1:30** | Fog peels back. Follow-cam + `last_action_reason`. *"This is Tier 2 — no LLM in this loop."* |
| **1:30** | **Hazard ignites** | nothing | Fire blooms at C4 and starts drifting |
| **2:00** | God view | toggle for ~8 s, then back | Bodies lying in the rubble, with pins over them, next to what the swarm actually knows. *The "it doesn't cheat" beat* |
| **3:00** | Reprioritise | nothing | Reasoning feed abandons the threatened sector; robots retreat out of it |
| **3:20** | **Scripted failure** | nothing | Particle burst + screen shake. Within 1–3 s: `task_orphaned` → `task_awarded` to a different robot. *The "watch it self-heal" beat* |
| **Separate check** | Hivemind offline | Start with `--no-hivemind` | Auction still runs. No H toggle is implemented. For live response-team termination, follow the current demo guide. |
| **3:18** | Sector abandoned | nothing | Tier 3 closes a burning sector; robots there get `retreat` |
| **4:30** | Fire peaks | nothing | Hazard stops growing at r≈45 m and begins receding |
| **4:30–7:00** | Endgame | nothing | Swarm reclaims burnt ground; HUD climbs. Ends ~`27/80` |
| **7:00** | Close | nothing | Final scorecard |

The 3:20 failure is **state-triggered** — first tick where `t ≥ 200` and `rescued ≥ 10` —
with a hard fallback at 260 s. On seed 42 the condition is met at exactly 200.0 s.

---

## 4. What to say

Four narration hooks, one line each:

- Three tiers: reflex at 20 Hz, a market auction at 1 Hz, a language model every 6 s.
- **The auction is the self-healing.** No LLM in that loop; `--no-hivemind` verifies it.
- Relays gate what the AI can even see. A robot out of contact reports nothing.
- The model runs locally on this laptop, wifi off, with JSON-schema-constrained
  decoding so it can only emit directives the swarm can act on.

> **Do not describe the language model as fine-tuned.** The hivemind training ladder was
> not run. Its GGUF is stock Qwen2.5-1.5B-Instruct; the model-backed team also uses
> `base-local`. Other components have separate training/evaluation records.

### The numbers, and how to say them

Historical measurements below span different configurations. The current generated
`SHIPPING.md` covers the four demo maps and now distinguishes gate outcomes from active
defaults. Do not transfer a historical margin to the new response team.

| | say |
|---|---|
| Evolved bodies | **"MAP-Elites bodies ship — 1.17x the hand-set baseline on ten maps they never saw."** They rescue 15% more and lose 31% fewer robots while exploring *less*. |
| Tier 3 | **"On the four maps we demo, it clears our bar: 1.05x, +3.75 rescued."** And the other half, if asked: on ten held-out maps it measured 1.02x, so it is proven *here*, not everywhere (M-45, M-76d). |
| Commander | **"We trained it three times and it doesn't ship."** 0.98x. This is a strong answer — say it plainly. |
| CV detector | **"The classical detector ships; the CNN was trained but missed its shipping margin (M-74)."** Perception is real either way. |
| Rescue rate | **"It finds ~65% of the casualties and delivers ~59% of what it finds."** Both measured on held-out seeds. |
| Why not everyone | **"Nothing found after about five minutes can be carried home before the clock stops"** — measured, 0 of 19 (M-50). That is the mission's real constraint and it is worth saying. |
| Unit policy | **"We trained a per-robot policy with PPO on these four maps and gated it out at 1.04x."** It beat the hand-written rule it was cloned from and not the bar; with Tier 3 live it is 0.997x. Four trained components, one ships — say that plainly. |
| Zone routing | **"We found carriers trapped against rubble a few metres from drop-off points. The fix is worth +15 rescues with the strategic layer off, and we could not show it helps the configuration we demo, so it is in the code and switched off."** The second half is the interesting part: two layers fixing the same congestion. |

### Never say

| say | not |
|---|---|
| market-based single-round reverse auction | CBBA |
| proximity-based fog-of-war reveal | SLAM |
| ROS2-compatible node graph on a pluggable transport | built on ROS2 |
| custom 2.5D kinematic simulator | Gazebo |
| **ground explored** | "sectors explored" — the metric is ground coverage ([M-44](MEASUREMENTS.md)) |
| — | **75% rescued.** That target was retired; it never described this scenario ([PLAN §6.1](PLAN.md)) |

**"Directives issued" means two things.** The scorecard counts *decisions* (~21 on seed
42); the event feed publishes every cycle (70). If a judge counts the feed and asks, the
difference is renewals — a directive re-sent to hold a position is not a new decision
([M-25](MEASUREMENTS.md)).

---

## 5. When it goes wrong

| symptom | cause | do this |
|---|---|---|
| `cannot listen on 127.0.0.1:8765` | a previous run is alive | `pkill -f swarmmind.cli`, or `--port 8766` and set `SWARMMIND_WS` for Godot |
| Dashboard stuck on "waiting for simulator" | connection dropped on an oversized frame ([M-12](MEASUREMENTS.md)) | Check the status line — it shows `receiving map 5/9`. If it never advances, restart Godot first, not the sim |
| `CONTRACT MISMATCH` in the feed | Godot build older than the sim | The dashboard is stale. Rebuild it; do not demo past this |
| Reasoning feed empty | model not answering | Harmless — the scripted rung is running. Do not restart mid-demo |
| Visible stutter | paging | Something else is running. Nothing to do mid-demo; note it and move on |
| Anything unrecoverable | — | **Switch to the backup video.** It is a complete clean run. Do not debug in front of judges |

---

## 6. Rehearsal checklist

Current preparation:

- [ ] Run 1 — full 7 minutes, no operator input beyond the four cues above
- [ ] Run 2 — same, timed against this beat sheet
- [ ] Run 3 — same, wifi off, model server *not* started (prove the fallback in public)
- [ ] Backup video re-recorded against the shipping build
- [ ] `make check` green on the exact commit being demoed
