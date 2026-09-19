# Problems faced, and what fixed them

An index of every problem this project has hit and the change that resolved it, grouped by
subsystem rather than by date.

**This file duplicates nothing.** [MEASUREMENTS.md](MEASUREMENTS.md) is the chronological
record and owns the numbers — M-*n* references below point into it.
[training_notes/](../training_notes/SUMMARY.md) owns the training runs.
[PLAN.md](PLAN.md) owns scope and [TECHNICAL.md](TECHNICAL.md) owns design. This file
answers a different question: *what has already gone wrong here, and what was the actual
cause* — so the next person hitting the same symptom does not re-derive it.

Entries are marked ✅ fixed, ⚠️ open, or ⛔ decided-against.

---

## 1. The patterns worth internalising

Seven failure shapes account for most of what follows. Each has bitten at least three
times in different subsystems.

### 1.1 The system that looks like it works

The most expensive category by a wide margin. Every one of these passed its tests, ran
without errors, and produced plausible output.

| what looked fine | what was actually true | found by |
|---|---|---|
| Casualties being discovered | `seen = dist <= radius` against ground truth — no camera, no detector | design review (D3) |
| Detector confirming contacts | 4% precision; the swarm spent the mission investigating rocks | counting the funnel (M-8) |
| Zero false positives at every range | Luminance-only noise left `R − B` — the exact detector signal — unperturbed | disbelief at a perfect number (M-7) |
| A relay lane at full strength | 14 of 192 relays ever held a post; 0 posts generated from 48 targets | watching a live run in Godot (M-33) |
| An antenna lane at maximum fitness | Fitness was "robots in comms" = swarm size, regardless of relay behaviour | every elite scoring *exactly* 16.000 (M-31) |
| A hivemind answering in 3.5 s | `step()` joined the worker thread; would have frozen `DemoSim` >50% of the demo | putting a real model behind it (M-28) |
| 221 directives issued | ~200 were renewals of the same directive — a 20x overstatement | separating renewal from decision (M-25) |
| The ground-truth-leak guard passing | It could not read the source it guards under a GBK locale | running the suite on a second machine (M-38) |
| A training curve rising monotonically to 1.10x | The policy had learned to stop partitioning — the degenerate imitation of "off" | reading `command.py`, not the score (run 1) |

**The tell is always the same: a number that is too clean, too flat, or too good.** 4%
precision, exactly 16.000, 52.5% of ticks at zero speed, "explored" pinned to 35.5% across
three unrelated changes.

### 1.2 Proxies that are not causal

| proxy | what it actually measured | replaced with |
|---|---|---|
| "mean robots in comms" as relay fitness | swarm size | `comms_via_relay` — the set that would drop out if relays vanished (M-31) |
| comms *reach* as a stand-in for coverage | one robot at the end of one chain | fraction of passable map inside the component — ⚠️ still to do (M-40) |
| explored % as mission progress | coverage, which Tier 3 correctly trades away for rescues | rescues, with coverage reported beside it (M-26) |
| camera pixels as search redundancy | nothing meaningful — the "13,008x" figure | unique cells per scout-second (M-35) |
| training score as evidence training worked | that the objective was being maximised, not that it was the right objective | held-out maps, evaluated *during* the run (run 1) |

### 1.3 Thresholds defeated by the last digit of nothing happening

Both halves of the same lesson, six weeks apart in project time, one day apart in reality:

- **Run 2:** the reward-hacking tripwire tested `held_out_slope <= 0`. A flat noisy series
  has slope +0.0028. Technically positive. It did not fire.
- **Run 3:** the fixed version tested training slope `> 0`. With `best_x` unchanged for 47
  generations every sample was byte-identical, and `np.polyfit` on constant data returns
  ~1e-16. It fired ten times announcing "training rising +0.000/gen".

**A slope test needs a magnitude floor on both sides.** `MIN_TRAIN_SLOPE = 0.01`, and the
held-out check now skips when `best_x` has not moved. A monitor is not trustworthy until
it has caught something — this one had to fail twice first.

### 1.4 Correct-looking fixes that measure as regressions

| the fix | why it was obviously right | what it measured |
|---|---|---|
| Deduplicate search targets (629 scouts → 44 targets) | 33 robots walking to one spot is textbook waste | rescues 13 → 5 (M-36) |
| 3x3 sweep waypoints per sector | 449 scouts, 436 distinct destinations | rescues 13 → 5 (M-14, M-36) |
| Stricter nav-grid passability threshold | a 51%-passable coarse cell can promise a route with no fine path | strictly worse at every value tried (M-23) |
| Wider camera FOV | more ground seen per robot | explored 84% → 98%, rescues 3 → 1 (M-8) |
| Relay post budget + gap-break guards | a chain with a hole carries nothing | no measurable effect on their own (M-40) |

**Exploration and comms pull against each other, and comms wins.** Spreading the swarm
past the comms envelope means `mark_seen` never records the extra ground. That single
mechanism explains the first three rows.

### 1.5 One line, one cascade

- **Relay anchoring.** Idle relays counted as anchors → the 128 sitting inside base radius
  blocked every chain's first link → no relay ever deployed → comms never grew → scouts
  revealed into private buffers → the explored region visibly stopped at a fixed radius.
  Four symptoms, one line (M-33).
- **Orphan release.** Releasing an out-of-contact relay's task freed it to bid on a
  frontier → it abandoned its post → the swarm behind it lost contact → more orphans. One
  run ended with 0 of 16 robots in comms (M-9).
- **A 64 KB socket buffer.** Godot's `WebSocketPeer` silently drops a connection on an
  oversized frame. The dashboard sat on "waiting for simulator" while the bridge reported
  a healthy client (M-12).

### 1.6 Measure before tuning — the hypotheses that were wrong

Recorded because each was confidently held and cost nothing to check:

| hypothesis | measurement | verdict |
|---|---|---|
| `max_explore_targets=48` starves 352 scouts | 332 of 352 held a task; 2 idle | wrong (M-34) |
| Coverage is robot-count limited | 768 → 1536 robots moved it 23.6% → 32.3% | badly sublinear (M-14) |
| Coverage is frontier-cap limited | cap 48 → 240 moved it 23.2% → 23.7% | wrong (M-14) |
| The remaining Tier 1 override rate is a coarse/fine mismatch | stricter thresholds left the rate untouched | wrong, still ⚠️ open (M-23) |
| RTF had regressed to 0.68x | 1.62x on an idle machine | contention (M-24) |

### 1.7 Scaling invalidates constants — always in threes

Every scale change broke at least three things that were correct at the old scale:
cluster count scaled by area (radius grew too, 33x coverage), the flat 24-task
announcement cap (throttled 512 robots to 24 assignments/second), the even 192/192/192/192
lane split, the ~22 m staging disc, and mission duration held at 420 s across a 2.31x map.
**Robot count, map size, casualty count, hazard growth and mission length are one
coupled parameter set.**

---

## 2. Simulation, terrain and scenario

| # | problem | cause | fix | ref |
|---|---|---|---|---|
| ✅ | Obstacle coverage rose 33x on a 10.8x larger map | cluster *radius* scaled with count, not just density | re-swept density; 380 clusters at 0.658 passable | M-2 |
| ✅ | 1 casualty in 6 unreachable — carrier dispatched, never arrives | placement checked the *fine* grid; robots navigate the 4x-downsampled one, and bodies have width | `_navigable()` requires clearance + reachability on the same grid `NavFields` uses; found 30 → 36 | M-18 |
| ✅ | Casualties hidden where nobody reaches | `distance_weight_exp` 1.5 put the tail on the far rim of a map at 42% coverage | 0.4; coverage identical, found 14 → 30 | M-17 |
| ✅ | Coverage collapsed on the 2.31x map (30.7%) | frontier exploration is perimeter-limited — a boundary absorbs a bounded number of robots | sector sweep opens independent fronts, density-gated at ≥8 robots/sector | M-14 |
| ✅ | One seed had *zero* reachable cells | terrain generated on top of the base; a river sealed the staging area | fixed points carved before pruning | M-21 |
| ✅ | Seed 44: 753 passable cells of 153,600 | obstacle clusters sealing the base in — pre-existing, surfaced by terrain | roads carved before connectivity pruning; 30/30 seeds build | M-22 |
| ✅ | Roads with 1.17 max slope (3x a wheel's limit) | 12 collection points = 66 corridors in a complete graph, each regrading the last | spanning tree (11 edges); roads regraded to constant slope and drained | M-22 |
| ✅ | Tracked chassis strictly dominated by wheeled | no barrier only tracks could clear | marsh tier (0.20–0.38 m) stops wheels only; rivers stop tracks; reach rises as speed falls | M-22 |
| ⚠️ | Terrain cost the mission 18 → 4 rescues | routes longer, a third of the swarm restricted to 58% of the map | partially recovered by M-23's four fixes (→ 11); scenario rebalance still open | M-22 |

**Four wrong turns on terrain**, each of which looked like the fix: blurring to reduce
gradient (a 3x3 mean of a linear ramp is the same ramp), eroding "passes" toward base (a
global smoother in disguise — took relief 70 m → 17 m), demanding a tracked-connected map
(ground only legged units reach is the locomotion axis *working*), and blaming the
mountains when the base noise octaves summed to ~1.27 gradient before a single peak was
placed (M-21).

### Landform, rivers and the road pass (M-62)

The visible complaint was that the map rendered as a flat plain with dark scratches on
it. Fixing that put real relief on the ground, and the relief then exposed five defects
in the road grading that had been latent for as long as the map was flat — **four of
sixteen held-out seeds had wheeled units on 0–1% of the map before any of this**.

| # | problem | cause | fix | ref |
|---|---|---|---|---|
| ✅ | 31 m of range renders as a pancake | zone aprons level-filled **91% of the map** (60 m discs, twelve fixed points, 360 × 240 m), and all the noise amplitude sat at 30 m wavelength | soften toward a σ≈20 m landform over 26 m, level only a small pad inside it; amplitude moved into 150/70 m octaves; smoothstep interpolation. Hill-scale relief 5.7 → 9.4 m | M-62 |
| ✅ | Rivers looked like scribbled dry cracks | a random-walk centreline, and `water` was a flat stamp on touched cells that nothing ever drew | straight edge-to-edge courses on alternating axes, sited clear of the collection points; parabolic bed filled to a downhill water surface; water drawn in `render3d.py` and on the dashboard | M-62 |
| ✅ | A 0.38 m marsh became a **14 m lake** | the pool levelled to the basin's *median*, which on sloping ground sits metres above its downhill side | fill from the basin floor, and carry a per-feature depth `cap` so no later step can deepen water past the feature that made it | M-62 |
| ✅ | One seed's marshes were all empty | the final `z -= z.min()` renormalised ground without the water surfaces; ditches cut ~2 m below zero, so the shift drained everything | shift ground and every surface together | M-62 |
| ✅ | Roads built **11 m causeways to bridge 0.38 m of marsh** | the deck's reference height read raw `level`, which is not the water surface where features overlap; the approach ramps graded to reach that deck severed the road out of base | one definition of depth (`min(level − ground, cap)`), used by the road pass and the final field alike | M-62 |
| ✅ | 214 one-cell cliffs, up to 5.2 gradient, two on the only road out of base | eleven corridors written one `np.where` at a time; on a map with relief the profiles crossing at a junction disagree by metres, and the last write lands as a step at the previous corridor's rim | accumulate all corridors and apply once, with the carriageway outvoting a neighbour's batter | M-62 |
| ✅ | A trench ringing every corridor, ~9% of its own elevation deep | `acc_r / max(acc_w, 1e-6)` returns a *fraction* of the right height where the weights are tiny | natural ground as the prior in the weighted mean, so the expression tends to `z` as the weights tend to zero | M-62 |
| ✅ | Roads graded flat with 3.4-gradient sides | a fixed 5 m shoulder feather: fine on flat ground, a cliff where the corridor cuts a hillside, and steep enough that the central difference contaminated the carriageway itself | batter as wide as the cut is deep (0.45, capped at 24 m) | M-62 |
| ✅ | The deck left the pad **3.3 m in the air** | the slope limiter was free to move its own endpoints, and iterating a two-sided clamp with them pinned does not converge — six passes left a 3.60 m step against a 0.283 m limit | closed-form Lipschitz envelope (min-plus distance transform) after clipping into the cones reachable from both endpoints | M-62 |
| ✅ | Steep faces rendered as horizontal stripes with the bed showing through | `render3d.py` keyed its side splats on height above the map floor, not on the drop to the neighbouring cell — identical while the map was flat | skirt depth from the local drop, plus a full skirt on the map rim so the slab edge stays solid | M-62 |

**What it cost the mission.** Rescues are a wash (63.7 → 65.3 mean over seeds 42–44) and
exploration is unchanged, but **robots lost rose 44 → 57 per run** — harder ground strands
more machines, and that is the number to watch if the scenario gets rebalanced. Seed 42 on
its own says the terrain costs 7 rescues; seeds 43 and 44 say otherwise. It is a
favourable seed for the old map, and one seed is not a measurement here: the spread within
a single variant is 21 rescues against a 3.6-rescue gap between variants.

**Water was never on the wire.** It gated traversal for weeks — it is the boundary between
what a tracked unit crosses and what only a legged one does — while the dashboard drew
rivers as dry trenches, so a wheeled robot stopping at a bank looked like a bug rather
than the locomotion axis working. A field that decides where robots may go has to be
visible.

---

## 3. Tier 1 — navigation and the safety floor

| # | problem | cause | fix | ref |
|---|---|---|---|---|
| ✅ | Swarm travelled 62 m in 420 s of a possible ~630 | four independent causes, below | 62 → 171 m; losses 147 → 61 | M-23 |
| ✅ | …Tier 1 blind to terrain | repulsion and the swept-circle override both tested `occ == WALL` | both made chassis-aware | M-23 |
| ✅ | …768 robots gridlocked at the start line | ~22 m staging disc, 2 m² per robot against a 2.5 m separation radius | radius derived from robot count and separation | M-23 |
| ✅ | …52.5% of robot-ticks had a goal and zero speed | spawn ignored chassis; a wheeled robot starting in marsh has its own cell impassable, forever | chassis-aware spawn | M-23 |
| ✅ | …any shove into bad terrain was permanent | `_collides` blocked every move because the robot's own centre sample failed | stranded robots move at 30% speed, like a machine struggling out of mud | M-23 |
| ✅ | Scouts stalling nose-first at walls, 40.8% override rate | repulsion scales by *how surrounded* a robot is; one wall ahead gives ~0.2 against a goal term of 1.0 | forward arc (±45°) treated as urgent, using the direction the probe ring already computed. **Invariant #2 untouched** — the override fires less because steering avoids the situation | M-34 |
| ✅ | 50% of episode runtime in `_goal_directions` | goals reissued at 1 Hz but re-resolved at 20 Hz; `descend` called once per (goal, chassis) group — full numpy dispatch for ~1 robot | memoised goal coords, precomputed int8 descent grid, `ndarray.clip`. **−40% wall, RTF 0.63 → 1.05, hash bit-identical** | M-38 |
| ✅ | Distance fields at 22.4 ms would cost 4.5 s of compute per sim-second | one BFS field per open task per cycle at 200+ tasks | all three of: 4x coarse nav grid, announcement cap, amortisation across the cycle's 20 ticks | M-4 |
| ✅ | 39 casualties still being carried at the buzzer, 34 with time to deliver; a carrier held one 12 m from its zone for ~320 s | inside a passable coarse cell the single descent direction points into fine-grid rock; repulsion pushes back, the queue behind pushes in, the override zeroes it. Same family as phantom edges, one level down, concentrated at collection points | `control/zone_routing.py`: one fine field per chassis to every zone, no corner-cutting, over ground eroded by one cell. **The un-eroded version lost 13 rescues on seed 44** by threading one-cell gaps. +7.5 rescued on 42–45 (M1), +8.1 over 42–49 (Kaggle) with Tier 3 silent. **With Tier 3 live the machines disagree in sign (+9.5 Kaggle, −1.25 M1), so it ships off by default** | **M-76, M-76a, M-76e, M-76f** |
| ✅ | ~33% Tier 1 override rate | **phantom coarse edges**: two blocks both pass the 50% majority test while their passable fine cells lie either side of rock, so the 8-connected wavefront routes through walls. 4.4–6.1% of all coarse edges, 75% diagonal | adjacency derived from the fine grid (`grid.coarse_edge_masks`); **rescues +35%, delivery 59.5% → 73.0%, densest clump 32 → 14** | M-23, **M-60** |

---

## 4. Perception

| # | problem | cause | fix | ref |
|---|---|---|---|---|
| ✅ | "The robots have computer vision" was false | `seen = dist <= sensor_radius` against ground-truth coordinates | appearance raster → egocentric occluded crop → detector → report tracker; oracle deleted | D3 / M-8 |
| ✅ | Fully buried casualties invisible to every camera | rendered by blending toward rubble colour — a buried victim rendered as pure rubble | *size* carries the occlusion, not colour | M-7 |
| ✅ | Zero false positives at every range | one noise value broadcast across R, G, B left `R − B` untouched | per-channel noise | M-7 |
| ✅ | Warm rubble could never fool the detector | rendered at `R − B` = 106, below the detector's band | raised to 128, inside the band — discrimination now needs *extent*, which is what gives the CNN something to beat | M-7 |
| ✅ | Confirmed-report precision 4%; swarm investigated rocks all mission | three compounding rules | 4% → 44%, reports 3765 → 232 | M-8 |
| ✅ | …a parked robot confirmed rubble in 1.4 s | repeated looks from one robot counted as corroboration | parallax: an observation counts only if the observer moved >2 m | M-8 |
| ✅ | …crowds of worthless distant detections promoted each other | precision at 6.5 m is 0.05 | range-trust ramp from the measured curve + one close look required | M-8 |
| ✅ | The swarm rediscovered the same rock forever | dismissed reports were pruned and immediately re-created | dismissed reports kept for the whole mission as "we looked there" | M-8 |
| ✅ | A detector test began passing for the wrong reason | M-18 moved casualties into clearings, so no warm rubble was in frame | test now asserts *recall* falls with range; a separate test points robots at rubble | M-18 |

---

## 5. Tier 2 — auction, tasks, self-healing

| # | problem | cause | fix | ref |
|---|---|---|---|---|
| ✅ | 512 robots, 24 assignments per second, the rest idle | `MAX_ANNOUNCED = 24`, sized against 22.4 ms *fine*-grid fields; bidding uses the cached coarse grid | cap scales with free-robot count (1.5x); 10/80 → 16/80 rescued | M-9 |
| ✅ | 609 connect/disconnect events per mission, each orphaning a task | robots on the exact range boundary flapping | hysteresis — joining needs 0.92x the radius, staying needs 1.0x; flaps 609 → 40 | M-9 |
| ✅ | One run ended with 0 of 16 robots in comms | orphaning *released* the robot, which then bid on a frontier and abandoned its relay post | orphaned work is re-offered **without** being taken away; antennas barred by policy from search work in both allocators | M-9 |
| ✅ | Casualties finished the mission in state `carried` | opportunistic pickup is right; the *task* not following it was the bug | a robot holding a casualty has its assignment rewritten to `extract` | M-9 |
| ✅ | Confirmed casualties sat unclaimed; self-healing broke outright | the auction awards only to *free* robots, and every carrier was exploring | rescue-tier work may preempt strictly lower-priority work (`PREEMPT_RANK`) | M-15 |
| ✅ | …and then preempted **only** when nobody free could bid | a free carrier 258 m away beat a busy one 4 m away, because the busy one was never evaluated | preemption competes in the normal round, with a `PREEMPT_RATIO` margin so churn is not free | M-42 |
| ✅ | One casualty awarded to three carriers at once, ~255 m each | several reports resolve onto one casualty and each became its own task; `claimed_v` dedups against live assignments, which none were yet | dedup within the generation pass | M-42 |
| ✅ | Worst-case reassignment was exactly 3.0 s — the number being claimed | orphan detection ran on the auction cycle | moved to heartbeat rate; an orphan forces an immediate auction | M-15 |
| ⛔ | 629 scouts walking to 44 distinct targets | search targets deliberately not deduplicated | **reverted** — the fix measured 13 → 5 rescues. Reasoning written into `tasks.py` where the next reader will look | M-36 |
| ⚠️ | Search runs at ~4% efficiency; no component owns territory | Tier 2 allocates *tasks*, not space; Tier 3's ±0.5 rank nudge cannot partition a swarm | open — the commander was the attempt, and it was gated out (§8) | M-35 |

---

## 6. Comms and the relay lane

The single longest-running thread in the project: **comms coverage gates fog reveal, which
gates discovery, which gates rescues.** `World.mark_seen` writes to the shared map only for
in-contact robots, so every search improvement that spread the swarm out died here.

| # | problem | cause | fix | ref |
|---|---|---|---|---|
| ✅ | 0 relay posts generated from 48 frontier targets; 14 of 192 relays deployed; median relay displacement 9 m over a whole mission | every *living relay's position* was an anchor, and candidates are rejected within 28.5 m of one. 128 relays inside the 40 m base radius blanketed the ground a chain's first link needs — self-blocking and stable | anchors mean **committed topology** only: base, plus posts a relay has actually been assigned | M-33 |
| ✅ | An even 192/192/192/192 lane split | the network saturates near 44 posts however many antennas exist | swept against comms coverage: **352 scouts / 176 diggers / 144 carriers / 96 relays** | M-33 |
| ✅ | A scout that leaves the envelope and never returns reports nothing | buffers flushed only on *reconnect*; on a 480x320 m map most never return | store-and-forward — an out-of-contact robot hands its buffer to any in-contact robot it passes. Deliberately does **not** rejoin it to the live component | M-37 |
| ✅ | Comms reach plateaued at ~240 m of a 577 m diagonal on every seed, with half the relay lane never on post | **the chain was aimed at the frontier, and the frontier is the comms boundary** — explored ground ends where contact ends. Every link waited on exploration that was waiting on that link | chains are aimed a bounded distance *past* the frontier. Reach +20% on 4/4 seeds, rescues +10.8% | M-39, M-40 |
| ⚠️ | The relay lane commits 96/96 posts by t=30 s and never re-aims | a relay holding a post never re-bids as the frontier moves | open — re-taskable posts are the obvious next experiment | M-40 |
| ⚠️ | Reach rose 20% but exploration only 2.2 points | reach is a *max* distance; coverage follows area | open — measure comms *area*, not the farthest robot | M-40 |

---

## 7. Tier 3 — the hivemind

| # | problem | cause | fix | ref |
|---|---|---|---|---|
| ✅ | Would have frozen the demo for >half its length | `step()` joined the worker thread with the rung timeout. **Every test passed** because they all used an instant provider | genuinely asynchronous — one tick starts a request, a later tick collects it | M-28 |
| ✅ | First model call took 10.61 s against a 4 s timeout | unconstrained generation: a 700-character essay plus four per-directive justifications. Every cycle would have fallen through to the scripted rung *while appearing to work* | `maxLength` in the schema compiles to a GBNF constraint; per-directive `reason` dropped (~half the tokens, used nowhere). 10.6 → 3.5 s mean, 0 timeouts | M-27 |
| ✅ | The base model was mechanically inert — four `priority: normal` directives, well-formed, changing nothing | it did not know `normal` is already every sector's default, and invented hazard figures | prompt states what each field *does to the swarm*, plus a worked example; filter rule F10 rejects asking for the state a sector is already in (renewals exempt) | M-27 |
| ✅ | "221 directives issued" — a 20x overstatement | ~200 were renewals of one directive re-sent to hold a position | renewals counted separately from decisions | M-25 |
| ✅ | The feasibility filter fired 21 times a run | the *scripted baseline* was proposing to abandon the sector containing the base | taught the provider the two rules; rejections 29 → 0 with the filter unweakened. A rejection on the dashboard now means something | M-25 |
| ✅ | Tier 3 aimed 61% of its search pushes beyond the comms horizon | "least explored" is very nearly "farthest from base", and `mark_seen` records nothing from an out-of-contact robot | push candidates filtered to the current comms reach plus one relay hop; misaimed pushes 61% → 27%, delivery +3.1 points | M-43 |
| ⚠️ | Does Tier 3 help at all? | — | **unresolved.** M-26 (3 seeds) was "promising, not demonstrated"; M-40 (4 seeds) has it level-to-behind, and behind on discovery on 4 of 4 seeds. The gate must settle it | M-26, M-40 |
| ⚠️ | The API rung has never run | no API key on this machine | `build_ladder` drops it silently; **do not describe it as working** | CLAUDE.md |

---

## 8. Training

### MAP-Elites (Tier-2 genome) — shipped

| # | problem | cause | fix | ref |
|---|---|---|---|---|
| ✅ | Budget planned at 300k evaluations in 20 minutes | never measured; assumed a bare policy rollout, not a full mission tick | re-derived from measurement: **~1,400/hour**, so ~690 evals in a 30-minute run. Off by ~200x | M-30 |
| ✅ | Benchmark presented as a hang, wrote 76 MB of tracebacks | `spawn` re-executes `__main__`; from a heredoc there is none, so every worker re-ran the caller and spawned its own | `pool._guard()` refuses, and a test asserts the refusal | M-30 |
| ✅ | 56 BLAS threads on 8 cores | seven workers each opening a thread per core | workers pin to one thread | M-30 |
| ✅ | Every antenna elite scored exactly 16.000 | fitness was "mean robots in comms" = swarm size | `comms_via_relay`, which the simulator already computed — the counterfactual episode was never needed | M-31 |
| ✅ | `obj_max` and `qd_score` reported one lane with noise on top | lane fitness scales differed ~20x | `LANE_REFERENCE`; monotone, so within-lane selection is untouched | M-31 |
| ✅ | Nearly every early digger genome tied at zero | fitness was `int(cleared) + 2·int(freed)` — a step function | continuous in debris removed | M-31 |
| ✅ | 15 → 16 archive cells over three iterations | all five emitters started from the all-0.5 midpoint, which decodes to `gripper` | one emitter per lane at its own midpoint; scout/scoop/gripper/antenna proposals 9/59/71/11 → 19/11/16/14 | M-32 |
| ✅ | 9 generations per hour — a warm-up, not a run | batch 30 is 2.5x the CMA-ES default for a 15-D genome | batch 15 → ~23/hour for identical spend | M-32 |
| ✅ | "45% coverage" was partly a statement about the axis | speed axis 0.0–1.8 m/s was guessed; 12 of 40 cells were unreachable by construction | recalibrated to 0.20–1.45 from 52 observed elites; the test now guards **both** directions | M-32 |
| ✅ | Coverage stuck at ~40% | cold start — emitters circling the origin | seed the archive with 150 uniform random genomes first: **29 of 40 cells filled before CMA-ES took a step** | run 0 |

**Result: 31/40 cells, roster shipped, worth ~+90% rescues** (10 → 19 on seed 42 against
hand-set archetypes, with 28% fewer losses). Also: *an archive is not a roster* —
`select.py` picks the extremes of measured speed per lane, not the top-N by fitness, which
is why one shipped elite has fitness 0.00.

### The unit policy (Tier 2) — in progress on branch `rl/unit-policy`

| # | problem | cause | fix | ref |
|---|---|---|---|---|
| ⛔ | "Send robots to settle unconfirmed contacts" looked like the obvious unit action (M-71's seen-and-discarded misses) | more discovery does not convert (M-50, M-64) | **dropped before training**: crude rule −7.25 rescued; ground-truth oracle +0.5 | M-76 |
| ✅ | A fresh policy would send ~80% of searching robots somewhere random on iteration 1 | near-uniform initial softmax | behaviour cloning of the heuristic first, until the *sampled* policy agrees ≥ 98% — argmax agreement reached 0.98 while sampled agreement was 0.35 | run 4 |
| ✅ | A staged carrier was immediately re-tasked to explore | idle robots are free to the auction | `policy_goal` reserves it from search work only; rescue work still sees it | TECHNICAL §5.2 |
| ⚠️ | Heuristic staging is flat on top of routing: +0.9 rescued, down on four seeds, up on four | — | the case for learning *when* to stage; bar is 1.05× the better of routing and heuristic | M-76a |

### The commander (Tier 3a) — ⛔ trained three times, gated out

| arm | score | rescued | explored |
|---|---:|---:|---:|
| **none** | **70.81** | 52 / 768 | 28.3% |
| heuristic | 65.57 | 47 / 768 | 27.3% |
| trained | 69.42 | 52 / 768 | 28.5% |

**0.98x against a 1.05x bar. No commander ships, and `command=False` was already the
default.** Training was not wasted: the hand-set commander was *actively harmful* (0.93x)
and training made it neutral. The three-arm gate is why that can be said honestly —
"trained beats the heuristic by 5.9%" is true, sounds good, and means nothing.

Four faults found by reading `nodes/command.py` rather than the training curve:

1. **`w_contacts` was a dead parameter** — `contacts` was `np.zeros(...)`, never filled. One
   of seven CMA-ES dimensions optimised pure noise for 121 generations.
2. **`clip(..., 0, None)` let a large `w_distance` destroy the territory split.** Nearly
   every sector clipped to zero weight; the policy learned to *stop partitioning* — the
   degenerate way to imitate the one thing known to beat it.
3. **A flat reward per rescue is not a neutral reward.** Distant casualties cost far more
   travel, dig and carry time, so 10-per-head quietly instructs the policy to harvest the
   cheap ones.
4. **Nothing was watching.** Held-out maps existed and were never evaluated during the run.

Then run 2 fixed all of it and **overfit three maps** — training +3.66, held-out +0.00.
Run 3 used six maps, settled `squadron_size` comfortably interior at 69.2, and plateaued:
47 generations failed to beat a result found in twelve minutes.

**Reopening requires changing a mechanism** — fenced relay tasks, bearing-cut pie-slice
wedges (a far slice covers enormous area with the same robots), or the zero-clipping value
function — **not searching the same seven numbers harder.** Full record in
[training_notes/](../training_notes/SUMMARY.md).

---

## 9. Dashboard, bridge and assets

| # | problem | cause | fix | ref |
|---|---|---|---|---|
| ✅ | Godot sat on "waiting for simulator" with no error on either side | `WebSocketPeer` defaults to a 64 KB inbound buffer and silently drops the connection on a larger frame. The `hello` carried terrain + heightfield + occupancy: ~200 KB | chunked into 32,000-char `blob` messages (largest frame now 31.3 KB), buffer raised to 4 MB anyway, status line shows `receiving map 5/9`, and a test fails the build above 60 KB | M-12 |
| ✅ | ~1 MB/s of JSON at 512 robots | robots serialised as objects | flat array of numbers — ~150 KB/s, parsed in one Godot loop | M-10 |
| ✅ | The dashboard cannot be tested (Godot will not run here) | — | `test_bridge_protocol.py` parses `main.gd`, extracts every field it reads, and asserts the bridge sends it — including positional widths GDScript indexes numerically | M-10 |
| ✅ | 3D work could not be inspected before shipping to GDScript | neither Godot nor a browser runs in this environment | `viz/render3d.py` renders the same scene to PNG offline; caught a confetti floor and a world too dark to read | M-11 |
| ✅ | The world was unreadably dark | the appearance raster is tuned for the *detector* — dark, low-contrast, warm rubble confusable with a casualty | two palettes on purpose. Do not unify them | M-11 |
| ✅ | Distance fog rendered the whole map as a grey rectangle from the operator camera | uniform density cannot serve a 1.1 m eye and a 340 m eye | height-attenuated exponential fog — which is also what actually happens | M-13 |
| ✅ | Ruins looked like dressing perched on grey lumps | the heightfield made walls 3.2–5.6 m tall, debris boxes were 3 m pillars, and there was no directional light for lit meshes to catch | terrain reduced to rubble mounds; the meshes carry the height; warm low sun at 1.25, ambient 0.42 | M-20 |
| ✅ | 28.5M triangles at 1 prop per 7 cells | Godot's MultiMesh does not cull per instance | measured budget: 1 per 40 cells → 1,592 props, 5.4M triangles | M-20 |
| ✅ | 41 MB of the nature pack was two normal maps with **no effect whatsoever** | the world shader is `render_mode unshaded` | textures stripped and the glTF rewritten with every index remapped; 292 MB → 3.1 MB | M-19 |
| ✅ | Meshes outside `godot/` cannot be loaded at all | no `res://` path exists for them | meshes live in `godot/assets/`; three tests guard paths, roots and sidecars | M-19 |
| ✅ | Every contact ring on the map was the same amber, and there were ~500 of them | two faults stacked: `bridge._contacts` sent one ring per **report** rather than per casualty (817 rings over 26 bodies at t=60, 103 coincident on one), and `main.gd` coloured them resolved-vs-not. The pile hid the palette and the palette hid the pile | dedup on `r.victim` (first report wins, list order, so determinism holds) → ~30 rings; `CONTACT_CODE` on the wire — unverified / buried / surface / dug / carried — indexed into `CONTACT_COLORS`, with a HUD legend built from the palette and a test asserting the two orders match | M-64 |
| ⚠️ | RuinsGR licence unresolved | no LICENSE, README, or glTF `copyright` field in any of 56 models | recorded in `assets/models/ATTRIBUTION.md`. The dashboard renders procedural geometry, so removal costs a look, not a feature | M-16 |

---

## 10. Tooling, benchmarking and determinism

| # | problem | cause | fix | ref |
|---|---|---|---|---|
| ✅ | RTF overstated 2x (10.3x vs 4.90x) | the sweep window was 25 s — the cheapest phase of a mission, before hazard, with a small frontier and few tasks | any window shorter than `hazard.ignite_t` measures the wrong thing; default now 150 s | M-5 |
| ✅ | An optimisation push nearly launched against 0.68x RTF | the run shared eight cores with a parameter sweep; idle it was 1.62x | benchmarks on this machine are only meaningful when nothing else is on it | M-24 |
| ✅ | The invariant-#4 guard died with `UnicodeDecodeError` on a GBK locale | `Path.read_text()` defaults to the locale encoding | `encoding="utf-8"` at every call site | M-38 |
| ✅ | `--workers` defaulted to 15 on an 8-core box | `cpu_count()` counts hyperthreads; SMT bought +9%, not +100% | not harmful, but the number means much less than it looks | M-38 |
| ⚠️ | Seed 42 hashes differently on the M1 and the Ryzen, with no sim code changed | platform float; `test_determinism.py` compares runs *within* a process, which is all invariant #6 claims | **a before/after pair must be taken on one machine.** A ±1-rescue platform delta is the same size as the effects being chased | M-40 |
| ⚠️ | `mapelites/run.py` sizing comments say 1,350 evaluations; the default run is 690 | emitters went 5 → 4 and batch 30 → 15; the prose did not follow | open | M-38 |
| ⚠️ | The suite is 11m14s, not the "seconds" CLAUDE.md claims | unsplit between platform and the suite growing at D9/D10 | open | M-38 |
| ⚠️ | `FaultInjector._fire` emits `robot_destroyed`, then `world._kill` emits it again | one death, two events — the dashboard timeline shows a robot dying twice | `scripts/diagnose.py` de-duplicates by robot id; **the source is not fixed** | M-39 |

---

## 11. The rescue rate — the one problem still open

The longest-running open item, and the only one that can still sink the demo. It has been
attacked from five directions, and each attempt found a *different* constraint underneath.

| stage | what was believed | what was measured | ref |
|---|---|---|---|
| D9 | Scouts are starved of frontier targets | 332 of 352 held a task | M-34 |
| D9 | Tier 1 is stopping them | true — 40.8% override, 0.53 m/s; fixed to 12.2%, and rescues moved 11 → 13 | M-34 |
| D9 | Comms gates exploration | true — unlink everyone and exploration +51%, discovery +84%… and **one fewer rescue** | M-34 |
| D9 | So rescue is delivery-bound | **only casualties found in the first ~150 s can be delivered**, and that early count is ~13 whatever the map looks like | M-35 |
| D10 | Out-of-contact discoveries are lost | true — store-and-forward, +5 found, +2 rescued | M-37 |
| D11 | The relay chain is the constraint | true — chain coupled to the frontier it is trying to move; reach pinned at ~240 m | M-39 |
| D12 | Breaking that coupling lifts discovery | **partly** — reach +20% on 4/4 seeds, rescues +10.8%, but the *discovery ceiling did not move* | M-40 |

**Where it stands.** ~30 of 120 casualties are ever found, so rescuing everything
discovered is ~25%. No component-level fix changes that — it is a statement about map
size, mission duration and swarm speed together, and the mission clock is pinned at 420 s
by the demo being seven minutes long.

**✅ Resolved as a scope decision.** The flat ≥ 75% rescued bar was replaced by two
numbers — **delivery** (rescued ÷ found, bar 55%, measured 60.5%) and **discovery** (found
of 120, floor 25) — both reported absolutely. The old target was set against a
320 × 208 m map with 8 casualties and never moved when the scenario became 480 × 320 m
with 120: the fourth instance of §1.7 in this file. Full reasoning and measurement in
[PLAN.md §6.1](PLAN.md); mirrored in [TECHNICAL.md §8](TECHNICAL.md). **Do not quote 75%
anywhere.**

---

## 12. Open, carried forward

| item | since | where |
|---|---|---|
| Discovery ceiling ~25% — map size vs mission duration vs swarm speed. **Accepted as scenario design, not a defect** (PLAN §6.1) | D9 | M-34, M-35, M-40 |
| ~~Hazard is 82% of all losses~~ **✅ fixed** — the fire burns out (`peak_t` / `decay_rate`); losses halve, rescues unchanged | D11 | M-39, M-41 |
| Relay lane commits within 30 s and never re-aims | D12 | M-40 |
| Comms *area* is the metric, not comms reach | D12 | M-40 |
| ~33% Tier 1 override rate, cause unknown | D5i | M-23 |
| Search runs at ~4% efficiency; nothing owns territory | D9 | M-35 |
| **Tier 3's effect is not separable from noise** — 0.91x on demo seeds 42–45, 1.02x on ten held-out seeds, under the 1.05x bar either way. Gate says *no Tier 3*. The *model* rung has still never run on the demo scenario | D7 | M-26, M-42, M-43, **M-45** |
| Terrain's cost to coverage never fully recovered | D5h | M-22 |
| Relay tasks territorially fenced (`auction.py:157`) — only bites if a commander ships | run 1 | training_notes |
| Wedges are bearing-cut pie slices; far slices are enormous | run 1 | training_notes |
| MAP-Elites σ₀ 0.15 against [0,1] bounds → >100 resamples | run 0 | training_notes |
| `FaultInjector` double-emits `robot_destroyed` | D11 | M-39 |

| Stale MAP-Elites sizing comments; 11-minute test suite | D10 | M-38 |
| RuinsGR licence unresolved | D5d | M-16 |
| Wheeled and tracked still poorly separated by terrain gradients | D5g | M-21 |
