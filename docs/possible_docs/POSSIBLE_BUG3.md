# POSSIBLE_BUG3 — the swarm stops moving, and exploration decays to nothing

**Symptom reported:** "near the end, nearly all of the robots don't move any more; as it
progresses the exploration rate just goes slower and slower."

**Status:** diagnosed, **not fixed**. Nothing in the working tree was modified to produce
any number below. Recorded (D13), against `f9b8b8f`.

All measurements are `demo`, **seed 42**. Unless a row says otherwise they come from
`Mission(scn, 42, hivemind=False)` — Tier 3 off, which is the control condition
(invariant #1). The one run with the scripted hivemind rung agrees on every shared figure
(76.2% explored either way), so Tier 3 is not involved.

---

## 1. The short version

The swarm's **default state is standing still**, and the mission spends itself pushing
robots back into that default faster than the auction can pull them out.

1. An explore assignment is retired the moment **anybody** reveals its target cell — not
   when its holder arrives. The median explore assignment therefore lives **1.0 second
   and dies 88 m short of its goal**. Only **7% of 4,432** explore assignments ever ended
   within 5 m of the thing they were sent to.
2. A robot with no assignment, in comms, is given **no goal at all** — so it stops dead
   until it wins another auction round, and the auction only places ~15 robots/second.
3. Three amplifiers make (1) and (2) worse monotonically: un-deduplicated targets pack
   scouts into piles the safety floor then freezes, a 20 s no-progress window that
   guarantees churn on 100–300 m walks, and a task supply that shrinks from 84 to 57 as
   sectors cross the sweep threshold.

Net effect over one mission:

| | t≈30 | t=420 |
|---|---:|---:|
| ground explored, gain per 20 s | **+26.16 pts** | **+0.42 pts** |
| alive robots moving >1.5 m per 30 s | 487 / 512 | **274 / 450** |
| time-averaged ticks at `v == 0` | 13.7% | **25.4%** |
| mean speed per alive robot | 0.93 m/s | **0.49 m/s** |
| robots alive but displaced <1.5 m in 60 s | 23 (4%) | **167 (37%)** |

---

## 2. Finding 1 — an explore task completes when anyone reveals the cell (primary)

[`skill_executor.py:335-341`](swarmmind/nodes/skill_executor.py#L335-L341)

```python
if a.kind == "explore":
    ix, iy = _cell(world, a.target)
    if world.explored[iy, ix]:
        return "done"
    if _dist(world.pos[i], a.target) <= arrive_radius:
        return "done"
    return "running"
```

`world.explored` is the **shared** map. The first branch does not mention the robot
holding the task. An explore target sits on the explored/unexplored boundary by
construction, 512 robots lift fog at 5 Hz, so the cell is revealed by whoever happens to
be nearest within a second or two — and every robot assigned to it is released wherever
it is standing.

Instrumenting every `SkillExecutor.release` for a whole mission:

```
explore: 4432 assignments ended
   completed   2681 ( 60%)  dist-to-goal median   88.1 m  p90  207.2 m  held median   1.0 s
   stalled     1687 ( 38%)  dist-to-goal median  173.9 m  p90  293.5 m  held median  22.0 s
   preempted     58 (  1%)  dist-to-goal median  100.8 m  p90  177.2 m  held median  19.5 s
   destroyed      6 (  0%)  dist-to-goal median  131.0 m  p90  175.0 m  held median  21.0 s
   ended within 5 m of goal: 320 of 4432 (7%)
```

**The median explore assignment lasts one second and ends 88 metres short.** The proximity
branch requires ≤ 2.5 m, so at least 93% of the "completed" ones went out through the
`explored[iy, ix]` branch without the robot going anywhere. 4,432 assignments over 420 s
across ~250 scouts is the churn this produces.

### 2a. And a fifth to a third of them are dead on announcement

Sampling the generator's output directly:

| t | explored | sweep targets already explored | frontier targets already explored | centroids that are actually frontier cells |
|---:|---:|---:|---:|---:|
| 60 | 37.5% | 6/32 (19%) | 18/48 (38%) | 26/48 |
| 180 | 55.3% | 3/18 (17%) | 12/47 (26%) | 29/48 |
| 300 | 69.4% | 2/11 (18%) | 9/47 (19%) | 32/48 |
| 420 | 76.2% | 3/9 (33%) | 15/48 (31%) | 24/48 |

Roughly **half the 48 frontier targets do not land on a frontier cell at all**, because
[`planner.py:356`](swarmmind/control/planner.py#L356) takes the **centroid** of the
frontier cells in an 8×8 bin:

```python
cx = (sums_x[k] / counts[k] + 0.5) * world.cell
```

A frontier bin is boundary-shaped — a curve or an L — and the mean of a curve lands on the
concave side, which is the *explored* side. That is precisely the failure
`frontier_mask`'s own docstring says was already found and fixed:

> "The frontier must lie on the unknown side of the boundary. Defining it as the explored
> side makes every explore task complete the instant it is issued — the target cell is
> already explored by construction — so robots receive a task, retire it, and never move."

The mask was fixed. The **centroid taken from it** puts the target back on the explored
side. Same failure, one function later.

Sector-sweep targets have the same problem for a different reason: the target is
`snap_passable(sector centre)` and the completion test is the same `explored` lookup, so a
sector at 30% explored whose centre cell happens to have been seen emits a task that every
robot awarded it retires on the same tick — for as long as the sector stays under
`sweep_below`.

### 2b. What that looks like from the target's side

Robot-seconds of assignment accumulated per explore target, against robots actually
present:

```
t=420   (291.5, 101.5)  robot-seconds=8321   robots within 5 m now = 1
        (292.5, 180.5)  robot-seconds=7606   robots within 5 m now = 0
        (292.5, 220.5)  robot-seconds=6778   robots within 5 m now = 0
        (246.5, 219.5)  robot-seconds=6611   robots within 5 m now = 0
```

Those coordinates are sector centres (45 × 40 m sectors). One point absorbed **8,321
robot-seconds** of assignment across the mission and never had more than one robot near
it.

---

## 3. Finding 2 — an idle robot in comms is given no goal, so it freezes

[`skill_executor.py:155-159`](swarmmind/nodes/skill_executor.py#L155-L159)

```python
if not world.in_comms[i]:
    g = _nearest_contact(world, i)
    self.reason[i] = "out of contact, closing on the nearest link"
else:
    continue          # <- in comms and idle: no goal
```

`goal_id = -1` → `_dir = 0` → `mag < 1e-6` → `moving = False` → `v = 0`. There is a
comms-recovery reflex for the out-of-contact case and **nothing at all** for the ordinary
one. Confirmed on every sample: of the robots alive, stationary and unassigned, *all* of
them had no goal.

| t | still & idle (in comms) | of which: no goal at all |
|---:|---:|---:|
| 120 | 34 | 34 |
| 180 | 47 | 47 |
| 300 | 38 | 38 |
| 420 | 29 | 29 |

Time-averaged over alive robot-ticks, the unassigned fraction rises **13.5% → 19.8%**
across the mission.

This is defensible in isolation — "drift looks like malfunction on the dashboard" —
but combined with Finding 1 it means the swarm's resting state is *motionless*, and the
allocator is the only thing that moves anybody.

---

## 4. Finding 3 — the auction cannot re-employ them fast enough

Per 30 s window, from `AuctionNode.stats`:

| t | tasks generated | announced | **awarded** | **no_bidder** | idle robots |
|---:|---:|---:|---:|---:|---:|
| 30 | 84 | 2672 | 995 | 1677 | 51 |
| 180 | 66 | 2143 | 477 | 1666 | 67 |
| 300 | 60 | 2186 | 296 | 1890 | 86 |
| 420 | 57 | 1947 | **456** | **1491** | 66 |

**~65 tasks are announced per cycle and ~15 are awarded**; the rest find no eligible free
bidder (`free` requires idle **and** `in_comms` **and** the right lane). So the machine
that is supposed to unfreeze robots places about fifteen per second, against a churn that
retires far more than that.

Task supply also shrinks. Frontier targets stay pinned at the 48 cap all mission, but
sector sweeps fall away as sectors cross `sweep_below = 0.55`:

| t | frontier targets | sweep targets | total tasks | explore assignments held | distinct explore targets | **max pile on one target** |
|---:|---:|---:|---:|---:|---:|---:|
| 30 | 48 | 35 | 84 | 318 | 42 | 24 |
| 120 | 48 | 26 | 77 | 299 | 39 | 48 |
| 240 | 48 | 14 | 61 | 252 | 54 | 43 |
| 420 | 48 | **9** | 57 | 258 | 43 | 38 |

---

## 5. Finding 4 — the piles, and the safety floor freezing them

Explore targets are deliberately not deduplicated
([`tasks.py:104`](swarmmind/nodes/tasks.py#L104), `max_per_target = 0`) with the M-36
revert written up in place. The consequence, measured on the cohort that holds a task and
has displaced < 1.5 m in the last 60 s:

| t | stuck with task | neighbours < 2.5 m (median / p90 / max) | blocked by wall override | swarm-wide | mean commanded v | their v_max | on rubble |
|---:|---:|---|---:|---:|---:|---:|---:|
| 60 | 8 | 0 / 2 / 4 | 0.2% | 3.2% | 0.357 | 0.57 | 25% |
| 180 | 62 | 6 / 15 / 28 | 36.8% | 7.9% | 0.201 | 1.64 | 23% |
| 300 | 68 | 39 / 53 / 54 | 34.8% | 9.2% | 0.135 | 1.70 | 69% |
| 420 | **100** | **10 / 61 / 63** | **34.6%** | 11.3% | **0.157** | **1.82** | 52% |

**Sixty-three robots inside a 2.5 m separation radius.** One separation disc is 19.6 m².
The force balance on that cohort:

```
t=420   |w_goal * dir| = 1.00    |w_obstacle * rep| = 1.10    |w_separation * sep| = 0.55
```

Obstacle repulsion **outvotes the goal term**, separation adds another half on top, the
Tier-1 swept-circle override zeroes them a third of the time against a swarm-wide 11%,
and the result is **0.157 m/s against a v_max of 1.82**. This is M-47's "137 robots simply
too crowded to move", relocated from the spawn area to the frontier and not caught by the
staging-area fix.

---

## 6. Finding 5 — the 20 s no-progress window guarantees churn

[`skill_executor.py:56`](swarmmind/nodes/skill_executor.py#L56) —
`NO_PROGRESS_AFTER["explore"] = 20.0`, against explore walks of 100–300 m. A robot in the
pile above cannot close `PROGRESS_EPS` (0.5 m) between 1 Hz checkpoints at 0.157 m/s, so
it is released every 20 s and re-awarded. 38% of explore assignments end this way, a
median **173.9 m** from the target.

**The relay lane is still broken, and M-55 did not close it.** Same instrumentation, same
run:

```
relay: 199 assignments ended
   stalled      193 ( 97%)  dist-to-goal median  75.1 m  p90 260.7 m  held median 27.0 s
   other          6 (  3%)
   ended within 5 m of goal: 2 of 199 (1%)
```

M-55 reports 310 → 199 assignment-ends and arrivals 30 → 35 after its two fixes. The
*count* fell; the *ratio* did not — **97% of relay posts still die stalled and 1% are
reached**. Whatever M-55 fixed, it was not the thing that stops relays arriving. Worth
re-opening that entry rather than treating it as closed.

`investigate` (41% stalled) and `clear_debris` (54% stalled) show the same shape.

---

## 7. Hypotheses tested and ruled out

Recorded because the project's own history says the wrong hypotheses are the useful part.

| hypothesis | verdict | evidence |
|---|---|---|
| Robots stand still because the flow field has no descent direction at their cell (unreachable coarse cell, disconnected pocket) | **wrong** | `assigned + ZERO-FLOW` = **0** at every sample, t=60…420. Not one robot. |
| The M-55 `UNREACHABLE → "no evidence"` early-return lets a robot hold a task forever while frozen | **wrong** | `unreachable-goal held` = **0** at every sample. The branch is real but nothing is sitting in it. |
| The coarse nav grid cuts regions off for some chassis | **negligible** | coarse cells connected to base: wheeled 96.3%, tracked 97.2%, legged 99.7%, rotor 99.7%. |
| Robots are dying (battery / hazard) and that is why they stop | **no** | 450 of 512 alive at t=420; losses start at t≈200 and total 62. The stationary count is 167 among the *survivors*. |
| The announcement cap starves exploration | **no**, as M-53 already found | 57–84 tasks generated, cap ≥ 24 and scaling with free robots; nothing is dropped. The cap is not the constraint — **the awarding is**. |
| Per-tick cost grows over the mission, so the demo visibly slows | **no** | flat at 45.9–57.7 ms/tick across every 30 s window. See §8 for the separate, non-progressive version of this. |

---

## 8. Separate issue found along the way — there is no realtime headroom left

```
$ time uv run python -m swarmmind.cli run --headless --seed 42
  ground explored     76.2%
  wall / rtf          434.32s / 0.97x
  hash                f7e7755146e16ffc
  424.65s user  449.74s total
```

**434 s of wall clock for a 420 s mission.** CLAUDE.md documents this run as ~25 s — that
line is now out by ~17x and should be corrected whatever else happens.

The cost is flat, not growing (~48 ms/tick against a 50 ms budget at `tick_hz = 20`), so
this is not the progressive part of the reported symptom. But it matters for the demo:
`DemoSim.step` ([`backend.py:71-78`](swarmmind/sim/backend.py#L71-L78)) sleeps only when
it is **ahead** of the wall clock. At 0.97x it is never ahead, so it can never catch up,
and `--demo` adds the WS bridge at 10 Hz, the blackboard at 10 Hz and the hivemind thread
on top of an already-negative margin. Everything on screen runs slow, uniformly.

Two things also grow without bound during a mission. Neither is currently the cost driver,
both are worth knowing about:

| | t=30 | t=420 |
|---|---:|---:|
| `tracker.reports` | 1,058 | **3,602** (983 dismissed + 2,533 resolved, never pruned) |
| `world._pending_cells` arrays | 46 | ~1,500 (peak 1,549) |

`VictimReportTracker.prune` drops only stale **candidates**; dismissed reports are kept
deliberately (the swarm's memory of "we looked, nobody there") and resolved ones are never
dropped at all. Every `_merge` builds a (detections × open_reports) distance matrix at
5 Hz over that growing list.

---

## 9. How to reproduce

`scripts/diagnose.py --every 20` gives the coverage/comms/relay decay directly. Everything
else came from ad-hoc probes written for this pass (kept in the session scratchpad, not
committed); each is a few lines around `Mission.tick()`:

1. **Release attribution (§2, §6)** — wrap `SkillExecutor.release`, record
   `(reason-tag, distance from `_goal_for`, `world.t - a.assigned_at`)` per task kind.
   This is the single most informative measurement and is the M-55 method applied to
   scouts.
2. **Stationary attribution (§3)** — every 60 s, bucket alive robots that displaced
   < 1.5 m into `idle / assigned+arrived / assigned+zero-flow / assigned+other`, reading
   `reflex.last_arrived` and `reflex._dir`.
3. **Tier-1 forces (§5)** — wrap `ReflexController._wall_override` to capture the blocked
   mask and returned `v`; report `_dir` / `_rep` / `_sep` magnitudes and neighbour counts
   for the stuck cohort.
4. **Target supply and piles (§4, §2b)** — every 30 s, `Counter` over
   `executor.assignment` targets for `kind == "explore"`, alongside `frontier_targets()`
   and the sector sweep count.
5. **Pre-explored targets (§2a)** — call `gen.generate(...)` and test
   `world.explored[cell(t.target)]` per task, split on `rank < 5.0` (sweep) vs `>= 5.0`
   (frontier).

---

## 10. Where a fix would go, if one is attempted

Not attempted here, and worth noting that D13's own conclusion (M-55) is that
**a mechanism change should be justified as a correctness fix, not a performance one**,
unless it ships with a measurement. Findings 1 and 2 are correctness — the code does not
do what its own documentation says. Findings 3–5 are the coupling CLAUDE.md warns about
and should be measured, not tuned.

- **§2** is the one to attack first, and it is small: the completion test conflates "this
  ground is now known" with "this robot's job is done". The centroid in
  `frontier_targets` is a second, independent defect in the same path.
- **§3** needs an answer to "what does a robot do when the market has nothing for it", and
  the honest options (hold, wander, spread) have different demo optics.
- **§5** is `max_per_target`, already a parameter, with the M-36 revert and the M-48
  re-test both written up in `tasks.py`. The premise of that revert (comms reach on a
  480×320 map) no longer holds; re-testing it is cheap.
- **§6** should re-open the M-55 entry: 193/199 relay stalls and 2/199 arrivals say the
  relay lane is not fixed.
