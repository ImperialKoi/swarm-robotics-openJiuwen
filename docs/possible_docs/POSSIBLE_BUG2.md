# POSSIBLE BUG 2 — the swarm grinds to a halt, and the flow field is why

**Status: diagnosed, not fixed. No code was changed.**
Investigated (D13) against `demo`, seed 42, `hivemind=False`.

Symptom as reported: *near the end nearly all of the robots don't move any more, and as
the run progresses the exploration rate gets slower and slower.*

Both halves are real and they are the same defect. This file records what was measured,
what the cause is, and what was ruled out. Numbers here belong in
[MEASUREMENTS.md](docs/MEASUREMENTS.md) as **M-56** if this is accepted; the last row
there is M-55.

---

## 1. It is not performance

Ruled out first, because "everything slows down" reads like the sim falling behind.

| block | wall s | RTF | auction.step | reflex | perceive | nav cache misses |
|---:|---:|---:|---:|---:|---:|---:|
| 0–30 s | 16.9 | 1.77 | 4.52 | 5.07 | 3.40 | 2150 |
| 180–210 | 20.6 | 1.46 | 1.45 | 5.43 | 2.76 | 388 |
| 390–420 | 19.4 | 1.55 | 0.98 | 4.45 | 1.96 | 139 |

Per-tick cost is flat to slightly *falling*. The `NavFields` cache is healthy (281,563
hits against 7,281 misses over the mission; 2,048 resident entries, under the 512-per-
chassis cap). **The slowdown is in the simulation, not the clock.**

## 2. What actually decays

Sampled every 60 s. "Travellers" = alive, holding a goal, not yet arrived.

| t | mean speed | v/v_max, travellers | nbrs < 2.5 m | \|rep\| | \|sep\| | cos(err) | override % | explored |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 60 | 0.71 m/s | 0.703 | 1.73 | 0.151 | 0.342 | 0.717 | 3.7 | 37.5% |
| 120 | 0.53 | 0.615 | 1.53 | 0.197 | 0.354 | 0.650 | 7.2 | 47.5% |
| 180 | 0.46 | 0.559 | 3.20 | 0.220 | 0.379 | 0.610 | 9.4 | 55.3% |
| 240 | 0.37 | 0.499 | 5.11 | 0.245 | 0.408 | 0.539 | 10.0 | 64.8% |
| 300 | 0.33 | 0.430 | 6.51 | 0.268 | 0.425 | 0.486 | 11.4 | 69.5% |
| 360 | 0.27 | 0.396 | 8.46 | 0.285 | 0.442 | 0.433 | 12.7 | 72.9% |
| 420 | 0.24 | 0.367 | 8.96 | 0.297 | 0.482 | 0.421 | 13.6 | 76.2% |

**The swarm is not going idle — it is grinding.** Robots holding a goal stay at ~89% of
the swarm and travellers at ~83%, both flat across the whole run. Only 7.6% are parked at
a goal at t=420. What collapses is the speed of robots that are *trying to move*.

The mechanism inside Tier 1 is visible in the same table: obstacle repulsion doubles,
separation rises 40%, so `desired = 1.0·dir + 2.2·rep + 0.7·sep` turns away from the goal,
`cos(err)` falls, and the speed gate in
[`ReflexController.commands`](swarmmind/control/tier1_reflex.py) throttles them. The
swept-circle override then zeroes 13.6% of traveller-ticks outright.

Task outcomes invert to match — `explore` assignments ending `stalled` overtake
`completed`:

| t | explore completed | explore stalled |
|---:|---:|---:|
| 0–60 | 668 | 112 |
| 120–180 | 522 | 210 |
| 240–300 | 168 | 304 |
| 300–360 | 193 | 314 |

## 3. The cause: coarse-grid corridors that are solid rock

At t=420, **58 of 450 living robots sit in one 6 × 6 m bin at (186, 162) m.** The clump
appears at t≈150 and grows monotonically — 19 → 29 → 36 → 45 → 47 → 52 → 52 → 56 → 60 → 58
— and never dissolves. The twelve densest bins hold 30.2% of the swarm.

Dumping every robot in it: they hold **different** goals scattered over the entire east
half of the map — (292,180), (337,220), (339,61), (292,101), (252,172), (200,11),
(219,199) — and every one of them has the **identical steepest-descent step `+0.7,+0.7`**,
at 0.00–0.19 m/s. Mean speed inside the clump 0.079 m/s, against 0.457 for the rest of
the swarm.

The fine grid at the clump, `#` impassable, digits = robots per cell:

```
  y=168 ............#############........#############
  y=167 ............#############........#############
  y=166 ............###...2499##.........#############
  y=165 ..................22796######.....############
  y=164 ...................13.#########...############
  y=163 .....................###########...###########
        ^x=170      ^x=182  ^x=190  ^x=198
```

A dead-end notch roughly 5 × 3 m, open only to the west.

**Why the field sends them there.** Verified cell by cell on seed 42:

| | |
|---|---|
| `NavFields` downsamples 4× with a majority rule | [planner.py:60](swarmmind/control/planner.py#L60), [grid.py:228](swarmmind/sim/grid.py#L228) |
| coarse (47,41) covers fine x 188–191, y 164–167 | 12/16 passable → **passable** |
| coarse (48,42) covers fine x 192–195, y 168–171 | 10/16 passable → **passable** |
| `distance_field` is an 8-connected wavefront | so the diagonal between them is a legal edge |
| the two fine cells that must touch: (191,167) and (192,168) | **both rock — no fine-grid link** |

Both coarse cells pass the majority test honestly; their passable fine cells simply lie on
opposite sides of the block. The corridor the flow field is routing through does not exist.

[`_direction_grid`](swarmmind/control/planner.py#L172) stores exactly **one** step per
coarse cell, computed once and static for the mission. There is no second-best step and no
invalidation. So the cell is an **absorbing trap**: a robot enters, the safety floor zeroes
it, and the field keeps ordering it northeast for the rest of the run.

**This is systemic, not one bad corner.** Directed coarse steps that cross ground with no
fine-grid link, chassis `wheeled`:

| seed | phantom / total | |
|---:|---|---:|
| 42 | 1090 / 19946 | 5.5% |
| 43 | 1138 / 18552 | 6.1% |
| 44 | 830 / 19608 | 4.2% |
| 45 | 1012 / 18368 | 5.5% |

Three-quarters of them are diagonal. Per-chassis phantom step counts on seed 42:
wheeled 1090, tracked 1182, legged 1180, rotor 1092.

Robots standing in a cell whose descent step for their own goal is a phantom edge:
85 (16.6% of the swarm) at t=60 → 135 (30.0%) at t=420, mean speed 0.176 m/s against
0.267 for everyone else. **That instantaneous flag over-counts** — at t=60 those robots
were moving at 0.738 m/s, faster than the rest, because most were merely passing through.
The trend and the speed differential are the signal; the single clump is the hard evidence.

### Downstream

- 14,111 passable cells (23.8%) finish never explored; the whole east third of the map.
- 24 of the 48 frontier targets become permanent zombies. (68,228) was announced **388
  times**, from t=30 to t=420. Median age of a live frontier target: 6 s at t=60, 82 s at
  t=420.
- `frontier_targets` is pinned at its 48 cap against 251 real clusters, so the zombies
  crowd out reachable work by occupying slots in the largest-48 ordering.

## 4. Why nothing recovers them — and one thing makes it worse

1. **Stall detection churns without moving anything.** `_stalled` does fire (315 explore
   stalls per 60 s at the end), but the released robot is re-awarded a *different* eastern
   target whose field gives the *same* step at that cell, and walks straight back in.
   Assignment ages inside the clump are mostly 1–26 s: they are cycling, not stuck on one
   task.
2. **Tier 1 cannot push them out.** Repulsion points west, but the goal term plus ~50
   robots of separation pressure from behind hold the front rank against the rock.
   Invariant #2 is working correctly here — the override is the symptom, not the fault.
3. **The auction actively feeds the trap.** `_best_bidder` reads its travel term off the
   same coarse field ([auction.py:203](swarmmind/nodes/auction.py#L203)), which reports
   100 m from the notch to (292,180). Robots in the trap therefore look like the *best*
   bidders for eastern work, so the auction keeps sending more. The mechanism is verified
   (the field value and the code path); the causal contribution to the monotonic growth is
   inferred from that plus the growth curve, **not** from an A/B.

## 5. Relationship to the existing record

[FIXES.md §3](docs/FIXES.md) carries this as ⚠️ open: *"~33% Tier 1 override rate remains;
the coarse/fine mismatch hypothesis was measured and rejected"* (M-23).

What M-23 tested was the **threshold** on individual cells, and it correctly found stricter
values strictly worse at every setting. That is a different hypothesis. The defect here is
not per-cell occupancy — it is that a **diagonal edge between two majority-passable coarse
cells whose shared corner is solid is unwalkable**, and no value of `threshold` fixes it.
The hypothesis should be reopened with that distinction recorded, so it is not rejected a
second time on the old evidence.

## 6. Secondary defect, independent and smaller

[skill_executor.py:300](swarmmind/nodes/skill_executor.py#L300) — the
`d >= UNREACHABLE * 0.5` branch resets `progress_at` and returns `False`, so a robot whose
goal is *genuinely* unreachable for its chassis **never stalls out and is never released**.

| t | held by this branch | median hold | max hold |
|---:|---:|---:|---:|
| 60 | 29 | 57 s | 60 s |
| 240 | 32 | 60 s | 240 s |
| 420 | 39 | 108 s | **420 s** |

A max equal to the mission length means at least one robot held a single assignment from
t=0 to the end. The M-55 comment justifying the branch is about *transient* UNREACHABLE
readings caused by the downsample, and that reasoning is sound; the branch as written also
swallows the permanent case, which has no such excuse. ~30–39 robots, so ~8% of the swarm
— real, but not the main story above.

## 7. Ruled out along the way

| hypothesis | measurement | verdict |
|---|---|---|
| The sim is falling behind wall-clock | RTF flat 1.4–1.8, per-tick cost falling | wrong (§1) |
| Batteries run out and robots die | 450 of 512 alive at t=420; battery does not gate speed, only death | wrong |
| Robots go idle waiting on the auction | goal-holders flat at ~89%, travellers flat at ~83% | wrong (§2) |
| Dead robots exert phantom separation forces | true — `_separation` has no alive mask — but only 0.22 dead neighbours/robot at t=420 | real, negligible |
| The pocket is disconnected from base on the fine grid | it is in the same 43,825-cell component as base | wrong — the trap is *local*, not a connectivity failure |
| `hazard_known` latches and permanently sterilises ground | recomputed each pass as `haz & explored`; shrinks as the fire decays | wrong |
| Task supply starves the swarm | 57–89 open tasks/cycle against a cap that never binds | wrong; the bottleneck is bidder eligibility, and behind that, the trap |

Also noted while reading: ~21–34 antenna robots sit permanently idle-in-comms once relay
posts stop generating, because `eligible()` bars them from every remaining task kind. By
design, but it is a visible chunk of stationary robots on the dashboard and worth knowing
about when judging "nothing is moving".

## 8. Reproduction

```bash
uv run python -m swarmmind.cli run --headless --seed 42     # baseline scorecard
```

Seed 42 endgame for reference: 42 rescued, 68 found of 110, 76.2% explored, 62 lost.

The diagnosis above came from ad-hoc probes (phase timings, stopped-robot classification,
frontier-target persistence, per-robot clump dump, phantom-edge enumeration). They were
scratch scripts and are not checked in. The two cheap ones worth keeping if this is
pursued:

- **phantom-edge count** — for each chassis, every 8-connected coarse step between two
  passable coarse cells, assert at least one adjacent passable fine-cell pair across the
  shared edge or corner. Static, runs in seconds, no mission needed. Would make a good
  test against map generation.
- **clump detector** — robots per 6 × 6 m bin each 30 s; flag any bin above ~15. Catches
  this class of failure regardless of cause.
