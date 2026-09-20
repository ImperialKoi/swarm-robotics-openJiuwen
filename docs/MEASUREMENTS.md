# Measurements

Numbers measured on the target machine (Apple M1, 8 GB, 8 cores). Re-run and append;
never delete a row -- the trend across days is what tells you whether a change regressed.

---

## M-1 · Robot-count sweep · D1

`uv run python scripts/stress.py`, `demo` scenario, seed 42.

**Controller: `control/wander.py` (D1 placeholder).** No raycasting, no A*, no auction,
no bidding. This is a floor-of-cost measurement, not the real one.

| N | ms/tick | RTF | steps/s | peak RSS MB |
|---:|---:|---:|---:|---:|
| 64 | 1.09 | 45.8 | 917 | 37 |
| 128 | 2.25 | 22.3 | 445 | 38 |
| 256 | 3.65 | 13.7 | 274 | 39 |
| 512 | 7.71 | 6.5 | 130 | 41 |
| 1024 | 14.31 | 3.5 | 70 | 42 |
| 2048 | 30.33 | 1.6 | 33 | 75 |
| 3072 | 48.82 | 1.0 | 20 | 134 |
| 4096 | 66.09 | 0.8 | 15 | 196 |
| 6144 | 95.87 | 0.5 | 10 | 418 |

- **Ceiling (RTF >= 0.8x): 3072.** 80% of ceiling = **2456**.
- Tick cost is linear in N up to ~2048, then superlinear: the O(N x relays) comms
  pairwise block starts to dominate, visible as RSS jumping 42 -> 134 -> 418 MB.
- 420 s mission at N=64 completes headless in 4.97 s wall (84x real time).

**This ceiling will fall, substantially.** Not yet measured with:
- D2 Tier 1: 8 raycasts per robot per tick at 20 Hz, plus A* on goal change.
- D3 auction: N x open_tasks bid evaluations at 1 Hz.
- D7 residency: Godot (~350 MB) and llama-server (~1.4 GB) sharing 8 GB.

Expect the post-D3 ceiling to land in the high hundreds. Re-run at D2, D3 and D7 and
append rows here.

### The constraint that actually binds

Compute is not what limits N. The mission is. At the current `demo` scenario
(12 sectors, 8 victims, 96 x 64 m), a swarm in the thousands:

- clears all fog in seconds, so the 5-7 minute demo becomes a 20 second one;
- gives the auction ~20 open tasks for thousands of bidders, so >99% of robots idle
  and there is no allocation problem left to solve;
- makes the scripted failure invisible -- one robot of 2456 dying is not a demo beat;
- renders as an undifferentiated smear on the dashboard.

**Robot count and mission size must scale together.** See PLAN.md section 3 for the
coupled scaling rule.

---

## M-2 · Obstacle density tuning for the scaled map · D1

320 x 208 m, cell 1.0 m, `cluster_radius_m: [2.5, 9.0]`, seed 42.

| n_clusters | passable frac | max geodesic dist from base |
|---:|---:|---:|
| 200 | 0.768 | 307 m |
| 260 | 0.758 | 316 m |
| 310 | 0.714 | 314 m |
| 380 | 0.658 | 323 m |
| 460 | 0.624 | 348 m |
| 970 | 0.337 | 528 m |

**Chosen: 380** (0.658 passable). Scaling cluster *count* by map area alone was wrong --
cluster *radius* grew too, so obstacle coverage rose 33x against a 10.8x larger map.

Carry distances at that density: victim to nearest extraction zone, median **64 m**,
worst **108 m**; worst loaded round trip 299 s of a 420 s mission. Four corner zones were
not enough (worst case 160 m) -- a centre zone at (160, 104) was added.

---

## M-3 · Scaled mission, 512 robots · D1

`--scenario demo --seed 42`, full 420 s, D1 wander controller.

- **25.26 s wall, 16.6x real time.** Comfortable headroom.
- 17/80 victims found, 0 rescued (no task allocation until D3), 39.6% explored,
  132/512 robots lost to the hazard (wanderers have no hazard aversion).

Random walk is 5-10x less efficient than frontier exploration, so 39.6% coverage here
suggests directed exploration will clear this map comfortably inside 420 s. **Re-check at
D3: if coverage completes too early the map is too small, and `map.width_m`/`height_m`
should grow before `mission_duration_s` shrinks.**

---

## M-4 · Distance-field cost · D1 · **blocks D3**

`grid.distance_field` on the demo grid (208 x 320, 43,841 passable cells), 8 random targets.

**22.4 ms per field.** Against a 1 Hz auction cycle:

| open tasks | cost per second | share of the 20 Hz tick budget |
|---:|---:|---:|
| 20 | 449 ms | 45% |
| 48 | 1,077 ms | 108% |
| 100 | 2,244 ms | 224% |

At 48 sectors and 80 victims the auction can legitimately see **200+ open tasks**
(48 explore + up to 80 clear_debris + 80 extract + relay). That is ~4.5 s of compute per
second of sim. **One field per task per cycle does not scale and must not be built that way.**

Three mitigations, all required (docs/TECHNICAL.md section 5.2):

1. **Coarse navigation grid.** Downsample `passable` 4x (80 x 52 = 4,160 cells) for
   bidding only. Fewer cells *and* a shorter wavefront diameter: ~16x cheaper, ~1.4 ms.
   Bidding ranks candidates; it does not need 1 m precision. Tier 1 keeps the fine grid.
2. **Cap announced tasks per cycle** to the top ~24 by priority rank. A real auctioneer
   does not announce every task at once either.
3. **Amortise** field computation across the 20 ticks of the 1 Hz cycle -- fields are
   only read at award time.

Together: ~24 x 1.4 ms = 34 ms per cycle, ~3% of budget.

---

## M-5 · Robot-count sweep, real Tier 1 + Tier 2 · D2

`scripts/stress.py --scenario demo --seconds 150`. Measured through `Mission`, so the
numbers include flow-field navigation, Tier 1 steering, task execution and allocation --
not just world physics.

| N | ms/tick | RTF | steps/s | peak RSS MB |
|---:|---:|---:|---:|---:|
| 128 | 2.59 | 19.3 | 386 | 53 |
| 256 | 3.42 | 14.6 | 293 | 54 |
| 512 | 9.63 | 5.2 | 104 | 68 |
| 1024 | 15.76 | 3.2 | 63 | 82 |
| 1536 | 22.26 | 2.2 | 45 | 133 |
| 2048 | 31.94 | 1.6 | 31 | 161 |
| 3072 | 54.85 | 0.9 | 18 | 359 |
| 4096 | 83.72 | 0.6 | 12 | 584 |

- **Ceiling (RTF >= 0.8x): 3072.** 80% = 2456. Unchanged conclusion from M-1: compute
  does not bind, mission size does. **N stays 512**, with ~6x headroom.
- Not comparable to M-1: that sweep ran the old 96 x 64 m map before the rescale.
- RSS grows superlinearly past ~1500 -- the O(N^2) separation buffers in
  `ReflexController`. At N=512 they are ~1 MB; at 4096, ~200 MB. Spatial binning is the
  fix if N ever needs to go there, not removing separation.

**Methodology note worth keeping.** A first attempt swept a 25 s window and reported
10.3x at N=512, while the full 420 s mission measured 4.90x -- a 2x overstatement. The
opening of a mission is its cheapest phase: no hazard, a small frontier, few victims
found, few open tasks. **Any window shorter than `hazard.ignite_t` measures the wrong
thing.** The default is now 150 s.

---

## M-6 · Mission outcomes with Tier 1 + Tier 2 · D2

Heuristic allocation, no auction, no hivemind. Seed 42.

| scenario | rescued | found | explored | robots lost | RTF |
|---|---:|---:|---:|---:|---:|
| `test` (16 robots, 8 victims) | 1/8 | 8/8 | 84.6% | 1/16 | 42x |
| `demo` (512 robots, 80 victims) | 12/80 | 32/80 | 57.4% | 98/512 | 4.9x |

Search works; extraction is the bottleneck and losses are high. Both are expected to
improve at D3 and D6, and both are *why those stages exist*:

- **Extraction throughput** is limited by greedy nearest-robot allocation, which does
  not consider battery, load, or hazard along the path. The auction's bid function does.
- **98 robots lost** is the hazard doing its job against a swarm with no strategic
  layer. Tier 1 flees hazard it can see ~6 m ahead and the heuristic pre-empts robots
  standing in it, but neither can decide to evacuate a sector *before* the front
  arrives. That is precisely the hivemind's abandon directive (D6).

Treat these as the **classical baseline** the gate measures against, not as a target.

---

## M-7 · Perception: cost and detector characteristic · D3

### Cost, 512 robots, 48 x 48 x 3 frames

| stage | ms per pass |
|---|---:|
| raster render | 1.5 |
| camera capture (crop, rotate, occlude, noise) | 46.2 |
| classical detector | 6.7 |
| **total** | **54.4** |

At 5 Hz that is **27.2% of the 20 Hz tick budget**. Peak RSS 223 MB. Expect mission RTF
to fall from ~4.9x (M-5) to roughly 2x once wired in -- still real-time with headroom,
but half of it. Resolution trade measured at 512 robots:

| H x W | total ms | % budget at 5 Hz |
|---|---:|---:|
| 32 x 32 | 32.8 | 16.4% |
| 40 x 40 | 45.8 | 22.9% |
| **48 x 48** | **54.4** | **27.2%** |
| 64 x 64 | 103.4 | 51.7% |

Batched robot stamping in the raster cut render from 9.8 ms to 1.5 ms -- a Python loop
over 512 robots at 5 Hz for no reason. **If perception has to get cheaper, drop `H x W`
before dropping `N`.**

### Detector characteristic (classical, test scenario, 16 robots facing victims)

| range | detections | true | phantom | distinct victims | precision |
|---:|---:|---:|---:|---:|---:|
| 1.5 m | 126 | 121 | 5 | 8/8 | 0.96 |
| 2.5 m | 100 | 99 | 1 | 7/8 | 0.99 |
| 3.5 m | 80 | 79 | 1 | 7/8 | 0.99 |
| 5.0 m | 66 | 32 | 34 | 7/8 | 0.48 |
| 6.5 m | 44 | 2 | 42 | 2/8 | 0.05 |

This is the point of the exercise: **precision collapses with range**, so the swarm will
chase phantoms at distance and only confirm up close. It is why the blackboard must
corroborate candidates before promoting them to confirmed victims (D4), and it is the
baseline the trained CNN has to beat at the gate (D11).

Buried victims are detected only at close range (4/4 at 2 m, 0/4 at 5.5 m), because a
buried victim renders as a *small* patch of victim colour that grows as debris clears.

### Two mistakes worth not repeating

1. **Blending a buried victim toward rubble.** The obvious way to render "covered by
   debris", and it makes a fully buried victim render as pure rubble -- invisible to
   every camera, therefore never found, therefore never dug out. Size carries the
   occlusion, not colour.
2. **Luminance-only sensor noise.** Broadcasting one noise value across R, G and B
   leaves `R - B` -- the exact signal the detector keys on -- completely unperturbed.
   The system reported zero false positives at every range, which looked like a good
   detector and was actually a silent bug. Noise must be per-channel.

A third, related: warm rubble was initially rendered at `R - B = 106`, below the
detector's band, so it could never fire. It is now 128, inside the band, and separating
it from a victim requires **extent** rather than colour -- which is what gives the
classical detector its characteristic errors and gives a CNN something to beat.

---

## M-8 · Perception wired in, oracle removed · D3

`world.py` no longer discovers casualties. The line `seen = dist <= sensor_radius`
against ground-truth coordinates is gone; discovery now runs
camera -> detector -> report tracker -> `mark_found`.

### Mission outcomes, seed 42

| scenario | rescued | found | explored | lost | RTF |
|---|---:|---:|---:|---:|---:|
| `test`, oracle (M-6) | 1/8 | 8/8 | 84.6% | 1/16 | 42x |
| `test`, real perception | 3/8 | 5/8 | 83.9% | 1/16 | 31x |
| `demo`, oracle (M-6) | 12/80 | 32/80 | 57.4% | 98/512 | 4.9x |
| **`demo`, real perception** | **9/80** | **32/80** | **62.8%** | **53/512** | **3.4x** |

Real perception costs ~30% of throughput and finds exactly as many casualties at scale.
Losses nearly halved (98 -> 53) because robots now travel to specific reported contacts
instead of wandering toward frontier centroids.

### The report funnel, and how badly the first version failed

Confirmation rules, `test` scenario, whole mission:

| version | created | confirmed | resolved (real) | dismissed | precision of confirmed |
|---|---:|---:|---:|---:|---:|
| first attempt | 3765 | 3698 | 148 | 3545 | **4%** |
| **shipped** | 232 | 112 | 49 | 63 | **44%** |

The first version confirmed essentially everything it saw and the swarm spent the
mission investigating rocks. Three things were wrong, all of them the kind of mistake
that looks like a working system:

1. **Repeated looks from one robot counted as corroboration.** At 5 Hz a robot parked in
   front of warm rubble reached a 7-observation threshold in 1.4 seconds. Fixed by
   requiring **parallax**: an observation only counts as a new view if the observer has
   moved more than 2 m since the last counted one.
2. **Distant sightings could confirm on their own.** Precision at 6.5 m is 0.05 (M-7),
   so a crowd of near-worthless detections promoted each other. Fixed by weighting
   confidence with a range-trust ramp taken from the measured curve, and requiring one
   genuinely close look before promotion.
3. **Dismissed reports were pruned and immediately re-created.** A robot would walk to a
   rock, see nothing, and the swarm would rediscover the same rock a minute later --
   forever. Dismissed reports are now kept for the whole mission as the swarm's memory
   of "we looked there, nobody home", and absorb repeat sightings instead of spawning
   new ones.

### Field of view

`test` scenario, whole mission:

| FOV | explored | found | rescued |
|---:|---:|---:|---:|
| 90 deg | 83.9% | 5/8 | 3/8 |
| 150 deg | 98.4% | 5/8 | 1/8 |

Wider explores more but rescues less -- more reports means more time investigating.
**Shipped 90 deg**, which is both a realistic camera and the better performer.

---

## M-9 · Auction, blackboard, self-healing · D4

**M1 reached.** A full mission allocates, executes and recovers with no LLM in the loop
-- the hivemind is not implemented at all yet, and nothing is waiting for it.

### Allocators, `demo`, seed 42

| allocator | rescued | found | explored | lost | orphaned | comms flaps | RTF |
|---|---:|---:|---:|---:|---:|---:|---:|
| greedy (baseline) | **18/80** | 24/80 | 54.0% | 94/512 | 690 | 955 | 3.2x |
| auction (shipped) | 16/80 | **28/80** | **56.9%** | **88/512** | **555** | **601** | 3.0x |

**The auction does not beat the baseline on rescues on this seed**, and that is recorded
rather than tuned away. It leads on coverage, casualties found, robots kept alive and
comms stability; the greedy assigner leads on the headline number. One seed is not a
gate -- the real comparison is 10 held-out seeds at D13, per TECHNICAL.md section 8.
Both consume an identical task list (`nodes/tasks.py`) and differ only in allocation,
which is what makes the comparison meaningful at all.

### Three bugs, each a cascade

1. **A flat announcement cap throttled the whole swarm.** `MAX_ANNOUNCED = 24` was sized
   against 22.4 ms fine-grid distance fields (M-4). Bidding uses the *coarse* nav grid --
   ~16x cheaper and cached for the mission since `passable` never changes -- so the real
   budget was far larger. At 512 robots the cap meant only 24 robots could be given work
   per second and the rest idled: 10/80 rescued, 18/80 found, 48% explored. Scaling the
   cap to 1.5x the free-robot count gave **16/80, 28/80, 56.9%**.

2. **Comms boundary flapping.** Robots sitting exactly on the range boundary connected
   and disconnected every cycle -- 609 events in one test mission, each orphaning a task.
   Fixed with hysteresis: joining the component needs 0.92x the radius, staying in needs
   1.0x. Flaps fell 609 -> 40 on `test`.

3. **Orphaning released the robot, which caused a cascade.** An out-of-contact relay had
   its task released, became free, bid on a frontier, walked off its post -- and the
   swarm behind it lost contact, orphaning more relays. One run ended with 0 of 16 robots
   in comms. Two fixes: an orphaned assignment is now re-offered *without* being taken
   away (the robot keeps working; duplicate effort is what really happens when a machine
   goes out of contact), and antenna robots are barred by policy from ever taking search
   work, in both allocators.

### A fourth, found only by reading the victim states

Three casualties finished the mission in state `carried` -- picked up and never
delivered. Pickup is proximity-based in the world, so a carrier doing an `investigate`
task grabs a casualty it passes, then carries them to the contact it was investigating
and onward, forever. Opportunistic pickup is the right behaviour; the *task* not
following it was the bug. A robot holding a casualty now has its assignment rewritten to
`extract` on the spot.

---

## M-10 · Dashboard bridge · D5

Stdlib WebSocket server (`bus/ws_server.py`), no `websockets` dependency. Verified
against a hand-rolled client: handshake with a checked `Sec-WebSocket-Accept`, all three
frame-length paths (7-bit, 16-bit, 64-bit), and masked client -> server commands.

Wire volume, `demo` scenario, 512 robots:

| message | rate | size | throughput |
|---|---|---:|---:|
| `hello` (incl. terrain PNG) | once | ~90 KB | -- |
| `state` (512 robots + HUD + contacts) | 10 Hz | ~15 KB | ~150 KB/s |
| `fog` (bit-packed explored mask) | 2 Hz | ~11 KB | ~22 KB/s |
| `truth` (ground truth, dashboard only) | 2 Hz | ~4 KB | ~8 KB/s |

**Robots are sent as a flat array of numbers, not objects.** The object form is ~1 MB/s
of JSON at 512 robots; flat is ~150 KB/s and Godot parses it in one loop. If that ever
bites, the next step is binary packing, not fewer robots.

**Godot cannot be run from this environment**, so `tests/test_bridge_protocol.py` stands
in for testing the client: it parses `godot/scripts/main.gd`, extracts every field the
dashboard reads, and asserts the bridge sends it -- including the positional widths of
the robot and contact rows, which GDScript indexes numerically. A rename on either side
fails the build rather than silently blanking the dashboard on demo day.

---

## M-11 · 3D world · D5b

The dashboard is now genuinely 3D. **The simulation is not, and should not be**: 512
robots with 3D physics do not fit this machine, and flow-field navigation, auction
distance fields and the perception raster all rest on a 2D grid. `world.height` is a
render-only heightfield derived from the same occupancy grid the sim uses, so what is
drawn and what is simulated cannot drift. Keep saying *custom 2.5D kinematic simulator*.

### The verification problem, and the fix

Godot cannot be run from the development environment, and neither can a browser. Writing
camera maths, eye-height framing and grid extrusion blind is how a week gets lost.
`swarmmind/viz/render3d.py` is a numpy point-splat rasteriser with a depth buffer that
renders the same scene to PNG, so 3D work is inspected rather than guessed at. Godot then
implements maths that has already been seen working.

It caught two things immediately that would otherwise have shipped:

1. **Confetti floor in first-person.** Splat size is `scale x f / z` clamped to 9 px, but
   from 1.1 m eye height a 0.5 m ground cell two metres away subtends far more than that,
   so the floor broke apart and sky showed through it. Fixed by raising the cap to 56 px
   with bucketed sizes, plus a horizon fill -- below the horizon every view ray meets the
   ground eventually, so painting it first is the plane at infinity, not a hack.
2. **The world was far too dark to read.** The appearance raster is tuned for the
   *detector*: dark ground, low contrast, warm rubble deliberately confusable with a
   casualty. Those are the right choices for computer vision and the wrong ones for a
   human. `render3d.py` and the Godot dashboard now use a separate display palette.
   What the cameras see and what the operator sees are different problems.

### Cost

| view | resolution | points | time |
|---|---|---:|---:|
| orbit | 1000 x 620 | 75,006 | ~160 ms |
| first-person | 1000 x 620 | 75,006 | ~920 ms |

Offline only -- this is a verification tool, not a renderer in the demo path. First-person
is slower because near-field splats are large. Godot draws the same scene with real meshes.

### Godot construction

- **Ground**: one `ArrayMesh`, `(w+1) x (h+1)` vertices, vertex colours carrying baked
  directional shading -- the same fake-diffuse term `render3d.py` uses, so the offline
  frames and the live view match.
- **Obstacles**: `MultiMeshInstance3D` of boxes on wall and rubble cells. This is what
  gives the blocky silhouette; extruding every cell into six faces would be ~660k
  triangles for the demo map.
- **Robots**: `MultiMeshInstance3D`, one instance per robot, per-instance colour.
- **Fog**: one shader shared by ground and obstacles, sampling a bit-packed explored
  texture by world XY. God-view flips a uniform rather than rebuilding anything.
- **Axis convention**: Godot is Y-up, the simulator is Z-up. The mapping lives in exactly
  one function, `world_to_godot()`.

---

## M-12 · The dashboard never connected · D5c

Godot 4.7.2 reported no errors and sat on "waiting for simulator" while the bridge
happily reported a connected client.

**Cause: `WebSocketPeer` defaults to a 64 KB inbound buffer and drops the connection when
a single frame exceeds it -- silently, with no error on either side.** The `hello` message
carried the terrain PNG, the heightfield and the occupancy grid together: ~100 KB on the
small scenario, ~200 KB on `demo`. It looked exactly like the simulator not running.

Fixed on both sides, deliberately:

- **Bridge:** the map payload is chunked into `blob` messages of 32,000 base64 characters,
  followed by `hello_done`. Largest frame is now 31.3 KB on `demo` (was ~200 KB). The
  dashboard works without depending on the client having raised its buffer.
- **Godot:** `inbound_buffer_size` raised to 4 MB as belt and braces, and the status line
  now shows `receiving map 5/9` so a stalled handshake is visible rather than mute.
- **Test:** `test_no_frame_exceeds_godots_default_inbound_buffer` fails the build if any
  handshake frame goes over 60 KB again.

Handshake on `demo`: 11 messages -- 1 `hello`, 9 `blob` (terrain 69 KB, height 65 KB,
occ 65 KB decoded), 1 `hello_done`.

---

## M-13 · Atmosphere · D5c

Height-attenuated exponential fog, in the shader rather than through `Environment`
post-processing -- the GL Compatibility renderer, chosen to keep memory free for the
simulator and the local model, does not support colour-adjustment effects reliably.

Plain distance fog failed immediately and instructively: at a density thick enough to feel
atmospheric in first-person, the operator's camera at ~340 m rendered the entire map as a
flat grey rectangle. Attenuating density with altitude
(`density * exp(-mid_height / 14 m)`) fixes both views at once -- thick down among the
rubble, transparent from above. An exponential atmosphere is also what actually happens,
so this is physics rather than a fudge.

The display palette is now desaturated (dust-covered concrete and earth) and the sky is
near-flat overcast. `render3d.py` and `main.gd` compute the identical fog term, so the
offline frames stay a truthful preview of the live dashboard.

---

## M-14 · Bigger map, and what it cost · D5d

Map scaled to **480 x 320 m** (2.31x area, 153,600 cells), 768 robots, 120 casualties,
12 collection points. Obstacle density re-swept to ~0.67 passable at 900 clusters --
never scale cluster count by area without re-sweeping.

### First result: much worse

| | rescued | found | explored | lost |
|---|---:|---:|---:|---:|
| 320 x 208, 512 robots | 16/80 | 28/80 | 56.9% | 88/512 |
| 480 x 320, 768 robots | 4/120 | 10/120 | 30.7% | 124/768 |

Area rose 2.31x, robots only 1.5x, so area per robot rose 54%.

### Two hypotheses measured and rejected

- **Robot count.** 768 -> 1536 moved coverage only 23.6% -> 32.3%. Badly sublinear.
- **Frontier target cap.** 48 -> 240 targets moved coverage 23.2% -> 23.7%, and distinct
  goals only went 99 -> 106. The frontier does not *have* more clusters to offer.

### The actual cause: frontier exploration is perimeter-limited

The known region grows only at its boundary, and a boundary absorbs a bounded number of
robots. Adding robots to one growing blob does nothing; a large swarm on a large map
needs **multiple independent fronts**.

`TaskGenerator` now also emits an explore target at the centre of every sector below
`sweep_below` coverage, ranked alongside frontier work rather than behind it.

| | explored @250 | found | in comms |
|---|---:|---:|---:|
| frontier only | 23.2% | 5 | 722/768 |
| **+ sector sweep** | **30.4%** | **10** | 602/768 |

In-comms drops because robots opening distant fronts outrun the relay chain -- the exact
tension the relay lane exists to resolve, now visible rather than hypothetical.

**Sweeping is density-gated.** On the 16-robot test fixture (1.3 robots per sector) it
*cut* casualties found from 5 to 3: opening a front costs travel time, which only pays
when there are robots to spare. Enabled at >= 8 robots per sector, so it scales with the
swarm instead of being a hardcoded choice.

---

## M-15 · Rescue work could not preempt search · D5d

**This broke self-healing outright, and the M1 test caught it.**

The auction awards only to *free* robots. Once every carrier was off exploring, a
confirmed casualty sat unclaimed no matter how it was ranked -- and the task released
when a robot dies simply went unfilled. `extract tasks offered: [0]`, `free grippers: 0`.
Ranking work is meaningless if the allocator cannot act on the ranking.

Two fixes:

1. **Rescue-tier tasks may take a robot off strictly lower-priority work**
   (`AuctionNode.PREEMPT_RANK`). A carrier can be pulled off a frontier for a casualty,
   never off a casualty for a frontier. Bids are compared exactly as in a normal round.
2. **Orphan detection moved to heartbeat rate.** Detecting a loss only on the auction
   cycle put the worst case at `orphan_timeout + auction_period` = exactly 3.0 s, which
   is the number the self-healing claim is measured against -- no margin at all. An
   orphan now forces an immediate auction, cutting it to roughly the timeout plus a tick.

### Net effect on the enlarged map

| | rescued | found | explored | RTF |
|---|---:|---:|---:|---:|
| before both fixes | 4/120 | 10/120 | 30.7% | 2.48x |
| **after** | **8/120** | **14/120** | **41.9%** | 2.03x |

Determinism holds; seed 42 still hashes identically across processes.

---

## M-16 · Ruins assets imported · D5d

33 of 56 models from the RuinsGR pack -> `assets/models/ruins/`, 6.9 MB, glTF binary with
embedded textures. Walls are **1.00 m thick and 6.6 m tall**, which snaps onto the 1 m
simulation grid. Per-model dimensions in `ruins/manifest.json`.

Left out (25 of the pack's 41 MB): moss, fantasy accessories, rigged skeleton characters,
sounds, scripts, `.tscn` scenes (they hardcode paths that break on move) and `.import`
files (Godot regenerates them).

**Licence unresolved** -- no LICENSE or README shipped and the glTF `copyright` field is
empty in all 56 models. Recorded in `assets/models/ATTRIBUTION.md` with a checklist. The
dashboard renders procedural geometry from the occupancy grid, so if the licence turns
out to prohibit this use, removing the meshes costs a look, not a feature.

---

## M-17 · Casualty placement bias · D5d

`demo`, 480 x 320 m, 768 robots, seed 42, full 420 s. Only `victims.distance_weight_exp`
varies -- placement weight is `distance_to_base ** exp`.

| exp | median dist from base | explored | found | rescued |
|---:|---:|---:|---:|---:|
| 1.5 | 348 m | 41.9% | 14/120 | 8 |
| 0.8 | 328 m | 41.7% | 23/120 | 14 |
| **0.4** | 316 m | **42.2%** | **30/120** | **18** |

**Coverage is identical across all three.** The swarm searches exactly as much ground;
the exponent only decides how many casualties are standing on it. Note the median barely
moves (348 -> 316 m, 9%) while found more than doubles -- it is the tail that matters,
and at 1.5 a large fraction sits on the far rim.

A strong far-bias was right when the swarm cleared the whole map: it pushed the last
casualties out far enough that the run used its full clock. At 2.3x the area and ~42%
coverage it does something quite different -- it hides casualties in ground nobody
reaches. **Shipped 0.4**, keeping the spec's intent that distant sectors are harder
without making them unreachable.

### Where the enlarged map now stands

| | area | robots | explored | found | rescued |
|---|---:|---:|---:|---:|---:|
| 320 x 208 (old) | 1.00x | 512 | 56.9% | 28/80 | 16 |
| **480 x 320 (shipped)** | **2.31x** | **768** | 42.2% | 30/120 | **18** |

The larger map now recovers **more casualties in absolute terms** than the map it
replaced, over 2.3x the ground. Percentages are lower and should be: a swarm that cannot
search everything in the time available is the situation where strategic prioritisation
is worth something, which is precisely the case the hivemind has to make at D7.

---

## M-18 · Casualties placed where nobody could reach them · D5e

Placement filtered on `passable & isfinite(dist_from_base)`. That is not sufficient, and
**17.3% of the cells it accepted on the demo map were unreachable in practice** -- roughly
one casualty in six would have been stranded.

Two things it missed:

1. **Robots navigate on the coarse grid, not the fine one.** `NavFields` downsamples 4x
   with majority-passable, so a one-cell pocket that is reachable on the fine grid has no
   route on the grid the flow fields are actually built from.
2. **Bodies have width.** A casualty in a narrow gap is `passable`, but no carrier can
   approach within `REACH_GRAB`. This is the worse failure: the swarm sees it, dispatches
   a carrier, and the carrier never arrives -- which looks exactly like the system working.

`World._navigable()` now requires clearance of at least one cell plus reachability on the
*same* downsampled grid `NavFields` uses. Candidate cells: 103,302 -> 85,430 on `demo`.

| | found | explored |
|---|---:|---:|
| before | 30/120 | 42.2% |
| after | **36/120** | 42.0% |

Coverage is unchanged, as it must be -- the swarm searches the same ground. Six more
casualties were simply standing somewhere a robot could get to.

### It also invalidated a detector measurement

`test_precision_degrades_with_range` began failing with precision 1.00 at 6.5 m, where
M-7 recorded 0.05. Not a regression: casualties now stand in clearings, so a robot facing
one sees open floor rather than the warm rubble that generates phantoms. The test asserted
a property of the old placement. It now checks **recall** falls with range, which survives
the change, and a separate test points robots at rubble specifically to confirm the
confuser still fools the detector.

---

## M-19 · Asset import · D5e

| pack | source size | imported | models |
|---|---:|---:|---:|
| RuinsGR (MakoviceMH) | 41 MB | 6.7 MB | 33 |
| nature pack | 292 MB | 3.1 MB | 12 |

The nature pack shrank 94x for two reasons. Only the ten dead trees and two grass tufts
were taken -- birch, maple, pine, bushes and flowers are lush and green, wrong for a
dust-covered collapsed city. And **41 MB of that was two normal maps**
(`NormalTree_Bark_Normal.png` 19 MB, `MapleTree_Bark_Normal.png` 22 MB), which have
**no effect whatsoever** here: the world shader is `render_mode unshaded`, with
directional shading baked into vertex colours to match `viz/render3d.py`.

The `.gltf` files were rewritten rather than left pointing at absent files: `normalTexture`
and `occlusionTexture` removed from each material, then the orphaned textures, images and
samplers deleted with every index remapped. All twelve validate.

**Meshes live in `godot/assets/`, not `assets/models/`.** Godot can only load resources
under its own project root; a mesh outside `godot/` has no `res://` path and cannot be
referenced at all. Three tests guard this: every `res://` path in `main.gd` resolves,
meshes are inside the project root, and every `.gltf` sidecar `.bin` and texture exists.

---

## M-20 · Making the world look like the assets · D5f

First render with meshes in was wrong: a field of grey procedural lumps with ruins
perched on them. The meshes were dressing on the terrain instead of being the structures.

Three causes, all mine:

1. **The heightfield made walls 3.2-5.6 m tall.** The terrain itself was the vertical
   feature, so 6.6 m wall meshes had nothing to stand in and everything to compete with.
   Walls are now 0.9-2.2 m rubble mounds; the meshes carry the height.
2. **The debris boxes were 3 m pillars.** Now 1.1 m mounds -- the debris the ruins stand
   in, not the ruins.
3. **No directional light, and white ambient at full energy.** The ground and debris use
   the unshaded world shader with baked vertex shading, but the imported meshes carry
   their own *lit* materials and had nothing to catch: no highlight, no shadow side, no
   silhouette. Added a warm low sun at 1.25 energy and dropped ambient to 0.42, tinted to
   the sky.

### Prop density is a measured budget, not a look

Imported meshes average ~3,500 triangles, and **Godot's MultiMesh does not cull per
instance** -- the whole buffer is submitted every frame.

| density | props | triangles |
|---|---:|---:|
| 1 per 7 cells | 8,288 | 28.5M — will not run |
| 1 per 22 | 2,829 | 9.7M |
| **1 per 40 (shipped)** | **1,592** | **5.4M** |

Directional shadow distance is 140 m, so distant ruins are lit but do not cast --
that is what keeps ~1,600 instances affordable alongside 768 robots.

Also fixed: **horizontal orbit was inverted** (dragging right rotated the world left),
and fog density eased 0.030 -> 0.020 in both the shader and the offline renderer, which
must stay in step or the previews stop being truthful.

---

## M-21 · Terrain and the locomotion axis · D5g

Added mountains, hills, valleys, rivers and ditches (`sim/terrain.py`), and a third robot
axis: **chassis** (wheeled / tracked / legged), mixed evenly within every actuator lane.
Slope and water now gate movement per robot, so the heightfield stopped being render-only.

| chassis | max slope | max wade | speed |
|---|---:|---:|---:|
| wheeled | 0.40 | 0.00 m | x1.18 |
| tracked | 0.58 | 0.45 m | x1.00 |
| legged | 0.95 | 0.95 m | x0.80 |

Routing is per chassis throughout: `NavSet` holds one flow-field cache each, Tier 1
groups by (goal, chassis), and a robot whose locomotion cannot reach a target reads
UNREACHABLE and bids astronomically -- so "the carriers on the far bank cannot take this"
needs no special case.

### Where it landed

| scenario | seeds building | wheeled | tracked | legged |
|---|---:|---:|---:|---:|
| `test` | 20/20 | 90% | 91% | 93% |
| `demo` | 16/20 | 56% | 57% | **84%** |

Fraction of the map **reachable from base**, which is the metric that matters -- water
covering 3% of cells can cut off 40% of the map, and "% traversable" hides that entirely.

**Legged units are meaningfully privileged; wheeled and tracked are not yet distinct.**
Their separation would need terrain with gradients in the 0.40-0.58 band, and the
landform currently has almost none: mountains are gentle enough to be climbable by both,
and the barrier that bites is water, which stops both equally. Unfinished.

### Five wrong turns, recorded because each looked right

1. **Terrain generated on top of the base.** One seed had *zero* reachable cells -- a
   river carved through the staging area sealed it off before the mission started.
2. **Blurring to fix steep ground.** A 3x3 mean of a linear ramp returns the same ramp:
   blurring removes high-frequency detail, not gradient. One failing seed had 97% of its
   cells moved by up to 10 m with connectivity unchanged at 1.5%. Amplitude scaling is
   what reduces a gradient.
3. **Eroding "passes" toward base.** Ran up to 40 rounds of double-blur over a growing
   band -- a global smoother in disguise. Took relief from 70 m to 17 m and flattened the
   differentiation it was meant to protect.
4. **Demanding a tracked-connected map.** The requirement was never real. Ground only
   legged units can reach is the locomotion axis working, not a defect. Judging
   workability on the *most* capable chassis removed the need for erosion entirely.
5. **Blaming the mountains.** The base noise octaves contribute roughly
   `amp / wavelength` each; the original four summed to ~1.27, over even a legged robot's
   limit before a single peak was placed. The *grain* was severing the map. Octave
   amplitudes are now set from a gradient budget.

### Also this round

- **Casualty and contact markers restored to the 3D dashboard.** Dropped in the 2D->3D
  rewrite: the bridge kept sending `truth` and `reports` and nothing drew them. `V`
  toggles casualties, contacts always show. Element 8 of the MVP dashboard spec.
- **Quaternius Ultimate Nature Pack (CC0) credited**, MakoviceMH credited for RuinsGR.
- Prop density set from a measured triangle budget: 1 per 40 cells, ~1,592 props,
  5.4M triangles. At 1 per 7 it was 28.5M and would not have run.

---

## M-22 · Three chassis that each earn their slot · D5h

Tracked was strictly dominated: same reach as wheeled, 18% slower, no reason to build
one. Fixed by giving each locomotion a barrier only it clears.

| chassis | max slope | max wade | speed | reach (demo, seed 42) |
|---|---:|---:|---:|---:|
| wheeled | 0.40 | 0.00 m | x1.18 | 58% |
| tracked | 0.58 | 0.45 m | x1.00 | 75% |
| legged | 1.60 | 1.20 m | x0.80 | 92% |

All three reach all 11 collection points. Reach rises as speed falls, which is the
trade that makes all three worth building -- asserted in `tests/test_terrain.py`.

Three barrier tiers, each sized against the limits above:

- **Marsh** (0.20-0.38 m): stops wheels, nothing else. *This is the entire reason the
  tracked chassis exists.*
- **Rivers** (0.64-0.86 m water): stop tracks, fordable by legs.
- **Slope**: mountain steepness is drawn from a ratio range so some flanks stop wheels
  only and others stop tracks too. Peak height is derived from radius, not set directly,
  because the max gradient of a gaussian peak is 1.27 x (peak / radius) and *slope* is
  what gates traversal.

### Scenario robustness: 30/30 seeds, up from 13/20

Three bugs, none of them terrain:

1. **A complete graph of roads.** Twelve collection points gave 66 corridors, each
   regrading the terrain it crossed, so intersections became steps -- measured max slope
   on a road was 1.17, three times a wheeled unit's limit. A spanning tree needs 11 edges
   and barely crosses itself. Road centreline is now 96% wheel-passable in aggregate.
2. **Roads that were not roads.** Clearing obstacles along a corridor leaves it crossing
   marsh and hillside. On the shipped seed a wheeled unit reached 2% of the map. Roads are
   now regraded to a constant slope between endpoints and drained.
3. **Obstacle clusters sealing the base in.** Seed 44 left **753 passable cells of
   153,600** once connectivity pruning ran -- a pre-existing map-generation bug, nothing
   to do with terrain. Roads are carved before pruning so the fixed points share a
   component.

### The cost, stated plainly

| | rescued | found | explored | lost | RTF |
|---|---:|---:|---:|---:|---:|
| before terrain | 18/120 | 30/120 | 42.2% | 147/768 | 1.58x |
| **with terrain** | **4/120** | **14/120** | **22.1%** | 111/768 | 1.68x |

**Terrain made the mission substantially harder and this is not yet tuned back.** Routes
are longer, a third of the swarm is restricted to 58% of the map, and coverage nearly
halved. Fewer robots are lost, because terrain keeps them out of the hazard. The
scenario needs rebalancing -- more time, more robots, or gentler terrain -- and that is
open work, not a finished result.

### Dead code removed

`world._build_heightfield` (superseded by `sim/terrain.py`), and `terrain._blur3` /
`_disc_at`, orphaned when the erosion experiment was deleted.

---

## M-23 · Why the swarm stopped moving · D5i

Terrain dropped the mission to 4/120 rescued and 22.1% explored. The instinct was to
rebalance the scenario. Measuring first showed the scenario was not the problem:

> **robots travelled a mean of 62 m in 420 s, out of a possible ~630.**

Not an exploration-strategy problem. The swarm was barely moving. Four causes, each found
by measurement and each a genuine bug:

| fix | mean distance | explored | found | rescued |
|---|---:|---:|---:|---:|
| baseline | 62 m | 22.1% | 14/120 | 4 |
| Tier 1 sees terrain | 82 m | 23.5% | 20/120 | 4 |
| staging area sized to the swarm | 89 m | 28.6% | 22/120 | 10 |
| chassis-aware spawn | 171 m | 35.5% | 26/120 | 11 |
| **stranded robots can escape** | **171 m** | **35.5%** | **26/120** | **11** |

Losses also fell sharply: **147 -> 61 of 768**. Robots that can perceive terrain stop
blundering into the hazard.

Against the pre-terrain baseline (18 rescued, 30 found, 42.2% explored), the mission is
still down. Terrain genuinely costs coverage -- routes are longer and a third of the
swarm reaches 58% of the map -- and closing that gap is scenario balance, not bug
hunting. **RTF still needs a clean re-measure**: the 0.68x on the final run was taken
while a parameter sweep was competing for the same eight cores, and the last idle
measurement was 1.68x.

1. **Tier 1 could not see terrain.** Obstacle repulsion and the swept-circle override
   both tested `occ == WALL`, so robots got no avoidance signal from water or steep
   ground: they drove into it and stalled. Both are now chassis-aware.
2. **The staging area did not scale.** 768 robots spawned into a ~22 m disc -- about
   2 m^2 each against a 2.5 m separation radius, so every robot sat inside every
   neighbour's repulsion field and the swarm gridlocked at the start line. Radius is now
   derived from robot count and separation radius.
3. **Spawn ignored chassis.** Placement checked walls only, so a wheeled robot could
   start in marsh. Its own cell being impassable meant the override zeroed its speed
   every tick, forever: **52.5% of robot-ticks had a goal and zero commanded speed.**
4. **Stranded robots could never recover.** Once a robot's own cell was impassable --
   hazard grew over it, a neighbour shoved it -- `_collides` blocked every move because
   its own centre sample failed. Any shove into bad terrain was permanent. Stranded
   robots now move at 30% speed regardless, like a machine struggling out of mud.

### A hypothesis measured and rejected

The remaining 33% override rate looked like a coarse/fine mismatch: the nav grid
downsamples 4x with a *majority* rule, so a coarse cell that is 51% passable can promise
a route with no fine path through it. Tightening the threshold should have helped.

| nav threshold | explored | found | rescued | override |
|---:|---:|---:|---:|---:|
| **0.5 (kept)** | **35.5%** | **26** | **11** | 33.4% |
| 0.75 | 30.0% | 24 | 11 | 33.6% |
| 0.9 | 24.2% | 18 | 8 | 33.0% |

Stricter thresholds are strictly worse and leave the override rate untouched -- they
remove real routes without removing false ones. The default stays at 0.5. The threshold
is now a documented parameter rather than a buried constant, and the cause of the
remaining override rate is still open.

---

## M-24 — real-time factor, re-measured on an idle machine (D7)

M-19 recorded **0.68x** and flagged it as suspect: that run shared eight cores with a
parameter sweep. Re-run with nothing else on the box, demo scenario, 768 robots, seed 42:

| condition | wall | sim | RTF |
|---|---:|---:|---:|
| M-19, contended | 616.1 s | 420 s | 0.68x |
| **M-24, idle** | **258.9 s** | **420 s** | **1.62x** |

The 2.4x gap is contention, not a regression. **The demo runs faster than real time with
Tier 3 running**, which is the number that actually matters -- the earlier reading would
have triggered an optimisation push against a problem that did not exist. Benchmarks on
this machine are only meaningful when nothing else is on it.

## M-25 — Tier 3: directive volume and the filter's rejection rate (D7)

Demo scenario, seed 42, 420 s, scripted provider (70 hivemind cycles).

| | issued | renewed | rejected | dominant rejection |
|---|---:|---:|---:|---|
| first working build | 221 | -- | 6 | -- |
| separating renewal from decision | 9 | 200 | 29 | F8 collection point (21) |
| provider skips collection points | 9 | 200 | 29 | F7 known casualty (29) |
| **provider respects the casualty floor** | **9** | **200** | **0** | -- |

Two things worth keeping:

**"221 directives issued" was a measurement artefact.** Almost all of them were the same
directive re-sent to hold a position. Counting a renewal as a decision would have put a
number on the scorecard that overstates what the hivemind did by 20x -- exactly the kind
of overstatement that costs credibility with a judge who looks closely.

**A filter that fires constantly is not a safety net.** The scripted baseline was tripping
F8 twenty-one times a run by proposing to abandon the sector containing the base. Teaching
the *provider* those two rules dropped rejections to zero without weakening the filter:
every rule still fires in `tests/test_hivemind.py` against a payload built to break it.
The filter now catches model error, not baseline sloppiness, so a rejection on the
dashboard means something when a judge sees one.

## M-26 — does Tier 3 actually help? (D7)

Demo scenario, 768 robots, 420 s, identical seeds, one variable: `hivemind=False` vs the
**scripted** provider. No model has run yet — this is the baseline rung, and it is the
number the trained hivemind has to beat at the gate.

| seed | mode | rescued | found | explored | lost |
|---:|---|---:|---:|---:|---:|
| 42 | off | 11 | 26 | 35.5% | 61 |
| 42 | **on** | **12** | **30** | 34.0% | 64 |
| 43 | off | 7 | 26 | 28.5% | 4 |
| 43 | **on** | **14** | **28** | **32.3%** | 5 |
| 44 | off | 10 | 24 | 22.5% | 1 |
| 44 | **on** | 10 | 24 | 21.2% | 1 |
| **total** | off | **28** | **76** | 28.8% | 66 |
| **total** | **on** | **36** | **82** | 29.2% | 70 |

**Read this cautiously.** Rescues are up 29% in aggregate, but almost all of that is one
seed: 43 nearly doubles, 42 gains one, and 44 is identical to the row above it. Three
seeds cannot separate a real effect from variance, and the honest summary is *promising,
not demonstrated*. The gate (§8) needs held-out seeds and more of them before any claim
about Tier 3's contribution goes near a judge.

Two things the table does show without ambiguity:

- **The mechanism works end to end.** Directives are issued, they change what the auction
  announces, they evacuate closed sectors, and they expire. Seed 43 is what that looks
  like when concentrating search happens to pay.
- **Explored fraction moves the other way on 2 of 3 seeds.** That is the expected trade,
  not a bug: prioritising sectors concentrates the swarm, which finds more casualties per
  cell covered and covers fewer cells. Rescue is the mission objective and coverage is
  the proxy, so trading coverage for rescues is the right direction — but it does mean
  "explored %" alone would report Tier 3 as a regression.

## M-27 — the local model, first contact (D7)

`llama serve` (llama.cpp build 10679, the unified `llama` CLI) serving
`Qwen2.5-1.5B-Instruct` Q4_K_M, 4096 context, Metal. Prompt is the real `prompt.build()`
output on `test`; decode is JSON-schema-constrained.

### Latency, and what it cost to get there

| build | reasoning cap | per-directive `reason` | max_tokens | mean | max | over timeout |
|---|---|---|---:|---:|---:|---|
| first working call | none | yes | 512 | — | **10.61 s** | **1/1** |
| length caps in the grammar | 180 | yes, ≤48 | 200 | 4.05 s | 5.35 s | 1/3 |
| **`reason` dropped** | **120** | **no** | **200** | **3.53 s** | **3.95 s** | **0/6** |

**Generation dominates, so the schema is the latency budget.** The first call spent 10.6 s
writing a 700-character essay and four verbose per-directive justifications — against a
4 s rung timeout, meaning every single cycle would have fallen through to the scripted
baseline and the model would have appeared to work while never once being used. llama.cpp
compiles `maxLength` into the GBNF grammar, so the cap is enforced at decode time rather
than requested in the prompt and hoped for.

Dropping the per-directive `reason` field alone cut mean latency ~13%: it was roughly half
the generated tokens, four short essays restating the sector table, and it appeared
nowhere except a fallback in one event string.

Timeout raised 4.0 s → 5.0 s afterwards. At a 6 s cadence a 5 s timeout still leaves the
instant scripted rung room to answer in the same cycle.

### Quality

Unprompted, the base model was **mechanically inert**: it emitted four `priority: normal`
directives — well-formed, filter-accepted, and changing nothing, since `normal` is already
every sector's default. It also chose the *most*-explored sectors to explore and invented
hazard figures ("A1 … with 100% hazard"; A1 was at 0%).

Two changes fixed it, and they belong at different layers:

- **The system prompt now states what each field does to the swarm** — that `normal` is
  the default and costs a slot for nothing, that unexplored ground is where undiscovered
  casualties are — plus one worked example.
- **Filter rule F10** rejects a directive asking for the state a sector is already in.
  Renewals are exempt, or a live `abandon` would be refused and the sector would silently
  reopen 30 s later.

Six-cycle run after both: **9 of 11 accepted directives changed something**, 0 rejections,
and the model reached for low-explored sectors, `rescue` where contacts were awaiting, and
`abandon` on the burning one.

### Full mission, model in the loop (`test`, seed 42)

    directives   11 issued, 0 rejected     calls 6, timed out 0, fell through 0
    latency      mean 3394 ms, max 4127 ms
    explored     82.6%   (scripted rung: 73.7%)
    rescued      3/8
    C3 abandoned at 373.0s -> reopened at 403.0s when the directive expired

**Headless Tier 3 is wall-clock bound, not sim bound.** A headless mission runs ~20x
real time, so the model answered 6 times in 420 s of simulated time where the demo gets
~70. Headless numbers therefore understate Tier 3's involvement; the realtime path is the
one that matters and it is measured below.

## M-28 — realtime cost of Tier 3, and the blocking bug it exposed (D7)

Putting a real model behind the node exposed a design bug that no test had caught, because
every test until now used a provider that answers instantly.

**`step()` joined the worker thread with the rung timeout.** The thread was there so a hung
provider could be abandoned — but the tick loop still waited for it. Headless that is
merely slow. `DemoSim` runs at wall-clock speed, so a 3.5 s call every 6 s would have
frozen the dashboard for **more than half the demo**. The cycle is now genuinely
asynchronous: one tick starts a request, a later tick collects it, and inference spans as
many ticks as it needs.

Realtime, `test`, 45 s of simulation, per-tick cost (50 ms budget at 20 Hz):

| | mean | p99 | max |
|---|---:|---:|---:|
| hivemind off | 50.34 ms | 55.31 ms | 351.81 ms |
| **hivemind on (base-local)** | **49.98 ms** | **55.10 ms** | **87.07 ms** |

Indistinguishable — and the *larger* outlier is on the run with Tier 3 disabled, which
places those spikes in warm-up, not the hivemind. Realtime factor held at 1.00x with
directives landing every 5–8 s.

## M-29 — memory, with the model resident (D7)

`llama serve`, Qwen2.5-1.5B Q4_K_M, `--ctx-size 4096`, measured with `vmmap`:

| | physical footprint |
|---|---:|
| steady state | **829 MB** |
| peak | **921 MB** |

Against 8.6 GB of physical RAM and the ~4.7 GB budget in TECHNICAL.md §9. `ps` RSS reports
only 89 MB for this process and is wrong for the purpose — the weights are mmap'd and
Metal holds them outside conventional resident memory. Use `vmmap`'s physical footprint.

Not yet measured: Godot, the simulator and the model resident *simultaneously*. That is
the number the budget actually cares about and it needs the dashboard running.

## M-30 — evaluation throughput, and a planning error of ~200x (D9)

`docs/TECHNICAL.md` §6.2 budgeted MAP-Elites at "~3.3 ms/eval/core x 8 cores ≈ 20 minutes
locally" for 300k evaluations. That estimate was never measured. The real numbers:

| | |
|---|---:|
| one 60 s `FastSim` episode, `test`, 16 robots | **1.85 s** (1.54 ms/tick) |
| one evaluation (3 training seeds) | **5.5 s** single-core |
| 21 evaluations, 7 workers | 2.10 s/eval — **1,716/hour** |
| **84 evaluations, 7 workers (steady state)** | **2.54 s/eval — 1,418/hour** |

**The planned budget was wrong by a factor of about 200.** 300k evaluations is ~210 hours,
not 20 minutes. The estimate assumed something close to a bare policy rollout; an
evaluation is a full mission tick — perception at 5 Hz, the auction, flow-field descent
for every robot. Profiling confirms the cost is real work, not waste: 55% is Tier 1's
`_goal_directions` → `descend`, which is the flow-field lookup the whole navigation design
is built on.

Revised budget, set from the measurement rather than from hope:

| run | evaluations | wall |
|---|---:|---|
| smoke / CI | ~100 | 4 min |
| a working run | ~1,400 | 1 hour |
| overnight | ~14,000 | 10 hours |

The archive is 4 lanes x 10 speed bins = **40 cells**, so even the one-hour run puts ~35
evaluations through each cell and the overnight run ~350. That is a workable MAP-Elites
budget for a 15-dimensional genome; what it is *not* is the 300k the plan assumed, and
the emitter choice matters much more at 1.4k than it would at 300k.

Two things found while building the harness:

- **numpy thread oversubscription.** Seven workers each opening a BLAS thread per core is
  56 threads on 8 cores. Workers now pin to one thread each; the simulator is almost
  entirely elementwise numpy and loses nothing.
- **`spawn` re-executes `__main__`.** Benchmarking from a heredoc (no importable
  `__main__`) meant every worker re-ran the whole benchmark and spawned its own workers.
  That is a fork bomb, and it presents as a hang — the first attempt wrote 76 MB of
  tracebacks before being killed. `pool._guard()` now refuses with an explanation, and
  `tests/test_evaluate.py` asserts the refusal.

## M-31 — first MAP-Elites run, and three flaws it exposed (D9)

3 iterations x 150 evaluations = 450, seven workers, `test`, training seeds (11, 12, 13).
17.2 minutes — consistent with M-30's 2.54 s/eval, so the throughput model holds.

| iteration | cells filled | coverage | obj_max | obj_mean | QD score |
|---:|---:|---:|---:|---:|---:|
| 1 | 15/40 | 37.5% | 16.000 | 3.310 | 49.6 |
| 2 | 15/40 | 37.5% | 16.000 | 3.361 | 50.4 |
| 3 | 16/40 | 40.0% | 16.000 | 3.348 | 53.6 |

The loop works end to end — `ask` → parallel evaluate → `tell` → checkpoint → `select.py`
→ a 32-robot roster — but **the numbers say the search is barely moving**, and three
distinct causes were behind that. All three are now fixed; this row is kept because the
symptoms are what a healthy-looking-but-dead QD run looks like.

**1. The relay fitness had zero discrimination.** Every antenna elite scored *exactly*
16.000. The measure was "mean robots in comms", and on a 16-robot map everybody is within
base range anyway, so the score was the swarm size regardless of what the relay did. The
lane was learning nothing while appearing to be at a maximum.

Fixed with a measure that is causal rather than a proxy, and free: the simulator already
separates `near_base` from `near_relay`, so `comms_via_relay = ~near_base & (near_relay |
connected_relays)` is exactly the set that would drop out if the relays were removed. This
had been written off as needing a counterfactual episode at double the evaluation cost —
it did not.

**2. Fitness scales differed by ~20x across lanes**, so `obj_max` and `qd_score` were the
antenna lane with noise on top and said nothing about the other three:

| lane | elites | raw fitness range | measured speed |
|---|---:|---|---|
| none | 3 | 0.733 .. 0.855 | 0.57–0.93 m/s |
| scoop | 6 | 2.000 .. 3.333 | 0.54–1.30 |
| gripper | 5 | 0.000 .. 1.042 | 0.39–1.13 |
| antenna | 2 | 16.000 .. 16.000 | 0.36–0.36 |

`LANE_REFERENCE` now divides each lane by a documented reference. It is a monotone
transform, so selection *within* a lane is untouched; what it fixes is cross-lane
reporting.

**3. Cold start.** All five emitters begin from the same midpoint, so the first
generations sample one small neighbourhood — hence 15 cells at iteration 1 and 16 by
iteration 3. The archive is now seeded with 150 uniform random genomes first: individually
poor, but spread across the measure space, giving the emitters elites to restart from
instead of circling the origin.

Also fixed alongside: the digger's fitness was `int(cleared) + 2 x int(freed)`, a step
function that leaves nearly every early genome tied at zero. It is now continuous in
debris removed, so a digger that got three casualties to 90% outranks one that touched
nothing — at a 1.4k-evaluation budget that difference is most of the gradient.

## M-32 — why the search was not searching (D9)

M-31's fixes made the archive *readable* (normalised fitness, a causal relay measure) and
random seeding lifted coverage from 15 cells to 18. But the CMA-ES iterations still
contributed almost nothing: 18 → 18 → 18 cells, QD 9.2 → 9.2 → 9.6, best fitness identical
to three decimal places. Three further causes, all measured.

### 1. Half the archive was barely being searched

All five emitters started from the all-0.5 midpoint. `actuator_g` = 0.5 decodes to
`gripper`, so with σ₀ 0.15 the entire search neighbourhood sat in one or two lanes.
Counted directly on a fresh `ask()`:

| | scout | scoop | gripper | antenna |
|---|---:|---:|---:|---:|
| 5 emitters from the midpoint | 9 | 59 | 71 | 11 |
| **one emitter per lane, at its own midpoint** | **19** | **11** | **16** | **14** |

### 2. The budget bought 9 generations, not tens

Batch 30 is 2.5x the CMA-ES default population for a 15-dimensional genome
(4 + 3·ln 15 ≈ 12). At ~1,400 evaluations/hour that is **9 generations an hour**. CMA-ES
adapts a covariance matrix; nine generations is not a run, it is a warm-up. Batch 15 buys
~23/hour for identical spend.

### 3. Twelve of forty cells were unreachable by construction

The speed axis was 0.0–1.8 m/s, guessed. Across **52 elites from three runs** the observed
span is 0.24–1.32: the bottom bin and the top two were never once reached.

| bin | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|---|
| elites | – | 3 | 11 | 9 | 9 | 9 | 8 | 3 | – | – |

Coverage could never exceed 28/40 however good the search was, so "45% coverage" was
partly a statement about the axis rather than the search — at 18/28 reachable cells it was
64%. Recalibrated to 0.20–1.45, which also makes the seven usable bins finer.
`tests/test_mapelites.py` now guards the axis in **both** directions; too wide is what let
dead cells masquerade as poor coverage.

### Result — same 630-evaluation budget, per-lane emitters at batch 15

| | cells | coverage | best fitness | QD score |
|---|---:|---:|---:|---:|
| 5 emitters, batch 30 | 18 → 18 | 45% → 45% | 1.102 → 1.102 | 9.2 → 9.6 |
| **4 emitters (one per lane), batch 15** | **18 → 20** | **45% → 50%** | **1.102 → 1.434** | **9.1 → 11.5** |

QD +25%, best fitness +30%, and — the part that matters — the trajectory is *rising* at
the end of the run instead of flat from iteration one. The speed-axis recalibration is not
in these numbers; it landed after this run and should lift reachable coverage again.

## M-33 — the relay lane was dead, and it was capping the whole mission (D9)

Found by watching a live run in Godot: a dense knot of relays sitting at spawn, and an
explored region that stopped growing at a fixed radius while scouts kept walking past it.

### The bug

`TaskGenerator.relay_posts` added **every living relay's current position** to the anchor
list, and a candidate post is rejected within `0.75 x relay_radius` (28.5 m) of any
anchor. With 192 relays spawned around base -- **128 of them inside the 40 m base
radius** -- they blanketed exactly the ground a chain's *first* link needs.

    frontier targets: 48        relay posts generated: 0
    relays holding a post: 14 / 192      median relay displacement over 420 s: 9 m

No chain could start, so no relay was given a post, so every relay stayed at spawn
continuing to be an anchor. Self-blocking, and stable.

### Why it reached far beyond the relay lane

`World.mark_seen` writes to the shared explored map **only for robots in comms**;
out-of-comms robots reveal into a private buffer that flushes on reconnect. With the
relay network dead, comms never extended past base radius, so:

* scouts walked beyond the comms edge and revealed **nothing anyone could see**
* the lit region stopped at a fixed radius -- the visible symptom
* casualties beyond that radius were never found, so never rescued

Three separate observations, one cause. Anchors now mean *committed topology* -- base plus
posts a relay has actually been assigned -- and nothing else.

### Lane sizes were also wrong, and are now measured

An even 192/192/192/192 split was never right: the network saturates near **44 posts**
however many antennas exist. Relay count swept against the thing it controls:

| relays | scouts | comms | explored | found | rescued | posts held |
|---:|---:|---:|---:|---:|---:|---:|
| 48 | 400 | 76% | 30.0% | 22 | 10 | 38 |
| **96** | **352** | **83%** | **35.5%** | **29** | **11** | **44** |
| 144 | 304 | 86% | 31.8% | 24 | 10 | 42 |

Too few and the network cannot reach; too many and they are taken from the scouts doing
the reaching. Shipped split: **352 scouts / 176 diggers / 144 carriers / 96 relays**.
Chain caps raised from 10x4 to 24x8 so the relay *count* is the binding constraint rather
than an arbitrary 40.

### Honest reading

Relay deployment went from 14/192 to 44/96 and median displacement from 9 m to 24 m. The
lane is alive. But **the headline numbers are at parity with before** -- 35.5% explored,
11 rescued, identical to the even split -- because the 178 idle relays had been providing
incidental comms coverage simply by standing near base. The fix removed a real bug and
freed 96 robots to do useful work; it did not fix the rescue rate. **That problem is
separate and still open.**

Two test flaws surfaced alongside, both of which had been passing for the wrong reason:

* `test_world` asserted all four lane counts were equal -- an assumption, not an invariant.
* `test_genes_change_behaviour` used `hazard_aversion`, which multiplies *observed* hazard
  and is therefore inert before the swarm sees the fire. It had been passing on incidental
  trajectory drift. It now uses `distance_penalty`, which applies to every bid
  unconditionally, and hazard aversion gets its own test with hazard planted **as a
  region** -- planting it uniformly adds a constant to every bid and changes no ranking.

## M-34 — the rescue rate: three caps, only one of which was the one I expected (D9)

The system gate wants **≥75% of casualties rescued**. Measured: 11/120 (9%). Chasing that
number turned up three independent constraints stacked in series.

### The frontier hypothesis was wrong

The standing guess was that `max_explore_targets = 48` starved 352 scouts of work.
Measured at t=60 s: **332 of 352 scouts held an explore task; 2 were idle**, against ~88
open explore tasks per cycle. Supply was never the constraint. Worth a day not spent.

### Cap 1 — Tier 1 was stopping scouts half the time

Scouts were being sent to targets 255–293 m from base, and the distance *to* their target
**grew** through the mission (197 m → 269 m). They averaged 0.53 m/s against a `v_max` of
1.99. Attributing every zero-speed tick:

| cause | share of scout-ticks holding a goal |
|---|---:|
| desired vector cancels to zero | 0.0% |
| "arrived" at goal | 0.0% |
| **wall / terrain override** | **52.5%** |

Nothing was stranded, so it was not terrain the robots were standing on — it was terrain
they kept driving into. `_obstacle_repulsion` scales by *how surrounded* a robot is, so
one heading straight at a single wall trips one or two probes of eight and gets a push of
~0.2. Against `w_obstacle` 2.2 that is 0.44, while the goal contributes a full 1.0. The
goal wins, the scout arrives nose-first at ground it cannot cross, the safety floor
correctly zeroes it, and it rotates on the spot for a dozen ticks.

That scaling was itself a deliberate fix (a robot grazing one wall should not flee as hard
as one boxed in on seven sides), so this is a real trade rather than an oversight. Resolved
by treating the forward arc as urgent: when anything within ±45° of heading is blocked,
repulsion is lifted to a magnitude that can outvote the goal, keeping the direction the
full probe ring already computed. **Invariant #2 is untouched** — the override is not
weakened, it simply fires less because steering avoids the situation.

| | override rate | scout speed | explored | found | rescued |
|---|---:|---:|---:|---:|---:|
| before | 40.8% | 0.53 m/s | 35.5% | 29 | 11 |
| **after** | **12.2%** | **0.68 m/s** | 35.5% | 31 | **13** |

### Cap 2 — comms gates exploration, and it binds

Explored stayed at 35.5% to three significant figures across that entire change, which is
the signature of a different constraint. `World.mark_seen` writes to the shared map only
for robots **in comms**. Removing the constraint entirely:

| | in comms | explored | found | rescued |
|---|---:|---:|---:|---:|
| as shipped | 87% | 35.5% | 31 | **13** |
| everyone always linked | 100% | **53.6%** | **57** | **12** |

Exploration +51%, discovery +84% — and **one fewer rescue**. Which exposes the third cap.

### Cap 3 — discovery arrives too late to act on

    rescued 13 · carried 7 · cleared-and-waiting 11 · still hidden 89
    found-but-still-buried: 0
    diggers  (153 alive): 137 on explore, 0 digging
    carriers (130 alive):  96 on explore, 16 extracting

Digging is not a bottleneck and the specialist lanes are not starved — **89 of 120
casualties are simply never found**, so there is no rescue work and the lanes explore
instead. `mean_time_to_rescue` is ~220 s against a 420 s mission, so a casualty discovered
after ~200 s cannot be delivered at all. That is why finding 84% more rescued none of
them: the extra discoveries came too late to carry home.

**So the rescue rate is bounded by map size against mission duration and swarm speed, not
by any single component.** CLAUDE.md already says robot count and mission size are coupled;
this is that rule being collected on. The map was scaled 2.31x at D6 and the mission
duration was not.

## M-35 — the rescue rate is capped by *early* discovery, and search is ~4% efficient (D9)

Rescued sits at **13** through every intervention tried:

| changed | result | rescued |
|---|---|---:|
| map area x0.29 .. x1.0 | found 31 -> 62 | 13, 13, 13, 12 |
| comms constraint removed entirely | explored 35.5% -> 53.6% | 13 -> 12 |
| Tier 1 override 40.8% -> 12.2% | scout speed +28% | 11 -> 13 |
| orphan re-offer 2 s -> 30 s | orphan events 624 -> 349 | 13 -> 13 |
| lane split even -> 352/176/144/96 | relays deploy 14/192 -> 44/96 | 11 -> 13 |

A number that flat under that much variation is not a scaling limit. The cause:

    casualties found by t=150s: 11        by t=250s: 21        total: 29
    rescued: 13      mean time to rescue: 273 s      mission: 420 s

**Only casualties discovered in roughly the first 150 s can be delivered at all**, and that
early count is ~13 whatever the map looks like. Everything else in the mission finds
casualties that cannot be carried home before the clock runs out. Shrinking the map raises
*total* discovery without raising *early* discovery, because early exploration is bounded
by how far the swarm has spread, not by how much map exists.

### Search efficiency

352 scouts spent 147,840 scout-seconds to explore **37,276 unique passable cells** of
105,180 — 0.25 new cells per scout-second. A scout moving at its measured 0.68 m/s with a
~10 m effective swath would clear ~6.8 cells/s if it never re-covered ground, so the swarm
is running at roughly **4% search efficiency**: about 96% of scouting effort re-treads
ground another scout has already seen.

(An earlier version of this measurement reported "13,008x redundancy". That figure counted
camera *pixels* rather than cells and is meaningless; it is recorded here only so the
number is not quoted from the run log later.)

### What this implies

The frontier auction has no notion of *who is already going where*. Every scout bids on
the globally-best frontier target independently, so they converge on the same ground and
fan out only as a side effect of collision avoidance. No component in the current
architecture owns spatial allocation of the search -- Tier 2 allocates *tasks*, not
*territory*.

That is the gap Tier 3 should be filling, and currently is not: sector priority nudges a
rank by +-0.5, which cannot partition a swarm.

## M-36 — a fix that measured as a regression, and what it says about comms (D9)

M-35 found the swarm covering new ground at ~4% of its theoretical rate. The obvious cause
was there in one measurement:

    scouts holding an explore task : 629
    distinct targets among them    : 44
    worst-shared target            : 33 robots walking to one spot

`TaskGenerator.generate` deduplicates casualty tasks (`claimed_v`) and report tasks
(`claimed_r`) against live assignments, and never deduplicated **search** targets, so the
same frontier was re-announced and re-awarded every cycle while its previous holders were
still walking there. Claiming a target within a 10 m scouting swath turned 629-on-44 into
**449 scouts on 436 distinct targets**.

It made the mission worse.

| configuration | explored | found | rescued |
|---|---:|---:|---:|
| unchanged | 35.4% | 29 | **13** |
| claim + factor-3 frontier + 3x3 sector waypoints | 37.0% | 24 | **5** |
| claim + factor-5 frontier + one sweep point | 32.9% | 24 | **6** |
| **fully reverted** | 38.9% | 28 | **10** |

**Exploration and comms pull against each other, and comms wins.** `World.mark_seen`
records only what an *in-contact* robot sees. Spreading the swarm across hundreds of
scattered targets pushes scouts past the comms envelope, so the extra ground they cover
never registers, while the concentration that had been feeding the rescue chain is gone.
Overlap is the price of staying in contact on this map.

Reverted in full, with the reasoning written into `tasks.py` at the point where the next
person will look -- 33 scouts converging on one target looks exactly like a bug, and the
next reader deserves to know it was tried.

**The real target is comms coverage.** Until an in-contact robot can be most of the way
across the map, every search improvement dies the same way.

### A baseline that was not a baseline

The reverted run gives 10 rescues, not the 13 it started at. The 13 predates the **rotor
chassis**, added the same afternoon: rotors are now a quarter of every lane except
carriers, and they are blind in flight, so a quarter of the scouts no longer reveal fog in
transit. Exploration is *up* (38.9% vs 35.4%) and rescues are down, which is consistent
with that and not with the reverted change. Quantifying the rotor's cost is a D10 job; the
lesson recorded here is that "restore the baseline" is meaningless when three other things
moved in between.

### The commander is off by default

`nodes/command.py` measures at **8 rescues against 10 without it**. Sixteen wedges cannot
separate 352 scouts, and fencing relay posts into wedges confines the comms chain that
everything else depends on. It is `command=False` until training gives it parameters that
beat no commander at all -- the same discipline the gate applies to every other learned
component.

## M-37 — store-and-forward, and D10 complete (D10)

### Comms was the thing upstream of everything

M-36 concluded that every search improvement dies against comms coverage, because
`World.mark_seen` records only what an **in-contact** robot sees. Buffered discoveries
existed but flushed only on *reconnect*, and on a 480x320 m map most scouts that leave the
envelope never return before the mission ends.

`_store_and_forward` lets an out-of-contact robot hand its buffer to any in-contact robot
it passes within relay range. Delay-tolerant networking: data walks home on whatever is
going that way. It deliberately does **not** rejoin the carrier to the live comms
component -- it still cannot be given orders or bid -- so the relay lane keeps its job.

| | explored | found | rescued |
|---|---:|---:|---:|
| before | 38.9% | 28 | 10 |
| **with store-and-forward** | **39.9%** | **33** | **12** |

+5 casualties found and +2 rescued from one mechanism, and it is the first change all day
that moved discovery without costing concentration.

### Commander training is launchable

    make train-command        # 10 h default, checkpoints every generation, safe to stop
    make gate-command         # scores it on 10 held-out maps it never trained on

Training cost, measured and then tuned down: one evaluation was 45 s at 180 s episodes and
240 robots; at 150 s and 160 robots it is **21 s**. A 9-candidate generation over 3 maps on
seven workers is ~2 minutes, so an overnight run is **~250 generations** rather than the
~60 the first sizing implied. CMA-ES's default population for 7 parameters is 9, so the
batch is already right.

Smoke run, 2 generations on one map: baseline 82.61, best 92.80 -- **1.12x within the
first generation**, which says the hand-set weights were not near a local optimum.

**The gate has three arms, not two.** none / heuristic / trained. The middle one is the
one people forget and the one that matters here: the hand-set commander currently scores
*worse* than no commander at all, so "trained beats heuristic" would be a meaningless
victory. Trained has to clear **none** by the same 1.05x margin every other component
faces.

### Resume, added before the first real run

The first version checkpointed `best.npz` every generation and nothing else. That is not
resumable: CMA-ES carries a covariance matrix, a step size and a distribution mean, and a
restart would have begun again from the hand-set baseline **and overwritten a good result
with the fresh run's worse one**. The whole scheduler now pickles (6.6 KB) into
`state.pkl`, resume is the default, `--restart` is explicit, and `best` is only ever
replaced by something strictly better.

Verified rather than assumed -- 2 generations, stop, restart:

    run A:  gen 1 best 51.86   gen 2 best 51.86  (gen-best 36.05, rejected)
    run B:  resumed at generation 2, best 51.86
            gen 3 best 51.86  (gen-best 46.17, rejected)
            gen 4 best 54.04  <- improved

## M-38 — a second training machine, and the flow field recomputed 20x too often (D10)

A Windows box (Ryzen 9 7940H, 8 cores / 16 threads, 39 GB, Radeon 780M) was set up as a
second training machine. Two things came out of it: a baseline that did not say what was
expected, and a profile that found the hot loop doing static work per tick.

### The machine is not faster per core, and SMT barely helps

| | ms/eval | evals/hour |
|---|---:|---:|
| M1, 7 workers (M-30, D9) | 2,540 | 1,418 |
| Ryzen, 7 workers | 3,007 | 1,197 |
| Ryzen, 15 workers | 2,758 | 1,305 |

Doubling the worker count bought **+9%**, not +100%: the evaluation is bound by Python
dispatch and memory latency, so hyperthreads contend for the same ports rather than
scaling. Before optimisation the 16-thread Ryzen was *slower at MAP-Elites than the
8-core M1*. Note M-30 was taken at D9 and the code has since gained a rotor chassis and
the commander, so the M1 row needs re-measuring at this commit before the comparison is
strictly fair.

GPU work cannot move here: the 780M is not CUDA and ROCm does not cover gfx1103 on
Windows. Kaggle remains the only GPU.

### Profile: half the episode is `_goal_directions`, and most of that is call overhead

One 60 s `test` episode, 16 robots, 1200 ticks, cProfile:

| site | cumulative | calls |
|---|---:|---:|
| `_goal_directions` | **50.3%** | 1,200 |
| `planner.descend` | 2.72 s | **15,984** |
| `planner.field` | 1.85 s | **17,629** |
| `np.clip` | 1.84 s | **162,390** |
| `np.finfo.__init__` | 0.27 s | **307,798** |

13 `descend` calls per tick at 16 robots means **each call processed about one robot** --
full numpy dispatch cost, no vectorisation benefit. The scaling rules in CLAUDE.md forbid
iterating *robots* in the tick loop; this was iterating goals x chassis, which is the same
mistake wearing a different hat.

Three fixes, all exactly result-preserving:

1. **Goal coordinates memoised to a cache key.** Goals are reissued at 1 Hz (`auction_hz`)
   but Tier 1 re-resolved them at 20 Hz, so `to_coarse` + `_nearest_passable` ran twenty
   times more often than the answer changed.
2. **Descent direction precomputed per goal.** `passable` is static and the fields never
   invalidate, so the steepest-descent step at a cell is static too. Computed once per
   goal into an int8 grid (a quarter the size of the float32 field beside it, and built
   only for goals actually navigated to), the per-tick job becomes one gather.
3. **`ndarray.clip` instead of `np.clip`.** The module function detours through
   `_wrapfunc` -> `_methods._clip` and builds two `np.finfo` objects per call; on
   twelve-element arrays that detour is the whole cost.

### Result

| | before | after | |
|---|---:|---:|---|
| headless demo, seed 42 (768 robots) | 668.9 s | **401.7 s** | **-40%** |
| demo RTF | 0.63x | **1.05x** | now faster than real time |
| MAP-Elites, 15 workers | 1,305/hr | **1,566/hr** | +20% |
| MAP-Elites, 7 workers | 1,197/hr | **1,251/hr** | +4.5% |

**Seed 42 still hashes `291ed26fcda6a0ca`,** with every scorecard field unchanged, which
is the acceptance test for a change of this kind -- the direction table is built by the
same expression `descend` used, so it returns bit-identical vectors rather than merely
close ones. `tests/test_planner.py` asserts the equality with `assert_array_equal`, and
`descend` is kept as the reference definition.

The demo gains more than the fixture because more robots spread over more distinct goals
means more (goal, chassis) groups per tick -- exactly the calls that were hoisted.

### Also found

- **`Path.read_text()` defaults to the locale encoding, not UTF-8.** On a GBK (cp936)
  Windows locale, three source-scanning tests died with `UnicodeDecodeError` -- including
  `test_no_ground_truth_leak.py`, the enforcement of invariant #4. A guard that cannot
  read the source it guards still looks like it passed. All call sites now pass
  `encoding="utf-8"` explicitly.
- **The MAP-Elites sizing comments are stale.** `run.py` says "9 iterations x 150 = 1,350
  evaluations ~ one hour", but `N_EMITTERS`(4) x `BATCH_SIZE`(15) = **60** per iteration,
  so the default run is 690 evaluations, ~30 min. The numbers changed when emitters went
  5 -> 4 and batch 30 -> 15; the prose did not follow.
- **`--workers` defaults to `cpu_count() - 1` = 15 here**, which assumes `cpu_count()`
  counts physical cores. It does not on an SMT machine. Not harmful, just much less than
  it looks -- see the +9% above.
- **The full test suite is 11m14s on this machine**, not the "seconds" CLAUDE.md claims.
  Unsplit between platform and the suite having grown at D9/D10.

## M-39 — the rescue ceiling is 25%, and the relay chain cannot outrun the frontier (D11)

`scripts/diagnose.py`, demo, seed 42, 768 robots, 120 casualties, 420 s, evolved roster.
This re-takes M-34/35/36, whose numbers predate the rotor chassis, store-and-forward
(M-37), `posts_per_chain` 1 -> 8, and the MAP-Elites roster.

| t | expl% | found | resc | alive | comms% | reach_m | relays on post |
|---:|---:|---:|---:|---:|---:|---:|---|
| 30 | 7.2 | 5 | 0 | 768 | 95.6 | 118 | 9/96 |
| 60 | 11.8 | 6 | 1 | 768 | 89.6 | 158 | 22/96 |
| 120 | 20.0 | 19 | 6 | 768 | 86.3 | 216 | 32/96 |
| 180 | 26.2 | 23 | 11 | 768 | 89.7 | 231 | 37/96 |
| 240 | 29.7 | 24 | 14 | 768 | 86.5 | 234 | 39/96 |
| 300 | 31.8 | 25 | 15 | 758 | 84.7 | 239 | 37/95 |
| 360 | 36.9 | 29 | 15 | 700 | 81.4 | 241 | 45/93 |
| **420** | **37.3** | **30** | **19** | **666** | 82.6 | **231** | **46/92** |

    rescued 19/120   found 30/120   explored 37.3%   lost 102/768   mttr 194 s

### The ceiling, stated as arithmetic

**Only 30 of 120 casualties are ever found.** Rescuing *everything discovered* is
25.0% — so the 75% gate is not a stretch target on this scenario, it is arithmetically
out of reach by 3x. **The rescue chain is not the constraint.** Digging, carrying and
allocation are working on a supply of casualties that barely exists; M-34 measured the
same thing from the other side (`diggers: 137 on explore, 0 digging`).

Any work aimed at extraction, allocation or Tier 3 is aimed at the wrong link.

### The mechanism: the relay chain cannot get ahead of the frontier

Three numbers plateau together, and they plateau at the same time (~t=180):

| | plateau | of |
|---|---:|---|
| comms reach | **~240 m** | 577 m map diagonal (42%) |
| explored | **37.3%** | the map |
| relays on post | **~46** | 92 alive (50%) |

Half the relay lane never takes a post, and the comms component stops growing at 240 m.
`TaskGenerator.relay_posts` builds each chain as a straight radial line from base toward a
**frontier target**, walking outward in `relay_radius * 0.8` = 30.4 m steps and rejecting
any candidate within `relay_radius * 0.75` = 28.5 m of an existing anchor.

**That couples the chain to the frontier, and the frontier to the chain.** The frontier
lies at the edge of *explored* ground; explored ground ends at the comms boundary, because
`World.mark_seen` only writes to the shared map for in-contact robots. So the chain is only
ever offered posts up to roughly where the swarm has already been — it cannot be sent
ahead of the explored region to open new ground. Each link waits for exploration that
waits for that link.

Consistent with the cap: `posts_per_chain` (8) x `step` (30.4 m) = **243 m**, and the
observed maximum reach is 247 m.

**This is the thing to fix, and it is a mechanism change, not a parameter.** Relay posts
need to be placed toward *unexplored* territory beyond the frontier — extending the
network ahead of the swarm rather than trailing it. Raising `posts_per_chain` alone does
not break the coupling; it only lengthens a chain that still has nowhere to be sent.

### Hazard is the dominant cause of loss, and nobody had attributed it

    deaths by cause: hazard 84, battery 17, structural collapse 1   (102 of 768)

**82% of all losses are the spreading hazard**, not battery and not the scripted fault.
Losses are concentrated late — 768 alive at t=240, 666 by t=420 — as the fire reaches
`r ~= 105 m` (`growth_rate: 0.30`). The swarm loses 13% of its capacity in the last third
of the mission, exactly when late discoveries would need carrying home. Whether the
hazard reflex (`w_hazard`, `probe_hazard`) is under-weighted or the growth rate is simply
too aggressive for a 420 s mission is not yet separated.

### The evolved roster is worth ~+90% rescues

Same seed, same code, with and without `assets/scenarios/demo_roster.yaml`:

| | rescued | found | explored | lost |
|---|---:|---:|---:|---:|
| hand-set archetypes | 10 | 32 | 42.3% | 142 |
| **evolved roster (MAP-Elites run 0)** | **19** | 30 | 37.3% | **102** |

**Rescues +90%, losses -28%, exploration -12%.** The evolved bodies trade some coverage
for survivability and delivery, which is what the fitness asked for. This is the clearest
evidence so far that the MAP-Elites archive is worth shipping, and it should be re-measured
across seeds before the number is quoted to anyone.

### A double-emit bug found while instrumenting

`FaultInjector._fire` emits `robot_destroyed` and *then* calls `world._kill`, which emits a
second `robot_destroyed` for the same robot. One death, two events -- the dashboard
timeline shows that robot dying twice. `scripts/diagnose.py` de-duplicates by robot id;
the source has not been fixed.

## M-40 — the relay chain builds ahead of the frontier, and the ceiling barely moves (D12)

M-39 named the mechanism; this is the fix, measured either side of it on four seeds and
both Tier-3 arms. Read M-39 first — the chain of constraint it establishes,
`rescued <- discovery <- explored <- comms reach`, is why reach is the number that has to
move first and rescues the number that moves last.

### The baseline had to be re-taken, because M-39 does not reproduce on this machine

No sim code changed between M-38 and here (`f07313a` touched `training/command/`, docs and
`scripts/` only), yet this laptop measures seed 42 at **18 / 29 / 36.9% / 106** where M-39
records 19 / 30 / 37.3% / 102, and the headless seed-42 scorecard hashes
`eb39e6a4ba22080b` where M-38 records `291ed26fcda6a0ca`. M-38 is exactly where the second
(Windows / Ryzen) machine enters the project, so this is platform float rather than a
regression: both runs here are byte-stable **across processes** (seed 42 hashed twice,
identical), which is all invariant #6 claims.

**The consequence is a working rule.** A before/after pair has to be taken on one machine.
A number recorded on the training box cannot be the control for a change measured on the
demo box, and a ±1 rescue platform delta is the same size as the effects being chased.

### Before and after, four seeds, both Tier-3 arms

`uv run python scripts/baseline_sweep.py`, demo scenario, 768 robots, 120 casualties,
420 s, evolved roster. One run per cell — determinism is per-process and verified, so a
repeat would be the same bytes, not a second sample.

| seed | arm | rescued | found | explored | lost | reach m | posts |
|---:|---|---:|---:|---:|---:|---:|---|
| 42 | off | 18 → **21** | 29 → **36** | 36.9 → **43.6** | 106 → 119 | 233 → **261** | 43/90 → 79/83 |
| 43 | off | 18 → **21** | 32 → 32 | 39.7 → 39.4 | 14 → 25 | 235 → **273** | 49/96 → 89/95 |
| 44 | off | 14 → **16** | 27 → 26 | 25.0 → **26.3** | 5 → 5 | 276 → **302** | 32/96 → 81/96 |
| 45 | off | 15 → **14** | 26 → 25 | 27.4 → **28.5** | 29 → 37 | 175 → **273** | 35/96 → 68/90 |
| **mean** | **off** | **16.25 → 18.00** | 28.50 → 29.75 | 32.26 → 34.43 | 38.5 → 46.5 | **230 → 277** | |
| **mean** | **on** | **15.00 → 17.50** | 26.00 → 29.50 | 31.65 → 33.84 | 50.8 → 54.0 | 235 → 255 | |

**Rescues +10.8% with Tier 3 silent, +16.7% with it on. Comms reach +20%, and it is up on
all four seeds in the control arm** — the mechanism metric moved, which is the thing M-39
said had to move first.

### The change, in three parts

[`nodes/tasks.py:relay_posts`](../swarmmind/nodes/tasks.py). Only the first is the fix;
the other two exist to stop the first from being a regression.

1. **The chain endpoint is the frontier target plus `lookahead_steps` links of open ground
   beyond it.** This is the M-39 deadlock: a frontier is the edge of explored ground,
   explored ground ends at the comms boundary, and the chain was being aimed at the
   boundary it exists to move.
2. **Posts are capped by the relay lane's free supply.** A lengthening chain announced in
   24 directions leaves a hole in every one of them, and a chain with a hole carries
   nothing. It also bounds compute: every announced target costs a BFS distance field
   (22.4 ms, M-4).
3. **An impassable or known-hazard candidate ends the chain instead of being skipped.**
   Every post past a hole is a relay standing in a network with no path back to base.

### Which part did the work — `lookahead_steps=0` as the control

Seeds 42 and 43, Tier 3 off. `0` keeps parts 2 and 3 and restores the old endpoint:

| lookahead | rescued | found | explored | lost | reach m |
|---:|---:|---:|---:|---:|---:|
| *old code* | 18.0 | 30.5 | 38.3% | 60 | 234 |
| 0 | 16.5 | 30.5 | 38.0% | 64 | 232 |
| 3 | 19.0 | 34.0 | 41.6% | 70 | 244 |
| 6 | 19.0 | 33.0 | 39.9% | 72 | 212 |
| 12 | **21.0** | 34.0 | 41.5% | 72 | 267 |

**Parts 2 and 3 on their own measure as nothing (or slightly negative, inside noise at
n=2). The extension is the whole effect.** Worth stating plainly, because both guards are
the kind of change that feels productive and would have been easy to bank as progress.

### The value saturates, so it is not a tuned number

Four seeds, Tier 3 off:

| lookahead | rescued | found | explored | lost | reach m |
|---:|---:|---:|---:|---:|---:|
| 12 | 18.0 | 29.8 | 34.5% | 46 | 277 |
| 24 | 17.0 | 31.2 | 34.9% | 50 | 262 |

24 doubles the allowance and measures the same as 12. Past a few links the chain is bound
by **how much of it the relay lane can staff and by the map edge**, not by how far ahead it
is allowed to look. `0` is clearly worse than any positive value and 3–24 are within noise
of each other, so the shipped default of 12 is a saturating choice, not a fitted one. The
things that actually bind are the 96-relay lane size and the 30.4 m post spacing.

### What did **not** move: the ceiling, which is the whole point of D12

Four-seed mean casualties *found* went 28.50 → 29.75 of 120. **The discovery ceiling is
still ~25%** (seed 42 alone reached 30.0%, and it is the only seed whose discovery moved
meaningfully: 29 → 36). The 75% whole-system gate in PLAN §6 remains out of reach by ~3x,
and the fix does not change that conclusion — it improves the rescue rate *under* the
ceiling without lifting it.

**PLAN §7.7 step 3 is therefore now the open item**, on the terms §7 always allowed: revise
the target in writing with the measurement behind it.

### Reach is a looser proxy than M-39 assumed

Reach +20% bought explored +2.2 points and found +4.4%. Reach is a *max* distance — one
robot at the end of one long chain — so it rewards depth and says nothing about breadth.
The chains got longer without the comms component getting much wider, and coverage follows
area, not radius. **The next measurement of this mechanism should be the fraction of
passable map inside the comms component**, not the farthest robot in it.

### Two findings worth carrying forward

- **The lane fully commits within 30 s and cannot be re-aimed.** At `lookahead_steps=12`
  the seed-42 trace shows `96/96` relays on post by t=30 (against `9/96` before D12). The
  network is therefore built toward wherever the frontier pointed in the first seconds of
  the mission, and a relay holding a post never re-bids as the frontier moves. Whether
  re-taskable posts beat statically committed ones is unmeasured and is the obvious next
  experiment.
- **Losses rise 21%** (38.5 → 46.5 mean, control arm; hazard deaths 85 → 101 on seed 42).
  Longer chains put more of the lane in the fire's path. Consistent with M-39's finding
  that hazard is 82% of all losses, and still not separated from `growth_rate: 0.30`
  simply being too aggressive for a 420 s mission.

### Tier 3, measured four ways for the first time

M-26 (D7) had the scripted rung ahead on seed 42 and honestly labelled it "promising, not
demonstrated". On four seeds before this change it was **behind on rescues (15.00 vs
16.25), behind on discovery on 4 of 4 seeds, and losing more robots on 4 of 4**. After the
change the arms are level (17.50 vs 18.00). It is still one scripted rung, four seeds, and
not separable from noise — but "Tier 3 helps" is not currently supported by any measurement
in this file, and the gate will have to settle it.

### Hashes

Seed 42, headless (Tier 3 on): `eb39e6a4ba22080b` → **`0c59302b5728021b`**. Control arm:
`b2738c3e276ee842` → `f9d1e3088eaac1d1`. Expected — this changes what the swarm does.
Every cell is in `runs/sweep_{before,after}.json` — which `runs/` being gitignored
means is local and `make clean` deletes, so the tables above are the record and
`scripts/baseline_sweep.py --label before` regenerates the files.

## M-41 — the fire burns out: losses halve, rescues unchanged (D12)

The hazard only ever grew: `r(t) = 6.0 + 0.30·(t−90)`, reaching 105 m by mission end.
Two consequences, one measured long ago and one found by inspecting a live demo run:

- **82% of every robot lost in a run was lost to the hazard** (M-39), concentrated in the
  last third — exactly when late discoveries need carrying home.
- **Ground the fire reached was gone for the rest of the mission.** `tasks.py` drops any
  task whose target is in known hazard, extract included, so a casualty the fire caught
  was written off in silence. At t=420 on seed 42, **3 of the 7 cleared-but-uncollected
  casualties were sitting inside the fire with no task generated for them.**

A fire that only grows is a countdown, not a hazard. Real ones exhaust their fuel, so this
one now does too: `peak_t: 270`, `decay_rate: 0.25` → grows 6 → 60 m over 90–270 s, recedes
to 22 m by 420 s. The growth phase is unchanged, which the t=170 control below confirms.

### Four seeds, Tier 3 off, against the D12 baseline (M-40)

| | always-growing | burns out |
|---|---:|---:|
| rescued | 18.00 | 17.50 |
| found | 29.75 | 29.75 |
| explored | 34.43% | **35.35%** |
| **robots lost** | **46.5** | **23.25** |
| delivery (rescued ÷ found) | 60.5% | 58.8% |
| comms reach | 277 m | 267 m |

Per-seed losses: 119 → 46, 25 → 26, 5 → 5, 37 → 16.

**Losses halve and the mission outcome does not move.** Rescued −0.5 and delivery −1.7
points are both inside the seed-to-seed spread; discovery is identical to two decimal
places. Gate (PLAN §6.1): delivery 58.8% ≥ 55% and discovery 34 ≥ 30 on seed 42, **PASS**.

**The interesting part is what that implies.** Twenty-three robots per run were dying
without changing the outcome — so they were not contributing to rescues when they burned.
That is consistent with M-35: discovery is bounded by how far the swarm spreads in the
first ~150 s, and hazard losses land late, after the casualties that can still be
delivered have already been found. The fire was expensive and, in outcome terms, inert.

### It gives the stranded casualties back

Seed 42 at t=420, casualty states:

| | always-growing | burns out |
|---|---:|---:|
| cleared, waiting for a carrier | 7 | **4** |
| being carried at the buzzer | 7 | **10** |
| of the waiting, stranded in fire | **3** | **0** |

Hazard radius at t=420 falls 105 m → 22 m, and no casualty is inside it. The four still
waiting are a *different* bug — each is claimed by a carrier 100–250 m away while a free
one stands 17–40 m off (see the open items below).

### What was considered and rejected

- **Place the fire where there are no casualties**, or **place casualties away from the
  fire.** Both gut the beat they exist for: the hivemind's `abandon` directive is only
  interesting because something is at risk, and a fire that threatens nothing makes the
  reprioritisation theatre. The hazard also drifts at 0.50 m/s — up to 165 m over a run —
  so a placement-time keepout would have to cover most of the map to hold.
- **A fifth "extinguisher" lane.** The best of the ideas and the wrong week for it: a new
  actuator lane changes `contracts/` (frozen), the skill executor, the dashboard,
  and the MAP-Elites archive from 4 lanes to 5 — and it takes robots from the scout lane,
  which is the binding constraint on discovery (M-35, M-40). Adding extinguishers means
  finding fewer casualties to rescue.
- **Exempting `extract` from the hazard task filter**, so a carrier could price the risk
  through its bid instead of being forbidden. Rejected because it fights an existing
  mechanism: `hazard_preemptions` pulls *any* robot standing in known hazard onto a
  `retreat` task, so a carrier sent in would be yanked straight back out and oscillate.
  Making that work means teaching the preemption about carriers on a rescue, which is a
  larger change than burn-out and buys the same casualties back.

**Determinism.** Seed 42 headless (Tier 3 on) `0c59302b5728021b` → `20da91a22fa76ae3`;
control arm → `bd3fcee6b4e0b2d0`. `test.yaml` keeps `peak_t: None` and is bit-identical,
so the fixture's determinism tests are unaffected. 286 tests green.

### Found while measuring this: the demo path under-reported its own headline feature

`cli.py` printed `m.world.scorecard()` on the `--demo` path. `World.scorecard()` cannot
know about Tier 3 or the wall clock, so the demo reported **"directives 0 issued, 0
rejected"** for a hivemind that was running, with `wall / rtf 0.00s / 0.00x` beside it as
the giveaway. Both paths now go through `Mission.scorecard(wall)`, with a regression test.
This is the M-25 lesson from the other side: that entry was about a count that overstated
Tier 3 by 20x, and this one understated it to zero.

## M-42 — the rescue chain: one casualty, one task, and preemption that competes (D12)

Two bugs found by asking why seven casualties lay cleared and uncollected at t=420 (M-41),
after burn-out had ruled out the fire as the reason for four of them.

### Bug 1 — one casualty offered as three tasks

`tracker.resolved_victims()` returns *reports*, and several reports can resolve onto the
same casualty; corroboration from different angles is what the tracker is for.
`TaskGenerator.generate` emitted one `extract` per report. `claimed_v` did not catch it,
because it dedups against **live assignments** and none of them were assigned yet — the
duplicates are created and awarded inside a single cycle.

Seed 42, t=396: **casualty v6 awarded to three carriers at once, 253 m, 258 m and 258 m
away, while a free carrier stood 4 m from it.** ~750 robot-metres of travel for one rescue
that a robot was already standing on.

### Bug 2 — preemption was a fallback, not a comparison

    i, score = self._best_bidder(world, task, free)
    if i < 0 and task.rank <= self.PREEMPT_RANK:      # <-- only when nobody free could bid
        i, score = self._preempt_for(...)

So a free carrier on the far side of the map always beat a nearer one that happened to be
exploring, because the nearer one was **never evaluated** while any free robot existed at
all. Measured on the same casualty: nearest carrier 4 m (busy), winner 258 m (free).

Preemption now runs alongside the normal round and wins on merit, with a margin:
`PREEMPT_RATIO = 0.5` — a busy robot must be at least twice as quick to get there before
it is worth taking off its work, because the abandoned task goes back on the market and
its progress is lost. Not zero (the old behaviour, in effect) and not one (churn).

This is M-15 from the other side. That entry fixed "the auction cannot preempt at all";
this one fixes "the auction preempts only when it has no alternative". Ranking work is
meaningless if the allocator cannot act on the ranking — twice now.

### Four seeds, Tier 3 off

| | burn-out (M-41) | + these two fixes |
|---|---:|---:|
| rescued | 17.50 | **19.50** |
| found | 29.75 | **30.75** |
| explored | 35.35% | **36.70%** |
| robots lost | 23.25 | 22.75 |
| comms reach | 267 m | **289 m** |
| **delivery** | 58.8% | **63.4%** |

Per-seed rescues 20 → 23, 20 → 21, 16 → 14, 14 → 20: three up, one down. Gate: delivery
63.4% ≥ 55% and discovery 37 ≥ 30 on seed 42, **PASS**, with more headroom than before.

### Where D12 started and where it ended

All four seeds, Tier 3 off, against this morning's re-taken baseline (M-40):

| | D11 baseline | relay lookahead | + fire burns out | + rescue chain |
|---|---:|---:|---:|---:|
| rescued | 16.25 | 18.00 | 17.50 | **19.50** |
| found | 28.50 | 29.75 | 29.75 | **30.75** |
| explored | 32.26% | 34.43% | 35.35% | **36.70%** |
| robots lost | 38.5 | 46.5 | 23.25 | **22.75** |
| delivery | 57.0% | 60.5% | 58.8% | **63.4%** |

**Rescues +20%, delivery +6.4 points, losses −41%, discovery +7.9%** across the day, on
four seeds and with every step measured against the one before it. The discovery ceiling
is still ~26% and still scenario-bound — none of this changes the §6.1 conclusion.

### Tier 3 is now clearly behind, on more evidence than before

| arm | rescued | found | delivery |
|---|---:|---:|---:|
| **off** | **19.50** | **30.75** | **63.4%** |
| on (scripted rung) | 17.00 | 29.50 | 57.6% |

Off beats on on 3 of 4 seeds, and the gap widened as the swarm got better — the control
arm gained 2.00 rescues from these fixes while the Tier-3 arm lost 0.25. M-26 (D7, three
seeds) called the scripted rung "promising, not demonstrated"; on this evidence it is a
net cost, and the hivemind gate has a real question to answer. **No claim about Tier 3
helping should be made to a judge on the current numbers.**

Seed 42 headless (Tier 3 on) hashes `70515ed82a880111`; control arm `57ab6dbf40299f1c`.
288 tests green.

## M-43 — Tier 3 was aiming the swarm past its own radio horizon (D12)

M-42 left the scripted rung measurably *costing* the mission: 17.00 rescued against 19.50
with Tier 3 silent, behind on 3 of 4 seeds, and the gap widening as the swarm improved.
This is why.

### The mechanism

`ScriptedProvider` pushes into the `push_sectors` **least-explored** sectors each cycle.
On a 480 × 320 m map with base in the corner, "least explored" is very nearly "farthest
from base", because exploration radiates outward. Instrumenting a seed-42 run:

| sector | pushes | distance from base | inside comms? |
|---|---:|---:|---|
| A7 | 51 | 374 m | no |
| A3 | 50 | 134 m | yes |
| A8 | 48 | 434 m | no |
| A6 | 13 | 314 m | no |
| A5 | 11 | 254 m | no |

**127 of 208 pushes (61%) aimed at sectors beyond the median comms reach of 239 m.**
`World.mark_seen` writes to the shared map only for in-contact robots, so Tier 3 was
concentrating the swarm on ground where nothing it found could be reported. The swarm
covered it and the blackboard never heard.

This is the same wall as M-33 (relays) and M-36 (search dedup), reached from a third
direction. Every mechanism in this system that spreads the swarm out dies against comms
coverage, and Tier 3 was spreading it out harder than anything else.

### The fix, and what it bought

Push candidates are now filtered to sectors within the current comms reach plus one relay
hop, falling back to the unfiltered order if nothing qualifies — a Tier 3 that goes silent
when the swarm is out of contact would go silent exactly when it is needed. Reach is read
from the swarm's own robots (`pos[in_comms]`), not from ground truth.

Misaimed pushes fell **61% → 27%**, and the pushes spread across the map (D5, B3, C1–C4,
D3) instead of piling into A7/A8.

| Tier 3 **on**, 4 seeds | before | after |
|---|---:|---:|
| rescued | 17.00 | **17.75** |
| found | 29.50 | 29.25 |
| explored | 35.34% | **36.52%** |
| delivery | 57.6% | **60.7%** |

The Tier-3-off arm is byte-identical across both sweeps (`57ab6dbf40299f1c`,
`e9e50b2172738439`, `4b1bc556aa8ad9da`, `8033e2fce2e72667`), which is the acceptance test
for a change that should touch only the hivemind path.

### It is still behind, and that is the honest position

| arm | rescued | found | delivery |
|---|---:|---:|---:|
| **off** | **19.50** | **30.75** | **63.4%** |
| on (scripted rung) | 17.75 | 29.25 | 60.7% |

Per seed, on vs off: 23–23 tie, 17–21 off, 14–14 tie, 17–20 off. Two ties and two losses,
improved from three losses, but **the scripted rung still does not beat no Tier 3 at all.**

What this does *not* say is anything about the tuned model. The scripted rung is rung 4 of
the ladder and the baseline the model has to beat; the model rung has never been measured
on the demo scenario. The D14 hivemind gate is where that gets settled, and it now has a
real question in front of it rather than a formality. **Until then, no claim that the
hivemind improves the mission should be made to a judge.**

### A hypothesis measured and rejected, in the useful direction

Before fixing anything I expected `priority: high` to be nearly inert: it shifts a task's
rank by ∓0.5, and if the auction announces everything anyway, reordering changes little.
Measured over a full run:

    auction cycles 434   open tasks median 85   cap median 52
    cycles where the cap actually bound: 395 (91%)

**The cap binds in 91% of cycles**, so rank decides which ~52 of ~85 tasks are announced
at all. `priority` is a strong lever, not a weak one — which means a misaimed push is
expensive rather than harmless, and explains how Tier 3 could be a *net cost* rather than
merely a wash. The remaining gap is most likely more push-quality problems of the same
kind, not an inert mechanism. Recorded because the expectation was wrong and acting on it
would have sent the next change in the opposite direction.

## M-44 — "sectors explored" never measured sectors (D12)

A sector audit, prompted by asking whether the sector grid was right. **It is.** On the
demo map: 48 sectors A1–F8, indices 0–47, `sector_rect` and `sector_of_cell` agree on
48/48 round trips, no sector is empty or tiny (1,637–2,666 passable cells), cell counts
vary 1.9% because 320 / 6 = 53.33 m does not divide evenly into 1 m cells, and base is in
A1. The `/0` guard on `sector_free` is present and never needed on either scenario.

What the audit found instead was a mislabelled headline number.

    sectors_explored_frac = self.explored[self.passable].mean()

That is the fraction of passable **ground**, not of sectors, and it was rendered on every
scorecard as `sectors explored 46.9%`. On the same run:

| | |
|---|---:|
| passable ground explored | **46.9%** |
| mean per-sector explored | 46.8% |
| sectors at least 1% explored | 37/48 (77%) |
| sectors at least 90% explored | 14/48 (29%) |

**The value was always honest; the name invited a reading it does not support.** "46.9%"
reads as "46.9% of sectors" when 77% had been touched and 29% were essentially finished.
It survived because sectors are near-equal in passable area, so ground coverage and mean
per-sector coverage track each other almost exactly — the mislabel was numerically
invisible.

It also mattered beyond the label: `training/gate.py` and `training/command/evaluate.py`
both weight this field in their objectives, and a reader had every reason to take the name
at face value. Renamed to `ground_explored_frac` across 15 files and relabelled
`ground explored` on the scorecard, before the contract freeze rather than after.
The dashboard reads `hud.explored` and is unaffected.

**Acceptance test, and it is the inverse of M-38's.** That entry required the seed-42 hash
to stay identical through an optimisation. Here the hash *must* change — `Scorecard.hash()`
serialises field names, so renaming one changes the digest — while every value must not:

    seed 42, headless, Tier 3 on
    before  rescued 23  found 33  explored 46.9%  lost 53   hash 49dcb2ce54f4819f
    after   rescued 23  found 33  explored 46.9%  lost 53   hash 4f5e92c4ca7bcb4a

Identical mission, new digest. Hashes recorded in M-40 through M-43 predate the rename and
cannot be compared against runs after it; the scorecard *values* in those entries stand.
288 tests green.

## M-45 — the gate runs, and its control arm had never been a control (D13)

`make gate` on the **demo** scenario, 10 held-out seeds (101–110), 4 workers, ~50 min.
First time this has been run end to end; `SHIPPING.md` is generated from it.

| component | ships | margin |
|---|---|---:|
| Tier-2 robot bodies | **evolved roster** | **1.17x** |
| Tier 3 hivemind | no Tier 3 | 1.02x |
| Tier 3a commander | no commander | 0.98x |
| CV detector | classical | not trained |

### Three defects in the gate itself, found by trying to use it

1. **The control arm was not a control.** `classical_baseline()` is documented as
   "hand-set archetypes, hand-tuned bid weights, no genome" and passed `genes=None` — but
   `World.__init__` loaded `demo_roster.yaml` unconditionally, so *both* arms ran evolved
   bodies and the comparison isolated the bid genome, which is not what it reported.
   `evolved=False` now threads `Mission → FastSim → World → evolved_roster`, and the arms
   were checked to actually differ before anything was run (archetype scout 2.538 m/s
   against evolved 2.397 m/s on the same seed).
2. **`make gate` wrote JSON into a file called `SHIPPING.md`.** The report generator did
   not exist; `main()` ran the baseline and dumped `{"baseline": ...}`.
3. **The comparison arms were never built.** The module docstring deferred them to
   "D10–D12 as each becomes available". Both now exist, plus static sections for the two
   components already decided elsewhere.

### Tier-2 bodies: 1.17x, and the trade is legible

| arm | score | rescued | found | explored | lost |
|---|---:|---:|---:|---:|---:|
| classical bodies | 167.01 | 14.10 | 35.70 | 40.3% | 94.80 |
| **evolved roster** | **196.16** | **16.20** | 32.40 | 38.2% | **65.10** |

The evolved bodies **find fewer casualties and explore less**, and rescue 15% more while
losing 31% fewer robots. That is the fitness function's trade showing up on seeds it never
saw, and it matches M-39's single-seed reading (10 → 19 rescues, −28% losses) closely
enough to believe. **Ships.** Note what does *not*: `select.py` writes bodies only, so the
genome's `BehaviorParams` half is not in the demo at all and both arms bid with the
hand-tuned classical weights.

### Tier 3: 1.02x, and my earlier reading was wrong

| arm | score | rescued | found | explored | lost |
|---|---:|---:|---:|---:|---:|
| no Tier 3 | 196.16 | 16.20 | 32.40 | 38.2% | 65.10 |
| Tier 3 scripted | 199.89 | 16.80 | 32.70 | 39.7% | 71.00 |

**On held-out seeds the scripted rung is slightly *ahead*, not behind.** M-42 and M-43
measured it on demo seeds 42–45 and found it losing (17.75 against 19.50 rescued), and
this file recorded that as "Tier 3 now measures as a net cost". Ten held-out seeds say
+2%: better on rescues, discovery and coverage, worse on losses.

**Both readings are consistent with one conclusion — the effect is not separable from
seed-to-seed variance, and it does not clear the 1.05x bar either way.** The sign flipped
between two four-and-ten-seed samples, which is exactly what a null effect looks like and
exactly the trap M-26 warned about at D7 with three seeds. The correction matters more
than the number: a component claimed as harmful on four seeds was not harmful, and the
same sample size would have been enough to claim it helpful if the seeds had fallen the
other way.

Nothing about the demo changes: `no Tier 3` ships as the gate's verdict, the hivemind
still runs in the demo as the visible strategic layer, and the honest sentence for a judge
is **"we measured it on ten held-out maps and it did not clear the bar, so we report it as
unproven rather than claiming it helps."** The tuned model — the rung this was always meant
to be a baseline for — has still never run against the demo scenario.

## M-46 — the scripted failure was not state-triggered, and the beat sheet had drifted (D13)

`docs/RUNBOOK.md` is written from measured beats rather than from PLAN §4, and measuring
them found two things the plan asserts that the build had stopped doing.

### The failure beat fired on its fallback, every time

PLAN §2 lists as a **locked decision**: "Fault injection: state-triggered, not wall-clock:
first tick where `t ≥ 200 ∧ victims_rescued ≥ 2`; hard fallback at t=260."

`demo.yaml` had `min_rescued: 30`, commented "scaled with victim count" — 25% of 120, from
the original 2-of-8. **The swarm rescues 22–23 in a good run**, so the condition could
never be met and the t=260 fallback fired on every single run. The locked decision had
quietly become false, and nothing tested it because the beat still happened.

Scaled instead against what the swarm *achieves*: ~12 rescued by t=200 on seed 42, so
`min_rescued: 10` fires the beat the moment the swarm is demonstrably working, and the
fallback still covers a run where it is not.

| | fault fires at |
|---|---|
| `min_rescued: 30` | 260.0 s — always the fallback |
| **`min_rescued: 10`** | **200.0 s — the state trigger described in PLAN §4** |

Cost: 23 → 22 rescued on seed 42, because a carrier is destroyed 60 s earlier. The beat is
worth one casualty.

### Measured beats, seed 42, shipping build

| beat |
|---|
| first directive (Tier 3 runs from t=0.1 s, every 6 s, 70 cycles) |
| first rescue |
| hazard ignites |
| scripted failure, state-triggered |
| hazard peaks at r≈60 m and begins receding |
| end, 22/120 rescued, 32 found |

`scripts/beats.py` produces this table and reads its parameters from the scenario, so it
cannot itself go stale. Run it whenever `demo.yaml` changes.

### "Directives issued" means two different things

The event feed publishes 70 `directive_issued` messages — one per 6 s cycle — while the
scorecard reports 21. Both are right: M-25 separated *decisions* from *renewals*, and the
scorecard counts decisions. Worth knowing before a judge counts the feed and asks.

## M-47 — half the swarm never left the start line, and the map was too big (D13)

Three observations off the first live `--demo` run, all correct, and the third one led to
the largest single improvement the mission has had.

### 1. "A bunch of units are not moving" — it was half the swarm

Robots that moved less than 2 m in the previous 30 s, seed 42, old scenario:

| t | stationary | of those, within 45 m of base | idle | **holding an assignment** |
|---:|---:|---:|---:|---:|
| 60 | 218 | 187 | 0 | **218** |
| 120 | 319 | 237 | 0 | **319** |
| 420 | 376 | 237 | 7 | **369** |

**~240 robots parked at spawn for the whole mission while holding tasks.** Two causes,
found by attributing the 326 robots assigned and within 45 m of base at t=120:

- **156 had goals their own chassis cannot reach.** `UNREACHABLE` is `1e9` — a *finite*
  sentinel. Divided by `v_max` it makes a huge but finite bid, `np.isfinite` accepts it,
  and the robot wins any round where no reachable robot happens to be free. The comment
  above that loop asserted such a robot "never wins"; it only never wins when a reachable
  bidder exists. Fixed: unreachable now bids `inf`, applied as a mask *after* the bid
  expression — putting `inf` inside it makes `inf * 0.0 = nan` on the genome path, which
  then relies on numpy's NaN sort order to be excluded.
- **137 were simply too crowded to move.** 326 robots inside 45 m of base is 19.5 m² each
  against a 2.5 m separation radius — one separation disc is 19.6 m². They were exactly at
  the density where repulsion cancels progress.

**The staging arithmetic did not match its own comment.** M-23 sized the staging area to
"roughly two separation discs of room", but `want_area = n * 2.0 * sep**2` is 12.5 m² and
a disc is `π * sep**2` = 19.6 m² — it allocated 0.64 of one disc. Worse, the loop stopped
growing the radius as soon as `2 * n` clear cells existed, which on a 1 m grid is **2 m²
per robot**, so the area term almost never bound. Both criteria are now the same quantity
in m² per robot and cannot drift apart again.

Fixing the unreachable bid alone moved nothing: the robots stopped falsely holding tasks
(319 → 202 assigned-and-stuck at t=120) and became *idle* instead (0 → 117), because they
still could not get anywhere. **A correctness fix that changes no outcome is still worth
making — but it is not the fix.**

### 2 and 3. The map was too big, and the fog is not what the number measures

`explored` was reported as ~25% while the operator judged ~40% from the screen. Both
readings were right about different things:

- The HUD and the scorecard count **passable ground**: `explored[passable].mean()`.
- The fog mask sent to Godot is `packbits(explored)` over **every** cell, and at t=120 it
  was 16.8% against the HUD's 22.4%.
- `fog_dim = 0.26` means unexplored ground renders at 26% brightness rather than black, so
  dimmed-but-legible rubble reads as explored to the eye.

Three different quantities, none wrong, none reconcilable by looking. The number is right;
the eye was reading the third one.

### The rescale, and what it bought

The map was cut to **360 × 240 m** with **512 robots** and **80 casualties** — lane ratio,
obstacle count, extraction lattice, terrain feature *sizes* and the hazard all scaled with
it. Scaling the map without the terrain features was tried first and broke three tests
(one seed with base on impassable ground, a collection point no wheeled unit could reach,
and chassis differentiation collapsing) — **exactly the M-2 failure of scaling a count
without its radius, repeated on the same day it was quoted.**

Four seeds, Tier 3 off, before and after:

| | 480 × 320, 768 robots, 120 casualties | 360 × 240, 512 robots, 80 casualties |
|---|---:|---:|
| rescued | 18.75 (15.6%) | **32.50 (40.6%)** |
| found | 29.25 (24.4%) | **51.25 (64.1%)** |
| ground explored | 35.5% | **75.4%** |
| **delivery** | 64.1% | 63.4% |
| robots lost | 23.25 / 768 (3.0%) | 77.25 / 512 (15.1%) |
| stationary at spawn | ~240 | **12–46** |

**Discovery and rescues both 2.6x, and delivery is unchanged.** That last column is the
important one: the rescue chain's efficiency did not improve at all — it was always
converting ~63% of what it found. Everything gained came from finding more, which is what
M-34 and M-35 said the constraint was, and what nine days of component work could not
move. The constraint was the scenario.

**Losses went 3% → 15%.** Partly the hazard being proportionally larger even after
scaling, partly that robots which used to be stuck at spawn are now out in the map where
the fire is. Seed 45 is an outlier at 191 of 512 and is not yet explained. Open.

### What this invalidates

- **`SHIPPING.md` is stale.** The gate ran on the old scenario; the 1.17x evolved-roster
  margin was measured on maps that no longer exist. Re-run `make gate` before quoting it.
- **The §6.1 discovery floor** was 30 of 120 and is now 40 of 80, re-derived from the
  measured 49 with the same ~18% headroom.
- **The §6.1 rationale** argued 75% was out of reach by 3x. On the rescaled map it is out
  by ~1.8x. The gate's *shape* survives — delivery held at ~63% across both scenarios,
  which is precisely why delivery is the right thing to gate on — but the arithmetic in
  that section now describes the old map and says so.
- Every hash and every baseline recorded in M-39 through M-46 is against the old scenario.

## M-48 — three live observations: two were the clock, one was a shimmer (D13)

All from watching the rescaled scenario run. None was the bug it looked like, and one
re-opened a decision made on the old map.

### "Carriers are not doing their job" — the pipeline is full when the clock stops

Seed 42, end of mission: 31 rescued, and of the 16 not rescued, **8 were being carried and
8 were cleared with a carrier already claiming them, 2–44 m away.**

The hypothesis was that the slowest evolved carriers cannot finish a delivery. Measured and
**rejected**: carriers span 0.43–2.64 m/s, loaded 0.30–1.85 m/s, and the distance to the
nearest collection point over passable ground is median 40 m, p90 58 m, max 76 m — so even
the slowest carrier needs 132 s of a 420 s mission for a median trip. All eight in transit
at the buzzer were on *fast* carriers (1.51–1.85 m/s) and were **7–26 seconds from
delivery**. Nothing is stuck; the mission ends mid-carry.

Worth noting what this means for the demo: ~20% of casualties are in the pipeline when
time runs out, so the final number understates what the swarm was doing.

### "The units should spread out" — measured for the third time, still no

M-14 tried 3×3 sweep waypoints per sector (rescues 13 → 5). M-36 tried claiming search
targets (13 → 5 again). Both were reverted, and both were measured on the 480 × 320 m map
where the comms component reached 240 m of a 577 m diagonal — the stated reason was that
spreading pushed scouts past the envelope, and `mark_seen` records nothing from an
out-of-contact robot.

**That premise no longer holds**: the map is 360 × 240 (433 m diagonal), reach is ~297 m,
and 74–87% of the swarm stays in contact. So it was re-tested with a cap on how many
robots may walk to the same search target (`TaskGenerator.max_per_target`, default 0):

| cap | rescued | found | explored | lost | in comms |
|---|---:|---:|---:|---:|---:|
| **off (shipped)** | **32.50** | 51.25 | 75.4% | 77.2 | 74% |
| 1 | 28.25 | 51.00 | **76.6%** | 82.8 | 73% |
| 3 | 31.50 | **52.25** | 75.7% | 81.0 | 76% |

**Spreading still costs rescues** — 13% at cap 1, 3% at cap 3 — and buys about a point of
exploration. But the *reason* has changed: comms barely moved this time (74% → 73%), so
the M-36 mechanism is not what is doing it now. The likely explanation is the report
tracker: confirmation needs parallax and repeated looks from different positions, so a
concentrated swath corroborates contacts that a thin one only glimpses. Unverified.

**Three independent attempts, three losses, two different mechanisms.** The parameter is
kept rather than deleted so the fourth person to have this idea can measure it in one
command instead of reimplementing it — but the default stays off, and 15–29 scouts
converging on one target remains what the rescue chain is built on.

### "Most robots are almost always disconnected" — 13%, flickering 3.7 times a second

| | |
|---|---:|
| out of contact at any instant | **13%** of the swarm |
| out of contact at some point in the run | 46% |
| median share of the run a robot spends out | **0%** |
| out for more than half the run | 52 of 512 |
| **connect/disconnect transitions** | **1,570 — 3.7 per second** |

The median robot never drops out at all, and the dashboard is correct: it darkens exactly
`status == OUT_OF_COMMS`, and relay-connected robots are `ACTIVE`, so there is no
rendering bug. What the operator sees is **3.7 state flips per second across the fleet**,
which reads as a shimmer and is indistinguishable from "most of them are disconnected".

M-9 already fixed the pathological case of this with join hysteresis (609 → 40 events on
the fixture). What is left is real: a robot at the edge genuinely does drop out for half a
second. So the fix is on the display side — `main.gd` now holds the darkened state for
`COMMS_HOLD_S = 1.0` after reconnection, in the same spirit as the separate display
palette. The simulator's state is unchanged and the scorecard is unaffected.

**Unverified against Godot**, which cannot run in the development environment. It is four
lines and one constant; set `COMMS_HOLD_S = 0.0` to draw the raw state.

## M-49 — relay range and count, re-swept: 96 again, and the bottleneck has moved (D13)

"Can we increase the number or range of comms?" Both, measured. M-33 swept *count* on the
480 × 320 m map with 768 robots and chose 96; that map is gone. *Range* had never been
swept at all. Three seeds, Tier 3 off; extra relays come out of the scout lane, because
the swarm size is fixed — that trade is what makes this a decision rather than an upgrade.

| range | relays | scouts | rescued | found | explored | in comms | reach |
|---:|---:|---:|---:|---:|---:|---:|---:|
| **38** | **64** | 235 | 33.3 | 54.3 | 78.6% | 83% | 305 |
| 46 | 64 | 235 | 32.0 | 59.0 | 80.1% | 92% | 324 |
| 54 | 64 | 235 | 33.3 | 59.3 | 81.1% | 94% | 340 |
| 38 | **96** | 203 | **34.7** | 58.3 | 79.6% | 88% | 306 |
| 38 | 128 | 171 | 32.3 | 54.3 | 79.6% | 83% | 301 |

### Count: 96, which is the same answer M-33 got on a different map

64 → 96 → 128 gives 33.3 / 34.7 / 32.3 rescued. **The optimum is 96 relays on both the
480 × 320 m map at 768 robots and the 360 × 240 m map at 512** — two independent sweeps on
different scenarios landing on the same absolute number is a stronger result than either
alone, and it says the relay lane is sized by the network's saturation point rather than
by swarm size. Past 96 the relays come out of the scouts who do the reaching, exactly as
M-33 found.

Shipped. Four-seed confirmation: rescued 32.50 → **33.00**, found 51.25 → **55.75**,
explored 75.4% → **77.4%**, reach 297 → 304 m.

### Range: no, and the reason is the interesting part

38 → 46 → 54 m moves comms **83% → 92% → 94%** and discovery **54.3 → 59.0 → 59.3**, and
leaves rescues **flat at 33.3 / 32.0 / 33.3**. More range buys coverage the mission no
longer converts. It also costs the thing the lane exists for: at 54 m the network
effectively blankets the map, and "96 relays cannot blanket 360 × 240 m, so placement stays
a real decision" stops being true. Held at 38 m.

### The bottleneck has moved, and this is the measurement that shows it

For nine days the chain was `rescued <- discovery <- explored <- comms reach` (M-34, M-35,
M-39, M-40), and every improvement to comms or coverage paid out in rescues. It no longer
does:

| | 64 relays | 96 relays | change |
|---|---:|---:|---:|
| found | 51.25 | 55.75 | **+8.8%** |
| rescued | 32.50 | 33.00 | +1.5% |
| **delivery** | 63.4% | **59.2%** | **−4.2 points** |

**Discovery rises and delivery falls by almost exactly the amount that keeps rescues
flat.** M-48 measured the same thing from the other side: ~20% of casualties are mid-chain
when the mission ends, eight of them within 26 seconds of a collection point. After the
D13 rescale the swarm finds more than it can carry home in 420 s, so the binding
constraint is **delivery time**, not discovery.

That retires the standing advice in this file that work aimed at extraction and allocation
is aimed at the wrong link (M-39). On this scenario it is now the *right* link — and the
levers are carrier count, carry speed, collection-point density, and the mission clock,
none of which have been swept since the rescale.

## M-50 — the delivery deadline: nothing found after t≈300 is ever rescued (D13)

M-49 concluded the binding constraint had moved from discovery to delivery. This is the
follow-up, and it sharpens that into something actionable — and rules out every obvious
delivery lever on the way.

### Every delivery lever measured, and none of them work

Three seeds, Tier 3 off. Extra carriers come out of the scout lane.

| config | rescued | found | delivery | mttr | carriers on extract / exploring |
|---|---:|---:|---:|---:|---|
| **shipped** | **34.7** | 58.3 | 59.4% | 165 | 18 / 53 of 79 |
| 128 carriers | 31.3 | 56.7 | 55.3% | 146 | 19 / 77 of 108 |
| 160 carriers | 35.0 | 59.3 | 59.0% | 158 | 19 / 105 of 141 |
| `carry_speed_factor` 0.85 | 30.7 | 51.0 | 60.1% | 125 | 15 / 56 of 80 |
| 20 collection points | 23.7 | 51.3 | 46.1% | 131 | 23 / 52 of 88 |

The last row is **confounded and should not be read as a result**: collection points are
keepouts for obstacle and casualty placement, so changing them changes the map. Noted
rather than deleted, because the next person to try it deserves to know why it looks bad.

The rest is unambiguous, and the answer is in the final column: **only 18–23 carriers are
ever on an extract task, whatever the lane size.** Doubling the carriers adds explorers,
not deliveries. Delivery is not supply-limited.

### It is deadline-limited, and the deadline is hard

175 casualties found across three seeds, by the time they were found:

| found in | n | rescued | rate |
|---|---:|---:|---:|
| 0–60 s | 79 | 56 | 71% |
| 60–120 s | 26 | 22 | **85%** |
| 120–180 s | 21 | 10 | 48% |
| 180–240 s | 15 | 8 | 53% |
| 240–300 s | 15 | 8 | 53% |
| **300–360 s** | 10 | **0** | **0%** |
| **360–421 s** | 9 | **0** | **0%** |

**Nothing found after t≈300 is ever delivered — 19 of 175 finds, 11%, are dead on
arrival.** And the 120–300 s band converts at ~51% against 71–85% for the first two
minutes. With `mean_time_to_rescue` at 165 s against a 420 s clock, that is arithmetic
rather than a defect.

This is M-35's conclusion — *only casualties discovered early can be delivered* — surviving
a rescale that changed every number around it. At D9 the deadline was ~150 s of a 420 s
mission with 24% discovery; it is now ~300 s with 70% discovery, and it still binds.

**No allocation fix addresses this.** Extract tasks rank above search, the announcement cap
never excludes them, and every one of them is awarded within a cycle — so triaging or
re-ordering them changes nothing. The lever is *when* casualties are found, not who is
sent.

### Faster scouts: more coverage, fewer rescues

The obvious response is to find things sooner. Every scout was set to the fastest evolved
scout's speed, and separately to the slowest, as a bound on the benefit:

| scout lane | rescued | found | explored | found by 120 s | lost |
|---|---:|---:|---:|---:|---:|
| **evolved (shipped)** | **34.7** | 58.3 | 79.6% | 35.0 | 37 |
| all at max speed | 32.3 | 58.3 | **83.7%** | 37.7 | 29 |
| all at min speed | 27.3 | 43.0 | 66.2% | 30.0 | 65 |

Speed buys **4 points of exploration and 2.7 more early finds, and costs 2.4 rescues.**
Total discovery is identical to a tenth. The slow arm confirms the manipulation works, so
the flat result is real rather than an inert knob.

The likely reason is the same one M-48 reached from the search-spread direction: the report
tracker confirms a contact through parallax and repeated looks, so a scout that covers
ground faster gets fewer looks per place. Coverage and confirmation are different
quantities and this project keeps rediscovering it.

**So the evolved roster's speed diversity is not a defect to be corrected** — which was the
open question from the D13 dashboard review, and it is now measured rather than assumed.

### What is actually left

Rescues have been flat at 32–35 across nine configurations spanning relay count, relay
range, carrier count, carry speed, scout speed and search spread. Everything that raises
coverage raises *late* discovery, which converts at zero. The remaining levers all act on
the first two minutes:

1. **The swarm starts massed and must disperse.** Staging is fixed (M-47) but dispersal
   still takes time, and the first 120 s is where 60% of all deliverable finds happen.
2. **The 0–60 s cohort converts at 71% while 60–120 s converts at 85%** — the earliest
   finds do *worse*, and nothing explains that yet. Worth understanding before tuning
   anything else.
3. **Shrinking the map again** would raise early discovery the way the D13 rescale did.
   That is a scope decision, not a code one.

## M-51 — five things off the dashboard: one hypothesis dead, one demo problem (D13)

Five observations from watching the rescaled scenario at t=303. All checked; the headline
is that the endgame is now *empty*, which is the flip side of the rescale that fixed
discovery.

### It is not battery

The obvious reading of "the robots stop near the end" plus a `batt 67%` readout is that
the swarm runs flat. **It does not.** Seed 42, over the run:

| t | alive | battery > 20% | battery < 5% | flat | moving > 2 m / 30 s | explored | newly |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 30 | 512 | 512 | 0 | 0 | 481 | 29.8% | 29.8 |
| 120 | 512 | 512 | 0 | 0 | 381 | 49.3% | 4.9 |
| 300 | 472 | 450 | 4 | **0** | 279 | 69.0% | 2.4 |
| 420 | 436 | 401 | 6 | **0** | **230** | 76.0% | 2.1 |

**No robot ever reaches zero charge**, and 92% finish above 20%. What actually falls is
work: robots moving drops 481 → 230 while newly-explored per 30 s collapses 29.8 → 2.1.
The swarm stops because the frontier does.

### The exploration ceiling is 96%, and the dark ground is solid rubble

| | |
|---|---:|
| passable (`occ != WALL`) | 69% of the map |
| of that, no chassis can traverse | 2.6% |
| of that, unreachable from base by any chassis | 3.8% |
| **ceiling on "ground explored"** | **96%** |
| reached at t=420 | 72–76% |

**Rivers and hills are searched.** Water and slope do not block line of sight — only WALL
does — and **87% of the ground no chassis can stand on is observed anyway**, from the bank.
The permanently dark regions are the *interiors of rubble and wall clusters*, which are 31%
of the map and which nothing can ever see inside. That is correct, and it is also why a
quarter of the map stays black forever and reads as "never searched".

Also checked, because rescaling terrain is where M-2 and M-47 both went wrong: mountain
peak height is `radius x steepness`, so max gradient is `1.27 x steepness` and is
**independent of radius** — the 0.75x radius scaling preserved the gradient budget by
construction. Heightfield spans 0–33 m with 27% of cells more than 2 m above mean, so the
relief is real.

One consequence of that scaling was *not* free: ditch depth was deliberately left unscaled
(depths gate chassis against absolute wade limits) while ditch width scaled 3–6 → 2.5–4.5 m,
which makes ditch walls steeper. Slope p99 is now 2.65 and max 10.4, and 2,268 cells are
unreachable by anything. It does not bind — the ceiling is 96% and the swarm reaches 76% —
but it is a real side effect of a decision recorded as harmless.

### The demo problem: the last two minutes are empty

From t≈300 exploration adds ~2 points per 30 s, half the swarm has stopped moving, and
nothing found after that point is ever delivered (M-50). **The final third of a
seven-minute demo has the swarm visibly idle**, and that is a direct consequence of cutting
the map 44% while leaving the mission at 420 s.

This is the same coupling CLAUDE.md states and this file keeps relearning: robot count,
map size, casualty count and mission length move together. The rescale moved three of the
four. Three ways out, none yet measured:

1. **More casualties** (80 → 110ish). Exploration still saturates, but the rescue chain
   keeps working through the endgame — and rescues are what the gate counts.
2. **A shorter mission** (420 → 360 s). Still inside the 5–7 minute demo target, and it
   deletes the dead third rather than filling it.
3. **A slightly larger map** (~400 x 270). Recovers frontier at the cost of some of the
   discovery the rescale bought.

Option 1 is the only one that makes the endgame *busier* rather than shorter, and it is the
one to measure first.

---

## M-53 — the casualty mesh: 919,567 triangles down to 1,801 (D13)

The victims were an inverted red cone. They are now a person lying curled on their side —
"Curled Up Silence" by bac213tv1, CC-BY-4.0, credited in `godot/assets/ATTRIBUTION.md`.

The download is a photogrammetry scan and is **not usable as shipped**:

| | source | shipped |
|---|---:|---:|
| triangles | 919,567 | 1,801 |
| vertices | 572,560 | 789 |
| textures | 3 × 2048² (52 MB buffer) | none, baked to vertex colours |
| bytes | 52.3 MB | 33.3 KB |

At source size a single casualty is 264x the 3,500-triangle average of a scattered prop
(M-20), and the god-view toggle instances one per live victim — 110 on the demo scenario.
That is **101M triangles**, against a whole-world prop budget of 5.4M. Shipped, the same
toggle costs **198k**, or 3.7% on top of the props.

### Where the reduction came from

Grid-cluster decimation, written in numpy because there is no Blender on this machine:
cluster vertices on a lattice, average position/normal/colour per cluster, drop the
degenerate and duplicated faces, binary-search the lattice size for a triangle budget.
Rendered at 1,800 / 3,600 / 6,000 triangles through `viz`-style offline previews and
compared at dashboard scale, **1,800 is indistinguishable from 6,000** — the body is
2.5 m on a 260 m orbit. It is decimation quality that is invisible at that size, not
detail worth paying for.

Textures went the same way the nature pack's normal maps went (M-19), for the same
reason: the marker material is `SHADING_MODE_UNSHADED`, so a metallic-roughness map and a
normal map have no effect whatsoever. The base colour is sampled per vertex and multiplied
by the `LIGHT_DIR` term the world already bakes (`n·L * 0.75 + 0.25`), which is why the
body sits in the same lighting as everything else on an unshaded surface.

### Two nodes per casualty, not one

The pin marker is kept and the body is drawn *as well*, because they answer different
questions and cannot share a material:

- The pin has `no_depth_test`. That is the whole point of a ground-truth toggle — it reads
  through rubble and haze — and it is safe on a thin cone.
- The **body cannot use `no_depth_test`**: it is a closed solid, and with the depth test
  off its far side overwrites its near side. It is depth-tested and part of the world.

So the pin says *where* (through anything), the body says *what* (where the camera can see
it), and buried casualties keep their lower pin. The pin shrank 5.0 → 3.0 m so it reads as
a marker above a body rather than as the marker.

### It is drawn at 1.8x life size, and that is still small

| orbit distance | 1 m in pixels | 2.5 m body |
|---|---:|---:|
| 260 m (default) | 5.6 | **14 px** |
| ~130 m (zoomed) | 11 | 28 px |
| ~50 m (close) | 28 | 70 px |

At 14 px the body is a smudge with a human outline; it is the pin that marks the position
from the default orbit, and the body only becomes readable as a person when the operator
zooms in — which is the beat it exists for. 1.8x (2.5 m) sits just above the 1.4 m robot
boxes so casualties separate from robots at a glance, without a giant lying in the rubble
in the robot-POV camera. Life size (1.0x) is 8 px and reads as noise; 3x looks staged.

Verified offline, since Godot cannot be run here: the shipped `.glb` is re-parsed by an
independent reader and rasterised from four angles at the tint `main.gd` applies. Both
`.glb` chunk lengths, the accessor bounds and unit normals are asserted on the file that
actually ships.

## M-53 — six sectors at 0% with robots standing in them (D13)

"Make exploration 100%." It sits at ~73–77% of reachable ground, and six sectors on the far
side of the map finish at **0.0% explored** on seed 42. Four hypotheses, three wrong.

### Wrong 1: unreported buffers

Out-of-contact robots buffer what they see, so the shared map might just be behind. It is
not: of the ground showing as unexplored, **0.4 points is seen-but-unreported** (205 cells,
4 holders). **26.8% of reachable ground is never seen by anyone.** Store-and-forward
(M-37) is doing its job.

### Wrong 2: the announcement cap starves exploration

With 623 contacts on screen and investigate outranking explore, the cap looked like it
must be full of phantoms. Measured composition of the announced set:

    t=180   67 open, cap 87   explore=67                    nothing dropped
    t=420   61 open, cap 73   explore=61                    nothing dropped

**Almost the entire announced set is exploration**, and after t=60 nothing is dropped at
all. The cap is not the constraint.

### Wrong 3: the far sectors are unreachable terrain

    wheeled reaches 81% of the far sectors, tracked 90%, legged 99%, rotor 88%

They are reachable by every chassis.

### What is actually happening

Tracing offers, assignments and presence into those six sectors over a whole mission:

| sector | task offers | robot-assignments | ever stood in | explored |
|---|---:|---:|---|---:|
| B8 | 422 | 3838 | no | 0.0% |
| C8 | 421 | 6166 | **yes** | **0.0%** |
| D8 | 423 | 4196 | **yes** | **0.0%** |
| E8 | 422 | 5733 | **yes** | **0.0%** |
| F8 | 421 | 5508 | **yes** | **0.0%** |

**Work is offered, robots are assigned, robots physically stand there, and the sector
reads 0%.** `World.mark_seen` writes the shared map only for in-contact robots, and those
sectors are 329–385 m from base against a comms reach of ~310 m. The swarm explores them
perfectly and none of it is ever heard.

### The fix that looked obvious and measured as a regression

If data has no value until delivered, an out-of-contact robot carrying a large backlog
should walk back and report. Implemented at a 1,500-cell threshold:

| | before | with report-home |
|---|---:|---:|
| rescued | 43.25 | 40.00 |
| found | 72.75 | 70.75 |
| ground explored | **76.7%** | **72.2%** |
| delivery | 59.5% | 56.5% |
| far sectors with a robot ever in them | 5 of 6 | **0 of 6** |

It made things strictly worse and **stopped anything reaching the far sectors at all**.
The buffer fills on the way out, the robot turns around, delivers, is re-tasked outward,
and commutes for the rest of the mission. Reverted, with the reasoning left at the point
in `skill_executor.py` where the next person will reach for it.

**The constraint is the reach of the network, not the willingness of the robot** — which
is the M-36 lesson again, from a fourth direction.

### What would actually work, and the arithmetic for it

The far corner needs ~390 m of comms reach; the chain plateaus at ~310 m. At a 30.4 m post
spacing that is **13 posts to reach 390 m, and 96 relays affords 7 such chains** — so the
lane can cover it. It does not, because `relay_posts` builds chains toward the largest
*frontier clusters*, and the frontier is at the comms boundary, so chain direction is
chosen by where the swarm already is rather than by where it still needs to go.

That is the same coupling M-39 broke for chain *length* (lookahead past the frontier), left
unfixed for chain *direction*. Not attempted here.

### Also fixed while measuring

- **Comms recovery aimed at base.** An idle out-of-contact robot walked all the way to the
  corner to regain a link that was often tens of metres away — 30 robots, median 281 m out.
  It now closes on the nearest robot still in contact. Correct, and worth ~0 on the
  mission (43.25 rescued before, 44.67 on the same seeds after): the robots were already
  returning, just expensively.
- **Comms state now invalidates the goal cache.** It became a goal input and was not
  tracked, so a robot that had just lost contact kept the stale answer until an unrelated
  assignment dirtied the cache.
- **A destroyed robot's buffer is dropped** rather than left in `_pending_cells` for the
  rest of the mission. Bookkeeping only — `_store_and_forward` already skipped the dead.

## M-54 — relay chains now aim at unexplored ground, and reach is capped by something else (D13)

M-53 ended with a specific claim: chain *direction* comes from the largest frontier
clusters, the frontier sits at the comms boundary, so the network grows where the swarm
already is. `TaskGenerator.chain_targets` makes that switchable — `frontier` (the old
behaviour) or `unexplored`, which aims each chain at the least-explored sector, spread
across the map by construction.

| aim | rescued | found | explored | **the six far sectors** | reach |
|---|---:|---:|---:|---:|---:|
| frontier | 43.25 | 72.75 | 76.7% | **18.7%** | 309 m |
| **unexplored** | 43.00 | 72.75 | **77.5%** | **24.2%** | 309 m |

It does what it was designed to do — **+5.5 points on the six sectors** that finished at
0%, seed 42 going 6.2% → 16.8% — and is neutral everywhere else. Shipped as the default.

**And it does not move reach at all: 309 m in both arms.** Direction was never the binding
constraint, which the four-seed A/B says more clearly than any argument would have.

### What actually caps the chain, as far as this goes

Tracking the relay lane through a mission:

| t | reach | relays with a post | **arrived** | **still walking** | farthest post assigned |
|---:|---:|---:|---:|---:|---:|
| 30 | 210 | 65 | 14 | 51 | **395** |
| 150 | 262 | 63 | 28 | 35 | 395 |
| 300 | 308 | 51 | 30 | 21 | 395 |
| 420 | 309 | 69 | **31** | **38** | 395 |

**Posts are assigned out to 395 m within the first 30 seconds** — the generator is not the
problem and never was. Roughly 65 relays hold a post at any time and **only ~30 ever
arrive**; 38–51 are permanently in transit. Arrivals plateau at 30 by t=300 and do not
move again.

### A hypothesis that was wrong, and the way it was wrong is the useful part

The obvious reading is duplicate dispatch: a relay walking 300 m is silent for 200+ s, the
task is re-offered after 30 s, and a second relay is sent to the same post. Exempting
`relay` from silence-based orphaning produced results **identical to the digit across all
fourteen rows** — so the branch never fired.

**Relays extending a chain do not go silent.** They stay in contact through the links
behind them, which is precisely the design working. The re-offer path was never involved,
and the change was a no-op dressed as a fix; it has been reverted rather than left in with
a comment claiming credit.

So the lane is not thrashing on duplicates. Sixty-five relays are assigned reachable posts,
stay in contact, walk, and two-thirds of them never arrive. **That is the open question**,
and the candidates not yet separated are `SkillExecutor._stalled` releasing them mid-walk,
hazard preemption pulling them onto `retreat`, and simple travel time against the clock —
395 m at ~1.4 m/s is 280 s of a 420 s mission.

Three mechanism hypotheses in two days (report-home, chain direction, silence-orphaning):
one regression, one small win, one no-op. The chain of reasoning was sound each time and
the measurement disagreed each time, which is the argument for the A/B switch being a
parameter rather than a rewrite.

## M-55 — the stall detector was killing the relay lane (D13)

M-54 left a narrow question: 65 relays hold reachable posts, stay in contact, walk, and
two-thirds never arrive. Instrumenting what ends a relay post assignment answers it:

    relay assignments ended during the mission: 310
      stalled            300  (97%)
      no longer needed     9
      destroyed            1
    distance from its post when released: median 106 m, p90 281 m, max 386 m
    released within 5 m of the post: 3 of 310

**97% of relay posts are killed by `SkillExecutor._stalled`, a median 106 m short.** The
lane spends the mission being reassigned rather than building the chain, which is why
comms reach sat at 309 m however the chains were aimed.

### Two defects behind it

**1. The code contradicted its own docstring.** `_stalled` documents its reference as "the
distance at the last checkpoint, not the best ever seen: with a min-ever reference a robot
that detours can never re-establish progress" — and then did `a.last_dist = min(a.last_dist, d)`,
which is exactly the min-ever reference. Any detour longer than the 25 s window is fatal,
and relay posts are the longest walks in the mission. Fixed to the documented behaviour:
310 → 255 stalls, median release distance 106 → 85 m.

**2. An unreadable distance counted as *no progress* rather than *no evidence*.**
Measured over 11,252 relay-seconds en route to a post:

    geodesic metres closed per second: median 0.00, mean -7,109,846
    83% of samples close less than PROGRESS_EPS (0.5 m) -- at every speed

That mean is the tell: `distance_at` returns `UNREACHABLE` (1e9) for a robot's *own
position*. The nav grid is a 4x downsample with a majority rule (M-23), so a robot can
stand on a fine cell inside a coarse cell its chassis cannot enter, and read "infinitely
far from my goal". Counting that against the robot burns the window. Now treated as no
evidence — the clock does not run on a reading that carries none. 255 → **199 stalls**,
median release distance 85 → 73 m, relays arriving at post 30 → **35**.

### And the mission does not move

| | before | after both fixes |
|---|---:|---:|
| rescued | 43.00 | **44.00** |
| found | 72.75 | 70.50 |
| ground explored | 77.5% | 75.7% |
| **delivery** | 59.5% | **62.4%** |
| comms reach (mean) | 309 m | 309 m |

Stalls down 36%, arrivals up 17%, peak reach up (353 m at t=300 against 308 m), and the
scorecard is a wash: +1 rescue, −2.25 found, +2.9 points of delivery, all inside the
seed-to-seed spread.

### The pattern across D13, stated plainly

Rescues have now been measured at **40–45 across roughly fifteen configurations** spanning
relay count, relay range, chain aim, chain length, carrier count, carry speed, scout speed,
search spread, task supply, comms recovery, and two genuine stall-detector bugs. Several of
those changes fixed real defects — code that contradicted its own documentation, a signal
that treated missing data as bad news, a control arm that was not a control. **None of them
moved the mission by more than noise.**

That is itself the finding. The system is on a plateau where the binding constraint is not
any single mechanism but the relationship between map size, mission clock and swarm speed —
the four-way coupling CLAUDE.md states and this file has now relearned from six directions.
The one change on D13 that *did* move the mission was scenario-shaped: 480x320 → 360x240
with 512 robots, which took rescues from 15.6% to 40.6% (M-47).

**A mechanism fix should be justified as a correctness fix from here on, not as a
performance one**, unless it comes with a measurement showing otherwise.

## M-56 — the swarm covers ground it never examines (D13)

Fifteen configurations moved rescues by nothing (M-55). This is why. Seed 42, 110
casualties, 68 found, 42 never found.

### The missed casualties are not in ground nobody visited

| of the 42 never found | |
|---|---:|
| standing on ground the swarm mapped | 13 |
| a robot came within 8 m of them | **27** |
| **median closest approach by any robot** | **3 m** (minimum 0 m) |

The swarm walks within three metres of the people it does not find.

### They are looked at an order of magnitude less

Counting 5 Hz frames in which a casualty was inside some robot's 90° camera arc:

| mean frames per casualty | found | never found |
|---|---:|---:|
| within sensor range | 1003.8 | 134.1 |
| **inside the 90° FOV** | **413.0** | **35.3** |
| framed and closer than 3 m | 142.6 | **7.5** |

A found casualty gets ~413 framed looks — about 80 seconds of camera time. A missed one
gets 35, about seven seconds. **Discovery is a dwell-time process, and the search pattern
allocates dwell very unevenly.**

Splitting the misses by exposure:

    never framed even once          19 of 42     <- genuine coverage failure
    framed 10+ times and still missed  21 of 42  <- looked at, not seen
    framed within 3 m at least once    16 of 42

**Half the misses were looked at repeatedly.** And burial is the discriminator: **55% of
missed casualties are buried against 31% of found ones** — a buried casualty renders as a
small patch that grows only as debris clears (M-7), so it needs far more looking than an
unburied one, and the search pattern gives every cell the same brief glance.

### Why the report pipeline is not the culprit either

Of 4,822 reports created, 53% resolve, 20% are dismissed as phantoms, 27% never leave
CANDIDATE and are pruned as stale. That funnel is working. **Only 7 of the 42 missed
casualties ever had a report on record within 6 m of them** — so 35 of them never produced
a surviving detection at all. The loss is upstream of the tracker, at the camera.

### What this explains

Every null result of D13 follows from it, and so does the one success:

| change | why it did nothing |
|---|---|
| faster scouts (M-50) | more ground covered, **less dwell per place** |
| spreading the search (M-48) | same: fewer looks per casualty |
| more relays, longer range (M-49) | buys coverage, and coverage is not the constraint |
| more carriers, faster carrying (M-50) | delivery was never the constraint |
| chain aim, stall fixes (M-54, M-55) | correctness, no effect on dwell |
| **cutting the map 480x320 → 360x240 (M-47)** | **the same looking concentrated on 44% less ground — rescues 15.6% → 40.6%** |

### The metric has been measuring the wrong thing

`ground_explored_frac` counts a cell once a camera pixel covered it. Finding a person there
additionally requires them inside a 90° arc, close, unoccluded, and — if buried — very
close indeed. **"Explored" and "examined" are different quantities, and only the first is
measured.** 76% explored against 64% found is the gap between them, and it is why raising
coverage has stopped buying discovery.

No fix attempted; this entry is the diagnosis. What follows from it is that the lever is
dwell — revisiting, slower sweeps, or a search pattern that spends looking time in
proportion to how hard a cell is to read — and that any candidate should be measured
against framed-looks-per-casualty rather than against explored fraction.

## M-57 — the single line upstream of everything, verified independently (D13)

`POSSIBLE_BUG3.md` (written separately, same day) names a cause I had not found, and it is
upstream of M-56. Re-instrumented here from scratch rather than taken on trust, and the
numbers reproduce **exactly**:

    explore: 4432 assignments ended
       completed 2681 (60%)  dist median  88.1 m  held median  1.0 s
       stalled   1687 (38%)  dist median 173.9 m  held median 22.0 s
       ended within 5 m of goal: 320 of 4432 (7%)

`SkillExecutor._state` retires an explore task when `world.explored[iy, ix]` is true — the
**shared** map, with no mention of the robot holding the task. Explore targets sit on the
explored/unexplored boundary by construction and 512 robots lift fog at 5 Hz, so the cell
is revealed by whoever is nearest and **every robot assigned to it is released where it
stands**. The median explore assignment lasts one second and ends 88 metres short.

The comparison that settles it is inside the same data. `investigate` completes on its own
holder arriving:

| task kind | completes when | ends within 5 m of goal | median hold |
|---|---|---:|---:|
| **explore** | *anyone* reveals the cell | **7%** | **1.0 s** |
| investigate | its holder arrives | **58%** | 14.6 s |
| relay | its holder arrives | 1% | 27.0 s |

BUG3 also finds that roughly half the frontier targets are *already explored when
announced*, because `frontier_targets` takes the **centroid** of an 8x8 bin of frontier
cells: a boundary bin is a curve, and the mean of a curve lands on its concave side, which
is the explored side. `frontier_mask`'s own docstring records that exact failure as found
and fixed — "the target cell is already explored by construction, so robots receive a task,
retire it, and never move" — and the centroid puts it back one function later.

### How this connects to M-56

One chain, from a single completion condition to the rescue rate:

1. An explore task retires on the shared map, so the median assignment lives **1.0 s**.
2. Robots churn — assigned, released, idle, waiting for the next auction round.
3. They never travel far enough to **dwell** anywhere.
4. Dwell is what discovery actually needs: found casualties get **413** framed camera
   frames, missed ones **35** (M-56).
5. Buried casualties need the most looking and are **55% of the misses against 31% of the
   finds**.
6. Rescues sit at 40–45 whatever else is changed, because nothing else was ever the
   constraint.

M-56 measured the symptom precisely and named dwell as the lever. This is why dwell is
low. **Both entries are the same finding at different depths, and this one is actionable.**

### Bookkeeping

`POSSIBLE_BUG2.md` and `POSSIBLE_BUG3.md` both propose to land as M-56, which is already
taken by the dwell measurement. They belong here as **M-57 (this entry)** and, for BUG2's
flow-field/congestion numbers, a further row — neither has been merged into this file yet,
and the two documents overlap in symptom while differing in named cause. Reconciling them
is worth doing before either is acted on.

## M-58 — 58% of the tick budget was one accidentally quadratic loop (D13)

[POSSIBLE_BUG3.md §8](../POSSIBLE_BUG3.md) reported the demo scenario running at **0.97x
realtime — 434 s of wall clock for a 420 s mission** — and correctly flagged it as
demo-fatal: `DemoSim.step` sleeps only when it is *ahead* of the wall clock, so below 1.0x
it can never catch up, and `--demo` adds the WS bridge, the blackboard and the hivemind
thread on top of an already-negative margin.

Profiled at t=240 on the demo scenario, 400 ticks:

| | cumulative |
|---|---:|
| `mission.tick` | 27.38 s |
| **`blackboard.publish` → `build` → `_found_in`** | **15.80 s (58%)** |
| `grid.world_to_cell` (633,156 calls) | 12.16 s |
| ...of which `np.clip` → `_wrapfunc` → `_methods._clip` | 8.39 s |
| `np.finfo.__init__` | **2,964,952 calls** |

### Two defects, one of them a repeat

**1. `_found_in` was O(sectors × casualties) computing an O(casualties) quantity.** It took
a `sector_id` and looped over every casualty converting its position to a cell, and was
called once per sector — 48 × 110 = **5,280 `world_to_cell` calls per publish at 10 Hz**.
Replaced with one pass and a `bincount`.

**2. `world_to_cell` used `np.clip`, which is M-38's finding verbatim.** That entry records
the module-level function detouring through `_wrapfunc` → `_methods._clip` and building
two `np.finfo` objects per call, and fixed it in `control/planner.py`. **This call site was
missed, and it is the hottest function in the codebase.** `ndarray.clip` now.

### Result

| | before | after |
|---|---:|---:|
| profiled window (400 ticks at t=240) | 27.38 s | **13.27 s** |
| `blackboard.publish` within it | 15.80 s | **2.08 s** |
| function calls | 29.8 M | **10.6 M** |
| **full mission, wall** | **434 s** | **245.5 s** |
| **realtime factor** | **0.97x** | **1.71x** |

**Seed 42 hashes `f7e7755146e16ffc` before and after** — the same digest BUG3 records for
its baseline — with every scorecard field unchanged (36 rescued, 70 found, 76.2% explored,
62 lost). That is the acceptance test for a change of this kind, and it is M-38's test
applied to M-38's own bug in a second location.

### What this does and does not fix

It closes BUG3 §8 and restores the demo's realtime headroom. **It changes no behaviour and
therefore fixes none of the mission findings** — the dwell problem (M-56), the explore
completion condition (M-57), and BUG2's phantom coarse-grid edges are all untouched and
all still open.

Also worth recording: CLAUDE.md still describes this run as "~25 s". It was ~434 s and is
now ~245 s. That line has been wrong by more than an order of magnitude for some time and
should be corrected whatever else happens.

**The general lesson is the repeat itself.** M-38 diagnosed `np.clip` precisely, fixed the
site it was profiling, and the same pattern sat in a hotter function for five days. A fix
recorded as "we replaced `np.clip` here" invites exactly that; the durable version is a
grep across the codebase, which is now worth doing for the remaining call sites.

## M-60 — the flow field was routing through walls: +35% rescues (D13)

[POSSIBLE_BUG2.md](../POSSIBLE_BUG2.md) claimed the coarse navigation grid contains edges
that do not exist in the world, and that robots routed onto them are held against rock
until the safety floor zeroes them. **It is right, and fixing it is the largest single
improvement the mission has had.**

### Verified independently before changing anything

BUG2's own suggested test — for every 8-connected step between two passable coarse cells,
assert at least one pair of 8-adjacent passable *fine* cells across the boundary — written
from scratch and run against `demo`:

| seed 42 | mine | BUG2 |
|---|---:|---:|
| wheeled | 1086 / 19946 (5.4%) | 1090 / 19946 (5.5%) |
| seed 43, wheeled | 1134 / 18552 (6.1%) | 1138 / 18552 (6.1%) |
| diagonal share | 75% | "three-quarters" |

Same denominators exactly, numerators within four (a marginally different definition of
"touching"). Two independent implementations agreeing to that precision is as close to
proof as this gets.

### The mechanism, and why diagonals dominate

`downsample` calls a coarse block passable when ≥50% of its fine cells are, and
`distance_field` then assumes plain 8-connectivity between passable blocks. Both are
individually defensible; together they invent corridors. Two blocks can each pass the
majority test while their passable fine cells sit on opposite sides of solid rock.

For a **pure diagonal** the only fine cells that can possibly touch are one corner cell
against one corner cell — which is why 75% of phantom edges are diagonal, and it is the
mechanism, not a coincidence.

`grid.coarse_edge_masks(fine, factor)` now derives the real adjacency from the fine grid
(one `(8, H, W)` bool array per chassis, built once — `passable` never changes), and
`distance_field` expands per direction through it instead of calling `dilate8`. The
vectorised masks reproduce the brute-force phantom count exactly on all four chassis.

### Result: four seeds, Tier 3 off

| | 8-connected | fine-linked |
|---|---:|---:|
| rescued | 44.00 | **59.50** |
| found | 70.50 | **81.50** |
| ground explored | 75.7% | **83.4%** |
| **delivery** | 59.5% | **73.0%** |
| comms reach | 309 m | **357 m** |
| robots lost | 75 | 67 |
| **densest 6 × 6 m bin** | **32** | **14** |

**Up on all four seeds** — 42→61, 45→69, 45→55, 44→53. Seed 42 goes 42 → 61 rescued,
76.2% → 88.2% explored, and its densest bin collapses from 58 robots to 9: BUG2's clump
dissolving, which is the specific prediction its diagnosis made.

Headless seed 42 with Tier 3 on: **66 rescued of 110, 86 found, 92.5% explored.**

### What it explains that nothing else did

- **Comms reach 309 → 357 m.** M-54 measured reach pinned at 309 m however chains were
  aimed, and M-55 found 97% of relay posts dying stalled a median 106 m short. Both are
  the same defect seen from downstream: relays were being routed into traps on the way to
  their posts. The relay lane was never broken.
- **The dwell problem (M-56).** Robots held against rock are robots not looking anywhere,
  and the whole D13 plateau — rescues 40–45 across fifteen configurations — sits on it.
- **M-23's open item.** FIXES.md carries "~33% Tier 1 override rate remains; the
  coarse/fine mismatch hypothesis was measured and rejected". M-23 tested the
  **threshold** and correctly found stricter values worse. That is a different hypothesis:
  no value of `threshold` fixes a diagonal edge between two majority-passable cells whose
  shared corner is solid. Reopened and now closed.

### The cost, which is real

The correct field is more expensive than the wrong one, and the mission also does far
more work now — 92.5% explored against 76.2%, so more frontier targets, more fields, more
casualties tracked. The first implementation allocated an array per direction and ran the
mission at **0.85x**; rewriting the expansion in `dilate8`'s in-place-slice style got it to
1.00x, which is break-even and not enough, since `--demo` adds the bridge, the blackboard
and the hivemind on top and `DemoSim` only sleeps when it is ahead.

**So the wavefront was given a bounding box.** It grows by exactly one cell per level, so
for most of a field only a small window can change, and operating on the whole 90 x 60
grid at every one of ~150 levels was the bulk of the cost. The window is tracked exactly,
one cell of margin per level:

| coarse distance field | 8-connected | fine-linked |
|---|---:|---:|
| whole-array (before) | 6.45 ms | 13.84 ms |
| **windowed** | **2.76 ms** | **5.65 ms** |

**The correct field is now cheaper than the wrong one used to be**, and twelve fields
across two chassis and six targets are bit-identical to a reference implementation.

| full mission, seed 42 | |
|---|---:|
| at session start (BUG3 §8) | 434 s, **0.97x** |
| after M-58 | 245 s, 1.71x |
| after the phantom-edge fix | 420 s, 1.00x |
| **after the windowed wavefront** | **189.6 s, 2.22x** |

Same hash before and after the window (`2a392483ac5b8158`), so the speedup is free of
behaviour. **The session ends with the mission 2.3x faster than it started *and* rescuing
83% more people.**

### One thing measured and not taken

The field cache holds 512 entries per chassis and finishes full, which looks like thrash.
It is not: a mission touches **7,044 distinct (goal, chassis) keys**, so 8192 entries
builds exactly as many fields as 2048 does. Raising the cap buys 4% of wall clock for
108 MB against an 8 GB budget with Godot at 1.5 GB. Left at 512, recorded so the next
person does not re-derive it.

294 tests green. Seed 42 hashes `2a392483ac5b8158`; the hash *must* move here, because
unlike M-58 this is a deliberate behaviour change.


---

## M-62 — the terrain was a pancake with scratches on it (D13)

Reported plainly: *"the terrain generation is terrible; there should be hills and
mountains; the rivers should just be straight lines and should be filled with water;
remove the valleys."* Rendering the shipped seed with `scripts/snapshot3d.py` shows all
three complaints in one frame — a flat plain, thin dark cracks where the rivers are, and
no water anywhere.

### Why a map with 31 m of range renders flat

Three causes, and the ranking was not the obvious one.

| | contribution |
|---|---|
| **Zone aprons level-filled 91% of the map** | 60 m discs around twelve fixed points on a 360 x 240 m map: 135,600 m2 of level fill over 86,400 m2 of ground, overlapping |
| **All the amplitude was in short wavelengths** | octaves `((30,5),(14,1.8),(7,0.8),(3,0.3))` — 30 m is grain, not a hillside |
| Bilinear lattice interpolation | diamond creases along the cell diagonals |

Relief measured at the scale a hill actually is — the standard deviation of the landform
sampled every 24 m, averaged over 16 seeds:

| | hill-scale relief | apron coverage | peak relief |
|---|---:|---:|---:|
| before | **5.7 m** | 91% level-filled | 36.5 m |
| after | **9.4 m** | 29% *softened*, not levelled | 48.3 m |

(The shipped landform is the softer of the two measured below. The steeper one reached
10.6 m and 51.6 m, and gave those 1.2 m back for 14 points of the casualties a wheeled
unit can reach.)

The apron is now two stages against a heavily smoothed landform (sigma ~ 20 m): blend
toward the smooth field over ~26 m, level a small pad inside that. Because stage two
blends toward a field that is already smooth, the rim gradient it can introduce is
bounded by the smooth field's own gradient, which is what the four earlier attempts at
this (FIXES §2) each failed to arrange.

### Rivers

Straight, edge to edge, alternating axis, sited to clear the collection points by
`width x 1.3 + 18 m`, and **filled to their banks**. Water is now carried as a *surface
elevation plus a cap*, with depth derived as `min(level - ground, cap)` once at the end,
so every later step that raises ground — a ford, an apron, a road causeway — drains
itself. The old code stamped a flat `0.8 x depth` on whatever cells a random-walk path
touched.

| | before | after |
|---|---|---|
| centreline | random walk (`river_wobble`) | straight |
| cross-section | flat stamp, one depth | parabolic bed, 90th/10th percentile depth ratio > 3 |
| water in the channel | none drawn anywhere | drawn in `render3d.py` and in the dashboard |
| deepest water on the map | 0.86 m | 0.98 m (and provably <= `river_depth_m`) |

Two bugs found only because the water became visible: overlapping features produced a
**14 m lake out of a 0.38 m marsh** (a marsh levelled to its basin *median* on sloping
ground), and the final `z -= z.min()` renormalised the ground without the water surfaces,
which on one seed emptied every marsh on the map.

### The part that was not cosmetic

Grading roads across a map that now has hills on it exposed **five defects in the road
pass**, all of which had been invisible while the map was flat. Held-out seeds 42-57,
fraction of the map each chassis can reach from base:

| | before | after |
|---|---:|---:|
| wheeled | 43% | **63%** |
| tracked | 66% | **80%** |
| legged | 95% | **99%** |
| seeds with a chassis stranded below 20% | **4 of 16** | **0 of 16** |

Seeds 46, 48, 50 and 51 had wheeled units on 0-1% of the map *before this change*, and
seed 48 had tracked units on 0% as well — a quarter of the held-out set, with a third of
the swarm unable to leave the staging area, which the gate would have scored as a
strategy result. The five defects are listed in [FIXES.md §2](FIXES.md).

### Renderer

`render3d.py` drew no water at all, and its side splats were keyed on height above the
map floor rather than on the drop to the neighbouring cell — the same thing while the map
was flat, and on a 50 m hillside the reason the steep faces broke into horizontal stripes
with the bed showing through. Both fixed; build 0.05 s, render 0.27 s, 185k points at
1200 x 760.

`water` is now on the wire (one byte per cell, its own scale) and the dashboard builds a
translucent surface from it, checked by `tests/test_bridge_protocol.py`. A field that
decides where robots may go has to be visible or a wheeled robot stopping at a bank looks
like a bug.

**Valleys are gone** — the config knobs, the code and the test.

### What it costs the mission

Terrain has cost the mission before (FIXES §2: 18 → 4 rescues), so this was measured
against the *current working tree* with only the terrain change backed out — not against
HEAD, which lacks four other people's uncommitted work.

**Seed 42 alone says the change is a regression. Three seeds say it is not.** Headless,
`--seed`, victims rescued of 110:

| seed | old landform | new, steeper (octaves 1.0, hill 0.11–0.26) | new, softer (octaves 0.8, hill 0.10–0.22) |
|---|---:|---:|---:|
| 42 | **66** | 59 | 57 |
| 43 | 63 | 72 | **78** |
| 44 | 62 | 54 | 61 |
| mean | 63.7 | 61.7 | **65.3** |
| ground explored | 89.3% | 89.8% | 89.4% |
| robots lost | **44.3** | 53.3 | 56.7 |

Seed 42 is a *favourable* seed for the old map, not a representative one, and reading a
scenario change off it would have got this backwards twice over: first by concluding the
terrain costs 7 rescues, then by rejecting the softer landform that is actually the best
of the three. The spread within one variant is 21 rescues (57–78) against a 3.6-rescue
gap between the best and worst variant, so **three seeds is the minimum here and even
that is thin.** The D13 plateau — "rescues 40–45 across fifteen configurations" (M-60) —
was measured the same way and deserves the same scepticism.

The softer landform ships. It keeps ~2x the visible relief (hill-scale 5.5 → 10.1 m),
routes far better than the old map (wheeled detour x2.30 → x1.13, mean geodesic 406 →
224 m — the old map was a maze), and its rescue count is at worst a wash.

**The one real cost is robots lost: 44 → 57 per run, up on two seeds of three.** Harder
ground strands more machines. That is the honest price of terrain that does something,
and it is the number to watch if the scenario gets rebalanced.

### Why it is slope, not water

Fraction of passable ground each chassis is denied, seed 42:

| | old steep | old wet | new steep | new wet |
|---|---:|---:|---:|---:|
| wheeled | 17.1% | 4.8% | **29.1%** | 9.6% |
| tracked | 11.5% | 0.7% | **18.7%** | 6.2% |

Slope roughly doubled the denial; water contributed half as much. The mechanism is not
route length — routes got *shorter* — it is **which casualties the fast chassis can reach
at all**: 82 → 56 for wheeled, 96 → 79 for tracked, so more rescues fall to legged
carriers at 0.80x speed. Deep water (> 0.45 m, the tracked/legged boundary) went 1.14% →
7.62%: before this change the rivers were barely a barrier at all and the marsh tier was
doing all the work.

296 tests green. Seed 42 hashes `0fb6962fa7c0ce8e`, from `2a392483ac5b8158`. The hash
*must* move here: this is a deliberate change to the ground the swarm drives on.

Realtime factor is **2.42x**, up from the old landform's 2.32x — shorter routes cost less
to plan. Any run measured while other missions share the machine reads far lower (0.85x
with three of them running); that is contention, not the scenario, and RTF is not a number
to read off a loaded box.

## M-60a — correction: the terrain changed underneath M-60 (D13)

M-60's absolute numbers were measured against a `demo.yaml` that no longer exists. While
those runs were in flight the scenario was reworked — `hills: 20` added as a mid-scale
relief layer, `mountains` 3 → 4 with larger radii, `valleys` removed, rivers widened
7–13 m → 16–26 m with parabolic beds, `river_wobble` and `ditch_wobble` dropped, `ditches`
16 → 14 — along with `terrain.py`, `generator.py`, `scenario.py`, `bridge.py`, `main.gd`
and two test files.

**How it surfaced.** Two runs of what should have been the same configuration disagreed
(seed 42, `hivemind=False`: 61 rescued, then 60). Chasing that ruled out, in order: the
`carrier_reserve` knob (reverted `auction.py` entirely — same result), replacing `NavSet`
after construction (identical hashes), and the windowed wavefront (**480/480 fields
bit-identical to a whole-array reference across all four chassis**). Determinism was
confirmed intact — two headless runs at HEAD both hash `0fb6962fa7c0ce8e`. Only then did
`git status` show a dozen files nobody in this session had touched, timestamped mid-run.

**The lesson is procedural, not technical.** Every conclusion in this file assumes the
scenario is fixed for the duration of a measurement. Nothing enforces that, and an A/B
whose arms straddle a `demo.yaml` edit is not an A/B. A cheap guard would be to record
`demo.yaml`'s hash in the sweep output alongside the seed.

### Re-measured on the current terrain

The A/B was re-run from scratch, both arms on the scenario as it now stands:

| coarse edges | rescued | found | explored | robots lost | densest 6 × 6 m bin |
|---|---:|---:|---:|---:|---:|
| 8-connected | 52.25 | 76.50 | 82.2% | 65 | 34 |
| **fine-linked** | **66.50** | **86.50** | **90.0%** | 85 | **11** |

**Rescues +27%, discovery +13%, exploration +7.8 points, up on all four seeds**
(55→65, 62→70, 47→69, 45→62). The clump signature collapses 34 → 11, and on the two seeds
where the old terrain trapped robots worst — 44 and 45, densest bins of 49 and 58 — the
gain is largest (+22 and +17 rescues).

**M-60's conclusion is unchanged and its margin is larger.** What does not survive is its
absolute before/after column, its `2a392483ac5b8158` hash, and its realtime figures; the
current headless seed 42 is 57 rescued, 81 found, 82.8% explored, hash `0fb6962fa7c0ce8e`.
Losses rise 65 → 85, which is expected and worth watching: robots that are no longer held
against rock actually travel, and travelling robots reach the fire.


## M-63 — a smaller, shorter fire; and the fog lag is real but small (D13)

Two requests off a live run: make the fire smaller and make it go out faster. Both are
downstream of M-60 — once robots stopped being held against rock they actually travelled,
and travelling robots reach the fire, so losses had gone 65 → 85 on the same seeds.

### The change

| | before | after |
|---|---|---|
| growth | `0.225 m/s` | `0.20 m/s` |
| peak | 45.0 m at t=270 | **34.5 m at t=240** |
| peak area | — | **59% of former** |
| at the buzzer | still 16.9 m | **out at t=326** |

It still ignites at t=90 and still grows through the t=180 abandon beat, so the demo beat
in PLAN §4 is intact. The last ~90 s are now spent reclaiming ground rather than avoiding
fire.

### Result, four seeds, Tier 3 off

| | before | after |
|---|---:|---:|
| rescued | 66.50 | **69.50** |
| found | 86.50 | **89.00** |
| ground explored | 90.0% | 89.8% |
| **robots lost** | **85** | **58.5** |
| delivery | — | **78.1%** |

**Losses fall 31% and rescues rise 4.5%.** Gate: delivery 78.1% and discovery 81 on seed
42, both passing with the widest margin the project has had. Seed 45 remains an outlier at
117 losses against 34–46 elsewhere and is still unexplained.

### "The scouts explore everything, comms just doesn't count it"

Tested directly, because it is exactly the shape of defect this file keeps finding. It is
not what is happening:

| t | reported (shared map) | + still in private buffers | gap |
|---:|---:|---:|---:|
| 180 | 65.0% | 67.0% | **2.1** |
| 300 | 82.3% | 83.1% | 0.8 |
| 420 | 87.8% | 88.4% | 0.6 |

The gap peaks at 2.1 points and closes to 0.6: store-and-forward (M-37) does deliver
buffered ground, just late. **That lag is real and is what an operator sees** — a scout
walks dark ground and the fog does not lift until it reconnects, sometimes minutes later.

The one path that loses ground permanently is a robot dying while holding observations.
Measured: **0 of 46 losses died holding a buffer, 0 cells lost.**

**So the scouts are not reaching 100%.** 12.2% of reachable ground is never visited by
anyone — down from 26.8% before M-60, but not a comms failure. Nobody goes there. Where
that 12.2% sits is not yet measured; the probe was killed by memory pressure.

### The machine ran out of memory during this work

Swap reached **5,056 MB of 6,144 MB with ~65 MB of RAM free**, and several measurement
processes were SIGKILLed mid-run — including two whose partial output nearly became
findings. Partly self-inflicted (parallel sweeps at 3–4 workers), but this is R1 arriving
in practice, and it is worth knowing that the margin on this machine is thin enough that
*measuring* the demo can break it. Sweeps are down to 2 workers.

### Addendum — two dashboard readings, one right and one wrong

**"A ton of scouts in the unexplored space."** Correct, and it is the buffering lag seen
directly. Robots standing on cells the shared map calls dark: 47 at t=120, 42 at t=300,
10 at t=420 — and **35 of those 42 are out of contact**. They are looking at it; the map
does not have it yet.

The handful that are *in* contact and still on dark ground point at something worth
stating plainly for the runbook: **fog records what a camera saw, not where a robot has
been.** The camera is a forward 90° frustum, so a robot's own cell can legitimately go
unobserved. Inferred from the code path, not measured.

**"A lot of victims lying there without a carrier."** Not supported. Unclaimed cleared
casualties across a whole run: **1, 0, 0, 0, 0, 0**. At t=420 all 22 casualties in the
pipeline — 11 carried, 6 cleared, 5 buried — have a claim, with carriers at 1, 3, 4, 6, 8,
20 and 49 m.

What makes it look abandoned: pickup needs a carrier within **2.0 m** *and* empty-handed,
so a loaded carrier standing a metre away cannot take a second casualty. That specific
case is worth a look. The rest is approach time, and the marker change (M-59) is what made
the work-in-progress visible in the first place.

**And the unvisited 12.2% is across the rivers.** Of that ground, wheeled units can reach
30.6%, tracked 48.7%, rotor 84.4%, legged 100%. Water now covers 11.4% of the map at up to
0.98 m; wheeled reach only 59.7% of the map overall and tracked 73.3%. The unvisited ground
sits a median 299 m from base, in E8 (21% explored), F8 (26%), C7 (35%). It is not a comms
failure and not an allocation failure — half the chassis types physically cannot get there,
which is the direct consequence of widening the rivers to 16–26 m. `river_fords: 2 -> 3` is
the obvious lever and is untested.

---

## M-64 — the contact rings were 30x overcounted, and one colour (D13)

Started from a display complaint — every ring on the dashboard is the same amber — and
found two faults stacked, the first hiding the second.

**The rings were one-per-report, not one-per-casualty.** Several reports resolving onto
the same body is the tracker working as designed; `nodes/tasks.py` already dedups on
`r.victim` before offering work, but `bridge._contacts` did not. Seed 42, demo scenario:

| t (s) | resolved reports sent | distinct casualties | worst pile on one body |
|---|---|---|---|
| 15 | 312 | 17 | 46 |
| 30 | 475 | 22 | 75 |
| 45 | 663 | 25 | 92 |
| 60 | 817 | 26 | 103 |

Rings for a `CARRIED`/`CLEARED` casualty snap to `v.pos`, so those 103 were **exactly
coincident** — a z-fighting blob that read as one fat marker. The visible ring count
therefore tracked how often the swarm had looked at somebody, not how many people it had
found. It also put ~800 rows in a 10 Hz frame against the 64 KB
inbound-buffer ceiling that already cost a day once (M-12).

This retires the standing claim in `main.gd` that the surplus rings were "mostly phantoms
it has not investigated yet". Measured composition over a 190 s run, sampled every 10 s
(8,092 ring-samples): **carried 6,456 · surface 1,136 · unverified 263 · dug 135 ·
buried 102**. Unverified contacts are 3.3% of what was drawn. The surplus was duplicate
resolved reports, not phantoms.

**After deduping on `r.victim`,** first report wins in `tracker.reports` order (a list, so
determinism holds):

| t (s) | rings | unverified | buried | surface | dug | carried |
|---|---|---|---|---|---|---|
| 10 | 34 | 18 | 0 | 3 | 0 | 13 |
| 60 | 31 | 15 | 0 | 3 | 0 | 13 |
| 110 | 40 | 16 | 2 | 9 | 1 | 12 |
| 130 | 33 | 8 | 4 | 9 | 2 | 10 |

~30 rings instead of ~500, and all five states are present at once, which is what makes
colour-coding worth anything. Ring colour is now `CONTACT_CODE` — unverified / buried /
surface / dug / carried — instead of resolved-vs-not. The fields it reads (`v.state`,
`v.buried`, `v.debris_remaining`) are the same ones `nodes/tasks.py` already reads on the
same victims to choose between a dig and an extract, so this is not new ground truth on
screen; it is the swarm's own working set, drawn.

The palette was checked offline against a real t=130 frame before it went into GDScript,
which took a throwaway splat pass over `_contacts` output — **`viz/render3d.py` does not
draw contact rings**, so the usual snapshot route cannot see them. That gap is worth
closing before anything else about the markers changes. The one thing the check caught:
`carried` as green was indistinguishable from the gripper robot inside the ring, since
`LANE_COLORS[2]` is already green. It is teal now.

Not measured: the frame-size saving, and whether ~30 markers changes anything about
Godot's `MultiMeshInstance2D` threshold. Rescue rate is untouched — this is display only.


## M-64 — comms gates exploration, and it gates allocation more than reporting (D13)

Control condition, requested from a live run: force `in_comms` True for every robot after
every tick, so perception writes straight to the shared map. Not shippable — it deletes
the relay lane's reason to exist — but it is the exact test of "is comms what keeps the
map dark?"

| seed 42 | rescued | found | ground explored | of *reachable* ground | lost |
|---|---:|---:|---:|---:|---:|
| normal | 60 | 81 | 85.8% | 87.8% | 46 |
| **always connected** | **60** | **93** | **94.3%** | **95.8%** | 53 |

| | darkest three sectors |
|---|---|
| normal | E8 21%, F8 26%, C7 35% |
| always connected | C7 53%, F7 64%, C8 69% |

**Exploration rises 8.5 points, to within 3 points of the 97% terrain ceiling, and
discovery rises 15%.** The far sectors stop being outliers: the worst sector goes from 21%
to 53%, and coverage becomes even instead of trailing off with distance.

### The prediction was wrong, and the way it was wrong is the finding

Stated before running it: *"explored rises a few points, not to 97% — the reported-vs-
buffered gap closes to 0.6 points by mission end, so the buffering is already being
delivered rather than lost."* That reasoning was sound about the wrong quantity.

The instantaneous buffer **is** small (0.6–2.1 points, M-63). But an out-of-comms robot is
not merely reporting late — **it cannot be given work at all.** `AuctionNode.step` builds
`free` as `ex.free_mask(world) & world.in_comms & alive`, so a robot past the network edge
cannot be allocated, redirected, or included in coordinated coverage. It has left the
swarm. Forcing comms on does not just flush buffers faster; it changes *where the swarm
goes*, which is why coverage evens out rather than simply catching up.

**Measuring the buffer measured the symptom. The constraint is allocation.**

### Rescues do not move: 60 either way

Twelve more casualties found, zero more delivered — M-50's deadline, unchanged. Anything
found after ~t=300 cannot be carried home before the clock stops.

This resolves an oddity in M-49, where relay range bought comms 83% → 94% and left rescues
flat. Same effect from the other end, and the split is now clean enough to design against:

- **comms → exploration and discovery.** Worth it for how the demo *looks*, and for the
  discovery number.
- **delivery time → rescues.** Comms cannot touch it.

Both numbers are on the scorecard, and they answer to different levers. Anyone tuning one
while quoting the other will be disappointed.


## M-65 — acting on M-64: more relays, more range, and idle relays that roam (D13)

M-64 established that comms gates exploration *and allocation* — an out-of-contact robot
cannot be given work at all. Three changes follow from it, made together and measured
together.

1. **Relays 96 -> 128**, taken from scouts (203 -> 171). M-49 had found 96 rescue-optimal
   and 128 slightly worse, but that was optimising the number comms cannot move.
2. **Relay radius 38 -> 46 m.** M-49 measured 46 m at 92% in comms against 38 m at 83%,
   and held it at 38 because rescues were flat. 54 m was rejected then and now: at that
   range the network blankets the map and the relay lane stops posing a placement
   decision at all.
3. **An idle relay is a moving relay.** Relays already extend the network wherever they
   stand — the comms graph is built from every living antenna, not only those holding a
   post — but an idle one in contact was given no goal and simply stopped, ~29 of 96 of
   them by the end of a mission. They now walk toward the nearest robot that is *out* of
   contact and become the link.

### Result, four seeds, Tier 3 off

| | before | after |
|---|---:|---:|
| rescued | 69.50 | **76.00** |
| found | 89.00 | **96.00** |
| ground explored | 89.76% | **95.43%** |
| comms reach | 356 m | **398 m** |
| delivery | 78.1% | 79.2% |
| robots lost | 58.5 | 57.25 |

**95.43% explored is what the always-connected control reached (94.3%, M-64)** — these
changes capture essentially all of the headroom that removing comms entirely would give.
Seeds 43 and 44 finish at 98.6% and 97.7%.

**And rescues rose 9.4%, which M-49 predicted they would not.** That prediction was made
before the roaming behaviour existed: recovering a stranded robot does not merely deliver
its report, it returns a robot the auction can *task*. Only 47-52 of 126-128 relays hold a
post at the end — the rest are roaming, which is the lane finally being fully employed.

### A third river ford: measured, rejected

| fords | rescued | found | explored | wheeled reach |
|---:|---:|---:|---:|---:|
| **2** | **76.00** | **96.00** | 95.4% | 63.0% |
| 3 | 72.75 | 92.75 | 95.7% | 65.1% |

+2.1 points of wheeled reach, +0.3 of exploration, **-3.25 rescues**, down on 3 of 4
seeds. The far side was never a fords problem; it was a comms problem, and with comms
fixed the extra crossing has nothing left to buy. Note the arms are mildly confounded —
carving a ford changes map generation — but the direction is consistent. Held at 2.

### Where the mission now stands

    rescued 76.0 / 110 (69%)   found 96.0 (87%)   explored 95.4%   delivery 79.2%

Against the start of D13: 44.00 rescued, 70.50 found, 75.7% explored. Gate passes on both
bars with the widest margin the project has recorded.

**Tier 3 is now clearly negative**: 76.00 rescued with it silent against 70.25 with the
scripted rung, on the same four seeds. That is a larger gap than M-42 or M-45 measured and
it is worth re-gating before the hivemind is described to anyone as helping.

---

## M-66 — particle effects: the dig lane finally has something to show (D13)

Digging was the one stage of the rescue chain with no motion in it. A scoop robot
clearing a slab and a scoop robot sitting idle are the same 1.4 m box at the same
coordinates for ~20 s, so the lane that unburies 32 of the 80 buried casualties was
legible only as a `victim_cleared` line in the event feed. Same class of problem as M-64
(rings that did not track the rescue) and the sector wash: a mechanism that works and
cannot be seen.

**The dust is driven by the simulator's own excavation predicate, not by proximity.**
`world.digging` is rebuilt inside `_update_victims` from the exact mask that decrements
`debris_remaining`, so a plume exists if and only if debris is moving. Deriving it
dashboard-side from "a digger is near a buried ring" was the obvious cheaper option and
it would have drifted the first time `REACH_DIG` or a lane index changed — silently, in
the direction of dust over holes nobody is digging.

### What it costs

Measured over demo/seed 42 to t=320, with the field (`swarmmind/viz/particles.py`) driven
at 60 Hz alongside the mission and bursts fired off the real `/swarm/events` stream:

| | |
|---|---:|
| concurrent dig sites, max | **4** |
| dig sites while digging, mean | 1.46 |
| ticks with a dig in progress | 2,782 |
| live particles, max | **199 / 448** |
| live particles, p99 | 147 |
| live particles, mean | 37.8 |
| frames at the pool cap | **0** |
| `state` frame without `digs` | 20,193 B |
| `state` frame with `digs` | 20,216 B (max 20,622; cap 65,535) |

**+23 bytes a frame**, and the pool never came within 2x of its ceiling. God-view embers
were not running in this measurement and add at most ~57 more (26/s at ~2.2 s of life),
so the realistic worst case is ~256 of 448.

Everything is one `MultiMeshInstance3D` — one draw call, `instance_count` fixed at 448
and allocated once, the live count carried on `visible_instance_count`, and a
`custom_aabb` over the map so Godot does not recompute the MultiMesh bounds from hundreds
of changing instance transforms every frame. Cubes rather than billboarded quads: a quad
is the prettier sprite and also the one that disappears when an operator-driven camera
catches it edge-on. 448 cubes is 5,376 triangles against the 5.4M the scattered props
already submit (M-53) — the cost of this system is its update loop, not its geometry.

### Determinism: unaffected, checked rather than assumed

`_update_victims` now appends to a list. A/B on demo/seed 42 to t=150, once with the
collection live and once with the append stubbed out in-process:

    with digging     38942ef960ab4731
    without digging  38942ef960ab4731

Identical, as it must be — nothing above the append reads it and no RNG stream moves.

### Two effects measured out rather than deferred

- **Dust trails behind moving robots.** 768 robots at even 2 particles/s each is 1,536
  spawns a second against a 448 pool: it would evict every dig plume three times over
  every second, and the thing the effect exists to show would be the first casualty.
- **A flash on `robot_out_of_comms` / `robot_reconnected`.** The swarm makes ~3.7
  connect/disconnect transitions a second (M-48). The dashboard already holds comms
  colour for 1 s to stop that reading as a shimmer; adding a particle burst to the same
  event would undo exactly that fix.

### Verified offline first

Godot cannot be run from this environment, so `swarmmind/viz/particles.py` holds the
model and `main.gd`'s `FX_KINDS` is a port of it —
`scripts/snapshot3d.py --scenario demo --at 120 --fx` renders the field to PNG from
beside each excavation, and `tests/test_bridge_protocol.py` compares the two tables
number by number. The first version of the dust was invisible for a measurable reason:
at `(0.72, 0.66, 0.55)` it sat inside the range `DISPLAY_RUBBLE` already covers under
lighting. Pulverised concrete is much paler than the rubble it comes from, and
`(0.87, 0.83, 0.74)` separates.


## M-66 — five dashboard observations, three fixes, one of them mine to undo (D13)

Five things reported off a live run. Measured first; two turned out to be correct
behaviour, three were real.

### 1. Loaded carriers standing still — a real bug, +2.5 rescues

At t=330 on seed 42, **9 of 17 carriers holding a casualty had moved less than 2 m in
30 s** — all in contact, batteries 0.56-0.71, a collection point 13-55 m away.

`_nearest_zone` picked the **straight-line** nearest zone with no regard for whether that
chassis could reach it. Rivers are now 16-26 m wide, so a zone 13 m away can be across
water: the flow field returns UNREACHABLE, Tier 1 has nothing to descend, and the carrier
holds the casualty for the rest of the mission. It is not even released to try again,
because `_stalled` treats an UNREACHABLE reading as *no evidence* rather than *no
progress* (M-55) — right for the transient case the downsample produces, wrong for a goal
that is permanently unreachable.

Now it picks the nearest zone reachable **on that chassis's own field**, falling back to
straight-line if none is. Isolated on four seeds: **+2.5 rescues** (69.75 -> 72.25).

### 2. Digging now takes time, and it is free up to a point

`debris_remaining` is 1.0 and `clear_rate` was 0.25 **per digger with no cap**, so one
digger took 4 s and a crowd of eight took half a second. "Buried" was a label, not a stage.

Capped at `MAX_DIGGERS = 3` working one casualty, and swept:

| clear_rate | 1 digger | 3 diggers | rescued | found |
|---:|---:|---:|---:|---:|
| 0.25 (old) | 4 s | 1.3 s | 70.50 | 95.50 |
| **0.12** | **8 s** | **2.8 s** | **70.75** | 92.00 |
| 0.07 | 14 s | 4.8 s | 67.75 | 91.75 |

**0.12 doubles the dig time for nothing** — 70.75 against 70.50 — and 0.07 is where it
starts costing. Shipped at 0.12.

### 3. "Check every corner" — half of it worked, and the half I designed did not

Two changes went in together, and they had to be separated:

- **`sweep_below` 0.55 -> 0.90.** Sectors keep producing a sweep point until nearly
  finished. **66.00 / 68.00 / 70.75 rescued at 0.55 / 0.75 / 0.90.** Kept.
- **Sweep points aimed at the centroid of a sector's unexplored cells** rather than its
  geometric centre. The reasoning was good — a half-explored sector has an explored
  centre, and `_state` retires an explore task the instant its target reads explored, so
  the sweep point died on the tick it was awarded. It measures **-1.0 rescues, -1.25
  found, -0.6 points explored**. Reverted, default off.

  The likely reason: the centroid of two separate dark corners is the *explored middle
  between them*, which is a worse target than the sector centre rather than a better one.

Isolating the two was necessary because together they cancelled: both-off 69.75, dark-only
68.75, zones-only 72.25, both-on 70.75. **Three changes went in at once and the sweep that
followed read as an 8-rescue regression** — which is the discipline this file keeps
recording and which I did not follow.

### 4. Robots on unexplored ground: correct behaviour

Six of 503 in-contact robots stand on cells the map calls dark, and **all six are rotors**.
Rotors are blind while airborne by design — fog lifts and casualties are detected only once
one is on the ground. Not a defect.

### 5. Casualties on explored ground with no marker: the perception gap

16 casualties finish on explored ground having **never raised a single report**, 31% of
them buried. This is M-56: the fog marks a cell when a camera pixel covers it, while
*finding a person there* needs them inside a 90° arc, close, unoccluded, and — if buried —
very close. **Explored is not examined**, and the dashboard cannot show a contact for a
casualty nobody has detected.

### Where it lands

    rescued 72.25 / 110   found 93.25   explored 94.4%   delivery 77.5%   gate PASS


## M-67 — 72 relays walking to the same point, and what removing the pile cost (D14)

Reported from a live run: *"there are so many purples, just clumped up here"*, in dark
ground that was not lighting up. Purple is the antenna lane.

### The clump was mine

M-65's roaming rule asked **each idle relay independently** for the nearest out-of-contact
robot. That is the same robot for most of them. Measured on seed 42:

| t | idle relays | roaming | distinct goals | **largest pile** |
|---:|---:|---:|---:|---:|
| 120 | 79 | 79 | 3 | **72** |
| 240 | 74 | 74 | 4 | 51 |
| 300 | 76 | 76 | 3 | 44 |

**Seventy-two relays walking to one point.** A pile of relays is worth exactly one relay,
and the chain the lane exists to build is a line, not a crowd.

Fixed two ways: targets are now assigned greedily and **exclusively** — one relay per
cut-off robot, in robot-id order for determinism — and the target is a point `0.8 x
relay_radius` *short* of the stranded robot rather than the robot itself, because a relay
that walks all the way to a cut-off robot is simply cut off too. Largest pile **72 -> 1**,
distinct goals 3 -> 57, roamers standing on dark ground 5 -> 0.

### And it cost exploration, consistently

| config | rescued | found | **explored** | delivery |
|---|---:|---:|---:|---:|
| converging (the pile) | 72.25 | 93.25 | **94.35%** | 77.5% |
| one relay per target | 72.50 | 92.75 | 91.61% | 78.2% |
| + surplus relays repositioning | **74.00** | 93.25 | 89.88% | **79.4%** |

Exploration falls on all four seeds (−2.9, −1.4, −1.8, −4.9) while rescues are noise
(+8, −7, +1, −1). **The pile was doing exploration work by accident**: seventy relays
marching across the map marked ground the whole way. Removing it removed that, and no
amount of it was intentional.

The attempt to give it back failed. Surplus idle relays — 126 alive against ~50 stranded,
so most of the lane has nobody to bridge — were sent round-robin to the least-explored
sectors. Rescues rose to 74.00 and exploration fell *further*, to 89.88%. The reason is
the one `_dark_centroid` already taught in M-66: **a sector centre is usually ground the
swarm has already covered**, so aiming at it moves robots without revealing anything.

Two targeting schemes tried, both underperforming, and the honest summary is that giving
the relay lane a coverage role conflicts with what it is for — `_INFRASTRUCTURE_EXEMPT`
bars relays from search work precisely because a relay that goes looking abandons the
chain behind it (M-9).

### The trade, stated plainly

Removing the pile costs **~4.5 points of exploration** and gains **~1.75 rescues** and
1.9 points of delivery. Shipped that way: rescues are what the gate counts, a 72-robot
pile is indefensible on a dashboard, and the exploration number was being earned by a
behaviour nobody designed. But if the *look* of a cleared map matters more for the demo
than the rescue count, reverting `roaming_relays` entirely is the measured-best choice for
that number, and it is one flag.


## M-68 — giving the antenna lane standing orders (D14)

M-67 left the relay lane with no role model: it was barred from search work, held a post
when the auction gave it one, and otherwise ran a patch. Every fix to it had been a patch
on the patch. Replaced with an explicit priority ladder in `_relay_orders`:

1. **Repair** — somebody is cut off *now*: become the link, stopping `0.8 x relay_radius`
   short so the relay bridges rather than joining the blackout.
2. **Reinforce** — somebody's only link is past `FRAGILE_AT` (0.80) of its range: insert a
   hop before it breaks. **This is the half the lane never had.** It acted only once
   contact was already lost, which is the one moment the robot it must reach can no
   longer be coordinated with.
3. **Explore** — nothing to mend: fall through to `_drift_targets`, which already aims at
   dark ground and already creeps relays instead of marching them.

Each rung consumes relays exclusively, nearest-first, in robot-id order (invariant #6).

### The lane at work (seed 42, tier 3 off)

| t | on post | repair | reinforce | explore | **standing still** | fragile links |
|---:|---:|---:|---:|---:|---:|---:|
| 60 | 48 | 14 | 4 | 62 | **0** | 4 |
| 180 | 50 | 14 | 7 | 56 | **0** | 7 |
| 300 | 48 | 1 | 2 | 74 | **0** | 2 |
| 420 | 48 | 6 | 6 | 64 | **0** | 6 |

**No relay stands still at any sample** — it was ~29 of 96 finishing a mission parked.
Reinforce equals the fragile-link count on every row, so rung 2 is never starved: the
lane covers every marginal link and still has 43-74 robots left over to search with.

### Mission result, against the three configs before it

| config | rescued | found | explored | delivery | losses |
|---|---:|---:|---:|---:|---:|
| the pile (M-67) | 72.25 | 93.25 | **94.35%** | 77.5% | — |
| one relay per target | 72.50 | 92.75 | 91.61% | 78.2% | — |
| + sector-centre repositioning | 74.00 | 93.25 | 89.88% | **79.4%** | 52.25 |
| **standing orders** | **74.25** | **95.25** | 91.11% | 78.0% | **43.75** |

Best rescued and best found of any config measured, losses down 16% per mission
(52.25 -> 43.75), and the seed-45 loss outlier 117 -> 94 — consistent with rung 2 doing
what it claims, since a robot that never drops out never becomes a casualty of its own
isolation. Mean time-to-rescue 182.1 -> 177.8 s.

### The exploration number, and why it is the wrong one to optimise

Explored is still **3.2 points below the pile** — relays creep in drift by design, and the
pile marched them the full width of the map. But the pile **found two fewer casualties
while revealing more ground**. Ground revealed in a straight line between a relay and the
one point seventy-one others were also walking to is ground nobody chose; sector coverage
finds people, and a higher explored fraction bought from a marching column does not.

Explored % is a *proxy* the swarm was accidentally gaming. `found` and `rescued` are the
things it is a proxy for, and both are higher here. Keeping the ladder.


## M-69 — "relays are moving posts" is true for the robot and false for the planner (D14)

`relay_posts` builds its anchor list from base plus the targets of *assigned* relay tasks,
and deliberately excludes idle relays (the spawn-wall note in that function). But after
M-68 most of the lane is idle *and out in the field*, extending the network wherever it
stands — `World._update_comms` builds the graph from every living antenna, post or not.
So the planner was blind to roughly 60% of its own lane and announcing posts on ground
already covered.

Fixed the obvious way: every idle, in-contact relay **more than `base_radius` from base**
becomes an anchor. The guard is exactly the spawn wall's failure mode — 128 relays inside
40 m of base — excluded by construction.

| | rescued | found | explored | delivery | relays on post |
|---|---:|---:|---:|---:|---:|
| standing orders (M-68) | **74.25** | **95.25** | 91.11% | 78.0% | ~49 |
| + idle relays as anchors | 72.75 | 93.25 | 90.92% | 78.0% | ~42 |

**Off. −1.5 rescued, −2.0 found.** A post is suppressed within `0.75 x relay_radius` of an
anchor, so each drifting relay silenced a post — and then walked away, because drifting is
what rung 3 told it to do. The chain was planned around a node that leaves. Seven fewer
relays on station per mission is the whole effect.

The lesson is a distinction, not a tuning value: **a moving relay is a link, but not a
commitment.** The planner cannot use a position the robot is about to abandon. The robot
can, because it is the one deciding whether to abandon it — which is what rung 0 does
(M-70). Kept switchable: at a larger `relay_radius` the suppression radius may stop
overlapping the drift, and the sign could flip.


## M-70 — rung 0: a relay that is somebody's only link stands still (D14)

M-69 established that the *planner* cannot use a moving relay's position. The robot can,
because it is the one deciding whether to move. `_relay_orders` gains a rung above every
other: a relay covering a robot that **no other relay and not base can reach** holds its
ground and is that robot's post, auction or no auction.

Computed exactly, not by proxy: for every in-contact non-relay robot outside `base_radius`,
count the connected relays within `relay_radius`; where that count is 1, the single relay
is load-bearing. Cheap enough to run per goal rebuild (~380 x 126).

| config | rescued | per seed | found | explored | delivery | lost |
|---|---:|---|---:|---:|---:|---:|
| M-68 standing orders | 74.25 | 68 / 80 / 79 / 70 | 95.25 | 91.11% | 78.0% | **43.75** |
| M-69 + mobile anchors | 72.75 | 70 / 80 / 77 / 64 | 93.25 | 90.92% | 78.0% | 40.00 |
| **M-70 + rung 0 hold** | **75.50** | **69 / 80 / 82 / 71** | **95.75** | **91.27%** | **78.9%** | 47.00 |

**Up on three seeds, level on the fourth, down on none** — the first change in this
sequence whose rescue gain is not inside seed noise. Best measured value of every headline
number: rescued, found, explored and delivery all peak here.

The cost is honest and unexplained: **losses 43.75 -> 47.00**, +3.25 robots a mission. The
plausible mechanism is that holding a relay in place holds it in whatever is coming for it
— rung 0 outranks everything except the out-of-contact reflex, and unlike a post it carries
no hazard check. `_state` retires a *relay task* whose post reads `hazard_known`; rung 0 has
no equivalent, and it should. Not fixed here because it is unmeasured, and three changes at
once is how M-60 went wrong.

### Rung 0 is a scalpel, not a behaviour

Sampling the lane every 60 s showed `hold` = 0 at most samples and I nearly recorded the
mechanism as inert. Tallying **every** rebuild instead of snapshots (seed 42, 2098
rebuilds of the relay orders):

| | |
|---|---:|
| rebuilds with at least one relay holding | **836 of 2098 (39.8%)** |
| share of all robot-assignments that are holds | 1.9% (914 of ~49k) |
| bridging / shoring | 82.1% / 16.0% |
| **distinct relays that ever held** | **10** |
| longest single holds, in rebuilds | 315, 172, 129, 75, 71 |

Ten relays out of 128, one to three at a time, each standing for a long time. That is the
shape the mechanism was aimed at -- a cut vertex in the comms graph is rare and matters
enormously -- and it is why +1.25 rescues comes out of 1.9% of assignments: the relay that
holds is keeping a whole subtree of the swarm reachable, and every robot in that subtree
stays employable.

The near-miss is the lesson worth keeping: **a rare high-value event is invisible to
interval sampling.** Four snapshots said the feature never fired; the mission says it fired
in two rebuilds out of five.

### The distinction this pair of measurements bought

"Relays are moving relay posts" is true, and which half of the system may act on it decides
whether it helps:

- **The planner may not.** A drifting relay's position is a link now and a gap in ten
  seconds; planning around it suppresses the post that would have covered the gap (M-69,
  -1.5 rescued).
- **The robot may.** It knows whether it is about to leave, so it can convert itself from a
  passer-by into a post exactly when someone is depending on it (M-70, +1.25 rescued).


## M-71 — a fifth of the swarm was flying blind (D14)

Reported from a live run: robots standing on dark ground, in contact, and the ground
staying dark. *"This shouldn't be possible."* It was, for two independent reasons.

### 1. The rotor lane went blind when idle robots got goals

`Mission.tick` set `w.airborne = rotor & (goal_id >= 0) & ~arrived`, and `_perceive`
excludes airborne robots outright — a rotor is a spotter that only looks once it lands.
That was survivable while idle robots had **no** goal: a rotor with nothing to do sat
down and looked. `idle_explore` (M-58) gave every idle robot a goal, and took the rotor
lane's eyes with it. Nobody noticed because no test asserts a rotor ever perceives.

Measured on seed 42 — 103 of 512 robots are rotors (20.1%):

| | before | after |
|---|---:|---:|
| mean share of life airborne | 82.9% | 81.7% |
| median | 98.6% | 98.0% |
| airborne >99% of their life | 45 of 103 | 42 of 103 |
| **airborne ticks over UNEXPLORED ground** | **188,664 (27.0%)** | **760 (0.1%)** |

A rotor now lands when the cell beneath it is unexplored and flies when it is not. The
lane keeps its 2x dash across known ground — the overall airborne fraction barely moves —
and pays it back exactly where the map still needs reading. An independent trace of the
fog path found that at t=150, **35 of 35 robots standing on unexplored ground were rotors,
21 of them in comms**; zero were ground robots.

### 2. A robot's own cell is never in its own camera frame

`CameraRig.near_clip` is `2.2 x world.radius.max()` = **1.027 m against a 1.0 m cell**.
Measured: **0 of 512 robots see their own cell.** Walking robots are covered by accident —
they saw the cell from a metre back on the way in — but anything arriving *without* a
ground approach (a rotor setting down, a robot at spawn) stands on ground it has never
revealed. `World.mark_underfoot` now stamps it, buffering for out-of-comms robots exactly
as `mark_seen` does. This is contact, not remote sensing, and does not touch invariant #3.

| seed 42 | before | after |
|---|---:|---:|
| cells stood on **in comms**, still unexplored | **746** | **2** |
| explored share of passable ground | 89.68% | **93.73%** |

### Mission result

| config | rescued | found | **explored** | delivery | lost |
|---|---:|---:|---:|---:|---:|
| M-70 rung 0 | 75.50 | 95.75 | 91.27% | 78.9% | 47.00 |
| **M-71 rotors land + underfoot** | 75.25 | 95.50 | **95.25%** | 78.8% | **43.25** |

Explored up on **all four seeds** (+6.0, +5.2, +3.4, +1.2), rescues and delivery flat,
losses down 3.75. Against the structural ceiling of 99.32% (406 cells of the demo map have
no standable, base-connected cell within max sensor radius), 95.25% leaves ~4 points, of
which 0.7 cannot be taken.

This also recovers, several times over, the 3.2 points of exploration that removing the
relay pile cost in M-67 — and it recovers them from ground the swarm chose to look at
rather than ground a marching column happened to cross.

### Unexplained, and worth a proper measurement: tier 3 flipped sign

Tier 3 has measured negative all along. On this build it is **+3.25 rescued** (78.50 on,
75.25 off) and +4.4 found. Per-seed it is +6 / -9 / +5 / +11 — wide enough that four seeds
cannot settle it. A plausible mechanism is that directives steering exploration produced
no reveal while a fifth of the swarm could not see, so the strategy layer was being scored
on a search it could not affect. **Not a claim, a lead.** Re-run on the held-out seeds
before anything is decided about `hivemind=True` for the demo.


## M-71 — the range trust window gates two things, and only one was reasoned about (D14)

Why casualties are missed, attributed per gate rather than per symptom. M-56 measured
dwell and named it the lever; this instruments the tracker's own promotion arithmetic
against every detection that landed within `merge_radius` of a real casualty, so each miss
is charged to the specific gate that rejected it.

### Seed 44, 15 missed casualties

| n | gate it failed | detections it got | closest look |
|---:|---|---:|---:|
| 4 | detector never fired at all (3 buried) | 0 | — |
| 2 | too far **and** too few views | 11.5 | 5.0 m |
| 3 | never seen close enough (`best_conf` < `min_conf`) | 18.7 | 4.4 m |
| 2 | confident, but < 3 parallax views | 17.0 | 2.4 m |
| 4 | promotable — pruned at 20 s or never investigated | 40.2 | 2.2 m |

For contrast the 95 **found** casualties averaged 334.8 detections, `best_conf` 0.96, and a
closest sighting of 1.2 m.

**11 of 15 misses produced detections — 11 to 40 of them — and the tracker discarded
them.** Only 4 are camera failures. The loss is not upstream at the pixel, it is at the
gates.

### The cause: one zero doing two jobs

`_range_trust` returns exactly `0.0` past `trust_far`, and `_observe` uses that value
**both** as the confirmation weight and as `w` in the confidence-weighted position mean.
So a sighting beyond 5 m was not weak evidence — it was discarded entirely. It could not
corroborate, and it could not sharpen where the report thought the casualty was. Five of
the fifteen misses had a closest approach of 4.4-5.0 m: seen 11 to 40 times, every look
scoring zero.

The original comment justified 5 m from M-7's precision curve (0.99 at 3.5 m, 0.05 at
6.5 m). That reasoning is correct about confirmation and silent about localisation.

### `trust_far` 5 -> 9, three seeds, Tier 3 off

| | rescued | found | explored | reports |
|---|---:|---:|---:|---:|
| baseline | 65.5 (42/45) · 73 (44) | 88.0 | 95.9% | 4,864 |
| **trust_far = 9** | **71.0 (42/45) · 79 (44)** | **92.0** | **96.6%** | **4,437-5,052** |

**+5, +6, +6 rescues on seeds 42, 44, 45.** Exploration flat to +0.7 points, and **11%
fewer reports** — distant looks merge duplicates and sharpen positions, so the swarm makes
fewer investigate trips and more of them pay out. Every axis improves; this is not a
recall-for-precision trade.

### Two knobs swept alongside it, both rejected

| arm | rescued | found | explored | confirmed reports |
|---|---:|---:|---:|---:|
| baseline | 73 | 95 | 95.9% | 3,564 |
| trust_far = 9 | **79** | 96 | **96.6%** | 3,185 |
| min_views 3 -> 2 | 75 | **100** | 94.6% | 8,691 |
| stale_s 20 -> 60 | 68 | 90 | 93.5% | 3,387 |

`min_views = 2` buys the most recall and costs rescues — on seeds 42 and 45 it is **-8
against `trust_far` alone**, and it 2.4x's the confirmed-report count, which is investigate
trips the swarm cannot afford. `stale_s = 60` is worse on every axis. **Do not reach for
either next.** A bundle of all three measured 100 found / 71 rescued — worse than
`trust_far` alone, because `stale_s` was in it.

### The arm that was not shipped, and probably should be revisited

| arm | found | rescued | explored | confirmed |
|---|---:|---:|---:|---:|
| baseline | 95 | 73 | 95.9% | 3,564 |
| targeted bundle | 100 | 71 | 92.8% | 9,296 |
| **wide open** | **102** | **81** | 92.2% | **31,813** |

`trust_far` 14, `min_conf` 0.08, `min_views` 1, `parallax` 1.0, `stale_s` 120, detector
`min_fill` 0.18 / `max_wide` 0.85. **+7 found and +8 rescued on seed 44**, which inverts
the standing assumption that loosening perception trades recall against mission
performance. It does not: confirmation is what generates `investigate` tasks, investigating
is what resolves a contact into a *found* casualty, and confirming sooner moves discovery
earlier — which M-50 established is the only thing delivery responds to.

Not shipped: one seed, and 31,813 reports run through an O(detections x reports) merge
that `DemoSim` must absorb at wall-clock 20 Hz. Worth validating properly before demo day.

## M-72 — every idle robot now has somewhere to be, and four ways to aim one at nowhere (D14)

`SkillExecutor.goals()` ended its no-assignment branch with `else: continue` — no goal at
all. The auction can only employ as many robots as it has tasks and task supply is bounded
by design, so at 512 robots there are always far more free robots than tasks: **84-131
unassigned at every sample on seed 42**. Those robots stood still.

Replaced with a drift fallback (nearest under-explored sector, shared targets so Tier 1
still builds one flow field per goal) plus a base recharge pad for idle low-battery robots.
`no_goal` went to **0 at every sample**. It also did nothing for the mission — and the
reason is four separate defects in the new code, each of which aims a robot somewhere
useless.

### The four, and what each was worth

| defect | measured | after |
|---|---:|---:|
| aim point = centroid of dark cells, lands **inside** the rock they ring | 22% of targets impassable | 0 |
| same centroid lands on ground already **explored** | 22% | 0 |
| components labelled on the **fine** grid, not the 4x coarse nav grid | — | matches `NavFields` |
| relay **creep** point (`pos + unit x 0.8r`) never validated at all | never checked | every one |
| **robots aimed at ground their chassis cannot reach** | **~30 of 84 idle** | **0** |

The second row is M-57's finding reintroduced one function later. That entry records
`frontier_targets` failing because *"a boundary bin is a curve, and the mean of a curve
lands on its concave side, which is the explored side"* — and this took the mean of a
sector's dark cells and put it back. **Do not aim at a centroid of a set; aim at a member
of it.** The result is then guaranteed passable *and* unexplored by construction.

The fix for reachability is connected components per chassis on the coarse grid, built
once (terrain never changes) and answered by array lookup. Distance fields would be
correct and cost 22.4 ms each (M-4) x 48 targets x 4 chassis per refresh.

### Four seeds, against M-68's standing orders

| | rescued | found | explored | lost |
|---|---:|---:|---:|---:|
| M-68 standing orders | 74.25 | 95.25 | 91.11% | 43.75 |
| centroid fix only | 73.75 | 95.00 | 92.21% | 48.75 |
| **+ reachability + creep validation** | **75.25** | **95.50** | **95.25%** | **43.25** |

**+4.14 points of exploration**, essentially the ceiling M-64's always-connected control
established (94.3%); seeds 43 and 44 finish at 98.4% and 97.7%. Rescues +1.0 is inside
noise at four seeds — the exploration number is the result here.

### What "idle" actually was

Stationary robots on seed 42 at t=300, by what they were doing:

    stationary 21 of 503 alive
      relay_post  20      holding a link, motionless BY DESIGN
      dig          1
      idle         0
      stuck        0

Before the drift fixes this was ~80 stationary, of which ~42 were genuinely stuck. **The
stuck population is now zero, and every remaining still robot is working.**

### The dashboard could not have told you any of that

`bridge.py::_state` took `executor` as a parameter and never read it. The wire carried
seven columns and none of `assignment`, `goal_id` or `reason`. A relay holding the link a
dozen robots report through drew as the same still box as one wedged against a rock.

Now eight columns, with an `ACTIVITY` table in `contracts/schemas.py` mirrored in `main.gd`
and cross-checked by `test_bridge_protocol.py`, the same guard `EVENT_COLOURS` and
`FX_KINDS` already have. Idle tints toward red, carrying toward the `victim_rescued` green
so the map and the feed agree; the follow-cam names the activity in words.

### Redundant casualty assignment, and why two seeds lied

`claimed_v` treated a claim as boolean, so a digger that stalled kept its casualty out of
task generation for the rest of the mission — nobody else was ever offered it. Replaced
with a count topped up to `carriers_per_victim` / `diggers_per_victim`; backups are exempt
from preemption, so they take an idle robot or nobody. Both casualty tasks already retire
themselves on world state, so the extras cancel the instant the job is done.

| | seeds 42/43/44/45 | mean | worst |
|---|---|---:|---:|
| drift off, redundancy off | [63, 74, 76, 72] | 71.25 | 63 |
| drift on, redundancy off | [59, 82, 73, 71] | 71.25 | 59 |
| **drift on, redundancy on** | [71, 79, 75, 71] | **74.00** | **71** |

**Neither change works without the other.** Redundancy alone is negative — backups are
rank 1-2 and are matched before search work, so each consumes the nearest free robot that
would have explored. Once idle robots already have somewhere to be, the insurance is free.
Measured on two seeds first, which gave the opposite sign; four seeds reversed it. The
spread matters as much as the mean for a rehearsed demo: worst seed 63 -> 71, range 23
points -> 8.

### Battery: the carrier lane could not finish the mission it was bred for

`battery_capacity` is an evolved trait and nothing in the MAP-Elites fitness function knows
a carrier is what converts a find into a rescue.

| lane | endurance min | under-mission |
|---|---:|---:|
| scouts | 602 s | 0 / 171 |
| diggers | 555 s | 0 / 117 |
| relays | 486 s | 0 / 128 |
| **carriers** | **333 s** | **24 / 96** |

A quarter of the delivery lane ran flat before a 420 s mission ended, worst case at t=333.
That is a broken robot, not a specialised one. `World` now clamps capacity to a
scenario-derived floor — one mission of continuous movement x 1.15, so it tracks
`mission_duration_s` rather than going stale — and under-mission robots go to **0 of 512**,
carrier minimum 333 -> 483 s.

A base recharge pad was added with a `charge` particle burst. **It fired zero times.**
Carriers deliver to the *nearest* collection point and that is almost never base, and
nothing else sends a robot home. Wiring the seek into the **idle** path only — a robot with
no task loses nothing by topping up, and never abandons live work for a 400 m walk — gives
12 recharges a mission.

## M-73 — a third view of one unit, and how close "close" is (D15)

`F` toggled between the orbit and the robot's own POV, and nothing in between. The orbit
at 260 m shows the swarm and cannot show a robot, and the POV shows what the robot sees
and cannot show the robot. Narrating one unit — "this one has just been orphaned and is
re-bidding" — had no shot to do it in. `F` now walks orbit → POV → **chase**, a close
third-person follow, and each press is one step around that loop.

### Picking the distance

Godot's `Camera3D.fov` is **vertical** under the default `KEEP_HEIGHT`; `Renderer3D` takes
a horizontal angle. Copying 55 across as one number would have made the offline frame a
third tighter than the dashboard, so `chase()` converts through the frame aspect, and the
frames below are what Godot will actually draw. At a 55 deg vertical field on a 620 px
frame, 1 m at 1 m of depth is 595 px, and the chassis box is 1.4 x 1.2 m:

| distance | robot height | of a 620 px frame | reads as |
|---|---:|---:|---|
| 2.2 m (zoom floor is 1.8) | 325 px | 52% | the unit and little else |
| **4.0 m (shipped)** | 179 px | **29%** | the unit is the subject, its path still in frame |
| 5.0 m (first try) | 143 px | 23% | near-field rubble starts winning the frame |
| 16.0 m | 45 px | 7% | a small orbit, not a follow |

Checked on `runs/3d/chase*.png` at seed 42, t=120 on `demo`, with a turned frame
(`yaw_off_deg=75`) for the drag and both ends of the wheel travel.

### Three things the first cut got wrong

- **Sharing `orbit_dist`.** The orbit sits at 260 m and the chase at 4; one variable makes
  every `F` press a jump cut, and loses the operator's framing on the way back. Separate
  zoom, rotation and pan state per view, reset together by `E` and `Esc`.
- **An absolute camera bearing.** Held as an offset from the *unit's heading* instead, so
  the camera stays behind a robot that is turning rather than being left on its flank.
- **Bolting the camera to the state frame.** Poses land at 10 Hz; at 4 m that judders
  visibly. Exponential catch-up on position (14/s) and heading (5/s), frame-rate
  independent, and a hard snap when the followed robot changes — smoothing a switch across
  a 260 m map is a second of staring at rubble.

Ground-plane panning was also wrong at this distance: a drag upward sends the look point
over the horizon. The chase pans in the camera's own screen plane, clamped against the
zoom, so the unit can be put off-centre but never dragged out of its own follow camera.


## M-74 — the CNN detector: trained, measured, and it does not ship (D15)

The first GPU work of the project, and the fourth component to lose at the gate. Full
record: `scripts/export_frames.py` → `training/notebooks/detector.ipynb` on a Kaggle T4 →
`perception/cnn.py` running forward in numpy → `make gate`.

### The gate, ten held-out seeds (101-110), Tier 3 off

| arm | score | rescued | found | explored | lost |
|---|---:|---:|---:|---:|---:|
| evolved roster (classical detector) | **961.05** | 77.40 | 99.80 | **97.1%** | 34.80 |
| CV detector | **971.28** | 78.30 | 100.10 | 96.6% | 33.50 |

> **1.01x, under the 1.05x bar. `perception/classical.py` ships.**

**The CNN is not worse — it is not better enough.** It finds 0.3 more casualties, rescues
0.9 more and loses 1.3 fewer robots. It also explores 0.5 points *less*, which is the
8.7x perception cost coming straight out of the tick budget. Net +1.1% for 2,789
parameters, a Kaggle dependency and a fifth of the sim's real-time headroom. That is what
the margin exists to refuse.

Perception is real either way. Only the *trained* label was ever at stake.

### It is a good detector, which is the point

Trained on seeds 1-6, validated on 7-8, no frame from a validation mission ever seen:

    precision 0.950   recall 0.980   F1 0.965

and **identical in numpy on the M1 to what torch reported on the T4** (max |numpy - torch|
= 1.5e-05). Frame-level accuracy was never the question the gate asks.

### Cell-grid F1 is not mission performance, and this is the number that proves it

F1 0.965 against +1.1% mission score. The detector sees better and the swarm barely
converts it, because discovery has not been the binding constraint since M-71 widened
`trust_far`: 104 of 110 casualties are already *promotable* under the shipped classical
detector, and the remaining gap is coverage and investigation throughput, not perception.
A better eye cannot fix a chain that is limited three links downstream.

The detector sweep (`scripts/sweep_detector.py`, 12 configs on shared frames) said the
same thing before any GPU time was spent: **every config gave identical recall, 104/110
and 42/44 buried.** That was the moment to stop, and it was not taken -- the training run
went ahead anyway. Worth remembering next time a sweep returns a flat line.

### Cost, measured rather than estimated

Building this needed three performance findings that are worth keeping:

| | measured | why |
|---|---:|---|
| im2col forward pass | 161 ms | materialising the column matrix, 31 MB of copy in layer 1 |
| accumulate over kernel offsets | 120 ms | same FLOPs, temporaries 3.5 MB instead of 31 MB |
| patch-embed layer 1 (stride == kernel) | 46 ms | patches *tile*, so extraction is a reshape, not 16 gathers |
| + trained weights, with normalisation | 82 ms | `(x - mean) / std` over 3.5M uint8 cost as much as the net |
| **+ normalisation folded into layer 1** | **47.7 ms** | exact, not approximate; bit-identical detections |

Against `ClassicalVictimDetector` at **5.5 ms**. Pre-pooling the frame was tried and is
*worse* (104 ms): the pooling costs more than the smaller convolution saves.

### And the training run itself

20 epochs in **~3 seconds** on a T4 after one fix. The first attempt ran 27 minutes,
because the dataset sat in host memory and every batch did a CPU fancy-index plus a
host-to-device copy -- for a 2,789-parameter model the transfer *is* the workload.
Pinning the dataset to the device (94 MB against 16 GB) removed it. Three earlier
attempts failed in under a minute each, all on input wiring, all with the assert naming
the cause; the dataset path is now searched rather than hardcoded.

### Where this leaves the ladder

Four components have now been trained and gated, and **none of them ship**:

| component | margin | verdict |
|---|---:|---|
| Tier-2 bodies (MAP-Elites) | 1.29x | **ships** |
| Tier 3a commander | 0.98x | no |
| Tier 3 hivemind (scripted rung) | 0.97x | no |
| CV detector | 1.01x | no |

One trained component ships, and it is the one that evolves interpretable parameter
vectors rather than a network. That is not an accident and it is the honest headline.

---

## M-75 — the detector panel moved onto the world, and the terrain got in the way (D15)

The sensor inset in the bottom-right corner drew the followed robot's contacts in their
own little frame: a second, smaller picture of a world the viewer was already looking at,
which asked them to map one onto the other. It is now an overlay -- each contact boxed
over the thing it describes, in the live view, in POV and chase only.

Moving it turned up a problem that the panel had hidden, and the numbers are the reason
the overlay does not look broken on the demo map.

### The detector is flat; the dashboard is not

Perception is 2.5D (invariant 3): the cone test is bearing-only, `world.height` is
render-only. So the detector admits a contact by bearing while the dashboard draws it on
top of real relief. How much relief, measured over open ground:

| map | passable height | \|dz\| over 5 m of open ground |
|---|---|---|
| `test` | 0.00 - 18.24 m, std 2.94 | mean 0.70, p95 1.84, max 5.08 |
| `demo` | 0.02 - 68.04 m, std 12.66 | **mean 1.14, p95 3.73, max 15.57** |

A POV camera at 74 deg sees about +/-25 deg vertically, which at 5 m is +/-2.3 m. So a
p95 slope puts a near contact off the top of the frame, and the first offline frame drew
**0 boxes for the one contact in field** on the `test` fixture -- a casualty 5.4 m away
and 4.5 m up a slope, projecting to y = -443 on a 620 px frame.

That is not an edge case to shrug at: it is the near contacts, on the interesting
terrain, that vanish. In-field contacts outside the frame now get a chevron on the screen
edge carrying their range, which is all a flat detector knows about them anyway.

### Box size, from frames rather than by eye

`demo`, seed 42, t=60 s, the robot with the most contacts in its cone (5 of 23 on the
wire). Projected box sizes on a 1000x620 frame:

| contact range | POV box | chase box |
|---:|---:|---:|
| 7.0 m | 304 px | 189 px |
| 11.8 m | 175 px | 127 px |
| 21.0 m | 97 px | 79 px |
| 23.6 m | 87 px | 72 px |
| 25.6 m | 80 px | 67 px |

At a 3.0 m box the nearest one covers half the screen height, which is bigger than
anything it could be marking. `DET_BOX_M` is now **2.52 m** -- `VICTIM_SCALE` (1.8) times
the 1.40 m casualty mesh, so the box is the size of the body it frames -- and a test
pins the two together so raising one cannot silently strand the other.

The contact ring is deliberately *not* what the box is sized to: rings scale with how
resolved a contact is (0.9 vs 1.8), so a ring-sized box would change size with certainty
and read as a depth cue that is not there. Ring on the ground, box on the thing.

### What it costs

Redrawn every frame rather than on each 10 Hz state frame, which the inset did not have
to be: the boxes are projected through the operator camera, and that camera moves on
every frame in chase and on every mouse move in both views. At most `DET_MAX_BOXES` = 24
boxes, each a rect, four corner ticks and one string.

## M-76 — before training the units: one action with no headroom, one routing bug worth +10% (D16, , branch `rl/unit-policy`)

Question on the table: RL-train the individual units? Measured the actions first, on the M1,
demo seeds 42-45, Tier 3 off, shipped roster. Shipped baseline for every row:
**rescued 75.25, found 95.5, explored 95.2%, lost 43.25** (identical to M-72's four-seed
row, so nothing had drifted).

### The headless demo is 367 s, not ~25 s

`uv run python -m swarmmind.cli run --headless --seed 42` on an idle M1: **367.3 s wall,
1.14x**. CLAUDE.md still says ~25 s (M-58 already flagged it at 245 s). Four missions in
parallel on this machine took ~10 min each. Size any training from these numbers.

### Where unrescued casualties go (440 casualties, 4 seeds)

| | n |
|---|---:|
| rescued | 301 |
| found, not rescued | 81 (awaiting dig 17, awaiting pickup 25, **in transit at the buzzer 39**) |
| never found | 58 (37 buried) |

Found-to-rescued by discovery time: 0-120 s 91%, 120-300 s 81%, 300-360 s 40%, 360-420 s 7%.

### Action 1, contact inspection: dropped, with an oracle bound

Sending searching robots to take the closer look that settles an unconfirmed contact (M-71's
"seen and discarded" misses):

| arm | rescued | found | explored |
|---|---:|---:|---:|
| shipped | 75.25 | 95.50 | 95.2% |
| inspect every contact within 25 m | **68.00** | 87.25 | 88.0% |
| **oracle**: only contacts on a real undiscovered casualty (reads ground truth) | **75.75** | 98.75 | 95.4% |

Crude inspection is -7.25, down 4/4 (~9,000 detours a mission). **Perfect selectivity is
+0.5 rescued (+6/+1/-6/+1) for +3.25 found** -- M-50/M-64 again: more discovery does not
convert. No learned inspection policy can beat its own oracle, so it was not built.

### The rescue chain, timed per casualty

| stage (rescued casualties) | median | share of chain time |
|---|---:|---:|
| found -> dug out (buried only) | 20.9 s | 11% |
| dug out -> picked up | 13.4 s (p90 65.8) | 33% |
| picked up -> delivered | 36.8 s (route minimum 28.1) | 56% |

**The 39 in transit at the buzzer were picked up at a median t=296 with a median 124 s
left, needing ~31 s of route at loaded speed. 34 of 39 would have been delivered.**

### Why: loaded carriers trapped by the coarse flow field

Seed 42, carrier 360: holding a casualty from t=82 to the end, 12 m (geodesic) from a
reachable zone, in contact, on passable ground, no fire. For 298 of its last 400 ticks Tier 1
wanted 2.1 m/s and the wall override gave it 0. The coarse field's one descent direction for
its 4 m cell was due east -- into rock, with rock to the north -- while repulsion pushed it
back and the queue behind pushed it in. M-60 fixed phantom edges *between* coarse cells; this
is the same mismatch *inside* one, and every loaded carrier converges on the same 12 points.

### The fix, and the version of it that was wrong

`control/zone_routing.py`: one exact fine-grid field per chassis to every collection point's
delivery disc (static targets, so four fields built once, ~0.5 s), no corner cutting; carriers
aimed at a zone descend it, everything else unchanged. Flag `Mission(zone_routing=...)`,
**off by default** -- the shipped hash is untouched.

| routing to zones | 42 | 43 | 44 | 45 | mean rescued | found | lost |
|---|---:|---:|---:|---:|---:|---:|---:|
| coarse (shipped) | 68 | 88 | 77 | 68 | 75.25 | 95.50 | 43.25 |
| exact fine field | 79 | 91 | **64** | 77 | 77.75 | 98.00 | 38.75 |
| **fine field on ground eroded by one cell** | **73** | **96** | **80** | **82** | **82.75** | 97.75 | 41.50 |

The exact field lost 13 on seed 44: it threaded a **one-cell gap** that a force-field-steered
robot cannot hold a heading through (carrier 331: override 400/400 ticks, a few metres from
spawn, for the whole mission). Eroding passability by one cell keeps routes >= 3 cells wide
and falls back to the coarse field where there is no wide route. **+7.5 rescued (+10%), up on
4/4 seeds.** Four seeds is thin for a scenario change (M-62); the Kaggle bound job re-runs it
on eight.

### What this means for unit RL

Action space rebuilt around what *has* time in it: carriers/diggers staging at dig sites and
confirmed contacts (the 33% waiting), plus dark ground and hold, with the default always
offered (`control/unit_policy.py`). Bounded against its heuristic on Kaggle before any long
run (`training/rl/bound.py`), then BC from the heuristic -> PPO (`training/rl/run.py`).

## M-76a — the bound on Kaggle: routing holds on average, not on fresh seeds; staging is flat (D16)

`python -m swarmmind.training.rl.bound`, Kaggle CPU session (4 vCPU), demo scenario, seeds
42-49, Tier 3 off. 24 missions in 6,281 s: **1,038 s per mission with 4 in parallel, 13.8
missions/hour** -- about 1.4 M1 cores' worth, consistent with 4 vCPU being ~2 physical
cores (M-38's SMT finding). Absolute numbers differ from the M1 (seed 42 shipped: 61 here,
68 in M-76) -- platform float, M-40 -- so only compare rows within this table.

| arm | rescued | found | lost | score | per-seed rescued |
|---|---:|---:|---:|---:|---|
| shipped | 73.00 | 95.88 | 33.75 | 909.67 | 61, 79, 68, 67, 77, 93, 61, 78 |
| routing (`zone_routing`) | **81.12** | 99.25 | 33.12 | **998.06** | 77, 95, 84, 79, 73, 93, 71, 77 |
| routing + heuristic staging | 82.00 | 98.12 | 29.12 | 1006.54 | 75, 92, 81, 75, 79, 96, 73, 85 |

### Routing: +11% overall, and the split that has to be said out loud

Per seed vs shipped: **+16, +16, +16, +12 on 42-45** -- the seeds the fix was diagnosed and
built on -- and **-4, 0, +10, -1 on 46-49**, which nothing had looked at. Mean +8.1 over
eight, **+1.25 over the four fresh ones**. The fix is a mechanism (one erosion step, one disc
fraction), not a fitted value, so memorising the dev seeds is not the obvious reading -- but
four seeds cannot tell "those maps trap more carriers" from noise, and M-62 is the standing
warning about exactly this sample size. **Not a demo candidate until the gate's held-out
seeds 101-110 say so.**

### Staging: no measurable gain over routing alone

+0.88 rescued, score 1.008x, losses -4.0. Per seed vs routing: -2, -3, -3, -4 on 42-45 and
+6, +3, +2, +8 on 46-49 -- down on four, up on four. The fixed rule helps some maps and hurts
others. That is the one argument for a learned policy on this action (stage *when* it
helps), and it is weak: the bar is 1.05x the routing arm, ~+5 rescues, against +0.9 for the
heuristic. Expected outcome of training: trained, gated, does not ship -- to be measured,
not assumed.

## M-76b — scope decision: the demo plays four maps, and everything for it runs on them

Not a measurement; recorded here because it changes how the rows above are read.

The owner decided the demo will never show more than four maps: **seeds 42, 43, 44, 45**,
now `demo.yaml` `demo_seeds`. Training, in-run checks, bounds and the gate all default to those
(`tests/test_demo_seeds.py`); `HELD_OUT_SEEDS` 101-110 stay in code behind `gate.py --held-out`.

What it does to M-76a: the "routing gain concentrates on the dev seeds" caveat stops being a
blocker **for the demo**, because 42-45 are the demo -- routing is +16, +16, +16, +12 there.
It stays true as a statement about other maps, and the wording follows it: tuned on, and gated
on, the four demo maps.

## M-76c — the unit policy, three sessions, gated out at 1.04x (D18)

98 PPO iterations on the four demo maps, ~33 h of Kaggle CPU, ~390 missions. Bar (routing
alone, same maps): **1015.24**. Deterministic checks of the policy the demo would run:

| session | config | checks | mean | best |
|---|---|---|---:|---:|
| 1 | lr 3e-4 | 1003, 989, 962, 988, 982, 996, 1024, 987 | 991 | 1023.7 |
| 2 | lr 1e-3 | **1057, 1057**, 1043, 1038, 1028 | 1044 | **1057.0 = 1.041x** |
| 3 | rewound to session 2's best, entropy 0.01 -> 0.003 | 1002, 1027, 1020, 1029, 977, 1038 | **1015.3** | 1037.8 |

**Does not ship.** Best 1.041x against a 1.05x bar, and session 3 -- starting from the exact
weights that scored 1057 -- averaged 1015.3, the bar to one decimal. The peak was the top of a
noisy distribution rather than a level the policy holds.

**Two mechanism findings worth keeping.** (1) The learning rate, not the budget, was the limit
in session 1: per-update KL ~1e-4 against a 0.045 early stop, and raising lr 3e-4 -> 1e-3 moved
the check 987 -> 1057 within five iterations. (2) An entropy bonus of 0.01 outgrew the advantage
signal and drove a twenty-iteration decay (entropy 0.16 -> 0.49, `hold` 3% -> 17%); 0.003 stopped
it. Both are tripwire-shaped and neither wire caught them -- the wires watch for entropy
*collapse* and for checks *under* the bar, not for a run sliding down from above it.

**Why the policy plateaued, stated as a reward problem rather than a tuning one.** Sampled
losses fell 43 -> 22 robots while found and rescued stayed flat: the -0.5 death penalty is
immediate and personal, rescue credit is delayed 150-200 s and shared across four lanes, so the
policy bought the cheap term of `mission_score` and could not move the expensive one. Staging
stayed at ~1.2% of decisions throughout -- the policy agreeing with M-76a's bound.

## M-76d — the gate, re-run on the four demo maps (D18)

`make gate` on Kaggle, scenario `demo`, seeds 42-45, 24 missions. Full report in
`SHIPPING.md`. Two results that change what can be said, and one gap that has to be closed
before demo day.

| component | arms (score) | margin | ships |
|---|---|---:|---|
| Tier-2 bodies | classical 652.25 -> evolved 856.26 | 1.31x | evolved roster |
| **Tier 3 hivemind** | evolved 856.26 -> **+ scripted 902.21** | **1.05x** | **scripted rung** |
| Tier-1 zone routing | evolved 856.26 -> **+ routing 1015.24** | **1.19x** | zone routing |
| Tier-2 unit policy | routing 1015.24 / heuristic 987.45 / **trained 1056.97** | 1.04x | routing alone |

### Tier 3 clears the bar here, and did not on the held-out seeds

M-45 measured the scripted rung at 1.02x over ten held-out seeds and the gate said *no Tier 3*;
M-65 measured it at a clear net cost on four seeds; M-71 saw the sign flip and called it "a lead,
not a claim". On the four maps the demo actually plays it is **+3.75 rescued, +4.0 found,
1.05x** -- just over the bar. Both readings stand: **it helps on these maps, and it is not
demonstrated on maps nobody has run.** The honest sentence for a judge is the first half, with
the second half said aloud if asked.

### The unit policy: 1.04x, does not ship

87.50 rescued against routing's 83.75 (and the heuristic's 80.75) on its own best checkpoint --
but the bar is 1.05x and session 3 showed the true level is ~1.00-1.03x (M-76c). The gate agrees
with the checks, which is what the checks were for.

### The gap: routing and Tier 3 have never been measured together

Every arm above runs Tier 3 **off** except the Tier 3 row itself, and the demo runs it **on**.
So `evolved + routing + scripted Tier 3` -- the configuration the demo would actually run if
routing is adopted -- is unmeasured. Eight missions (two arms, four maps) settles it. Do not
adopt routing for the demo without it: two changes that each help alone are exactly the pair
this file has watched cancel before (M-66 §3, M-72's drift + redundancy).

## M-76e — the configuration the demo actually runs: routing with Tier 3 live (D18)

The gate measures routing with Tier 3 **off** and Tier 3 without routing, so the combination on
stage was unmeasured. `training.rl.bound --arms tier3 tier3+routing tier3+routing+policy`,
Kaggle, demo maps, scripted rung (deterministic), 12 missions, ~950 s each.

| arm | rescued | found | explored | lost | score | per-seed rescued (42/43/44/45) |
|---|---:|---:|---:|---:|---:|---|
| `tier3` -- `main` today | 72.50 | 97.00 | 96.7% | 43.25 | 902.21 | 65, 86, 74, 65 |
| **`tier3+routing`** | **82.00** | 98.50 | 96.9% | 40.50 | **1001.60** | **74, 94, 78, 82** |
| `tier3+routing+policy` | 81.75 | 96.75 | 96.8% | 34.50 | 998.59 | 72, 95, 79, 81 |

`tier3` reproduces the gate's Tier-3 arm to the digit (902.21), so the pairing is sound.

### Routing and Tier 3 overlap, and routing still wins

+9.5 rescued with Tier 3 live (1.110x), against +15.0 with Tier 3 off (M-76d). Both are real:
Tier 3 already pulls the swarm off some of the ground where carriers were trapping, so part of
routing's gain was already being recovered. **Up on 4 of 4 maps** (+9, +8, +4, +17), which is
what the check was for -- M-66 and M-72 are the recorded cases of two good changes cancelling.

### The unit policy, measured a fourth time, in the configuration that matters

0.997x against `tier3+routing`: **no gain with Tier 3 live**, after 1.04x at the gate with Tier 3
off, ~1.00-1.03x across session 3's checks, and 1.008x for the heuristic it was cloned from. It
does buy lower losses again (34.5 against 40.5) and gives back a little discovery (96.75 against
98.50) -- the same trade the whole programme kept finding. The verdict does not move.

## M-76f — the demo machine disagrees with Kaggle in *sign*, so routing stays off (D18)

M-76e adopted zone routing on Kaggle evidence: +9.5 rescued with Tier 3 live, up on 4 of 4.
Before merging, the same paired comparison was run **on the demo machine**, because that is what
demo day runs. It does not reproduce.

| four demo maps, Tier 3 scripted | rescued | found | lost | score | per-seed rescued |
|---|---:|---:|---:|---:|---|
| `tier3`, M1 | **78.50** | 99.75 | 44.75 | **967.00** | 74, 79, 82, 79 |
| `tier3+routing`, M1 | 77.25 | 96.50 | 37.25 | 951.72 | 65, 91, 80, 73 |
| `tier3`, Kaggle (M-76e) | 72.50 | 97.00 | 43.25 | 902.21 | 65, 86, 74, 65 |
| `tier3+routing`, Kaggle | **82.00** | 98.50 | 40.50 | **1001.60** | 74, 94, 78, 82 |

**Kaggle: +9.5, up 4 of 4. The M1: -1.25, up 1 of 4.** Same code, same seeds, same scripted rung.

### The cause is platform float against a chaotic sim, and the size is the finding

Compare the *identical* `tier3` arm across machines: 65/86/74/65 against 74/79/82/79 -- deltas of
+9, -7, +8, +14 per seed. **Floating-point differences alone move one map by ~10 rescues**, which
is the whole size of the effect being measured. M-40 established that a before/after pair must be
taken on one machine; this is that rule at full scale, and it also means *neither* four-seed
result above is strong enough to carry a 1% decision on its own.

### What survives

* **Routing with Tier 3 silent is real and reproduced:** +7.5 on the M1 (M-76), +15.0 on Kaggle
  (M-76d), up on 8 of 8 seed-arms across two machines. The carrier trap it fixes is a genuine
  defect (M-76: one carrier held a casualty 12 m from its zone for ~320 s).
* **Routing with Tier 3 live is unresolved.** The strategic layer already pulls the swarm off some
  of the ground where carriers trapped, so the gains overlap -- and what is left is smaller than
  the noise this scenario can resolve with four maps.

### Decision

`zone_routing` **stays off by default**. The demo keeps the build it rehearsed, its beat sheet and
its backup video. The code, the flag and the tests stay in the tree so the fix is one argument
away and the claim is reproducible. **Nothing about the demo changes**, which is the right outcome
the day before a freeze -- and the honest sentence is that the fix helps a swarm with no strategic
layer, and that we could not show it helps the one we demo.

## M-77 — original articulated rescue fleet

User-requested display models replace the dashboard's single robot box. Four purpose
modules — scout camera, digger bucket, carrier cradle and relay mast — combine with
wheels, tracks, legs and rotors. The simulator bars rotor carriers, leaving **15 variants**.
Source geometry and joint poses are in `swarmmind/viz/units.py`; the GLBs, offline
renderer and batched Godot shader all consume that source. No simulation/control,
scenario, contract, training or demo-policy setting changed.

| Measurement | Result |
|---|---:|
| Original model variants | 15 |
| Named clips per GLB | 4: idle, move, purpose work, disabled |
| Triangles per variant | 1,780–4,028 |
| Unique triangles across all variants | 38,660 |
| Self-contained GLBs, all variants | 5,560,424 bytes |
| Indexed runtime geometry/joint manifest | 995,177 bytes |
| Expanded runtime mesh attribute payload, calculated at 56 bytes/vertex | 6,494,880 bytes |
| Current 512-robot roster: unit triangles submitted, calculated | 1,310,580 |
| Current roster: instance transform/colour/custom data, calculated | 40,960 bytes |
| Fleet batches / surfaces | 15 maximum |
| Full offline PNG/APNG preview build, development machine | 127.05 s |
| Animated preview files | 8, each 96 or 192 frames; PNG CRCs and differing poses checked |
| `make check` | Ruff green; 439 tests in 311.88 s; fixture smoke passed |
| Final model/export/dashboard tests after regeneration | 80 passed in 4.60 s |
| Seed-42 `test.yaml` smoke | 4/8 rescued; 6/8 found; hash `86a44954eda1a756` |
| Native Godot frame rate / RSS / shader compilation | **Not measured: unavailable in development environment** |

The 439-test suite was collected before adding the final shipped-asset drift test;
that test is included in the subsequent 80-test focused run. No demo mission, gate or
training job was run for this display change. `make check` now selects the documented
tiny fixture through `make smoke`; `make headless` remains the explicit full demo run.

The GPU shader moves rigid joints instead of instantiating per-robot scene trees.
Wheels and legs use observed travel, buckets require a dig assignment near a reported
active excavation, and carrier payloads require carrying activity. Mechanisms stop
for failed/destroyed units; extrapolation is capped at 0.2 s on a stale feed. Travel
phases wrap per cycle to retain precision in Godot Compatibility's half-float instance
data. Existing colour modes and comms dimming remain available. Rotor blades animate
at terrain height: the frozen wire has no airborne flag, so flight is not inferred.

Offline studio, pose-series, first-person and chase frames were inspected before and
after the Godot port. New tests cover GLB accessors/loops, joint handedness, shader
angle parity, ground clearance, payload visibility, asset drift and simulation state
preservation. Rendering previously consumed the map RNG for floor colour; it now uses
an independent seeded display generator, and a next-tick comparison verifies that
opening a renderer cannot change a mission. See [the gallery](units/index.html) and
[design notes](UNIT_MODELS.md).


## M-78 — streamed terrain and unit detail

User-requested rendering rewrite: offscreen geometry streams out, distant orbit geometry
simplifies, and **every visible terrain/ruin tile uses full detail in first-person and
chase**. The display reconstructs smooth land from the existing wire heightfield, shares
water vertices, and replaces imported scatter plus cell boxes with merged procedural
ruins. The four demo layouts, simulation, contracts, navigation and training settings
are unchanged. See [terrain rendering](TERRAIN_RENDERING.md).

Godot 4.7.2 is now installed on the development M1. Native editor/script parsing and
Compatibility (OpenGL 4.1 Metal) rendering were checked, superseding M-77's runtime
availability limitation. Measurements below were taken on this machine, with a static
seed-42 demo **construct-only** bridge fixture (512 initial robot rows), at 1100×700,
VSync disabled, god-view enabled and sector tint disabled. No demo mission or training
job ran. Each camera settled for 150 update frames; the following 100 frames supplied
the median/P95. The renderer's frame interval includes CPU submission and the engine
frame boundary; it is not an isolated GPU timer or a concurrent live-mission FPS claim.

| Measurement | Result |
|---|---:|
| Detailed unit mesh | 1,780–4,028 triangles |
| Medium / far unit mesh | 200–324 / 84–120 triangles |
| Runtime startup index | 3,964 bytes; geometry loaded from 45 per-variant/detail sidecars |
| Demo overview, all 512 units: old full detail → new far detail | 1,310,580 → 49,188 triangles (96.25% fewer) |
| Demo wall cells → merged wall rectangles | 27,037 → 2,715 |
| Old per-cell wall/rubble boxes alone | 475,356 triangles, excluding imported dressing |
| New all-map procedural props: full / medium / far | 74,652 / 43,032 / 32,580 triangles |
| Native world setup, before progressive visible tile builds | 393.08 ms |
| Orbit frame interval, median / P95 | 4.277 / 9.648 ms |
| Chase frame interval, median / P95 | 4.574 / 7.127 ms |
| First-person frame interval, median / P95 | 5.317 / 7.086 ms |
| Native rendered primitives: orbit / chase / first-person, including HUD | 151,778 / 247,204 / 269,048 |
| Terrain resident tiles: orbit / chase / first-person | 96 / 65 / 69 of 96 |
| Chase / first-person terrain strides | All visible tiles at stride 1 |
| Units culled in chase / first-person | 440 / 393 of 512 |
| Grounding 512 units at stride 1, old/new implementation median | 84.635 → 4.691 ms |
| Python/Godot support transform parity after optimisation | Maximum difference 1.18e-6 over 48 poses |
| `make check` | Ruff; 502 tests in 300.06 s; fixture smoke passed |
| Seed-42 fixture smoke | 4/8 rescued; 6/8 found; hash `86a44954eda1a756` unchanged |

The old/new grounding microbenchmark ran 30 batches of the same 512 poses on this
machine. The original implementation redundantly sampled 25 support points plus the
containing terrain vertices. Bounding the footprint analytically and reading lattice
vertices directly retains the conservative ground-clearance guarantee while removing
that frame hitch. Mixed terrain strides share one support basis, and any terrain LOD
change invalidates unit poses immediately. Candidate culling accounts for support lift;
reconnection clears old trajectories until fresh telemetry arrives.

Native regression scripts exercise frustum exclusion, mesh eviction, progressive tile
budgets, full-detail follow views, LOD hysteresis, telemetry retained while culled,
selected-unit priority, pose invalidation, reconnects and terrain seam clearance.
Python tests additionally verify wet-mask/dry-crossing preservation, source/export
geometry, animations and unchanged simulation arrays/RNG/next-tick results.

The static fixture timing excludes Python mission/LLM contention and does not measure
process RSS. Native screenshots and the complete runtime/check logs are under
`runs/3d/streamed/`; the screenshot run ended with `DASHBOARD_STREAM_CHECK_OK` and no
native script/shader errors. No previous measurement row was removed.

---

## M-79 · Response-team build

Apple M1, 8 GB; Python 3.12.14. Added an opt-in rescue lead / logistics / safety team.
The official **WorkSwarm 0.2.6 SwarmFlow engine** runs in an isolated interpreter with
a custom restricted backend; the installed wheel resolves `openjiuwen==0.1.18` and
236 distributions in total. The full workbench, browser and MCP server are not launched.
The separate dependency lock is under `integrations/workswarm/`.

Model: stock Qwen2.5-1.5B-Instruct Q4_K_M on localhost:8080, served by the existing
`llama` launcher. The missing GGUF was restored from the official repository and its
SHA256 matched `models.lock`. No model training or gate jobs ran. Initial cold framework
import took **42.31 s**; startup is separate from the 20 s workflow deadline and cannot
block the simulation. The core environment retains its original dependencies.

### Real complete mission, then final-build verification

Ran one complete realtime `demo.yaml`, **seed 42**, with 512 robots, native Godot 4.7.2
at 1100×700, WorkSwarm and the local model. Command was `cli run --demo --scenario demo
--seed 42 --wait --response-team --team-trace runs/team/rehearsal.jsonl`. No other demo
seed was evaluated. The team was stopped between t=340 and t=350; the mission continued
to t=420 with scripted strategy and the autonomous auction.

| Observation | Result |
|---|---:|
| Rescued / found | 59/110 rescued; 92/110 found |
| Explored / lost | 94.0% ground; 26/512 robots |
| Team episodes started | 18 |
| Reviewed proposals applied | 9 |
| Rejected against changed observations | 2; rescue work no longer observed |
| Completed workflows withholding dispatch | 6 |
| Peer revisions | 2; one applied, one subsequently vetoed by safety |
| Model calls / reported total tokens | 45 / 61,811 |
| Applied-episode wall latency | 8.377–13.117 s, nine samples |
| Longest successful model call | 4.876 s |
| Subsequent task awards associated with leased sectors | 548; not causal attribution |
| Godot runtime log | No script/shader errors; one connected dashboard throughout |

The old clock counted setup / `--wait` time, causing early catch-up and a reported
411.25 s mission wall time / 1.02× scorecard RTF. Do not use that as a speedup claim.
The final code starts its pacing clock after dashboard readiness. The rehearsal also
counted the same stop twice; the final supervisor acknowledgment path counts it once.
Original logs are preserved rather than rewritten. Free-text model notes sometimes
misdescribe connected **robots** as sectors; the final HUD reasoning is generated from
validated structured fields, while the separate trace retains the model's actual notes.

**Final-code real-runtime check:** `test.yaml`, seed 42, `--demo --response-team
--max-time 65`, no Godot, run during regression checks. At the second episode logistics
revised A3 to A2, the lead accepted, safety approved, and the order was applied. A task
award followed in A2. Stopping the team after t=40 produced **one** fallback acknowledgment;
the simulation continued to t=65 in **65.01 s**, reporting 1.00× RTF. This verifies the
final clock, trace and termination behavior; it is not a new full-stack performance run.

Curated, explicitly recorded examples are in
[`integrations/workswarm/examples/`](../integrations/workswarm/examples/README.md).
The original full trace and logs remain in `runs/team/` and are ignored by Git. The
example excerpt retains complete selected episodes and identifies omitted episodes.

### Negative results and memory limits

The first 75 s real-runtime fixture run withheld every plan: the prompt confused
proposed priorities with priorities already set. Explicit candidate indices and clearer
role instructions enabled actual dispatch. A separate synthetic peer-revision diagnostic
still failed with the real small model: it withheld while its note proposed rescue work.
That failed trace is retained. Deterministic tests prove the peer/veto control paths;
the live mission and final fixture independently demonstrate actual model-driven revisions.

`scripts/residency.py` now includes the team worker. Six full-stack samples over 123 s
of the rehearsal, using physical footprint rather than RSS:

| Sampled peak | MB |
|---|---:|
| Model server processes | 1,154 |
| Simulator | 161 |
| WorkSwarm worker | 224 |
| Godot | 220 |
| Concurrent total | 1,755 |
| System swap growth above 10,836 MB baseline | **+280** |

**This is not a clean residency pass.** Other applications were still present and the
machine was already heavily swapped. Attribution of swap growth to a particular process
was not measured. After initial startup, printed mission RTF was about 1.00×, but native
frame timing and tick-stall P95 were not sampled. A clean cold-boot rehearsal, physically
disconnected networking check, and backup video remain operator preparation. Configured
model endpoints are loopback only. No claim of rescue uplift or generalization follows
from this single live mission; no matched baseline/single-advisor/team comparison ran.

### Regression result

Final `make check`: **542 tests passed in 284.02 s**, Ruff clean, tiny fixture smoke
4/8 rescued, 6/8 found, hash **`86a44954eda1a756` unchanged**. The first test run caught
private trace methods named `emit` being mistaken for bus events by the protocol guard;
private logging is now named `record`. No new swarm event, bus schema or Godot parser
change was needed. Tests cover peer revisions, vetoes, observation isolation, disconnected
telemetry, malformed/stale/mismatched evidence, duplicate responses, directive arbitration
and expiry, worker crash/deadline/termination, late results after close, CLI constraints,
and preservation of historical gate data while annotating current defaults.

Commander, unit policy and zone routing remain off. The team remains opt-in and
heuristic/model/custom-runtime modes are labeled separately. Setup instructions,
scope decisions and precise limits are in [MULTI_AGENT_DEMO.md](MULTI_AGENT_DEMO.md).


## M-80 — landscape display and actual casualty cover

The landscape display replaces rectangular platforms with irregular embedded scree,
forest stands, rock outcrops and damaged houses. River banks gain sediment transitions
and shared chamfered shore vertices; steep faces gain irregular erosion and rock strata.
The simulation height, occupancy, water, perception, four demo layouts and policies are
unchanged. Original geometry is authored in Python and mirrored/exported to Godot.

Buried casualties previously used the same exposed body transform as surface casualties;
only their pin was lower. They now sit inside a solid rubble core with broken slabs,
timber and masonry. Cover follows the existing hidden/found → cleared transition,
not the historical `buried` flag. The model retains a small exposed sleeve, consistent
with the existing detector raster's visible clue. Pickup still requires actual excavation.
Normal view only draws piles at confirmed buried contacts; unseen truth requires V/G.

Measured with a construct-only seed-42 demo world (360×240 m, 512 robots, 110 casualties,
44 buried); these counts are static reference geometry, not GPU timings or live FPS:

| Measurement | Result |
|---|---:|
| All-map scenery triangles, full / medium / far | 112,320 / 96,612 / 86,280 |
| Reference conifers / ruined houses | 300 / 35 |
| Shared casualty cover mesh | 216 triangles, 648 exported vertices |
| Cover for all 44 buried casualties | 9,504 triangles, one MultiMesh batch |
| Terrain Python/native numerical parity tolerance | 3e-6 |
| Shoreline configurations checked for dry-cell intrusion | All 512 local 3×3 masks |

Offline landscape views and buried/cleared close-ups were inspected. The latter uses a
copy set to the cleared state, not a measured digging duration. A separate state-machine
regression proves a carrier cannot pick up a buried casualty until scoop work reduces
its debris to zero. Geometry tests verify every buried body vertex lies inside the rubble
core, and the exported mesh exactly matches its Python source. Headless Godot tests
exercise normal/god visibility, burial and clearance, body transforms, and removal of
carried/rescued ground bodies.

Native graphical capture was attempted, but the sandboxed launch aborted and automatic
approval review rejected the unsandboxed launch because its review service quota was
exhausted. Headless Godot checks ran successfully; no new graphical frame timing, native
screenshot or live-mission FPS is claimed. Previews and review metadata are under
`runs/3d/landscape/`; see [terrain rendering](TERRAIN_RENDERING.md).

## M-81 — simulated unit thermal display

`H` enables a display-only thermal view in POV/chase. The existing truth rows supply
surface positions and burial state; no detector, autonomous decision, contract or
simulation RNG changes. The position hash varies relative heat between casualties;
simulation time drives a small shimmer and 5 Hz pixel grain. Hidden/found buried
casualties use a dim cover signature; cleared casualties use the warmer body mesh;
carried/rescued casualties disappear from their former ground position.

| Check | Result |
|---|---|
| Five targeted Python/native tests | Passed |
| Full regression suite | 550 passed in 566.91 s; the added depth/RNG test also passed in the five-test targeted rerun |
| `make check` smoke (`test`, seed 42) | 4/8 rescued, 6/8 found, hash `86a44954eda1a756`; 30.20 s wall time |
| Sensor gate | 90° cone, 26 m, wall checks at quarter-cell intervals |
| Opaque foreground at 1 m vs body at 7 m | Body fully occluded in the reference depth buffer |
| Simulation RNG state before/after rendering | Every stream byte-for-byte unchanged |
| Graphical runtime | Godot 4.7.2, Compatibility / OpenGL 4.1 Metal, Apple M1 |
| Native controls/visibility/shader check | `THERMAL_CHECK_OK`; no shader/script errors |
| Extra display structure | Two shared MultiMeshes and one screen pass; no extra viewport/process |

Inspected normal/exposed/buried reference frames at 640×420 each and a native
1600×980 dashboard capture. The latter shows both heat levels with an unchanged HUD
and the simulated-sensor label. Native windowed execution required leaving the sandbox
after its initial launch aborted. Images: `runs/3d/thermal/comparison.png` and
`runs/3d/thermal/native.png`. These are isolated visual fixtures, not live-mission FPS
or evidence of improved autonomous discovery. Intensity is illustrative, not °C;
buried warmth models a stylised surface signature, not infrared transmission through walls.

Validation: **91 focused terrain/burial/bridge/isolation tests passed**. The full suite
completed with 545 passing tests and one headless-test assertion failure: the dummy
renderer does not retain GPU MultiMesh transforms. The test now checks the actual
body-pose helper used by the dashboard, and the corrected test passed on rerun.
`make check` with pytest's failed-test selection then passed Ruff, the corrected test,
and the fixture smoke. Smoke remains **4/8 rescued, 6/8 found**, hash
**`86a44954eda1a756` unchanged**. The complete run and rerun logs are retained under
`runs/3d/landscape/`; no native graphical performance claim is made.

## M-82 · Native Leader/Teammate migration

**Owner request:** use WorkSwarm's native leader and teammate system. `--response-team`
now uses installed **WorkSwarm 0.2.6** `TeamAgentSpec`, `LeaderSpec`, `TeamMemberSpec`,
`Runner.run_agent_team_streaming`, the scheduled task board, native message delivery,
logistics teammate and native task reviewer. SwarmFlow's custom `AgentBackend` is no
longer on this path. M-79 remains a historical SwarmFlow record.

The leader creates a native task assigned to logistics with safety as reviewer.
Logistics writes its recommendation to the shared task and completes it. Safety uses
the SDK's reviewer-scoped `VerifyTaskTool`; the native scheduler settles the vote and
notifies the leader, which must submit the exact verified choice. The independent
simulator filter and 30-second lease still control effects. Team topology and task
shape are predefined; this does not demonstrate unrestricted dynamic task discovery.

**Integration fixes:** use the team Runner's native session lifecycle. The scheduled
reviewer requires an explicit model configuration; the model pool alone did not provide
one. In-memory SQLite lost updates in concurrent coordination probes; temporary
file-backed SQLite with small caches completed the same checks. SDK rails compact
native shared-task records for inference and remove repeated workspace/roster prompt
announcements. Native scheduling, task ownership, message routing and vote settlement
remain in the SDK. Stage-specific prompts resolved an observed leader returning prose
instead of its submission tool; prose is never treated as a vote. A trace-field `kind`
collision was caught by simulator testing and fixed; actual worker-protocol tests now
cover native stream records and repeated requests.

**Checks:** `UV_CACHE_DIR=/tmp/swarmmind-uv-cache make check`: lint, **551 tests passed**
in 550.38 s, unchanged smoke hash **86a44954eda1a756**. The response-team, knowledge-boundary
and bridge selection passed **65 tests** after isolated native refinements. The installed
SDK suite passed five cases with deterministic HTTP model responses (revision, veto,
unauthorized teammate tool, token budget and repeated episode), plus two requests through
the actual worker subprocess. These tests establish integration behavior, not LLM quality.

**Real inference:** stock local Qwen2.5-1.5B-Instruct Q4_K_M, llama.cpp on loopback,
context 8192, one slot. No cloud key or inference was used. The SDK's `local-no-secret`
API-key field is a compatible-client placeholder. A standalone native call over recorded
observations completed in **14.702 s**, four calls and **2,425 reported tokens**, under
the 20 s deadline. It produced a reviewed A1/explore order without simulator dispatch.

**Live fixture:** `--demo --scenario test --seed 42 --max-time 80 --response-team`.
Two native episodes completed in **12.553 s / 2,428 tokens** and **16.639 s / 2,502 tokens**,
four calls each. A1 and A3 orders passed current-state validation and were applied.
An A3 auction assignment is recorded as association, not causation. The operator stopped
the third episode: exactly one fallback acknowledgment, mission continued to t=80 at
**1.00× realtime**, both accepted leases expired. Stats: **3 episodes, 2 applied,
0 rejected, 1 fallback**. No live peer revision or veto occurred in this fixture;
deterministic SDK tests cover those branches. This fixture preceded the final extra
context reduction and 8 s first-chunk setting described below.

**Timing failures retained:** an early pre-compaction fixture hit the 20 s wall deadline
after safety review and correctly kept running on scripted fallback. A subsequent
512-robot seed-42 smoke run also timed out after an early model retry. It completed
t=70 with **0.78× realtime**, zero native orders applied; it is not a passing native
application or rescue-uplift result. Final tuning further compacts model inputs from
the native task board and raises the per-call/first-chunk timeout from **5 to 8 s** to
avoid premature retries. The overall deadline and maximum result age remain **20 s**.
The 120 generated-token limit, 9,000 reported-token budget and eight candidates remain.

**Memory scope:** three samples over 31 s around the tiny fixture measured native-worker
physical footprint **246.3 MB**, target-process total **2,876.1 MB**. Model-process aggregate
peaked at 1,563.7 MB, simulator at 43.5 MB, existing Godot processes at 1,024 MB. Those Godot
processes were not launched or visually verified by this check. Swap started at 12,037.8 MB
and decreased 115 MB. No growth was observed in this short window; this is not a full
cold-boot residency pass or 512-robot performance evidence. The machine was heavily swapped.

Evidence: [native recordings](../integrations/workswarm/examples/README.md),
`runs/team/native-verified.jsonl`, `runs/team/native-residency.json`, and the failed
load-check trace `runs/team/native-seed42-smoke.jsonl`. No matched rescue-performance
baseline, full 420 s native rehearsal or new Godot visual verification is claimed.

**Final model-format adapter:** another load probe returned prose from the safety
model and timed out (t=60 mission continued at 0.83× realtime). A direct local HTTP
probe confirmed `tool_choice="required"` could return prose; the named-function form
was silently downgraded with a server warning. The latter is also documented in an
[upstream report](https://github.com/ggml-org/llama.cpp/issues/27217); this report does
not establish that our template has the same underlying required-tool bug. ChatML
alone did not enforce tool calls either.

`native_model.py` now registers a small SDK model-client extension. It subclasses the
SDK's `OpenAIModelClient`, forwards the single authorized tool's argument schema as
`response_format=json_schema` to local Qwen, validates the actual model JSON and returns
an SDK `ToolCall`. It never supplies a candidate or vote itself and never executes a
tool. Native Leader/Teammate execution and task coordination are unchanged. Invalid or
forged argument fields fail closed. The adapter uses ChatML in the documented tested
launch command; notes/feedback have a 24-character limit to bound output latency.
This is a custom local-model format adapter, not the unmodified native model provider.


**Final 512-robot native smoke:** `--demo --scenario demo --seed 42 --max-time 65
--response-team`, with the registered SDK JSON-schema model adapter, ChatML, 8192
context, 8 s per-call limit and **20 s total deadline**. Native episodes completed in
**12.429 / 12.704 / 17.064 s**, four model decisions each, **1,781 / 1,790 / 1,748 reported
tokens**. Three orders applied: B1, B4, A6. In episode three, logistics changed the
leader's D5 choice to A6; safety voted pass, the native scheduler completed the task,
and the leader accepted the revised choice. The simulator applied it at t=49.95.
Subsequent sector task awards are associations, not rescue attribution. The first two
leases expired; the third was still live at the normal t=65 mission end. A fourth
request was cancelled by mission shutdown. Stats: **4 episodes, 3 applied, 0 rejected,
0 fallbacks**. A stop file was created after mission completion, so it is not evidence
of operator-stop handling; that check is the earlier t=80 fixture.

The run took **91.90 wall-seconds / 0.71× realtime** on the loaded machine. Its partial
score was 15/110 rescued, 27 found, 0/512 robots lost; no matched baseline was run and
no rescue-uplift claim follows. This establishes actual native-team application on the
demo map but leaves smooth full-length presentation performance unverified. Final
SDK tests passed five deterministic HTTP-model cases plus two worker requests; the
65 core boundary/protocol tests and lint passed again. The malformed/forged-arguments
case expects a bounded failure with no submission, not recovery or a fabricated vote.

Final evidence: [native demo-map trace](../integrations/workswarm/examples/native-demo-smoke.md),
`runs/team/native-shipping.jsonl`. The final model adapter remains an explicit custom
extension; native Leader/Teammate coordination, shared tasks and verification are the
installed library's implementation.

## M-83 — casualty placement follows debris and shelter

Casualties were sampled from navigable cells with only a distance bias; burial was
assigned from the farthest 60%. This left most buried people away from rubble and
some casualties in shallow water. Placement now uses physical occupancy: burial
prefers traversable rubble, other covered sites prefer solid-obstacle margins, and
an exposed minority remains. YAML sets a target covered fraction of 0.85 and a 3 m
cover radius. The artificial map border does not count as cover. No scenery labels
or hidden coordinates are passed to the swarm; discovery still uses the existing
occluded camera and detector.

Construction-only before/after measurements on this M1, using exactly `demo_seeds`.
Terrain, count (110), buried quota (44), collection keepouts and 14 m spacing are
retained. The new sites also require dry ground, fine-grid legged access and the
same edge-aware coarse reachability used by navigation. Habitat/spacing preferences
may fall back on sparse custom maps, but these safety conditions never relax.

| Seed | Near cover, before → after | Buried on rubble, before → after | Surface near wall, before → after | In water, before → after |
|---|---:|---:|---:|---:|
| 42 | 58 → 94 | 8 → 44 | 19 → 50 | 3 → 0 |
| 43 | 60 → 94 | 8 → 44 | 17 → 50 | 7 → 0 |
| 44 | 64 → 94 | 10 → 44 | 16 → 50 | 8 → 0 |
| 45 | 62 → 94 | 9 → 44 | 17 → 50 | 4 → 0 |

Each map now has 44 buried rubble sites, 50 surface sites beside solid obstacles and
16 exposed sites. Full world construction in this review took 0.836–1.189 s, including
terrain and robot initialization; this is not a placement-only benchmark. All four
maps retain the requested 14 m separation without relaxation.

As a limited geometry check, 16 viewpoints were sampled 6 m from each casualty,
discarding off-map or legged-impassable viewpoints. The existing LOS test blocked
38/1277, 40/1329, 56/1342 and 59/1261 sightlines after the change, versus 36/1374,
27/1429, 20/1364 and 29/1431 before. A wall margin does not mean a casualty is
invisible from every direction. This check measures geometry only, not detector
recall. Buried sites retain the small visible clue and existing excavation requirement.

Offline 3D buried and sheltered site previews were inspected using `render3d.py`.
Evidence: `runs/3d/victim_placement/review.json`, `before_after.png` (before left,
after right; gold buried, red surface), `buried.png`, and `sheltered.png`.
Regenerate the current placement audit with `uv run python scripts/review_victim_placement.py`.
Focused world, perception and determinism checks passed, along with all nine new
placement regressions (cover, access, dry ground, counts, spacing, sparse terrain,
metric radius and RNG isolation).

Full-suite validation during concurrent flight/rescue edits completed with **594
passes and one failure**: the terrain assertion loaded before the flight change
treated airborne rotors as grounded. After that assertion was updated by the flight
work, its rerun plus placement and determinism checks passed (**13 tests**). Lint and
the complete `test.yaml` headless smoke also passed: 3/8 rescued, 5/8 found, 79.8%
explored, hash `7222837e33e1d4e1`. That smoke reflects the combined workspace, not an
isolated before/after comparison for placement. The full suite was not rerun after
the concurrent edits; no clean whole-suite result on a frozen final tree is claimed.

This intentionally changes casualty coordinates and historical scorecard hashes.
Repeatability is preserved, but **historical mission scores and gates describe the
previous casualty distribution**. No full demo mission, training run, new gate,
rescue-rate improvement or unseen-map generalization is claimed here. The retained
unit policy was tuned on the four demo maps and remains off; its earlier margin has
not been re-measured against these new casualty sites.

## M-84 — rescue execution audit and a steering experiment rejected as a default

Measured against **83686e0**, on this Mac, Python 3.12.14. Concurrent placement (M-83)
and flight work changed the shared workspace during the audit. Comparisons below use
isolated source snapshots. The first comparison uses the original scenario/roster;
the separate combined-build comparison below includes the flight and placement changes.
Original `test.yaml`
SHA256: `c1acc11bd126cfc655c168c86a1cc4ed23d4998307b8af52f78067cf6a533b5d`.

### Reproduced faults

- Between auction cycles, `SkillExecutor._candidates` rebuilt delivery goals without
  the navigation object, caching the closest zone even when it was unreachable.
- A carrier picking up while idle acquired no extraction assignment. One collecting
  a different casualty than its assigned extract kept pursuing the original casualty.
  Both now deliver the casualty actually held, and loaded carriers cannot bid as free.
- Opportunistic-pickup handling overwrote explicit hazard retreats on the following
  tick. It now completes the retreat, then resumes delivery.
- A partially cleared casualty exempted *every* assigned digger from stall release,
  including robots stranded far from it. The exemption now requires physical reach.
- Fault injection emitted a destruction event twice. The world now owns that event.
- Distance fields rejected blocked coarse edges, but both steering implementations
  ignored those same masks. The optional repair maps the differing direction orders
  explicitly and checks edge legality in reference and cached steering.

The original regression suite has 27 cases: **24 fail against the original source**, three
control cases pass. All 27 pass with the fixes (steering repair explicitly enabled
where it is being tested).

A 28th case covers the concurrent flight changes: an airborne scoop within horizontal
digging range is not physically excavating and must not receive the working exemption.

Four additional perception regressions bring the suite to **32 cases**. The tracker
previously resolved a confirmed report through an airborne or disconnected robot;
both negative cases now leave it pending, while a grounded connected robot still
resolves it. Separately, skipping camera frames for stationary robots skipped all
report resolution and expiry, even after buffered observations had arrived. Only frame
capture/detection is now movement-gated; report bookkeeping and underfoot sensing still
run. These enforce the perception/comms boundary and are not a rescue-uplift claim.

### Fixture outcomes — seed 42, 420 simulated seconds

| Configuration | Tier 3 | Rescued | Found | Lost | Mean rescue time |
|---|---|---:|---:|---:|---:|
| Original | off | 1/8 | 4/8 | 1 | 73.1 s |
| Execution fixes | off | 1/8 | 4/8 | 1 | 73.1 s |
| Execution + steering repair | off | 1/8 | 4/8 | 4 | 71.8 s |
| Original | scripted | 4/8 | 6/8 | 1 | 246.6 s |
| Execution fixes | scripted | 4/8 | 5/8 | 1 | 207.0 s |
| Execution + steering repair | scripted | 2/8 | 5/8 | 4 | 160.9 s |

**No rescue-count improvement on the original layout is established.** Execution fixes preserve rescue counts
in these two fixture arms; scripted discovery falls by one. The steering change is a
correctness improvement with a worse mission outcome here, so `--edge-steering` is
**opt-in, off by default**. A faster mean for fewer rescues is not an improvement.
The fixture is a functional check, not a substitute for the four demo maps.

A static construction of demo seed 42, without running a mission, checked 148,812
reachable (chassis, collection-point goal, coarse-cell) combinations. The repair
removed **3,629 blocked steering steps → 0** (wheeled 908, tracked 922, legged 941,
rotor 858). These are route choices, not rescued people, and the check does not prove
that a robot's body can follow every coarse path.

No local demo mission, training, gate, live model, or response-team comparison was
run. Unit policy, commander and zone-routing defaults remain off. A new same-machine
comparison on demo seeds 42–45 is required after the concurrent changes settle.

Raw fixture records, source/scenario/roster checksums and static steering counts:
[`runs/rescue_audit/fixture_comparison.json`](../runs/rescue_audit/fixture_comparison.json).

### Combined build — flight and new casualty placement held fixed

A second comparison uses frozen snapshots of the current combined build, `test`
seed 42, 420 sim-seconds, with steering experimental mode off in every arm. Here
`test.yaml` SHA256 is `47c1b1d8ce2d2f797f299705faf42a4b1527e4057a7860563ae1b6d97137092f`.
Do not compare these counts against the previous table's different casualty sites.

| Audit changes | Tier 3 | Rescued | Found | Lost | Scorecard hash |
|---|---|---:|---:|---:|---|
| None | off | 3/8 | 3/8 | 5 | `8e7bb841ff60c1ea` |
| Perception corrections only | off | 2/8 | 2/8 | 5 | `69d12dd9cb8cdc3c` |
| All default audit fixes | off | 2/8 | 2/8 | 5 | `69d12dd9cb8cdc3c` |
| None | scripted | 1/8 | 6/8 | 3 | `ae75f1277a107e0f` |
| Perception corrections only | scripted | 3/8 | 4/8 | 2 | `2e9b29dc80f5b5c2` |
| All default audit fixes | scripted | 3/8 | 4/8 | 2 | `2e9b29dc80f5b5c2` |

**Mixed result:** +2 rescues with the scripted advisor, −1 with Tier 3 silent.
The perception-only control produces byte-identical scorecards to the full patch
in both arms, so the observed count changes on this fixture belong to the sensing
corrections; the delivery defects are established by targeted regressions, not a
count gain here. The sensing restriction enforces the ground-camera/comms boundary
and is retained regardless of score. This is not evidence of demo-map uplift, a
trained component, or performance on an unseen map.

### Validation

`make check` passed on the frozen combined build: Ruff clean, **607 tests passed**
(including the 32 rescue regression cases), and the headless fixture completed at
t=420. Its scorecard hash `2e9b29dc80f5b5c2` exactly matches the prior scripted run.
All 176 snapshotted source/config files still matched the workspace after the check;
documentation and measurement artifacts were updated afterward. The
[check log](../runs/rescue_audit/check.log) and
[source manifest](../runs/rescue_audit/checked_sources.json) are retained with the results.

## M-85 — rotor flight, terrain clearance and dashboard altitude

The owner requested that flying chassis visibly fly above the ground fleet and climb
over mountains. Inspection found two independent gaps: `World.airborne` already removed
collision and rubble drag, but Tier 1 still used ground flow fields, obstacle repulsion
and the ground wall override; the bridge also omitted flight state, so every rotor model
was grounded regardless of the simulator's mode.

Flight is now selected before steering. Airborne rotors use direct horizontal guidance,
open-air bid fields and same-layer separation; map bounds and known-hazard avoidance
remain active. They land only on dry traversable support, wait for a ground camera
sample, and cannot dig or recharge aloft. No rotor carrier is introduced. The additive
`DashboardFlight` contract carries actual airborne flags alongside the unchanged eight
robot columns. Models, distant markers, follow cameras, culling and picking use that
state; missing telemetry and disabled units remain grounded.

The custom 2.5D simulator keeps a binary flight layer, with a deterministic display
altitude envelope over terrain/water. It provides 8 m clearance, includes the body
footprint and terrain LOD vertices, and limits each horizontal axis to 0.5 rise/run.
Aircraft therefore climb before a mountain face and remain level above ground traffic.
This is a kinematic/display representation, not an aerodynamic or 3D physics model.

Measured on this M1, constructing demo seed 42 only (103 rotors, no mission):

| Flight-height cache | Build | Resident array | Sample all 103 rotors |
|---|---:|---:|---:|
| Python offline renderer | 9.433 ms | 696,008 bytes | — |
| Godot initial window scan | 1,412.050 ms | 348,004 bytes | 0.297 ms |
| Godot linear window maximum | 261.977 ms | 348,004 bytes | 0.307 ms |

The cache is built once per displayed map, then sampled with four lattice lookups.
The native timing is 100 batches on a loaded development machine; it is not a full
stack memory or realtime benchmark and introduces no resident process.

Verification includes wall/water/slope/rubble crossings at flight speed; intact ground
collision and map bounds; first-tick takeoff; unsafe-landing rejection; aircraft/ground
separation; clearance over a synthetic 30 m cliff at every terrain LOD; Python/Godot
height parity; actual bridge flags; and native camera, picking, culling and landing checks.
An additional regression reproduced a disconnected rotor stuck waiting for shared fog
after its own camera inspection. Flight now recognizes that locally inspected cell while
keeping the observations buffered until reconnection; the regression passes.
Offline inspection frames are `runs/flight/above_ground_units.png` and
`runs/flight/clearance.png`; these are constructed visual fixtures, not mission results.

This intentionally changes rotor routes and mission outcomes. No rescue uplift, new
training or demo-map performance gate is claimed. Commander, unit policy and zone
routing remain off; previous gates describe the earlier movement/placement build.

Validation on the combined checkout: `make check` passed lint, **598 tests** (895.64 s
on the loaded laptop), and the seed-42 fixture smoke. After the final private-inspection
adjustment, all **25 focused flight, determinism, mission and terrain-safety checks**
passed as well. The smoke completed t=420 at 18.10× realtime (23.20 wall-seconds),
with hash `7222837e33e1d4e1`. This is the small `test.yaml` scenario and includes the
concurrent placement/rescue changes; it is not a demo-map comparison or performance gate.

## M-86 — positional landslide audio

The owner supplied two MP3s and selected a **5 m radius-growth step** for impacts.
Inspection of the fetched merged `main` (`6dc7e3d`) and available branch history found
no earlier audio manager. `/world/hazard_zone` is declared but has no publisher;
the existing dashboard receives the actual centre and radius through `truth.hz`.
The new dashboard audio manager uses that dispatcher and its existing state/event
handlers. No simulator, transport or frozen schema change is needed.

| Asset | Duration (`afinfo`) | Compressed audio bytes | Playback |
|---|---:|---:|---|
| `soundreality-landslide-128314.mp3` | 22.032 s | 705,024 | looping rumble |
| `floraphonic-rocks-and-gravel-slide-4-204995.mp3` | 5.256 s | 168,192 | one-shot impact over the rumble |

Both are stereo MP3, 48 kHz, 256 kbps; copied from the owner's Downloads folder into
`godot/audio/sfx/` with identical SHA-256 checksums. There is no new resident process.
Both players use built-in camera-relative attenuation at the reported hazard centre.

`godot/tests/audio_check.gd` passes in native Godot 4.7.2 with the dummy audio driver.
It exercises the actual dashboard dispatcher and imported assets: simultaneous
rumble/impact, loop rollover and one-shot completion, duplicate ignition and truth
frames, the 5 m boundaries including
decimal radii, decay/regrowth, skipped snapshots, unrelated/source-sector abandonment,
directive expiry, burnout, joining mid-mission, mission completion, reset and
disconnect. Growth does not restart the rumble. Playback positions verify that
duplicate frames do not restart either asset. This verifies playback behavior, not
the audible mix or subjective loop quality; no listening session is claimed.

Validation on the combined checkout: `UV_CACHE_DIR=/private/tmp/swarmmind-uv-cache
make check` passed lint, **618 tests** (533.58 s), and the seed-42 `test.yaml` smoke.
The smoke reached t=420 with hash `2e9b29dc80f5b5c2` in 20.67 wall-seconds. This is a
fixture check, not a demo-map performance comparison; the audio change is confined
to the dashboard.

## M-87 — hosted model moved to OpenRouter (openJiuwen team + hivemind API rung)

The owner supplied an OpenRouter key (`sk-or-v1-…`) and asked to switch the hosted model
from the direct OpenAI endpoint to it. openJiuwen itself issues no keys; it is the agent
SDK the native team already runs on, and it lists `OpenRouter` among its client providers.
Endpoint is now `https://openrouter.ai/api/v1/chat/completions`, key `OPENROUTER_API_KEY`,
model `openai/gpt-4.1-mini` (the same model). The key lives only in the gitignored `.env`.

OpenRouter serves this model from three hosts (OpenAI, two Azure regions), all listing
`structured_outputs`. Every OpenRouter request sets `provider.require_parameters` so it is
never routed to a host that would drop the JSON schema. No host lists
`parallel_tool_calls`, which openJiuwen sends from the team's request config: with
`require_parameters` every native call failed **HTTP 404, "No endpoints found that can
handle the requested parameters"**. The adapter already sends no tools, so it now omits
that field. Found by a live call; the HTTP test double cannot show it.

All runs below: this laptop, `test.yaml`, seed 42, realtime. Wiring and latency only —
no demo-map run, no rescue-uplift claim.

| Check | Result |
|---|---|
| Hivemind API rung, 40 sim-s (`--demo --hivemind-allow-api`) | 7/7 cycles answered by `api`; 0 timed out / errored / unparseable / fell through; latency **850–1,330 ms** |
| Native team, 65 sim-s (`--demo --response-team`) | 4 episodes: 3 applied, 1 rejected by the final filter (no fresh in-contact unit); 0 fallbacks; 16 calls, 7,373 tokens; applied-episode latency 3.5–6.9 s; 1.00× realtime |
| `check_response_team.py --runtime local` | **PASS** — logistics revised the lead's A1 to B1, safety approved |
| `check_response_team.py --runtime workswarm` (native), twice | **FAIL** both times — the lead chose the feasible B1 on its first call, so there was nothing for logistics to revise; 4 calls, 1,365 tokens each |

The native FAIL is recorded, not tuned away: the fixture is a trap for a lead that picks
unsupported rescue demand, and this model did not take it. It says nothing about whether
peer revision happens on real maps. Historical local-Qwen native episodes took 12.4–17.1 s
(M-82) on the seed-42 demo map; the figures above are the fixture and not a like-for-like
comparison.

Team orders were published with `source: "base-local"` even when hosted. They now use the
frozen contract's `"api"` for the OpenRouter endpoint (loopback keeps `base-local`).

Validation: lint clean; **624 passed, 2 failed** in 409.65 s. The two failures are
`test_native_burial_*` and `test_native_thermal_*`, 30 s timeouts inside headless Godot 4.7.2.
They fail identically without this change. Three more native-Godot tests failed on this checkout
until the gitignored class cache was built (`Godot --headless --path godot --import`). The isolated
SDK suite passed 5 coordination cases and 2 worker requests. The `test.yaml` seed-42 smoke hash
is unchanged at `2e9b29dc80f5b5c2` (headless never calls a model).

## M-88 — Nepal confluence crop (2026-09-19)

Owner-requested reconstruction from the supplied river-confluence/settlement photographs.
The map is **320 × 216 m**, down from 360 × 240 m: 69,120 cells at the same 1 m
resolution, **20% less area**. River widths are 14/18/22 m, roads 4.8 m and the 22
house footprints 6–8 m across. The photographs provide no scale bar or elevation survey;
these are authored local proportions, not georeferenced Nepal terrain. Scope and
reproduction commands: [NEPAL_TERRAIN.md](NEPAL_TERRAIN.md).

All measurements below are construction-only on this laptop: **zero mission ticks**,
512 robots, 110 casualties, seeds 42–45. They do not measure rescue performance.

| Seed | Build (s) | Occupancy-passable | Relief (m) | Wet area | Reach: wheels / tracks / legs |
|---|---:|---:|---:|---:|---:|
| 42 | 0.418 | 87.23% | 47.625 | 10.06% | 49.39% / 82.79% / 99.46% |
| 43 | 0.352 | 87.37% | 47.961 | 10.06% | 48.35% / 82.76% / 99.50% |
| 44 | 0.347 | 87.65% | 47.550 | 10.06% | 52.27% / 82.78% / 99.45% |
| 45 | 0.358 | 87.45% | 48.025 | 10.06% | 49.32% / 82.80% / 99.48% |

Reach is the base-connected fraction of occupancy-passable cells, with each chassis's
water and slope gates applied. Every ground chassis reaches all twelve collection
points on all four seeds; spawns are chassis-passable, house foundations are dry and
level, and no obstacle footprint occupies an interior wet channel. The two short road
crossings are dry causeways in the existing 2.5D model, not simulated bridge hydraulics.

Two implementation defects were caught before the final map: independently pinned
short roads had infeasible endpoint elevations, stranding wheels at 17% reach; project
junction heights onto the road graph's grade bound before grading the segments. House
pad shoulders could tilt the neighbouring foundation; apply the level cores after the
shoulders while retaining the road deck. Tests cover both outcomes.

Offline 1000 × 700 previews are in `runs/3d/nepal/`, with camera/geometry timings in
`review.json` and the per-seed construction numbers in `construction.json`. The renderer
leaves world arrays and RNG states unchanged. Python/Godot numerical parity covers both
generic terrain and the new forest/sediment/road colours. Godot's display layout is an
export of scenario YAML; an equality test prevents the two copies drifting.

The initial full check exposed the two native failures already recorded in M-87. Logs
identified the missing `.godot/imported/CurledUpPerson…scn` cache entry; reimporting the
existing assets repaired it, and the native burial/thermal tests then passed. No source
change to burial or thermal behaviour was needed. The independent `test.yaml` headless
smoke reached t=420 with the original hash **`2e9b29dc80f5b5c2`** (13.53 wall-seconds).

Historical gates and learned-component margins remain records of the previous terrain.
No full demo mission, training run or new performance gate was run for this change.

Final validation: the full run finished with **633 passed / 1 failed** in 362.82 s.
The remaining burial timeout had started before the cache repair. The follow-up
`UV_CACHE_DIR=/private/tmp/swarmmind-uv-cache PYTEST_ADDOPTS=--lf make check` passed:
ruff clean, the one previously failing test passed (0.84 s), and the complete fixture
smoke passed in 13.82 s with hash `2e9b29dc80f5b5c2`. Thus all **634 tests** passed
across the full run and its failed-test rerun; this is not a claim that the initial
full command exited successfully. Both native burial/thermal cases also passed together
after import. The final terrain/reference tests passed on all four demo seeds.

## M-88 — hosted model chosen by measurement: `openai/gpt-5.6-terra`

The owner asked for the fastest and best available models, cost disregarded. 268 OpenRouter
models in the main families advertise `structured_outputs`; 15 were tried on **both real
request shapes** — the Tier-3 directive call with the captured seed-42 demo-map prompt
(48 sectors, ~1,200 input tokens) and a full native openJiuwen team episode against a real
demo-map snapshot, through a recording proxy. A model counts only if the JSON validates
against the shipped schema, the same check the adapter makes.

Two failure modes decided most of it, and neither is visible without a live call:

- **`temperature` is a 404, not a warning.** Frontier models (GPT-5.6/GPT-6, Claude Opus 5)
  do not accept `temperature` on their structured-output hosts. With `require_parameters`
  the whole request fails: *"No endpoints found that can handle the requested parameters."*
  It is now sent only to models measured to take it.
- **Anthropic's flagships refuse this prompt.** `claude-opus-5` and `claude-fable-5.1`
  return `finish_reason: content_filter` with an empty body on every route and schema
  tried: *"This request triggered restrictions on violative cyber content."* A swarm being
  commanded over a disaster map reads as cyber content to the classifier. `claude-sonnet-5`
  answers, at 3.8–10.3 s. **No Anthropic model is usable here.**

Hidden reasoning tokens also count against `max_tokens`, so the old 200-token cap truncated
every reasoning model's answer. Each model's reasoning control, temperature and token cap
now live in `TUNING` in `providers/openai_api.py`, measured rather than guessed.

Directive call, 8 samples each on the demo-map prompt, schema-validated:

| model | valid | median | max | $/call | note |
|---|---|---|---|---:|---|
| `openai/gpt-6-astra` | 8/8 | 4.94 s | 6.34 s | 0.0089 | flagship; needs 1,200 tokens for thinking |
| `openai/gpt-6-astra-pro` | 8/8 | 6.33 s | **8.29 s** | 0.0441 | exceeds the 8 s rung timeout |
| `openai/gpt-5.6-sol` | 8/8 | 1.65 s | 1.81 s | 0.0054 | |
| **`openai/gpt-5.6-terra`** | **8/8** | **1.51 s** | **1.77 s** | 0.0012 | **shipped, owner's choice** |
| `openai/gpt-5.6-luna` | 8/8 | 1.20 s | 2.24 s | 0.0001 | cheapest fast tier |
| `google/gemini-3.1-pro-preview` | 3/3 | 7.15 s | — | 0.0122 | too slow for a 6 s cycle |
| `x-ai/grok-4.6` | 3/3 | 10.72 s | — | 0.0061 | too slow |
| `google/gemini-3.8-flash` | 3/3 | 1.03 s | — | 0.0012 | directive ok, **team episodes time out** |
| `z-ai/glm-5.3` | 3/3 | 0.49 s | — | 0.0018 | fastest usable |
| `deepseek/deepseek-v4.1-flash` | 3/3 | 0.57 s | — | 0.0002 | |
| `anthropic/claude-*` | 0 | — | — | — | refused or prose, see above |
| `openai/gpt-4.1-mini` (previous) | 8/8 | 1.10 s | — | 0.0003 | |

Live on the demo map, seed 42, 180 simulated seconds each, `gpt-5.6-terra`, this laptop:

| run | model work | result | cost |
|---|---|---|---:|
| `--response-team` | 9 episodes, 36 calls, 17,093 tokens, episode latency 4.1–4.3 s | 4 applied, 5 rejected as stale/infeasible, **0 fallbacks**, 46/110 rescued, **1.00× realtime** | **$0.028** |
| `--hivemind-allow-api` | 20/20 cycles answered by the `api` rung, 1.33–1.61 s | 0 timed out, errored, unparseable or fallen through; 0 filter rejections; 48/110 rescued, **1.00× realtime** | **$0.083** |

Both held 1.00× realtime with 512 robots, so the model is not on the simulation's critical
path. The five rejected team proposals were refused by the existing feasibility filter, not
by the model. Rescue counts from single runs on one seed are **not** a performance
comparison: no uplift over the scripted baseline is claimed or measured here.

**Which map these ran on.** `demo.yaml` was being rewritten in the same working tree while
this was measured (the authored-landscape work: 360x240 m becomes 320x216 m). The captured
directive prompts, and so the 8-sample latency table, come from the **previous** demo map;
the two 180 s live runs come from the **revised** one. Both maps carry 48 sectors and a
~1,200-token prompt, so the latency and cost figures stand, but the rescue counts above are
not comparable with any earlier row, and were never a comparison to begin with.

Validation for M-87 and M-88: lint clean; the 116 tests covering the provider, team,
hivemind, mission, determinism, contract and ground-truth boundaries pass, as do the
isolated SDK suite's 5 coordination cases and 2 worker requests; `test.yaml` seed-42 smoke
hash unchanged at `2e9b29dc80f5b5c2`. The full suite was not re-run clean here because the
concurrent terrain work in the tree owns part of it.

## M-89 — spawn, orders, river stops and delivery recovery (2026-09-19)

Owner-reported symptoms: clusters at spawn, units waiting in the river, loaded carriers
not delivering promptly, and a jagged/flat-looking river. Baseline source: `68abb03`.
Measurements below are from this Mac, Python 3.12, with no hosted-model requests,
training, gate or full demo mission. This is execution repair and display work.

### Reproduced mechanisms and repairs

- A valid fine-grid spawn cell did not imply a body-clear, dry or coarse-connected
  starting position. Repair only invalid starts, deterministically, while reserving
  valid original placements and preserving the robot RNG stream.
- An idle goal cache could outlive the fog/communications state that produced it.
  Refresh every 2 s and immediately on changed sector orders. Idle search honours
  reachable priorities and avoids abandoned ground; high-priority search can redirect
  ordinary exploration without taking robots off delivery, digging or relay posts.
- A centroid of bank frontier cells could land in water; rounding a valid bank cell
  centre could also move its goal into the adjacent river. Choose an actual dry frontier
  member and retain goal coordinates. Computed idle relay positions seek a dry bank.
  Component labels now honour fine-derived routing edges and exclude the invalid `-1`
  component; aircraft can seek dry landing sites across disconnected ground regions.
- Coarse descent can still trap a robot against fine terrain. After 6 s with less than
  1 m displacement, Tier 1 requests a shared fine-grid detour over an approximately
  32 m patch. It builds at most one per tick and caches 64. Routes prefer body clearance,
  forbid diagonal corner cutting, and retain hazard/separation and swept-circle safety.
  Loaded carriers keep extraction assignments. Neither historical experimental
  `zone_routing` nor `edge_steering` is enabled by this change.
- Leader/team snapshots now include connected assignment and payload counts. Relay
  narration distinguishes movement from a useful hold, and stalled-route recovery is
  visible in the robot's reason. No frozen contract or topic changed.

### Construction-only demo check — seed 42

The intermediate execution repair exposed **20 of 512 robots without a usable initial
search goal**, including 17 on fine-passable cells labelled blocked by coarse navigation.
After the spawn correction, **512/512 have goals**, shared across 39 target positions;
**zero goals are wet**. Every ground robot has a finite coarse route to base, every
spawn passes body landing clearance, all starts are dry, and no two use the same cell.
These are construction checks with **zero simulation ticks**, not a mission score.

Initial goal construction took **62.8 ms**. Twelve cold recovery queries, including
coarse field creation, took **4.21–5.73 ms** each. Their cached direction arrays occupied
12,288 bytes and the four clearance masks 276,480 bytes. The 64-patch limit is about
64 KiB of direction grids at the demo's 1 m resolution. No new resident process.
Raw data: [`construction.json`](../runs/movement_audit/construction.json).

### Same-machine fixture comparison — test.yaml, seed 42, 420 s

**The first table below is a superseded diagnostic, retained as history.** Its sampler
called `executor.goals()` after a tick, refreshing the new time-sensitive goal cache
ahead of perception/assignment bookkeeping. That can change trajectories. The final
headless smoke hash exposed the mismatch. These rows describe the instrumented probe,
not the normal mission; the corrected read-only comparison follows below.

The fixture is small (16 robots, 8 casualties) and does not measure the Nepal crop.
The source, scenario, seed and tier configuration are fixed within each before/after
comparison. Wall timings were affected by concurrent checks and are not a throughput
benchmark. Stationary means moving under 0.1 m over a one-second sample while alive;
"away from goal" additionally requires an assigned/fallback goal more than 1.5 m away.
Counts below are summed robot-seconds, not unique robots or causal rescue attribution.

| Tier 3 | Build | Rescued | Found | Lost | Stationary away from goal | Loaded stationary | Wet stationary |
|---|---|---:|---:|---:|---:|---:|---:|
| off | before | 2/8 | 2/8 | 5 | 836 | 0 | 721 |
| off | after | 4/8 | 4/8 | 2 | 317 | 5 | 0 |
| scripted | before | 3/8 | 4/8 | 2 | 940 | 70 | 350 |
| scripted | after | 2/8 | 3/8 | 2 | 348 | 7 | 11 |

**The invalid probe appeared mixed:** stationary behavior improves in both arms, but scripted
rescues fall **3 → 2** and discovery **4 → 3**. The silent arm improves **2 → 4** rescues.
A faster mean rescue time for fewer rescues is not a rescue-rate improvement. These
probe results cannot establish a regression or uplift in the normal mission. The new
spawn constraints and explicit instruction priority alter trajectories; a live Nepal
rehearsal remains necessary before claiming a better seven-minute rescue result or
a benefit from the hosted leader.

Scorecard hashes: off `69d12dd9cb8cdc3c` → `8ff234b35824f27e`; scripted
`2e9b29dc80f5b5c2` → `a13497522a98c6a4`. Raw results:
[`before_probe.json`](../runs/movement_audit/before_probe.json),
[`after_probe.json`](../runs/movement_audit/after_probe.json). Corrected reproduction harness:
[`fixture_compare.py`](../runs/movement_audit/fixture_compare.py), taking a source
checkout path and output JSON path as arguments. Baseline sources were extracted from
`68abb03`; both arms use the project's Python 3.12 environment.

### River display and regression coverage

Shared shoreline vertices round inward by up to half a cell, retaining the exact dry
crossing mask and matching across streamed tiles. Water uses depth-dependent teal/silt
colours, channel-aligned downstream ripples, reflection ribbons and broken shallow foam.
`viz/water.py` provides the static shader reference. This adds no textures, processes,
physics or perception inputs. The offline render precedes the Godot port; views are
[`before`](../runs/3d/river_before/river.png) and
[`after`](../runs/3d/river_after/river.png), with camera/timing/fingerprint JSON beside them.
Live appearance has not been manually rehearsed in the dashboard.

Twelve targeted movement regressions cover goal refresh, priority and assignment
precedence, dry targets, river-bank relay positions, aircraft crossing, invalid
components, preemption protection, bounded recovery work, a loaded carrier physically
escaping a U-shaped pocket and delivering with wall safety active, legitimate relay
holds, and valid demo spawning. Existing exhaustive shoreline-mask/tile-seam checks
pass; additional coverage verifies downstream coordinates, animated colour variation,
and the team's connected-only workload telemetry. Full check record:
[`check.log`](../runs/movement_audit/check.log).

### Corrected read-only comparison and final validation

The sampler now reads the executor's cached goals from the last control tick and never
calls methods that can refresh them. The scripted result's hash matches the independent
`make check` headless smoke exactly, resolving the probe discrepancy above.

| Tier 3 | Build | Rescued | Found | Lost | Stationary away from goal | Loaded stationary | Wet stationary |
|---|---|---:|---:|---:|---:|---:|---:|
| off | before | 2/8 | 2/8 | 5 | 810 | 0 | 721 |
| off | after | 3/8 | 4/8 | 2 | 533 | 12 | 62 |
| scripted | before | 3/8 | 4/8 | 2 | 931 | 70 | 350 |
| scripted | after | 4/8 | 4/8 | 3 | 333 | 9 | 47 |

On this fixture, both configurations rescue one additional casualty. Stationary time
away from goals decreases **34%** with Tier 3 off and **64%** with scripted leadership.
The scripted arm's loaded stationary time falls **70 → 9 robot-seconds** and wet
stationary time **350 → 47**. Tradeoffs remain: scripted losses increase **2 → 3**, and
mean rescue time rises **169.7 → 236.5 s** while rescuing more casualties. These are
small-fixture observations, not evidence of hosted-leader or Nepal-map performance.

Final hashes: off `9a938f620b49a562`; scripted `a7cb3f81ea18f690`.
Corrected raw records: [`before.json`](../runs/movement_audit/before.json) and
[`after.json`](../runs/movement_audit/after.json). Source/test checksums are in
[`source_manifest.json`](../runs/movement_audit/source_manifest.json).

`make check` passed: Ruff clean, **653 tests passed**, and the headless seed-42 fixture
completed at t=420 s with **4/8 rescued, 4/8 found**, hash `a7cb3f81ea18f690`.
The suite reported two existing NumPy empty-slice warnings in the sampled-scenario
fixture, with no failures. The exact/legacy collection-coordinate compatibility change
was additionally checked with all five zone-routing tests and Ruff. Safety, determinism,
Tier-3-off execution, contract and ground-truth-boundary checks remain green.

## M-90 — 20% faster demo travel; 180 s rescue target (2026-09-19)

Owner request: slightly faster units, with most rescues completed by t=180 s.
`demo.yaml` now sets `robot_speed_multiplier: 1.20`. The scenario applies that factor
to every robot's translation limit after loading either the evolved or fallback roster.
Candidate morphology overrides in MAP-Elites respect the same scenario factor. No
training was run. Scenarios without the setting retain 1.0, including `test.yaml`.
The setting must be finite and positive.

Terrain drag, loaded-carrier penalties and airborne speed still multiply the resulting
limit. Turning limits, the 20 Hz simulation clock, casualty count, extraction points,
hazard schedule and mission duration do not change. At a fixed heading on identical
terrain, 20% greater speed cuts travel time by 16.7%; it does not compress the entire
rescue sequence by that amount.

### Demo construction only — seed 42, zero ticks

| Lane | Previous median rated speed (m/s) | New median rated speed (m/s) |
|---|---:|---:|
| scout | 1.876 | 2.251 |
| digger | 0.643 | 0.772 |
| carrier, unloaded | 1.831 | 2.197 |
| carrier, loaded on clear ground | 1.281 | 1.538 |
| relay | 1.478 | 1.774 |

These are speed limits, not measured mission-average velocities. All 512 robots receive
the factor. Raw values: [`construction.json`](../runs/speed_audit/construction.json).

### Same-machine timing comparison — test.yaml, seed 42, scripted Tier 3

Only the scenario speed multiplier changes between arms. The sampler reads world
counters after ordinary mission ticks; it never refreshes goals or changes control state.

| Multiplier | Rescued at 60 s | at 120 s | at 180 s | at 420 s | Found at 420 s | Robots lost |
|---|---:|---:|---:|---:|---:|---:|
| 1.00 | 0 | 2 | 2 | 4 | 4 | 3 |
| 1.20 | 1 | 2 | 3 | 3 | 3 | 2 |

There are **8 total casualties** in this fixture. Early rescues improve **2 → 3**,
but final discovery and rescues both fall **4 → 3**. Speed changes search trajectories;
this is an early-completion tradeoff, not a demonstrated improvement in total rescues.
The faster arm delivers at 55.95, 93.55 and 145.50 s; the baseline at 67.35, 114.10,
355.75 and 408.65 s. Rescuing all three discovered casualties by 180 s does **not** mean
most of the eight casualties have been rescued.

Hashes: 1.00 `a7cb3f81ea18f690` (unchanged fixture baseline), 1.20
`dc1f479ddac07bdc`. Reproduction:
[`measure.py`](../runs/speed_audit/measure.py),
[`timing.json`](../runs/speed_audit/timing.json).
The full Nepal demo with hosted leadership has not been rerun; **most of its 110
casualties rescued by 180 s remains an unverified target**. The 20% adjustment implements
the requested modest speed increase, not a claim that this deadline is met.

Physical-motion checks cover both evolved and fallback bodies, loaded ground travel,
airborne travel, terrain drag and unchanged simulation time. The existing 60 s wall
collision check also runs at 1.20×. All five targeted cases pass. Full-suite output is
kept in [`check.log`](../runs/speed_audit/check.log).

Final `make check`: Ruff clean, **656 tests passed** (two existing NumPy empty-slice
warnings), followed by a successful headless seed-42 fixture: **4/8 rescued, 4/8 found**,
hash `a7cb3f81ea18f690`. The default fixture remains byte-identical; only the demo
configuration opts into faster travel.
