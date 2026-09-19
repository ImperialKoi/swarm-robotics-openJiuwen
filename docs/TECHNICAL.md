# SwarmMind — Technical Plan

Companion to [PLAN.md](PLAN.md). This is the design of record: interfaces, algorithms, schemas, numbers.
Anything here marked **FROZEN** must not change without updating every consumer in the same commit.

**Optional extension:** the opt-in response team is specified in
[MULTI_AGENT_DEMO.md](MULTI_AGENT_DEMO.md). It runs WorkSwarm in an isolated process,
uses detached observed-state tools, and returns peer-reviewed proposals to the existing
HivemindNode filter on the simulation thread. Private traces do not extend the frozen
bus contracts. Existing scripted fallback and 30-second directive expiry remain active;
team leases prevent fallback from overwriting a live reviewed order in the same sector.
M-79 records the actual runtime and memory measurements, superseding budget assumptions
for this optional configuration.

---

## 1. Architecture

```
┌──────────────────────────────────────────────────────────────┐
│  Godot 4 dashboard  (native macOS, no ROS2 dependency)       │
└───────────────────────────▲──────────────────────────────────┘
                            │  WebSocket, JSON lines, 10 Hz
┌───────────────────────────┴──────────────────────────────────┐
│  bridge_node — the only transport-aware node                 │
└───────────────────────────▲──────────────────────────────────┘
                            │
┌───────────────────────────┴──────────────────────────────────┐
│  Bus  (LocalBus in-process asyncio | Ros2Bus rclpy)          │
│  topic names and payload shapes identical on both            │
└──▲────────▲────────▲────────▲────────▲────────▲──────────────┘
   │        │        │        │        │        │
blackboard auction hivemind hazard  events  fault_injector
   │        │        │        │        │        │
   └────────┴────────┴───┬────┴────────┴────────┘
                         │
        ┌────────────────┴─────────────────┐
        │  N × skill_executor  (Tier 2)    │
        │  N × reflex_controller (Tier 1)  │
        └────────────────▲─────────────────┘
                         │  MotorCmd / WorldState
        ┌────────────────┴─────────────────┐
        │  SimBackend                      │
        │  ├─ FastSim   headless, training │
        │  ├─ DemoSim   realtime, the demo │
        │  └─ GazeboSim optional, stretch  │
        └──────────────────────────────────┘
```

**Two invariants that everything else follows from:**

1. **`FastSim` and `DemoSim` are the same `World` class**, differing only in how the tick loop is clocked. Training and the demo therefore share physics exactly. There is no sim-to-sim transfer problem on the primary path.
2. **Tier 2 never reads `/hivemind/directives` as a requirement.** Directives are an *optional modifier* on task priority. With the topic silent, the auction must still allocate, execute, and self-heal. `test_mission.py` runs with the hivemind disabled and must pass.

---

## 2. Repository layout

```
swarm-robotics/
├── CLAUDE.md
├── SHIPPING.md                    # what is trained vs heuristic
├── pyproject.toml                 # uv, requires-python = ">=3.12,<3.13"
├── Makefile                       # make check / demo / headless / stress
├── docs/
│   ├── PLAN.md  TECHNICAL.md  RUNBOOK.md
├── swarmmind/
│   ├── contracts/                 # ★ FROZEN — the interface contract
│   │   ├── topics.py              # topic name constants
│   │   ├── schemas.py             # pydantic models, §4
│   │   └── version.py             # SCHEMA_VERSION = "1.0"
│   ├── bus/
│   │   ├── base.py                # Bus protocol
│   │   ├── local.py               # asyncio in-process
│   │   ├── ros2.py                # rclpy adapter (optional import)
│   │   └── ws_server.py           # WebSocket fan-out to Godot
│   ├── sim/
│   │   ├── world.py               # grid, sectors, victims, hazard, comms
│   │   ├── robot.py               # unicycle body, battery, actuator
│   │   ├── grid.py                # generation, connectivity, LOS, distance fields
│   │   ├── scenario.py            # YAML loader + validation
│   │   ├── backend.py             # SimBackend protocol + FastSim + DemoSim
│   │   └── gazebo/                # optional, stretch
│   ├── nodes/
│   │   ├── blackboard.py  auction.py  hazard.py  hivemind.py
│   │   ├── skill_executor.py  events.py  fault_injector.py  bridge.py
│   ├── control/
│   │   ├── tier1_reflex.py        # 20 Hz, the safety floor
│   │   ├── tier2_policy.py        # BehaviorParams → skill behavior
│   │   ├── heuristic.py           # the classical baseline (gate reference)
│   │   └── planner.py             # flow fields (NavFields), A*, frontier
│   ├── hivemind/
│   │   ├── prompt.py  filter.py  fallback.py
│   │   └── providers/{local_gguf.py, anthropic.py, scripted.py, base.py}
│   ├── training/
│   │   ├── mapelites/{genome.py, evaluate.py, run.py, archive_io.py, select.py}
│   │   ├── llm/{rollout.py, reward.py, dataset.py, raft.py, dpo.py, grpo.py}
│   │   ├── gate.py                # §8, the shipping decision
│   │   └── notebooks/             # Kaggle .ipynb, thin wrappers only
│   ├── metrics.py                 # Scorecard
│   ├── mission.py                 # assembly: world + nav + Tier 1 + Tier 2
│   ├── rng.py                     # seeded per-subsystem streams
│   └── cli.py
├── godot/                         # Godot 4 project
│   ├── project.godot
│   ├── scenes/{Main,MapView,RobotMarker,ReasoningFeed,EventLog,Hud,Ticker}.tscn
│   ├── scripts/{ws_client.gd, map_view.gd, robot_marker.gd, ...}
│   └── shaders/fog.gdshader
├── assets/
│   ├── scenarios/{demo.yaml, train_*.yaml, fallback_directives.yaml}
│   ├── archives/                  # pyribs archive .csv/.pkl
│   └── models/                    # GGUF files (gitignored, checksummed)
├── tests/
└── scripts/{stress.py, record_demo.sh, export_gguf.sh}
```

---

## 3. World model

### 3.1 Geometry

- Map `320.0 × 208.0` m, occupancy grid at **`1.0` m** → `320 × 208` = 66,560 cells, `uint8` (`0` free, `1` wall, `2` rubble-passable-slow). 43,841 passable at the shipped density.
- **Why 1.0 m, not 0.5 m:** distance-field cost is `O(cells × diameter)`, so a 4× finer grid on a 10× larger map is ~40× the auction cost (MEASUREMENTS.md M-4). Robot radii (0.22–0.45 m) still sit inside one cell, so collision response is unchanged.
- 48 sectors: rows `A..F` (y) × cols `1..8` (x), each 40.0 × 34.7 m.
- Rubble: procedurally generated from the scenario seed — elliptical obstacle clusters, then `grid.ensure_connected` walls off every region unreachable from base. Filling pockets is preferred over regenerating: it is deterministic and cannot loop. Density is tuned to ~0.66 passable (MEASUREMENTS.md M-2); `tests/test_world.py` fails the build outside `0.55 < frac < 0.98`.

### 3.2 Fog of war

Per-cell `explored: bool` bitmask. A cell flips explored when any robot is within `sensor_radius` **and** has line of sight. `sector.explored_pct = explored_free_cells / total_free_cells`. Reveal is proximity-based, not SLAM — say so to judges.

**Decimation (required at high `N`):** the sensor/LOS pass runs at **5 Hz**, not 20 Hz, and only for robots whose position has moved more than half a cell since their last update. LOS is a 3-sample check along the segment robot→cell (t = 0.25/0.5/0.75), vectorized over a precomputed disc-offset template — not per-cell Bresenham. At 64 robots this is ~150k array lookups per update, five times a second. Exact Bresenham per cell at 64 robots × 20 Hz is roughly 60× more work for a fog edge nobody can see.

### 3.3 Victims

```python
@dataclass
class Victim:
    id: str; pos: tuple[float, float]
    buried: bool                    # 4 of 8; requires clear_debris first
    debris_remaining: float         # 0.0..1.0, decremented by scoop robots
    state: Literal["hidden","found","cleared","carried","rescued"]
    carrier: str | None
```

Placement: 80 victims drawn from reachable free cells, weighted by `geodesic_distance_to_base ** 1.5` (geodesic, not euclidean — a victim behind a wall should read as far away). Weighted-without-replacement via a **Gumbel top-k** draw, not `rng.choice(replace=False, p=…)`, which is an O(n·k) Python loop over ~44k candidate cells. Minimum pairwise separation 14 m. 32 flagged `buried`, drawn from the farthest 60%.

**Rescue chain (this is what forces all four lanes to matter):**
`hidden → found` (any robot within `sensor_radius`, in comms) → `found → cleared` (scoop robot at the victim reduces `debris_remaining` to 0 at `0.25/s`; non-buried victims start cleared) → `cleared → carried` (gripper robot attaches; carrier speed ×0.6 while loaded) → `carried → rescued` (carrier reaches an extraction zone).

Only `rescued` counts. Extraction zones, radius 6 m: base `(12,16)`, three corners, and **a centre zone at `(160,104)`**. The corners alone leave the map centre 160 m from the nearest zone — a 3.7-minute one-way trip for a loaded carrier at 0.72 m/s. With the centre zone: median 64 m, worst 108 m. Asserted in `tests/test_world.py`, which is how the missing zone was caught.

### 3.4 Hazard

Ground truth owned by `hazard_node`. Ignites at `t = 90 s` at a seeded passable cell inside sector `C4`.

```
r(t)      = 6.0 + 0.22 * (t - 90)           # metres, ~79 m by t=420
centre(t) = origin + drift_dir * 0.40 * (t - 90)
```

Scaled with the map: on 320 x 208 m a 28 m disc threatens nothing. At 79 m it covers roughly four sectors, which is what forces the hivemind's reprioritise beat.

A cell is hazardous if inside the disc. `sector.hazard_level = hazardous_free_cells / total_free_cells`, clamped `[0,1]`. Robots inside drain battery at `1.5 %/s` and are destroyed at `0%`. The hazard is **not** published on `/swarm/state` — the swarm only knows about hazard in cells it has observed, which is what makes the hivemind's reprioritization a real decision rather than a lookup.

### 3.5 Comms model (why relays matter)

- Base at `(12,16)` broadcasts with radius `40 m`.
- Any robot with `actuator == "antenna"` and status `active` relays with radius `38 m`.
- Build an undirected graph over {base, all antenna robots}; edge if within the smaller of the two radii. A non-antenna robot is **in comms** if it is within `38 m` of any node in base's connected component. 128 relays at 38 m cannot blanket 320 × 208 m, so relay placement stays a decision rather than a formality.
- Out-of-comms robots: buffer discoveries locally, do not appear to update on the blackboard (last-known position is shown greyed), cannot receive task awards. On reconnect, the buffer flushes and everything lands at once — which reads beautifully on the event log.

**Comms-recovery reflex.** A robot outside the component cannot be awarded work, so an idle one would sit where it is forever — and because its discoveries stay buffered, the shared map stops growing, the frontier freezes, and the entire swarm deadlocks. An unassigned out-of-comms robot therefore walks back toward base on its own. This needs no coordination, which is the point.

Implemented with a union-find over vectorized pairwise distances, rebuilt each auction cycle (1 Hz), not per tick. The cost is `O(N × relays)`, and it is the term that eventually bites: it is visible in MEASUREMENTS.md M-1 as RSS climbing 42 → 134 → 418 MB between N=1024 and N=6144. At N=512 the temporary is ~1 MB and the pass is ~1 ms.

**Out-of-comms buffering is exact, not approximate.** Each disconnected robot holds its own pending list of revealed cells and victim sightings; reconnecting flushes only that robot's list. A shared global pending mask would be cheaper and wrong — one robot reconnecting would publish every disconnected robot's discoveries.

### 3.6 Robot body

```python
@dataclass
class RobotBody:
    actuator: Literal["none","gripper","scoop","antenna"]
    radius: float          # 0.20 .. 0.55 m      (chassis_scale)
    mass: float            # ∝ radius**2
    v_max: float           # 0.6 .. 2.4 m/s      (motor_power / mass)
    omega_max: float       # 1.5 .. 4.0 rad/s
    sensor_radius: float   # 3.0 .. 9.0 m
    battery_capacity: float
    payload: bool          # gripper only
```

Speed and size are *coupled* through `v_max ∝ motor_power / mass` — the fast/light ↔ slow/heavy tradeoff is emergent from the genome, not a hand-set dial. Unicycle kinematics, `dt = 0.05 s`, circle-vs-grid collision with slide response.

### 3.7 Battery

`0.05 %/s` idle + `0.20 %/s` moving (scaled by `v/v_max`) + `1.5 %/s` in hazard. No recharge. Sized so a robot lasts ~500 s of continuous motion — long enough to finish, short enough that battery is a live term in the bid function.

### 3.8 `SimBackend` protocol

```python
class SimBackend(Protocol):
    def reset(self, scenario: Scenario, seed: int) -> WorldState: ...
    def step(self, cmds: dict[RobotId, MotorCmd]) -> WorldState: ...
    def spawn(self, specs: list[RobotSpec]) -> None: ...
    @property
    def sim_time(self) -> float: ...
    @property
    def dt(self) -> float: ...
```

`FastSim.step` returns immediately. `DemoSim.step` sleeps to the wall clock, reports realtime-factor, and drops to catch-up mode (no sleep) if it falls more than 100 ms behind. `GazeboSim.step` publishes `cmd_vel` and blocks on `/clock`.

**Vectorization:** robot state is stored as parallel numpy arrays (`pos[N,2]`, `theta[N]`, `v[N]`, `battery[N]`, …), not a list of objects. Integration, collision, sensing, comms and battery are whole-array operations. There is no per-robot Python object in the hot loop and there must never be one.

**Choosing `N` — measured, not chosen (D2).** `scripts/stress.py` sweeps `N ∈ {16, 32, 48, 64, 96, 128, 192, 256}`, runs a full 420 s mission at each, and reports `DemoSim` RTF, `FastSim` steps/s, peak RSS, and per-phase tick cost. **Ceiling = the largest `N` holding RTF ≥ 0.8×. Ship `floor(0.8 × ceiling)`.** Planning default until measured: **64**.

Three things must be true for that sweep to be honest, and all three are design constraints, not optimizations:
1. **Auction bidding is O(1) per robot per task** — a BFS distance field per task target, not A\* per pair. See §5.2.
2. **Fog updates are decimated** to 5 Hz with a movement gate. See §3.2.
3. **Nothing in the tick loop iterates robots in Python.** If a profile shows a `for` over robots outside of the 5 Hz sensor pass, that is the bug.

Re-run the sweep with Godot and `llama-server` resident — the ceiling under real memory pressure is the one that matters, and on 8 GB it will be lower than the clean-room number.

---

## 3.9 Perception — real computer vision

Until D3, victim discovery was `seen = dist <= sensor_radius` evaluated against ground-truth
victim coordinates: the robot was *told*. There was no image and no detector, so the claim
"the robots have computer vision" was false. This is the replacement.

### 3.9.1 The split that matters

| Layer | Who owns it | Why |
|---|---|---|
| **Appearance raster** | simulation | What the world looks like. Ground truth, rendered once per tick. |
| **Optics** — crop, rotate, occlude | simulation | What light physically reaches each camera. A real camera cannot see through a wall; computing that geometrically is *simulating optics*, not shortcutting perception. |
| **Detector** — pixels → victim reports | **the robot** | The part that must be real. Runs unchanged on hardware. |
| **Blackboard** | swarm | Only ever sees detector output, never the raster. |

Everything below the detector is physics. Everything from the detector up is the robot.

### 3.9.2 Appearance raster, not a semantic map

The raster is **RGB-like appearance with noise**, rendered once per tick at grid resolution.
Victims deliberately **do not get their own channel** — a channel that says "victim here" makes
detection a threshold and the CV claim hollow again. Instead, materials have overlapping
appearance and the detector must *discriminate*:

| Material | Appearance | Confusable with |
|---|---|---|
| floor | dark, low texture | — |
| wall | light, flat | — |
| rubble | mid, mottled, warm-tinted variants | **victims** |
| victim | warm, compact, high-saturation blob | warm rubble |
| hazard | bright, saturated, flickering | — |
| robot | cool, compact | victims at low resolution |

Per-frame Gaussian sensor noise is applied at the camera, not the raster, so two robots looking
at the same cell get independently noisy pixels. False positives and missed detections are
therefore real, seed-reproducible, and measurable.

### 3.9.3 Batched egocentric cameras

Rendering 512 separate camera views is infeasible on an M1 — 512 robots x 5 Hz is 2,560
renders/second. The world is 2.5D top-down, so a view is an **affine crop-and-rotate sample of
the one shared raster**: a batched gather, not a render.

For robot `i` at `(x, y, θ)`, image pixel `(r, c)`:

```
forward = (r + 0.5) / H * range                 # metres ahead
lateral = ((c + 0.5) / W - 0.5) * 2 * half_width(forward)
world   = pos + R(θ) @ (forward, lateral)
```

- Image `H x W = 48 x 48`, 3 channels, `uint8` -> 512 x 48 x 48 x 3 = **3.5 MB per batch**.
- Occlusion: one raycast **per image column** (per bearing), marching until the first wall.
  512 x 48 = 24.5k rays x ~8 steps = ~200k operations, fully vectorised. Samples beyond the
  column's visible range are blacked out.
- Runs at **5 Hz** with the same half-cell movement gate as the old fog pass.

Cost scales with `robots x H x W`, not with world size. Measure it and record in
MEASUREMENTS.md; if it does not fit, drop `H x W` before dropping `N`.

### 3.9.4 Detectors — classical first, learned must beat it

Same discipline as everywhere else in this project (CLAUDE.md working rules).

**`perception/classical.py` — ships by default.** Colour-distance thresholding in the victim
hue band, morphological cleanup, connected components, area and aspect gates. Pure numpy, no
ML runtime, no install, deterministic. Emits `(bearing, range, confidence)` per blob, which the
camera geometry converts back to a world position.

**`perception/cnn.py` — trains on Kaggle GPU.** Small conv net over the 48x48x3 batch,
trained on rendered frames with ground-truth labels from the simulator. Must beat the classical
detector on held-out seeds at the gate (§8) or the classical one ships.

Both expose the same interface, so nothing above perception changes when the detector is swapped:

```python
class Detector(Protocol):
    def detect(self, frames: np.ndarray) -> list[list[Detection]]: ...
    # frames: (N, H, W, 3) uint8  ->  per-robot list of Detection(bearing, range, conf, kind)
```

### 3.9.5 From detections to beliefs (`perception/tracker.py`)

A single detection is not knowledge. At 5 m the detector's precision is 0.48, so acting
on one sighting sends half the swarm to rubble. The tracker clusters sightings into
**reports** and moves them through `candidate -> confirmed -> resolved | dismissed`.

Promotion needs **both** of:

- **Parallax:** three observations from vantage points more than 2 m apart. Repeated
  looks from one spot are not independent evidence -- a robot parked in front of warm
  rubble otherwise manufactures corroboration at 5 Hz.
- **One close look:** confidence weighted by a range-trust ramp taken from the measured
  precision curve (full below 2 m, zero beyond 5 m). A crowd of distant, near-worthless
  sightings must not promote each other.

**Resolution** happens when any robot comes within 3 m: it either matches a real
casualty (`mark_found`) or is **dismissed**. Dismissed reports are kept for the whole
mission and absorb later sightings of the same spot -- they are the swarm's memory of
"we looked there, nobody home". Without that, a robot walks to a rock, sees nothing, and
the swarm rediscovers the same rock a minute later, forever.

Getting these three rules wrong took confirmed-report precision from 4% to 44%
(MEASUREMENTS.md M-8). Detections from out-of-comms robots buffer in the tracker and
publish on reconnect, exactly as observed cells do in the world.

**`investigate` is a first-class task kind.** A confirmed report is not yet a casualty,
so the swarm sends a robot to look. Roughly half of long-range contacts turn out to be
rubble, and that wasted trip is the honest price of perceiving rather than being told --
it is also a good demo beat, visible in the event log as `report_dismissed`.

### 3.9.6 What this changes upstream

- **Victim discovery is probabilistic.** A victim becomes `found` when a robot's detector
  reports it above a confidence threshold *and* the robot is in comms. Misses and false
  positives are now possible, so mission tuning and `test_mission` thresholds move.
- **Duplicate and false reports must be reconciled.** The blackboard clusters detections by
  world position and requires corroboration (repeat observations, or two robots) before
  promoting a candidate to a confirmed victim. This is new blackboard work at D4.
- **Fog comes from what was actually seen**, not from a geometric disc: cells covered by a
  camera frame are marked explored.
- **Determinism holds.** The classical detector is pure numpy; the CNN is deterministic given
  fixed weights and pinned thread counts. Seed 42 must still hash identically.

### 3.9.7 Honest description

"Each robot runs a detector over its own occluded camera frame; nothing reads victim positions
from the simulator." That is true once this ships. It is **not** "real robots" and not a 3D
renderer — say *2.5D top-down egocentric imagery*, and say which detector actually shipped.

---

## 4. Contract — **FROZEN**

Topic names are exactly the original spec's. Every payload carries `"schema": "1.0"`. Defined once in `swarmmind/contracts/schemas.py` as pydantic models; **no other module may define these shapes**, and the Godot client parses these and only these.

### 4.1 `/swarm/state` — published by `blackboard_node` @ 10 Hz

```json
{
  "schema": "1.0", "tick": 1234, "sim_time": 61.7,
  "robots": [{
    "id": "r07", "type": "scout", "actuator": "none",
    "pos": [12.4, 8.1], "heading": 1.02, "battery": 0.82,
    "status": "active",              // active | out_of_comms | failed | destroyed
    "current_task": "explore_A3",
    "carrying": null,
    "last_action_reason": "Bid won: closest capable scout for A3, 3.1 m"
  }],
  "sectors": [{
    "id": "A3", "explored_pct": 0.6,
    "hazard_level": 0.0,             // KNOWN hazard only, not ground truth
    "victims_found": 1, "priority": "high", "abandoned": false
  }],
  "victims_found_total": 3, "victims_rescued_total": 1, "victims_total": 8,
  "robots_active": 23, "robots_lost": 1,
  "comms_component_size": 22
}
```

`type` is a **derived display label** from `actuator` (`none→scout`, `scoop→digger`, `gripper→carrier`, `antenna→relay`) kept for dashboard readability and continuity with the original spec. `actuator` is the source of truth for capability checks.

`last_action_reason` is a **template string produced by `skill_executor`**, never an LLM call. It feeds the follow-cam panel and the Tier-2 ticker.

### 4.2 `/world/ground_truth` — published by `hazard_node` @ 2 Hz, **dashboard only**

```json
{
  "schema": "1.0",
  "victims_actual": [{"id":"v1","pos":[20.1,15.4],"state":"hidden","buried":true}],
  "hazard_zone_actual": {"centre":[30.0,10.0],"radius":8.2,"active":true},
  "explored_mask_rle": "…"
}
```

**No swarm node and no hivemind provider may subscribe to this topic.** Enforced by `tests/test_no_ground_truth_leak.py`, which greps the `swarmmind/nodes/`, `swarmmind/control/`, and `swarmmind/hivemind/` trees for the topic constant and fails on any hit outside `bridge.py`. Without that test, someone eventually "just peeks" and the fog-of-war claim becomes false.

### 4.3 `/hivemind/directives` — published by `hivemind_node` every 6 s

```json
{
  "schema": "1.0", "issued_at": 66.0, "source": "tuned-local",
  "latency_ms": 1840,
  "reasoning": "A3 has two unexplored pockets and no known hazard — pushing scouts there. B1's hazard is spreading toward the digger; pulling it out before it's cut off.",
  "directives": [
    {"sector": "A3", "priority": "high", "action": "explore"},
    {"sector": "B1", "priority": "abandon", "action": "abandon", "reason": "hazard_spreading"}
  ],
  "rejected": [{"directive": {...}, "rule": "cannot_abandon_sector_with_known_victim"}]
}
```

`source ∈ {tuned-local, base-local, api, scripted}` — rendered as a badge on the dashboard. `rejected` is published deliberately: showing the feasibility filter catching a bad directive is a *good* demo moment, not something to hide.

### 4.4 `/swarm/tasks_available`, `/swarm/bids`, `/robot_N/assigned_task`

```json
// task
{"schema":"1.0","task_id":"clear_B1_debris_v3","kind":"clear_debris",
 "sector":"B1","target":[41.2,29.8],"requires":"scoop",
 "priority_rank":1,"created_at":123.0,"value":3.0}

// bid  (lower wins)
{"schema":"1.0","robot_id":"r12","task_id":"clear_B1_debris_v3",
 "bid_score":4.2,"capable":true}

// award
{"schema":"1.0","robot_id":"r12","task_id":"clear_B1_debris_v3",
 "awarded_at":124.3,"reason":"lowest bid among 4 capable scoop robots"}
```

`kind ∈ {explore, clear_debris, extract, relay, retreat}`. `requires ∈ {any, scoop, gripper, antenna}`.

### 4.5 `/swarm/events`

```json
{"schema":"1.0","t":204.5,"kind":"robot_destroyed","robot":"r09",
 "sector":"B2","pos":[38.0,26.5],"text":"r09 destroyed by hazard in B2"}
```

`kind ∈ {victim_found, victim_cleared, victim_rescued, task_created, task_awarded, task_orphaned, task_completed, robot_destroyed, robot_out_of_comms, robot_reconnected, hazard_ignited, sector_abandoned, directive_issued, directive_rejected, hivemind_offline, mission_complete}`.

`task_orphaned` immediately followed by `task_awarded` **is** the self-healing moment. The dashboard highlights that pair.

---

## 5. Control tiers

### 5.1 Tier 1 — reflex, 20 Hz

Fully vectorised, no per-robot Python in the loop, no allocation in the hot loop:

1. **Global guidance from a shared flow field, not per-robot A\*.** Robots descend a coarse distance field computed once per *goal* (`control/planner.NavFields`). A* per robot does not scale: 512 robots replanning every 2 s is ~256 searches/second over 66k cells. Fields are the same ones the auction reads for bidding (§5.2), and since `passable` never changes mid-mission they cache for the whole run — a repeated goal is free. Cost is per goal, not per robot, which is the entire reason 512 robots are affordable.
2. Local obstacle repulsion: two rings of 8 body-relative probes against the fine grid.
3. Inter-robot separation: inverse-square, `O(N²)` but preallocated (~1 ms at N=512).
4. **Hazard repulsion** from `world.hazard_known` — hazard the swarm has *observed*, never the ground-truth disc. Weighted above obstacles: a wall costs a robot time, fire costs the robot.
5. Combine, convert to `(v, ω)`, clamp, and gate forward speed by `cos(heading error)` so a robot that must turn hard slows instead of carving an arc into an obstacle.
6. **Hard override:** if the swept circle for this tick would intersect a wall, `v` is zeroed and only rotation is allowed.

Step 6 is the safety floor from the original spec. Nothing above Tier 1 can reach past it. `tests/test_tier1_safety.py` drives the controller at goals placed *inside walls* and asserts zero penetrations.

**Repulsion and separation are scaled, never normalised to unit vectors.** Normalising discards magnitude, so one probe grazing a wall pushes exactly as hard as being boxed in on seven sides — and at `w_obstacle > w_goal` that means a robot in a rubble field spends its mission fleeing walls instead of reaching goals.

A\* survives in `planner.astar` for single-robot use — tests, debugging, one-off unique goals. It is never called per-robot per-tick.

**Two radii must stay consistent, and both bugs cost real time at D2.** Tier 1 parks a robot within a per-task `stop_radius` of a goal that has been *snapped to the cell grid*; if the world's interaction reach (`REACH_GRAB`, `REACH_DIG`) is smaller than `stop_radius + cell`, carriers park just outside pickup range and nothing is ever rescued. Likewise, task completion must tolerate the snap, or a robot parks in the gap between the two radii and holds the task forever. `tests/test_skills.py::test_interaction_reach_exceeds_stop_radius` asserts the ordering for every scenario.

~~The A\* cost multipliers for `abandoned` sectors and known hazard (`+5.0 ×` / `+2.0 ×`) apply to the flow-field build at D3.~~ **Revised at D7: flow fields do not depend on directives.**

The planned third channel was a routing penalty through abandoned sectors, baked into the flow-field build. It cannot be had at this price. Fields are cached for the whole mission precisely because `passable` never changes; making them a function of `sector_abandoned` means rebuilding on every directive, and a field costs 22.4 ms ([MEASUREMENTS.md M-4](MEASUREMENTS.md)). That is the exact cost the scaling rules exist to avoid.

The alternative — penalising abandoned sectors in Tier 1's own short-range A\* — was rejected for a better reason than cost: **it would make Tier 1 read Tier 3.** The safety floor answering to the strategic layer is precisely the coupling the architecture claims not to have, and the claim is worth more than the routing detour.

So a directive changes behavior through **two** channels, both inside Tier 2, both in `nodes/tasks.py`:

1. **Task supply.** An abandoned sector generates no `explore`, `investigate` or `relay` work; sector priority shifts a task's rank by ∓0.5, which is the auction's first sort key.
2. **Evacuation.** `abandoned_preemptions()` pulls robots standing in a closed sector onto `retreat`, at heartbeat rate, by the same mechanism the hazard uses. Without it a closed sector stops producing work but keeps the robots already grinding away in it.

Robots still route *through* a closed sector if the geometry calls for it. That is correct: the sector is closed to *work*, not to transit, and the hazard override in Tier 1 remains the thing that keeps them out of fire.

**Exact routing to collection points (`Mission(zone_routing=True)`, off by default).** A coarse cell has one descent direction, and it ignores fine-grid rock beside the robot. M-60 removed phantom edges *between* coarse cells; this is the same mismatch *inside* one, and it bit hardest on loaded carriers because they all converge on twelve points. Measured: 39 casualties still being carried at the buzzer on demo seeds 42–45, 34 of them with time to deliver; one carrier sat 12 m from its zone for ~320 s with the override zeroing a 2.1 m/s command. Collection points never move, so `control/zone_routing.py` builds **one multi-source fine field per chassis** from every zone's delivery disc, once per mission (~0.5 s), with no diagonal corner-cutting. It routes over passability **eroded by one cell**: the exact field threaded one-cell gaps a force-field-steered robot cannot hold a heading through and lost 13 rescues on seed 44. `NavSet.descend_to` uses the fine step when the goal is a zone and the eroded field has an answer, the coarse step otherwise. Invariant #2 is untouched. **With Tier 3 silent the fix is unambiguous:** +7.5 rescued on the M1 (M-76), +15.0 on Kaggle (M-76d), +8.1 over eight seeds (M-76a) -- up on 8 of 8 seed-arms across two machines. **With Tier 3 live it is unresolved and therefore off by default:** +9.5 on Kaggle (M-76e) against −1.25 on the demo machine (M-76f), same code and seeds. The layers overlap -- Tier 3 already steers the swarm off some of the ground where carriers trapped -- and what remains is smaller than the platform-float noise this scenario carries (~10 rescues per map between machines). Turn it on for a swarm with no strategic layer.

### 5.2 Tier 2 — auction, 1 Hz

**Task generation** (`auction_node`, each cycle):
- `explore(frontier)` — targets are clustered **frontier** cells: *unexplored* passable cells adjacent to explored ones. The direction matters: defining the frontier on the explored side makes every explore task complete the instant it is issued, and the swarm never moves. `requires: any`, but never awarded to `antenna` robots — a relay that takes search work walks out past the chain it is holding up and drops the swarm behind it out of contact.
- `clear_debris(victim)` — per `found ∧ buried ∧ debris_remaining > 0` victim. `requires: scoop`.
- `extract(victim)` — per `cleared ∧ not carried` victim. `requires: gripper`.
- `relay(pos)` — generated at frontier positions that would extend the comms component. `requires: antenna`.
- `retreat(safe_pos)` — for any robot inside a sector that just became `abandoned`. `requires: any`, `priority_rank: 0`.

**Bid** (each capable, in-comms, unassigned robot bids on each open task):

```
bid = dist_field[task][robot_cell] / v_max
    + 20.0 * (1 - battery)
    + 15.0 * queue_len
    + 40.0 * hazard_exposure_along_path
    + evolved_behavior_terms          # tier2_policy.BehaviorParams
```

Units are seconds; the coefficients convert the other terms into seconds-equivalent. Tunable in `demo.yaml`.

**`dist_field` is why bidding is independent of `N`.** Once per auction cycle each open task target gets one wavefront BFS, cached in `auction_node`; every robot's travel term is then a single array lookup at its cell. Bidding is `O(tasks × cells + robots × tasks)` — the expensive term does not involve `N`. Calling A\* per (robot, task) pair instead would be ~10,000 searches/second at 512 robots and is the single easiest way to make this project not scale. A\* survives only in Tier 1, for the robot's own path, recomputed on goal change or every 2 s.

**But the field itself must be made cheap, and this is measured, not assumed.** On the demo grid one field costs **22.4 ms** (MEASUREMENTS.md M-4). At 48 sectors and 80 victims the auction can legitimately see 200+ open tasks — 4.5 seconds of compute per second of sim. One field per task per cycle **does not scale and must not be built that way**. Three mitigations, all required:

1. **Coarse navigation grid.** Downsample `passable` 4× (80 × 52 = 4,160 cells) for bidding only — fewer cells *and* a shorter wavefront diameter, ~16× cheaper at ~1.4 ms. Bidding ranks candidates; it does not need 1 m precision.
2. **Cap announced tasks** to the top ~24 by `priority_rank` per cycle. A real auctioneer does not announce everything at once either.
3. **Amortise** field computation across the 20 ticks of the 1 Hz cycle — fields are read only at award time.

Together: ~24 × 1.4 ms ≈ 34 ms per cycle, ~3% of budget. `hazard_exposure_along_path` reads a second, hazard-weighted field built in the same pass.

**Resolution** — a single-round reverse auction with greedy sequential assignment:
```
sort open tasks by (priority_rank asc, value desc)
for task in sorted_tasks:
    bids = [b for b in bids_on(task) if b.capable and robot_free(b.robot_id)]
    if bids: award(task, min(bids, key=bid_score)); mark robot busy
```

This is **not** CBBA — no bundles, no consensus rounds. Say "market-based reverse auction."

**Task release is progress-based, not deadline-based.** A wall-clock cap short enough to free a genuinely stuck robot also fires mid-carry on a healthy one — a loaded carrier crossing the map legitimately needs minutes. The signal that separates them is *geodesic* distance to goal decreasing (read off the flow field; euclidean calls a healthy detour around an obstacle a stall). A robot standing on its goal is never stalled: a relay holding station is the whole point of the relay lane.

**Self-healing** — the entire mechanism, and it contains no LLM:
- Every robot heartbeats at 2 Hz on `/swarm/heartbeat`.
- `auction_node` marks a robot `failed` after 2.0 s of silence and returns all its tasks to the open pool with `priority_rank` bumped up one.
- The next cycle (≤1 s later) re-auctions them among remaining capable robots.
- Worst case from destruction to reassignment: 2.0 s timeout + 1.0 s cycle = **3.0 s**, asserted in `tests/test_auction.py`.

**Task commitment / hysteresis:** a robot keeps its current task unless a competing assignment improves total cost by more than `switch_cost` (an evolved parameter). Without this, robots thrash between tasks every cycle and the demo looks broken.

**Reserved robots (unit policy, §7a).** A robot the unit policy has staged holds `SkillExecutor.policy_goal` and has no assignment. The auction withholds it from every task *except* `extract`, `clear_debris` and `retreat`: a carrier parked beside a dig must not be sent off exploring, and must still win the extract when the casualty comes out. Awarding it real work clears the hold. With no unit policy running, `policy_goal` is empty, the reserved mask is never built, and the auction is byte-identical.

### 5.3 Tier 3 — hivemind, every 6 s

Pipeline: `blackboard state → prompt.build() → provider.generate() → filter.validate() → publish → auction applies`.

**How a directive actually changes behavior** (the whole point — a directive that does nothing is theatre):
1. `priority: high|normal|low` → `priority_rank 0|1|2` on every task in that sector, which is the first sort key in auction resolution.
2. `action: abandon` → cancel open tasks in that sector, set `sector.abandoned = true`, add the `×5.0` A* cost penalty, and emit `retreat` tasks for every robot inside it.
3. `action: explore | rescue | hold` → biases which task kinds are generated for that sector next cycle.
4. **Directives expire after 30 s** unless renewed. A stale `abandon` that permanently locks a third of the map is the most likely way this system quietly fails.

**Prompt** (~700–900 tokens, cached prefix + volatile suffix):

```
system: You are the strategic coordinator for a {N}-robot search-and-rescue swarm.
        Output ONLY JSON matching the schema. Reasoning: at most two sentences.
        You issue SECTOR-LEVEL priorities. You never command individual robots.

user:   MISSION t=126s / 420s   RESCUED 14/80  FOUND 31/80  ROBOTS 498/512
        SECTORS (id, explored%, known_hazard, victims_found, current_priority)
          A1 100%  0.00  1  normal
          A2  84%  0.00  0  high
          ...  (48 rows, one terse line each, ~12 tokens per row)
        ROBOTS BY LANE (lane, active, idle, avg_battery)   # aggregated: prompt size is independent of N
          scout   6  1  0.71
          digger  5  3  0.83
          ...
        VICTIMS KNOWN (id, sector, state)
          v1 A1 rescued / v3 B2 found,buried / ...
        RECENT EVENTS (last 6 lines)
        PREVIOUS DIRECTIVES (last cycle)
```

Stable content first (system prompt, schema, sector *names*), volatile last — so the prefix caches on the API path and the KV cache reuses on the local path.

**Feasibility filter** (`hivemind/filter.py`) — every rule rejects a *directive*, not the whole message:

| # | Rule |
|---|---|
| F1 | Payload parses and validates against the pydantic schema |
| F2 | `sector` exists in the scenario |
| F3 | `action` and `priority` in enum |
| F4 | ≤ 4 directives per message (excess dropped, lowest first) |
| F5 | ≥ 3 sectors remain non-abandoned after applying |
| F6 | Cannot request a capability no `active` robot currently has |
| F7 | Cannot `abandon` a sector the **swarm believes** holds an un-rescued casualty unless that sector's **known** `hazard_level ≥ 0.6`. Revised at D7: judged from `tracker.believed_positions()`, not the simulator's victim list — the filter must reason from the same incomplete picture as everything else, or it vetoes using information no other component has |
| F8 | Cannot abandon the sector containing an extraction zone |
| F9 | Cannot combine `action: abandon` with `priority: high`/`low`. Added at D7: the sector would close to new work *and* outrank every other sector for it. A model asking for both has not decided, and picking a half for it is a filter inventing strategy |

Rejections are published on `/hivemind/directives.rejected` and logged as `directive_rejected` events. Rejection rate is a gate metric (§8) — a model that gets filtered constantly has not learned the task.

**The baseline must not need the filter.** At D7 the scripted provider was tripping F8 twenty-one times a run by proposing to abandon the sector containing the base ([MEASUREMENTS.md M-25](MEASUREMENTS.md)). Teaching the *provider* F8 and F7 dropped its rejection rate to zero without weakening a single rule — every one still fires in `tests/test_hivemind.py` against a payload written to break it. This matters beyond tidiness: rejection rate is a gate metric, and a baseline that rejects itself sets the bar in the wrong place. Rung 4's thresholds live in `assets/scenarios/fallback_directives.yaml`, and a startup check refuses a config that contradicts `ABANDON_HAZARD_FLOOR`.

**Provider ladder** (`hivemind/fallback.py`) — try in order, **5.0 s** hard timeout each (raised from 4.0 at D7: measured max latency is 3.95 s, and at a 6 s cadence a 5 s timeout still leaves the instant scripted rung room to answer in the same cycle), fall through on timeout / parse failure / total filter rejection:

| Order | Provider | Notes |
|---|---|---|
| 1 | `local_gguf` (tuned) → `source: "tuned-local"` | `llama.cpp` server, Metal, JSON-schema-constrained decode |
| 2 | `local_gguf` (base) → `source: "base-local"` | same server, base Qwen2.5-1.5B + few-shot |
| 3 | `anthropic` | only if `--hivemind-allow-api` **and** network reachable |
| 4 | `scripted` | phase-keyed directives from `assets/scenarios/fallback_directives.yaml` |

The active tier is stamped into `source` and shown on the dashboard. If a cycle produces nothing in time, the previous directive stays in force (it has 30 s of life) and a `hivemind_offline` event fires.

**The node is asynchronous, and this is load-bearing.** One tick starts a request; a later tick collects it. Inference spans as many ticks as it needs and `step()` never blocks.

The obvious design — a worker thread joined with the rung timeout — is wrong, and it passed every test for a day because every test used a provider that answers instantly. `DemoSim` runs at wall-clock speed: a 3.5 s model call every 6 s would have frozen the dashboard for more than half the demo. Measured after the fix, per-tick cost with Tier 3 running is indistinguishable from Tier 3 disabled ([MEASUREMENTS.md M-28](MEASUREMENTS.md)).

A consequence worth knowing: **headless Tier 3 cadence is wall-clock bound, not sim bound.** A headless mission runs ~20x real time, so the model answers ~6 times per mission where the demo gets ~70. `--headless` therefore forces the scripted rung outright — it is what the flag's help has always claimed ("no dashboard, no LLM, no network"), and without it the seed-42 hash in `make check` would depend on whether a `llama serve` happened to be listening.

**Local serving:** `scripts/serve_hivemind.sh`, which runs `llama serve -m assets/models/hivemind-base.gguf --port 8080 --ctx-size 4096 --no-webui --jinja` (llama.cpp now ships a unified `llama` binary with subcommands; `llama-server` is gone, the HTTP surface is not). Called via its OpenAI-compatible `/v1/chat/completions` with `response_format: {"type":"json_schema", ...}` so the JSON is grammar-constrained at decode time. `max_tokens: 200`.

**The schema is the latency budget, not a formality.** Generation dominates the cycle. Unconstrained, the 1.5B model wrote a 700-character rationale plus four per-directive justifications and took **10.6 s** against the rung timeout — every cycle would have fallen through to the scripted baseline while appearing to work. `maxLength` in the schema becomes a GBNF constraint enforced at decode time; capping `reasoning` at 120 chars and deleting the per-directive `reason` field brought it to **mean 3.53 s, max 3.95 s** ([MEASUREMENTS.md M-27](MEASUREMENTS.md)). Any future change that lets the model write more prose is a latency change first and a quality change second.

**API provider** (`hivemind/providers/anthropic_api.py` — the `_api` suffix is load-bearing: a module named `anthropic.py` inside the package shadows the `anthropic` distribution on import), used for A/B and as rung 3:
- `client.messages.create(model="claude-opus-5", output_config={"effort":"low","format":{...}}, thinking={"type":"adaptive"}, max_tokens=1024)`
- Structured outputs via `output_config.format`, **not** assistant prefill (prefill returns 400 on Opus 5).
- Cost is negligible: ~1k in / ~200 out × ~60 calls ≈ 60k in + 12k out ≈ **$0.60 per 7-minute run**. Cost is not the reason the local model is primary — wifi is.

---

## 6. Guided evolution — MAP-Elites

`pyribs`, run locally on the 8 M1 cores. Not Kaggle: this is the one CPU-bound, embarrassingly parallel part of the project, eight native cores beat a shared Kaggle CPU session, and there is no GPU work here to justify the trip.

Two traps, both hit and both now guarded in `mapelites/pool.py`:

- **numpy thread oversubscription.** Seven worker processes each opening a BLAS thread per core is 56 threads on 8 cores. Workers pin to one thread each; the simulator is almost entirely elementwise numpy and loses nothing.
- **`spawn` re-executes `__main__`.** Run from a REPL or a heredoc there is no importable `__main__`, so every worker re-runs the caller and spawns its own workers. That is a fork bomb that presents as a hang. `pool._guard()` refuses with an explanation before any process starts.

### 6.1 Genome — 15 real values in `[0,1]`, decoded

**Morphology (5):** `actuator_g` (→ `floor(g*4)` ∈ lanes), `chassis_scale`, `motor_power`, `sensor_scale`, `battery_scale`.

**Behavior (10)** — these *are* the Tier-2 policy. Interpretable parameters beat a raw MLP here: they evolve faster, they cannot produce uninspectable garbage, and they read on a dashboard.

| Gene | Effect |
|---|---|
| `frontier_gain` | weight on unexplored-area gain when bidding `explore` |
| `distance_penalty` | multiplier on the travel-time term |
| `hazard_aversion` | multiplier on `hazard_exposure_along_path` |
| `comms_tether` | penalty for bids that would leave the comms component |
| `revisit_penalty` | discourages re-covering explored cells |
| `task_commitment` | the switch-cost hysteresis in §5.2 |
| `formation_spacing` | inter-robot repulsion radius in Tier 1 |
| `wander_bias` | exploration randomness when no task is held |
| `battery_reserve` | battery level below which it only bids near base |
| `switch_cost` | additional churn damping |

**Why not an MLP:** see [PLAN.md §0.1 D6](PLAN.md). If time allows, `tier2_policy.py` has a second implementation slot for a 16→16→4 MLP; the interface is the same.

### 6.2 Archive

- `pyribs.archives.GridArchive`, dims `(4, 10)`.
- Measure 1: `floor(actuator_g * 4)` — the lane (discrete, 4 bins).
- Measure 2: **measured** mean speed over the evaluation episode, binned into 10 — *measured, not intended*, per the original spec. Range **0.20–1.45 m/s**, calibrated at D9 to what robots actually achieve (0.24–1.32 across 52 elites). The first guess of 0.0–1.8 left the bottom bin and the top two permanently unreachable, capping coverage at 28/40 however good the search was — the number was measuring the axis, not the search. `tests/test_mapelites.py` now guards the axis in **both** directions: wide enough not to clip real robots, tight enough that dead cells cannot masquerade as poor coverage.
- Emitters: **one `EvolutionStrategyEmitter` (CMA-ES) per lane**, batch 15, σ₀ 0.15, bounded to `[0,1]`, each starting at *its own lane's* midpoint. Both numbers were wrong on the first pass and both cost the run outright ([MEASUREMENTS.md M-32](MEASUREMENTS.md)):
  - Five emitters all starting at the all-0.5 midpoint means `actuator_g` starts at 0.5, which decodes to `gripper`. Measured on a fresh `ask()`: **87% of proposals were gripper or scoop, 13% reached scout and relay.** Half the archive was barely being searched.
  - Batch 30 is 2.5× the CMA-ES default population for 15 dimensions (4 + 3·ln 15 ≈ 12). Against a ~1,400-evaluation budget that buys **9 generations**; CMA-ES needs tens. Batch 15 buys ~23/hour for the same spend.
- **The archive is seeded with 150 uniform random genomes before CMA-ES starts.** All emitters otherwise begin in one neighbourhood and coverage stalls — cold, the first run filled 15 cells and reached 16 after three iterations; random seeding alone reaches 18. Random genomes are individually poor but land across the measure space, giving the emitters elites to restart from. **Installed without `[visualize]`** — matplotlib is ~200 MB resident on an 8 GB machine, and archive heatmaps go through `viz/png.py`, which is already stdlib-only. CMA-ES rather than Gaussian mutation is a direct consequence of the corrected budget: at ~1.4k evaluations, sample efficiency decides whether the archive gets covered or merely sampled.
- ~~Budget: 2000 iterations ≈ 300k evaluations. At ~3.3 ms/eval/core × 8 cores ≈ 20 minutes locally.~~ **Corrected at D9 by measurement ([MEASUREMENTS.md M-30](MEASUREMENTS.md)): 1,418 evaluations/hour on 7 workers, so 300k evaluations is ~210 hours, not 20 minutes — a ~200x planning error.** An evaluation is a full mission tick (perception, auction, flow-field descent per robot), not a bare policy rollout. Revised: **~1,400 evaluations for a one-hour run, ~14,000 overnight**, against a 40-cell archive — 35 and 350 per cell respectively. Sample efficiency now matters far more than it would have at 300k, which is what makes the emitter choice a real decision rather than a detail.

### 6.3 Fitness — lane-specific, on a 60 s `FastSim` episode

| Lane | Fitness |
|---|---|
| `none` (scout) | `new_cells_explored / energy_used`, ×0.5 penalty if it spends >30% of the episode out of comms |
| `scoop` (digger) | `debris_cleared + 2 × victims_cleared`, minus time-to-first-clear |
| `gripper` (carrier) | `victims_delivered + 0.3 × (1 − mean_delivery_time/60)` |
| `antenna` (relay) | `connected_robot_seconds / episode_length` — integral of how many robots were in comms because of it |

Averaged over **3 training seeds**. The gate re-evaluates on **10 held-out seeds** never used during evolution.

### 6.4 Roster selection → the demo

`training/mapelites/select.py` reads the archive and picks the demo roster: for each lane, the elites with the highest gate score, plus deliberate diversity across the speed axis (e.g. two fast scouts and one slow-tough scout) so the archive's diversity is *visible* on screen rather than just claimed. Output: `assets/scenarios/demo_roster.yaml`, consumed by `scenario.py`.

If the `GazeboSim` stretch happens, the same genome decodes into a URDF: `actuator` → joint/plugin attachment, `chassis_scale` → link geometry and inertia, `motor_power` → joint effort limits.

---

## 7. Hivemind training — the RAFT → DPO → GRPO ladder

The shared, expensive part is the **rollout scorer**. Build it once, correctly; the three rungs are then thin.

### 7.1 Rollout scorer (`training/llm/rollout.py`)

```python
def score(state: BlackboardState, directive: Directive, seed: int) -> float:
    world = FastSim.restore(state.world_snapshot, seed=seed)
    apply_directive(world, directive)
    before = world.scorecard()
    world.run_for(60.0)                # sim-seconds, ~0.2 s wall
    after = world.scorecard()
    return reward(before, after, directive, state)
```

**Every candidate in a group uses the same `seed`.** Different seeds put state-difficulty variance directly into the group-relative advantage and destroy the signal. This is the single most important line in the training code.

### 7.2 Reward

```
R =  1.0 * json_valid
  +  1.0 * (1 - filter_rejection_fraction)
  +  3.0 * Δvictims_rescued
  +  1.0 * Δvictims_found
  +  0.5 * Δground_explored_frac
  -  2.0 * robots_lost_to_hazard
  -  0.2 * energy_used_norm
  -  0.5 * directive_churn        # sector priorities flipped vs. previous cycle
```

**Curriculum:** steps 0–200 use only the first two terms. Mission terms switch on at step 200. Skipping this is how GRPO runs burn six hours learning bracket placement.

**Reward-hacking watch (R6):** read 20 sampled `(prompt, directive, reasoning, reward)` tuples per checkpoint by hand. The tell is the reasoning text drifting away from the directives it accompanies — the model has found a directive pattern that scores well and the prose has become decoration.

### 7.3 Dataset (`training/llm/dataset.py`)

~3000 prompts: run 200 self-play missions with the heuristic Tier 2 and mixed directive sources, snapshot the blackboard at 15 points per mission, stratify by `(hazard_active, victims_found_bucket, robots_lost > 0)` so the interesting states are not swamped by the easy early-game. 2700 train / 300 validation. The gate's 10 held-out **scenario seeds** are disjoint from anything used here.

Each row stores the world snapshot (so `rollout.score` can restore it), the rendered prompt, and the stratum.

### 7.4 The three rungs

| | Method | TRL / Unsloth | Session | Ships as |
|---|---|---|---|---|
| **R1** | RAFT | sample G=8, keep argmax, `SFTTrainer` | ~2–3 h T4 | gate-approved checkpoint |
| **R2** | DPO | (argmax, argmin) pairs, `DPOTrainer`, β=0.1 | ~2–3 h T4 | gate-approved upgrade |
| **R3** | GRPO | `GRPOTrainer`, G=8, `loss_type="dr_grpo"`, vLLM generation via Unsloth `fast_inference=True` | 6–9 h T4 | gate-approved upgrade |

Common: `Qwen2.5-1.5B-Instruct`, LoRA `r=16, α=32`, 4-bit base, `lr 5e-6` (GRPO) / `1e-5` (SFT/DPO), `max_prompt_len 1024`, `max_completion_len 256`.

**If GRPO destabilizes:** `loss_type="dr_grpo"` (already default here — removes length/std normalization bias), DAPO-style clip-higher `ε_high=0.28`, and **dynamic sampling** — drop groups where all G rewards are equal. That last one matters a lot in this domain: many swarm states are insensitive to the directive, so zero-variance groups are common and they contribute pure noise. `RLOO` is the simpler sibling if GRPO stays unstable.

### 7.5 Kaggle mechanics (R4)

- Checkpoint every 100 steps **and** at 8 h wall clock, to `/kaggle/working/ckpt/`.
- `/kaggle/working` is **not** ambiently persistent. It survives as a *saved notebook version's output*. To resume: Save Version, then attach that version's output (or a private Dataset) as an input to the next run and `resume_from_checkpoint`.
- Implement the resume path before the first session.
- GPU quota ~30 h/week; CPU sessions are a separate ~30 h/week and are where MAP-Elites goes.

### 7.6 Export to GGUF

```
merge LoRA into base (fp16)
  → llama.cpp convert_hf_to_gguf.py  → f16 GGUF
  → llama-quantize → Q4_K_M  (~1.0 GB)
  → assets/models/hivemind-<rung>-<date>.gguf   (gitignored, sha256 in models.lock)
```

`scripts/export_gguf.sh` does this end to end. Every exported model is gate-evaluated locally on the M1 before it can be selected — including its **p95 latency**, which is a gate criterion and can only be measured on the real machine.

---

## 7a. Unit policy — behaviour cloning → PPO (branch `rl/unit-policy`)

A per-robot decision layer inside Tier 2 for robots with nothing urgent. Record of the programme: [training_notes/run4-unit-policy.md](../training_notes/run4-unit-policy.md). Scope and adoption criteria: [PLAN.md §7.8](PLAN.md).

**What it decides.** Once per auction cycle, after the auction has allocated, every in-contact carrier, digger and scout that is idle or holding `explore` (not carrying, not a relay, at most every 5 s per robot) picks one of five candidates:

| slot | candidate | notes |
|---|---|---|
| 0 | default | what the executor would have done; always offered; a policy that always picks it is `main` byte for byte (`tests/test_unit_policy.py`) |
| 1, 2 | stage at the nearest / second-nearest rescue site | carriers and diggers only; sites are resolved buried casualties awaiting a dig, and confirmed contacts. Held up to 60 s, released when the site resolves |
| 3 | nearest dark ground | per chassis, from `_dark_points`, reachable on the robot's own component. 30 s |
| 4 | hold position | 15 s |

**Why this action** is measured, not assumed (M-76): an oracle for contact inspection moved rescues +0.5, so inspection was dropped; waiting for a carrier is 33% of rescue-chain time.

**Observation — swarm knowledge only (invariants #3, #4).** Per candidate (25 features): lane, rotor, battery, mission fraction, speed rating, whether holding `explore`; candidate type, distance, bearing relative to heading, local explored fraction and robot crowding at the target (integral images, one pass per cycle), known hazard; for sites: kind, age, diggers and carriers already near, robots already staged there, report confidence. Value context (16): the robot features plus explored/found/rescued fractions, fraction alive, open dig sites and confirmed contacts, distance to base, fraction of the swarm on holds. Sites come from `tracker.resolved_victims()` and `tracker.reports` exactly as `nodes/tasks.py` reads them.

**Network.** One candidate-scoring MLP shared by every robot (25→64→64→1, tanh), softmax over valid candidates; a separate value MLP (16→64→64→1). Numpy with hand-written backprop, every gradient checked against finite differences (`tests/test_rl_core.py`). No torch: CLAUDE.md forbids it on the laptop, the network is tiny, and rollouts dominate the cost. Deployed deterministically (argmax); ties go to slot 0.

**Rewards — per robot, read from the simulator in `training/rl/rollout.py`.** Delivery +1.0 to the carrier; pickup +0.2; dug out +0.3 shared by diggers in reach; found +0.2 shared by the robots whose sightings built the report; destroyed −0.5. The gate never sees these numbers.

**Algorithm.** Behaviour cloning of the heuristic staging rule until the *sampled* policy agrees ≥ 98% (argmax agreement is not enough — PPO samples), then PPO: clip 0.2, lr 3e-4, 4 epochs, minibatch 4096, entropy 0.01, grad-norm 0.5, KL early stop 0.045. **Semi-Markov GAE:** decisions are unevenly spaced, so γ = 0.995 applies per second of elapsed time and λ = 0.95 per decision; each reward is credited to the decision in force when it was earned.

**Data.** The four demo maps, 42–45 (`demo_seeds`), full 420 s missions with zone routing on. One iteration = one episode per map in parallel; the sampling stream is keyed on (map, iteration) so replaying a map does not replay its exploration. Every 5 iterations the deterministic policy plays the same four maps against the routing-only and heuristic arms; the best by `mission_score` is kept. Tripwires (warn, never kill): entropy < 0.05, policy never leaves the default, three checks in a row under the bar. State is pickled atomically every iteration; resuming refuses a checkpoint trained on other seeds.

**Honest description.** A win is a win *on the four demo maps*. It is not evidence the policy works on a map it has not seen.

**Outcome: kept, unused.** Gated at **1.04x** against routing alone with Tier 3 off (`SHIPPING.md`), and **0.997x** with Tier 3 live -- the configuration the demo runs (M-76e). The code, the trainer and the Kaggle notebook stay so the claim is reproducible and so the next person does not rebuild them; `Mission(unit_policy=None)` is the default and the demo never constructs one.

---

## 8. The gate (`training/gate.py`)

One command, one report, one decision:

```bash
uv run python -m swarmmind.training.gate --seeds 10 --report SHIPPING.md
```

| Component | Criterion |
|---|---|
| Evolved lane × 4 | mean lane fitness ≥ 1.10 × heuristic baseline **and** success rate ≥ 0.60, on 10 held-out seeds |
| Hivemind checkpoint | mean mission reward ≥ 1.05 × prompted-base baseline **and** filter rejection ≤ 10% **and** p95 local latency ≤ 4.0 s |
| System — behaviour | `test_mission.py` with the hivemind silent: every lane performs its lane-unique skill, failure observed, orphan reassigned ≤ 3 s |
| System — mission quality | `scripts/baseline_sweep.py` on demo seeds 42–45, Tier 3 off: **delivery (rescued ÷ found) ≥ 55%** and **discovery ≥ 55 of 110 on seed 42**. Revised from a flat ≥ 75% rescued, which was set against a 320 × 208 m / 8-casualty scenario and is unreachable on this one — reasoning and measurement in [PLAN.md §6.1](PLAN.md) |

`gate.py` writes `SHIPPING.md` stating, per component, whether the trained or classical version ships and by what margin. That file is the honest answer to "what did you actually train?"

**Held-out seeds (D9):** `HELD_OUT_SEEDS = (101 … 110)`, disjoint from `mapelites.evaluate.TRAIN_SEEDS = (11, 12, 13)` and asserted so by `tests/test_evaluate.py`. Training may never touch them.

**The gate judges on the four demo maps by default** (`demo.yaml` `demo_seeds`, 42–45) — the only maps the demo plays. `gate.py --held-out` still runs 101–110. Two sections were added on branch `rl/unit-policy`: **zone routing** (evolved roster vs + routing) and **the unit policy** (routing-alone, heuristic staging, and the trained checkpoint from `runs/rl/policy_best.npz` if present, judged against the *better* of the first two). The bodies and detector were trained on other seeds, so the demo maps are unseen for them; the unit policy was trained on these maps, and `SHIPPING.md` says so in its section.

**The control arm is `genes=None`, not an all-0.5 genome.** The gene-driven bid carries four terms the classical bid does not have at all (`frontier_gain`, `revisit_penalty`, `comms_tether`, `battery_reserve`), so a midpoint genome is already a richer policy — on `test`/seed 42 it explores 47.7% against the classical 35.8%. Gating against it would credit evolution with a change that was made by hand, so `run_mission()` and `gate.classical_baseline()` both pass no genome.

**`mission_score` is one number, weighted the way the demo is judged:** `10 × rescued + 2 × found + 5 × explored_frac − 0.5 × robots_lost`. Rescues dominate because rescues are the mission and exploration is a proxy for it; losses are penalised because a swarm that clears the map by driving into fire has not solved the problem. `tests/test_gate.py` pins each of those orderings, and pins that a 1% win does **not** ship — beating the baseline by 1% on ten seeds is noise, and shipping on noise is how a demo acquires a component worse than the one it replaced.

---

## 9. Performance budget (M1, 8 GB)

| Process | RSS | Notes |
|---|---|---|
| macOS + Finder | ~2.5 GB | baseline |
| Godot 4 dashboard | ~350 MB | |
| Python (sim + nodes + bridge) | ~450 MB | numpy arrays; grows ~1 MB per 32 robots, dominated by the grid and distance-field cache |
| `llama-server` Q4_K_M 1.5B | ~1.4 GB | `-c 4096`, Metal |
| **Total** | **~4.7 GB** | ~3.3 GB headroom |

Measured on D7 with `scripts/stress.py`. If it's tight: Q4_0 quant (−200 MB), `-c 2048` (−150 MB), or 8 s hivemind cadence.

**Compute budget per 20 Hz tick (50 ms), 64 robots — estimates to be replaced by D2 measurements:**

| Pass | Rate | Est. cost |
|---|---|---|
| Integration + collision + battery | 20 Hz | ~0.8 ms |
| Tier 1 reflex (8 raycasts × N, vectorized) | 20 Hz | ~2.5 ms |
| Sensor / fog / LOS | 5 Hz | ~6 ms on the ticks it runs |
| Comms union-find (N² pairwise) | 1 Hz | ~1 ms |
| Auction: BFS distance fields × ~20 tasks | 1 Hz | ~40 ms on the tick it runs |
| Sector hazard/explored aggregation | 2 Hz | ~3 ms |

The 1 Hz auction tick is the spike. If it threatens RTF at high `N`, amortize the BFS fields across the 20 ticks of the cycle (fields are only read at award time) before reducing `N`.

---

## 10. Godot dashboard

### 10.1 Bridge protocol (WebSocket, one JSON object per frame)

Server → client:

| `t` | Rate | Payload |
|---|---|---|
| `hello` | once on connect | scenario meta, sector rects, occupancy grid as base64 RLE, robot roster |
| `state` | 10 Hz | §4.1 verbatim |
| `truth` | 2 Hz | §4.2 verbatim |
| `directive` | on issue | §4.3 verbatim |
| `event` | immediate | §4.5 verbatim |

Client → server: `{"t":"cmd","cmd":"reset"|"focus","arg":...}` only. Every toggle (fog/god view, sectors, victims, eagle-eye, follow-cam) is **client-side** — the client already holds both feeds. No round trip, no lag, nothing to go wrong on stage.

Send queue is drop-oldest for `state` (idempotent snapshots — a dropped frame is invisible) and never-drop for `event` and `directive` (they are the narrative).

### 10.2 Rendering

- `MapView`: static grid drawn once to a `TextureRect`. Fog as a shader sampling an `explored` mask texture updated per `state` frame — one texture upload per frame beats 24k node updates.
- `RobotMarker`: pooled scene instances, never freed. Lane → color, status → alpha/outline. `out_of_comms` = 50% alpha + dashed outline.
  **Above ~48 robots, switch to `MultiMeshInstance2D`** with per-instance color — one draw call for the whole swarm. Keep individual `Node2D` markers only for the follow-cam target and the failing robot (which needs its own particle child). Godot handles a few hundred `Node2D`s, but not while also running a fog shader and two scrolling text feeds on an 8 GB M1.
- `ReasoningFeed`: `RichTextLabel`, append `reasoning` + source badge + latency. Highest-value element for non-technical judges — per the original spec, prioritize its reliability over polish elsewhere.
- `Ticker`: fast scroll of `last_action_reason` deltas. This is what makes Tier 2 visible next to Tier 3.
- Failure VFX: `GPUParticles2D` one-shot at the event position + `Camera2D` shake via a 0.4 s decaying offset.
- Comms links: `Line2D` between antenna robots and their component neighbours (P2).

---

## 11. Testing

| Test | Asserts |
|---|---|
| `test_contracts.py` | every schema round-trips; `SCHEMA_VERSION` matches what Godot's parser expects |
| `test_determinism.py` | seed 42 twice → identical scorecard hash. Guards R7 |
| `test_no_ground_truth_leak.py` | no swarm/hivemind module references the ground-truth topic. Guards the fog-of-war claim |
| `test_auction.py` | capability gating; greedy award correctness; **kill a robot → its task is reassigned to a different capable robot within 3.0 s** |
| `test_filter.py` | 14 adversarial directive payloads (malformed JSON, unknown sector, abandon-everything, abandon-with-victim, capability that doesn't exist, 40 directives, …) |
| `test_tier1_safety.py` | 10k random goal/obstacle configs → zero wall penetrations |
| `test_mission.py` | full headless mission, seed 42, **with the hivemind disabled** → the swarm allocates, executes, self-heals and finds casualties with Tier 3 silent. This is the "swarm survives without the LLM" claim, as a test. Runs against `test.yaml` and is deliberately **not** a mission-quality benchmark — that is `scripts/baseline_sweep.py` on demo seeds, per [PLAN.md §6.1](PLAN.md) |
| `test_comms.py` | a robot outside the component does not update the blackboard; reconnect flushes the buffer |

`make check` = `ruff check` + `pytest` + the tiny fixture headless mission. Green on every commit.

---

## 12. Determinism rules (non-negotiable)

Rehearsed demo timings only hold if the run is reproducible.

1. One `numpy.random.Generator` per subsystem, all derived from the master seed via `SeedSequence.spawn()`. No global `np.random`, no bare `random`.
2. **Never iterate a `set` or an unordered `dict` inside the sim or auction loop.** Sort by robot/task id.
3. Ties in the auction break on `robot_id` lexicographically, never on insertion order.
4. Wall-clock time never enters sim logic. `DemoSim` uses it only for sleeping.
5. **The hivemind is not deterministic, and cannot be made so.** Two independent reasons, and it is worth being precise about both because only one is usually named:
   - *Sampling.* A model at `temperature > 0` returns different directives for identical prompts.
   - *Latency.* Rung selection depends on whether a provider answers inside its 4 s timeout, which depends on machine load. Even a greedily-decoded model would drift, because a rung that answered in 3.9 s on one run answers in 4.1 s on the next and the ladder falls through to a different provider.

   So the reproducibility guarantee is scoped, not universal:

   | Path | Deterministic? |
   |---|---|
   | `run_mission()` — training rollouts, MAP-Elites, the gate | **Yes.** Tier 3 is off by default here (D7) precisely so evolution optimises Tier 2 in isolation |
   | `Mission(scripted_hivemind=True)` | **Yes.** No network, no sampling, no timeout — `tests/test_mission.py::test_a_mission_is_deterministic_with_the_hivemind_running` asserts it |
   | `Mission()` with a live model | **No**, and no test claims otherwise |

   The rehearsed demo timings in [RUNBOOK.md](RUNBOOK.md) therefore hold for the *swarm's* behaviour, which is what the beats depend on. The hivemind's exact wording varies run to run — that is honest and worth saying aloud to a judge rather than hiding.

---

## 13. Optional: `GazeboSim` (stretch, event-only, time-boxed 6 h)

Only if everything else is done. Docker `linux/arm64`, `ros:humble` + Gazebo, XQuartz for the GUI, 4–6 robots, Nav2 per robot, `Ros2Bus` as the active transport. Genome → URDF via `sim/gazebo/urdf_gen.py`. Expect RTF ~0.3–0.5×. **Its only purpose is a 30-second "and here it is in a real physics stack" side clip.** It is not the demo and it never becomes the demo.

---

## 14. Open questions

1. Godot 4.4 vs 4.5 — pick whichever has the stabler macOS arm64 build and pin it in `RUNBOOK.md`.
2. Qwen2.5-1.5B vs 3B — start at 1.5B (fits T4 GRPO comfortably, ~1.0 GB local). Revisit only if the gate shows the 1.5B can't hold the JSON schema *and* reason.
3. Whether `retreat` should be a task or a Tier-1 override. Task, for now — it shows up in the event log, which is worth more than the marginal robustness.
