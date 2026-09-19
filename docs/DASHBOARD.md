# Running the dashboard

Everything you need to watch a SwarmMind mission, in order. No prior context assumed.

---

## 1. Start the simulator

Two terminals. The first runs the swarm, the second is Godot.

```bash
cd ~/Documents/swarm-robotics-rl

# small and fast -- 16 robots, good for checking the connection works
uv run python -m swarmmind.cli run --demo --scenario test --wait

# the demo -- 512 robots on the 360x240 m map
uv run python -m swarmmind.cli run --demo --scenario demo --wait
```

`--wait` holds the simulation at t=0 until a dashboard connects, so you see the mission
from its first tick instead of joining thirty seconds late. Drop it and the swarm starts
immediately.

It prints:

```
  SwarmMind  scenario=demo  seed=42  robots=512
  dashboard: ws://127.0.0.1:8765   (open the Godot project and press F5)
```

Leave it running. `Ctrl-C` stops it.

### Optional: the hivemind's local model

```bash
./scripts/serve_hivemind.sh          # in a third terminal, before the simulator
```

**Not required.** With nothing listening on port 8080 the provider ladder drops to its
scripted rung in under a millisecond and the demo runs exactly the same, minus the
model's own wording in the feed. Nothing hangs waiting for it.

To watch the swarm with no strategic layer at all, add `--no-hivemind`. That is the
control condition, not a broken mode — the swarm is built to work without it.

---

## 2. Open Godot

1. Launch **Godot 4.7.2**.
2. **Import** → select `~/Documents/swarm-robotics-rl/godot/project.godot` → **Open**.
3. Press **F5**.

The window says `waiting for simulator` until it connects, then loads the map. It
reconnects on its own, so the order you start things in does not matter.

If the robots still appear as plain coloured boxes, check the project path. The older
`~/Documents/swarm-robotics/godot` checkout contains the box renderer. Restarting that
project does not load changes from `swarm-robotics-rl`. Open the project above, then
click a robot and press `F` twice from orbit to inspect its model in chase view.

---

## 3. What you are looking at

### The HUD

The 3D world shows through the middle of the window, framed by instrument panels.
Every number on them is read off the simulator's wire — nothing is decorative, and none
of it is ground truth. The HUD lives in `godot/scripts/hud.gd`.

**Top bar**

| | |
|---|---|
| mission clock | simulated time, `T+` seconds |
| rescued | casualties delivered, out of the number seeded |
| casualties found | real casualties the swarm has found |
| swarm active | robots alive, with how many have been lost |
| area explored | share of passable ground any robot has seen, and how fast that is rising (% per minute over the last 30 s) |
| link state | `CONNECTED` / `WAITING`, the simulator's address, and the measured rate of state frames (should read ~10 Hz) |

**Left rail**

| | |
|---|---|
| contact taxonomy | contact rings on the map, counted by colour — the same five colours as the rings |
| hivemind sectors | one cell per sector. **Hue** is the hivemind's decision (blue high priority, grey low, amber abandoned, teal normal); **brightness** is how much of that sector the swarm has searched |
| comms mesh | alive, in comms, out of contact, relays holding a post, relays moving, lost |
| chassis roster | alive / total for each chassis, in the same colours `C` paints the robots — a chassis whose bar shrinks is the one getting killed |

**Right rail**

| | |
|---|---|
| overlay bus | every toggle and its state. **Click a row** to flip it — same as the key |
| camera rig bindings | the controls in §4 |
| swarm activity | what the robots are doing right now, one row per activity: exploring, investigating a contact, clearing debris, carrying a casualty, holding a relay post, and so on. The row of the robot you are following is marked `▸`. **Idle turns red** when any robot has no task and nowhere useful to go — the one row that means something is wrong |

Swarm activity is there because a still robot on the map is ambiguous: a relay holding
its post and a robot wedged against a rock are the same motionless box. This panel says
how many of each there are.

**Bottom bar**

| | |
|---|---|
| event stream | every `/swarm/events` message: time, a short tag in the event's colour, the text. Scroll back with the wheel |
| search coverage | explored share of the map, rescue rate per minute, and a ledger: digs in progress, casualties on carriers, casualties waiting for a carrier, and contacts dismissed as false |
| hivemind | the last directives from Tier 3, and any its safety filter rejected, kept apart from the stream so they do not scroll away. `no directive received yet` is normal under `--no-hivemind` |

**Frame round the view**

| | |
|---|---|
| top left | `VIEWPORT · FOG OF WAR` normally. Turns amber — `GOD VIEW · GROUND TRUTH SHOWN` or `CASUALTY PINS · GROUND TRUTH` — whenever ground truth is on screen, so a screenshot cannot pass for what the swarm sees |
| top centre | view mode, camera bearing, elevation and field of view |
| top right | frame rate and primitives drawn |
| bottom left | the ground under the mouse, in map metres, with its elevation |
| bottom centre | the robot you are following: id, job, chassis, battery, what it is doing |
| bottom right | how many metres one pixel covers at the camera's target |
| crosshair | orbit view only: the point the camera orbits, always in the middle of the view. `RANGE` is the distance from the camera to the ground under it, measured along the line of sight against the terrain — not the orbit distance |

### Robots — coloured by **job**

| colour | job | what it does |
|---|---|---|
| 🔵 light blue | **scout** | no actuator. Fast and far-seeing; clears fog and finds casualties |
| 🟠 amber | **digger** | scoop. Removes debris from buried casualties |
| 🟢 green | **carrier** | gripper. Picks casualties up and carries them to a collection point |
| 🟣 purple | **relay** | antenna. Holds position to extend the comms network |
| 🟥 dark red | **destroyed** | any lane, killed by the hazard or a flat battery |

Every casualty needs the chain: a scout finds it, a digger uncovers it if buried, a
carrier takes it home. A relay keeps them all in contact while they do it.

The robots now have distinct 3D tools and animated chassis: camera scouts, bucket diggers,
stretcher carriers and mast relays, in 15 valid combinations. See the
[animated fleet gallery](units/index.html) and [model notes](UNIT_MODELS.md).
Wheels and legs follow measured travel; digging requires an active excavation; the
carrier's rescue load appears only while carrying. Rotor blades animate at ground
height because airborne state is absent from the dashboard telemetry.

Offscreen units and terrain chunks are culled and unloaded after a short reuse grace.
Orbit simplifies distant geometry; first-person and chase keep every visible terrain
chunk and ruin at full detail. The selected robot stays detailed. New terrain tiles
load progressively, with nearby tiles first. See [terrain rendering](TERRAIN_RENDERING.md).

**Only if a unit policy is running** (`--unit-policy`, off in the demo): a robot it has staged shows activity *drift*
(the frozen contract has no new code for it) and a follow-cam reason starting `unit policy:` —
"waiting at a dig for the casualty", "waiting near a contact", "heading for unexplored ground",
"holding position". The demo plays seeds 42–45; start it with `--seed` set to one of them.

### Robots — coloured by **locomotion** (press `C`)

Colour normally shows the job. `C` switches it to show the chassis instead, which is the
view that answers *"is it always the wheeled ones getting stuck?"*.

| colour | chassis | slope | water | speed |
|---|---|---|---|---|
| 🔴 red | **wheeled** | ≤ 0.40 | none | fastest (1.18×) |
| 🟡 yellow | **tracked** | ≤ 0.58 | fords 0.45 m | 1.00× |
| 🔷 cyan | **legged** | ≤ 1.60 | fords 1.20 m | slowest (0.80×) |

Chassis is mixed evenly *within* every job, on purpose: a casualty on the far bank of a
river needs a carrier that can cross it, which only matters if carriers differ among
themselves.

### Everything else

| | |
|---|---|
| 🔴 red pins | casualties — **ground truth**, only shown with `V` or god-view |
| ⚪ white rings | contacts the swarm *believes* it has found. Some are wrong |
| 🟠 orange dome | the hazard. It grows and drifts, and it kills robots |
| dark ground | fog of war — nowhere any robot has been |
| blue wash | a sector the hivemind marked **high priority** |
| amber wash | a sector the hivemind **abandoned** |
| grey wash | a sector marked **low priority** |

### Particle effects

Digging was the one stage of the rescue chain with nothing to show for itself — a scoop
robot clearing a slab and a scoop robot sitting idle are the same box at the same
coordinates for twenty seconds. What each effect means:

| | |
|---|---|
| pale dust column with dark chips | a **dig in progress**. Emitted from the simulator's own excavation predicate, so it appears exactly while debris is being removed, and thickens with the number of diggers on the hole |
| one broad white puff | the slab came off — a casualty **dug out**, matching `victim_cleared` in the feed |
| green plume | a casualty **delivered** to an extraction zone |
| orange sparks over dark smoke | a robot **destroyed** |
| orange embers over the dome | the hazard burning. Ground truth, so like the dome itself it only appears under `V` or god-view |

Press `P` to turn all of it off; the particles row in the overlay bus then reads `OFF`. Everything is one
`MultiMesh` of at most 448 cubes — one draw call — and nothing is emitted more than 190 m
from the camera.

The event stream along the bottom is colour-coded too: green `RESCUE`, orange-red `LOST`
for a robot destroyed, violet `ISSUED` for a hivemind directive, pink `FILTER` for one
its filter **rejected**. A rejection is a good moment, not a bug — it is the safety net
catching a bad plan.

---

## 4. Controls

| key | does |
|---|---|
| `E` | **eagle-eye** — whole map at once. Start here |
| `G` | **god-view** — removes the fog and shows the truth. For debugging, not for judging |
| `V` | casualty pins and contact rings |
| `C` | colour robots by **chassis** instead of job |
| `S` | hivemind sector tint on/off |
| `P` | particle effects on/off |
| `F` | cycle the view of the followed robot: **POV** → **chase** → back to the orbit. One press is first-person from the robot's own eye; two is the close third-person follow camera |
| click | follow a robot. The foot of the view names its job, chassis, battery and activity |
| `Esc` | stop following |
| wheel | zoom |
| drag | orbit |
| right-drag | pan |

Every toggle is also a clickable row in the HUD's overlay bus. The top of the view names
the view you are in — `ORBIT`, `POV` or `CHASE`.

**Chase view** sits about 4 m behind the unit and looks at it, close enough to read the
chassis and still show the ground it is walking into. It has its own zoom, rotation and
framing, kept separate from the orbit's so that pressing `F` never loses the wide shot:

| | |
|---|---|
| drag | swing the camera around the unit. The angle is held relative to the unit's heading, so it stays behind as the robot turns |
| wheel | 1.8 m to 30 m |
| right-drag | slide the unit around inside the frame |
| `Esc` / `E` | back to the orbit, and the chase camera resets |

Pressing `F` with nothing followed picks the first robot, and clicking another robot
while in chase snaps straight to it rather than flying across the map.

### The contact overlay

In POV and chase — and only there — the followed robot's contacts are boxed **over the
world itself**, where each one stands. Colour is the contact code from the legend, and
the label carries the range and bearing *the robot* has, not the camera's.

A box is drawn for a contact inside the robot's own 90° camera cone within 26 m. That
test is the detector's, so a casualty in plain view on your screen but behind the robot's
shoulder is not boxed — the overlay is what the machine has, not what the operator can
see. The readout at the foot of the view counts what is in field.

The detector is flat (perception is 2.5D; terrain height is render-only), so a contact
can be in field and still be off the top of the frame — the demo map moves 3.7 m in
height over 5 m of ground at the 95th percentile, which is a whole screen at close range.
Those get a chevron on the screen edge with their range instead of a box, rather than
disappearing.

Boxes do not hide behind walls. These are beliefs the swarm holds, not pixels the
detector is looking at, so a contact known to be behind rubble still draws.

---

## 5. Reading a run

A healthy mission looks like this:

1. **0–20 s** the swarm leaves the base staging ring and fans outward.
2. **20–120 s** fog peels back. White rings appear as the detector reports contacts —
   *including wrong ones*, which later vanish when a robot gets close and finds nothing.
3. **~90 s** the hazard ignites and starts growing. Robots inside it lose battery fast.
4. **throughout** casualties get dug out and carried home; the rescued count climbs.
5. **on a loss** the feed shows `robot_destroyed`, then `task_orphaned`, then the task
   being re-awarded to somebody else within three seconds. That is the self-healing
   claim, visible.

**Known problem in the measured build:** the swarm does not spread out properly. The median robot
ends a seven-minute mission only ~43 m from base on a map whose diagonal is 577 m, so
most of the map is never searched and only ~7% of casualties are rescued. It is not
crowding — a 48-robot swarm stops at the same distance — and it is not the auction,
which has work assigned to 528 of 768 robots. If the swarm looks like it is milling
around near home, that is this bug and not your machine.

---

## 6. If something looks wrong

| symptom | cause |
|---|---|
| stuck on `waiting for simulator` | the simulator is not running, or is on another port. It prints its URL |
| connects, then immediately drops | a frame exceeded Godot's 64 KB inbound buffer. The map is sent in 32 KB chunks to avoid exactly this; if it recurs, something new is being sent whole |
| very low frame rate at 768 robots | expected on 8 GB. The simulator alone runs at 0.94× real time; Godot shares the same eight cores. Use `--scenario test`, or `--no-hivemind` to free ~830 MB |
| no directives in the feed | no model server. Harmless — the scripted rung still issues them. `hivemind_offline` in the feed confirms it |
| casualties never appear | press `V`. They are hidden by default because the swarm is not supposed to know where they are |

---

## 7. Without Godot

The terrain and model mathematics can also be verified offline, without a Godot window:

```bash
uv run python scripts/snapshot3d.py --scenario test --at 0
uv run python scripts/snapshot3d.py --scenario demo --at 0 --pov 0 --chase 0  # construct only
```

`--fx` runs `swarmmind/viz/particles.py` alongside the mission and draws the field, and
writes a `dig*.png` from beside each excavation. That module is the particle model of
record — `main.gd`'s `FX_KINDS` is a port of it, and `tests/test_bridge_protocol.py`
compares the two tables number by number.

Frames land in `runs/`. `scripts/fullscale.py` also prints a full breakdown: survivors by
lane and chassis, comms, perception counts, hivemind latency, and per-tick cost.
