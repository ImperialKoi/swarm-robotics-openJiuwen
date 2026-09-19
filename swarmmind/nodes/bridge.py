"""The dashboard bridge: the node graph on one side, Godot on the other.

**This is the only module permitted to touch ground truth.** `/world/ground_truth` exists
so the operator can toggle god-view and show what the swarm has *not* found; no swarm
node, control module, or hivemind provider may read it
(tests/test_no_ground_truth_leak.py, CLAUDE.md invariant #4).

Wire format, one JSON object per WebSocket text frame:

| `t` | rate | payload |
|---|---|---|
| `hello` | on connect | scenario meta, sector rects, robot roster -- small |
| `blob` | on connect | terrain PNG / heightfield / occupancy, **chunked** (see below) |
| `state` | 10 Hz | robots as a flat array, HUD counters, believed contacts, dig sites |
| `fog` | 2 Hz | explored mask, bit-packed and base64'd |
| `truth` | 2 Hz | **ground truth** -- real casualty positions and the true hazard disc |
| `event` | immediate | one `/swarm/events` message |

Inbound, dashboard -> simulator, the same framing the other way: `drive` and `release`
(control/manual.py). They are the operator's WASD override of one followed robot, and the
`state` frame echoes who holds it in `manual`, which is what the dashboard's badge draws.

Robots are sent as a flat array of numbers rather than objects: at 512 robots and 10 Hz
the object form is ~1 MB/s of JSON, the flat form ~150 KB/s, and Godot parses it with a
single loop. If that ever bites, the next step is binary packing, not fewer robots.

**The map payload is chunked, and it has to be.** Godot's `WebSocketPeer` defaults to a
64 KB inbound buffer and drops the connection when a single frame exceeds it -- silently,
with no error on either side. Terrain, heightfield and occupancy together are ~100 KB
even on the small scenario, so sending them in one `hello` made the dashboard sit on
"waiting for simulator" forever while the simulator happily reported a client. Chunks are
capped well under the default so the dashboard works without depending on the client
having raised its buffer.
"""

from __future__ import annotations

import base64

import numpy as np

from ..contracts import topics
from ..contracts.schemas import DashboardFlight
from ..contracts.version import SCHEMA_VERSION
from ..control.manual import ManualOverride
from ..perception.raster import AppearanceRaster
from ..sim.robot import CHASSIS_INDEX, LANES
from ..sim.world import CARRIED, CLEARED, RESCUED
from ..viz import png

LANE_INDEX_OF = {a: i for i, a in enumerate(LANES)}
STATUS_CODE = {"active": 0, "out_of_comms": 1, "failed": 2, "destroyed": 3}

#: Contact-ring codes, sent as `reports[i][2]`. The dashboard indexes `CONTACT_COLORS`
#: in main.gd with this, so **the order here is the wire contract** -- adding a state
#: means adding a colour, and tests/test_bridge_protocol.py fails the build if the two
#: lists drift apart.
#:
#: This used to be the tracker's own report state, which gave the ring exactly two
#: colours: amber for anything resolved and a near-invisible grey for everything else.
#: Every casualty the swarm was working on therefore looked identical, whether it was
#: still under a slab or already on a carrier's back halfway home.
#:
#: 0 is a *belief*: a confirmed contact nobody has reached yet, and 56% of those are
#: rubble (MEASUREMENTS.md M-8). 1-4 are contacts resolved onto a real casualty, coloured
#: by what that casualty needs next. Reading victim state here is not a ground-truth leak
#: -- nodes/tasks.py already reads these exact fields on these exact victims to choose
#: between a dig and an extract, so the ring shows what the swarm is acting on anyway,
#: and the bridge is the one place allowed to see the world (CLAUDE.md invariant 4).
CONTACT_CODE = {"unverified": 0, "buried": 1, "surface": 2, "dug": 3, "carried": 4}


class BridgeNode:
    def __init__(self, world, server, bus=None, *, state_hz: float = 10.0,
                 fog_hz: float = 2.0, truth_hz: float = 2.0) -> None:
        self.server = server
        self.bus = bus
        self.world = world
        self._terrain_png: str | None = None
        tick_hz = world.scn.rates.tick_hz
        self._state_every = max(1, int(round(tick_hz / state_hz)))
        self._fog_every = max(1, int(round(tick_hz / fog_hz)))
        self._truth_every = max(1, int(round(tick_hz / truth_hz)))
        self.commands: list[dict] = []
        #: The operator's WASD override. Here rather than on the Mission because the
        #: bridge is the only way a command can arrive: `--headless` has no bridge, so
        #: no evaluation or hash can see this.
        self.manual = ManualOverride(world)
        server.on_connect = self._greet
        if bus is not None:
            bus.subscribe(topics.SWARM_EVENTS, self._on_event)

    # ------------------------------------------------------------------ hello

    def _terrain(self, world) -> str:
        """Static terrain as a PNG, sent once. Godot decodes it into a texture."""
        if self._terrain_png is None:
            ras = AppearanceRaster(world)
            self._terrain_png = base64.b64encode(png.encode(ras._static)).decode()
        return self._terrain_png

    #: Base64 characters per `blob` message. Well under Godot's 64 KB default inbound
    #: buffer, with room for the JSON envelope.
    CHUNK = 32000

    def _send_blob(self, client, name: str, payload: str) -> None:
        total = max(1, -(-len(payload) // self.CHUNK))
        for i in range(total):
            self.server.send_to(client, {
                "t": "blob", "name": name, "i": i, "n": total,
                "data": payload[i * self.CHUNK:(i + 1) * self.CHUNK],
            })

    def _greet(self, client) -> None:
        w = self.world
        sectors = []
        for sid in w.sector_ids:
            x0, y0, x1, y1 = w.scn.sector_rect(sid)
            sectors.append({"id": sid, "r": [x0, y0, x1, y1]})
        self.server.send_to(client, {
            "t": "hello",
            # The dashboard refuses to run against a contract it does not know. Without
            # this a stale Godot build against a newer sim fails the M-12 way: silently,
            # with both sides believing they are fine.
            "schema": SCHEMA_VERSION,
            "scenario": w.scn.name,
            "cell": w.cell,
            "w": w.shape[1],
            "h": w.shape[0],
            "map_m": [w.scn.map.width_m, w.scn.map.height_m],
            "sectors": sectors,
            "lanes": list(LANES),
            "victims_total": len(w.victims),
            "robots": [{"id": rid, "lane": int(w.actuator[i])}
                       for i, rid in enumerate(w.robot_ids)],
            "zones": [list(w.scn.base), *[list(z) for z in w.scn.extraction_zones]],
            "zone_r": w.scn.extraction_radius,
            "height_scale": self._height_scale(w),
            "water_scale": self._water_scale(w),
        })
        self._send_blob(client, "terrain", self._terrain(w))
        self._send_blob(client, "height", self._heightfield(w))
        self._send_blob(client, "water", self._waterfield(w))
        self._send_blob(client, "occ",
                        base64.b64encode(w.occ.astype(np.uint8).tobytes()).decode())
        self.server.send_to(client, {"t": "hello_done"})

    def _height_scale(self, world) -> float:
        return float(max(world.height.max(), 1e-3)) / 255.0

    def _water_scale(self, world) -> float:
        return float(max(world.water.max(), 1e-3)) / 255.0

    def _waterfield(self, world) -> str:
        """Per-cell water depth, quantised to a byte, zero on dry land.

        Water gates traversal -- it is the boundary between what a tracked unit can cross
        and what only a legged one can -- and it was not on the wire at all, so the
        dashboard drew rivers as dry trenches. A judge watching a wheeled robot stop at
        an invisible line learns nothing from it. Its own scale, not the heightfield's: a
        byte over ~1.1 m of depth is 4 mm, where sharing the height scale would have put
        the whole river inside two quantisation steps.
        """
        q = np.clip(world.water / self._water_scale(world), 0, 255).astype(np.uint8)
        return base64.b64encode(q.tobytes()).decode()

    def _heightfield(self, world) -> str:
        """Per-cell elevation, quantised to a byte.

        The dashboard extrudes its 3D world from this, so what is rendered and what is
        simulated come from one array. A byte gives ~3 cm resolution over an 8 m range,
        which is finer than anything the eye resolves at dashboard zoom, and keeps the
        payload at ~90 KB base64 instead of ~350 KB as float32.
        """
        q = np.clip(world.height / self._height_scale(world), 0, 255).astype(np.uint8)
        return base64.b64encode(q.tobytes()).decode()

    # ------------------------------------------------------------------ per tick

    def step(self, world, executor, tracker) -> None:
        if self.server.client_count == 0:
            self._take_commands(world)
            return
        if world.tick % self._state_every == 0:
            self.server.broadcast(self._state(world, executor, tracker))
        if world.tick % self._fog_every == 0:
            self.server.broadcast(self._fog(world))
        if world.tick % self._truth_every == 0:
            self.server.broadcast(self._truth(world))
        self._take_commands(world)

    def _take_commands(self, world) -> None:
        # In arrival order, so a `release` of the old unit and the first `drive` of the
        # new one, sent in the same dashboard frame, land on the same tick.
        for c in self.server.poll_commands():
            if not self.manual.receive(world, c):
                self.commands.append(c)

    def _state(self, world, executor, tracker) -> dict:
        # 8 columns. chassis is on the wire because locomotion is half of what makes a
        # robot's behaviour legible -- a wheeled unit stranded in marsh and a legged one
        # strolling through it look identical without it -- and activity is the other
        # half. `executor` was already a parameter here and was never read, so a relay
        # holding the link a dozen robots report through drew as the same still box as
        # one wedged against a rock. Of ~80 stationary robots on seed 42, ~32 were
        # working correctly and ~42 were stuck, and the dashboard could not say which.
        r = np.empty((world.n, 8), dtype=object)
        pos = np.round(world.pos, 2)
        r[:, 0] = pos[:, 0]
        r[:, 1] = pos[:, 1]
        r[:, 2] = np.round(world.theta, 2)
        r[:, 3] = world.actuator
        r[:, 4] = world.status
        r[:, 5] = np.round(world.battery, 2)
        r[:, 6] = world.chassis
        r[:, 7] = executor.activity
        alive = world.status <= 1
        reports = self._contacts(world, tracker)
        # Tier 3 state, one code per sector in `sector_ids` order:
        # 0 high, 1 normal, 2 low, 3 abandoned. Sent every frame rather than on change
        # because it is 48 small ints and a dashboard that joins late must not show a
        # stale map -- reconnecting mid-demo is the case this protects.
        sec = np.where(world.sector_abandoned, 3, world.sector_priority).astype(int)
        return {
            "t": "state",
            "tick": world.tick,
            "time": round(world.t, 2),
            "r": r.tolist(),
            "flight": DashboardFlight(airborne=(
                world.airborne & alive & (world.chassis == CHASSIS_INDEX["rotor"])
            ).tolist()).model_dump(),
            "sec": sec.tolist(),
            "hud": {
                "rescued": world.victims_rescued,
                "found": world.victims_found,
                "total": len(world.victims),
                "active": int(alive.sum()),
                "lost": world.robots_lost,
                "incomms": int(world.in_comms.sum()),
                "explored": round(float(world.explored[world.passable].mean()), 4),
            },
            "reports": reports,
            # Excavation sites, forwarded verbatim from `world.digging` so the
            # dashboard's dust is driven by the predicate that actually moves debris
            # rather than by a robot standing near a ring. `[x, y, diggers]`, with
            # `diggers` already capped at MAX_DIGGERS -- it is the rate multiplier, so
            # the plume scales with how fast the slab is coming off rather than with how
            # many machines are milling about. At most one row per buried casualty (32
            # on the demo map), so tens of bytes in a frame with a 64 KB ceiling.
            "digs": [[round(x, 1), round(y, 1), n] for x, y, n in world.digging],
            # `[robot index, seconds until autonomy]` while the operator has a unit, else
            # empty. The simulator's word, so the dashboard's MANUAL badge cannot claim
            # control the robot is not actually under.
            "manual": self.manual.wire(world),
        }

    def _contacts(self, world, tracker) -> list[list]:
        """Contact rings for the dashboard, following the casualty rather than the sighting.

        A report records *where a contact was seen*, which is the right thing for the
        tracker to remember and the wrong thing to leave on screen: once a carrier picks
        the casualty up, the ring sat at the original rubble pile for the rest of the
        mission while the casualty travelled home. On a 480x320 m map that reads as "the
        swarm found people and abandoned them".

        So a resolved report follows its casualty while it is being carried, and is
        dropped once delivered -- the ring travels back to base with the carrier and then
        goes out, which is the whole rescue visible in one marker.

        The third field is a `CONTACT_CODE`: what the casualty needs next, so the ring's
        colour tracks the rescue rather than being amber from the moment of discovery.

        **One ring per casualty, not per report.** Several reports resolving onto the
        same casualty is the tracker working -- corroboration from different angles is
        the whole point of it, and nodes/tasks.py already dedups the same way before it
        offers work. On the wire it was a disaster: at t=60 on seed 42 the bridge sent
        **817 rings for 26 distinct casualties, 103 of them stacked on one body**
        (MEASUREMENTS.md M-64). Coincident rings at identical positions z-fight into a
        solid blob, so the ring count tracked how *often* the swarm had looked at
        somebody rather than how many people it had found, and no amount of colour would
        have shown through the pile. It also put ~800 rows in a 10 Hz frame that has a
        64 KB ceiling.

        The first report wins, in `tracker.reports` order -- a list, so this stays
        deterministic (invariant 6). The `seen` set is membership-tested only, never
        iterated.
        """
        out: list[list] = []
        seen: set[int] = set()
        for r in (tracker.reports if tracker else []):
            if r.state not in (1, 2):
                continue
            x, y = float(r.pos[0]), float(r.pos[1])
            code = CONTACT_CODE["unverified"]
            if r.state == 2 and r.victim is not None:
                v = world.victims[r.victim]
                if v.state == RESCUED:
                    continue                      # home. Stop drawing it.
                if r.victim in seen:
                    continue                      # already drawn; this is corroboration.
                seen.add(r.victim)
                if v.state in (CARRIED, CLEARED):
                    x, y = float(v.pos[0]), float(v.pos[1])
                code = self._contact_code(v)
            out.append([round(x, 1), round(y, 1), code])
        return out

    @staticmethod
    def _contact_code(v) -> int:
        """Which stage of the rescue a resolved casualty is at.

        A casualty that was never buried leaves FOUND on the tick after it is found
        (world.py `_step_victims`), so FOUND on screen always means *still under debris*.
        CLEARED then splits on `buried`: dug out by a scoop robot, or lying in the open
        the whole time. Both want a carrier next, but a dug-out casualty is dig work the
        swarm has already finished, and collapsing the two hid the entire scoop lane's
        contribution behind the same marker as a casualty nobody had to touch.
        """
        if v.state == CARRIED:
            return CONTACT_CODE["carried"]
        if v.buried and v.debris_remaining > 0.0:
            return CONTACT_CODE["buried"]
        return CONTACT_CODE["dug"] if v.buried else CONTACT_CODE["surface"]

    def _fog(self, world) -> dict:
        bits = np.packbits(world.explored.ravel())
        return {"t": "fog", "bits": base64.b64encode(bits.tobytes()).decode()}

    def _truth(self, world) -> dict:
        """Ground truth. Dashboard only -- this is the god-view toggle's data source."""
        h = world.hazard
        return {
            "t": "truth",
            "v": [[round(float(v.pos[0]), 1), round(float(v.pos[1]), 1), int(v.state),
                   int(v.buried)] for v in world.victims],
            "hz": [round(float(h.centre[0]), 1), round(float(h.centre[1]), 1),
                   round(float(h.radius), 1)] if h.active else None,
        }

    def _on_event(self, ev: dict) -> None:
        if self.server.client_count == 0:
            return
        self.server.broadcast({
            "t": "event", "kind": ev.get("kind"), "text": ev.get("text", ""),
            "time": ev.get("t", 0.0), "pos": ev.get("pos"), "sector": ev.get("sector"),
        })
