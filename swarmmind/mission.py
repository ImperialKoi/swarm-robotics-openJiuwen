"""Assembly point: wire the world, Tier 1 and Tier 2 into a runnable mission.

D3 replaces ``HeuristicAssigner`` here with the bus plus the real node graph
(blackboard, auction, events, fault_injector). The signature stays the same so the
training rollouts and the gate do not change.
"""

from __future__ import annotations

import dataclasses
import time

import numpy as np

from .bus.local import LocalBus
from .control.heuristic import GreedyAssigner
from .control.planner import NavSet
from .control.tier1_reflex import ReflexController, ReflexParams
from .hivemind.fallback import build_ladder
from .metrics import Scorecard
from .nodes.auction import AuctionNode
from .nodes.blackboard import BlackboardNode
from .nodes.command import Commander
from .nodes.events import EventLog
from .nodes.fault_injector import FaultInjector
from .nodes.hivemind import HivemindNode
from .nodes.skill_executor import SkillExecutor
from .perception.camera import CameraRig
from .perception.classical import ClassicalVictimDetector
from .perception.raster import AppearanceRaster
from .perception.tracker import VictimReportTracker
from .sim import grid
from .sim.backend import DemoSim, FastSim
from .sim.robot import CHASSIS_INDEX, OUT_OF_COMMS
from .sim.scenario import Scenario


class Mission:
    """One mission: world + navigation + Tier 1 + Tier 2."""

    def __init__(self, scenario: Scenario, seed: int, *, realtime: bool = False,
                 params: ReflexParams | None = None, detector=None,
                 allocator: str = "auction", hivemind: bool = True,
                 allow_api: bool = False, scripted_hivemind: bool = False,
                 genes=None, command: bool = False, evolved: bool = True,
                 unit_policy=None, zone_routing: bool = False, response_team=None,
                 edge_steering: bool = False, voice=None) -> None:
        if response_team is not None and not hivemind:
            raise ValueError("response_team requires the directive filter; remove --no-hivemind")
        if voice is not None and not hivemind:
            raise ValueError("voice requires the directive filter; remove --no-hivemind")
        self.sim = (DemoSim if realtime else FastSim)(scenario, seed, evolved=evolved)
        #: Optional per-robot decision layer inside Tier 2 (`control/unit_policy.py`).
        #: Consulted once per auction cycle, after the auction has allocated, for
        #: searching robots only. `None` is the shipped swarm, unchanged.
        self.unit_policy = unit_policy
        w = self.sim.world
        self.nav = NavSet(w, edge_steering=edge_steering)
        # **Off by default, and that is a measurement rather than caution (M-76f).** The fix
        # is real with Tier 3 silent -- +7.5 rescues on the M1 and +15.0 on Kaggle, up on 8
        # of 8 seed-arms -- but the demo runs Tier 3 live, and there the two machines
        # disagree in *sign*: +9.5 on Kaggle (M-76e), -1.25 on the demo machine itself.
        # Platform float alone moves a single seed by ~10 rescues here, which is the size of
        # the effect. Turn it on with `zone_routing=True` for a swarm with no strategic
        # layer, where the evidence is unambiguous.
        if zone_routing:
            self.nav.enable_zone_routing(w)
        self.reflex = ReflexController(w, params)
        #: Evolved Tier-2 behaviour, one genome per robot. `None` is the classical
        #: baseline -- and is what `run_mission` uses, so the gate's control arm is the
        #: hand-tuned heuristic rather than a degenerate genome.
        self.genes = genes
        self.reflex.set_genes(genes)
        self.executor = SkillExecutor(w)

        # Node graph over the bus. Topic names and payload shapes are the contract's,
        # so a Ros2Bus swap changes nothing above this line.
        # Tier 3a: territory. Built before the allocator, which consults it.
        #
        # **Off by default until it earns its place.** The hand-set commander is
        # currently a net negative -- 8 rescues against 10 without it -- because 16
        # wedges cannot separate 352 scouts and fencing relays confines the comms
        # chain. Training it is the point (`training/command/`); shipping an
        # untrained one that loses to no commander at all would be the same mistake
        # the gate exists to prevent.
        # Built before the commander, which reads its open reports for `w_contacts`.
        # Perception's other pieces stay below; only the belief store is needed here.
        self.tracker = VictimReportTracker()
        self.commander = Commander(w, tracker=self.tracker) if command else None

        self.bus = LocalBus()
        self.events = EventLog(self.bus)
        self.blackboard = BlackboardNode(self.bus)
        self.fault = FaultInjector(scenario)

        period = 1.0 / scenario.rates.auction_hz
        self.allocator_name = allocator
        if allocator == "auction":
            self.allocator = AuctionNode(w, self.nav, genes=genes,
                                         commander=self.commander)
            self._alloc_period = period
            self._alloc_next = 0.0
        elif allocator == "greedy":
            self.allocator = GreedyAssigner(w, period=period)
            self._alloc_period = period
            self._alloc_next = 0.0
        else:
            raise ValueError(f"unknown allocator {allocator!r}; use 'auction' or 'greedy'")

        self.bridge = None
        self._hb_every = max(1, int(round(scenario.rates.tick_hz / scenario.rates.heartbeat_hz)))
        self._bb_every = max(1, int(round(scenario.rates.tick_hz / scenario.rates.blackboard_hz)))

        # Perception. The detector is swappable (classical now, CNN at D11); nothing
        # above this line knows which one is running.
        self.raster = AppearanceRaster(w)
        self.rig = CameraRig(w)
        self.detector = detector or ClassicalVictimDetector()
        self._cam_every = max(1, int(round(scenario.rates.tick_hz / scenario.rates.fog_hz)))
        self._last_cam_pos = w.pos.copy()
        self._cam_never = np.ones(w.n, dtype=bool)
        self._rotor = w.chassis == CHASSIS_INDEX["rotor"]
        #: A rotor over unexplored ground lands and looks instead of flying on blind.
        self.rotors_land_to_look = True

        # Tier 3. `hivemind=False` is not a degraded mode, it is the control condition:
        # invariant #1 says the mission must complete without it, and the only way that
        # stays true is by running it that way regularly.
        self.hivemind = None
        if hivemind:
            self.hivemind = HivemindNode(
                w,
                build_ladder(w, self.tracker, allow_api=allow_api,
                             scripted_only=scripted_hivemind or response_team is not None),
                self.tracker,
            )

        self.response_team = None
        if response_team is not None:
            from .hivemind.team.controller import ResponseTeam

            self.response_team = ResponseTeam(w, self.bus, **response_team)

        #: The operator's voice channel (`voice/console.py`). Demo only, and additive:
        #: `None` is every headless run, every training rollout and the gate, so nothing
        #: here can move the seed-42 hash.
        self.voice = None
        if voice is not None:
            from .voice.console import OperatorConsole

            self.voice = OperatorConsole(w, voice)
            self.voice.start()

    @property
    def world(self):
        return self.sim.world

    def _perceive(self) -> None:
        """Render, look, detect, believe. 5 Hz with a half-cell movement gate.

        This replaced a one-line distance check against ground truth. Everything the
        swarm knows about casualties now originates here.
        """
        w = self.sim.world
        for i in w.reconnected:
            self.tracker.flush(w, i)
        w.reconnected.clear()

        alive = w.status <= OUT_OF_COMMS
        moved = np.abs(w.pos - self._last_cam_pos).max(axis=1) > w.cell * 0.5
        # A rotor in flight sees nothing. It is a spotter: it crosses ground at twice
        # the speed of anything else and pays for it by only looking once it lands, so
        # a rotor sprinting across the map lifts no fog on the way.
        idx = np.nonzero(alive & ~w.airborne & (moved | self._cam_never))[0]
        if len(idx):
            self._cam_never[idx] = False
            self._last_cam_pos[idx] = w.pos[idx]

            frames, cells = self.rig.capture(w, self.raster.render(w), idx)
            w.mark_seen(cells, idx)
        # Independent of the movement gate above: a robot that has stopped still knows
        # what it is standing on, and a rotor that has just set down has no trail.
        w.mark_underfoot(np.nonzero(alive & ~w.airborne)[0])
        if len(idx):
            self.tracker.ingest(w, self.detector.detect(w, self.rig, frames, idx))

        # A reconnect can supply evidence while every robot is stationary. The frame
        # movement gate must not suspend resolution, underfoot sensing or report expiry.
        for report, was_real, by in self.tracker.resolve(w):
            if was_real and report.victim is not None:
                w.mark_found(report.victim, by)
            elif not was_real:
                self._emit("report_dismissed",
                           f"{w.robot_ids[by]} found no casualty at {report.id}",
                           robot=w.robot_ids[by],
                           pos=(float(report.pos[0]), float(report.pos[1])))
        self.tracker.prune(w)

    def _emit(self, kind: str, text: str, **kw):
        return self.events.emit(kind, text, t=self.sim.world.t, **kw)

    def tick(self) -> None:
        w = self.sim.world
        if w.tick % self._cam_every == 0:
            self._perceive()
        if w.tick % self._hb_every == 0:
            self.allocator.heartbeat(w)
            # A loss forces an immediate auction rather than waiting for the next
            # scheduled cycle -- that wait is the difference between reallocating in
            # ~2 s and in exactly the 3 s the self-healing claim is measured against.
            if self.allocator.check_orphans(w, self.executor, self._emit):
                self._alloc_next = w.t
        if self.commander is not None:
            self.commander.step(w)
        self.fault.step(w, self.executor, self._emit)
        # Task bookkeeping runs at the auction rate (1 Hz), not per tick: completion is
        # a slow signal, and walking every robot in Python at 20 Hz is the one thing the
        # scaling rules forbid. Tier 1 still steers at the full 20 Hz.
        full = w.t >= self._alloc_next
        self.executor.update(w, self.nav, arrive_radius=self.reflex.p.arrive_radius, full=full)
        if full:
            self._alloc_next = w.t + self._alloc_period
            self.allocator.step(w, self.executor, self.tracker, emit=self._emit, bus=self.bus)
            if self.unit_policy is not None:
                self.unit_policy.step(self)
        # **Before the team, on purpose.** A spoken order is applied and its sectors
        # claimed in the same tick it lands, so when the team next plans it already sees
        # the operator's intent as a standing goal and their sectors as spoken for.
        if self.voice is not None:
            self.voice.step(w, self.executor, self.tracker, self.events,
                            self.hivemind, emit=self._emit, bus=self.bus)
            if self.response_team is not None and self.voice.goal:
                self.response_team.set_goal(self.voice.goal, self.voice.goal_seq)
        if self.response_team is not None:
            self.response_team.step(w, self.executor, self.tracker, self.bus,
                                    self.hivemind, self._emit)
        if self.hivemind is not None:
            self.hivemind.step(w, self.executor, self.tracker, self.events,
                               emit=self._emit, bus=self.bus)
        goal_xy, goal_id = self.executor.goals(w, self.nav)
        stop_r = self.executor.stop_radii(w, self.reflex.p.arrive_radius)
        arrived = self.reflex.arrived_at_goals(w, goal_xy, goal_id, stop_r)
        # Rotors fly while they have somewhere to be and land the moment they arrive --
        # **or the moment they are over ground nobody has seen.**
        #
        # Flight is blindness: `_perceive` excludes `airborne` outright, and a rotor
        # crosses anything (`World.step`: `ok |= airborne`). Landing only on arrival was
        # survivable while idle robots had no goal, because a rotor with nothing to do
        # sat down and looked. `idle_explore` gave every idle robot a goal, and the
        # rotor lane silently went blind with it: 103 of 512 robots are rotors, and they
        # spend a **mean 82.9% of their lives airborne, a median of 98.6%, with 45 of
        # them airborne for over 99%** -- 27% of that time directly above unexplored
        # ground (M-71). Every robot found standing on dark ground in contact was one of
        # these.
        #
        # So a rotor over dark ground sets down and looks. It keeps its 2x dash across
        # ground the swarm already knows, which is the point of the lane, and pays the
        # speed back exactly where the map still needs reading.
        can_land = w.can_land()
        # Reaching the horizontal arrival disc above a river is not a landing. Keep
        # approaching the task's dry target until the whole body can set down.
        arrived &= ~self._rotor | can_land
        air = self._rotor & (w.status <= OUT_OF_COMMS) & (
            ((goal_id >= 0) & ~arrived) | ~can_land)
        if self.rotors_land_to_look:
            ix, iy = grid.world_to_cell(w.pos[:, 0], w.pos[:, 1], w.cell, w.shape)
            seen_x, seen_y = grid.world_to_cell(
                self._last_cam_pos[:, 0], self._last_cam_pos[:, 1], w.cell, w.shape)
            inspected_here = ~self._cam_never & (ix == seen_x) & (iy == seen_y)
            # Never try to inspect by setting down in a river, wall or steep face.
            # Out-of-contact inspections are buffered, so shared fog alone would
            # strand a rotor even after its own camera has looked at this cell.
            air &= w.explored[iy, ix] | inspected_here | ~can_land
        # The operator's WASD override, demo only: the bridge owns it, and headless has no
        # bridge. A driven rotor's height is the operator's to choose and nobody else's:
        # the action key latches it airborne and sets it down again (`World.operator_act`),
        # and the landing rules above are where *autonomy* chooses to look, which would
        # otherwise leave a driven rotor unable to take off. It still cannot land where
        # autonomy could not -- over water it stays up.
        #
        # The latch replaced an implicit "airborne while a key is held". That version made
        # the drive keys a throttle and left the operator with no way to keep a drone up
        # while looking around it, which is the whole point of having one under the keys.
        manual = self.bridge.manual.command(w) if self.bridge is not None else None
        if manual is not None and self._rotor[manual[0]]:
            air[manual[0]] = w.operator_hover or not can_land[manual[0]]
        self._cam_never |= w.airborne & ~air
        w.airborne = air
        # Choose the flight layer BEFORE steering, including the first takeoff tick.
        v, omega = self.reflex.commands(w, self.nav, goal_xy, goal_id, stop_r, arrived,
                                        manual)
        if full:
            for i in np.flatnonzero(self.reflex.recovering):
                reason = self.executor.reason[i].removesuffix("; navigating around blocked terrain")
                self.executor.reason[i] = reason + "; navigating around blocked terrain"
        # A rotor taking a ground camera sample waits for the next sensor pass. It
        # must not crawl through rubble with its landing skids between flights.
        v = np.where(self._rotor & ~air, 0.0, v)
        self.sim.step(v, omega)
        self.events.drain_world(w)
        if w.tick % self._bb_every == 0:
            self.blackboard.publish(w, self.executor, self.tracker)
        if self.bridge is not None:
            self.bridge.step(w, self.executor, self.tracker)

    def scorecard(self, wall_seconds: float = 0.0) -> Scorecard:
        """The world's scorecard, completed with what only the Mission knows.

        `World.scorecard()` cannot fill in directive counts or wall-clock: Tier 3 and the
        clock live up here. **Any caller that prints the world's scorecard directly gets
        zeros in four fields**, which is what the `--demo` path did -- it reported
        "0 issued, 0 rejected" for a hivemind that was working, alongside the giveaway
        `0.00s / 0.00x`. A demo that under-reports its own headline feature is worse than
        one that over-reports it, because nobody thinks to check.
        """
        w = self.sim.world
        hm = self.hivemind.stats if self.hivemind else {}
        return dataclasses.replace(
            w.scorecard(),
            wall_seconds=round(wall_seconds, 3),
            rtf=round(w.t / wall_seconds, 2) if wall_seconds > 0 else 0.0,
            directives_issued=hm.get("issued", 0),
            directives_rejected=hm.get("rejected", 0),
        )

    def run(self, max_time: float | None = None) -> Scorecard:
        w = self.sim.world
        limit = max_time if max_time is not None else w.scn.mission_duration_s
        t0 = time.perf_counter()
        try:
            while not w.done and w.t < limit:
                self.tick()
            return self.scorecard(time.perf_counter() - t0)
        finally:
            self.close()

    def close(self):
        if self.response_team is not None:
            self.response_team.close()
        if self.voice is not None:
            self.voice.close()


def run_mission(scenario: Scenario, seed: int, *, max_time: float | None = None,
                hivemind: bool = False) -> Scorecard:
    """The rollout entry point for training and the gate.

    **Tier 3 is off by default here, and on by default in `Mission`.** That asymmetry is
    deliberate. MAP-Elites (D9) evolves Tier-2 behaviour parameters; if a scripted
    strategic layer were reprioritising sectors underneath it, the fitness signal would
    be measuring the two together and the archive would encode a dependency on whichever
    provider happened to answer. The gate (D12) compares learned against heuristic, and
    both arms have to differ in exactly one thing.

    The demo runs with the hivemind on -- that is `Mission`'s default and what the CLI
    uses. Evaluation runs without it unless asked.
    """
    return Mission(scenario, seed, hivemind=hivemind).run(max_time)
