"""The world: grid, sectors, victims, hazard, comms, and the robot state arrays.

One World class serves both SimBackends. FastSim and DemoSim differ only in how the
tick loop is clocked (docs/TECHNICAL.md section 1, invariant 1), which is what makes
policies trained in FastSim transfer to the demo for free.

Robot state is parallel numpy arrays. Nothing here may iterate robots in Python inside
the 20 Hz path -- the only per-robot Python loop is the 5 Hz sensor pass, and it runs
whole-array numpy operations per robot rather than per cell.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from ..contracts.schemas import OPERATOR_ACTION
from ..rng import RngBook
from . import grid
from . import terrain as terrain_gen
from .placement import cover_masks, select_sites
from .robot import (
    ACTIVE,
    AIRBORNE_SPEED,
    CHASSIS,
    CHASSIS_INDEX,
    CHASSIS_LIMITS,
    DESTROYED,
    LANE_INDEX,
    LANES,
    OPERATOR_SPEED,
    OUT_OF_COMMS,
    REVERSE_SPEED,
    RobotSpec,
    evolved_roster,
)
from .scenario import Scenario

# Victim state codes (kept as ints; there are only ~8 so Python-side logic is fine).
HIDDEN, FOUND, CLEARED, CARRIED, RESCUED = 0, 1, 2, 3, 4
VICTIM_STATE_NAME = {HIDDEN: "hidden", FOUND: "found", CLEARED: "cleared",
                     CARRIED: "carried", RESCUED: "rescued"}

# Interaction reach. These MUST exceed Tier 2's stop radius plus the goal-snapping
# error (one cell), or robots park just outside reach and never interact. Asserted in
# tests/test_skills.py::test_interaction_reach_exceeds_stop_radius.
#: A robot outside the comms component must come this fraction of the way inside the
#: nominal radius before it counts as reconnected. Pure hysteresis.
JOIN_MARGIN = 0.92

#: Speed multiplier while escaping ground the robot cannot properly traverse.
ESCAPE_SPEED_FACTOR = 0.3

#: Must match control.planner.NavFields' factor -- casualty placement checks
#: reachability on the same grid the flow fields are built from.
NAV_DOWNSAMPLE = 4

#: Endurance floor as a multiple of one mission of continuous movement. 1.15 rather than
#: 1.0 because a robot that finishes with exactly zero has spent the endgame unable to
#: take a detour round fire.
ENDURANCE_MARGIN = 1.15

#: Robots this close to base draw power from it.
RECHARGE_RADIUS = 14.0

#: Battery fraction restored per second on the pad. A full charge takes ~13 s, which is
#: long enough to read as a stop on the dashboard and short enough that a carrier
#: dropping a casualty at base tops up without abandoning the run.
RECHARGE_RATE = 0.075

#: Charge level below which a robot is worth announcing as recharging. Without a
#: threshold every carrier delivering to base emits an event, and the feed becomes
#: nothing else.
RECHARGE_EVENT_BELOW = 0.55

REACH_DIG = 2.5      # metres a scoop robot must be within to clear debris
#: Diggers that can usefully work one casualty at once. More bodies around a hole
#: do not make it deeper.
MAX_DIGGERS = 3
REACH_GRAB = 2.0     # metres a gripper robot must be within to pick up


@dataclass
class Victim:
    id: str
    pos: np.ndarray                # (2,) float64; follows its carrier while CARRIED
    buried: bool
    debris_remaining: float
    state: int = HIDDEN
    carrier: int = -1              # robot index, -1 = none
    found_t: float = -1.0
    rescued_t: float = -1.0


@dataclass
class Hazard:
    active: bool = False
    origin: np.ndarray = field(default_factory=lambda: np.zeros(2))
    centre: np.ndarray = field(default_factory=lambda: np.zeros(2))
    drift: np.ndarray = field(default_factory=lambda: np.zeros(2))
    radius: float = 0.0


class World:
    # ------------------------------------------------------------------ construction

    def __init__(self, scenario: Scenario, seed: int, *, evolved: bool = True) -> None:
        self.scn = scenario
        self.seed = int(seed)
        self.rng = RngBook(seed)
        self.t = 0.0
        self.tick = 0
        self.dt = scenario.dt
        self.events: list[dict[str, Any]] = []
        #: Casualties being actively excavated this tick, as `(x, y, diggers)`.
        #:
        #: Rebuilt by `_update_victims` from the predicate that actually moves debris,
        #: so it cannot drift from the simulation the way a separate distance test
        #: would. The bridge forwards it to the dashboard, which is the only consumer:
        #: digging is the one stage of the rescue chain with no motion to show for it,
        #: and a scoop robot clearing a slab looked identical to one sitting idle.
        #: Rebuilt rather than appended so a stale site cannot survive a tick.
        self.digging: list[tuple[float, float, int]] = []

        cell = scenario.map.cell
        self.cell = cell
        self.shape = scenario.grid_shape

        # --- terrain ---------------------------------------------------------------
        occ = grid.generate(
            self.rng["map"],
            width_m=scenario.map.width_m,
            height_m=scenario.map.height_m,
            cell=cell,
            n_clusters=scenario.map.n_clusters,
            cluster_radius_m=scenario.map.cluster_radius_m,
            rubble_fraction=scenario.map.rubble_fraction,
            keepout=scenario.keepouts(),
        )
        if scenario.terrain.reference is not None:
            from .landscape import occupancy

            occ = occupancy(self.rng["map"], scenario, scenario.terrain.reference)
        base_cell = grid.world_to_cell(*scenario.base, cell, self.shape)
        base_cell = (int(base_cell[0]), int(base_cell[1]))
        # Roads first, so the base and every collection point share a component before
        # anything unreachable gets walled off.
        self.road_points = [scenario.base, *scenario.extraction_zones]
        if scenario.terrain.reference is not None:
            from .landscape import road_network

            self.road_points, self.road_edges = road_network(scenario.terrain.reference)
        else:
            self.road_edges = grid.road_network(self.road_points)
        occ = grid.carve_roads(occ, cell, self.road_points, self.road_edges)
        self.occ = grid.ensure_connected(occ, base_cell)
        self.passable = self.occ != grid.WALL
        self.n_free = int(self.passable.sum())
        self.explored = np.zeros(self.shape, dtype=bool)

        # Landform, and the traversability it implies. `height` is what gets rendered;
        # `terrain` is the smooth landform slope is measured from -- taking the gradient
        # of `height` would read every one-metre rubble block as a cliff and make the
        # entire map impassable.
        # Attenuation is judged against the MOST capable chassis, so it only fires when
        # a peak walls the base off from everything -- not merely from wheeled units.
        self.height, self.terrain, self.water = terrain_gen.generate(
            self.rng["map"], self.occ, cell, scenario.terrain, scenario.keepouts(),
            base_cell=base_cell, passable=self.passable,
            max_slope=CHASSIS_LIMITS["legged"][0],
            roads=self.road_points, road_edges=self.road_edges,
        )
        self.slope = terrain_gen.slope_of(self.terrain, cell)
        self.chassis_passable = self._build_chassis_passability()
        #: Reachable by at least one kind of robot.
        self.any_passable = self.chassis_passable.any(axis=0)
        self._check_terrain_is_workable()

        # Geodesic distance from base, in metres. Used for victim placement weighting
        # and available to the auction as a cached field.
        self.dist_from_base = grid.distance_field(self.passable, base_cell) * cell

        # --- sectors ---------------------------------------------------------------
        self.sector_ids = scenario.sector_ids()
        self.sector_of_cell = self._build_sector_map()
        self.sector_free = np.bincount(
            self.sector_of_cell[self.passable], minlength=len(self.sector_ids)
        ).astype(np.float64)
        self.sector_free[self.sector_free == 0] = 1.0  # avoid /0 on a fully walled sector
        n_sec = len(self.sector_ids)
        self.sector_explored_pct = np.zeros(n_sec)
        self.sector_hazard_known = np.zeros(n_sec)
        self.sector_hazard_true = np.zeros(n_sec)
        self.sector_abandoned = np.zeros(n_sec, dtype=bool)
        #: Hazard cells the swarm has actually observed. This -- never the ground-truth
        #: hazard disc -- is what any swarm-side code may react to (CLAUDE.md #3).
        self.hazard_known = np.zeros(self.shape, dtype=bool)
        self.sector_priority = np.full(n_sec, 1, dtype=np.int8)  # 0 high, 1 normal, 2 low

        # --- victims ---------------------------------------------------------------
        self.victims = self._place_victims()

        # --- hazard ----------------------------------------------------------------
        self.hazard = self._init_hazard()

        # --- robots ----------------------------------------------------------------
        # An evolved roster if MAP-Elites has produced one, the hand-set archetypes
        # otherwise. The fallback is not a degraded mode -- `control/heuristic.py` and
        # its archetypes are the gate's baseline, and a run with no archive present must
        # behave exactly as it did before D9.
        self.specs: list[RobotSpec] = evolved_roster(
            scenario.robots_per_lane, self.rng["robots"],
            counts=scenario.lane_counts or None, use_evolved=evolved,
        )
        self._init_robot_arrays()

        # --- comms buffering (out-of-comms robots hold discoveries until reconnect) --
        self._pending_cells: dict[int, list[np.ndarray]] = {}
        #: Robots that regained contact this cycle; perception flushes their buffered
        #: sightings into the tracker.
        self.reconnected: list[int] = []

        self._update_comms()
        self._update_sectors()

    def _check_terrain_is_workable(self) -> None:
        """Fail only if the landform made the mission impossible for *every* robot.

        Checked against the most capable chassis, not the middle one. Ground that only
        legged units can reach is the whole point of the locomotion axis -- a map where
        every robot can go everywhere has no terrain in any meaningful sense. Demanding
        tracked-connectivity instead sent me eroding mountains flat to satisfy a
        requirement that does not exist, and took relief from 70 m to 17 m.
        """
        tracked = self.chassis_passable[CHASSIS_INDEX["legged"]]
        bx, by = grid.world_to_cell(
            np.asarray(self.scn.base[0]), np.asarray(self.scn.base[1]), self.cell, self.shape
        )
        if not tracked[int(by), int(bx)]:
            raise RuntimeError(
                f"base at {self.scn.base} is not traversable even by a legged robot "
                f"(slope {self.slope[int(by), int(bx)]:.2f}, "
                f"water {self.water[int(by), int(bx)]:.2f} m). Terrain keepouts failed."
            )
        reach = grid._flood(tracked, (int(bx), int(by)))
        frac = reach.sum() / max(1, tracked.sum())
        # Deliberately lenient. A landform that cuts part of the map off from base is a
        # legitimate scenario -- casualty placement already restricts itself to ground a
        # legged carrier can reach, and the auction routes around what it cannot. This
        # check exists to catch the pathological case where the base is sealed in, not
        # to demand an unobstructed map. A stricter bar sent me eroding mountains flat
        # to satisfy a requirement that was never real.
        if frac < 0.30:
            raise RuntimeError(
                f"only {frac:.0%} of legged-traversable ground connects to base -- the "
                f"landform has cut the map apart for every chassis. Ease "
                f"terrain.mountain_height_m or terrain.rivers."
            )
        self.reachable_frac = float(frac)

    def _build_chassis_passability(self) -> np.ndarray:
        """(len(CHASSIS), H, W) -- where each locomotion type can actually go.

        Terrain nothing is blocked by is scenery. A wheeled unit stops at a 30% grade and
        cannot enter water at all; a legged one walks up almost anything and wades nearly
        a metre. That difference is what turns a river into a routing problem.
        """
        out = np.zeros((len(CHASSIS), *self.shape), dtype=bool)
        for i, name in enumerate(CHASSIS):
            max_slope, max_wade, _ = CHASSIS_LIMITS[name]
            out[i] = self.passable & (self.slope <= max_slope) & (self.water <= max_wade)
        return out

    # ------------------------------------------------------------------ init helpers

    def _build_sector_map(self) -> np.ndarray:
        gx, gy = grid.cell_centres(self.shape, self.cell)
        sw = self.scn.map.width_m / self.scn.map.sector_cols
        sh = self.scn.map.height_m / self.scn.map.sector_rows
        col = np.clip((gx / sw).astype(np.int32), 0, self.scn.map.sector_cols - 1)
        row = np.clip((gy / sh).astype(np.int32), 0, self.scn.map.sector_rows - 1)
        return (row * self.scn.map.sector_cols + col).astype(np.int16)

    def _place_victims(self) -> list[Victim]:
        rng = self.rng["victims"]
        cfg = self.scn.victims
        # People lie on dry, workable ground, not in the water a legged robot can wade.
        reachable = self._navigable() & (self.water == 0)
        # Never place a victim inside an extraction disc -- it would be free.
        gx, gy = grid.cell_centres(self.shape, self.cell)
        for kx, ky, kr in self.scn.keepouts():
            reachable &= ((gx - kx) ** 2 + (gy - ky) ** 2) > (kr * 1.5) ** 2

        iy, ix = np.nonzero(reachable)
        xy = np.column_stack([gx[iy, ix], gy[iy, ix]])
        near_wall, near_rubble = cover_masks(self.occ, self.cell, cfg.cover_radius_m)
        d = self.dist_from_base[iy, ix]
        weight = np.power(np.maximum(d, 1.0), cfg.distance_weight_exp)

        # Weighted draw without replacement via the Gumbel top-k trick -- same
        # distribution as rng.choice(replace=False, p=...) but vectorised rather than
        # an O(n*k) Python loop over ~20k candidate cells.
        keys = np.log(weight) + rng.gumbel(size=len(ix))
        order = np.argsort(keys, kind="stable")[::-1]

        # Relax separation rather than failing outright. Terrain legitimately shrinks the
        # placeable region -- water, steep ground, and cells no carrier can reach are all
        # excluded -- and a scenario that raises on a tight map is brittle in exactly the
        # situation the terrain was added to create. Spacing is a preference; placing the
        # casualties is a requirement.
        chosen: list[int] = []
        for relax in (1.0, 0.75, 0.5, 0.3, 0.0):
            chosen = select_sites(
                xy, order, self.occ[iy, ix] == grid.RUBBLE,
                near_wall[iy, ix], near_rubble[iy, ix], count=cfg.count,
                buried_count=cfg.buried_count, cover_fraction=cfg.cover_fraction,
                separation=cfg.min_separation_m * relax,
            )
            if len(chosen) == cfg.count:
                self.victim_separation = cfg.min_separation_m * relax
                break
        if len(chosen) < cfg.count:
            raise RuntimeError(
                f"placed only {len(chosen)}/{cfg.count} casualties even with no minimum "
                f"separation: only {len(ix)} cells are reachable and workable. Reduce "
                f"victims.count, or ease map.n_clusters / terrain."
            )

        victims = []
        for n, c in enumerate(chosen):
            buried = n < cfg.buried_count
            victims.append(
                Victim(
                    id=f"v{n + 1}",
                    pos=np.array([gx[iy[c], ix[c]], gy[iy[c], ix[c]]], dtype=np.float64),
                    buried=buried,
                    debris_remaining=1.0 if buried else 0.0,
                )
            )
        return victims

    def _navigable(self) -> np.ndarray:
        """Cells a carrier can actually get to and work at.

        `passable & finite dist_from_base` is not enough, and placing casualties on that
        basis strands some of them permanently. Two things it misses:

        1. **Robots navigate on the coarse grid**, not this one. `NavFields` downsamples
           4x with majority-passable, so a one-cell pocket that is reachable here has no
           route on the grid the flow fields are actually built from.
        2. **Bodies have width.** A casualty in a narrow gap is `passable` but a carrier
           cannot approach within REACH_GRAB, so it is seen, dispatched to, and never
           collected -- the worst failure mode, because it looks like the swarm working.

        Enforced by tests/test_world.py::test_every_casualty_is_reachable_and_workable.
        """
        # Clearance from walls, plus fine-grid access for the most capable carrier.
        clear = grid.clearance_mask(self.occ, cells=1) & (
            self.water <= CHASSIS_LIMITS["legged"][1] * 0.5
        )
        legged = self.chassis_passable[CHASSIS_INDEX["legged"]]
        base = grid.world_to_cell(*self.scn.base, self.cell, self.shape)
        clear &= grid._flood(legged, (int(base[0]), int(base[1])))

        # Coarse-grid reachability, matching control.planner.NavFields exactly.
        factor = NAV_DOWNSAMPLE
        # Legged: carriers are mixed across all three chassis, so a casualty is
        # collectable as long as the most capable of them can get there. Requiring
        # tracked access would delete every casualty behind a ridge -- and those are
        # exactly the ones that make the locomotion axis worth having.
        coarse = grid.downsample(legged, factor)
        bx, by = grid.world_to_cell(
            np.asarray(self.scn.base[0]), np.asarray(self.scn.base[1]),
            self.cell * factor, coarse.shape,
        )
        if not coarse[int(by), int(bx)]:
            piy, pix = np.nonzero(coarse)
            k = int(np.argmin((pix - int(bx)) ** 2 + (piy - int(by)) ** 2))
            bx, by = pix[k], piy[k]
        # Match NavFields' actual edges: adjacent coarse cells alone can invent a
        # route through rock. Concealed sites must still have a real approach.
        edges = grid.coarse_edge_masks(legged, factor)
        creach = np.isfinite(grid.distance_field(coarse, (int(bx), int(by)), edges))

        h, w = self.shape
        fine = np.repeat(np.repeat(creach, factor, axis=0), factor, axis=1)[:h, :w]
        return clear & fine

    def _init_hazard(self) -> Hazard:
        rng = self.rng["hazard"]
        x0, y0, x1, y1 = self.scn.sector_rect(self.scn.hazard.origin_sector)
        gx, gy = grid.cell_centres(self.shape, self.cell)
        inside = self.passable & (gx >= x0) & (gx < x1) & (gy >= y0) & (gy < y1)
        iy, ix = np.nonzero(inside)
        k = int(rng.integers(len(ix)))
        origin = np.array([gx[iy[k], ix[k]], gy[iy[k], ix[k]]], dtype=np.float64)
        ang = float(rng.uniform(0, 2 * np.pi))
        return Hazard(
            active=False,
            origin=origin,
            centre=origin.copy(),
            drift=np.array([np.cos(ang), np.sin(ang)]) * self.scn.hazard.drift_speed,
            radius=0.0,
        )

    def _init_robot_arrays(self) -> None:
        rng = self.rng["robots"]
        n = len(self.specs)
        self.n = n
        self.robot_ids = [s.id for s in self.specs]
        self.actuator = np.array([LANE_INDEX[s.actuator] for s in self.specs], dtype=np.int8)
        self.radius = np.array([s.radius for s in self.specs])
        self.v_max = np.array([s.v_max for s in self.specs]) * self.scn.robot_speed_multiplier
        self.omega_max = np.array([s.omega_max for s in self.specs])
        self.sensor_radius = np.array([s.sensor_radius for s in self.specs])
        self.chassis = np.array([CHASSIS_INDEX[s.chassis] for s in self.specs], dtype=np.int8)
        # Endurance floor, derived from the scenario rather than chosen.
        #
        # `battery_capacity` is an evolved trait, and MAP-Elites had no reason to protect
        # it: nothing in the fitness function knows that a carrier is what converts a
        # find into a rescue. The shipped roster gave the carrier lane the worst
        # endurance in the swarm -- 24 of 96 could not survive 420 s of continuous
        # movement, worst case flat at t=333, against zero robots under-mission in every
        # other lane. A body that cannot finish the mission it was bred for is a broken
        # robot, not a specialised one; the same argument the wheeled slope limit already
        # makes in CHASSIS_LIMITS.
        #
        # The floor is a full mission of movement plus a margin for hazard exposure, so
        # it moves with `mission_duration_s` instead of going stale when the clock does.
        raw = np.array([s.battery_capacity for s in self.specs])
        floor = ((self.scn.battery.idle + self.scn.battery.moving)
                 * self.scn.mission_duration_s * ENDURANCE_MARGIN)
        self.battery_cap = np.maximum(raw, floor)

        self.pos = self._spawn_positions(n, rng)  # needs self.chassis, set above
        self.theta = rng.uniform(0, 2 * np.pi, n)
        self.battery = np.ones(n)
        self.status = np.full(n, ACTIVE, dtype=np.int8)
        self.carrying = np.full(n, -1, dtype=np.int32)
        #: Per-robot multiplier on the commanded speed ceiling. Ones for the swarm; the
        #: operator's override raises it for the one robot it holds (`set_operator`).
        #: Nothing sets it without a dashboard, so headless clips exactly as before.
        self.speed_boost = np.ones(n)
        #: The robot the dashboard's operator is driving, or -1. See `set_operator`.
        self.operator = -1
        #: A driven rotor the operator has told to hold its height. Read by `Mission`,
        #: which owns the airborne decision; toggled by the action key.
        self.operator_hover = False
        #: Robots currently on the pad, so the recharge event fires once per visit.
        self._charging = np.zeros(n, dtype=bool)
        self.in_comms = np.ones(n, dtype=bool)
        self.comms_via_relay = np.zeros(n, dtype=bool)
        #: In flight. Rotors only; set each tick by `Mission` from whether the robot has
        #: reached its goal. Airborne robots cross terrain freely at double speed and
        #: reveal nothing -- they are spotters that have to land to look.
        self.airborne = np.zeros(n, dtype=bool)
        self.energy_used = np.zeros(n)
        self._last_fog_pos = self.pos.copy()
        self._fog_never_run = np.ones(n, dtype=bool)

    def _spawn_positions(self, n: int, rng: np.random.Generator) -> np.ndarray:
        """Spawn every robot on ground its own chassis can traverse.

        Sampling a polar ring around base is wrong: at base (6, 10) a 7.2 m ring reaches
        x = -1.2, outside the map and inside the border wall. Sampling cells fixes that.

        Sampling cells that are merely *wall-free* is also wrong, and worse: a wheeled
        robot spawned in marsh can never move. Its own position is impassable, so the
        safety override zeroes its speed every tick forever. Measured before this fix:
        52.5% of robot-ticks had a goal and zero commanded speed, and the swarm covered
        82 m of a possible 630.
        """
        gx, gy = grid.cell_centres(self.shape, self.cell)
        clear = grid.clearance_mask(self.occ, cells=1)
        base = np.array(self.scn.base)
        d2 = (gx - base[0]) ** 2 + (gy - base[1]) ** 2

        # Size the staging area from the swarm, not from the extraction zone.
        #
        # The original radius packed 768 robots into ~22 m and the swarm gridlocked at
        # the start line (M-23). The fix was right in intent and wrong in arithmetic:
        # the comment claimed "roughly two separation discs of room" but `2.0 * sep**2`
        # is 12.5 m^2, and a separation *disc* is `pi * sep**2` = 19.6 m^2 -- so it
        # allocated 0.64 of one disc, not two. Worse, the loop stopped growing as soon as
        # `2 * n` clear cells existed, which on a 1 m grid is **2 m^2 per robot**, so the
        # area term almost never bound at all.
        #
        # Measured consequence on the 768-robot map: 326 robots still within 45 m of base
        # at t=120, ~19.5 m^2 each, half the swarm moving less than 2 m per 30 s while
        # holding assignments (M-47).
        #
        # Both criteria are now the same quantity, in m^2 per robot, so they cannot drift
        # apart again. The base sits in a map corner, so roughly a quarter of any disc
        # around it is on-map -- the loop grows the radius until enough *clear* cells
        # actually exist, rather than assuming the disc is usable.
        from ..control.planner import UNREACHABLE, NavFields
        from ..control.tier1_reflex import ReflexParams
        from ..control.zone_routing import erode8

        sep = ReflexParams().separation_radius
        want_area = n * np.pi * (sep ** 2)          # one separation disc each
        need_cells = int(np.ceil(want_area / (self.cell ** 2)))
        radius = max(self.scn.extraction_radius * 1.8, float(np.sqrt(want_area / np.pi)))
        for _ in range(8):
            iy, ix = np.nonzero(clear & (d2 <= radius * radius))
            if len(ix) >= need_cells:
                break
            radius *= 1.5
        else:
            raise RuntimeError(
                f"no room to spawn {n} robots near base {tuple(base)}; "
                f"reduce robots_per_lane or lower map.n_clusters"
            )

        pos = np.zeros((n, 2), dtype=np.float64)
        for c in range(self.chassis_passable.shape[0]):
            who = np.nonzero(self.chassis == c)[0]
            if len(who) == 0:
                continue
            ok = clear & (d2 <= radius * radius) & self.chassis_passable[c]
            cy, cx = np.nonzero(ok)
            grow = radius
            while len(cx) < len(who) and grow < max(self.scn.map.width_m,
                                                    self.scn.map.height_m):
                grow *= 1.5
                cy, cx = np.nonzero(clear & (d2 <= grow * grow) & self.chassis_passable[c])
            if len(cx) < len(who):
                raise RuntimeError(
                    f"no room to spawn {len(who)} robots of chassis {c} near base; "
                    f"terrain around the staging area is impassable to them"
                )
            sel = rng.choice(len(cx), size=len(who), replace=False)
            pos[who] = np.stack([gx[cy[sel], cx[sel]], gy[cy[sel], cx[sel]]], axis=1)
        # Sub-cell jitter so robots are not perfectly grid-aligned, bounded to stay
        # inside the cell that was verified clear.
        jitter = rng.uniform(-0.15, 0.15, (n, 2)) * self.cell
        # Repair only invalid starts, preserving valid placements and the RNG stream.
        # Fine passability alone admits isolated coarse pockets and bodies straddling
        # a bank. Ground units need the auction's route to base; aircraft need a pad.
        pads = []
        for c in range(self.chassis_passable.shape[0]):
            available = clear & erode8(self.chassis_passable[c]) & (self.water == 0)
            if c != CHASSIS_INDEX["rotor"]:
                nav = NavFields(self.chassis_passable[c], self.cell, NAV_DOWNSAMPLE)
                reachable = nav.field(*self.scn.base) < UNREACHABLE
                reachable = np.repeat(np.repeat(reachable, NAV_DOWNSAMPLE, axis=0),
                                      NAV_DOWNSAMPLE, axis=1)[:self.shape[0], :self.shape[1]]
                available &= reachable
            pads.append(available)
        ix, iy = grid.world_to_cell(*pos.T, self.cell, self.shape)
        valid = np.asarray(pads)[self.chassis, iy, ix]
        # Reserve valid original cells before relocating anyone. Shared initial cells
        # keep the first robot by id; later robots receive their own landing space.
        first = np.unique(iy * self.shape[1] + ix, return_index=True)[1]
        unique = np.zeros(n, dtype=bool)
        unique[first] = True
        valid &= unique
        occupied = np.zeros(self.shape, dtype=bool)
        occupied[iy[valid], ix[valid]] = True
        for i in np.flatnonzero(~valid):
            cy, cx = np.nonzero(pads[int(self.chassis[i])] & ~occupied)
            if not len(cx):
                raise RuntimeError(f"no reachable dry spawn for {self.robot_ids[i]}")
            k = np.argmin((gx[cy, cx] - pos[i, 0])**2 + (gy[cy, cx] - pos[i, 1])**2)
            pos[i] = (gx[cy[k], cx[k]], gy[cy[k], cx[k]])
            occupied[cy[k], cx[k]] = True
        return pos + jitter

    # ------------------------------------------------------------------ the tick

    def step(self, v_cmd: np.ndarray, omega_cmd: np.ndarray) -> None:
        """Advance one tick. ``v_cmd``/``omega_cmd`` are (N,) arrays from Tier 1."""
        dt = self.dt
        self.tick += 1
        self.t = self.tick / self.scn.rates.tick_hz
        alive = self.status <= OUT_OF_COMMS

        # --- heading -----------------------------------------------------------
        self.airborne &= alive & (self.chassis == CHASSIS_INDEX["rotor"])
        omega = np.clip(omega_cmd, -self.omega_max, self.omega_max) * alive
        self.theta = np.mod(self.theta + omega * dt, 2 * np.pi)

        # --- speed, with terrain and payload penalties --------------------------
        ix, iy = grid.world_to_cell(self.pos[:, 0], self.pos[:, 1], self.cell, self.shape)
        terrain = np.where(self.occ[iy, ix] == grid.RUBBLE, grid.RUBBLE_SPEED_FACTOR, 1.0)
        payload = np.where(self.carrying >= 0, self.scn.victims.carry_speed_factor, 1.0)
        # A rotor in flight is over the terrain, not on it: no rubble drag, and twice
        # the speed. It pays for that by being blind -- see `Mission._perceive`.
        terrain = np.where(self.airborne, 1.0, terrain)
        # Reverse exists for the operator's manual override alone; nothing autonomous
        # commands a negative speed, so for the swarm this is the old [0, v_max] clip.
        # The ceiling is per-robot because the operator's unit is allowed past its own
        # rating (robot.py `OPERATOR_SPEED`). `speed_boost` is all ones everywhere else,
        # so this is the old clip bit for bit for every autonomous robot.
        cap = self.v_max * self.speed_boost
        v = np.clip(v_cmd, -REVERSE_SPEED * cap, cap) * terrain * payload * alive
        v = np.where(self.airborne, v * AIRBORNE_SPEED, v)

        # --- translation, axis-separated so robots slide along walls -------------
        #
        # A robot whose *current* cell is impassable to it is stranded: the hazard grew
        # over it, or a neighbour shoved it into marsh. Ordinary collision blocks every
        # move for such a robot -- its own centre sample already fails -- so the shove is
        # permanent and it sits there for the rest of the mission. Stranded robots may
        # move regardless, at reduced speed: a machine struggling out of mud, not one
        # teleporting across a river.
        cix, ciy = grid.world_to_cell(self.pos[:, 0], self.pos[:, 1], self.cell, self.shape)
        stranded = ~self.chassis_passable[self.chassis, ciy, cix] & alive & ~self.airborne
        v = np.where(stranded, v * ESCAPE_SPEED_FACTOR, v)

        dx = v * np.cos(self.theta) * dt
        dy = v * np.sin(self.theta) * dt
        nx = self.pos[:, 0] + dx
        blocked = self._collides(nx, self.pos[:, 1]) & ~stranded
        self.pos[:, 0] = np.where(blocked, self.pos[:, 0], nx)
        ny = self.pos[:, 1] + dy
        blocked = self._collides(self.pos[:, 0], ny) & ~stranded
        self.pos[:, 1] = np.where(blocked, self.pos[:, 1], ny)
        self.stranded = int(stranded.sum())

        # --- hazard --------------------------------------------------------------
        self._update_hazard()
        in_hazard = np.zeros(self.n, dtype=bool)
        if self.hazard.active:
            d2 = ((self.pos - self.hazard.centre) ** 2).sum(axis=1)
            in_hazard = alive & (d2 <= self.hazard.radius ** 2)

        # --- battery -------------------------------------------------------------
        drain = (
            self.scn.battery.idle
            + self.scn.battery.moving * (np.abs(v) / np.maximum(self.v_max, 1e-9))
            + self.scn.hazard.battery_drain * in_hazard
        ) * dt * alive
        drain = drain / np.maximum(self.battery_cap, 1e-9)
        self.battery = np.maximum(self.battery - drain, 0.0)
        self.energy_used += drain

        # --- recharge at base ----------------------------------------------------
        #
        # Base is a place the swarm already goes -- it is collection point one -- so this
        # costs a carrier nothing it was not already spending. It is deliberately *not*
        # offered at the other ten collection points: a pad everywhere is a pad nowhere,
        # and the walk home is what makes charging a decision rather than scenery.
        pad = np.asarray(self.scn.base, dtype=float)
        near = alive & ~self.airborne & (((self.pos - pad) ** 2).sum(axis=1) <= RECHARGE_RADIUS ** 2)
        if near.any():
            was = self.battery.copy()
            self.battery[near] = np.minimum(self.battery[near] + RECHARGE_RATE * dt, 1.0)
            # Edge-triggered, and only for robots that actually needed it: the event
            # drives the dashboard's charge burst, and one per robot per visit is the
            # difference between an effect and a strobe.
            started = near & (was < RECHARGE_EVENT_BELOW) & ~self._charging
            for i in np.nonzero(started)[0]:
                self._emit("robot_recharged",
                            f"{self.robot_ids[i]} recharging at base "
                            f"({100 * was[i]:.0f}%)",
                            robot=self.robot_ids[i], pos=tuple(self.pos[i]))
            self._charging = near & (self.battery < 1.0)
        elif self._charging.any():
            self._charging = np.zeros(self.n, dtype=bool)

        # --- deaths --------------------------------------------------------------
        newly_dead = alive & (self.battery <= 0.0)
        if newly_dead.any():
            for i in np.nonzero(newly_dead)[0]:
                self._kill(int(i), "hazard" if in_hazard[i] else "battery")

        # --- decimated passes ----------------------------------------------------
        self._update_victims()
        if self._due(self.scn.rates.comms_hz):
            self._update_comms()
        if self._due(self.scn.rates.sector_hz):
            self._update_sectors()

    def _due(self, hz: float) -> bool:
        every = max(1, int(round(self.scn.rates.tick_hz / hz)))
        return self.tick % every == 0

    def can_land(self) -> np.ndarray:
        """Dry, traversable support for the entire body, even while in flight."""
        return ~self._collides(self.pos[:, 0], self.pos[:, 1], allow_airborne=False)

    def _collides(self, x: np.ndarray, y: np.ndarray, *, allow_airborne=True) -> np.ndarray:
        """True where a robot cannot stand: a wall, too steep for it, or too deep.

        Per-robot, keyed on chassis -- a slope a legged unit walks up is a wall to a
        wheeled one, and the whole point of the locomotion axis is that they differ.
        """
        r = self.radius[:, None]
        ox = np.array([0.0, 1.0, -1.0, 0.0, 0.0])
        oy = np.array([0.0, 0.0, 0.0, 1.0, -1.0])
        px = x[:, None] + r * ox
        py = y[:, None] + r * oy
        ix, iy = grid.world_to_cell(px, py, self.cell, self.shape)
        ok = self.chassis_passable[self.chassis[:, None], iy, ix]
        # Flight clears everything underneath it.
        if allow_airborne:
            ok |= self.airborne[:, None]
        ok &= ((px >= 0) & (py >= 0)
               & (px <= self.scn.map.width_m) & (py <= self.scn.map.height_m))
        return ~ok.all(axis=1)

    # ------------------------------------------------------------------ subsystems

    def _update_hazard(self) -> None:
        h = self.scn.hazard
        if self.t < h.ignite_t:
            return
        if not self.hazard.active:
            self.hazard.active = True
            self._emit("hazard_ignited", f"hazard ignited near {self._sector_at(self.hazard.origin)}",
                       sector=self._sector_at(self.hazard.origin),
                       pos=tuple(self.hazard.origin))
        age = self.t - h.ignite_t
        # Grow, then burn out. The centre keeps drifting either way -- a fire that is
        # going out is still moving downwind, and stopping the drift at the peak would
        # make the recession look like the disc simply deflating in place.
        if h.peak_t is not None and self.t > h.peak_t:
            peak_r = h.radius0 + h.growth_rate * (h.peak_t - h.ignite_t)
            self.hazard.radius = max(0.0, peak_r - h.decay_rate * (self.t - h.peak_t))
        else:
            self.hazard.radius = h.radius0 + h.growth_rate * age
        self.hazard.centre = self.hazard.origin + self.hazard.drift * age

    def _update_comms(self) -> None:
        """Rebuild the connectivity component rooted at base; flush reconnect buffers.

        Antenna robots relay. A robot outside the component cannot report discoveries,
        which is what makes the relay lane load-bearing rather than decorative.
        """
        alive = self.status <= OUT_OF_COMMS
        base = np.array(self.scn.base)
        br, rr = self.scn.comms.base_radius, self.scn.comms.relay_radius

        is_relay = alive & (self.actuator == LANE_INDEX["antenna"])
        relay_idx = np.nonzero(is_relay)[0]
        connected_relays = np.zeros(self.n, dtype=bool)

        if len(relay_idx):
            rp = self.pos[relay_idx]
            seed = ((rp - base) ** 2).sum(axis=1) <= br * br
            # Pairwise relay-relay adjacency, then a small BFS over the relay graph.
            d2 = ((rp[:, None, :] - rp[None, :, :]) ** 2).sum(axis=2)
            adj = d2 <= rr * rr
            reach = seed.copy()
            while True:
                grown = reach | (adj[:, reach].any(axis=1))
                if grown.sum() == reach.sum():
                    break
                reach = grown
            connected_relays[relay_idx[reach]] = True

        was = self.in_comms.copy()
        # Hysteresis. A robot sitting exactly on the range boundary otherwise flaps in
        # and out every cycle: 609 connect/disconnect events in one test mission, each
        # one orphaning the robot's task and re-auctioning it. Joining the component
        # takes a firmer signal than staying in it.
        scale = np.where(was, 1.0, JOIN_MARGIN)
        near_base = ((self.pos - base) ** 2).sum(axis=1) <= (br * scale) ** 2
        near_relay = np.zeros(self.n, dtype=bool)
        cr_idx = np.nonzero(connected_relays)[0]
        if len(cr_idx):
            d2 = ((self.pos[:, None, :] - self.pos[None, cr_idx, :]) ** 2).sum(axis=2)
            near_relay = (d2 <= (rr * scale[:, None]) ** 2).any(axis=1)

        self.in_comms = alive & (near_base | near_relay | connected_relays)
        #: Robots that are connected *only* because a relay reached them. This is the
        #: relay lane's actual contribution, and it is exact rather than a proxy: it is
        #: the set that would drop out if the relays were removed. Evolution scores the
        #: antenna lane on it (`training/mapelites/evaluate.py`), which is the difference
        #: between rewarding a relay for holding a bridge and rewarding it for standing
        #: next to the base while everyone else does the work.
        self.comms_via_relay = alive & ~near_base & (near_relay | connected_relays)

        for i in np.nonzero(alive & was & ~self.in_comms)[0]:
            self.status[i] = OUT_OF_COMMS
            self._emit("robot_out_of_comms", f"{self.robot_ids[i]} lost contact",
                       robot=self.robot_ids[i], pos=tuple(self.pos[i]))
        for i in np.nonzero(alive & ~was & self.in_comms)[0]:
            self.status[i] = ACTIVE
            self._flush_buffers(int(i))

        self._store_and_forward(alive)

    def _store_and_forward(self, alive: np.ndarray) -> None:
        """An out-of-contact robot hands its findings to any in-contact robot it passes.

        Without this a scout that goes dark and never comes back reports nothing it saw:
        buffers only emptied on *reconnect*, and on a 480x320 m map most scouts that leave
        the comms envelope do not return before the mission ends. Every search improvement
        died against that -- spreading the swarm out covered more ground and registered
        less of it (MEASUREMENTS.md M-36).

        This is delay-tolerant networking, and it is how real mesh systems behave: data
        walks home on whatever is going that way. It deliberately does **not** put the
        carrier back in the live comms component -- the robot still cannot be given orders
        or bid on work, so the relay lane keeps its job. Only what it has already seen
        gets through.
        """
        holders = [i for i in self._pending_cells if alive[i] and not self.in_comms[i]]
        if not holders:
            return
        linked = np.nonzero(alive & self.in_comms)[0]
        if len(linked) == 0:
            return
        rr = self.scn.comms.relay_radius
        hp = self.pos[holders]
        d2 = ((hp[:, None, :] - self.pos[None, linked, :]) ** 2).sum(axis=2)
        for j, i in enumerate(holders):
            if d2[j].min() <= rr * rr:
                cells = self._pending_cells.pop(i, [])
                if cells:
                    arr = np.concatenate(cells, axis=0)
                    self.explored[arr[:, 0], arr[:, 1]] = True
                self.reconnected.append(i)

    def _flush_buffers(self, i: int) -> None:
        """A reconnecting robot's buffered discoveries all land at once."""
        cells = self._pending_cells.pop(i, [])
        if cells:
            # Each entry is a (K, 2) array of (iy, ix) -- concatenate and scatter once.
            arr = np.concatenate(cells, axis=0)
            self.explored[arr[:, 0], arr[:, 1]] = True
        self.reconnected.append(i)
        text = f"{self.robot_ids[i]} back in contact"
        if cells:
            text += f", syncing {sum(len(c) for c in cells)} observed cells"
        self._emit("robot_reconnected", text, robot=self.robot_ids[i], pos=tuple(self.pos[i]))

    def buffered_count(self, i: int) -> int:
        """Cells robot ``i`` has observed and not yet been able to report.

        Information the swarm does not have yet. A robot far outside the component can
        accumulate thousands of these and, if it never regains contact, every one of them
        is lost at the buzzer -- which is how six sectors finished a run at 0% explored
        with robots standing in them (M-53).
        """
        return sum(len(c) for c in self._pending_cells.get(i, ()))

    def mark_seen(self, cells: np.ndarray, idx: np.ndarray) -> None:
        """Record what the cameras actually observed. Called by the perception pass.

        ``cells`` is (K, H, W, 2) of (iy, ix) with -1 where the pixel was occluded --
        the output of ``CameraRig.capture``. Explored space is therefore literally what
        was seen through a lens, not a geometric disc drawn around each robot.

        Out-of-comms robots reveal into their own buffer instead of the shared map: an
        unreachable robot cannot tell anyone what it saw.
        """
        if len(idx) == 0:
            return
        ok = cells[..., 0] >= 0
        in_c = self.in_comms[idx]

        shared = ok & in_c[:, None, None]
        if shared.any():
            sel = cells[shared]
            self.explored[sel[:, 0], sel[:, 1]] = True

        for j in np.nonzero(~in_c)[0]:
            mj = ok[j]
            if mj.any():
                self._pending_cells.setdefault(int(idx[j]), []).append(cells[j][mj])

    def mark_underfoot(self, idx: np.ndarray) -> None:
        """The cell a robot is standing on, which its own camera can never show it.

        `CameraRig` has a near clip of `2.2 x world.radius.max()` -- 1.03 m against a
        1.0 m cell -- so **a robot's own cell is never inside its own frustum**
        (measured: 0 of 512 self-hits on the demo map). A walking robot's trail is
        covered anyway, because it saw the cell from a metre back on the way in. What is
        not covered is anything that arrives *without* a ground approach: a rotor setting
        down, or a robot at spawn. Those sit on ground they have never revealed, in
        contact, in plain sight of the dashboard -- 746 cells of the demo map per mission.

        This is not a perception shortcut and it does not weaken invariant #3. A camera
        is how a robot learns about ground it is *not* on; standing on a cell is direct
        physical contact with it. Out-of-comms robots still buffer, exactly as `mark_seen`
        does -- an unreachable robot cannot tell anyone where it has been either.
        """
        if len(idx) == 0:
            return
        ix, iy = grid.world_to_cell(self.pos[idx, 0], self.pos[idx, 1],
                                    self.cell, self.shape)
        in_c = self.in_comms[idx]
        if in_c.any():
            self.explored[iy[in_c], ix[in_c]] = True
        for j in np.nonzero(~in_c)[0]:
            self._pending_cells.setdefault(int(idx[j]), []).append(
                np.array([[iy[j], ix[j]]], dtype=np.int32))

    def _update_sectors(self) -> None:
        ns = len(self.sector_ids)
        sec = self.sector_of_cell
        expl = self.passable & self.explored
        self.sector_explored_pct = (
            np.bincount(sec[expl], minlength=ns).astype(np.float64) / self.sector_free
        )
        if self.hazard.active:
            gx, gy = grid.cell_centres(self.shape, self.cell)
            haz = self.passable & (
                ((gx - self.hazard.centre[0]) ** 2 + (gy - self.hazard.centre[1]) ** 2)
                <= self.hazard.radius ** 2
            )
            self.sector_hazard_true = (
                np.bincount(sec[haz], minlength=ns).astype(np.float64) / self.sector_free
            )
            # What the swarm knows: hazard only in cells it has actually observed.
            self.hazard_known = haz & self.explored
            self.sector_hazard_known = (
                np.bincount(sec[self.hazard_known], minlength=ns).astype(np.float64)
                / self.sector_free
            )

    def _update_victims(self) -> None:
        alive = self.status <= OUT_OF_COMMS
        self.digging = []
        for vi, v in enumerate(self.victims):
            if v.state == RESCUED:
                continue

            if v.state == HIDDEN:
                # Discovery is NOT decided here. A victim becomes found when the
                # swarm's own detector reports one and a robot gets close enough to
                # resolve it -- see perception/tracker.py and Mission._perceive. This
                # branch used to be `dist <= sensor_radius` against ground truth, which
                # meant the robot was simply told.
                continue

            if v.state == FOUND:
                if not v.buried or v.debris_remaining <= 0.0:
                    v.state = CLEARED
                    continue
                d2 = ((self.pos - v.pos) ** 2).sum(axis=1)
                diggers = (alive & ~self.airborne & (self.actuator == LANE_INDEX["scoop"])
                           & (d2 <= REACH_DIG ** 2))
                if diggers.any():
                    # Only so many machines fit around one casualty. Without a cap the
                    # rate scaled with however many diggers happened to be nearby, so a
                    # crowd of eight cleared a buried casualty in half a second and
                    # digging stopped being a stage of the rescue chain at all.
                    n = min(int(diggers.sum()), MAX_DIGGERS)
                    v.debris_remaining -= self.scn.victims.clear_rate * self.dt * n
                    self.digging.append((float(v.pos[0]), float(v.pos[1]), n))
                    if v.debris_remaining <= 0.0:
                        v.debris_remaining = 0.0
                        v.state = CLEARED
                        self._emit("victim_cleared", f"{v.id} dug out in {self._sector_at(v.pos)}",
                                   victim=v.id, sector=self._sector_at(v.pos), pos=tuple(v.pos))
                continue

            if v.state == CLEARED:
                d2 = ((self.pos - v.pos) ** 2).sum(axis=1)
                free_grippers = (
                    alive & (self.actuator == LANE_INDEX["gripper"])
                    & (self.carrying < 0) & (d2 <= REACH_GRAB ** 2)
                )
                # The unit the operator is driving picks up when they press the key, not
                # by walking past. Not a flourish: `operator_act` lets them set a
                # casualty down, and a robot that re-grabbed it on the next tick would
                # make that key look broken. Digging has no such conflict and is left
                # automatic. Autonomy resumes the moment the lease lapses.
                if self.operator >= 0:
                    free_grippers[self.operator] = False
                if free_grippers.any():
                    i = int(np.argmin(np.where(free_grippers, d2, np.inf)))
                    v.state, v.carrier = CARRIED, i
                    self.carrying[i] = vi
                continue

            if v.state == CARRIED:
                i = v.carrier
                if self.status[i] >= DESTROYED:      # carrier died: victim is dropped
                    v.state, v.carrier = CLEARED, -1
                    continue
                v.pos = self.pos[i].copy()
                for zx, zy in self.scn.extraction_zones + [self.scn.base]:
                    if (v.pos[0] - zx) ** 2 + (v.pos[1] - zy) ** 2 <= self.scn.extraction_radius ** 2:
                        v.state, v.rescued_t = RESCUED, self.t
                        self.carrying[i] = -1
                        v.carrier = -1
                        self._emit("victim_rescued",
                                   f"{v.id} extracted by {self.robot_ids[i]}",
                                   victim=v.id, robot=self.robot_ids[i], pos=(zx, zy))
                        break

    # ------------------------------------------------------------------ the operator

    def set_operator(self, i: int) -> None:
        """Hand the operator's lease to robot ``i``, or -1 for nobody.

        `control.manual.ManualOverride` owns the lease and calls this once a tick. The
        world has to hold it because two things *inside* the tick turn on it -- the speed
        ceiling in `step` and the automatic pickup in `_update_victims` -- and neither can
        reach up into the bridge to ask.

        Nothing calls this without a dashboard attached, so `speed_boost` stays all ones
        and both branches are dead code in headless, the gate and every rollout.
        """
        if i == self.operator:
            return
        if self.operator >= 0:
            self.speed_boost[self.operator] = 1.0
        if i >= 0:
            self.speed_boost[i] = OPERATOR_SPEED
        # A new unit starts under its own lane's rules, not the last one's: letting the
        # hover latch carry over would hand the next robot a state nobody chose for it.
        self.operator_hover = False
        self.operator = i

    def operator_offer(self, i: int) -> int:
        """The `OPERATOR_ACTION` code the action key would perform on robot ``i`` now.

        The dashboard's hint is drawn from this and `operator_act` performs exactly the
        action it named, so the label and the key cannot disagree -- a key that offers
        PICK UP and then does nothing is worse than a key that offers nothing.
        """
        return self._operator_action(i)[0]

    def operator_act(self, i: int) -> int:
        """Perform the operator's action on robot ``i``. The code performed, 0 for none.

        Emitted, like every other change to a casualty's state -- the operator picking
        one up is exactly as much a part of the mission's story as a carrier doing it.
        """
        code, vi = self._operator_action(i)
        rid = self.robot_ids[i] if 0 <= i < self.n else "?"
        if code == OPERATOR_ACTION["set_down"]:
            v = self.victims[vi]
            v.pos = self.pos[i].copy()
            v.state, v.carrier = CLEARED, -1
            self.carrying[i] = -1
            self._emit("operator_action", f"{rid} set {v.id} down in {self._sector_at(v.pos)}",
                       robot=rid, victim=v.id, sector=self._sector_at(v.pos), pos=tuple(v.pos))
        elif code == OPERATOR_ACTION["pick_up"]:
            v = self.victims[vi]
            v.state, v.carrier = CARRIED, i
            self.carrying[i] = vi
            self._emit("operator_action", f"{rid} picked up {v.id} in {self._sector_at(v.pos)}",
                       robot=rid, victim=v.id, sector=self._sector_at(v.pos), pos=tuple(v.pos))
        elif code in (OPERATOR_ACTION["take_off"], OPERATOR_ACTION["land"]):
            # The latch only; `Mission` owns the airborne array and reads it next tick.
            self.operator_hover = code == OPERATOR_ACTION["take_off"]
            what = "took off" if self.operator_hover else "landed"
            self._emit("operator_action", f"{rid} {what} in {self._sector_at(self.pos[i])}",
                       robot=rid, sector=self._sector_at(self.pos[i]), pos=tuple(self.pos[i]))
        return code

    def _operator_action(self, i: int) -> tuple[int, int]:
        """``(OPERATOR_ACTION code, victim index)``; the index is -1 where there is none.

        Ground truth is read here and nowhere above it. The operator's unit gets the
        reach its own autonomy already has and not a metre more -- `_update_victims`
        grabs at exactly `REACH_GRAB` -- so the key is a trigger for a rule the swarm
        already follows, not a longer arm.
        """
        if not (0 <= i < self.n) or self.status[i] > OUT_OF_COMMS:
            return OPERATOR_ACTION["none"], -1
        held = int(self.carrying[i])
        if held >= 0:
            return OPERATOR_ACTION["set_down"], held
        if self.chassis[i] == CHASSIS_INDEX["rotor"]:
            # A rotor can never hold a casualty (robot.py `CHASSIS_BARRED`), so its key
            # is its height. It may not set down anywhere its autonomy could not: over a
            # river the offer is nothing at all rather than a landing that cannot happen.
            if not self.airborne[i]:
                return OPERATOR_ACTION["take_off"], -1
            return (OPERATOR_ACTION["land"], -1) if self.can_land()[i] \
                else (OPERATOR_ACTION["none"], -1)
        if self.actuator[i] != LANE_INDEX["gripper"]:
            return OPERATOR_ACTION["none"], -1
        best, best_d2 = -1, REACH_GRAB ** 2
        for vi, v in enumerate(self.victims):
            if v.state != CLEARED:
                continue
            d2 = float((self.pos[i, 0] - v.pos[0]) ** 2 + (self.pos[i, 1] - v.pos[1]) ** 2)
            if d2 <= best_d2:
                best, best_d2 = vi, d2
        if best < 0:
            return OPERATOR_ACTION["none"], -1
        return OPERATOR_ACTION["pick_up"], best

    def _has_los(self, i: int, target: np.ndarray) -> bool:
        """Line of sight from robot ``i`` to a world point. Walls block, rubble does not."""
        tix, tiy = grid.world_to_cell(
            np.array([target[0]]), np.array([target[1]]), self.cell, self.shape
        )
        return bool(
            grid.visible_cells(self.occ, self.pos[i, 0], self.pos[i, 1], tix, tiy, self.cell)[0]
        )

    def mark_found(self, vi: int, by: int) -> None:
        """Called by perception when a report resolves onto a real casualty."""
        if self.victims[vi].state == HIDDEN:
            self._find_victim(vi, by)

    def _find_victim(self, vi: int, by: int) -> None:
        v = self.victims[vi]
        v.state, v.found_t = FOUND, self.t
        self._emit(
            "victim_found",
            f"{v.id} found in {self._sector_at(v.pos)} by {self.robot_ids[by]}"
            + (" (buried)" if v.buried else ""),
            victim=v.id, robot=self.robot_ids[by],
            sector=self._sector_at(v.pos), pos=tuple(v.pos),
        )

    def _kill(self, i: int, cause: str) -> None:
        self.status[i] = DESTROYED
        self.airborne[i] = False
        self.in_comms[i] = False
        # Whatever it had seen and not yet reported dies with it. Dropping the entry is
        # bookkeeping, not policy: `_store_and_forward` already skips the dead, so the
        # buffer was unreachable -- it just sat in the dict for the rest of the mission.
        self._pending_cells.pop(i, None)
        held = int(self.carrying[i])
        if held >= 0:
            self.carrying[i] = -1
            self.victims[held].state = CLEARED
            self.victims[held].carrier = -1
        self._emit(
            "robot_destroyed",
            f"{self.robot_ids[i]} destroyed by {cause} in {self._sector_at(self.pos[i])}",
            robot=self.robot_ids[i], sector=self._sector_at(self.pos[i]), pos=tuple(self.pos[i]),
        )

    # ------------------------------------------------------------------ queries

    def _sector_at(self, p) -> str:
        ix, iy = grid.world_to_cell(np.asarray(p[0]), np.asarray(p[1]), self.cell, self.shape)
        return self.sector_ids[int(self.sector_of_cell[int(iy), int(ix)])]

    def _emit(self, kind: str, text: str, **kw: Any) -> None:
        self.events.append({"t": round(self.t, 3), "kind": kind, "text": text, **kw})

    def drain_events(self) -> list[dict[str, Any]]:
        out, self.events = self.events, []
        return out

    @property
    def victims_found(self) -> int:
        return sum(1 for v in self.victims if v.state != HIDDEN)

    @property
    def victims_rescued(self) -> int:
        return sum(1 for v in self.victims if v.state == RESCUED)

    @property
    def robots_lost(self) -> int:
        return int((self.status >= DESTROYED).sum())

    @property
    def done(self) -> bool:
        return (
            self.t >= self.scn.mission_duration_s
            or self.victims_rescued == len(self.victims)
            or bool((self.status >= DESTROYED).all())
        )

    def scorecard(self):
        from ..metrics import Scorecard

        times = [v.rescued_t - 0.0 for v in self.victims if v.state == RESCUED]
        return Scorecard(
            seed=self.seed,
            sim_time=round(self.t, 3),
            victims_rescued=self.victims_rescued,
            victims_found=self.victims_found,
            victims_total=len(self.victims),
            ground_explored_frac=round(float(self.explored[self.passable].mean()), 6),
            robots_lost=self.robots_lost,
            robots_total=self.n,
            mean_time_to_rescue=round(float(np.mean(times)) if times else 0.0, 3),
            energy_used=round(float(self.energy_used.sum()), 6),
            directives_issued=0,
            directives_rejected=0,
        )

    def lane_counts(self) -> dict[str, int]:
        return {a: int((self.actuator == LANE_INDEX[a]).sum()) for a in LANES}
