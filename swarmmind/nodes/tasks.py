"""Task generation, shared by every allocator.

What work exists is separate from who does it. The auction (nodes/auction.py) and the
greedy baseline (control/heuristic.py) consume exactly the same task list, which is what
makes the gate comparison honest -- they differ only in allocation.

Everything casualty-related comes off the report tracker, never the world's victim list.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass

import numpy as np

from ..control.planner import FrontierTarget, frontier_targets
from ..sim import grid
from ..sim.robot import LANE_INDEX, OUT_OF_COMMS
from ..sim.world import CARRIED, CLEARED, FOUND, RESCUED

#: Lower rank dispatches first. Rescue beats search: a found casualty left uncollected
#: while the swarm keeps exploring is the classic failure mode.
#: Two robots inside this of each other are covering the same ground -- one
#: scouting swath. Used only when `TaskGenerator.max_per_target` is on.
SWATH_M = 10.0

RANK = {"retreat": 0, "extract": 1, "clear_debris": 2, "investigate": 3,
        "relay": 4, "explore": 5}


@dataclass(frozen=True)
class OpenTask:
    kind: str
    target: tuple[float, float]
    rank: float
    victim: int | None = None
    report: str | None = None
    value: float = 1.0
    #: A second or third robot sent to a casualty as insurance. Backups may only take a
    #: robot that is *idle*: `AuctionNode.step` refuses to preempt for them, so a backup
    #: never pulls a robot off work it is already doing.
    backup: bool = False


class TaskGenerator:
    def __init__(self, max_explore_targets: int = 48, max_relay_chains: int = 24,
                 posts_per_chain: int = 8, sector_sweep: bool = True,
                 sweep_below: float = 0.90, sweep_min_density: float = 8.0,
                 lookahead_steps: int = 12, max_per_target: int = 0,
                 chain_targets: str = "unexplored",
                 carriers_per_victim: int = 2, diggers_per_victim: int = 3,
                 sweep_at_dark: bool = False,
                 mobile_anchors: bool = False) -> None:
        self.max_explore_targets = max_explore_targets
        #: Dispatch robots to open a NEW exploration front in an untouched sector,
        #: instead of only widening the existing one.
        #:
        #: Frontier exploration is a perimeter process: the known region grows at its
        #: boundary, and a boundary absorbs only so many robots. On a large map that
        #: caps coverage regardless of swarm size -- 768 -> 1536 robots moved coverage
        #: 23.6% -> 32.3%, and raising the frontier target cap from 48 to 240 moved it
        #: 23.2% -> 23.7%, because the frontier simply does not have more clusters to
        #: offer. More robots need more fronts, not a longer queue for one front.
        #: Robots offered the same casualty, as insurance against a holder that stalls.
        #:
        #: Only one carrier can lift a casualty and only `MAX_DIGGERS` can usefully dig
        #: one, so these are not throughput numbers -- they are redundancy. The cost is
        #: bounded and self-cancelling: extras are drawn only from idle robots (backups
        #: never preempt), and they release themselves the moment the casualty is dug out
        #: or lifted, because both casualty tasks complete on world state rather than on
        #: their own holder arriving.
        #:
        #: 2 carriers, not 3: a second is insurance, a third starts competing with the
        #: search lanes for no gain, and the failure this guards against -- one holder
        #: stalling -- is already covered by two.
        self.carriers_per_victim = carriers_per_victim
        self.diggers_per_victim = diggers_per_victim
        self.sector_sweep = sector_sweep
        self.sweep_below = sweep_below
        #: Robots per sector below which sweeping is switched off.
        #:
        #: Opening a new front costs a robot the travel time to get there, which only
        #: pays when there are robots to spare relative to the number of fronts. On the
        #: 16-robot test fixture (1.3 robots per sector) sweeping cut casualties found
        #: from 5 to 3; on the 768-robot demo map (16 per sector) it raised coverage
        #: 23.2% -> 30.4% and doubled them. Scale with the swarm, do not hardcode.
        self.sweep_min_density = sweep_min_density
        self.max_relay_chains = max_relay_chains
        #: New relay posts emitted per chain per cycle.
        #:
        #: One was the original design -- grow the chain a link at a time so relays do
        #: not all race to the same gap. Fine on a 320 m map, badly wrong on a 577 m
        #: diagonal: the comms component reached only 193 m from base by t=250 with 29
        #: of 192 relays deployed, so most of the swarm could not report what it found.
        #: Chain length scales with the map, so the build rate has to as well.
        self.posts_per_chain = posts_per_chain
        #: Links of open ground a chain is aimed *past* the frontier, in `step` units.
        #:
        #: This is the whole of the M-39 fix and it is a mechanism, not a tuning knob:
        #: at zero the chain stops at the frontier, which is the comms boundary, which
        #: is what the chain is trying to move -- the deadlock that held reach at ~230 m
        #: of a 577 m diagonal on all four seeds measured.
        #:
        #: **The value saturates, which is why it is not a tuned number.** 12 and 24
        #: measure the same on four seeds (18.0 vs 17.0 rescued, 277 vs 262 m reach):
        #: past a point the chain is limited by how much of it the relay lane can staff
        #: and by the map edge, not by how far ahead it is allowed to look. 0 is clearly
        #: worse than any positive value; 3 through 24 are within noise of each other
        #: (MEASUREMENTS.md M-40). Do not spend a run tuning it -- change the budget or
        #: the lane size instead, which are the things that actually bind.
        self.lookahead_steps = lookahead_steps
        #: Cap on how many robots may be walking to the same search target. 0 = no cap.
        #:
        #: **This was tried and reverted** (M-36): claiming search targets turned
        #: 629 scouts on 44 targets into 449 on 436, and rescues fell 13 -> 5. The reason
        #: was comms, not crowding -- spreading the swarm pushed scouts past the envelope
        #: and `mark_seen` records nothing from an out-of-contact robot, so the extra
        #: ground covered never registered.
        #:
        #: That measurement was taken on the 480 x 320 m map, where the comms component
        #: reached 240 m of a 577 m diagonal. The scenario is now 360 x 240 (433 m
        #: diagonal) with reach ~297 m, and 78-99% of the swarm stays in contact. The
        #: premise of the revert may no longer hold, which is why the cap is a parameter
        #: rather than a deletion -- see M-48 for the re-test.
        self.max_per_target = max_per_target
        #: Where relay chains are *aimed*: "frontier" or "unexplored".
        #:
        #: M-39 broke the coupling that capped chain *length* -- chains now reach past the
        #: frontier instead of stopping at it. The same coupling still governs chain
        #: *direction*: targets come from the largest frontier clusters, the frontier sits
        #: at the comms boundary, so the network grows where the swarm already is and
        #: never toward ground it has not reached. Measured consequence (M-53): six
        #: sectors 329-385 m from base finish at 0.0% explored with robots standing in
        #: them, because reach plateaus at ~310 m and nothing they see is ever reported.
        #:
        #: "unexplored" aims each chain at the least-explored sector instead, which is
        #: spread across the map by construction. Kept switchable because M-53's obvious
        #: fix measured as a regression and this one is the same shape.
        self.chain_targets = chain_targets
        #: Sweep points aim at the centroid of a sector's *unexplored* cells rather
        #: than its geometric centre.
        #:
        #: **Off, and measured that way.** The reasoning is sound -- a sector half
        #: explored has an already-explored centre, and `_state` retires an explore task
        #: the moment its target cell reads explored, so the sweep point died on the tick
        #: it was awarded. It measures at **-1.0 rescues, -1.25 found and -0.6 points of
        #: exploration** on four seeds (M-66). Likely because the centroid of two
        #: separate dark corners is the explored middle between them, which is a worse
        #: target than the sector centre rather than a better one.
        #:
        #: The half of "sweep every corner" that *did* pay is `sweep_below`, raised from
        #: 0.55 to 0.90 in the same pass: 66.00 / 68.00 / 70.75 rescued at 0.55 / 0.75 /
        #: 0.90, so sectors keep getting a sweep point until they are nearly finished.
        self.sweep_at_dark = sweep_at_dark
        #: An idle relay out in the field counts as network topology, not just the ones
        #: holding a post.
        #:
        #: **Off, and measured that way.** A drifting relay is a link *now* and gone in
        #: seconds, so planning a chain around it suppresses the post that would have
        #: filled the gap it then walks away from: relays on station 49 -> 42, and
        #: **-1.5 rescued, -2.0 found** on four seeds (M-69). The lane's mobility is real
        #: but it belongs to the robot, not the planner -- `_relay_orders` rung 0 makes a
        #: relay that is *currently* somebody's only link stand still and be that post.
        self.mobile_anchors = mobile_anchors

    def generate(self, world, ex, tracker) -> list[OpenTask]:
        # An orphaned assignment does not hold a claim: that is exactly what orphaning
        # means -- the work is back on the market even though its robot is still on it.
        live = [a for a in ex.assignment if a is not None and not a.orphaned]
        #: How many robots are on each casualty, not merely *whether* one is.
        #:
        #: A boolean claim is a hostage. `claimed_v` removed a casualty from task
        #: generation entirely the moment anything was assigned to it, and nothing
        #: expires a claim: a digger that stalls on terrain, or whose task never retires,
        #: keeps its casualty off the market for the rest of the mission. Nobody else is
        #: offered it -- not another digger, and not a carrier once it is dug out. From
        #: the outside that is indistinguishable from a busy swarm, because no task goes
        #: unfilled and nothing errors; the work simply never enters the market.
        #:
        #: Over-assigning is safe here in a way it is not for search work, because both
        #: casualty tasks retire *themselves* on the world state rather than on their
        #: holder: `SkillExecutor._state` returns "done" for every `clear_debris` on a
        #: casualty the moment it is dug out, and "invalid" for every `extract` whose
        #: casualty another carrier has already lifted. So the redundant robots cost one
        #: cycle of walking and release on their own the instant the job is done.
        on_victim: Counter[int] = Counter(
            a.victim for a in live if a.victim is not None)
        claimed_r = {a.report for a in live if a.report is not None}
        # NOTE: search targets are deliberately *not* deduplicated against live
        # assignments, although casualties and reports are.
        #
        # At t=60 s, 629 scouts are walking to 44 distinct targets, 33 of them converging
        # on one spot, and the swarm covers new ground at ~4% of its theoretical rate.
        # That looks exactly like the bug it is not. Claiming search targets within a
        # scouting swath was implemented and measured: **13 rescues fell to 5**, and
        # reverting only the most aggressive part still left 6. Spreading the swarm out
        # pushes scouts past the comms envelope, and `World.mark_seen` records only what
        # an in-contact robot sees -- so the extra ground never registers, while the
        # concentration that was feeding the rescue chain is gone.
        #
        # Overlap is the price of staying in contact on this map. Fixing it means fixing
        # comms coverage first (MEASUREMENTS.md M-36).
        out: list[OpenTask] = []

        if tracker is not None:
            # One casualty, one task. `resolved_victims()` returns *reports*, and several
            # reports can resolve onto the same casualty -- corroboration from different
            # angles is exactly what the tracker is for. Without this, each of them
            # became its own extract task in the same pass, and `claimed_v` did not catch
            # it because it dedups against *live assignments*, which none of them were
            # yet. Measured on seed 42: casualty v6 awarded to three carriers in one
            # cycle at t=396, each ~255 m away, while a free carrier stood 4 m from it.
            offered: set[int] = set()
            for r in tracker.resolved_victims():
                vi = r.victim
                v = world.victims[vi]
                if vi in offered or v.state == RESCUED:
                    continue
                offered.add(vi)
                if v.state == CLEARED:
                    job = ("extract", RANK["extract"], 3.0, self.carriers_per_victim)
                elif v.state == FOUND and v.buried and v.debris_remaining > 0.0:
                    # Capped at what the simulation will actually pay for: `World.
                    # _update_victims` clears debris at `clear_rate x min(diggers,
                    # MAX_DIGGERS)`, so a fourth digger round the hole does nothing.
                    job = ("clear_debris", RANK["clear_debris"], 2.5, self.diggers_per_victim)
                elif (v.state == CARRIED and v.carrier >= 0
                      and world.status[v.carrier] > OUT_OF_COMMS):
                    job = ("extract", RANK["extract"], 3.0, self.carriers_per_victim)
                else:
                    continue
                kind, rank, value, want = job
                have = on_victim.get(vi, 0)
                for k in range(max(0, want - have)):
                    out.append(OpenTask(kind, tuple(v.pos), rank, vi, r.id, value,
                                        backup=(have + k) > 0))

            for r in tracker.open_reports():
                if r.id in claimed_r:
                    continue
                pos = (float(r.pos[0]), float(r.pos[1]))
                out.append(OpenTask("investigate", pos, RANK["investigate"],
                                    None, r.id, 1.0 + r.conf))

        density = world.n / max(1, len(world.sector_ids))
        if self.sector_sweep and density >= self.sweep_min_density:
            for k, sid in enumerate(world.sector_ids):
                if world.sector_explored_pct[k] >= self.sweep_below:
                    continue
                x0, y0, x1, y1 = world.scn.sector_rect(sid)
                # One sweep point per sector, not a grid of them.
                #
                # A 3x3 grid per sector was tried and reverted: it gave 449 scouts 436
                # distinct destinations, which sounds like the fix for double-treading and
                # measured as **13 rescues down to 5**. Spreading the swarm that thin
                # pushes scouts past the comms envelope, and `mark_seen` only records what
                # an in-contact robot sees -- so the extra ground covered never registered
                # while the swarm lost the concentration that was feeding the rescue
                # chain. Exploration and comms pull in opposite directions here, and
                # rescues are the objective (MEASUREMENTS.md M-36).
                # Aim at what is still dark, not at the middle of the box.
                #
                # The sweep point used to be the sector's geometric centre, which on a
                # sector that is already half explored is usually ground the swarm has
                # covered -- and `_state` retires an explore task the moment its target
                # cell reads explored, so the task died on the tick it was awarded. A
                # sector could sit under `sweep_below` for the whole mission, emit a
                # sweep point every cycle, and never get anyone to the corner that was
                # actually missing. Measured consequence: 16 casualties finished a run on
                # explored ground with no contact ever raised (M-56).
                #
                # The centroid of the sector's unexplored passable cells is the one point
                # that always names ground still needing a look. Still ONE point per
                # sector -- the 3x3 grid below was tried and reverted, and this does not
                # reopen that.
                centre = (_dark_centroid(world, k, x0, y0, x1, y1)
                          if self.sweep_at_dark
                          else snap_passable(world, ((x0 + x1) * 0.5,
                                                    (y0 + y1) * 0.5)))
                # Ranked alongside frontier work, not behind it: the point is that
                # some of the swarm goes somewhere new rather than queueing.
                if centre is not None and not in_known_hazard(world, centre):
                    out.append(OpenTask("explore", centre, RANK["explore"] - 0.5,
                                        value=2.0))

        fts = frontier_targets(world, max_targets=self.max_explore_targets)
        for post in self.relay_posts(world, ex, self._chain_targets(world, fts)):
            out.append(OpenTask("relay", post, RANK["relay"], value=1.5))

        # How many robots are already walking to each search target, if the cap is on.
        headed: list[np.ndarray] = []
        if self.max_per_target > 0:
            headed = [np.asarray(a.target, dtype=np.float64) for a in ex.assignment
                      if a is not None and a.kind == "explore"]
        for i, ft in enumerate(fts):
            if self.max_per_target > 0 and headed:
                near = sum(1 for h in headed
                           if abs(h[0] - ft.pos[0]) < SWATH_M
                           and abs(h[1] - ft.pos[1]) < SWATH_M)
                if near >= self.max_per_target:
                    continue
            out.append(OpenTask("explore", ft.pos, RANK["explore"] + 0.001 * i,
                                value=1.0 + min(ft.size, 40) / 40.0))

        out = [t for t in out if not in_known_hazard(world, t.target)]

        # --- where Tier 3 actually bites --------------------------------------------
        #
        # A directive that only shows up in a text feed is theatre. Sector priority
        # shifts a task's rank, which is the auction's first sort key, and an abandoned
        # sector stops producing search work altogether. Neither is required for the
        # swarm to function -- with no directives every sector sits at `normal` and this
        # reduces to the unmodified ordering.
        adjusted: list[OpenTask] = []
        for t in out:
            k = sector_index(world, t.target)
            if k is None:
                adjusted.append(t)
                continue
            if world.sector_abandoned[k] and t.kind in ("explore", "investigate", "relay"):
                continue
            # 0 high / 1 normal / 2 low -> -0.5 / 0 / +0.5 on the rank.
            bump = (int(world.sector_priority[k]) - 1) * 0.5
            adjusted.append(
                t if bump == 0.0 else OpenTask(
                    t.kind, t.target, t.rank + bump, t.victim, t.report, t.value, t.backup
                )
            )

        adjusted.sort(key=lambda t: (t.rank, t.target))
        return adjusted


    def _chain_targets(self, world, fts):
        """Where the relay chains should be pointed, as `FrontierTarget`-shaped goals.

        `frontier` is the original behaviour. `unexplored` ranks the sectors by how little
        of them has been seen and aims a chain at each, so direction is chosen by what the
        swarm still needs rather than by where its boundary happens to be densest. Sectors
        are used because they are spread evenly over the map by construction and they
        already carry an explored fraction the swarm computes anyway.
        """
        if self.chain_targets != "unexplored":
            return fts
        pct = np.asarray(world.sector_explored_pct)
        # Ascending explored fraction, ties on sector index: deterministic (invariant #5).
        order = sorted(range(len(pct)), key=lambda k: (float(pct[k]), k))
        out = []
        for k in order[: self.max_relay_chains]:
            if pct[k] >= 0.95:
                break                       # nothing left to reach for out there
            x0, y0, x1, y1 = world.scn.sector_rect(world.sector_ids[k])
            centre = snap_passable(world, ((x0 + x1) * 0.5, (y0 + y1) * 0.5))
            if centre is not None and not in_known_hazard(world, centre):
                out.append(FrontierTarget(pos=centre, size=1, sector=k))
        return out or fts

    def relay_posts(self, world, ex, fts) -> list[tuple[float, float]]:
        """Positions that extend the comms component **past** the frontier, not up to it.

        The chain used to be walked from base to a frontier target and stopped there.
        That reads as correct and deadlocks the mission (MEASUREMENTS.md M-39). A
        frontier is the edge of *explored* ground, and `World.mark_seen` writes the
        shared map only for in-contact robots -- so explored ground ends at the comms
        boundary, which is the exact thing the chain exists to move. Every link waited
        on exploration that was waiting on that link, and three numbers plateaued
        together at t~=180: reach ~240 m of a 577 m diagonal, 37% explored, and half the
        relay lane never on post at all.

        The endpoint is therefore the frontier target **plus `lookahead_steps` links of
        open ground beyond it**. The network is built ahead of the swarm, and the loop
        now runs forwards: contact extends, fog opens behind it, the next frontier sits
        further out, and the chain is offered more ground than it had last cycle.
        """
        rr = world.scn.comms.relay_radius
        step = rr * 0.8
        anchors = [np.array(world.scn.base, dtype=np.float64)]
        for a in ex.assignment:
            if a is not None and a.kind == "relay":
                anchors.append(np.array(a.target, dtype=np.float64))
        # NOTE: an *idle* relay is deliberately not an anchor, wherever it is standing.
        #
        # It used to be, and that single line cost the entire mission. A candidate post is
        # rejected within 0.75 x relay_radius of any anchor. With 192 relays spawned
        # around base -- 128 of them inside the 40 m base radius -- they formed a wall of
        # anchors over exactly the ground a chain's *first* link needs. No chain could
        # start, so no relay was ever given a post, so every relay stayed at spawn
        # continuing to be an anchor: **0 posts generated from 48 frontier targets**.
        #
        # The consequences reached much further than the relay lane. Comms never extended
        # past base radius; `World.mark_seen` only writes to the shared map for robots in
        # comms, so scouts walking beyond it revealed into private buffers and the
        # explored region visibly stopped growing at the comms boundary while the swarm
        # kept working. Anchors now mean *committed network topology* -- base, plus posts
        # a relay has actually been assigned -- and nothing else.

        # How many new posts the relay lane can actually staff.
        #
        # Extending chains without this is what turns the fix into a regression: 96
        # relays spread over 24 lengthening chains leave a hole in every one of them,
        # and a chain with a hole carries nothing, so the breadth costs reach instead of
        # buying coverage. The lane is 96 relays x 30.4 m = 2.9 km of network in total;
        # announcing more posts than that is announcing gaps. Posts are spent
        # highest-priority chain first, because `fts` is ordered by frontier size.
        alive_relay = (world.actuator == LANE_INDEX["antenna"]) & (world.status <= OUT_OF_COMMS)
        staffed = sum(1 for i, a in enumerate(ex.assignment)
                      if a is not None and a.kind == "relay" and alive_relay[i])
        budget = int(alive_relay.sum()) - staffed
        if budget <= 0:
            return []

        # A relay does not have to hold a post to be a link. `World._update_comms` builds
        # the graph from **every living antenna**, so a relay standing out in the field is
        # a post that happens to be moving, and planning as though it were not is how the
        # lane announces a post on ground it is already covering -- spending a budget that
        # is measured in relays on links that already exist.
        #
        # The base-radius guard is what makes this safe rather than a rerun of the wall
        # described above. That failure was 128 relays *at spawn*, every one of them
        # inside the base radius, and every one of them excluded here by construction. A
        # relay past that radius is not part of a wall; it is the chain.
        #
        # Only in-contact relays count. One that has drifted out of the component is not
        # topology -- it is rung 1's next job.
        if self.mobile_anchors:
            base_xy = np.array(world.scn.base, dtype=np.float64)
            br = world.scn.comms.base_radius
            free_relay = alive_relay & world.in_comms
            for i in np.nonzero(free_relay)[0]:
                if ex.assignment[i] is not None:
                    continue
                q = world.pos[i].astype(np.float64)
                if float(np.linalg.norm(q - base_xy)) > br:
                    anchors.append(q)

        posts: list[tuple[float, float]] = []
        base = np.array(world.scn.base, dtype=np.float64)
        lookahead = step * self.lookahead_steps
        for ft in fts[: self.max_relay_chains]:
            if len(posts) >= budget:
                break
            tgt = np.array(ft.pos, dtype=np.float64)
            span = float(np.linalg.norm(tgt - base))
            if span < rr * 0.6:
                continue
            direction = (tgt - base) / span
            # Clamped to the map, because `grid.world_to_cell` clips rather than fails:
            # a ray aimed past the border otherwise piles every post of every chain that
            # leaves on that side onto the same edge cells.
            span = min(span + lookahead, ray_exit(base, direction, world))
            placed = 0
            for k in range(1, max(1, int(np.ceil(span / step))) + 1):
                if placed >= self.posts_per_chain or len(posts) >= budget:
                    break
                p = base + direction * min(step * k, span)
                if any(float(np.linalg.norm(p - a)) <= rr * 0.75 for a in anchors):
                    continue  # already covered by committed topology -- not a gap
                snapped = snap_passable(world, p)
                if snapped is None or in_known_hazard(world, snapped):
                    # A hole this chain cannot fill. Every post past it would be a relay
                    # standing in a network with no path back to base, so end the chain
                    # here rather than spending the lane behind a break.
                    break
                posts.append(snapped)
                anchors.append(np.array(snapped))
                placed += 1
        return posts


def _dark_centroid(world, k: int, x0: float, y0: float, x1: float, y1: float):
    """Centroid of the unexplored passable cells of sector ``k``, snapped to passable.

    Falls back to the geometric centre when the sector has no unexplored passable cell
    left, which is the only case where the old behaviour was the right answer anyway.
    """
    ix0 = int(x0 / world.cell)
    ix1 = int(np.ceil(x1 / world.cell))
    iy0 = int(y0 / world.cell)
    iy1 = int(np.ceil(y1 / world.cell))
    sub_pass = world.passable[iy0:iy1, ix0:ix1]
    sub_expl = world.explored[iy0:iy1, ix0:ix1]
    dark = sub_pass & ~sub_expl
    if not dark.any():
        return snap_passable(world, ((x0 + x1) * 0.5, (y0 + y1) * 0.5))
    ys, xs = np.nonzero(dark)
    cx = (ix0 + float(xs.mean()) + 0.5) * world.cell
    cy = (iy0 + float(ys.mean()) + 0.5) * world.cell
    return snap_passable(world, (cx, cy))


def sector_index(world, p) -> int | None:
    ix, iy = grid.world_to_cell(np.asarray(p[0]), np.asarray(p[1]), world.cell, world.shape)
    return int(world.sector_of_cell[int(iy), int(ix)])


def abandoned_preemptions(world, ex) -> list[int]:
    """Robots standing in a sector Tier 3 has closed. They leave, whatever they were on.

    This is the second half of what makes an `abandon` mean something: without it the
    sector stops producing work but the robots already there keep grinding away in it.
    """
    if not world.sector_abandoned.any():
        return []
    ix, iy = grid.world_to_cell(world.pos[:, 0], world.pos[:, 1], world.cell, world.shape)
    in_closed = world.sector_abandoned[world.sector_of_cell[iy, ix]]
    alive = world.status <= OUT_OF_COMMS
    out = []
    for i in np.nonzero(in_closed & alive)[0]:
        a = ex.assignment[i]
        if a is None or a.kind != "retreat":
            out.append(int(i))
    return out


def hazard_preemptions(world, ex) -> list[int]:
    """Robots standing in observed hazard that need pulling out, whatever they were doing.

    Tier 1's hazard repulsion only sees ~6 m. Inside a front tens of metres across every
    probe reads hazardous and there is no usable local gradient: the robot needs a
    destination, not a nudge.
    """
    ix, iy = grid.world_to_cell(world.pos[:, 0], world.pos[:, 1], world.cell, world.shape)
    burning = world.hazard_known[iy, ix] & (world.status <= OUT_OF_COMMS)
    out = []
    for i in np.nonzero(burning)[0]:
        a = ex.assignment[i]
        if a is None or a.kind != "retreat":
            out.append(int(i))
    return out


def nearest_haven(world, i: int) -> tuple[float, float] | None:
    havens = [np.array(z, dtype=np.float64)
              for z in [world.scn.base, *world.scn.extraction_zones]
              if not in_known_hazard(world, z)]
    if not havens:
        return None
    h = min(havens, key=lambda h: float(np.linalg.norm(world.pos[i] - h)))
    return (float(h[0]), float(h[1]))


def in_known_hazard(world, p) -> bool:
    ix, iy = grid.world_to_cell(np.asarray(p[0]), np.asarray(p[1]), world.cell, world.shape)
    return bool(world.hazard_known[int(iy), int(ix)])


def ray_exit(origin, direction, world) -> float:
    """Distance along a unit ray from ``origin`` until it leaves the map.

    `grid.world_to_cell` clips rather than raising, so a target off the edge silently
    becomes a target *on* the edge -- and every chain leaving on that side then snaps
    its posts onto the same border cells. One cell of margin keeps the endpoint inside.
    """
    limits = ((float(origin[0]), float(direction[0]), world.shape[1] * world.cell),
              (float(origin[1]), float(direction[1]), world.shape[0] * world.cell))
    t = float("inf")
    for o, d, hi in limits:
        if abs(d) < 1e-9:
            continue
        edge = (hi - world.cell) if d > 0.0 else world.cell
        t = min(t, (edge - o) / d)
    return max(0.0, t)


def snap_passable(world, p) -> tuple[float, float] | None:
    """Nearest passable cell centre to ``p``, searching a small ring. None if boxed in."""
    for r in range(0, 6):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                ix, iy = grid.world_to_cell(
                    np.asarray(p[0] + dx * world.cell), np.asarray(p[1] + dy * world.cell),
                    world.cell, world.shape,
                )
                if world.passable[int(iy), int(ix)]:
                    return (float((int(ix) + 0.5) * world.cell),
                            float((int(iy) + 0.5) * world.cell))
    return None
