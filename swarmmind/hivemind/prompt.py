"""Building the hivemind's view of the swarm.

Tier 3 sees an aggregate, never individual robots. That is not a simplification for the
model's benefit -- it is the architectural claim. A strategic layer that issues per-robot
commands is not a strategic layer, and it would make Tier 2 depend on Tier 3, which is
the one thing that must never happen (CLAUDE.md invariant #1).

Prompt size is independent of swarm size -- 768 robots and 16 render identically, because
everything is summarised by lane -- but it was *not* independent of map size, and that
cost real calls. The sector table dominates the prompt, and the demo map has 48 sectors
against the test fixture's 12: 2,963 characters versus 1,374, mean latency 4.61 s versus
3.45 s, against a 5 s rung timeout calibrated on the small map. **Half the hivemind calls
in the first full-scale run timed out** (MEASUREMENTS.md M-33).

So only the `MAX_SECTOR_ROWS` most decision-relevant sectors are listed, with the rest
folded into one summary line. This is better than a bigger timeout on its own: a strategic
layer does not need a row for a sector that is fully explored, unthreatened and empty, and
48 near-identical rows are exactly what a 1.5B model loses the thread in.

**Every number here comes from the swarm's own belief** -- the report tracker and the
executor -- and none from the simulator's victim list. The hivemind is not told how many
casualties exist, because knowing the denominator is exactly the privileged information
that would let it plan against ground truth rather than against what has been seen. It
reasons over an incomplete picture for the same reason the robots do.
"""

from __future__ import annotations

import numpy as np

from ..sim.robot import CHASSIS, LANES

#: Events with no strategic content. `task_awarded` fires several times a second at 768
#: robots; left in, it is the *entire* RECENT section and the model's only narrative
#: context is a list of task ids. The dashboard drops it for the same reason.
#: `directive_issued` is excluded too: its text is the model's own last reasoning, which
#: already has its own section below. Left in, four of the six RECENT lines were the
#: hivemind quoting itself, which both wastes the window and invites it to repeat.
#: `directive_rejected` deliberately stays -- being told your last plan was refused, and
#: why, is the most useful line the model can get.
NOISE_EVENTS = frozenset({
    "task_awarded", "task_created", "task_completed", "directive_issued",
})

#: Lane 0 is the plain scout chassis -- "none" means no actuator, not no robot.
LANE_LABEL = {"none": "scout"}

SYSTEM = """You are the strategic coordinator for a robot swarm searching a collapsed \
disaster zone for casualties.

You set SECTOR PRIORITIES. You never command individual robots -- the swarm allocates its \
own work and keeps operating if you go silent.

How your output is used, exactly:
- priority "high"    the swarm sends more robots to that sector
- priority "normal"  THE DEFAULT. Sending it changes nothing. Do not spend a directive on it.
- priority "low"     the swarm deprioritises the sector but keeps working it
- priority "abandon" the sector CLOSES: no new search work, and robots there are pulled out

Rules that follow from the map you are given:
- Send robots to sectors with LOW explored% -- undiscovered casualties are in ground \
nobody has covered yet. A sector at 100% explored needs nothing.
- Use "rescue" where awaiting > 0. Those are confirmed casualties waiting for a carrier.
- Only "abandon" a sector whose hazard is high. Abandoning costs you the whole sector.
- Prefer 1-3 directives. Changing everything is the same as changing nothing.

Reply with JSON only: {"reasoning": "<one short sentence>", "directives": [...]}.
Each directive is {"sector": "<id>", "priority": "...", "action": "..."}.

Example, for a map where C1 is unexplored, B3 holds a confirmed casualty, and A3 is burning:
{"reasoning": "Push into unexplored C1, extract the B3 contact, write off burning A3.",
 "directives": [{"sector": "C1", "priority": "high", "action": "explore"},
                {"sector": "B3", "priority": "high", "action": "rescue"},
                {"sector": "A3", "priority": "abandon", "action": "abandon"}]}

A directive persists 30 seconds unless you repeat it, so an abandon you forget about \
keeps a sector closed."""


#: How many sector rows to render. Everything else is one summary line. Sized so the
#: prompt stays near the latency the rung timeout was set against, whatever the map.
MAX_SECTOR_ROWS = 16


def _interest(world, k: int, awaiting: int) -> float:
    """How much this sector should matter to a strategic layer, high first.

    Unexplored ground is where undiscovered casualties are; a confirmed contact is work
    already waiting; hazard is what forces a decision. A sector that is fully explored,
    unthreatened and empty is exactly the row worth cutting.
    """
    return (
        3.0 * awaiting
        + 2.0 * (1.0 - float(world.sector_explored_pct[k]))
        + 1.5 * float(world.sector_hazard_known[k])
        + (1.0 if world.sector_abandoned[k] or world.sector_priority[k] != 1 else 0.0)
    )


def _bar(frac: float, width: int = 5) -> str:
    filled = int(round(max(0.0, min(1.0, frac)) * width))
    return "#" * filled + "." * (width - filled)


def build(world, executor, tracker, events, last_directives=None) -> str:
    """The user-turn text. Compact on purpose: this runs every 6 s on a local model."""
    lines: list[str] = []
    open_reports = tracker.open_reports() if tracker else []
    # Distinct victims, not resolved *reports*: several reports routinely resolve onto the
    # same casualty, and counting reports reported 48 rescues out of a possible 8.
    confirmed = len({r.victim for r in tracker.resolved_victims()}) if tracker else 0
    lines.append(
        f"MISSION t={world.t:.0f}s / {world.scn.mission_duration_s:.0f}s   "
        f"DELIVERED {world.victims_rescued}   CONFIRMED {confirmed}   "
        f"AWAITING INVESTIGATION {len(open_reports)}   "
        f"ROBOTS {int((world.status <= 1).sum())}/{world.n}   "
        f"IN CONTACT {int(world.in_comms.sum())}"
    )
    lines.append("  (how many casualties exist is unknown -- these are your own findings)")

    lines.append("")
    lines.append("SECTORS  id  explored  hazard  awaiting  priority")
    # Open contacts only. Resolved ones are history: they are no longer work, and
    # including them showed 24 "contacts" in a sector holding at most a handful.
    per_sector: dict[str, int] = {}
    for r in open_reports:
        sid = world._sector_at((float(r.pos[0]), float(r.pos[1])))
        per_sector[sid] = per_sector.get(sid, 0) + 1
    ranked = sorted(
        range(len(world.sector_ids)),
        key=lambda k: (-_interest(world, k, per_sector.get(world.sector_ids[k], 0)), k),
    )
    shown, rest = ranked[:MAX_SECTOR_ROWS], ranked[MAX_SECTOR_ROWS:]
    for k in sorted(shown):
        sid = world.sector_ids[k]
        expl = float(world.sector_explored_pct[k])
        haz = float(world.sector_hazard_known[k])
        found = per_sector.get(sid, 0)
        flag = " ABANDONED" if world.sector_abandoned[k] else ""
        lines.append(
            f"  {sid:>3}  {_bar(expl)} {expl * 100:3.0f}%  "
            f"{_bar(haz)} {haz * 100:3.0f}%  {found:>2}  "
            f"{['high', 'normal', 'low'][int(world.sector_priority[k])]}{flag}"
        )
    if rest:
        quiet = float(np.mean([world.sector_explored_pct[k] for k in rest]))
        lines.append(f"  ({len(rest)} further sectors omitted: quieter, "
                     f"{quiet * 100:.0f}% explored on average, no contacts awaiting)")

    lines.append("")
    lines.append("ROBOTS BY LANE   active  idle  in-contact  avg battery")
    for lane in LANES:
        sel = world.actuator == LANES.index(lane)
        alive = sel & (world.status <= 1)
        idle = sum(
            1 for i in range(world.n)
            if sel[i] and alive[i] and executor.assignment[i] is None
        )
        batt = float(world.battery[alive].mean()) if alive.any() else 0.0
        lines.append(
            f"  {LANE_LABEL.get(lane, lane):<8} {int(alive.sum()):>3} active  {idle:>3} idle  "
            f"{int(world.in_comms[sel].sum()):>3} in contact  {batt:.2f}"
        )
    lines.append(
        "  chassis mix: " + ", ".join(
            f"{c} {int((world.chassis == i).sum())}" for i, c in enumerate(CHASSIS)
        ) + "  (wheeled fastest, legged reaches roughest ground)"
    )

    recent = [e for e in (events.events[-80:] if events else [])
              if e.get("kind") not in NOISE_EVENTS][-6:]
    if recent:
        lines.append("")
        lines.append("RECENT")
        for ev in recent:
            lines.append(f"  {ev.get('t', 0):6.1f}s  {ev.get('text', '')}")

    if last_directives:
        lines.append("")
        lines.append("YOUR PREVIOUS DIRECTIVES")
        for d in last_directives:
            lines.append(f"  {d['sector']}: {d['priority']} / {d['action']}")

    return "\n".join(lines)
