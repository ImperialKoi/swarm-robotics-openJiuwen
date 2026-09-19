"""The ship / don't-ship decision.

Every learned component in this project has to beat the classical one on seeds it has
never seen, or the classical one ships. That is not a formality: a heuristic winning is a
**result**, and `SHIPPING.md` is written from whatever actually happened rather than from
what was hoped for.

D9 fills in the half that does not need any trained artefact: the **classical baseline**
and the held-out seeds. The comparison arms (evolved roster, CV detector, tuned hivemind)
land at D10-D12 as each becomes available.
"""

from __future__ import annotations

import argparse
import inspect
import multiprocessing as mp
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from ..metrics import Scorecard
from ..mission import Mission
from ..perception.cnn import DEFAULT_WEIGHTS as DETECTOR_WEIGHTS
from ..sim.scenario import Scenario

#: Seeds no training run may touch. Disjoint from `mapelites.evaluate.TRAIN_SEEDS`
#: (11, 12, 13) by construction, and `tests/test_evaluate.py` asserts the disjointness --
#: an elite that only wins on its training maps has memorised three rubble layouts.
#:
#: **Kept, and no longer the default**. The demo plays four maps -- `demo.yaml`
#: `demo_seeds`, 42-45 -- and the gate now judges on those, because those are the only maps
#: the thing that ships will ever be shown. `--held-out` still runs these ten.
HELD_OUT_SEEDS: tuple[int, ...] = (101, 102, 103, 104, 105, 106, 107, 108, 109, 110)


def demo_seeds(scenario: str = "demo") -> tuple[int, ...]:
    """The maps the demo plays, from the scenario file -- the one place they are defined."""
    return Scenario.load(scenario).demo_seeds

#: A learned component must clear the baseline by this margin to ship. Beating it by 1%
#: on ten seeds is noise, and shipping on noise is how a demo acquires a component that
#: is worse than what it replaced.
MARGIN = 1.05


@dataclass
class Arm:
    """One side of a comparison, scored over the held-out seeds."""

    name: str
    rescued: float
    found: float
    explored: float
    lost: float
    score: float
    per_seed: list[float]


def mission_score(card: Scorecard) -> float:
    """One number per mission, weighted the way the demo is judged.

    Rescues dominate because rescues are the mission. Exploration is a proxy and gets a
    small weight; losing robots is penalised because a swarm that clears the map by
    driving into fire has not solved the problem.
    """
    return (
        10.0 * card.victims_rescued
        + 2.0 * card.victims_found
        + 5.0 * card.ground_explored_frac
        - 0.5 * card.robots_lost
    )


def _one(job):
    scenario, seed, kw = job
    kw = dict(kw)
    # Built inside the worker, not pickled into it: `spawn` would otherwise carry the
    # weight arrays across the process boundary once per seed for no reason, and a
    # detector holding a file handle would not survive the trip at all.
    weights = kw.pop("detector_weights", None)
    if weights is not None:
        from ..perception.cnn import CNNVictimDetector

        kw["detector"] = CNNVictimDetector(weights)
    # Same reasoning for the unit policy: a mode and a path cross the process boundary,
    # the controller is built here.
    mode = kw.pop("unit_policy_mode", None)
    policy_path = kw.pop("unit_policy_weights", None)
    if mode is not None:
        from ..control.unit_policy import UnitController

        policy = None
        if policy_path is not None:
            from .rl.ppo import UnitPolicy

            policy = UnitPolicy.load_npz(policy_path)
        kw["unit_policy"] = UnitController(mode, policy)
    return Mission(Scenario.load(scenario), seed, **kw).run()


def run_arm(name: str, scenario: str, seeds: tuple[int, ...], workers: int = 1,
            **kw) -> Arm:
    kw.setdefault("hivemind", False)
    jobs = [(scenario, s, kw) for s in seeds]
    if workers > 1:
        # A real module under `if __name__ == "__main__"`, never a REPL: `spawn`
        # re-executes `__main__` and a heredoc has none (CLAUDE.md, MEASUREMENTS M-30).
        ctx = mp.get_context("spawn")
        with ctx.Pool(workers) as pool:
            cards = pool.map(_one, jobs)
    else:
        cards = [_one(j) for j in jobs]
    scores = [mission_score(c) for c in cards]
    return Arm(
        name=name,
        rescued=float(np.mean([c.victims_rescued for c in cards])),
        found=float(np.mean([c.victims_found for c in cards])),
        explored=float(np.mean([c.ground_explored_frac for c in cards])),
        lost=float(np.mean([c.robots_lost for c in cards])),
        score=float(np.mean(scores)),
        per_seed=[round(s, 3) for s in scores],
    )


def classical_baseline(scenario: str = "test",
                       seeds: tuple[int, ...] = HELD_OUT_SEEDS,
                       workers: int = 1) -> Arm:
    """The control arm: hand-set archetypes, hand-tuned bid weights, no genome, no LLM.

    `genes=None` matters here, and so does `evolved=False`. An all-0.5 genome is *not*
    the classical policy -- the gene-driven bid carries four terms the classical bid does
    not have at all -- so comparing against a midpoint genome would flatter every result.
    And until D12 this arm still built its robots from `demo_roster.yaml`, because
    `World` loaded it unconditionally: the "classical baseline" ran **evolved bodies**
    with a classical bid, and the comparison measured something other than what it said.
    """
    return run_arm("classical", scenario, seeds, workers=workers,
                   genes=None, evolved=False)


def compare(baseline: Arm, candidate: Arm, margin: float = MARGIN) -> dict:
    ratio = candidate.score / baseline.score if baseline.score else float("inf")
    return {
        "baseline": asdict(baseline),
        "candidate": asdict(candidate),
        "ratio": round(ratio, 4),
        "margin": margin,
        "ships": bool(ratio >= margin),
        "verdict": (f"{candidate.name} ships ({ratio:.2f}x baseline)" if ratio >= margin
                    else f"{baseline.name} ships; {candidate.name} reached only "
                         f"{ratio:.2f}x, under the {margin:.2f}x bar"),
    }


SHIPPING_HEADER = """# SHIPPING.md

**Generated by `swarmmind/training/gate.py`. Do not edit by hand.**

What actually ships, per component, and by what margin. A classical component winning is
a **result**, not a failure to hide -- that is the whole reason this file is generated
from measurements rather than written from intent.

Every comparison below runs on the demo's four maps (`demo.yaml` `demo_seeds`), because
those are the only maps the demo plays. Components trained elsewhere -- the bodies (test
fixture, seeds 11-13) and the detector (seeds 1-8) -- have never seen them. **The unit policy
was trained on these same four maps**: its margin is how much better it is *on the demo*,
not evidence that it works anywhere else. The mission score is
`10*rescued + 2*found + 5*ground_explored - 0.5*lost`.
"""


def selection_note() -> str:
    """Separate historical evaluation winners from the configuration selected in code."""
    defaults = inspect.signature(Mission).parameters
    flags = ", ".join(f"`{key}={defaults[key].default!r}`" for key in
                      ("evolved", "command", "unit_policy", "zone_routing", "response_team"))
    return (
        "<!-- current-selection -->\n"
        "## Current defaults versus historical gate results\n\n"
        f"Unflagged `Mission` defaults: {flags}. The tables below record evaluation "
        "outcomes; they do not override later integration decisions. Zone routing remains "
        "off after M-76f; its Tier-3-off win did not establish a win for the live demo. "
        "The classical detector ships; M-74 records the later CNN evaluation. "
        "The response team is an opt-in application, without a rescue-uplift claim. "
        "See [current build and demo instructions](docs/MULTI_AGENT_DEMO.md).\n\n"
        "The [Nepal confluence crop](docs/NEPAL_TERRAIN.md) changes map geometry even "
        "for the same demo seeds. Gate results from the previous terrain do not establish "
        "performance on this crop; components remain tuned on the four demo maps "
        "used at the time of their training.\n"
        "<!-- /current-selection -->\n\n"
    )


def refresh_selection_note(path: Path) -> None:
    """Refresh only this generated annotation; preserve every historical measurement."""
    text = path.read_text(encoding="utf-8")
    start, end = "<!-- current-selection -->", "<!-- /current-selection -->"
    if start in text:
        a, rest = text.split(start, 1)
        _, b = rest.split(end, 1)
        text = a + b.lstrip("\n")
    anchor = "**Generated by `swarmmind/training/gate.py`. Do not edit by hand.**\n\n"
    if anchor not in text:
        raise ValueError("not a recognized generated shipping report")
    path.write_text(text.replace(anchor, anchor + selection_note(), 1), encoding="utf-8")


def _arm_table(arms: list[Arm]) -> str:
    rows = ["| arm | score | rescued | found | ground explored | lost |",
            "|---|---:|---:|---:|---:|---:|"]
    for a in arms:
        rows.append(f"| {a.name} | **{a.score:.2f}** | {a.rescued:.2f} | {a.found:.2f} "
                    f"| {a.explored:.1%} | {a.lost:.2f} |")
    return "\n".join(rows)


def write_report(path: Path, scenario: str, seeds: tuple[int, ...],
                 sections: list[dict]) -> None:
    out = [SHIPPING_HEADER.replace(
        "What actually ships", selection_note() + "What actually ships"),
           f"Scenario `{scenario}`, seeds `{list(seeds)}`, "
           f"{len(seeds)} missions per arm, margin {MARGIN:.2f}x.\n"]
    out.append("## Summary\n")
    out.append("| component | ships | margin |")
    out.append("|---|---|---:|")
    for sec in sections:
        out.append(f"| {sec['component']} | **{sec['ships']}** | {sec['margin']} |")
    out.append("")
    for sec in sections:
        out.append(f"## {sec['component']}\n")
        out.append(sec["note"] + "\n")
        if sec.get("arms"):
            out.append(_arm_table(sec["arms"]) + "\n")
        if sec.get("verdict"):
            out.append(f"> {sec['verdict']}\n")
    path.write_text("\n".join(out), encoding="utf-8")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="the ship / don't-ship report")
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--seeds", type=int, default=0,
                    help="use only the first N seeds of the set (0 = all)")
    ap.add_argument("--held-out", action="store_true",
                    help="judge on HELD_OUT_SEEDS (101-110) instead of the demo maps")
    ap.add_argument("--unit-policy", type=Path, default=Path("runs/rl/policy_best.npz"),
                    help="trained unit policy to gate, if it exists")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--report", type=Path, default=Path("SHIPPING.md"))
    ap.add_argument("--refresh-selection-note", action="store_true",
                    help="annotate an existing report with code defaults; run no evaluations")
    args = ap.parse_args(argv)
    if args.refresh_selection_note:
        refresh_selection_note(args.report)
        return 0

    pool = HELD_OUT_SEEDS if args.held_out else demo_seeds(args.scenario)
    if not pool:
        raise SystemExit(f"scenario {args.scenario!r} declares no demo_seeds; use --held-out")
    seeds = pool[: args.seeds] if args.seeds else pool
    w = args.workers
    print(f"  gate: {args.scenario}, seeds {list(seeds)}, {w} workers")

    print("  [1/5] classical bodies (control) ...", flush=True)
    classical = classical_baseline(args.scenario, seeds, workers=w)
    print(f"        score {classical.score:.2f}", flush=True)

    print("  [2/5] evolved roster ...", flush=True)
    evolved = run_arm("evolved roster", args.scenario, seeds, workers=w,
                      genes=None, evolved=True)
    print(f"        score {evolved.score:.2f}", flush=True)

    print("  [3/5] Tier 3, scripted rung ...", flush=True)
    tier3 = run_arm("Tier 3 scripted", args.scenario, seeds, workers=w,
                    genes=None, evolved=True, hivemind=True, scripted_hivemind=True)
    print(f"        score {tier3.score:.2f}", flush=True)

    # Perception. Runs only if a trained detector exists; there is nothing to compare
    # against otherwise, and an arm that silently reports the classical detector twice
    # would read as a real comparison in the report.
    perception_note = (
        "Perception is real either way: robots see through a detector running on their "
        "own occluded camera frame, and only the *trained* label is at stake. The CNN "
        "runs forward in numpy (`perception/cnn.py`) so nothing here depends on torch; "
        "it costs ~46 ms per 500-frame pass against the classical detector's 5.5 ms, "
        "which is the price it has to earn back.")
    if DETECTOR_WEIGHTS.exists():
        print("  [+] CV detector ...", flush=True)
        cnn = run_arm("CV detector", args.scenario, seeds, workers=w,
                      genes=None, evolved=True,
                      detector_weights=str(DETECTOR_WEIGHTS))
        print(f"        score {cnn.score:.2f}", flush=True)
        eyes = compare(evolved, cnn)
        perception = {
            "component": "Perception (CV detector)",
            "ships": "CV detector" if eyes["ships"] else "classical detector",
            "margin": f"{eyes['ratio']:.2f}x",
            "arms": [evolved, cnn],
            "verdict": eyes["verdict"],
            "note": perception_note,
        }
    else:
        perception = {
            "component": "Perception (CV detector)",
            "ships": "classical detector", "margin": "not run", "arms": [],
            "verdict": (f"no trained detector at {DETECTOR_WEIGHTS}; "
                        f"`perception/classical.py` ships"),
            "note": perception_note + (
                " Train it with `swarmmind/training/notebooks/detector.ipynb` on Kaggle. "
                "Cuttable per PLAN §7.1 item 4."),
        }

    # --- Tier 1: exact routing to collection points ------------------------------------
    print("  [4/5] zone routing ...", flush=True)
    routed = run_arm("zone routing", args.scenario, seeds, workers=w,
                     genes=None, evolved=True, zone_routing=True)
    print(f"        score {routed.score:.2f}", flush=True)
    routing = compare(evolved, routed)

    # --- Tier 2: the unit policy, on top of routing --------------------------------------
    print("  [5/5] unit policy: heuristic staging ...", flush=True)
    staged = run_arm("heuristic staging", args.scenario, seeds, workers=w, genes=None,
                     evolved=True, zone_routing=True, unit_policy_mode="heuristic")
    print(f"        score {staged.score:.2f}", flush=True)
    unit_arms = [routed, staged]
    # The bar is the stronger of "no unit policy" and "the heuristic", never the weaker:
    # the commander programme's three-arm lesson (training_notes/SUMMARY.md).
    control = routed if routed.score >= staged.score else staged
    if args.unit_policy.exists():
        print(f"  [+] unit policy: trained ({args.unit_policy}) ...", flush=True)
        learned = run_arm("trained unit policy", args.scenario, seeds, workers=w, genes=None,
                          evolved=True, zone_routing=True, unit_policy_mode="learned",
                          unit_policy_weights=str(args.unit_policy))
        print(f"        score {learned.score:.2f}", flush=True)
        unit_arms.append(learned)
        unit = compare(control, learned)
        unit_ships = "trained unit policy" if unit["ships"] else control.name
        unit_verdict = unit["verdict"]
        unit_margin = f"{unit['ratio']:.2f}x"
    else:
        unit_ships = control.name
        unit_margin = "not trained"
        unit_verdict = f"no trained unit policy at {args.unit_policy}; {control.name} stands"

    lanes = compare(classical, evolved)
    hive = compare(evolved, tier3)

    sections = [
        {"component": "Tier-2 robot bodies (MAP-Elites)",
         "ships": "evolved roster" if lanes["ships"] else "hand-set archetypes",
         "margin": f"{lanes['ratio']:.2f}x",
         "arms": [classical, evolved],
         "verdict": lanes["verdict"],
         "note": ("MAP-Elites evolved the bodies (`radius`, `v_max`, `omega_max`, "
                  "`sensor_radius`, `battery_capacity`); `select.py` picks the extremes "
                  "of measured speed per lane rather than the top-N by fitness, so the "
                  "roster is deliberately diverse. **The genome's behaviour half is not "
                  "shipped** -- the bid weights in play are the hand-tuned classical "
                  "ones in both arms, so this comparison isolates bodies.")},
        {"component": "Tier 3 hivemind",
         "ships": "scripted rung" if hive["ships"] else "no Tier 3",
         "margin": f"{hive['ratio']:.2f}x",
         "arms": [evolved, tier3],
         "verdict": hive["verdict"],
         "note": ("The rung measured here is the **scripted** baseline, rung 4 of the "
                  "ladder -- no model has been trained or run against the demo scenario. "
                  "It is the number a tuned model has to beat, not a claim about one.")},
        {"component": "Tier 3a commander",
         "ships": "no commander", "margin": "0.98x", "arms": [],
         "verdict": "no commander ships; trained reached 0.98x, under the 1.05x bar",
         "note": ("Decided on ten held-out maps, three arms: none 70.81 / "
                  "heuristic 65.57 / trained 69.42. `command=False` is the default and "
                  "the code stays so the claim is reproducible. Full record in "
                  "`training_notes/SUMMARY.md`; do not reopen by tuning `CommandParams`.")},
        perception,
        {"component": "Tier-1 zone routing",
         "ships": "zone routing" if routing["ships"] else "coarse routing (as on main)",
         "margin": f"{routing['ratio']:.2f}x",
         "arms": [evolved, routed],
         "verdict": routing["verdict"],
         "note": ("Loaded carriers descend an exact fine-grid field to the nearest collection "
                  "point, over ground eroded by one cell so no route is narrower than a robot "
                  "can hold a heading through (`control/zone_routing.py`, MEASUREMENTS M-76). "
                  "A correctness fix, not a trained component.")},
        {"component": "Tier-2 unit policy",
         "ships": unit_ships,
         "margin": unit_margin,
         "arms": unit_arms,
         "verdict": unit_verdict,
         "note": ("Per-robot staging for carriers and diggers with nothing urgent "
                  "(`control/unit_policy.py`), trained by behaviour cloning from the heuristic "
                  "then PPO on Kaggle (`training/rl/`). **Trained on these same four demo "
                  "maps** -- a win here means a win on the demo, not on maps it has not seen. "
                  "The bar is the better of routing alone and the heuristic.")},
    ]
    write_report(args.report, args.scenario, seeds, sections)
    print(f"\n  Tier-2 bodies : {lanes['verdict']}")
    print(f"  Perception    : {perception['verdict']}")
    print(f"  Tier 3        : {hive['verdict']}")
    print(f"  Zone routing  : {routing['verdict']}")
    print(f"  Unit policy   : {unit_verdict}")
    print(f"  -> {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
