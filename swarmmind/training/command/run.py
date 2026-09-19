"""Train the commander: `make train-command`.

CMA-ES over `CommandParams` -- seven numbers deciding how territory is valued and cut --
scored on whole missions across **procedurally generated maps**. Not MAP-Elites: there is
no diversity axis worth illuminating here. There is one commander and it should be the
best one, so this is plain optimisation with an archive of the best-so-far.

Sized from measurement. A commander evaluation is ~29 s per map and the default is three
maps, so ~86 s each, ~290/hour on seven workers. An overnight run of 10 hours is roughly
2,900 evaluations -- for a 7-dimensional search that is a long run, not a short one
(CMA-ES's default population for n=7 is 4 + 3*ln 7 ~ 9).

**Checkpoints and resumes every generation.** Stopping and restarting must lose at most
one generation, which means saving the *search state* and not merely the best answer:
CMA-ES carries a covariance matrix, a step size and a distribution mean, and a run that
reloads only `best.npz` throws all of that away and starts again from the hand-set
baseline. Worse, it would then overwrite a good `best.npz` with a fresh run's worse one.
`state.pkl` holds the scheduler itself; `best` is only ever replaced by something better.
"""

from __future__ import annotations

import argparse
import json
import os
import pickle
import time
from pathlib import Path

import numpy as np
from ribs.archives import GridArchive
from ribs.emitters import EvolutionStrategyEmitter
from ribs.schedulers import Scheduler

from ...nodes.command import CommandParams
from ..progress import Progress, fmt_duration
from .evaluate import HELD_OUT_MAP_SEEDS, TRAIN_MAP_SEEDS, evaluate
from .tripwire import Tripwires, render

#: (low, high) for each parameter, in the order `CommandParams` declares them. CMA-ES
#: searches the unit cube and these decode it, so a bound change does not invalidate a
#: stored solution -- the *vector* is what is stored, already decoded.
#: **Widened after run 1 pinned against them.** 121 generations settled on
#: `squadron_size` 155.4 against a ceiling of 160.0 -- an optimum pressed flat against an
#: artificial wall, which means the wall was choosing the answer rather than the search.
#: pyribs also warned that the ES was resampling >100 times for bounds handling. The
#: headroom goes where the pressure was; the rest is loosened only a little, because wide
#: bounds cost sample efficiency and the budget is generations, not evaluations.
BOUNDS: list[tuple[float, float]] = [
    # Run 1 pinned at 155.4 against a 160 ceiling, so this was widened to (8, 240). Run 2
    # then pinned at 12.3 against the *floor* -- the objective rewrite flipped the policy
    # from "one squadron, do not partition" to "many small squadrons", and it wants finer
    # grain than 8 allows. Floor dropped to 4; a squadron below that is a patrol, not a
    # unit, so this is the end of the useful range rather than another arbitrary wall.
    (4.0, 240.0),     # squadron_size
    (5.0, 120.0),     # rebalance_s
    (0.20, 0.99),     # done_frac
    (0.0, 4.0),       # w_unexplored
    (0.0, 4.0),       # w_contacts
    (0.0, 4.0),       # w_hazard
    (0.0, 4.0),       # w_distance
]
N_PARAMS = len(BOUNDS)

#: CMA-ES's default population for 7 dimensions is 4 + 3·ln 7 ≈ 9, which is where this
#: started. Raised to 12 to match the worker count: with a batch of 9 only 9 workers ever
#: had work, so a 12-worker pool idled a quarter of itself for the whole run. A larger
#: population also averages more samples into each covariance update, which is worth
#: having on an objective this noisy -- paid for in proportionally fewer generations.
BATCH_SIZE = 12
SIGMA0 = 0.18
CHECKPOINT = Path("runs/command")

#: Generations between held-out checks. One check is a full evaluation on maps the search
#: never sees, so it is pure overhead -- ~1 generation's cost every 10, about 9%. That is
#: the price of being able to tell improvement from reward hacking, and run 1 is what not
#: paying it costs.
HELD_OUT_EVERY = 10

#: Held-out maps sampled at each check. The full gate uses all ten; three is enough for a
#: trend and keeps the overhead at one generation rather than three.
HELD_OUT_CHECK_MAPS = HELD_OUT_MAP_SEEDS[:3]


def _vs(best: float, base: float) -> str:
    """How `best` compares to the baseline, in a form that survives negative scores.

    A ratio stops meaning anything once the objective can go below zero, and since
    `command_score` subtracts every unrescued casualty the baseline routinely does.
    `best / max(base, 1e-9)` then divides by 1e-9 and prints `-652406686652.36x`. Above
    zero a ratio is still the more readable summary, so keep it there and fall back to a
    signed delta otherwise.
    """
    return f"{best / base:.2f}x" if base > 0 else f"{best - base:+.2f}"


def decode(x) -> CommandParams:
    x = np.clip(np.asarray(x, dtype=float), 0.0, 1.0)
    return CommandParams.from_vector(
        [lo + (hi - lo) * v for v, (lo, hi) in zip(x, BOUNDS, strict=True)]
    )


def encode(p: CommandParams) -> np.ndarray:
    return np.array([(v - lo) / (hi - lo)
                     for v, (lo, hi) in zip(p.as_vector(), BOUNDS, strict=True)])


def _score(x, map_seeds) -> float:
    return evaluate(decode(x), tuple(map_seeds)).score


def _worker(job) -> tuple[int, float]:
    i, x, map_seeds = job
    # The index rides along so `imap_unordered` results can be put back in submission
    # order; CMA-ES matches scores to the candidates it asked for by position.
    return i, _score(x, map_seeds)


def _score_batch(xs, map_seeds, pool, on_done=None) -> list[float]:
    """A generation is 9 independent missions; running them serially wastes seven cores.

    ~86 s per evaluation means a serial generation is 13 minutes and an overnight run gets
    ~45 of them. Spread over seven workers it is ~2 minutes and ~300 -- which is the
    difference between CMA-ES adapting a covariance matrix and merely sampling.
    """
    jobs = [(i, np.asarray(x), tuple(map_seeds)) for i, x in enumerate(xs)]
    out: list[float] = [0.0] * len(jobs)
    stream = ((_worker(j) for j in jobs) if pool is None
              else pool.imap_unordered(_worker, jobs, chunksize=1))
    for k, (i, score) in enumerate(stream, start=1):
        out[i] = score
        if on_done is not None:
            on_done(k, len(jobs))
    return out


def _init_worker() -> None:
    """One BLAS thread per worker. Seven workers each opening a thread per core is 56
    threads on 8 cores, and they spend their time descheduling one another (M-30)."""
    for v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
              "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
        os.environ[v] = "1"


def build(seed: int = 42) -> Scheduler:
    # A 1x1 archive: pyribs wants one, and this is optimisation rather than illumination.
    archive = GridArchive(solution_dim=N_PARAMS, dims=(1, 1),
                          ranges=[(0.0, 1.0), (0.0, 1.0)], seed=seed)
    emitter = EvolutionStrategyEmitter(
        archive, x0=encode(CommandParams()), sigma0=SIGMA0,
        batch_size=BATCH_SIZE, bounds=[(0.0, 1.0)] * N_PARAMS, seed=seed,
    )
    return Scheduler(archive, [emitter])


def save(path: Path, sched, best_x, best_score: float, meta: dict) -> None:
    """Atomic-ish: state first, then best, then meta. Interrupted mid-save, the worst case
    is a state one generation ahead of the recorded best, which resume tolerates."""
    path.mkdir(parents=True, exist_ok=True)
    with open(path / "state.pkl", "wb") as fh:
        pickle.dump({"sched": sched, "best_x": np.asarray(best_x),
                     "best": float(best_score), "gen": int(meta.get("generation", 0)),
                     "baseline": meta.get("baseline")}, fh)
    np.savez_compressed(path / "best.npz", x=np.asarray(best_x),
                        params=decode(best_x).as_vector())
    (path / "meta.json").write_text(json.dumps(
        {**meta, "best_score": best_score,
         "params": dict(zip(CommandParams.__dataclass_fields__,
                            decode(best_x).as_vector().round(4).tolist(), strict=True))},
        indent=2))


def load(path: Path):
    """Previous run's state, or None. Resume is the default -- losing a night's search to
    a forgotten flag is precisely the failure this exists to prevent."""
    f = Path(path) / "state.pkl"
    if not f.exists():
        return None
    with open(f, "rb") as fh:
        return pickle.load(fh)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="evolve the commander")
    ap.add_argument("--hours", type=float, default=10.0, help="wall-clock budget")
    ap.add_argument("--generations", type=int, default=0, help="0 = run until --hours")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1),
                    help="parallel evaluations; 1 = serial")
    ap.add_argument("--maps", type=int, nargs="+", default=list(TRAIN_MAP_SEEDS))
    ap.add_argument("--out", type=Path, default=CHECKPOINT)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--restart", action="store_true",
                    help="discard any saved state and start over (resume is the default)")
    ap.add_argument("--held-out-every", type=int, default=HELD_OUT_EVERY,
                    help="generations between held-out checks; 0 disables the tripwires")
    ap.add_argument("--stop-on-trip", action="store_true",
                    help="abort when a tripwire fires (default: warn and keep going)")
    args = ap.parse_args(argv)

    pool = None
    if args.workers > 1:
        # `spawn` re-executes __main__, so this must be run as a script or `python -m`.
        # The same trap that fork-bombed the MAP-Elites benchmark (M-30).
        import multiprocessing as mp
        import sys

        if getattr(sys.modules.get("__main__"), "__file__", None) is None:
            raise RuntimeError("run this as a script or `python -m`, not from a REPL")
        ctx = mp.get_context("spawn")
        pool = ctx.Pool(args.workers, initializer=_init_worker)

    base = CommandParams()
    t0 = time.perf_counter()

    state = None if args.restart else load(args.out)
    if state is not None:
        sched = state["sched"]
        best_x, best, gen0 = state["best_x"], state["best"], state["gen"]
        base_score = state.get("baseline") or _score(encode(base), args.maps)
        print(f"  resumed from {args.out}/state.pkl at generation {gen0}, "
              f"best {best:.2f} (baseline {base_score:.2f})")
    else:
        sched = build(args.seed)
        base_score = _score(encode(base), args.maps)
        best_x, best, gen0 = encode(base), base_score, 0
        print(f"  hand-set baseline: {base_score:.2f}  "
              f"({time.perf_counter()-t0:.0f}s for {len(args.maps)} maps)")
    print(f"  budget {args.hours:.1f} h   batch {BATCH_SIZE}   maps {args.maps}",
          flush=True)

    budget_s = args.hours * 3600.0
    deadline = time.perf_counter() + budget_s
    gen = gen0
    # Two ways to bound this run, so two things to show progress against: an explicit
    # generation count if one was given, otherwise the wall-clock budget. Measuring the
    # bar in seconds makes Progress's linear ETA reduce to the time remaining, which is
    # exactly right for a deadline-bounded run.
    by_gens = bool(args.generations)
    wires = Tripwires(param_names=tuple(CommandParams.__dataclass_fields__),
                      last_improved_gen=gen0)
    fired: list[dict] = []
    #: Sentinel: never equal to a real `best_x`, so the first check always runs.
    last_checked_x = np.full(N_PARAMS, np.nan)
    # Counted in *evaluations* rather than generations: a generation is ~2 minutes, so a
    # bar with one tick per generation shows 0% until it shows 100% and never has two
    # samples to build an ETA from. The time-budget mode is already fine-grained.
    total = ((args.generations - gen0) * BATCH_SIZE) if by_gens else int(budget_s)
    progress = Progress(max(total, 1), label="command")
    try:
        while time.perf_counter() < deadline:
            if args.generations and gen >= args.generations:
                break
            gen += 1

            def tick(k, n, g=gen, b=best):
                # `b` is bound per generation on purpose: `best` cannot change until
                # this generation's scores are in, so the value at entry is the right
                # one to display and binding it keeps the closure free of loop state.
                #
                # The clock, by contrast, is read live -- capturing it at generation
                # start would freeze the bar for the ~2 minutes a generation takes.
                pos = ((g - 1 - gen0) * BATCH_SIZE + k if by_gens
                       else int(min(time.perf_counter() - t0, budget_s)))
                progress.update(pos, suffix=f"gen {g}  eval {k}/{n}  best {b:.2f}")

            xs = sched.ask()
            scores = _score_batch(xs, args.maps, pool, on_done=tick)
            # pyribs maximises `objective`; the measures are unused here.
            sched.tell(np.array(scores), np.full((len(xs), 2), 0.5))
            i = int(np.argmax(scores))
            # Strictly better only. A resumed run must never replace a good result with a
            # worse one just because it happens to be more recent.
            if scores[i] > best:
                best, best_x = scores[i], xs[i]
                wires.last_improved_gen = gen
                wires.has_search_result = True
            elapsed = (time.perf_counter() - t0) / 60.0
            progress.update(((gen - gen0) * BATCH_SIZE) if by_gens
                            else int(min(elapsed * 60, budget_s)))
            progress.log(f"  gen {gen:>4}  best {best:9.2f}  (baseline {base_score:.2f}, "
                         f"{_vs(best, base_score)})  "
                         f"gen-best {scores[i]:9.2f}  {fmt_duration(elapsed * 60)}")
            # --- tripwires ------------------------------------------------------
            # Scored on maps the search has never optimised against. This is the only
            # measurement that can tell "the policy got better" from "the objective got
            # gamed", which is exactly what run 1 could not distinguish.
            #
            # Skipped when `best_x` has not moved since the last check: the evaluation is
            # deterministic, so it would spend ~2 minutes recomputing a number we already
            # have. Run 3 plateaued for 47 generations and logged four identical held-out
            # scores, which is both wasted compute and a constant series that no trend can
            # be fitted to.
            if (args.held_out_every and gen % args.held_out_every == 0
                    and not np.array_equal(best_x, last_checked_x)):
                held_out_score = _score(best_x, HELD_OUT_CHECK_MAPS)
                last_checked_x = np.asarray(best_x).copy()
                wires.record_held_out(gen, best, held_out_score)
                progress.log(f"  {'':>8}held-out {held_out_score:9.2f} on "
                             f"{list(HELD_OUT_CHECK_MAPS)}  (train best {best:.2f})")

            trips = wires.check(gen, best_x)
            if trips:
                progress.log(render(trips))
                fired = [{"generation": gen, "name": t.name, "detail": t.detail}
                         for t in trips]

            save(args.out, sched, best_x, best,
                 {"generation": gen, "elapsed_min": round(elapsed, 1),
                  "baseline": base_score, "maps": args.maps,
                  "held_out": list(HELD_OUT_MAP_SEEDS),
                  "held_out_history": wires.history,
                  "tripwires_firing": fired})

            if trips and args.stop_on_trip:
                progress.log("  --stop-on-trip set; aborting.")
                break
    finally:
        # Even on Ctrl-C: the bar must not be left painted under the shell prompt.
        progress.close()

    print(f"\n  {gen} generations, best {best:.2f} vs baseline {base_score:.2f} "
          f"({_vs(best, base_score)})")
    if pool is not None:
        pool.terminate()
        pool.join()
    print(f"  -> {args.out}/best.npz   evaluate with: make gate-command")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
