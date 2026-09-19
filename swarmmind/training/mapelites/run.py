"""The MAP-Elites run: `make evolve`.

Illumination, not optimisation. The output is not "the best robot" -- it is an archive
with a good robot in *every* cell of (lane x measured speed), so the demo roster can show
four lanes and a real spread of fast-light against slow-tough bodies, and say truthfully
that the diversity was discovered rather than authored.

Sizing is measured, not assumed. At **1,418 evaluations/hour** on seven workers
([MEASUREMENTS.md M-30](../../../docs/MEASUREMENTS.md)) the original 300k-evaluation plan
was ~210 hours. The defaults here are a one-hour run; `--iterations` buys more.

Because the budget is ~1.4k evaluations rather than 300k, **sample efficiency is the whole
game** -- which is why the emitters are CMA-ES (`EvolutionStrategyEmitter`) and not
Gaussian mutation. Every restart it does not waste is a cell that gets filled.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
from ribs.archives import GridArchive
from ribs.emitters import EvolutionStrategyEmitter
from ribs.schedulers import Scheduler

from ...sim.robot import LANES
from ..progress import Progress, fmt_duration
from .evaluate import TRAIN_SEEDS
from .genome import GENE_INDEX, N_GENES, baseline
from .pool import EvaluationPool, n_workers

#: Archive shape: one bin per lane, ten bins of measured speed.
ARCHIVE_DIMS = (len(LANES), 10)

#: Measure ranges. Lane is exact.
#:
#: **The speed axis is calibrated to measurement, and getting it wrong costs coverage
#: twice over.** The first version spanned 0.0-1.8 m/s, chosen with a guess at headroom.
#: Across 52 elites from three runs, *no robot ever reached* the bottom bin or the top
#: two: measured mean speed spans 0.24-1.32 m/s. So 12 of 40 cells were unreachable by
#: construction, the archive could never exceed 70% coverage however good the search was,
#: and the seven usable bins were coarser than they needed to be. Recalibrated to the
#: observed span plus real headroom (MEASUREMENTS.md M-32).
MEASURE_RANGES = [(0.0, float(len(LANES))), (0.20, 1.45)]

#: **One emitter per lane, batch at the CMA-ES default.** Both numbers were wrong on the
#: first pass and both were costing the run outright (MEASUREMENTS.md M-32):
#:
#: * 5 emitters all starting from the all-0.5 midpoint means `actuator_g` starts at 0.5,
#:   which decodes to `gripper`. Measured on a fresh `ask()`: 87% of proposals were
#:   gripper or scoop and only 13% reached scout and relay. Two of four lanes were
#:   barely being searched at all. One emitter per lane, each seeded at its own lane's
#:   midpoint, fixes it at the source.
#: * Batch 30 is 2.5x the CMA-ES default population for 15 dimensions (4 + 3·ln 15 ≈ 12).
#:   With a ~1,400-evaluation budget, batch 30 buys **9 generations**; CMA-ES needs tens.
#:   Batch 15 buys ~23 generations an hour for the same spend.
N_EMITTERS = len(LANES)
BATCH_SIZE = 15
SIGMA0 = 0.15

CHECKPOINT = Path("runs/mapelites")


def lane_x0(lane_index: int) -> np.ndarray:
    """The midpoint genome, with `actuator_g` moved to the centre of one lane's band.

    `lane_of` is `floor(g * 4)`, so the centre of lane *k* is `(k + 0.5) / 4`. Starting an
    emitter here means its whole search neighbourhood decodes to that lane instead of
    leaking across boundaries into whichever lane the midpoint happened to land in.
    """
    g = baseline()
    g[GENE_INDEX["actuator_g"]] = (lane_index + 0.5) / len(LANES)
    return g


def build_scheduler(seed: int = 42) -> Scheduler:
    archive = GridArchive(solution_dim=N_GENES, dims=ARCHIVE_DIMS,
                          ranges=MEASURE_RANGES, seed=seed)
    # Each emitter starts from the hand-tuned midpoint *of its own lane*, rather than
    # from noise: with a 1.4k budget there is no time to rediscover a working robot from
    # scratch, and no budget to spend three lanes' worth of samples in one lane.
    emitters = [
        EvolutionStrategyEmitter(
            archive, x0=lane_x0(i), sigma0=SIGMA0, batch_size=BATCH_SIZE,
            # Genes are [0,1] by definition; the decode clamps anyway, but bounding the
            # search stops CMA-ES spending its covariance on regions that all decode to
            # the same clipped robot.
            bounds=[(0.0, 1.0)] * N_GENES,
            ranker="2imp", seed=seed + i,
        )
        for i in range(N_EMITTERS)
    ]
    return Scheduler(archive, emitters)


def save(archive: GridArchive, path: Path, meta: dict) -> None:
    """Checkpoint every iteration. A ten-hour run that dies at hour nine has nothing."""
    path.mkdir(parents=True, exist_ok=True)
    data = archive.data()
    np.savez_compressed(
        path / "archive.npz",
        solution=np.asarray(data["solution"]),
        objective=np.asarray(data["objective"]),
        measures=np.asarray(data["measures"]),
    )
    (path / "meta.json").write_text(json.dumps(meta, indent=2))


def seed_archive(archive: GridArchive, pool: EvaluationPool, n: int,
                 rng: np.random.Generator, progress: Progress | None = None,
                 base: int = 0) -> None:
    """Fill the archive with uniform random genomes before CMA-ES starts.

    All five emitters begin from the same midpoint, so the first generations sample one
    small neighbourhood and coverage stalls -- the first run filled 15 cells at iteration
    one and 16 by iteration three (MEASUREMENTS.md M-31). Random genomes are individually
    poor but land all over the measure space, which gives the emitters archive elites to
    restart from instead of circling the origin.
    """
    if n <= 0:
        return
    genomes = rng.random((n, N_GENES))
    cb = None
    if progress is not None:
        cb = lambda k, total: progress.update(base + k, suffix="seeding")  # noqa: E731
    evals = pool.evaluate(genomes, on_done=cb)
    archive.add(
        genomes,
        np.array([e.fitness for e in evals]),
        np.array([[e.lane + 0.5, e.mean_speed] for e in evals]),
    )
    st = archive.stats
    line = (f"  seeded with {n} random genomes -> {st.num_elites}/{archive.cells} cells "
            f"({st.coverage:.1%})")
    progress.log(line) if progress is not None else print(line, flush=True)


def run(iterations: int, workers: int, scenario: str, seeds: tuple[int, ...],
        out: Path, seed: int = 42, init_random: int = 0) -> GridArchive:
    scheduler = build_scheduler(seed)
    archive = scheduler.archive
    per_iter = N_EMITTERS * BATCH_SIZE
    print(f"  MAP-Elites: {iterations} iterations x {per_iter} evaluations "
          f"= {iterations * per_iter:,} total")
    print(f"  archive {ARCHIVE_DIMS[0]}x{ARCHIVE_DIMS[1]} = {archive.cells} cells   "
          f"workers {workers}   seeds {seeds}")

    t0 = time.perf_counter()
    # The bar spans every evaluation the run will perform, so the ETA is for the whole
    # job rather than for the current iteration -- which is the number you actually want
    # at minute forty of a ten-hour run.
    total_evals = init_random + iterations * per_iter
    with EvaluationPool(workers=workers, seeds=seeds, scenario=scenario) as pool, \
            Progress(total_evals, label="evolve") as progress:
        seed_archive(archive, pool, init_random, np.random.default_rng(seed),
                     progress, base=0)
        for it in range(1, iterations + 1):
            base = init_random + (it - 1) * per_iter
            genomes = scheduler.ask()
            evals = pool.evaluate(
                genomes,
                on_done=lambda k, total, b=base, i=it: progress.update(
                    b + k, suffix=f"it {i}/{iterations}"),
            )
            objective = np.array([e.fitness for e in evals])
            measures = np.array([[e.lane + 0.5, e.mean_speed] for e in evals])
            scheduler.tell(objective, measures)

            elapsed = time.perf_counter() - t0
            stats = archive.stats
            progress.log(f"  it {it:>4}/{iterations}  "
                         f"filled {stats.num_elites:>3}/{archive.cells}  "
                         f"({stats.coverage:5.1%})  "
                         f"max {stats.obj_max:8.3f}  mean {stats.obj_mean:8.3f}  "
                         f"qd {stats.qd_score:10.1f}  {fmt_duration(elapsed)}")
            save(archive, out, {
                "iteration": it, "iterations": iterations, "scenario": scenario,
                "seeds": list(seeds), "elapsed_s": round(elapsed, 1),
                "coverage": float(stats.coverage), "qd_score": float(stats.qd_score),
                "num_elites": int(stats.num_elites), "obj_max": float(stats.obj_max),
                "sigma0": SIGMA0, "emitters": N_EMITTERS, "batch": BATCH_SIZE,
            })
    return archive


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="MAP-Elites over the Tier-2 genome")
    # 9 iterations x 150 = 1,350 evaluations ~ one hour at the measured rate.
    ap.add_argument("--iterations", type=int, default=9)
    ap.add_argument("--workers", type=int, default=n_workers())
    ap.add_argument("--scenario", default="test")
    ap.add_argument("--seeds", type=int, nargs="+", default=list(TRAIN_SEEDS))
    ap.add_argument("--out", type=Path, default=CHECKPOINT)
    ap.add_argument("--seed", type=int, default=42)
    # 150 random genomes ~ one iteration's budget, spent on coverage instead of depth.
    ap.add_argument("--init-random", type=int, default=150,
                    help="random genomes to seed the archive with before CMA-ES")
    ap.add_argument("--bench", type=int, default=0,
                    help="evaluate N random genomes and report throughput, then exit")
    args = ap.parse_args(argv)

    if args.bench:
        rng = np.random.default_rng(7)
        t0 = time.perf_counter()
        with EvaluationPool(workers=args.workers, seeds=tuple(args.seeds),
                            scenario=args.scenario) as p:
            res = p.evaluate(rng.random((args.bench, N_GENES)))
        dt = time.perf_counter() - t0
        print(f"  {len(res)} evals in {dt:.1f}s -> {dt / len(res) * 1000:.0f} ms/eval "
              f"({3600 / (dt / len(res)):,.0f}/hour)")
        print(f"  lanes {np.bincount([r.lane for r in res], minlength=4).tolist()}   "
              f"speed {min(r.mean_speed for r in res):.2f}.."
              f"{max(r.mean_speed for r in res):.2f} m/s")
        return 0

    archive = run(args.iterations, args.workers, args.scenario, tuple(args.seeds),
                  args.out, args.seed, args.init_random)
    print(f"\n  archive -> {args.out}/archive.npz  "
          f"({archive.stats.num_elites}/{archive.cells} cells filled)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
