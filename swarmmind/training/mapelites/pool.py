"""Parallel evaluation across the eight M1 cores.

This is the only part of the project that is CPU-bound and embarrassingly parallel, and
it is why MAP-Elites runs locally rather than on Kaggle: eight native cores beat a shared
Kaggle CPU session, and there is no GPU work here to justify the trip.

`spawn` is the start method on macOS, so each worker re-imports the package and builds
its own `World`. That costs a second of warm-up per worker and buys process isolation --
one genome that drives a robot into a numerical corner cannot take the run down with it.

**`spawn` also re-executes `__main__` in every worker.** Called from a REPL, a notebook,
or `python - <<EOF`, there is no importable `__main__` file, and each worker re-runs the
whole script -- which spawns its own workers. That is a fork bomb, and it looks like a
hang: the first attempt here produced 76 MB of tracebacks and had to be killed. `_guard()`
turns it into an error message before a single process is started.
"""

from __future__ import annotations

import multiprocessing as mp
import os
import sys

import numpy as np

from .evaluate import TRAIN_SEEDS, Evaluation, evaluate

#: Each worker gets one core. Without this every worker's BLAS opens a thread per core
#: -- 7 workers x 8 threads on 8 cores -- and they spend their time descheduling one
#: another. The simulator is numpy-heavy but almost entirely elementwise, so it loses
#: nothing by being single-threaded and the parallelism belongs at the genome level.
_SINGLE_THREAD = {
    "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1", "VECLIB_MAXIMUM_THREADS": "1",
}


def _init_worker() -> None:
    os.environ.update(_SINGLE_THREAD)


def _guard() -> None:
    main = sys.modules.get("__main__")
    if getattr(main, "__file__", None) is None:
        raise RuntimeError(
            "EvaluationPool needs a real __main__ script: the 'spawn' start method "
            "re-executes it in every worker, so running this from a REPL, a notebook or "
            "`python - <<EOF` forks bombs instead of evaluating.\n"
            "Run scripts/evolve.py, or put your call under `if __name__ == '__main__':` "
            "in a file."
        )


def _worker(args) -> tuple[int, float, int, float]:
    i, genome, seeds, scenario = args
    ev = evaluate(np.asarray(genome, dtype=float), tuple(seeds), scenario)
    # Only the three numbers the archive needs cross the process boundary. Shipping the
    # per-seed detail back would dominate the transfer for data nothing reads.
    #
    # The index rides along so results can be restored to submission order after
    # `imap_unordered`, which is what lets progress be reported as each evaluation
    # lands rather than only when the whole batch does.
    return i, ev.fitness, ev.lane, ev.mean_speed


def n_workers(reserve: int = 1) -> int:
    """Leave a core free. The machine also runs an editor, and on demo day a dashboard."""
    return max(1, (os.cpu_count() or 2) - reserve)


class EvaluationPool:
    def __init__(self, workers: int | None = None, seeds=TRAIN_SEEDS,
                 scenario: str = "test") -> None:
        self.workers = workers or n_workers()
        self.seeds = tuple(seeds)
        self.scenario = scenario
        self._pool: mp.pool.Pool | None = None

    def __enter__(self) -> EvaluationPool:
        if self.workers > 1:
            _guard()
            ctx = mp.get_context("spawn")
            self._pool = ctx.Pool(self.workers, initializer=_init_worker)
        return self

    def __exit__(self, *exc) -> None:
        if self._pool is not None:
            self._pool.terminate()
            self._pool.join()
            self._pool = None

    def evaluate(self, genomes, on_done=None) -> list[Evaluation]:
        """Order-preserving. MAP-Elites must map results back to the genomes it sent.

        `on_done(k, total)` is called as each evaluation completes, for progress
        reporting. Completion order is *not* submission order -- hence the index carried
        through `_worker` -- but the returned list is, which is the part the archive
        depends on. Callers that pass no callback get exactly the previous behaviour.
        """
        genomes = np.atleast_2d(np.asarray(genomes, dtype=float))
        payload = [(i, g, self.seeds, self.scenario) for i, g in enumerate(genomes)]
        total = len(payload)
        out: list[tuple[float, int, float] | None] = [None] * total

        if self._pool is None:
            stream = (_worker(p) for p in payload)
        else:
            # imap_unordered, not map: `map` blocks until the whole batch is finished,
            # so a 60-evaluation iteration would report nothing for two minutes and then
            # everything at once.
            stream = self._pool.imap_unordered(_worker, payload, chunksize=1)

        for k, (i, f, lane, s) in enumerate(stream, start=1):
            out[i] = (f, lane, s)
            if on_done is not None:
                on_done(k, total)

        return [Evaluation(fitness=f, lane=lane, mean_speed=s, detail={})
                for f, lane, s in out]  # type: ignore[misc]
