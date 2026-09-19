"""Tripwires: catch a training run that has already failed.

Run 1 ran for 3h37m and 121 generations before anyone looked closely enough to see that
its "1.10x improvement" was a policy which had learned to stop partitioning the map. Every
signal needed to catch that was available by **generation 15**. Nothing was watching.

Each wire answers a question the headline number cannot:

* `HeldOutDivergence` -- *is the score going up because the policy is better, or because
  it found a hole in the objective?* Training score alone cannot distinguish those. Only a
  map the search has never optimised against can. This is the wire that would have caught
  run 1.
* `BoundPinned` -- *is a bound choosing the answer instead of the search?* Run 1 settled
  `squadron_size` at 155.4 against a ceiling of 160.0 and reported it as a result. An
  optimum flat against a wall is a statement about the wall.
* `Stalled` -- *is anything still happening?* CMA-ES with a collapsed step size, or a flat
  objective, both look exactly like a run that is working.

**They warn; they do not kill.** A false positive that aborts a ten-hour overnight run
costs more than it saves, and two of the three are heuristics over noisy signals. Firing
is recorded in `meta.json` and shouted in the log; `--stop-on-trip` is there for anyone
who would rather lose the run than the night.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

#: A coordinate this close to the edge of the unit cube is pinned. 0.02 of a bound's
#: width -- tight enough that ordinary drift near an edge does not fire it, loose enough
#: to catch a parameter genuinely pressed flat against the wall.
EDGE_LO, EDGE_HI = 0.02, 0.98

#: Generations without a new best before the search is called stalled. ~50 is normal late
#: in a CMA-ES run (run 1 went 49 while still improving its mean), so this is set well
#: above that to stay a signal rather than noise.
STALL_GENERATIONS = 80

#: Held-out checks needed before a divergence claim is allowed. Two points make a line
#: through noise; three is the minimum that can show a trend.
MIN_HELD_OUT_POINTS = 3

#: Fraction of the training gain that held-out must capture to count as real progress.
#:
#: **The first version tested `held_out_slope <= 0`, and it missed the very run it was
#: built for.** Run 2 gained +3.66 on training while held-out moved 0.00 across four
#: checks -- but the flat held-out series had a slope of +0.0028/gen, which is noise and
#: is also, technically, greater than zero. "Flat" is not "negative", and a threshold at
#: exactly zero can be defeated by the last digit of nothing happening. Comparing the two
#: slopes is the test that matches the intent: if held-out is capturing under a quarter of
#: what training is gaining, the score is improving without the policy improving.
HELD_OUT_CAPTURE = 0.25

#: Training slope below this magnitude counts as "not rising", in score units per
#: generation.
#:
#: **The second bug in this wire, and the mirror image of the first.** Fixing "flat
#: held-out defeats `<= 0`" introduced "flat *training* defeats `> 0`": run 3 plateaued so
#: hard that all four training samples and all four held-out samples were identical, and
#: `np.polyfit` on constant data returns floating-point dust around 1e-16 rather than an
#: exact zero. `+1e-16 > 0.0` is true, so the wire fired ten times reporting
#: "training rising +0.000/gen". A slope test needs a magnitude floor on *both* sides; a
#: sign check on either is defeated by the last bits of nothing happening.
#:
#: Scores here run around -80, and a real gain is ~1 point over a 10-generation window,
#: i.e. ~0.1/gen. This is set an order of magnitude below that: small enough never to mask
#: a genuine trend, enormous compared to 1e-16.
MIN_TRAIN_SLOPE = 0.01


@dataclass
class Trip:
    name: str
    detail: str


@dataclass
class Tripwires:
    """Accumulates run history and reports which wires are currently firing."""

    param_names: tuple[str, ...]
    stall_generations: int = STALL_GENERATIONS
    #: (generation, training best, held-out score) at each held-out checkpoint.
    history: list[tuple[int, float, float]] = field(default_factory=list)
    last_improved_gen: int = 0
    #: Set once the search beats its starting point. Until then `best_x` is still the
    #: hand-set baseline, and the bound wire must stay quiet -- see `check`.
    has_search_result: bool = False

    def record_held_out(self, gen: int, train_best: float, held_out: float) -> None:
        self.history.append((int(gen), float(train_best), float(held_out)))

    @staticmethod
    def _slope(xs, ys) -> float:
        """Least-squares slope. Sign is all that is read, so scale does not matter."""
        x = np.asarray(xs, dtype=float)
        y = np.asarray(ys, dtype=float)
        if len(x) < 2 or np.ptp(x) == 0:
            return 0.0
        return float(np.polyfit(x, y, 1)[0])

    def check(self, gen: int, best_x: np.ndarray) -> list[Trip]:
        trips: list[Trip] = []

        # 1. Reward hacking: training climbing while held-out does not follow.
        if len(self.history) >= MIN_HELD_OUT_POINTS:
            gens = [h[0] for h in self.history]
            tr = self._slope(gens, [h[1] for h in self.history])
            ho = self._slope(gens, [h[2] for h in self.history])
            if tr > MIN_TRAIN_SLOPE and ho < HELD_OUT_CAPTURE * tr:
                pct = 100.0 * ho / tr
                trips.append(Trip(
                    "reward-hacking",
                    f"training rising {tr:+.3f}/gen but held-out only {ho:+.3f}/gen -- "
                    f"held-out is capturing {pct:.0f}% of the training gain over "
                    f"{len(self.history)} checks (floor is "
                    f"{HELD_OUT_CAPTURE * 100:.0f}%). The score is improving without the "
                    f"policy improving: suspect the objective, or too few training maps.",
                ))

        # 2. A bound is choosing the answer.
        #
        # Only once the search has actually beaten its starting point. `CommandParams`
        # defaults `w_contacts`, `w_hazard` and `w_distance` to exactly 0.0, which is the
        # low bound, so before the first improvement `best_x` is the hand-set baseline
        # sitting legitimately on three edges. Firing there reports the baseline's own
        # defaults as a search pathology -- caught in a smoke run, and precisely the kind
        # of false positive that teaches people to ignore the wire.
        x = np.asarray(best_x, dtype=float)
        pinned = [
            f"{n}={v:.3f}" for n, v in zip(self.param_names, x, strict=False)
            if v < EDGE_LO or v > EDGE_HI
        ] if self.has_search_result else []
        if pinned:
            trips.append(Trip(
                "bound-pinned",
                f"best_x pressed against its bounds: {', '.join(pinned)} "
                f"(unit-cube coords; outside [{EDGE_LO}, {EDGE_HI}]). Widen BOUNDS or "
                f"the wall, not the search, is setting these values.",
            ))

        # 3. Nothing is moving.
        since = gen - self.last_improved_gen
        if since >= self.stall_generations:
            trips.append(Trip(
                "stalled",
                f"no new best for {since} generations -- step size collapsed, or the "
                f"objective is flat in the region the search is in.",
            ))
        return trips


def render(trips: list[Trip]) -> str:
    """A block that is hard to scroll past in a ten-hour log."""
    if not trips:
        return ""
    lines = ["", "  " + "!" * 74]
    for t in trips:
        lines.append(f"  !! TRIPWIRE [{t.name}]")
        for chunk in _wrap(t.detail, 68):
            lines.append(f"  !!   {chunk}")
    lines.append("  " + "!" * 74)
    return "\n".join(lines)


def _wrap(text: str, width: int) -> list[str]:
    out, line = [], ""
    for word in text.split():
        if len(line) + len(word) + 1 > width:
            out.append(line)
            line = word
        else:
            line = f"{line} {word}".strip()
    if line:
        out.append(line)
    return out


__all__ = ["EDGE_HI", "EDGE_LO", "STALL_GENERATIONS", "Trip", "Tripwires", "render"]
