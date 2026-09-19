# Training runs — summary

Index of every training run, what it produced, and what it taught. One file per run; this
is the page to read first.

Companion to [docs/MEASUREMENTS.md](../docs/MEASUREMENTS.md), which records *measurements*.
These files record *runs* — configuration, results, and the reasoning that followed. Never
delete an entry: a superseded run is the evidence that the change which superseded it was
needed.

**Hardware for every run below:** Windows 11, AMD Ryzen 9 7940H (8 cores / 16 threads),
39 GB RAM. Not the M1 the rest of the docs were measured on — see
[MEASUREMENTS.md M-38](../docs/MEASUREMENTS.md).

---

## The runs

| | run | result | headline |
|---|---|---|---|
| 0 | [MAP-Elites, Tier-2 genome](run0-mapelites.md) | **succeeded** | 31/40 cells (77.5%), roster shipped |
| 1 | [Commander, CMA-ES](run1.md) | **failed** | +1.10× that was a degenerate policy |
| 2 | [Commander, CMA-ES](run2.md) | **failed** | fixed run 1's faults, overfit 3 maps |
| 3 | [Commander, CMA-ES](run3.md) | **plateaued → gated** | **no commander ships (0.98×)** |
| 4 | [Unit policy, BC → PPO](run4-unit-policy.md) | **gated out (1.04×)** | 98 iterations, ~33 h Kaggle. Best 1.041× the bar, mean ≈ bar. **The result was the routing bug it found on the way: +22% rescues** |

## The commander verdict

Ten held-out maps, three arms, `make gate-command`:

| arm | score | rescued | explored |
|---|---:|---:|---:|
| **none** | **70.81** | 52 / 768 | 28.3% |
| heuristic | 65.57 | 47 / 768 | 27.3% |
| trained | 69.42 | 52 / 768 | 28.5% |

> no commander ships; trained reached only 0.98x, under the 1.05x bar

`mission.py:66` already defaults `command=False`, so **the shipped behaviour is already
the honest one**. Three runs took a component that was *actively harmful* (0.93× — the
hand-set commander was worse than no commander) and made it neutral (0.98×). That is a
real result, and the three-arm gate is why it can be stated honestly: "trained beats the
heuristic by 5.9%" would have been true and meaningless.

**Tuning seven numbers cannot fix a mechanism.** Further commander work should change the
mechanism — relay fencing, pie-slice wedges, the zero-clipping value function — and
re-gate, not search the same parameterisation harder.

---

## What each run taught

### Run 0 — seeding matters more than the emitter, at a small budget

150 random genomes filled 29 of 40 archive cells **before CMA-ES took a step**; nine
iterations of actual search added two more. At a ~700-evaluation budget, coverage comes
overwhelmingly from cheap broad sampling, and the emitters' job is depth in cells that are
already occupied. M-31 got 40% coverage without the seeding pass; this got 77.5% with it.

Also: an archive is not a roster. `select.py` deliberately picks the *extremes* of measured
speed per lane, not the top-N by fitness, which is why one shipped elite has fitness 0.00.
Diversity that is only claimed is worth nothing.

### Run 1 — a training curve cannot tell you whether training worked

Run 1 improved monotonically to 1.10× its baseline and was **wrong in four separate ways**,
none of which were visible in the number it was reporting:

1. **`w_contacts` was a dead parameter** — `contacts` was `np.zeros(...)` and never filled,
   so one of seven CMA-ES dimensions optimised pure noise for 121 generations.
2. **A `clip(..., 0, None)` let a large `w_distance` destroy the territory split** — nearly
   every sector clipped to zero weight, collapsing the cumsum that cuts wedges. It learned
   to *stop partitioning*, which is the degenerate way to imitate "no commander" — the one
   thing known to beat the hand-set commander.
3. **A flat reward per rescue is not a neutral reward.** Distant casualties cost far more
   travel, dig and carry time than near ones, so a fixed 10-per-head quietly instructs the
   policy to harvest the cheap ones.
4. **Nothing was watching.** Held-out maps existed and were never evaluated during the run.

**The lesson, and the one that generalises:** every signal needed to catch this was
available by generation 15. It was caught at 121, by hand, after 3h37m — by reading
`nodes/command.py`, not by reading the score. A score that only ever goes up tells you
nothing about *why*.

### Run 2 — fixing the objective can expose the next constraint, and "flat" is not "negative"

The six fixes worked, and the proof is in the policy rather than the score: `w_contacts`
went from inert to **3.0023 of a possible 4.0** the moment it had an effect, and
`squadron_size` flipped from 155 (one squadron, do not partition) to 12.3 (thirteen
squadrons, partition finely). Read with `w_distance = 3.39`, the learned doctrine is *value
ground near base unless casualties are known to be there* — a sentence that could not have
been written about run 1's commander, which had no way to know where casualties were.

It then failed on the next constraint down: **training gained +3.66 while held-out gained
0.00**. Seven free parameters against three fixed maps is enough capacity to fit the maps
rather than the problem — overfitting, a *different* failure from run 1 with the same
outward signature. That distinction is exactly why held-out is the measurement that matters.

**And a lesson about the safety net itself.** The `reward-hacking` tripwire — written
specifically to catch this — **did not fire**, because its condition was
`held_out_slope <= 0` and a flat noisy series has a slope of +0.0028. Technically positive.
"Flat" is not "negative", and a threshold at exactly zero can be defeated by the last digit
of nothing happening. It now compares held-out gain against training gain
(`ho < 0.25 * tr`), which on run 2's real numbers fires at generation 30.

A monitor is not trustworthy until it has caught something. This one had to fail once
first, and the run's four real data points are now its regression test.

### Run 3 — a sign check is not a threshold, and seven numbers cannot fix a mechanism

Six maps did what they were meant to: `squadron_size` settled at 69.2, comfortably
interior, where runs 1 and 2 had both run to a bound. Then the search plateaued — 47
generations failed to beat a result found in twelve minutes — and the gate said **no
commander ships**.

**The same tripwire failed a second time, in the mirror image of the first.** Run 2 taught
"flat held-out defeats `<= 0`". Run 3 taught "flat *training* defeats `> 0`": with
`best_x` unchanged for 47 generations the held-out check kept re-evaluating it
deterministically, all samples were byte-identical, and `np.polyfit` on constant data
returns ~1e-16 rather than zero. The wire fired ten times announcing "training rising
+0.000/gen".

**A slope test needs a magnitude floor on both sides.** A sign check on either is defeated
by the last bits of nothing happening. Fixed with `MIN_TRAIN_SLOPE = 0.01`, and the
held-out check now skips when `best_x` has not moved — which also stops it burning two
minutes per check to recompute a constant.

The wider lesson from the whole commander programme: **tuning parameters cannot repair a
mechanism.** Three runs of increasingly careful search over the same seven numbers ended
at 0.98× because the machinery underneath — bearing-cut wedges, territorially fenced
relays, a value function that clips to zero — is what limits it.

---

## Standing decisions

- **The demo plays four maps, 42–45, and everything for it trains and gates on those** (owner,
  `demo.yaml` `demo_seeds`). A policy tuned on them is a policy *for them* -- run 2 of
  the commander is the record of how far that can be from generalising. Describe it that way.
- **Bound an action before training it.** Contact inspection was dropped when a ground-truth
  oracle for it moved rescues +0.5 (run 4). The detector (M-74) is the case where the flat sweep
  came back first and the training ran anyway.

- **The gate is not the training objective, on purpose.** `command/gate.py` scores on
  `mission_score` (flat rescues) while training optimises `command_score`
  (distance-weighted, scale-free). If distance-weighting only improves the weighted score
  and not the rescue count, the gate must be able to say so and reject it.
- **The gate has three arms — none / heuristic / trained — and `none` is the bar.** The
  hand-set commander scores *worse* than no commander at all (M-37), so "beats the
  heuristic" is a meaningless victory. Only `make gate-command` on maps 2001–2010 decides,
  and "no commander ships, and we say so" remains an honest outcome.
- **Tripwires warn, they do not kill.** A false positive that aborts a ten-hour overnight
  run costs more than it saves. `--stop-on-trip` opts in.
- **Never compare scores across objective changes.** Run 1's 64.61 baseline and run 2's
  −80.71 measure different things. Each run's notes record its own baseline for this
  reason.

## Open faults, carried across runs

| fault | first noted | status |
|---|---|---|
| Relay tasks territorially fenced (`auction.py:157`) — comms cascade risk | run 1 | **open** |
| Wedges are bearing-cut pie slices; far slices are enormous | run 1 | **open** |
| MAP-Elites σ₀ 0.15 vs [0,1] bounds → >100 resamples | run 0 | **open** |
| `mapelites/run.py` sizing comments stale (says 1,350 evals; is 690) | run 0 | **open** |
