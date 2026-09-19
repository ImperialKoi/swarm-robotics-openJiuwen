# Run 1 — Commander, CMA-ES — FAILED

Part of the training record — see [SUMMARY.md](SUMMARY.md) for the index and what each run
taught.

**Hardware:** Windows 11, AMD Ryzen 9 7940H (8 cores / 16 threads), 39 GB RAM. Not the M1
the rest of the docs were measured on — see [MEASUREMENTS.md M-38](../docs/MEASUREMENTS.md).

**Superficially it succeeded.** It ran to plan, improved monotonically, never crashed, and
reported a 1.10× improvement over its baseline. It was discarded because the policy it
found is degenerate, and the reported improvement does not mean what it appears to mean.

### Configuration

    uv run python -m swarmmind.training.command.run --hours 10 --workers 11

| | |
|---|---|
| Stopped at | 121 generations, 3h 37m of a 10 h budget |
| Batch | 9 (CMA-ES default population for 7 parameters) |
| σ₀ | 0.18 |
| Workers | 11 (~59% CPU — see "CPU" below) |
| Training maps | 1001, 1002, 1003 (procedural, `sim/generator.sample`) |
| Held-out maps | 2001–2010 — **never evaluated during the run** |
| Episode | 150 s, ≤160 robots (`TRAIN_MAX_ROBOTS`) |
| Objective | `gate.mission_score` — flat 10 per rescue |

Bounds in force:

| parameter | low | high | final value |
|---|---:|---:|---:|
| `squadron_size` | 16.0 | 160.0 | **155.4495** ← pinned |
| `rebalance_s` | 5.0 | 90.0 | 38.9188 |
| `done_frac` | 0.40 | 0.99 | 0.5422 |
| `w_unexplored` | 0.0 | 3.0 | **0.0476** |
| `w_contacts` | 0.0 | 3.0 | 1.5601 |
| `w_hazard` | 0.0 | 3.0 | 0.2089 |
| `w_distance` | 0.0 | 3.0 | **1.6444** |

### Results

| | |
|---|---:|
| Hand-set baseline | 64.61 |
| Best found | **71.36 (1.10×)** |
| Best found at | generation 72 |
| Generations since last improvement | 49 |

Every improvement to `best` across the whole run:

    gen  11 -> 64.68 (1.00x)      gen  15 -> 68.72 (1.06x)
    gen  12 -> 65.28 (1.01x)      gen  38 -> 69.96 (1.08x)
    gen  13 -> 65.36 (1.01x)      gen  72 -> 71.36 (1.10x)
    gen  14 -> 66.66 (1.03x)

Generation-best distribution — the search *was* adapting, not merely sampling:

| | mean | max |
|---|---:|---:|
| first 30 generations | 62.25 | 68.72 |
| last 30 generations | 64.93 | 70.01 |

### Cause

Four distinct faults, found by reading `nodes/command.py` rather than the training curve.

**1. `w_contacts` was a dead parameter.** `_cut` built the contacts array and never filled
it:

```python
contacts = np.zeros(len(dark))          # never assigned anything else
value = np.clip(self.p.w_unexplored * dark
                + self.p.w_contacts * contacts      # always x 0
                - self.p.w_hazard * hazard
                - self.p.w_distance * dist, 0.0, None)
```

So `w_contacts` could not affect a single decision. 121 generations were spent tuning it
to 1.5601 — **one of seven CMA-ES dimensions optimising pure noise**, which also wastes
covariance adaptation on a direction that carries no signal. The class docstring claimed
"contacts pull squadrons toward casualties already found"; the code did not do that.

**2. The `clip(..., 0, None)` let a large `w_distance` destroy the partition.** `dist` is
normalised to [0, 1]. With the trained `w_unexplored = 0.0476` and `w_distance = 1.6444`,

    value = clip(0.0476 * dark - 1.6444 * dist, 0, None)

is zero for any sector beyond ~3% of maximum range. Nearly every sector therefore has
weight zero — and the territory split is a cumulative sum of those weights:

```python
cum = np.cumsum(weight)
share = np.minimum((cum / total * self.n_squadrons).astype(int), self.n_squadrons - 1)
```

With flat weights `cum` barely advances, so almost every sector collapses onto the same
squadron index. **Run 1 did not learn to prefer near ground. It learned to stop
partitioning the map** — the degenerate way to imitate having no commander, which is the
one thing known to beat the hand-set commander. The objective rewarded it for doing so.

**3. A flat reward per rescue is not a neutral reward.** `mission_score` pays 10 per
rescue regardless of where the casualty is. A casualty near base costs a fraction of the
travel, dig and carry time of one at the map edge, so a flat rate quietly instructs the
policy to harvest the cheap ones and abandon the rest. Combined with fault 2, "ignore
distant ground" was the highest-scoring available behaviour.

**4. Nothing was watching.** The run was judged solely on training score. Training score
cannot distinguish "the policy improved" from "the objective was gamed" — only a map the
search never optimised against can, and held-out maps (2001–2010) were defined but never
evaluated until after the run was stopped. Every signal needed to catch this existed by
**generation 15**; it was caught at 121, by hand, 3h 37m in.

### Related faults found at the same time (not causes of run 1, but load-bearing)

- **Relay tasks are territorially fenced.** `auction.py:157` fences `explore`, `sweep`
  **and `relay`**. A relay is infrastructure that must sit *between* base and the swarm,
  frequently inside a neighbouring squadron's wedge. This is the cascade CLAUDE.md
  documents ("0 of 16 robots in comms"). There is a fallback (`if fenced.any()`), so it
  degrades allocation rather than blocking it: the best bidder *within* the squadron wins
  even when a far better-placed robot exists next door. **Not yet changed.**
- **Wedges are pie slices** cut by bearing from base, so a far slice covers enormous area
  and a near one almost none — with the same ~48 robots either way. **Not yet changed.**
- **CPU.** The run used ~59% of the machine, not the ~70% intended. `BATCH_SIZE = 9` means
  only 9 units of work exist per generation, so workers 10 and 11 idled regardless of
  `--workers`. Parallelising over `(candidate, map)` pairs does not help either: 27 units
  ÷ 12 workers = 3 rounds, the same 3 rounds as 3 sequential maps inside 9 workers. The
  only lever is the population size.

---

## Fixes implemented after run 1

### 1. `w_contacts` wired to real data

`Commander` now takes the `VictimReportTracker` and `_contacts()` counts
`tracker.open_reports()` — confirmed sightings nobody has resolved yet — per sector,
normalised to [0, 1] to match `dark`, `hazard` and `dist`.

**Why this fixes it:** the parameter now has an effect, so the dimension carries signal
instead of noise, and the behaviour the docstring claims is the behaviour the code
performs. Open reports are used rather than all reports because the set empties as
casualties are handled, so a finished sector stops attracting squadrons.

**Invariant #3 note:** the commander reads *reports*, never `world.victims`. It knows only
what the swarm observed. `tests/test_no_ground_truth_leak.py` passes.

Guarded by `test_w_contacts_actually_changes_the_cut`, which fails if the term ever goes
dead again.

### 2. Objective rewritten — distance-weighted, penalised, and scale-free

`training/command/evaluate.command_score` replaces `gate.mission_score` for *training*
only. Three changes:

- **A casualty's worth rises with its distance from base**, from 1.0 at base to 2.0 at the
  furthest casualty on that map. *Why:* removes the implicit "harvest the cheap ones"
  incentive that fault 3 describes. This is the half that actually redirects the search.
- **Every unrescued casualty is subtracted at that same weight.** *Why: honesty of the
  number, not search behaviour.* Since

      sum(rescued w) - sum(unrescued w)  ==  2*sum(rescued w) - sum(all w)

  and `sum(all w)` is constant for a map, this is an **affine transform** of the previous
  objective, and CMA-ES ranks candidates — so it **cannot change which policy the search
  prefers**. It is kept because a negative score legibly means "left more on the ground
  than brought home", and because it would matter if candidates were ever compared across
  differing map sets. Recorded explicitly so nobody later mistakes it for the fix.
- **Every term is normalised to be scale-free.** *Why:* this one does change the search.
  The generator draws maps with different casualty counts, so an unnormalised sum let a
  60-casualty map contribute six times the magnitude of a 10-casualty one — the mean over
  three maps was mostly the largest map's opinion. Dividing by each map's own total makes
  maps commensurable, which is what "rescue everyone" must mean when "everyone" differs
  per map.

Weights: `W_VICTIMS=100` (term is in [-1, 1]), `W_FOUND=10`, `W_EXPLORED=10`, `W_LOST=5`.

**The gate is deliberately not changed.** `command/gate.py` still scores on
`mission_score`. A training reward and an acceptance metric are allowed to differ, and
should: if distance-weighting only improves the weighted score and not the rescue count,
the gate must be able to say so and reject it.

### 3. Bounds widened

| parameter | was | now | reason |
|---|---|---|---|
| `squadron_size` | (16, 160) | **(8, 240)** | pinned at 155.4 |
| `rebalance_s` | (5, 90) | (5, 120) | headroom |
| `done_frac` | (0.40, 0.99) | (0.20, 0.99) | headroom |
| all four weights | (0, 3) | (0, 4) | headroom |

**Why:** an optimum pressed flat against a bound is a statement about the bound, not about
the problem. Widened where the pressure actually was; the rest only a little, because wide
bounds cost sample efficiency and the budget is generations.

Side effect: widening `done_frac` broke `test_the_baseline_sits_inside_the_search_space`,
which asserted `decode(encode(x)) == x` exactly. Encode divides by the bound width and
decode multiplies back, which is not bit-exact — 0.85 round-trips to 0.8499999999999999.
The assertion now uses `pytest.approx`; exact equality only ever held by coincidence of
the old bounds.

### 4. Batch size 9 → 12

**Why:** with a batch of 9 a 12-worker pool idled a quarter of itself all run. A larger
population also averages more samples into each covariance update, which is worth having
on an objective this noisy. Paid for in proportionally fewer generations.

### 5. Tripwires — `training/command/tripwire.py`

The fix for fault 4, and the one that matters most, because it is the one that would have
caught run 1 without anybody reading the source.

| wire | fires when | catches |
|---|---|---|
| `reward-hacking` | training-best slope > 0 while held-out slope ≤ 0, over ≥3 checks | **run 1, by ~generation 15 instead of 121** |
| `bound-pinned` | any coordinate of `best_x` outside [0.02, 0.98] of the unit cube | run 1's `squadron_size` wall, live |
| `stalled` | no new best for 80 generations | collapsed σ, or a flat objective |

Held-out checks run every `--held-out-every` generations (default 10) on
`HELD_OUT_MAP_SEEDS[:3]` — maps the search never optimises against. Cost is one
evaluation per check, about 9% overhead. That is the price of being able to tell
improvement from reward hacking, and run 1 is what not paying it costs.

`stalled` is set at 80 generations deliberately: run 1 went 49 without a new best *while
its generation-best mean was still climbing*, so a lower threshold would be noise.

**A false positive found in its own smoke test, and fixed before run 2.** The first
version of `bound-pinned` fired immediately on generation 2 reporting
`w_contacts=0.000, w_hazard=0.000, w_distance=0.000` as pinned bounds. Those are
`CommandParams`' hand-set defaults, which legitimately sit at the low bound of (0, 4) —
and before the first improvement `best_x` *is* the baseline, so the wire was reporting the
starting point as a search pathology. It now stays quiet until the search has actually
beaten its baseline (`has_search_result`). A tripwire that cries wolf in the first two
minutes of every run is worse than no tripwire, because people learn to scroll past it.
Guarded by `test_bound_tripwire_stays_quiet_until_the_search_beats_the_baseline`.

**They warn; they do not kill.** A false positive that aborts a ten-hour overnight run
costs more than it saves, and two of the three are heuristics over noisy signals. Firings
are shouted in the log and recorded in `meta.json` under `tripwires_firing`, with the
held-out series under `held_out_history`. `--stop-on-trip` opts into aborting.

### 6. Display fix

`best / max(base_score, 1e-9)` printed `-652406686652.36x` once scores could go negative.
Ratios are meaningless for signed objectives; `_vs()` shows a ratio above zero and a
signed delta otherwise.

---

## Why these changes should fix the failure

Run 1 failed because **the highest-scoring available behaviour was a degenerate one**, and
nothing could see that from the training score. Each change removes one leg of that:

1. Distance-weighted rewards mean abandoning distant casualties now *costs* score, so the
   degenerate policy is no longer the top of the objective.
2. Scale-free per-map terms stop the largest map from dictating the answer.
3. A live `w_contacts` gives the commander a way to value ground for a reason other than
   its distance — the mechanism it needs in order to prefer distant ground that actually
   contains casualties, rather than distant ground in general.
4. Wider bounds mean the search reports what it found rather than where it hit a wall.
5. The tripwires mean that if there is *still* a hole in the objective, it surfaces in the
   first half hour instead of after four hours and a manual code review.

**What is deliberately not claimed:** none of this guarantees the trained commander beats
the *none* arm. The gate has three arms — none / heuristic / trained — and the hand-set
commander scores worse than no commander at all, so "beats the heuristic" is a meaningless
victory (M-37). Only `make gate-command` on maps 2001–2010 decides, and the honest
possible outcome remains "no commander ships, and we say so".

**Known remaining risk:** the commander controls territory. If what actually prevents
distant rescues is the comms chain and the relay fencing above, then telling it to value
distant ground will push squadrons outward until robots drop out of comms, and it will
score *worse*. That would show up as the trained arm losing to *none* at the gate — which
is the system working. The relay-fencing fault is documented above and not yet fixed.

---

## Outcome

The six fixes went into **[run 2](run2.md)**, which cleared run 1's specific failure —
`w_contacts` became one of the largest weights the moment it had an effect, and the
squadron-size policy flipped from "do not partition" to "partition finely". Run 2 then
failed a *different* way, on too few training maps.
