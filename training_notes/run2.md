# Run 2 — Commander, CMA-ES — FAILED (differently)

Part of the training record — see [SUMMARY.md](SUMMARY.md) for the index and what each run
taught. Run 2 carried all six fixes from [run 1](run1.md).

**It fixed what it set out to fix.** The two faults run 1 died of — a dead parameter and a
degenerate territory split — are gone, and the evidence for that is in the learned policy
rather than in the score. It was stopped at 50 generations for a *different* reason: it
was fitting three maps instead of the problem.

## Configuration

    uv run python -m swarmmind.training.command.run --hours 10 --workers 12 --restart

| | | vs run 1 |
|---|---|---|
| Stopped at | 50 generations, 1h 44m of a 10 h budget | 121 gens, 3h 37m |
| Batch | 12 | 9 |
| σ₀ | 0.18 | same |
| Workers | 12 (**76.9% CPU**) | 11 (~59%) |
| Training maps | 1001–1003 | same |
| Held-out checks | **every 10 gens on 2001–2003** | none |
| Episode | 150 s, ≤160 robots | same |
| Objective | `evaluate.command_score` — distance-weighted, scale-free | flat `mission_score` |
| Tripwires | armed | none |

Batch 12 against 12 workers gave full pool utilisation for the first time; run 1's batch of
9 left a quarter of an 11-worker pool idle for its entire duration.

## Results

| | |
|---|---:|
| Hand-set baseline | −80.71 |
| Best found | **−75.85 (+4.86)** |
| Best found at | generation 27 |
| Generations completed | 50 |

Every improvement to `best`:

    gen   1 -> -80.71      gen  11 -> -79.03
    gen   3 -> -80.27      gen  18 -> -78.37
    gen   5 -> -79.66      gen  27 -> -75.85
    gen   9 -> -79.51

Generation-best mean rose −80.50 (first 10) → −79.14 (last 10), so the search was adapting
rather than sampling.

Final parameters:

| parameter | run 1 | **run 2** | note |
|---|---:|---:|---|
| `squadron_size` | 155.4495 | **12.3486** | pinned at the *floor* now, not the ceiling |
| `rebalance_s` | 38.9188 | 12.7659 | |
| `done_frac` | 0.5422 | 0.5753 | |
| `w_unexplored` | 0.0476 | 0.3412 | |
| `w_contacts` | 1.5601 *(inert)* | **3.0023** | the parameter that was dead in run 1 |
| `w_hazard` | 0.2089 | 0.9988 | |
| `w_distance` | 1.6444 | 3.3853 | |

## What run 2 proves the fixes worked

**1. The contacts fix landed.** `w_contacts` went from a dead parameter run 1 tuned to a
meaningless 1.56, to **3.0023 — near the top of its (0, 4) range** — within 50 generations
of being given an effect. The search grabbed the newly-available signal immediately, which
is about as direct as evidence gets that the term now carries information.

Read together with `w_distance = 3.3853`, the learned policy is legible: *value ground near
base, unless casualties are known to be there.* That is a sensible search doctrine, and it
is a sentence that could not have been written about run 1's commander, because run 1's
commander had no way to know where casualties were.

**2. The degenerate partition is gone.** Run 1 drove `w_unexplored` to 0.0476 against
`w_distance` 1.6444, which clipped nearly every sector's value to zero and collapsed the
territory split — it learned to *stop partitioning*, imitating "no commander". Run 2
settled `squadron_size` at **12.3 robots per squadron**, roughly 13 squadrons on a
160-robot training swarm: the opposite behaviour, and fine-grained partitioning is what
Tier 3a exists to do. The objective rewrite flipped the policy from "do not partition" to
"partition finely."

## Why it was stopped

**Training improved. Held-out did not.**

| check | train best | held-out (2001–2003) |
|---|---:|---:|
| gen 10 | −79.51 | −95.02 |
| gen 20 | −78.37 | −95.30 |
| gen 30 | −75.85 | −95.02 |
| gen 40 | −75.85 | −95.02 |

Training gained **+3.66**. Held-out moved **0.00** — a slope of +0.0028/gen, which is
noise. The commander was getting better at maps 1001–1003 and no better at maps it had
never seen.

This is **overfitting, not objective-gaming** — a different failure from run 1 with the
same outward signature, which is precisely why the held-out series is the measurement that
matters and the training curve is not. `command/gate.py`'s own docstring names it: *"a
commander that only wins on the three maps it trained against has memorised three maps."*
Seven free parameters against three fixed maps is enough capacity to fit the maps.

## The tripwires: one caught it, one missed it, one was noise

**`bound-pinned` fired correctly**, from generation ~40:

    best_x pressed against its bounds: squadron_size=0.019
    (unit-cube coords; outside [0.02, 0.98])

Run 1 pinned this parameter at the **ceiling** (155 of 160). Run 2 pinned it at the
**floor**. Both times the bound, not the search, was choosing the value — and this time it
was visible live instead of after the fact.

**`reward-hacking` did not fire, and should have.** This is a bug in the wire, found by the
run it was built for. The condition was written as:

```python
if tr > 0.0 and ho <= 0.0:          # WRONG
```

Held-out was flat, and a flat noisy series has a slope of +0.0028 — technically greater
than zero, so the wire stayed silent through exactly the divergence it existed to detect.
**"Flat" is not "negative", and a threshold at exactly zero can be defeated by the last
digit of nothing happening.** Fixed to compare the two slopes:

```python
if tr > 0.0 and ho < HELD_OUT_CAPTURE * tr:     # HELD_OUT_CAPTURE = 0.25
```

Held-out capturing under a quarter of the training gain now fires. On run 2's real numbers
that is 2%, so it would have fired at the third check — **generation 30**. Guarded by
`test_tripwire_catches_a_FLAT_held_out_series_not_only_a_falling_one`, which is built from
this run's actual four data points.

**`stalled` never fired**, correctly — the run was stopped at 50 generations with its last
improvement at 27, well inside the 80-generation threshold.

## Fixes carried into run 3

| change | from | to | why |
|---|---|---|---|
| `TRAIN_MAP_SEEDS` | 3 maps | **6 maps** (1001–1006) | three maps were memorisable |
| `squadron_size` floor | 8.0 | **4.0** | run 2 pinned at the floor |
| `reward-hacking` wire | `ho <= 0` | `ho < 0.25 * tr` | flat held-out must fire |

**The map-count cost is real and was accepted deliberately.** Doubling the maps doubles
evaluation cost, so a 10-hour budget buys ~150 generations instead of ~290. That trade is
worth taking: 290 generations of memorising three maps is worth less than 150 that
generalise.

Rotating a different map subset each generation would have kept the cost flat and was
rejected — `run.py` keeps the best-so-far with `if scores[i] > best`, and that comparison
is meaningless once the maps underneath it change between generations.

## Still not fixed

Carried forward from run 1 and untouched:

- **Relay tasks are territorially fenced** (`auction.py:157`). A relay must often sit in a
  neighbouring squadron's wedge; fencing it is the comms cascade CLAUDE.md documents. With
  `squadron_size` now at ~12 robots there are *more* wedges than in run 1, so this fault
  may bite harder, not less.
- **Wedges are pie slices** cut by bearing from base — a far slice covers enormous area, a
  near one almost none, with the same robots either way.
- **MAP-Elites σ₀ vs bounds** (see [run 0](run0-mapelites.md)).
