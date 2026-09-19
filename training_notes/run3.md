# Run 3 — Commander, CMA-ES — PLATEAUED, then GATED

Part of the training record — see [SUMMARY.md](SUMMARY.md) for the index. Run 3 carried the
fixes from [run 2](run2.md): six training maps instead of three, a `squadron_size` floor of
4, and the corrected `reward-hacking` wire.

**This is the run that ended the commander programme**, not because it failed but because
it plateaued and the gate was finally asked the question. Verdict: **no commander ships**.

## Configuration

    uv run python -m swarmmind.training.command.run --hours 10 --workers 12 --restart

| | run 2 | **run 3** |
|---|---|---|
| Training maps | 1001–1003 | **1001–1006 (six)** |
| `squadron_size` bounds | (8, 240) | **(4, 240)** |
| Batch | 12 | 12 |
| Workers / CPU | 12 / 76.9% | 12 / 77.0% |
| Baseline | −80.71 (3 maps) | **−82.83 (6 maps)** |
| Stopped at | 50 gens, 1h 44m | **51 gens, 3h 13m** |

Evaluation cost scaled linearly with maps as expected: 118 s for six against 60 s for
three, giving ~3.8 min/generation and a projected ~160 generations in the budget.

## Results

| | |
|---|---:|
| Baseline | −82.83 |
| Best found | **−76.84 (+5.99)** |
| Best found at | **generation 3** |
| Generations without improvement | **47** |
| gen-best mean | −79.80 (first 10) → −79.60 (last 10) |
| gen-best max, last 10 | −77.99 — never beat gen 3 |

Every improvement, in the first twelve minutes and then nothing:

    gen 1 -> -82.83      gen 2 -> -80.63      gen 3 -> -76.84

Final parameters — and the notable thing is where `squadron_size` landed:

| parameter | run 1 | run 2 | **run 3** |
|---|---:|---:|---:|
| `squadron_size` | 155.4 *(pinned high)* | 12.3 *(pinned low)* | **69.2 (interior)** |
| `rebalance_s` | 38.92 | 12.77 | 22.35 |
| `done_frac` | 0.5422 | 0.5753 | 0.8945 |
| `w_unexplored` | 0.0476 | 0.3412 | 0.5254 |
| `w_contacts` | 1.5601 *(inert)* | 3.0023 | 2.1972 |
| `w_hazard` | 0.2089 | 0.9988 | 1.0051 |
| `w_distance` | 1.6444 | 3.3853 | 3.3318 |

**Six maps stopped the search running to a wall.** Runs 1 and 2 both pinned
`squadron_size` against a bound (ceiling, then floor). Run 3 settled at 69.2 — 0.28 of the
unit cube, comfortably interior — and `bound-pinned` never fired. That is consistent with
the diagnosis that three maps left enough map-specific structure to chase to the extremes;
six averaged it out.

## The plateau

47 generations and three hours failed to beat a result found in twelve minutes. With six
maps a map-specific exploit no longer pays, so gains are genuinely harder to find — the
change working as intended, not a fault. But it also meant the search had nothing left to
give, and the honest move was to stop and ask the gate rather than run out the budget.

## The tripwire failed a second time, in the mirror image of the first

The `reward-hacking` wire fired **ten times as a false positive**:

    !! TRIPWIRE [reward-hacking]
    !!   training rising +0.000/gen but held-out only -0.000/gen

Because `best_x` never changed after generation 3, and the held-out check re-evaluates
`best_x` deterministically, all four training samples and all four held-out samples were
**byte-identical**. `np.polyfit` on constant data returns floating-point dust near 1e-16,
not an exact zero — and `+1e-16 > 0.0` is true.

Run 2 fixed *"flat held-out defeats `<= 0`"*. Run 3 exposed *"flat **training** defeats
`> 0`"* — the same error one line over. **A slope test needs a magnitude floor on both
sides; a sign check on either is defeated by the last bits of nothing happening.**

Two fixes:

- `MIN_TRAIN_SLOPE = 0.01` — training must be *meaningfully* rising, not merely
  numerically positive. Scores run near −80 and a real gain is ~0.1/gen, so this sits an
  order of magnitude below anything genuine and enormously above 1e-16.
- **The held-out check is skipped when `best_x` has not moved.** It was spending ~2 minutes
  every 10 generations to recompute a number we already had, and feeding the wire a
  constant series no trend can be fitted to.

Guarded by `test_reward_hacking_wire_stays_quiet_when_training_is_also_flat`, built from
run 3's actual four data points.

## The gate — the decision the programme existed to make

    uv run python -m swarmmind.training.command.gate --report runs/command/gate.json

Ten held-out maps (2001–2010), never trained against, three arms:

| arm | score | rescued (abs) | rescued/map | explored |
|---|---:|---:|---:|---:|
| **none** | **70.81** | 52 / 768 | 10.9% | 28.3% |
| heuristic | 65.57 | 47 / 768 | 10.7% | 27.3% |
| trained | 69.42 | **52 / 768** | **11.7%** | **28.5%** |

> no commander ships; trained reached only 0.98x, under the 1.05x bar

**The verdict stands.** `mission.py:66` already defaults `command=False`, so the shipped
behaviour is already the honest one and nothing needs changing to be truthful.

**Training was not worthless.** The hand-set commander was *actively harmful* — 65.57
against 70.81 for having none at all, 0.93×. Three runs turned a component that hurt the
mission into one that is merely neutral (0.98×). That is a real result, and it is precisely
why the gate has three arms: reporting "trained beats heuristic by 5.9%" without the
`none` arm would have been a meaningless victory (M-37).

**One detail, stated carefully.** Trained and `none` rescued the *same 52 casualties*, and
trained is marginally ahead on per-map rescue fraction (11.7% vs 10.9%) and exploration.
The deciding 1.39-point gap therefore comes from the proxy terms — `victims_found` (×2) and
`robots_lost` (×0.5) — not from the mission objective. That is worth knowing and **not**
worth spinning: `robots_lost` is in the score deliberately, because a swarm that clears the
map by driving into fire has not solved the problem. `gate.py`'s `_run` does not return
`found`/`lost` per map, so the decomposition needs an instrumented re-run.

## Where this leaves the commander

The three faults identified in run 1 and never fixed are the obvious suspects for why a
territory policy cannot beat no territory policy at all:

- **Relay tasks are territorially fenced** (`auction.py:157`). A relay must often sit in a
  neighbouring squadron's wedge; fencing it is the comms cascade CLAUDE.md documents.
- **Wedges are bearing-cut pie slices** — a far slice covers enormous area, a near one
  almost none, with the same robots either way.
- **The value function clips at zero**, so a large `w_distance` can still flatten the split.

Tuning seven numbers cannot fix a mechanism. Any further commander work should change the
mechanism and re-gate, not search the same parameterisation harder.
