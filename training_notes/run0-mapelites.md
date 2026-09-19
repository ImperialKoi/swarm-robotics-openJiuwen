# Run 0 — MAP-Elites, Tier-2 genome — SUCCEEDED

The run that produced the roster every commander run since has trained against.

**Hardware:** Windows 11, AMD Ryzen 9 7940H (8 cores / 16 threads), 39 GB RAM. Not the M1
the rest of the docs were measured on — see [MEASUREMENTS.md M-38](../docs/MEASUREMENTS.md).

    uv run python -m swarmmind.training.mapelites.run --workers 11

| | |
|---|---|
| Wall clock | 12.2 min (729.5 s) |
| Evaluations | 690 — 150 random seeding + 9 iterations × 60 |
| Workers | 11 (~71% CPU) |
| Scenario / seeds | `test`, training seeds (11, 12, 13) |
| Archive | 4 lanes × 10 speed bins = 40 cells |

## Results

| | this run | M-31 (D9) |
|---|---:|---:|
| Cells filled | **31/40 (77.5%)** | 16/40 (40%) |
| After random seeding alone | 29/40 (72.5%) | 15/40 (37.5%) |
| `obj_max` | 1.331 | — |
| `obj_mean` | 0.511 | — |
| QD score | 15.85 | — |

Coverage per iteration: 29 → 30 → 30 → 30 → 31 → 31 → 31 → 31 → 31.

`obj_max` per iteration: 0.973 → 1.019 → 1.080 → 1.211 → 1.235 → 1.251 → 1.331 → 1.331 →
1.331.

The archive nearly doubled M-31's coverage, and most of that came from the random seeding
pass rather than from CMA-ES: 150 random genomes filled 29 of 40 cells before the emitters
took a single step. That is the `--init-random` change doing exactly what it was added for
— giving the emitters archive elites to restart from instead of circling the origin.

## Output

`select.py` turned the archive into `assets/scenarios/demo_roster.yaml`: 12 elites, 3 per
lane, cycled to 32 robots.

    none     3 elites   speeds ['1.46', '1.31', '0.39']   fitness ['1.33', '1.20', '0.58']
    scoop    3 elites   speeds ['0.49', '1.44', '0.42']   fitness ['0.28', '0.28', '0.22']
    gripper  3 elites   speeds ['1.40', '1.22', '0.44']   fitness ['0.37', '0.34', '0.00']
    antenna  3 elites   speeds ['0.48', '0.59', '0.31']   fitness ['0.62', '0.62', '0.56']

The measured-speed spread is real and was discovered rather than designed — scouts at
1.46 / 1.31 / 0.39 m/s, carriers at 1.40 / 1.22 / 0.44. That within-lane spread is the
whole illumination claim, and it is defensible to a judge.

One gripper elite has fitness 0.00. That is the slow-extreme pick, held for diversity
rather than for performance — the archive is meant to fill cells, not to be uniformly
strong.

**Consequence to remember:** once `demo_roster.yaml` exists, `world.py`'s
`evolved_roster()` picks it up automatically, so every run afterwards uses evolved bodies
and the seed-42 demo hash changes. That is intended, but it means commander runs 1 and 2
trained against *different swarms* than any pre-roster measurement.

## Outstanding issues

**Bounds too tight for σ₀.** pyribs warned at iteration ~3:

    UserWarning: During bounds handling, this ES resampled at least 100 times.
    This may indicate that your solution space bounds are too tight.

`SIGMA0 = 0.15` against `[0, 1]` bounds makes the emitters propose out-of-bounds genomes
and resample. Wasted budget, and it can bias the proposal distribution. **Not addressed** —
it cost little on a 12-minute run and would cost real budget on an overnight one.

**Stale sizing comments.** `mapelites/run.py`'s docstring says "9 iterations x 150 = 1,350
evaluations ~ one hour". `N_EMITTERS`(4) × `BATCH_SIZE`(15) = **60** per iteration, so the
default is 690 evaluations and ~26 minutes. The numbers changed when emitters went 5 → 4
and batch 30 → 15; the prose did not follow. **Not addressed.**
