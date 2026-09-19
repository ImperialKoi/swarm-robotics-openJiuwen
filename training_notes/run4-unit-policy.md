# Run 4 — Unit policy, behaviour cloning → PPO — IN PROGRESS

Part of the training record — see [SUMMARY.md](SUMMARY.md) for the index. Branch
`rl/unit-policy`, worktree `~/Documents/swarm-robotics-rl`, **uncommitted**. Nothing here is on
`main` or in the demo.

**The request:** RL-train the individual units, on Kaggle, with the ~29 hours available, and not
on the owner's machine. **Scope set by the owner:** the demo plays four maps, seeds
42–45, and the policy trains, is checked, and is gated on exactly those.

---

## Before anything was trained: bound the action

The detector (M-74) is the recorded case of a training run that went ahead after a flat sweep
said there was nothing to find. This run measured first.

### Contact inspection — dropped

Demo seeds 42–45 on the M1, Tier 3 off. Shipped: rescued 75.25.

| arm | rescued | found |
|---|---:|---:|
| inspect every contact within 25 m | **68.00** | 87.25 |
| **oracle** — only contacts on a real undiscovered casualty (reads ground truth) | **75.75** | 98.75 |

A learned inspection policy cannot beat its own oracle, and the oracle is +0.5 (+6/+1/−6/+1):
more discovery does not convert (M-50, M-64). Not built.

### Where the time goes, and the bug it exposed

Rescue-chain time: carrying 56%, **waiting for a carrier 33%**, waiting for a digger 11%. And
39 casualties were still being carried at the buzzer with a median 124 s left against ~31 s of
route — traced to loaded carriers trapped by the coarse flow field (MEASUREMENTS M-76). Fixed as a
Tier 1 correctness change, `control/zone_routing.py`, flag off by default.

### The bound, on Kaggle (M-76a)

| arm | rescued | score | per-seed rescued (42–49) |
|---|---:|---:|---|
| shipped | 73.00 | 909.67 | 61, 79, 68, 67, 77, 93, 61, 78 |
| routing | **81.12** | **998.06** | 77, 95, 84, 79, 73, 93, 71, 77 |
| routing + heuristic staging | 82.00 | 1006.54 | 75, 92, 81, 75, 79, 96, 73, 85 |

- **Routing:** 1.097×, but +16/+16/+16/+12 on 42–45 and −4/0/+10/−1 on 46–49. With the demo
  fixed to 42–45 (below), the dev-seed split no longer blocks it for the demo.
- **Heuristic staging:** 1.008× over routing, down on four seeds and up on four. A fixed rule that
  helps some maps and hurts others is the one argument for learning *when* to stage. The bar a
  trained policy has to clear is ~+5 rescues; the rule found +0.9. Expected outcome, stated before
  the run so it cannot be reframed after: **trained, gated, probably does not ship.**

---

## Configuration

| | |
|---|---|
| Action | 5 candidates per searching robot: default / stage at nearest site / stage at second site / dark ground / hold (TECHNICAL §7a) |
| Deciders | in-contact carriers, diggers, scouts that are idle or on `explore`; not carrying; ≤ once per 5 s |
| Policy | shared candidate scorer 25→64→64→1 (tanh), value 16→64→64→1; numpy, hand backprop, gradient-checked |
| Rung 1 | behaviour cloning of the heuristic until sampled agreement ≥ 98% (≤ 60 epochs, lr 2e-3) |
| Rung 2 | PPO: clip 0.2, lr 3e-4, 4 epochs, minibatch 4096, entropy 0.01, grad-norm 0.5, KL stop 0.045 |
| Advantages | semi-Markov GAE, γ = 0.995 per second, λ = 0.95 per decision |
| Rewards (training only) | delivery +1.0, pickup +0.2, dug out +0.3 shared, found +0.2 shared, destroyed −0.5 |
| Maps | **seeds 42, 43, 44, 45** — train, check and gate; zone routing on |
| Iteration | one episode per map in parallel, ~17 min on Kaggle; sampling keyed on (map, iteration) |
| Checks | every 5 iterations, deterministic policy on the same four maps vs routing-alone and heuristic |
| Kept | `policy_best.npz` by check `mission_score`, not by training reward |
| Tripwires (warn) | entropy < 0.05; never leaves the default; three checks under the bar |
| Compute | Kaggle CPU session, 4 vCPU, ~11 h per version, resumed from the previous version's output |

## How to run it

```bash
cd ~/Documents/swarm-robotics-rl
uv run python scripts/kaggle_bundle.py      # -> runs/kaggle/swarmmind-rl-src.zip
```

Upload the zip as a new version of the `swarmmind-rl-src` Kaggle dataset, open
`swarmmind/training/notebooks/unit_policy.ipynb` on Kaggle (Accelerator None, Internet On, the
dataset as input), `JOB = "train"`, **Save Version → Save & Run All**. To continue: Add Input →
Notebook Output → previous version, Save & Run All again.

## What to read in `rl/log.jsonl`

- `phase: bc` — `sampled_acc` should reach ~0.98. If it stalls far below, the heuristic's choices
  are not predictable from the features, and PPO will start from noise.
- `phase: baselines` — `routing` and `heuristic` on the four maps. `bar` is the larger score.
- `phase: ppo` — `action_frac[0]` (share of default) should start ~0.9 and move slowly; a jump
  toward 0 is the policy abandoning search. `entropy`, `kl`, `clip_frac`, `explained_var`.
- `check` every 5 iterations — the only number that decides anything. Compare to `bar`.
- `tripwires` — any line with one is worth sending straight away.

## Results

### Session 1 — Kaggle CPU, 10.9 h, lr 3e-4

**Behaviour cloning:** 85,487 decisions from four heuristic missions; sampled agreement 0.835 ->
0.980 in 12 epochs. **Reproducibility on Kaggle confirmed:** the heuristic arm scored exactly
the bound's numbers for the same arm on 42-45 (75, 92, 81, 75), and routing-alone exactly
(77, 95, 84, 79).

**Baselines on the four maps:** routing alone **1015.24** (83.75 rescued, lost 44.25);
heuristic staging 987.45 (80.75, lost 36.75). **The bar is routing alone** -- on the demo maps
the heuristic that training was cloned from is *worse* than doing nothing, by 3 rescues.

**PPO, 40 iterations (~13.5 min each).** Deterministic checks on the four maps:

| iter | 5 | 10 | 15 | 20 | 25 | 30 | 35 | 40 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| score | 1003.1 | 988.8 | 961.7 | 987.6 | 982.4 | 995.7 | **1023.7** | 987.1 |
| rescued | 82.25 | 81.00 | 78.50 | 81.00 | 80.75 | 81.75 | **84.00** | 81.25 |

- **No win.** Best (iter 35, `policy_best.npz`) is **1.008x** the bar; the gate needs 1.05x.
  Seven of eight checks are under routing alone.
- **The check is noisy in a specific way.** Near-identical deterministic policies swing ~60
  points: a handful of different decisions cascade through a 420 s, 512-robot mission. Iter
  35's lead is inside that band, and "best of eight noisy checks" is a winner's curse. Do not
  read it as a result.
- **What PPO did learn: hold.** Share of `hold` rose 0.03% -> 2.9%, `dark` 0.01% -> 0.4%,
  staging flat at ~1.7%, default 98.7% -> 95.0%. Sampled training losses fell ~43 -> 27-31
  robots while rescues stayed ~81-85. The death penalty (-0.5, paid to the robot itself) is the
  densest per-robot signal in the reward, and the policy is following it.
- **It moved slowly.** Per-update KL ~1e-4 to 6e-4 against an early stop at 0.045; clip
  fraction <= 1%. The learning rate, not the budget, was the limit.
- **A tripwire was wrong.** "Entropy collapsed" fired every iteration: cloning a 98.7%-default
  heuristic gives entropy ~0.04 by design, under the 0.05 floor. Floor lowered to 0.01. Third
  threshold in this project set without looking at what healthy looks like (run 2, run 3).
  "Three checks under the bar" fired from iter 15 and was right.

**Changed for session 2:** PPO learning rate 3e-4 -> **1e-3**, applied on resume via `--lr`
(the checkpoint, optimiser moments and best-so-far carry over). Nothing else, so session 2
answers one question: was the policy slow, or is there nothing there.

### Session 2 — Kaggle CPU, 10.6 h, iterations 41-67, lr 1e-3

**It was slow, not empty.** Within five iterations of the rate change the check rose from 987
to 1057, and the next check agreed:

| iter | 45 | 50 | 55 | 60 | 65 |
|---|---:|---:|---:|---:|---:|
| check score | **1057.0** | 1056.7 | 1042.6 | 1037.7 | 1028.2 |
| rescued | **87.5** | 86.75 | 85.5 | 84.75 | 84.0 |
| vs bar 1015.2 | **1.041x** | 1.041x | 1.027x | 1.022x | 1.013x |

**Best: 1.041x, under the 1.05x bar.** +3.75 rescued over routing alone (87.5 vs 83.75),
+6.75 over the heuristic. Two consecutive checks at ~1057 put it outside session 1's ~60-point
noise band, so the gain is real; it is simply not 5%.

**Then it decayed for twenty iterations, monotonically.** The cause is in the same rows:
entropy 0.16 -> 0.49, default share 0.95 -> 0.76, `hold` 2.9% -> 17%, sampled losses 27 -> 20,
found 97.5 -> ~95. **The entropy bonus (0.01) outgrew the advantage signal**, so the policy was
pushed toward randomness, and the cheapest randomness that pays is holding robots still: fewer
deaths (-0.5 each, the densest per-robot reward) at the cost of search. The tripwires did not
catch this -- they watch for entropy *collapse* and for checks under the bar, and this was
neither. A wire for "checks falling for three consecutive checks while still above the bar"
would have.

**Changed for session 3:** resume from `policy_best.npz` (iteration 45) rather than the drifted
policy, with the optimiser moments reset (`--from-best`), entropy bonus **0.01 -> 0.003**
(`--ent-coef`), learning rate held at 1e-3. One question again: was the decay the entropy bonus,
or the ceiling of this action space?

### Session 3 — Kaggle CPU, 11.2 h, iterations 68-98, rewound to iter 45, ent 3e-3

**The entropy fix did what it was meant to.** No drift: the default share held at 0.89-0.90
(session 2 ended at 0.76), entropy sat at 0.22-0.26 instead of climbing past 0.49, and `hold`
stayed ~7-8% rather than 17%.

**And the peak never came back.** Checks: 1001.7, 1026.7, 1020.1, 1028.9, 976.6, 1037.8 --
**mean 1015.3 against a bar of 1015.2**. Thirty iterations from the best weights this run ever
had, and the average landed exactly on "no better than routing alone".

## Verdict — trained, gated, does not ship

| | checks | mean | best |
|---|---|---:|---:|
| session 1, lr 3e-4 | 8 | 991 | 1023.7 |
| session 2, lr 1e-3 | 5 | 1044 | **1057.0 (1.041x)** |
| session 3, rewound, ent 3e-3 | 6 | 1015 | 1037.8 |

98 PPO iterations, ~33 h of Kaggle CPU, ~390 demo missions. **Best single check 1.041x, bar
1.05x, and the session-3 mean says the policy's true value is ~1.00-1.03x.** The 1057 pair was
the top of a noisy distribution, not a level the policy holds: the cleanest evidence is that
rewinding *to those exact weights* and training on did not reproduce them.

**What it did learn, and why it does not pay.** `hold` and `dark` grew at the expense of the
default; sampled losses fell from ~43 to ~22 robots while found and rescued stayed flat. The
death penalty is the densest per-robot signal in the reward, so the policy bought the cheap
part of `mission_score` (-0.5 per loss) and could not move the expensive part (10 per rescue).
Staging -- the action the whole design was built around -- stayed at ~1.2% of decisions
throughout and never grew, which is the policy agreeing with the bound (M-76a): there is
little to win there.

**What would be worth trying next, if anyone reopens this:** the reward, not the algorithm.
Rescue credit is delayed by 150-200 s and shared across a chain of four lanes, while the death
penalty is immediate and personal. Until a searcher's reward reflects the delivery its find
leads to more strongly than its own survival, PPO will keep finding this local optimum. That is
a mechanism change, in the sense this project keeps rediscovering -- not a hyperparameter.

**The programme's actual result is the bug it found on the way.** Diagnosing where carry time
went produced `control/zone_routing.py`: **+22% rescues on the demo maps** (68.75 -> 83.75),
reproduced exactly in three independent Kaggle runs.

## Gate — `make gate` on the demo maps (SHIPPING.md)

| arm | score | rescued | lost |
|---|---:|---:|---:|
| zone routing (the bar) | 1015.24 | 83.75 | 44.25 |
| heuristic staging | 987.45 | 80.75 | 36.75 |
| **trained unit policy** | **1056.97** | **87.50** | **33.75** |

**1.04x -- does not ship**, exactly as the in-run checks predicted. It beats the heuristic it was
cloned from by 6.75 rescued and loses 10.5 fewer robots than routing alone, and neither is worth
anything against a 1.05x bar the project applies to every component.

The same gate run found the programme's real contribution one row up: **zone routing 1.19x**
(68.75 -> 83.75 rescued), the carry-leg bug this work exposed while measuring where rescue time
went.

## Post-gate — in the live configuration, 0.997x (M-76e)

With the scripted Tier 3 running, as the demo does: `tier3+routing` 82.00 rescued / 1001.60,
`tier3+routing+policy` 81.75 / 998.59. The policy is a wash on stage. Four measurements now
agree -- 1.008x (heuristic bound), ~1.00-1.03x (session 3 checks), 1.04x (gate, Tier 3 off),
0.997x (live) -- and the programme's result stands as the routing bug it found.
