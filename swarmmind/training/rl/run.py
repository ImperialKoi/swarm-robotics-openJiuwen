"""Train the unit policy on the four demo maps: behaviour cloning, then PPO. Built for Kaggle.

    python -m swarmmind.training.rl.run --hours 11 --out /kaggle/working/rl

**Four maps, on purpose.** The demo plays seeds 42-45 and nothing else (`demo.yaml`
`demo_seeds`), so the policy trains, is checked, and is gated on exactly
those. That makes it a policy *for these maps*. The commander programme's run 2 is the
record of what that means -- +3.66 on its three training maps, 0.00 on maps it had not
seen -- and it is fine here only because the demo will never show it a fifth map. Say
"tuned on the four demo maps", never "generalises".

**Sized for a Kaggle CPU session.** Rollouts are the numpy simulator and the network is a
few thousand parameters, so a GPU adds nothing. Measured on Kaggle (M-76a): one demo
mission is ~1,040 s with four running at once. An iteration here is one episode per demo
seed, run in parallel -- ~17 minutes.

**Resumable, because Kaggle sessions end.** `/kaggle/working` survives only as a saved
version's output (PLAN.md R4). Policy, optimiser moments, RNG and history are pickled to
`state.pkl` after every iteration, atomically, so a session killed mid-save leaves the
previous one intact. Attach the previous version's output as an input to continue.

**What it watches.** Every `--check-every` iterations the *deterministic* policy -- what
the demo would run -- plays the demo seeds, scored with the gate's own `mission_score`,
against the routing-only and heuristic arms on the same seeds. The best policy by that
score, not by training reward, is kept as `policy_best.npz`. Wires warn and never kill.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import pickle
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from ...control import unit_policy as up
from ...sim.scenario import Scenario
from ..gate import mission_score
from .nn import Adam
from .ppo import PPO, Batch, PPOConfig, UnitPolicy, bc_loss_and_grad
from .rollout import run_episode, to_batch

#: Policy entropy below this is a collapse. **Was 0.05, and it fired on every iteration of
#: session 1 as a false alarm:** cloning a heuristic that picks the default 98.7% of the time
#: produces entropy ~0.04 *by design*. Same shape as run 3's slope floor -- a threshold set
#: without looking at what healthy looks like.
ENTROPY_FLOOR = 0.01
MOVED_FLOOR = 0.005


def default_seeds(scenario: str) -> tuple[int, ...]:
    """The scenario's demo maps. Refuses to guess for a scenario that declares none."""
    seeds = Scenario.load(scenario).demo_seeds
    if not seeds:
        raise SystemExit(f"scenario {scenario!r} declares no demo_seeds; pass --seeds")
    return seeds


def _single_thread() -> None:
    for v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
              "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
        os.environ[v] = "1"


def _episode_job(job):
    return run_episode(**job)


def _pool(workers: int):
    if workers <= 1:
        return None
    import multiprocessing as mp

    if getattr(sys.modules.get("__main__"), "__file__", None) is None:
        raise RuntimeError("run as `python -m swarmmind.training.rl.run`, not from a REPL: "
                           "spawn re-executes __main__ (M-30)")
    return mp.get_context("spawn").Pool(workers, initializer=_single_thread)


def _map(pool, jobs):
    return [_episode_job(j) for j in jobs] if pool is None else pool.map(_episode_job, jobs)


def _save(out: Path, state: dict) -> None:
    out.mkdir(parents=True, exist_ok=True)
    tmp = out / "state.pkl.tmp"
    with open(tmp, "wb") as fh:
        pickle.dump(state, fh)
    os.replace(tmp, out / "state.pkl")


def _log(out: Path, rec: dict) -> None:
    print(json.dumps(rec, default=float), flush=True)
    with open(out / "log.jsonl", "a", encoding="utf-8") as fh:
        fh.write(json.dumps(rec, default=float) + "\n")


def _score_cards(eps) -> dict:
    """The gate's own `mission_score`, over whole episodes, with the per-seed breakdown."""
    scores = [mission_score(SimpleNamespace(
        victims_rescued=e.card["rescued"], victims_found=e.card["found"],
        ground_explored_frac=e.card["explored"], robots_lost=e.card["lost"])) for e in eps]
    return {"score": float(np.mean(scores)),
            "rescued": float(np.mean([e.card["rescued"] for e in eps])),
            "found": float(np.mean([e.card["found"] for e in eps])),
            "lost": float(np.mean([e.card["lost"] for e in eps])),
            "per_seed": {int(e.seed): e.card["rescued"] for e in eps},
            "per_seed_score": [round(s, 2) for s in scores]}


def _jobs(policy, seeds, scenario, max_time, routing, *, mode, sample, sample_seed=0):
    return [dict(policy_state=policy.state(), seed=int(s), scenario=scenario,
                 max_time=max_time, sample=sample, mode=mode, routing_fix=routing,
                 sample_seed=int(sample_seed)) for s in seeds]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="train the unit policy on the demo maps (BC -> PPO)")
    ap.add_argument("--hours", type=float, default=11.0)
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--seeds", type=int, nargs="+", default=None,
                    help="maps to train and check on (default: the scenario's demo_seeds)")
    ap.add_argument("--max-time", type=float, default=None, help="episode length; smoke tests")
    ap.add_argument("--out", type=Path, default=Path("runs/rl"))
    ap.add_argument("--resume-from", type=Path, default=None,
                    help="a state.pkl from a previous session (default: <out>/state.pkl)")
    ap.add_argument("--restart", action="store_true")
    ap.add_argument("--bc-epochs", type=int, default=60)
    ap.add_argument("--bc-lr", type=float, default=2e-3)
    ap.add_argument("--bc-target-acc", type=float, default=0.98,
                    help="stop cloning once the *sampled* policy agrees this often")
    ap.add_argument("--check-every", type=int, default=5)
    ap.add_argument("--iterations", type=int, default=0, help="0 = until --hours")
    ap.add_argument("--no-routing-fix", action="store_true")
    ap.add_argument("--lr", type=float, default=None,
                    help="PPO learning rate; applied on resume too (default: keep the saved one)")
    ap.add_argument("--ent-coef", type=float, default=None,
                    help="entropy bonus; applied on resume too (default: keep the saved one)")
    ap.add_argument("--from-best", action="store_true",
                    help="resume the *best* checkpoint's weights rather than the last policy, "
                         "and reset the optimiser moments. For a run that peaked and drifted.")
    args = ap.parse_args(argv)

    seeds = tuple(args.seeds) if args.seeds else default_seeds(args.scenario)
    t0 = time.perf_counter()
    deadline = t0 + args.hours * 3600.0
    routing = not args.no_routing_fix
    pool = _pool(min(args.workers, len(seeds)))
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    print(f"  seeds {list(seeds)}  scenario {args.scenario}  routing fix {routing}", flush=True)

    src = args.resume_from or (out / "state.pkl")
    state = None
    if not args.restart and src.exists():
        with open(src, "rb") as fh:
            state = pickle.load(fh)
        if tuple(state.get("seeds", ())) != seeds:
            raise SystemExit(f"{src} was trained on seeds {state.get('seeds')}, not {seeds}. "
                             f"Pass --restart to start over, or the matching --seeds.")
        print(f"  resumed from {src}: iteration {state['iter']}, "
              f"best check {state['best_check']:.2f}", flush=True)
    if state is None:
        policy = UnitPolicy(up.CAND_DIM, up.CTX_DIM, seed=0)
        state = {"iter": 0, "ppo": PPO(policy, PPOConfig()), "rng": np.random.default_rng(0),
                 "seeds": seeds, "bc_done": False, "baselines": None,
                 "best_check": -np.inf, "checks": [], "config": vars(args) | {"out": str(out)}}
    ppo: PPO = state["ppo"]
    policy = ppo.policy
    cfg = ppo.cfg
    rng = state["rng"]
    # Session 1 ran at 3e-4 and the policy barely moved: per-update KL ~1e-4 against an early
    # stop at 0.045, clip fraction under 1%. The optimiser's rate is the one thing worth
    # changing mid-run without discarding the checkpoint.
    if args.lr is not None:
        ppo.opt_pi.lr = ppo.opt_vf.lr = float(args.lr)
    # Session 2 rose to 1.041x the bar by iteration 45 and then decayed for twenty
    # iterations while entropy climbed 0.16 -> 0.49 and `hold` grew to 17% of decisions:
    # the entropy bonus had become larger than the advantage signal.
    if args.ent_coef is not None:
        ppo.cfg = cfg = dataclasses.replace(cfg, ent_coef=float(args.ent_coef))
    # Rewinding to the best checkpoint is only honest if the optimiser is rewound too --
    # Adam moments from the drifted policy would push straight back where they came from.
    if args.from_best:
        best = out / "policy_best.npz"
        if not best.exists():
            raise SystemExit(f"--from-best: no {best}")
        loaded = UnitPolicy.load_npz(best)
        policy.pi.set_params(loaded.pi.params())
        policy.vf.set_params(loaded.vf.params())
        ppo.opt_pi = Adam(policy.pi.params(), lr=ppo.opt_pi.lr)
        ppo.opt_vf = Adam(policy.vf.params(), lr=ppo.opt_vf.lr)
        print(f"  rewound to {best} (check {state['best_check']:.2f}), optimiser reset",
              flush=True)
    print(f"  PPO learning rate {ppo.opt_pi.lr:g}  entropy bonus {cfg.ent_coef:g}", flush=True)
    bar = -np.inf

    try:
        # --- rung 1: clone the heuristic on the demo maps ---------------------------------
        if not state["bc_done"]:
            eps = _map(pool, _jobs(policy, seeds, args.scenario, args.max_time, routing,
                                   mode="heuristic", sample=False))
            b = to_batch(eps, policy, cfg.gamma, cfg.lam)
            # Its own optimiser: PPO's Adam moments should start clean, not carry a
            # supervised phase's momentum into the first policy-gradient step.
            bc_opt = Adam(policy.pi.params(), lr=args.bc_lr)
            for epoch in range(args.bc_epochs):
                order = rng.permutation(len(b))
                for s in range(0, len(b), cfg.minibatch):
                    i = order[s:s + cfg.minibatch]
                    mb = Batch(b.cand[i], b.mask[i], b.ctx[i], b.action[i], b.logp[i],
                               b.adv[i], b.ret[i])
                    st, g = bc_loss_and_grad(policy, mb)
                    bc_opt.step(policy.pi.params(), g)
                # Argmax agreement is not enough -- PPO *samples*. A policy that picks the
                # heuristic's choice at argmax but only with 40% probability would send
                # most searching robots somewhere random on the first iteration.
                a, _ = policy.act(b.cand, b.mask, rng)
                sampled_acc = float((a == b.action).mean())
                _log(out, {"phase": "bc", "epoch": epoch, **st, "sampled_acc": sampled_acc,
                           "samples": len(b), "heuristic": _score_cards(eps)})
                if sampled_acc >= args.bc_target_acc:
                    break
            state["bc_done"] = True
            _save(out, state)
            policy.save_npz(out / "policy_bc.npz")

        # --- the bars, on the same maps ------------------------------------------------------
        # Deterministic, so each is one set of missions for the whole run.
        if state["baselines"] is None:
            state["baselines"] = {
                "routing": _score_cards(_map(pool, _jobs(
                    policy, seeds, args.scenario, args.max_time, routing,
                    mode="default", sample=False))),
                "heuristic": _score_cards(_map(pool, _jobs(
                    policy, seeds, args.scenario, args.max_time, routing,
                    mode="heuristic", sample=False))),
            }
            _log(out, {"phase": "baselines", **state["baselines"]})
            _save(out, state)
        bar = max(b["score"] for b in state["baselines"].values())

        # --- rung 2: PPO -----------------------------------------------------------------------
        last_iter_s = 0.0
        while True:
            if args.iterations and state["iter"] >= args.iterations:
                break
            if time.perf_counter() + 1.3 * last_iter_s > deadline:
                print("  stopping: the next iteration would overrun --hours", flush=True)
                break
            it0 = time.perf_counter()
            state["iter"] += 1
            eps = _map(pool, _jobs(policy, seeds, args.scenario, args.max_time, routing,
                                   mode="learned", sample=True, sample_seed=state["iter"]))
            batch = to_batch(eps, policy, cfg.gamma, cfg.lam)
            stats = ppo.update(batch, rng) if len(batch) else {}
            counts = np.sum([e.counts for e in eps], axis=0)
            frac = (counts / max(counts.sum(), 1)).round(4).tolist()
            rec = {"phase": "ppo", "iter": state["iter"], "samples": len(batch),
                   "action_frac": frac, "train": _score_cards(eps), **stats,
                   "iter_s": round(time.perf_counter() - it0, 1),
                   "elapsed_h": round((time.perf_counter() - t0) / 3600, 2)}
            wires = []
            if stats and stats.get("entropy", 1.0) < ENTROPY_FLOOR:
                wires.append("entropy collapsed: the policy has stopped exploring")
            if 1.0 - frac[0] < MOVED_FLOOR:
                wires.append("policy never leaves the default: nothing is being learned")

            if state["iter"] % args.check_every == 0:
                chk = _score_cards(_map(pool, _jobs(policy, seeds, args.scenario,
                                                    args.max_time, routing,
                                                    mode="learned", sample=False)))
                chk["iter"] = state["iter"]
                state["checks"].append(chk)
                rec["check"] = chk
                rec["bar"] = bar
                policy.save_npz(out / f"policy_iter{state['iter']:04d}.npz")
                if chk["score"] > state["best_check"]:
                    state["best_check"] = chk["score"]
                    policy.save_npz(out / "policy_best.npz")
                recent = state["checks"][-3:]
                if len(recent) == 3 and all(c["score"] < bar for c in recent):
                    wires.append(f"three checks in a row under the bar ({bar:.1f}): "
                                 f"routing alone or the heuristic is winning")
            if wires:
                rec["tripwires"] = wires
            _log(out, rec)
            _save(out, state)
            last_iter_s = time.perf_counter() - it0
    finally:
        if pool is not None:
            pool.terminate()
            pool.join()
    policy.save_npz(out / "policy_last.npz")
    print(f"  done: {state['iter']} iterations on seeds {list(seeds)}, best check "
          f"{state['best_check']:.2f} against bar {bar:.2f}. Outputs in {out}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
