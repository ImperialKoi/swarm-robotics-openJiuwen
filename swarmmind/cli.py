"""SwarmMind command line."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from .mission import Mission
from .sim.scenario import Scenario


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="swarmmind")
    sub = ap.add_subparsers(dest="cmd", required=True)

    run = sub.add_parser("run", help="run a mission")
    run.add_argument("--scenario", default="demo")
    run.add_argument("--seed", type=int, default=42)
    run.add_argument("--headless", action="store_true", help="no dashboard, no LLM, no network")
    run.add_argument("--demo", action="store_true",
                     help="realtime + WebSocket bridge for the Godot dashboard")
    run.add_argument("--allocator", default="auction", choices=("auction", "greedy"))
    run.add_argument("--host", default="127.0.0.1")
    run.add_argument("--port", type=int, default=8765)
    run.add_argument("--max-time", type=float, default=None)
    run.add_argument("--wait", action="store_true",
                     help="hold at t=0 until a dashboard connects")
    run.add_argument("--no-hivemind", action="store_true",
                     help="run with Tier 3 silent -- the control condition for the "
                          "claim that Tier 2 does not depend on it")
    run.add_argument("--hivemind-scripted", action="store_true",
                     help="skip the model entirely and use the scripted baseline "
                          "(already implied by --headless)")
    run.add_argument("--hivemind-allow-api", action="store_true",
                     help="use OpenRouter with scripted fallback (needs OPENROUTER_API_KEY and network)")
    run.add_argument("--response-team", nargs="?", const="workswarm",
                     choices=("workswarm", "local", "heuristic"),
                     help="optional lead/logistics/safety team; default uses native WorkSwarm Leader/Teammate")
    run.add_argument("--team-python", help="interpreter for the isolated team worker")
    run.add_argument("--team-trace", default="runs/team/trace.jsonl",
                     help="team evidence log; a Markdown report is written alongside it")
    run.add_argument("--team-goal", help="high-level response goal (up to 500 characters)")
    # Branch `rl/unit-policy`. Both default off, so an unflagged run is the shipped swarm.
    run.add_argument("--zone-routing", action="store_true",
                     help="exact fine-grid routing to collection points: +15 rescues with "
                          "Tier 3 silent, unresolved with it live (M-76e vs M-76f)")
    run.add_argument("--edge-steering", action="store_true",
                     help="experimental steering through fine-linked coarse edges; "
                          "off by default after a fixture regression (M-84)")
    run.add_argument("--unit-policy", metavar="heuristic|PATH.npz",
                     help="per-robot staging inside Tier 2: 'heuristic' for the classical "
                          "rule, or a trained policy (runs/rl/policy_best.npz). Kept and "
                          "unused: gated at 1.04x with Tier 3 off and 0.997x with it live "
                          "(M-76d, M-76e), so it is not part of the demo")

    args = ap.parse_args(argv)
    if args.cmd != "run":
        return 1
    if args.demo and args.headless:
        ap.error("choose --demo or --headless")
    if args.response_team:
        if args.no_hivemind or args.hivemind_scripted or args.hivemind_allow_api:
            ap.error("--response-team owns Tier 3; remove the conflicting hivemind flag")
        if args.response_team != "heuristic" and not args.demo:
            ap.error("a model response team requires --demo; use heuristic for headless checks")
    elif args.team_python or args.team_goal:
        ap.error("--team-python and --team-goal require --response-team")

    scn = Scenario.load(args.scenario)
    opts = _extras(args)
    # The demo plays four maps (`demo.yaml` `demo_seeds`) and nothing is tuned or rehearsed
    # on any other. Not refused -- a debugging run on seed 7 is legitimate -- but said.
    if args.demo and scn.demo_seeds and args.seed not in scn.demo_seeds:
        print(f"  note: seed {args.seed} is not one of the demo maps {list(scn.demo_seeds)}; "
              f"nothing has been rehearsed or tuned on it.")
    if not args.demo:
        print(Mission(scn, args.seed, **_hivemind_opts(args), **opts)
              .run(max_time=args.max_time).render())
        return 0
    return _demo(scn, args, opts)


def _extras(args) -> dict:
    """Zone routing and the unit policy, both off unless asked for.

    Printed when on, because a run that is not the shipped configuration must never be
    mistaken for one -- these change the seed-42 hash and every rehearsed beat.
    """
    out: dict = {"zone_routing": args.zone_routing, "edge_steering": args.edge_steering}
    if args.response_team:
        out["response_team"] = {
            "mode": args.response_team, "python": args.team_python,
            "trace_path": args.team_trace, "goal": args.team_goal, "console": True,
        }
        print(f"  response team: {args.response_team}; trace: {args.team_trace}")
        print(f"  disable team: touch {Path(args.team_trace).with_suffix('.stop')} "
              "(scripted fallback continues; accepted orders expire normally)")
    if args.zone_routing:
        print("  zone routing ON: exact routing to collection points (not the demo default)")
    if args.edge_steering:
        print("  edge steering ON: experimental; rescue improvement is unproven")
    if args.unit_policy:
        from .control.unit_policy import UnitController

        if args.unit_policy in ("heuristic", "default"):
            out["unit_policy"] = UnitController(args.unit_policy)
            print(f"  unit policy ON: {args.unit_policy} (no trained weights)")
        else:
            from .training.rl.ppo import UnitPolicy

            path = Path(args.unit_policy)
            if not path.exists():
                raise SystemExit(f"no unit policy at {path}. Download `rl/policy_best.npz` "
                                 f"from the Kaggle training output, or pass 'heuristic'.")
            out["unit_policy"] = UnitController("learned", UnitPolicy.load_npz(path))
            print(f"  unit policy ON: trained weights from {path} (gated 1.04x, "
                  f"under the 1.05x bar)")
    return out


def _hivemind_opts(args) -> dict:
    """Tier 3 configuration for one run.

    `--headless` forces the scripted rung, which is what its help text has always
    promised: "no dashboard, no LLM, no network". This is not tidiness -- a headless run
    that silently picks up a local `llama serve` because one happens to be listening
    would make `make check`'s seed-42 hash depend on whether a server was running on the
    developer's machine, and invariant #6 would quietly stop meaning anything.

    The demo path gets the full ladder.
    """
    return {
        "hivemind": not args.no_hivemind,
        "scripted_hivemind": args.hivemind_scripted or not args.demo,
        "allow_api": args.hivemind_allow_api,
    }


def _demo(scn, args, opts: dict | None = None) -> int:
    from .bus.ws_server import WebSocketServer
    from .nodes.bridge import BridgeNode

    m = Mission(scn, args.seed, realtime=True, allocator=args.allocator,
                **_hivemind_opts(args), **(opts or {}))
    server = WebSocketServer(args.host, args.port)
    m.bridge = BridgeNode(m.world, server, m.bus)
    try:
        server.start()
    except OSError as exc:
        m.close()
        print(f"  cannot listen on {args.host}:{args.port} -- {exc}\n"
              f"  another SwarmMind is probably still running. Stop it, or pass "
              f"--port 8766 here and set SWARMMIND_WS=ws://{args.host}:8766 for Godot.")
        return 2

    print(f"  SwarmMind  scenario={scn.name}  seed={args.seed}  robots={m.world.n}")
    print(f"  dashboard: ws://{args.host}:{args.port}   (open the Godot project and press F5)")
    w = m.world
    limit = args.max_time if args.max_time is not None else scn.mission_duration_s
    t0 = time.perf_counter()
    try:
        if args.wait:
            print("  waiting for a dashboard to connect ...", flush=True)
            while server.client_count == 0:
                time.sleep(0.1)
        m.sim.start_clock()
        t0 = time.perf_counter()
        print("  running. ctrl-c to stop.\n", flush=True)
        while not w.done and w.t < limit:
            m.tick()
            if w.tick % 200 == 0:
                print(f"  t={w.t:6.1f}s  rescued {w.victims_rescued}/{len(w.victims)}  "
                      f"found {w.victims_found}  rtf {m.sim.rtf:.2f}x  "
                      f"clients {server.client_count}", flush=True)
    except KeyboardInterrupt:
        print("\n  stopped.")
    finally:
        server.stop()
        m.close()
    # `m.scorecard(...)`, never `m.world.scorecard()`: the directive counts and the
    # wall clock are the Mission's to fill in, and this path printed zeros for both.
    print(m.scorecard(time.perf_counter() - t0).render())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
