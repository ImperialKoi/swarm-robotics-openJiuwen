"""CLAUDE.md invariant #3.

/world/ground_truth exists for the dashboard alone. If any swarm-side module reads it,
the fog-of-war claim in the demo is false. This test is the only thing standing between
"the hivemind doesn't cheat" being a fact and being a slogan.
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "swarmmind"

#: Trees that must never see ground truth.
#: `voice` is here because the operator console's answers become directives. It renders
#: `MissionRenderer.views()[0]` -- the swarm view, the appearance raster masked by the
#: fog -- and never `[1]`, which carries COLOR_VICTIM_TRUE dots read from world.victims.
GUARDED = ("nodes", "control", "hivemind", "training", "voice")

#: The only files permitted to reference it: the dashboard bridge, and the constant itself.
ALLOWED = {"bridge.py", "topics.py"}

NEEDLES = ("WORLD_GROUND_TRUTH", "/world/ground_truth", "ground_truth_view")


def test_no_swarm_module_reads_ground_truth():
    offenders = []
    for tree in GUARDED:
        for path in (ROOT / tree).rglob("*.py"):
            if path.name in ALLOWED:
                continue
            # encoding is explicit because `read_text()` defaults to the *locale*
            # encoding, not UTF-8. On a GBK (cp936) Windows box the non-ASCII
            # characters in a comment in training/mapelites/run.py made this test --
            # the one enforcing invariant #4 -- die with a UnicodeDecodeError instead
            # of reporting on leaks. A guard that cannot read the source it guards is
            # worse than no guard, because it still looks like it ran.
            text = path.read_text(encoding="utf-8")
            for needle in NEEDLES:
                if needle in text:
                    offenders.append(f"{path.relative_to(ROOT.parent)}: {needle}")
    assert not offenders, (
        "ground-truth leak into swarm-side code:\n  " + "\n  ".join(offenders)
        + "\n\nThe swarm may only know what it has observed. Read /swarm/state instead."
    )
