"""The ship / don't-ship decision.

These tests deliberately avoid running missions: what needs guarding is the *decision
rule*, not the simulator. A gate that quietly passes a component worse than the one it
replaces is the single most expensive bug in the project, because it ships.
"""

from __future__ import annotations

import pytest

from swarmmind.metrics import Scorecard
from swarmmind.training.gate import (
    HELD_OUT_SEEDS,
    MARGIN,
    Arm,
    compare,
    mission_score,
)
from swarmmind.training.mapelites.evaluate import TRAIN_SEEDS


def _card(**kw) -> Scorecard:
    base = dict(seed=1, sim_time=420.0, victims_rescued=0, victims_found=0,
                victims_total=8, ground_explored_frac=0.0, robots_lost=0,
                robots_total=16, mean_time_to_rescue=0.0, energy_used=0.0,
                directives_issued=0, directives_rejected=0)
    base.update(kw)
    return Scorecard(**base)


def _arm(name: str, score: float) -> Arm:
    return Arm(name=name, rescued=0.0, found=0.0, explored=0.0, lost=0.0,
               score=score, per_seed=[score])


def test_held_out_seeds_are_disjoint_from_training():
    """An elite that only wins on its training maps has memorised three rubble layouts."""
    assert not set(HELD_OUT_SEEDS) & set(TRAIN_SEEDS)
    assert len(set(HELD_OUT_SEEDS)) == len(HELD_OUT_SEEDS)
    assert len(HELD_OUT_SEEDS) >= 10


def test_rescue_dominates_the_score():
    """Rescues are the mission; exploration is a proxy for it."""
    rescuer = mission_score(_card(victims_rescued=1))
    explorer = mission_score(_card(ground_explored_frac=1.0))
    assert rescuer > explorer, "a full map sweep outscores a rescue"


def test_losing_robots_is_penalised():
    """Clearing the map by driving into fire has not solved the problem."""
    careful = mission_score(_card(victims_rescued=2, robots_lost=0))
    reckless = mission_score(_card(victims_rescued=2, robots_lost=20))
    assert careful > reckless


def test_finding_counts_for_less_than_rescuing():
    assert mission_score(_card(victims_rescued=1)) > mission_score(_card(victims_found=1))


def test_a_marginal_win_does_not_ship():
    """Beating the baseline by 1% on ten seeds is noise."""
    r = compare(_arm("classical", 100.0), _arm("evolved", 101.0))
    assert not r["ships"]
    assert "under the" in r["verdict"]


def test_a_clear_win_ships():
    r = compare(_arm("classical", 100.0), _arm("evolved", 100.0 * MARGIN + 1))
    assert r["ships"]
    assert "evolved ships" in r["verdict"]


def test_a_loss_ships_the_classical_component():
    """A heuristic winning is a result, not a failure to hide."""
    r = compare(_arm("classical", 100.0), _arm("evolved", 60.0))
    assert not r["ships"]
    assert r["verdict"].startswith("classical ships")
    assert r["ratio"] == pytest.approx(0.6)


def test_the_margin_is_above_one():
    assert MARGIN > 1.0, "a gate that passes on a tie is not a gate"


def test_selection_annotation_preserves_measurements_and_is_idempotent(tmp_path):
    from swarmmind.training.gate import SHIPPING_HEADER, refresh_selection_note

    path = tmp_path / "SHIPPING.md"
    measured = "## Summary\n\n| zone routing | zone routing | 1.15x |\n"
    path.write_text(SHIPPING_HEADER + measured)
    refresh_selection_note(path)
    first = path.read_text()
    assert measured in first
    assert "`zone_routing=False`" in first
    assert "`response_team=None`" in first
    refresh_selection_note(path)
    assert path.read_text() == first
