"""The demo plays four maps. Every place that picks seeds for it must agree on which."""

from __future__ import annotations

import json
import pickle
from pathlib import Path

import pytest

from swarmmind.sim.scenario import Scenario
from swarmmind.training import gate
from swarmmind.training.rl import run

DEMO = (42, 43, 44, 45)
ROOT = Path(__file__).resolve().parents[1]


def test_demo_yaml_declares_the_four_demo_maps_and_the_fixture_declares_none():
    assert Scenario.load("demo").demo_seeds == DEMO
    assert Scenario.load("test").demo_seeds == ()


def test_gate_and_trainer_read_the_same_seeds_from_the_scenario():
    assert gate.demo_seeds("demo") == DEMO
    assert run.default_seeds("demo") == DEMO
    # Kept, not used by default.
    assert len(gate.HELD_OUT_SEEDS) == 10 and not set(gate.HELD_OUT_SEEDS) & set(DEMO)


def test_trainer_refuses_a_scenario_with_no_demo_seeds():
    with pytest.raises(SystemExit):
        run.default_seeds("test")


def test_the_kaggle_notebook_trains_on_the_same_seeds():
    nb = json.loads((ROOT / "swarmmind/training/notebooks/unit_policy.ipynb").read_text())
    config = next("".join(c["source"]) for c in nb["cells"]
                  if c["cell_type"] == "code" and "SEEDS =" in "".join(c["source"]))
    ns: dict = {}
    exec(config, ns)
    assert tuple(ns["SEEDS"]) == DEMO


def test_trainer_refuses_to_resume_a_checkpoint_from_other_seeds(tmp_path):
    with open(tmp_path / "state.pkl", "wb") as fh:
        pickle.dump({"seeds": (1, 2, 3, 4), "iter": 3, "best_check": 0.0}, fh)
    with pytest.raises(SystemExit, match="trained on seeds"):
        run.main(["--scenario", "test", "--seeds", "11", "--workers", "1",
                  "--out", str(tmp_path)])


def _notebook_cell(marker: str) -> str:
    nb = json.loads((ROOT / "swarmmind/training/notebooks/unit_policy.ipynb").read_text())
    return next("".join(c["source"]) for c in nb["cells"]
                if c["cell_type"] == "code" and marker in "".join(c["source"]))


def test_notebook_resume_takes_the_checkpoint_not_the_trainer_source(tmp_path):
    """The source dataset contains `swarmmind/training/rl`. Resume must never mistake it for
    a checkpoint -- doing so starts a Kaggle session from zero without saying so."""
    inp, work = tmp_path / "input", tmp_path / "working"
    (inp / "src/swarmmind/training/rl").mkdir(parents=True)
    (inp / "src/swarmmind/training/rl/run.py").write_text("# source, not a checkpoint")
    ckpt = inp / "zz-previous-version/rl"
    ckpt.mkdir(parents=True)
    (ckpt / "state.pkl").write_bytes(b"checkpoint")
    (ckpt / "log.jsonl").write_text("{}\n")
    (ckpt / "stray.py").write_text("# copied by the old cell")
    work.mkdir()
    ns = {"INPUT": inp, "WORK": work}
    exec("import glob, os, shutil\nfrom pathlib import Path\n"
         + _notebook_cell("def resume(name):").split("def run(")[0]
         + "\n" + _notebook_cell("def resume(name):")[
             _notebook_cell("def resume(name):").index("def resume(name):"):], ns)
    ns["resume"]("rl")
    assert (work / "rl/state.pkl").read_bytes() == b"checkpoint"
    assert (work / "rl/log.jsonl").exists()
    assert not list((work / "rl").glob("*.py"))
