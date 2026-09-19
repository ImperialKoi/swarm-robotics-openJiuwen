PY := uv run python

.PHONY: check test lint smoke headless demo stress evolve gate kaggle-bundle clean

## Must be green on every commit from D3 onward.
check: lint test smoke

lint:
	uv run ruff check swarmmind tests scripts

test:
	uv run pytest

## Local verification uses the tiny fixture; the full demo is an explicit run.
smoke:
	$(PY) -m swarmmind.cli run --headless --scenario test --seed 42

## The canonical run. Prints a scorecard. No LLM, no dashboard, no network.
headless:
	$(PY) -m swarmmind.cli run --headless --seed 42

## Realtime sim + WebSocket bridge for the Godot dashboard.
## Start this first, then open godot/ in Godot and press F5 (it reconnects either way).
demo:
	$(PY) -m swarmmind.cli run --demo --scenario demo

## Same, on the small fixture -- boots in a second, good for checking the dashboard.
demo-test:
	$(PY) -m swarmmind.cli run --demo --scenario test

## Robot-count sweep -> ceiling at RTF 0.8x -> ship 80% of it.  (D2)
stress:
	$(PY) scripts/stress.py

evolve:
	$(PY) -m swarmmind.training.mapelites.run

## Train the commander (Tier 3a) on generated maps. Local CPU, checkpoints every
## generation, safe to interrupt. --hours sets the budget.
train-command:
	$(PY) -m swarmmind.training.command.run --hours 10

## Score the trained commander against no-commander and the hand-set one, held-out maps.
gate-command:
	$(PY) -m swarmmind.training.command.gate --report runs/command/gate.json

## Zip the simulator for Kaggle (unit-policy bound and training run there, not here).
kaggle-bundle:
	$(PY) scripts/kaggle_bundle.py

gate:
	$(PY) -m swarmmind.training.gate --scenario demo --workers 4 --report SHIPPING.md

clean:
	rm -rf .pytest_cache .ruff_cache runs
	find . -name __pycache__ -type d -exec rm -rf {} +
