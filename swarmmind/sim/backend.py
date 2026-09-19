"""SimBackend: the swappable simulation adapter.

FastSim and DemoSim wrap the *same* World and differ only in clocking -- that identity
is what makes anything trained in FastSim transfer to the demo for free
(docs/TECHNICAL.md section 1, invariant 1). GazeboSim (stretch, event-day only)
implements the same protocol against a real physics stack.
"""

from __future__ import annotations

import time
from typing import Protocol

import numpy as np

from .scenario import Scenario
from .world import World


class SimBackend(Protocol):
    world: World

    def reset(self, scenario: Scenario, seed: int) -> World: ...
    def step(self, v_cmd: np.ndarray, omega_cmd: np.ndarray) -> World: ...
    @property
    def sim_time(self) -> float: ...


class FastSim:
    """Headless, as-fast-as-possible. Every training rollout runs here."""

    def __init__(self, scenario: Scenario, seed: int, *, evolved: bool = True) -> None:
        #: `evolved=False` builds the swarm from the hand-set archetypes instead of
        #: `demo_roster.yaml`. It exists so `training/gate.py` can run a control arm --
        #: see the note in `sim/robot.evolved_roster`.
        self._evolved = evolved
        self.world = World(scenario, seed, evolved=evolved)

    def reset(self, scenario: Scenario, seed: int) -> World:
        self.world = World(scenario, seed, evolved=self._evolved)
        return self.world

    def step(self, v_cmd: np.ndarray, omega_cmd: np.ndarray) -> World:
        self.world.step(v_cmd, omega_cmd)
        return self.world

    @property
    def sim_time(self) -> float:
        return self.world.t


class DemoSim(FastSim):
    """Realtime-clocked. Reports RTF; drops to catch-up mode when it falls behind.

    Catch-up exists so a transient hiccup (a GC pause, the hivemind thread waking)
    does not permanently desynchronise the run from the rehearsed demo timings.
    """

    CATCHUP_THRESHOLD_S = 0.100

    def __init__(self, scenario: Scenario, seed: int, *, evolved: bool = True) -> None:
        super().__init__(scenario, seed, evolved=evolved)
        self._t0 = time.perf_counter()
        self.rtf = 1.0

    def reset(self, scenario: Scenario, seed: int) -> World:
        w = super().reset(scenario, seed)
        self._t0 = time.perf_counter()
        return w

    def start_clock(self) -> None:
        """Begin realtime pacing after setup or waiting for the dashboard."""
        self._t0 = time.perf_counter() - self.world.t

    def step(self, v_cmd: np.ndarray, omega_cmd: np.ndarray) -> World:
        self.world.step(v_cmd, omega_cmd)
        wall = time.perf_counter() - self._t0
        self.rtf = self.world.t / wall if wall > 0 else 0.0
        lag = self.world.t - wall
        if lag > 0:
            time.sleep(lag)
        return self.world
