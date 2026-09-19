"""Scenario configuration. All tunable constants live in YAML, never in node code."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

SCENARIO_DIR = Path(__file__).resolve().parents[2] / "assets" / "scenarios"


@dataclass(frozen=True)
class MapCfg:
    width_m: float
    height_m: float
    cell: float
    sector_rows: int
    sector_cols: int
    n_clusters: int
    cluster_radius_m: tuple[float, float]
    rubble_fraction: float


@dataclass(frozen=True)
class TerrainCfg:
    #: The mid scale: many rounded bumps, mostly traversable. Without them the landform
    #: is a plain with a few peaks on it -- there is nothing between 30 m grain and a
    #: 70 m mountain, and the map reads as flat however much range it has (M-62).
    hills: int
    hill_radius_m: tuple[float, float]
    #: Peak height as a fraction of radius, as for mountains. Max slope is ~1.21x this.
    hill_steepness: tuple[float, float]
    mountains: int
    mountain_radius_m: tuple[float, float]
    #: Peak height as a fraction of radius. Max slope is ~1.27x this (a steeper
    #: gaussian than a hill's, so the same ratio buys a harder barrier).
    mountain_steepness: tuple[float, float]
    marshes: int
    marsh_radius_m: tuple[float, float]
    marsh_depth_m: tuple[float, float]
    #: Rivers are straight, edge to edge, and hold water to their banks. There is no
    #: wobble knob: a random-walk centreline reads as a scribble from the operator's
    #: camera, and the naturalism lives in the cross-section instead.
    rivers: int
    river_width_m: tuple[float, float]
    river_depth_m: tuple[float, float]
    river_fords: int
    ditches: int
    ditch_length_m: tuple[float, float]
    ditch_width_m: tuple[float, float]
    ditch_depth_m: tuple[float, float]


@dataclass(frozen=True)
class HazardCfg:
    ignite_t: float
    radius0: float
    growth_rate: float       # m/s
    drift_speed: float       # m/s
    origin_sector: str
    battery_drain: float     # fraction/s while inside
    #: When the fire stops growing and starts burning out, and how fast it recedes.
    #:
    #: A fire that only ever grows is not a hazard, it is a countdown: ground it reaches
    #: is lost for the rest of the mission, and a casualty inside it is written off
    #: silently, because `tasks.py` will not generate work whose target is in known
    #: hazard. Real fires exhaust their fuel. Letting this one do the same gives the
    #: swarm a second act -- ground reopens, and casualties the fire reached become
    #: recoverable -- without weakening the threat while it is burning.
    #:
    #: `peak_t: None` (the default) is the old always-growing behaviour, which is what
    #: the `test` fixture keeps.
    peak_t: float | None = None
    decay_rate: float = 0.0  # m/s, after peak_t


@dataclass(frozen=True)
class CommsCfg:
    base_radius: float
    relay_radius: float


@dataclass(frozen=True)
class VictimCfg:
    count: int
    buried_count: int
    min_separation_m: float
    distance_weight_exp: float
    clear_rate: float        # debris fraction cleared per second per scoop robot
    carry_speed_factor: float
    #: Target share at debris/obstacle margins; the remainder prefer exposed ground.
    cover_fraction: float = 0.85
    #: Distance from actual rubble or an interior wall, in metres, not grid cells.
    cover_radius_m: float = 3.0

    def __post_init__(self) -> None:
        if not 0 <= self.buried_count <= self.count:
            raise ValueError("victims.buried_count must be between zero and victims.count")
        if not 0 <= self.cover_fraction <= 1 or not self.cover_radius_m > 0:
            raise ValueError("victims.cover_fraction must be in [0, 1]; cover_radius_m must be > 0")


@dataclass(frozen=True)
class RatesCfg:
    tick_hz: float
    fog_hz: float
    comms_hz: float
    sector_hz: float
    auction_hz: float
    blackboard_hz: float
    heartbeat_hz: float
    hivemind_period_s: float
    orphan_timeout_s: float


@dataclass(frozen=True)
class BatteryCfg:
    idle: float              # fraction/s
    moving: float            # fraction/s at v_max


@dataclass(frozen=True)
class FaultCfg:
    enabled: bool
    min_t: float
    min_rescued: int
    fallback_t: float


@dataclass(frozen=True)
class Scenario:
    name: str
    map: MapCfg
    terrain: TerrainCfg
    base: tuple[float, float]
    extraction_zones: list[tuple[float, float]]
    extraction_radius: float
    robots_per_lane: int
    victims: VictimCfg
    hazard: HazardCfg
    comms: CommsCfg
    battery: BatteryCfg
    rates: RatesCfg
    fault: FaultCfg
    mission_duration_s: float
    bid_weights: dict[str, float] = field(default_factory=dict)
    #: Optional per-lane counts, overriding the flat `robots_per_lane * 4` split.
    #: An even split is the wrong shape for this mission: the relay network saturates
    #: at ~40 posts however many antennas exist, while exploration is the binding
    #: constraint on everything downstream. Keys are actuator names.
    lane_counts: dict[str, int] = field(default_factory=dict)
    #: The maps the demo plays (`demo.yaml`). Empty for scenarios that are not a demo.
    demo_seeds: tuple[int, ...] = ()

    # --- derived ------------------------------------------------------------------

    @property
    def n_robots(self) -> int:
        if self.lane_counts:
            return sum(self.lane_counts.values())
        return self.robots_per_lane * 4

    def lane_size(self, actuator: str) -> int:
        """How many robots of one actuator lane. Falls back to the flat split."""
        if self.lane_counts:
            return int(self.lane_counts.get(actuator, 0))
        return self.robots_per_lane

    @property
    def grid_shape(self) -> tuple[int, int]:
        return (
            int(round(self.map.height_m / self.map.cell)),
            int(round(self.map.width_m / self.map.cell)),
        )

    @property
    def dt(self) -> float:
        return 1.0 / self.rates.tick_hz

    def sector_ids(self) -> list[str]:
        rows = "ABCDEFGH"[: self.map.sector_rows]
        return [f"{r}{c}" for r in rows for c in range(1, self.map.sector_cols + 1)]

    def sector_rect(self, sector_id: str) -> tuple[float, float, float, float]:
        """(x0, y0, x1, y1) in metres."""
        row = "ABCDEFGH".index(sector_id[0])
        col = int(sector_id[1:]) - 1
        sw = self.map.width_m / self.map.sector_cols
        sh = self.map.height_m / self.map.sector_rows
        return (col * sw, row * sh, (col + 1) * sw, (row + 1) * sh)

    def keepouts(self) -> list[tuple[float, float, float]]:
        r = self.extraction_radius * 2.0
        return [(x, y, r) for x, y in [self.base, *self.extraction_zones]]

    # --- loading ------------------------------------------------------------------

    @classmethod
    def load(cls, name_or_path: str | Path) -> Scenario:
        path = Path(name_or_path)
        if not path.suffix:
            path = SCENARIO_DIR / f"{path}.yaml"
        with open(path) as fh:
            raw: dict[str, Any] = yaml.safe_load(fh)
        return cls.from_dict(raw)

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> Scenario:
        m = raw["map"]
        return cls(
            name=raw["name"],
            map=MapCfg(
                width_m=float(m["width_m"]),
                height_m=float(m["height_m"]),
                cell=float(m["cell"]),
                sector_rows=int(m["sector_rows"]),
                sector_cols=int(m["sector_cols"]),
                n_clusters=int(m["n_clusters"]),
                cluster_radius_m=tuple(float(v) for v in m["cluster_radius_m"]),  # type: ignore[arg-type]
                rubble_fraction=float(m["rubble_fraction"]),
            ),
            terrain=TerrainCfg(**{
                k: (tuple(float(x) for x in v) if isinstance(v, list) else v)
                for k, v in raw["terrain"].items()
            }),
            base=tuple(float(v) for v in raw["base"]),  # type: ignore[arg-type]
            extraction_zones=[tuple(float(v) for v in z) for z in raw["extraction_zones"]],  # type: ignore[misc]
            extraction_radius=float(raw["extraction_radius"]),
            robots_per_lane=int(raw["robots_per_lane"]),
            lane_counts={str(k): int(v) for k, v in (raw.get("lane_counts") or {}).items()},
            victims=VictimCfg(**raw["victims"]),
            hazard=HazardCfg(**{
                **raw["hazard"],
                "peak_t": (float(raw["hazard"]["peak_t"])
                           if raw["hazard"].get("peak_t") is not None else None),
                "decay_rate": float(raw["hazard"].get("decay_rate", 0.0)),
            }),
            comms=CommsCfg(**raw["comms"]),
            battery=BatteryCfg(**raw["battery"]),
            rates=RatesCfg(**raw["rates"]),
            fault=FaultCfg(**raw["fault"]),
            mission_duration_s=float(raw["mission_duration_s"]),
            bid_weights=dict(raw.get("bid_weights", {})),
            demo_seeds=tuple(int(v) for v in (raw.get("demo_seeds") or ())),
        )
