"""Configuration for the optional team; separate from frozen bus contracts."""

from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field

ROOT = Path(__file__).resolve().parents[3]


class TeamConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    goal: str = Field(min_length=1, max_length=500)
    interval_s: float = Field(gt=0)
    deadline_s: float = Field(gt=0)
    max_age_s: float = Field(gt=0, le=30)
    call_timeout_s: float = Field(gt=0)
    max_tokens: int = Field(gt=0, le=256)
    token_budget: int = Field(gt=0)
    max_candidates: int = Field(gt=0, le=16)
    hazard_ceiling: float = Field(ge=0, le=1)
    explored_ceiling: float = Field(ge=0, le=1)
    model_url: str
    model: str

    @classmethod
    def load(cls):
        path = ROOT / "assets/scenarios/team_response.yaml"
        return cls.model_validate(yaml.safe_load(path.read_text(encoding="utf-8")))
