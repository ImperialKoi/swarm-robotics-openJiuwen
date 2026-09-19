"""Configuration for the optional team; separate from frozen bus contracts."""

import os
from pathlib import Path
from urllib.parse import urlparse

import yaml
from pydantic import BaseModel, ConfigDict, Field

from ..providers.openai_api import OPENROUTER_URL, api_key

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

    def model_api_key(self) -> str:
        if self.model_url == OPENROUTER_URL:
            return api_key()
        endpoint = urlparse(self.model_url)
        if (endpoint.scheme in {"http", "https"}
                and endpoint.hostname in {"127.0.0.1", "localhost", "::1"}
                and not endpoint.username and not endpoint.password):
            return "local-no-secret"
        raise ValueError("Model endpoint must be official OpenRouter HTTPS or loopback")

    @classmethod
    def load(cls):
        path = ROOT / "assets/scenarios/team_response.yaml"
        values = yaml.safe_load(path.read_text(encoding="utf-8"))
        if os.environ.get("OPENROUTER_MODEL"):
            values["model"] = os.environ["OPENROUTER_MODEL"]
        return cls.model_validate(values)
