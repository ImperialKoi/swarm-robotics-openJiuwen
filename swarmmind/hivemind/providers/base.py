"""Provider interface. Everything that can produce a directive looks like this."""

from __future__ import annotations

from typing import Protocol


class Provider(Protocol):
    #: Stamped into the published message and shown as a badge on the dashboard, so an
    #: operator can always see which rung of the ladder actually answered.
    name: str

    def generate(self, system: str, user: str, timeout: float) -> str:
        """Return raw model output, or raise on failure/timeout."""
        ...
