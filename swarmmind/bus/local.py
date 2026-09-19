"""In-process synchronous bus. The default transport."""

from __future__ import annotations

from collections import defaultdict
from typing import Any


class LocalBus:
    """Synchronous pub/sub with a retained last message per topic.

    Synchronous on purpose: the headless mission is a single deterministic loop, and an
    async bus would introduce scheduling order as a hidden input to seed-42 runs.
    Retention exists so a late subscriber -- the dashboard bridge connecting mid-run --
    gets current state immediately instead of waiting for the next publish.
    """

    def __init__(self) -> None:
        self._subs: dict[str, list] = defaultdict(list)
        self._latest: dict[str, Any] = {}
        self.published = 0

    def publish(self, topic: str, msg: Any) -> None:
        self._latest[topic] = msg
        self.published += 1
        for cb in self._subs[topic]:
            cb(msg)

    def subscribe(self, topic: str, callback) -> None:
        self._subs[topic].append(callback)

    def latest(self, topic: str) -> Any | None:
        return self._latest.get(topic)

    def topics(self) -> list[str]:
        return sorted(self._latest)
