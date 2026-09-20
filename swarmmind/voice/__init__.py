"""The operator's voice channel.

Additive, and off unless `--voice` asks for it. Nothing in `sim/`, `nodes/` or `control/`
imports this package: with the microphone muted or `sounddevice` absent the mission runs
exactly as it did, which is CLAUDE.md invariant #1 applied to a human instead of Tier 3.
"""

from .mic import Microphone, wav_bytes
from .speaker import Speaker

__all__ = ["Microphone", "Speaker", "wav_bytes"]
