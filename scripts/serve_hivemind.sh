#!/usr/bin/env bash
# Start the local hivemind model. Run this before `cli run --demo` on demo day.
#
# The dashboard and the swarm do not need it: with no server listening, `build_ladder`
# skips the rung in under a millisecond (connection refused is instant on loopback) and
# Tier 3 falls to the scripted baseline. Nothing hangs, nothing waits.
set -euo pipefail

MODEL="${1:-assets/models/hivemind-base.gguf}"
PORT="${PORT:-8080}"

if [ ! -e "$MODEL" ]; then
  echo "no model at $MODEL" >&2
  echo "fetch it with:" >&2
  echo "  llama download -hf Qwen/Qwen2.5-1.5B-Instruct-GGUF:Q4_K_M" >&2
  echo "  ln -sf <path it prints> assets/models/hivemind-base.gguf" >&2
  echo "checksums are in assets/models/models.lock" >&2
  exit 1
fi

# --ctx-size 4096 is ample: the prompt is ~1.4 KB and independent of swarm size.
# --no-webui keeps the footprint down; see MEASUREMENTS.md M-29 (829 MB steady).
exec llama serve -m "$MODEL" --port "$PORT" --ctx-size 4096 --no-webui --jinja
