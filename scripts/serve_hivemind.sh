#!/usr/bin/env bash
# Start the local hivemind model. Run this before `cli run --demo` on demo day.
#
# The dashboard and the swarm do not need it: with no server listening, `build_ladder`
# skips the rung in under a millisecond (connection refused is instant on loopback) and
# Tier 3 falls to the scripted baseline. Nothing hangs, nothing waits.
set -euo pipefail

MODEL="${1:-assets/models/hivemind-base.gguf}"
PORT="${PORT:-8080}"
CTX_SIZE="${CTX_SIZE:-4096}"
LLAMA_BIN="${LLAMA_BIN:-llama}"
LLAMA_TEMPLATE_ARGS=()
if [ -n "${CHAT_TEMPLATE:-}" ]; then
  LLAMA_TEMPLATE_ARGS=(--chat-template "$CHAT_TEMPLATE")
fi
if ! command -v "$LLAMA_BIN" >/dev/null 2>&1 && [ -x "$HOME/.llama-app/llama" ]; then
  LLAMA_BIN="$HOME/.llama-app/llama"
fi

if [ ! -e "$MODEL" ]; then
  echo "no model at $MODEL" >&2
  echo "fetch it with:" >&2
  echo "  llama download -hf Qwen/Qwen2.5-1.5B-Instruct-GGUF:Q4_K_M" >&2
  echo "  ln -sf <path it prints> assets/models/hivemind-base.gguf" >&2
  echo "checksums are in assets/models/models.lock" >&2
  exit 1
fi

# 4096 covers the original advisor; use CTX_SIZE=8192 for the native response team.
# --no-webui keeps the footprint down; see MEASUREMENTS.md M-29 (829 MB steady).
exec "$LLAMA_BIN" serve -m "$MODEL" --port "$PORT" --ctx-size "$CTX_SIZE" --parallel 1 --no-webui --jinja "${LLAMA_TEMPLATE_ARGS[@]}"
