#!/usr/bin/env bash
# One-time installation only; demo inference is entirely local.
set -euo pipefail
cd "$(dirname "$0")/.."
if [ ! -x integrations/workswarm/.venv/bin/python ]; then
  uv venv --python 3.12 integrations/workswarm/.venv
fi
uv pip sync --python integrations/workswarm/.venv/bin/python integrations/workswarm/requirements.lock
integrations/workswarm/.venv/bin/python -c 'from openjiuwen.agent_teams import TeamAgent, TeamAgentSpec, LeaderSpec, TeamMemberSpec; import importlib.metadata as m; print("WorkSwarm", m.version("workswarm"), "native Leader/Teammate ready")'
