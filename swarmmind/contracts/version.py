"""Contract version. FROZEN.

Bumping this is a breaking change: it requires updating swarmmind/contracts/schemas.py,
the Godot client parser in godot/scripts/ws_client.gd, and tests/test_contracts.py in
the same commit. See CLAUDE.md invariant #4.
"""

SCHEMA_VERSION = "1.0"
