"""Topic names. FROZEN at D7.

These are the exact names from the MVP spec's node graph. They are identical whether
the active transport is LocalBus (in-process) or Ros2Bus (rclpy) -- that is what makes
ROS2 a swappable adapter rather than a dependency.
"""

# --- published by blackboard_node -------------------------------------------------
SWARM_STATE = "/swarm/state"
SWARM_EVENTS = "/swarm/events"
SWARM_HEARTBEAT = "/swarm/heartbeat"

# --- auction ----------------------------------------------------------------------
TASKS_AVAILABLE = "/swarm/tasks_available"
BIDS = "/swarm/bids"


def assigned_task(robot_id: str) -> str:
    """Per-robot award topic, e.g. ``/robot_r07/assigned_task``."""
    return f"/robot_{robot_id}/assigned_task"


# --- hivemind ---------------------------------------------------------------------
HIVEMIND_DIRECTIVES = "/hivemind/directives"

# --- operator voice channel -------------------------------------------------------
#: Captions and phase for the operator's spoken exchange with the swarm. Dashboard-facing
#: like the event feed: it carries what was *said*, never anything the swarm could not
#: already see. Published by `voice/console.py`, drawn under the map by main.gd.
OPERATOR_VOICE = "/operator/voice"

# --- world ------------------------------------------------------------------------
WORLD_HAZARD_ZONE = "/world/hazard_zone"

#: Dashboard-only. NO swarm node, control module, or hivemind provider may subscribe
#: to this -- it is the ground truth the fog-of-war is meant to hide. Enforced by
#: tests/test_no_ground_truth_leak.py. See CLAUDE.md invariant #3.
WORLD_GROUND_TRUTH = "/world/ground_truth"
