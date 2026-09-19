"""The wire protocol between the Python bridge and the Godot dashboard.

Godot cannot be run from here, so this is what stands in for testing the client: it
reads every field `godot/scripts/main.gd` pulls out of a message and asserts the bridge
actually sends it. A rename on either side fails the build instead of silently blanking
the dashboard on demo day.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from swarmmind.bus.ws_server import WebSocketServer
from swarmmind.mission import Mission
from swarmmind.nodes.bridge import BridgeNode
from swarmmind.sim.scenario import Scenario

ROOT = Path(__file__).resolve().parents[1]
GD = ROOT / "godot" / "scripts" / "main.gd"
HUD = ROOT / "godot" / "scripts" / "hud.gd"
DRIVE = ROOT / "godot" / "scripts" / "manual_drive.gd"


@pytest.fixture(scope="module")
def payloads():
    m = Mission(Scenario.load("test"), 42, allocator="auction")
    srv = WebSocketServer(port=8799)
    bridge = BridgeNode(m.world, srv, m.bus)
    for _ in range(400):
        m.tick()
    w = m.world
    greet: list = []
    srv.send_to = lambda _c, obj: greet.append(obj)
    bridge._greet(None)

    # Reassemble the chunked map payload exactly as main.gd does, which also checks the
    # chunking round-trips.
    blobs: dict = {}
    for msg in greet:
        if msg["t"] == "blob":
            blobs.setdefault(msg["name"], {})[msg["i"]] = msg["data"]
    sent = {
        "hello": next(o for o in greet if o["t"] == "hello"),
        "greet": greet,
        "blobs": {k: "".join(v[i] for i in range(len(v))) for k, v in blobs.items()},
        "state": bridge._state(w, m.executor, m.tracker),
        "fog": bridge._fog(w),
        "truth": bridge._truth(w),
    }
    return sent


def _keys_read_by_gdscript(handler: str) -> set[str]:
    """Field names main.gd pulls out of a message, from `msg.get("x", ...)`."""
    src = GD.read_text(encoding="utf-8")
    body = src.split(f'"{handler}":', 1)[1] if f'"{handler}":' in src else src
    return set(re.findall(r'msg\.get\("([a-z_]+)"', body))


def test_gdscript_exists_and_is_tab_indented():
    for path in (GD, HUD, DRIVE):
        src = path.read_text(encoding="utf-8")
        assert src.strip(), f"{path.name} is empty"
        bad = [i + 1 for i, ln in enumerate(src.splitlines())
               if ln.startswith("    ") and not ln.startswith("\t")]
        assert not bad, f"space-indented GDScript lines in {path.name}: {bad[:5]}"


def test_the_hud_activity_panel_has_a_row_for_every_activity_code():
    """SWARM ACTIVITY is one row per code, and the rows are the whole swarm.

    A code with no row is robots the panel silently leaves out, so the rows stop adding
    up to the alive count and nobody can tell which robots are missing. A row naming a
    code that does not exist is worse: hud.gd looks it up in `ACTIVITY` on every refresh,
    and a missing key stops the HUD updating mid-demo.
    """
    from swarmmind.contracts.schemas import ACTIVITY

    body = re.search(r"const ACTIVITY_ROWS := \[(.*?)\n\]", HUD.read_text(encoding="utf-8"), re.S)
    assert body, "ACTIVITY_ROWS not found in hud.gd"
    rows = re.findall(r'\["(\w+)",', body.group(1))
    assert len(rows) == len(set(rows)), f"duplicate activity rows: {rows}"
    assert set(rows) == set(ACTIVITY), (
        "hud.gd's activity rows have drifted from contracts/schemas.py\n"
        f"  missing rows: {sorted(set(ACTIVITY) - set(rows))}\n"
        f"  unknown rows: {sorted(set(rows) - set(ACTIVITY))}"
    )


def test_the_hud_fonts_ship_inside_the_project():
    """The demo does not have network, so the HUD's faces are files in the project.

    A `res://` font that does not resolve does not error: hud.gd falls back to the engine
    font and the dashboard quietly stops looking like its design.
    """
    root = HUD.resolve().parents[1]
    paths = re.findall(r'"(res://[^"]+\.ttf)"', HUD.read_text(encoding="utf-8"))
    assert paths, "hud.gd references no fonts at all"
    missing = [p for p in paths if not (root / p.removeprefix("res://")).exists()]
    assert not missing, f"hud.gd references fonts that do not exist: {missing}"
    # Both faces are SIL OFL, which requires the licence to travel with the font.
    assert len(list((root / "assets" / "fonts").glob("OFL*.txt"))) >= 2


def test_hello_carries_everything_the_dashboard_reads(payloads):
    hello = payloads["hello"]
    for key in ("cell", "w", "h", "map_m", "sectors", "zones", "zone_r", "lanes",
                "victims_total", "scenario", "robots", "height_scale"):
        assert key in hello, f"hello is missing {key!r}, which main.gd reads"
    assert len(hello["sectors"]) > 0 and "r" in hello["sectors"][0]
    assert payloads["blobs"]["terrain"].startswith("iVBORw0K"), "terrain must be a PNG"
    assert {o["t"] for o in payloads["greet"]} == {"hello", "blob", "hello_done"}


def test_no_frame_exceeds_godots_default_inbound_buffer(payloads):
    """Godot's WebSocketPeer drops the connection when a frame exceeds its 64 KB inbound
    buffer -- silently, with no error on either side, which looks exactly like the
    simulator not running. This is the bug that made the dashboard sit on
    "waiting for simulator" while the bridge reported a happy client."""
    import json

    for msg in payloads["greet"]:
        size = len(json.dumps(msg))
        assert size < 60_000, f"{msg['t']} frame is {size} bytes; Godot's default is 65535"


def test_state_shape_matches_the_renderer(payloads):
    st = payloads["state"]
    assert set(st) >= {"t", "tick", "time", "r", "hud", "reports"}
    # main.gd indexes r[i][0..7] and rp[0..2] positionally -- the width is the contract.
    from swarmmind.contracts.schemas import ACTIVITY

    codes = set(ACTIVITY.values())
    for row in st["r"]:
        assert len(row) == 8, (
            "robot rows are [x, y, heading, lane, status, battery, chassis, activity]"
        )
        assert int(row[7]) in codes, (
            f"activity code {row[7]} is not in ACTIVITY; main.gd would index its "
            "colour table out of range"
        )
    from swarmmind.nodes.bridge import CONTACT_CODE

    for rp in st["reports"]:
        assert len(rp) == 3, "contact rows are [x, y, contact_code]"
        assert rp[2] in CONTACT_CODE.values(), (
            f"contact code {rp[2]} is not one of CONTACT_CODE; main.gd would clamp it "
            "and draw the ring in the wrong colour"
        )
    for key in ("rescued", "found", "total", "active", "lost", "incomms", "explored"):
        assert key in st["hud"], f"HUD is missing {key!r}"


def test_flight_telemetry_is_actual_mode_and_keeps_the_eight_column_contract():
    import numpy as np

    from swarmmind.contracts.schemas import DashboardFlight
    from swarmmind.sim.robot import CHASSIS_INDEX, FAILED

    m = Mission(Scenario.load("test"), 42, hivemind=False)
    w = m.world
    rotors = np.flatnonzero(w.chassis == CHASSIS_INDEX["rotor"])
    w.airborne[rotors[:2]] = True
    w.status[rotors[1]] = FAILED
    bridge = BridgeNode(w, WebSocketServer(port=8799))
    state = bridge._state(w, m.executor, m.tracker)
    flight = DashboardFlight.model_validate(state["flight"])
    expected = np.zeros(w.n, dtype=bool)
    expected[rotors[0]] = True
    assert flight.airborne == expected.tolist()
    assert all(len(row) == 8 for row in state["r"])
    src = GD.read_text()
    assert 'airborne = msg.get("flight", {}).get("airborne", [])' in src
    assert 'airborne.clear()' in src


def test_detector_overlay_uses_the_real_camera_geometry():
    """The contact overlay decides what is in field with the rig's own bearing mapping.

    `perception/camera.py` assigns each image column a fixed bearing of
    `u * tan(fov/2)`, and the dashboard inverts that to ask whether a contact falls
    inside the robot's cone at all -- `|u| <= 1` is the whole admission test, so this
    constant is what separates "the robot can see it" from "the operator can see it".
    If `fov_deg` ever changes, the overlay keeps boxing the old cone: silently, because
    it still looks like a sensor view. So the constant is pinned to the rig.
    """
    import numpy as np

    from swarmmind.perception.camera import CameraRig
    from swarmmind.sim.scenario import Scenario
    from swarmmind.sim.world import World

    body = re.search(r"const DET_FOV_TAN := ([0-9.]+)", GD.read_text(encoding="utf-8"))
    assert body, "DET_FOV_TAN not found in main.gd"
    gd_tan = float(body.group(1))

    rig = CameraRig(World(Scenario.load("test"), 1))
    assert abs(gd_tan - rig.tan_half) < 1e-6, (
        f"main.gd draws a {2 * np.degrees(np.arctan(gd_tan)):.0f} deg field; the camera "
        f"has {2 * np.degrees(np.arctan(rig.tan_half)):.0f} deg. Every box in the "
        f"detector inset would be in the wrong column."
    )

    # And the mapping itself: a contact at image column `col` must come back at the
    # normalised offset the panel places it at.
    w = rig.w
    for col in (0, w // 4, w // 2, w - 1):
        u_cam = ((col + 0.5) / w - 0.5) * 2.0          # camera.py's column -> u
        depth = 7.0
        lat = u_cam * depth * rig.tan_half             # what the rig would see there
        u_panel = lat / (depth * gd_tan)               # what main.gd computes back
        assert abs(u_panel - u_cam) < 1e-9, (
            f"column {col}: overlay u {u_panel:.6f} != camera u {u_cam:.6f}"
        )


def test_the_contact_overlay_is_limited_to_the_views_that_sit_with_a_robot():
    """Boxes belong to POV and chase, and to no other view.

    The overlay makes a claim on behalf of one machine: these are the contacts *this*
    robot holds. From the eagle-eye orbit, two hundred metres above the whole swarm,
    that claim has no viewpoint attached to it and reads as the map being annotated with
    ground truth -- which is exactly the thing this dashboard must never appear to do.
    """
    src = GD.read_text(encoding="utf-8")
    body = re.search(r"func _update_detector_hud\(\) -> void:(.*?)\nfunc ", src, re.S)
    assert body, "_update_detector_hud not found in main.gd"
    gate = re.search(r"var want: bool = (.*?)\n\tif ", body.group(1), re.S)
    assert gate, "the overlay's visibility condition is no longer a `var want: bool`"
    cond = gate.group(1)
    assert "View.POV" in cond and "View.CHASE" in cond, (
        f"the contact overlay is gated on {cond.strip()!r}, which does not name both "
        "follow views"
    )
    assert "View.ORBIT" not in cond, (
        "the contact overlay names View.ORBIT in its visibility condition; the boxes "
        "must not be drawn over the eagle-eye view"
    )
    assert "_on_unit()" in cond, (
        "the overlay must require a followed robot -- without one there is no cone to "
        "draw contacts from"
    )


def test_the_contact_box_is_the_size_of_the_casualty_it_frames():
    """`DET_BOX_M` is the drawn height of a casualty, not a round number.

    The mesh ships 1.40 m head to feet and the dashboard draws it at `VICTIM_SCALE`, so
    the box frames the body at every distance. Tie the two together: raising
    `VICTIM_SCALE` to make casualties readable from the orbit would otherwise leave the
    overlay boxing thin air around a bigger body, and that drift is invisible until
    somebody looks at a POV frame.
    """
    src = GD.read_text(encoding="utf-8")

    def const(name):
        m = re.search(rf"const {name} := ([0-9.]+)", src)
        assert m, f"{name} not found in main.gd"
        return float(m.group(1))

    mesh_m = 1.40          # CurledUpPerson.glb, as main.gd's own comment records
    assert const("DET_BOX_M") == pytest.approx(const("VICTIM_SCALE") * mesh_m, abs=0.01)


def test_activity_table_matches_the_dashboard():
    """A code the dashboard cannot name is a robot drawn as something it is not.

    This is the same cross-check EVENT_COLOURS and FX_KINDS get, for the same reason:
    the two tables are written in different languages and drift silently.
    """
    from swarmmind.contracts.schemas import ACTIVITY

    body = re.search(r"const ACTIVITY := \{(.*?)\n\}", GD.read_text(encoding="utf-8"), re.S)
    assert body, "ACTIVITY not found in main.gd"
    gd = {name: int(code)
          for name, code in re.findall(r'"(\w+)":\s*(\d+)', body.group(1))}
    assert gd == ACTIVITY, (
        "main.gd's ACTIVITY has drifted from contracts/schemas.py\n"
        f"  gd:      {sorted(gd.items())}\n"
        f"  python:  {sorted(ACTIVITY.items())}"
    )


def test_heightfield_and_occupancy_cover_the_whole_grid(payloads):
    """Godot extrudes its 3D world from these, so a short array silently truncates the
    map rather than erroring."""
    import base64

    hello = payloads["hello"]
    cells = hello["w"] * hello["h"]
    blobs = payloads["blobs"]
    assert len(base64.b64decode(blobs["height"])) == cells, "heightfield is not one byte per cell"
    assert len(base64.b64decode(blobs["occ"])) == cells, "occupancy is not one byte per cell"
    assert len(base64.b64decode(blobs["water"])) == cells, "water is not one byte per cell"
    assert hello["height_scale"] > 0.0
    assert hello["water_scale"] > 0.0


def test_the_dashboard_draws_the_water_that_stops_robots(payloads):
    """Water is the boundary between what a tracked unit can cross and what only a legged
    one can. It gated traversal for weeks without being on the wire at all, so the
    dashboard drew rivers as dry trenches and a robot stopping at the bank looked like a
    bug. A field that decides where robots may go must be visible."""
    import base64

    raw = base64.b64decode(payloads["blobs"]["water"])
    assert max(raw) > 0, "every cell is dry -- the water blob carries nothing"

    src = GD.read_text(encoding="utf-8")
    assert "_joined(\"water\")" in src, "the dashboard never reads the water blob"
    assert "water_scale" in src, "the dashboard does not dequantise water"
    assert re.search(r"func _build_water\(", src), "the dashboard never builds a surface"
    assert re.search(r"_build_water\(\)\s*$", src, re.M), "_build_water is never called"
    # The depth at which water reads as opaque is a shared display constant; drifting it
    # makes the dashboard disagree with the offline renderer that verifies it.
    from swarmmind.viz import render3d

    gd_opaque = float(re.search(r"const WATER_OPAQUE_M := ([\d.]+)", src).group(1))
    assert gd_opaque == pytest.approx(render3d.WATER_OPAQUE_M)


def test_godot_reads_the_same_lane_and_occupancy_codes():
    """GDScript hardcodes OCC_WALL / OCC_RUBBLE; they must match swarmmind.sim.grid."""
    from swarmmind.sim import grid

    src = GD.read_text(encoding="utf-8")
    wall = int(re.search(r"const OCC_WALL := (\d+)", src).group(1))
    rubble = int(re.search(r"const OCC_RUBBLE := (\d+)", src).group(1))
    assert wall == int(grid.WALL) and rubble == int(grid.RUBBLE)


def test_fog_is_bit_packed_to_the_grid(payloads):
    m = Mission(Scenario.load("test"), 42)
    import base64

    raw = base64.b64decode(payloads["fog"]["bits"])
    cells = m.world.shape[0] * m.world.shape[1]
    assert len(raw) == (cells + 7) // 8, "fog must be one bit per grid cell"


def test_truth_is_only_ever_sent_by_the_bridge(payloads):
    """God-view data. If any other node could produce this, fog of war is a slogan."""
    truth = payloads["truth"]
    assert set(truth) == {"t", "v", "hz"}
    for v in truth["v"]:
        assert len(v) == 4, "casualty rows are [x, y, state, buried]"


def test_lane_colours_agree_between_godot_and_the_png_renderer():
    """The snapshots and the live dashboard must read the same way."""
    from swarmmind.sim.robot import CHASSIS, LANES

    src = GD.read_text(encoding="utf-8")
    names = re.search(r"const LANE_NAMES := \[(.*?)\]", src, re.S).group(1)
    gd_names = re.findall(r'"([a-z]+)"', names)
    assert gd_names == ["scout", "digger", "carrier", "relay"]

    # Scope each regex to its own const block. A bare search over the whole file picks up
    # every annotated Color() in main.gd, so adding a second palette silently broke this.
    def block(const: str) -> list[str]:
        body = re.search(rf"const {const} := \[(.*?)\n\]", src, re.S)
        assert body, f"{const} not found in main.gd"
        return re.findall(r"Color\([\d.,\s]+\),\s+#\s*\d (\w+)", body.group(1))

    assert block("LANE_COLORS") == list(LANES), "lane colour order drifted"
    assert block("CHASSIS_COLORS") == list(CHASSIS), "chassis colour order drifted"


def test_contact_ring_colours_cover_every_contact_code():
    """`reports[i][2]` indexes CONTACT_COLORS directly.

    A code the palette is short of gets clamped in main.gd rather than crashing the
    dashboard, which means a new casualty state would ship as the wrong colour and
    nobody would notice until a judge asked what the ring meant.
    """
    from swarmmind.nodes.bridge import CONTACT_CODE

    src = GD.read_text(encoding="utf-8")
    body = re.search(r"const CONTACT_COLORS := \[(.*?)\n\]", src, re.S)
    assert body, "CONTACT_COLORS not found in main.gd"
    gd_order = re.findall(r"Color\([\d.,\s]+\),\s+#\s*\d (\w+)", body.group(1))

    wire_order = [n for n, _ in sorted(CONTACT_CODE.items(), key=lambda kv: kv[1])]
    assert gd_order == wire_order, "contact colour order drifted from bridge.CONTACT_CODE"
    assert sorted(CONTACT_CODE.values()) == list(range(len(CONTACT_CODE))), (
        "CONTACT_CODE must be a dense 0..n index -- it is an array subscript in main.gd"
    )

    names = re.search(r"const CONTACT_NAMES := \[(.*?)\]", src, re.S).group(1)
    assert re.findall(r'"(\w+)"', names) == wire_order, "legend labels drifted"


def test_every_mesh_path_in_gdscript_exists():
    """A `res://` path that does not resolve fails silently in Godot -- the prop simply
    never appears. Since Godot cannot be run from here, this is the only thing standing
    between a typo and a world with no ruins in it."""
    root = GD.resolve().parents[1]           # godot/
    paths = re.findall(r'"(res://[^"]+\.(?:glb|gltf|gdshader))"', GD.read_text(encoding="utf-8"))
    assert paths, "no mesh or shader paths found in main.gd at all"
    missing = [p for p in paths if not (root / p.removeprefix("res://")).exists()]
    assert not missing, f"main.gd references files that do not exist: {missing}"


def test_meshes_live_inside_the_godot_project():
    """Godot can only load resources under its own project root. A mesh in the
    top-level assets/ directory has no res:// path and cannot be referenced at all."""
    root = GD.resolve().parents[1]
    meshes = list((root / "assets").rglob("*.glb")) + list((root / "assets").rglob("*.gltf"))
    assert len(meshes) >= 40, f"only {len(meshes)} meshes under godot/assets/"


def test_gltf_external_dependencies_are_present():
    """.gltf keeps geometry in a sidecar .bin and textures as separate files. Importing
    the .gltf without them gives an empty mesh and no error worth the name."""
    import json

    root = GD.resolve().parents[1]
    for f in (root / "assets" / "nature").glob("*.gltf"):
        g = json.loads(f.read_text(encoding="utf-8"))
        for uri in ([b["uri"] for b in g.get("buffers", [])]
                    + [i["uri"] for i in g.get("images", [])]):
            assert (f.parent / uri).exists(), f"{f.name} needs missing {uri}"
        for m in g.get("materials", []):
            assert "normalTexture" not in m, f"{f.name} still carries a normal map"


# --------------------------------------------------------------------------------------
# CLAUDE.md: "New /swarm/events kinds need a dashboard handler in the same change, or they
# are invisible." A rule that is only written down is a rule that gets forgotten at 2am on
# the night before the demo, so it is a test.


def _emitted_event_kinds() -> set[str]:
    kinds: set[str] = set()
    for path in (ROOT / "swarmmind").rglob("*.py"):
        kinds |= set(re.findall(r'emit\(\s*"([a-z_]+)"', path.read_text(encoding="utf-8")))
    return kinds


def _dashboard_event_kinds() -> tuple[set[str], set[str]]:
    text = GD.read_text(encoding="utf-8")
    colours = set(re.findall(r'^\t"([a-z_]+)":\s*"#', text, re.M))
    block = re.search(r"const EVENT_SUPPRESSED := \[(.*?)\]", text, re.S)
    suppressed = set(re.findall(r'"([a-z_]+)"', block.group(1))) if block else set()
    return colours, suppressed


def test_every_event_kind_has_a_dashboard_handler():
    emitted = _emitted_event_kinds()
    coloured, suppressed = _dashboard_event_kinds()
    assert coloured, "could not parse EVENT_COLOURS out of main.gd"
    missing = sorted(emitted - coloured - suppressed)
    assert not missing, (
        "event kinds the dashboard will silently drop: " + ", ".join(missing)
        + "\n\nAdd each to EVENT_COLOURS (or EVENT_SUPPRESSED, deliberately) in main.gd."
    )


def test_the_dashboard_does_not_handle_events_that_no_longer_exist():
    """The other direction: dead handlers are how a feed quietly stops matching reality."""
    emitted = _emitted_event_kinds()
    coloured, suppressed = _dashboard_event_kinds()
    stale = sorted((coloured | suppressed) - emitted)
    assert not stale, "main.gd handles event kinds nothing emits: " + ", ".join(stale)


def test_the_dashboard_reads_the_tier3_sector_state(payloads):
    """Tier 3 has to be *visible*, not just logged.

    The dashboard was receiving sector rectangles in the hello and nothing else, so a
    directive changed allocation, changed where 768 robots went, and changed nothing a
    judge could see. `sec` carries priority and abandonment per frame; main.gd washes the
    ground with it.
    """
    state = payloads["state"]
    assert "sec" in state, "the state frame carries no sector priority"
    assert len(state["sec"]) == len(payloads["hello"]["sectors"])
    assert all(isinstance(c, int) and 0 <= c <= 3 for c in state["sec"])
    assert "sec" in _keys_read_by_gdscript("state"), "main.gd never reads `sec`"

    src = GD.read_text(encoding="utf-8")
    assert "show_sectors" in src and "sector_tex" in src
    shader = (ROOT / "godot" / "scripts" / "world_shader.gdshader").read_text(encoding="utf-8")
    assert "sector_tex" in shader, "the shader cannot see the overlay texture"


def test_the_sector_overlay_matches_the_offline_reference():
    """CLAUDE.md: anything visual is verified in render3d.py first, then ported.

    Both sides must agree on what abandonment looks like, or the frame that was checked
    offline is not the frame the judge sees.
    """
    from swarmmind.viz.render3d import Renderer3D

    amber = Renderer3D.SECTOR_TINT_ABANDONED / 255.0
    src = GD.read_text(encoding="utf-8")
    gd = re.search(r"3: Color\(([\d.]+), ([\d.]+), ([\d.]+),", src)
    assert gd, "no abandoned tint in main.gd's SECTOR_TINTS"
    for i in range(3):
        assert abs(float(gd.group(i + 1)) - amber[i]) < 0.02, (
            "the dashboard's abandoned tint has drifted from the offline reference"
        )


def test_the_chase_camera_matches_the_offline_reference():
    """The third-person follow camera, checked the same way the sector overlay is.

    CLAUDE.md: anything visual is framed in render3d.py first and the GDScript is a port
    of it. Five numbers decide that framing -- how far back, how high, what it looks at,
    how close it may get to the ground, and the angle -- and a change on one side alone
    means the dashboard no longer shows the frame anybody actually looked at.
    """
    import numpy as np

    from swarmmind.viz.render3d import Renderer3D

    assert _gd_const("CHASE_DIST") == pytest.approx(Renderer3D.CHASE_DIST)
    assert _gd_const("CHASE_ANCHOR_Z") == pytest.approx(Renderer3D.CHASE_ANCHOR_Z)
    assert _gd_const("CHASE_CLEAR") == pytest.approx(Renderer3D.CHASE_CLEARANCE)
    # main.gd holds the pitch in radians and Godot's `Camera3D.fov` is vertical, which
    # is the convention `Renderer3D.chase` converts from -- so compare the angles, not
    # the literals.
    assert np.degrees(_gd_const("CHASE_PITCH")) == pytest.approx(
        Renderer3D.CHASE_ELEV_DEG, abs=0.05)
    assert _gd_const("CHASE_FOV") == pytest.approx(Renderer3D.CHASE_FOV_V)


def test_the_chase_camera_sits_behind_the_unit_it_follows():
    """A follow camera in *front* of its subject is the one way to get this wrong that
    still renders a plausible frame: the world looks fine, the robot is in shot, and it
    is reversing through the scene the whole time. The heading sign is the same one
    `_update_robots` and the POV camera both depend on, so pin it here.
    """
    import numpy as np

    from swarmmind.sim.world import World
    from swarmmind.viz.render3d import Renderer3D

    w = World(Scenario.load("test"), 1)
    r = Renderer3D(w, 200, 120)
    elev = np.deg2rad(Renderer3D.CHASE_ELEV_DEG)
    for th in (0.0, 1.9, -2.6, 3.0):
        w.theta[0] = th
        cam = r.chase(w, 0)
        fwd = np.array([np.cos(th), np.sin(th)], np.float32)
        to_eye = cam.pos[:2] - w.pos[0, :2]
        assert float(to_eye @ fwd) < 0.0, "the chase camera is in front of the unit"
        # The clearance clamp only ever lifts the eye, so the ground distance is exact.
        assert float(np.linalg.norm(to_eye)) == pytest.approx(
            Renderer3D.CHASE_DIST * np.cos(elev), abs=1e-3)
        assert cam.target[2] > r.surface.height_at_world(*w.pos[0]), (
            "the camera looks at the ground under the unit rather than the unit")

        # Drag turns the camera around the unit and keeps looking at it: 180 deg of yaw
        # puts the eye out in front, the same distance away.
        ahead = r.chase(w, 0, yaw_off_deg=180.0)
        assert float((ahead.pos[:2] - w.pos[0, :2]) @ fwd) > 0.0
        assert float(np.linalg.norm(ahead.pos[:2] - w.pos[0, :2])) == pytest.approx(
            Renderer3D.CHASE_DIST * np.cos(elev), abs=1e-3)

    # And it never reverses into the terrain, whatever is standing behind the unit.
    for i in range(min(len(w.pos), 24)):
        cam = r.chase(w, i)
        # Clearance is against the continuous displayed triangles, not the wire's
        # cell-centred samples (which include an obsolete rubble decoration lift).
        terrain = r.surface.height_at_world(*cam.pos[:2])
        assert cam.pos[2] >= terrain + Renderer3D.CHASE_CLEARANCE - 1e-4


def test_the_dashboard_and_the_simulator_agree_on_the_contract_version(payloads):
    """`version.py` says bumping SCHEMA_VERSION requires updating the Godot parser in the
    same commit -- but until D12 the version was never sent and never checked, so nothing
    enforced that. A stale dashboard against a newer sim would have failed the M-12 way:
    silently, with both sides reporting success.
    """
    from swarmmind.contracts.version import SCHEMA_VERSION

    assert payloads["hello"].get("schema") == SCHEMA_VERSION, (
        "the bridge is not announcing the contract version"
    )

    m = re.search(r'const\s+SCHEMA_VERSION\s*:=\s*"([^"]+)"', GD.read_text(encoding="utf-8"))
    assert m, "main.gd declares no SCHEMA_VERSION -- it cannot detect a mismatch"
    assert m.group(1) == SCHEMA_VERSION, (
        f"main.gd speaks {m.group(1)}, contracts/version.py says {SCHEMA_VERSION} -- "
        "a contract change was made without updating the dashboard"
    )


# --- particle effects ---------------------------------------------------------------
#
# Same problem as the sector overlay, and the same answer: Godot cannot be run from here,
# so the model was written and looked at in swarmmind/viz/particles.py first and main.gd
# is a port of it. These tests are what keep the port a port.


def _gd_fx_kinds() -> dict[str, list[float]]:
    src = GD.read_text(encoding="utf-8")
    body = re.search(r"const FX_KINDS := \{(.*?)\n\}", src, re.S)
    assert body, "FX_KINDS not found in main.gd"
    out = {}
    for name, row in re.findall(r'"(\w+)":\s*\[([-\d.,\s]+)\]', body.group(1)):
        out[name] = [float(v) for v in row.split(",")]
    return out


def _gd_const(name: str) -> float:
    m = re.search(rf"const {name} := ([-\d.]+)", GD.read_text(encoding="utf-8"))
    assert m, f"{name} not found in main.gd"
    return float(m.group(1))


def test_state_carries_the_dig_sites_the_dashboard_draws(payloads):
    """Digging is the one stage of the rescue chain with no motion to show for itself.

    A scoop robot clearing a slab and a scoop robot sitting idle are the same box at the
    same coordinates for twenty seconds, so the lane that unburies 32 of the 80
    casualties was legible only in the event feed. `digs` is what the dust is emitted
    from, and it comes straight off the simulator's own excavation predicate rather than
    from a robot standing near a ring -- so a plume exists exactly when debris moves.
    """
    from swarmmind.sim.world import MAX_DIGGERS

    state = payloads["state"]
    assert "digs" in state, "the state frame carries no dig sites"
    for row in state["digs"]:
        assert len(row) == 3, "dig rows are [x, y, diggers]"
        assert 1 <= int(row[2]) <= MAX_DIGGERS, (
            "the digger count is the excavation rate multiplier, so it must already be "
            "capped at MAX_DIGGERS -- the dashboard scales the plume by it"
        )
    assert "digs" in _keys_read_by_gdscript("state"), "main.gd never reads `digs`"


def test_digging_is_reported_exactly_when_debris_moves():
    """`world.digging` must be rebuilt from the predicate that actually clears debris.

    A separate distance test in the bridge would drift the first time REACH_DIG or the
    lane index changed, and the failure would be silent: dust over a hole nobody is
    digging, or a dig with no dust.
    """
    m = Mission(Scenario.load("test"), 42, allocator="auction")
    w = m.world
    for _ in range(200):
        m.tick()
        for x, y, n in w.digging:
            v = min(w.victims, key=lambda v: (v.pos[0] - x) ** 2 + (v.pos[1] - y) ** 2)
            assert v.debris_remaining > 0.0 and v.buried, (
                "a dig site was reported for a casualty with nothing left to clear"
            )
            assert n >= 1
    # And it must not accumulate: a stale site would leave dust running for the mission.
    assert len(w.digging) <= len(w.victims)


def test_particle_kinds_agree_between_godot_and_the_offline_model():
    """CLAUDE.md: write the maths, look at the frame, then port it to GDScript.

    `scripts/snapshot3d.py --fx` renders swarmmind/viz/particles.py, so that module is
    the frame that was actually inspected. If main.gd's table drifts from it, the
    dashboard is showing something nobody has ever seen.
    """
    from swarmmind.viz import particles

    gd = _gd_fx_kinds()
    assert set(gd) == set(particles.KINDS), (
        f"particle species differ: main.gd has {sorted(gd)}, "
        f"particles.py has {sorted(particles.KINDS)}"
    )
    for name, k in particles.KINDS.items():
        want = [*k.colour, k.alpha, *k.life, *k.up, *k.out,
                k.gravity, k.drag, *k.size, k.grow, k.jitter]
        assert len(gd[name]) == len(want), (
            f"{name}: main.gd has {len(gd[name])} numbers, the model has {len(want)} -- "
            "the row is [r, g, b, alpha, life, up, out, gravity, drag, size, grow, jitter]"
        )
        for i, (a, b) in enumerate(zip(gd[name], want, strict=True)):
            assert a == pytest.approx(b), f"{name}[{i}] drifted: main.gd {a}, model {b}"


def test_particle_budget_agrees_with_the_offline_model():
    """The pool ceiling and the emission rates are a frame-time budget, not a look.

    They are set from the cost of the GDScript update loop, so a change on one side that
    does not reach the other means the number that was measured is not the number that
    ships.
    """
    from swarmmind.viz import particles

    assert _gd_const("FX_MAX") == particles.MAX_PARTICLES
    assert _gd_const("FX_DIG_RATE") == pytest.approx(particles.DIG_RATE)
    assert _gd_const("FX_DIG_PER_DIGGER") == pytest.approx(particles.DIG_PER_DIGGER)
    assert _gd_const("FX_DIG_CHIP_EVERY") == particles.DIG_CHIP_EVERY
    assert _gd_const("FX_CULL_M") == pytest.approx(particles.CULL_M)
    assert _gd_const("FX_EMBER_PER_M") == pytest.approx(particles.EMBER_PER_M)
    assert _gd_const("FX_EMBER_MAX_RATE") == pytest.approx(particles.EMBER_MAX_RATE)

    src = GD.read_text(encoding="utf-8")
    assert "visible_instance_count" in src, (
        "the pool must ride on visible_instance_count -- assigning instance_count "
        "reallocates the whole buffer, and this MultiMesh changes size every frame"
    )
    assert "custom_aabb" in src, (
        "without a fixed AABB Godot recomputes the MultiMesh bounds from every instance "
        "transform, which is the most expensive thing in the system"
    )
    # One fade curve for every kind, because it runs per particle per frame.
    assert "c.a *= f * f" in src, "main.gd's fade curve is not particles.fade()"


def test_hazard_embers_ride_the_god_view_toggle():
    """CLAUDE.md invariant #4, in its display form.

    The true hazard disc is ground truth and mk_hazard already draws it only when the
    operator asks. Embers scattered over the same disc are the same information in a
    prettier form, so they must ride on exactly the same condition -- not on
    `truth_hazard != null`, which is set the moment the bridge sends a truth frame.
    """
    src = GD.read_text(encoding="utf-8")
    body = re.search(r"func _emit_hazard\(.*?\n\nfunc ", src, re.S)
    assert body, "_emit_hazard not found in main.gd"
    assert "mk_hazard.visible" in body.group(0), (
        "hazard embers are not gated on the god-view toggle -- they would show the "
        "operator the true fire with the toggle off, which is what the toggle is for"
    )


def test_particle_bursts_fire_on_real_events():
    """A burst wired to an event kind nothing emits is invisible, and the event feed
    would look identical -- exactly the failure the EVENT_COLOURS cross-check exists to
    catch. Every burst needs a live kind on both sides."""
    import typing

    from swarmmind.contracts.schemas import EventKind
    from swarmmind.viz import particles

    kinds = set(typing.get_args(EventKind))
    assert set(particles.BURSTS) <= kinds, (
        f"bursts on unknown event kinds: {sorted(set(particles.BURSTS) - kinds)}"
    )

    body = re.search(r"const FX_BURSTS := \{(.*?)\n\}", GD.read_text(encoding="utf-8"), re.S)
    assert body, "FX_BURSTS not found in main.gd"
    gd = {}
    for name, rows in re.findall(r'"(\w+)":\s*\[(.+)\],\s*$', body.group(1), re.M):
        gd[name] = [(k, int(n)) for k, n in re.findall(r'\["(\w+)",\s*(\d+)\]', rows)]
    assert gd == {k: [tuple(p) for p in v] for k, v in particles.BURSTS.items()}, (
        "the dashboard's burst table has drifted from particles.BURSTS"
    )
    # A burst needs somewhere to happen. `pos` has been on the wire since the bridge was
    # written and nothing drew it, so this checks the whole chain: the emit carries a
    # position, the bridge forwards it, and main.gd uses it.
    sources = "\n".join(f.read_text(encoding="utf-8")
                        for f in (ROOT / "swarmmind").rglob("*.py"))
    for kind in particles.BURSTS:
        sites = [m.end() for m in re.finditer(rf'"{kind}"', sources)]
        assert any("pos=" in sources[i:i + 400] for i in sites), (
            f"{kind} is emitted without a position -- its burst has nowhere to happen"
        )
    assert '"pos": ev.get("pos")' in (ROOT / "swarmmind" / "nodes" / "bridge.py").read_text(
        encoding="utf-8"), "the bridge stopped forwarding event positions"
    src = GD.read_text(encoding="utf-8")
    assert 'msg.get("pos"' in src and "_fx_burst" in src, (
        "main.gd receives event positions and never fires a burst at them"
    )


# --- manual override ------------------------------------------------------------------
#
# The only traffic that flows dashboard -> simulator. Both ends are checked against each
# other here because a mismatch fails silently in the worst way: the keys do nothing and
# the badge, drawn from the echo, says AUTONOMOUS -- or NO ACK, on stage.


def test_state_carries_the_manual_echo_the_badge_is_drawn_from(payloads):
    assert payloads["state"]["manual"] == [], "nobody is driving, so the echo is empty"
    assert "manual" in _keys_read_by_gdscript("state"), "main.gd never reads `manual`"
    src = DRIVE.read_text(encoding="utf-8")
    # `[robot index, seconds until autonomy]`, indexed positionally by manual_drive.gd.
    assert "int(_echo[0])" in src and "float(_echo[1])" in src


def test_the_drive_messages_the_dashboard_sends_are_the_ones_the_simulator_reads():
    from swarmmind.control.manual import KINDS

    src = DRIVE.read_text(encoding="utf-8")
    sent = re.findall(r"host\.send\(\{(.*?)\}\)", src)
    assert sent, "manual_drive.gd sends nothing through main.gd's socket"
    kinds = set()
    for body in sent:
        keys = re.findall(r'"(\w+)":', body)
        kind = re.search(r'"t":\s*"(\w+)"', body).group(1)
        kinds.add(kind)
        expected = {"t", "robot", "v", "w"} if kind == "drive" else {"t", "robot"}
        assert set(keys) == expected, f"{kind} sends {sorted(keys)}, control/manual.py reads {sorted(expected)}"
    assert kinds == set(KINDS), f"dashboard sends {sorted(kinds)}, simulator handles {sorted(KINDS)}"
    assert re.search(r"^func send\(msg: Dictionary\) -> bool:", GD.read_text(encoding="utf-8"), re.M)


def test_the_drive_badge_and_its_modes_agree():
    """hud.gd names every ManualDrive mode it draws; a renamed mode would be a parse error
    on the projector, which is the one place Godot gets run."""
    drive = DRIVE.read_text(encoding="utf-8")
    modes = re.search(r"enum Mode \{([^}]*)\}", drive)
    assert modes, "ManualDrive.Mode not found"
    declared = {m.strip() for m in modes.group(1).split(",") if m.strip()}
    used = set(re.findall(r"ManualDrive\.Mode\.(\w+)", HUD.read_text(encoding="utf-8")))
    assert used and used <= declared, f"hud.gd uses modes {sorted(used - declared)} that do not exist"
    assert "drive.on_state(" in GD.read_text(encoding="utf-8")
