class_name SwarmDashboard
extends Node3D
##
## SwarmMind 3D dashboard.
##
## Godot uses Y-up; the simulator is X/Y with Z as height. The mapping is fixed in one
## place, world_to_godot(), and used everywhere: sim (x, y, z) -> Godot (x, z, y).
##
## The world is extruded from the same occupancy grid and heightfield the simulator
## actually uses, so what is drawn and what is simulated cannot drift. Directional
## shading is baked into vertex colours on the CPU, matching swarmmind/viz/render3d.py,
## which is the offline renderer used to verify this camera and POV maths.

# Must equal swarmmind/contracts/version.py SCHEMA_VERSION.
# tests/test_bridge_protocol.py fails the build if these drift apart.
const SCHEMA_VERSION := "1.0"
const WS_DEFAULT := "ws://127.0.0.1:8765"
const RECONNECT_EVERY := 1.0

const LANE_COLORS := [
	Color(0.35, 0.78, 1.00),   # 0 none    -- scout
	Color(1.00, 0.75, 0.24),   # 1 scoop   -- digger
	Color(0.47, 1.00, 0.55),   # 2 gripper -- carrier
	Color(0.84, 0.51, 1.00),   # 3 antenna -- relay
]
const LANE_NAMES := ["scout", "digger", "carrier", "relay"]
const CHASSIS_NAMES := ["wheeled", "tracked", "legged", "rotor"]
#: The rotor's index in the line above, which three things turn on: whether a unit can be
#: in the air, whether its camera is worth aiming, and therefore who owns the arrow keys.
const CHASSIS_ROTOR := 3
#: Colour normally encodes the *job*. `C` switches it to encode *locomotion* instead,
#: which is the view that answers "is it always the wheeled ones getting stuck?".
const CHASSIS_COLORS := [
	Color(1.00, 0.42, 0.42),   # 0 wheeled -- fast, road-bound
	Color(0.98, 0.86, 0.36),   # 1 tracked -- middling, fords shallow water
	Color(0.36, 0.90, 0.86),   # 2 legged  -- slow, goes anywhere
	Color(1.00, 0.55, 0.95),   # 3 rotor   -- flies at 2x, blind until it lands
]
const COLOR_DEAD := Color(0.42, 0.16, 0.16)
#: Contact rings, indexed by the code the bridge sends in `reports[i][2]`. The order is
#: CONTACT_CODE in swarmmind/nodes/bridge.py and nothing else; test_bridge_protocol.py
#: fails the build if they drift apart.
#:
#: Every ring used to be amber the moment a contact resolved, which made a casualty
#: still pinned under a slab and one already riding home on a carrier look identical --
#: the map read as a field of people the swarm had found and then left. The hues are a
#: rescue ramp: red for trapped, warm for waiting on a carrier, teal for on its way.
#:
#: Neighbours on the ramp are also separated by brightness (buried 0.41 -> surface 0.70
#: -> dug 0.91) or by blue (dug 0.42 -> carried 0.74), so no adjacent pair relies on the
#: red-green axis alone. The carried ring is additionally the only one that moves.
#:
#: Carried is teal rather than green because a carried casualty is *inside* a gripper
#: robot, and the gripper lane is already green: at LANE_COLORS[2] the ring and the robot
#: it encircles merged into one blob. Checked offline against a real t=130 frame -- the
#: rings are not in viz/render3d.py, so that took a throwaway splat pass over
#: `bridge._contacts` output rather than the usual snapshot script.
const CONTACT_COLORS := [
	Color(0.72, 0.78, 0.86, 0.38),   # 0 unverified -- may well be rubble; deliberately faint
	Color(0.96, 0.26, 0.22, 0.95),   # 1 buried     -- under debris, owed a digger
	Color(1.00, 0.66, 0.16, 0.95),   # 2 surface    -- in the open, owed a carrier
	Color(0.97, 0.94, 0.42, 0.95),   # 3 dug        -- debris cleared, owed a carrier
	Color(0.16, 0.88, 0.74, 0.95),   # 4 carried    -- on a carrier, heading for the zone
]
const CONTACT_NAMES := ["unverified", "buried", "surface", "dug", "carried"]

# --- particle effects ---------------------------------------------------------------
#
# Digging is the one stage of the rescue chain with nothing to show for itself. A scoop
# robot clearing a slab and a scoop robot sitting idle are the same box at the same
# coordinates for twenty seconds, so the lane that unburies 32 of the 80 casualties was
# legible only in the event feed.
#
# **This is a port of swarmmind/viz/particles.py**, which is where the maths was written
# and looked at (`scripts/snapshot3d.py --fx`), because Godot cannot be run from the
# development environment. Same integration, same per-kind numbers, same fade curve;
# tests/test_bridge_protocol.py compares the two tables field by field and fails the
# build if they drift.
#
# One MultiMesh, one draw call, a pool allocated once. The per-particle step is a handful
# of vector operations with no branches and no allocation, and `visible_instance_count`
# -- not `instance_count` -- carries the live count, so the buffer is never reallocated.

#: Pool ceiling. **Measured, not chosen** (MEASUREMENTS.md M-66): demo/seed 42 peaks at
#: 199 live particles over 4 simultaneous excavations and never reaches the cap, and
#: god-view embers add at most ~57. Mirrors MAX_PARTICLES in particles.py.
const FX_MAX := 448
const FX_DIG_RATE := 22.0
const FX_DIG_PER_DIGGER := 0.45
const FX_DIG_CHIP_EVERY := 4
#: Nothing is emitted past this range. At eagle-eye distance a 0.4 m particle is well
#: under a pixel, so this discards work that could not have been seen; particles already
#: alive finish normally, so nothing pops.
const FX_CULL_M := 190.0
#: Embers per second per metre of hazard radius, and the ceiling on that. The disc grows
#: for most of the mission and an unbounded rate would spend the whole pool on it.
#: Tumble axis. Deliberately not UP: a cube spun about the vertical keeps its flat top
#: square to an overhead camera the whole time and reads as a sliding tile rather than a
#: chunk of masonry going end over end.
#: Written out normalised -- it is (1, 2, 1)/sqrt(6). `Basis(axis, angle)` fails an
#: assertion on an axis that is not unit length, and a const cannot call `.normalized()`.
const FX_TUMBLE := Vector3(0.4082483, 0.8164966, 0.4082483)
const FX_EMBER_PER_M := 0.6
const FX_EMBER_MAX_RATE := 26.0

#: One row per species: [r, g, b, alpha, life_min, life_max, up_min, up_max, out_min,
#: out_max, gravity, drag, size_min, size_max, grow, jitter]. Metres and seconds.
#: KINDS in swarmmind/viz/particles.py is the same table and the test compares them.
#:
#: Colours are from the **display** palette, not perception/raster.py. The detector's
#: world is dark and low-contrast by design; dust tuned for a conv net would be invisible
#: on a projector in a lit room.
const FX_KINDS := {
	# Excavation dust: warm, pale, slow -- pulverised concrete, light enough to read
	# against both DISPLAY_RUBBLE ground and shadow.
	"dust":  [0.87, 0.83, 0.74, 0.62, 1.4, 2.4, 2.2, 4.5, 0.8, 2.2, 1.6, 1.1, 0.34, 0.62, 0.8, 0.7],
	# Rubble thrown clear of the hole. Ballistic, dark, and it does not expand. Dust
	# alone reads as a smoke machine; chips are what say debris is being moved.
	"chip":  [0.34, 0.28, 0.22, 0.95, 0.7, 1.2, 2.2, 4.2, 0.8, 2.6, 9.8, 0.15, 0.12, 0.22, 0.0, 0.35],
	# A robot dying: smoke rising (negative gravity is buoyancy) over falling sparks.
	"smoke": [0.2, 0.19, 0.18, 0.7, 1.4, 2.4, 1.4, 3.0, 0.5, 1.8, -0.6, 1.2, 0.45, 0.85, 1.3, 0.6],
	"spark": [1.0, 0.56, 0.16, 1.0, 0.5, 1.0, 3.0, 7.0, 1.5, 4.5, 11, 0.2, 0.1, 0.18, 0.0, 0.3],
	# A casualty reaching an extraction zone: the one unambiguously good event in the
	# mission, and the only effect that rises without falling back. Matches the
	# victim_rescued colour in the feed so the burst and the log line agree.
	"lift":  [0.47, 1.0, 0.55, 0.85, 1.1, 1.8, 2.6, 5.0, 0.3, 1.2, -1.1, 0.9, 0.16, 0.3, 0.25, 0.9],
	# The slab coming off: one broad, slow puff, bigger and paler than working dust so
	# the end of a dig is distinguishable from the middle of one.
	"puff":  [0.92, 0.89, 0.81, 0.62, 1.3, 2.1, 0.6, 1.8, 1.4, 3.2, 1.4, 1.9, 0.45, 0.8, 1.1, 0.8],
	# Hazard embers, gated on the god-view toggle exactly like the hazard disc itself --
	# the true fire is ground truth and the operator only sees it with the toggle on.
	# Power drawn at the base pad. The only cool colour in the table -- every other
	# effect is warm, so a charging robot reads at a glance even in a crowd. Brief
	# and gentle: a top-up is a pause, not an event, and must not compete with a
	# rescue burst. Mirrors "charge" in particles.py.
	"charge": [0.55, 0.82, 1.0, 0.9, 0.6, 1.0, 1.6, 3.2, 0.6, 1.6, -0.9, 0.8, 0.1, 0.2, 0.3, 0.55],
	"ember": [1.0, 0.42, 0.13, 0.85, 1.6, 2.8, 2.0, 5.0, 0.2, 1.0, -1.4, 0.7, 0.14, 0.28, 0.1, 1.2],
}

#: One-shot bursts, keyed by `/swarm/events` kind. An event with no entry gets no
#: particles. BURSTS in particles.py is the same table.
const FX_BURSTS := {
	"victim_rescued": [["lift", 22], ["dust", 8]],
	"victim_cleared": [["puff", 14], ["chip", 6]],
	"robot_destroyed": [["smoke", 14], ["spark", 18]],
	"robot_recharged": [["charge", 12]],
}

# Seconds to keep a robot drawn as out-of-contact after it reconnects.
#
# The simulator is right to flip status the instant contact is lost -- a robot that drops
# out for half a second genuinely cannot report during it. But the swarm makes ~3.7
# connect/disconnect transitions per second (M-48), so drawing every one of them turns the
# fleet into a shimmer and reads as "most of them are disconnected" when the instantaneous
# figure is 13% and the median robot never drops out at all. This is a display hold, in
# the same spirit as the separate display palette: what the operator sees and what the
# detector sees are different problems. Set to 0.0 to draw the raw state.
const COMMS_HOLD_S := 1.0

# Display palette. Deliberately not the appearance raster the detector sees: that one is
# dark and low-contrast because it is tuned for computer vision, not for human eyes.
# Desaturated, dust-covered. Matches DISPLAY_* in swarmmind/viz/render3d.py so the
# offline verification frames are a truthful preview of this.
const C_FLOOR := Color(0.408, 0.396, 0.337)
const C_RUBBLE := Color(0.455, 0.373, 0.286)
const C_WALL := Color(0.588, 0.573, 0.529)
const C_SKY := Color(0.694, 0.725, 0.698)
const LIGHT_DIR := Vector3(0.45, 0.35, 0.82)

const OCC_WALL := 1
const OCC_RUBBLE := 2

# Water, drawn as a surface at ground + depth. Ported from swarmmind/viz/render3d.py --
# WATER_SHALLOW / WATER_DEEP / WATER_OPAQUE_M there are these three, and the offline
# renderer is where this was checked before it was written here.
const C_WATER_SHALLOW := Color(0.361, 0.424, 0.392)
const C_WATER_DEEP := Color(0.149, 0.227, 0.298)
#: Depth at which water reaches its deep colour. Rivers run to ~1.1 m and marsh to ~0.4,
#: so the two tiers of barrier -- the one only legged units cross and the one tracked
#: units can wade -- are visibly different tiers rather than one flat blue.
const WATER_OPAQUE_M := 0.85

var url := WS_DEFAULT
var socket := WebSocketPeer.new()
var state_name := "connecting"
var _retry := 0.0
var ready_world := false

# --- world -------------------------------------------------------------------------
var cell := 1.0
var gw := 0
var gh := 0
var map_w := 0.0
var map_h := 0.0
var heights := PackedFloat32Array()
var water := PackedFloat32Array()
var reference_landscape: Dictionary = {}
var surface: TerrainSurface
var _blobs := {}
var _height_scale := 1.0 / 255.0
var _water_scale := 1.0 / 255.0
var _blob_progress := ""

var occ := PackedByteArray()
var victims_total := 0
var zones: Array = []
var scenario_name := ""
#: Robot ids from the hello, in the same order as the state frame's rows.
var robot_ids: Array = []
var airborne: Array = []
var sector_ids: Array = []

# Casualty mesh remains separate from streamed procedural ruin geometry.
const VICTIM_MESH := "res://assets/victims/CurledUpPerson.glb"

#: The mesh ships life-size: 1.40 m from head to feet as it lies curled up. That is
#: legible up close and almost nothing from a 260 m orbit, so it is drawn slightly
#: oversized -- 2.5 m, a little larger than the 1.4 m robot boxes, which is what
#: separates a casualty from a robot at a glance without looking like a giant in the
#: robot-POV camera.
const VICTIM_SCALE := 1.8

var props: PropStream
var markers: Node3D
var mk_contact: MultiMeshInstance3D
var mk_victim: MultiMeshInstance3D
var mk_body: MultiMeshInstance3D
var mk_burial: MultiMeshInstance3D
var mk_hazard: MeshInstance3D
var ground: TerrainStream
var _terrain_revision := -1
var bots: UnitFleet
var mat: ShaderMaterial
var fog_tex: ImageTexture
var fog_img: Image
var sector_tex: ImageTexture
var sector_img: Image
#: robot index -> wall time until which it stays drawn as out of contact (COMMS_HOLD_S).
var _out_until: Dictionary = {}
var _now: float = 0.0
var sector_rects: Array = []
var sector_codes: Array = []
var show_sectors := true

# Tier 3's decisions, as a wash over the terrain. Ported from Renderer3D.SECTOR_TINT and
# checked against runs/3d/orbit_sectors.png before it landed here. Amber, not red, for
# abandonment: red is the hazard, and "burning" and "written off" are different claims.
const SECTOR_TEX_N := 96
const SECTOR_TINTS := {
	0: Color(0.47, 0.78, 1.00, 0.30),   # high
	1: Color(0.00, 0.00, 0.00, 0.00),   # normal -- no wash at all
	2: Color(0.35, 0.35, 0.43, 0.28),   # low
	3: Color(1.00, 0.82, 0.29, 0.36),   # abandoned
}

# --- live --------------------------------------------------------------------------
var robots: Array = []
var reports: Array = []
#: `[x, y, diggers]` per casualty being actively excavated, straight from the simulator's
#: own excavation predicate (`world.digging`). Held between state frames on purpose --
#: digging is continuous and the plume must not stutter at 10 Hz -- and cleared on
#: disconnect so a dropped socket does not leave dust pouring out of a frozen map.
var digs: Array = []
var truth_victims: Array = []
var truth_hazard = null
var hud := {}
var sim_time := 0.0

# --- view --------------------------------------------------------------------------
#: Three ways to watch one robot, walked in this order by F. The orbit shows the swarm,
#: the chase shows one unit *doing* something, and the POV shows what it can actually
#: see -- which is the argument the fog-of-war claim rests on, so it stays the last rung
#: rather than the first.
enum View { ORBIT, POV, CHASE }
const VIEW_COUNT := 3
const VIEW_NAMES := ["orbit", "POV", "chase"]

var cam: Camera3D
#: What the camera is looking at this frame. The HUD projects its reticle and its
#: metres-per-pixel readout from this, so they describe the real shot.
var cam_target := Vector3.ZERO
var orbit_yaw := 0.9
var orbit_pitch := 0.62
var orbit_dist := 260.0
var follow := -1
var view_mode: int = View.ORBIT

# Third-person follow. Verified in swarmmind/viz/render3d.py first (`Renderer3D.chase`,
# runs/3d/chase*.png) and ported here -- same anchor height, same azimuth convention,
# same terrain clearance. Its distance is its own variable, not `orbit_dist`: the two
# live two orders of magnitude apart and sharing one would make every F press a jump
# cut, and would lose the operator's orbit framing every time they looked at a unit.
const CHASE_DIST := 4.0
const CHASE_DIST_MIN := 1.8
const CHASE_DIST_MAX := 30.0
#: 0.30 rad = 17.2 deg above the unit. Shallow on purpose: the shot is about the ground
#: the robot is walking into, and a steeper camera turns it into a small overhead orbit.
const CHASE_PITCH := 0.30
#: Vertical, which is what `Camera3D.fov` means under the default `KEEP_HEIGHT`. The
#: offline reference converts it to the horizontal angle its own renderer wants -- at 5 m
#: reading one as the other changes the framing by a third.
const CHASE_FOV := 55.0
#: The chassis box is 1.2 m tall centred 0.6 m up, so this is its waist: the unit sits
#: on the centre of the frame rather than sinking to the bottom of it.
const CHASE_ANCHOR_Z := 0.7
#: Keep the eye this far above whatever terrain is under it. A follow camera reversing
#: into a hillside is the classic third-person failure and the heightfield is right here.
const CHASE_CLEAR := 0.7
#: Exponential catch-up rates. State frames land at 10 Hz and the robots in them jump
#: between poses; a camera bolted straight to a 5 m anchor judders at every frame. The
#: heading is slower than the position because a robot spinning on the spot should not
#: whip the camera round with it.
const CHASE_POS_RATE := 14.0
const CHASE_YAW_RATE := 5.0

#: Radians a second the arrow keys swing a drone's camera. Slower than a mouse drag on
#: purpose: a key held is a shot being composed, not a glance.
const CAM_KEY_YAW := 1.2
const CAM_KEY_PITCH := 0.9
#: How far a drone's own view may look off its heading. Short of the tail on both sides:
#: past that its own rotor booms fill the frame and the horizon it is flying against is
#: gone, which is the thing the shot is for.
const POV_LOOK_YAW := 2.2
const POV_LOOK_DOWN := -1.1
const POV_LOOK_UP := 0.6

#: Where a drone's first-person camera is looking, relative to the unit's own heading:
#: (yaw, pitch) in radians, both zero for everything else. The arrow keys move it --
#: see `_camera_keys` for why a rotor is the one chassis that gets them.
var pov_look := Vector2.ZERO
#: Which robot `pov_look` belongs to, so swinging the camera round one drone does not
#: leave the next unit picked up staring off at nothing.
var _pov_of := -1

#: Drag offset from *behind the unit's heading*, not an absolute compass bearing -- so
#: the camera keeps its shot as the robot turns instead of being left staring at a flank.
var chase_yaw := 0.0
var chase_pitch := CHASE_PITCH
var chase_dist := CHASE_DIST
#: Right-drag framing offset, in the camera's own screen plane.
var chase_pan := Vector3.ZERO
var _chase_anchor := Vector3.ZERO
var _chase_az := 0.0
#: Which robot `_chase_anchor` belongs to. A change means snap, not fly: smoothing a
#: switch across a 260 m map is a second of staring at rubble.
var _chase_of := -1
var god_view := false
var show_victims := false
var thermal_on := false
var thermal: ThermalVision
#: WASD / arrow-key override of the followed unit -- scripts/manual_drive.gd.
var drive: ManualDrive
var audio: SwarmAudio
var colour_by_chassis := false
var _drag := false
var _pan := false
# Ground-plane offset added to the orbit centre by right-drag. Reset by eagle-eye.
var pan_off := Vector3.ZERO

#: Full-screen, transparent, drawn under the rest of the HUD. Not a panel any more --
#: the boxes go over the contacts themselves, out in the world.
var det_hud: Control
#: Resolved once, not looked up per call per frame in `_draw`.
var _det_font: Font = null
#: Rails, bars and the viewport frame -- scripts/hud.gd.
var ui: SwarmHud

# --- particle pool ------------------------------------------------------------------
# Parallel packed arrays rather than an array of objects, for the same reason robot state
# is parallel numpy arrays on the Python side: this is the one loop in the dashboard that
# runs over hundreds of items every frame. Live particles occupy [0, fx_n); death is a
# swap-remove, so nothing here may depend on particle order.
var fx: MultiMeshInstance3D
var fx_on := true
var fx_n := 0
var fx_pos := PackedVector3Array()
var fx_vel := PackedVector3Array()
var fx_col := PackedColorArray()
var fx_age := PackedFloat32Array()
var fx_life := PackedFloat32Array()
var fx_size := PackedFloat32Array()
var fx_grow := PackedFloat32Array()
var fx_grav := PackedFloat32Array()
var fx_drag := PackedFloat32Array()
var fx_spin := PackedFloat32Array()
#: Fractional particles owed to each dig site, so a rate of 1.4 a frame emits 1 and 2
#: alternately instead of rounding to the same number and quietly halving the plume.
#: Rebuilt from the live dig list on every state frame, so it cannot grow without bound.
var fx_carry := {}
var fx_ember_carry := 0.0
#: Seeded, and entirely separate from the simulator's streams. A rehearsed demo should
#: look the same on the second run as on the first.
var fx_rng := RandomNumberGenerator.new()


func world_to_godot(x: float, y: float, z: float) -> Vector3:
	return Vector3(x, z, y)


func _ready() -> void:
	if OS.has_environment("SWARMMIND_WS"):
		url = OS.get_environment("SWARMMIND_WS")
	for a in OS.get_cmdline_user_args():
		if a.begins_with("ws://"):
			url = a

	var env := WorldEnvironment.new()
	var e := Environment.new()
	e.background_mode = Environment.BG_COLOR
	# Match the shader's fog colour so geometry dissolves into the horizon rather than
	# ending against a visible edge.
	e.background_color = C_SKY
	e.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	# Ambient tinted toward the overcast sky and turned well down. At white and full
	# energy every lit material renders as flat colour, which is what made the imported
	# ruins read as grey mud instead of stone.
	e.ambient_light_color = C_SKY
	e.ambient_light_energy = 0.42
	env.environment = e
	add_child(env)

	# A real sun. The ground and debris use the unshaded world shader with baked vertex
	# shading, but the imported meshes carry their own lit materials and have nothing to
	# catch without this -- no highlights, no shadow side, no silhouette.
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-38.0, 142.0, 0.0)
	sun.light_color = Color(1.0, 0.93, 0.82)
	sun.light_energy = 1.25
	sun.shadow_enabled = true
	# Shadow casters are culled past this, which is what keeps ~1,600 prop instances
	# affordable: distant ruins are lit but do not cast.
	sun.directional_shadow_max_distance = 140.0
	add_child(sun)

	cam = Camera3D.new()
	cam.far = 4000.0
	add_child(cam)
	cam.make_current()

	_build_ui()
	thermal = ThermalVision.new()
	add_child(thermal)
	thermal.setup(self)
	drive = ManualDrive.new()
	add_child(drive)
	drive.setup(self)
	audio = SwarmAudio.new()
	add_child(audio)
	audio.setup(self)
	_connect_ws()


func _connect_ws() -> void:
	state_name = "connecting"
	# Godot's default 64 KB inbound buffer silently drops the connection when a single
	# frame exceeds it, with no error on either side -- which looks exactly like the
	# simulator not running. The bridge chunks the map payload well under that limit;
	# this is belt and braces.
	socket.inbound_buffer_size = 1 << 22
	socket.max_queued_packets = 8192
	if socket.connect_to_url(url) != OK:
		state_name = "cannot reach %s" % url


# A Control whose only job is to call back into this script, so the drawing code lives
# next to the state it draws instead of in a second file that has to be kept in sync.
class DetectorHUD extends Control:
	var host = null

	func _draw() -> void:
		if host != null:
			host._draw_detector(self)


func _build_ui() -> void:
	ui = SwarmHud.new()
	add_child(ui)

	# Added first so it sits *under* every other HUD element: a contact box may land
	# anywhere in the view, and the rails and the event stream have to stay readable
	# over it.
	det_hud = DetectorHUD.new()
	det_hud.host = self
	det_hud.mouse_filter = Control.MOUSE_FILTER_IGNORE
	ui.add_child(det_hud)
	# After `add_child`: the preset resolves against the parent rect, and the viewport is
	# only the parent once it is in the tree. It then tracks the window on resize, which
	# a fixed size would not -- the projector is not the size of this screen. Full-rect
	# rather than the view hole, because it draws in the camera's projection space; it
	# confines itself to `ui.view_rect()`.
	det_hud.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)

	# The contact legend, key help and mission counters now live in the HUD's rails, all
	# built from this script's palettes so a swatch cannot drift from what it describes.
	ui.build(self)


# --- network -----------------------------------------------------------------------


func _process(delta: float) -> void:
	_now += delta
	socket.poll()
	var st := socket.get_ready_state()
	if st != WebSocketPeer.STATE_OPEN:
		audio.reset()
	if st == WebSocketPeer.STATE_OPEN:
		state_name = "connected"
		while socket.get_available_packet_count() > 0:
			var msg = JSON.parse_string(socket.get_packet().get_string_from_utf8())
			if msg is Dictionary:
				_handle(msg)
	elif st == WebSocketPeer.STATE_CLOSED:
		state_name = "waiting for simulator at %s" % url
		# A dropped socket freezes the last state frame. Robots stopping is honest; dust
		# still pouring out of a hole nobody is digging any more is not.
		if not digs.is_empty():
			_set_digs([])
		_retry += delta
		if _retry >= RECONNECT_EVERY:
			_retry = 0.0
			socket = WebSocketPeer.new()
			_connect_ws()

	_camera_keys(delta)
	_update_camera(delta)
	if ready_world:
		var full_terrain := view_mode != View.ORBIT and _on_unit()
		ground.update_view(cam, full_terrain)
		props.update_view(cam, full_terrain)
	if bots != null:
		if ground != null and ground.revision != _terrain_revision:
			bots.invalidate_grounding()
			_terrain_revision = ground.revision
		# The followed shell is hidden only at the camera inside it. Chase and
		# orbit retain the complete model; stale telemetry freezes its mechanisms.
		var hidden_robot := follow if view_mode == View.POV and _on_unit() else -1
		bots.advance(_now, st == WebSocketPeer.STATE_OPEN, hidden_robot)
		bots.update_view(cam, follow)
	# After the camera: the overlay projects world points through it, so it wants this
	# frame's pose, not last frame's.
	_update_detector_hud()
	_step_fx(delta)
	thermal.update_view(self)
	ui.tick(delta)


func _handle(msg: Dictionary) -> void:
	match msg.get("t", ""):
		"hello":
			_on_hello(msg)
		"state":
			robots = msg.get("r", [])
			# Additive contracts.schemas.DashboardFlight; old recordings stay grounded.
			airborne = msg.get("flight", {}).get("airborne", [])
			_update_sectors(msg.get("sec", []))
			audio.on_state(msg.get("sec", []))
			reports = msg.get("reports", [])
			_set_digs(msg.get("digs", []))
			hud = msg.get("hud", {})
			sim_time = msg.get("time", 0.0)
			# `[robot index, seconds until autonomy]` while the operator has a unit, else
			# empty. An older simulator omits it, and a held drive key then reads NO ACK.
			drive.on_state(msg.get("manual", []))
			_update_robots(true)
			_update_markers()
			ui.on_state()
		"blob":
			_on_blob(msg)
		"hello_done":
			_assemble()
		"fog":
			_on_fog(msg.get("bits", ""))
		"truth":
			truth_victims = msg.get("v", [])
			truth_hazard = msg.get("hz", null)
			audio.on_truth(truth_hazard)
			_update_markers()
		"event":
			_on_event(msg)


func link_up() -> bool:
	return socket.get_ready_state() == WebSocketPeer.STATE_OPEN


func send(msg: Dictionary) -> bool:
	"""One JSON command to the simulator -- the only outbound traffic on the socket.

	Dropped, not queued, while the link is down: every command the dashboard sends is a
	live control input, and replaying stale ones after a reconnect is worse than losing them.
	"""
	if not link_up():
		return false
	return socket.send_text(JSON.stringify(msg)) == OK


func _on_hello(msg: Dictionary) -> void:
	# A reconnect/reset starts a fresh trajectory even when the roster is identical.
	audio.reset()
	ready_world = false
	robots.clear()
	airborne.clear()
	truth_victims.clear()
	truth_hazard = null
	reports.clear()
	_out_until.clear()
	if bots != null:
		bots.reset_motion()
	var got := str(msg.get("schema", "?"))
	if got != SCHEMA_VERSION:
		_log("[color=#ff6b6b]CONTRACT MISMATCH[/color]  dashboard speaks %s, simulator speaks %s -- update godot/scripts/main.gd" % [SCHEMA_VERSION, got])
	cell = msg.get("cell", 1.0)
	gw = int(msg.get("w", 0))
	gh = int(msg.get("h", 0))
	var mm: Array = msg.get("map_m", [0, 0])
	map_w = mm[0]
	map_h = mm[1]
	victims_total = int(msg.get("victims_total", 0))
	zones = msg.get("zones", [])
	scenario_name = str(msg.get("scenario", "?"))
	sector_rects.clear()
	sector_ids.clear()
	for sec in (msg.get("sectors", []) as Array):
		sector_rects.append(sec.get("r", [0, 0, 0, 0]))
		sector_ids.append(str(sec.get("id", "?")))
	sector_codes.clear()
	robot_ids.clear()
	for rb in (msg.get("robots", []) as Array):
		robot_ids.append(str(rb.get("id", "?")))
	_height_scale = msg.get("height_scale", 1.0 / 255.0)
	_water_scale = msg.get("water_scale", 1.0 / 255.0)
	_blobs.clear()
	ui.on_hello()
	_log("[color=#7fd1ff]connected[/color]  %s  %d robots  %d casualties" % [
		scenario_name, robot_ids.size(), victims_total])


func _on_blob(msg: Dictionary) -> void:
	var name := str(msg.get("name", ""))
	if not _blobs.has(name):
		_blobs[name] = {"parts": {}, "n": int(msg.get("n", 1))}
	_blobs[name]["parts"][int(msg.get("i", 0))] = str(msg.get("data", ""))
	var have := 0
	var want := 0
	for k in _blobs:
		have += (_blobs[k]["parts"] as Dictionary).size()
		want += int(_blobs[k]["n"])
	_blob_progress = "receiving map %d/%d" % [have, want]


func _joined(name: String) -> String:
	if not _blobs.has(name):
		return ""
	var e: Dictionary = _blobs[name]
	var parts: Dictionary = e["parts"]
	var out := ""
	for i in range(int(e["n"])):
		out += str(parts.get(i, ""))
	return out


func _assemble() -> void:
	var hraw := Marshalls.base64_to_raw(_joined("height"))
	occ = Marshalls.base64_to_raw(_joined("occ"))
	if hraw.size() < gw * gh or occ.size() < gw * gh:
		_log("[color=#ff5c1c]map payload incomplete: height %d, occ %d, expected %d[/color]"
			% [hraw.size(), occ.size(), gw * gh])
		return
	heights.resize(gw * gh)
	for i in range(gw * gh):
		heights[i] = float(hraw[i]) * _height_scale

	# Water is optional on the wire: an older simulator that does not send it leaves the
	# array empty and every cell reads as dry, rather than failing to build a map.
	var wraw := Marshalls.base64_to_raw(_joined("water"))
	water.resize(gw * gh)
	for i in range(gw * gh):
		water[i] = float(wraw[i]) * _water_scale if wraw.size() > i else 0.0

	_build_world()
	_eagle_eye()
	ui.on_world()
	ready_world = true
	_blob_progress = ""
	_log("[color=#7fd1ff]world built[/color]  %d x %d cells  %.0f x %.0f m" % [
		gw, gh, map_w, map_h])


func h_at(ix: int, iy: int) -> float:
	ix = clampi(ix, 0, gw - 1)
	iy = clampi(iy, 0, gh - 1)
	return heights[iy * gw + ix]


# --- world construction --------------------------------------------------------------


func _build_world() -> void:
	mat = ShaderMaterial.new()
	mat.shader = load("res://scripts/world_shader.gdshader")
	mat.set_shader_parameter("map_size", Vector2(map_w, map_h))
	fog_img = Image.create(gw, gh, false, Image.FORMAT_L8)
	fog_img.fill(Color(0, 0, 0))
	fog_tex = ImageTexture.create_from_image(fog_img)
	mat.set_shader_parameter("fog_tex", fog_tex)
	sector_img = Image.create(SECTOR_TEX_N, SECTOR_TEX_N, false, Image.FORMAT_RGBA8)
	sector_img.fill(Color(0, 0, 0, 0))
	sector_tex = ImageTexture.create_from_image(sector_img)
	mat.set_shader_parameter("sector_tex", sector_tex)
	mat.set_shader_parameter("show_sectors", show_sectors)

	surface = TerrainSurface.new()
	reference_landscape = preload("res://scripts/reference_landscape.gd").load_layout(
		scenario_name, Vector2(gw*cell, gh*cell))
	surface.setup(heights, occ, water, gw, gh, cell, reference_landscape)
	_build_ground()
	_build_water()
	_build_props()
	_build_markers()
	_build_bots()
	_build_fx()


func _build_ground() -> void:
	if ground:
		ground.queue_free()
	ground = TerrainStream.new()
	add_child(ground)
	ground.setup(surface, gw, gh, mat)


func _build_water() -> void:
	# Wet cells stream with their terrain tile, sharing continuous shoreline vertices.
	# Keep water's fog-of-war/sector settings in step with the land material.
	ground.sync_material()


func _surface_height(x: float, y: float) -> float:
	return ground.height_at_world(x, y) if ground != null else 0.0


func _robot_surface_pose(x: float, y: float, heading: float, chassis: int) -> Transform3D:
	return ground.robot_pose(x, y, heading, chassis)


func _robot_flight_pose(x: float, y: float, heading: float, chassis: int) -> Transform3D:
	return surface.robot_pose(x, y, heading, chassis, 1, true)


func _is_airborne(i: int) -> bool:
	return i >= 0 and i < robots.size() and i < airborne.size() and bool(airborne[i]) \
		and int(robots[i][6]) == CHASSIS_ROTOR and int(robots[i][4]) < 2


func _unit_pose(i: int, detailed: bool = false) -> Transform3D:
	var r: Array = robots[i]
	if _is_airborne(i):
		return _robot_flight_pose(r[0], r[1], r[2], int(r[6]))
	var pose := ground.robot_pose(r[0], r[1], r[2], int(r[6]))
	if detailed:
		pose.origin.y = maxf(pose.origin.y, surface.robot_pose(r[0], r[1], r[2], int(r[6])).origin.y)
	return pose


func _mesh_of(path: String) -> Mesh:
	"""Godot imports glTF as a PackedScene, so pull the Mesh out for MultiMesh use."""
	var res := load(path)
	if res == null:
		_log("[color=#ff5c1c]missing mesh %s[/color]" % path)
		return null
	if res is Mesh:
		return res
	var inst := (res as PackedScene).instantiate()
	var found: Mesh = null
	var stack: Array = [inst]
	while stack.size() > 0 and found == null:
		var node = stack.pop_back()
		if node is MeshInstance3D:
			found = (node as MeshInstance3D).mesh
		for c in node.get_children():
			stack.append(c)
	inst.queue_free()
	if found == null:
		_log("[color=#ff5c1c]no mesh inside %s[/color]" % path)
	return found


func _build_props() -> void:
	if props:
		props.queue_free()
	props = PropStream.new()
	add_child(props)
	props.setup(surface, occ, gw, gh, cell, mat, ground, reference_landscape)


func _marker_multimesh(mesh: Mesh, colour: Color,
		per_instance: bool = false) -> MultiMeshInstance3D:
	# `per_instance` is opt-in: `set_instance_color` errors on a MultiMesh that has not
	# declared `use_colors`, and the victim and hazard markers want one fixed colour.
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.use_colors = per_instance
	mm.mesh = mesh
	mm.instance_count = 0
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.vertex_color_use_as_albedo = per_instance
	mat.albedo_color = Color.WHITE if per_instance else colour
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	# Markers are an overlay on the world, not part of it: they must read through
	# rubble and haze, which is the whole point of a ground-truth toggle.
	mat.no_depth_test = true
	mat.render_priority = 4
	var node := MultiMeshInstance3D.new()
	node.multimesh = mm
	node.material_override = mat
	node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	return node


func _build_markers() -> void:
	"""Contacts the swarm believes in, and -- on the ground-truth toggle -- where the
	casualties actually are.

	This existed in the 2D dashboard and was dropped in the 3D rewrite: the bridge kept
	sending `truth` and `reports`, and nothing drew them. It is element 8 of the MVP
	dashboard spec, and without it the fog/god-view comparison has nothing to compare.
	"""
	if markers:
		markers.queue_free()
	mk_body = null
	markers = Node3D.new()
	add_child(markers)

	# Two nodes per casualty, because they answer different questions. The pin is an
	# overlay -- no depth test, so it reads through rubble and haze, which is the whole
	# point of a ground-truth toggle. The body is part of the world and depth-tests
	# normally: it is a solid mesh, and drawing it without a depth test would let its
	# far side overwrite its near side. Pin says *where*; body says *what*.
	var pin := CylinderMesh.new()
	pin.top_radius = 0.0
	pin.bottom_radius = 0.8
	pin.height = 3.0
	mk_victim = _marker_multimesh(pin, Color(1.0, 0.30, 0.30, 0.9))
	markers.add_child(mk_victim)

	# A missing mesh must not blank the god view, so the body is additive: if the glTF
	# fails to load the pins alone still mark every casualty, exactly as before.
	var body := _mesh_of(VICTIM_MESH)
	if body != null:
		var bmm := MultiMesh.new()
		bmm.transform_format = MultiMesh.TRANSFORM_3D
		bmm.mesh = body
		bmm.instance_count = 0
		var bm := StandardMaterial3D.new()
		bm.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		# The mesh carries albedo x directional shading in its vertex colours, baked
		# against LIGHT_DIR at import so it matches the unshaded world and viz/render3d.
		# `albedo_color` multiplies over that: casualty red, with the form still legible.
		bm.vertex_color_use_as_albedo = true
		bm.albedo_color = Color(1.0, 0.55, 0.52)
		# Cluster decimation leaves a handful of inverted faces; two-sided is cheaper
		# than trying to fix winding on a photogrammetry scan.
		bm.cull_mode = BaseMaterial3D.CULL_DISABLED
		mk_body = MultiMeshInstance3D.new()
		mk_body.multimesh = bmm
		mk_body.material_override = bm
		mk_body.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		markers.add_child(mk_body)

	# Rubble is real depth-tested geometry, not a coloured pin. One small shared mesh
	# covers bodies until the existing state reports excavation complete.
	var rubble := MultiMesh.new()
	rubble.transform_format = MultiMesh.TRANSFORM_3D
	rubble.mesh = Burial.build_mesh()
	rubble.instance_count = 0
	mk_burial = MultiMeshInstance3D.new()
	mk_burial.multimesh = rubble
	mk_burial.material_override = mat
	mk_burial.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	markers.add_child(mk_burial)

	var ring := TorusMesh.new()
	ring.inner_radius = 1.9
	ring.outer_radius = 2.5
	mk_contact = _marker_multimesh(ring, Color(1, 1, 1, 0.75), true)
	markers.add_child(mk_contact)

	var cyl := CylinderMesh.new()
	cyl.top_radius = 1.0
	cyl.bottom_radius = 1.0
	cyl.height = 1.0
	mk_hazard = MeshInstance3D.new()
	mk_hazard.mesh = cyl
	var hm := StandardMaterial3D.new()
	hm.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	hm.albedo_color = Color(1.0, 0.36, 0.11, 0.22)
	hm.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	hm.cull_mode = BaseMaterial3D.CULL_DISABLED
	mk_hazard.material_override = hm
	mk_hazard.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	markers.add_child(mk_hazard)


func _update_markers() -> void:
	if markers == null:
		return

	# The bridge sends a CONTACT_CODE in r[2]: 0 = unverified (the swarm believes
	# something is there, and 56% of the time it is rubble -- M-8), 1-4 = resolved onto a
	# real casualty and coloured by what that casualty needs next.
	#
	# Two things used to be wrong here at once, and the first hid the second. This
	# renderer ignored r[2] and drew every ring alike, and the bridge sent one ring per
	# *report* rather than per casualty -- 817 rings over 26 bodies at t=60 (M-64). The
	# stack of coincident rings is deduped bridge-side now, so the ring count is finally
	# the number of casualties the swarm knows about, and hue separates trapped from
	# in-transit: the scoop and gripper lanes doing their job is visible on the map
	# instead of only in the event log. Size still separates belief from fact.
	var cmm := mk_contact.multimesh
	cmm.instance_count = reports.size()
	for i in range(reports.size()):
		var r: Array = reports[i]
		# An unknown code must still draw. A ring in the wrong colour is a cosmetic bug;
		# an out-of-range index is a crash mid-demo.
		var code: int = clampi(int(r[2]), 0, CONTACT_COLORS.size() - 1)
		var resolved: bool = code != 0
		var lift: float = 2.2 if resolved else 1.2
		var b := Basis().scaled(Vector3.ONE * (1.8 if resolved else 0.9))
		cmm.set_instance_transform(i, Transform3D(b, world_to_godot(r[0], r[1], _surface_height(r[0], r[1]) + lift)))
		cmm.set_instance_color(i, CONTACT_COLORS[code])

	# Ground truth: operator only. Nothing in the swarm can see this.
	var show := show_victims or god_view
	var buried_sites: Array = []
	if show:
		for row: Array in truth_victims:
			if Burial.needs_excavation(row):
				buried_sites.append(row)
	else:
		# Normal operator view uses confirmed buried contacts only. Unknown truth
		# positions must not reveal casualties before the swarm finds them.
		for report: Array in reports:
			if int(report[2]) == 1:
				buried_sites.append(report)
	mk_burial.multimesh.instance_count = buried_sites.size()
	for i in range(buried_sites.size()):
		var site: Array = buried_sites[i]
		mk_burial.multimesh.set_instance_transform(i, _casualty_pose(site[0], site[1]))
	mk_victim.visible = show
	if mk_body:
		mk_body.visible = show
	mk_hazard.visible = show and truth_hazard != null
	if show:
		var vmm := mk_victim.multimesh
		var bmm: MultiMesh = mk_body.multimesh if mk_body else null
		var live: Array = []
		for v in truth_victims:
			if int(v[2]) != 4:            # not yet rescued
				live.append(v)
		vmm.instance_count = live.size()
		if bmm:
			bmm.instance_count = live.filter(func(row: Array) -> bool: return int(row[2]) < 3).size()
		var body_index := 0
		for i in range(live.size()):
			var v: Array = live[i]
			var ground := _surface_height(v[0], v[1])
			# Only casualties still awaiting excavation get the lower pin and body.
			# The pin is 3.0 m tall and drawn point-down, so the centre sits 1.5 m above
			# where the tip lands: 0.9 m over a buried casualty, 2.1 m over a surface one.
			var buried := Burial.needs_excavation(v)
			var lift: float = 2.4 if buried else 3.6
			var t := Transform3D(Basis(Vector3.RIGHT, PI),
				world_to_godot(v[0], v[1], ground + lift))
			vmm.set_instance_transform(i, t)
			if bmm and int(v[2]) < 3:
				# Yaw is cosmetic -- the simulator has no victim heading -- but it must
				# be stable, so it comes from the position rather than an RNG that would
				# make bodies spin as the array reorders.
				bmm.set_instance_transform(body_index, _casualty_body_pose(v[0], v[1], buried))
				body_index += 1
		if truth_hazard != null:
			var hx: float = truth_hazard[0]
			var hy: float = truth_hazard[1]
			var hr: float = maxf(float(truth_hazard[2]), 0.1)
			var b := Basis().scaled(Vector3(hr * 2.0, 12.0, hr * 2.0))
			mk_hazard.transform = Transform3D(b, world_to_godot(hx, hy, 6.0))


func _casualty_pose(x: float, y: float) -> Transform3D:
	var yaw := fposmod(x * 1.7 + y * 3.1, TAU)
	if ground != null:
		return ground.robot_pose(x, y, yaw, 0)
	return Transform3D(Basis(Vector3.UP, -yaw), Vector3(x, 0.0, y))


func _casualty_body_pose(x: float, y: float, buried: bool) -> Transform3D:
	var pose := _casualty_pose(x, y)
	pose.origin += pose.basis.y * (Burial.BURIED_LIFT if buried else Burial.SURFACE_LIFT)
	pose.basis = pose.basis.scaled(Vector3.ONE * VICTIM_SCALE)
	return pose


func _build_bots() -> void:
	if bots:
		bots.queue_free()
	bots = UnitFleet.new()
	add_child(bots)
	bots.load_models(map_w, map_h, _height_scale * 255.0)
	if not robots.is_empty():
		_update_robots(true)


func _update_robots(state_frame: bool = false) -> void:
	if bots == null:
		return
	if state_frame:
		bots.sync_state(robots, digs, sim_time, _now, h_at, cell, gw, gh,
			_robot_surface_pose, _surface_height, airborne, _robot_flight_pose)
	for i in range(robots.size()):
		var r: Array = robots[i]
		var status := int(r[4])
		var col: Color = COLOR_DEAD
		if status < 2:
			col = CHASSIS_COLORS[int(r[6])] if colour_by_chassis else LANE_COLORS[int(r[3])]
		# Activity before comms: a robot that is out of contact AND idle should read as
		# out of contact, because that is the cause and the idleness is the symptom.
		if status < 2 and r.size() > 7:
			var tint = ACTIVITY_TINT.get(int(r[7]))
			if tint != null:
				col = col.lerp(tint as Color, ACTIVITY_TINT_MIX)
		if status == 1:
			_out_until[i] = _now + COMMS_HOLD_S
		var dim := 1.0
		if status == 1 or _out_until.get(i, 0.0) > _now:
			dim = 0.45
		bots.set_unit_color(i, col, dim)


# --- particles ----------------------------------------------------------------------


func _build_fx() -> void:
	"""One MultiMesh for every effect on the map. Allocated once, never resized.

	A cube, not a billboarded quad. A quad is the prettier sprite and it is also the one
	that vanishes when the camera catches it edge-on -- and the camera here is under the
	operator's hand, orbiting and dropping into robot POV. A 12-triangle cube reads the
	same from every angle, tumbles for free, and 448 of them is 5,376 triangles against
	the 5.4M the scattered props already submit. The cost here is the update loop, not
	the geometry.
	"""
	if fx:
		fx.queue_free()
	fx_n = 0
	fx_carry.clear()
	fx_ember_carry = 0.0
	fx_rng.seed = 20260918

	fx_pos.resize(FX_MAX)
	fx_vel.resize(FX_MAX)
	fx_col.resize(FX_MAX)
	fx_age.resize(FX_MAX)
	fx_life.resize(FX_MAX)
	fx_size.resize(FX_MAX)
	fx_grow.resize(FX_MAX)
	fx_grav.resize(FX_MAX)
	fx_drag.resize(FX_MAX)
	fx_spin.resize(FX_MAX)

	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.use_colors = true
	var cube := BoxMesh.new()
	cube.size = Vector3.ONE
	mm.mesh = cube
	mm.instance_count = FX_MAX
	# The live count rides on `visible_instance_count`. Assigning `instance_count` every
	# frame -- the obvious way, and what the robot MultiMesh does because its size only
	# changes on connect -- reallocates the whole instance buffer, and this one changes
	# size dozens of times a second.
	mm.visible_instance_count = 0

	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.vertex_color_use_as_albedo = true
	m.albedo_color = Color.WHITE
	m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	# Depth *test* on, depth *write* off -- which is what TRANSPARENCY_ALPHA already
	# does, so this is a note rather than a setting. Dust behind a wall must be hidden by
	# it (these are particles in the world, not an overlay on it like the contact rings,
	# which set no_depth_test), but a puff must not occlude the puff behind it or a plume
	# reads as a stack of discs.

	fx = MultiMeshInstance3D.new()
	fx.multimesh = mm
	fx.material_override = m
	fx.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	# Godot recomputes a MultiMesh's bounds from every instance transform whenever one
	# changes. With hundreds changing per frame that is the most expensive thing in this
	# system, and it buys nothing: a fixed box over the whole map culls just as correctly.
	fx.custom_aabb = AABB(Vector3(-8.0, -40.0, -8.0),
		Vector3(map_w + 16.0, 200.0, map_h + 16.0))
	add_child(fx)


func _set_digs(rows: Array) -> void:
	"""Adopt the state frame's dig sites, keeping each site's fractional-particle carry.

	Rebuilding the dictionary rather than updating it is what bounds it: sites appear and
	vanish as casualties are uncovered, and a carry left behind for every hole ever dug
	would grow for the whole mission.
	"""
	digs = rows
	var keep := {}
	for d in digs:
		var key := _fx_key(float(d[0]), float(d[1]))
		keep[key] = fx_carry.get(key, 0.0)
	fx_carry = keep


func _fx_key(x: float, y: float) -> int:
	# The bridge rounds dig positions to 0.1 m, so this is exact rather than a hash.
	return int(roundf(x * 10.0)) * 100000 + int(roundf(y * 10.0))


func _step_fx(dt: float) -> void:
	"""Emit, integrate and upload, in one pass over the live particles.

	Explicit Euler. Over a one-second life of ballistic dust it is visually identical to
	anything better and it is what the interpreter can afford at 60 Hz.
	"""
	if fx == null:
		return
	if not fx_on:
		if fx_n > 0:
			fx_n = 0
			fx.multimesh.visible_instance_count = 0
		return

	_emit_digs(dt)
	_emit_hazard(dt)

	var mm := fx.multimesh
	var i := 0
	while i < fx_n:
		var age := fx_age[i] + dt
		if age >= fx_life[i]:
			fx_n -= 1
			_fx_move(fx_n, i)
			continue            # the survivor moved into slot i has not been stepped yet
		fx_age[i] = age
		var v := fx_vel[i]
		v.y -= fx_grav[i] * dt
		v *= maxf(0.0, 1.0 - fx_drag[i] * dt)
		fx_vel[i] = v
		var pos: Vector3 = fx_pos[i] + v * dt
		fx_pos[i] = pos
		# Quadratic ease-out, one curve for every kind: a per-kind curve would put a
		# branch or a table lookup in the hot loop, and (1-u)^2 already reads as dust
		# thinning rather than dust switching off.
		var f := 1.0 - age / fx_life[i]
		var c: Color = fx_col[i]
		c.a *= f * f
		var sz := maxf(fx_size[i] + fx_grow[i] * age, 0.01)
		mm.set_instance_transform(i, Transform3D(
			Basis(FX_TUMBLE, fx_spin[i] * age).scaled(Vector3.ONE * sz), pos))
		mm.set_instance_color(i, c)
		i += 1
	mm.visible_instance_count = fx_n


func _emit_digs(dt: float) -> void:
	if digs.is_empty() or cam == null:
		return
	var eye := cam.global_position
	var cull := FX_CULL_M * FX_CULL_M
	for d in digs:
		var x := float(d[0])
		var y := float(d[1])
		# Just above the ground rather than at it, so the first frame of a particle is
		# not half-buried in the terrain mesh.
		var at := world_to_godot(x, y, _surface_height(x, y) + 0.15)
		if eye.distance_squared_to(at) > cull:
			continue
		# Sub-linear in the digger count on purpose: world.MAX_DIGGERS caps the
		# excavation rate at three, and the plume tracks the rate, so a fourth machine
		# parked on the same hole must not make the dust grow.
		var rate := FX_DIG_RATE * (1.0 + FX_DIG_PER_DIGGER * float(maxi(int(d[2]), 1) - 1))
		var key := _fx_key(x, y)
		var want := float(fx_carry.get(key, 0.0)) + rate * dt
		var count := int(want)
		fx_carry[key] = want - float(count)
		if count <= 0:
			continue
		var chips := count / FX_DIG_CHIP_EVERY
		_fx_spawn("dust", at, count - chips)
		_fx_spawn("chip", at, chips)


func _emit_hazard(dt: float) -> void:
	"""Embers over the fire -- and only when the operator has asked to see the fire.

	The true hazard disc is ground truth, drawn by mk_hazard on the same toggle. Embers
	outside it would be a second, prettier ground-truth leak, so they ride on exactly the
	same condition.
	"""
	if truth_hazard == null or mk_hazard == null or not mk_hazard.visible:
		return
	var hr := maxf(float(truth_hazard[2]), 0.1)
	var rate := minf(hr * FX_EMBER_PER_M, FX_EMBER_MAX_RATE)
	fx_ember_carry += rate * dt
	var count := int(fx_ember_carry)
	fx_ember_carry -= float(count)
	var hx := float(truth_hazard[0])
	var hy := float(truth_hazard[1])
	for _n in range(count):
		# Weighted to the rim: that is the edge the swarm has to route around, and it is
		# the part of the disc that moves.
		var bearing := fx_rng.randf_range(0.0, TAU)
		var r := hr * (0.55 + 0.45 * fx_rng.randf())
		var x := hx + cos(bearing) * r
		var y := hy + sin(bearing) * r
		_fx_spawn("ember", world_to_godot(x, y, _surface_height(x, y) + 0.4), 1)


func _fx_burst(event_kind: String, x: float, y: float) -> void:
	var at := world_to_godot(x, y, _surface_height(x, y) + 0.5)
	for pair in (FX_BURSTS[event_kind] as Array):
		_fx_spawn(str(pair[0]), at, int(pair[1]))


func _fx_spawn(kind: String, at: Vector3, count: int) -> void:
	if not fx_on or fx == null or count <= 0:
		return
	var k: Array = FX_KINDS[kind]
	# Never more than the pool holds, however large a burst grows later.
	count = mini(count, FX_MAX)
	var room := FX_MAX - fx_n
	if count > room:
		_fx_recycle(count - room)
	for _n in range(count):
		var i := fx_n
		fx_n += 1
		var bearing := fx_rng.randf_range(0.0, TAU)
		# sqrt of a uniform, so the scatter is even over the disc rather than piled at
		# its centre.
		var jr: float = float(k[15]) * sqrt(fx_rng.randf())
		fx_pos[i] = at + Vector3(cos(bearing) * jr,
			fx_rng.randf_range(0.0, 0.35), sin(bearing) * jr)
		var launch := fx_rng.randf_range(0.0, TAU)
		var out := fx_rng.randf_range(float(k[8]), float(k[9]))
		fx_vel[i] = Vector3(cos(launch) * out,
			fx_rng.randf_range(float(k[6]), float(k[7])), sin(launch) * out)
		fx_age[i] = 0.0
		fx_life[i] = fx_rng.randf_range(float(k[4]), float(k[5]))
		fx_size[i] = fx_rng.randf_range(float(k[12]), float(k[13]))
		fx_grow[i] = float(k[14])
		fx_grav[i] = float(k[10])
		fx_drag[i] = float(k[11])
		fx_spin[i] = fx_rng.randf_range(-4.0, 4.0)
		fx_col[i] = Color(float(k[0]), float(k[1]), float(k[2]), float(k[3]))


func _fx_recycle(count: int) -> void:
	"""Free slots by killing the particles furthest through their lives.

	Recycling the oldest rather than refusing the newest is what keeps an over-subscribed
	field looking like a thinner field instead of a frozen one. `count` is at most one
	burst (22), so the repeated scan is bounded and only runs on a frame that overflows.
	"""
	for _n in range(count):
		if fx_n == 0:
			return
		var worst := 0
		var worst_u := -1.0
		for i in range(fx_n):
			var u := fx_age[i] / maxf(fx_life[i], 0.001)
			if u > worst_u:
				worst_u = u
				worst = i
		fx_n -= 1
		_fx_move(fx_n, worst)


func _fx_move(from: int, to: int) -> void:
	if from == to:
		return
	fx_pos[to] = fx_pos[from]
	fx_vel[to] = fx_vel[from]
	fx_col[to] = fx_col[from]
	fx_age[to] = fx_age[from]
	fx_life[to] = fx_life[from]
	fx_size[to] = fx_size[from]
	fx_grow[to] = fx_grow[from]
	fx_grav[to] = fx_grav[from]
	fx_drag[to] = fx_drag[from]
	fx_spin[to] = fx_spin[from]


func _on_fog(b64: String) -> void:
	if gw == 0 or fog_img == null:
		return
	var bits := Marshalls.base64_to_raw(b64)
	var data := PackedByteArray()
	data.resize(gw * gh)
	for idx in range(gw * gh):
		# numpy packbits is MSB-first.
		data[idx] = 255 if ((bits[idx >> 3] >> (7 - (idx & 7))) & 1) == 1 else 0
	fog_img = Image.create_from_data(gw, gh, false, Image.FORMAT_L8, data)
	fog_tex.update(fog_img)
	ui.on_fog(data)


# Every event kind the swarm can emit is named here, including the ones deliberately
# dropped. tests/test_bridge_protocol.py cross-checks this list against the emit() calls
# in Python and fails the build on a kind that has no entry -- an event with no handler
# is an event nobody at the demo will ever see.
const EVENT_SUPPRESSED := ["task_awarded"]
# What a robot is doing. **Mirrors ACTIVITY in swarmmind/contracts/schemas.py**, and
# test_bridge_protocol.py compares the two tables -- a code this side cannot name is a
# robot drawn as something it is not.
#
# This exists because a still box was ambiguous. A relay on its post is motionless for
# the rest of the mission *on purpose* -- it is the link a dozen robots report through --
# and it looked exactly like one wedged against a rock. On seed 42, of ~80 stationary
# robots ~32 were working correctly and ~42 were stuck.
const ACTIVITY := {
	"idle": 0,
	"drift": 1,
	"explore": 2,
	"investigate": 3,
	"dig": 4,
	"extract": 5,
	"carry": 6,
	"relay_post": 7,
	"relay_move": 8,
	"retreat": 9,
	"recover": 10,
	"recharge": 11,
}

# The two activities worth calling out in the crowd, and nothing else -- lane colour is
# already carrying most of the signal and a second full palette would fight it.
#
#   idle      the robot has no task AND nowhere useful to go. The only genuinely bad
#             state on the map, so it is the only one tinted toward alarm.
#   carry     a casualty is being carried. The one unambiguously good thing happening,
#             matching the victim_rescued green so the map and the feed agree.
const ACTIVITY_TINT := {
	0: Color(1.0, 0.35, 0.35),
	6: Color(0.47, 1.0, 0.55),
}

# How far to pull a robot's colour toward its activity tint. Low: this is a hint on top
# of lane colour, not a replacement for it.
const ACTIVITY_TINT_MIX := 0.45

# Read back out for the follow-cam, so clicking a still robot says why it is still.
const ACTIVITY_NAMES := [
	"idle", "walking to dark ground", "exploring", "investigating a contact",
	"clearing debris", "en route to a casualty", "carrying a casualty",
	"holding a relay post", "moving the relay net", "retreating",
	"out of contact, rejoining", "recharging",
]

# --- detector view ------------------------------------------------------------------
#
# The contacts the followed robot believes in, boxed **over the world itself** -- each
# box sits on the thing it describes, in the live 3D view, the way a sensor overlay on a
# real vehicle would. It replaces an inset panel in the corner that drew the same
# contacts in their own little sensor frame: a second, smaller picture of the world the
# viewer was already looking at, which asked them to map one onto the other themselves.
#
# **Which contacts** is still the robot's own camera geometry, and that is the honest
# part: `perception/camera.py` assigns each image column a fixed bearing of
# `u * tan(fov/2)`, so a contact is in field when `|lat / (depth * tan(fov/2))| <= 1`
# within range. Nothing outside that cone is drawn, however plainly the operator's camera
# can see it -- the overlay is what the *robot* has, not what the screen shows.
#
# **Where each box lands** is then the operator camera's projection, not the robot's:
# `unproject_position` puts it over the contact as rendered. The two geometries are
# different on purpose, and only the first one makes a claim about perception.
#
# Boxes do not depth-test against the terrain -- a canvas overlay cannot -- so a contact
# behind a wall still draws. That is the truthful reading anyway: these are beliefs held
# by the swarm, not pixels the detector is looking at right now.
#
# It carries no new data: robot pose and contact positions are both already on the wire.
# That matters -- `contracts/` is frozen, and a detector view that needed a new field
# would not be shippable at all.
const DET_FOV_TAN := 1.0          # tan(45 deg): CameraRig(fov_deg=90) in camera.py
const DET_MAX_RANGE := 26.0       # contacts beyond this are out of the detector's field
#: Side of the drawn box, in metres, centred `DET_LIFT` above the ground under the
#: contact. **This is the drawn size of a casualty** -- `VICTIM_SCALE` (1.8) on a mesh
#: that ships 1.40 m head to feet -- so the box frames the body rather than a round
#: number that happens to look right at one distance. Checked against offline frames at
#: 7 m and 25 m (MEASUREMENTS.md M-75) before it landed here.
#:
#: Fixed rather than tied to the contact ring's radius: the rings scale with how resolved
#: a contact is (0.9 vs 1.8), and a box that changed size with certainty rather than with
#: distance would read as a depth cue that is not there. The ring stays the ground
#: marker; the box marks the thing standing on it.
const DET_BOX_M := 2.52
const DET_LIFT := 1.4
#: A box smaller than this is a smudge; hold it at a legible minimum and let the label
#: carry the range instead.
const DET_BOX_MIN_PX := 14.0
#: Most boxes drawn at once, nearest first. Beyond this the screen is a thicket and the
#: thing the operator is being shown -- which contact is near -- is lost in it.
const DET_MAX_BOXES := 24

const EVENT_COLOURS := {
	"victim_rescued": "#78ff8c",
	"victim_found": "#5ac8ff",
	"victim_cleared": "#a0ffb4",
	"robot_destroyed": "#ff5c1c",
	"robot_out_of_comms": "#ff9a3c",
	"robot_reconnected": "#7ad6a0",
	# Cool blue, matching the charge particles so the feed line and the burst agree.
	"robot_recharged": "#6ec8ff",
	"task_orphaned": "#ffbe3c",
	"report_dismissed": "#8790a1",
	"hazard_ignited": "#ff3c3c",
	# Tier 3. The reasoning line is the one a judge actually reads, so it gets the
	# strongest colour on the feed; a rejection is a *good* moment and is coloured to be
	# noticed rather than hidden.
	"directive_issued": "#c8a0ff",
	"directive_rejected": "#ff78d2",
	"sector_abandoned": "#ffd24a",
	"hivemind_offline": "#8790a1",
	# The operator's own hands on one unit. The same amber the drive badge and the
	# "operator driving" log line use, so everything they do reads as one voice.
	"operator_action": "#ffbf3d",
}


func _on_event(msg: Dictionary) -> void:
	audio.on_event(msg)
	var kind := str(msg.get("kind", ""))
	if kind in EVENT_SUPPRESSED:
		return
	# Directives and filter rejections are tagged ISSUED / FILTER in the stream, and also
	# go to the HUD's hivemind panel, where they do not scroll away under the dismissals.
	ui.on_event(kind, float(msg.get("time", 0.0)), str(msg.get("text", "")),
		str(EVENT_COLOURS.get(kind, "#c8d0dd")))

	# The three events with somewhere to happen also get a burst. `pos` has been on the
	# wire since the bridge was written and nothing drew it: a robot destroyed on the
	# far side of the map was a line of orange text and no change to the map at all.
	var at = msg.get("pos", null)
	if at is Array and (at as Array).size() >= 2 and FX_BURSTS.has(kind):
		_fx_burst(kind, float(at[0]), float(at[1]))


func _update_sectors(codes: Array) -> void:
	# Rebuilt only when Tier 3 actually changes something. The codes arrive at 10 Hz and
	# change a few times a mission; rasterising 96x96 on every frame for that would be
	# pure waste.
	if sector_img == null or codes.is_empty() or codes == sector_codes:
		return
	sector_codes = codes.duplicate()
	sector_img.fill(Color(0, 0, 0, 0))
	if map_w <= 0.0 or map_h <= 0.0:
		return
	for k in range(mini(codes.size(), sector_rects.size())):
		var tint: Color = SECTOR_TINTS.get(int(codes[k]), Color(0, 0, 0, 0))
		if tint.a <= 0.0:
			continue
		var r: Array = sector_rects[k]
		var x0 := int(floor(float(r[0]) / map_w * SECTOR_TEX_N))
		var y0 := int(floor(float(r[1]) / map_h * SECTOR_TEX_N))
		var x1 := int(ceil(float(r[2]) / map_w * SECTOR_TEX_N))
		var y1 := int(ceil(float(r[3]) / map_h * SECTOR_TEX_N))
		for y in range(maxi(y0, 0), mini(y1, SECTOR_TEX_N)):
			for x in range(maxi(x0, 0), mini(x1, SECTOR_TEX_N)):
				sector_img.set_pixel(x, y, tint)
	sector_tex.update(sector_img)


func _log(line: String) -> void:
	ui.log_system(line)


func _update_detector_hud() -> void:
	"""Show the contact overlay in the two views that sit with a robot, and nowhere else.

	**POV and chase only.** Both of those are a viewpoint at the unit: what it can see
	from where it is standing. The orbit is the operator two hundred metres up looking at
	the whole swarm, and boxing one robot's contacts up there would be labelling the map
	with a claim that belongs to a single machine.

	**Redrawn every frame while visible**, unlike the inset panel this replaces, which
	redrew only when a state frame landed. It has to be: every box is projected through
	the operator camera, and that camera moves continuously -- the chase smooths toward
	its anchor on every frame, and drag, zoom and pan all move it between state frames.
	Boxes pinned to a stale projection would swim off their contacts on every mouse move.
	"""
	if det_hud == null:
		return
	var want: bool = (view_mode == View.POV or view_mode == View.CHASE) \
		and _on_unit() and ready_world
	if want != det_hud.visible:
		det_hud.visible = want
	if want:
		det_hud.queue_redraw()


func _draw_detector(c: Control) -> void:
	"""Box every contact the followed robot has in field, over the contact itself.

	Two geometries, and the order matters. *Whether* a contact is drawn comes from the
	robot's own camera -- the same `lat / (depth * tan(fov/2))` bearing test
	`perception/camera.py` uses to assign a column, inside `DET_MAX_RANGE`. *Where* it is
	drawn comes from the operator camera's projection of the contact's world position, so
	the box lands on the ring and the casualty under it rather than in a separate frame.
	Range on the label is the robot's, not the camera's: it is the detector's reading.
	"""
	if not _on_unit() or not ready_world or cam == null:
		return
	if _is_airborne(follow):
		return  # Rotor cameras only inspect when landed, just like Mission._perceive.
	var r: Array = robots[follow]
	var bx: float = float(r[0])
	var by: float = float(r[1])
	var th: float = float(r[2])
	var ct: float = cos(th)
	var st: float = sin(th)

	var vp: Vector2 = c.size
	if vp.x < 2.0 or vp.y < 2.0:
		return                 # first frame, before the layout pass has sized us
	# The part of the window the 3D view actually shows through -- the hole between the
	# HUD rails. Boxes, cues and labels keep inside it; projection is still the camera's,
	# which covers the whole window, so positions come out in the same space.
	var screen: Rect2 = ui.view_rect()
	if screen.size.x < 2.0 or screen.size.y < 2.0:
		return
	if _det_font == null:
		# The HUD's own mono face, so the boxes and the rails read as one instrument.
		_det_font = ui.mono_font()
	var font: Font = _det_font
	var ink := Color(0.55, 0.95, 1.0, 0.85)
	var faint := Color(0.55, 0.95, 1.0, 0.45)

	# No frame of its own: the HUD draws corner brackets round the view in every mode.

	# Bore sight, first person only. At the window centre, not the hole's: that is the
	# camera's optical axis, which is where the robot is looking. From the robot's own eye the screen centre is where
	# the robot is looking; from the chase camera it is just the middle of a picture.
	# Not while a drone's camera is swung off its heading: the screen centre is then the
	# camera's axis and not the robot's, and a cross drawn there claims a bore sight the
	# unit does not have.
	if view_mode == View.POV and pov_look == Vector2.ZERO:
		var mid: Vector2 = vp * 0.5
		c.draw_line(mid - Vector2(9, 0), mid - Vector2(3, 0), faint, 1.0)
		c.draw_line(mid + Vector2(3, 0), mid + Vector2(9, 0), faint, 1.0)
		c.draw_line(mid - Vector2(0, 9), mid - Vector2(0, 3), faint, 1.0)
		c.draw_line(mid + Vector2(0, 3), mid + Vector2(0, 9), faint, 1.0)

	# Contacts inside the robot's cone, nearest first and capped.
	#
	# The cap is a drawing budget, not a policy: every contact in field is counted in the
	# readout, but past ~24 boxes the screen is a thicket.
	var in_field: Array = []
	for rp in reports:
		var qx: float = float(rp[0]) - bx
		var qy: float = float(rp[1]) - by
		var qd: float = qx * ct + qy * st
		if qd <= 0.4 or qd > DET_MAX_RANGE:
			continue
		if absf((-qx * st + qy * ct) / (qd * DET_FOV_TAN)) > 1.0:
			continue
		in_field.append([qd, rp])
	in_field.sort_custom(func(a, b): return a[0] < b[0])
	var total_in_field: int = in_field.size()
	if in_field.size() > DET_MAX_BOXES:
		in_field.resize(DET_MAX_BOXES)

	var drawn: int = 0
	var cued: int = 0
	for pair in in_field:
		var rp = pair[1]
		var depth: float = pair[0]
		var centre: Vector3 = world_to_godot(rp[0], rp[1], _surface_height(rp[0], rp[1]) + DET_LIFT)
		# A contact behind the camera unprojects to a point that is numerically fine and
		# geometrically nonsense -- in chase view, swung round to face the unit, that is
		# most of them.
		if cam.is_position_behind(centre):
			continue
		# Size from the projection rather than from `1/range`: the camera's own maths
		# already knows its FOV and aspect, and projecting the top of the box is exact
		# where a reciprocal is a guess that drifts as the operator zooms.
		var p: Vector2 = cam.unproject_position(centre)
		var edge: Vector2 = cam.unproject_position(centre + Vector3.UP * (DET_BOX_M * 0.5))
		var side: float = clampf(absf(p.y - edge.y) * 2.0, DET_BOX_MIN_PX, vp.y * 0.9)
		var box := Rect2(p - Vector2(side, side) * 0.5, Vector2(side, side))

		var code: int = clampi(int(rp[2]), 0, CONTACT_COLORS.size() - 1)
		var col: Color = CONTACT_COLORS[code]
		# The unverified ring is deliberately faint on the map, which is right for a
		# marker and too weak for a line of type. Lift the overlay's alpha off the floor
		# so an unverified box is still a box.
		col = Color(col.r, col.g, col.b, maxf(col.a, 0.8))

		if not screen.intersects(box):
			_draw_edge_cue(c, font, p, col, depth, screen)
			cued += 1
			continue
		drawn += 1

		c.draw_rect(box, Color(col.r, col.g, col.b, 0.07), true)
		c.draw_rect(box, col, false, 1.6)
		# Ticks on the box corners: a tracking reticle, and it keeps small distant boxes
		# legible when the outline alone is only a few pixels a side.
		var t: float = minf(side * 0.28, 11.0)
		for cn in [[box.position, Vector2(1, 1)],
				[box.position + Vector2(side, 0), Vector2(-1, 1)],
				[box.position + Vector2(0, side), Vector2(1, -1)],
				[box.position + Vector2(side, side), Vector2(-1, -1)]]:
			var o2: Vector2 = cn[0]
			var d2: Vector2 = cn[1]
			c.draw_line(o2, o2 + Vector2(t * d2.x, 0), col, 2.0)
			c.draw_line(o2, o2 + Vector2(0, t * d2.y), col, 2.0)

		# Every box carries its label. Small distant boxes get a shorter form and a
		# smaller face rather than no label at all -- an unlabelled box is a shape, and
		# the whole point is that each one says what it is and how far.
		var lat: float = -(float(rp[0]) - bx) * st + (float(rp[1]) - by) * ct
		var bearing: float = rad_to_deg(atan2(lat, depth))
		var label: String
		var fsize: int
		if side > 46.0:
			label = "%s  %.0fm  %+.0f deg" % [CONTACT_NAMES[code], depth, bearing]
			fsize = 12
		elif side > 26.0:
			label = "%s %.0fm" % [CONTACT_NAMES[code], depth]
			fsize = 11
		else:
			label = "%.0fm" % depth
			fsize = 10
		# Above the box, unless that would run off the top of the view.
		var ly: float = box.position.y - 4.0
		if ly < screen.position.y + float(fsize) + 2.0:
			ly = box.position.y + side + float(fsize)
		# Keep it in the view horizontally so the text is never half-hidden under a rail.
		var lw: float = float(label.length()) * float(fsize) * 0.55
		var lx: float = clampf(box.position.x, screen.position.x + 4.0,
			maxf(screen.end.x - 4.0 - lw, screen.position.x + 4.0))
		# A dark backing plate: these labels sit over rubble, sky and fog in turn, and
		# coloured type on its own is unreadable against at least one of them.
		c.draw_rect(Rect2(lx - 3.0, ly - float(fsize), lw + 6.0, float(fsize) + 5.0),
			Color(0.02, 0.05, 0.07, 0.55), true)
		c.draw_string(font, Vector2(lx, ly), label,
			HORIZONTAL_ALIGNMENT_LEFT, -1, fsize, col)

	# One readout, centred at the foot of the screen, saying what the overlay is and what
	# it is holding back. The cone is computed from the constant the rig pins rather than
	# written out, so it cannot drift from the geometry above.
	var more: String = ""
	if total_in_field > drawn + cued:
		more = "  (+%d)" % (total_in_field - drawn - cued)
	var off: String = ""
	if cued > 0:
		off = "   %d off-frame" % cued
	var foot: String = "DETECTOR   %d boxed%s%s   %.0f deg cone   %.0f m" % [
		drawn, off, more, rad_to_deg(2.0 * atan(DET_FOV_TAN)), DET_MAX_RANGE]
	# Above the HUD's footer strip, which carries the cursor and selection readouts.
	c.draw_string(font, Vector2(screen.position.x, screen.end.y - ui.strip_px() - 8.0), foot,
		HORIZONTAL_ALIGNMENT_CENTER, screen.size.x, 11, ink)


func _draw_edge_cue(c: Control, font: Font, p: Vector2, col: Color, depth: float,
		view: Rect2) -> void:
	"""A chevron on the screen edge for a contact in field but outside the frame.

	This case is not an edge case. The detector is **flat** -- perception is 2.5D and
	`world.height` is render-only -- so its cone admits a contact by bearing alone, while
	the dashboard draws that contact on top of real relief. Over 5 m of open ground the
	demo map moves 1.1 m in height on average and 3.7 m at the 95th percentile, so a
	robot standing at the foot of a slope genuinely detects things well above its own
	view. Dropping them silently would make the overlay look broken exactly where the
	terrain is most interesting; a chevron says `it is over there, this far away`, which
	is all the flat detector knows anyway.
	"""
	var mid: Vector2 = view.get_center()
	var d: Vector2 = p - mid
	if d.length() < 1.0:
		return
	# Walk the direction out until it meets an inset rectangle, so the cue sits on the
	# edge nearest to where the contact actually is.
	var half: Vector2 = view.size * 0.5 - Vector2(34.0, 34.0)
	var s: float = minf(half.x / maxf(absf(d.x), 0.001), half.y / maxf(absf(d.y), 0.001))
	var at: Vector2 = mid + d * s
	var dir: Vector2 = d.normalized()
	var n := Vector2(-dir.y, dir.x)
	c.draw_polygon(PackedVector2Array([at + dir * 9.0, at - dir * 4.0 + n * 7.0,
		at - dir * 4.0 - n * 7.0]), PackedColorArray([col, col, col]))
	var lbl: String = "%.0fm" % depth
	var lw: float = float(lbl.length()) * 10.0 * 0.55
	c.draw_string(font, at - dir * 18.0 - Vector2(lw * 0.5, -4.0), lbl,
		HORIZONTAL_ALIGNMENT_LEFT, -1, 10, col)


# --- camera ------------------------------------------------------------------------


func _on_unit() -> bool:
	"""Whether there is a robot to follow at all.

	Both follow views need one; with nobody they fall through to the orbit rather than
	freezing on the last pose of a robot that is gone -- and the controls have to follow
	them there, or the wheel and the right-drag go dead while an orbit is on screen.
	"""
	return follow >= 0 and follow < robots.size()


func _chase_active() -> bool:
	return view_mode == View.CHASE and _on_unit()


func followed_is_drone() -> bool:
	"""Whether the unit the camera is on is a rotor.

	The arrow keys turn on this and nothing else: on a drone they aim the camera
	(`_camera_keys`), on every other chassis they stay the drive keys' second home
	(manual_drive.gd `ARROW_KEYS`). Read off the state frame's chassis column rather than
	guessed from the lane -- a rotor is never a carrier, but it can be any other lane.
	"""
	return _on_unit() and int(robots[follow][6]) == CHASSIS_ROTOR


func _pov_bind(i: int) -> void:
	"""Point the first-person camera straight down unit `i`'s nose when it is a new unit.

	The aim belongs to the drone the operator swung it on, not to the view: carrying it
	to the next unit picked would leave a ground robot's POV staring off at a flank with
	no key on the keyboard able to straighten it, since the arrows are not that unit's.
	"""
	if _pov_of == i:
		return
	_pov_of = i
	pov_look = Vector2.ZERO


func _camera_keys(delta: float) -> void:
	"""The arrow keys aim a drone's camera. Drones only, and only in a unit view.

	A ground unit looks where it is going, which is what the POV already shows and what
	the drive keys already control. A drone does not: it crosses ground at 2x, it is
	blind until it lands, and the whole reason to put one under the keys is to fly it
	somewhere and *look* -- so it is the one chassis where aiming the camera is a second
	control worth having, and the one that can spare the arrows to do it.

	Both views take the keys, and each moves the rig it already has, so a key and a mouse
	drag cannot end up disagreeing about where the camera is.
	"""
	if view_mode == View.ORBIT or not followed_is_drone():
		return
	var yaw := (1.0 if Input.is_physical_key_pressed(KEY_RIGHT) else 0.0) \
		- (1.0 if Input.is_physical_key_pressed(KEY_LEFT) else 0.0)
	var tilt := (1.0 if Input.is_physical_key_pressed(KEY_UP) else 0.0) \
		- (1.0 if Input.is_physical_key_pressed(KEY_DOWN) else 0.0)
	if yaw == 0.0 and tilt == 0.0:
		return
	if view_mode == View.POV:
		pov_look.x = clampf(pov_look.x + yaw * CAM_KEY_YAW * delta,
			-POV_LOOK_YAW, POV_LOOK_YAW)
		pov_look.y = clampf(pov_look.y + tilt * CAM_KEY_PITCH * delta,
			POV_LOOK_DOWN, POV_LOOK_UP)
	else:
		# Unbounded like the drag, which walks all the way round the unit on purpose.
		chase_yaw += yaw * CAM_KEY_YAW * delta
		# Up tilts the *shot* up, which drops the camera below the unit -- so it is the
		# opposite sign to `chase_pitch`, which is how high the eye sits. Same limits as
		# the drag, whose floor dips just under the anchor for the shot against the sky.
		chase_pitch = clampf(chase_pitch - tilt * CAM_KEY_PITCH * delta, -0.10, 1.35)


func _update_camera(delta: float) -> void:
	if not ready_world:
		return
	# Only the orbit shifts its lens. POV is the robot's own eye and chase frames the unit
	# from a fixed rig that render3d.py verifies, so neither may move.
	cam.h_offset = 0.0
	cam.v_offset = 0.0
	if view_mode == View.POV and _on_unit():
		var r: Array = robots[follow]
		var pose := _unit_pose(follow, true)
		var eye := pose.origin + Vector3.UP * 1.1
		_pov_bind(follow)
		# Yaw rides on the unit's own heading, so a drone swung 90 degrees off its nose
		# keeps that shot as it flies rather than being dragged back round. At
		# `pov_look == ZERO` -- which is every unit but a drone under the arrow keys --
		# this is the original eye vector untouched, the one render3d.py signed off on.
		var th := float(r[2]) + pov_look.x
		var look := Vector3(cos(th), -0.12, sin(th))
		if pov_look.y != 0.0:
			# About the camera's own right, so the horizon stays level however far round.
			look = look.rotated(look.cross(Vector3.UP).normalized(), pov_look.y)
		var ahead := eye + look * 14.0
		cam.position = eye
		cam.look_at(ahead, Vector3.UP)
		cam.fov = 74.0
		cam_target = ahead
	elif _chase_active():
		_chase_camera(robots[follow], delta)
	else:
		var centre := Vector3(map_w * 0.5, 0.0, map_h * 0.5)
		if follow >= 0 and follow < robots.size():
			var r2: Array = robots[follow]
			centre = world_to_godot(r2[0], r2[1], 0.0)
		centre += pan_off
		centre.y = _surface_height(centre.x, centre.z)
		if _is_airborne(follow):
			centre.y = surface.flight_height_at_world(centre.x, centre.z)
		var off := Vector3(
			cos(orbit_yaw) * cos(orbit_pitch),
			sin(orbit_pitch),
			sin(orbit_yaw) * cos(orbit_pitch)) * orbit_dist
		cam.position = centre + off
		cam.position.y = maxf(cam.position.y, _surface_height(cam.position.x, cam.position.z) + CHASE_CLEAR)
		cam.look_at(centre, Vector3.UP)
		cam.fov = 50.0
		cam_target = centre
		_centre_on_view(orbit_dist)


func _centre_on_view(dist: float) -> void:
	"""Slide the orbit camera so its look-at point lands in the middle of the 3D view.

	The camera centres its picture on the window, but the view is the hole between the
	HUD panels, and the bottom bar is twice the height of the top one -- so the point
	being orbited sat ~46 px below the visible centre, and the reticle marking it looked
	low. `h_offset`/`v_offset` translate the camera in its own screen plane; scaled by the
	metres a pixel covers at the target's distance, that moves the target exactly the
	pixels needed, and every projection (picking, reticle, contact boxes) follows it.
	"""
	var vr: Rect2 = ui.view_rect()
	var vp: Vector2 = get_viewport().get_visible_rect().size
	if vr.size.y < 2.0 or vp.y < 2.0:
		return
	var per_px := 2.0 * dist * tan(deg_to_rad(cam.fov * 0.5)) / vp.y
	var shift: Vector2 = vr.get_center() - vp * 0.5
	# Moving the camera right slides the scene left, and up slides it down.
	cam.h_offset = -shift.x * per_px
	cam.v_offset = shift.y * per_px


func _chase_camera(r: Array, delta: float) -> void:
	"""Close third-person follow: behind the unit, looking at it.

	Port of `Renderer3D.chase` in swarmmind/viz/render3d.py. The azimuth is the unit's
	heading plus `chase_yaw`, so drag turns the camera *around the unit* and the unit
	turning carries the camera with it; the wheel moves `chase_dist` and right-drag
	slides `chase_pan` in the screen plane, which frames the unit without unpinning the
	camera from it.
	"""
	var pose := _unit_pose(follow, true)
	var anchor := pose.origin + Vector3.UP * CHASE_ANCHOR_Z
	var want_az := float(r[2]) + PI + chase_yaw
	if _chase_of != follow:
		_chase_of = follow
		_chase_anchor = anchor
		_chase_az = want_az
	else:
		# Frame-rate independent exponential catch-up, so the smoothing does not change
		# character between the projector's 60 Hz and a laptop dropping frames.
		_chase_anchor = _chase_anchor.lerp(anchor, 1.0 - exp(-delta * CHASE_POS_RATE))
		_chase_az = lerp_angle(_chase_az, want_az, 1.0 - exp(-delta * CHASE_YAW_RATE))
	var centre := _chase_anchor + chase_pan
	var eye := centre + Vector3(
		cos(_chase_az) * cos(chase_pitch),
		sin(chase_pitch),
		sin(_chase_az) * cos(chase_pitch)) * chase_dist
	eye.y = maxf(eye.y, _surface_height(eye.x, eye.z) + CHASE_CLEAR)
	cam.position = eye
	cam.look_at(centre, Vector3.UP)
	cam.fov = CHASE_FOV
	cam_target = centre


func _reset_chase() -> void:
	chase_yaw = 0.0
	chase_pitch = CHASE_PITCH
	chase_dist = CHASE_DIST
	chase_pan = Vector3.ZERO
	_chase_of = -1
	# The first-person aim goes with it: both are the operator's framing of one unit, and
	# leaving one behind is how a camera ends up pointing somewhere nobody chose.
	pov_look = Vector2.ZERO
	_pov_of = -1


func _eagle_eye() -> void:
	follow = -1
	view_mode = View.ORBIT
	orbit_yaw = 0.9
	orbit_pitch = 0.62
	orbit_dist = maxf(map_w, map_h) * 1.05
	pan_off = Vector3.ZERO
	_reset_chase()


# --- input -------------------------------------------------------------------------


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_F:
				toggle("view")
			KEY_H:
				toggle("thermal")
			KEY_V:
				toggle("victims")
			KEY_C:
				toggle("chassis")
			KEY_G:
				toggle("god")
			KEY_S:
				# Orbit only in practice: in POV and chase S is reverse, and ManualDrive
				# takes it in `_input` before it gets here.
				toggle("sectors")
			KEY_P:
				toggle("fx")
			KEY_E:
				_eagle_eye()
			KEY_ESCAPE:
				follow = -1
				view_mode = View.ORBIT
				_reset_chase()
	elif event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP and event.pressed:
			if _chase_active():
				chase_dist = maxf(CHASE_DIST_MIN, chase_dist * 0.88)
			else:
				orbit_dist = maxf(12.0, orbit_dist * 0.88)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN and event.pressed:
			if _chase_active():
				chase_dist = minf(CHASE_DIST_MAX, chase_dist * 1.12)
			else:
				orbit_dist = minf(3000.0, orbit_dist * 1.12)
		elif event.button_index == MOUSE_BUTTON_LEFT:
			_drag = event.pressed
			if event.pressed:
				_pick(event.position)
		elif event.button_index == MOUSE_BUTTON_RIGHT:
			_pan = event.pressed
	elif event is InputEventMouseMotion:
		if _pan:
			_pan_by(event.relative)
		elif _drag:
			if _chase_active():
				# Rotation is distance-independent, so the chase takes the same rate as
				# the orbit. The pitch floor dips slightly below the anchor -- a camera
				# looking up at a unit against the sky is a shot worth having, and the
				# terrain clearance in `_chase_camera` keeps it out of the ground.
				chase_yaw += event.relative.x * 0.006
				chase_pitch = clampf(chase_pitch + event.relative.y * 0.005, -0.10, 1.35)
			else:
				orbit_yaw += event.relative.x * 0.006
				orbit_pitch = clampf(orbit_pitch + event.relative.y * 0.005, 0.08, 1.45)


func toggle(what: String) -> void:
	"""One entry point for every overlay switch, shared by the keys and the HUD's
	overlay rows, so a click and a key press cannot do different things."""
	match what:
		"thermal":
			thermal_on = not thermal_on
			if thermal_on and view_mode == View.ORBIT:
				_cycle_view()
		"view":
			_cycle_view()
		"victims":
			show_victims = not show_victims
			_update_markers()
		"chassis":
			colour_by_chassis = not colour_by_chassis
			_update_robots()
		"god":
			god_view = not god_view
			if mat:
				mat.set_shader_parameter("god_view", god_view)
				ground.sync_material()
			_update_markers()
		"sectors":
			show_sectors = not show_sectors
			if mat:
				mat.set_shader_parameter("show_sectors", show_sectors)
				ground.sync_material()
		"fx":
			fx_on = not fx_on
	ui.refresh()


func _cycle_view() -> void:
	"""F walks orbit -> POV -> chase -> orbit.

	Two presses is the third-person follow, which is the shot to narrate one unit from.
	Both follow views need a robot: with nothing picked, take the first rather than
	silently doing nothing, which is what a key that appears dead looks like on stage.
	"""
	view_mode = (view_mode + 1) % VIEW_COUNT
	if view_mode != View.ORBIT and follow < 0 and robots.size() > 0:
		follow = 0
	if view_mode != View.CHASE:
		# Leaving drops the anchor, so coming back snaps onto the unit instead of flying
		# in from wherever it was standing last time.
		_chase_of = -1


func _pan_by(delta: Vector2) -> void:
	if cam == null or map_w <= 0.0 or map_h <= 0.0:
		return
	if _chase_active():
		_chase_pan_by(delta)
		return
	# A first-person camera has no orbit centre to move, so there is nothing to pan.
	if view_mode == View.POV and _on_unit():
		return
	# Grab-the-world panning on the ground plane: the point under the cursor stays under the
	# cursor. Screen-to-world scale follows the zoom, so a drag covers the same amount of
	# screen at any orbit distance.
	var vp_h := maxf(get_viewport().get_visible_rect().size.y, 1.0)
	var per_px := 2.0 * orbit_dist * tan(deg_to_rad(cam.fov * 0.5)) / vp_h
	# Camera forward and right, flattened onto the ground plane.
	var fwd := -Vector3(cos(orbit_yaw), 0.0, sin(orbit_yaw))
	var right := fwd.cross(Vector3.UP)
	# Vertical drags cover more ground the shallower the camera sits, and the ground point
	# changes depth as it travels, so the divisor carries both the pitch and that
	# foreshortening. The floor keeps a big flick at a shallow pitch from jumping.
	var fore := sin(orbit_pitch) + delta.y * per_px * cos(orbit_pitch) / orbit_dist
	pan_off -= right * (delta.x * per_px)
	pan_off += fwd * (delta.y * per_px / maxf(fore, 0.25))
	# Keep the map reachable -- an unclamped pan strands the camera over empty ground.
	pan_off.x = clampf(pan_off.x, -map_w * 0.75, map_w * 0.75)
	pan_off.y = 0.0
	pan_off.z = clampf(pan_off.z, -map_h * 0.75, map_h * 0.75)


func _chase_pan_by(delta: Vector2) -> void:
	# Framing, not travel: this slides the unit around inside the frame and the camera
	# keeps following it. The offset is in the camera's screen plane, not on the ground
	# plane the orbit pans on -- from 4 m a ground-plane drag upward sends the look point
	# over the horizon, and raising the shot is the thing an operator actually wants here.
	var vp_h := maxf(get_viewport().get_visible_rect().size.y, 1.0)
	var per_px := 2.0 * chase_dist * tan(deg_to_rad(CHASE_FOV * 0.5)) / vp_h
	var fwd := -Vector3(cos(_chase_az), 0.0, sin(_chase_az))
	var right := fwd.cross(Vector3.UP)
	chase_pan -= right * (delta.x * per_px)
	chase_pan.y += delta.y * per_px
	# Clamped against the zoom, so the unit can be put off-centre but never dragged out
	# of its own follow camera.
	var lim := clampf(chase_dist * 1.2, 2.0, 10.0)
	var flat := Vector2(chase_pan.x, chase_pan.z).limit_length(lim)
	chase_pan.x = flat.x
	chase_pan.z = flat.y
	chase_pan.y = clampf(chase_pan.y, -1.0, lim)


func _pick(screen_pos: Vector2) -> void:
	# Nearest robot to the click ray, in screen space -- simpler and more forgiving than
	# physics picking, and there are no collision bodies to raycast against.
	var best := -1
	var best_d := 40.0
	for i in range(robots.size()):
		var p := _unit_pose(i).origin + Vector3.UP * 0.6
		if cam.is_position_behind(p):
			continue
		var d := cam.unproject_position(p).distance_to(screen_pos)
		if d < best_d:
			best_d = d
			best = i
	if best >= 0:
		follow = best
