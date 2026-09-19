class_name SwarmHud
extends CanvasLayer
##
## The operator HUD: a mission bar across the top, an instrument rail either side, the
## event stream and search coverage along the bottom, and a frame over the 3D view in
## the hole in the middle.
##
## Layout and type are from the "SAR Swarm HUD" design (claude.ai/design), sized in its
## own unit -- one percent of the logical width -- so the proportions are the design's
## whatever the window is.
##
## **Every readout is read off the wire.** The mock filled its panels with plausible
## instrument copy -- link encryption, a pheromone constant, a FIFO task queue with ETAs,
## a lat/long cursor -- and the simulator has a source for none of it. Where a field had
## no source, the panel shows the nearest real quantity instead, because a judge who
## catches one invented number discounts every other one on the screen.
##
## Nothing here reads ground truth. The HUD counts what the swarm reports: robot rows,
## contacts, fog, events. `victims_total` comes from the hello, not from a truth frame.

const FONT_MONO := "res://assets/fonts/IBMPlexMono-Regular.ttf"
const FONT_MONO_SEMI := "res://assets/fonts/IBMPlexMono-SemiBold.ttf"
const FONT_DISPLAY := "res://assets/fonts/Oxanium-Variable.ttf"

enum { MONO, MONO_SEMI, DISP, DISP_BOLD }

#: The mock was drawn in a 1920 px browser tab; this project's logical width is 1600,
#: where its smallest labels would land at 7 px. On a projector that is not type.
const FONT_BOOST := 1.2
const FONT_MIN_PX := 9

#: Design palette. Neutral, faintly green-grey, one accent. Data colours -- contact
#: rings, lanes, chassis -- are *not* restated here: every swatch on the HUD is read from
#: main.gd's own palette, so a legend cannot drift from the marker it describes.
const C_BG := Color(0.024, 0.031, 0.031)
const C_PANEL := Color(0.039, 0.051, 0.055)
const C_LINE := Color(0.588, 0.659, 0.651, 0.16)
const C_LINE_SOFT := Color(0.588, 0.659, 0.651, 0.10)
const C_TRACK := Color(0.588, 0.659, 0.651, 0.13)
const C_ACCENT := Color(0.435, 0.941, 0.847)
const C_TITLE := Color(0.859, 0.890, 0.882)
const C_BRIGHT := Color(0.875, 0.906, 0.898)
const C_HEAD := Color(0.804, 0.839, 0.831)
const C_VALUE := Color(0.812, 0.847, 0.839)
const C_ROW := Color(0.682, 0.722, 0.714)
const C_BIND := Color(0.647, 0.686, 0.678)
const C_MSG := Color(0.608, 0.647, 0.639)
const C_VIEW := Color(0.486, 0.529, 0.522)
const C_UNIT := Color(0.408, 0.447, 0.435)
const C_DIM := Color(0.427, 0.471, 0.467)
const C_DIMMER := Color(0.369, 0.408, 0.404)
const C_FAINT := Color(0.310, 0.349, 0.345)
#: Amber for "not live" -- the link down, god-view on. Red is taken by idle robots and
#: the hazard, and neither of those is what a dropped socket means.
const C_WARN := Color(1.0, 0.75, 0.24)

#: Panel refresh rate. State frames land at 10 Hz, so faster buys nothing but text
#: reshaping; the viewport frame is still redrawn every frame because the camera moves.
const REFRESH_HZ := 10.0

const METRICS := [
	["RESCUED", ""],
	["CASUALTIES FOUND", ""],
	["SWARM ACTIVE", "UNITS"],
	["AREA EXPLORED", ""],
]

#: Label, overlay id. The id is what `SwarmDashboard.toggle()` takes, so a click on the
#: row and the key do exactly the same thing.
const OVERLAYS := [
	["H", "thermal vision", "thermal"],
	["V", "casualties", "victims"],
	["C", "colour by chassis", "chassis"],
	["S", "hivemind sectors", "sectors"],
	["P", "particles", "fx"],
	["G", "god-view", "god"],
	["F", "view", "view"],
]

const BINDINGS := [
	["F", "view: POV / chase / orbit"],
	["E", "eagle-eye, whole map"],
	["click", "follow a unit"],
	["Esc", "release camera"],
	["WASD", "drive unit (POV / chase)"],
	["wheel", "zoom"],
	["drag", "orbit / swing chase"],
	["R-drag", "pan / reframe"],
]

#: What replaced the mock's "solver diagnostics": what the swarm is doing, one row per
#: activity code, in the order a casualty moves through them -- with idle last, because
#: it is the only row that means something is wrong.
#:
#: This is the question the map cannot answer on its own. A relay on its post and a
#: robot wedged against a rock are the same still box; of ~80 stationary robots on seed
#: 42, ~32 were working and ~42 were stuck. Every name must be a key of
#: `SwarmDashboard.ACTIVITY`.
const ACTIVITY_ROWS := [
	["explore", "exploring"],
	["drift", "seeking dark ground"],
	["investigate", "investigating contact"],
	["dig", "clearing debris"],
	["extract", "en route to casualty"],
	["carry", "carrying casualty"],
	["relay_post", "holding relay post"],
	["relay_move", "moving relay net"],
	["recharge", "recharging"],
	["retreat", "retreating"],
	["recover", "rejoining comms"],
	["idle", "idle, no useful work"],
]

const COMMS := ["ALIVE", "IN COMMS", "OUT OF CONTACT", "RELAYS ON POST", "RELAYS MOVING", "LOST"]
const LEDGER := ["DIGS ACTIVE", "ON CARRIERS", "AWAITING CARRIER", "DISMISSED"]

#: Short tags for the event stream's second column. A kind with no tag still prints, as
#: its own name cut to fit -- a new event kind must never vanish from the feed.
const EVENT_TAGS := {
	"victim_rescued": "RESCUE",
	"victim_found": "FOUND",
	"victim_cleared": "DUG OUT",
	"robot_destroyed": "LOST",
	"robot_out_of_comms": "NO LINK",
	"robot_reconnected": "RELINK",
	"robot_recharged": "CHARGE",
	"task_orphaned": "ORPHAN",
	"report_dismissed": "DISMISS",
	"hazard_ignited": "HAZARD",
	"directive_issued": "ISSUED",
	"directive_rejected": "FILTER",
	"sector_abandoned": "SECTOR",
	"hivemind_offline": "OFFLINE",
}
#: The Tier 3 kinds, which also go to the hivemind panel. Its reasoning is the line a
#: judge actually reads, and in the main stream it scrolls away under ten dismissals a
#: second.
const T3_KINDS := ["directive_issued", "directive_rejected", "sector_abandoned", "hivemind_offline"]
#: Two lines each. Directive reasoning front-loads its counts and ends on what it is
#: actually doing about them, so one truncated line cut off exactly the part worth reading.
const T3_ROWS := 4

const LOG_MAX_LINES := 240
#: Sector coverage samples one cell in STRIDE x STRIDE. The fog frame is already a full
#: pass over the grid in GDScript at 2 Hz; a second full pass for a panel readout would
#: double it, and a 1-in-16 sample of a 60 m sector is ~200 cells.
const SECTOR_STRIDE := 4

var host: SwarmDashboard

var log_box: RichTextLabel

var _u := 16.0
var _t := 0.0
var _refresh_acc := 0.0
var _fonts := {}

var _view_hole: Control
var _view_draw: HudCanvas

var _title_sub: Label
var _clock: Label
var _clock_dot: Panel
var _metric_val: Array[Label] = []
var _metric_sub: Array[Label] = []
var _metric_fill: Array[ColorRect] = []
var _link_dot: Panel
var _link_state: Label
var _link_detail: Label

var _tax_side: Label
var _tax_count: Array[Label] = []
var _tax_pct: Array[Label] = []
var _tax_seg: Array[ColorRect] = []
var _sec_side: Label
var _sec_grid: GridContainer
var _sec_cells: Array[ColorRect] = []
var _sec_foot_l: Label
var _sec_foot_r: Label
var _sec_caption: Label
var _comms_val: Array[Label] = []
var _chassis_side: Label
var _chassis_n: Array[Label] = []
var _chassis_fill: Array[ColorRect] = []

var _ov_side: Label
var _ov_state: Array[Label] = []
var _act_side: Label
var _act_mark: Array[Label] = []
var _act_name: Array[Label] = []
var _act_n: Array[Label] = []
var _act_pct: Array[Label] = []
var _act_fill: Array[ColorRect] = []

var _log_side: Label
var _cov_head: Label
var _cov_fill: ColorRect
var _cov_rate: Label
var _cov_seeded: Label
var _ledger_val: Array[Label] = []
var _t3_side: Label
var _t3_rows: Array = []

# --- tallies, rebuilt from each state frame ------------------------------------------
var _act := PackedInt32Array()
var _chassis_alive := PackedInt32Array()
var _chassis_all := PackedInt32Array()
var _contacts := PackedInt32Array()

# --- sector coverage, from fog frames -------------------------------------------------
var _sec_sample := PackedInt32Array()
var _sec_den := PackedInt32Array()
var _sec_cov := PackedFloat32Array()

# --- event counters, since the dashboard connected -------------------------------------
var _ev_wall: Array[float] = []
var _n_dismissed := 0
var _n_issued := 0
var _n_rejected := 0
var _n_offline := 0
var _t3: Array = []

# --- rates -----------------------------------------------------------------------------
var _explore_hist: Array[Vector2] = []
var _state_wall: Array[float] = []


# A Control whose only job is to call back into this script, so the drawing lives next
# to the state it draws. The same pattern as main.gd's DetectorHUD.
class HudCanvas extends Control:
	var paint: Callable

	func _draw() -> void:
		if paint.is_valid():
			paint.call(self)


# --- construction ----------------------------------------------------------------------


func build(dashboard: SwarmDashboard) -> void:
	host = dashboard
	_u = maxf(host.get_viewport().get_visible_rect().size.x, 640.0) / 100.0
	_act.resize(SwarmDashboard.ACTIVITY.size())
	_chassis_alive.resize(SwarmDashboard.CHASSIS_NAMES.size())
	_chassis_all.resize(SwarmDashboard.CHASSIS_NAMES.size())
	_contacts.resize(SwarmDashboard.CONTACT_NAMES.size())

	# The viewport frame draws under the panels, over the world. Full-rect so that it
	# shares the camera's projection space; it confines itself to the hole.
	_view_draw = HudCanvas.new()
	_view_draw.paint = _draw_view
	_view_draw.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_view_draw)
	_view_draw.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)

	var root := _vbox(self, 0.0)
	root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_build_top(root)
	var mid := _hbox(root, 0.0)
	mid.size_flags_vertical = Control.SIZE_EXPAND_FILL
	_build_left(mid)
	# The hole the 3D view shows through. IGNORE, and every container above it IGNORE,
	# so clicks, drags and the wheel fall through to `_unhandled_input` as before; the
	# rails and bars are STOP, so a click on a panel never picks a robot behind it.
	_view_hole = Control.new()
	_view_hole.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_view_hole.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_view_hole.size_flags_vertical = Control.SIZE_EXPAND_FILL
	mid.add_child(_view_hole)
	_build_right(mid)
	_build_bottom(root)
	_refresh()


func _build_top(root: Control) -> void:
	var bar := _panel(root, _style(C_PANEL, _px(0.9), _px(0.6), C_LINE, 8))
	bar.custom_minimum_size.y = _px(5.6)
	var row := _hbox(bar, 0.0)

	var title := _vbox(row, _px(0.18))
	title.alignment = BoxContainer.ALIGNMENT_CENTER
	title.custom_minimum_size.x = _px(11.5)
	_text(title, "SWARMMIND // SAR", 0.95, C_TITLE, DISP_BOLD, 0.16)
	_title_sub = _text(title, "NO SIMULATOR", 0.5, C_DIM, MONO, 0.2)
	_vsep(row, _px(0.9), C_LINE_SOFT)

	var clock := _hbox(row, _px(0.4))
	var cv := _vbox(clock, 0.0)
	cv.alignment = BoxContainer.ALIGNMENT_CENTER
	_text(cv, "MISSION CLOCK", 0.48, C_DIM, MONO, 0.2)
	_clock = _text(cv, "T+0.0s", 1.7, C_ACCENT, DISP_BOLD)
	# Wide enough for T+999.9s, so the whole bar does not reflow as digits arrive.
	_clock.custom_minimum_size.x = _px(7.2)
	_clock_dot = _dot(clock, 0.38, C_ACCENT)
	_vsep(row, _px(0.9), C_LINE_SOFT)

	for m in METRICS:
		var pad := _pad(row, _px(0.8), 0.0)
		pad.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		var col := _vbox(pad, _px(0.24))
		col.alignment = BoxContainer.ALIGNMENT_CENTER
		var head := _hbox(col, _px(0.4))
		var label := _text(head, m[0], 0.48, C_DIM, MONO, 0.2)
		label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		label.clip_text = true
		_metric_sub.append(_text(head, "", 0.46, C_FAINT, MONO, 0.1))
		var vrow := _hbox(col, _px(0.3))
		var val := _text(vrow, "--", 1.3, C_BRIGHT, DISP)
		val.size_flags_vertical = Control.SIZE_SHRINK_END
		_metric_val.append(val)
		if m[1] != "":
			var unit := _text(vrow, m[1], 0.52, C_UNIT)
			unit.size_flags_vertical = Control.SIZE_SHRINK_END
		_metric_fill.append(_bar(col, 0.18, Color(C_ACCENT, 0.7)))
		_vline(row, C_LINE_SOFT)

	var lpad := _pad(row, _px(0.8), 0.0)
	var link := _vbox(lpad, _px(0.16))
	link.alignment = BoxContainer.ALIGNMENT_CENTER
	link.custom_minimum_size.x = _px(8.6)
	_text(link, "LINK STATE", 0.48, C_DIM, MONO, 0.2)
	var ls := _hbox(link, _px(0.35))
	_link_dot = _dot(ls, 0.36, C_WARN)
	_link_state = _text(ls, "WAITING", 0.86, C_WARN, DISP, 0.08)
	_link_detail = _text(link, "", 0.48, C_DIMMER, MONO, 0.12)


func _build_left(mid: Control) -> void:
	var rail := _rail(mid, 4)

	var tax := _section(rail, "CONTACT TAXONOMY")
	_tax_side = tax[1]
	for i in range(SwarmDashboard.CONTACT_NAMES.size()):
		var c: Color = SwarmDashboard.CONTACT_COLORS[i]
		# Opaque in the swatch: the unverified ring's own alpha is what makes it faint on
		# the map, and reproducing it here just makes the word hard to read.
		var solid := Color(c.r, c.g, c.b)
		var r := _hbox(tax[0], _px(0.45))
		_dot(r, 0.45, solid)
		var n := _text(r, str(SwarmDashboard.CONTACT_NAMES[i]).to_upper(), 0.56, C_ROW, MONO, 0.12)
		n.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		_tax_count.append(_text(r, "00", 0.56, C_BRIGHT))
		var pct := _text(r, "0%", 0.46, C_DIMMER)
		pct.custom_minimum_size.x = _px(2.6)
		pct.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
		_tax_pct.append(pct)
	var stack := _hbox(tax[0], 1.0)
	stack.custom_minimum_size.y = maxf(3.0, _px(0.3))
	for i in range(SwarmDashboard.CONTACT_NAMES.size()):
		var c2: Color = SwarmDashboard.CONTACT_COLORS[i]
		var seg := ColorRect.new()
		seg.color = Color(c2.r, c2.g, c2.b, 0.7)
		seg.mouse_filter = Control.MOUSE_FILTER_IGNORE
		seg.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		seg.visible = false
		stack.add_child(seg)
		_tax_seg.append(seg)

	var sec := _section(rail, "HIVEMIND SECTORS")
	_sec_side = sec[1]
	_sec_grid = GridContainer.new()
	_sec_grid.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_sec_grid.add_theme_constant_override("h_separation", 1)
	_sec_grid.add_theme_constant_override("v_separation", 1)
	sec[0].add_child(_sec_grid)
	var foot := _hbox(sec[0], _px(0.3))
	_sec_foot_l = _text(foot, "", 0.44, C_FAINT, MONO, 0.14)
	_sec_caption = _text(foot, "COVERAGE · TIER 3", 0.44, C_FAINT, MONO, 0.14)
	var cap := _sec_caption
	cap.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	cap.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_sec_foot_r = _text(foot, "", 0.44, C_FAINT, MONO, 0.14)
	# A wash nobody can decode is decoration. Tints are read from SECTOR_TINTS, the same
	# table that washes the terrain.
	var key := _hbox(sec[0], _px(0.35))
	for pair in [[0, "HIGH"], [2, "LOW"], [3, "ABANDONED"]]:
		_dot(key, 0.4, _sector_ink(int(pair[0])))
		_text(key, pair[1], 0.44, C_DIM, MONO, 0.12)

	var comms := _section(rail, "COMMS MESH")
	var grid := GridContainer.new()
	grid.columns = 2
	grid.mouse_filter = Control.MOUSE_FILTER_IGNORE
	grid.add_theme_constant_override("h_separation", roundi(_px(0.6)))
	grid.add_theme_constant_override("v_separation", roundi(_px(0.3)))
	comms[0].add_child(grid)
	for k in COMMS:
		var cell := _vbox(grid, 0.0)
		cell.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		_text(cell, k, 0.44, C_DIM, MONO, 0.16)
		_comms_val.append(_text(cell, "--", 0.78, C_VALUE, DISP))

	var roster := _section(rail, "CHASSIS ROSTER", true, true)
	_chassis_side = roster[1]
	for i in range(SwarmDashboard.CHASSIS_NAMES.size()):
		var r2 := _hbox(roster[0], _px(0.45))
		var nm := _text(r2, str(SwarmDashboard.CHASSIS_NAMES[i]).to_upper(), 0.54, C_ROW, MONO, 0.1)
		nm.custom_minimum_size.x = _px(3.6)
		var holder := _vbox(r2, 0.0)
		holder.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		holder.alignment = BoxContainer.ALIGNMENT_CENTER
		# The robots' own `C` colours, so this panel is also the legend for that view.
		_chassis_fill.append(_bar(holder, 0.26, Color(SwarmDashboard.CHASSIS_COLORS[i] as Color, 0.8)))
		var cnt := _text(r2, "--", 0.52, C_VALUE)
		cnt.custom_minimum_size.x = _px(3.4)
		cnt.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
		_chassis_n.append(cnt)


func _build_right(mid: Control) -> void:
	var rail := _rail(mid, 1)

	var ov := _section(rail, "OVERLAY BUS")
	_ov_side = ov[1]
	var idle := _style(Color(0.588, 0.659, 0.651, 0.03), _px(0.35), _px(0.2),
		Color(0.588, 0.659, 0.651, 0.14), 15)
	var hot := _style(Color(C_ACCENT, 0.10), _px(0.35), _px(0.2), Color(C_ACCENT, 0.45), 15)
	for o in OVERLAYS:
		var row := PanelContainer.new()
		row.add_theme_stylebox_override("panel", idle)
		row.mouse_filter = Control.MOUSE_FILTER_STOP
		row.mouse_default_cursor_shape = Control.CURSOR_POINTING_HAND
		row.focus_mode = Control.FOCUS_NONE
		row.gui_input.connect(_on_overlay_input.bind(str(o[2])))
		row.mouse_entered.connect(row.add_theme_stylebox_override.bind("panel", hot))
		row.mouse_exited.connect(row.add_theme_stylebox_override.bind("panel", idle))
		ov[0].add_child(row)
		var inner := _hbox(row, _px(0.45))
		var keybox := _panel(inner, _style(Color(0, 0, 0, 0), 0.0, 0.0,
			Color(0.588, 0.659, 0.651, 0.3), 15))
		keybox.custom_minimum_size.x = _px(1.4)
		keybox.mouse_filter = Control.MOUSE_FILTER_IGNORE
		var kl := _text(keybox, o[0], 0.6, C_HEAD, DISP_BOLD)
		kl.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
		var nl := _text(inner, str(o[1]).to_upper(), 0.54, C_ROW, MONO, 0.1)
		nl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		_ov_state.append(_text(inner, "OFF", 0.5, C_FAINT, MONO, 0.14))

	var binds := _section(rail, "CAMERA RIG BINDINGS")
	for b in BINDINGS:
		var r := _hbox(binds[0], _px(0.5))
		var k := _text(r, b[0], 0.5, C_ACCENT, MONO, 0.1)
		k.custom_minimum_size.x = _px(3.6)
		k.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
		_text(r, b[1], 0.52, C_BIND, MONO, 0.08)

	var act := _section(rail, "SWARM ACTIVITY", true, true)
	_act_side = act[1]
	for a in ACTIVITY_ROWS:
		var box := _vbox(act[0], _px(0.06))
		var r3 := _hbox(box, _px(0.3))
		# Marks the followed unit's row, so clicking a still robot says which of these
		# it is without reading the footer.
		var mark := _text(r3, "▸", 0.52, C_ACCENT)
		mark.custom_minimum_size.x = _px(0.6)
		mark.modulate.a = 0.0
		_act_mark.append(mark)
		var nm := _text(r3, str(a[1]).to_upper(), 0.5, C_ROW, MONO, 0.06)
		nm.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		nm.clip_text = true
		_act_name.append(nm)
		var n := _text(r3, "0", 0.56, C_VALUE)
		n.custom_minimum_size.x = _px(2.2)
		n.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
		_act_n.append(n)
		var pct := _text(r3, "0%", 0.46, C_DIMMER)
		pct.custom_minimum_size.x = _px(2.2)
		pct.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
		_act_pct.append(pct)
		_act_fill.append(_bar(box, 0.14, _activity_ink(str(a[0]))))


func _build_bottom(root: Control) -> void:
	var bar := _panel(root, _style(C_PANEL, 0.0, 0.0, C_LINE, 2))
	bar.custom_minimum_size.y = _px(11.4)
	var row := _hbox(bar, 0.0)

	# Event stream.
	var tp := _pad(row, _px(0.8), _px(0.5))
	tp.custom_minimum_size.x = _px(31.0)
	var tv := _vbox(tp, _px(0.25))
	var th := _hbox(tv, _px(0.4))
	var tt := _text(th, "EVENT STREAM", 0.58, C_HEAD, DISP, 0.24)
	tt.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_log_side = _text(th, "", 0.46, C_DIMMER, MONO, 0.14)
	_hline(tv, C_LINE_SOFT)
	log_box = RichTextLabel.new()
	log_box.bbcode_enabled = true
	log_box.scroll_following = true
	log_box.autowrap_mode = TextServer.AUTOWRAP_OFF
	log_box.size_flags_vertical = Control.SIZE_EXPAND_FILL
	log_box.mouse_filter = Control.MOUSE_FILTER_PASS
	log_box.add_theme_font_override("normal_font", _font(MONO, 0))
	log_box.add_theme_font_size_override("normal_font_size", _fs(0.53))
	log_box.add_theme_color_override("default_color", C_MSG)
	log_box.add_theme_constant_override("line_separation", 0)
	tv.add_child(log_box)
	# Still scrolls on the wheel; the bar itself is not in the design and the stream
	# follows its newest line anyway.
	log_box.get_v_scroll_bar().modulate = Color(1, 1, 1, 0)
	_vline(row, C_LINE_SOFT)

	# Search coverage.
	var cp := _pad(row, _px(0.9), _px(0.6))
	cp.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	var cv := _vbox(cp, _px(0.3))
	cv.alignment = BoxContainer.ALIGNMENT_CENTER
	var ch := _hbox(cv, _px(0.4))
	var ct := _text(ch, "SEARCH COVERAGE", 0.48, C_DIM, MONO, 0.18)
	ct.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_cov_head = _text(ch, "", 0.48, C_DIM, MONO, 0.18)
	_cov_fill = _bar(cv, 0.68, Color(C_ACCENT, 0.65))
	var ticks := HudCanvas.new()
	ticks.mouse_filter = Control.MOUSE_FILTER_IGNORE
	ticks.paint = _draw_ticks
	_cov_fill.get_parent().add_child(ticks)
	ticks.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	var axis := _hbox(cv, _px(0.4))
	_text(axis, "0%", 0.44, C_FAINT, MONO, 0.14)
	_spring(axis)
	_cov_rate = _text(axis, "", 0.44, C_FAINT, MONO, 0.14)
	_spring(axis)
	_cov_seeded = _text(axis, "", 0.44, C_FAINT, MONO, 0.14)
	_spring(axis)
	_text(axis, "100%", 0.44, C_FAINT, MONO, 0.14)
	_hline(cv, C_LINE_SOFT)
	var ledger := GridContainer.new()
	ledger.columns = LEDGER.size()
	ledger.mouse_filter = Control.MOUSE_FILTER_IGNORE
	ledger.add_theme_constant_override("h_separation", roundi(_px(0.6)))
	cv.add_child(ledger)
	for k in LEDGER:
		var cell := _vbox(ledger, _px(0.1))
		cell.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		_text(cell, k, 0.44, C_DIM, MONO, 0.16)
		_ledger_val.append(_text(cell, "0", 0.76, C_VALUE, DISP))
	_vline(row, C_LINE_SOFT)

	# Hivemind. Stands where the mock had a task queue: the bridge carries no queue, and
	# Tier 3's own output is the one stream that deserves a panel of its own.
	var hp := _pad(row, _px(0.8), _px(0.5))
	hp.custom_minimum_size.x = _px(20.0)
	var hv := _vbox(hp, _px(0.25))
	var hh := _hbox(hv, _px(0.4))
	var ht := _text(hh, "HIVEMIND", 0.58, C_HEAD, DISP, 0.24)
	ht.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_t3_side = _text(hh, "", 0.46, C_DIMMER, MONO, 0.1)
	_hline(hv, C_LINE_SOFT)
	for _i in range(T3_ROWS):
		var r := _hbox(hv, _px(0.4))
		var tm := _text(r, "", 0.52, C_DIM)
		tm.custom_minimum_size.x = _px(2.6)
		tm.vertical_alignment = VERTICAL_ALIGNMENT_TOP
		tm.size_flags_vertical = Control.SIZE_SHRINK_BEGIN
		var tag := _text(r, "", 0.52, C_ACCENT)
		tag.custom_minimum_size.x = _px(2.9)
		tag.vertical_alignment = VERTICAL_ALIGNMENT_TOP
		tag.size_flags_vertical = Control.SIZE_SHRINK_BEGIN
		var msg := _text(r, "", 0.52, C_MSG)
		msg.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		msg.vertical_alignment = VERTICAL_ALIGNMENT_TOP
		msg.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		msg.max_lines_visible = 2
		msg.text_overrun_behavior = TextServer.OVERRUN_TRIM_ELLIPSIS
		msg.custom_minimum_size.x = _px(10.0)
		_t3_rows.append([tm, tag, msg])


# --- inputs from main.gd ---------------------------------------------------------------


func on_hello() -> void:
	"""A new connection: forget every counter from the last one, re-lay the sector grid."""
	_ev_wall.clear()
	_state_wall.clear()
	_explore_hist.clear()
	_t3.clear()
	_n_dismissed = 0
	_n_issued = 0
	_n_rejected = 0
	_n_offline = 0
	_sec_sample.clear()
	_sec_cov.resize(host.sector_rects.size())
	_sec_cov.fill(0.0)
	_rebuild_sector_grid()
	_refresh()


func on_world() -> void:
	"""The map is in: precompute which sector each coverage sample falls in.

	Walls are left out of the denominator, so a sector that is half building can still
	read as fully searched.
	"""
	var n := host.sector_rects.size()
	_sec_sample.clear()
	_sec_den.resize(n)
	_sec_den.fill(0)
	if n == 0 or host.gw == 0 or _sec_grid.columns <= 0:
		return
	var r0: Array = host.sector_rects[0]
	var sw := maxf(float(r0[2]) - float(r0[0]), 0.001)
	var sh := maxf(float(r0[3]) - float(r0[1]), 0.001)
	var cols := _sec_grid.columns
	for iy in range(0, host.gh, SECTOR_STRIDE):
		for ix in range(0, host.gw, SECTOR_STRIDE):
			var k := -1
			if host.occ[iy * host.gw + ix] != SwarmDashboard.OCC_WALL:
				# `sector_ids` is row-major (A1..A8, B1..), so the grid index is exact.
				var col := mini(int((ix + 0.5) * host.cell / sw), cols - 1)
				var row := int((iy + 0.5) * host.cell / sh)
				k = row * cols + col
				if k >= n:
					k = -1
			_sec_sample.append(k)
			if k >= 0:
				_sec_den[k] += 1


func on_fog(explored: PackedByteArray) -> void:
	var n := _sec_den.size()
	if _sec_sample.is_empty() or n == 0 or explored.size() < host.gw * host.gh:
		return
	var num := PackedInt32Array()
	num.resize(n)
	num.fill(0)
	var j := 0
	for iy in range(0, host.gh, SECTOR_STRIDE):
		for ix in range(0, host.gw, SECTOR_STRIDE):
			var k := _sec_sample[j]
			j += 1
			if k >= 0 and explored[iy * host.gw + ix] != 0:
				num[k] += 1
	_sec_cov.resize(n)
	for k in range(n):
		_sec_cov[k] = float(num[k]) / float(maxi(_sec_den[k], 1))


func on_state() -> void:
	"""Tally the robot rows and contacts. One pass, once per state frame, never per draw."""
	_state_wall.append(_t)
	_act.fill(0)
	_chassis_alive.fill(0)
	_chassis_all.fill(0)
	var nch := _chassis_all.size()
	for rr in host.robots:
		var r: Array = rr
		var ch := clampi(int(r[6]), 0, nch - 1)
		_chassis_all[ch] += 1
		# status <= 1 is alive -- active or out of comms -- which is the bridge's own
		# `alive` and what `hud.active` counts, so the rails and the top bar agree.
		if int(r[4]) > 1:
			continue
		_chassis_alive[ch] += 1
		if r.size() > 7:
			var a := int(r[7])
			if a >= 0 and a < _act.size():
				_act[a] += 1
	_contacts.fill(0)
	var nc := _contacts.size()
	for rp in host.reports:
		_contacts[clampi(int((rp as Array)[2]), 0, nc - 1)] += 1

	var t := host.sim_time
	var ex := float(host.hud.get("explored", 0.0))
	if not _explore_hist.is_empty() and t < _explore_hist[-1].x:
		_explore_hist.clear()             # a restarted mission, not time running backwards
	if _explore_hist.is_empty() or t - _explore_hist[-1].x >= 1.0:
		_explore_hist.append(Vector2(t, ex))
		while _explore_hist.size() > 2 and t - _explore_hist[0].x > 30.0:
			_explore_hist.pop_front()


func on_event(kind: String, t: float, text: String, colour: String) -> void:
	_ev_wall.append(_t)
	var tag: String = EVENT_TAGS.get(kind, kind.left(7).to_upper())
	var ink := Color.html(colour)
	# The tag carries the event's colour at full strength; the message is pulled toward
	# the panel's text colour so a burst of dismissals reads as texture, not as alarm.
	var body := ink.lerp(C_MSG, 0.45)
	_append("[color=#%s]%6.1fs[/color]  [color=#%s]%-7s[/color]  [color=#%s]%s[/color]" % [
		C_DIMMER.to_html(false), t, ink.to_html(false), tag, body.to_html(false),
		_escape(text)])
	match kind:
		"report_dismissed":
			_n_dismissed += 1
		"directive_issued":
			_n_issued += 1
		"directive_rejected":
			_n_rejected += 1
		"hivemind_offline":
			_n_offline += 1
	if kind in T3_KINDS:
		_t3.push_front([t, tag, text, ink])
		if _t3.size() > T3_ROWS:
			_t3.pop_back()


func log_system(line: String) -> void:
	"""Dashboard-side lines -- connection, world build -- in the stream's column layout."""
	_append("[color=#%s]%7s  %-7s[/color]  %s" % [C_FAINT.to_html(false), "", "SYS", line])


func view_rect() -> Rect2:
	"""Where the 3D view actually shows, in viewport coordinates."""
	return _view_hole.get_global_rect() if _view_hole != null else Rect2()


func strip_px() -> float:
	return _px(2.6)


func refresh() -> void:
	"""Now, rather than at the next 10 Hz tick -- a key press should not lag its row."""
	_refresh()


func mono_font() -> Font:
	return _font(MONO, 0)


func tick(delta: float) -> void:
	_t += delta
	var pulse := 0.5 - 0.5 * cos(TAU * _t / 1.2)
	_clock_dot.modulate.a = 0.2 + 0.7 * pulse
	_view_draw.queue_redraw()
	_refresh_acc += delta
	if _refresh_acc >= 1.0 / REFRESH_HZ:
		_refresh_acc = 0.0
		_refresh()


func _on_overlay_input(event: InputEvent, what: String) -> void:
	if event is InputEventMouseButton and event.pressed \
			and event.button_index == MOUSE_BUTTON_LEFT:
		host.toggle(what)
		_refresh()
		get_viewport().set_input_as_handled()


# --- refresh ---------------------------------------------------------------------------


func _refresh() -> void:
	if host == null:
		return
	var h: Dictionary = host.hud
	var live := host.ready_world and not h.is_empty()
	var total := int(h.get("total", host.victims_total))
	var rescued := int(h.get("rescued", 0))
	var found := int(h.get("found", 0))
	var active := int(h.get("active", 0))
	var incomms := int(h.get("incomms", 0))
	var lost := int(h.get("lost", 0))
	var explored := float(h.get("explored", 0.0))
	var fleet := host.robots.size()

	# Top bar.
	if host.scenario_name != "":
		_title_sub.text = "%s · %.0f×%.0f M · %d UNITS" % [
			host.scenario_name.to_upper(), host.map_w, host.map_h, host.robot_ids.size()]
	_clock.text = "T+%.1fs" % host.sim_time
	_metric_val[0].text = "%d/%d" % [rescued, total] if live else "--"
	_metric_sub[0].text = "%.0f%%" % (100.0 * rescued / maxi(total, 1)) if live else ""
	_fill(_metric_fill[0], float(rescued) / maxi(total, 1))
	_metric_val[1].text = str(found) if live else "--"
	_metric_sub[1].text = "OF %d" % total if live else ""
	_fill(_metric_fill[1], float(found) / maxi(total, 1))
	_metric_val[2].text = str(active) if live else "--"
	_metric_sub[2].text = "LOST %d" % lost if live else ""
	_fill(_metric_fill[2], float(active) / maxi(fleet, 1))
	_metric_val[3].text = "%.0f%%" % (explored * 100.0) if live else "--"
	var rate := _explore_rate()
	_metric_sub[3].text = "%+.1f%%/MIN" % rate if live and _explore_hist.size() > 1 else ""
	_fill(_metric_fill[3], explored)

	var connected := host.state_name == "connected"
	var ink := C_ACCENT if connected else C_WARN
	_link_state.text = "CONNECTED" if connected else "WAITING"
	_link_state.add_theme_color_override("font_color", ink)
	(_link_dot.get_theme_stylebox("panel") as StyleBoxFlat).bg_color = ink
	while not _state_wall.is_empty() and _t - _state_wall[0] > 2.0:
		_state_wall.pop_front()
	var addr := host.url.trim_prefix("ws://")
	if connected:
		_link_detail.text = "%s · STATE %.1f HZ" % [addr, _state_wall.size() / 2.0]
	elif host._blob_progress != "":
		_link_detail.text = host._blob_progress.to_upper()
	else:
		_link_detail.text = addr

	# Contact taxonomy.
	var nrep := host.reports.size()
	_tax_side.text = "N=%d" % nrep
	for i in range(_contacts.size()):
		var c := _contacts[i]
		_tax_count[i].text = "%02d" % c
		_tax_pct[i].text = "%.0f%%" % (100.0 * c / maxi(nrep, 1))
		_tax_seg[i].visible = c > 0
		_tax_seg[i].size_flags_stretch_ratio = maxf(float(c), 0.001)

	# Sectors: Tier 3's decision is the hue, the swarm's coverage is the strength.
	var directed := 0
	for k in range(_sec_cells.size()):
		var code := int(host.sector_codes[k]) if k < host.sector_codes.size() else 1
		if code != 1:
			directed += 1
		var cov := _sec_cov[k] if k < _sec_cov.size() else 0.0
		var base := _sector_ink(code)
		# A directed sector keeps a floor, so an abandoned sector nobody has searched
		# does not vanish into the panel -- that is the case most worth seeing.
		var a := (0.30 if code != 1 else 0.06) + cov * 0.55
		_sec_cells[k].color = Color(base, minf(a, 0.9))
	if _sec_grid.columns > 0 and not _sec_cells.is_empty():
		_sec_side.text = "%d DIRECTED" % directed
		_sec_caption.text = "%d×%d · COVERAGE · TIER 3" % [
			_sec_grid.columns, ceili(float(_sec_cells.size()) / _sec_grid.columns)]

	# Comms mesh.
	var posted := _act[SwarmDashboard.ACTIVITY["relay_post"]]
	var moving := _act[SwarmDashboard.ACTIVITY["relay_move"]]
	var vals := [active, incomms, maxi(active - incomms, 0), posted, moving, lost]
	for i in range(_comms_val.size()):
		_comms_val[i].text = str(vals[i]) if live else "--"

	# Chassis roster.
	_chassis_side.text = "%d UNITS" % fleet
	for i in range(_chassis_n.size()):
		_chassis_n[i].text = "%d/%d" % [_chassis_alive[i], _chassis_all[i]]
		_fill(_chassis_fill[i], float(_chassis_alive[i]) / maxi(_chassis_all[i], 1))

	# Overlay bus.
	var on := 0
	for i in range(OVERLAYS.size()):
		var id: String = OVERLAYS[i][2]
		var state := _overlay_state(id)
		if state == "ON":
			on += 1
		_ov_state[i].text = state
		_ov_state[i].add_theme_color_override("font_color",
			C_FAINT if state == "OFF" else C_ACCENT)
	_ov_side.text = "%d/%d ACTIVE" % [on, OVERLAYS.size() - 1]

	# Swarm activity.
	var alive := 0
	for n in _act:
		alive += n
	_act_side.text = "%d ALIVE" % alive
	var mine := -1
	if host._on_unit():
		var fr: Array = host.robots[host.follow]
		if fr.size() > 7:
			mine = int(fr[7])
	for i in range(ACTIVITY_ROWS.size()):
		var act_id: String = ACTIVITY_ROWS[i][0]
		var code2: int = SwarmDashboard.ACTIVITY[act_id]
		var n2 := _act[code2]
		_act_n[i].text = str(n2)
		_act_pct[i].text = "%.0f%%" % (100.0 * n2 / maxi(alive, 1))
		_fill(_act_fill[i], float(n2) / maxi(alive, 1))
		_act_mark[i].modulate.a = 1.0 if code2 == mine else 0.0
		if act_id == "idle":
			_act_name[i].add_theme_color_override("font_color",
				_activity_ink(act_id) if n2 > 0 else C_ROW)

	# Event stream header.
	while not _ev_wall.is_empty() and _t - _ev_wall[0] > 10.0:
		_ev_wall.pop_front()
	_log_side.text = "%.1f EV/S · BUF %d" % [_ev_wall.size() / 10.0, LOG_MAX_LINES]

	# Coverage.
	_fill(_cov_fill, explored)
	_cov_head.text = "%.1f%% MAPPED · %+.1f%%/MIN OVER 30 S" % [explored * 100.0, rate] \
		if live else "NO DATA"
	_cov_rate.text = "RESCUES %.1f/MIN" % (rescued / maxf(host.sim_time / 60.0, 1.0 / 60.0)) \
		if live else ""
	_cov_seeded.text = "%d CASUALTIES SEEDED" % host.victims_total if host.victims_total > 0 else ""
	var ledger := [host.digs.size(), _contacts[4], _contacts[2] + _contacts[3], _n_dismissed]
	for i in range(_ledger_val.size()):
		_ledger_val[i].text = str(ledger[i])

	# Hivemind.
	_t3_side.text = "ISSUED %d · FILTERED %d" % [_n_issued, _n_rejected]
	if _n_offline > 0:
		_t3_side.text += " · OFFLINE %d" % _n_offline
	for i in range(_t3_rows.size()):
		var row: Array = _t3_rows[i]
		if i < _t3.size():
			var e: Array = _t3[i]
			(row[0] as Label).text = "%.0fs" % float(e[0])
			(row[1] as Label).text = str(e[1])
			(row[1] as Label).add_theme_color_override("font_color", e[3] as Color)
			(row[2] as Label).text = str(e[2])
		else:
			(row[0] as Label).text = ""
			(row[1] as Label).text = ""
			# Silence is a legitimate state -- `--no-hivemind` is the control condition,
			# and Tier 2 is built to run without a word from Tier 3.
			(row[2] as Label).text = "no directive received yet" if i == 0 else ""


func _overlay_state(id: String) -> String:
	match id:
		"thermal":
			if not host.thermal_on:
				return "OFF"
			return "ON" if host.thermal != null and host.thermal.active else "READY"
		"victims":
			return "ON" if host.show_victims else "OFF"
		"chassis":
			return "ON" if host.colour_by_chassis else "OFF"
		"sectors":
			return "ON" if host.show_sectors else "OFF"
		"fx":
			return "ON" if host.fx_on else "OFF"
		"god":
			return "ON" if host.god_view else "OFF"
		"view":
			return str(SwarmDashboard.VIEW_NAMES[host.view_mode]).to_upper()
	return "?"


func _explore_rate() -> float:
	"""Explored percentage points per minute of sim time, over the last 30 s."""
	if _explore_hist.size() < 2:
		return 0.0
	var a := _explore_hist[0]
	var b := _explore_hist[-1]
	return (b.y - a.y) / maxf(b.x - a.x, 0.001) * 60.0 * 100.0


func _rebuild_sector_grid() -> void:
	for c in _sec_grid.get_children():
		_sec_grid.remove_child(c)
		c.queue_free()
	_sec_cells.clear()
	var n := host.sector_rects.size()
	if n == 0 or host.map_w <= 0.0:
		_sec_foot_l.text = ""
		_sec_foot_r.text = ""
		return
	var r0: Array = host.sector_rects[0]
	var sw := maxf(float(r0[2]) - float(r0[0]), 0.001)
	var sh := maxf(float(r0[3]) - float(r0[1]), 0.001)
	var cols := clampi(roundi(host.map_w / sw), 1, n)
	var rows := ceili(float(n) / cols)
	_sec_grid.columns = cols
	var inner := _px(17.0 - 1.6)
	var cw := floorf((inner - float(cols - 1)) / cols)
	# The cell keeps the sector's own aspect, capped so a tall map cannot push the rest
	# of the rail off the bottom.
	var chh := clampf(floorf(cw * sh / sw), 4.0, floorf(_px(10.0) / rows))
	for k in range(n):
		var cell := ColorRect.new()
		cell.mouse_filter = Control.MOUSE_FILTER_IGNORE
		cell.custom_minimum_size = Vector2(cw, chh)
		cell.color = Color(C_ACCENT, 0.06)
		_sec_grid.add_child(cell)
		_sec_cells.append(cell)
	_sec_foot_l.text = str(host.sector_ids[0]) if host.sector_ids.size() > 0 else ""
	_sec_foot_r.text = str(host.sector_ids[-1]) if host.sector_ids.size() > 0 else ""


func _sector_ink(code: int) -> Color:
	# Normal sectors carry the accent; directed ones borrow the terrain wash's hue.
	if code == 1:
		return C_ACCENT
	var t: Color = SwarmDashboard.SECTOR_TINTS.get(code, C_ACCENT)
	return Color(t.r, t.g, t.b) if code != 2 else C_DIM


func _activity_ink(act_id: String) -> Color:
	# The same two tints the robots themselves carry on the map, and only those two.
	var code: int = SwarmDashboard.ACTIVITY[act_id]
	var tint = SwarmDashboard.ACTIVITY_TINT.get(code)
	if tint != null:
		return Color(tint as Color, 0.85)
	return Color(C_ACCENT, 0.6)


func _append(line: String) -> void:
	if log_box == null:
		return
	log_box.append_text(line + "\n")
	if log_box.get_paragraph_count() > LOG_MAX_LINES:
		log_box.remove_paragraph(0)


func _escape(s: String) -> String:
	return s.replace("[", "[lb]")


# --- viewport frame ----------------------------------------------------------------------


func _draw_view(c: Control) -> void:
	var r := view_rect()
	if r.size.x < 8.0 or r.size.y < 8.0:
		return
	var strip := strip_px()
	var fs := _fs(0.48)
	var font := _font(MONO, roundi(0.18 * fs))

	# Header and footer shades. A solid band under the text, then a fade: the mock's
	# gradient alone was drawn over a dark render, and this world's overcast sky is pale
	# enough to wash readout type out entirely.
	var shade := Color(C_BG, 0.86)
	var clear := Color(C_BG, 0.0)
	var band := _px(1.5)
	var w := r.size.x
	var top := r.position
	var bot := Vector2(r.position.x, r.end.y)
	c.draw_rect(Rect2(top, Vector2(w, band)), shade, true)
	c.draw_polygon(PackedVector2Array([top + Vector2(0, band), top + Vector2(w, band),
		top + Vector2(w, strip), top + Vector2(0, strip)]),
		PackedColorArray([shade, shade, clear, clear]))
	c.draw_rect(Rect2(bot - Vector2(0, band), Vector2(w, band)), shade, true)
	c.draw_polygon(PackedVector2Array([bot - Vector2(0, band), bot + Vector2(w, -band),
		bot + Vector2(w, -strip), bot - Vector2(0, strip)]),
		PackedColorArray([shade, shade, clear, clear]))

	# Frame corners. Amber while the operator has the followed unit, so the whole frame says
	# a hand is on the controls before anyone reads the badge.
	var inset := _px(0.5)
	var arm := _px(1.4)
	var dm := _drive_mode()
	var manual := dm == ManualDrive.Mode.MANUAL or dm == ManualDrive.Mode.HOLDING
	var edge := Color(C_WARN, 0.9) if manual else Color(C_ACCENT, 0.5)
	for corner in [[r.position + Vector2(inset, inset), Vector2(1, 1)],
			[Vector2(r.end.x - inset, r.position.y + inset), Vector2(-1, 1)],
			[Vector2(r.position.x + inset, r.end.y - inset), Vector2(1, -1)],
			[r.end - Vector2(inset, inset), Vector2(-1, -1)]]:
		var o: Vector2 = corner[0]
		var d: Vector2 = corner[1]
		c.draw_line(o, o + Vector2(arm * d.x, 0), edge, 1.0)
		c.draw_line(o, o + Vector2(0, arm * d.y), edge, 1.0)

	var pad := _px(2.2)
	var x := r.position.x + pad
	var inner := w - pad * 2.0
	var head_y := r.position.y + band * 0.5 + fs * 0.35
	var foot_y := r.end.y - band * 0.5 + fs * 0.35

	if not host.ready_world:
		var msg: String = host._blob_progress if host._blob_progress != "" else host.state_name
		var big := _fs(0.9)
		c.draw_string(_font(DISP, roundi(0.16 * big)), Vector2(r.position.x, r.get_center().y),
			msg.to_upper(), HORIZONTAL_ALIGNMENT_CENTER, w, big, C_WARN)
		return

	var cam := host.cam
	# God-view is ground truth and says so on the frame: a screenshot taken with it on
	# must not pass for what the swarm can see.
	var left := "VIEWPORT · FOG OF WAR"
	var left_ink := C_ROW
	if host.god_view:
		left = "GOD VIEW · GROUND TRUTH SHOWN"
		left_ink = C_WARN
	elif host.show_victims:
		left = "CASUALTY PINS · GROUND TRUTH"
		left_ink = C_WARN
	elif host.thermal != null and host.thermal.active:
		left = "THERMAL · SIMULATED SENSOR VIEW"
		left_ink = C_WARN
	var fwd := -cam.global_transform.basis.z
	var az := fposmod(rad_to_deg(atan2(fwd.z, fwd.x)), 360.0)
	var el := rad_to_deg(asin(clampf(fwd.y, -1.0, 1.0)))
	var mode := str(SwarmDashboard.VIEW_NAMES[host.view_mode]).to_upper()
	var centre := "%s · AZ %.1f° · EL %+.1f° · FOV %.0f°" % [mode, az, el, cam.fov]
	var prims := Performance.get_monitor(Performance.RENDER_TOTAL_PRIMITIVES_IN_FRAME)
	var right := "RENDER %.0f FPS · %.2f M PRIMS" % [Engine.get_frames_per_second(), prims / 1.0e6]
	c.draw_string(font, Vector2(x, head_y), left, HORIZONTAL_ALIGNMENT_LEFT, inner, fs, left_ink)
	c.draw_string(font, Vector2(x, head_y), centre, HORIZONTAL_ALIGNMENT_CENTER, inner, fs, C_ROW)
	c.draw_string(font, Vector2(x, head_y), right, HORIZONTAL_ALIGNMENT_RIGHT, inner, fs, C_ROW)
	_draw_drive_badge(c, r)

	var dist := cam.global_position.distance_to(host.cam_target)
	var vp_h := maxf(host.get_viewport().get_visible_rect().size.y, 1.0)
	var per_px := 2.0 * dist * tan(deg_to_rad(cam.fov * 0.5)) / vp_h
	c.draw_string(font, Vector2(x, foot_y), _cursor_text(r), HORIZONTAL_ALIGNMENT_LEFT, inner, fs, C_ROW)
	c.draw_string(font, Vector2(x, foot_y), _selection_text(), HORIZONTAL_ALIGNMENT_CENTER, inner, fs,
		C_ACCENT if host._on_unit() else C_ROW)
	var scale := "%.2f M" % per_px if per_px >= 0.1 else "%.1f CM" % (per_px * 100.0)
	c.draw_string(font, Vector2(x, foot_y), "1 PX ≈ %s AT TARGET" % scale,
		HORIZONTAL_ALIGNMENT_RIGHT, inner, fs, C_ROW)

	# The reticle marks the orbit's look-at point, projected rather than drawn at the
	# middle of the hole. The orbit camera's lens is shifted so the two coincide
	# (`SwarmDashboard._centre_on_view`); projecting keeps the mark honest if they ever
	# do not. POV and chase have their own instrument, the detector overlay, and do not
	# get a second one on top.
	if host.view_mode == SwarmDashboard.View.ORBIT or not host._on_unit():
		if not cam.is_position_behind(host.cam_target):
			var p := cam.unproject_position(host.cam_target)
			if r.grow(-_px(4.0)).has_point(p):
				# Range to the ground under the crosshair, measured along the ray through
				# it. Not `dist`: the orbit centre sits at height zero, underneath the
				# terrain, so the distance to it overstates what the crosshair is on.
				var label := ""
				var aim := ground_hit(p)
				if aim.size() > 0:
					label = "RANGE %.1f M" % cam.project_ray_origin(p).distance_to(aim[0])
					if host._on_unit() and host.follow < host.robot_ids.size():
						label = str(host.robot_ids[host.follow]).to_upper() + " · " + label
				_draw_reticle(c, p, label)


func _drive_mode() -> int:
	return host.drive.mode() if host.drive != null else ManualDrive.Mode.NONE


func _draw_drive_badge(c: Control, r: Rect2) -> void:
	"""Who is steering the followed unit: its autonomy, or the operator at the keys.

	Read from the simulator's echo (`ManualDrive.mode`), so it reports what the robot is
	obeying rather than which keys are down. Unit views only, because that is where the keys
	drive -- a badge in the orbit would offer a control that is not there.
	"""
	var m := _drive_mode()
	if m == ManualDrive.Mode.NONE:
		return
	var text := "AUTONOMOUS · WASD TO DRIVE"
	var ink := C_ACCENT
	if m == ManualDrive.Mode.MANUAL:
		text = "MANUAL · OPERATOR DRIVING"
		ink = C_WARN
	elif m == ManualDrive.Mode.HOLDING:
		text = "MANUAL · HOLDING · AUTONOMY IN %.1f S" % host.drive.hold_left()
		ink = C_WARN
	elif m == ManualDrive.Mode.NO_ACK:
		text = "MANUAL REQUESTED · NO ACK FROM SIMULATOR"
		ink = C_WARN
	var fs := _fs(0.5)
	var font := _font(MONO_SEMI, roundi(0.12 * fs))
	var pad := _px(0.45)
	var bar := maxf(_px(0.18), 2.0)
	var size := font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, fs)
	var plate := Rect2(Vector2(r.position.x + _px(2.2), r.position.y + strip_px()),
		Vector2(bar + size.x + pad * 3.0, font.get_height(fs) + pad * 2.0))
	# On a backing plate, like the reticle label: bare type over lit rubble disappears.
	c.draw_rect(plate, Color(C_BG, 0.78), true)
	c.draw_rect(Rect2(plate.position, Vector2(bar, plate.size.y)), ink, true)
	var base := Vector2(plate.position.x + bar + pad * 1.5,
		plate.position.y + pad + font.get_ascent(fs))
	c.draw_string(font, base, text, HORIZONTAL_ALIGNMENT_LEFT, -1, fs, ink)


func _draw_reticle(c: Control, p: Vector2, label: String) -> void:
	# Small on purpose: it marks a point, and at eagle-eye a large one covers the robots
	# gathered at exactly the place the operator is looking.
	var arm := _px(0.4)
	var rad := arm + _px(0.3)     # where the label sits above the cross
	var cross := Color(C_ACCENT, 0.7)
	c.draw_line(p - Vector2(arm, 0), p + Vector2(arm, 0), cross, 1.0)
	c.draw_line(p - Vector2(0, arm), p + Vector2(0, arm), cross, 1.0)
	# On a backing plate: the label floats over terrain, and accent type alone vanishes
	# against lit rubble.
	var fs := _fs(0.46)
	var font := _font(MONO, roundi(0.16 * fs))
	if label == "":
		return                    # nothing under the crosshair to range
	var size := font.get_string_size(label, HORIZONTAL_ALIGNMENT_LEFT, -1, fs)
	var base := Vector2(p.x - size.x * 0.5, p.y - rad - _px(0.5))
	c.draw_rect(Rect2(base + Vector2(-4.0, -fs - 1.0), Vector2(size.x + 8.0, fs + 6.0)),
		Color(C_BG, 0.72), true)
	c.draw_string(font, base, label, HORIZONTAL_ALIGNMENT_LEFT, -1, fs, Color(C_ACCENT, 0.9))


func _draw_ticks(c: Control) -> void:
	var step := _px(1.6)
	var ink := Color(C_PANEL, 0.7)
	var x := step
	while x < c.size.x:
		c.draw_line(Vector2(x, 0), Vector2(x, c.size.y), ink, 1.0)
		x += step


func _cursor_text(r: Rect2) -> String:
	"""Where the mouse is on the ground, in the simulator's own metres.

	The mock had latitude and longitude; the map has no geodetic frame, so this is the
	frame the event stream and every contact position are already in.
	"""
	var m := host.get_viewport().get_mouse_position()
	if not r.has_point(m):
		return "CURSOR --"
	var hit := ground_hit(m)
	if hit.is_empty():
		return "CURSOR OFF MAP"
	var at: Vector3 = hit[0]
	return "CURSOR %.1f, %.1f M · ELEV %.1f M" % [at.x, at.z, at.y]


func ground_hit(screen: Vector2) -> Array:
	"""Where the camera ray through `screen` first meets the terrain: `[Vector3]`, or
	`[]` for a ray that misses the map.

	Marched, not solved against a plane, because the terrain is a heightfield: a ray
	skimming a ridge meets its near face first, and a plane at any one height puts the
	hit metres from the surface the operator is looking at. The march covers only the
	slab between the highest cell and zero -- heights are never negative -- in half-cell
	steps, then bisects. At the eagle-eye pitch that is a few dozen samples.

	The surface is the one `_build_ground` draws: heights live on cell corners and the
	mesh interpolates between them, so this does too.
	"""
	var cam := host.cam
	if cam == null or host.gw == 0:
		return []
	var o := cam.project_ray_origin(screen)
	var d := cam.project_ray_normal(screen)
	if d.y > -0.0001:
		return []                 # level or upward: it never comes down
	if o.y <= _terrain_y(o.x, o.z):
		return []                 # the orbit has no ground clearance; this eye is in a hill
	var top := host._height_scale * 255.0 + 0.01
	var t := maxf((top - o.y) / d.y, 0.0)
	var t_end := -o.y / d.y
	if t_end < 0.0:
		return []
	var step := host.cell * 0.5
	var prev := t
	while t <= t_end + step:
		var p := o + d * t
		if p.y <= _terrain_y(p.x, p.z):
			var lo := prev
			var hi := t
			for _i in range(14):
				var mid := (lo + hi) * 0.5
				var q := o + d * mid
				if q.y <= _terrain_y(q.x, q.z):
					hi = mid
				else:
					lo = mid
			return [o + d * hi]
		prev = t
		t += step
	return []


func _terrain_y(x: float, z: float) -> float:
	# Off the map there is no ground at all. `h_at` clamps to the edge cell, which would
	# extend the border heights out forever and stop a ray from outside the map on a
	# wall of terrain that is not drawn.
	if x < 0.0 or z < 0.0 or x > host.map_w or z > host.map_h:
		return -1.0
	var fx := x / host.cell
	var fz := z / host.cell
	var ix := int(floor(fx))
	var iz := int(floor(fz))
	var tx := fx - ix
	var tz := fz - iz
	var a := lerpf(host.h_at(ix, iz), host.h_at(ix + 1, iz), tx)
	var b := lerpf(host.h_at(ix, iz + 1), host.h_at(ix + 1, iz + 1), tx)
	return lerpf(a, b, tz)


func _selection_text() -> String:
	if not host._on_unit():
		return "SEL -- · CLICK A UNIT TO FOLLOW"
	var r: Array = host.robots[host.follow]
	var id := str(host.robot_ids[host.follow]) if host.follow < host.robot_ids.size() \
		else "#%d" % host.follow
	var what := ""
	if r.size() > 7 and int(r[7]) >= 0 and int(r[7]) < SwarmDashboard.ACTIVITY_NAMES.size():
		what = " · " + str(SwarmDashboard.ACTIVITY_NAMES[int(r[7])])
	var status := ""
	if int(r[4]) == 1:
		status = " · NO LINK"
	elif int(r[4]) >= 2:
		status = " · DOWN"
	return ("SEL %s · %s/%s · BATT %.0f%%%s%s" % [id, SwarmDashboard.LANE_NAMES[int(r[3])],
		SwarmDashboard.CHASSIS_NAMES[int(r[6])], float(r[5]) * 100.0, status, what]).to_upper()


# --- widget helpers -----------------------------------------------------------------------


func _px(cqw: float) -> float:
	return cqw * _u


func _fs(cqw: float) -> int:
	return maxi(FONT_MIN_PX, roundi(cqw * _u * FONT_BOOST))


func _font(face: int, track_px: int) -> Font:
	"""One FontVariation per face and tracking, cached.

	Tracking is per glyph in pixels, so it is resolved against the label's size by the
	caller. A missing font file falls back to the engine font rather than blanking the
	HUD -- the .ttf files need an editor import before `load` sees them.
	"""
	var key := face * 1000 + track_px
	if _fonts.has(key):
		return _fonts[key]
	var path: String = [FONT_MONO, FONT_MONO_SEMI, FONT_DISPLAY, FONT_DISPLAY][face]
	var base: Font = null
	if ResourceLoader.exists(path):
		base = load(path) as Font
	var fv := FontVariation.new()
	if base == null:
		base = ThemeDB.fallback_font
	elif face == DISP or face == DISP_BOLD:
		# Oxanium ships as one variable font; the weight is an axis, not a file.
		var wght := TextServerManager.get_primary_interface().name_to_tag("wght")
		fv.variation_opentype = {wght: 700 if face == DISP_BOLD else 600}
	fv.base_font = base
	fv.spacing_glyph = track_px
	_fonts[key] = fv
	return fv


func _text(parent: Node, s: String, size: float, col: Color, face: int = MONO,
		track: float = 0.0) -> Label:
	var l := Label.new()
	var fs := _fs(size)
	l.text = s
	l.mouse_filter = Control.MOUSE_FILTER_IGNORE
	l.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	l.add_theme_font_override("font", _font(face, roundi(track * fs)))
	l.add_theme_font_size_override("font_size", fs)
	l.add_theme_color_override("font_color", col)
	parent.add_child(l)
	return l


func _style(bg: Color, px: float, py: float, line: Color = Color(0, 0, 0, 0),
		sides: int = 0) -> StyleBoxFlat:
	# `sides` is a bitmask: 1 left, 2 top, 4 right, 8 bottom.
	var s := StyleBoxFlat.new()
	s.bg_color = bg
	s.content_margin_left = px
	s.content_margin_right = px
	s.content_margin_top = py
	s.content_margin_bottom = py
	s.border_color = line
	s.border_width_left = 1 if sides & 1 else 0
	s.border_width_top = 1 if sides & 2 else 0
	s.border_width_right = 1 if sides & 4 else 0
	s.border_width_bottom = 1 if sides & 8 else 0
	return s


func _panel(parent: Node, style: StyleBox) -> PanelContainer:
	var p := PanelContainer.new()
	p.add_theme_stylebox_override("panel", style)
	p.mouse_filter = Control.MOUSE_FILTER_STOP
	parent.add_child(p)
	return p


func _rail(parent: Node, border: int) -> VBoxContainer:
	var p := _panel(parent, _style(C_PANEL, 0.0, 0.0, C_LINE, border))
	p.custom_minimum_size.x = _px(17.0)
	p.clip_contents = true
	return _vbox(p, 0.0)


func _section(parent: Node, title: String, grow: bool = false, last: bool = false) -> Array:
	"""A titled block of a rail. Returns [body, the label at the right of the title]."""
	var p := _panel(parent, _style(Color(0, 0, 0, 0), _px(0.8), _px(0.65), C_LINE_SOFT,
		0 if last else 8))
	p.mouse_filter = Control.MOUSE_FILTER_IGNORE
	if grow:
		p.size_flags_vertical = Control.SIZE_EXPAND_FILL
	var body := _vbox(p, _px(0.22))
	var head := _hbox(body, _px(0.4))
	var t := _text(head, title, 0.6, C_HEAD, DISP, 0.24)
	t.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	var side := _text(head, "", 0.48, C_DIMMER)
	var gap := Control.new()
	gap.custom_minimum_size.y = _px(0.15)
	gap.mouse_filter = Control.MOUSE_FILTER_IGNORE
	body.add_child(gap)
	return [body, side]


func _vbox(parent: Node, gap: float) -> VBoxContainer:
	var b := VBoxContainer.new()
	b.mouse_filter = Control.MOUSE_FILTER_IGNORE
	b.add_theme_constant_override("separation", roundi(gap))
	parent.add_child(b)
	return b


func _hbox(parent: Node, gap: float) -> HBoxContainer:
	var b := HBoxContainer.new()
	b.mouse_filter = Control.MOUSE_FILTER_IGNORE
	b.add_theme_constant_override("separation", roundi(gap))
	parent.add_child(b)
	return b


func _pad(parent: Node, x: float, y: float) -> MarginContainer:
	var m := MarginContainer.new()
	m.mouse_filter = Control.MOUSE_FILTER_IGNORE
	m.add_theme_constant_override("margin_left", roundi(x))
	m.add_theme_constant_override("margin_right", roundi(x))
	m.add_theme_constant_override("margin_top", roundi(y))
	m.add_theme_constant_override("margin_bottom", roundi(y))
	parent.add_child(m)
	return m


func _spring(parent: Node) -> void:
	var s := Control.new()
	s.mouse_filter = Control.MOUSE_FILTER_IGNORE
	s.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	parent.add_child(s)


func _vline(parent: Node, col: Color) -> void:
	var l := ColorRect.new()
	l.color = col
	l.mouse_filter = Control.MOUSE_FILTER_IGNORE
	l.custom_minimum_size = Vector2(1, 0)
	parent.add_child(l)


func _hline(parent: Node, col: Color) -> void:
	var l := ColorRect.new()
	l.color = col
	l.mouse_filter = Control.MOUSE_FILTER_IGNORE
	l.custom_minimum_size = Vector2(0, 1)
	parent.add_child(l)


func _vsep(parent: Node, pad: float, col: Color) -> void:
	var a := Control.new()
	a.custom_minimum_size.x = pad
	a.mouse_filter = Control.MOUSE_FILTER_IGNORE
	parent.add_child(a)
	_vline(parent, col)
	var b := Control.new()
	b.custom_minimum_size.x = pad
	b.mouse_filter = Control.MOUSE_FILTER_IGNORE
	parent.add_child(b)


func _dot(parent: Node, d: float, col: Color) -> Panel:
	var side := maxf(5.0, roundf(_px(d)))
	var s := StyleBoxFlat.new()
	s.bg_color = col
	s.set_corner_radius_all(roundi(side * 0.5))
	var p := Panel.new()
	p.add_theme_stylebox_override("panel", s)
	p.custom_minimum_size = Vector2(side, side)
	p.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	p.mouse_filter = Control.MOUSE_FILTER_IGNORE
	parent.add_child(p)
	return p


func _bar(parent: Node, h: float, col: Color) -> ColorRect:
	"""A thin track with a fill. Returns the fill; `_fill` sets it."""
	var track := ColorRect.new()
	track.color = C_TRACK
	track.mouse_filter = Control.MOUSE_FILTER_IGNORE
	track.custom_minimum_size = Vector2(0, maxf(2.0, roundf(_px(h))))
	parent.add_child(track)
	var fill := ColorRect.new()
	fill.color = col
	fill.mouse_filter = Control.MOUSE_FILTER_IGNORE
	track.add_child(fill)
	# `keep_offset` true, or Godot moves the offsets to hold the old rect in place and the
	# fill never changes width.
	fill.set_anchor(SIDE_BOTTOM, 1.0, true)
	fill.set_anchor(SIDE_RIGHT, 0.0, true)
	return fill


func _fill(fill: ColorRect, f: float) -> void:
	fill.set_anchor(SIDE_RIGHT, clampf(f, 0.0, 1.0), true)
