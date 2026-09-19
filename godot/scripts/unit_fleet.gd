class_name UnitFleet
extends Node3D
## Batched port of swarmmind/viz/units.py, using its exported geometry and joints.
## Only visible units enter variant/LOD batches; geometry is loaded on demand.

const MANIFEST := "res://assets/units/fleet_index.json"
const UNIT_SHADER := preload("res://shaders/units.gdshader")
const LANES := ["none", "scoop", "gripper", "antenna"]
const CHASSIS := ["wheeled", "tracked", "legged", "rotor"]
const JOINT_KINDS := ["static", "wheel", "gait", "rotor", "scan", "dig", "grip", "relay", "payload"]
const MAX_JOINTS := 32
const MAX_COLORS := 16
const MAX_EXTRAPOLATION := 0.2
const DIG_REACH := 2.5
const HIGH_PIXELS := 34.0
const MEDIUM_PIXELS := 10.0
const LOD_HYSTERESIS := 0.8
const MAX_HIGH_UNITS := 12
const RELEASE_DELAY := 8.0
const VIEW_INTERVAL := 0.05
const SUPPORT_CULL_MARGIN := 8.0
const REST := 0
const SCAN := 1
const DIG := 2
const CARRY := 3
const RELAY := 4
const DISABLED := 5
const WORK_ACTIVITY := {
	"explore": 2,
	"investigate": 3,
	"dig": 4,
	"carry": 6,
	"relay_post": 7,
}

var _groups: Dictionary = {}
var _catalog: Dictionary = {}
var _signature := PackedInt32Array()
var _slots := PackedInt32Array()
var _slot_batches := PackedInt32Array()
var _lods := PackedInt32Array()
var _poses: Array[Transform3D] = []
var _states := PackedInt32Array()
var _colors := PackedColorArray()
var _headings := PackedFloat32Array()
var _ground_times := PackedFloat64Array()
var _ground_ready := PackedByteArray()
var _surface_pose := Callable()
var _surface_height := Callable()
var _ground_centres_dirty := false
var _previous := PackedVector2Array()
var _travel := PackedFloat32Array()
var _speed := PackedFloat32Array()
var _last_time := -1.0
var _received_at := 0.0
var _can_extrapolate := false
var _extra := 0.0
var _dirty := true
var _next_view := 0.0
var _hidden_robot := -1
var _animation_time := 0.0
var stats: Dictionary = {"visible": 0, "culled": 0, "triangles": 0,
	"lod_counts": [0, 0, 0], "loaded_meshes": 0, "draw_calls": 0}


func load_models(_map_width: float, _map_depth: float, _elevation: float) -> void:
	for group in _groups.values():
		group["node"].queue_free()
	_groups.clear()
	_catalog.clear()
	var decoded = JSON.parse_string(FileAccess.get_file_as_string(MANIFEST))
	if not decoded is Dictionary:
		push_error("Unit fleet manifest is missing or invalid: " + MANIFEST)
		return
	for model in decoded.get("models", []):
		var key: int = LANES.find(str(model["lane"])) * 4 + CHASSIS.find(str(model["chassis"]))
		_catalog[key] = model
	_dirty = true
	stats["loaded_meshes"] = 0


func _ensure_group(batch: int, now: float) -> bool:
	if _groups.has(batch):
		_groups[batch]["last_used"] = now
		return true
	var variant := batch / 3
	var lod := batch % 3
	var entry: Dictionary = _catalog[variant]["lods"][lod]
	var path := MANIFEST.get_base_dir().path_join(str(entry["file"]))
	var decoded = JSON.parse_string(FileAccess.get_file_as_string(path))
	if not decoded is Dictionary:
		push_error("Unit LOD geometry is missing or invalid: " + path)
		return false
	var material := ShaderMaterial.new()
	material.shader = UNIT_SHADER
	material.set_shader_parameter("animation_time", _animation_time)
	material.set_shader_parameter("extrapolation", _extra)
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.use_colors = true
	mm.use_custom_data = true
	mm.mesh = _mesh_for(decoded, material)
	var node := MultiMeshInstance3D.new()
	node.name = str(decoded["name"])
	node.multimesh = mm
	node.material_override = material
	node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(node)
	_groups[batch] = {"mesh": mm, "material": material, "node": node,
		"last_used": now, "triangles": int(entry["triangles"])}
	return true


static func _map_vector(v: Array) -> Vector3:
	return Vector3(float(v[0]), float(v[2]), float(v[1]))


func _mesh_for(model: Dictionary, material: ShaderMaterial) -> ArrayMesh:
	var pivots := PackedVector3Array()
	var axes := PackedVector3Array()
	var parameters := PackedColorArray()
	pivots.resize(MAX_JOINTS)
	axes.resize(MAX_JOINTS)
	parameters.resize(MAX_JOINTS)
	var joints: Array = model["joints"]
	assert(joints.size() <= MAX_JOINTS)
	for j in range(joints.size()):
		var joint: Dictionary = joints[j]
		pivots[j] = _map_vector(joint["pivot"])
		# X/Z/Y is a reflection: rotation axes are pseudovectors. Negating the
		# mapped axis keeps the original right-handed joint rotation unchanged.
		axes[j] = -_map_vector(joint["axis"]).normalized()
		parameters[j] = Color(float(JOINT_KINDS.find(str(joint["kind"]))),
			float(joint["amplitude"]), float(joint["frequency"]), float(joint["phase"]))
	material.set_shader_parameter("joint_pivots", pivots)
	material.set_shader_parameter("joint_axes", axes)
	material.set_shader_parameter("joint_parameters", parameters)

	var vertices := PackedVector3Array()
	var normals := PackedVector3Array()
	var colors := PackedColorArray()
	var uv := PackedVector2Array()
	var uv2 := PackedVector2Array()
	var palette := PackedVector3Array()
	for part in model["parts"]:
		var rgb: Array = part["color"]
		var paint := Vector3(float(rgb[0]), float(rgb[1]), float(rgb[2]))
		var color_index := palette.find(paint)
		if color_index < 0:
			color_index = palette.size()
			palette.append(paint)
		var points: Array = part["vertices"]
		for triangle in part["triangles"]:
			var a := _map_vector(points[int(triangle[0])])
			var b := _map_vector(points[int(triangle[1])])
			var c := _map_vector(points[int(triangle[2])])
			# Source is CCW in Z-up; reflection makes Godot's clockwise winding.
			# Geometric normals transform as vectors, hence this reversed cross.
			var normal := (c - a).cross(b - a).normalized()
			for point in [a, b, c]:
				vertices.append(point)
				normals.append(normal)
				# Godot multiplies mesh COLOR by MultiMesh instance COLOR. White
				# leaves the per-instance accent/dimming intact; UV2 selects paint.
				colors.append(Color.WHITE)
				uv.append(Vector2(float(part["joint"]), float(part["accent"])))
				uv2.append(Vector2(float(color_index), 0.0))
	assert(palette.size() <= MAX_COLORS)
	palette.resize(MAX_COLORS)
	material.set_shader_parameter("part_palette", palette)
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_COLOR] = colors
	arrays[Mesh.ARRAY_TEX_UV] = uv
	arrays[Mesh.ARRAY_TEX_UV2] = uv2
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh


func reset_motion() -> void:
	_last_time = -1.0
	_can_extrapolate = false
	_extra = 0.0
	_travel.fill(0.0)
	_speed.fill(0.0)
	_previous.clear()
	_ground_ready.fill(0)
	invalidate_grounding()


func invalidate_grounding() -> void:
	# Terrain residency/LOD changes can move the drawn triangle without telemetry.
	# Bypass both refresh timers so the new surface and its units update together.
	_ground_times.fill(-1.0)
	_ground_centres_dirty = true
	_dirty = true


func _map_roster(rows: Array) -> bool:
	var changed := rows.size() != _signature.size()
	if not changed:
		for i in range(rows.size()):
			if _signature[i] != int(rows[i][3]) * 4 + int(rows[i][6]):
				changed = true
				break
	if not changed:
		return false
	_signature.resize(rows.size())
	_slots.resize(rows.size())
	_slot_batches.resize(rows.size())
	_lods.resize(rows.size())
	_poses.resize(rows.size())
	_states.resize(rows.size())
	_colors.resize(rows.size())
	_headings.resize(rows.size())
	_ground_times.resize(rows.size())
	_ground_ready.resize(rows.size())
	_travel.resize(rows.size())
	_speed.resize(rows.size())
	_slots.fill(-1)
	_slot_batches.fill(-1)
	_lods.fill(2)
	_colors.fill(Color.WHITE)
	for i in range(rows.size()):
		var key := int(rows[i][3]) * 4 + int(rows[i][6])
		_signature[i] = key
	for key in _groups:
		var mm: MultiMesh = _groups[key]["mesh"]
		mm.visible_instance_count = 0
	reset_motion()
	_dirty = true
	return true


static func animation_state(lane: int, chassis: int, activity: int, status: int,
		position: Vector2, sites: Array) -> int:
	if status >= 2:
		return DISABLED
	if lane == 1 and activity == WORK_ACTIVITY["dig"]:
		for site in sites:
			if position.distance_squared_to(Vector2(float(site[0]), float(site[1]))) <= DIG_REACH * DIG_REACH:
				return DIG
	if lane == 2 and activity == WORK_ACTIVITY["carry"]:
		return CARRY
	if lane == 3 and activity == WORK_ACTIVITY["relay_post"]:
		return RELAY
	if lane == 0 and chassis != 3 and activity in [WORK_ACTIVITY["explore"], WORK_ACTIVITY["investigate"]]:
		return SCAN
	return REST


func sync_state(rows: Array, sites: Array, time: float, wall_time: float,
		height_at: Callable, cell_size: float, grid_width: int, grid_height: int,
		surface_pose: Callable = Callable(), surface_height: Callable = Callable()) -> void:
	_map_roster(rows)
	_surface_pose = surface_pose
	_surface_height = surface_height
	_ground_times.fill(-1.0)
	var dt := time - _last_time
	var reset := _last_time < 0.0 or dt < 0.0 or _previous.size() != rows.size()
	if reset:
		reset_motion()
		_previous.resize(rows.size())
	# Duplicate frames refresh no motion or time. Even delayed positive frames get
	# measured displacement/dt, never a guessed speed from an activity label.
	_can_extrapolate = not reset and dt > 0.0 and dt <= MAX_EXTRAPOLATION * 2.0
	if dt != 0.0 or reset:
		_received_at = wall_time
		_last_time = time
	for i in range(rows.size()):
		var r: Array = rows[i]
		var point := Vector2(float(r[0]), float(r[1]))
		_speed[i] = 0.0
		if not reset and dt > 0.0 and int(r[4]) < 2:
			var displacement := point - _previous[i]
			var forward := Vector2(cos(float(r[2])), sin(float(r[2])))
			var distance := displacement.length()
			if displacement.dot(forward) < 0.0:
				distance = -distance
			# Compatibility stores INSTANCE_CUSTOM in half precision. Keep distance
			# inside one mechanism cycle so long missions do not quantise wheel turns
			# into metre-sized jumps. Every wheel on a chassis has the same radius.
			var period: float = _catalog.get(_signature[i], {}).get("travel_period", 1.0)
			_travel[i] = fposmod(_travel[i] + distance, period)
			_speed[i] = distance / dt if _can_extrapolate else 0.0
		_previous[i] = point
		if not _catalog.has(_signature[i]):
			continue
		_headings[i] = float(r[2])
		var ix := clampi(int(point.x / cell_size), 0, grid_width - 1)
		var iy := clampi(int(point.y / cell_size), 0, grid_height - 1)
		var ground_height := float(height_at.call(ix, iy))
		if _ground_ready[i] != 0 and not _ground_centres_dirty:
			# Keep the last supported elevation between telemetry frames. Reverting
			# to a raw cell here can cull a grounded unit before resampling its pose.
			ground_height = _poses[i].origin.y
		elif _surface_height.is_valid():
			ground_height = float(_surface_height.call(point.x, point.y))
		# The exact rendered centre is cheap; footprint/tilt remains visible-only.
		_poses[i] = Transform3D(Basis(Vector3.UP, -float(r[2])),
			Vector3(point.x, ground_height, point.y))
		_states[i] = animation_state(int(r[3]), int(r[6]), int(r[7]), int(r[4]), point, sites)
	_dirty = true


func set_unit_color(index: int, accent: Color, dim: float) -> void:
	if index >= 0 and index < _signature.size():
		var color := Color(accent.r, accent.g, accent.b, dim)
		if _colors[index] != color:
			_colors[index] = color
			_dirty = true


static func choose_lod(pixels: float, previous: int, followed: bool,
		force_detail: bool = false) -> int:
	if followed:
		return 0
	# The viewport projection, including FOV, controls detail. Follow views can
	# request slightly earlier upgrades without forcing distant units to full detail.
	var bias := 0.8 if force_detail else 1.0
	var high := HIGH_PIXELS * bias * (LOD_HYSTERESIS if previous == 0 else 1.0)
	var medium := MEDIUM_PIXELS * bias * (LOD_HYSTERESIS if previous <= 1 else 1.0)
	if pixels >= high:
		return 0
	return 1 if pixels >= medium else 2


func _release_idle_batches(now: float) -> void:
	for batch in _groups.keys():
		var group: Dictionary = _groups[batch]
		if now - float(group["last_used"]) > RELEASE_DELAY:
			group["node"].queue_free()
			_groups.erase(batch)
	stats["loaded_meshes"] = _groups.size()


func update_view(camera: Camera3D, followed_robot: int, force_detail: bool = false) -> void:
	if camera == null:
		return
	var now := Time.get_ticks_msec() * 0.001
	_release_idle_batches(now)
	if _previous.size() != _signature.size():
		# A new hello clears trajectories before the next complete state arrives.
		for group in _groups.values():
			group["mesh"].visible_instance_count = 0
			group["node"].visible = false
		stats["visible"] = 0
		stats["triangles"] = 0
		return
	# Telemetry/color changes upload immediately; camera-only changes are capped
	# at 20 Hz. Animated joints advance separately at the display frame rate.
	if not _dirty and now < _next_view:
		return
	_next_view = now + VIEW_INTERVAL
	_dirty = false
	if _ground_centres_dirty:
		if _surface_height.is_valid():
			for i in range(_signature.size()):
				_poses[i].origin.y = float(_surface_height.call(_previous[i].x, _previous[i].y))
		_ground_centres_dirty = false
	var planes := camera.get_frustum()
	var projection := camera.get_camera_projection()
	var focal := absf(projection.y.y) * camera.get_viewport().get_visible_rect().size.y * 0.5
	var eye := camera.global_position
	var forward := -camera.global_basis.z
	var candidates: Array = []
	var visible := PackedInt32Array()
	var chosen := PackedInt32Array()
	chosen.resize(_signature.size())
	chosen.fill(-1)
	_slots.fill(-1)
	_slot_batches.fill(-1)
	for i in range(_signature.size()):
		if not _catalog.has(_signature[i]):
			continue
		var centre := _poses[i].origin + _poses[i].basis.y
		var radius: float = _catalog[_signature[i]]["radius"]
		# A support footprint on a coarse crest may sit above its centre sample.
		# Extrude the candidate sphere upward while retaining tight horizontal
		# bounds; final instance bounds still use the actual supported transform.
		var cull_centre := centre + Vector3.UP * (SUPPORT_CULL_MARGIN * 0.5)
		var in_view := true
		for plane in planes:
			if plane.distance_to(cull_centre) > radius + absf(plane.normal.y) * (SUPPORT_CULL_MARGIN * 0.5):
				in_view = false
				break
		if not in_view:
			continue
		var depth := maxf((centre - eye).dot(forward), 0.1)
		var pixels := 2.0 * focal
		if camera.projection != Camera3D.PROJECTION_ORTHOGONAL:
			pixels /= depth
		var lod := choose_lod(pixels, _lods[i], i == followed_robot, force_detail)
		chosen[i] = lod
		visible.append(i)
		if lod == 0:
			candidates.append([i, 1.0e20 if i == followed_robot else pixels])
	# Dense close-up crowds remain bounded. Retain the selected unit first, then
	# the largest projected silhouettes; robot id makes ties deterministic.
	candidates.sort_custom(func(a: Array, b: Array) -> bool:
		return int(a[0]) < int(b[0]) if a[1] == b[1] else float(a[1]) > float(b[1]))
	for rank in range(MAX_HIGH_UNITS, candidates.size()):
		chosen[int(candidates[rank][0])] = 1
	var batches: Dictionary = {}
	var counts := [0, 0, 0]
	for i in visible:
		var lod := chosen[i]
		_lods[i] = lod
		counts[lod] += 1
		var batch := _signature[i] * 3 + lod
		if not batches.has(batch):
			batches[batch] = PackedInt32Array()
		batches[batch].append(i)
	for batch in _groups:
		_groups[batch]["mesh"].visible_instance_count = 0
		_groups[batch]["node"].visible = false
	var triangles := 0
	var draw_calls := 0
	for batch in batches:
		if not _ensure_group(batch, now):
			continue
		var group: Dictionary = _groups[batch]
		var mm: MultiMesh = group["mesh"]
		var indices: PackedInt32Array = batches[batch]
		if mm.instance_count < indices.size():
			var capacity := maxi(mm.instance_count, 8)
			while capacity < indices.size():
				capacity *= 2
			mm.instance_count = capacity
		var bounds := AABB(_poses[indices[0]].origin - Vector3.ONE * 3.0, Vector3.ONE * 6.0)
		var hidden_slot := -1
		for slot in range(indices.size()):
			var i := indices[slot]
			if _surface_pose.is_valid() and now - _ground_times[i] >= 0.09:
				var point := _previous[i]
				_poses[i] = _surface_pose.call(point.x, point.y, _headings[i], _signature[i] % 4)
				_ground_times[i] = now
				_ground_ready[i] = 1
			_slots[i] = slot
			_slot_batches[i] = batch
			mm.set_instance_transform(slot, _poses[i])
			mm.set_instance_color(slot, _colors[i])
			mm.set_instance_custom_data(slot, Color(_travel[i], _speed[i], float(_states[i]), 0.0))
			bounds = bounds.merge(AABB(_poses[i].origin - Vector3.ONE * 3.0, Vector3.ONE * 6.0))
			if i == _hidden_robot:
				hidden_slot = slot
		mm.custom_aabb = bounds
		mm.visible_instance_count = indices.size()
		group["node"].visible = true
		group["material"].set_shader_parameter("hidden_instance", hidden_slot)
		triangles += indices.size() * int(group["triangles"])
		draw_calls += 1
	stats = {"visible": visible.size(), "culled": _signature.size() - visible.size(),
		"triangles": triangles, "lod_counts": counts,
		"loaded_meshes": _groups.size(), "draw_calls": draw_calls}


func advance(wall_time: float, connected: bool, hidden_robot: int) -> void:
	# No engine TIME: all mechanisms stop when telemetry stops. Freeze at the last
	# displayed phase on disconnect, rather than jumping back to the source frame.
	if connected and _can_extrapolate:
		_extra = clampf(wall_time - _received_at, 0.0, MAX_EXTRAPOLATION)
	elif connected:
		_extra = 0.0
	_animation_time = maxf(_last_time, 0.0) + _extra
	_hidden_robot = hidden_robot
	for key in _groups:
		var material: ShaderMaterial = _groups[key]["material"]
		material.set_shader_parameter("animation_time", _animation_time)
		material.set_shader_parameter("extrapolation", _extra)
		var hidden_slot := -1
		if hidden_robot >= 0 and hidden_robot < _signature.size() and _slot_batches[hidden_robot] == key:
			hidden_slot = _slots[hidden_robot]
		material.set_shader_parameter("hidden_instance", hidden_slot)
