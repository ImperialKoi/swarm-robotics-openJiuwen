class_name TerrainStream
extends Node3D
## Camera-driven terrain residency. Only height samples and tile bounds are permanent.
## Orbit uses projected-cell LOD; POV/chase always request stride 1 throughout the view.

const TILE_CELLS := 32
const RELEASE_SECONDS := 1.2
const BUILD_BUDGET_USEC := 3000
const MAX_BUILDS := 2

var surface: TerrainSurface
var _material: ShaderMaterial
var _water_material: ShaderMaterial
var _cliff: MeshInstance3D
var _tiles: Array[Dictionary] = []
var _resident: Dictionary = {}
var _visible: Dictionary = {}
var stats: Dictionary = {}
var revision := 0


func setup(source: TerrainSurface, width: int, height: int, material: ShaderMaterial) -> void:
	surface = source
	_material = material
	_water_material = material.duplicate() as ShaderMaterial
	_water_material.shader = load("res://shaders/water.gdshader")
	for child in get_children():
		child.queue_free()
	_cliff = MeshInstance3D.new()
	_cliff.mesh = surface.build_cliff_mesh()
	_cliff.material_override = material
	_cliff.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(_cliff)
	_tiles.clear()
	_resident.clear()
	_visible.clear()
	revision += 1
	for y in range(0, height, TILE_CELLS):
		for x in range(0, width, TILE_CELLS):
			var right := mini(x + TILE_CELLS, width)
			var bottom := mini(y + TILE_CELLS, height)
			_tiles.append({"rect": Rect2i(x, y, right - x, bottom - y),
				"bounds": surface.tile_bounds(x, y, right, bottom)})


static func intersects_view(bounds: AABB, planes: Array[Plane]) -> bool:
	var centre := bounds.get_center()
	var half := bounds.size * 0.5
	for plane in planes:
		# Godot's frustum normals point outwards; reject only a wholly outside box.
		if plane.distance_to(centre) > plane.normal.abs().dot(half):
			return false
	return true


func _desired_stride(camera: Camera3D, bounds: AABB, detail: bool, old: int) -> int:
	if detail:
		return 1
	var distance := maxf(1.0, camera.global_position.distance_to(bounds.get_center())
		- bounds.size.length() * 0.5)
	var pixels := camera.get_viewport().get_visible_rect().size.y
	var focal := pixels * 0.5 / tan(deg_to_rad(camera.fov * 0.5))
	var cell_pixels := surface.cell * focal / distance
	var stride := 1
	# Far triangles become larger, visibly faceted shapes. At most 8 cells per edge.
	while stride < 8 and cell_pixels * stride < 5.0:
		stride *= 2
	# A dead band prevents rebuilding tiles repeatedly while orbiting across a threshold.
	if old > 0 and stride != old:
		var projected := cell_pixels * old
		if projected >= 4.0 and projected <= 12.0:
			return old
	return stride


func update_view(camera: Camera3D, force_detail: bool = false) -> void:
	if surface == null:
		return
	var now := Time.get_ticks_msec() * 0.001
	var planes := camera.get_frustum()
	_visible.clear()
	var pending: Array[Dictionary] = []
	for id in range(_tiles.size()):
		var tile: Dictionary = _tiles[id]
		var bounds: AABB = tile.bounds
		if not intersects_view(bounds, planes):
			continue
		var old := int(_resident[id].stride) if _resident.has(id) else 0
		var stride := _desired_stride(camera, bounds, force_detail, old)
		_visible[id] = stride
		if old != stride:
			pending.append({"id": id, "stride": stride,
				"distance": camera.global_position.distance_squared_to(bounds.get_center())})
	for id in _resident.keys():
		var item: Dictionary = _resident[id]
		var visible_now := _visible.has(id)
		(item.node as Node3D).visible = visible_now
		if visible_now:
			item.last_seen = now
		elif now - float(item.last_seen) > RELEASE_SECONDS:
			(item.node as Node3D).queue_free()
			_resident.erase(id)
			revision += 1
	pending.sort_custom(func(a: Dictionary, b: Dictionary) -> bool: return a.distance < b.distance)
	var started := Time.get_ticks_usec()
	var built := 0
	for request in pending:
		if built >= MAX_BUILDS or (built > 0 and Time.get_ticks_usec() - started > BUILD_BUDGET_USEC):
			break
		_build_tile(int(request.id), int(request.stride), now)
		built += 1
	var triangles := 0
	var levels := {1: 0, 2: 0, 4: 0, 8: 0}
	for id in _resident:
		if _visible.has(id):
			triangles += int(_resident[id].triangles)
			levels[int(_resident[id].stride)] += 1
	stats = {"visible_tiles": _visible.size(), "loaded_tiles": _resident.size(),
		"total_tiles": _tiles.size(), "triangles": triangles, "lod": levels,
		"pending": maxi(0, pending.size() - built)}


func _build_tile(id: int, stride: int, now: float) -> void:
	var rect: Rect2i = _tiles[id].rect
	var node := Node3D.new()
	var terrain_mesh := surface.build_mesh(rect.position.x, rect.position.y,
		rect.end.x, rect.end.y, stride)
	var terrain := MeshInstance3D.new()
	terrain.mesh = terrain_mesh
	terrain.material_override = _material
	terrain.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	node.add_child(terrain)
	var triangles := terrain_mesh.surface_get_array_index_len(0) / 3
	var wet := surface.build_water_mesh(rect.position.x, rect.position.y, rect.end.x, rect.end.y)
	if wet != null and wet.get_surface_count() > 0:
		var water_node := MeshInstance3D.new()
		water_node.mesh = wet
		water_node.material_override = _water_material
		water_node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		node.add_child(water_node)
		triangles += wet.surface_get_array_index_len(0) / 3
	add_child(node)
	# Keep the previous LOD until its replacement is complete; never leave a swap hole.
	if _resident.has(id):
		(_resident[id].node as Node3D).visible = false
		(_resident[id].node as Node3D).queue_free()
	_resident[id] = {"node": node, "stride": stride, "last_seen": now, "triangles": triangles}
	revision += 1


func sync_material() -> void:
	# Textures are shared; toggles are values and need forwarding to the water shader.
	for parameter in ["map_size", "fog_tex", "sector_tex", "god_view", "show_sectors"]:
		_water_material.set_shader_parameter(parameter, _material.get_shader_parameter(parameter))


func stride_at(x: float, y: float) -> int:
	var columns := ceili(float(surface.gw) / TILE_CELLS)
	var tx := clampi(int(x / surface.cell), 0, surface.gw - 1) / TILE_CELLS
	var ty := clampi(int(y / surface.cell), 0, surface.gh - 1) / TILE_CELLS
	var id := ty * columns + tx
	return int(_resident[id].stride) if _resident.has(id) else 1


func height_at_world(x: float, y: float) -> float:
	return surface.height_at_world(x, y, stride_at(x, y))


func robot_pose(x: float, y: float, heading: float, chassis: int) -> Transform3D:
	var stride := stride_at(x, y)
	var pose := surface.robot_pose(x, y, heading, chassis, stride)
	# At a mixed-LOD tile seam, the neighbouring mesh may sit higher than the centre's
	# mesh. Check the full support footprint against each tile's currently drawn LOD.
	var width := 1.05 if chassis == 3 else (0.92 if chassis == 2 else 0.73)
	var rear := -1.05 if chassis == 3 else -0.85
	var checked := {stride: true}
	for forward in [rear, 1.85]:
		for lateral in [-width, width]:
			var offset := pose.basis * Vector3(forward, 0.0, -lateral)
			var other := stride_at(x + offset.x, y + offset.z)
			if not checked.has(other):
				pose.origin.y = maxf(pose.origin.y,
					surface.support_height(x, y, pose.basis, chassis, other))
				checked[other] = true
	return pose
