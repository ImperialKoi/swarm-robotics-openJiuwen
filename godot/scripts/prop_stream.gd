class_name PropStream
extends Node3D
## Streamed disaster scenery. Reference: swarmmind/viz/prop_stream.py.
## Tall geometry stays within blocked cells; passable rubble is only 6 cm deep.
## Every LOD retains the same merged obstacle cover. No imported scene is loaded.

const TILE_METRES := 32.0
const MERGE_METRES := 8.0
const NEAR_DISTANCE := 95.0
const MID_DISTANCE := 210.0
const PRELOAD_MARGIN := 8.0
const RETAIN_MSEC := 1200
const SCAN_MSEC := 80
const BUILD_BUDGET := 2
const BUILD_BUDGET_USEC := 3000
const WALL_COLOR := Color(0.47, 0.45, 0.38)
const SLAB_COLOR := Color(0.76, 0.70, 0.57)
const RUBBLE_COLOR := Color(0.43, 0.39, 0.31)
const ROOF_COLOR := Color(0.43, 0.22, 0.15)
const WOOD_COLOR := Color(0.28, 0.21, 0.13)
const LEAF_COLOR := Color(0.16, 0.29, 0.20)
const ROCK_COLOR := Color(0.48, 0.49, 0.43)
const REGION_METRES := 40.0
const FACES := [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4],
	[1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]

var _surface: RefCounted
var _terrain: Node
var _occupancy := PackedByteArray()
var _width := 0
var _height := 0
var _cell := 1.0
var _material: ShaderMaterial
var _tiles: Array[Dictionary] = []
var _resident: Dictionary = {}
var _wanted: Dictionary = {}
var _queue: Array[int] = []
var _next_scan := 0
var _last_force_detail := false


func setup(surface: RefCounted, occupancy: PackedByteArray, width: int, height: int,
		cell: float, material: ShaderMaterial, terrain: Node = null) -> void:
	for entry: Dictionary in _resident.values():
		(entry["node"] as Node3D).queue_free()
	_resident.clear()
	_tiles.clear()
	_wanted.clear()
	_queue.clear()
	_surface = surface
	_terrain = terrain
	_occupancy = occupancy.duplicate()
	# The rendering copy omits submerged props; simulation occupancy is untouched.
	for i in range(_occupancy.size()):
		if float(surface.water[i]) > 0.02:
			_occupancy[i] = 0
	_width = width
	_height = height
	_cell = cell
	_material = material
	_next_scan = 0
	var step := maxi(1, int(TILE_METRES / _cell))
	for y in range(0, _height, step):
		for x in range(0, _width, step):
			var end := Vector2i(mini(x + step, _width), mini(y + step, _height))
			# Metadata only: no meshes, materials, scenes or GPU instances per tile yet.
			var bounds: AABB = _surface.tile_bounds(x, y, end.x, end.y)
			bounds.size.y += 8.0
			_tiles.append({"start": Vector2i(x, y), "end": end, "bounds": bounds})


func update_view(camera: Camera3D, force_detail: bool = false) -> void:
	if _surface == null or not camera.is_inside_tree():
		return
	var now := Time.get_ticks_msec()
	# Revisit visibility at 12.5 Hz, but queue construction is bounded per render frame.
	if now >= _next_scan or force_detail != _last_force_detail:
		_scan(camera, force_detail, now)
		_next_scan = now + SCAN_MSEC
		_last_force_detail = force_detail
	var budget := BUILD_BUDGET
	var started := Time.get_ticks_usec()
	while budget > 0 and not _queue.is_empty():
		if budget < BUILD_BUDGET and Time.get_ticks_usec() - started >= BUILD_BUDGET_USEC:
			break
		var key: int = _queue.pop_front()
		if not _wanted.has(key):
			continue
		var wish: Dictionary = _wanted[key]
		var lod: int = wish["lod"]
		if _resident.has(key) and int(_resident[key]["lod"]) == lod \
				and _resident[key]["ground_key"] == _ground_key(_tiles[key]):
			continue
		var built := _build_tile(key, lod)
		built["last_seen"] = now
		(built["node"] as Node3D).visible = wish["visible"]
		if _resident.has(key):
			(_resident[key]["node"] as Node3D).queue_free()
		_resident[key] = built
		budget -= 1


func _scan(camera: Camera3D, force_detail: bool, now: int) -> void:
	var planes := camera.get_frustum()
	_wanted.clear()
	_queue.clear()
	var priority: Array[Dictionary] = []
	for key in range(_tiles.size()):
		var bounds: AABB = _tiles[key]["bounds"]
		var visible_now := _intersects_frustum(bounds, planes, 0.0)
		var warm := visible_now or _intersects_frustum(bounds, planes, PRELOAD_MARGIN)
		if not warm:
			continue
		var distance := camera.global_position.distance_to(bounds.get_center())
		var lod := _choose_lod(distance, force_detail, key)
		_wanted[key] = {"lod": lod, "visible": visible_now}
		if not _resident.has(key) or int(_resident[key]["lod"]) != lod \
				or _resident[key]["ground_key"] != _ground_key(_tiles[key]):
			priority.append({"key": key, "distance": distance + (0.0 if visible_now else 10000.0)})
	for key: int in _resident.keys():
		var entry: Dictionary = _resident[key]
		var node := entry["node"] as Node3D
		if _wanted.has(key):
			entry["last_seen"] = now
			node.visible = _wanted[key]["visible"]
		else:
			node.visible = false
			if now - int(entry["last_seen"]) > RETAIN_MSEC:
				node.queue_free()
				_resident.erase(key)
	priority.sort_custom(func(a: Dictionary, b: Dictionary) -> bool:
		return float(a["distance"]) < float(b["distance"]))
	for item: Dictionary in priority:
		_queue.append(item["key"])


func _choose_lod(distance: float, force_detail: bool, key: int) -> int:
	# First-person and chase views never discard distant dressing detail.
	if force_detail:
		return 0
	var old := int(_resident[key]["lod"]) if _resident.has(key) else -1
	if old == 0 and distance < NEAR_DISTANCE + 15.0:
		return 0
	if old == 2 and distance > MID_DISTANCE - 15.0:
		return 2
	if distance < NEAR_DISTANCE - (10.0 if old == 1 else 0.0):
		return 0
	if distance < MID_DISTANCE + (15.0 if old == 1 else 0.0):
		return 1
	return 2


static func _intersects_frustum(bounds: AABB, planes: Array[Plane], margin: float) -> bool:
	var center := bounds.get_center()
	var extent := bounds.size * 0.5
	for plane: Plane in planes:
		var radius := absf(plane.normal.x) * extent.x + absf(plane.normal.y) * extent.y \
			+ absf(plane.normal.z) * extent.z
		if plane.distance_to(center) > radius + margin:
			return false
	return true


static func _cell_hash(x: int, y: int) -> int:
	var value := ((x + 1) * 73856093) ^ ((y + 1) * 19349663)
	return ((value ^ (value >> 13)) * 1274126177) & 0x7fffffff


func _height_at(x: float, y: float) -> float:
	if _terrain != null:
		return float(_terrain.height_at_world(x, y))
	return float(_surface.height_at_world(x, y))


func _ground_key(tile: Dictionary) -> String:
	if _terrain == null:
		return "1"
	var first: Vector2i = tile["start"]
	var last: Vector2i = tile["end"]
	var span: int = _terrain.TILE_CELLS
	var result := ""
	# A 32 m prop tile covers four ground tiles on the 0.5 m test fixture.
	# Inspect every covered tile, also for partial tiles and other cell sizes.
	for y in range(int(first.y / span), int((last.y - 1) / span) + 1):
		for x in range(int(first.x / span), int((last.x - 1) / span) + 1):
			result += str(_terrain.stride_at((x * span + 0.5) * _cell, (y * span + 0.5) * _cell))
	return result


func _rectangles(start: Vector2i, end: Vector2i) -> Array[Rect2i]:
	var result: Array[Rect2i] = []
	var width := end.x - start.x
	var used := PackedByteArray()
	used.resize(width * (end.y - start.y))
	var span := maxi(1, int(MERGE_METRES / _cell))
	for y in range(start.y, end.y):
		for x in range(start.x, end.x):
			if used[(y - start.y) * width + x - start.x] or _occupancy[y * _width + x] != 1:
				continue
			var right := x + 1
			while right < mini(end.x, x + span):
				if used[(y - start.y) * width + right - start.x] or _occupancy[y * _width + right] != 1:
					break
				right += 1
			var bottom := y + 1
			while bottom < mini(end.y, y + span):
				var clear := true
				for ix in range(x, right):
					if used[(bottom - start.y) * width + ix - start.x] or _occupancy[bottom * _width + ix] != 1:
						clear = false
						break
				if not clear:
					break
				bottom += 1
			for iy in range(y, bottom):
				for ix in range(x, right):
					used[(iy - start.y) * width + ix - start.x] = 1
			result.append(Rect2i(x, y, right - x, bottom - y))
	return result


static func _landscape_kind(x: float, y: float) -> String:
	var region := _cell_hash(int(floor(x / REGION_METRES)), int(floor(y / REGION_METRES))) % 9
	return "forest" if region < 4 else ("rock" if region < 7 else "ruin")


# Geometry is assembled in Python's z-up coordinates, then converted once on emission.
# Keeping the same eight vertices makes the offline and dashboard silhouettes identical.
static func _shape(vertices: PackedVector3Array, colors: PackedColorArray,
		origin: Vector3, center: Vector3, size: Vector3, color: Color, bounds: Rect2,
		yaw: float = 0.0, taper: float = 1.0, roll: float = 0.0, lean: float = 0.0,
		turn: bool = false) -> void:
	var prism := PackedVector3Array()
	var corners := [Vector3(-.5, -.5, -.5), Vector3(.5, -.5, -.5),
		Vector3(.5, .5, -.5), Vector3(-.5, .5, -.5), Vector3(-.5, -.5, .5),
		Vector3(.5, -.5, .5), Vector3(.5, .5, .5), Vector3(-.5, .5, .5)]
	for i in range(8):
		var p: Vector3 = corners[i] * size
		if i >= 4:
			p.x = p.x * taper + lean * size.x
			p.y *= taper
		var py := p.y * cos(roll) - p.z * sin(roll)
		p.z = p.y * sin(roll) + p.z * cos(roll)
		p.y = py
		var px := p.x * cos(yaw) - p.y * sin(yaw)
		p.y = p.x * sin(yaw) + p.y * cos(yaw)
		p.x = px
		p += center
		if turn:
			p.x = 2.0 * bounds.get_center().x - p.x
			p.y = 2.0 * bounds.get_center().y - p.y
		p.x = clampf(p.x, bounds.position.x, bounds.end.x)
		p.y = clampf(p.y, bounds.position.y, bounds.end.y)
		prism.append(Vector3(p.x, p.z, p.y) - origin)
	_append_prism(vertices, colors, prism, color)


static func _landmark(vertices: PackedVector3Array, colors: PackedColorArray,
		origin: Vector3, cx: float, cy: float, width: float, depth: float, base: float,
		hashed: int, biome: String, lod: int) -> int:
	var before := vertices.size()
	var bounds := Rect2(cx-width*.48, cy-depth*.48, width*.96, depth*.96)
	var turn := hashed % 2 != 0 and hashed % 3 != 0
	# The local closure mirrors Python's add helper; all dimensions remain z-up here.
	var add := func(x: float, y: float, z: float, size: Vector3, color: Color,
			yaw: float = 0.0, taper: float = 1.0, roll: float = 0.0, lean: float = 0.0) -> void:
		_shape(vertices, colors, origin, Vector3(cx+x, cy+y, base+z), size, color,
			bounds, yaw, taper, roll, lean, turn)
	if biome == "forest" and minf(width, depth) >= 1.8:
		var trees := PackedVector2Array([Vector2(-.22, -.16), Vector2(.23, .22)]) \
			if minf(width, depth) >= 4.5 else PackedVector2Array([Vector2.ZERO])
		for i in range(trees.size()):
			var radius := minf(minf(width, depth) * (.22 if trees.size() == 2 else .40), 1.65)
			var tall := 3.2 + ((hashed >> (i*3)) % 7)*.32
			var x := trees[i].x*width
			var y := trees[i].y*depth
			add.call(x, y, tall*.37, Vector3(.20, .20, tall*.74), WOOD_COLOR)
			for tier in range(2 if lod == 2 else 3):
				var z := tall*(.40+tier*.20)
				var span := radius*(1.0-tier*.22)
				var tint := Color(LEAF_COLOR.r*(1+tier*.07), LEAF_COLOR.g*(1+tier*.07),
					LEAF_COLOR.b*(1+tier*.07))
				add.call(x, y, z, Vector3(2*span, 2*span, tall*.54), tint,
					(hashed % 11)*.13+i, .025, 0.0, .06)
		if lod == 0 and width >= 4 and depth >= 3:
			add.call(0, -depth*.30, .22, Vector3(width*.67, .26, .27), WOOD_COLOR, .13)
			add.call(-width*.21, -depth*.28, .42, Vector3(.62, .50, .65), WOOD_COLOR,
				0.0, .62, 0.0, .1)
	elif biome == "ruin" and minf(width, depth) >= 3:
		var sx := minf(width*.76, 6.0)
		var sy := minf(depth*.76, 5.0)
		var tall := 1.5+(hashed % 5)*.18
		if hashed % 3 == 0:
			tall *= .58
		var roof_color := ROOF_COLOR if hashed % 3 != 0 else Color(.29, .31, .29)
		add.call(0, sy*.40, tall*.46, Vector3(sx*.92, .25, tall*.92), SLAB_COLOR, 0.0, .94)
		add.call(-sx*.43, sy*.05, tall*.38, Vector3(.26, sy*.75, tall*.76), SLAB_COLOR)
		add.call(sx*.43, sy*.12, tall*.27, Vector3(.26, sy*.62, tall*.54), SLAB_COLOR)
		add.call(-sx*.02, sy*.20, tall+sy*.14, Vector3(sx, sy*.58, .14), roof_color,
			0.0, 1.0, -.48)
		add.call(sx*.06, -sy*.18, tall*.51, Vector3(sx*.84, sy*.53, .17), roof_color,
			.09, 1.0, .34)
		if lod < 2:
			add.call(-sx*.33, -sy*.31, tall*.52, Vector3(.29, .34, tall), SLAB_COLOR)
			add.call(sx*.27, -sy*.31, tall*.25, Vector3(.33, .37, tall*.48), SLAB_COLOR)
			add.call(sx*.21, sy*.25, tall+.50, Vector3(.42, .46, 1.0), RUBBLE_COLOR)
		if lod == 0:
			add.call(-sx*.08, -sy*.29, .30, Vector3(sx*.77, .16, .20), WOOD_COLOR, -.14, 1.0, .2)
			for i in range(3):
				add.call((i-1)*sx*.22, -sy*.20+(i%2)*sy*.16, .18+i*.055,
					Vector3(.52, .41, .29), SLAB_COLOR, i*.67, .55, 0.0, .13)
	elif minf(width, depth) >= 1.8:
		add.call(-width*.08, depth*.04, .36, Vector3(width*.75, depth*.76, 1.05),
			ROCK_COLOR, .12, .57, 0.0, .09)
		if lod < 2:
			add.call(width*.25, -depth*.21, .20, Vector3(width*.35, depth*.40, .71),
				WALL_COLOR, -.3, .46, 0.0, -.12)
		if lod == 0 and minf(width, depth) >= 3:
			add.call(-width*.29, -depth*.24, .08, Vector3(width*.30, depth*.27, .43),
				ROCK_COLOR, .4, .40, 0.0, .1)
	return int((vertices.size()-before)/36)


func _build_tile(key: int, lod: int) -> Dictionary:
	var tile: Dictionary = _tiles[key]
	var start: Vector2i = tile["start"]
	var end: Vector2i = tile["end"]
	var origin := Vector3(start.x * _cell, 0.0, start.y * _cell)
	var node := Node3D.new()
	node.name = "Scenery_%d_%d" % [start.x, start.y]
	node.position = origin
	var vertices := PackedVector3Array()
	var colors := PackedColorArray()
	var details := 0
	var rectangles := _rectangles(start, end)
	for rect: Rect2i in rectangles:
		var hashed := _cell_hash(rect.position.x, rect.position.y)
		var corners := PackedVector3Array()
		var x0 := rect.position.x * _cell
		var y0 := rect.position.y * _cell
		var x1 := rect.end.x * _cell
		var y1 := rect.end.y * _cell
		for point: Vector2 in [Vector2(x0, y0), Vector2(x1, y0), Vector2(x1, y1), Vector2(x0, y1)]:
			corners.append(Vector3(point.x, _height_at(point.x, point.y), point.y) - origin)
		var slope_x := (corners[1].y - corners[0].y)/(x1-x0)
		var slope_y := (corners[3].y - corners[0].y)/(y1-y0)
		var cx := (x0+x1)*.5
		var cy := (y0+y1)*.5
		var biome := _landscape_kind(cx, cy)
		if Vector2(slope_x, slope_y).length() > .40:
			biome = "rock"
		var tint := RUBBLE_COLOR if biome == "ruin" else \
			(Color(.31, .35, .25) if biome == "forest" else ROCK_COLOR)
		# Small embedded stones break the former rectangular foundation silhouette.
		var nx := maxi(1, ceili((x1-x0)/3.0))
		var ny := maxi(1, ceili((y1-y0)/3.0))
		var size := Vector2((x1-x0)/nx, (y1-y0)/ny)
		for sy in range(ny):
			for sx in range(nx):
				var stone := _cell_hash(rect.position.x*11+sx, rect.position.y*11+sy)
				var centre := Vector2(x0, y0)+size*Vector2(sx+.5, sy+.5)
				centre += size*Vector2(((stone >> 3) % 5-2)*.025, ((stone >> 6) % 5-2)*.025)
				var offsets := [Vector2(-.5, -.5), Vector2(.5, -.5), Vector2(.5, .5), Vector2(-.5, .5)]
				var prism := PackedVector3Array()
				for offset: Vector2 in offsets:
					var p := centre+offset*size*.78
					prism.append(Vector3(p.x, _height_at(p.x, p.y)-.12, p.y)-origin)
				for i in range(4):
					var p: Vector2 = centre+offsets[i]*size*(.30+((stone >> (i*4)) % 5)*.035)
					prism.append(Vector3(p.x, _height_at(p.x, p.y)+.16+(stone % 5)*.035, p.y)-origin)
				var stone_color := Color(tint.r, tint.g, tint.b)*(.88+(stone % 7)*.025)
				_append_prism(vertices, colors, prism, stone_color)
		var base := _height_at(cx, cy)+.06
		details += _landmark(vertices, colors, origin, cx, cy, x1-x0, y1-y0, base, hashed, biome, lod)
	if lod == 0:
		for y in range(start.y, end.y):
			for x in range(start.x, end.x):
				if _occupancy[y*_width+x] != 2 or _cell_hash(x, y) % 13 != 0:
					continue
				var cx := (x+.5)*_cell
				var cy := (y+.5)*_cell
				_shape(vertices, colors, origin, Vector3(cx, cy, _height_at(cx, cy)+.03),
					Vector3(.56*_cell, .37*_cell, .06), RUBBLE_COLOR,
					Rect2(x*_cell, y*_cell, _cell, _cell))
				details += 1
	if not vertices.is_empty():
		var instance := MeshInstance3D.new()
		instance.mesh = _mesh(vertices, colors)
		instance.material_override = _material
		instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		node.add_child(instance)
	add_child(node)
	return {"node": node, "lod": lod, "ground_key": _ground_key(tile),
		"rectangles": rectangles.size(), "details": details, "triangles": vertices.size()/3}


static func _append_prism(vertices: PackedVector3Array, colors: PackedColorArray,
		corners: PackedVector3Array, color: Color) -> void:
	var light := Vector3(.45, .82, .35).normalized()
	for quad: Array in FACES:
		var normal := (corners[quad[2]]-corners[quad[0]]).cross(corners[quad[1]]-corners[quad[0]]).normalized()
		var shade := .40+.60*maxf(0.0, normal.dot(light))
		for index: int in [quad[0], quad[2], quad[1], quad[0], quad[3], quad[2]]:
			vertices.append(corners[index])
			colors.append(Color(color.r*shade, color.g*shade, color.b*shade))


static func _mesh(vertices: PackedVector3Array, colors: PackedColorArray) -> ArrayMesh:
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_COLOR] = colors
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh


func stats() -> Dictionary:
	var visible_count := 0
	var rectangles := 0
	var details := 0
	var triangles := 0
	for entry: Dictionary in _resident.values():
		if (entry["node"] as Node3D).visible:
			visible_count += 1
			rectangles += int(entry["rectangles"])
			details += int(entry["details"])
			triangles += int(entry["triangles"])
	return {"total_tiles": _tiles.size(), "loaded_tiles": _resident.size(),
		"visible_tiles": visible_count, "queued_tiles": _queue.size(),
		"wall_rectangles": rectangles, "detail_instances": details, "triangles": triangles}
