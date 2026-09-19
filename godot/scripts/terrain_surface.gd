class_name TerrainSurface
extends RefCounted
## Display-only Y-up port of swarmmind/viz/terrain_surface.py.
## Ground mesh and robot supports share a corner lattice and triangle interpolation.
## Frozen wire samples contain obstacle decoration: removing its mean is approximate;
## occupancy-faithful obstacle meshes remain separate and navigation is unchanged.

const LANDSCAPE = preload("reference_landscape.gd")

const WALL_LIFT := 1.55
const RUBBLE_LIFT := 1.0
const SMOOTH_PASSES := 2
const CLEARANCE := 0.045
const WET := 0.02
const FLIGHT_CLEARANCE := 8.0
const FLIGHT_SLOPE := 0.5
const FLIGHT_FOOTPRINT := 2.5
const FLIGHT_MAX_STRIDE := 8

var gw := 0
var gh := 0
var cell := 1.0
var minimum := 0.0
var maximum := 0.0
var corners := PackedFloat32Array()
var water_xy := PackedVector2Array()
var water_corners := PackedFloat32Array()
var water_depths := PackedFloat32Array()
var water := PackedFloat32Array()
var colors := PackedColorArray()
var normals := PackedVector3Array()
var flight_corners := PackedFloat32Array()


func setup(heights: PackedFloat32Array, occupancy: PackedByteArray,
		depths: PackedFloat32Array, width: int, depth: int, cell_size: float,
		reference: Dictionary = {}) -> void:
	gw = width
	gh = depth
	cell = cell_size
	flight_corners.clear()
	water = depths.duplicate()
	assert(gw > 0 and gh > 0 and cell > 0.0)
	assert(heights.size() == gw * gh and occupancy.size() == gw * gh)
	assert(water.size() == gw * gh)
	var recovered := PackedFloat64Array()
	recovered.resize(gw * gh)
	for i in range(gw * gh):
		recovered[i] = heights[i]
		if occupancy[i] == 1:
			recovered[i] -= WALL_LIFT
		elif occupancy[i] == 2:
			recovered[i] -= RUBBLE_LIFT
	var smooth := _region_smooth(recovered, SMOOTH_PASSES)
	var level := recovered.duplicate()
	for i in range(gw * gh):
		level[i] += water[i]
	var water_level := _region_smooth(level, SMOOTH_PASSES)
	var n := (gw + 1) * (gh + 1)
	corners.resize(n)
	water_xy.resize(n)
	water_corners.resize(n)
	water_depths.resize(n)
	colors.resize(n)
	normals.resize(n)
	var rubble := PackedFloat32Array()
	rubble.resize(n)
	var wet_fraction := PackedFloat32Array()
	wet_fraction.resize(n)
	for y in range(gh + 1):
		for x in range(gw + 1):
			var i := y * (gw + 1) + x
			var ground := 0.0
			var top := 0.0
			var wet_depth := 0.0
			var wet_count := 0.0
			var wet_x := 0.0
			var wet_y := 0.0
			var debris := 0.0
			for dy in [-1, 0]:
				for dx in [-1, 0]:
					var cy := clampi(y + dy, 0, gh - 1)
					var cx := clampi(x + dx, 0, gw - 1)
					var j := cy * gw + cx
					ground += smooth[j] * 0.25
					if occupancy[j] == 2:
						debris += 0.25
					if water[j] > WET:
						top += water_level[j]
						wet_depth += water[j]
						wet_x += (cx + 0.5) * cell
						wet_y += (cy + 0.5) * cell
						wet_count += 1.0
			corners[i] = ground
			water_corners[i] = top / maxf(wet_count, 0.000001)
			water_depths[i] = wet_depth / maxf(wet_count, 0.000001)
			rubble[i] = debris
			wet_fraction[i] = wet_count * 0.25
			var wx := x * cell
			var wy := y * cell
			# Shared shore vertices move inward by < a quarter cell. Dry crossings
			# retain their exact wet mask, including at streamed tile boundaries.
			if wet_count > 0.0 and wet_count < 4.0:
				if x > 0 and x < gw:
					wx += (wet_x / wet_count - wx) * 0.44
				if y > 0 and y < gh:
					wy += (wet_y / wet_count - wy) * 0.44
			water_xy[i] = Vector2(wx, wy)
	# Irregular erosion affects steep rock only. Read the unmodified lattice throughout
	# this pass: an in-place gradient would depend on traversal order.
	var eroded := corners.duplicate()
	minimum = INF
	maximum = -INF
	for y in range(gh + 1):
		for x in range(gw + 1):
			var i := y * (gw + 1) + x
			var rocky := smoothstep(0.48, 1.15, _gradient(x, y).length())
			var wx := x * cell
			var wy := y * cell
			var erosion := 1.15 * (_noise(wx + 7.0, wy, 11.0) - 0.5)
			erosion += 0.42 * (_noise(wx, wy + 19.0, 3.7) - 0.5)
			eroded[i] += rocky * (1.0 - wet_fraction[i]) * erosion
			minimum = minf(minimum, eroded[i])
			maximum = maxf(maximum, eroded[i])
	corners = eroded
	# Smoothing uses wet neighbours only; banks cannot turn the water surface
	# into a staircase. Clearance is evaluated at the actual chamfered vertex.
	for i in range(n):
		water_corners[i] = maxf(water_corners[i],
			height_at_world(water_xy[i].x, water_xy[i].y) + 0.025)
	# Short synchronous distance propagation widens sediment without moving water.
	var distance := PackedFloat32Array()
	distance.resize(n)
	for i in range(n):
		distance[i] = 0.0 if wet_fraction[i] > 0.0 else 1000000.0
	for _pass in range(maxi(1, ceili(4.0 / cell))):
		var out := distance.duplicate()
		for y in range(gh + 1):
			for x in range(gw + 1):
				var i := y * (gw + 1) + x
				var near := minf(distance[y * (gw + 1) + maxi(x - 1, 0)],
					distance[y * (gw + 1) + mini(x + 1, gw)])
				near = minf(near, minf(distance[maxi(y - 1, 0) * (gw + 1) + x],
					distance[mini(y + 1, gh) * (gw + 1) + x]))
				out[i] = minf(distance[i], near + cell)
		distance = out
	var road_distances := PackedFloat32Array()
	if not reference.is_empty():
		road_distances = LANDSCAPE.road_distances(reference, gw, gh, cell)
	var light := Vector3(0.45, 0.82, 0.35).normalized()
	for y in range(gh + 1):
		for x in range(gw + 1):
			var i := y * (gw + 1) + x
			var gradient := _gradient(x, y)
			var normal := Vector3(-gradient.x, 1.0, -gradient.y).normalized()
			normals[i] = normal
			var wx := x * cell
			var wy := y * cell
			var broad := _noise(wx + 117.0, wy + 53.0, 43.0)
			var detail := _noise(wx + 31.0, wy + 91.0, 9.0)
			var slope := smoothstep(0.22, 0.90, gradient.length())
			var color := Color(0.28, 0.37, 0.25).lerp(Color(0.49, 0.52, 0.33), broad * 0.8 + detail * 0.2)
			var earth := smoothstep(0.46, 0.80, _noise(wx + 271.0, wy + 9.0, 22.0)) * 0.62
			color = color.lerp(Color(0.49, 0.43, 0.32), earth)
			var strata := 0.5 + 0.5 * sin(corners[i] * 1.9 + _noise(wx, wy, 14.0) * 3.0)
			var grain := strata * 0.09 + detail * 0.10
			color = color.lerp(Color(0.37 + grain, 0.40 + grain, 0.39 + grain), slope)
			color = color.lerp(Color(0.48, 0.41, 0.32), rubble[i] * 0.32)
			var bank := (1.0 - smoothstep(0.3, 4.0, distance[i])) * (1.0 - slope * 0.6)
			var sand_grain := (detail - 0.5) * 0.08
			color = color.lerp(Color(0.59 + sand_grain, 0.55 + sand_grain, 0.43 + sand_grain), bank)
			if not reference.is_empty():
				color = LANDSCAPE.dress_color(color, wx, wy, detail, wet_fraction[i], road_distances[i], reference)
			color *= (1.0 - wet_fraction[i] * 0.22) * (0.40 + 0.60 * clampf(normal.dot(light), 0.0, 1.0))
			color.a = 1.0
			colors[i] = color


func _region_smooth(values: PackedFloat64Array, passes: int) -> PackedFloat64Array:
	# Wet and dry regions filter independently, preserving banks and causeways.
	var result := values.duplicate()
	for _pass in range(passes):
		var out := result.duplicate()
		for y in range(gh):
			for x in range(gw):
				var i := y * gw + x
				var wet := water[i] > WET
				var total := result[i] * 4.0
				var weight := 4.0
				for j in [maxi(y - 1, 0) * gw + x, mini(y + 1, gh - 1) * gw + x,
					y * gw + maxi(x - 1, 0), y * gw + mini(x + 1, gw - 1)]:
					if (water[j] > WET) == wet:
						total += result[j]
						weight += 1.0
				out[i] = total / weight
		result = out
	return result


func _gradient(x: int, y: int) -> Vector2:
	var ax := maxi(x - 1, 0)
	var bx := mini(x + 1, gw)
	var ay := maxi(y - 1, 0)
	var by := mini(y + 1, gh)
	return Vector2((_corner(bx, y) - _corner(ax, y)) / (float(bx - ax) * cell),
		(_corner(x, by) - _corner(x, ay)) / (float(by - ay) * cell))


func _noise_hash(x: int, y: int) -> float:
	var n := ((x + 1) * 73856093) ^ ((y + 1) * 19349663)
	# Mask before multiplication as well to avoid signed overflow on large maps;
	# the low 31 result bits are identical to the Python integer-noise reference.
	return float((((n ^ (n >> 13)) & 0x7fffffff) * 1274126177) & 0x7fffffff) / 2147483647.0


func _noise(x: float, y: float, scale: float) -> float:
	x /= scale
	y /= scale
	var ix := floori(x)
	var iy := floori(y)
	var u := smoothstep(0.0, 1.0, x - ix)
	var v := smoothstep(0.0, 1.0, y - iy)
	return lerpf(lerpf(_noise_hash(ix, iy), _noise_hash(ix + 1, iy), u),
		lerpf(_noise_hash(ix, iy + 1), _noise_hash(ix + 1, iy + 1), u), v)


func _corner(x: int, y: int) -> float:
	return corners[clampi(y, 0, gh) * (gw + 1) + clampi(x, 0, gw)]


func _sample(x: float, y: float, stride: int = 1, flying: bool = false) -> Vector3:
	# Returns (height, dz/dx, dz/dy), choosing the rendered a-c-b / b-c-d diagonal.
	stride = maxi(stride, 1)
	var gx := clampf(x / cell, 0.0, float(gw))
	var gy := clampf(y / cell, 0.0, float(gh))
	var ix := mini(int(floor(gx / stride)) * stride, int(floor(float(gw - 1) / stride)) * stride)
	var iy := mini(int(floor(gy / stride)) * stride, int(floor(float(gh - 1) / stride)) * stride)
	var nx := mini(ix + stride, gw)
	var ny := mini(iy + stride, gh)
	var u := (gx - ix) / float(nx - ix)
	var v := (gy - iy) / float(ny - iy)
	var samples := flight_corners if flying else corners
	var a := samples[iy * (gw + 1) + ix]
	var b := samples[iy * (gw + 1) + nx]
	var c := samples[ny * (gw + 1) + ix]
	var d := samples[ny * (gw + 1) + nx]
	if u + v <= 1.0:
		return Vector3(a + (b - a) * u + (c - a) * v,
			(b - a) / (float(nx - ix) * cell), (c - a) / (float(ny - iy) * cell))
	return Vector3(d + (c - d) * (1.0 - u) + (b - d) * (1.0 - v),
		(d - c) / (float(nx - ix) * cell), (d - b) / (float(ny - iy) * cell))


func height_at_world(x: float, y: float, stride: int = 1) -> float:
	return _sample(x, y, stride).x


func normal_at_world(x: float, y: float, stride: int = 1) -> Vector3:
	var sample := _sample(x, y, stride)
	return Vector3(-sample.y, 1.0, -sample.z).normalized()


func flight_height_at_world(x: float, y: float) -> float:
	# Port of the cached max-plus envelope in terrain_surface.py. Full-map samples
	# make altitude independent of visibility and of terrain tile residency.
	if flight_corners.is_empty():
		flight_corners = corners.duplicate()
		for i in range(flight_corners.size()):
			if water_depths[i] > 0.0:
				flight_corners[i] = maxf(flight_corners[i], water_corners[i])
		var radius := ceili(FLIGHT_FOOTPRINT / cell) + FLIGHT_MAX_STRIDE
		var queue := PackedInt32Array()
		queue.resize(maxi(gw, gh) + 1)
		for axis in [0, 1]:
			var source := flight_corners.duplicate()
			var length := gh + 1 if axis == 0 else gw + 1
			var lines := gw + 1 if axis == 0 else gh + 1
			var step := gw + 1 if axis == 0 else 1
			# A monotone queue makes dilation linear in map size, rather than
			# rescanning a 23-cell window at every vertex on the first takeoff.
			for line in range(lines):
				var base := line if axis == 0 else line * (gw + 1)
				var head := 0
				var tail := 0
				var next := 0
				for at in range(length):
					while next <= mini(at + radius, length - 1):
						var value := source[base + next * step]
						while tail > head and source[base + queue[tail - 1] * step] <= value:
							tail -= 1
						queue[tail] = next
						tail += 1
						next += 1
					while queue[head] < at - radius:
						head += 1
					flight_corners[base + at * step] = source[base + queue[head] * step]
		var fall := FLIGHT_SLOPE * cell
		for cy in range(gh + 1):
			var row := cy * (gw + 1)
			for cx in range(1, gw + 1):
				flight_corners[row + cx] = maxf(flight_corners[row + cx], flight_corners[row + cx - 1] - fall)
			for cx in range(gw - 1, -1, -1):
				flight_corners[row + cx] = maxf(flight_corners[row + cx], flight_corners[row + cx + 1] - fall)
		for cx in range(gw + 1):
			for cy in range(1, gh + 1):
				var i := cy * (gw + 1) + cx
				flight_corners[i] = maxf(flight_corners[i], flight_corners[i - gw - 1] - fall)
			for cy in range(gh - 1, -1, -1):
				var i := cy * (gw + 1) + cx
				flight_corners[i] = maxf(flight_corners[i], flight_corners[i + gw + 1] - fall)
		for i in range(flight_corners.size()):
			flight_corners[i] += FLIGHT_CLEARANCE
	return _sample(x, y, 1, true).x


func robot_pose(x: float, y: float, heading: float, chassis: int = 0, stride: int = 1,
		airborne: bool = false) -> Transform3D:
	if airborne and chassis == 3:
		return Transform3D(Basis(Vector3.UP, -heading), Vector3(x, flight_height_at_world(x, y), y))
	var up := normal_at_world(x, y, stride)
	up /= maxf(up.y, 0.000001)
	var slope := Vector2(up.x, up.z).length()
	if slope > 0.85:
		up.x *= 0.85 / slope
		up.z *= 0.85 / slope
	up = up.normalized()
	var forward := Vector3(cos(heading), 0.0, sin(heading))
	forward = (forward - up * forward.dot(up)).normalized()
	var side := forward.cross(up).normalized()
	var basis := Basis(forward, up, side)
	return Transform3D(basis, Vector3(x, support_height(x, y, basis, chassis, stride), y))


func support_height(x: float, y: float, basis: Basis, chassis: int = 0, stride: int = 1) -> float:
	# Evaluate multiple neighbouring LODs with ONE fixed basis at a streamed tile seam.
	var forward := basis.x
	var up := basis.y
	var side := basis.z
	var rotor := chassis == 3
	var width := 1.05 if rotor else (0.92 if chassis == 2 else 0.73)
	# Piecewise-linear terrain cannot rise above all its vertices: include every
	# lattice vertex in the footprint's bounding cells, catching crests between feet.
	# Its four transformed corners reduce analytically to centre +/- extent. Contact
	# samples inside these cells cannot exceed the vertex bound and are redundant.
	var midpoint := 0.4 if rotor else 0.5
	var half_length := 1.45 if rotor else 1.35
	var centre := Vector2(x + forward.x * midpoint, y + forward.z * midpoint)
	var extent := Vector2(absf(forward.x) * half_length + absf(side.x) * width,
		absf(forward.z) * half_length + absf(side.z) * width)
	var lo := centre - extent
	var hi := centre + extent
	var step := maxi(stride, 1)
	var first := Vector2i(floori(lo.x / (cell * step)) * step, floori(lo.y / (cell * step)) * step)
	var last := Vector2i(ceili(hi.x / (cell * step)) * step, ceili(hi.y / (cell * step)) * step)
	var slope_x := -up.x / up.y
	var slope_y := -up.z / up.y
	var z := -INF
	for cy in range(first.y, last.y + step, step):
		var iy := clampi(cy, 0, gh)
		var row := iy * (gw + 1)
		var plane_y := slope_y * (iy * cell - y)
		for cx in range(first.x, last.x + step, step):
			var ix := clampi(cx, 0, gw)
			var plane := slope_x * (ix * cell - x) + plane_y
			z = maxf(z, corners[row + ix] - plane)
	var clearance := 0.14 if chassis == 2 else CLEARANCE
	return z + clearance


func tile_bounds(x0: int, y0: int, x1: int, y1: int) -> AABB:
	# Includes all LOD vertices, water and skirts. Call once per tile and cache.
	var top := -INF
	var low_x := x0 * cell
	var high_x := x1 * cell
	var low_y := y0 * cell
	var high_y := y1 * cell
	for y in range(y0, y1 + 1):
		for x in range(x0, x1 + 1):
			var i := y * (gw + 1) + x
			top = maxf(top, maxf(corners[i], water_corners[i]))
			# A shared concave shore can move just across a tile boundary.
			low_x = minf(low_x, water_xy[i].x)
			high_x = maxf(high_x, water_xy[i].x)
			low_y = minf(low_y, water_xy[i].y)
			high_y = maxf(high_y, water_xy[i].y)
	return AABB(Vector3(low_x, minimum - 1.0, low_y),
		Vector3(high_x - low_x, top - minimum + 1.0, high_y - low_y))


func _coordinates(start: int, stop: int, stride: int) -> PackedInt32Array:
	var result := PackedInt32Array()
	for value in range(start, stop, maxi(stride, 1)):
		result.append(value)
	result.append(stop)
	return result


func build_mesh(x0: int, y0: int, x1: int, y1: int, stride: int = 1) -> ArrayMesh:
	var xs := _coordinates(x0, x1, stride)
	var ys := _coordinates(y0, y1, stride)
	var vertices := PackedVector3Array()
	var mesh_colors := PackedColorArray()
	var mesh_normals := PackedVector3Array()
	var uv := PackedVector2Array()
	var indices := PackedInt32Array()
	for y in ys:
		for x in xs:
			var i := y * (gw + 1) + x
			vertices.append(Vector3(x * cell, corners[i], y * cell))
			mesh_colors.append(colors[i])
			mesh_normals.append(normals[i])
			uv.append(Vector2(float(x) / gw, float(y) / gh))
	var width := xs.size()
	for y in range(ys.size() - 1):
		for x in range(width - 1):
			var a := y * width + x
			indices.append_array([a, a + width, a + 1, a + 1, a + width, a + width + 1])
	# Full-depth skirts close mixed LOD seams; shared top corners never move on rebuild.
	var perimeter := PackedInt32Array()
	for x in range(width):
		perimeter.append(x)
	for y in range(1, ys.size()):
		perimeter.append(y * width + width - 1)
	for x in range(width - 2, -1, -1):
		perimeter.append((ys.size() - 1) * width + x)
	for y in range(ys.size() - 2, 0, -1):
		perimeter.append(y * width)
	var low := vertices.size()
	for index in perimeter:
		var bottom := vertices[index]
		bottom.y = minimum - 1.0
		vertices.append(bottom)
		var color := mesh_colors[index] * 0.68
		color.a = 1.0
		mesh_colors.append(color)
		mesh_normals.append(mesh_normals[index])
		uv.append(uv[index])
	for i in range(perimeter.size()):
		var next := (i + 1) % perimeter.size()
		indices.append_array([perimeter[i], perimeter[next], low + i,
			perimeter[next], low + next, low + i])
	return _make_mesh(vertices, mesh_colors, mesh_normals, uv, indices)


func build_water_mesh(x0: int, y0: int, x1: int, y1: int) -> ArrayMesh:
	# Preserve the exact wet-cell mask at every LOD: no artificial dams or crossings.
	var vertices := PackedVector3Array()
	var mesh_colors := PackedColorArray()
	var mesh_normals := PackedVector3Array()
	var uv := PackedVector2Array()
	var indices := PackedInt32Array()
	for y in range(y0, y1):
		for x in range(x0, x1):
			if water[y * gw + x] <= WET:
				continue
			var first := vertices.size()
			for offset in [Vector2i(0, 0), Vector2i(1, 0), Vector2i(0, 1), Vector2i(1, 1)]:
				var cx: int = x + offset.x
				var cy: int = y + offset.y
				var i: int = cy * (gw + 1) + cx
				vertices.append(Vector3(water_xy[i].x, water_corners[i], water_xy[i].y))
				var depth := clampf(water_depths[i] / 0.85, 0.0, 1.0)
				var color := Color(0.39, 0.52, 0.47).lerp(Color(0.16, 0.33, 0.35), depth)
				# Animated glints belong to the live shader; Python bakes its still counterpart.
				color.a = 0.84 + 0.12 * depth
				mesh_colors.append(color)
				mesh_normals.append(Vector3.UP)
				uv.append(Vector2(water_xy[i].x / (gw * cell), water_xy[i].y / (gh * cell)))
			indices.append_array([first, first + 2, first + 1, first + 1, first + 2, first + 3])
	return _make_mesh(vertices, mesh_colors, mesh_normals, uv, indices)


func _make_mesh(vertices: PackedVector3Array, mesh_colors: PackedColorArray,
		mesh_normals: PackedVector3Array, uv: PackedVector2Array,
		indices: PackedInt32Array) -> ArrayMesh:
	var mesh := ArrayMesh.new()
	if vertices.is_empty():
		return mesh
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_COLOR] = mesh_colors
	arrays[Mesh.ARRAY_NORMAL] = mesh_normals
	arrays[Mesh.ARRAY_TEX_UV] = uv
	arrays[Mesh.ARRAY_INDEX] = indices
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh
