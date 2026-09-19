class_name ThermalVision
extends Node3D
## Display-only simulation of relative heat. Reference: swarmmind/viz/thermal.py.
## Two shared meshes and one screen pass; no additional viewport or resident process.

const MAX_RANGE := 26.0
var bodies: MultiMeshInstance3D
var covers: MultiMeshInstance3D
var screen: ColorRect
var active := false


static func intensity(x: float, y: float, state: int, buried: bool, time: float) -> float:
	if state >= 3:
		return 0.0
	var ix := floorf(x * 100.0)
	var iy := floorf(y * 100.0)
	var noise := fposmod(ix * 73.0 + iy * 151.0 + ix * iy * 3.0, 997.0) / 997.0
	var base := 0.34 if buried and state < 2 else 0.78
	return base + 0.10 * (noise - 0.5) + 0.018 * sin(time * 1.7 + noise * 6.283185307)


func setup(host: SwarmDashboard) -> void:
	bodies = _instances(host._mesh_of(host.VICTIM_MESH), host.VICTIM_SCALE)
	covers = _instances(Burial.build_mesh(), 1.0)
	# Below the HUD's CanvasLayer (1), after the complete 3D scene. The screen
	# texture never captures labels, contact boxes or clickable controls.
	var layer := CanvasLayer.new()
	layer.layer = 0
	add_child(layer)
	screen = ColorRect.new()
	screen.mouse_filter = Control.MOUSE_FILTER_IGNORE
	screen.material = ShaderMaterial.new()
	screen.material.shader = preload("res://shaders/thermal_screen.gdshader")
	layer.add_child(screen)
	screen.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	screen.hide()
	bodies.hide()
	covers.hide()


func _instances(mesh: Mesh, scale_factor: float) -> MultiMeshInstance3D:
	var node := MultiMeshInstance3D.new()
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.use_custom_data = true
	mm.mesh = mesh
	node.multimesh = mm
	var material := ShaderMaterial.new()
	material.shader = preload("res://shaders/thermal_surface.gdshader")
	material.set_shader_parameter("local_scale", scale_factor)
	node.material_override = material
	node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(node)
	return node


func _in_sensor(host: SwarmDashboard, robot: Array, target: Vector2) -> bool:
	var origin := Vector2(robot[0], robot[1])
	var delta := target - origin
	var forward := Vector2(cos(robot[2]), sin(robot[2]))
	var depth := delta.dot(forward)
	if depth <= 0.0 or absf(delta.dot(Vector2(-forward.y, forward.x))) > depth or delta.length() > MAX_RANGE:
		return false
	var steps := maxi(1, int(ceil(delta.length() / (host.cell * 0.25))))
	for i in range(1, steps + 1):
		var point := origin + delta * (float(i) / steps)
		var x := int(floor(point.x / host.cell))
		var y := int(floor(point.y / host.cell))
		if x < 0 or y < 0 or x >= host.gw or y >= host.gh:
			return false
		if host.occ[y * host.gw + x] == 1:
			return false
	return true


func update_view(host: SwarmDashboard) -> void:
	active = host.thermal_on and host.ready_world and host._on_unit() and host.view_mode != host.View.ORBIT
	if active:
		active = int(host.robots[host.follow][4]) < 2
	screen.visible = active
	bodies.visible = active
	covers.visible = active
	# Each heat surface replaces its normal mesh to avoid coplanar depth fighting.
	if host.mk_body != null:
		host.mk_body.visible = not active and (host.show_victims or host.god_view)
	if host.mk_burial != null:
		host.mk_burial.visible = not active
	if not active:
		return
	screen.material.set_shader_parameter("sim_time", host.sim_time)
	var exposed: Array = []
	var buried: Array = []
	var robot: Array = host.robots[host.follow]
	for row: Array in host.truth_victims:
		if int(row[2]) >= 3 or not _in_sensor(host, robot, Vector2(row[0], row[1])):
			continue
		if Burial.needs_excavation(row):
			buried.append(row)
		else:
			exposed.append(row)
	_fill(host, bodies, exposed, false)
	_fill(host, covers, buried, true)


func _fill(host: SwarmDashboard, node: MultiMeshInstance3D, rows: Array, buried: bool) -> void:
	var mm := node.multimesh
	if mm.instance_count != rows.size():
		mm.instance_count = rows.size()
	for i in range(rows.size()):
		var row: Array = rows[i]
		var pose := host._casualty_pose(row[0], row[1])
		if not buried:
			pose.origin += pose.basis.y * Burial.SURFACE_LIFT
			pose.basis = pose.basis.scaled(Vector3.ONE * host.VICTIM_SCALE)
		mm.set_instance_transform(i, pose)
		var heat := intensity(row[0], row[1], int(row[2]), int(row[3]) == 1, host.sim_time)
		mm.set_instance_custom_data(i, Color(heat, 0, 0, 1))
