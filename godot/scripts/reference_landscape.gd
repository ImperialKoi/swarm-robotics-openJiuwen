extends RefCounted
## Display-only copy of scenario geometry. Source: assets/scenarios/demo.yaml.
## Export with scripts/design_terrain.py --export-layout; parity is tested.

static func load_layout(scenario: String, map_size: Vector2) -> Dictionary:
	var path := "res://assets/landscapes/%s.json" % scenario.get_file()
	if not FileAccess.file_exists(path):
		return {}
	var value = JSON.parse_string(FileAccess.get_file_as_string(path))
	if not value is Dictionary or not value.has("map_m"):
		return {}
	if Vector2(float(value.map_m[0]), float(value.map_m[1])) != map_size:
		return {}
	return value


static func biome_at(layout: Dictionary, x: float, y: float) -> String:
	for rect: Array in layout.buildings:
		if x >= rect[0] and x <= rect[0]+rect[2] and y >= rect[1] and y <= rect[1]+rect[3]:
			return "ruin"
	for ellipse: Array in layout.woodlands:
		if pow((x-ellipse[0])/ellipse[2], 2) + pow((y-ellipse[1])/ellipse[3], 2) <= 1.0:
			return "forest"
	return "rock"


static func road_distances(layout: Dictionary, width: int, height: int, cell: float) -> PackedFloat32Array:
	var distances := PackedFloat32Array()
	distances.resize((width+1)*(height+1))
	distances.fill(1000000.0)
	var margin := float(layout.road_width_m)*0.5 + 1.0
	# Only visit a segment's bounds; no full-grid search per vertex at runtime.
	for path: Array in layout.roads:
		for k in range(path.size()-1):
			var a := Vector2(float(path[k][0]), float(path[k][1]))
			var b := Vector2(float(path[k+1][0]), float(path[k+1][1]))
			var segment := b-a
			var length2 := segment.length_squared()
			if length2 < 0.000001:
				continue
			for y in range(maxi(0, floori((minf(a.y,b.y)-margin)/cell)), mini(height, ceili((maxf(a.y,b.y)+margin)/cell))+1):
				for x in range(maxi(0, floori((minf(a.x,b.x)-margin)/cell)), mini(width, ceili((maxf(a.x,b.x)+margin)/cell))+1):
					var delta := Vector2(x*cell,y*cell)-a
					var along := clampf(delta.dot(segment)/length2, 0.0, 1.0)
					var i := y*(width+1)+x
					distances[i] = minf(distances[i], (delta-segment*along).length())
	return distances


static func dress_color(color: Color, x: float, y: float, detail: float, wet: float,
		distance: float, layout: Dictionary) -> Color:
	for ellipse: Array in layout.woodlands:
		var weight := clampf((1.0-pow((x-ellipse[0])/ellipse[2],2)-pow((y-ellipse[1])/ellipse[3],2))*3.0,0.0,1.0)
		var grain := detail*0.065
		color = color.lerp(Color(0.19+grain,0.29+grain,0.18+grain), weight*0.85)
	for ellipse: Array in layout.landslides:
		var weight := clampf((1.0-pow((x-ellipse[0])/ellipse[2],2)-pow((y-ellipse[1])/ellipse[3],2))*5.0,0.0,1.0)
		var grain := (detail-0.5)*0.08
		color = color.lerp(Color(0.51+grain,0.48+grain,0.41+grain), weight)
	var road := clampf(float(layout.road_width_m)*0.5+0.6-distance,0.0,1.0)*(1.0-wet)
	var grain := (detail-0.5)*0.04
	return color.lerp(Color(0.66+grain,0.63+grain,0.54+grain),road)
