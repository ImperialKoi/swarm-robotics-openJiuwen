shader_type spatial;
render_mode unshaded, cull_back;

// Rigid-joint port of swarmmind/viz/units.py::joint_angle. Godot's instance
// custom data and palette-index UVs keep all articulated parts in one draw call.
// https://docs.godotengine.org/en/stable/tutorials/shaders/shader_reference/spatial_shader.html
uniform vec3 joint_pivots[32];
uniform vec3 joint_axes[32];
// kind, amplitude, frequency, phase; order is UnitFleet.JOINT_KINDS.
uniform vec4 joint_parameters[32];
uniform vec3 part_palette[16];
uniform float animation_time = 0.0;
uniform float extrapolation = 0.0;
uniform int hidden_instance = -1;

varying vec3 surface_color;
varying float invisible;

vec3 rotate_joint(vec3 point, vec3 axis, float angle) {
	float c = cos(angle);
	float s = sin(angle);
	return point * c + cross(axis, point) * s + axis * dot(axis, point) * (1.0 - c);
}

float joint_angle(vec4 p, float travel, int state) {
	int kind = int(p.x + 0.5);
	if (state == 5) {
		return kind == 5 ? -p.y : 0.0;
	}
	float wave = sin(TAU * p.z * animation_time + p.w);
	if (kind == 1) { return travel / p.y; }
	if (kind == 2) { return p.y * sin(TAU * p.z * travel + p.w); }
	if (kind == 3) { return TAU * p.z * animation_time + p.w; }
	if (kind == 4) { return state == 1 ? p.y * wave : 0.0; }
	if (kind == 5) { return state == 2 ? -p.y * (0.5 + 0.5 * wave) : -p.y; }
	if (kind == 6) { return state == 3 ? p.y : 0.0; }
	if (kind == 7) { return state == 4 ? p.y * wave : 0.0; }
	return 0.0;
}

void vertex() {
	int joint = int(UV.x + 0.5);
	vec4 p = joint_parameters[joint];
	int state = int(INSTANCE_CUSTOM.z + 0.5);
	float travel = INSTANCE_CUSTOM.x + INSTANCE_CUSTOM.y * extrapolation;
	float angle = joint_angle(p, travel, state);
	vec3 pivot = joint_pivots[joint];
	VERTEX = rotate_joint(VERTEX - pivot, joint_axes[joint], angle) + pivot;
	NORMAL = rotate_joint(NORMAL, joint_axes[joint], angle);
	invisible = ((int(p.x + 0.5) == 8 && state != 3) || INSTANCE_ID == hidden_instance) ? 1.0 : 0.0;
	vec3 paint = part_palette[int(UV2.x + 0.5)];
	// Instance RGB recolors only marked parts. Alpha dims the entire body while
	// remaining fully opaque, so loss of comms does not erase its silhouette.
	paint = mix(paint, COLOR.rgb, UV.y);
	if (state == 5) { paint = mix(paint * 0.30, COLOR.rgb, 0.75); }
	vec3 world_normal = normalize(MODEL_NORMAL_MATRIX * NORMAL);
	float diffuse = 0.40 + 0.60 * max(dot(world_normal, normalize(vec3(0.45, 0.82, 0.35))), 0.0);
	surface_color = paint * COLOR.a * diffuse;
}

void fragment() {
	if (invisible > 0.5) { discard; }
	ALBEDO = surface_color;
}
