shader_type spatial;
render_mode unshaded, cull_disabled;

uniform float local_scale = 1.0;
varying float heat;

void vertex() {
	heat = clamp(INSTANCE_CUSTOM.r + 0.035 * sin(VERTEX.x * local_scale * 8.0
		+ VERTEX.z * local_scale * 5.0), 0.0, 0.94);
}

void fragment() {
	// Reserved colour decoded by thermal_screen. Opaque geometry keeps ordinary
	// depth testing: ruins, terrain and robots can all occlude the heat surface.
	ALBEDO = vec3(1.0, heat, 1.0);
}
