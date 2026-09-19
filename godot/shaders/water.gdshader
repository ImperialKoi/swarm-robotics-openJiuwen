shader_type spatial;
render_mode unshaded, cull_disabled, depth_draw_opaque;

uniform sampler2D fog_tex : filter_nearest, source_color;
uniform sampler2D sector_tex : filter_nearest, source_color;
uniform bool show_sectors = true;
uniform bool god_view = false;
uniform vec2 map_size = vec2(1.0);
varying vec2 world_uv;
varying vec3 world_position;

// Use the land's height-attenuated atmosphere, so low river views share a horizon.
uniform vec3 fog_color = vec3(0.694, 0.725, 0.698);
uniform float fog_density = 0.020;
uniform float fog_scale_height = 14.0;
uniform float fog_max = 0.93;

void vertex() {
	vec3 wp = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	world_position = wp;
	world_uv = wp.xz / map_size;
}

void fragment() {
	float seen = god_view ? 1.0 : texture(fog_tex, world_uv).r;
	// Restrained moving reflections. Geometry never oscillates across a dry ford.
	vec2 p = world_position.xz;
	float phase = dot(p, vec2(0.73, 1.21)) - TIME * 0.65;
	float cross_wave = sin(dot(p, vec2(0.19, -0.47)) + TIME * 0.23);
	float filter_width = fwidth(phase);
	float glint = pow(max(0.0, sin(phase) * cross_wave), 8.0);
	glint *= 1.0 - smoothstep(0.6, 2.0, filter_width);
	float shallow = 1.0 - clamp((COLOR.a - 0.84) / 0.12, 0.0, 1.0);
	float ripple = 0.5 + 0.5 * sin(dot(p, vec2(1.3, 0.8)) - TIME * 0.8);
	vec3 water_color = COLOR.rgb + vec3(glint * 0.10);
	water_color = mix(water_color, vec3(0.67, 0.71, 0.61), shallow * ripple * 0.11);
	vec3 base = water_color * mix(0.26, 1.0, seen);
	if (show_sectors) {
		vec4 sec = texture(sector_tex, world_uv);
		base = mix(base, sec.rgb, sec.a * mix(0.25, 1.0, seen));
	}
	float mid = max(0.5 * (CAMERA_POSITION_WORLD.y + world_position.y), 0.0);
	float density = fog_density * exp(-mid / fog_scale_height);
	float haze = clamp(1.0 - exp(-density * length(VERTEX)), 0.0, fog_max);
	ALBEDO = mix(base, fog_color, haze);
	ALPHA = COLOR.a;
}
