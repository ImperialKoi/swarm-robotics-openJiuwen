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
	// UV2 follows each channel downstream. Geometry and dry crossings stay fixed.
	float s = UV2.x;
	float t = UV2.y;
	float depth = clamp((COLOR.a - 0.84) / 0.12, 0.0, 1.0);
	float blend = smoothstep(0.0, 1.0, depth);
	vec3 water_color = mix(vec3(0.46, 0.57, 0.48), vec3(0.105, 0.31, 0.32), blend);
	float bend = sin(s * 0.17 - TIME * 0.21) * 0.7 + sin(s * 0.071 + t * 0.43) * 0.4;
	float ribbons = pow(max(0.0, sin(t * 1.9 + bend)), 10.0);
	float pulse = 0.5 + 0.5 * sin(s * 0.64 - TIME * 1.15 + sin(t * 0.8));
	float ripple = pow(max(0.0, sin(s * 1.7 - TIME * 2.1 + bend)), 12.0);
	float detail = 1.0 - smoothstep(0.6, 2.0, max(fwidth(s * 1.7), fwidth(t * 1.9)));
	float glint = ribbons * (0.025 + 0.075 * pulse) + ripple * 0.035;
	float foam = pow(1.0 - blend, 3.0) * ribbons * (0.12 + 0.16 * pulse);
	water_color += vec3(glint * detail);
	water_color = mix(water_color, vec3(0.78, 0.83, 0.73), foam * detail);
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
