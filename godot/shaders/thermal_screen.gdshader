shader_type canvas_item;
render_mode unshaded;

uniform sampler2D screen_texture : hint_screen_texture, repeat_disable, filter_nearest;
uniform float sim_time = 0.0;

vec3 palette(float value) {
	vec3 a = vec3(8.0, 12.0, 25.0);
	vec3 b = vec3(39.0, 29.0, 73.0);
	vec3 c = vec3(133.0, 45.0, 79.0);
	vec3 d = vec3(231.0, 100.0, 48.0);
	vec3 e = vec3(255.0, 194.0, 98.0);
	vec3 f = vec3(255.0, 246.0, 216.0);
	float v = clamp(value, 0.0, 1.0) * 5.0;
	if (v < 1.0) { return mix(a, b, v) / 255.0; }
	if (v < 2.0) { return mix(b, c, v - 1.0) / 255.0; }
	if (v < 3.0) { return mix(c, d, v - 2.0) / 255.0; }
	if (v < 4.0) { return mix(d, e, v - 3.0) / 255.0; }
	return mix(e, f, v - 4.0) / 255.0;
}

void fragment() {
	vec3 source = texture(screen_texture, SCREEN_UV).rgb;
	bool heat = source.r > 0.98 && source.b > 0.98 && source.g < 0.95;
	float value = heat ? source.g : 0.045 + 0.18 * dot(source, vec3(0.2126, 0.7152, 0.0722));
	uvec2 pixel = uvec2(floor(FRAGCOORD.xy));
	uint noise = ((pixel.x + 1u) * 73856093u) ^ ((pixel.y + 1u) * 19349663u)
		^ (uint(floor(sim_time * 5.0)) * 83492791u);
	noise = ((noise ^ (noise >> 13u)) * 1274126177u) & 2147483647u;
	float grain = (float(noise) / 2147483647.0 - 0.5) * 0.012;
	COLOR = vec4(palette(value + grain), 1.0);
}
