// Ground and obstacles share one shader.
//
// Unshaded on purpose: directional shading is baked into vertex colours on the CPU from
// the heightfield gradient, exactly as swarmmind/viz/render3d.py does it. That keeps the
// offline verification frames and the live dashboard looking the same, and removes
// lighting setup as a source of blind bugs.
//
// Fog is computed here rather than through Environment post-processing, because the GL
// Compatibility renderer -- chosen to keep memory free for the simulator and the local
// model -- does not support colour-adjustment post effects reliably.
shader_type spatial;
render_mode unshaded, cull_disabled;

uniform sampler2D fog_tex : filter_nearest, source_color;
// One texel per sector: rgb is the tint, a is how strongly to apply it. Ported from
// Renderer3D.SECTOR_TINT, verified offline in runs/3d/orbit_sectors.png before it got
// here. Weak on purpose -- a wash over the terrain, never a repaint.
uniform sampler2D sector_tex : filter_nearest, source_color;
uniform bool show_sectors = true;
uniform vec2 map_size = vec2(1.0, 1.0);
uniform float fog_dim = 0.26;
uniform bool god_view = false;

// Airborne dust, pooling low the way real haze does. Plain distance fog is wrong here:
// the operator camera sits ~340 m out, so any density thick enough to feel atmospheric
// in first-person erases the whole map from the eagle view. Attenuating density with
// altitude fixes both at once, and an exponential atmosphere is what actually happens.
uniform vec3 fog_color = vec3(0.694, 0.725, 0.698);
uniform float fog_density = 0.020;
uniform float fog_scale_height = 14.0;
uniform float fog_max = 0.93;

varying vec2 world_uv;
varying float frag_height;

void vertex() {
	vec3 wp = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	world_uv = vec2(wp.x / map_size.x, wp.z / map_size.y);
	frag_height = wp.y;
}

void fragment() {
	float seen = god_view ? 1.0 : texture(fog_tex, world_uv).r;
	vec3 base = COLOR.rgb * mix(fog_dim, 1.0, seen);

	if (show_sectors) {
		vec4 sec = texture(sector_tex, world_uv);
		// Only tint ground the swarm has actually seen. Colouring unexplored sectors
		// would let the operator read strategy off territory nobody has been to.
		base = mix(base, sec.rgb, sec.a * mix(0.25, 1.0, seen));
	}

	float dist = length(VERTEX);
	float mid = max(0.5 * (CAMERA_POSITION_WORLD.y + frag_height), 0.0);
	float d = fog_density * exp(-mid / fog_scale_height);
	float f = clamp(1.0 - exp(-d * dist), 0.0, fog_max);

	ALBEDO = mix(base, fog_color, f);
}
