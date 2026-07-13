// Road surface: very dark navy with extremely subtle large-scale procedural
// value noise (world-space XZ) so it is not dead flat, plus a distance fade
// into the scene background.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_opaque;

uniform vec4 base_color : source_color = vec4(0.0, 0.075, 0.137, 1.0);
uniform float noise_scale : hint_range(0.001, 1.0) = 0.02; // 1/meters
uniform float noise_strength : hint_range(0.0, 1.0) = 0.18;

// Fade to background far from the camera.
uniform float fade_start : hint_range(0.0, 2000.0) = 250.0;
uniform float fade_end : hint_range(0.0, 2000.0) = 450.0;

varying vec2 v_world_xz;

float hash2(vec2 p) {
	return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453123);
}

float value_noise(vec2 p) {
	vec2 i = floor(p);
	vec2 f = fract(p);
	vec2 u = f * f * (3.0 - 2.0 * f);
	return mix(mix(hash2(i), hash2(i + vec2(1.0, 0.0)), u.x),
			mix(hash2(i + vec2(0.0, 1.0)), hash2(i + vec2(1.0, 1.0)), u.x), u.y);
}

void vertex() {
	v_world_xz = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xz;
}

void fragment() {
	// Two octaves of large-scale variation.
	float n = value_noise(v_world_xz * noise_scale);
	n += 0.5 * value_noise(v_world_xz * noise_scale * 3.7);
	n /= 1.5;

	vec3 col = base_color.rgb * (1.0 + (n - 0.5) * 2.0 * noise_strength);
	ALBEDO = col;

	float dist_fade = 1.0 - smoothstep(fade_start, fade_end, length(VERTEX));
	ALPHA = base_color.a * dist_fade;
}
