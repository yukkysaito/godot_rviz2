// Orb, the voice copilot: a small glass sphere of smoke (as on the splash screen). It is calm
// while idle, churns and glows while listening (with the voice level) and swirls while thinking.
shader_type canvas_item;

uniform float activity : hint_range(0.0, 1.0) = 0.0;  // 0 idle, 1 listening / thinking
uniform float level : hint_range(0.0, 1.0) = 0.0;     // voice level while listening
uniform float spin = 0.0;                              // swirl while thinking
uniform vec3 color_a = vec3(0.28, 0.45, 1.0);
uniform vec3 color_b = vec3(0.62, 0.38, 1.0);
uniform vec3 color_c = vec3(0.3, 0.92, 0.86);

float hash(vec2 i) {
	uvec2 u = uvec2(ivec2(i) + 4096);
	uint h = u.x * 0x8da6b343u ^ u.y * 0xd8163841u;
	h ^= h >> 16u; h *= 0x7feb352du; h ^= h >> 15u;
	return float(h) * (1.0 / 4294967296.0);
}

float noise(vec2 p) {
	vec2 i = floor(p), f = fract(p);
	vec2 u = f * f * (3.0 - 2.0 * f);
	return mix(mix(hash(i), hash(i + vec2(1, 0)), u.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), u.x), u.y);
}

float fbm(vec2 p) {
	float v = 0.0, a = 0.5;
	for (int i = 0; i < 4; i++) {
		v += a * noise(p);
		p = mat2(vec2(0.8, 0.6), vec2(-0.6, 0.8)) * p * 2.03;
		a *= 0.5;
	}
	return v;
}

void fragment() {
	vec2 q = (UV - 0.5) * 2.0;
	float r = length(q);
	float t = TIME * (0.4 + 1.2 * activity);
	float radius = 0.62 * (1.0 + 0.04 * sin(TIME * 1.3) + 0.12 * level);
	vec2 sq = mat2(vec2(cos(spin), sin(spin)), vec2(-sin(spin), cos(spin))) * (q / radius);
	vec2 warp = vec2(fbm(sq * 2.0 + vec2(0.0, t * 0.5)), fbm(sq * 2.0 + vec2(5.2, -t * 0.4)));
	float smoke = smoothstep(0.35, 0.85, fbm(sq * 2.4 + warp * 2.2));
	vec3 tint = mix(mix(color_a, color_b, warp.x), color_c, warp.y * 0.6);
	float inside = 1.0 - smoothstep(radius - fwidth(r) * 1.5, radius, r);
	vec3 c = tint * (0.25 + 0.9 * smoke) * (0.6 + 0.6 * activity);
	float rim = pow(clamp(r / radius, 0.0, 1.0), 5.0) * inside;
	c += mix(color_a, color_b, 0.5) * rim * (0.5 + 0.6 * activity);
	float halo = exp(-max(r - radius, 0.0) * (14.0 - 6.0 * activity)) * (1.0 - inside);
	vec3 col = c * inside + mix(color_a, color_b, 0.4) * halo * (0.25 + 0.6 * activity + 0.6 * level);
	COLOR = vec4(col, max(inside * 0.92, halo * (0.35 + 0.6 * activity + 0.5 * level)));
}
