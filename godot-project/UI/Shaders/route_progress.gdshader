// Route progress (UI/RouteProgress.gd): the travelled part of the route as a band of smoke in the
// colors of the splash screen's orb, with two strands and a ribbon of light between them, flowing
// toward a white-hot head at the vehicle's progress. The smoke and the strands vary with noise
// along the band and in time, so that it does not repeat. Added to the view, like light.
// UV.x: 0..1 along the bar, UV.y: 0..1 across it; the band fills its middle, so that its light
// fades out before the edges.
shader_type canvas_item;
render_mode blend_add;

uniform vec3 color_a = vec3(0.28, 0.45, 1.00);  // blue
uniform vec3 color_b = vec3(0.62, 0.38, 1.00);  // violet
uniform vec3 color_c = vec3(0.30, 0.92, 0.86);  // teal
uniform float progress = 0.0;  // 0..1 along the bar
uniform float length_px = 600.0;  // of the bar

// Noise as the splash screen's smoke (Splash/Shaders/orb.gdshader)
float hash_cell(vec2 i) {
	uint h = uint(int(i.x)) * 0x8da6b343u ^ uint(int(i.y)) * 0xd8163841u;
	h ^= h >> 16u;
	h *= 0x7feb352du;
	h ^= h >> 15u;
	h *= 0x846ca68bu;
	h ^= h >> 16u;
	return float(h) * (1.0 / 4294967296.0);
}

vec2 grad(vec2 i) {
	float a = hash_cell(i) * TAU;
	return vec2(cos(a), sin(a));
}

// Gradient noise in [0, 1] (no grid-aligned blocks like value noise)
float noise(vec2 p) {
	vec2 i = floor(p);
	vec2 f = fract(p);
	vec2 u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
	float a = dot(grad(i), f);
	float b = dot(grad(i + vec2(1.0, 0.0)), f - vec2(1.0, 0.0));
	float c = dot(grad(i + vec2(0.0, 1.0)), f - vec2(0.0, 1.0));
	float d = dot(grad(i + vec2(1.0, 1.0)), f - vec2(1.0, 1.0));
	return clamp(0.5 + 0.85 * mix(mix(a, b, u.x), mix(c, d, u.x), u.y), 0.0, 1.0);
}

float fbm(vec2 p) {
	float v = 0.0;
	float a = 0.5;
	mat2 rot = mat2(vec2(0.8, 0.6), vec2(-0.6, 0.8));
	for (int i = 0; i < 5; i++) {
		v += a * noise(p);
		p = rot * p * 2.03;
		a *= 0.5;
	}
	return v;
}

// Position (y: -1..1 across the band) at `x` [px] of a strand: a curve of noise (so its waves have
// random lengths) flowing at `speed`. `seed` sets the strands apart.
float strand_y(float x, float t, float seed, float speed) {
	float flow = t * speed;
	return (noise(vec2(x / 60.0 - flow * 1.3, seed)) - 0.5) * 1.6
		+ (noise(vec2(x / 21.0 - flow * 2.2, seed + 1.0)) - 0.5) * 0.7;
}

// Light of a strand at `strand` [y], its width and brightness wandering
float strand_light(float x, float y, float t, float strand, float seed, float speed) {
	float flow = t * speed;
	float width = 0.06 + 0.07 * noise(vec2(x / 50.0 - flow * 0.7, seed + 2.0));
	float brightness = 0.3 + 0.9 * noise(vec2(x / 45.0 - flow * 1.1, seed + 3.0));
	return exp(-pow((y - strand) / width, 2.0)) * brightness;
}

void fragment() {
	float x = UV.x * length_px;  // [px] along the bar
	float y = (UV.y * 2.0 - 1.0) * 2.4;  // about -1..1 across the band
	float head_x = progress * length_px;
	float behind = head_x - x;  // [px] behind the head
	float filled = smoothstep(-1.5, 1.5, behind);
	float t = TIME;

	// Smoke flowing toward the head, thicker and wider where `density` is high
	float density = noise(vec2(x / 110.0 - t * 0.35, 7.0));
	float smoke = fbm(vec2(x / 34.0 - t * 1.6, y * 1.1 + t * 0.25));
	smoke = smoothstep(0.6 - 0.35 * density, 0.85, smoke) * exp(-y * y * (2.2 - 1.4 * density));
	// Two independent strands, each a random curve of its own flowing toward the head, and a ribbon
	// of light between them, brighter where they are close (as a twisted ribbon seen edge-on)
	float y1 = strand_y(x, t, 0.0, 1.0);
	float y2 = strand_y(x, t, 40.0, 1.35);
	float strands = strand_light(x, y, t, y1, 0.0, 1.0) + strand_light(x, y, t, y2, 40.0, 1.35);
	float low = min(y1, y2);
	float high = max(y1, y2);
	float between = smoothstep(low - 0.04, low + 0.04, y)
		* (1.0 - smoothstep(high - 0.04, high + 0.04, y));
	float ribbon = between * (0.3 + 0.4 * (1.0 - clamp((high - low) / 1.2, 0.0, 1.0)));
	float core = exp(-y * y / 0.012);

	vec3 tint = mix(color_a, color_b, smoothstep(0.3, 0.7, noise(vec2(x / 160.0 - t * 0.25, 9.0))));
	tint = mix(tint, color_c, smoothstep(0.55, 0.95, smoke) * 0.6);
	// White-hot head, its light trailing back along the band
	float head = exp(-max(behind, 0.0) / 34.0) * exp(-y * y * 2.5);
	float flicker = 0.85 + 0.15 * noise(vec2(t * 9.0, 0.0));

	vec3 light = tint * (smoke * 0.75 + strands * 0.55 + ribbon)
		+ vec3(0.9, 0.95, 1.0) * (core * 0.35 + head * 1.1 * flicker);
	float start = smoothstep(0.0, 24.0, x);  // fades in from the bar's start
	COLOR = vec4(light * filled * start, 1.0);  // added as it is (faded by the node's modulate)
}
