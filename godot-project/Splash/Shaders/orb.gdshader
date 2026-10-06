shader_type canvas_item;

// Loading screen background: a cloud of smoke inside a glass sphere. The smoke flows and
// circulates while loading and a thin arc around the orb shows the progress. On exit the glass
// vanishes and the smoke drifts outward over the whole screen, each part at its own speed, and
// thins out.

uniform float u_time = 0.0;
uniform float u_progress : hint_range(0.0, 1.0) = 0.0;  // load progress
uniform float u_done : hint_range(0.0, 1.0) = 0.0;      // exit animation (0 -> 1)
uniform vec3 u_color_a = vec3(0.28, 0.45, 1.00);        // blue
uniform vec3 u_color_b = vec3(0.62, 0.38, 1.00);        // violet
uniform vec3 u_color_c = vec3(0.30, 0.92, 0.86);        // teal
uniform float u_radius = 0.16;                          // orb radius (fraction of screen height)

const vec2 ORB_OFFSET = vec2(0.0, 0.05);   // orb center above the screen center
const vec3 BACKGROUND = vec3(0.012, 0.013, 0.022);
const float GLASS_FADE = 0.12;             // the glass is gone at this point of the exit
const float DRIFT_DISTANCE = 2.4;          // how far the smoke drifts on exit (orb radii, x ~2)
const float ARC_RADIUS = 1.42;             // progress arc radius (orb radii)
const float ARC_WIDTH = 0.0016;

// --- noise

float hash21(vec2 p) {
	p = fract(p * vec2(123.34, 456.21));
	p += dot(p, p + 45.32);
	return fract(p.x * p.y);
}

vec2 grad(vec2 i) {
	float a = hash21(i) * TAU;
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

mat2 rot2(float a) {
	return mat2(vec2(cos(a), sin(a)), vec2(-sin(a), cos(a)));
}

// --- smoke (positions in orb space: orb radius = 1)

// Soft color blobs drifting through the smoke
vec3 smoke_tint(vec2 q, float t) {
	vec3 palette[4] = { u_color_a, u_color_b, u_color_c, mix(u_color_a, u_color_b, 0.5) };
	vec3 tint = vec3(0.0);
	float weight = 0.0;
	for (int k = 0; k < 4; k++) {
		float fk = float(k);
		vec2 c = 0.5 * vec2(sin(t * (0.21 + 0.05 * fk) + fk * 2.1), cos(t * (0.17 + 0.04 * fk) + fk * 1.7));
		float w = exp(-dot(q - c, q - c) * 3.0);
		tint += palette[k] * w;
		weight += w;
	}
	return tint / (weight + 0.25);
}

// Domain-warped smoke value; the smoke slowly circulates and churns
float smoke_value(vec2 q, float t, float turbulence, out vec2 warp) {
	vec2 sq = rot2(t * 0.15) * q;
	warp = vec2(fbm(sq * 2.2 + vec2(0.0, t * 0.3)), fbm(sq * 2.2 + vec2(5.2, 1.3) - t * 0.25));
	return fbm(sq * 2.4 + warp * 2.6 * turbulence + vec2(t * 0.12, -t * 0.08));
}

// Cloud of smoke at orb-space position q (alpha in .a). d: eased exit progress.
// On exit the cloud spreads and thins out; p (screen space) adds texture that does not scale
// with the spread, so it reads as diffusion rather than zoom.
vec4 smoke_cloud(vec2 q, vec2 p, float t, float d) {
	// gentle swirl of the smoke and its colors, stronger toward the center
	vec2 swirled = rot2(0.7 * sin(t * 0.23) * (1.0 - clamp(length(q), 0.0, 1.0))) * q;

	vec2 warp;
	float value = smoke_value(swirled, t, 1.0 + 0.8 * d, warp);
	// puffier, low-frequency smoke once it has spread out (fine detail turns into thin lines)
	float puff = fbm(q * 1.1 + warp * 1.5 + vec2(t * 0.05, -t * 0.04));
	float shape = mix(value, puff, smoothstep(0.0, 0.6, d));
	float detail = fbm(p / u_radius * 0.9 + vec2(t * 0.06, -t * 0.05));

	// soft, ragged boundary that stays inside the glass while loading
	float ragged = length(q) + (fbm(q * 1.4 + vec2(t * 0.1, 0.0)) - 0.5) * 0.4;
	float mask = 1.0 - smoothstep(0.4 + 0.35 * d, 0.95 + 0.3 * d, ragged);
	float density = smoothstep(0.32 - 0.05 * d, 0.8, shape) * mask * mix(1.0, 0.5 + detail, d);

	vec3 color = mix(smoke_tint(swirled, t), vec3(0.9, 0.93, 1.0), 0.15 + 0.35 * smoothstep(0.55, 0.9, shape))
		* (0.8 + 0.6 * exp(-dot(q, q) * 1.5));
	return vec4(color, density * 0.9 * (1.0 - d));
}

// --- glass sphere

// Glass in front of the smoke: faint tint, luminous rim, soft highlight and a halo just outside.
// r: distance from the orb center, edge: silhouette radius (screen space), q: orb space.
vec3 glass(vec2 q, float r, float edge) {
	float inside = 1.0 - smoothstep(edge - 0.006, edge + 0.004, r);
	float rim = pow(clamp(r / max(edge, 1e-4), 0.0, 1.0), 6.0);
	vec3 tint = mix(u_color_a, u_color_b, 0.5 + 0.3 * q.x);

	vec2 hl = q - vec2(-0.38, 0.42);
	float highlight = exp(-dot(hl, hl) * 9.0) * 0.22 + exp(-dot(hl, hl) * 60.0) * 0.18;
	float halo = exp(-max(r - edge, 0.0) * 14.0) * smoothstep(edge * 0.95, edge, r);

	return (tint * (0.04 + 0.55 * rim) + vec3(0.85, 0.9, 1.0) * highlight) * inside + tint * halo * 0.18;
}

// --- progress arc

// Thin arc around the orb, filling clockwise from the top
vec3 progress_arc(vec2 p, float r) {
	float ring = 1.0 - smoothstep(ARC_WIDTH * 0.5, ARC_WIDTH * 1.5, abs(r - u_radius * ARC_RADIUS));
	float a = fract(atan(p.x, p.y) / TAU);  // 0 at the top, clockwise
	float filled = 1.0 - smoothstep(u_progress - 0.002, u_progress, a);
	float head = exp(-abs(a - u_progress) * 140.0) * step(0.001, u_progress) * (1.0 - step(0.999, u_progress));
	return mix(u_color_a, u_color_c, a) * ring * (0.07 + 0.75 * filled + head * 0.8);
}

void fragment() {
	float aspect = SCREEN_PIXEL_SIZE.y / SCREEN_PIXEL_SIZE.x;
	vec2 p = vec2((UV.x - 0.5) * aspect, 0.5 - UV.y) - ORB_OFFSET;
	float t = u_time;
	float r = length(p);

	float d = 1.0 - (1.0 - u_done) * (1.0 - u_done);   // exit, eased out: drifts and slowly settles
	float glass_alpha = 1.0 - smoothstep(0.0, GLASS_FADE, u_done);

	float radius = u_radius * (1.0 + 0.015 * sin(t * 1.1));  // breathing
	vec2 q = p / radius;
	float rq = length(q);
	vec2 dir = q / max(rq, 1e-4);

	// background with a faint tint and a soft glow around the orb
	vec3 col = BACKGROUND + mix(u_color_a, u_color_b, 0.5) * 0.05 * exp(-r * 3.0);
	col += mix(u_color_a, u_color_b, 0.4 + 0.3 * sin(t * 0.3)) * exp(-rq * rq * 1.2) * 0.12 * glass_alpha;

	// smoke: on exit every part drifts outward at its own speed (no uniform scaling)
	float drift = d * (0.5 + 2.0 * fbm(q * 0.9 + 7.3)) * DRIFT_DISTANCE * smoothstep(0.0, 1.0, rq);
	vec4 smoke = smoke_cloud(q - dir * drift, p, t, d);
	col = mix(col, smoke.rgb, smoke.a);

	// glass sphere (silhouette wobbles slightly) and progress arc
	float edge = radius * (1.0 + (fbm(dir * 1.3 + t * 0.15) - 0.5) * 0.04);
	col += (glass(q, r, edge) + progress_arc(p, r)) * glass_alpha;

	// vignette + fine grain
	vec2 v = UV - 0.5;
	col *= 1.0 - dot(v, v) * 0.9;
	col += (hash21(FRAGCOORD.xy + fract(t) * 91.0) - 0.5) * 0.012;

	COLOR = vec4(col, 1.0);
}
