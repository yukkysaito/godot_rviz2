// Traffic light lamp drawn procedurally (no texture), so that it stays a crisp circle at any
// distance: a round lens, or an arrow for arrow lamps. Lit: glows in its color; off: a dim tint of it.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_never, cull_back, shadows_disabled;

uniform vec3 color = vec3(1.0, 0.04, 0.03);  // linear
uniform bool lit = false;
uniform int arrow = 0;  // 0: none, 1: left, 2: right, 3: up, 4: down, 5: up_left, 6: up_right
uniform float lit_intensity = 1.6;  // a little above 1 for a slight glow, not a blur
uniform float off_tint = 0.045;

float sd_box(vec2 p, vec2 b) {
	vec2 d = abs(p) - b;
	return length(max(d, 0.0)) + min(max(d.x, d.y), 0.0);
}

float sd_triangle(vec2 p, vec2 a, vec2 b, vec2 c) {
	vec2 e0 = b - a, e1 = c - b, e2 = a - c;
	vec2 v0 = p - a, v1 = p - b, v2 = p - c;
	vec2 pq0 = v0 - e0 * clamp(dot(v0, e0) / dot(e0, e0), 0.0, 1.0);
	vec2 pq1 = v1 - e1 * clamp(dot(v1, e1) / dot(e1, e1), 0.0, 1.0);
	vec2 pq2 = v2 - e2 * clamp(dot(v2, e2) / dot(e2, e2), 0.0, 1.0);
	float s = sign(e0.x * e2.y - e0.y * e2.x);
	vec2 d = min(min(vec2(dot(pq0, pq0), s * (v0.x * e0.y - v0.y * e0.x)),
		vec2(dot(pq1, pq1), s * (v1.x * e1.y - v1.y * e1.x))),
		vec2(dot(pq2, pq2), s * (v2.x * e2.y - v2.y * e2.x)));
	return -sqrt(d.x) * sign(d.y);
}

// Signed distance to an arrow pointing to +x, in a unit lens
float sd_arrow(vec2 p) {
	float shaft = sd_box(p - vec2(-0.25, 0.0), vec2(0.55, 0.17));
	float head = sd_triangle(p, vec2(0.9, 0.0), vec2(0.15, 0.68), vec2(0.15, -0.68));
	return min(shaft, head);
}

void fragment() {
	vec2 p = UV * 2.0 - 1.0;
	p.y = -p.y;
	float r = length(p);
	vec3 c;
	float alpha;
	if (arrow == 0) {
		// Round lens: lit glows in its color, off keeps a dim tint of it; a slightly darker rim
		float aa = max(fwidth(r), 0.002) * 1.2;
		alpha = 1.0 - smoothstep(0.92 - aa, 0.92 + aa, r);
		float shade = 1.0 - 0.3 * smoothstep(0.55, 0.92, r);
		c = (lit ? color * lit_intensity : color * off_tint) * shade;
	} else {
		// Arrow on the board, no lens
		float angle = 0.7854;  // up_right
		if (arrow == 1) {
			angle = 3.14159;
		} else if (arrow == 2) {
			angle = 0.0;
		} else if (arrow == 3) {
			angle = 1.5708;
		} else if (arrow == 4) {
			angle = -1.5708;
		} else if (arrow == 5) {
			angle = 2.3562;
		}
		vec2 q = mat2(vec2(cos(angle), -sin(angle)), vec2(sin(angle), cos(angle))) * p;
		float d = sd_arrow(q);
		float a_aa = max(fwidth(d), 0.002) * 1.2;
		alpha = 1.0 - smoothstep(-a_aa, a_aa, d);
		c = lit ? color * lit_intensity : color * off_tint * 0.6 + vec3(0.012);
	}
	if (alpha <= 0.0) {
		discard;
	}
	ALBEDO = c;
	ALPHA = alpha;
}
