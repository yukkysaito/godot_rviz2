// Traffic light board: a rounded rectangle drawn procedurally on one quad, so that its corners
// stay smooth at any distance. Size per instance (set by TrafficLightBoard). It is behind the
// lamps (TrafficLightGroupsManager.z_offset_board).
shader_type spatial;
render_mode unshaded, cull_disabled, shadows_disabled;

uniform vec3 color = vec3(0.246, 0.246, 0.246);  // linear, sRGB 0.533
uniform float corner_ratio = 0.3;  // corner radius / the shorter side
uniform float rim = 0.015;  // darker rim [m]
instance uniform vec2 size = vec2(1.0, 0.5);  // [m]

void fragment() {
	vec2 p = (UV - 0.5) * size;
	float corner_radius = min(size.x, size.y) * corner_ratio;
	vec2 q = abs(p) - size * 0.5 + corner_radius;
	float d = length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - corner_radius;  // < 0 inside
	float aa = max(fwidth(d), 0.0005) * 1.2;
	float inside = 1.0 - smoothstep(-aa, aa, d);
	float edge = smoothstep(-rim - aa, -rim + aa, d);
	ALBEDO = mix(color, color * 0.6, edge);
	// Opaque (it hides what is behind it), with smooth corners by alpha to coverage
	ALPHA = inside;
	ALPHA_SCISSOR_THRESHOLD = 0.5;
	ALPHA_ANTIALIASING_EDGE = 0.3;
}
