// Animated holographic ground ring drawn on a flat unit plane under each
// dynamic object: thin outer circle, soft inner glow and a slow rotating
// dash pattern with a gentle brightness pulse. Cyan by default; a magenta
// variant material is used for unknown objects (DynamicObjectMesh.gd).
// Unshaded additive: HDR albedo values > 1.0 are picked up by the glow pass.
shader_type spatial;
render_mode unshaded, blend_add, depth_draw_never, cull_disabled;

uniform vec3 ring_color : source_color = vec3(0.0, 0.898, 1.0); // cyan #00E5FF
uniform float energy : hint_range(0.0, 8.0) = 2.2;
uniform float ring_radius : hint_range(0.1, 1.0) = 0.88;
uniform float ring_thickness : hint_range(0.005, 0.2) = 0.045;
uniform float dash_radius : hint_range(0.1, 1.0) = 0.7;
uniform float dash_count = 24.0;
uniform float dash_speed = 0.5; // revolutions-ish per second
uniform float pulse_speed = 2.0;

void fragment() {
	vec2 p = UV * 2.0 - 1.0;
	float r = length(p);
	float ang = atan(p.y, p.x);

	// Thin main circle.
	float ring = 1.0 - smoothstep(0.0, ring_thickness, abs(r - ring_radius));
	// Rotating dashed inner circle.
	float dash_mask = step(0.5, fract(ang / 6.2831853 * dash_count + TIME * dash_speed));
	float dash = dash_mask * (1.0 - smoothstep(0.0, ring_thickness, abs(r - dash_radius)));
	// Soft glow filling toward the center.
	float glow = (1.0 - smoothstep(0.2, ring_radius, r)) * 0.15;

	float pulse = 0.85 + 0.15 * sin(TIME * pulse_speed);
	float a = (ring + dash * 0.5 + glow) * pulse;

	ALBEDO = ring_color * energy;
	ALPHA = clamp(a, 0.0, 1.0);
}
