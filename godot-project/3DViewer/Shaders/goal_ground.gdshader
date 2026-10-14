// Ring on the ground at the foot of the goal's curtain (goal_curtain.gdshader), with a faint fill.
// The mesh is a square plane of `size` centered on the goal.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled, shadows_disabled;

uniform vec3 color = vec3(0.79, 0.0, 0.006);  // linear (set by GoalMarker.gd)
uniform float size = 2.0;  // [m] of the mesh
uniform float radius = 0.8;  // [m] of the ring
uniform float ring_width = 0.08;  // [m]
uniform float visibility = 1.0;  // 0..1 to fade it in and out
uniform bool high_contrast = false;

void fragment() {
	float r = length((UV - 0.5) * size);
	float aa = max(fwidth(r), 0.002) * 1.5;
	float ring = 1.0 - smoothstep(ring_width * 0.5 - aa, ring_width * 0.5 + aa, abs(r - radius));
	float fill = 0.18 * (1.0 - smoothstep(radius - aa, radius + aa, r));
	ALBEDO = high_contrast ? vec3(1.0, 1.0, 0.0) : color;
	ALPHA = max(ring * 0.9, fill) * visibility;
}
