// Low curtain of light around the goal on the ground, fading upward like the road borders
// (road_border_curtain.gdshader), so that the goal can be seen from low camera angles too.
// Alpha blended (not additive), so that the screen's glow does not spread it.
// The mesh is an open cylinder centered on its origin.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled, shadows_disabled;

uniform vec3 color = vec3(0.79, 0.0, 0.006);  // linear (set by GoalMarker.gd)
uniform float height = 0.6;  // [m] of the mesh
uniform float strength = 0.55;
uniform float base_line = 0.08;  // brighter line at the bottom, fraction of the height
uniform float visibility = 1.0;  // 0..1 to fade it in and out
uniform bool high_contrast = false;

varying float altitude;  // 0..1 from the bottom to the top

void vertex() {
	altitude = clamp(VERTEX.y / height + 0.5, 0.0, 1.0);
}

void fragment() {
	float fade = pow(1.0 - altitude, 2.0);
	float base = 1.0 - smoothstep(0.0, base_line, altitude);
	ALBEDO = high_contrast ? vec3(1.0, 1.0, 0.0) : color;
	ALPHA = clamp(fade * strength + base * 0.35, 0.0, 1.0) * visibility;
}
