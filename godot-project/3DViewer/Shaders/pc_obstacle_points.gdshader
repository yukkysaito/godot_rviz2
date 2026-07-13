// Obstacle segmentation point cloud: amber-hot round points with a white
// core and a shader-side breathing pulse (replaces the old CPU transparency
// blink). Pulse timing uniforms are fed by ObstacleSegmentationMesh.gd from
// the scene-tuned exports (transparency_speed / cycle_time).
// Unshaded additive: HDR albedo values > 1.0 are picked up by the glow pass.
shader_type spatial;
render_mode unshaded, blend_add, depth_draw_never, cull_disabled;

uniform vec3 base_color : source_color = vec3(1.0, 0.702, 0.0); // amber #FFB300
uniform vec3 core_color : source_color = vec3(1.0, 0.96, 0.82); // hot white core
uniform float energy : hint_range(0.0, 8.0) = 2.4;
uniform float base_alpha : hint_range(0.0, 1.0) = 0.9;
uniform float pulse_speed = 3.0; // set by script (was transparency_speed)
uniform float cycle_time = 3.0; // set by script (seconds per breath cycle)
uniform float pulse_depth : hint_range(0.0, 1.0) = 1.0; // set by script (was transparency_scale)
uniform float pulse_floor : hint_range(0.0, 1.0) = 0.35; // never dim below this (keep readable)
uniform float point_scale = 420.0;
uniform float min_point_size = 2.0;
uniform float max_point_size = 7.0;

void vertex() {
	float view_dist = length((MODELVIEW_MATRIX * vec4(VERTEX, 1.0)).xyz);
	POINT_SIZE = clamp(point_scale / max(view_dist, 1.0), min_point_size, max_point_size);
}

void fragment() {
	// Round point sprite with a hot core.
	vec2 pc = POINT_COORD - vec2(0.5);
	float r = length(pc) * 2.0;
	float mask = 1.0 - smoothstep(0.55, 1.0, r);
	float core = 1.0 - smoothstep(0.0, 0.5, r);

	// One breath per cycle (cos ramp), then hold at full brightness —
	// mirrors the original CPU blink but in emission strength.
	float t = mod(TIME, max(cycle_time, 0.001));
	float wave = 1.0;
	if (t * pulse_speed < 6.2831853) {
		wave = 0.5 - 0.5 * cos(t * pulse_speed);
	}
	float breath = mix(1.0, max(wave, pulse_floor), pulse_depth);

	vec3 col = mix(base_color, core_color, core);
	ALBEDO = col * energy * breath;
	ALPHA = base_alpha * mask * mix(0.75, 1.0, breath);
}
