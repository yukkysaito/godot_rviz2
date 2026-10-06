// Lit ground around the camera, so that shadows fall on it; it fades out toward its edge into
// the lower half of the sky. Drawn before the map without writing depth, so the map always shows
// on top of it.
shader_type spatial;
render_mode blend_mix, depth_draw_never, cull_back, specular_disabled;

uniform vec4 color : source_color = vec4(0.065, 0.083, 0.137, 1.0);
uniform float self_light = 0.0;  // emission (e.g. at night, when there is no sun)
uniform float fade_start = 300.0;  // [m] from the center
uniform float fade_end = 480.0;

varying float distance_from_center;

void vertex() {
	distance_from_center = length(VERTEX.xz);
}

void fragment() {
	ALBEDO = color.rgb;
	ROUGHNESS = 1.0;
	EMISSION = color.rgb * self_light;
	ALPHA = 1.0 - smoothstep(fade_start, fade_end, distance_from_center);
}
