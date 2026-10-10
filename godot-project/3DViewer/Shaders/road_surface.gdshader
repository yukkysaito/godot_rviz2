// Road surface: a flat color, a little brighter around the middle of the ego vehicle. By day it is
// drawn as emission only (it looks unshaded); at night it gets a dark albedo, so that the head
// lights light it up (see MapLayerMesh.lit_at_night).
shader_type spatial;
render_mode blend_mix, cull_back, shadows_disabled;

global uniform vec3 ego_center;  // middle of the ego vehicle
// Linear colors (no source_color hint: its default values are not converted from sRGB)
uniform vec3 color = vec3(0.0, 0.00651, 0.01675);  // sRGB (0, 0.0745, 0.137)
uniform vec3 near_color = vec3(0.0031, 0.0196, 0.055);  // around the ego, sRGB (0.04, 0.15, 0.26)
uniform float glow_radius = 5.0;  // [m]
uniform vec3 night_albedo = vec3(0.0042, 0.0050, 0.0070);  // sRGB (0.05, 0.06, 0.08)
uniform bool night = false;

varying vec3 world;

void vertex() {
	world = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	float d = length(world.xz - ego_center.xz);
	float near = 1.0 - smoothstep(glow_radius * 0.25, glow_radius, d);
	ALBEDO = night ? night_albedo : vec3(0.0);
	EMISSION = mix(color, near_color, near);
	ROUGHNESS = 1.0;
	SPECULAR = night ? 0.5 : 0.0;  // as the default material at night (the head beam shines on it)
	ALPHA = 1.0;  // stays in the transparent pass like before (drawn under the markings)
}
