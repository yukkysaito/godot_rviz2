// Road surface: a flat color, a little brighter around the middle of the ego vehicle. By day it is
// drawn as emission only (it looks unshaded); at night it gets a dark albedo, so that the head
// lights light it up (see MapLayerMesh.lit_at_night).
// Its depth is written as that of the plane `depth_lowering` below it, where the pixel's view ray
// meets it: what lies under a road farther than that (a road under a bridge with its lines and
// borders) is hidden, while what lies on it, also where road polygons overlap at slightly different
// heights, is not.
shader_type spatial;
render_mode blend_mix, cull_back, shadows_disabled, depth_draw_always;

global uniform vec3 ego_center;  // middle of the ego vehicle
// Linear colors (no source_color hint: its default values are not converted from sRGB)
uniform vec3 color = vec3(0.0, 0.00651, 0.01675);  // sRGB (0, 0.0745, 0.137)
uniform vec3 near_color = vec3(0.0031, 0.0196, 0.055);  // around the ego, sRGB (0.04, 0.15, 0.26)
uniform float glow_radius = 5.0;  // [m]
uniform vec3 night_albedo = vec3(0.0042, 0.0050, 0.0070);  // sRGB (0.05, 0.06, 0.08)
uniform bool night = false;
uniform float depth_lowering = 1.5;  // [m]

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

	vec3 ray = normalize(world - CAMERA_POSITION_WORLD);
	vec3 lowered = world + ray * (depth_lowering / max(-ray.y, 0.02));
	vec4 clip = PROJECTION_MATRIX * VIEW_MATRIX * vec4(lowered, 1.0);
	DEPTH = clip.z / clip.w;
}
