// Light of a lamp on the ground (Vehicle/LampLight.gd): the irradiance from a point light `height`
// above the middle of the plane, (h^2 / (h^2 + r^2))^1.5, fading out softly toward the other side
// of the vehicle (under the body). Added to the ground, so that it brightens it like light.
shader_type spatial;
render_mode unshaded, blend_add, depth_draw_never, cull_disabled, shadows_disabled;

uniform vec3 color = vec3(1.0, 0.35, 0.013);  // linear (set by LampLight.gd)
uniform float strength = 0.12;
uniform float height = 1.0;  // [m] of the lamp above the ground
uniform float side = 0.0;  // z of the vehicle's side the lamp is on (-1, 1); 0: all around
uniform float brightness = 0.0;  // 0..1, as it fades in and out
uniform bool high_contrast = false;

varying vec2 local;  // [m] from below the lamp

void vertex() {
	local = VERTEX.xz;
}

void fragment() {
	float h2 = height * height;
	float irradiance = pow(h2 / (h2 + dot(local, local)), 1.5);
	float reach = height * 3.0;  // the plane's half size: fade out before its edges
	float edge = 1.0 - smoothstep(reach * 0.6, reach, length(local));
	float outside = side != 0.0 ? smoothstep(-height * 1.5, height * 0.3, side * local.y) : 1.0;
	vec3 light = high_contrast ? vec3(1.0, 1.0, 0.0) : color;
	ALBEDO = light * irradiance * edge * outside * strength * brightness;
}
