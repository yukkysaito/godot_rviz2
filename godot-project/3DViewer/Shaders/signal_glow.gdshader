// Soft glow around a lit traffic light lamp, always facing the camera, so that the color of a
// light seen from the side (where the flat lamp is thin or hidden) can still be told. Faint from
// the front, where the lamp itself shows the color, so the lamp does not get blurred.
// Also seen from behind, to tell the state of the lights for the crossing roads.
shader_type spatial;
render_mode unshaded, blend_add, depth_draw_never, cull_disabled, shadows_disabled;

uniform vec3 color = vec3(1.0, 0.04, 0.03);  // linear
uniform float front_strength = 0.12;
uniform float side_strength = 1.6;
uniform float back_strength = 0.8;  // seen from behind the board

varying float strength;

void vertex() {
	vec3 center = MODEL_MATRIX[3].xyz;
	vec3 front = normalize(MODEL_MATRIX[2].xyz);  // +Z: the side the lamp faces
	vec3 to_camera = normalize(CAMERA_POSITION_WORLD - center);
	float facing = dot(front, to_camera);
	strength = facing > 0.0 ? mix(side_strength, front_strength, facing) : mix(side_strength, back_strength, -facing);
	// Billboard, pulled toward the camera by its radius so that the board does not cut it
	vec3 center_view = (VIEW_MATRIX * vec4(center, 1.0)).xyz;
	vec3 v = center_view + vec3(VERTEX.xy, 0.0) + normalize(-center_view) * abs(VERTEX.x);
	POSITION = PROJECTION_MATRIX * vec4(v, 1.0);
}

void fragment() {
	float r = length(UV * 2.0 - 1.0);
	float glow = exp(-r * r * 6.0) * (1.0 - smoothstep(0.8, 1.0, r));
	float core = 1.0 - smoothstep(0.18, 0.26, r);
	ALBEDO = color * strength * (glow * 0.8 + core * 0.6);
}
