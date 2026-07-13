// Holographic bounding box for dynamic objects (triangle mode + unknown
// object surface): fresnel edge glow, translucent faces and an upward
// scrolling scanline. Two ShaderMaterials share this shader — cyan for known
// classes, magenta for the unknown-object surface (set via uniforms in
// DynamicObjectMesh.gd).
// Unshaded: HDR albedo values > 1.0 are picked up by the glow pass.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled;

uniform vec3 edge_color : source_color = vec3(0.0, 0.898, 1.0); // cyan #00E5FF
uniform vec3 face_color : source_color = vec3(0.0, 0.24, 0.35);
uniform float face_alpha : hint_range(0.0, 1.0) = 0.15;
uniform float edge_energy : hint_range(0.0, 8.0) = 2.0;
uniform float fresnel_power : hint_range(0.5, 8.0) = 2.5;
uniform float scan_speed = 1.2; // upward scroll [m/s-ish]
uniform float scan_density = 2.5; // scanlines per meter
uniform float scan_strength : hint_range(0.0, 2.0) = 0.5;

varying vec3 v_world_pos;
varying vec3 v_world_normal;

void vertex() {
	v_world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	v_world_normal = (MODEL_MATRIX * vec4(NORMAL, 0.0)).xyz;
}

void fragment() {
	vec3 view_dir = normalize(CAMERA_POSITION_WORLD - v_world_pos);
	float facing = clamp(abs(dot(normalize(v_world_normal), view_dir)), 0.0, 1.0);
	float fresnel = pow(1.0 - facing, fresnel_power);

	// Upward-scrolling scanline in world space.
	float scan = smoothstep(0.8, 1.0, fract(v_world_pos.y * scan_density - TIME * scan_speed));

	vec3 col = face_color + edge_color * (fresnel * edge_energy + scan * scan_strength);
	ALBEDO = col;
	ALPHA = clamp(face_alpha + fresnel * 0.55 + scan * 0.2, 0.0, 1.0);
}
