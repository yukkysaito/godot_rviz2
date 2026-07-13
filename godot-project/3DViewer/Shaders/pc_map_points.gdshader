// Holographic LiDAR map point cloud.
// - Height-based gradient: deep blue (low) -> cyan -> teal-white (high).
// - Expanding LiDAR pulse ring radiating from the ego vehicle (ego_pos is
//   updated every frame by PointCloudMapMesh.gd).
// - Distance-attenuated point size and far-distance fade.
// Unshaded additive: HDR albedo values > 1.0 are picked up by the glow pass.
shader_type spatial;
render_mode unshaded, blend_add, depth_draw_never, cull_disabled;

uniform vec3 ego_pos = vec3(0.0);
uniform float pulse_speed : hint_range(1.0, 100.0) = 18.0; // ring expansion speed [m/s]
uniform float pulse_wavelength : hint_range(10.0, 200.0) = 60.0; // distance between rings [m]
uniform float pulse_strength : hint_range(0.0, 8.0) = 2.2;
uniform float pulse_sharpness : hint_range(1.0, 16.0) = 7.0;
uniform float base_energy : hint_range(0.0, 4.0) = 1.1;
uniform float base_alpha : hint_range(0.0, 1.0) = 0.38;
uniform float point_scale = 240.0;
uniform float min_point_size = 1.0;
uniform float max_point_size = 4.0;
uniform float fade_start = 180.0;
uniform float fade_end = 420.0;
uniform float height_min = -2.0;
uniform float height_max = 14.0;
uniform vec3 color_low : source_color = vec3(0.02, 0.09, 0.38); // deep blue
uniform vec3 color_mid : source_color = vec3(0.0, 0.898, 1.0); // cyan #00E5FF
uniform vec3 color_high : source_color = vec3(0.72, 1.0, 0.98); // teal-white

varying float v_height;
varying float v_pulse;
varying float v_fade;

void vertex() {
	vec3 world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	v_height = clamp((world_pos.y - height_min) / max(height_max - height_min, 0.001), 0.0, 1.0);

	// Expanding LiDAR pulse: bright wavefront sweeping outward from ego.
	float ego_dist = distance(world_pos.xz, ego_pos.xz);
	float phase = fract((ego_dist - TIME * pulse_speed) / pulse_wavelength);
	v_pulse = pow(1.0 - phase, pulse_sharpness);

	float view_dist = length((MODELVIEW_MATRIX * vec4(VERTEX, 1.0)).xyz);
	POINT_SIZE = clamp(point_scale / max(view_dist, 1.0), min_point_size, max_point_size);
	v_fade = 1.0 - smoothstep(fade_start, fade_end, view_dist);
}

void fragment() {
	vec3 col;
	if (v_height < 0.5) {
		col = mix(color_low, color_mid, v_height * 2.0);
	} else {
		col = mix(color_mid, color_high, (v_height - 0.5) * 2.0);
	}
	float energy = base_energy * (1.0 + v_pulse * pulse_strength);
	ALBEDO = col * energy;
	ALPHA = base_alpha * v_fade * (1.0 + v_pulse * 0.5);
}
