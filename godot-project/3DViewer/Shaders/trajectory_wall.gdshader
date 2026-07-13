// Hologram-barrier look for the trajectory stop wall.
//
// Channel encoding (written by TrajectoryMesh.gd):
//   COLOR = per-vertex color coming from the C++ Trajectory module (kept).
// Scanlines run on world-space height, so no UVs are required.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_opaque, cull_disabled;

uniform bool high_contrast = false;
uniform vec4 high_contrast_color : source_color = vec4(0.05, 1.0, 0.20, 1.0);

uniform float emission_energy : hint_range(0.0, 8.0) = 2.0;
uniform float fresnel_power : hint_range(0.5, 8.0) = 2.5;

// Upward-scrolling scanlines (world-space meters).
uniform float scanline_spacing : hint_range(0.05, 2.0) = 0.35;
uniform float scanline_speed : hint_range(0.0, 5.0) = 0.7; // m/s upward
uniform float scanline_strength : hint_range(0.0, 1.0) = 0.35;

// Camera distance fade (disabled in high-contrast mode).
uniform float fade_start : hint_range(0.0, 500.0) = 45.0;
uniform float fade_end : hint_range(0.0, 500.0) = 90.0;

varying float v_world_y;

void vertex() {
	v_world_y = (MODEL_MATRIX * vec4(VERTEX, 1.0)).y;
}

void fragment() {
	// Edge-glow fresnel; abs() so back faces (cull_disabled) behave the same.
	float fres = pow(1.0 - clamp(abs(dot(normalize(NORMAL), normalize(VIEW))),
			0.0, 1.0), fresnel_power);

	// Thin horizontal band scrolling upward.
	float line_f = fract((v_world_y - TIME * scanline_speed) / scanline_spacing);
	float tri = abs(line_f - 0.5) * 2.0; // 0 at band center, 1 between bands
	float scan = (1.0 - smoothstep(0.0, 0.3, tri)) * scanline_strength;

	// NOTE: render_mode unshaded discards EMISSION in Godot 4.2 (the unshaded
	// path outputs albedo only), so the HDR glow is written into ALBEDO where
	// values > 1.0 are picked up by the glow pass.
	if (high_contrast) {
		vec3 hc = high_contrast_color.rgb;
		ALBEDO = hc * (1.0 + fres * 0.5 + scan * 0.5);
		ALPHA = 1.0;
	} else {
		vec3 col = COLOR.rgb;
		ALBEDO = col * emission_energy * (0.35 + fres + scan);

		float dist_fade = 1.0 - smoothstep(fade_start, fade_end, length(VERTEX));
		// COLOR.a fades to 0 toward the wall top (set by the C++ module);
		// gate every term by it so scanlines do not float past the fade.
		float alpha = COLOR.a * (0.40 + 0.60 * fres + 0.4 * scan);
		ALPHA = clamp(alpha, 0.0, 1.0) * dist_fade;
	}
}
