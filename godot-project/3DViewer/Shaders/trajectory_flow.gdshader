// Holographic trajectory ribbon with animated flow chevrons.
//
// Channel encoding (written by TrajectoryMesh.gd):
//   UV.x    = normalized velocity 0..1 (clamp(v / 5.0, 0, 1))
//   UV.y    = cumulative distance along the path, in meters
//   COLOR.r = across-ribbon coordinate: 0.0 on the left edge, 1.0 on the
//             right edge (used for the soft width fade and chevron shape)
//   COLOR.a = overall alpha multiplier (currently 1.0)
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_opaque, cull_disabled;

uniform bool high_contrast = false;

// Velocity-mapped base ribbon: slow = deeper blue, fast = bright cyan.
uniform vec4 slow_color : source_color = vec4(0.0, 0.18, 0.75, 0.45);
uniform vec4 fast_color : source_color = vec4(0.0, 0.898, 1.0, 0.70);
uniform vec4 chevron_color : source_color = vec4(0.60, 1.0, 1.0, 1.0);
uniform vec4 high_contrast_color : source_color = vec4(0.05, 1.0, 0.20, 1.0);

// Chevron flow animation.
uniform float flow_speed : hint_range(0.0, 20.0) = 4.5; // m/s along the path
uniform float chevron_interval : hint_range(0.5, 20.0) = 3.0; // m between chevrons
uniform float chevron_skew : hint_range(0.0, 5.0) = 1.4; // m of V-shape sweep-back
uniform float chevron_sharpness : hint_range(0.02, 0.5) = 0.12;
uniform float chevron_width : hint_range(0.05, 0.9) = 0.28;

uniform float base_emission_energy : hint_range(0.0, 8.0) = 1.2;
uniform float chevron_emission_energy : hint_range(0.0, 8.0) = 3.0;

// Soft alpha falloff toward the ribbon edges (fraction of half-width).
uniform float edge_fade_start : hint_range(0.0, 0.5) = 0.30;

// Camera distance fade (disabled in high-contrast mode).
uniform float fade_start : hint_range(0.0, 500.0) = 45.0;
uniform float fade_end : hint_range(0.0, 500.0) = 90.0;

void fragment() {
	float velocity = UV.x; // 0..1
	float along = UV.y; // meters
	float across = COLOR.r; // 0..1 over ribbon width
	float half_off = abs(across - 0.5); // 0 center -> 0.5 edge

	// Chevron: a repeating band whose phase is swept back toward the edges,
	// forming a V that points forward and scrolls with time.
	float phase = (along - half_off * 2.0 * chevron_skew - TIME * flow_speed)
			/ chevron_interval;
	float f = fract(phase);
	float band = smoothstep(0.5 - chevron_width * 0.5 - chevron_sharpness,
			0.5 - chevron_width * 0.5, f)
			- smoothstep(0.5 + chevron_width * 0.5,
			0.5 + chevron_width * 0.5 + chevron_sharpness, f);
	band = clamp(band, 0.0, 1.0);

	// NOTE: render_mode unshaded discards EMISSION in Godot 4.2 (the unshaded
	// path outputs albedo only), so the HDR glow is written into ALBEDO where
	// values > 1.0 are picked up by the glow pass.
	if (high_contrast) {
		// Solid high-visibility green, full alpha, no fades.
		vec3 hc = high_contrast_color.rgb;
		ALBEDO = hc * (1.0 + band * 0.8);
		ALPHA = 1.0;
	} else {
		vec4 base = mix(slow_color, fast_color, velocity);
		ALBEDO = base.rgb * base_emission_energy
				+ chevron_color.rgb * band * chevron_emission_energy;

		float edge_fade = 1.0 - smoothstep(edge_fade_start, 0.5, half_off);
		float dist_fade = 1.0 - smoothstep(fade_start, fade_end, length(VERTEX));
		float alpha = max(base.a, band * chevron_color.a);
		ALPHA = clamp(alpha * edge_fade, 0.0, 1.0) * dist_fade * COLOR.a;
	}
}
