// Neon drivable-area boundary ribbons: emissive cyan-white with a subtle
// +/-20% emission pulse. Vertex COLOR (written by PathMesh.gd) tints the line.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_opaque, cull_disabled;

uniform vec4 line_color : source_color = vec4(0.65, 0.97, 1.0, 0.9);
uniform float emission_energy : hint_range(0.0, 8.0) = 2.5;
uniform float pulse_speed : hint_range(0.0, 10.0) = 2.0; // rad/s
uniform float pulse_amount : hint_range(0.0, 1.0) = 0.2;

// Camera distance fade.
uniform float fade_start : hint_range(0.0, 500.0) = 60.0;
uniform float fade_end : hint_range(0.0, 500.0) = 120.0;

void fragment() {
	vec3 col = line_color.rgb * COLOR.rgb;
	float pulse = 1.0 + pulse_amount * sin(TIME * pulse_speed);
	// NOTE: render_mode unshaded discards EMISSION in Godot 4.2 (the unshaded
	// path outputs albedo only), so the HDR glow is written into ALBEDO where
	// values > 1.0 are picked up by the glow pass.
	ALBEDO = col * emission_energy * pulse;

	float dist_fade = 1.0 - smoothstep(fade_start, fade_end, length(VERTEX));
	ALPHA = line_color.a * COLOR.a * dist_fade;
}
