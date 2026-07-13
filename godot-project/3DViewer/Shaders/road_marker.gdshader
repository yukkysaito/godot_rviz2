// Road markers (lane lines / stop lines / pedestrian markings).
// Vertex COLOR carries the marker tint written by RoadMarkerMesh.gd:
// cool-white (#CFE8FF) for lane markings, amber (#FFB300) for stop lines.
// Slight emission so lanes read clearly at night without overpowering bloom.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_opaque;

uniform float emission_energy : hint_range(0.0, 4.0) = 0.9;

// Camera distance fade (matches the previous StandardMaterial3D behavior).
uniform float fade_start : hint_range(0.0, 1000.0) = 120.0;
uniform float fade_end : hint_range(0.0, 1000.0) = 220.0;

void fragment() {
	// NOTE: render_mode unshaded discards EMISSION in Godot 4.2 (the unshaded
	// path outputs albedo only), so the slight glow is folded into ALBEDO
	// (base color + emission term) where values > 1.0 feed the glow pass.
	ALBEDO = COLOR.rgb * (1.0 + emission_energy);

	float dist_fade = 1.0 - smoothstep(fade_start, fade_end, length(VERTEX));
	ALPHA = COLOR.a * dist_fade;
}
