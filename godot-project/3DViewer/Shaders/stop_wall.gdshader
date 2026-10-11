// Stop wall where the vehicle is planned to stop: red hazard stripes in a frame, fading toward the
// top, with a line on the ground at its foot.
// UV: 0..1 across and from the bottom to the top; UV2.x: 1 on the ground line.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled, shadows_disabled;

uniform vec3 color = vec3(1.0, 0.06, 0.04);  // linear
uniform vec2 size = vec2(2.6, 1.8);  // [m] of the wall
uniform float stripe_period = 0.4;  // [m]
uniform float frame = 0.03;  // [m]
uniform bool high_contrast = false;

void fragment() {
	float alpha = 0.85;  // ground line
	if (UV2.x < 0.5) {
		vec2 p = UV * size;
		float s = fract((p.x + p.y) / stripe_period);
		float aa = fwidth((p.x + p.y) / stripe_period) * 1.5;
		float stripes = smoothstep(0.5 - aa, 0.5 + aa, s) * (1.0 - smoothstep(1.0 - aa, 1.0, s));
		float edge = min(min(p.x, size.x - p.x), size.y - p.y);  // the sides and the top
		float frame_aa = max(fwidth(edge), 0.001);
		float frame_line = 1.0 - smoothstep(frame - frame_aa, frame + frame_aa, edge);
		alpha = max(frame_line * 0.9, stripes * mix(0.55, 0.15, UV.y));
	}
	ALBEDO = high_contrast ? vec3(0.0, 1.0, 0.0) : color;
	ALPHA = alpha;
}
