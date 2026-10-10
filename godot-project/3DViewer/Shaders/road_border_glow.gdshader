// Soft glow on the ground along the road borders (UV.x: 0..1 across the strip)
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled, shadows_disabled;

uniform vec4 color : source_color = vec4(0.55, 0.68, 1.0, 1.0);
uniform float strength : hint_range(0.0, 1.0) = 0.3;

void fragment() {
	float x = abs(UV.x * 2.0 - 1.0);  // 0 at the border, 1 at the edges of the strip
	float glow = 1.0 - smoothstep(0.0, 1.0, x);
	ALBEDO = color.rgb;
	ALPHA = glow * glow * strength;
}
