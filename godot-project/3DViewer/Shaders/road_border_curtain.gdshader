// Road border as a curtain of light that fades upward (UV.y: 0 at the bottom, 1 at the top).
// Seen edge-on (e.g. from above) a curtain would be a hard thin line, so it fades out there and
// the soft glow on the ground (road_border_glow.gdshader) shows the border instead.
// Alpha blended (not additive), so that the screen's glow does not spread it.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled, shadows_disabled;

uniform vec4 color : source_color = vec4(0.55, 0.68, 1.0, 1.0);
uniform float strength : hint_range(0.0, 1.0) = 0.45;
uniform float base_line = 0.06;  // brighter line at the bottom, fraction of the height
uniform float edge_on_fade = 0.35;  // |cos| of the view angle below which the curtain fades out

void fragment() {
	float fade = pow(1.0 - UV.y, 2.0);
	float base = 1.0 - smoothstep(0.0, base_line, UV.y);
	float facing = smoothstep(0.0, edge_on_fade, abs(dot(NORMAL, VIEW)));
	ALBEDO = color.rgb;
	ALPHA = clamp(fade * strength + base * 0.3, 0.0, 1.0) * facing;
}
