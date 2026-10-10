// Predicted path of an object: a soft strip that fades toward its end and with lower confidence.
// UV.x: 0 / 1 at the edges, UV.y: 0 at the object to 1 at the end; COLOR.a: confidence.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled, shadows_disabled;

uniform vec4 color : source_color = vec4(1.0, 0.7, 0.3, 1.0);
uniform float max_alpha : hint_range(0.0, 1.0) = 0.35;

void fragment() {
	float across = abs(UV.x * 2.0 - 1.0);
	float aa = fwidth(across) * 1.5;
	float body = 1.0 - smoothstep(1.0 - aa * 2.0, 1.0, across);
	float fade = 1.0 - UV.y;
	ALBEDO = color.rgb;
	ALPHA = body * fade * fade * COLOR.a * max_alpha;
}
