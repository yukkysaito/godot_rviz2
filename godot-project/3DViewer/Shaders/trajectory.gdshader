// Planned trajectory: bright edges, a translucent fill and chevrons flowing forward.
// UV.x: 0 / 1 at the left / right edge, UV.y: distance along the trajectory [m].
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled, shadows_disabled;

uniform vec4 color : source_color = vec4(0.05, 0.3, 1.0, 1.0);
uniform float fill_alpha : hint_range(0.0, 1.0) = 0.22;
uniform float edge_alpha : hint_range(0.0, 1.0) = 0.9;
uniform float edge_width : hint_range(0.0, 0.5) = 0.07;  // fraction of the width
uniform float chevron_alpha : hint_range(0.0, 1.0) = 0.35;
uniform float chevron_spacing = 3.0;  // [m]
uniform float chevron_speed = 2.0;  // [m/s]
uniform float fade_in = 3.0;  // [m] from the start
uniform float length = 100.0;  // [m], set by the script
uniform float fade_out = 15.0;  // [m] before the end

void fragment() {
	float across = abs(UV.x * 2.0 - 1.0);  // 0 at the center, 1 at the edges
	float along = UV.y;
	float aa = fwidth(across) * 1.5;
	float edge = smoothstep(1.0 - edge_width * 2.0 - aa, 1.0 - edge_width * 2.0 + aa, across);
	float phase = fract((along - TIME * chevron_speed + across * 1.2) / chevron_spacing);
	float chevron = smoothstep(0.0, 0.04, phase) * (1.0 - smoothstep(0.1, 0.16, phase));
	float alpha = max(fill_alpha + chevron * chevron_alpha, edge * edge_alpha);
	alpha *= smoothstep(0.0, fade_in, along) * (1.0 - smoothstep(length - fade_out, length, along));
	ALBEDO = color.rgb;
	ALPHA = alpha * color.a;
}
