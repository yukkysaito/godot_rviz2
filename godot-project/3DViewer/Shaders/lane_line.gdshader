// Lane line: anti-aliased, fainter far away (UV.x: 0 / 1 at the edges of the strip). The strip is
// wider than the line, so that its edges can be smoothed.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled, shadows_disabled;

uniform vec3 color = vec3(0.242, 0.242, 0.242);  // linear, sRGB 0.529 as the other road markings
uniform float core : hint_range(0.0, 1.0) = 0.2;  // line width, fraction of the strip
uniform float fade_start = 40.0;  // [m] from the camera
uniform float fade_end = 160.0;
uniform float far_alpha : hint_range(0.0, 1.0) = 0.35;

void fragment() {
	float x = abs(UV.x * 2.0 - 1.0);
	float aa = max(fwidth(x), 0.001) * 1.2;
	float line = 1.0 - smoothstep(core - aa, core + aa, x);
	float far = mix(1.0, far_alpha, smoothstep(fade_start, fade_end, length(VERTEX)));
	ALBEDO = color;
	ALPHA = line * far;
}
