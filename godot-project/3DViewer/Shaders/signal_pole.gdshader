// Traffic light pole and arm, in the board's color and flat like it: darker toward the silhouette,
// as the board's rim, so that the round shape still shows.
shader_type spatial;
render_mode unshaded;

uniform vec3 color = vec3(0.246, 0.246, 0.246);  // linear

void fragment() {
	float facing = abs(dot(NORMAL, VIEW));
	ALBEDO = color * mix(0.6, 1.0, smoothstep(0.0, 0.7, facing));
}
