// Shared look of the perceived objects (vehicles, pedestrians): one matte color with a light rim
// at the silhouette, so they read as a unified set apart from the detailed ego vehicle.
shader_type spatial;
render_mode cull_back;

uniform vec4 color : source_color = vec4(0.72, 0.77, 0.95, 1.0);
uniform vec4 rim_color : source_color = vec4(0.75, 0.85, 1.0, 1.0);
uniform float rim_strength = 0.8;
uniform float self_light = 0.18;  // keeps the objects readable at night

void fragment() {
	float rim = pow(1.0 - clamp(dot(NORMAL, VIEW), 0.0, 1.0), 3.0);
	ALBEDO = color.rgb;
	ROUGHNESS = 0.7;
	SPECULAR = 0.3;
	EMISSION = rim_color.rgb * rim * rim_strength + color.rgb * self_light;
}
