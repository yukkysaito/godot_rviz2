// Mini map (UI/MiniMap.gd): its viewport's picture over the ground color, with rounded corners.
shader_type canvas_item;

uniform vec3 ground : source_color = vec3(0.035, 0.05, 0.085);
uniform vec2 size = vec2(300.0);  // [px] of the map
uniform float radius = 18.0;  // [px] of the corners

void fragment() {
	vec4 map = texture(TEXTURE, UV);
	// Distance outside the rounded rectangle [px]
	vec2 q = abs((UV - 0.5) * size) - (size * 0.5 - radius);
	float outside = length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - radius;
	float aa = max(fwidth(outside), 0.5);
	COLOR = vec4(mix(ground, map.rgb, map.a), 1.0 - smoothstep(-aa, aa, outside));
}
