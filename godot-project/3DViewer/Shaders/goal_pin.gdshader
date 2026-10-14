// Map pin above the goal: a flat red pin with a see-through hole, fully facing the camera (like
// the object icons) with its tip at the origin. The shape is drawn here to stay sharp at any
// size. Its size is `height` nearby and kept at least `min_angle` tall on screen far away; it is
// drawn over everything so that the goal can be found behind buildings.
// The mesh is a 1 x 1 quad whose bottom edge is at the origin.
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_never, depth_test_disabled, cull_disabled, shadows_disabled;

uniform vec3 color = vec3(0.79, 0.0, 0.006);  // linear (set by GoalMarker.gd)
uniform float height = 1.5;  // [m] of the pin nearby
uniform float min_angle = 0.05;  // [rad] the pin is at least this tall on screen
uniform float visibility = 1.0;  // 0..1 to fade it in and out
uniform bool high_contrast = false;

const float WIDTH = 0.8;  // of the quad, relative to its height
const float MARGIN = 0.02;  // below the tip, for its antialiasing
const vec2 HEAD = vec2(0.0, 0.64);  // center of the round head (the tip is at the origin)
const float HEAD_RADIUS = 0.34;
const float HOLE_RADIUS = 0.15;

void vertex() {
	vec3 anchor = MODEL_MATRIX[3].xyz;
	float size = max(height, length(CAMERA_POSITION_WORLD - anchor) * min_angle);
	VERTEX = vec3(VERTEX.x * WIDTH, VERTEX.y * (1.0 + MARGIN) - MARGIN, 0.0) * size;
	MODELVIEW_MATRIX = VIEW_MATRIX * mat4(INV_VIEW_MATRIX[0], INV_VIEW_MATRIX[1], INV_VIEW_MATRIX[2], MODEL_MATRIX[3]);
}

// Signed distance to the pin: the round head and the cone from the tip that is tangent to it
float pin_distance(vec2 p) {
	float head = length(p - HEAD) - HEAD_RADIUS;
	float tip_to_head = length(HEAD);
	float s = HEAD_RADIUS / tip_to_head;  // sine of the cone's half angle
	float c = sqrt(1.0 - s * s);
	float side = abs(p.x) * c - p.y * s;
	float cone = max(side, max(p.y - tip_to_head * c * c, -p.y));
	return min(head, cone);
}

void fragment() {
	vec2 p = vec2((UV.x - 0.5) * WIDTH, (1.0 - UV.y) * (1.0 + MARGIN) - MARGIN);
	float d = pin_distance(p);
	float aa = max(fwidth(d), 0.0005);
	float pin = 1.0 - smoothstep(-aa, aa, d);
	float hole_d = length(p - HEAD) - HOLE_RADIUS;
	float hole = 1.0 - smoothstep(-aa, aa, hole_d);
	ALBEDO = high_contrast ? vec3(1.0, 1.0, 0.0) : color;
	ALPHA = pin * (1.0 - hole) * visibility;
}
