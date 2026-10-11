// Icon above an object: a quad fully facing the camera (a Y-only billboard would turn edge-on in
// top-down views). The texture is sampled from a sharper mipmap level than the default, which
// looks blurry for these small icons.
shader_type spatial;
render_mode unshaded, cull_back, shadows_disabled;

uniform sampler2D icon : source_color, filter_linear_mipmap_anisotropic;
uniform float mipmap_bias = -1.0;  // < 0: sharper

void vertex() {
	MODELVIEW_MATRIX = VIEW_MATRIX * mat4(INV_VIEW_MATRIX[0], INV_VIEW_MATRIX[1], INV_VIEW_MATRIX[2], MODEL_MATRIX[3]);
}

void fragment() {
	vec4 color = texture(icon, UV, mipmap_bias);
	ALBEDO = color.rgb;
	ALPHA = color.a;
}
