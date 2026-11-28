// The settings menu's backdrop (see MenuBackdrop.gd): the screen behind it blurred and tinted to
// the HUD's dark blue, as frosted glass. Drawn with a white shape whose alpha is its coverage (its
// anti-aliased rounded corners).
shader_type canvas_item;
render_mode unshaded;

uniform sampler2D screen_texture : hint_screen_texture, filter_linear_mipmap;
uniform vec4 tint = vec4(0.03, 0.05, 0.09, 0.78);
uniform float blur_lod = 3.5;  // the mipmap level the blur samples (each level halves the size)
uniform float top_glow = 0.05;  // the accent added at the top, fading down
uniform float height = 1080.0;  // [px] of the backdrop, for the glow

varying float local_y;

void vertex() {
	local_y = VERTEX.y;
}

void fragment() {
	// A few taps around the pixel at a coarse mipmap level: smooth, unlike a single one (blocky)
	vec2 texel = SCREEN_PIXEL_SIZE * exp2(blur_lod);
	vec3 blurred = vec3(0.0);
	for (int x = -1; x <= 1; x++) {
		for (int y = -1; y <= 1; y++) {
			blurred += textureLod(screen_texture, SCREEN_UV + vec2(float(x), float(y)) * texel, blur_lod).rgb;
		}
	}
	blurred /= 9.0;
	vec3 color = mix(blurred, tint.rgb, tint.a);
	color += vec3(0.45, 0.68, 1.0) * top_glow * (1.0 - smoothstep(0.0, 0.35 * height, local_y));
	COLOR = vec4(color, COLOR.a);
}
