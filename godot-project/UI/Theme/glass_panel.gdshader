// Frosted-glass panel shader for the holographic AD cockpit UI.
//
// Apply as a material on a Panel / PanelContainer whose StyleBoxFlat has an
// opaque (white) bg_color: the shader replaces the RGB of every pixel the
// item draws with a blurred sample of the screen behind it, mixed with a
// dark tint. The item's alpha (stylebox alpha * modulate) is preserved, so
// rounded corners, corner anti-aliasing and fade animations keep working.
shader_type canvas_item;

uniform sampler2D screen_tex : hint_screen_texture, filter_linear_mipmap;

// Dark tint mixed over the blurred backdrop (#0A0E14 at 0.55 by default).
uniform vec4 tint : source_color = vec4(0.0392157, 0.054902, 0.0784314, 0.55);

// Mip level used for the cheap blur (higher = blurrier).
uniform float blur_lod : hint_range(0.0, 6.0) = 2.5;

// Subtle bright highlight along the top edge of the panel.
uniform float highlight_strength : hint_range(0.0, 0.5) = 0.06;
uniform float highlight_height_px = 26.0;

varying vec2 v_local;

void vertex() {
	// Local (pixel) position inside the item; y == 0 is the panel's top edge.
	v_local = VERTEX;
}

void fragment() {
	vec3 backdrop = textureLod(screen_tex, SCREEN_UV, blur_lod).rgb;
	vec3 col = mix(backdrop, tint.rgb, tint.a);
	float hl = smoothstep(highlight_height_px, 0.0, v_local.y) * highlight_strength;
	col += vec3(hl);
	COLOR = vec4(col, COLOR.a);
}
