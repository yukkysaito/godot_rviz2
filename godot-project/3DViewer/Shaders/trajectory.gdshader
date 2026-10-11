// Planned trajectory as chevrons that flow along at the planned speed. They are spaced in planned
// time, so they bunch up where the vehicle slows down. A trail behind each one shows the
// acceleration: longer the stronger, blue while speeding up and red while slowing down; the
// chevron then takes on the trail's color and steps back (thinner, fainter).
// UV.x: 0..1 across the strip, UV.y: distance along it [m].
// CUSTOM0: planned speed [m/s], planned time to get there [s], acceleration [m/s^2], length [m].
shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_never, cull_disabled, shadows_disabled;

// Linear colors (no source_color hint: its default values are not converted from sRGB)
uniform vec3 chevron_color = vec3(0.35, 0.37, 0.4);  // gray, sRGB (0.63, 0.64, 0.67)
uniform vec3 accel_color = vec3(0.1, 0.35, 1.0);
uniform vec3 decel_color = vec3(1.0, 0.08, 0.04);
uniform float chevron_alpha = 0.7;
uniform float accel_scale = 1.0;  // [m/s^2] for the strongest look
uniform float trail_max = 1.8;  // [m] at accel_scale
uniform float interval = 0.5;  // [s] of planned travel between chevrons
uniform float half_width = 0.9;  // [m] of the strip
uniform float far_start = 60.0;  // [m] from the camera
uniform float far_end = 200.0;
uniform float far_alpha = 0.5;
uniform bool high_contrast = false;

varying vec4 info;

void vertex() {
	info = CUSTOM0;
}

void fragment() {
	float speed = info.x, time = info.y, accel = info.z, len = info.w;
	float strength = clamp(abs(accel) / accel_scale, 0.0, 1.0);
	float x = abs(UV.x * 2.0 - 1.0) * half_width;  // [m] from the middle
	float v = max(speed, 0.3);
	float aa = max(fwidth(UV.y), 0.002) * 1.5;  // [m]
	float inside = 1.0 - smoothstep(half_width * 0.85, half_width, x);

	// Chevrons pointing forward (their arms are further back the farther out), moving with time
	float t = time - TIME + x * 0.8 / v;
	float phase = fract(t / interval) * interval;
	float behind_next = (interval - phase) * v;  // [m] to the chevron in front
	float in_front = phase * v;  // [m] past the chevron behind
	float thickness = mix(0.08, 0.035, strength);
	float chevron = 1.0 - smoothstep(thickness, thickness + aa, min(in_front, behind_next));
	float trail_len = strength * trail_max;
	float trail = trail_len > 0.001 ? exp(-behind_next / (trail_len * 0.45)) * 0.6 : 0.0;
	// Fading out just before the stop, where they would merge
	float moving = smoothstep(0.1, 0.8, speed) * inside;
	chevron *= moving;
	trail *= moving;

	vec3 trail_c = accel >= 0.0 ? accel_color : decel_color;
	vec3 chevron_c = mix(chevron_color, trail_c, strength * 0.9);
	vec3 c = mix(trail_c, chevron_c, chevron >= trail ? 1.0 : chevron / max(trail, 0.001));
	float alpha = max(chevron * chevron_alpha * mix(1.0, 0.4, strength), trail);
	// Faded in from the vehicle and out at the end
	alpha *= smoothstep(0.0, 2.0, UV.y) * (1.0 - smoothstep(len - 6.0, len, UV.y));
	if (high_contrast) {
		c = vec3(0.0, 1.0, 0.0);
		alpha = clamp(alpha * 2.0, 0.0, 1.0);
	} else {
		alpha *= mix(1.0, far_alpha, smoothstep(far_start, far_end, length(VERTEX)));
	}
	ALBEDO = c;
	ALPHA = clamp(alpha, 0.0, 1.0);
}
