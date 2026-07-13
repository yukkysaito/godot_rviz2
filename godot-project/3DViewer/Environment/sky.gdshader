// Holographic AD cockpit sky.
// Day:   deep blue gradient with a subtle sun disk and warm horizon haze.
// Night: near-black (#0A0E14 family) sky, procedural star field with subtle
//        twinkle, cyan (#00E5FF) horizon glow.
// The day/night state is driven by `night_blend`, tweened from World.gd.
shader_type sky;

// 0.0 = full day, 1.0 = full night.
uniform float night_blend : hint_range(0.0, 1.0) = 0.0;

// --- Day palette ---
uniform vec3 day_zenith_color : source_color = vec3(0.035, 0.09, 0.22);
uniform vec3 day_horizon_color : source_color = vec3(0.32, 0.52, 0.78);
uniform vec3 day_ground_horizon_color : source_color = vec3(0.20, 0.28, 0.38);
uniform vec3 day_ground_color : source_color = vec3(0.05, 0.06, 0.08);

// --- Night palette (#0A0E14 family) ---
uniform vec3 night_zenith_color : source_color = vec3(0.006, 0.010, 0.018);
uniform vec3 night_horizon_color : source_color = vec3(0.020, 0.060, 0.085);
uniform vec3 night_ground_color : source_color = vec3(0.004, 0.006, 0.010);

// Cyan accent (#00E5FF) horizon glow at night.
uniform vec3 accent_color : source_color = vec3(0.0, 0.898, 1.0);
uniform float night_glow_strength : hint_range(0.0, 4.0) = 1.0;

// --- Sun ---
uniform vec3 sun_color : source_color = vec3(1.0, 0.88, 0.65);
uniform float sun_disk_size : hint_range(0.001, 0.1) = 0.012;
uniform float sun_disk_energy : hint_range(0.0, 32.0) = 10.0;

// --- Stars ---
uniform float star_density : hint_range(8.0, 128.0) = 56.0;
uniform float star_probability : hint_range(0.0, 0.2) = 0.03;
uniform float star_energy : hint_range(0.0, 8.0) = 2.0;
uniform float star_twinkle_speed : hint_range(0.0, 8.0) = 1.2;

float hash13(vec3 p) {
	p = fract(p * 0.1031);
	p += dot(p, p.zyx + 31.32);
	return fract((p.x + p.y) * p.z);
}

vec3 star_field(vec3 dir, float time) {
	// Quantize the view direction into cells; at most one star per cell.
	vec3 cell = floor(dir * star_density);
	float h = hash13(cell);
	if (h > star_probability) {
		return vec3(0.0);
	}
	vec3 jitter = vec3(
			hash13(cell + 17.1),
			hash13(cell + 31.7),
			hash13(cell + 47.3)) - 0.5;
	vec3 star_dir = normalize(cell + 0.5 + jitter * 0.8);
	float dist = distance(dir, star_dir);
	float radius = 0.0015 + hash13(cell + 5.9) * 0.0035;
	float star = 1.0 - smoothstep(radius * 0.3, radius, dist);
	// Subtle per-star twinkle.
	float twinkle = 0.7 + 0.3 * sin(time * star_twinkle_speed * (0.5 + 4.0 * h) + h * 100.0);
	// Slight cool/warm tint variation.
	vec3 tint = mix(vec3(0.80, 0.92, 1.0), vec3(1.0, 0.94, 0.82), hash13(cell + 3.3));
	return tint * star * twinkle;
}

void sky() {
	vec3 dir = EYEDIR;
	float height = dir.y;
	float horizon_band = pow(1.0 - clamp(abs(height), 0.0, 1.0), 8.0);

	vec3 sun_dir = LIGHT0_ENABLED ? LIGHT0_DIRECTION : normalize(vec3(-0.63, 0.57, 0.53));
	float sun_amount = clamp(dot(dir, sun_dir), 0.0, 1.0);

	// --- Day ---
	vec3 day_sky = mix(day_horizon_color, day_zenith_color, pow(clamp(height, 0.0, 1.0), 0.4));
	vec3 day_ground = mix(day_ground_horizon_color, day_ground_color, pow(clamp(-height, 0.0, 1.0), 0.5));
	vec3 day_col = (height >= 0.0) ? day_sky : day_ground;
	// Warm haze around the sun and along the horizon.
	day_col += sun_color * (pow(sun_amount, 32.0) * 0.4 + pow(sun_amount, 6.0) * 0.1);
	day_col += day_horizon_color * horizon_band * 0.5;

	// --- Night ---
	vec3 night_sky = mix(night_horizon_color, night_zenith_color, pow(clamp(height, 0.0, 1.0), 0.5));
	vec3 night_ground = mix(night_horizon_color * 0.7, night_ground_color, pow(clamp(-height, 0.0, 1.0), 0.4));
	vec3 night_col = (height >= 0.0) ? night_sky : night_ground;
	// Cyan horizon glow.
	night_col += accent_color * horizon_band * 0.22 * night_glow_strength;

	// Stars only above the horizon; skipped in the radiance cubemap pass so
	// ambient lighting stays temporally stable.
	if (!AT_CUBEMAP_PASS && night_blend > 0.001) {
		float star_mask = smoothstep(0.01, 0.15, height);
		night_col += star_field(dir, TIME) * star_energy * star_mask;
	}

	vec3 col = mix(day_col, night_col, night_blend);

	// HDR sun disk (energy > 1 so bloom picks it up), fades out at night.
	if (!AT_CUBEMAP_PASS) {
		float sun_disk = smoothstep(1.0 - sun_disk_size, 1.0 - sun_disk_size * 0.35, sun_amount);
		col += sun_color * sun_disk * sun_disk_energy * (1.0 - night_blend);
	}

	COLOR = col;
}
