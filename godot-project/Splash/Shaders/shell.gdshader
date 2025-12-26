shader_type spatial;
render_mode cull_back, fog_disabled;

uniform float u_alpha    = 0.08;   // Base shell opacity
uniform float u_rim_pow  = 2.2;    // Rim sharpness
uniform float u_rim_gain = 3.0;    // Rim intensity
uniform vec3  u_color    = vec3(1.0);

uniform float u_done_t    = 0.0;   // Time since disperse started
uniform float u_fade_time = 0.6;   // Fade-out duration

/* Rim lighting term based on view angle */
float rim_term(vec3 N, vec3 V){
	float ndv = abs(dot(N, V));
	return pow(1.0 - ndv, u_rim_pow);
}

/* Base alpha using rim contribution */
float base_alpha(float rim){
	return u_alpha * (0.4 + rim * 1.8);
}

/* Quadratic ease-out */
float ease_out_quad(float x){
	return 1.0 - (1.0 - x) * (1.0 - x);
}

/* Fade factor driven by done time */
float fade_factor(float done_t){
	if (done_t <= 0.0) return 1.0;

	float t = clamp(done_t / max(u_fade_time, 0.0001), 0.0, 1.0);
	t = ease_out_quad(t);
	return 1.0 - t;
}

void fragment(){
	vec3 N = normalize(NORMAL);
	vec3 V = normalize(VIEW);

	float rim  = rim_term(N, V);
	float fade = fade_factor(u_done_t);

	float a = base_alpha(rim) * fade;

	ALBEDO   = u_color * 0.05;
	EMISSION = u_color * rim * u_rim_gain * fade;
	ALPHA    = clamp(a, 0.0, 1.0);
}
