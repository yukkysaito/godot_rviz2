shader_type spatial;
render_mode fog_disabled, depth_draw_opaque, cull_disabled;

/* Time and animation control */
uniform float u_time   = 0.0;
uniform float u_radius = 1.0;
uniform float u_mode   = 0.0;   // 0: orb state, 1: disperse state
uniform float u_done_t = 0.0;

/* Motion parameters */
uniform float u_flow   = 1.2;
uniform float u_spread = 2.5;
uniform float u_fade   = 1.4;

/* Impulse parameters (same as original) */
const float IMP_STRENGTH  = 2000.0;
const float IMP_DECAY     = 3.5;
const float IMP_MAX_BOOST = 600.0;

/* Simple hash for pseudo-random direction */
float hash11(float p){
	p = fract(p * 0.1031);
	p *= p + 33.33;
	p *= p + p;
	return fract(p);
}

/* Convert spherical angles to Cartesian direction */
vec3 sphere_from_angles(float theta, float phi){
	float s = sin(phi);
	return vec3(cos(theta) * s, cos(phi), sin(theta) * s);
}

/* Decode per-instance custom data
   r,g : random seeds
   b   : theta (0..1)
   a   : phi   (0..1)
*/
void decode_custom(
	vec4 cd,
	out float seed0,
	out float seed1,
	out float theta0,
	out float phi0
){
	seed0  = cd.r;
	seed1  = cd.g;
	theta0 = cd.b * TAU;
	phi0   = cd.a * PI;
}

/* Orb position with time-based wobble */
vec3 compute_orb_pos(
	float seed0,
	float seed1,
	float theta0,
	float phi0,
	float t
){
	float n1 = sin(t + seed0 * 10.0) * 0.15;
	float n2 = cos(t * 0.9 + seed1 * 12.0) * 0.12;

	float theta = theta0 + n1 + sin(t * 0.37 + seed1 * 20.0) * 0.10;
	float phi   = phi0   + n2 + cos(t * 0.41 + seed0 * 18.0) * 0.08;

	return sphere_from_angles(theta, phi) * u_radius;
}

/* Direction used during disperse phase */
vec3 compute_dir(vec3 p_orb, float seed0, float seed1){
	vec3 base_dir = normalize(p_orb);

	float r1 = hash11(seed0 * 17.0 + seed1 * 3.0);
	float r2 = hash11(seed1 * 19.0 + seed0 * 5.0);

	vec3 rand_dir = normalize(
		vec3(
			r1 - 0.5,
			r2 - 0.5,
			(r1 + r2) * 0.5 - 0.5
		)
	);

	return normalize(base_dir * 0.8 + rand_dir * 0.2);
}

/* Brightness impulse during disperse */
float impulse_boost(float dt){
	float b = 1.0 + IMP_STRENGTH * dt * exp(-dt * IMP_DECAY);
	return min(b, IMP_MAX_BOOST);
}

void vertex(){
	/* Read per-instance data here (required for Godot) */
	vec4 cd = INSTANCE_CUSTOM;

	float seed0, seed1, theta0, phi0;
	decode_custom(cd, seed0, seed1, theta0, phi0);

	float t = u_time * u_flow;

	vec3 p_orb = compute_orb_pos(
		seed0,
		seed1,
		theta0,
		phi0,
		t
	);

	vec3 dir = compute_dir(p_orb, seed0, seed1);

	float dt = max(u_done_t, 0.0);
	float fly = pow(dt, 1.25) * u_spread;

	/* Blend orb and disperse positions */
	vec3 p = mix(
		p_orb,
		p_orb + dir * fly,
		clamp(u_mode, 0.0, 1.0)
	);

	VERTEX += p;
}

void fragment(){
	float dt = max(u_done_t, 0.0);

	float a = 1.0;
	float boost = 1.0;

	/* Fade and boost only in disperse mode */
	if (u_mode > 0.5){
		a = exp(-dt * u_fade);
		boost = impulse_boost(dt);
	}

	/* Radial sprite shape from quad UV */
	vec2 p = UV * 2.0 - 1.0;
	float r2 = dot(p, p);

	float core = exp(-r2 * 10.0);
	float haze = exp(-r2 * 2.5);
	float sprite = clamp(core + haze * 0.35, 0.0, 1.0);

	float alpha = a * sprite;

	ALBEDO   = vec3(1.0) * sprite;
	EMISSION = vec3(0.01) * sprite * a * boost;
	ALPHA    = alpha;
}
