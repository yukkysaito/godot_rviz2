extends Node
## Owns the world environment: sky, sun and atmosphere.
## set_night_mode() performs a cinematic day/night transition: the sun sinks
## and fades out, the custom sky shader blends to a starry night, ambient
## light dims and volumetric fog fades in so headlights produce light shafts.

@export var world_env: WorldEnvironment
@export var sun: DirectionalLight3D

@export_group("Transition")
@export var transition_duration: float = 2.5

@export_group("Day")
@export var day_sun_energy: float = 1.3
@export var day_sun_elevation_deg: float = -35.0
@export var day_ambient_energy: float = 1.0

@export_group("Night")
@export var night_sun_energy: float = 0.0
@export var night_sun_elevation_deg: float = -8.0
@export var night_ambient_energy: float = 0.35
@export var night_fog_density: float = 0.035

var _night_mode: bool = false
var _tween: Tween


func _ready() -> void:
	if world_env == null:
		world_env = get_node_or_null("BackGround") as WorldEnvironment
	if sun == null:
		sun = get_node_or_null("Sun") as DirectionalLight3D
	_apply_day_state_immediate()


func is_night_mode() -> bool:
	return _night_mode


func get_environment() -> Environment:
	if world_env == null:
		return null
	return world_env.environment


func get_sky_material() -> ShaderMaterial:
	var env := get_environment()
	if env == null or env.sky == null:
		return null
	return env.sky.sky_material as ShaderMaterial


func set_night_mode(enabled: bool) -> void:
	var tween_active := _tween != null and _tween.is_running()
	if enabled == _night_mode and not tween_active:
		return
	_night_mode = enabled

	if _tween != null:
		_tween.kill()

	var env := get_environment()
	var sky_mat := get_sky_material()

	# State that must flip before the fade starts.
	if enabled:
		if env != null:
			env.volumetric_fog_enabled = true
	else:
		if sun != null:
			sun.visible = true

	_tween = create_tween()
	_tween.set_parallel(true)
	_tween.set_trans(Tween.TRANS_SINE)
	_tween.set_ease(Tween.EASE_IN_OUT)

	if sky_mat != null:
		var current_blend := _get_night_blend(sky_mat)
		var target_blend := 1.0 if enabled else 0.0
		_tween.tween_method(_set_night_blend, current_blend, target_blend, transition_duration)

	if sun != null:
		var target_energy := night_sun_energy if enabled else day_sun_energy
		var target_elevation := night_sun_elevation_deg if enabled else day_sun_elevation_deg
		_tween.tween_property(sun, "light_energy", target_energy, transition_duration)
		_tween.tween_property(sun, "rotation_degrees:x", target_elevation, transition_duration)

	if env != null:
		var target_ambient := night_ambient_energy if enabled else day_ambient_energy
		var target_fog := night_fog_density if enabled else 0.0
		_tween.tween_property(env, "ambient_light_energy", target_ambient, transition_duration)
		_tween.tween_property(env, "volumetric_fog_density", target_fog, transition_duration)

	_tween.chain().tween_callback(_on_transition_finished)


func _apply_day_state_immediate() -> void:
	_night_mode = false
	_set_night_blend(0.0)
	if sun != null:
		sun.visible = true
		sun.light_energy = day_sun_energy
		sun.rotation_degrees.x = day_sun_elevation_deg
	var env := get_environment()
	if env != null:
		env.ambient_light_energy = day_ambient_energy
		env.volumetric_fog_enabled = false
		env.volumetric_fog_density = 0.0


func _on_transition_finished() -> void:
	var env := get_environment()
	if _night_mode:
		# The sun has fully faded out; hide it so it stops casting shadows.
		if sun != null:
			sun.visible = false
	else:
		# The fog has fully faded out; disable it to save GPU time.
		if env != null:
			env.volumetric_fog_enabled = false


func _get_night_blend(sky_mat: ShaderMaterial) -> float:
	var value: Variant = sky_mat.get_shader_parameter("night_blend")
	if value == null:
		return 0.0
	return float(value)


func _set_night_blend(value: float) -> void:
	var sky_mat := get_sky_material()
	if sky_mat != null:
		sky_mat.set_shader_parameter("night_blend", value)
