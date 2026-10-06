extends Control

# Loading screen: loads the main scene in the background while an orb of smoke shows the
# progress (see Shaders/orb.gdshader), then lets the smoke drift away and switches scenes.

@export var main_scene_path: String = "res://3DViewer/Main.tscn"

# Splash is visible at least this long
@export var min_splash_seconds: float = 1.5

# Optional debug hold (keeps the splash after loading finished)
@export var debug_hold_seconds: float = 0.0

# Exit animation (the smoke drifts away) and the final fade to black at its end
@export var exit_seconds: float = 3.0
@export var fade_out_seconds: float = 1.2

# The loader reports progress only in coarse steps; in between, the shown progress creeps
# toward creep_limit over about creep_seconds so it never looks stuck.
@export var creep_seconds: float = 6.0
@export var creep_limit: float = 0.85

const PROGRESS_SPEED := 0.5         # max change of the shown progress per second
const PROGRESS_SPEED_LOADED := 1.6  # ... once loading finished
const STATUS_FADE_IN_SECONDS := 0.8

const STATUS_STEPS := [
	[0.0, "initializing"],
	[0.3, "loading assets"],
	[0.7, "building scene"],
	[1.0, "ready"],
]

const U_TIME := &"u_time"
const U_PROGRESS := &"u_progress"
const U_DONE := &"u_done"

@onready var _material: ShaderMaterial = $Background.material as ShaderMaterial
@onready var _status_label: Label = $StatusLabel
@onready var _fade: ColorRect = $FadeLayer/Fade

var _load_progress: Array = [0.0]  # filled by ResourceLoader.load_threaded_get_status()
var _shown_progress: float = 0.0
var _start_msec: int = 0
var _is_finishing: bool = false

func _ready() -> void:
	PerfMonitor.mark("splash_ready")
	_start_msec = Time.get_ticks_msec()
	_fade.modulate.a = 0.0

	_status_label.modulate.a = 0.0
	create_tween().tween_property(_status_label, "modulate:a", 1.0, STATUS_FADE_IN_SECONDS) \
		.set_trans(Tween.TRANS_SINE)

	var err := ResourceLoader.load_threaded_request(main_scene_path)
	if err != OK:
		_show_error("failed to load (%s)" % err)

func _process(delta: float) -> void:
	_material.set_shader_parameter(U_TIME, Time.get_ticks_msec() * 0.001)
	if _is_finishing:
		return

	var status := ResourceLoader.load_threaded_get_status(main_scene_path, _load_progress)
	if status == ResourceLoader.THREAD_LOAD_FAILED or status == ResourceLoader.THREAD_LOAD_INVALID_RESOURCE:
		_show_error("loading failed")
		return

	var loaded := status == ResourceLoader.THREAD_LOAD_LOADED
	var elapsed := _elapsed_seconds()
	var speed := PROGRESS_SPEED_LOADED if loaded else PROGRESS_SPEED
	_set_progress(move_toward(_shown_progress, _target_progress(loaded, elapsed), delta * speed))

	if loaded:
		PerfMonitor.mark("main_scene_loaded")
	if loaded and _shown_progress >= 1.0 and elapsed >= min_splash_seconds:
		_is_finishing = true
		_finish()

func _elapsed_seconds() -> float:
	return float(Time.get_ticks_msec() - _start_msec) / 1000.0

func _target_progress(loaded: bool, elapsed: float) -> float:
	if loaded:
		return 1.0
	var creep := creep_limit * (1.0 - exp(-2.0 * elapsed / maxf(creep_seconds, 0.01)))
	return maxf(float(_load_progress[0]), creep)

func _set_progress(value: float) -> void:
	_shown_progress = value
	_material.set_shader_parameter(U_PROGRESS, value)
	_status_label.text = "%s   %d%%" % [_status_text(value), int(round(value * 100.0))]

func _status_text(progress: float) -> String:
	var text: String = STATUS_STEPS[0][1]
	for step in STATUS_STEPS:
		if progress >= step[0]:
			text = step[1]
	return text

func _show_error(message: String) -> void:
	_status_label.text = message
	set_process(false)

func _finish() -> void:
	if debug_hold_seconds > 0.0:
		await get_tree().create_timer(debug_hold_seconds).timeout

	# The smoke drifts away, the status fades out, and the screen fades to black at the end
	var tw := create_tween().set_parallel(true)
	tw.tween_method(func(v: float): _material.set_shader_parameter(U_DONE, v), 0.0, 1.0, exit_seconds)
	tw.tween_property(_status_label, "modulate:a", 0.0, exit_seconds * 0.5)
	tw.tween_property(_fade, "modulate:a", 1.0, fade_out_seconds) \
		.set_delay(maxf(exit_seconds - fade_out_seconds * 0.5, 0.0)).set_trans(Tween.TRANS_SINE)
	await tw.finished

	var packed := ResourceLoader.load_threaded_get(main_scene_path) as PackedScene
	if packed == null:
		_fade.modulate.a = 0.0
		_status_label.modulate.a = 1.0
		_show_error("invalid main scene")
		return

	PerfMonitor.mark("splash_finished")
	get_tree().change_scene_to_packed(packed)
