extends Control

# Loading screen: loads the main scene in the background while an orb of smoke shows the
# progress (see Shaders/orb.gdshader), then lets the smoke drift away and switches scenes.
#
# Besides the main scene, it waits until the vector map is built and the ego pose is known (both
# are received by RosBridge from start-up), so the main screen starts complete. Without Autoware
# (no map publisher) it does not wait for them; each wait also has a time limit.

@export var main_scene_path: String = "res://3DViewer/Main.tscn"

# Splash is visible at least this long
@export var min_splash_seconds: float = 1.5

# Optional debug hold (keeps the splash after loading finished)
@export var debug_hold_seconds: float = 0.0

# Exit animation (the smoke drifts away) and the final fade to black at its end
@export var exit_seconds: float = 3.0
@export var fade_out_seconds: float = 1.2

# Waiting for Autoware: time to discover the map publisher, the longest wait for the map, and
# the longest wait for the ego pose after the map is ready
@export var discovery_seconds: float = 2.0
@export var map_wait_seconds: float = 20.0
@export var pose_wait_seconds: float = 3.0

const PROGRESS_SPEED := 0.5         # max change of the shown progress per second
const PROGRESS_SPEED_LOADED := 1.6  # ... once loading finished
const STATUS_FADE_IN_SECONDS := 0.8

# Stage -> [progress at start, progress at end, typical duration in seconds]. Within a stage the
# shown progress creeps toward its end, so it never looks stuck.
const STAGES := {
	"loading scene": [0.0, 0.5, 1.5],
	"receiving map": [0.5, 0.6, 1.0],
	"building map": [0.6, 0.9, 3.0],
	"localizing": [0.9, 0.97, 2.0],
}
const STAGE_READY := "ready"

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
var _stage: String = ""
var _stage_start: float = 0.0  # [s] since the splash started
var _pose_wait_start: float = -1.0

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
	if loaded:
		PerfMonitor.mark("main_scene_loaded")
	var elapsed := _elapsed_seconds()
	var stage := _current_stage(loaded, elapsed)
	if stage != _stage:
		_stage = stage
		_stage_start = elapsed
		PerfMonitor.mark("splash_stage_" + stage.replace(" ", "_"))

	var ready := stage == STAGE_READY
	var speed := PROGRESS_SPEED_LOADED if ready else PROGRESS_SPEED
	var target := maxf(_target_progress(elapsed), _shown_progress)  # never goes back
	_set_progress(move_toward(_shown_progress, target, delta * speed))

	if ready and _shown_progress >= 1.0 and elapsed >= min_splash_seconds:
		_is_finishing = true
		_finish()

func _elapsed_seconds() -> float:
	return float(Time.get_ticks_msec() - _start_msec) / 1000.0

# What the splash is waiting for (STAGE_READY when it can finish)
func _current_stage(loaded: bool, elapsed: float) -> String:
	if not loaded:
		return "loading scene"
	# Without a map publisher (Autoware not running) there is nothing to wait for
	var autoware := RosBridge.has_map_publisher() or RosBridge.is_map_ready or RosBridge.is_map_building
	if not autoware and elapsed < discovery_seconds:
		return "receiving map"
	if autoware and not RosBridge.is_map_ready and elapsed < map_wait_seconds:
		return "building map" if RosBridge.is_map_building else "receiving map"
	if autoware and not RosBridge.is_ego_pose_ready():
		if _pose_wait_start < 0.0:
			_pose_wait_start = elapsed
		if elapsed - _pose_wait_start < pose_wait_seconds:
			return "localizing"
	return STAGE_READY

func _target_progress(elapsed: float) -> float:
	if _stage == STAGE_READY:
		return 1.0
	var stage: Array = STAGES[_stage]
	var t := (elapsed - _stage_start) / maxf(float(stage[2]), 0.01)
	var target: float = lerpf(stage[0], stage[1], 1.0 - exp(-2.0 * t))
	if _stage == "loading scene":
		target = maxf(target, float(_load_progress[0]) * float(stage[1]))
	return target

func _set_progress(value: float) -> void:
	_shown_progress = value
	_material.set_shader_parameter(U_PROGRESS, value)
	_status_label.text = "%s   %d%%" % [_stage, int(round(value * 100.0))]

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
