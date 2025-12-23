extends Control

@export var main_scene_path: String = "res://Main.tscn"
@export var min_splash_seconds: float = 2.0
@export var debug_hold_seconds: float = 0.0

@onready var status_label: Label = $Center/StatusLabel
@onready var fade: ColorRect = $FadeLayer/Fade

var _progress: Array = [0.0]
var _start_msec: int = 0
var _is_finishing: bool = false

func _ready() -> void:
	_start_msec = Time.get_ticks_msec()

	# Force the fade rect to be black (important: prevents non-black tinting in some setups).
	fade.color = Color(0, 0, 0, 1)
	# Start fully transparent; we will fade to opaque right before scene transition.
	fade.modulate = Color(1, 1, 1, 0)

	status_label.text = "Loading"

	var err: int = ResourceLoader.load_threaded_request(main_scene_path)
	if err != OK:
		status_label.text = "Failed to load: %s" % err
		set_process(false)
		return

	set_process(true)

func _process(_delta: float) -> void:
	if _is_finishing:
		return

	var st: int = ResourceLoader.load_threaded_get_status(main_scene_path, _progress)

	if st == ResourceLoader.THREAD_LOAD_LOADED:
		_is_finishing = true
		await _finish()
	elif st == ResourceLoader.THREAD_LOAD_FAILED:
		status_label.text = "Loading failed"
		set_process(false)

func _fade_to(alpha: float, duration: float) -> void:
	# Tween the modulate alpha of the ColorRect to create a fade effect.
	# NOTE: We set RGB to (1,1,1) and only animate alpha; the actual color is controlled by `fade.color`.
	var tw := create_tween()
	tw.set_trans(Tween.TRANS_SINE)
	tw.set_ease(Tween.EASE_IN_OUT)
	tw.tween_property(fade, "modulate", Color(1, 1, 1, alpha), duration)
	await tw.finished

func _finish() -> void:
	set_process(false)

	# Ensure the splash stays visible for at least `min_splash_seconds`.
	var elapsed: float = float(Time.get_ticks_msec() - _start_msec) / 1000.0
	var remain: float = max(0.0, min_splash_seconds - elapsed)
	if remain > 0.0:
		await get_tree().create_timer(remain).timeout

	status_label.text = "Finished"
	await get_tree().create_timer(0.12).timeout  # Small pause for readability.

	if debug_hold_seconds > 0.0:
		await get_tree().create_timer(debug_hold_seconds).timeout

	# Fade out to fully opaque (black) before switching scenes to avoid a visible pop.
	await _fade_to(1.0, 3.0)

	# Retrieve the loaded resource and switch to it.
	var res: Resource = ResourceLoader.load_threaded_get(main_scene_path)
	var packed := res as PackedScene
	if packed == null:
		status_label.text = "Invalid main scene"
		return

	get_tree().change_scene_to_packed(packed)
