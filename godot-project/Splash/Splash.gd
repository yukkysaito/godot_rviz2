extends Control

@export var main_scene_path: String = "res://Main.tscn"

# Splash is visible at least this long
@export var min_splash_seconds: float = 1.0

# Optional debug hold (keeps splash after "Finished")
@export var debug_hold_seconds: float = 0.0

@export var fade_out_seconds: float = 3.0

@onready var status_label: Label = $Center/StatusLabel
@onready var fade: ColorRect = $FadeLayer/Fade

# --- Orb viewport (optional)
@onready var orb_slot: SubViewportContainer = get_node_or_null("OrbSlot")
@onready var orb_vp: SubViewport = get_node_or_null("OrbSlot/OrbViewport")
@onready var orb3d: Node = get_node_or_null("OrbSlot/OrbViewport/Orb3D")

var _progress: Array = [0.0]
var _start_msec: int = 0
var _is_finishing: bool = false

func _ready() -> void:
	_start_msec = Time.get_ticks_msec()

	# Fade setup
	fade.color = Color.BLACK
	fade.modulate = Color(1, 1, 1, 0)

	status_label.text = "Loading"

	_apply_viewport_size()
	get_viewport().size_changed.connect(_apply_viewport_size)

	var err := ResourceLoader.load_threaded_request(main_scene_path)
	if err != OK:
		status_label.text = "Failed to load: %s" % err
		set_process(false)
		return

	set_process(true)

func _process(_delta: float) -> void:
	if _is_finishing:
		return

	var st := ResourceLoader.load_threaded_get_status(main_scene_path, _progress)

	if st == ResourceLoader.THREAD_LOAD_LOADED:
		_is_finishing = true
		await _finish()
	elif st == ResourceLoader.THREAD_LOAD_FAILED:
		status_label.text = "Loading failed"
		set_process(false)


func _finish() -> void:
	set_process(false)

	# Minimum splash duration
	var elapsed := float(Time.get_ticks_msec() - _start_msec) / 1000.0
	var remain: float = max(0.0, min_splash_seconds - elapsed)
	if remain > 0.0:
		await get_tree().create_timer(remain).timeout

	status_label.text = "Finished"
	await get_tree().create_timer(0.12).timeout

	if debug_hold_seconds > 0.0:
		await get_tree().create_timer(debug_hold_seconds).timeout

	# Orb disperse
	if is_instance_valid(orb3d) and orb3d.has_method("trigger_done"):
		orb3d.call("trigger_done")

	# Start fade out
	await _fade_to_black(fade_out_seconds)

	var res := ResourceLoader.load_threaded_get(main_scene_path)
	var packed := res as PackedScene
	if packed == null:
		status_label.text = "Invalid main scene"
		return

	get_tree().change_scene_to_packed(packed)


func _fade_to_black(duration: float) -> void:
	var tw := create_tween()
	tw.set_trans(Tween.TRANS_SINE)
	tw.set_ease(Tween.EASE_IN_OUT)
	tw.tween_property(
		fade,
		"modulate",
		Color(1, 1, 1, 1),
		max(duration, 0.0)
	)
	await tw.finished


func _apply_viewport_size() -> void:
	if orb_vp == null or orb_slot == null:
		return

	var s := get_viewport_rect().size
	orb_vp.size = s
	orb_slot.stretch = true
