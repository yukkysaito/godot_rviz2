# res://DynamicObject/PedestrianAnimator.gd
extends Node3D
class_name PedestrianAnimator
# Drive "Walk" using AnimationPlayer only, scaled by speed_mps meta.
# AnimationTree (if present) is disabled to avoid conflicts.

@export var walk_clip_name: StringName = &"Walk"   # Must match your clip name
@export var walk_ref_speed_mps: float = 1.5        # Tune 1.3–1.5 to reduce foot sliding
@export var debug_print: bool = false

const IDLE_THRES: float = 0.15
const MIN_PLAY_RATE: float = 0.0
const MAX_PLAY_RATE: float = 2.0
const LOG_INTERVAL: float = 0.5

var _ap: AnimationPlayer
var _walk_name: StringName = StringName()
var _prev_pos: float = 0.0
var _log_t: float = 0.0

func _ready() -> void:
	# Disable AnimationTree if it exists (prevent double-driving).
	var tree: AnimationTree = get_node_or_null("AnimationTree") as AnimationTree
	if tree != null:
		tree.active = false
		if debug_print:
			print("[PedestrianAnimator] AnimationTree disabled.")

	_ap = get_node_or_null("AnimationPlayer") as AnimationPlayer
	if _ap == null:
		if debug_print:
			print("[PedestrianAnimator] AnimationPlayer not found.")
		return

	# Ensure the player is not in MANUAL mode.
	_ap.playback_process_mode = AnimationPlayer.ANIMATION_PROCESS_IDLE

	# Resolve and start the walk clip (looped).
	_walk_name = _resolve_walk(_ap, walk_clip_name)
	if _walk_name != StringName():
		var anim: Animation = _ap.get_animation(_walk_name)
		if anim != null:
			anim.loop_mode = Animation.LOOP_LINEAR
		_ap.play(_walk_name)
		_ap.speed_scale = 1.0
		_prev_pos = _ap.current_animation_position
	else:
		if debug_print:
			print("[PedestrianAnimator] Walk clip not found: ", String(walk_clip_name))

	# Process only while visible (good for pooled instances).
	set_process(visible)

func _notification(what: int) -> void:
	if what == NOTIFICATION_VISIBILITY_CHANGED:
		set_process(visible)
		# Resume playback if the node becomes visible again.
		if visible and _ap != null and _walk_name != StringName() and not _ap.is_playing():
			_ap.play(_walk_name)

func _process(delta: float) -> void:
	if _ap == null or _walk_name == StringName() or not visible:
		return

	# Read speed from Renderer: node.set_meta("speed_mps", value)
	var speed_mps: float = 0.0
	if has_meta("speed_mps"):
		speed_mps = float(get_meta("speed_mps"))

	# Map speed [m/s] to playback rate.
	var rate: float = MIN_PLAY_RATE
	if speed_mps >= IDLE_THRES:
		var ref: float = walk_ref_speed_mps
		if ref < 0.01:
			ref = 0.01
		rate = clamp(speed_mps / ref, MIN_PLAY_RATE, MAX_PLAY_RATE)

	_ap.speed_scale = rate

	# Keep playback alive.
	if not _ap.is_playing():
		_ap.play(_walk_name)

	# Optional debug: ensure animation time is advancing.
	if debug_print:
		_log_t += delta
		if _log_t >= LOG_INTERVAL:
			_log_t = 0.0
			var pos_now: float = _ap.current_animation_position
			var pos_delta: float = pos_now - _prev_pos
			_prev_pos = pos_now
			print("[PedestrianAnimator] speed=", speed_mps, " rate=", rate, " pos_delta=", pos_delta)

func _resolve_walk(ap: AnimationPlayer, preferred: StringName) -> StringName:
	if preferred != StringName() and ap.has_animation(preferred):
		return preferred
	var list: PackedStringArray = ap.get_animation_list()
	for anim_name in list:  # pick the first clip containing "walk"
		var low: String = String(anim_name).to_lower()
		if low.find("walk") >= 0:
			return StringName(anim_name)
	if list.size() > 0:
		return StringName(list[0])
	return StringName()
