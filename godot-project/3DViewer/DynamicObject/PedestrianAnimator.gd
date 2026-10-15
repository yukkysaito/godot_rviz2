# res://DynamicObject/PedestrianAnimator.gd
extends Node3D
class_name PedestrianAnimator
# Loop a "Walk" clip via AnimationPlayer.
# Playback rate is scaled by node meta "speed_mps" (float, m/s). When the pedestrian stops, the
# clip plays on to the next pose with the feet together (rest_times) and stops there, instead of
# freezing mid-stride; it starts walking again from there.

@export var walk_clip_name: StringName = &"Walk"   # Exact clip name if known
@export var walk_ref_speed_mps: float = 1.5        # Speed where rate ≈ 1.0
# [s] times in the walk clip with the feet together (where the feet are closest in Pedestrian.glb)
@export var rest_times: PackedFloat32Array = PackedFloat32Array([0.25, 0.875])
@export var settle_rate: float = 0.8  # play rate at least, walking on to the feet together

const IDLE_THRESHOLD: float = 0.15
const MAX_PLAY_RATE: float = 2.0

var _ap: AnimationPlayer
var _walk_name: StringName = StringName()
var _resting := true  # stopped with the feet together

func _ready() -> void:
	# Avoid conflicts: if an AnimationTree exists, disable it.
	var tree: AnimationTree = get_node_or_null("AnimationTree") as AnimationTree
	if tree != null:
		tree.active = false

	_ap = get_node_or_null("AnimationPlayer") as AnimationPlayer
	if _ap == null:
		set_process(false)
		return

	_ap.playback_process_mode = AnimationPlayer.ANIMATION_PROCESS_IDLE

	# Resolve a walk clip (preferred name > contains "walk" > first clip).
	_walk_name = _resolve_walk(_ap, walk_clip_name)
	if _walk_name == StringName():
		set_process(false)
		return

	var anim: Animation = _ap.get_animation(_walk_name)
	if anim != null:
		anim.loop_mode = Animation.LOOP_LINEAR

	_ap.play(_walk_name)
	_ap.speed_scale = 0.0
	if not rest_times.is_empty():
		_ap.seek(rest_times[0], true)  # standing with the feet together

	# Run only when visible: a pooled, hidden instance would still pose its skeleton every frame
	_set_running(visible)

func _notification(what: int) -> void:
	if what == NOTIFICATION_VISIBILITY_CHANGED:
		_set_running(visible)
		# If it becomes visible again, ensure playback is alive.
		if visible and _ap != null and not _ap.is_playing():
			_ap.play(_walk_name)

func _set_running(running: bool) -> void:
	set_process(running)
	if _ap != null:
		_ap.process_mode = Node.PROCESS_MODE_INHERIT if running else Node.PROCESS_MODE_DISABLED

func _process(delta: float) -> void:
	if _ap == null or _walk_name == StringName() or not visible:
		return

	# Expect the renderer to set: node.set_meta("speed_mps", float_mps)
	var speed_mps: float = 0.0
	if has_meta("speed_mps"):
		speed_mps = float(get_meta("speed_mps"))

	if speed_mps < IDLE_THRESHOLD:
		_settle(delta)
	else:
		_resting = false
		# manual max(walk_ref_speed_mps, 0.01) to keep type as float
		var ref: float = walk_ref_speed_mps
		if ref < 0.01:
			ref = 0.01

		var rate: float = speed_mps / ref
		# manual clamp to keep type as float
		if rate < 0.0:
			rate = 0.0
		elif rate > MAX_PLAY_RATE:
			rate = MAX_PLAY_RATE

		_ap.speed_scale = rate

	# Keep playback alive (AnimationPlayer may be stopped externally).
	if not _ap.is_playing():
		_ap.play(_walk_name)

# Plays on until the feet are together, then stops there
func _settle(delta: float) -> void:
	if _resting or rest_times.is_empty():
		_ap.speed_scale = 0.0
		return
	var length := _ap.current_animation_length
	var position := _ap.current_animation_position
	var next := INF  # [s] of the clip, past its end for the next loop
	for time in rest_times:
		next = minf(next, time if time >= position else time + length)
	var rate := maxf(_ap.speed_scale, settle_rate)
	if next - position <= rate * delta:
		_ap.seek(fmod(next, length), true)
		_ap.speed_scale = 0.0
		_resting = true
	else:
		_ap.speed_scale = rate

func _resolve_walk(ap: AnimationPlayer, preferred: StringName) -> StringName:
	if preferred != StringName() and ap.has_animation(preferred):
		return preferred

	var names: PackedStringArray = ap.get_animation_list()
	for anim_name in names:
		var low: String = String(anim_name).to_lower()
		if low.find("walk") >= 0:
			return StringName(anim_name)

	if names.size() > 0:
		return StringName(names[0])

	return StringName()
