# res://DynamicObject/PedestrianAnimator.gd
extends Node3D
class_name PedestrianAnimator
# Loop a "Walk" clip via AnimationPlayer.
# Playback rate is scaled by node meta "speed_mps" (float, m/s).

@export var walk_clip_name: StringName = &"Walk"   # Exact clip name if known
@export var walk_ref_speed_mps: float = 1.5        # Speed where rate ≈ 1.0

const IDLE_THRESHOLD: float = 0.15
const MAX_PLAY_RATE: float = 2.0

var _ap: AnimationPlayer
var _walk_name: StringName = StringName()

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

	# Run only when visible (good for pooled instances).
	set_process(visible)

func _notification(what: int) -> void:
	if what == NOTIFICATION_VISIBILITY_CHANGED:
		set_process(visible)
		# If it becomes visible again, ensure playback is alive.
		if visible and _ap != null and not _ap.is_playing():
			_ap.play(_walk_name)

func _process(_delta: float) -> void:
	if _ap == null or _walk_name == StringName() or not visible:
		return

	# Expect the renderer to set: node.set_meta("speed_mps", float_mps)
	var speed_mps: float = 0.0
	if has_meta("speed_mps"):
		speed_mps = float(get_meta("speed_mps"))

	if speed_mps < IDLE_THRESHOLD:
		_ap.speed_scale = 0.0
	else:
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
