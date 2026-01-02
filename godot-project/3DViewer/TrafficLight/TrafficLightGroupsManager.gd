extends Node3D
class_name TrafficLightGroupsManager

# Spawns / destroys TrafficLightGroupActor instances based on HDMap groups,
# then updates their glow state from the perception topic.

@export var actor_scene: PackedScene            # Recommended: TrafficLightGroupActor.tscn
@export var board_scene: PackedScene            # Optional: injected into each actor

@export var default_board_color: Color = Color(0.10, 0.10, 0.10, 1.0)
@export var z_offset_board: float = 0.02
@export var z_offset_bulb: float = 0.00

# If no recognition update arrives for this duration, turn off the glow for that group.
@export var auto_off_seconds: float = 0.7

var traffic_light_recognition := TrafficLights.new()

# gid(int) -> TrafficLightGroupActor
var _actors: Dictionary = {}

# gid(int) -> last time(sec) we applied recognition results for that group
var _last_seen_sec: Dictionary = {}

@onready var _root: Node3D = _ensure_root()

func _ready() -> void:
	traffic_light_recognition.subscribe(
		"/perception/traffic_light_recognition/traffic_signals",
		false
	)

func _process(_delta: float) -> void:
	_apply_recognition_if_new()
	_auto_turn_off_if_stale()

# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------

func set_map(groups: Array) -> void:
	# Called whenever the HDMap traffic light groups update.
	# This method:
	# 1) creates/updates actors for incoming groups
	# 2) removes actors that no longer exist in the incoming list

	var alive: Dictionary = {}  # Used as a "set": gid -> true
	var now: float = _now_sec()

	for g_any in groups:
		if typeof(g_any) != TYPE_DICTIONARY:
			continue
		var group: Dictionary = g_any as Dictionary

		var gid: int = int(group.get("group_id", -1))
		if gid < 0:
			continue

		alive[gid] = true

		var actor: TrafficLightGroupActor = _get_or_create_actor(gid)
		actor.build_from_hdmap(group)

		# Ensure the group starts from "all off" at map-update time.
		# (Recognition will re-light it on the next update.)
		actor.set_all_off()

		# Initialize last seen so newly created actors do not immediately auto-off.
		_last_seen_sec[gid] = now

	_remove_missing_actors(alive)

# -----------------------------------------------------------------------------
# Recognition update
# -----------------------------------------------------------------------------

func _apply_recognition_if_new() -> void:
	if not traffic_light_recognition.has_new():
		return

	var now: float = _now_sec()
	var status_list: Array = traffic_light_recognition.get_traffic_light_status()

	for st_any in status_list:
		if typeof(st_any) != TYPE_DICTIONARY:
			continue
		var status: Dictionary = st_any as Dictionary

		var gid: int = int(status.get("group_id", -1))
		if gid < 0:
			continue

		var actor: TrafficLightGroupActor = _get_actor(gid)
		if actor == null:
			continue  # Group not loaded / not present

		# status_elements should be an Array of dictionaries {color, arrow, ...}
		var elems: Array = _extract_array(status, "status_elements")

		actor.apply_status(elems)
		_last_seen_sec[gid] = now

	traffic_light_recognition.set_old()

func _auto_turn_off_if_stale() -> void:
	if auto_off_seconds <= 0.0:
		return

	var now: float = _now_sec()

	for k_any in _actors.keys():
		var gid: int = int(k_any)

		# If we have never seen recognition for this gid, do nothing.
		# (This avoids accidental "immediate stale" when initialization order changes.)
		if not _last_seen_sec.has(gid):
			continue

		var last: float = float(_last_seen_sec[gid])
		if (now - last) <= auto_off_seconds:
			continue

		var actor: TrafficLightGroupActor = _get_actor(gid)
		if actor != null:
			actor.set_all_off()

		# Update last seen to avoid turning off every frame after timeout.
		_last_seen_sec[gid] = now

# -----------------------------------------------------------------------------
# Actor life-cycle
# -----------------------------------------------------------------------------

func _get_actor(gid: int) -> TrafficLightGroupActor:
	# Centralized typed lookup (keeps Variant casting in one place).
	return _actors.get(gid, null) as TrafficLightGroupActor

func _get_or_create_actor(gid: int) -> TrafficLightGroupActor:
	var existing: TrafficLightGroupActor = _get_actor(gid)
	if existing != null:
		return existing

	var actor: TrafficLightGroupActor = _instantiate_actor()
	actor.name = "TrafficLightGroup_%d" % gid
	_root.add_child(actor)

	_configure_actor(actor)

	_actors[gid] = actor
	_last_seen_sec[gid] = _now_sec()
	return actor

func _instantiate_actor() -> TrafficLightGroupActor:
	if actor_scene != null:
		return actor_scene.instantiate() as TrafficLightGroupActor
	return TrafficLightGroupActor.new()

func _configure_actor(actor: TrafficLightGroupActor) -> void:
	# Dependency injection / tuning parameters.
	# Using set(...) keeps this manager decoupled from actor's exact export fields.
	actor.set("board_color", default_board_color)
	actor.set("z_offset_board", z_offset_board)
	actor.set("z_offset_bulb", z_offset_bulb)

	if board_scene != null:
		actor.set("board_scene", board_scene)

func _remove_missing_actors(alive: Dictionary) -> void:
	# Remove actors whose gid is not present in the incoming map list.
	var remove_list: Array[int] = []

	for k_any in _actors.keys():
		var gid: int = int(k_any)
		if not alive.has(gid):
			remove_list.append(gid)

	for gid in remove_list:
		_remove_actor(gid)

func _remove_actor(gid: int) -> void:
	var actor: TrafficLightGroupActor = _get_actor(gid)
	if actor == null:
		return

	_actors.erase(gid)
	_last_seen_sec.erase(gid)
	actor.queue_free()

func _ensure_root() -> Node3D:
	# Keeps spawned actors under a single node for easier debugging / cleanup.
	var n := get_node_or_null("ActorsRoot") as Node3D
	if n != null:
		return n

	n = Node3D.new()
	n.name = "ActorsRoot"
	add_child(n)
	return n

func _now_sec() -> float:
	return Time.get_ticks_msec() / 1000.0

func _extract_array(d: Dictionary, key: String) -> Array:
	# Safely extract an Array from a Dictionary without triggering Variant type inference warnings.
	var raw: Variant = d.get(key, null)
	if typeof(raw) == TYPE_ARRAY:
		return raw as Array
	return []
