extends Node3D
class_name TrafficLightGroupsManager

# Spawns / destroys TrafficLightGroupActor instances based on HDMap groups,
# then updates their glow state from the perception topic.

@export var actor_scene: PackedScene            # Recommended: TrafficLightGroupActor.tscn
@export var board_scene: PackedScene            # Optional: injected into each actor

@export var default_board_color: Color = Color(0.10, 0.10, 0.10, 1.0)
@export var z_offset_board: float = 0.03
@export var z_offset_bulb: float = 0.00

# If no recognition update arrives for this duration, turn off the glow for that group.
@export var auto_off_seconds: float = 0.7

# Actors are built over several frames so that large maps (hundreds of groups) do not freeze
# the screen: at most this much time is spent on building per frame.
@export var build_budget_msec: float = 4.0

var traffic_light_recognition: TrafficLights = RosBridge.traffic_signals

# gid(int) -> TrafficLightGroupActor
var _actors: Dictionary = {}

# Groups waiting to be built (see set_map)
var _pending_groups: Array = []


@onready var _root: Node3D = _ensure_root()

func _process(_delta: float) -> void:
	if not _pending_groups.is_empty():
		_build_pending_groups()
	# Only groups whose status changed are returned (stale groups come back with no elements),
	# so this stays cheap even for maps with hundreds of traffic lights.
	var stale_seconds := auto_off_seconds if auto_off_seconds > 0.0 else INF
	_apply_status_list(traffic_light_recognition.get_traffic_light_status_changes(stale_seconds))

# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------

func set_map(groups: Array) -> void:
	# Called whenever the HDMap traffic light groups update.
	# This method:
	# 1) creates/updates actors for incoming groups
	# 2) removes actors that no longer exist in the incoming list

	# 3) (once all are built) re-lights them from the latest recognition result
	# Actors are built a few at a time in _process.

	var alive: Dictionary = {}  # Used as a "set": gid -> true
	_pending_groups.clear()

	for g_any in groups:
		if typeof(g_any) != TYPE_DICTIONARY:
			continue
		var group: Dictionary = g_any as Dictionary

		var gid: int = int(group.get("group_id", -1))
		if gid < 0:
			continue

		alive[gid] = true
		_pending_groups.append(group)

	_remove_missing_actors(alive)
	# Build from the end so that popping is cheap
	_pending_groups.reverse()
	PerfMonitor.measure_begin("traffic_lights_build")

func _build_pending_groups() -> void:
	var deadline := Time.get_ticks_usec() + int(build_budget_msec * 1000.0)
	while not _pending_groups.is_empty() and Time.get_ticks_usec() < deadline:
		var group: Dictionary = _pending_groups.pop_back()
		var actor: TrafficLightGroupActor = _get_or_create_actor(int(group["group_id"]))
		actor.build_from_hdmap(group)

		# Ensure the group starts from "all off" at map-update time.
		actor.set_all_off()

	if _pending_groups.is_empty():
		PerfMonitor.measure_end("traffic_lights_build")
		PerfMonitor.mark("traffic_lights_built")
		# Re-light the rebuilt actors from the latest recognition result
		_apply_status_list(traffic_light_recognition.get_traffic_light_status())

# -----------------------------------------------------------------------------
# Recognition update
# -----------------------------------------------------------------------------

func _apply_status_list(status_list: Array) -> void:
	for st_any in status_list:
		if typeof(st_any) != TYPE_DICTIONARY:
			continue
		var status: Dictionary = st_any as Dictionary

		var actor: TrafficLightGroupActor = _get_actor(int(status.get("group_id", -1)))
		if actor == null:
			continue  # Group not loaded / not present

		# status_elements: Array of dictionaries {color, arrow, ...}; empty turns the group off
		actor.apply_status(_extract_array(status, "status_elements"))

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

func _extract_array(d: Dictionary, key: String) -> Array:
	# Safely extract an Array from a Dictionary without triggering Variant type inference warnings.
	var raw: Variant = d.get(key, null)
	if typeof(raw) == TYPE_ARRAY:
		return raw as Array
	return []
