extends Node3D
class_name TrafficLightGroupsManager

# Spawns / destroys TrafficLightGroupActor instances for the HDMap groups near the camera,
# then updates their glow state from the perception topic.

@export var actor_scene: PackedScene            # Recommended: TrafficLightGroupActor.tscn
@export var board_scene: PackedScene            # Optional: injected into each actor

@export var default_board_color: Color = Color(0.10, 0.10, 0.10, 1.0)
@export var z_offset_board: float = 0.03
@export var z_offset_bulb: float = 0.00

# If no recognition update arrives for this duration, turn off the glow for that group.
@export var auto_off_seconds: float = 0.7

# Only groups within this distance from the camera get an actor (they are created / freed as the
# camera moves), so that large maps with thousands of groups stay cheap.
@export var view_radius: float = 300.0
# Actors are built over several frames: at most this much time is spent on building per frame.
@export var build_budget_msec: float = 4.0
@export var residency_interval: float = 0.25  # how often the groups in view are updated [s]

var traffic_light_recognition: TrafficLights = RosBridge.traffic_signals

# Per group (index = residency id)
var _groups: Array[Dictionary] = []
var _actors: Array[TrafficLightGroupActor] = []  # null while out of view
var _in_view: TileResidency

var _pending := PackedInt32Array()  # groups that came into view, waiting for their actor
var _pending_head := 0
var _residency_timer := 0.0
var _status := {}  # gid -> latest status elements (also for groups without an actor)
var _gid_to_id := {}  # gid -> residency id

@onready var _root: Node3D = _ensure_root()

func _process(delta: float) -> void:
	# Only groups whose status changed are returned (stale groups come back with no elements),
	# so this stays cheap even for maps with hundreds of traffic lights.
	var stale_seconds := auto_off_seconds if auto_off_seconds > 0.0 else INF
	_apply_status_list(traffic_light_recognition.get_traffic_light_status_changes(stale_seconds))

	if _in_view == null:
		return
	_residency_timer -= delta
	if _residency_timer <= 0.0:
		_residency_timer = residency_interval
		_update_residency()
	_build_pending_actors()

# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------

func set_map(groups: Array) -> void:
	# Called whenever the HDMap traffic light groups update: actors are (re)created for the groups
	# in view, a few per frame.
	_clear_actors()
	_in_view = TileResidency.new(view_radius, view_radius * 0.2, 100.0)
	for g_any in groups:
		if typeof(g_any) != TYPE_DICTIONARY:
			continue
		var group: Dictionary = g_any as Dictionary
		var gid: int = int(group.get("group_id", -1))
		if gid < 0:
			continue
		_gid_to_id[gid] = _in_view.add(_group_position(group))
		_groups.append(group)
		_actors.append(null)
	_residency_timer = 0.0

	# Status of all groups from the latest recognition result
	_apply_status_list(traffic_light_recognition.get_traffic_light_status())
	PerfMonitor.measure_begin("traffic_lights_build")

# -----------------------------------------------------------------------------
# Recognition update
# -----------------------------------------------------------------------------

func _apply_status_list(status_list: Array) -> void:
	for st_any in status_list:
		if typeof(st_any) != TYPE_DICTIONARY:
			continue
		var status: Dictionary = st_any as Dictionary
		var gid := int(status.get("group_id", -1))

		# status_elements: Array of dictionaries {color, arrow, ...}; empty turns the group off
		var elements := _extract_array(status, "status_elements")
		_status[gid] = elements
		var actor := _get_actor(gid)
		if actor != null:
			actor.apply_status(elements)

# -----------------------------------------------------------------------------
# Actor life-cycle
# -----------------------------------------------------------------------------

func _get_actor(gid: int) -> TrafficLightGroupActor:
	var id: int = _gid_to_id.get(gid, -1)
	return _actors[id] if id >= 0 else null

func _update_residency() -> void:
	var camera := get_viewport().get_camera_3d()
	if camera == null:
		return
	var change := _in_view.update(camera.global_position)
	for id in change[1]:
		if _actors[id] != null:
			_actors[id].queue_free()
			_actors[id] = null
	_pending.append_array(change[0])

func _build_pending_actors() -> void:
	var deadline := Time.get_ticks_usec() + int(build_budget_msec * 1000.0)
	while _pending_head < _pending.size() and Time.get_ticks_usec() < deadline:
		var id := _pending[_pending_head]
		_pending_head += 1
		if _in_view.is_loaded(id) and _actors[id] == null:
			_actors[id] = _create_actor(_groups[id])
	if _pending_head > 0 and _pending_head >= _pending.size():
		_pending.clear()
		_pending_head = 0
		PerfMonitor.measure_end("traffic_lights_build")
		PerfMonitor.mark("traffic_lights_built")
		if PerfMonitor.enabled:
			print("[perf] traffic lights: %d of %d groups in view" % [_in_view.loaded_count(), _groups.size()])

func _create_actor(group: Dictionary) -> TrafficLightGroupActor:
	var gid := int(group["group_id"])
	var actor: TrafficLightGroupActor = _instantiate_actor()
	actor.name = "TrafficLightGroup_%d" % gid
	_root.add_child(actor)
	_configure_actor(actor)
	actor.build_from_hdmap(group)  # starts with all bulbs off
	actor.apply_status(_status.get(gid, []))
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

func _clear_actors() -> void:
	for actor in _actors:
		if actor != null:
			actor.queue_free()
	_groups.clear()
	_actors.clear()
	_gid_to_id.clear()
	_pending.clear()
	_pending_head = 0

# Position of a group (for the distance to the camera): the mean of its boards / bulbs
func _group_position(group: Dictionary) -> Vector3:
	var sum := Vector3.ZERO
	var count := 0
	for tl_any in _extract_array(group, "traffic_lights"):
		if typeof(tl_any) != TYPE_DICTIONARY:
			continue
		var tl: Dictionary = tl_any as Dictionary
		var board: Variant = tl.get("board", null)
		if typeof(board) == TYPE_DICTIONARY and board.has("left_top_position"):
			sum += board["left_top_position"]
			count += 1
		for bulb_any in _extract_array(tl, "light_bulbs"):
			if typeof(bulb_any) == TYPE_DICTIONARY and bulb_any.has("position"):
				sum += bulb_any["position"]
				count += 1
	return sum / count if count > 0 else Vector3.ZERO

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
