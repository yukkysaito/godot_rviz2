class_name TrafficLightGroupActor
extends Node3D

# -----------------------------
# Inspector settings
# -----------------------------
@export var board_scene: PackedScene        # Assign Board9SlicePrefab.tscn
@export var board_color: Color = Color(0.1, 0.1, 0.1, 1.0)

# Small offsets along the board/bulb normal to avoid Z-fighting.
# (Negative pushes backward, positive pulls forward)
@export var z_offset_board: float = -0.002
@export var z_offset_bulb: float =  0.000

# -----------------------------
# Public state
# -----------------------------
var group_id: int = -1

# key = "color:arrow" -> Array[TrafficLightBulb]
# Stored as Variant arrays inside Dictionary, so access via helper methods.
var _bulbs_by_key: Dictionary = {}

# Flat list for fast "turn all off"
var _all_bulbs: Array[TrafficLightBulb] = []

# Root node for all generated content (easy rebuild / cleanup)
var _content_root: Node3D = null

# -----------------------------
# Public API
# -----------------------------

func clear_all() -> void:
	_ensure_content_root()

	_bulbs_by_key.clear()
	_all_bulbs.clear()

	# Free all generated children
	for c in _content_root.get_children():
		c.queue_free()

func build_from_hdmap(group: Dictionary) -> void:
	# group = { group_id, traffic_lights:[ {board:{...}, light_bulbs:[...]} ... ] }
	_ensure_content_root()
	clear_all()

	group_id = int(group.get("group_id", -1))

	var tls_any: Variant = group.get("traffic_lights", null)
	if typeof(tls_any) != TYPE_ARRAY:
		# No traffic lights in this group
		return
	var tls: Array = tls_any as Array

	for i in range(tls.size()):
		var tl_any: Variant = tls[i]
		if typeof(tl_any) != TYPE_DICTIONARY:
			continue
		_build_one_traffic_light(tl_any as Dictionary, i)

	# Start with all bulbs off (glow off; lens stays visible)
	set_all_off()

func set_all_off() -> void:
	# Fast path: scan flat array once.
	for b in _all_bulbs:
		if b != null:
			b.turn_off()

func apply_status(status_elements: Array) -> void:
	# Turn off everything once per update, then enable matched bulbs.
	set_all_off()

	for se_any in status_elements:
		if typeof(se_any) != TYPE_DICTIONARY:
			continue
		var se: Dictionary = se_any as Dictionary

		var color: String = str(se.get("color", ""))
		var arrow: String = str(se.get("arrow", "none"))
		var k: String = _key(color, arrow)

		var bulbs_for_key: Array[TrafficLightBulb] = _get_bulbs_by_key(k)
		if bulbs_for_key.is_empty():
			continue

		for b in bulbs_for_key:
			if b != null:
				b.set_lit(true)

# -----------------------------
# Internal helpers
# -----------------------------

func _key(color: String, arrow: String) -> String:
	return "%s:%s" % [color, arrow]

func _ensure_content_root() -> void:
	# build_from_hdmap() may be called before _ready(), so we must lazily create this.
	if _content_root != null:
		return
	_content_root = Node3D.new()
	_content_root.name = "Content"
	add_child(_content_root)

func _get_bulbs_by_key(k: String) -> Array[TrafficLightBulb]:
	# Centralize Variant handling in one place.
	if not _bulbs_by_key.has(k):
		return []
	var arr_any: Variant = _bulbs_by_key[k]
	if typeof(arr_any) != TYPE_ARRAY:
		return []
	var arr: Array = arr_any as Array

	var out: Array[TrafficLightBulb] = []
	out.resize(arr.size())
	var n := 0
	for x in arr:
		var b := x as TrafficLightBulb
		if b != null:
			out[n] = b
			n += 1
	out.resize(n)
	return out

func _register_bulb(k: String, b: TrafficLightBulb) -> void:
	_all_bulbs.append(b)

	if not _bulbs_by_key.has(k):
		_bulbs_by_key[k] = []
	(_bulbs_by_key[k] as Array).append(b)

func _build_one_traffic_light(tl: Dictionary, index: int) -> void:
	var tl_root := Node3D.new()
	tl_root.name = "TrafficLight_%d" % index
	_content_root.add_child(tl_root)

	# --- Board ---
	if tl.has("board"):
		var board_any: Variant = tl["board"]
		if typeof(board_any) == TYPE_DICTIONARY:
			var board_dict: Dictionary = board_any as Dictionary
			var board_center: Dictionary = _board_4pts_to_center_whn(board_dict)
			var board_node := _create_board(board_center)
			if board_node != null:
				tl_root.add_child(board_node)

	# --- Bulbs ---
	var bulbs_any: Variant = tl.get("light_bulbs", null)
	if typeof(bulbs_any) != TYPE_ARRAY:
		return
	var bulbs: Array = bulbs_any as Array

	for bulb_any in bulbs:
		if typeof(bulb_any) != TYPE_DICTIONARY:
			continue
		var bulb_dict: Dictionary = bulb_any as Dictionary

		var bulb_node := _create_bulb(bulb_dict)
		if bulb_node == null:
			continue
		tl_root.add_child(bulb_node)

		# Register for fast lookup by recognition results (color + arrow)
		var color: String = str(bulb_dict.get("color", "none"))
		var arrow: String = str(bulb_dict.get("arrow", "none"))
		var k: String = _key(color, arrow)
		_register_bulb(k, bulb_node)

func _create_board(board: Dictionary) -> Node3D:
	if board_scene == null:
		# If prefab isn't set, we cannot render the board.
		return null

	var pos: Vector3 = board.get("position", Vector3.ZERO)
	var w: float = float(board.get("width", 0.4))
	var h: float = float(board.get("height", 0.9))
	var normal: Vector3 = (board.get("normal", Vector3.FORWARD) as Vector3).normalized()

	var board_node := board_scene.instantiate() as TrafficLightBoard

	# Use the official TrafficLightBoard API when available.
	# (Safer than directly setting board_color if the implementation changes.)
	if board_node.has_method("set_color"):
		board_node.call("set_color", board_color)
	else:
		board_node.board_color = board_color

	board_node.set_size(w, h)

	# Place and orient the board so that its local +Z faces `normal`.
	board_node.position = pos + normal * z_offset_board
	board_node.basis = _basis_from_normal(normal)
	return board_node

func _create_bulb(bulb: Dictionary) -> TrafficLightBulb:
	var pos: Vector3 = bulb.get("position", Vector3.ZERO)
	var r: float = float(bulb.get("radius", 0.05))
	var n: Vector3 = (bulb.get("normal", Vector3.FORWARD) as Vector3).normalized()

	var color: String = str(bulb.get("color", "none"))
	var arrow: String = str(bulb.get("arrow", "none"))

	var b := TrafficLightBulb.new()

	# Avoid global_* usage so this is safe even before entering the scene tree.
	b.setup_from_hdmap(pos + n * z_offset_bulb, n, r, color, arrow)

	# Initial state: glow off (lens stays visible)
	b.turn_off()
	return b

func _basis_from_normal(n: Vector3) -> Basis:
	var normal := n.normalized()

	# Choose an "up" vector that isn't parallel to the normal.
	var up := Vector3.UP
	if abs(normal.dot(up)) > 0.98:
		up = Vector3.RIGHT

	# This makes local +Z face towards `normal`.
	return Basis.looking_at(normal, up)

func _board_4pts_to_center_whn(board: Dictionary) -> Dictionary:
	# Convert 4-corner representation into {position,width,height,normal}
	# Expected keys:
	# - left_top_position, right_top_position, right_bottom_position, left_bottom_position
	var lt: Vector3 = board["left_top_position"]
	var rt: Vector3 = board["right_top_position"]
	var rb: Vector3 = board["right_bottom_position"]
	var lb: Vector3 = board["left_bottom_position"]

	var center: Vector3 = (lt + rt + rb + lb) * 0.25

	# Average top/bottom edge lengths for width
	var w_top: float = (rt - lt).length()
	var w_bottom: float = (rb - lb).length()
	var width: float = (w_top + w_bottom) * 0.5

	# Average left/right edge lengths for height
	var h_left: float = (lb - lt).length()
	var h_right: float = (rb - rt).length()
	var height: float = (h_left + h_right) * 0.5

	var normal: Vector3
	if board.has("normal"):
		normal = (board["normal"] as Vector3).normalized()
	else:
		# Compute normal from the rectangle edges
		var u := (rt - lt).normalized()
		var v := (lb - lt).normalized()
		normal = u.cross(v).normalized()

	return {
		"position": center,
		"width": width,
		"height": height,
		"normal": normal,
	}
