extends MeshInstance3D
class_name DynamicObjectRenderer

# ================== Editor-exposed settings ==================
@export var only_known_object: bool = true
@export var object_3d_model_mode: bool = false

@export var initial_pool: int = 16     # Initial pool size per type (keep small)
@export var pool_growth_step: int = 8  # Grow when shortage happens

# If model's origin is the "center" and you want to place it on the ground.
@export var use_ground_offset: bool = true

# What does the incoming 'position' mean?
# - true  : position == ground contact point
# - false : position == model center
@export var position_is_ground_contact: bool = false

# Rotation unit of incoming data:
# - false: radians (Godot's node.rotation expects radians)
# - true : degrees  (we assign rotation_degrees)
@export var rotation_is_degrees: bool = false

# ================== Internal states ==================
var dynamic_objects := DynamicObjects.new()
var array_mesh := ArrayMesh.new()  # For triangle rendering mode

# pools[type] = {
#   "scene": PackedScene,
#   "pool": Array[Node3D],
#   "used": int  # number of used nodes in the current frame
# }
var pools := {
	"car":        {"scene": preload("res://DynamicObject/Car.tscn")        as PackedScene, "pool": [] as Array[Node3D], "used": 0},
	"pedestrian": {"scene": preload("res://DynamicObject/Pedestrian.tscn") as PackedScene, "pool": [] as Array[Node3D], "used": 0},
	"truck":      {"scene": preload("res://DynamicObject/Truck.tscn")      as PackedScene, "pool": [] as Array[Node3D], "used": 0},
	"trailer":    {"scene": preload("res://DynamicObject/Trailer.tscn")    as PackedScene, "pool": [] as Array[Node3D], "used": 0},
	"bus":        {"scene": preload("res://DynamicObject/Bus.tscn")        as PackedScene, "pool": [] as Array[Node3D], "used": 0},
	"bicycle":    {"scene": preload("res://DynamicObject/Bicycle.tscn")    as PackedScene, "pool": [] as Array[Node3D], "used": 0},
	"motorcycle": {"scene": preload("res://DynamicObject/Motorcycle.tscn") as PackedScene, "pool": [] as Array[Node3D], "used": 0},
}

func _ready() -> void:
	# Subscribe to your object topic
	dynamic_objects.subscribe("/perception/object_recognition/objects", false)

	# Prepare initial pools
	_initialize_pools(initial_pool)

	# Set default mesh for triangle mode (will be reused)
	mesh = array_mesh

# ------------------ Pool management ------------------
func _initialize_pools(count: int) -> void:
	for t in pools.keys():
		_grow_pool(t, count)

func _grow_pool(t: String, count: int) -> void:
	var scene := pools[t]["scene"] as PackedScene
	var pool: Array[Node3D] = pools[t]["pool"]
	for i in range(count):
		var node := scene.instantiate() as Node3D
		node.visible = false
		add_child(node)
		pool.append(node)

# ------------------ Main loop ------------------
func _process(_delta: float) -> void:
	if not dynamic_objects.has_new():
		return

	if object_3d_model_mode:
		_render_models()
	else:
		_render_triangles()

	dynamic_objects.set_old()

# ------------------ Model rendering ------------------
func _render_models() -> void:
	# Clear triangle mesh surface to avoid interference when switching mode
	array_mesh.clear_surfaces()

	# Reset usage counters for all types
	for t in pools.keys():
		pools[t]["used"] = 0

	var objects := dynamic_objects.get_dynamic_object_list(only_known_object)
	for obj in objects:
		var t: String = obj.get("class", "")
		if not pools.has(t):
			continue

		var pool: Array[Node3D] = pools[t]["pool"]
		var used: int = pools[t]["used"]

		# Grow pool if needed
		if used >= pool.size():
			_grow_pool(t, pool_growth_step)

		var node: Node3D = pool[used]

		# Robustly parse vectors (accept Vector3 or {x,y,z} dictionary)
		var pos: Vector3  = _to_v3(obj.get("position",  Vector3.ZERO), Vector3.ZERO)
		var size: Vector3 = _to_v3(obj.get("size",      Vector3.ONE),  Vector3.ONE)
		var rot:  Vector3 = _to_v3(obj.get("rotation",  Vector3.ZERO), Vector3.ZERO)

		# Ground offset handling:
		# If input position is ground point and origin is center -> shift UP by +0.5*size.y
		# If input position is center and you want ground contact   -> shift DOWN by -0.5*size.y
		if use_ground_offset:
			if position_is_ground_contact:
				pos.y += 0.5 * float(size.y)
			else:
				pos.y -= 0.5 * float(size.y)

		# Assign transform
		node.position = pos
		if rotation_is_degrees:
			node.rotation_degrees = rot
		else:
			node.rotation = rot

		node.visible = true
		pools[t]["used"] = used + 1

	# Hide only the unused nodes (minimal work per frame)
	for t in pools.keys():
		var pool2: Array[Node3D] = pools[t]["pool"]
		var used2: int = pools[t]["used"]
		for i in range(used2, pool2.size()):
			var n: Node3D = pool2[i]
			if n.visible:
				n.visible = false

# ------------------ Triangle rendering ------------------
func _render_triangles() -> void:
	# Turn off all models (when switching from model mode)
	for t in pools.keys():
		var pool: Array[Node3D] = pools[t]["pool"]
		pools[t]["used"] = 0
		for n in pool:
			if n.visible:
				n.visible = false

	# Build triangle arrays
	var triangles := dynamic_objects.get_triangle_list(only_known_object)
	var verts := PackedVector3Array()
	var norms := PackedVector3Array()
	verts.clear()
	norms.clear()

	for p in triangles:
		# Fallbacks keep sizes in sync even if 'normal' is missing.
		verts.append(_to_v3(p.get("position", Vector3.ZERO), Vector3.ZERO))
		norms.append(_to_v3(p.get("normal",   Vector3.UP),   Vector3.UP))

	array_mesh.clear_surfaces()

	if not verts.is_empty():
		var arr := []
		arr.resize(Mesh.ARRAY_MAX)
		arr[Mesh.ARRAY_VERTEX] = verts
		arr[Mesh.ARRAY_NORMAL] = norms
		array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)

	# Ensure mesh reference is correct
	if mesh != array_mesh:
		mesh = array_mesh

# ------------------ UI callbacks ------------------
func _on_OnlyKnownObjectCheckButton_toggled(button_pressed: bool) -> void:
	only_known_object = button_pressed

func _on_d_model_object_toggled(toggled_on: bool) -> void:
	object_3d_model_mode = toggled_on

# ------------------ Utilities ------------------
# Accepts either a Vector3 or a Dictionary {x,y,z}; returns 'default' otherwise.
static func _to_v3(v: Variant, default: Vector3) -> Vector3:
	if v is Vector3:
		return v
	if v is Dictionary and v.has("x") and v.has("y") and v.has("z"):
		return Vector3(v["x"], v["y"], v["z"])
	return default
