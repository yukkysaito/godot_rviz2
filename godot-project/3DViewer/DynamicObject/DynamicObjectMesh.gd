extends MeshInstance3D
class_name DynamicObjectRenderer

# ================== Editor-exposed settings ==================
var ignore_unknown_object: bool
var object_3d_model_mode: bool

@export var initial_pool: int = 16     # Initial pool size per type
@export var pool_growth_step: int = 8  # Pool growth step when shortage occurs

# If model origin is "center" and you want it to sit on the ground.
@export var use_ground_offset: bool = true

# Meaning of incoming 'position':
# - true  : position == ground contact point
# - false : position == model center
@export var position_is_ground_contact: bool = false

# Rotation unit of incoming data:
# - false: radians (Godot's node.rotation expects radians)
# - true : degrees  (we assign rotation_degrees)
@export var rotation_is_degrees: bool = false

# ===== 2D Icon (billboard) settings =====
@export var show_icons: bool = true
const PX_TO_M := 0.001                      # 1 pixel = 0.001 m
const ICON_EXTRA_OFFSET_M := 0.5            # Additional +1.0 m above object height

# ================== Internal state ==================
var dynamic_objects: DynamicObjects = RosBridge.objects
var array_mesh := ArrayMesh.new()  # Surface for triangle mode

# Model pools: pools[type] = { "scene": PackedScene, "pool": Array[Node3D], "used": int }
var pools := {
	"car":        {"scene": preload("res://3DViewer/DynamicObject/Car.tscn")        as PackedScene, "pool": [] as Array[Node3D], "used": 0},
	"pedestrian": {"scene": preload("res://3DViewer/DynamicObject/Pedestrian.tscn") as PackedScene, "pool": [] as Array[Node3D], "used": 0},
	"truck":      {"scene": preload("res://3DViewer/DynamicObject/Truck.tscn")      as PackedScene, "pool": [] as Array[Node3D], "used": 0},
	"trailer":    {"scene": preload("res://3DViewer/DynamicObject/Trailer.tscn")    as PackedScene, "pool": [] as Array[Node3D], "used": 0},
	"bus":        {"scene": preload("res://3DViewer/DynamicObject/Bus.tscn")        as PackedScene, "pool": [] as Array[Node3D], "used": 0},
	"bicycle":    {"scene": preload("res://3DViewer/DynamicObject/Bicycle.tscn")    as PackedScene, "pool": [] as Array[Node3D], "used": 0},
	"motorcycle": {"scene": preload("res://3DViewer/DynamicObject/Motorcycle.tscn") as PackedScene, "pool": [] as Array[Node3D], "used": 0},
}

# Icon textures (type -> Texture2D). Paths follow: res://DynamicObject/Foo.png
const ICON_TEX := {
	"car": preload("res://3DViewer/DynamicObject/Car.png"),
	"pedestrian": preload("res://3DViewer/DynamicObject/Pedestrian.png"),
	"truck": preload("res://3DViewer/DynamicObject/Truck.png"),
	"trailer": preload("res://3DViewer/DynamicObject/Trailer.png"),
	"bus": preload("res://3DViewer/DynamicObject/Bus.png"),
	"bicycle": preload("res://3DViewer/DynamicObject/Bicycle.png"),
	"motorcycle": preload("res://3DViewer/DynamicObject/Motorcycle.png"),
}

# Icon mesh/material shared by all icons of a type: icon_assets[type] = { "mesh", "material" }
var icon_assets := {}

# Icon pools: icon_pools[type] = { "pool": Array[MeshInstance3D], "used": int }
var icon_pools := {
	"car": {"pool": [] as Array[MeshInstance3D], "used": 0},
	"pedestrian": {"pool": [] as Array[MeshInstance3D], "used": 0},
	"truck": {"pool": [] as Array[MeshInstance3D], "used": 0},
	"trailer": {"pool": [] as Array[MeshInstance3D], "used": 0},
	"bus": {"pool": [] as Array[MeshInstance3D], "used": 0},
	"bicycle": {"pool": [] as Array[MeshInstance3D], "used": 0},
	"motorcycle": {"pool": [] as Array[MeshInstance3D], "used": 0},
}

func _ready() -> void:
	# Initialize pools
	_initialize_model_pools(initial_pool)
	_initialize_icon_assets()
	_initialize_icon_pools(initial_pool)

	# Prepare triangle mode mesh
	mesh = array_mesh

	_bind_settings()

# ================== Main loop ==================
func _process(_delta: float) -> void:
	if dynamic_objects.has_new():
		PerfMonitor.mark("dynamic_objects_received")
	if not dynamic_objects.has_new():
		return

	# Fetch once and reuse for both model/triangle and icons
	var objects := dynamic_objects.get_dynamic_object_list(ignore_unknown_object)

	if object_3d_model_mode:
		_render_models(objects)
		if not ignore_unknown_object:
			_render_unknown_triangles() 
	else:
		_render_triangles()

	if show_icons:
		_render_icons(objects)

	_render_predicted_paths()

	dynamic_objects.set_old()

# ================== Predicted paths ==================
const PREDICTED_PATH_WIDTH := 0.35  # [m]
const PREDICTED_PATH_MIN_CONFIDENCE := 0.1
const PREDICTED_PATH_LIFT := 0.05  # above the road [m]

var show_predicted_paths := false
var _predicted_path_instance: MeshInstance3D

func _render_predicted_paths() -> void:
	if _predicted_path_instance == null:
		_predicted_path_instance = MeshInstance3D.new()
		_predicted_path_instance.mesh = ArrayMesh.new()
		_predicted_path_instance.position.y = PREDICTED_PATH_LIFT
		var material := ShaderMaterial.new()
		material.shader = preload("res://3DViewer/Shaders/predicted_path.gdshader")
		material.render_priority = -1
		_predicted_path_instance.material_override = material
		_predicted_path_instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		add_child(_predicted_path_instance)
	var path_mesh := _predicted_path_instance.mesh as ArrayMesh
	path_mesh.clear_surfaces()
	if not show_predicted_paths:
		return
	var paths := dynamic_objects.get_predicted_paths(
		PREDICTED_PATH_WIDTH, PREDICTED_PATH_MIN_CONFIDENCE, ignore_unknown_object)
	var vertices: PackedVector3Array = paths["vertices"]
	if vertices.is_empty():
		return
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_TEX_UV] = paths["uvs"]
	arrays[Mesh.ARRAY_COLOR] = paths["colors"]
	path_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)

# ================== Models ==================
func _render_models(objects: Array) -> void:
	# Clear triangle surfaces when switching from triangle mode
	array_mesh.clear_surfaces()

	_reset_usage_counters()
	for obj in objects:
		var t: String = obj.get("class", "")
		if not pools.has(t):
			continue

		var node := _borrow_model_node(t)
		var pos: Vector3  = _to_v3(obj.get("position",  Vector3.ZERO), Vector3.ZERO)
		var size: Vector3 = _to_v3(obj.get("size",      Vector3.ONE),  Vector3.ONE)
		var rot:  Vector3 = _to_v3(obj.get("rotation",  Vector3.ZERO), Vector3.ZERO)

		# Apply ground offset consistently with "position_is_ground_contact"
		pos = _apply_ground_offset(pos, size)

		# Assign transform
		node.position = pos
		if rotation_is_degrees:
			node.rotation_degrees = rot
		else:
			node.rotation = rot
		node.visible = true

		if t == "pedestrian":
			var vel: Vector3 = _to_v3(obj.get("velocity", Vector3.ZERO), Vector3.ZERO)
			var speed_mps: float = abs(vel.x)  # spec: x is forward speed [m/s]
			# robust fallback: if feeds ever set y/z, use magnitude
			if not (is_zero_approx(vel.y) and is_zero_approx(vel.z)):
				speed_mps = vel.length()
			node.set_meta("speed_mps", speed_mps)


	_hide_unused_nodes()

func _render_triangles() -> void:
	# Disable all model nodes (triangle mode renders the mesh surface only)
	for t in pools.keys():
		_disable_all_in_pool(pools[t]["pool"])
		pools[t]["used"] = 0

	# Reset icon usage (icons will be placed from object list later)
	for t in icon_pools.keys():
		icon_pools[t]["used"] = 0

	# Build triangle arrays
	var triangles := dynamic_objects.get_triangle_list(ignore_unknown_object)
	var verts := PackedVector3Array()
	var norms := PackedVector3Array()

	for p in triangles:
		verts.append(_to_v3(p.get("position", Vector3.ZERO), Vector3.ZERO))
		norms.append(_to_v3(p.get("normal",   Vector3.UP),   Vector3.UP))

	array_mesh.clear_surfaces()
	if not verts.is_empty():
		var arr := []
		arr.resize(Mesh.ARRAY_MAX)
		arr[Mesh.ARRAY_VERTEX] = verts
		arr[Mesh.ARRAY_NORMAL] = norms
		array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)

	if mesh != array_mesh:
		mesh = array_mesh

func _render_unknown_triangles() -> void:
	var triangles := dynamic_objects.get_unknown_object_triangle_list() as Array

	var verts := PackedVector3Array()
	var norms := PackedVector3Array()

	for p in triangles:
		verts.append(_to_v3(p.get("position", Vector3.ZERO), Vector3.ZERO))
		norms.append(_to_v3(p.get("normal",   Vector3.UP),   Vector3.UP))

	# ★ Important:
	# In model mode, array_mesh currently has 0 surfaces (cleared in _render_models()).
	# We add exactly one surface for unknown triangles.
	# In triangle mode, _render_triangles() clears and then adds the "all triangles" surface.
	array_mesh.clear_surfaces()
	if not verts.is_empty():
		var arr := []
		arr.resize(Mesh.ARRAY_MAX)
		arr[Mesh.ARRAY_VERTEX] = verts
		arr[Mesh.ARRAY_NORMAL] = norms
		array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)

	if mesh != array_mesh:
		mesh = array_mesh

# ================== Icons (billboard) ==================
func _render_icons(objects: Array) -> void:
	# Place exactly one icon per dynamic object
	_reset_icon_usage_counters()
	for obj in objects:
		var t: String = obj.get("class", "")
		var pos: Vector3  = _to_v3(obj.get("position",  Vector3.ZERO), Vector3.ZERO)
		var size: Vector3 = _to_v3(obj.get("size",      Vector3.ONE),  Vector3.ONE)

		pos = _apply_ground_offset(pos, size)
		_place_icon_for_object(t, pos, size)

	_hide_unused_icons()

func _place_icon_for_object(t: String, obj_center_pos: Vector3, obj_size: Vector3) -> void:
	# Resolve icon by class; skip if none (e.g. "unknown")
	if not icon_assets.has(t):
		return

	var icon_node := _borrow_icon_node(t)
	var h_m: float = (icon_assets[t]["mesh"] as QuadMesh).size.y

	# Vertical placement: above the object top
	var icon_center_y := obj_center_pos.y + float(obj_size.y) + ICON_EXTRA_OFFSET_M + (0.5 * h_m)

	# Facing is handled by the billboard material (no rotation needed)
	icon_node.position = Vector3(obj_center_pos.x, icon_center_y, obj_center_pos.z)
	icon_node.visible = true

# ================== Pool helpers ==================
func _initialize_model_pools(count: int) -> void:
	for t in pools.keys():
		_grow_model_pool(t, count)

func _grow_model_pool(t: String, count: int) -> void:
	var scene := pools[t]["scene"] as PackedScene
	var pool: Array[Node3D] = pools[t]["pool"]
	for i in range(count):
		var node := scene.instantiate() as Node3D
		node.visible = false
		add_child(node)
		pool.append(node)

func _borrow_model_node(t: String) -> Node3D:
	var pool: Array[Node3D] = pools[t]["pool"]
	var used: int = pools[t]["used"]
	if used >= pool.size():
		_grow_model_pool(t, pool_growth_step)
	var node: Node3D = pool[used]
	pools[t]["used"] = used + 1
	return node

func _hide_unused_nodes() -> void:
	for t in pools.keys():
		var pool: Array[Node3D] = pools[t]["pool"]
		var used: int = pools[t]["used"]
		for i in range(used, pool.size()):
			if pool[i].visible:
				pool[i].visible = false

func _disable_all_in_pool(pool: Array) -> void:
	for n in pool:
		if n.visible:
			n.visible = false

func _reset_usage_counters() -> void:
	for t in pools.keys():
		pools[t]["used"] = 0

# ================== Icon pool helpers ==================
func _initialize_icon_assets() -> void:
	for t in ICON_TEX.keys():
		var tex: Texture2D = ICON_TEX[t]
		var qm := QuadMesh.new()
		qm.size = Vector2(tex.get_size()) * PX_TO_M  # quad size in meters from texture pixels

		var mat := StandardMaterial3D.new()
		mat.albedo_texture = tex
		mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		# Fully camera-facing: a Y-only billboard turns edge-on in top-down (BEV) views
		mat.billboard_mode = BaseMaterial3D.BILLBOARD_ENABLED
		mat.cull_mode = BaseMaterial3D.CULL_BACK
		icon_assets[t] = {"mesh": qm, "material": mat}

func _initialize_icon_pools(count: int) -> void:
	for t in icon_pools.keys():
		_grow_icon_pool(t, count)

func _grow_icon_pool(t: String, count: int) -> void:
	var pool: Array[MeshInstance3D] = icon_pools[t]["pool"]
	for i in range(count):
		var mi := _make_icon_node(t)
		add_child(mi)
		pool.append(mi)

func _make_icon_node(t: String) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	mi.mesh = icon_assets[t]["mesh"]
	mi.material_override = icon_assets[t]["material"]
	# The billboard rotates the quad in the shader; widen the cull bounds so it is not culled
	mi.extra_cull_margin = 1.0
	mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	mi.visible = false
	return mi

func _borrow_icon_node(t: String) -> MeshInstance3D:
	var pool: Array[MeshInstance3D] = icon_pools[t]["pool"]
	var used: int = icon_pools[t]["used"]
	if used >= pool.size():
		_grow_icon_pool(t, pool_growth_step)
	var node := pool[used]
	icon_pools[t]["used"] = used + 1
	return node

func _reset_icon_usage_counters() -> void:
	for t in icon_pools.keys():
		icon_pools[t]["used"] = 0

func _hide_unused_icons() -> void:
	for t in icon_pools.keys():
		var pool: Array[MeshInstance3D] = icon_pools[t]["pool"]
		var used: int = icon_pools[t]["used"]
		for i in range(used, pool.size()):
			if pool[i].visible:
				pool[i].visible = false

# ================== Utilities ==================
# Accepts Vector3 or Dictionary {x,y,z}; returns 'default' otherwise.
func _to_v3(v: Variant, default: Vector3) -> Vector3:
	if v is Vector3:
		return v
	if v is Dictionary and v.has("x") and v.has("y") and v.has("z"):
		return Vector3(v["x"], v["y"], v["z"])
	return default

# Apply consistent ground offset relative to model origin and "position_is_ground_contact".
func _apply_ground_offset(pos: Vector3, size: Vector3) -> Vector3:
	if not use_ground_offset:
		return pos
	var p := pos
	if position_is_ground_contact:
		# Incoming position is ground point; move to model center by +0.5 * height
		p.y += 0.5 * float(size.y)
	else:
		# Incoming position is model center; move down so the model sits on the ground
		p.y -= 0.5 * float(size.y)
	return p

# ------------------ Settings ------------------
func _bind_settings() -> void:
	Settings.bind("view/ignore_unknown_objects", _on_ignore_unknown_object_toggle_toggled)
	Settings.bind("view/object_icons", set_icon_visibility)
	Settings.bind("view/predicted_paths", func(on): show_predicted_paths = on)
	Settings.bind("view/object_mode", func(mode): set_3d_model_mode(mode == "model"))

func set_3d_model_mode(enabled: bool) -> void:
	object_3d_model_mode = enabled

func set_icon_visibility(enabled: bool) -> void:
	show_icons = enabled
	if not show_icons:
		_reset_icon_usage_counters()
		_hide_unused_icons()

func _on_ignore_unknown_object_toggle_toggled(toggled_on):
	ignore_unknown_object = toggled_on
