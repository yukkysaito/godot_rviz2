extends MeshInstance3D
class_name DynamicObjectRenderer

# ================== Editor-exposed settings ==================
var ignore_unknown_object: bool
@export var ignore_unknown_object_toggle: BaseButton
@export var icon_visibility_toggle: BaseButton
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

# ===== Frame smoothing (perception arrives ~10Hz; render every frame) =====
@export var smoothing_rate: float = 12.0   # lerp rate multiplier (per second)

# ===== 2D Icon (billboard) settings =====
@export var show_icons: bool = true
@export var icon_bob_amplitude: float = 0.06  # gentle vertical bob [m]
@export var icon_bob_speed: float = 1.8
const PX_TO_M := 0.001                      # 1 pixel = 0.001 m
const ICON_EXTRA_OFFSET_M := 0.5            # Additional +1.0 m above object height

# ===== Holographic look =====
const HOLOGRAM_BOX_SHADER := preload("res://3DViewer/Shaders/obj_hologram_box.gdshader")
const GROUND_RING_SHADER := preload("res://3DViewer/Shaders/obj_ground_ring.gdshader")
const HOLO_CYAN := Color(0.0, 0.898, 1.0)      # #00E5FF
const HOLO_MAGENTA := Color(1.0, 0.176, 0.584) # #FF2D95
const RING_SCALE_FACTOR := 1.3                 # ring diameter vs object footprint
const RING_GROUND_OFFSET := 0.05               # lift above ground to avoid z-fighting

# ================== Internal state ==================
var dynamic_objects := DynamicObjects.new()
var array_mesh := ArrayMesh.new()  # Surface for triangle mode

# Hologram materials (built in _ready; scene material_override is discarded)
var _holo_known_mat := ShaderMaterial.new()
var _holo_unknown_mat := ShaderMaterial.new()
var _ring_known_mat := ShaderMaterial.new()
var _ring_unknown_mat := ShaderMaterial.new()
var _ring_mesh := PlaneMesh.new()  # unit quad lying flat, shared by all rings

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

# Ground ring pool (single pool for all classes)
var _ring_pool: Array[MeshInstance3D] = []
var _ring_used: int = 0

# Smoothing targets — rebuilt on every perception message (~10Hz), consumed
# every rendered frame. Only slots in use this cycle are present.
var _smooth_model_nodes: Array[Node3D] = []
var _smooth_model_pos := PackedVector3Array()
var _smooth_model_rot := PackedVector3Array()  # radians
var _smooth_icon_nodes: Array[MeshInstance3D] = []
var _smooth_icon_pos := PackedVector3Array()
var _smooth_ring_nodes: Array[MeshInstance3D] = []
var _smooth_ring_pos := PackedVector3Array()

var _time: float = 0.0

func _ready() -> void:
	# Subscribe to dynamic object topic
	dynamic_objects.subscribe("/perception/object_recognition/objects", false)

	# Synchronize UI status
	ignore_unknown_object = ignore_unknown_object_toggle.button_pressed
	if icon_visibility_toggle != null:
		show_icons = icon_visibility_toggle.button_pressed

	# Holographic materials: drop the scene-assigned override so per-surface
	# materials (known = cyan / unknown = magenta) take effect.
	material_override = null
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF

	_holo_known_mat.shader = HOLOGRAM_BOX_SHADER
	_holo_known_mat.render_priority = 2  # draw over road / ground overlays
	_holo_unknown_mat.shader = HOLOGRAM_BOX_SHADER
	_holo_unknown_mat.render_priority = 2
	_holo_unknown_mat.set_shader_parameter("edge_color", HOLO_MAGENTA)
	_holo_unknown_mat.set_shader_parameter("face_color", Color(0.35, 0.04, 0.18))

	_ring_known_mat.shader = GROUND_RING_SHADER
	_ring_known_mat.render_priority = 1
	_ring_unknown_mat.shader = GROUND_RING_SHADER
	_ring_unknown_mat.render_priority = 1
	_ring_unknown_mat.set_shader_parameter("ring_color", HOLO_MAGENTA)

	_ring_mesh.size = Vector2(1.0, 1.0)  # unit quad; per-object footprint via node.scale

	# Initialize pools
	_initialize_model_pools(initial_pool)
	_initialize_icon_pools(initial_pool)
	_initialize_ring_pool(initial_pool)

	# Prepare triangle mode mesh
	mesh = array_mesh

# ================== Main loop ==================
func _process(delta: float) -> void:
	_time += delta

	# New perception message: update targets only. Actual motion happens in
	# _update_smoothing() every rendered frame.
	if dynamic_objects.has_new():
		var objects := dynamic_objects.get_dynamic_object_list(ignore_unknown_object)

		if object_3d_model_mode:
			_render_models(objects)
			if not ignore_unknown_object:
				_render_unknown_triangles()
		else:
			_render_triangles()

		if show_icons:
			_render_icons(objects)
		else:
			_clear_icon_targets()

		_render_rings(objects)

		dynamic_objects.set_old()

	_update_smoothing(delta)

# ================== Frame smoothing ==================
func _update_smoothing(delta: float) -> void:
	var alpha := clampf(delta * smoothing_rate, 0.0, 1.0)

	for i in _smooth_model_nodes.size():
		var node := _smooth_model_nodes[i]
		node.position = node.position.lerp(_smooth_model_pos[i], alpha)
		var target_rot := _smooth_model_rot[i]
		node.rotation = Vector3(
			lerp_angle(node.rotation.x, target_rot.x, alpha),
			lerp_angle(node.rotation.y, target_rot.y, alpha),
			lerp_angle(node.rotation.z, target_rot.z, alpha)
		)

	for i in _smooth_icon_nodes.size():
		var icon := _smooth_icon_nodes[i]
		# Lerp toward a gently bobbing target: smoothing + bob in one pass.
		var bob := sin(_time * icon_bob_speed + float(i) * 1.7) * icon_bob_amplitude
		icon.position = icon.position.lerp(_smooth_icon_pos[i] + Vector3(0.0, bob, 0.0), alpha)

	for i in _smooth_ring_nodes.size():
		var ring := _smooth_ring_nodes[i]
		ring.position = ring.position.lerp(_smooth_ring_pos[i], alpha)

func _clear_model_targets() -> void:
	_smooth_model_nodes.clear()
	_smooth_model_pos.clear()
	_smooth_model_rot.clear()

func _clear_icon_targets() -> void:
	_smooth_icon_nodes.clear()
	_smooth_icon_pos.clear()

func _clear_ring_targets() -> void:
	_smooth_ring_nodes.clear()
	_smooth_ring_pos.clear()

# ================== Models ==================
func _render_models(objects: Array) -> void:
	# Clear triangle surfaces when switching from triangle mode
	array_mesh.clear_surfaces()

	_reset_usage_counters()
	_clear_model_targets()
	for obj in objects:
		var t: String = obj.get("class", "")
		if not pools.has(t):
			continue

		var node := _borrow_model_node(t)
		var newly_borrowed := not node.visible
		var pos: Vector3  = _to_v3(obj.get("position",  Vector3.ZERO), Vector3.ZERO)
		var size: Vector3 = _to_v3(obj.get("size",      Vector3.ONE),  Vector3.ONE)
		var rot:  Vector3 = _to_v3(obj.get("rotation",  Vector3.ZERO), Vector3.ZERO)

		# Apply ground offset consistently with "position_is_ground_contact"
		pos = _apply_ground_offset(pos, size)

		# Store target in radians (node.rotation expects radians)
		var rot_rad := rot
		if rotation_is_degrees:
			rot_rad = Vector3(deg_to_rad(rot.x), deg_to_rad(rot.y), deg_to_rad(rot.z))

		# Newly borrowed nodes snap to the target (no lerp from stale pose);
		# already-active nodes glide there in _update_smoothing().
		if newly_borrowed:
			node.position = pos
			node.rotation = rot_rad
		node.visible = true

		_smooth_model_nodes.append(node)
		_smooth_model_pos.append(pos)
		_smooth_model_rot.append(rot_rad)

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
	_clear_model_targets()

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
		array_mesh.surface_set_material(0, _holo_known_mat)

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
		array_mesh.surface_set_material(0, _holo_unknown_mat)

	if mesh != array_mesh:
		mesh = array_mesh

# ================== Icons (billboard) ==================
func _render_icons(objects: Array) -> void:
	# Place exactly one icon per dynamic object
	_reset_icon_usage_counters()
	_clear_icon_targets()
	for obj in objects:
		var t: String = obj.get("class", "")
		var pos: Vector3  = _to_v3(obj.get("position",  Vector3.ZERO), Vector3.ZERO)
		var size: Vector3 = _to_v3(obj.get("size",      Vector3.ONE),  Vector3.ONE)

		pos = _apply_ground_offset(pos, size)
		_place_icon_for_object(t, pos, size)

	_hide_unused_icons()

func _place_icon_for_object(t: String, obj_center_pos: Vector3, obj_size: Vector3) -> void:
	# Resolve texture by class; skip if none
	if not ICON_TEX.has(t):
		return
	var tex: Texture2D = ICON_TEX[t]

	# Borrow icon node for this type
	var icon_node := _borrow_icon_node(t)
	var newly_borrowed := not icon_node.visible

	# Compute quad size in meters from texture pixels
	var tex_size: Vector2i = tex.get_size()
	var w_m := float(tex_size.x) * PX_TO_M
	var h_m := float(tex_size.y) * PX_TO_M

	# Ensure mesh size matches the texture (per-instance)
	var qm := icon_node.mesh as QuadMesh
	if qm.size.x != w_m or qm.size.y != h_m:
		qm.size = Vector2(w_m, h_m)

	# Ensure material is configured and texture is set
	var mat := icon_node.get_active_material(0)
	if mat is StandardMaterial3D:
		var sm := mat as StandardMaterial3D
		if sm.albedo_texture != tex:
			sm.albedo_texture = tex
		# Keep billboard/alpha settings stable
		sm.billboard_mode = BaseMaterial3D.BILLBOARD_FIXED_Y
		sm.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		sm.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		sm.cull_mode = BaseMaterial3D.CULL_BACK

	# Vertical placement:
	var icon_center_y := obj_center_pos.y + float(obj_size.y) + ICON_EXTRA_OFFSET_M + (0.5 * h_m)

	# Target icon position; facing is handled by BILLBOARD_FIXED_Y (no rotation needed)
	var target := Vector3(obj_center_pos.x, icon_center_y, obj_center_pos.z)
	if newly_borrowed:
		icon_node.position = target
	icon_node.visible = true

	_smooth_icon_nodes.append(icon_node)
	_smooth_icon_pos.append(target)

# ================== Ground rings ==================
func _render_rings(objects: Array) -> void:
	# One animated holographic ring under every dynamic object, in both
	# 3D-model and triangle modes.
	_ring_used = 0
	_clear_ring_targets()
	for obj in objects:
		var t: String = obj.get("class", "")
		var pos: Vector3  = _to_v3(obj.get("position",  Vector3.ZERO), Vector3.ZERO)
		var size: Vector3 = _to_v3(obj.get("size",      Vector3.ONE),  Vector3.ONE)

		# Same ground reference as the models use.
		pos = _apply_ground_offset(pos, size)

		var ring := _borrow_ring_node()
		var newly_borrowed := not ring.visible

		var diameter := maxf(size.x, size.z) * RING_SCALE_FACTOR
		ring.scale = Vector3(diameter, 1.0, diameter)
		ring.set_surface_override_material(0, _ring_unknown_mat if t == "unknown" else _ring_known_mat)

		var target := Vector3(pos.x, pos.y + RING_GROUND_OFFSET, pos.z)
		if newly_borrowed:
			ring.position = target
		ring.visible = true

		_smooth_ring_nodes.append(ring)
		_smooth_ring_pos.append(target)

	_hide_unused_rings()

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
func _initialize_icon_pools(count: int) -> void:
	for t in icon_pools.keys():
		_grow_icon_pool(t, count)

func _grow_icon_pool(t: String, count: int) -> void:
	var pool: Array[MeshInstance3D] = icon_pools[t]["pool"]
	for i in range(count):
		var mi := _make_icon_node()
		add_child(mi)
		pool.append(mi)

func _make_icon_node() -> MeshInstance3D:
	# Create a MeshInstance3D with QuadMesh + StandardMaterial3D (billboard)
	var mi := MeshInstance3D.new()

	var qm := QuadMesh.new()
	mi.mesh = qm

	var mat := StandardMaterial3D.new()
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.billboard_mode = BaseMaterial3D.BILLBOARD_FIXED_Y
	mat.vertex_color_use_as_albedo = false
	mat.cull_mode = BaseMaterial3D.CULL_BACK
	mi.set_surface_override_material(0, mat)

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

# ================== Ring pool helpers ==================
func _initialize_ring_pool(count: int) -> void:
	_grow_ring_pool(count)

func _grow_ring_pool(count: int) -> void:
	for i in range(count):
		var mi := MeshInstance3D.new()
		mi.mesh = _ring_mesh
		mi.set_surface_override_material(0, _ring_known_mat)
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		mi.visible = false
		add_child(mi)
		_ring_pool.append(mi)

func _borrow_ring_node() -> MeshInstance3D:
	if _ring_used >= _ring_pool.size():
		_grow_ring_pool(pool_growth_step)
	var node := _ring_pool[_ring_used]
	_ring_used += 1
	return node

func _hide_unused_rings() -> void:
	for i in range(_ring_used, _ring_pool.size()):
		if _ring_pool[i].visible:
			_ring_pool[i].visible = false

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

# ------------------ UI callbacks ------------------
func set_3d_model_mode(enabled: bool) -> void:
	object_3d_model_mode = enabled

func set_icon_visibility(enabled: bool) -> void:
	show_icons = enabled
	if not show_icons:
		_reset_icon_usage_counters()
		_hide_unused_icons()
		_clear_icon_targets()

func _on_ignore_unknown_object_toggle_toggled(toggled_on):
	ignore_unknown_object = toggled_on
