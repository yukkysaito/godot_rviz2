extends MeshInstance3D
class_name DynamicObjectRenderer

# ================== Editor-exposed settings ==================
var ignore_unknown_object: bool
var object_3d_model_mode: bool

@export var initial_pool: int = 16     # Initial pool size per type

# If model origin is "center" and you want it to sit on the ground.
@export var use_ground_offset: bool = true

# Meaning of incoming 'position':
# - true  : position == ground contact point
# - false : position == model center
@export var position_is_ground_contact: bool = false

# Rotation unit of incoming data:
# - false: radians (Godot's node.rotation expects radians)
# - true : degrees
@export var rotation_is_degrees: bool = false

# ===== 2D Icon (billboard) settings =====
@export var show_icons: bool = true
const ICON_SIZE_M := 1.024  # icon width [m] (the icons are square)
const ICON_EXTRA_OFFSET_M := 0.5  # gap between the object top and the icon [m]

# ===== Motion =====
# The objects arrive about 10 times a second. Between them each object moves on with its velocity,
# and what is drawn follows that smoothly, so that the objects glide instead of jumping. An object
# is matched to the one of the previous message by its id, or, without ids (no tracking), by the
# nearest one of the same class. New objects fade in, lost ones fade out.
@export var smoothing_time: float = 0.08  # [s] time constant of following the measured pose
@export var fade_time: float = 0.3  # [s]
@export var match_distance: float = 2.0  # [m] for matching without ids, plus 0.3 s of travel
@export var snap_distance: float = 5.0  # [m] farther jumps are not smoothed
# [deg] larger turns between two measurements are not smoothed: an object does not turn that much
# in a frame of the perception, so it was seen the other way round (e.g. a car's front and back
# taken for each other), and spinning it around to there would look like a real turn
@export var snap_turn: float = 120.0
@export var max_extrapolation: float = 0.5  # [s] without a new message, the objects stop there

# Opacity of the shown models: below 1 they are drawn see-through
@export_range(0.0, 1.0) var model_opacity: float = 0.8

# ================== Internal state ==================
var dynamic_objects: DynamicObjects = RosBridge.objects
var array_mesh := ArrayMesh.new()  # Surface for triangle mode

# Model pools: pools[type] = { "scene": PackedScene, "free": Array[Node3D] }
var pools := {
	"car":        {"scene": preload("res://3DViewer/DynamicObject/Car.tscn")        as PackedScene, "free": [] as Array[Node3D]},
	"pedestrian": {"scene": preload("res://3DViewer/DynamicObject/Pedestrian.tscn") as PackedScene, "free": [] as Array[Node3D]},
	"truck":      {"scene": preload("res://3DViewer/DynamicObject/Truck.tscn")      as PackedScene, "free": [] as Array[Node3D]},
	"bus":        {"scene": preload("res://3DViewer/DynamicObject/Bus.tscn")        as PackedScene, "free": [] as Array[Node3D]},
	"bicycle":    {"scene": preload("res://3DViewer/DynamicObject/Bicycle.tscn")    as PackedScene, "free": [] as Array[Node3D]},
	"motorcycle": {"scene": preload("res://3DViewer/DynamicObject/Motorcycle.tscn") as PackedScene, "free": [] as Array[Node3D]},
}

# Classes drawn as another: a trailer looks like a truck, and the detection often switches between them
const SAME_AS := {"trailer": "truck"}

# Icon textures (type -> Texture2D). Paths follow: res://DynamicObject/Foo.png
const ICON_TEX := {
	"car": preload("res://3DViewer/DynamicObject/Car.png"),
	"pedestrian": preload("res://3DViewer/DynamicObject/Pedestrian.png"),
	"truck": preload("res://3DViewer/DynamicObject/Truck.png"),
	"bus": preload("res://3DViewer/DynamicObject/Bus.png"),
	"bicycle": preload("res://3DViewer/DynamicObject/Bicycle.png"),
	"motorcycle": preload("res://3DViewer/DynamicObject/Motorcycle.png"),
}

# While fading or see-through, the models are drawn with transparent copies of their materials
# (shared per material), which still write depth so that the model's own parts hide each other
var _see_through_materials := {}  # Material -> Material
var _default_material := StandardMaterial3D.new()

# Icon mesh/material shared by all icons of a type: icon_assets[type] = { "mesh", "material" }
var icon_assets := {}
var _free_icons := {}  # type -> Array[MeshInstance3D]

# One object followed over the messages
class Track:
	var id: String
	var type: String
	var model: Node3D  # null in triangle mode
	var meshes: Array[MeshInstance3D] = []  # of the model, for the opacity
	var icon: MeshInstance3D  # null without icon
	var position: Vector3  # drawn
	var rotation: Vector3  # drawn
	var target: Vector3  # measured, moved on with the velocity
	var target_rotation: Vector3
	var velocity: Vector3  # world [m/s]
	var speed: float
	var size: Vector3
	var length: float  # drawn, for the models fitted to it
	var age: float  # since the last message [s]
	var fade: float = 0.0
	var see_through := false  # has the see-through materials
	var alive := true
	var matched := false

var _tracks: Array[Track] = []

func _ready() -> void:
	for t in pools.keys():
		for i in initial_pool:
			pools[t]["free"].append(_new_model(t))
	_initialize_icon_assets()

	# Prepare triangle mode mesh
	mesh = array_mesh

	_bind_settings()

# ================== Main loop ==================
func _process(delta: float) -> void:
	if dynamic_objects.has_new():
		PerfMonitor.mark("dynamic_objects_received")
		var objects := dynamic_objects.get_dynamic_object_list(ignore_unknown_object)
		_update_tracks(objects)
		if object_3d_model_mode:
			if ignore_unknown_object:
				array_mesh.clear_surfaces()
			else:
				_render_unknown_triangles()
		else:
			_render_triangles()
		dynamic_objects.set_old()
	_advance_tracks(delta)

# ================== Tracks ==================
func _update_tracks(objects: Array) -> void:
	for track in _tracks:
		track.matched = false
	# By id first, then the nearest unmatched track of the same class
	var by_id := {}
	for track in _tracks:
		if track.alive and track.id != "":
			by_id[track.type + track.id] = track
	var rest: Array[Dictionary] = []
	for obj in objects:
		var t := _object_class(obj)
		if not pools.has(t):
			continue
		var id := _object_id(obj)
		var track: Track = by_id.get(t + id) if id != "" else null
		if track != null and not track.matched:
			_measure(track, obj)
		else:
			rest.append(obj)
	for obj in rest:
		var t := _object_class(obj)
		var id := _object_id(obj)
		var position := _object_position(obj)
		var best: Track = null
		var best_distance := INF
		for track in _tracks:
			if not track.alive or track.matched or track.type != t:
				continue
			if id != "" and track.id != "" and id != track.id:
				continue
			var distance := track.target.distance_to(position)
			if distance < match_distance + track.speed * 0.3 and distance < best_distance:
				best = track
				best_distance = distance
		if best == null:
			best = Track.new()
			best.type = t
			_tracks.append(best)
			_measure(best, obj)
			best.position = best.target
			best.rotation = best.target_rotation
			best.length = best.size.x
		else:
			_measure(best, obj)
	for track in _tracks:
		if not track.matched:
			track.alive = false

func _measure(track: Track, obj: Dictionary) -> void:
	track.matched = true
	track.id = _object_id(obj)
	track.size = _to_v3(obj.get("size", Vector3.ONE), Vector3.ONE)
	track.target = _object_position(obj)
	var rotation := _to_v3(obj.get("rotation", Vector3.ZERO), Vector3.ZERO)
	if rotation_is_degrees:
		rotation = rotation * (PI / 180.0)
	track.target_rotation = rotation
	# The velocity is in the object's frame (x: forward)
	var velocity := _to_v3(obj.get("velocity", Vector3.ZERO), Vector3.ZERO)
	track.velocity = Basis.from_euler(rotation) * velocity
	track.speed = velocity.length()
	track.age = 0.0
	if track.position.distance_to(track.target) > snap_distance:
		track.position = track.target
		track.rotation = rotation
	elif absf(angle_difference(track.rotation.y, rotation.y)) > deg_to_rad(snap_turn):
		track.rotation = rotation

func _advance_tracks(delta: float) -> void:
	var follow := 1.0 - exp(-delta / maxf(smoothing_time, 0.001))
	var index := 0
	while index < _tracks.size():
		var track := _tracks[index]
		if track.alive and track.age < max_extrapolation:
			track.target += track.velocity * delta
		track.age += delta
		track.position = track.position.lerp(track.target, follow)
		track.length = lerpf(track.length, track.size.x, follow)
		track.rotation = Vector3(lerp_angle(track.rotation.x, track.target_rotation.x, follow),
			lerp_angle(track.rotation.y, track.target_rotation.y, follow),
			lerp_angle(track.rotation.z, track.target_rotation.z, follow))
		track.fade = move_toward(track.fade, 1.0 if track.alive else 0.0, delta / maxf(fade_time, 0.001))
		if not track.alive and track.fade <= 0.0:
			_release(track)
			_tracks.remove_at(index)
			continue
		_draw(track)
		index += 1

func _draw(track: Track) -> void:
	if object_3d_model_mode and track.model == null:
		track.model = _take_model(track.type)
		track.meshes = []
		_collect_meshes(track.model, track.meshes)
		track.see_through = false
	elif not object_3d_model_mode and track.model != null:
		_release_model(track)
	var wants_icon := show_icons and icon_assets.has(track.type)
	if wants_icon and track.icon == null:
		track.icon = _take_icon(track.type)
	elif not wants_icon and track.icon != null:
		_release_icon(track)

	if track.model != null:
		track.model.position = track.position
		track.model.rotation = track.rotation
		track.model.set_meta("speed_mps", track.speed)  # for PedestrianAnimator
		if track.model is LengthFittedModel:
			(track.model as LengthFittedModel).fit_length(track.length)
		# Transparent only when not fully opaque (it costs sorting), as modeled otherwise
		var opacity := track.fade * model_opacity
		var see_through := opacity < 1.0
		if see_through or see_through != track.see_through:
			for mesh_instance in track.meshes:
				if see_through != track.see_through:
					_set_see_through(mesh_instance, see_through)
				mesh_instance.transparency = 1.0 - opacity
			track.see_through = see_through
		track.model.visible = true
	if track.icon != null:
		# Above the object top; the icon shader turns it to the camera
		var height: float = (icon_assets[track.type]["mesh"] as QuadMesh).size.y
		track.icon.position = track.position + Vector3.UP * (track.size.y + ICON_EXTRA_OFFSET_M + 0.5 * height)
		track.icon.transparency = 1.0 - track.fade
		track.icon.visible = true

func _release(track: Track) -> void:
	if track.model != null:
		_release_model(track)
	if track.icon != null:
		_release_icon(track)

func _object_class(obj: Dictionary) -> String:
	var t: String = obj.get("class", "")
	return SAME_AS.get(t, t)

# The tracking id, "" without one (all zero when the objects are not tracked)
func _object_id(obj: Dictionary) -> String:
	var id: String = obj.get("id", "")
	return "" if id.replace("0", "") == "" else id

func _object_position(obj: Dictionary) -> Vector3:
	var size := _to_v3(obj.get("size", Vector3.ONE), Vector3.ONE)
	return _apply_ground_offset(_to_v3(obj.get("position", Vector3.ZERO), Vector3.ZERO), size)

# ================== Triangles ==================
func _render_triangles() -> void:
	_set_triangles(dynamic_objects.get_triangle_list(ignore_unknown_object))

func _render_unknown_triangles() -> void:
	_set_triangles(dynamic_objects.get_unknown_object_triangle_list())

func _set_triangles(triangles: Array) -> void:
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

# ================== Model pool ==================
func _new_model(t: String) -> Node3D:
	var node := (pools[t]["scene"] as PackedScene).instantiate() as Node3D
	node.visible = false
	add_child(node)
	return node

func _take_model(t: String) -> Node3D:
	var free: Array[Node3D] = pools[t]["free"]
	return free.pop_back() if not free.is_empty() else _new_model(t)

func _release_model(track: Track) -> void:
	if track.see_through:
		for mesh_instance in track.meshes:
			_set_see_through(mesh_instance, false)
			mesh_instance.transparency = 0.0
		track.see_through = false
	track.model.visible = false
	pools[track.type]["free"].append(track.model)
	track.model = null
	track.meshes = []

func _collect_meshes(node: Node, out: Array[MeshInstance3D]) -> void:
	if node is MeshInstance3D:
		out.append(node as MeshInstance3D)
	for child in node.get_children():
		_collect_meshes(child, out)

func _set_see_through(mesh_instance: MeshInstance3D, see_through: bool) -> void:
	for i in mesh_instance.mesh.get_surface_count():
		var material := mesh_instance.mesh.surface_get_material(i)
		mesh_instance.set_surface_override_material(i, _see_through_material(material) if see_through else null)

func _see_through_material(material: Material) -> Material:
	if material == null:
		material = _default_material  # what a surface without a material is drawn with
	if not material is BaseMaterial3D:
		return material
	if not _see_through_materials.has(material):
		var copy := material.duplicate() as BaseMaterial3D
		copy.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		copy.depth_draw_mode = BaseMaterial3D.DEPTH_DRAW_ALWAYS
		_see_through_materials[material] = copy
	return _see_through_materials[material]

# ================== Icon pool ==================
func _initialize_icon_assets() -> void:
	for t in ICON_TEX.keys():
		var tex: Texture2D = ICON_TEX[t]
		var qm := QuadMesh.new()
		qm.size = Vector2(ICON_SIZE_M, ICON_SIZE_M * tex.get_height() / tex.get_width())

		var mat := ShaderMaterial.new()
		mat.shader = preload("res://3DViewer/Shaders/object_icon.gdshader")
		mat.set_shader_parameter("icon", tex)
		# After the see-through models, which would otherwise be drawn over the icons in front of them
		mat.render_priority = 1
		icon_assets[t] = {"mesh": qm, "material": mat}
		_free_icons[t] = [] as Array[MeshInstance3D]

func _take_icon(t: String) -> MeshInstance3D:
	var free: Array[MeshInstance3D] = _free_icons[t]
	if not free.is_empty():
		return free.pop_back()
	var mi := MeshInstance3D.new()
	mi.mesh = icon_assets[t]["mesh"]
	mi.material_override = icon_assets[t]["material"]
	# The billboard rotates the quad in the shader; widen the cull bounds so it is not culled
	mi.extra_cull_margin = 1.0
	mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(mi)
	return mi

func _release_icon(track: Track) -> void:
	track.icon.visible = false
	_free_icons[track.type].append(track.icon)
	track.icon = null

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
	Settings.bind("view/object_mode", func(mode): set_3d_model_mode(mode == "model"))

func set_3d_model_mode(enabled: bool) -> void:
	object_3d_model_mode = enabled

func set_icon_visibility(enabled: bool) -> void:
	show_icons = enabled

func _on_ignore_unknown_object_toggle_toggled(toggled_on):
	ignore_unknown_object = toggled_on
