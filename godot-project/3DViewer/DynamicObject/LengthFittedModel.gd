extends Node3D
class_name LengthFittedModel
# A vehicle model fitted to a measured length without squashing it: the front and the rear keep
# their shape, and the section between them is repeated, each copy stretched a little to fill
# the gap. Expects these nodes from the model file (PARTS):
#   Front, Rear: the ends (meshes), with their origins at the seams
#   Front/FrontEnd, Rear/RearEnd: markers at the very front and rear of the body
#   MidBay: one section between the seams (a mesh), with its origin at its center
# A model without them is reported and shown as it is, with its modeled length.
#
# The parts are joined into one skinned mesh (made once per model file), with a bone for the front,
# the rear and every section, and the length is fitted by moving the bones. So the model is one
# mesh like the others: one draw, and DynamicObjectMesh.gd can draw it see-through in one piece.

# A section shorter than this would look squeezed: a shorter model is scaled down as a whole, to
# MIN_SCALE at most
const MIN_BAY_SCALE := 0.7
const MIN_SCALE := 0.8
# Keeps the number of sections while the length jitters around a switch [sections]
const BAY_HYSTERESIS := 0.6

@export var max_length := 20.0  # [m] longer, the sections stretch

const PARTS := ["Front", "Front/FrontEnd", "Rear", "Rear/RearEnd", "MidBay"]
enum { FRONT_BONE, REAR_BONE, FIRST_BAY_BONE }
const HIDDEN := Vector3(1e-4, 1e-4, 1e-4)  # the scale of a section not shown (collapsed)

# Per model file: {"mesh": the joined ArrayMesh, "skin": Skin, "bays": section count}
static var _joined := {}

var _skeleton: Skeleton3D
var _front_length: float  # from the front end to the front seam
var _rear_length: float
var _bay_length: float
var _bays := 0  # sections in the mesh
var _bay_count := 0  # shown
var _length := 0.0

func _ready() -> void:
	var missing := PARTS.filter(func(path: String) -> bool: return get_node_or_null(path) == null)
	var parts: Array[MeshInstance3D] = [
		get_node_or_null("Front") as MeshInstance3D, get_node_or_null("Rear") as MeshInstance3D,
		get_node_or_null("MidBay") as MeshInstance3D]
	if not missing.is_empty() or parts.has(null):
		push_error("%s: cannot fit the length, the model has no %s" % [scene_file_path,
			", ".join(missing) if not missing.is_empty() else "meshes Front, Rear and MidBay"])
		return
	var front := parts[0]
	var rear := parts[1]
	var mid := parts[2]
	_front_length = ($Front/FrontEnd as Node3D).position.x
	_rear_length = -($Rear/RearEnd as Node3D).position.x
	_bay_length = mid.get_aabb().size.x
	var modeled := _front_length + _rear_length + front.position.x - rear.position.x
	if not _joined.has(scene_file_path):
		# As many sections as max_length can show (the hysteresis can keep one more than it rounds to)
		var count := ceili((max_length - _front_length - _rear_length) / _bay_length + BAY_HYSTERESIS)
		_joined[scene_file_path] = _join(front.mesh, rear.mesh, mid.mesh, maxi(count, 1))
	var joined: Dictionary = _joined[scene_file_path]
	_bays = joined["bays"]

	_skeleton = Skeleton3D.new()
	_skeleton.name = "Skeleton"
	for bone in ["Front", "Rear"] + range(_bays).map(func(i: int) -> String: return "Bay%d" % i):
		_skeleton.add_bone(bone)
	add_child(_skeleton)
	var body := MeshInstance3D.new()
	body.name = "Body"
	body.mesh = joined["mesh"]
	body.skin = joined["skin"]
	body.custom_aabb = _bounds(front, rear)  # the mesh's own box has the parts on top of each other
	_skeleton.add_child(body)
	body.skeleton = body.get_path_to(_skeleton)
	for part in parts:
		remove_child(part)
		part.queue_free()
	fit_length(modeled)

# Without a length (<= 0), the model keeps its current one
func fit_length(length: float) -> void:
	if _skeleton == null or length <= 0.0 or absf(length - _length) < 0.01:
		return
	_length = length
	var mid := length - _front_length - _rear_length
	var squash := 1.0
	if mid < MIN_BAY_SCALE * _bay_length:
		mid = MIN_BAY_SCALE * _bay_length
		squash = maxf(length / (_front_length + _rear_length + mid), MIN_SCALE)
		_bay_count = 1
	elif _bay_count == 0 or absf(mid / _bay_length - _bay_count) > BAY_HYSTERESIS:
		_bay_count = clampi(roundi(mid / _bay_length), 1, _bays)
	var bay := mid / _bay_count
	var front_seam := 0.5 * (mid + _rear_length - _front_length)  # the model is centered
	_skeleton.set_bone_pose_position(FRONT_BONE, Vector3(front_seam, 0, 0))
	_skeleton.set_bone_pose_position(REAR_BONE, Vector3(front_seam - mid, 0, 0))
	for i in _bays:
		var bone := FIRST_BAY_BONE + i
		_skeleton.set_bone_pose_position(bone, Vector3(front_seam - (i + 0.5) * bay, 0, 0))
		_skeleton.set_bone_pose_scale(bone, Vector3(bay / _bay_length, 1, 1) if i < _bay_count else HIDDEN)
	scale.x = squash

# The parts in one surface (they have the same material: one per model), each vertex moved by the
# bone of its part: the front, the rear and count sections
static func _join(front: Mesh, rear: Mesh, mid: Mesh, count: int) -> Dictionary:
	var parts := [[front, FRONT_BONE], [rear, REAR_BONE]]
	for i in count:
		parts.append([mid, FIRST_BAY_BONE + i])
	var joined := []
	joined.resize(Mesh.ARRAY_MAX)
	var bones := PackedInt32Array()
	var weights := PackedFloat32Array()
	var vertex_count := 0
	for part in parts:
		var arrays: Array = (part[0] as Mesh).surface_get_arrays(0)
		for type in Mesh.ARRAY_MAX:
			if arrays[type] == null or type == Mesh.ARRAY_BONES or type == Mesh.ARRAY_WEIGHTS:
				continue
			var values = arrays[type]
			if type == Mesh.ARRAY_INDEX:
				var shifted := PackedInt32Array(values)
				for i in shifted.size():
					shifted[i] += vertex_count
				values = shifted
			if joined[type] == null:
				joined[type] = values
			else:
				joined[type].append_array(values)
		var vertices := (arrays[Mesh.ARRAY_VERTEX] as PackedVector3Array).size()
		for v in vertices:
			bones.append_array([part[1], 0, 0, 0])
			weights.append_array([1.0, 0.0, 0.0, 0.0])
		vertex_count += vertices
	joined[Mesh.ARRAY_BONES] = bones
	joined[Mesh.ARRAY_WEIGHTS] = weights
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, joined)
	mesh.surface_set_material(0, mid.surface_get_material(0))
	var skin := Skin.new()
	for bone in FIRST_BAY_BONE + count:
		skin.add_bind(bone, Transform3D.IDENTITY)
	return {"mesh": mesh, "skin": skin, "bays": count}

# Room for the longest model (a stretched one is longer still: some margin)
func _bounds(front: MeshInstance3D, rear: MeshInstance3D) -> AABB:
	var box := front.get_aabb().merge(rear.get_aabb())
	var half := 0.5 * maxf(max_length, _front_length + _rear_length) + 2.0
	return AABB(Vector3(-half, box.position.y, box.position.z), Vector3(2.0 * half, box.size.y, box.size.z))
