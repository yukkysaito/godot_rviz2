class_name SeeThroughDepth

# Depth twin of a see-through mesh: a child with the same mesh (and skin) that draws only its depth
# (Shaders/see_through_depth.gdshader), placed earlier in the back to front order of the transparent
# drawing by its sorting offset. The mesh's color drawn after it then shows only where the mesh is
# nearest, so that its parts behind (the box behind a truck's cab, the inside of a vehicle) do not
# show through it, while what is behind it still does, except what is transparent too and drawn
# after the twin.

const NODE_NAME := "SeeThroughDepth"

static var _material: ShaderMaterial

static func is_twin(node: Node) -> bool:
	return node.name == NODE_NAME

# Shows or hides the twin of mesh_instance, drawn `sorting` [m] further back than it (made the
# first time it is shown)
static func set_shown(mesh_instance: MeshInstance3D, shown: bool, sorting: float) -> void:
	var twin := mesh_instance.get_node_or_null(NODE_NAME) as MeshInstance3D
	if twin == null and shown:
		twin = _add_twin(mesh_instance, sorting)
	if twin != null:
		twin.visible = shown

static func _add_twin(mesh_instance: MeshInstance3D, sorting: float) -> MeshInstance3D:
	if _material == null:
		_material = ShaderMaterial.new()
		_material.shader = preload("res://3DViewer/Shaders/see_through_depth.gdshader")
	var twin := MeshInstance3D.new()
	twin.name = NODE_NAME
	twin.mesh = mesh_instance.mesh
	twin.skin = mesh_instance.skin
	twin.custom_aabb = mesh_instance.custom_aabb
	twin.material_override = _material
	twin.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	twin.sorting_offset = mesh_instance.sorting_offset - sorting
	mesh_instance.add_child(twin)
	var skeleton := mesh_instance.get_node_or_null(mesh_instance.skeleton)
	if skeleton != null:
		twin.skeleton = twin.get_path_to(skeleton)
	return twin
