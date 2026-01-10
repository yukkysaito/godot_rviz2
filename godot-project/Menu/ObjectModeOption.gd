extends OptionButton
class_name DynamicObjectRenderModeOption

@export var dynamic_object_mesh_path: NodePath
@onready var dynamic_object_mesh: Node = get_node(dynamic_object_mesh_path)

enum Preset {
	MODEL_3D,
	GEOMETRY
}

var _suppress := false

func _ready() -> void:
	_setup_items()
	item_selected.connect(_on_item_selected)

	# Synchronize initial state (default: 3D Model)
	_suppress = true
	select(_find_index_by_id(Preset.MODEL_3D))
	_apply_preset(Preset.MODEL_3D)
	_suppress = false

func _setup_items() -> void:
	clear()
	add_item("3D Model", Preset.MODEL_3D)
	add_item("Geometry", Preset.GEOMETRY)

func _on_item_selected(index: int) -> void:
	# Ignore callbacks triggered by programmatic changes (select/apply)
	if _suppress:
		return
	var preset := get_item_id(index)
	_apply_preset(preset)

func _apply_preset(preset: int) -> void:
	if dynamic_object_mesh == null:
		return

	# Guard: target node must implement the expected API
	if not dynamic_object_mesh.has_method("set_3d_model_mode"):
		push_warning("DynamicObjectMesh does not implement set_3d_model_mode(enabled)")
		return

	match preset:
		Preset.MODEL_3D:
			dynamic_object_mesh.call("set_3d_model_mode", true)
		Preset.GEOMETRY:
			dynamic_object_mesh.call("set_3d_model_mode", false)

func _find_index_by_id(id: int) -> int:
	for i in range(item_count):
		if get_item_id(i) == id:
			return i
	return 0
