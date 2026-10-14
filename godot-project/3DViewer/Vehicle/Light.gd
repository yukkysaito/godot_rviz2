extends Node

# A lamp of a vehicle body, lit by turning on the emission of a material: that of surface `surface`
# of the mesh at `mesh_path` (the node itself by default; a child node can point at a surface of
# the body's mesh). With hide_when_off the mesh shows only while lit (e.g. an overlay over the
# lamp's frame).

@export var mesh_path: NodePath = ^"."
@export var surface: int = 0
@export var hide_when_off: bool = false

@onready var _mesh: MeshInstance3D = get_node(mesh_path)

func _ready() -> void:
	set_lit(false)

func turn_on() -> void:
	set_lit(true)

func turn_off() -> void:
	set_lit(false)

func set_lit(lit: bool) -> void:
	var material := _mesh.get_surface_override_material(surface) as BaseMaterial3D
	if material != null:
		material.emission_enabled = lit
	if hide_when_off:
		_mesh.visible = lit
