extends Node3D
class_name TrafficLightBoard

@export var corner_size: float = 0.20
@export var base_edge_len: float = 0.20
@export var base_center_size: float = 0.20

@export var corner_mesh: Mesh
@export var edge_mesh: Mesh
@export var center_mesh: Mesh

@export var board_color: Color = Color(0.2, 0.2, 0.2, 1.0)

# Small overlap to avoid cracks between parts (meters).
# 0.0 means no overlap. Try 0.0005 - 0.002 depending on your scale.
@export var seam_overlap: float = 0.001

var _board_mat: StandardMaterial3D

var _corner_tl: MeshInstance3D
var _corner_tr: MeshInstance3D
var _corner_bl: MeshInstance3D
var _corner_br: MeshInstance3D
var _edge_top: MeshInstance3D
var _edge_bottom: MeshInstance3D
var _edge_left: MeshInstance3D
var _edge_right: MeshInstance3D
var _center: MeshInstance3D

func _ready() -> void:
	_build_if_needed()
	# Ensure inspector value is applied even if it was changed before runtime.
	set_color(board_color)

func _build_if_needed() -> void:
	if _corner_tl != null:
		return

	# Create one shared material for all parts (cheap and consistent).
	_board_mat = StandardMaterial3D.new()
	_board_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_board_mat.albedo_color = board_color
	_board_mat.roughness = 1.0
	_board_mat.metallic = 0.0

	_corner_tl = _make_mesh(corner_mesh)
	_corner_tr = _make_mesh(corner_mesh)
	_corner_bl = _make_mesh(corner_mesh)
	_corner_br = _make_mesh(corner_mesh)

	_edge_top = _make_mesh(edge_mesh)
	_edge_bottom = _make_mesh(edge_mesh)
	_edge_left = _make_mesh(edge_mesh)
	_edge_right = _make_mesh(edge_mesh)

	_center = _make_mesh(center_mesh)

	# Reuse one edge mesh by rotating it.
	_edge_top.rotation.z = 0.0
	_edge_bottom.rotation.z = PI
	_edge_left.rotation.z = PI * 0.5
	_edge_right.rotation.z = PI * 1.5

func set_color(c: Color) -> void:
	board_color = c
	_build_if_needed()
	_board_mat.albedo_color = board_color

func _make_mesh(m: Mesh) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	mi.mesh = m
	mi.material_override = _board_mat
	add_child(mi)
	return mi

func set_size(width: float, height: float) -> void:
	_build_if_needed()

	var half_w := width * 0.5
	var half_h := height * 0.5
	var c := corner_size * 0.5

	# Corners (do not scale to keep the rounding shape stable)
	_corner_tl.position = Vector3(-half_w + c,  half_h - c, 0)
	_corner_tr.position = Vector3( half_w - c,  half_h - c, 0)
	_corner_bl.position = Vector3(-half_w + c, -half_h + c, 0)
	_corner_br.position = Vector3( half_w - c, -half_h + c, 0)

	_corner_tl.rotation.z = 0.0
	_corner_tr.rotation.z = PI * 1.5
	_corner_br.rotation.z = PI
	_corner_bl.rotation.z = PI * 0.5

	var mid_w: float = max(width  - corner_size * 2.0, 0.001)
	var mid_h: float = max(height - corner_size * 2.0, 0.001)

	# Edges: extend only scale.x (because the mesh is rotated)
	_edge_top.position    = Vector3(0,  half_h - c, 0)
	_edge_bottom.position = Vector3(0, -half_h + c, 0)
	_edge_left.position   = Vector3(-half_w + c, 0, 0)
	_edge_right.position  = Vector3( half_w - c, 0, 0)

	# Add a tiny overlap to hide cracks.
	var w_scale := (mid_w + seam_overlap * 2.0) / base_edge_len
	var h_scale := (mid_h + seam_overlap * 2.0) / base_edge_len

	_edge_top.scale.x = w_scale
	_edge_bottom.scale.x = w_scale
	_edge_left.scale.x = h_scale
	_edge_right.scale.x = h_scale

	# Center: extend X/Y only. Add the same overlap.
	_center.position = Vector3.ZERO
	_center.scale = Vector3(
		(mid_w + seam_overlap * 2.0) / base_center_size,
		(mid_h + seam_overlap * 2.0) / base_center_size,
		1.0
	)
