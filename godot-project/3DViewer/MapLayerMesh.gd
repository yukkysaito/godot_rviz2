extends MeshInstance3D

# One layer of the vector map (road surface, road markings) split into tiles: only the tiles
# within the camera's far distance (nothing beyond it is drawn) get a mesh, so large maps stay
# cheap to draw. The tiles use this node's material_override.

# At night the layer is lit (e.g. by the head lights), keeping its day color as emission. With a
# ShaderMaterial, its "night" parameter is set instead.
@export var lit_at_night: bool = false
@export var night_albedo: Color = Color(0.05, 0.06, 0.08)
@export var build_budget_msec: float = 2.0  # time spent on creating meshes per frame
@export var residency_interval: float = 0.25  # how often the tiles in view are updated [s]

var _vertices: Array[PackedVector3Array] = []  # triangle vertices per tile, relative to its center
var _normals: Array[PackedVector3Array] = []
var _uvs: Array[PackedVector2Array] = []
var _in_view: TileResidency
var _meshes: TileMeshes
var _residency_timer := 0.0
var _day_albedo: Color

func _ready() -> void:
	var shader_material := material_override as ShaderMaterial
	if lit_at_night and shader_material != null:
		Settings.bind("view/day_mode", func(mode): shader_material.set_shader_parameter("night", mode == "night"))
	var material := material_override as BaseMaterial3D
	if lit_at_night and material != null:
		_day_albedo = material.albedo_color
		Settings.bind("view/day_mode", func(mode): _set_night(material, mode == "night"))

func _set_night(material: BaseMaterial3D, night: bool) -> void:
	material.shading_mode = BaseMaterial3D.SHADING_MODE_PER_PIXEL if night else BaseMaterial3D.SHADING_MODE_UNSHADED
	material.albedo_color = night_albedo if night else _day_albedo
	material.emission_enabled = night
	material.emission = _day_albedo

# tiles: Array of {"center": Vector3, "vertices": PackedVector3Array, "normals": PackedVector3Array,
# "uvs": PackedVector2Array} (see VectorMap.start_build())
func set_tiles(tiles: Array, tile_size: float) -> void:
	if _meshes != null:
		_meshes.clear()
	mesh = null
	_in_view = TileResidency.new(0.0, tile_size, tile_size)
	_meshes = TileMeshes.new(self, [material_override], _make_mesh, _desired_level)
	_vertices.clear()
	_normals.clear()
	_uvs.clear()
	for tile in tiles:
		_in_view.add(tile["center"])
		_meshes.add_tile(tile["center"])
		_vertices.append(tile["vertices"])
		_normals.append(tile.get("normals", PackedVector3Array()))
		_uvs.append(tile.get("uvs", PackedVector2Array()))
	_residency_timer = 0.0

func _process(delta: float) -> void:
	if _meshes == null:
		return
	_residency_timer -= delta
	if _residency_timer <= 0.0:
		_residency_timer = residency_interval
		var camera := get_viewport().get_camera_3d()
		if camera != null:
			_in_view.radius = camera.far
			var change := _in_view.update(camera.global_position)
			_meshes.mark_dirty(change[0])
			_meshes.mark_dirty(change[1])
	_meshes.process(Time.get_ticks_usec() + int(build_budget_msec * 1000.0))

func _desired_level(id: int) -> int:
	return 0 if _in_view.is_loaded(id) else -1

func _make_mesh(id: int, _level: int) -> Mesh:
	var vertices := _vertices[id]
	var normals := _normals[id]
	if normals.size() != vertices.size():
		normals = PackedVector3Array()
		normals.resize(vertices.size())
		normals.fill(Vector3.UP)
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	if _uvs[id].size() == vertices.size():
		arrays[Mesh.ARRAY_TEX_UV] = _uvs[id]
	var tile_mesh := ArrayMesh.new()
	tile_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return tile_mesh
