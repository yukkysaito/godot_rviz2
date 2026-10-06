extends MeshInstance3D

# One layer of the vector map (road surface, road markings) split into tiles: only the tiles
# within the camera's far distance (nothing beyond it is drawn) get a mesh, so large maps stay
# cheap to draw. The tiles use this node's material_override.

@export var build_budget_msec: float = 2.0  # time spent on creating meshes per frame
@export var residency_interval: float = 0.25  # how often the tiles in view are updated [s]

var _vertices: Array[PackedVector3Array] = []  # triangle vertices per tile, relative to its center
var _in_view: TileResidency
var _meshes: TileMeshes
var _residency_timer := 0.0

# tiles: Array of {"center": Vector3, "vertices": PackedVector3Array} (see VectorMap.start_build())
func set_tiles(tiles: Array, tile_size: float) -> void:
	if _meshes != null:
		_meshes.clear()
	mesh = null
	_in_view = TileResidency.new(0.0, tile_size, tile_size)
	_meshes = TileMeshes.new(self, [material_override], _make_mesh, _desired_level)
	_vertices.clear()
	for tile in tiles:
		_in_view.add(tile["center"])
		_meshes.add_tile(tile["center"])
		_vertices.append(tile["vertices"])
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
	var normals := PackedVector3Array()
	normals.resize(vertices.size())
	normals.fill(Vector3.UP)
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	var tile_mesh := ArrayMesh.new()
	tile_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return tile_mesh
