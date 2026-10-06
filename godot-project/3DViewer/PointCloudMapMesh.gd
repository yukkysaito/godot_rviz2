extends MeshInstance3D

# Point cloud map, downsampled and split into tiles so that large maps stay cheap to draw:
# each tile is its own MeshInstance3D, so tiles outside the view are culled.
# (The material fades points in with distance, so the map mainly shows as a far skyline;
# a coarse voxel size is enough for that.)
#
# The map message is converted on a worker thread as soon as it arrives (also while hidden) and
# then released, so only the tiles are kept. Tiles near the ego come first and get their meshes
# a few per frame.

@export var visualize_pointcloud_map_toggle: BaseButton

@export var voxel_size: float = 0.5  # keep one point per voxel [m] (0: no downsampling)
@export var tile_size: float = 50.0  # tile edge length [m]
# The material blends additively, so fewer points look darker; brighten to compensate
@export var downsampled_brightness: float = 1.4
@export var build_budget_msec: float = 4.0  # time spent on creating tile meshes per frame

var pointcloud: PointCloud = RosBridge.pointcloud_map

var _tiles: Array[MeshInstance3D] = []
var _pending_tiles: Array = []  # tiles waiting for their mesh, in arrival order
var _pending_index := 0
var _is_building := false
var _tile_material: Material

func _ready():
	_tile_material = material_override
	if voxel_size > 0.0 and material_override is BaseMaterial3D:
		_tile_material = material_override.duplicate()
		_tile_material.albedo_color *= downsampled_brightness
	visible = visualize_pointcloud_map_toggle.button_pressed

func _process(_delta):
	if pointcloud.has_new() and pointcloud.start_tiles("map", voxel_size, tile_size, RosBridge.get_ego_position()):
		pointcloud.set_old()
		_clear_tiles()
		_is_building = true
		PerfMonitor.measure_begin("pointcloud_map_build")
	if not _is_building:
		return

	_pending_tiles.append_array(pointcloud.take_tiles())
	_build_pending_tiles()
	if _pending_index >= _pending_tiles.size() and not pointcloud.is_tiling():
		_is_building = false
		_pending_tiles = []
		_pending_index = 0
		PerfMonitor.measure_end("pointcloud_map_build")
		PerfMonitor.mark("pointcloud_map_built")
		if PerfMonitor.enabled:
			var point_count := 0
			for tile in _tiles:
				point_count += tile.mesh.surface_get_array_len(0)
			print("[perf] pointcloud map: %d points in %d tiles" % [point_count, _tiles.size()])

func _build_pending_tiles() -> void:
	var deadline := Time.get_ticks_usec() + int(build_budget_msec * 1000.0)
	while _pending_index < _pending_tiles.size() and Time.get_ticks_usec() < deadline:
		var tile: Dictionary = _pending_tiles[_pending_index]
		_pending_tiles[_pending_index] = null  # drop the reference once the mesh is made
		_pending_index += 1
		_add_tile(tile["center"], tile["points"])  # points are relative to the tile center
		if _tiles.size() == 1:
			PerfMonitor.mark("pointcloud_map_first_tile")

func _add_tile(center: Vector3, points: PackedVector3Array) -> void:
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = points
	var tile_mesh := ArrayMesh.new()
	tile_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arrays)

	var tile := MeshInstance3D.new()
	tile.mesh = tile_mesh
	tile.material_override = _tile_material
	tile.position = center
	tile.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(tile)
	_tiles.append(tile)

func _clear_tiles() -> void:
	for tile in _tiles:
		tile.queue_free()
	_tiles.clear()
	_pending_tiles = []
	_pending_index = 0
	mesh.clear_surfaces()

func _on_point_cloud_map_toggle_toggled(toggled_on):
	visible = toggled_on
