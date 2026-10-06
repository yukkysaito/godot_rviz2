extends MeshInstance3D

# Point cloud map, downsampled and split into tiles so that large maps stay cheap to draw:
# each tile is its own MeshInstance3D, so tiles outside the view are culled.
# (The material fades points in with distance, so the map mainly shows as a far skyline;
# a coarse voxel size is enough for that.)

@export var visualize_pointcloud_map_toggle: BaseButton

@export var voxel_size: float = 0.5  # keep one point per voxel [m] (0: no downsampling)
@export var tile_size: float = 50.0  # tile edge length [m]
# The material blends additively, so fewer points look darker; brighten to compensate
@export var downsampled_brightness: float = 1.4
@export var build_budget_msec: float = 4.0  # time spent on creating tile meshes per frame

var pointcloud: PointCloud = RosBridge.pointcloud_map
var visualize_again = false

var _tiles: Array[MeshInstance3D] = []
var _pending_tiles: Array = []  # tiles waiting for their mesh (built a few per frame)
var _tile_material: Material

func _ready():
	_tile_material = material_override
	if voxel_size > 0.0 and material_override is BaseMaterial3D:
		_tile_material = material_override.duplicate()
		_tile_material.albedo_color *= downsampled_brightness
	visible = visualize_pointcloud_map_toggle.button_pressed

func _process(_delta):
	if not visible:
		return
	# Tiling (transform, downsampling) runs on a worker thread; only the meshes are made here
	if pointcloud.is_tiles_done():
		_apply_tiles(pointcloud.take_tiles())
	if not _pending_tiles.is_empty():
		_build_pending_tiles()
	if (pointcloud.has_new() or visualize_again) and pointcloud.start_tiles("map", voxel_size, tile_size):
		PerfMonitor.measure_begin("pointcloud_map_tiles")
		visualize_again = false
		pointcloud.set_old()

func _apply_tiles(tiles: Array) -> void:
	PerfMonitor.measure_end("pointcloud_map_tiles")
	PerfMonitor.measure_begin("pointcloud_map_build")
	_clear_tiles()
	_pending_tiles = tiles
	_pending_tiles.reverse()  # build from the end so that popping is cheap

func _build_pending_tiles() -> void:
	var deadline := Time.get_ticks_usec() + int(build_budget_msec * 1000.0)
	while not _pending_tiles.is_empty() and Time.get_ticks_usec() < deadline:
		var tile: Dictionary = _pending_tiles.pop_back()
		_add_tile(tile["center"], tile["points"])  # points are relative to the tile center
	if _pending_tiles.is_empty():
		PerfMonitor.measure_end("pointcloud_map_build")
		PerfMonitor.mark("pointcloud_map_built")
		if PerfMonitor.enabled:
			var point_count := 0
			for tile in _tiles:
				point_count += tile.mesh.surface_get_array_len(0)
			print("[perf] pointcloud map: %d points in %d tiles" % [point_count, _tiles.size()])

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
	_pending_tiles = []
	for tile in _tiles:
		tile.queue_free()
	_tiles.clear()
	mesh.clear_surfaces()

func _on_point_cloud_map_toggle_toggled(toggled_on):
	visible = toggled_on
	if not visible:
		_clear_tiles()
	else:
		visualize_again = true
