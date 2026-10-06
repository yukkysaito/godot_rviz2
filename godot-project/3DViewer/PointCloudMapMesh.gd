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

var pointcloud = PointCloud.new()
var visualize_again = false

var _tiles: Array[MeshInstance3D] = []
var _tile_material: Material

func _ready():
	pointcloud.subscribe("/map/pointcloud_map", true)
	_tile_material = material_override
	if voxel_size > 0.0 and material_override is BaseMaterial3D:
		_tile_material = material_override.duplicate()
		_tile_material.albedo_color *= downsampled_brightness
	visible = visualize_pointcloud_map_toggle.button_pressed

func _process(_delta):
	if not visible:
		return
	if not (pointcloud.has_new() or visualize_again):
		return

	PerfMonitor.measure_begin("pointcloud_map_build")
	_clear_tiles()
	var point_count := 0
	PerfMonitor.measure_begin("pointcloud_map_tiles")
	var tiles: Array = pointcloud.get_pointcloud_tiles("map", voxel_size, tile_size)
	PerfMonitor.measure_end("pointcloud_map_tiles")
	for tile in tiles:
		var points: PackedVector3Array = tile["points"]  # relative to the tile center
		_add_tile(tile["center"], points)
		point_count += points.size()
	PerfMonitor.measure_end("pointcloud_map_build")
	PerfMonitor.mark("pointcloud_map_built")
	if PerfMonitor.enabled:
		print("[perf] pointcloud map: %d points in %d tiles" % [point_count, _tiles.size()])

	visualize_again = false
	pointcloud.set_old()

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
	mesh.clear_surfaces()

func _on_point_cloud_map_toggle_toggled(toggled_on):
	visible = toggled_on
	if not visible:
		_clear_tiles()
	else:
		visualize_again = true
