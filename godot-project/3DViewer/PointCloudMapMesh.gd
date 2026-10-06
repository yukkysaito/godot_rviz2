extends MeshInstance3D

# Point cloud map for large maps.
#
# The map message is converted on a worker thread as soon as it arrives (also while hidden) and
# then released: it is split into tiles, each downsampled at two levels of detail and kept
# compactly by PointCloud. Only tiles that can be seen get a mesh: fine near the ego, coarse up to
# the camera's far distance (nothing beyond it is drawn). So the GPU memory and draw cost stay
# bounded however large the map is. Meshes are created / freed a few per frame, nearest first.
# (The material fades points in with distance, so the map mainly shows as a far skyline.)

@export var visualize_pointcloud_map_toggle: BaseButton

@export var tile_size: float = 50.0  # tile edge length [m]
@export var fine_voxel_size: float = 0.5  # keep one point per voxel [m] near the ego
@export var coarse_voxel_size: float = 1.0  # ... elsewhere
@export var fine_radius: float = 300.0  # tiles within this distance from the ego are fine [m]
# The material blends additively, so fewer points look darker; brighten to compensate
@export var fine_brightness: float = 1.4
@export var coarse_brightness: float = 2.2
@export var build_budget_msec: float = 4.0  # time spent on creating meshes per frame
@export var residency_interval: float = 0.25  # how often the tiles in view are updated [s]

const FINE := 0
const COARSE := 1

var pointcloud: PointCloud = RosBridge.pointcloud_map

var _is_tiling := false
var _residency_timer := 0.0
# Tiles near the ego (fine) and within the camera's far distance (coarse); the margin of one
# tile keeps tiles at the border from switching back and forth
var _near: TileResidency
var _in_view: TileResidency
var _meshes: TileMeshes

func _ready():
	_near = TileResidency.new(fine_radius, tile_size, tile_size)
	_in_view = TileResidency.new(0.0, tile_size, tile_size)
	var materials := []
	for brightness in [fine_brightness, coarse_brightness]:
		var material := material_override
		if material is BaseMaterial3D:
			material = material.duplicate()
			material.albedo_color *= brightness
		materials.append(material)
	_meshes = TileMeshes.new(self, materials, _make_mesh, _desired_level)
	visible = visualize_pointcloud_map_toggle.button_pressed

func _process(delta):
	if pointcloud.has_new() and pointcloud.start_tiles(
			"map", PackedFloat64Array([fine_voxel_size, coarse_voxel_size]), tile_size,
			RosBridge.get_ego_position()):
		pointcloud.set_old()
		_clear_tiles()
		_is_tiling = true
		PerfMonitor.measure_begin("pointcloud_map_build")

	if _is_tiling:
		var tiles := pointcloud.take_tiles()
		for info in tiles:
			# ids match the PointCloud tile ids (tiles are added in order)
			_near.add(info["center"])
			_in_view.add(info["center"])
			_meshes.add_tile(info["center"])
		if not tiles.is_empty():
			_residency_timer = 0.0  # pick up the new tiles right away

	_residency_timer -= delta
	if _residency_timer <= 0.0:
		_residency_timer = residency_interval
		_update_residency()

	_meshes.process(Time.get_ticks_usec() + int(build_budget_msec * 1000.0))

	if _is_tiling and not pointcloud.is_tiling() and _meshes.is_idle():
		_is_tiling = false
		PerfMonitor.measure_end("pointcloud_map_build")
		PerfMonitor.mark("pointcloud_map_built")
		_print_stats()

func _update_residency() -> void:
	if _in_view.size() == 0:
		return
	var camera := get_viewport().get_camera_3d()
	if camera != null:
		_in_view.radius = camera.far
		_mark_changed(_in_view.update(camera.global_position))
	if RosBridge.is_ego_pose_ready():
		_mark_changed(_near.update(RosBridge.get_ego_position()))

func _mark_changed(change: Array) -> void:
	_meshes.mark_dirty(change[0])
	_meshes.mark_dirty(change[1])
	if PerfMonitor.enabled and not _is_tiling and not (change[0].is_empty() and change[1].is_empty()):
		print("[perf] pointcloud map: tiles +%d -%d" % [change[0].size(), change[1].size()])

func _desired_level(id: int) -> int:
	if _near.is_loaded(id):
		return FINE
	if _in_view.is_loaded(id):
		return COARSE
	return -1

func _make_mesh(id: int, level: int) -> Mesh:
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = pointcloud.get_tile_points(id, level)  # relative to the center
	var tile_mesh := ArrayMesh.new()
	tile_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arrays)
	return tile_mesh

func _clear_tiles() -> void:
	_meshes.clear()
	_near.clear()
	_in_view.clear()
	mesh.clear_surfaces()

func _print_stats() -> void:
	if not PerfMonitor.enabled:
		return
	var stats := _meshes.stats(2)
	print("[perf] pointcloud map: %d tiles, fine %d points in %d tiles, coarse %d points in %d tiles" % [
		_in_view.size(), stats["vertices"][FINE], stats["tiles"][FINE],
		stats["vertices"][COARSE], stats["tiles"][COARSE]])

func _on_point_cloud_map_toggle_toggled(toggled_on):
	visible = toggled_on
