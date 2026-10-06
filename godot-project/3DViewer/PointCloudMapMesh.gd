extends MeshInstance3D

# Point cloud map for large maps.
#
# The map message is converted on a worker thread as soon as it arrives (also while hidden) and
# then released: it is split into tiles, each downsampled at two levels of detail and kept
# compactly by PointCloud. Every tile gets a coarse mesh (the far skyline of the whole map), and
# only tiles around the ego also get a fine mesh, so the GPU memory and draw cost stay bounded
# however large the map is. Meshes are created a few per frame, nearest first.
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
@export var residency_interval: float = 0.25  # how often the fine tiles are updated [s]

const FINE := 0
const COARSE := 1

var pointcloud: PointCloud = RosBridge.pointcloud_map

var _materials: Array[Material] = []
var _is_tiling := false

# Per tile (index = tile id)
var _centers := PackedVector3Array()
var _coarse: Array[MeshInstance3D] = []  # null until created
var _fine: Array[MeshInstance3D] = []  # null unless near the ego
var _grid := {}  # Vector2i (tile index in ROS x/y) -> tile id

var _coarse_queue := PackedInt32Array()  # tiles waiting for their coarse mesh, nearest first
var _coarse_queue_index := 0
var _fine_queue := PackedInt32Array()  # tiles near the ego waiting for their fine mesh
var _fine_loaded := {}  # tile id -> true
var _residency_timer := 0.0

func _ready():
	for brightness in [fine_brightness, coarse_brightness]:
		var material := material_override
		if material is BaseMaterial3D:
			material = material.duplicate()
			material.albedo_color *= brightness
		_materials.append(material)
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
		_add_tiles(pointcloud.take_tiles())

	_residency_timer -= delta
	if _residency_timer <= 0.0:
		_residency_timer = residency_interval
		_update_fine_tiles()

	var deadline := Time.get_ticks_usec() + int(build_budget_msec * 1000.0)
	_build_fine_meshes(deadline)
	_build_coarse_meshes(deadline)

	if _is_tiling and not pointcloud.is_tiling() and _coarse_queue_index >= _coarse_queue.size():
		_is_tiling = false
		PerfMonitor.measure_end("pointcloud_map_build")
		PerfMonitor.mark("pointcloud_map_built")
		_print_stats()

func _add_tiles(tiles: Array) -> void:
	for info in tiles:
		var id: int = info["id"]
		_centers.append(info["center"])
		_coarse.append(null)
		_fine.append(null)
		_grid[info["grid"]] = id
		_coarse_queue.append(id)

func _build_coarse_meshes(deadline: int) -> void:
	while _coarse_queue_index < _coarse_queue.size() and Time.get_ticks_usec() < deadline:
		var id := _coarse_queue[_coarse_queue_index]
		_coarse_queue_index += 1
		_coarse[id] = _create_tile_mesh(id, COARSE)
		_coarse[id].visible = _fine[id] == null
		if _coarse_queue_index == 1:
			PerfMonitor.mark("pointcloud_map_first_tile")

func _build_fine_meshes(deadline: int) -> void:
	while not _fine_queue.is_empty() and Time.get_ticks_usec() < deadline:
		var id := _fine_queue[0]
		_fine_queue.remove_at(0)
		if not _fine_loaded.has(id) or _fine[id] != null:
			continue  # left the radius meanwhile
		_fine[id] = _create_tile_mesh(id, FINE)
		if _coarse[id] != null:
			_coarse[id].visible = false

# Loads the fine level of the tiles around the ego and unloads it from tiles that are far away
# (with a margin of one tile, so tiles at the border do not switch back and forth).
func _update_fine_tiles() -> void:
	if _centers.is_empty() or not RosBridge.is_ego_pose_ready():
		return
	var ego := RosBridge.get_ego_position()
	var ego_flat := Vector2(ego.x, ego.z)

	var unload_distance := fine_radius + tile_size
	var unloaded := 0
	for id in _fine_loaded.keys():
		if Vector2(_centers[id].x, _centers[id].z).distance_to(ego_flat) > unload_distance:
			unloaded += 1
			_fine_loaded.erase(id)
			if _fine[id] != null:
				_fine[id].queue_free()
				_fine[id] = null
			if _coarse[id] != null:
				_coarse[id].visible = true

	var added := []
	for id in _tiles_around(ego):
		if _fine_loaded.has(id):
			continue
		var distance := Vector2(_centers[id].x, _centers[id].z).distance_to(ego_flat)
		if distance <= fine_radius:
			_fine_loaded[id] = true
			added.append([distance, id])
	if PerfMonitor.enabled and (unloaded > 0 or not added.is_empty()):
		print("[perf] pointcloud map: fine tiles +%d -%d (%d near the ego)" % [
			added.size(), unloaded, _fine_loaded.size()])
	if added.is_empty():
		return
	added.sort_custom(func(a, b): return a[0] < b[0])
	for entry in added:
		_fine_queue.append(entry[1])

# Tiles whose grid cell may be within fine_radius of position
func _tiles_around(position: Vector3) -> Array:
	var reach := ceili(fine_radius / tile_size)
	if (2 * reach + 1) * (2 * reach + 1) >= _centers.size():
		return range(_centers.size())
	# Godot (x, z) -> ROS (x, y) = (x, -z), see ros2_to_godot()
	var center := Vector2i(floori(position.x / tile_size), floori(-position.z / tile_size))
	var ids := []
	for gx in range(center.x - reach, center.x + reach + 1):
		for gy in range(center.y - reach, center.y + reach + 1):
			var id: int = _grid.get(Vector2i(gx, gy), -1)
			if id >= 0:
				ids.append(id)
	return ids

func _create_tile_mesh(id: int, level: int) -> MeshInstance3D:
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = pointcloud.get_tile_points(id, level)  # relative to the center
	var tile_mesh := ArrayMesh.new()
	tile_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arrays)

	var tile := MeshInstance3D.new()
	tile.mesh = tile_mesh
	tile.material_override = _materials[level]
	tile.position = _centers[id]
	tile.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(tile)
	return tile

func _clear_tiles() -> void:
	for tile in _coarse + _fine:
		if tile != null:
			tile.queue_free()
	_centers.clear()
	_coarse.clear()
	_fine.clear()
	_grid.clear()
	_coarse_queue.clear()
	_coarse_queue_index = 0
	_fine_queue.clear()
	_fine_loaded.clear()
	mesh.clear_surfaces()

func _print_stats() -> void:
	if not PerfMonitor.enabled:
		return
	var counts := [0, 0]
	for id in _centers.size():
		if _coarse[id] != null:
			counts[COARSE] += _coarse[id].mesh.surface_get_array_len(0)
		if _fine[id] != null:
			counts[FINE] += _fine[id].mesh.surface_get_array_len(0)
	print("[perf] pointcloud map: %d tiles, coarse %d points, fine %d points in %d tiles" % [
		_centers.size(), counts[COARSE], counts[FINE], _fine_loaded.size()])

func _on_point_cloud_map_toggle_toggled(toggled_on):
	visible = toggled_on
