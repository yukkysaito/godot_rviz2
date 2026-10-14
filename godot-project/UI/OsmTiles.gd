extends Node2D

# OpenStreetMap tiles around the mini map's view (UI/MiniMap.gd), placed in its 2D world (the map
# frame's x and Godot's z [m]) with the map's projection (RosBridge.map_projection).
# Following the tile usage policy of openstreetmap.org, the tiles are requested a few at a time
# with an identifying User-Agent and kept on disk (user://osm_tiles), so each one is downloaded
# once. After a failed download (e.g. without a connection) the downloads pause for a while.

const URL := "https://tile.openstreetmap.org/%d/%d/%d.png"
const USER_AGENT := "User-Agent: godot_rviz2 (https://github.com/yukkysaito/godot_rviz2)"
const CACHE_DIR := "user://osm_tiles"
const MAX_DOWNLOADS := 2  # at a time
const RETRY_DELAY := 30.0  # [s] without downloads after a failed one
const MAX_DISK_LOADS := 4  # per frame
const MAX_TILES := 160  # kept in memory
const MAX_LEVEL := 19
const TILE_PIXELS := 256.0
const EARTH_CIRCUMFERENCE := 40075016.686  # [m] at the equator

var _projection: MapProjection = RosBridge.map_projection
var _tiles := {}  # Vector3i(level, x, y) -> Polygon2D, null until loaded
var _wanted: Array[Vector3i] = []  # tiles in view without a picture yet, the nearest first
var _idle_downloads: Array[HTTPRequest] = []
var _retry_time := 0.0  # [s] (engine time) from when downloads are tried again
var _shown_in_view := 0  # tiles with a picture in the last view
var _projection_revision := -1  # of the projection the tiles were placed with

func _ready() -> void:
	DirAccess.make_dir_recursive_absolute(CACHE_DIR)
	for i in MAX_DOWNLOADS:
		var download := HTTPRequest.new()
		download.timeout = 10.0
		add_child(download)
		_idle_downloads.append(download)

# True if the last view had tiles with a picture (false e.g. offline, before they were downloaded)
func is_showing_tiles() -> bool:
	return _shown_in_view > 0

# Loads the tiles within `radius` [m] of `center` [m], at the level whose pixels are about the
# size of the map's pixels at `zoom` [px/m]
func update_view(center: Vector2, zoom: float, radius: float) -> void:
	# Another map (projection) places the tiles elsewhere
	if _projection.get_revision() != _projection_revision:
		_projection_revision = _projection.get_revision()
		_clear()
	if not _projection.is_available():
		return
	var lat_lon := _projection.to_lat_lon(Vector3(center.x, 0.0, center.y))
	var latitude := deg_to_rad(lat_lon.x)
	var ground_per_level0_pixel := EARTH_CIRCUMFERENCE * cos(latitude) / TILE_PIXELS  # [m]
	var level := clampi(ceili(log(ground_per_level0_pixel * zoom) / log(2.0)), 0, MAX_LEVEL)
	var count := 1 << level
	var tile_size := ground_per_level0_pixel * TILE_PIXELS / count  # [m]
	var middle := Vector2((lat_lon.y + 180.0) / 360.0,
		(1.0 - log(tan(latitude) + 1.0 / cos(latitude)) / PI) * 0.5) * count
	var reach := ceili(radius / tile_size)

	_wanted.clear()
	_shown_in_view = 0
	for y in range(floori(middle.y) - reach, floori(middle.y) + reach + 1):
		for x in range(floori(middle.x) - reach, floori(middle.x) + reach + 1):
			if y < 0 or y >= count:
				continue
			var key := Vector3i(level, posmod(x, count), y)
			if _tiles.get(key) == null:
				_wanted.append(key)
			else:
				_shown_in_view += 1
	var distance := func(key: Vector3i) -> float:
		return middle.distance_squared_to(Vector2(key.y, key.z))
	_wanted.sort_custom(func(a: Vector3i, b: Vector3i): return distance.call(a) < distance.call(b))
	_load_wanted()
	_evict(level, middle)

func _load_wanted() -> void:
	var disk_loads := 0
	for key in _wanted:
		if _tiles.has(key):
			continue  # downloading
		var path := _cache_path(key)
		if FileAccess.file_exists(path):
			if disk_loads == MAX_DISK_LOADS:
				continue
			disk_loads += 1
			_add_tile(key, FileAccess.get_file_as_bytes(path))
		elif not _idle_downloads.is_empty() and _now() >= _retry_time:
			var download: HTTPRequest = _idle_downloads.pop_back()
			var done := _on_downloaded.bind(download, key)
			download.request_completed.connect(done, CONNECT_ONE_SHOT)
			if download.request(URL % [key.x, key.y, key.z], [USER_AGENT]) == OK:
				_tiles[key] = null
			else:
				download.request_completed.disconnect(done)
				_idle_downloads.append(download)

func _on_downloaded(result: int, code: int, _headers: PackedStringArray, body: PackedByteArray,
		download: HTTPRequest, key: Vector3i) -> void:
	_idle_downloads.append(download)
	_tiles.erase(key)  # requested again while wanted, if this failed
	if result != HTTPRequest.RESULT_SUCCESS or code != 200:
		_retry_time = _now() + RETRY_DELAY
		return
	DirAccess.make_dir_recursive_absolute(_cache_path(key).get_base_dir())
	var file := FileAccess.open(_cache_path(key), FileAccess.WRITE)
	if file != null:
		file.store_buffer(body)
	_add_tile(key, body)

func _add_tile(key: Vector3i, png: PackedByteArray) -> void:
	var image := Image.new()
	if image.load_png_from_buffer(png) != OK:
		return
	image.generate_mipmaps()
	var tile := Polygon2D.new()
	tile.texture = ImageTexture.create_from_image(image)
	tile.texture_filter = CanvasItem.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS
	# The tile's corners in the map frame (the projection is near enough to affine over a tile)
	var corners := PackedVector2Array()
	for corner in [Vector2(0, 0), Vector2(1, 0), Vector2(1, 1), Vector2(0, 1)]:
		corners.append(_tile_corner(key, corner))
	tile.polygon = corners
	tile.uv = PackedVector2Array([Vector2(0, 0), Vector2(TILE_PIXELS, 0),
		Vector2(TILE_PIXELS, TILE_PIXELS), Vector2(0, TILE_PIXELS)])
	tile.z_index = key.x - MAX_LEVEL - 1  # finer levels over coarser ones, all under the route
	add_child(tile)
	_tiles[key] = tile

# Position [m] in the 2D world of a point of a tile (corner: 0..1 across it)
func _tile_corner(key: Vector3i, corner: Vector2) -> Vector2:
	var count := float(1 << key.x)
	var longitude := (key.y + corner.x) / count * 360.0 - 180.0
	var latitude := rad_to_deg(atan(sinh(PI * (1.0 - 2.0 * (key.z + corner.y) / count))))
	var position_3d := _projection.from_lat_lon(latitude, longitude)
	return Vector2(position_3d.x, position_3d.z)

# Frees the loaded tiles beyond MAX_TILES, those of other levels and the farthest first
func _evict(level: int, middle: Vector2) -> void:
	var loaded: Array = _tiles.keys().filter(func(key): return _tiles[key] != null)
	if loaded.size() <= MAX_TILES:
		return
	var cost := func(key: Vector3i) -> float:
		var scale := pow(2.0, level - key.x)
		return absf(key.x - level) * 1e6 + middle.distance_squared_to(Vector2(key.y, key.z) * scale)
	loaded.sort_custom(func(a, b): return cost.call(a) > cost.call(b))
	for key in loaded.slice(0, loaded.size() - MAX_TILES):
		_tiles[key].queue_free()
		_tiles.erase(key)

# Frees the tiles; downloads in progress still add theirs, placed with the current projection
func _clear() -> void:
	var downloading := {}
	for key in _tiles:
		if _tiles[key] == null:
			downloading[key] = null
		else:
			_tiles[key].queue_free()
	_tiles = downloading
	_shown_in_view = 0

static func _now() -> float:
	return Time.get_ticks_msec() / 1000.0

static func _cache_path(key: Vector3i) -> String:
	return "%s/%d/%d/%d.png" % [CACHE_DIR, key.x, key.y, key.z]
