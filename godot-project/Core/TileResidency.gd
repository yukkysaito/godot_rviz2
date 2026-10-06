class_name TileResidency
extends RefCounted

# Tracks which items (map tiles, traffic lights, ...) are within a radius of a moving position, so
# that only those need nodes / GPU memory however large the map is.
#
# Items are added with ids 0, 1, 2, ... and looked up on a grid. An item is "loaded" when it comes
# within radius and released only when it is farther than radius + margin, so items at the border
# do not switch back and forth.

var radius: float
var margin: float

var _cell_size: float
var _positions := PackedVector2Array()  # (x, z) per item
var _grid := {}  # Vector2i -> PackedInt32Array of item ids
var _loaded := {}  # item id -> true

func _init(radius_: float, margin_: float, cell_size: float) -> void:
	radius = radius_
	margin = margin_
	_cell_size = maxf(cell_size, 1.0)

func add(position: Vector3) -> int:
	var id := _positions.size()
	_positions.append(Vector2(position.x, position.z))
	var cell := _cell_of(_positions[id])
	if not _grid.has(cell):
		_grid[cell] = PackedInt32Array()
	_grid[cell].append(id)
	return id

func size() -> int:
	return _positions.size()

func clear() -> void:
	_positions.clear()
	_grid.clear()
	_loaded.clear()

func is_loaded(id: int) -> bool:
	return _loaded.has(id)

func loaded_count() -> int:
	return _loaded.size()

# Updates the loaded items for center. Returns [added ids (nearest first), released ids].
func update(center: Vector3) -> Array:
	var c := Vector2(center.x, center.z)
	var released := []
	var release_distance := radius + margin
	for id in _loaded.keys():
		if _positions[id].distance_to(c) > release_distance:
			_loaded.erase(id)
			released.append(id)

	var added := []
	for id in _candidates(c):
		if _loaded.has(id):
			continue
		var distance := _positions[id].distance_to(c)
		if distance <= radius:
			_loaded[id] = true
			added.append([distance, id])
	added.sort_custom(func(a, b): return a[0] < b[0])
	return [added.map(func(entry): return entry[1]), released]

# Items in the grid cells that may be within radius of c
func _candidates(c: Vector2) -> Array:
	var reach := ceili(radius / _cell_size)
	if (2 * reach + 1) * (2 * reach + 1) >= _grid.size():
		return range(_positions.size())
	var center := _cell_of(c)
	var ids := []
	for x in range(center.x - reach, center.x + reach + 1):
		for y in range(center.y - reach, center.y + reach + 1):
			var cell := Vector2i(x, y)
			if _grid.has(cell):
				ids.append_array(_grid[cell])
	return ids

func _cell_of(p: Vector2) -> Vector2i:
	return Vector2i(floori(p.x / _cell_size), floori(p.y / _cell_size))
