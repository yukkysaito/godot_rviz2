class_name TileMeshes
extends RefCounted

# Mesh instances of map tiles, created / freed a few per frame so that streaming tiles in and out
# never stalls a frame.
#
# Each tile shows at most one level of detail at a time. The owner says which level a tile should
# show (desired_level(id) -> int, -1: none) and how to build it (make_mesh(id, level) -> Mesh), and
# marks tiles whose desired level may have changed with mark_dirty().

var _parent: Node3D
var _materials: Array  # Material per level
var _make_mesh: Callable
var _desired_level: Callable

var _positions := PackedVector3Array()
var _instances: Array[MeshInstance3D] = []  # null while the tile shows nothing
var _levels := PackedInt32Array()  # level shown per tile (-1: none)

var _queue := PackedInt32Array()  # dirty tiles in order
var _head := 0
var _queued := {}  # tile id -> true

func _init(parent: Node3D, materials: Array, make_mesh: Callable, desired_level: Callable) -> void:
	_parent = parent
	_materials = materials
	_make_mesh = make_mesh
	_desired_level = desired_level

func add_tile(position: Vector3) -> int:
	_positions.append(position)
	_instances.append(null)
	_levels.append(-1)
	return _positions.size() - 1

func mark_dirty(ids: Array) -> void:
	for id in ids:
		if not _queued.has(id):
			_queued[id] = true
			_queue.append(id)

# True when no dirty tile is left
func is_idle() -> bool:
	return _head >= _queue.size()

# Updates dirty tiles until deadline (Time.get_ticks_usec())
func process(deadline: int) -> void:
	while _head < _queue.size() and Time.get_ticks_usec() < deadline:
		var id := _queue[_head]
		_head += 1
		_queued.erase(id)
		_update_tile(id)
	if _head >= _queue.size():
		_queue.clear()
		_head = 0

func clear() -> void:
	for instance in _instances:
		if instance != null:
			instance.queue_free()
	_positions.clear()
	_instances.clear()
	_levels.clear()
	_queue.clear()
	_head = 0
	_queued.clear()

# Number of tiles showing each level, and the vertices they hold
func stats(level_count: int) -> Dictionary:
	var tiles := PackedInt32Array()
	var vertices := PackedInt32Array()
	tiles.resize(level_count)
	vertices.resize(level_count)
	for id in _instances.size():
		if _instances[id] != null:
			tiles[_levels[id]] += 1
			vertices[_levels[id]] += _instances[id].mesh.surface_get_array_len(0)
	return {"tiles": tiles, "vertices": vertices}

func _update_tile(id: int) -> void:
	var level: int = _desired_level.call(id)
	if level == _levels[id]:
		return
	if _instances[id] != null:
		_instances[id].queue_free()
		_instances[id] = null
	_levels[id] = level
	if level < 0:
		return

	var instance := MeshInstance3D.new()
	instance.mesh = _make_mesh.call(id, level)
	instance.material_override = _materials[level]
	instance.position = _positions[id]
	instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_parent.add_child(instance)
	_instances[id] = instance
