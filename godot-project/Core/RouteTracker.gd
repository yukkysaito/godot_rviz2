extends RefCounted

# The ego vehicle's progress along the route (see RosBridge) and the remaining distance to the goal,
# along the centerline of the route's lanelets. The vehicle is followed along the route from where
# it was, so that a route passing the same place twice (a loop) does not make it jump.

const SEARCH_BEHIND := 20.0  # [m] the vehicle is looked for this far behind where it was
const SEARCH_AHEAD := 100.0  # [m] and this far ahead of it
# [m] from the route beyond which the vehicle is looked for along all of it
const MAX_OFFSET := 10.0

var valid := false  # the vehicle was found on a route
var remaining_distance := 0.0  # [m]
var progress := 0.0  # 0..1 from the route's start to its goal

var _points := PackedVector2Array()  # (x, z) of the centerline
var _distances := PackedFloat32Array()  # [m] along the route at each point
var _start := 0.0  # [m] along the route
var _goal := 0.0
var _position := -1.0  # [m] along the route where the vehicle was (< 0: not found yet)

# The start lies within `start_reach` [m] of the route's beginning (its first lanelet) and the goal
# within `goal_reach` [m] of its end (its last lanelet)
func set_route(points: PackedVector3Array, start: Vector3, goal: Vector3, start_reach: float,
		goal_reach: float) -> void:
	clear()
	if points.size() < 2:
		return
	var distance := 0.0
	for i in points.size():
		var point := Vector2(points[i].x, points[i].z)
		if i > 0:
			distance += point.distance_to(_points[i - 1])
		_points.append(point)
		_distances.append(distance)
	_start = _locate(Vector2(start.x, start.z), 0.0, start_reach).x
	_goal = _locate(Vector2(goal.x, goal.z), distance - goal_reach, distance).x

func clear() -> void:
	_points.clear()
	_distances.clear()
	_position = -1.0
	valid = false

func update(ego: Vector3) -> void:
	if _points.size() < 2:
		valid = false
		return
	var point := Vector2(ego.x, ego.z)
	var found := Vector2(0.0, INF)  # (distance along the route, offset from it)
	if _position >= 0.0:
		found = _locate(point, _position - SEARCH_BEHIND, _position + SEARCH_AHEAD)
	if found.y > MAX_OFFSET:
		found = _locate(point, 0.0, _distances[_distances.size() - 1])
	_position = found.x
	valid = true
	remaining_distance = maxf(_goal - _position, 0.0)
	progress = clampf((_position - _start) / maxf(_goal - _start, 0.1), 0.0, 1.0)

# Nearest point of the route to `point` between `from` and `to` [m] along it:
# (distance along the route, distance to it)
func _locate(point: Vector2, from: float, to: float) -> Vector2:
	var best := Vector2(clampf(from, 0.0, _distances[_distances.size() - 1]), INF)
	var first := maxi(_distances.bsearch(from) - 1, 0)
	var last := mini(_distances.bsearch(to, false), _points.size() - 1)
	for i in range(first, last):
		var a := _points[i]
		var segment := _points[i + 1] - a
		var length := segment.length()
		var t := clampf((point - a).dot(segment) / maxf(length * length, 1e-6), 0.0, 1.0)
		var offset := point.distance_to(a + segment * t)
		if offset < best.y:
			best = Vector2(_distances[i] + length * t, offset)
	return best
