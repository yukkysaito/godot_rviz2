extends Node3D
class_name LengthFittedModel
# A vehicle model fitted to a measured length without squashing it: the front and the rear keep
# their shape, and the section between them is repeated, each copy stretched a little to fill
# the gap. Expects these nodes from the model file:
#   Front, Rear: the ends, with their origins at the seams; their children move with them
#   Front/FrontEnd, Rear/RearEnd: markers at the very front and rear of the body
#   MidBay: one section between the seams, with its origin at its center

# A section shorter than this would look squeezed: a shorter model is scaled down as a whole, to
# MIN_SCALE at most
const MIN_BAY_SCALE := 0.7
const MIN_SCALE := 0.8
# Keeps the number of sections while the length jitters around a switch [sections]
const BAY_HYSTERESIS := 0.6

@export var max_length := 20.0  # [m] longer, the sections stretch

@onready var _front: Node3D = $Front
@onready var _rear: Node3D = $Rear
var _front_length: float  # from the front end to the front seam
var _rear_length: float
var _bay_length: float
var _bays: Array[Node3D] = []
var _bay_count := 0
var _length := 0.0

func _ready() -> void:
	var mid := $MidBay as MeshInstance3D
	_front_length = ($Front/FrontEnd as Node3D).position.x
	_rear_length = -($Rear/RearEnd as Node3D).position.x
	_bay_length = mid.get_aabb().size.x
	var modeled := _front_length + _rear_length + _front.position.x - _rear.position.x
	# Every section exists from the start, so that the users of the model find all its meshes: as
	# many as max_length can show (the hysteresis can keep one more than it rounds to)
	var count := ceili((max_length - _front_length - _rear_length) / _bay_length + BAY_HYSTERESIS)
	_bays.append(mid)
	for i in maxi(count, 1) - 1:
		var bay := mid.duplicate() as Node3D
		add_child(bay)
		_bays.append(bay)
	fit_length(modeled)

# Without a length (<= 0), the model keeps its current one
func fit_length(length: float) -> void:
	if length <= 0.0 or absf(length - _length) < 0.01:
		return
	_length = length
	var mid := length - _front_length - _rear_length
	var squash := 1.0
	if mid < MIN_BAY_SCALE * _bay_length:
		mid = MIN_BAY_SCALE * _bay_length
		squash = maxf(length / (_front_length + _rear_length + mid), MIN_SCALE)
		_bay_count = 1
	elif _bay_count == 0 or absf(mid / _bay_length - _bay_count) > BAY_HYSTERESIS:
		_bay_count = clampi(roundi(mid / _bay_length), 1, _bays.size())
	var bay := mid / _bay_count
	var front_seam := 0.5 * (mid + _rear_length - _front_length)  # the model is centered
	_front.position.x = front_seam
	_rear.position.x = front_seam - mid
	for i in _bays.size():
		_bays[i].visible = i < _bay_count
		_bays[i].position.x = front_seam - (i + 0.5) * bay
		_bays[i].scale.x = bay / _bay_length
	scale.x = squash
