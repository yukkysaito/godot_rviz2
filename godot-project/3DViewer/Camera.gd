extends Node3D

# --- Indices for view_mode entries (keep original array structure) ---
const MODE_NAME := 0
const MODE_OFFSET := 1
const MODE_ROT := 2

# --- Cache node references to avoid repeated lookups ---
@onready var ego_vehicle: Node = get_parent()
@onready var horizon: Node3D = $Horizon
@onready var vertical: Node3D = $Horizon/Vertical
@onready var view_camera: Camera3D = $Horizon/Vertical/ViewCamera

# --- Original data structure preserved (no logic change) ---
var view_mode = [
	["tpv", Vector3(-9, 4.25, 0), Vector3(deg_to_rad(-25), deg_to_rad(-90), deg_to_rad(0))],
	["bev", Vector3(0, 100, 0), Vector3(deg_to_rad(-90), deg_to_rad(0), deg_to_rad(0))]
]

# --- State variables (unchanged semantics) ---
var current_view_mode: int = 0
var mouse_sensitivity: float = 0.5
var camera_rotation_h: float = 0.0
var camera_rotation_v: float = 0.0
var camera_zoom_ratio: float = 1.0
var camera_zoom_change_ratio: float = 0.95
@export var turn_signal_camera_offset: Vector3 = Vector3(-6.0, 18.0, 0.0)
@export var turn_signal_camera_pitch_deg: float = 45.0
@export var turn_signal_transition_speed: float = 0.72
var enable_camera_rotation: bool = false
var touch_points := {}
var prev_pinch_distance: float = -1.0
var auto_return_enabled: bool = true
var adaptive_camera_work_enabled: bool = true
var auto_return_speed: float = 2.0
var auto_return_delay: float = 2.0
var _auto_return_timer: float = 0.0
var turn_signal_view_blend: float = 0.0

func _is_touch_gesture_active() -> bool:
	return touch_points.size() > 0

# --- Small helpers for readability (pure accessors, no logic change) ---
func _mode_name(idx: int) -> String:
	return view_mode[idx][MODE_NAME]

func _mode_offset(idx: int) -> Vector3:
	return view_mode[idx][MODE_OFFSET]

func _mode_rot(idx: int) -> Vector3:
	return view_mode[idx][MODE_ROT]

func _is_turn_signal_active() -> bool:
	if not is_instance_valid(ego_vehicle):
		return false
	if not ego_vehicle.has_method("is_turn_indicator_active"):
		return false
	return bool(ego_vehicle.call("is_turn_indicator_active"))

func _effective_camera_offset() -> Vector3:
	var offset: Vector3 = _mode_offset(current_view_mode)
	if _mode_name(current_view_mode) == "tpv":
		offset += turn_signal_camera_offset * turn_signal_view_blend
	return offset

func _effective_camera_position() -> Vector3:
	return _effective_camera_offset() * camera_zoom_ratio

func _effective_camera_rotation() -> Vector3:
	var target_rotation: Vector3 = _mode_rot(current_view_mode)
	if _mode_name(current_view_mode) == "tpv":
		target_rotation.x = lerpf(target_rotation.x, deg_to_rad(-turn_signal_camera_pitch_deg), turn_signal_view_blend)
	return target_rotation

# --- Initialization helpers (same behavior as original) ---
func init_camera_rotation():
	# Reset stored horizontal/vertical rotations and apply immediately to pivot nodes (degrees)
	camera_rotation_h = 0.0
	camera_rotation_v = 0.0
	horizon.rotation_degrees.y = camera_rotation_h
	vertical.rotation_degrees.z = camera_rotation_v

func init_camera_translation():
	# Reset zoom and snap camera to mode offset * zoom (position only)
	camera_zoom_ratio = 1.0
	view_camera.set_position(_effective_camera_position())

func set_camera_translation():
	# Apply current offset * zoom to camera (position only)
	view_camera.set_position(_effective_camera_position())

func _ready():
	# Keep original no-op (explicitly preserve behavior)
	pass

func _process(delta):
	var turn_signal_target_blend: float = 0.0
	if adaptive_camera_work_enabled and _is_turn_signal_active():
		turn_signal_target_blend = 1.0
	turn_signal_view_blend = lerpf(turn_signal_view_blend, turn_signal_target_blend, delta * turn_signal_transition_speed)

	# Auto-return: when not dragging, gradually reset rotation and zoom
	if auto_return_enabled and not enable_camera_rotation and not _is_touch_gesture_active():
		_auto_return_timer += delta
		if _auto_return_timer > auto_return_delay:
			var t = auto_return_speed * delta
			camera_rotation_h = lerpf(camera_rotation_h, 0.0, t)
			camera_rotation_v = lerpf(camera_rotation_v, 0.0, t)
			camera_zoom_ratio = lerpf(camera_zoom_ratio, 1.0, t)

	# Smoothly interpolate pivot rotations towards target (degrees)
	horizon.rotation_degrees.y = lerp(horizon.rotation_degrees.y, camera_rotation_h, delta * 10)
	vertical.rotation_degrees.z = lerp(vertical.rotation_degrees.z, camera_rotation_v, delta * 10)

	# Smoothly interpolate camera transform towards the turn-signal view
	var target_pos: Vector3 = _effective_camera_position()
	view_camera.set_position(lerp(view_camera.get_position(), target_pos, delta * 5))
	var target_rot: Vector3 = _effective_camera_rotation()
	view_camera.set_rotation(view_camera.get_rotation().lerp(target_rot, delta * 5.0))

func _unhandled_input(event):
	# Toggle view mode and apply mode rotation/position, then reset rot/zoom as in original
	if Input.is_action_just_pressed("ui_view_switch"):
		current_view_mode = (current_view_mode + 1) % view_mode.size()
		view_camera.set_position(_effective_camera_position())
		view_camera.set_rotation(_effective_camera_rotation())
		init_camera_rotation()
		init_camera_translation()

	# Mouse drag to rotate: horizontal always, vertical only when not "bev"
	if event is InputEventMouseMotion:
		if enable_camera_rotation and not _is_touch_gesture_active():
			camera_rotation_h += -event.relative.x * mouse_sensitivity
			if _mode_name(current_view_mode) != "bev":
				camera_rotation_v += -event.relative.y * mouse_sensitivity
			_auto_return_timer = 0.0

	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			camera_zoom_ratio *= camera_zoom_change_ratio
			_auto_return_timer = 0.0
		if event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			camera_zoom_ratio *= 2.0 - camera_zoom_change_ratio
			_auto_return_timer = 0.0
		if event.button_index == MOUSE_BUTTON_LEFT and not _is_touch_gesture_active():
			enable_camera_rotation = event.pressed
			if not event.pressed:
				_auto_return_timer = 0.0

	# Track screen touches for pinch zoom
	if event is InputEventScreenTouch:
		enable_camera_rotation = false
		if event.pressed:
			touch_points[event.index] = event.position
		else:
			touch_points.erase(event.index)
		if touch_points.size() != 2:
			prev_pinch_distance = -1.0
		_auto_return_timer = 0.0

	# Two-finger pinch to zoom on touch devices
	if event is InputEventScreenDrag:
		if touch_points.has(event.index):
			touch_points[event.index] = event.position
		if touch_points.size() == 1:
			camera_rotation_h += -event.relative.x * mouse_sensitivity
			if _mode_name(current_view_mode) != "bev":
				camera_rotation_v += -event.relative.y * mouse_sensitivity
		elif touch_points.size() == 2:
			var points = touch_points.values()
			var pinch_distance = points[0].distance_to(points[1])
			if prev_pinch_distance > 0.0 and pinch_distance > 0.0:
				camera_zoom_ratio *= prev_pinch_distance / pinch_distance
			prev_pinch_distance = pinch_distance
		_auto_return_timer = 0.0

func _on_camera_auto_return_toggle_toggled(toggled_on: bool):
	auto_return_enabled = toggled_on

func _on_adaptive_camera_work_toggle_toggled(toggled_on: bool):
	adaptive_camera_work_enabled = toggled_on
