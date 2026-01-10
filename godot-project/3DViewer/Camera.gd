extends Node3D

# --- Indices for view_mode entries (keep original array structure) ---
const MODE_NAME := 0
const MODE_OFFSET := 1
const MODE_ROT := 2

# --- Cache node references to avoid repeated lookups ---
@onready var horizon: Node3D = $Horizon
@onready var vertical: Node3D = $Horizon/Vertical
@onready var view_camera: Camera3D = $Horizon/Vertical/ViewCamera

# --- Original data structure preserved (no logic change) ---
var view_mode = [
	["tpv", Vector3(-8, 4, 0), Vector3(deg_to_rad(-20), deg_to_rad(-90), deg_to_rad(0))],
	["bev", Vector3(0, 100, 0), Vector3(deg_to_rad(-90), deg_to_rad(0), deg_to_rad(0))]
]

# --- State variables (unchanged semantics) ---
var current_view_mode = 0
var mouse_sensitivity = 0.5
var camera_rotation_h = 0.0
var camera_rotation_v = 0.0
var camera_zoom_ratio = 1.0
var camera_zoom_change_ratio = 0.95
var enable_camera_rotation = false

# --- Small helpers for readability (pure accessors, no logic change) ---
func _mode_name(idx: int) -> String:
	return view_mode[idx][MODE_NAME]

func _mode_offset(idx: int) -> Vector3:
	return view_mode[idx][MODE_OFFSET]

func _mode_rot(idx: int) -> Vector3:
	return view_mode[idx][MODE_ROT]

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
	view_camera.set_position(_mode_offset(current_view_mode) * camera_zoom_ratio)

func set_camera_translation():
	# Apply current offset * zoom to camera (position only)
	view_camera.set_position(_mode_offset(current_view_mode) * camera_zoom_ratio)

func _ready():
	# Keep original no-op (explicitly preserve behavior)
	pass

func _process(delta):
	# Smoothly interpolate pivot rotations towards target (degrees)
	horizon.rotation_degrees.y = lerp(horizon.rotation_degrees.y, camera_rotation_h, delta * 10)
	vertical.rotation_degrees.z = lerp(vertical.rotation_degrees.z, camera_rotation_v, delta * 10)

	# Smoothly interpolate camera position towards offset * zoom
	var target_pos = _mode_offset(current_view_mode) * camera_zoom_ratio
	view_camera.set_position(lerp(view_camera.get_position(), target_pos, delta * 5))

func _unhandled_input(event):
	# Toggle view mode and apply mode rotation/position, then reset rot/zoom as in original
	if Input.is_action_just_pressed("ui_view_switch"):
		current_view_mode = (current_view_mode + 1) % view_mode.size()
		view_camera.set_position(_mode_offset(current_view_mode))
		view_camera.set_rotation(_mode_rot(current_view_mode))
		init_camera_rotation()
		init_camera_translation()

	# Mouse drag to rotate: horizontal always, vertical only when not "bev"
	if event is InputEventMouseMotion:
		if enable_camera_rotation:
			camera_rotation_h += -event.relative.x * mouse_sensitivity
			# Apply vertical rotation only for non-"bev" modes (preserve original condition)
			if _mode_name(current_view_mode) != "bev":
				camera_rotation_v += -event.relative.y * mouse_sensitivity

	# Mouse wheel to zoom, left button to enable/disable rotation
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			camera_zoom_ratio *= camera_zoom_change_ratio
		if event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			camera_zoom_ratio *= 2.0 - camera_zoom_change_ratio
		if event.button_index == MOUSE_BUTTON_LEFT:
			enable_camera_rotation = event.pressed
