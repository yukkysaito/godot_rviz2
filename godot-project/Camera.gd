extends Spatial


var view_mode = [
	["tpv", Vector3(-8, 4, 0), Vector3(deg2rad(-20), deg2rad(-90), deg2rad(0))], 
	["bev", Vector3(0, 100, 0), Vector3(deg2rad(-90), deg2rad(0), deg2rad(0))]]

var current_view_mode = 0
var mouse_sensitivity = 0.5
var camera_rotation_h = 0.0
var camera_rotation_v = 0.0
var camera_zoom_ratio = 1.0
var camera_zoom_change_ratio = 0.95
var enable_camera_rotation = false

func init_camera_rotation():
	camera_rotation_h = 0.0
	camera_rotation_v = 0.0
	$Horizon.rotation_degrees.y = camera_rotation_h
	$Horizon/Vertical.rotation_degrees.z = camera_rotation_v

func init_camera_translation():
	camera_zoom_ratio = 1.0
	$Horizon/Vertical/ViewCamera.set_translation(view_mode[current_view_mode][1] * camera_zoom_ratio)

func set_camera_translation():
	$Horizon/Vertical/ViewCamera.set_translation(view_mode[current_view_mode][1] * camera_zoom_ratio)

func _ready():
	pass

func _process(delta):
	$Horizon.rotation_degrees.y = lerp($Horizon.rotation_degrees.y, camera_rotation_h, delta*10)
	$Horizon/Vertical.rotation_degrees.z = lerp($Horizon/Vertical.rotation_degrees.z, camera_rotation_v, delta*10)
	$Horizon/Vertical/ViewCamera.set_translation(lerp($Horizon/Vertical/ViewCamera.get_translation(), view_mode[current_view_mode][1] * camera_zoom_ratio, delta*5))

func _input(event):
	if Input.is_action_just_pressed("ui_view_switch"):
		current_view_mode = (current_view_mode + 1) % view_mode.size()
		$Horizon/Vertical/ViewCamera.set_translation(view_mode[current_view_mode][1])
		$Horizon/Vertical/ViewCamera.set_rotation(view_mode[current_view_mode][2])
		init_camera_rotation()
		init_camera_translation()

	if event is InputEventMouseMotion:
		if enable_camera_rotation:
			camera_rotation_h += -event.relative.x * mouse_sensitivity
			camera_rotation_v += -event.relative.y * mouse_sensitivity
	if event is InputEventMouseButton:
		if event.button_index == BUTTON_WHEEL_UP:
			camera_zoom_ratio *= camera_zoom_change_ratio
		if event.button_index == BUTTON_WHEEL_DOWN:
			camera_zoom_ratio *= 2.0 - camera_zoom_change_ratio
		if event.button_index == BUTTON_LEFT:
			enable_camera_rotation = event.pressed


func _on_NightModeCheckButton_toggled(button_pressed):
	if (button_pressed):
		$Horizon/Vertical/ViewCamera.environment.background_sky.set_sky_top_color(Color(0.2,0.2,0.2,1))
	else:
		$Horizon/Vertical/ViewCamera.environment.background_sky.set_sky_top_color(Color(1,1,1,1))
