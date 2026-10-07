extends MeshInstance3D

# Ground plane that follows the camera at the height of the ego vehicle (see
# Shaders/ground.gdshader).

# At night (no sun) the ground glows faintly, so that it keeps about its night brightness
@export var night_self_light: float = 0.1

func _ready() -> void:
	Settings.bind("view/day_mode", func(mode):
		(material_override as ShaderMaterial).set_shader_parameter(
			"self_light", night_self_light if mode == "night" else 0.0))

func _process(_delta: float) -> void:
	var camera := get_viewport().get_camera_3d()
	if camera == null:
		return
	var ego := RosBridge.get_ego_position()
	global_position = Vector3(camera.global_position.x, ego.y - 0.02, camera.global_position.z)
