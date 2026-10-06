extends Sprite2D

@export var angle_scale: float = 17.0
func _process(_delta):
	set_rotation(-RosBridge.steering_angle * angle_scale)
