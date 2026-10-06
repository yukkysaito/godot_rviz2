extends Sprite2D


var velocity_scale = 2.96706/180.0
var _shown_velocity := NAN

func _process(_delta):
	var velocity: float = RosBridge.velocity * 3.6
	if velocity == _shown_velocity:
		return
	_shown_velocity = velocity

	$VelocityLabel.text = str(velocity).pad_decimals(0)+"km"

	$Hand.set_rotation(velocity * velocity_scale)
