extends Sprite

var angle_scale = 17.0
var steering_report = SteeringReport.new()

func _ready():
	steering_report.subscribe("/vehicle/status/steering_status", false)

func _process(_delta):
	if !steering_report.has_new():
		return
	var angle = steering_report.get_angle()
	set_rotation(-angle * angle_scale)
	steering_report.set_old()
