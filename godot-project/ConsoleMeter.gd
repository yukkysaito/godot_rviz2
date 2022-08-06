extends Sprite


var velocity_report = VelocityReport.new()
var velocity_scale = 2.96706/180.0

func _ready():
	velocity_report.subscribe("/vehicle/status/velocity_status", false)

func _process(_delta):
	if !velocity_report.has_new():
		return
	var velocity = velocity_report.get_velocity() * 3.6
	
	$VelocityLabel.text = str(velocity).pad_decimals(0)+"km"
	$Hand.set_rotation(velocity * velocity_scale)
	velocity_report.set_old()
