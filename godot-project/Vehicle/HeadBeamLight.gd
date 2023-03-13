extends SpotLight

func _ready():
	visible = false

func turn_on():
	visible = true

func turn_off():
	visible = false
	
func _process(_delta):
	pass
