extends TextureRect

var camera_image = CameraImage.new()
var tex = ImageTexture.new()

func _ready():
	camera_image.subscribe("/sensing/camera/camera0/image_rect_color", "compressed")
	pass
func _process(_delta):
	if !camera_image.has_new():
		return	
	var img = camera_image.get_image()
	tex = ImageTexture.create_from_image(img)
	camera_image.set_old()
	texture = tex
	pass
