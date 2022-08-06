extends Spatial

var spinner = GodotRviz2Spinner.new()

func _ready():
	pass # Replace with function body.

func _process(_delta):
	spinner.spin_some()

func _on_TextureButton_pressed():
	pass # Replace with function body.
