extends Spatial

var spinner = GodotRviz2Spinner.new()

func _ready():
	$Menu/RenderingQuality/MSAASlider.set_value(get_viewport().get_msaa())

func _process(_delta):
	spinner.spin_some()

func _on_TextureButton_pressed():
	pass # Replace with function body.

func _on_MSAASlider_value_changed(value):
	get_viewport().set_msaa(value)
