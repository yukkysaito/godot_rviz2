extends Node3D

var spinner = GodotRviz2Spinner.new()

func _ready():
	$Menu/RenderingQuality/MSAASlider.set_value(get_viewport().get_msaa_3d())

func _process(_delta):
	spinner.spin_some()

func _on_MSAASlider_value_changed(value):
	get_viewport().set_msaa_3d(value)
