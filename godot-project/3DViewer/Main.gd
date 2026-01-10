extends Node3D

var spinner = GodotRviz2Spinner.new()

func _process(_delta):
	spinner.spin_some()
