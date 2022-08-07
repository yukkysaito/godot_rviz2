extends Control

func _ready():
	visible = false

func _input(_event):
	if Input.is_action_just_pressed("ui_select"):
		visible = not visible
