# PlayButton.gd
extends PressColorAnimButton

var operation_mode_changer := OperationModeChanger.new()

func _ready() -> void:
	operation_mode_changer.create_client("/api/operation_mode/change_to_autonomous")
	super._ready()
	pressed.connect(_on_pressed)

func _on_pressed() -> void:
	operation_mode_changer.change_to_autonomous_mode()
