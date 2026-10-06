# PlayButton.gd
extends PressColorAnimButton

var operation_mode_changer: OperationModeChanger = RosBridge.create_operation_mode_changer()

func _ready() -> void:
	super._ready()
	pressed.connect(_on_pressed)

func _on_pressed() -> void:
	operation_mode_changer.change_to_autonomous_mode()
