extends Control

@export var vehicle_actuator_status_toggle: BaseButton

const DEFAULT_SIZE_PRESET := 2
const ROOT_BOTTOM_OFFSET := -21.0
const BASE_ROOT_SIZE := Vector2(274.0, 128.0)
const BASE_SLOT_SIZE := Vector2(128.0, 128.0)
const BASE_SPACER_SIZE := Vector2(10.0, 0.0)
const BASE_STEERING_POSITION := Vector2(64.0, 64.0)
const BASE_CONSOLE_METER_POSITION := Vector2(64.0, 43.0)
const BASE_ICON_SCALE := Vector2(0.5, 0.5)
const SIZE_FACTORS := [0.5, 0.75, 1.0, 1.25, 1.5]

@onready var steering_slot: Control = $HBoxContainer/SteeringSlot
@onready var steering: Sprite2D = $HBoxContainer/SteeringSlot/Steering
@onready var spacer: Control = $HBoxContainer/Spacer
@onready var console_meter_slot: Control = $HBoxContainer/ConsoleMeterSlot
@onready var console_meter: Sprite2D = $HBoxContainer/ConsoleMeterSlot/ConsoleMeter

func _ready() -> void:
	visible = vehicle_actuator_status_toggle.button_pressed
	set_size_preset(DEFAULT_SIZE_PRESET)

func _on_vehicle_actuator_atatus_toggle_toggled(toggled_on: bool) -> void:
	visible = toggled_on

func set_size_preset(preset: int) -> void:
	var preset_index := clampi(preset, 0, SIZE_FACTORS.size() - 1)
	var factor: float = SIZE_FACTORS[preset_index]
	var root_size := BASE_ROOT_SIZE * factor

	offset_left = -root_size.x / 2.0
	offset_top = ROOT_BOTTOM_OFFSET - root_size.y
	offset_right = root_size.x / 2.0
	offset_bottom = ROOT_BOTTOM_OFFSET

	steering_slot.custom_minimum_size = BASE_SLOT_SIZE * factor
	console_meter_slot.custom_minimum_size = BASE_SLOT_SIZE * factor
	spacer.custom_minimum_size = BASE_SPACER_SIZE * factor

	steering.position = BASE_STEERING_POSITION * factor
	console_meter.position = BASE_CONSOLE_METER_POSITION * factor
	steering.scale = BASE_ICON_SCALE * factor
	console_meter.scale = BASE_ICON_SCALE * factor
