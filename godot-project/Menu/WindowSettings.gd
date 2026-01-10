extends Node
class_name WindowSettings

@export var display_mode_option_path: NodePath
@export var always_on_top_toggle_path: NodePath
@export var anti_alias_option_path: NodePath
@export var target_viewport_path: NodePath

@onready var display_mode_option: OptionButton = get_node(display_mode_option_path)
@onready var always_on_top_toggle: CheckButton = get_node(always_on_top_toggle_path)
@onready var aa_option: OptionButton = get_node(anti_alias_option_path)

@onready var target_vp: Viewport = (
	get_node(target_viewport_path) as Viewport
	if target_viewport_path != NodePath("")
	else get_viewport()
)

enum WindowPreset {
	WINDOWED,
	MAXIMIZED,
	BORDERLESS_MAXIMIZED,
	FULLSCREEN
}

enum AAPreset { AA_OFF, FXAA, TAA, MSAA_2X, MSAA_4X, MSAA_8X }

var _suppress := false

func _ready() -> void:
	_setup_display_mode_options()
	_setup_aa_options()
	_sync_ui_from_current()

	display_mode_option.item_selected.connect(_on_mode_selected)
	always_on_top_toggle.toggled.connect(_on_always_on_top_toggled)
	aa_option.item_selected.connect(_on_aa_selected)

func _setup_display_mode_options() -> void:
	display_mode_option.clear()
	display_mode_option.add_item("Windowed", WindowPreset.WINDOWED)
	display_mode_option.add_item("Maximized", WindowPreset.MAXIMIZED)
	display_mode_option.add_item("Borderless Maximized", WindowPreset.BORDERLESS_MAXIMIZED)
	display_mode_option.add_item("Fullscreen", WindowPreset.FULLSCREEN)

func _setup_aa_options() -> void:
	aa_option.clear()
	aa_option.add_item("AA Off", AAPreset.AA_OFF)
	aa_option.add_item("FXAA", AAPreset.FXAA)
	aa_option.add_item("TAA", AAPreset.TAA)
	aa_option.add_item("MSAA 2x", AAPreset.MSAA_2X)
	aa_option.add_item("MSAA 4x", AAPreset.MSAA_4X)
	aa_option.add_item("MSAA 8x", AAPreset.MSAA_8X)

func _on_aa_selected(index: int) -> void:
	if _suppress:
		return
	var preset := aa_option.get_item_id(index)
	_apply_aa_preset(preset)

func _apply_aa_preset(preset: int) -> void:
	if target_vp == null:
		return

	# まず両方OFFにしてから、どちらかを有効化（排他）
	target_vp.msaa_3d = Viewport.MSAA_DISABLED
	target_vp.use_taa = false
	target_vp.screen_space_aa = Viewport.SCREEN_SPACE_AA_DISABLED

	match preset:
		AAPreset.AA_OFF:
			pass

		AAPreset.FXAA:
			target_vp.screen_space_aa = Viewport.SCREEN_SPACE_AA_FXAA

		AAPreset.TAA:
			target_vp.use_taa = true

		AAPreset.MSAA_2X:
			target_vp.msaa_3d = Viewport.MSAA_2X

		AAPreset.MSAA_4X:
			target_vp.msaa_3d = Viewport.MSAA_4X

		AAPreset.MSAA_8X:
			target_vp.msaa_3d = Viewport.MSAA_8X

func _on_mode_selected(index: int) -> void:
	if _suppress:
		return
	var preset := display_mode_option.get_item_id(index)
	_apply_window_preset(preset)

func _apply_window_preset(preset: int) -> void:
	match preset:
		WindowPreset.WINDOWED:
			DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_BORDERLESS, false)
			DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_WINDOWED)
		WindowPreset.MAXIMIZED:
			DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_BORDERLESS, false)
			DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_MAXIMIZED)
		WindowPreset.BORDERLESS_MAXIMIZED:
			DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_BORDERLESS, true)
			DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_MAXIMIZED)
		WindowPreset.FULLSCREEN:
			DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_BORDERLESS, false)
			DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_FULLSCREEN)

	_emit_window_changed_soon()

func _on_always_on_top_toggled(on: bool) -> void:
	# Godot 4.x: WINDOW_FLAG_ALWAYS_ON_TOP
	DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_ALWAYS_ON_TOP, on)
	_emit_window_changed_soon()

func _sync_ui_from_current() -> void:
	_suppress = true

	# mode preset
	var mode := DisplayServer.window_get_mode()
	var borderless := DisplayServer.window_get_flag(DisplayServer.WINDOW_FLAG_BORDERLESS)
	var preset := WindowPreset.WINDOWED
	if mode == DisplayServer.WINDOW_MODE_FULLSCREEN:
		preset = WindowPreset.FULLSCREEN
	elif mode == DisplayServer.WINDOW_MODE_MAXIMIZED and borderless:
		preset = WindowPreset.BORDERLESS_MAXIMIZED
	elif mode == DisplayServer.WINDOW_MODE_MAXIMIZED:
		preset = WindowPreset.MAXIMIZED

	for i in range(display_mode_option.item_count):
		if display_mode_option.get_item_id(i) == preset:
			display_mode_option.select(i)
			break

	# always on top
	always_on_top_toggle.button_pressed = DisplayServer.window_get_flag(DisplayServer.WINDOW_FLAG_ALWAYS_ON_TOP)

	# AA
	_sync_aa_ui()

	_suppress = false

func _sync_aa_ui() -> void:
	if target_vp == null:
		return

	var preset := AAPreset.AA_OFF

	# MSAA優先で判定
	match target_vp.msaa_3d:
		Viewport.MSAA_2X: preset = AAPreset.MSAA_2X
		Viewport.MSAA_4X: preset = AAPreset.MSAA_4X
		Viewport.MSAA_8X: preset = AAPreset.MSAA_8X
		_:
			if target_vp.use_taa:
				preset = AAPreset.TAA
			elif target_vp.screen_space_aa == Viewport.SCREEN_SPACE_AA_FXAA:
				preset = AAPreset.FXAA
			else:
				preset = AAPreset.AA_OFF

	for i in range(aa_option.item_count):
		if aa_option.get_item_id(i) == preset:
			aa_option.select(i)
			break

# 画面モード変更後はサイズ反映が次フレームになることが多いので通知
signal window_changed
func _emit_window_changed_soon() -> void:
	call_deferred("_emit_window_changed")
func _emit_window_changed() -> void:
	emit_signal("window_changed")
