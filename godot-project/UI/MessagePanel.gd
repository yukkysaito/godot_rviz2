# MessagePanel.gd
extends Control

# A message in a pill at the top middle of the window (see 3DViewer/OpeationControl.gd).
# Sized for a 1080 px high window and scaled with the window height, as the HUD (UI/Hud.gd), and
# smaller in a window too narrow for its text. In a portrait window, where the mini map
# (UI/MiniMap.gd) is at the top right, it goes under the mini map when it would cover it.

@export var font_size: float = 26.0  # [px] at a 1080 px high window
@export var padding: Vector2 = Vector2(36, 14)  # [px] around the text
@export var top: float = 100.0  # [px] from the top of the window
@export var side_margin: float = 24.0  # [px] kept free at the window's sides
@export var map_gap: float = 16.0  # [px] under the mini map
@export var border_width: float = 2.7  # [px]
@export var shadow_size: float = 16.0  # [px]

# The mini map to keep clear of (set before the panel enters the tree)
var mini_map: Control = null

@onready var anim: AnimationPlayer = $AnimationPlayer
@onready var _container: CenterContainer = $CenterContainer
@onready var _panel: PanelContainer = $CenterContainer/PanelContainer
@onready var label: Label = $CenterContainer/PanelContainer/VBoxContainer/Label

var _box: PillStyleBox  # sized in _layout
var _laid_out := []  # what it was laid out for

func _ready() -> void:
	_box = _panel.get_theme_stylebox("panel").duplicate() as PillStyleBox
	_panel.add_theme_stylebox_override("panel", _box)
	_layout()
	# Optional: small delay so it doesn't pop in on scene enter
	await get_tree().process_frame
	if anim.has_animation("fade_in"):
		anim.play("fade_in")

# The window and the mini map change without telling, so they are checked every frame (cheap)
func _process(_delta: float) -> void:
	_layout()

func show_message(text: String) -> void:
	# Update text and play fade-in
	label.text = text
	_layout()
	if anim.has_animation("fade_in"):
		anim.play("fade_in")
	else:
		modulate.a = 1.0

func hide_message() -> void:
	# Play fade-out and free after complete
	if anim.has_animation("fade_out"):
		anim.play("fade_out")
		await anim.animation_finished
		queue_free()
	else:
		queue_free()

func _layout() -> void:
	var window := get_viewport_rect().size
	var map_rect := Rect2()
	if mini_map != null and is_instance_valid(mini_map) and mini_map.is_visible_in_tree():
		map_rect = mini_map.corner_rect()
	var state := [window, label.text, map_rect]
	if state == _laid_out:
		return
	_laid_out = state

	var font := label.get_theme_font("font")
	var text_width := font.get_string_size(label.text, HORIZONTAL_ALIGNMENT_LEFT, -1, int(font_size)).x
	var full_width := text_width + 2.0 * padding.x
	var s := window.y / 1080.0
	s = minf(s, (window.x - 2.0 * side_margin * s) / full_width)  # a narrow window: fit its width
	s = maxf(s, 0.01)  # narrower than its side margins

	var size_px := maxi(1, roundi(font_size * s))
	var height := font.get_height(size_px) + 2.0 * roundf(padding.y * s)
	label.add_theme_font_size_override("font_size", size_px)
	_box.content_margin_left = roundf(padding.x * s)
	_box.content_margin_right = roundf(padding.x * s)
	_box.content_margin_top = roundf(padding.y * s)
	_box.content_margin_bottom = roundf(padding.y * s)
	_box.border_width = maxf(1.0, border_width * s)
	_box.shadow_size = shadow_size * s

	var pill := Rect2((window.x - full_width * s) * 0.5, roundf(top * s), full_width * s, height)
	if map_rect.has_area() and pill.intersects(map_rect.grow(map_gap * s)):
		pill.position.y = roundf(map_rect.end.y + map_gap * s)
	# The container spans the window's width, in the window's coordinates
	_container.global_position = Vector2(0, pill.position.y)
	_container.size = Vector2(window.x, height)
