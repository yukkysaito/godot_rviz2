class_name MenuTheme

# The settings menu's look, in the HUD's colors (UI/Hud.gd): light text on a frosted dark blue, the
# accent blue for what is on or chosen. Its title, the section titles (small spaced capitals, see
# Selection.gd), the rows, the choices' boxes and their lists, the sliders, the scroll bar and the
# quit button. The toggles and the segmented choices draw themselves (UI/SettingToggle.gd,
# UI/SettingSegmented.gd) in the same colors.

const FONT := preload("res://UI/Fonts/Barlow-Medium.ttf")
const FONT_BOLD := preload("res://UI/Fonts/Barlow-SemiBold.ttf")

const TEXT := Hud.TEXT
const MUTED := Hud.MUTED
const ACCENT := Hud.ACCENT
const STOP := Hud.STOP
const RIGHT_COLUMN := 160.0  # [px] the width of the choices, aligned on the right

static func build() -> Theme:
	var theme := Theme.new()
	theme.default_font = FONT
	theme.default_font_size = 15

	# Text: the labels, the choices and their lists
	theme.set_color("font_color", "Label", TEXT)
	theme.set_font_size("font_size", "OptionButton", 14)
	theme.set_font_size("font_size", "PopupMenu", 14)

	# Rows: a label and its control
	var row := StyleBoxEmpty.new()
	row.content_margin_top = 8
	row.content_margin_bottom = 8
	theme.set_stylebox("normal", "Label", row)
	theme.set_constant("separation", "HBoxContainer", 12)
	theme.set_constant("separation", "VBoxContainer", 0)

	_add_titles(theme)
	_add_option_button(theme)
	_add_popup(theme)
	_add_slider(theme)
	_add_scroll(theme)
	_add_quit_button(theme)
	_add_toggle(theme)
	return theme

static func _add_titles(theme: Theme) -> void:
	# The section titles (Selection.gd draws their line, icon and chevron)
	var title_font := FontVariation.new()
	title_font.base_font = FONT_BOLD
	title_font.spacing_glyph = 2
	theme.set_type_variation("SectionTitle", "Label")
	theme.set_font("font", "SectionTitle", title_font)
	theme.set_font_size("font_size", "SectionTitle", 13)
	theme.set_color("font_color", "SectionTitle", MUTED)
	var title_box := StyleBoxEmpty.new()
	title_box.content_margin_left = 26  # room for the icon
	title_box.content_margin_top = 20
	title_box.content_margin_bottom = 8
	theme.set_stylebox("normal", "SectionTitle", title_box)

	# The menu's title
	theme.set_type_variation("MenuTitle", "Label")
	theme.set_font("font", "MenuTitle", FONT_BOLD)
	theme.set_font_size("font_size", "MenuTitle", 26)
	var menu_title_box := StyleBoxEmpty.new()
	menu_title_box.content_margin_bottom = 6
	theme.set_stylebox("normal", "MenuTitle", menu_title_box)

static func _box(bg: Color, border: Color, radius: int, margin_h: float = 12, margin_v: float = 6) -> StyleBoxFlat:
	var box := StyleBoxFlat.new()
	box.bg_color = bg
	box.border_color = border
	box.set_border_width_all(1)
	box.set_corner_radius_all(radius)
	box.corner_detail = 6
	box.anti_aliasing = true
	box.content_margin_left = margin_h
	box.content_margin_right = margin_h
	box.content_margin_top = margin_v
	box.content_margin_bottom = margin_v
	return box

static func _add_option_button(theme: Theme) -> void:
	theme.set_stylebox("normal", "OptionButton", _box(Color(1, 1, 1, 0.05), Color(1, 1, 1, 0.12), 8))
	theme.set_stylebox("hover", "OptionButton", _box(Color(1, 1, 1, 0.08), Color(ACCENT, 0.55), 8))
	theme.set_stylebox("pressed", "OptionButton", _box(Color(ACCENT, 0.16), Color(ACCENT, 0.8), 8))
	theme.set_stylebox("hover_pressed", "OptionButton", _box(Color(ACCENT, 0.16), Color(ACCENT, 0.8), 8))
	theme.set_stylebox("focus", "OptionButton", StyleBoxEmpty.new())
	for state in ["font_color", "font_hover_color", "font_pressed_color", "font_hover_pressed_color", "font_focus_color"]:
		theme.set_color(state, "OptionButton", TEXT)
	theme.set_constant("modulate_arrow", "OptionButton", 1)
	theme.set_icon("arrow", "OptionButton", _chevron_icon(10, Color(MUTED, 1.0)))

static func _add_popup(theme: Theme) -> void:
	var panel := _box(Color(0.04, 0.06, 0.11, 0.97), Color(1, 1, 1, 0.12), 10, 6, 6)
	panel.shadow_color = Color(0, 0, 0, 0.45)
	panel.shadow_size = 12
	theme.set_stylebox("panel", "PopupMenu", panel)
	theme.set_stylebox("hover", "PopupMenu", _box(Color(ACCENT, 0.2), Color(ACCENT, 0.0), 6, 8, 4))
	theme.set_color("font_color", "PopupMenu", TEXT)
	theme.set_color("font_hover_color", "PopupMenu", Color.WHITE)
	theme.set_constant("v_separation", "PopupMenu", 10)
	theme.set_constant("item_start_padding", "PopupMenu", 10)
	theme.set_constant("item_end_padding", "PopupMenu", 14)
	theme.set_icon("radio_checked", "PopupMenu", _dot_icon(14, 3.5, ACCENT))
	theme.set_icon("radio_unchecked", "PopupMenu", _dot_icon(14, 0.0, Color(0, 0, 0, 0)))

static func _add_slider(theme: Theme) -> void:
	var track := StyleBoxFlat.new()
	track.bg_color = Color(1, 1, 1, 0.12)
	track.set_corner_radius_all(2)
	track.content_margin_top = 2
	track.content_margin_bottom = 2
	theme.set_stylebox("slider", "HSlider", track)
	var fill := StyleBoxFlat.new()
	fill.bg_color = ACCENT
	fill.set_corner_radius_all(2)
	fill.shadow_color = Color(ACCENT, 0.35)
	fill.shadow_size = 4
	theme.set_stylebox("grabber_area", "HSlider", fill)
	var fill_hover := fill.duplicate() as StyleBoxFlat
	fill_hover.bg_color = ACCENT.lightened(0.15)
	theme.set_stylebox("grabber_area_highlight", "HSlider", fill_hover)
	theme.set_icon("grabber", "HSlider", _knob_icon(18, Color(0.95, 0.97, 1.0), ACCENT))
	theme.set_icon("grabber_highlight", "HSlider", _knob_icon(18, Color.WHITE, ACCENT.lightened(0.2)))
	theme.set_type_variation("SliderValue", "Label")
	theme.set_color("font_color", "SliderValue", MUTED)
	theme.set_font_size("font_size", "SliderValue", 14)

static func _add_scroll(theme: Theme) -> void:
	# A thin bar beside the rows, with room for the glow of the controls at their right end
	theme.set_stylebox("panel", "ScrollContainer", StyleBoxEmpty.new())
	theme.set_constant("scrollbar_h_separation", "ScrollContainer", 10)  # between the rows and the bar beside them
	var lane := StyleBoxEmpty.new()
	lane.content_margin_left = 2
	lane.content_margin_right = 2
	theme.set_stylebox("scroll", "VScrollBar", lane)
	theme.set_stylebox("scroll_focus", "VScrollBar", lane)
	var grab := StyleBoxFlat.new()
	grab.bg_color = Color(1, 1, 1, 0.14)
	grab.set_corner_radius_all(2)
	grab.content_margin_left = 2
	grab.content_margin_right = 2
	theme.set_stylebox("grabber", "VScrollBar", grab)
	var grab_hover := grab.duplicate() as StyleBoxFlat
	grab_hover.bg_color = Color(ACCENT, 0.6)
	theme.set_stylebox("grabber_highlight", "VScrollBar", grab_hover)
	theme.set_stylebox("grabber_pressed", "VScrollBar", grab_hover)

static func _add_quit_button(theme: Theme) -> void:
	theme.set_type_variation("QuitButton", "Button")
	var normal := PillStyleBox.new()
	normal.bg_color = Color(1, 1, 1, 0.04)
	normal.border_color = Color(1, 1, 1, 0.14)
	normal.content_margin_top = 9
	normal.content_margin_bottom = 9
	var hover := PillStyleBox.new()
	hover.bg_color = Color(STOP, 0.16)
	hover.border_color = Color(STOP, 0.75)
	hover.content_margin_top = 9
	hover.content_margin_bottom = 9
	var pressed := hover.duplicate() as PillStyleBox
	pressed.bg_color = Color(STOP, 0.32)
	theme.set_stylebox("normal", "QuitButton", normal)
	theme.set_stylebox("hover", "QuitButton", hover)
	theme.set_stylebox("pressed", "QuitButton", pressed)
	theme.set_stylebox("hover_pressed", "QuitButton", pressed)
	theme.set_stylebox("focus", "QuitButton", StyleBoxEmpty.new())
	theme.set_font("font", "QuitButton", FONT_BOLD)
	theme.set_font_size("font_size", "QuitButton", 14)
	theme.set_color("font_color", "QuitButton", MUTED)
	theme.set_color("font_hover_color", "QuitButton", Color(1.0, 0.82, 0.8))
	theme.set_color("font_pressed_color", "QuitButton", Color(1.0, 0.82, 0.8))
	theme.set_color("font_hover_pressed_color", "QuitButton", Color(1.0, 0.82, 0.8))

static func _add_toggle(theme: Theme) -> void:
	# SettingToggle draws its switch where the check box would be: these only keep that room
	var room := _empty_icon(SettingToggle.SWITCH_SIZE)
	for icon in ["checked", "unchecked", "checked_disabled", "unchecked_disabled",
			"checked_mirrored", "unchecked_mirrored", "checked_disabled_mirrored", "unchecked_disabled_mirrored"]:
		theme.set_icon(icon, "CheckButton", room)
	for state in ["normal", "hover", "pressed", "hover_pressed", "focus", "disabled"]:
		theme.set_stylebox(state, "CheckButton", StyleBoxEmpty.new())

# --- Icons drawn into images (small, so a few pixels of anti-aliasing by coverage are enough) ---

static func _empty_icon(size: Vector2i) -> ImageTexture:
	return ImageTexture.create_from_image(Image.create_empty(size.x, size.y, false, Image.FORMAT_RGBA8))

# Covered fraction of a pixel by a disc (approximated by the distance to its edge)
static func _disc(p: Vector2, center: Vector2, radius: float) -> float:
	return clampf(radius - p.distance_to(center) + 0.5, 0.0, 1.0)

static func _knob_icon(size: int, fill: Color, ring: Color) -> ImageTexture:
	var image := Image.create_empty(size, size, false, Image.FORMAT_RGBA8)
	var center := Vector2(size, size) * 0.5
	for y in size:
		for x in size:
			var p := Vector2(x + 0.5, y + 0.5)
			var outer := _disc(p, center, size * 0.5 - 0.5)
			var inner := _disc(p, center, size * 0.5 - 3.0)
			var color := ring.lerp(fill, inner)
			color.a *= outer
			image.set_pixel(x, y, color)
	return ImageTexture.create_from_image(image)

static func _dot_icon(size: int, radius: float, color: Color) -> ImageTexture:
	var image := Image.create_empty(size, size, false, Image.FORMAT_RGBA8)
	var center := Vector2(size, size) * 0.5
	for y in size:
		for x in size:
			image.set_pixel(x, y, Color(color, color.a * _disc(Vector2(x + 0.5, y + 0.5), center, radius)))
	return ImageTexture.create_from_image(image)

# A downward chevron, `size` px wide
static func _chevron_icon(size: int, color: Color) -> ImageTexture:
	var height := size / 2 + 2
	var image := Image.create_empty(size, height, false, Image.FORMAT_RGBA8)
	var a := Vector2(1.0, 1.5)
	var b := Vector2(size * 0.5, height - 1.5)
	var c := Vector2(size - 1.0, 1.5)
	for y in height:
		for x in size:
			var p := Vector2(x + 0.5, y + 0.5)
			var d := minf(_segment_distance(p, a, b), _segment_distance(p, b, c))
			image.set_pixel(x, y, Color(color, color.a * clampf(1.4 - d, 0.0, 1.0)))
	return ImageTexture.create_from_image(image)

static func _segment_distance(p: Vector2, a: Vector2, b: Vector2) -> float:
	return p.distance_to(Geometry2D.get_closest_point_to_segment(p, a, b))
