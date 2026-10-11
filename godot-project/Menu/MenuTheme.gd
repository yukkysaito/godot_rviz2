class_name MenuTheme

# The settings menu's typography: its title, the section titles (small spaced capitals above a thin
# line, see Selection.gd), the text sizes and the rows' spacing. The panel and the controls keep
# their own look.

const FONT_BOLD := preload("res://UI/Fonts/Barlow-SemiBold.ttf")

static func build() -> Theme:
	var theme := Theme.new()

	# Text a little smaller than the default, also in the choices' lists
	theme.set_font_size("font_size", "Label", 15)
	theme.set_font_size("font_size", "OptionButton", 14)
	theme.set_font_size("font_size", "PopupMenu", 14)
	theme.set_constant("v_separation", "PopupMenu", 6)

	# Rows: a label and its control
	var row := StyleBoxEmpty.new()
	row.content_margin_top = 7
	row.content_margin_bottom = 7
	theme.set_stylebox("normal", "Label", row)
	theme.set_constant("separation", "HBoxContainer", 12)
	theme.set_constant("separation", "VBoxContainer", 2)

	# Section titles
	var title_font := FontVariation.new()
	title_font.base_font = FONT_BOLD
	title_font.spacing_glyph = 2
	theme.set_type_variation("SectionTitle", "Label")
	theme.set_font("font", "SectionTitle", title_font)
	theme.set_font_size("font_size", "SectionTitle", 13)
	theme.set_color("font_color", "SectionTitle", Color(0.7, 0.72, 0.76))
	var title_box := StyleBoxFlat.new()
	title_box.bg_color = Color(0, 0, 0, 0)
	title_box.border_color = Color(1, 1, 1, 0.1)
	title_box.border_width_top = 1
	title_box.content_margin_top = 16
	title_box.content_margin_bottom = 6
	theme.set_stylebox("normal", "SectionTitle", title_box)

	# The menu's title
	theme.set_type_variation("MenuTitle", "Label")
	theme.set_font("font", "MenuTitle", FONT_BOLD)
	theme.set_font_size("font_size", "MenuTitle", 24)
	return theme
