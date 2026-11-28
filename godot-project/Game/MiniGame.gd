extends CanvasLayer

# The mini game, "Planner Challenge" (Game/PlannerGame.gd), as a self-contained add-on: this
# autoload (MiniGame in project.godot) is its only hook into the viewer. To remove the game, delete
# the Game folder and that autoload line; nothing else refers to it.
#
# It is on unless the project or the preset turns it off with
#   [game]
#   enabled=false
# in Config/project.cfg or Config/Presets/<preset>.cfg (the presets for a real vehicle do). The
# viewer's settings (Core/Settings.gd) ignore that section, so the lines are harmless without the
# game.

const PROJECT_FILE := "res://Config/project.cfg"
const PRESET_DIR := "res://Config/Presets/"

func _ready() -> void:
	layer = 10  # over the viewer's HUD and menu
	if not _enabled():
		return
	var slide := GameSlide.new()
	slide.name = "GameSlide"
	add_child(slide)

static func _enabled() -> bool:
	var enabled := true
	var files := [PROJECT_FILE]
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--preset="):
			files.append(PRESET_DIR + arg.trim_prefix("--preset=") + ".cfg")
	for path in files:
		var config := ConfigFile.new()
		if FileAccess.file_exists(path) and config.load(path) == OK:
			enabled = bool(config.get_value("game", "enabled", enabled))
	return enabled
