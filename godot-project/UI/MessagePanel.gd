# MessagePanel.gd
extends Control

@onready var anim: AnimationPlayer = $AnimationPlayer
@onready var label: Label = $CenterContainer/PanelContainer/VBoxContainer/Label

func _ready() -> void:
	# Optional: small delay so it doesn't pop in on scene enter
	await get_tree().process_frame
	if anim.has_animation("fade_in"):
		anim.play("fade_in")

func show_message(text: String) -> void:
	# Update text and play fade-in
	label.text = text
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
