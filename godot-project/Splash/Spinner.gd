extends Control

@export var dot_count: int = 12
@export var radius: float = 26.0
@export var dot_size: float = 3.5
@export var speed: float = 1.8
@export var base_alpha: float = 0.25
@export var peak_alpha: float = 1.0

var _t: float = 0.0

func _ready() -> void:
	set_process(true)
	queue_redraw()

func _process(delta: float) -> void:
	# Accumulate time and redraw every frame to animate the spinner.
	_t += delta
	queue_redraw()

func _draw() -> void:
	var center: Vector2 = size * 0.5

	# Guard against division by zero (just in case).
	var n: int = max(dot_count, 1)
	var step: float = TAU / float(n)

	# Phase of the "leading" bright dot. Keeping this outside the loop avoids recomputing it.
	var lead_phase: float = _t * speed

	for i: int in range(n):
		var ang: float = lead_phase + float(i) * step
		var pos: Vector2 = center + Vector2(cos(ang), sin(ang)) * radius

		# Phase difference: closer to 0 => brighter (Windows-like "bright head").
		var phase: float = fposmod(lead_phase - (float(i) * step), TAU)

		# Normalize to [0, 1] where 1.0 is the head and 0.0 is the tail.
		var headness: float = 1.0 - (phase / TAU)

		# Bias the curve so the head stands out (cubic falloff).
		var a: float = lerp(base_alpha, peak_alpha, headness * headness * headness)

		draw_circle(pos, dot_size, Color(1.0, 1.0, 1.0, a))
