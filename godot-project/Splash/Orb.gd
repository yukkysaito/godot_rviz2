extends Node3D
class_name Orb3D

@export_category("Particles")
@export var count: int = 200
@export var radius: float = 0.5
@export var quad_size: float = 0.08
@export var flow_speed: float = 1.2
@export var spread: float = 0.7
@export var fade_speed: float = 3.0
@export_file("*.gdshader") var shader_path: String = "res://Shaders/orb_particles.gdshader"

@export_category("Shell")
@export var shell_fade_time: float = 0.4
@export var shell_expand_scale: float = 1.4

@onready var particles: MultiMeshInstance3D = $Particles
@onready var cam: Camera3D = $Camera3D
@onready var shell: MeshInstance3D = $Shell

const U_TIME: StringName = &"u_time"
const U_RADIUS: StringName = &"u_radius"
const U_MODE: StringName = &"u_mode"
const U_DONE_T: StringName = &"u_done_t"
const U_FLOW: StringName = &"u_flow"
const U_SPREAD: StringName = &"u_spread"
const U_FADE: StringName = &"u_fade"
const U_SHELL_FADE_TIME: StringName = &"u_fade_time"

const MODE_ORB := 0.0
const MODE_DISPERSE := 1.0

var _particle_mat: ShaderMaterial
var _shell_mat: ShaderMaterial
var _shell_mesh: SphereMesh

var _done := false
var _done_t := 0.0

var _shell_radius0 := 0.0
var _shell_height0 := 0.0

func _ready() -> void:
	_setup_camera()
	_setup_particles_material()
	_setup_multimesh()
	_setup_shell()
	set_process(_particle_mat != null or _shell_mat != null)

func _process(delta: float) -> void:
	_update_time_uniform()

	if not _done:
		return

	_done_t += delta
	_set_done_t(_done_t)
	_update_shell_expand(_done_t)

func trigger_done() -> void:
	# Idempotent: safe against repeated calls.
	if _done:
		return

	_done = true
	_done_t = 0.0

	_set_particle_uniform(U_MODE, MODE_DISPERSE)
	_set_done_t(0.0)

func _setup_camera() -> void:
	if cam == null:
		return
	cam.position = Vector3(0.0, 0.0, 3.0)
	cam.look_at(Vector3.ZERO, Vector3.UP)

func _setup_multimesh() -> void:
	if particles == null:
		return

	var quad := QuadMesh.new()
	quad.size = Vector2(quad_size, quad_size)

	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.use_custom_data = true
	mm.mesh = quad
	mm.instance_count = max(count, 0)

	particles.multimesh = mm

	# Custom data encodes: (seed0, seed1, theta_norm, phi_norm)
	var rng := RandomNumberGenerator.new()
	rng.randomize()

	for i in mm.instance_count:
		var u := rng.randf()
		var v := rng.randf()
		var theta := TAU * u
		var phi := acos(2.0 * v - 1.0)

		var seed0 := rng.randf()
		var seed1 := rng.randf()

		mm.set_instance_custom_data(i, Color(seed0, seed1, theta / TAU, phi / PI))
		mm.set_instance_transform(i, Transform3D.IDENTITY)

func _setup_particles_material() -> void:
	var shader := load(shader_path) as Shader
	if shader == null:
		push_error("Orb3D: Failed to load shader: %s" % shader_path)
		return

	_particle_mat = ShaderMaterial.new()
	_particle_mat.shader = shader

	_set_particle_uniform(U_RADIUS, radius)
	_set_particle_uniform(U_MODE, MODE_ORB)
	_set_particle_uniform(U_DONE_T, 0.0)
	_set_particle_uniform(U_FLOW, flow_speed)
	_set_particle_uniform(U_SPREAD, spread)
	_set_particle_uniform(U_FADE, fade_speed)

	if particles != null:
		particles.material_override = _particle_mat

func _setup_shell() -> void:
	if shell == null or shell.mesh == null:
		return

	# Duplicate mesh to avoid mutating shared SphereMesh resources.
	shell.mesh = shell.mesh.duplicate(true)

	_shell_mat = shell.mesh.surface_get_material(0) as ShaderMaterial
	_shell_mesh = shell.mesh as SphereMesh

	if _shell_mat != null:
		_shell_mat.set_shader_parameter(U_DONE_T, 0.0)
		_shell_mat.set_shader_parameter(U_SHELL_FADE_TIME, shell_fade_time)

	if _shell_mesh != null:
		_shell_radius0 = _shell_mesh.radius
		_shell_height0 = _shell_mesh.height

func _update_time_uniform() -> void:
	# Use engine ticks as a stable time source for the shader.
	var t := Time.get_ticks_msec() * 0.001
	_set_particle_uniform(U_TIME, t)

func _set_done_t(v: float) -> void:
	_set_particle_uniform(U_DONE_T, v)
	if _shell_mat != null:
		_shell_mat.set_shader_parameter(U_DONE_T, v)

func _update_shell_expand(dt: float) -> void:
	if _shell_mesh == null:
		return

	# Keep original behavior: linear expansion with factor 0.6.
	var k := 1.0 + dt * 0.6
	_shell_mesh.radius = _shell_radius0 * k
	_shell_mesh.height = _shell_height0 * k

func _set_particle_uniform(var_name: StringName, value) -> void:
	if _particle_mat == null:
		return
	_particle_mat.set_shader_parameter(var_name, value)
