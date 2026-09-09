extends Node3D

## Builds the test environment: ground, grid, lighting, chase camera and HUD.

const GRID_EXTENT := 8.0
const GRID_STEP := 0.25

var hexapod: Hexapod
var camera: Camera3D
var hud: Label
var cam_target := Vector3.ZERO


func _ready() -> void:
	_build_environment()
	_build_ground()

	hexapod = Hexapod.new()
	hexapod.name = "Hexapod"
	add_child(hexapod)

	camera = Camera3D.new()
	camera.fov = 55.0
	camera.near = 0.01
	add_child(camera)
	cam_target = hexapod.global_position

	_build_hud()


func _process(delta: float) -> void:
	# Chase camera: track the body's position but not its yaw, so turning reads
	# as the robot rotating rather than the world spinning.
	cam_target = cam_target.lerp(hexapod.global_position, clampf(delta * 3.0, 0.0, 1.0))
	camera.global_position = cam_target + Vector3(0.42, 0.34, 0.52)
	camera.look_at(cam_target + Vector3(0.0, 0.02, 0.0), Vector3.UP)
	_update_hud()


func _build_environment() -> void:
	var sky_mat := ProceduralSkyMaterial.new()
	sky_mat.sky_top_color = Color(0.28, 0.38, 0.52)
	sky_mat.sky_horizon_color = Color(0.62, 0.66, 0.70)
	sky_mat.ground_bottom_color = Color(0.16, 0.17, 0.19)
	sky_mat.ground_horizon_color = Color(0.42, 0.43, 0.45)

	var sky := Sky.new()
	sky.sky_material = sky_mat

	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.ambient_light_energy = 0.6
	env.tonemap_mode = Environment.TONE_MAPPER_FILMIC

	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-52.0, -38.0, 0.0)
	sun.light_energy = 1.1
	sun.shadow_enabled = true
	add_child(sun)


func _build_ground() -> void:
	var plane := PlaneMesh.new()
	plane.size = Vector2(GRID_EXTENT * 2.0, GRID_EXTENT * 2.0)
	var ground := MeshInstance3D.new()
	ground.name = "Ground"
	ground.mesh = plane
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.30, 0.31, 0.33)
	mat.roughness = 0.95
	ground.material_override = mat
	add_child(ground)

	# A grid gives the eye something fixed to judge the robot's motion against.
	# Without it, a walking robot and a sliding robot look identical.
	var line_mat := StandardMaterial3D.new()
	line_mat.albedo_color = Color(0.40, 0.42, 0.45)
	var grid := MultiMeshInstance3D.new()
	grid.name = "Grid"
	var line := BoxMesh.new()
	line.size = Vector3(GRID_EXTENT * 2.0, 0.0012, 0.004)
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.mesh = line
	var steps := int(GRID_EXTENT * 2.0 / GRID_STEP) + 1
	mm.instance_count = steps * 2
	var i := 0
	for s in steps:
		var offset := -GRID_EXTENT + s * GRID_STEP
		mm.set_instance_transform(i, Transform3D(Basis(), Vector3(0.0, 0.001, offset)))
		i += 1
		mm.set_instance_transform(i, Transform3D(
			Basis(Vector3.UP, PI * 0.5), Vector3(offset, 0.001, 0.0)))
		i += 1
	grid.multimesh = mm
	grid.material_override = line_mat
	add_child(grid)


func _build_hud() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	hud = Label.new()
	hud.position = Vector2(16, 12)
	hud.add_theme_color_override("font_color", Color(0.92, 0.94, 0.96))
	hud.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.8))
	hud.add_theme_constant_override("outline_size", 5)
	layer.add_child(hud)


func _update_hud() -> void:
	var speed := hexapod.cmd_vel.length()
	var tripod := "A" if Gait.is_stance(hexapod.phase, 0) else "B"
	var lines := [
		"W/S  forward / back      A/D  strafe      Q/E  turn",
		"",
		"speed      %5.3f m/s" % speed,
		"turn       %5.2f rad/s" % hexapod.cmd_yaw,
		"phase      %5.2f   (tripod %s planted)" % [hexapod.phase, tripod],
		"stand      %5.3f m" % RobotConfig.STAND_HEIGHT,
	]
	if not hexapod.limit_violations.is_empty():
		lines.append("")
		lines.append("SERVO LIMIT EXCEEDED: " + ", ".join(hexapod.limit_violations))
	hud.text = "\n".join(lines)
