extends Node3D

## Builds the test environment: collidable ground, a couple of obstacles,
## lighting, chase camera and HUD.

# Flip to false to run the Phase 0 kinematic rig instead - handy for looking
# at a gait's foot paths without physics arguing back.
const PHYSICS := true

const GRID_EXTENT := 8.0
const GRID_STEP := 0.25
const LAYER_WORLD := 1

var hexapod: Node3D
var camera: Camera3D
var hud: Label
var cam_target := Vector3.ZERO


func _ready() -> void:
	_build_environment()
	_build_ground()
	_build_obstacles()

	if PHYSICS:
		hexapod = Hexapod.new()
	else:
		hexapod = HexapodKinematic.new()
	hexapod.name = "Hexapod"
	add_child(hexapod)

	camera = Camera3D.new()
	camera.fov = 55.0
	camera.near = 0.01
	add_child(camera)
	cam_target = hexapod.focus_position()

	_build_hud()


func _process(delta: float) -> void:
	# Chase camera: track the body's position but not its yaw, so turning reads
	# as the robot rotating rather than the world spinning.
	cam_target = cam_target.lerp(hexapod.focus_position(), clampf(delta * 3.0, 0.0, 1.0))
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
	var ground := StaticBody3D.new()
	ground.name = "Ground"
	ground.collision_layer = LAYER_WORLD
	ground.physics_material_override = _surface(1.0)
	add_child(ground)

	var cs := CollisionShape3D.new()
	var slab := BoxShape3D.new()
	slab.size = Vector3(GRID_EXTENT * 2.0, 0.1, GRID_EXTENT * 2.0)
	cs.shape = slab
	cs.position.y = -0.05
	ground.add_child(cs)

	var plane := PlaneMesh.new()
	plane.size = Vector2(GRID_EXTENT * 2.0, GRID_EXTENT * 2.0)
	var floor_vis := MeshInstance3D.new()
	floor_vis.mesh = plane
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.30, 0.31, 0.33)
	mat.roughness = 0.95
	floor_vis.material_override = mat
	ground.add_child(floor_vis)

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
	ground.add_child(grid)


## Things to trip over, placed on the path straight ahead (-Z).
func _build_obstacles() -> void:
	# A 12 mm kerb: under the 35 mm foot lift, but enough to tilt the body.
	_block("Step", Vector3(0.6, 0.012, 0.3), Vector3(0.0, 0.006, -0.9), 0.0)

	# A hill: 5 degree ramp up from z = -1.4, a flat top, and a ramp back down.
	# The rise is 7 cm, roughly the robot's standing height, so the top edge
	# would be a cliff without the descent - a guaranteed tumble that says
	# nothing about the gait.
	var slope := deg_to_rad(5.0)
	var length := 0.8
	var rise := length * sin(slope)
	var run := length * cos(slope)
	var top_len := 0.4
	_block("RampUp", Vector3(0.6, 0.02, length),
		Vector3(0.0, 0.5 * rise - 0.01, -1.4 - 0.5 * run), slope)
	_block("HillTop", Vector3(0.6, 0.02, top_len),
		Vector3(0.0, rise - 0.01, -1.4 - run - 0.5 * top_len), 0.0)
	_block("RampDown", Vector3(0.6, 0.02, length),
		Vector3(0.0, 0.5 * rise - 0.01, -1.4 - run - top_len - 0.5 * run), -slope)


func _block(block_name: String, size: Vector3, pos: Vector3, tilt_x: float) -> void:
	var sb := StaticBody3D.new()
	sb.name = block_name
	sb.collision_layer = LAYER_WORLD
	sb.physics_material_override = _surface(1.0)
	sb.position = pos
	sb.rotation.x = tilt_x
	var cs := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = size
	cs.shape = box
	sb.add_child(cs)
	var mesh := BoxMesh.new()
	mesh.size = size
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.52, 0.48, 0.40)
	mat.roughness = 0.9
	mi.material_override = mat
	sb.add_child(mi)
	add_child(sb)


func _surface(friction: float) -> PhysicsMaterial:
	var pm := PhysicsMaterial.new()
	pm.friction = friction
	pm.bounce = 0.0
	return pm


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
	var tripod := "A" if Gait.is_stance(hexapod.phase, 0) else "B"
	var lines := [
		"W/S  forward / back      A/D  strafe      Q/E  turn      R  reset",
		"",
		"command   %5.3f m/s   turn %5.2f rad/s" % [hexapod.cmd_vel.length(), hexapod.cmd_yaw],
		"phase     %5.2f   (tripod %s planted)" % [hexapod.phase, tripod],
	]

	if hexapod is Hexapod:
		var h: Hexapod = hexapod
		var v: Vector3 = h.body.linear_velocity
		lines.append("body      %5.3f m/s   height %5.3f m" % [
			Vector3(v.x, 0.0, v.z).length(), h.body.global_position.y])
		lines.append("")
		lines.append("imu  roll %6.1f  pitch %6.1f  deg      gyro |%5.2f| rad/s   accel |%5.2f| m/s2" % [
			rad_to_deg(h.imu.roll), rad_to_deg(h.imu.pitch), h.imu.gyro.length(), h.imu.accel.length()])
		var feet := PackedStringArray()
		for leg in h.legs:
			feet.append("%s%s" % [leg.name, "\u25a0" if h.contacts.get(leg.name, false) else "\u25a1"])
		lines.append("feet " + "  ".join(feet))
		lines.append("servo     worst tracking error %4.1f deg   (%s / %d ticks)" % [
			h.tracking_error_deg,
			ProjectSettings.get_setting("physics/3d/physics_engine"),
			Engine.physics_ticks_per_second])
		lines.append("spring    k %6.2f N.m/rad   c %5.3f N.m.s/rad     gyro rms %5.3f rad/s     [1/2 k  3/4 c]" % [
			h.servo_stiffness, h.servo_damping, h.imu.gyro_rms()])
		lines.append("torque    peak %5.2f N.m of %4.2f rated     [5/6 rating]" % [h.peak_torque, h.servo_torque])
		if not h.stalled.is_empty():
			lines.append("")
			lines.append("SERVO STALL: " + ", ".join(h.stalled))
		if not h.contact_faults.is_empty():
			lines.append("")
			lines.append("STUMBLE: planted but airborne: " + ", ".join(h.contact_faults))
		if h.fallen():
			lines.append("")
			lines.append("FALLEN - press R")

	if not hexapod.limit_violations.is_empty():
		lines.append("")
		lines.append("SERVO LIMIT EXCEEDED: " + ", ".join(hexapod.limit_violations))
	hud.text = "\n".join(lines)
