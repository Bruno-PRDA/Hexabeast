class_name Hexapod
extends Node3D

## Phase 0: kinematic hexapod.
##
## The body is moved directly by the velocity command and the legs are solved
## to match. There is no physics here yet - nothing can trip, slip or fall over.
## That is deliberate. This stage exists to prove the geometry, the IK and the
## gait timing are right, which is far easier to see without a physics engine
## arguing with you. Rigid bodies and real contacts come next.

const SPEED := 0.10        # m/s at full stick
const TURN := 1.0          # rad/s at full stick
const BLEND_RATE := 4.0    # how fast the legs settle when the command stops

var legs: Array = []
var phase := 0.0
var blend := 0.0
var cmd_vel := Vector3.ZERO
var cmd_yaw := 0.0

## Latest solved joint angles, radians, keyed by leg name. Read by the HUD and
## by anything that wants to stream poses out to real servos.
var joint_angles := {}
var limit_violations: Array[String] = []


func _ready() -> void:
	position.y = RobotConfig.STAND_HEIGHT
	_build_body()
	for spec in RobotConfig.legs():
		legs.append(_build_leg(spec))


func _physics_process(delta: float) -> void:
	_read_input()

	var moving := cmd_vel.length() > 0.001 or absf(cmd_yaw) > 0.001
	blend = move_toward(blend, 1.0 if moving else 0.0, BLEND_RATE * delta)
	if blend > 0.001:
		phase = fposmod(phase + delta / RobotConfig.CYCLE_TIME, 1.0)

	# Body travels at the commanded velocity; the legs are then solved to keep
	# their feet where they should be. Phase 1 inverts this - feet push, body
	# responds - but for now this is what makes the gait legible.
	translate(cmd_vel * blend * delta)
	rotate_y(cmd_yaw * blend * delta)
	# Incremental rotations accumulate float error in the basis; scrub it
	# each step so the body never drifts off unit scale.
	transform = transform.orthonormalized()

	_solve_legs()


func _read_input() -> void:
	var fwd := 0.0
	var strafe := 0.0
	var turn := 0.0
	if Input.is_physical_key_pressed(KEY_W): fwd -= 1.0
	if Input.is_physical_key_pressed(KEY_S): fwd += 1.0
	if Input.is_physical_key_pressed(KEY_A): strafe -= 1.0
	if Input.is_physical_key_pressed(KEY_D): strafe += 1.0
	if Input.is_physical_key_pressed(KEY_Q): turn += 1.0
	if Input.is_physical_key_pressed(KEY_E): turn -= 1.0
	cmd_vel = Vector3(strafe, 0.0, fwd).limit_length(1.0) * SPEED
	cmd_yaw = turn * TURN


func _solve_legs() -> void:
	limit_violations.clear()
	for leg in legs:
		var offset: Vector3 = Gait.foot_offset(
			leg.neutral, cmd_vel, cmd_yaw, phase, leg.group) * blend
		# Gait works in the body frame; IK wants the leg frame. The leg root's
		# own transform is exactly that change of basis, so let it do the work.
		var target: Vector3 = leg.root.transform.affine_inverse() * (leg.neutral + offset)
		var angles := LegIK.solve(target)

		leg.coxa.rotation.y = angles.x
		leg.femur.rotation.z = angles.y
		leg.tibia.rotation.z = angles.z

		joint_angles[leg.name] = angles
		if not LegIK.within_limits(angles):
			limit_violations.append(leg.name)


# --- rig construction ---------------------------------------------------

func _build_body() -> void:
	var mesh := BoxMesh.new()
	mesh.size = Vector3(RobotConfig.BODY_WID, RobotConfig.BODY_THK, RobotConfig.BODY_LEN)
	var mi := MeshInstance3D.new()
	mi.name = "Body"
	mi.mesh = mesh
	mi.material_override = _mat(Color(0.20, 0.42, 0.72))
	add_child(mi)

	# A nose marker, so which way is forward is never in doubt.
	var nose := MeshInstance3D.new()
	nose.name = "Nose"
	var nose_mesh := BoxMesh.new()
	nose_mesh.size = Vector3(0.016, 0.010, 0.030)
	nose.mesh = nose_mesh
	nose.material_override = _mat(Color(0.95, 0.75, 0.20))
	nose.position = Vector3(0.0, RobotConfig.BODY_THK * 0.5, -RobotConfig.BODY_LEN * 0.5)
	add_child(nose)


func _build_leg(spec: Dictionary) -> Dictionary:
	var root := Node3D.new()
	root.name = "Leg_" + spec.name
	root.position = spec.mount
	root.rotation.y = deg_to_rad(spec.yaw)
	add_child(root)

	var coxa := Node3D.new()
	coxa.name = "Coxa"
	root.add_child(coxa)
	coxa.add_child(_segment(RobotConfig.COXA, 0.016, Color(0.55, 0.57, 0.60)))

	var femur := Node3D.new()
	femur.name = "Femur"
	femur.position = Vector3(RobotConfig.COXA, 0.0, 0.0)
	coxa.add_child(femur)
	femur.add_child(_segment(RobotConfig.FEMUR, 0.014, Color(0.85, 0.45, 0.15)))

	var tibia := Node3D.new()
	tibia.name = "Tibia"
	tibia.position = Vector3(RobotConfig.FEMUR, 0.0, 0.0)
	femur.add_child(tibia)
	tibia.add_child(_segment(RobotConfig.TIBIA, 0.011, Color(0.75, 0.78, 0.82)))

	var foot := MeshInstance3D.new()
	foot.name = "Foot"
	var ball := SphereMesh.new()
	ball.radius = 0.008
	ball.height = 0.016
	foot.mesh = ball
	foot.material_override = _mat(Color(0.15, 0.15, 0.17))
	foot.position = Vector3(RobotConfig.TIBIA, 0.0, 0.0)
	tibia.add_child(foot)

	# Where this foot sits when the robot simply stands, in the body frame.
	var neutral: Vector3 = spec.mount + Basis(Vector3.UP, deg_to_rad(spec.yaw)) \
		* Vector3(RobotConfig.REACH, -RobotConfig.STAND_HEIGHT, 0.0)

	return {
		"name": spec.name, "group": spec.group, "channel": spec.channel,
		"root": root, "coxa": coxa, "femur": femur, "tibia": tibia,
		"foot": foot, "neutral": neutral,
	}


## A limb segment drawn from the pivot outward along +X.
func _segment(length: float, thickness: float, color: Color) -> MeshInstance3D:
	var mesh := BoxMesh.new()
	mesh.size = Vector3(length, thickness, thickness)
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	mi.material_override = _mat(color)
	mi.position = Vector3(length * 0.5, 0.0, 0.0)
	return mi


func _mat(color: Color) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = color
	m.roughness = 0.65
	return m
