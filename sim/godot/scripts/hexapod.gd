class_name Hexapod
extends Node3D

## Phase 1: physics hexapod.
##
## The body is a rigid body and every leg link is a rigid body of its own,
## chained with 6-DoF joints locked down to a single hinge. Each hinge carries
## an angular spring whose equilibrium point is the commanded servo angle -
## that is the servo model: a stiff position spring with a torque cap and a
## speed cap, which is what a hobby servo's internal control loop amounts to
## from the outside.
##
## Nothing moves the body directly any more. The gait commands foot positions,
## IK turns them into joint angles, the springs pull the links there, and the
## feet push against the floor. Whether the robot goes anywhere is now a
## question for the physics engine - which is the point.
##
## The rig is built in the servo-centre pose from RobotConfig. A joint's zero
## is captured from wherever the links are when it is created, so building
## there makes every joint angle equal to its servo angle - the number the
## firmware will actually write.

const SPEED := 0.10
const TURN := 1.0
const BLEND_RATE := 4.0
const STARTUP_TIME := 1.0   # ease from the boot pose into the commanded pose

# Collision layers: robot parts only look for the world, never each other.
# Adjacent links overlap at every joint and would otherwise fight.
const LAYER_WORLD := 1
const LAYER_ROBOT := 2

const AXIS_Y := 1
const AXIS_Z := 2

var body: RigidBody3D
var legs: Array = []
var phase := 0.0
var blend := 0.0
var startup := 0.0
var cmd_vel := Vector3.ZERO
var cmd_yaw := 0.0
var imu := SimIMU.new()

# Live-tunable copies of the servo spring, for finding stable values without
# a restart per guess. Keys 1/2 halve/double stiffness, 3/4 damping.
var servo_stiffness: float = RobotConfig.SERVO_STIFFNESS
var servo_damping: float = RobotConfig.SERVO_DAMPING
var servo_torque: float = RobotConfig.SERVO_TORQUE   # keys 5/6 halve/double

# The spring's torque cap is a server-level joint parameter that the node API
# does not expose - the node's motor force limit only governs the velocity
# motor. Resolved by name at startup so an engine without it degrades to an
# uncapped spring instead of a parse error.
var _torque_cap_param := -1

var joint_angles := {}                     # commanded joint angles by leg name
var limit_violations: Array[String] = []
var tracking_error_deg := 0.0              # worst commanded-vs-measured servo error
var peak_torque := 0.0                     # highest torque any servo has had to supply, N.m
var stalled: Array[String] = []            # joints demanding more than the servo has
var contacts := {}                         # leg name -> tibia touching something
var contact_faults: Array[String] = []     # planted by the gait, airborne in fact


func _ready() -> void:
	if ClassDB.class_has_integer_constant("PhysicsServer3D", "G6DOF_JOINT_ANGULAR_DRIVE_TORQUE_LIMIT"):
		_torque_cap_param = ClassDB.class_get_integer_constant(
			"PhysicsServer3D", "G6DOF_JOINT_ANGULAR_DRIVE_TORQUE_LIMIT")
		print("servo torque capped at %.2f N.m" % RobotConfig.SERVO_TORQUE)
	else:
		push_warning("PhysicsServer3D has no angular drive torque limit - servo springs are uncapped")
	_build_body()
	for spec in RobotConfig.legs():
		legs.append(_build_leg(spec))


func focus_position() -> Vector3:
	return body.global_position


func fallen() -> bool:
	return absf(imu.roll) > deg_to_rad(60.0) or absf(imu.pitch) > deg_to_rad(60.0)


func _physics_process(delta: float) -> void:
	_read_input()
	startup = minf(startup + delta / STARTUP_TIME, 1.0)

	var moving := cmd_vel.length() > 0.001 or absf(cmd_yaw) > 0.001
	blend = move_toward(blend, 1.0 if moving else 0.0, BLEND_RATE * delta)
	if blend > 0.001:
		phase = fposmod(phase + delta / RobotConfig.CYCLE_TIME, 1.0)

	_drive_servos(delta)
	imu.update(body, delta)
	_read_contacts()


func _input(event: InputEvent) -> void:
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	var key: int = event.physical_keycode if event.physical_keycode != 0 else event.keycode
	match key:
		KEY_1: servo_stiffness *= 0.5
		KEY_2: servo_stiffness *= 2.0
		KEY_3: servo_damping *= 0.5
		KEY_4: servo_damping *= 2.0
		KEY_5: servo_torque *= 0.5
		KEY_6: servo_torque *= 2.0
		KEY_R:
			get_tree().call_deferred("reload_current_scene")
			return
		_: return
	_apply_servo_params()


func _apply_servo_params() -> void:
	for leg in legs:
		for i in 3:
			var axis := AXIS_Y if i == 0 else AXIS_Z
			_joint_param(leg.joints[i], axis, Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_STIFFNESS, servo_stiffness)
			_joint_param(leg.joints[i], axis, Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_DAMPING, servo_damping)


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


func _drive_servos(delta: float) -> void:
	limit_violations.clear()
	stalled.clear()
	tracking_error_deg = 0.0
	var slew := RobotConfig.SERVO_SPEED * delta

	for leg in legs:
		var offset: Vector3 = Gait.foot_offset(
			leg.neutral, cmd_vel, cmd_yaw, phase, leg.group) * blend
		var target: Vector3 = leg.root_xf.affine_inverse() * (leg.neutral + offset)
		var angles := LegIK.solve(target)
		joint_angles[leg.name] = angles
		if not LegIK.within_limits(angles):
			limit_violations.append(leg.name)

		# Ease in from the boot pose (every servo centred) over the first
		# second, then chase the command no faster than the servo is rated for.
		var want := LegIK.to_servo(angles) * startup
		var servo: Vector3 = leg.servo
		servo.x = move_toward(servo.x, want.x, slew)
		servo.y = move_toward(servo.y, want.y, slew)
		servo.z = move_toward(servo.z, want.z, slew)
		leg.servo = servo

		# The spring deflection is a torque gauge: what the servo is supplying
		# right now is k * error. A real servo cannot supply more than its
		# rating, so when the demand exceeds it, let the equilibrium trail the
		# measured angle at exactly the rated-torque deflection. The spring then
		# never pulls harder than the servo could - it stalls, like the real one,
		# instead of flinging the robot with an infinitely strong motor.
		var measured := _measure(leg)
		var stall_err := servo_torque / servo_stiffness
		var eq := servo
		for i in 3:
			var err: float = wrapf(measured[i] - servo[i], -PI, PI)
			tracking_error_deg = maxf(tracking_error_deg, rad_to_deg(absf(err)))
			var torque := servo_stiffness * absf(err)
			if torque > servo_torque:
				eq[i] = measured[i] - signf(err) * stall_err
				torque = servo_torque
				stalled.append("%s.%s" % [leg.name, ["coxa", "femur", "tibia"][i]])
			peak_torque = maxf(peak_torque * (1.0 - delta * 0.5), torque)   # slow decay

		# Godot's 6-DoF joint takes angular targets with the opposite sign to a
		# right-handed rotation about the axis - a convention inherited from
		# Bullet - so the equilibrium is negated here. _measure() reports the
		# geometric sign, which is also the one the servo channels use.
		leg.joints[0].set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_EQUILIBRIUM_POINT, -eq.x)
		leg.joints[1].set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_EQUILIBRIUM_POINT, -eq.y)
		leg.joints[2].set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_EQUILIBRIUM_POINT, -eq.z)


func _read_contacts() -> void:
	contact_faults.clear()
	for leg in legs:
		var down: bool = leg.links[2].get_contact_count() > 0
		contacts[leg.name] = down
		# Mid-stance with nothing under the foot is the signature of a stumble,
		# or of the ground dropping away. Ignore the edges of stance where the
		# foot is still arriving or already leaving.
		var progress := Gait.stance_progress(phase, leg.group)
		if blend > 0.5 and progress >= 0.2 and progress <= 0.8 and not down:
			contact_faults.append(leg.name)


## Measured servo angles for one leg, from the links' actual relative poses.
func _measure(leg: Dictionary) -> Vector3:
	var c: float = _hinge_angle(body, leg.links[0], AXIS_Y) - leg.rest.x
	var f: float = _hinge_angle(leg.links[0], leg.links[1], AXIS_Z) - leg.rest.y
	var t: float = _hinge_angle(leg.links[1], leg.links[2], AXIS_Z) - leg.rest.z
	return Vector3(wrapf(c, -PI, PI), wrapf(f, -PI, PI), wrapf(t, -PI, PI))


static func _hinge_angle(parent: Node3D, child: Node3D, axis: int) -> float:
	# The child's X axis expressed in the parent's frame; a hinge rotation shows
	# up as that vector's angle in the plane perpendicular to the hinge.
	var x := (parent.global_basis.transposed() * child.global_basis).x
	if axis == AXIS_Y:
		return atan2(-x.z, x.x)
	return atan2(x.y, x.x)


# --- rig construction ---------------------------------------------------

func _build_body() -> void:
	body = RigidBody3D.new()
	body.name = "Body"
	body.mass = RobotConfig.MASS_BODY
	body.position = Vector3(0.0, RobotConfig.SPAWN_HEIGHT, 0.0)
	body.can_sleep = false
	body.collision_layer = LAYER_ROBOT
	body.collision_mask = LAYER_WORLD
	var size := Vector3(RobotConfig.BODY_WID, RobotConfig.BODY_THK, RobotConfig.BODY_LEN)
	body.add_child(_box_collider(size, Vector3.ZERO))
	body.add_child(_box_visual(size, Vector3.ZERO, Color(0.20, 0.42, 0.72)))
	body.add_child(_box_visual(Vector3(0.016, 0.010, 0.030),
		Vector3(0.0, RobotConfig.BODY_THK * 0.5, -RobotConfig.BODY_LEN * 0.5),
		Color(0.95, 0.75, 0.20)))
	add_child(body)


func _build_leg(spec: Dictionary) -> Dictionary:
	var center: Vector3 = RobotConfig.JOINT_CENTER_DEG * (PI / 180.0)
	var yaw: float = deg_to_rad(spec.yaw)

	# Rest transforms in body space, chained in the servo-centre pose.
	var root_xf := Transform3D(Basis(Vector3.UP, yaw), spec.mount)
	var coxa_xf := root_xf * Transform3D(Basis(Vector3.UP, center.x), Vector3.ZERO)
	var femur_xf := coxa_xf * Transform3D(
		Basis(Vector3.BACK, center.y), Vector3(RobotConfig.COXA, 0.0, 0.0))
	var tibia_xf := femur_xf * Transform3D(
		Basis(Vector3.BACK, center.z), Vector3(RobotConfig.FEMUR, 0.0, 0.0))

	var coxa := _link("Coxa_" + spec.name, body.transform * coxa_xf,
		RobotConfig.MASS_COXA, RobotConfig.COXA, 0.016, Color(0.55, 0.57, 0.60))
	var femur := _link("Femur_" + spec.name, body.transform * femur_xf,
		RobotConfig.MASS_FEMUR, RobotConfig.FEMUR, 0.014, Color(0.85, 0.45, 0.15))
	var tibia := _link("Tibia_" + spec.name, body.transform * tibia_xf,
		RobotConfig.MASS_TIBIA, RobotConfig.TIBIA, 0.011, Color(0.75, 0.78, 0.82), true)

	# Rubber foot and the contact sensor both live on the tibia.
	var foot := CollisionShape3D.new()
	var ball := SphereShape3D.new()
	ball.radius = 0.008
	foot.shape = ball
	foot.position = Vector3(RobotConfig.TIBIA, 0.0, 0.0)
	tibia.add_child(foot)
	var foot_mesh := SphereMesh.new()
	foot_mesh.radius = 0.008
	foot_mesh.height = 0.016
	var foot_vis := MeshInstance3D.new()
	foot_vis.mesh = foot_mesh
	foot_vis.material_override = _mat(Color(0.15, 0.15, 0.17))
	foot_vis.position = foot.position
	tibia.add_child(foot_vis)
	var rubber := PhysicsMaterial.new()
	rubber.friction = RobotConfig.FOOT_FRICTION
	rubber.bounce = 0.0
	tibia.physics_material_override = rubber
	tibia.contact_monitor = true
	tibia.max_contacts_reported = 4

	var joints := [
		_hinge(body, coxa, body.transform * root_xf, AXIS_Y),
		_hinge(coxa, femur, body.transform * femur_xf, AXIS_Z),
		_hinge(femur, tibia, body.transform * tibia_xf, AXIS_Z),
	]

	var neutral: Vector3 = spec.mount + Basis(Vector3.UP, yaw) \
		* Vector3(RobotConfig.REACH, -RobotConfig.STAND_HEIGHT, 0.0)

	return {
		"name": spec.name, "group": spec.group, "channel": spec.channel,
		"root_xf": root_xf, "neutral": neutral,
		"links": [coxa, femur, tibia], "joints": joints,
		# Relative rotation each hinge shows at rest; measured angles subtract it.
		"rest": Vector3(yaw + center.x, center.y, center.z),
		"servo": Vector3.ZERO,
	}


func _link(link_name: String, xf: Transform3D, mass: float, length: float,
		thickness: float, color: Color, rounded := false) -> RigidBody3D:
	var rb := RigidBody3D.new()
	rb.name = link_name
	rb.transform = xf
	rb.mass = mass
	rb.inertia = Vector3.ONE * RobotConfig.SERVO_REFLECTED_INERTIA
	rb.can_sleep = false
	rb.collision_layer = LAYER_ROBOT
	rb.collision_mask = LAYER_WORLD
	var size := Vector3(length, thickness, thickness)
	var offset := Vector3(length * 0.5, 0.0, 0.0)
	if rounded:
		# A capsule has no edges to snag on a step; the box is only for looks.
		var cs := CollisionShape3D.new()
		var capsule := CapsuleShape3D.new()
		capsule.radius = thickness * 0.5
		capsule.height = length
		cs.shape = capsule
		cs.position = offset
		cs.rotation.z = PI * 0.5
		rb.add_child(cs)
	else:
		rb.add_child(_box_collider(size, offset))
	rb.add_child(_box_visual(size, offset, color))
	add_child(rb)
	return rb


## One servo: a 6-DoF joint with everything locked except `axis`, which gets
## the servo's travel limits, its position spring and its torque cap.
func _hinge(a: PhysicsBody3D, b: PhysicsBody3D, frame: Transform3D, axis: int) -> Generic6DOFJoint3D:
	var j := Generic6DOFJoint3D.new()
	j.name = "Servo_" + b.name
	j.transform = frame
	var travel := deg_to_rad(RobotConfig.JOINT_TRAVEL_DEG)
	_joint_param(j, axis, Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, -travel)
	_joint_param(j, axis, Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, travel)
	_joint_param(j, axis, Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_STIFFNESS, servo_stiffness)
	_joint_param(j, axis, Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_DAMPING, servo_damping)
	_joint_param(j, axis, Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_EQUILIBRIUM_POINT, 0.0)
	_joint_param(j, axis, Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, RobotConfig.SERVO_TORQUE)
	if axis == AXIS_Y:
		j.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_SPRING, true)
	else:
		j.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_SPRING, true)
	add_child(j)
	j.node_a = j.get_path_to(a)
	j.node_b = j.get_path_to(b)
	if _torque_cap_param >= 0:
		PhysicsServer3D.generic_6dof_joint_set_param(
			j.get_rid(), axis as Vector3.Axis, _torque_cap_param as PhysicsServer3D.G6DOFJointAxisParam,
			RobotConfig.SERVO_TORQUE)
	return j


static func _joint_param(j: Generic6DOFJoint3D, axis: int, param: int, value: float) -> void:
	if axis == AXIS_Y:
		j.set_param_y(param, value)
	else:
		j.set_param_z(param, value)


func _box_collider(size: Vector3, offset: Vector3) -> CollisionShape3D:
	var cs := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = size
	cs.shape = box
	cs.position = offset
	return cs


func _box_visual(size: Vector3, offset: Vector3, color: Color) -> MeshInstance3D:
	var mesh := BoxMesh.new()
	mesh.size = size
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	mi.material_override = _mat(color)
	mi.position = offset
	return mi


func _mat(color: Color) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = color
	m.roughness = 0.65
	return m
