class_name LegIK
extends RefCounted

## Inverse kinematics for one 3-DoF leg: coxa (yaw), femur (pitch), tibia (pitch).
##
## Deliberately pure - no nodes, no engine state, no side effects. That is what
## makes it portable: this same arithmetic is what will run on the ESP32, so
## anything validated here is validated for the real robot too. Keep it that way.

## Solve for the joint angles that put the foot at `target`.
##
## `target` is in the LEG frame: origin at the coxa pivot, +X pointing outward
## along the leg's rest direction, +Y up. Returns (coxa, femur, tibia) radians.
##
## Targets outside the reachable annulus are clamped to the nearest point the
## leg can actually reach rather than returning NaN, so a bad gait parameter
## produces a visibly wrong pose instead of an exploded robot.
static func solve(target: Vector3, coxa := RobotConfig.COXA, femur := RobotConfig.FEMUR, tibia := RobotConfig.TIBIA) -> Vector3:
	# Coxa simply yaws to face the target's horizontal bearing. A +Y rotation
	# carries +X toward -Z, hence the negated Z.
	var coxa_angle := atan2(-target.z, target.x)

	# Reduce to a 2-link planar problem in the vertical plane the leg now
	# occupies: `horiz` out from the femur pivot, `vert` up.
	var horiz := sqrt(target.x * target.x + target.z * target.z) - coxa
	var vert := target.y
	var dist := sqrt(horiz * horiz + vert * vert)

	# Clamp into the annulus femur+tibia can span, with a hair of margin so the
	# law-of-cosines terms never sit exactly on +/-1.
	var near := absf(femur - tibia) + 0.001
	var far := femur + tibia - 0.001
	dist = clampf(dist, near, far)

	# Law of cosines. The `+acos` branch picks the knee-up solution, which is
	# the one a hexapod uses.
	var bearing := atan2(vert, horiz)
	var femur_angle := bearing + acos(clampf(
		(femur * femur + dist * dist - tibia * tibia) / (2.0 * femur * dist), -1.0, 1.0))

	# Interior angle at the knee. Subtracting PI re-expresses it relative to the
	# femur, so a straight leg reads as 0 and the knee folds negative (downward).
	var knee := acos(clampf(
		(femur * femur + tibia * tibia - dist * dist) / (2.0 * femur * tibia), -1.0, 1.0))

	return Vector3(coxa_angle, femur_angle, knee - PI)


## Forward kinematics - foot position in the leg frame for a given pose.
## Used to check that solve() actually hit the target it was given.
static func foot_position(angles: Vector3, coxa := RobotConfig.COXA, femur := RobotConfig.FEMUR, tibia := RobotConfig.TIBIA) -> Vector3:
	var horiz := coxa + femur * cos(angles.y) + tibia * cos(angles.y + angles.z)
	var vert := femur * sin(angles.y) + tibia * sin(angles.y + angles.z)
	return Vector3(horiz * cos(angles.x), vert, -horiz * sin(angles.x))


## Joint angles -> servo angles, radians, relative to each servo's centre.
## This is the number that actually gets turned into a PWM pulse width.
static func to_servo(angles: Vector3) -> Vector3:
	return angles - RobotConfig.JOINT_CENTER_DEG * (PI / 180.0)


## True when every servo would sit inside its usable travel for this pose.
static func within_limits(angles: Vector3) -> bool:
	var travel := deg_to_rad(RobotConfig.JOINT_TRAVEL_DEG)
	var servo := to_servo(angles)
	return absf(servo.x) <= travel and absf(servo.y) <= travel and absf(servo.z) <= travel
