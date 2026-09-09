class_name Gait
extends RefCounted

## Tripod gait generator - also pure, also destined for the ESP32.
##
## The whole gait is one function. Given where a foot rests when standing and
## how fast the body wants to move, it returns where that foot should be right
## now. Each leg calls it independently; the coordination falls out of the
## phase offset between the two tripods rather than from any central state.

## Displacement from a foot's neutral stance position, in the BODY frame.
##
## `neutral`  - the foot's resting position relative to the body centre
## `vel`      - commanded body velocity, m/s, body frame (-Z is forward)
## `yaw_rate` - commanded turn rate, rad/s, positive counter-clockwise
## `phase`    - gait cycle position, wrapping 0..1
## `group`    - 0 or 1, the leg's tripod
static func foot_offset(neutral: Vector3, vel: Vector3, yaw_rate: float, phase: float,
		group: int, cycle := RobotConfig.CYCLE_TIME, lift := RobotConfig.STEP_HEIGHT) -> Vector3:
	# A planted foot must travel backward exactly as fast as the body travels
	# forward, or the robot drags itself. The spin term is omega x r, the extra
	# motion a turn demands of a foot offset from the body centre.
	var spin := Vector3(yaw_rate * neutral.z, 0.0, -yaw_rate * neutral.x)
	var foot_vel := -(vel + spin)

	# Stance occupies half the cycle, and the stroke is centred on neutral, so
	# the foot swings between -amp and +amp.
	var amp := foot_vel * cycle * 0.25

	# The two tripods run half a cycle apart. That single offset is the gait.
	var local := fposmod(phase + (0.5 if group == 1 else 0.0), 1.0)

	if local < 0.5:
		# Stance: on the ground, pushing the body along at a constant rate.
		return (-amp).lerp(amp, local / 0.5)

	# Swing: lift, return to the front of the stroke, set back down. Smoothstep
	# on the horizontal keeps touchdown and lift-off gentle; the sine arc puts
	# peak height at mid-swing and exactly zero at both ends, so the foot never
	# clips into the ground.
	var u := (local - 0.5) / 0.5
	return amp.lerp(-amp, smoothstep(0.0, 1.0, u)) + Vector3.UP * lift * sin(PI * u)


## True when this leg is in stance (foot loaded) at the given phase.
## The real robot cross-checks this against its foot contact switches - a leg
## that should be planted but reads no contact means the robot is falling or
## the ground dropped away.
static func is_stance(phase: float, group: int) -> bool:
	return fposmod(phase + (0.5 if group == 1 else 0.0), 1.0) < 0.5
