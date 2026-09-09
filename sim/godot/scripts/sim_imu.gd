class_name SimIMU
extends RefCounted

## Simulated 6-axis IMU, rigidly mounted at the body centre.
##
## Sensor frame is the body frame: +X right, +Y up, -Z forward. Mount the real
## chip to match, or add a fixed rotation here - not in the control code.
##
## Reports what an IMU actually reports, noise included. In particular the
## accelerometer measures specific force, not acceleration: a body at rest
## reads +9.81 on its up axis, not zero. Firmware that forgets this levels
## the robot upside down.

const GYRO_NOISE := 0.005    # rad/s, one sigma - MPU6050 class
const ACCEL_NOISE := 0.03    # m/s^2

var gyro := Vector3.ZERO           # rad/s, sensor frame
var accel := Vector3(0.0, 9.81, 0.0)  # m/s^2, sensor frame

# Ground-truth orientation, for the HUD and for checking a fusion filter
# against. A real IMU has to estimate these from the two vectors above.
var roll := 0.0    # about the forward axis
var pitch := 0.0   # about the sideways axis
var yaw := 0.0

var _prev_vel := Vector3.ZERO
var _primed := false
var _gyro_ms := 0.0   # running mean square of |gyro|, ~0.25 s window


func update(body: RigidBody3D, dt: float) -> void:
	var to_sensor := body.global_basis.transposed()
	var vel := body.linear_velocity
	var world_accel := (vel - _prev_vel) / dt if _primed else Vector3.ZERO
	_prev_vel = vel
	_primed = true

	var g: float = ProjectSettings.get_setting("physics/3d/default_gravity", 9.8)
	var gravity := Vector3(0.0, -g, 0.0)
	accel = to_sensor * (world_accel - gravity) + _noise(ACCEL_NOISE)
	gyro = to_sensor * body.angular_velocity + _noise(GYRO_NOISE)
	_gyro_ms = lerpf(_gyro_ms, gyro.length_squared(), minf(1.0, dt * 4.0))

	# Godot's default YXZ Euler order hands back (pitch, yaw, roll) for a
	# Y-up, -Z-forward body.
	var e := body.global_basis.get_euler()
	pitch = e.x
	yaw = e.y
	roll = e.z


## Recent RMS angular rate. A standing robot should read close to the noise
## floor; anything above ~0.1 rad/s means the servo model is vibrating.
func gyro_rms() -> float:
	return sqrt(_gyro_ms)


static func _noise(sigma: float) -> Vector3:
	return Vector3(randfn(0.0, sigma), randfn(0.0, sigma), randfn(0.0, sigma))
