class_name RobotConfig
extends RefCounted

## Single source of truth for the robot's geometry.
##
## Everything downstream (simulation, IK, gait, and later the ESP32 firmware)
## reads its dimensions from here, so re-measuring the real robot means editing
## one file. Lengths are metres, angles degrees.

# --- Leg link lengths ---------------------------------------------------
const COXA := 0.030     # hip pivot -> femur pivot (the horizontal stub)
const FEMUR := 0.060    # femur pivot -> knee
const TIBIA := 0.090    # knee -> foot tip

# --- Body ---------------------------------------------------------------
const BODY_LEN := 0.180
const BODY_WID := 0.100
const BODY_THK := 0.022

# --- Posture and gait defaults ------------------------------------------
const STAND_HEIGHT := 0.075  # body underside above ground when standing
const REACH := 0.095         # horizontal coxa-pivot -> foot at neutral stance
const STEP_HEIGHT := 0.035   # peak foot lift during swing
const CYCLE_TIME := 0.9      # seconds for one full gait cycle

# --- Servo calibration --------------------------------------------------
# A joint angle is not a servo angle. Solving the IK for the walking envelope
# shows the tibia living around -94 deg and never near zero, so a horn mounted
# "straight" would spend the whole gait jammed against its end stop.
#
# JOINT_CENTER_DEG is therefore the pose the leg must be held in while the horn
# is fitted: that pose becomes servo centre (1500 us), and the servo then only
# has to travel a little either side of it. These three numbers are the
# assembly instructions for the real robot, and they came out of the sim - see
# tools/check_ik.py, which prints the measured travel per joint.
const JOINT_CENTER_DEG := Vector3(0.0, 22.0, -94.0)

# Usable travel either side of centre. Hobby servos nominally do +/-90, but the
# last few degrees are unreliable under load, so leave margin.
const JOINT_TRAVEL_DEG := 80.0

## Leg layout, front to back.
##
## `yaw` is the direction the leg points away from the body, as a rotation
## about +Y applied to +X. Godot is Y-up with -Z forward, so a rotation of
## `yaw` sends +X to (cos yaw, 0, -sin yaw): 0 deg is straight out to the
## right, 180 deg straight out to the left.
##
## `group` is the tripod set. A hexapod walks by alternating two tripods of
## three legs each - front-left/middle-right/rear-left against
## front-right/middle-left/rear-right. Three feet are always planted, which
## is why a hexapod is statically stable and a quadruped is not.
##
## `channel` is the servo driver output for (coxa, femur, tibia), numbered
## across two PCA9685 boards: 0-15 on the board at I2C 0x40, 16-31 on the one at
## 0x41. Left legs live on the first board and right legs on the second, so
## each board's wiring stays on its own side of the body.
static func legs() -> Array:
	return [
		{"name": "FL", "mount": Vector3(-0.055, 0.0, -0.075), "yaw": 135.0, "group": 0, "channel": [0, 1, 2]},
		{"name": "ML", "mount": Vector3(-0.070, 0.0, 0.0), "yaw": 180.0, "group": 1, "channel": [3, 4, 5]},
		{"name": "RL", "mount": Vector3(-0.055, 0.0, 0.075), "yaw": 225.0, "group": 0, "channel": [6, 7, 8]},
		{"name": "FR", "mount": Vector3(0.055, 0.0, -0.075), "yaw": 45.0, "group": 1, "channel": [16, 17, 18]},
		{"name": "MR", "mount": Vector3(0.070, 0.0, 0.0), "yaw": 0.0, "group": 0, "channel": [19, 20, 21]},
		{"name": "RR", "mount": Vector3(0.055, 0.0, 0.075), "yaw": -45.0, "group": 1, "channel": [22, 23, 24]},
	]
