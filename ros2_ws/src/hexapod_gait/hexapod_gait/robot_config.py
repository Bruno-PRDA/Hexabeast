"""Single source of truth for the robot's geometry and servo model.

Frames follow ROS: X forward, Y left, Z up. Lengths in metres, masses in kg,
angles in degrees where the name says so. Everything else - the URDF, the
controller list, the gait node, later the firmware - is derived from here.
"""

# --- Leg link lengths: axis to axis, not part length ------------------------
# Measured off the SolidWorks leg (docs/cad-measurements.md) at 59.8 / 139.9 /
# 259.0 mm, then scaled to 42 %. The proportions are the CAD's exactly
# (1 : 2.36 : 4.36 against its 1 : 2.34 : 4.33) - this is the same leg, smaller.
#
# 42 % is set by torque, not by looks. A DT996 gives 1.12 N.m usable, the robot
# masses 1.61 kg, and a tripod puts a third of that on each planted foot at the
# end of the stance lever. 102 mm of stance reach needs 0.54 N.m standing and
# 0.94 N.m climbing - a 1.19x margin. Every extra centimetre of reach spends it.
COXA = 0.025     # hip pivot -> femur pivot
FEMUR = 0.059    # femur pivot -> knee
TIBIA = 0.109    # knee -> foot tip
FOOT_RADIUS = 0.008

# --- Body -------------------------------------------------------------------
BODY_LEN = 0.180
BODY_WID = 0.100
BODY_THK = 0.022

# --- Posture and gait ---------------------------------------------------------
STAND_HEIGHT = 0.081   # body centre above the feet when standing
REACH = 0.102          # horizontal coxa pivot -> foot at neutral stance
STEP_HEIGHT = 0.038    # peak foot lift during swing
CYCLE_TIME = 0.9       # seconds per gait cycle

# --- Servo calibration ----------------------------------------------------------
# Pose to hold each joint in while fitting the horn: that pose is servo centre.
# Found by tools/check_ik.py, which sweeps the walking envelope and reports the
# midpoint of each joint's travel. Re-run it after any geometry change: these
# moved from (0, 22, -94) when the links were rescaled to the CAD.
# Worst-case excursion from centre is then coxa 44, femur 48, tibia 62 deg,
# all inside the 80 deg of usable travel either side.
JOINT_CENTER_DEG = (0.0, 45.0, -105.0)   # coxa, femur, tibia
JOINT_TRAVEL_DEG = 80.0                 # usable travel either side of centre

# --- Servo model: DT996 (MG996R form factor, digital, metal gear, 180 deg) -----
# Sold as 15 kg.cm: 13.5 kg.cm (1.32 N.m) at 4.8 V, 15.2 kg.cm (1.49 N.m) at 6 V.
# Those are labels. Clones of this class routinely measure nearer a genuine
# MG996R's 11 kg.cm, so the figure below is the advertised 6 V torque derated
# 25 % - design to this and the robot still works if the servos under-deliver.
# Run them at 6 V, not 4.8: the extra torque is free and this design uses it.
SERVO_TORQUE = 1.10             # N.m, derated design value (advertised 1.49)
SERVO_SPEED = 6.5               # rad/s no-load (0.16 s per 60 deg at 6 V)
SERVO_REFLECTED_INERTIA = 5e-4  # kg.m^2 at the output shaft, keeps the solver calm
FOOT_FRICTION = 0.8

# --- Masses (kg) - from the CAD's STL volumes plus real servo weights ----------
# A link's mass includes the servo it CARRIES, not the one that drives it.
# Printed parts are 30 % infill (~46 % of solid PLA); the servo STL must never
# be taken at plastic density - a DT996 is motor, metal gears and a PCB, 55 g.
MASS_BODY = 0.79     # 6 coxa servos 330 + battery 260 + electronics 50 + chassis 150
MASS_COXA = 0.065    # femur servo 55 + printed bracket 10
MASS_FEMUR = 0.063   # knee servo 55 + printed link 8
MASS_TIBIA = 0.008   # printed only, no servo beyond it

# --- Leg layout ---------------------------------------------------------------------
# name, hip pivot in the body frame, rest direction (yaw from +X, degrees), tripod.
# Tripod 0 is FL/MR/RL, tripod 1 is FR/ML/RR; they alternate.
LEGS = [
    ("FL", (0.075, 0.055, 0.0), 45.0, 0),
    ("ML", (0.000, 0.070, 0.0), 90.0, 1),
    ("RL", (-0.075, 0.055, 0.0), 135.0, 0),
    ("FR", (0.075, -0.055, 0.0), -45.0, 1),
    ("MR", (0.000, -0.070, 0.0), -90.0, 0),
    ("RR", (-0.075, -0.055, 0.0), -135.0, 1),
]

# Joint names in the order ros2_control and the gait node use.
JOINT_NAMES = [f"{leg}_{joint}" for leg, _m, _y, _g in LEGS for joint in ("coxa", "femur", "tibia")]

# --- Arm (SpiderPi-Pro style: 5 DoF + gripper on the front) ------------------------
# The arm's first joint frame is rotated so its +X points up; pitch axes are
# then +Y (positive tips the link forward) and yaw/roll are +X. Link lengths are
# placeholders until the CAD exists. Set ARM = False for a plain hexapod - and
# drop the arm_controller spawner from sim.launch.py.
ARM = False
ARM_MOUNT = (0.060, 0.0, 0.011)   # top of the body, near the front
# name, axis in the joint frame, link length after the joint (m), link mass (kg), rest angle (deg)
ARM_JOINTS = [
    ("arm_base", "1 0 0", 0.040, 0.06, 0.0),        # yaw about the vertical post
    ("arm_shoulder", "0 1 0", 0.100, 0.07, 35.0),   # pitch, positive tips forward
    ("arm_elbow", "0 1 0", 0.100, 0.06, 100.0),
    ("arm_wrist", "0 1 0", 0.040, 0.05, 45.0),
    ("arm_roll", "1 0 0", 0.030, 0.03, 0.0),
    ("arm_gripper", "0 0 1", 0.040, 0.02, 20.0),    # the moving finger; opening angle
]
ARM_JOINT_NAMES = [j[0] for j in ARM_JOINTS] if ARM else []

# --- Hardware wiring (firmware only; the URDF uses proper joint axes) ---------
# PCA9685 channel per joint, in JOINT_NAMES order: left legs on the board at
# 0x40 (channels 0-15), right legs on the board at 0x41 (16-31).
SERVO_CHANNELS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 16, 17, 18, 19, 20, 21, 22, 23, 24]

# Which way a servo turns for a positive joint angle. The left and right legs
# are mirror images, so a horn that advances the joint on one side retards it
# on the other. These are the mirror assumption, NOT measured - the calibration
# mode in the firmware flips and saves them per joint. Treat as a starting
# guess and verify on the bench before the legs are attached to anything.
SERVO_DIR = [
    +1, -1, -1,   # FL coxa, femur, tibia
    +1, -1, -1,   # ML
    +1, -1, -1,   # RL
    +1, +1, +1,   # FR
    +1, +1, +1,   # MR
    +1, +1, +1,   # RR
]

# Pulse width bounds for a DT996. 500-2500 us spans its ~180 deg; centre is
# nominally 1500 us but every servo differs, which is what per-joint trim fixes.
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_CENTER_US = 1500
SERVO_RANGE_DEG = 180.0    # travel between MIN_US and MAX_US
