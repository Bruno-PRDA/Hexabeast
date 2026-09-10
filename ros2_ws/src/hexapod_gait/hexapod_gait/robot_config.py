"""Single source of truth for the robot's geometry and servo model.

Frames follow ROS: X forward, Y left, Z up. Lengths in metres, masses in kg,
angles in degrees where the name says so. Everything else - the URDF, the
controller list, the gait node, later the firmware - is derived from here.
"""

# --- Leg link lengths: axis to axis, not part length ------------------------
COXA = 0.030     # hip pivot -> femur pivot
FEMUR = 0.060    # femur pivot -> knee
TIBIA = 0.090    # knee -> foot tip
FOOT_RADIUS = 0.008

# --- Body -------------------------------------------------------------------
BODY_LEN = 0.180
BODY_WID = 0.100
BODY_THK = 0.022

# --- Posture and gait ---------------------------------------------------------
STAND_HEIGHT = 0.075   # body centre above the feet when standing
REACH = 0.095          # horizontal coxa pivot -> foot at neutral stance
STEP_HEIGHT = 0.035    # peak foot lift during swing
CYCLE_TIME = 0.9       # seconds per gait cycle

# --- Servo calibration ----------------------------------------------------------
# Pose to hold each joint in while fitting the horn: that pose is servo centre.
# Found by tools/check_ik.py; the tibia lives around -94 deg for the whole gait.
JOINT_CENTER_DEG = (0.0, 22.0, -94.0)   # coxa, femur, tibia
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

# --- Masses (kg) - weigh the real parts, the torque margin scales with them ----
MASS_BODY = 0.45     # frame + ESP32 + two PCA9685 + 2S battery
MASS_COXA = 0.06     # essentially one servo
MASS_FEMUR = 0.065   # servo + bracket
MASS_TIBIA = 0.02

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

# PCA9685 channel per joint, same order: left legs on the board at 0x40
# (channels 0-15), right legs on the board at 0x41 (16-31).
SERVO_CHANNELS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 16, 17, 18, 19, 20, 21, 22, 23, 24]
