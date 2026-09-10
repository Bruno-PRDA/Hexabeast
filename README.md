# Hexabeast

An 18-servo FPV hexapod: designed in SolidWorks, simulated in Gazebo, driven
by ROS 2, and flown from a phone over its own WiFi.

| | |
|---|---|
| Size | 34 cm foot span, 18 cm body, ~1.3 kg |
| Servos | 18x DT996 (MG996R form factor, 15 kg.cm digital) at 6 V |
| Brain | ESP32-S3 camera module, SoftAP, MJPEG video + WebSocket control |
| Drivers | 2x PCA9685, 32 PWM channels from two I2C pins |

Sizing is not arbitrary: `tools/scale_torque.py` puts the DT996 ceiling at
~41 cm foot span, and 34 cm leaves a comfortable margin. See
[docs/hardware.md](docs/hardware.md).

## The one rule

**The robot's brain never touches hardware directly.** Inverse kinematics,
gait generation and body control are pure functions of numbers in, numbers
out. Sensors and servos sit behind a thin interface with two implementations:
one that talks to real pins, one that talks to the simulator. Same control
code in both worlds - so a gait that walks in the sim walks on the robot, and
a bug found on the robot can be reproduced in the sim.

## Layout

```
ros2_ws/src/
  hexapod_description/   URDF (placeholder generated from the config; the SolidWorks
                         export replaces its body half), ros2_control config, Gazebo
                         test world, launch files
  hexapod_gait/          Python: robot_config (single source of truth), leg_ik, gait
                         (pure functions, later ported to the ESP32), gait_node (ROS 2)
tools/
  gen_urdf.py            regenerates description + controller config; --check verifies
                         the URDF joint chain against the IK, no ROS needed
  check_ik.py            IK round-trip and joint-travel report (servo centre table)
docs/
  wsl-ros2-setup.md      installing WSL2 + ROS 2 Jazzy + Gazebo Harmonic, running the sim
  solidworks-export.md   naming and axis conventions for the sw2urdf export
  hardware.md            wiring, power, camera/FPV link, servo calibration, torque
  bom.md                 electronics bill of materials with addresses and gotchas
  cad-measurements.md    the CAD leg measured, and why it exceeds the DT996 budget
sim/godot/               ARCHIVED Phase 0/1 Godot simulation. Its findings are kept below
                         and in hardware.md; it is not developed further.
```

## Running the simulation

Everything Gazebo-side runs inside WSL2 - see [docs/wsl-ros2-setup.md](docs/wsl-ros2-setup.md).
Once installed:

```
ros2 launch hexapod_description sim.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The gait node listens on `/cmd_vel` and drives the 18 joints through a
`JointGroupPositionController`. In Gazebo a position command becomes a joint
velocity target clamped to the URDF `velocity` limit and enforced up to its
`effort` limit - the servo's free speed and stall torque from `robot_config.py`.
The servo model comes with the physics engine.

## Roadmap

- [x] **Phase 0 - kinematic sim** (Godot, archived). Proved geometry, IK and
      gait timing; produced the servo-centre calibration table.
- [x] **Phase 1 - physics sim** (Godot, archived). Found the servo modelling
      lessons below and measured the torque demand of every manoeuvre.
- [ ] **Phase 2 - Gazebo.** WSL2 + ROS 2 Jazzy + Gazebo Harmonic. The
      placeholder description walks the same course under the same gait.
      Packages written; need WSL to build and verify.
- [ ] **Phase 3 - SolidWorks.** Model the robot, export with sw2urdf, drop the
      body half into `hexapod_description`. Real masses and inertias.
- [ ] **Phase 4 - ESP32 firmware and FPV.** Gait on the board, servos over
      two PCA9685s, MJPEG stream and a WebSocket taking the same `/cmd_vel`
      the sim's gait node takes - one brain, sim and real robot. The phone
      opens `192.168.4.1`; no app.
- [ ] **Phase 5 - the real robot.** Calibrate per `docs/hardware.md`, stand,
      walk, then close the loop on the IMU and the foot switches.

## What the Godot physics said (Phase 1, archived)

Measured in the sim with the defaults above (1.32 kg, MG996R-class servos):

| Situation | Peak torque | Outcome |
|-----------|-------------|---------|
| standing | 0.25 N.m | steady, 1 deg deflection |
| walking 0.1 m/s on the flat | 0.39 N.m | tripod pattern clean, level within 2 deg |
| turning in place at 1 rad/s | 0.78 N.m | body yaw rate 1.02 rad/s - the tightest case |
| 12 mm kerb | 0.46 N.m | crossed |
| 5 deg ramp, climbing | 0.83 N.m | climbed - the heaviest sustained load |
| hill top and 5 deg descent | 0.45 N.m | traversed, level within 2 deg |
| 7 cm drop-off (before the hill got its descent) | - | fell; stumble and fall detectors fired |

So an MG996R build has a 2.3x margin walking on the flat and almost none turning or climbing. If the
real robot weighs more than 1.3 kg, or you want it to turn briskly, plan on
20 kg.cm servos (DS3218 class, ~2 N.m).

Three lessons that carry over to any simulator: servo damping is not a free
parameter (it is T_stall / w_free, the speed-torque line); a geared servo's
output shaft carries reflected rotor inertia far above the link's own; and a
servo must be allowed to stall - Gazebo enforces the URDF `effort` limit
natively, Godot needed it emulated.
