# Hexapod

An 18-servo hexapod on an ESP32, built simulation-first.

## The one rule

**The robot's brain never touches hardware directly.** Inverse kinematics,
gait generation and body control are pure functions of numbers in, numbers
out. Sensors and servos sit behind a thin interface with two implementations:
one that talks to real pins, one that talks to the simulator. Same control
code in both worlds - so a gait that walks in the sim walks on the robot, and
a bug found on the robot can be reproduced in the sim.

## Layout

```
sim/godot/          Godot 4.7 simulation (Jolt physics)
  scripts/
    robot_config.gd   geometry, masses, servo model, calibration - single source of truth
    leg_ik.gd         3-DoF leg inverse kinematics            (pure, ports to C++)
    gait.gd           tripod gait generator                    (pure, ports to C++)
    hexapod.gd        physics rig: rigid links, spring-driven servo joints, sensors
    hexapod_kinematic.gd  the Phase 0 rig, no physics - fast gait visualisation
    sim_imu.gd        simulated 6-axis IMU with noise
    world.gd          floor, kerb and hill course, camera, telemetry HUD
tools/
  check_ik.py         round-trip test of the IK + joint travel report
firmware/           ESP32 firmware (PlatformIO)               - phase 3
core/               shared C++ control code                   - phase 2
hal/                hardware abstraction: esp32/ and sim/     - phase 2
docs/
  hardware.md       parts, wiring, power, servo calibration
```

## Running the sim

Open `sim/godot` in Godot 4.7 and press F5, or:

```
Godot_v4.7.2-stable_win64.exe --path sim/godot
```

| Key | Action |
|-----|--------|
| W / S | forward / back |
| A / D | strafe |
| Q / E | turn |
| R | reset |
| 1 / 2 | halve / double servo stiffness |
| 3 / 4 | halve / double servo damping |
| 5 / 6 | halve / double servo torque rating |

The HUD shows the IMU, which feet are loaded, the worst servo tracking
error, peak servo torque against the rating, and flags stalls, stumbles,
falls and any pose outside a servo's travel. Set `PHYSICS := false` in
`world.gd` to run the kinematic rig instead.

## The servo model

Each of the 18 joints is a `Generic6DOFJoint3D` locked to one hinge with an
angular spring whose equilibrium is the commanded servo angle. Four numbers in
`robot_config.gd` describe the servo, and three of them are not free:

| Parameter | Value | Where it comes from |
|-----------|-------|---------------------|
| `SERVO_TORQUE` | 0.90 N.m | datasheet stall torque (MG996R at ~5 V) |
| `SERVO_SPEED` | 6 rad/s | datasheet no-load speed |
| `SERVO_DAMPING` | torque / speed = 0.15 | the speed-torque line *is* a damping coefficient. Guessing this high is the classic mistake: at 0.5 the swing legs spent the whole rating fighting their own damping and never left the ground. |
| `SERVO_STIFFNESS` | 12.5 N.m/rad | ~4 deg give at rated load (deadband + backlash + compliance). Stiffer, and the three planted legs fight each other into stall. |

Two more things make it behave:

- **Reflected rotor inertia** (`SERVO_REFLECTED_INERTIA`) on every link. A
  geared servo's output shaft carries rotor inertia times gear ratio squared,
  which dwarfs the plastic link. Without it the model buzzed at 0.9 rad/s RMS;
  with it, 0.009 - the sensor noise floor.
- **Stall emulation.** Godot 4.7 has no torque cap on joint springs, so when
  the spring deflection demands more than the rating, the equilibrium is let
  trail the measured angle at the rated-torque deflection. The spring then
  never pulls harder than the servo could. Before this, an infinitely strong
  servo catching a foot on the kerb launched the robot 10 cm into the air.

The spring deflection doubles as a torque gauge (tau = k * error), which is what
the HUD's torque line reads.

## What the physics said

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

Small-scale physics needs small-scale settings: Jolt's default penetration
slop and speculative contact distance are 2 cm, which is more than a foot's
diameter here. `project.godot` sets them to 3 mm and 5 mm, runs 240 ticks/s,
and halves Baumgarte stabilisation to stop contact pops.

Known limits: leg links do not collide with each other (they never need to in
a tripod gait, and adjacent links overlap at every joint); servo torque limiting
is emulated, not solved inside the constraint.

## Roadmap

- [x] **Phase 0 - kinematic sim.** Body moves as commanded, legs solved to
      follow. No physics. Proves the geometry, IK and gait timing.
- [x] **Phase 1 - physics sim.** Rigid body and links, spring-driven servo
      joints with a stall model, feet that push against a real floor. It can
      trip, slip and fall, and reports an IMU and six foot switches.
- [ ] **Phase 2 - shared core.** Port `leg_ik.gd` and `gait.gd` to C++ in
      `core/`. Build it natively and drive the Godot sim from it over a
      socket, so Godot is only physics and rendering. Needs a C++ compiler
      (MSVC Build Tools or MSYS2 mingw-w64).
- [ ] **Phase 3 - firmware.** PlatformIO project linking the same `core/`.
      Hardware-in-the-loop: the real ESP32 runs the gait, the sim supplies
      fake sensor data and animates its servo commands.
- [ ] **Phase 4 - the real robot.** Calibrate servos per `docs/hardware.md`,
      stand, walk. Then close the loop on the IMU (body levelling) and the foot
      switches (stumble detection).
- [ ] **Phase 5 - beyond.** Terrain, teleop over WiFi, telemetry plots.
