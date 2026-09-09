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
sim/godot/          Godot 4.7 simulation
  scripts/
    robot_config.gd   geometry, leg layout, servo calibration - single source of truth
    leg_ik.gd         3-DoF leg inverse kinematics            (pure, ports to C++)
    gait.gd           tripod gait generator                    (pure, ports to C++)
    hexapod.gd        builds the rig from the config and drives it
    world.gd          ground, lighting, chase camera, HUD
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

The HUD flags any pose that would exceed a servo's usable travel.

## Roadmap

- [x] **Phase 0 - kinematic sim.** Body moves as commanded, legs solved to
      follow. No physics. Proves the geometry, IK and gait timing.
- [ ] **Phase 1 - physics sim.** Body becomes a RigidBody3D, legs get
      collision shapes, feet push against a real floor. The robot can now
      trip, slip and fall. Add simulated IMU and foot contact sensors.
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
