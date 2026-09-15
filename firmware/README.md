# Hexabeast firmware

ESP32-S3. Stage 1: servos, calibration and the gait loop — everything testable
on a bench with a board, two PCA9685s and as few as one servo. WiFi, the camera
and the FPV link come next.

## Layout

```
include/
  robot_geometry.h   GENERATED from robot_config.py - never edit
  leg_ik.h           inverse kinematics, pure, no Arduino
  gait.h             tripod gait, pure, no Arduino
  servo_bus.h        two PCA9685s over I2C
src/
  main.cpp           serial console, calibration, gait loop
  servo_bus.cpp
test/
  test_kinematics.cpp  C++ checked against the Python reference
  fixtures.h           GENERATED expected values
```

`leg_ik.h` and `gait.h` carry no Arduino dependency on purpose: that is what
lets them compile and be tested on a laptop, and it is why the same maths can
be trusted in the simulator and on the robot.

## The geometry is generated, not typed

`robot_config.py` owns the robot's dimensions. The firmware gets its copy from
the generator, so the two cannot drift:

```bash
python tools/gen_firmware_header.py           # write robot_geometry.h
python tools/gen_firmware_header.py --check   # fail if it is stale
```

Re-run it after any geometry change, the same as `gen_urdf.py`.

## Testing without hardware

```bash
python tools/test_firmware.py
```

This regenerates the fixtures **from the Python**, compiles the C++ against
them and runs it — so "the firmware agrees with the simulator" is a checked
fact. It uses zig's C++ frontend (`pip install ziglang`), which needs no admin
rights and no Visual Studio.

Current state: 2615 checks, 0 failures. Worst walking excursion 33.7 deg of the
80 available.

## Building and flashing

Needs [PlatformIO](https://platformio.org/install/cli) (`pip install platformio`).

```bash
cd firmware
pio run                 # build
pio run -t upload       # flash
pio device monitor      # serial console at 115200
```

## Bench calibration

Do this **before the legs are attached to anything**. Horns fitted at the wrong
angle will drive a leg into its own chassis on the first step.

1. **Power the PCA9685s from their UBECs**, not from the ESP32. Common ground.
   Boards must answer at `0x40` and `0x41` — if one is missing, the A0 jumper on
   board 1 is unbridged, or the INA219 is still squatting on `0x40`.

2. Flash, open the monitor. It boots **relaxed**: every output dark, servos
   limp. `x` is the panic key at any time.

3. `e` to enable, then `k` for calibration mode. All 18 servos go to their
   mechanical centre.

4. **Fit each horn now**, with the leg held at its design angle:

   | Joint | Joint angle at servo centre |
   |-------|-----------------------------|
   | coxa | 0 deg — leg straight out along its rest direction |
   | femur | +45 deg — femur raised |
   | tibia | -105 deg — tibia folded down |

   These come from `JOINT_CENTER_DEG` and change whenever the geometry does.
   Re-run `python tools/check_ik.py` if in doubt; it prints them.

5. **Trim.** `j<n>` selects a joint (`n`/`p` step through them). `+`/`-` move
   the trim 10 us, `>`/`<` by 2. A servo spline is about 1.4 deg per tooth, so
   trim covers what the horn cannot.

6. **Direction.** Press `f` and watch the joint. If it moves the wrong way for
   a positive angle, leave it flipped. The defaults in `SERVO_DIR` are the
   mirror assumption — left legs inverted — and are a guess until you check.

7. `s` saves trim and direction to NVS. They survive reflashing.

## Running the gait

`r` for run mode, then `w<vx>,<vy>,<wz>`:

```
w0.08,0,0     forward at 80 mm/s
w0,0,0.5      turn left
h             halt
x             relax
```

Commands expire after 500 ms. A console that goes quiet stops the robot — the
same guarantee the WiFi link will need, which is why it lives in the control
loop rather than in the transport.

On enable, the legs ease from centre to stance over 2 seconds rather than
snapping, and the gait phase only advances while a command is non-zero, so
stopping settles the feet at neutral instead of freezing mid-swing.

## Wiring

Pins in `main.cpp`. I2C defaults to GPIO 1 (SDA) and 2 (SCL) — any free pair
works on the S3, but avoid 19/20, which are USB D-/D+.

Full parts list and the I2C address map: [../docs/bom.md](../docs/bom.md).

## Not done yet

- WiFi SoftAP, WebSocket control, MJPEG stream, the phone page
- IMU (MPU6050) and foot switches (PCF8574)
- Battery telemetry (INA219)
- Body attitude control — using the IMU to keep the chassis level on a slope
