# Hardware

## Servo drivers: two PCA9685 boards

18 servos, 16 channels per board. Two boards on the same I2C bus:

| Board | I2C address | Solder jumper | Drives |
|-------|-------------|---------------|--------|
| 0 | `0x40` | none | left legs FL, ML, RL (channels 0-8) |
| 1 | `0x41` | bridge **A0** | right legs FR, MR, RR (channels 0-8) |

Bridge A0 on the second board only, or both answer at `0x40` and neither
works. The channel map per joint is in `sim/godot/scripts/robot_config.gd`;
channels 16-31 in that file are board 1's 0-15.

ESP32 default I2C pins: **SDA = GPIO 21, SCL = GPIO 22.** Both boards' SDA,
SCL, GND and VCC (3.3 V logic) daisy-chain.

## Power - the part that breaks robots

**Never power servos from the ESP32's 5 V pin.** It cannot source the current
and the resulting brownout resets the ESP32 mid-step.

Each board has a separate V+ terminal block for servo power. Budget:

| Servo | Stall current | Realistic walking draw (18 servos) |
|-------|---------------|------------------------------------|
| SG90 / MG90S | ~0.7 A | 3-5 A peaks |
| MG996R | ~2.5 A | 8-12 A peaks |
| DS3218 / 20 kg class | ~3 A | 10-15 A peaks |

Feed V+ from a supply rated for the peak, e.g. a 2S LiPo (7.4 V) through a
**5 V / 10 A+ UBEC** for 5 V servos, or direct 6-7.4 V for servos rated for it.
Put a 1000 uF+ electrolytic across V+/GND on each board to absorb the
current spikes when all six legs move at once. Common the ESP32 ground with
the servo ground.

## Servo torque - what the simulation found

With the geometry in `robot_config.gd` and a 1.32 kg all-up mass, the physics
sim measured the peak torque any servo has to supply:

| standing | walking 0.1 m/s | 12 mm kerb | turning 1 rad/s | climbing 5 deg |
|----------|-----------------|------------|-----------------|----------------|
| 0.25 N.m | 0.39 N.m | 0.46 N.m | 0.78 N.m | 0.83 N.m |

An MG996R (0.9 N.m at ~5 V, 1.1 at 6 V) walks with a 2.3x margin and turns
or climbs with almost none. Two consequences:

- **Run the servos at 6 V**, not 5. The extra 20% torque is free.
- **Weigh the real robot.** The margin scales with mass. Over ~1.5 kg, or if
  you want brisk turning, use 20 kg.cm servos (DS3218 class, ~2 N.m) - and a
  supply that can feed them (see Power).

Change `SERVO_TORQUE`, `SERVO_SPEED` and the masses to your parts and re-run;
the HUD's torque line and stall warnings do the rest.

## Sensors

| Sensor | Bus / pins | Notes |
|--------|-----------|-------|
| IMU (MPU6050 `0x68` / BNO055 `0x28`) | shared I2C | BNO055 does sensor fusion on-chip and hands back a clean orientation; MPU6050 needs a complementary filter in firmware. Mount rigidly at body centre. |
| Foot contact switches x6 | one GPIO each, `INPUT_PULLUP` | Micro-switch or a spring-loaded foot tip. Wire to GND so a press reads LOW. Debounce in software. |

Safe GPIOs for the switches: 13, 14, 25, 26, 27, 32, 33. Avoid 0, 2, 12, 15
(boot-mode strapping pins) and 34-39 (input-only, no pull-ups).

## Servo calibration - do this before the first stand

A joint angle is not a servo angle. The IK found that the tibia lives around
-94 deg for the whole gait, so a horn fitted with the servo centred and the leg
straight would spend its life jammed at an end stop. Fit each horn with the
leg held in the **centre pose**:

| Joint | Hold the leg at | Servo centre = |
|-------|----------------|----------------|
| Coxa | pointing straight along its rest direction | 0 deg |
| Femur | raised 22 deg above horizontal | +22 deg |
| Tibia | folded 94 deg below the femur line | -94 deg |

Procedure per servo:

1. Command 1500 us (centre) with the horn **off**.
2. Hold the leg segment at the angle in the table.
3. Press the horn on the nearest spline tooth, screw it down.
4. Record the residual error - the spline is coarse (~15 deg on a 25 T) so you
   will not land exactly. That residual goes in a per-servo trim table in
   firmware.

The centre values come from `tools/check_ik.py`; re-run it after changing
any link length in `robot_config.gd` and update this table.
