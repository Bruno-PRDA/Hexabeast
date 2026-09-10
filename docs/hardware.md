# Hardware

## Servo drivers: two PCA9685 boards

Shopping list with part numbers, quantities and prices: [bom.md](bom.md).

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

## Camera and FPV control

### Board: an ESP32-S3 camera module, not the old ESP32-CAM

A camera's parallel (DVP) interface eats about 16 GPIO. On the AI-Thinker
ESP32-CAM that leaves roughly four usable pins, no native USB (you need an
FTDI adapter to flash), 4 MB PSRAM, and a well-earned reputation for
browning out. An **ESP32-S3 camera board** (Freenove ESP32-S3-WROOM CAM,
XIAO ESP32S3 Sense, or similar) gives 8 MB PSRAM, hardware JPEG, native USB
flashing, better WiFi throughput and more free pins for the same money.

Either way the servos do **not** hang off the ESP32's GPIO - that is what the
two PCA9685 boards are for, and with a camera fitted they stop being a
convenience and become the only option:

- 18 legs (24 with the arm) from **two pins**, SDA and SCL.
- The PCA9685 generates the 50 Hz servo pulses **in hardware**. Servo timing
  is then immune to whatever the CPU is doing. Bit-banging PWM from a board
  that is simultaneously encoding JPEG and servicing WiFi produces visible
  twitching; this design cannot suffer from it.

### Topology: the robot is its own access point

Run the ESP32 in **SoftAP** mode - it publishes a WiFi network (say
`HEXABEAST`), the phone joins it, and the robot lives at `192.168.4.1`. No
router, so it works in a park as well as on the bench, and latency is one hop.
Station mode (joining your home WiFi) only makes sense if you want the phone
to keep internet access at the same time; range then belongs to the router.

Fit a board with a **u.FL external antenna** connector if range matters: the
onboard PCB antenna manages 15-25 m, an external whip 50 m or more.

### The link: video one way, intent the other

| Channel | Transport | Rate |
|---------|-----------|------|
| Video | MJPEG over HTTP, `multipart/x-mixed-replace` on `/stream` | QVGA 320x240 at ~25 fps, VGA 640x480 at ~12-15 fps |
| Control | WebSocket on `/ws`, JSON or packed binary | 20 Hz is ample |

Expect 150-300 ms of glass-to-glass latency. Fine for driving, not for
reflexes.

**The phone needs no app.** The ESP32 serves one HTML page: an `<img>` pointed
at `/stream` with two touch joysticks overlaid, which post to the WebSocket.
Open a browser at `192.168.4.1` and that is the FPV rig, on iOS and Android
alike.

### What crosses the link is intent, never joint angles

The control message carries what the *body* should do - forward, sideways,
turn - and the gait runs **on the ESP32**:

```json
{"vx": 0.08, "vy": 0.0, "wz": 0.4, "gait": "tripod"}
```

That is deliberately the same shape as the `/cmd_vel` Twist the simulation's
gait node already consumes, so the phone drives the sim and the real robot
identically. It also means a WiFi dropout makes the robot *stop*, not
collapse mid-step: the last command expires, velocity goes to zero, the legs
settle. Streaming 18 joint angles over WiFi at 50 Hz would put a lossy radio
link inside the control loop - never do that.

### Power, again - the camera makes it worse

The camera and WiFi transmitter together want a clean 500 mA-1 A, and they
are far more brownout-sensitive than the ESP32 alone. Servo current spikes
that a bare ESP32 shrugged off will drop the video feed or reset the board.

Give logic and servos **separate regulators** off the same battery:

```
2S LiPo 7.4 V --+-- 6 V / 10 A UBEC ---- PCA9685 V+  (servos)
                |
                +-- 5 V / 2 A BEC ------ ESP32-S3 5 V (logic + camera)

grounds common at one point; 1000 uF on the servo rail, 470 uF at the ESP32
```

### Mounting

The body pitches and rolls with every gait cycle, so the steadiest place for
the camera is the **body front**, as low and as close to the centre of
rotation as the frame allows. Two spare PCA9685 channels make a pan/tilt head
worth having - it lets you look around without turning the whole robot, which
costs the most torque of any manoeuvre (see below).

## Servo torque and how big the robot can be

The Phase-1 physics sim measured the peak torque any one servo must supply,
for the current geometry (foot span ~34 cm) at 1.3-1.6 kg all-up:

| standing | walking 0.1 m/s | 12 mm kerb | turning 1 rad/s | climbing 5 deg |
|----------|-----------------|------------|-----------------|----------------|
| 0.25 N.m | 0.39 N.m | 0.46 N.m | 0.78 N.m | 0.83 N.m |

Turning in place and climbing are the stress cases; flat walking is trivial.

### What a DT996 can carry

The DT996 is an MG996R-form-factor digital servo advertised at 15 kg.cm
(1.49 N.m at 6 V). Designing to 75 % of that - 1.10 N.m, the value in
`robot_config.py` - covers stall spikes, gear wear and the gap between a
clone's label and its real output.

`tools/scale_torque.py` extrapolates the measured 0.83 N.m datapoint across
robot sizes. The physics that matters: **the 18 servos are a fixed 1 kg no
matter how big the frame is**, so mass barely grows with size while the leg's
lever arm grows linearly. Torque tracks mass x reach.

| foot span | body | all-up | climb torque | verdict on DT996 |
|-----------|------|--------|--------------|------------------|
| 24 cm | 13 cm | 1.4 kg | 0.52 N.m | comfortable even at 4.8 V |
| 29 cm | 15 cm | 1.4 kg | 0.66 N.m | comfortable even at 4.8 V |
| **34 cm** | **18 cm** | **1.5 kg** | **0.83 N.m** | **comfortable - the current design** |
| 39 cm | 21 cm | 1.7 kg | 1.04 N.m | OK at 6 V, tight on obstacles |
| 44 cm | 23 cm | 1.8 kg | 1.29 N.m | walks flat, stalls climbing |
| 51 cm+ | 27 cm+ | 2.1 kg+ | 1.73 N.m+ | too big for DT996 |

**Ceiling: ~41 cm foot span** with a 25 % margin, ~37 cm with the arm fitted.
Past that you need 20-25 kg.cm servos (DS3218 class, ~2 N.m) and a supply to
match. Note how fast it runs out - going from 34 to 51 cm doubles the torque
demand for only 40 % more size, because reach and mass grow together.

### The arm costs 4 cm of that budget (currently off)

`ARM = False` in `robot_config.py`, so the build is a plain hexapod: 18
servos, 1.32 kg, 73 mm of static margin. Recorded here for when it comes back.

Six more servos (0.33 kg) hung off the front cut the comfortable span from 41
to 37 cm, shift the centre of mass 23 mm forward (margin 73 -> 52 mm, still
stable), and the shoulder servo limits the gripper to roughly **275 g at 24 cm
full reach** - fold the arm in to lift more. Flipping `ARM` back on regenerates
the URDF, the controller list and the launch file's spawner together.

Re-run `python tools/scale_torque.py --arm` after changing any mass or length.

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
