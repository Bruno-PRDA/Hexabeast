# Electronics bill of materials

For the locked spec: 18-servo hexapod, 34 cm foot span, ~1.3 kg, DT996 servos
at 6 V, ESP32-S3 camera, FPV over its own WiFi. Prices are rough EUR for
France, ex-servos, and drift constantly - treat them as budgeting, not quotes.

## The whole robot hangs off one I2C bus

A camera's parallel interface consumes about 16 GPIO, which leaves nowhere
near enough pins for 18 servos plus sensors. It doesn't matter, because
everything here is I2C - two pins, five devices:

| Device | Address | Function | Notes |
|--------|---------|----------|-------|
| PCA9685 #0 | `0x40` | left legs FL/ML/RL, 9 channels | address jumpers untouched |
| PCA9685 #1 | `0x41` | right legs FR/MR/RR, 9 channels | **bridge A0** |
| MPU6050 | `0x68` | 6-axis IMU | `AD0` low |
| PCF8574 | `0x20` | 6 foot contact switches | inputs, switch to GND |
| INA219 | `0x44` | battery voltage + current | **must be moved off `0x40`** - see gotchas |
| SSD1306 | `0x3C` | 1.3" OLED - the robot's face | `0x3D` if its jumper is bridged |
| VL53L0X | `0x29` | time-of-flight proximity | fixed at boot; software-settable |

Run the bus at 400 kHz. `0x70` is the PCA9685 "all call" address and `0x00`
its reset address, so avoid both if you add anything later.

## 1. Actuation

| # | Part | Qty | ~EUR | Why |
|---|------|-----|------|-----|
| 1.1 | DT996 digital servo, 15 kg.cm, 180 deg | **20** | 90-120 | 18 joints + 2 spares. These strip gears, and a dead servo with two weeks of shipping ahead of it stops the build |
| 1.2 | Servo extension lead, 3-pin, 200-300 mm | 12 | 10 | The femur and tibia servos sit out on the legs; only the six coxa servos are close enough to reach the body on their stock leads |
| 1.3 | Servo horn / bracket set | 1 kit | 15 | Usually bundled with the servos - check before ordering |

## 2. Servo drive

| # | Part | Qty | ~EUR | Why |
|---|------|-----|------|-----|
| 2.1 | PCA9685 16-channel 12-bit PWM driver | 2 | 12-18 | 32 channels from two pins. Generates the 50 Hz pulses **in hardware**, so servo timing is immune to whatever the CPU is doing while encoding JPEG |

Nine servos per board. Splitting left and right legs across the two also
splits the current, which matters (see the power budget).

## 3. Compute and camera

| # | Part | Qty | ~EUR | Why |
|---|------|-----|------|-----|
| 3.1 | **Freenove ESP32-S3-WROOM CAM** (OV2640, 8 MB PSRAM, USB-C) | 1 | 15-20 | 8 MB PSRAM, hardware JPEG, native USB so no FTDI adapter, good broken-out GPIO. XIAO ESP32S3 Sense is a smaller alternative but exposes very few pins |
| 3.2 | u.FL / IPEX 2.4 GHz antenna + pigtail | 1 | 5 | Only if your board has the connector. Onboard PCB antenna gives 15-25 m; an external whip 50 m+ |
| 3.3 | microSD card, 16-32 GB, class 10 | 1 | 8 | Optional - onboard recording and log capture |

Assign I2C to any two free GPIO in firmware; the S3's GPIO matrix is flexible.
**Avoid GPIO 19 and 20** - they are USB D-/D+ and you want native USB flashing.

## 4. Power - the part that breaks robots

| # | Part | Qty | ~EUR | Why |
|---|------|-----|------|-----|
| 4.1 | LiPo 2S 7.4 V, **2200 mAh**, 25C+, XT60 | 1 | 15-22 | Sized by the robot, not the runtime: a 2200 pack is ~90 x 34 x 20 mm and slings under the plate between the rear coxa servos, which leave only ~8 mm either side. A 5000 pack is 132 x 43 and will not pass. Vendor spread is large - 1500 mAh packs alone range 69-107 mm long - so measure yours before cutting straps |
| 4.2 | UBEC / switching BEC, **6 V 10 A** | 2 | 16-24 | One per PCA9685. Do **not** feed 7.4 V to these servos - they are 6 V parts |
| 4.3 | Buck converter, 5 V 3 A (MP1584 or similar) | 1 | 5 | Separate logic rail for the ESP32 and camera |
| 4.4 | Electrolytic capacitor, 1000-2200 uF, 16 V | 2 | 3 | One across V+/GND at each PCA9685 |
| 4.5 | Electrolytic capacitor, 470 uF, 10 V | 1 | 1 | At the ESP32's 5 V input |
| 4.6 | XT60 connector pair | 2 | 4 | Battery and a spare |
| 4.7 | Inline blade fuse holder + 20 A fuses | 1 | 5 | A shorted servo lead on an unfused LiPo starts a fire |
| 4.8 | Rocker/toggle switch rated 20 A+, or XT60 anti-spark | 1 | 6 | Small switches weld shut at these currents |
| 4.9 | LiPo low-voltage alarm buzzer | 1 | 5 | 2S LiPo below ~3.3 V/cell is permanently damaged. The robot will happily walk until it kills the pack |
| 4.10 | Balance charger (IMAX B6 or similar) + LiPo safe bag | 1 | 30-40 | Skip only if you already charge LiPos |

### Budget

| Load | Draw |
|------|------|
| 18 servos, standing | ~1.5 A |
| 18 servos, walking | 3-5 A average |
| 18 servos, worst-case simultaneous move | 10-12 A peak |
| One DT996 stalled | ~2.5 A |
| ESP32-S3 + camera + WiFi | 0.4-0.8 A, spiky on TX |

Two 6 V/10 A UBECs cover the servo peak with margin and halve what each
PCA9685's traces carry. The logic rail stays separate so a servo spike cannot
brown out the camera.

```
LiPo 2S ---[20 A fuse]---[switch]---+--- 6 V 10 A UBEC --- PCA9685 #0 V+   (left legs)
                                    +--- 6 V 10 A UBEC --- PCA9685 #1 V+   (right legs)
                                    +--- 5 V 3 A buck ---- ESP32-S3 5 V    (logic + camera)

All grounds meet at ONE point. 1000 uF+ at each PCA9685, 470 uF at the ESP32.
```

## 5. Sensors

| # | Part | Qty | ~EUR | Why |
|---|------|-----|------|-----|
| 5.1 | MPU6050 6-axis IMU breakout | 1 | 3 | Body attitude. The sim models exactly this part, noise included. Upgrade: BNO055 (~25 EUR) fuses orientation on-chip and saves ESP32 cycles |
| 5.2 | PCF8574 I2C 8-bit GPIO expander | 1 | 3 | Six foot switches on a board with no spare pins. Inputs have weak pull-ups, so wire each switch to GND |
| 5.3 | Micro lever switch (SS-5GL / D2F class) | 8 | 5 | One per foot + spares. Binary contact, which is what the gait's stumble detector expects |
| 5.4 | INA219 current/voltage sensor | 1 | 5 | Battery telemetry on the FPV overlay. Knowing your remaining flight time matters more than it sounds |
| 5.5 | 1.3" OLED, SSD1306, I2C | 1 | 6 | The face. 128x64 is enough for eyes with real expression, and it draws about 20 mA |
| 5.6 | VL53L0X time-of-flight module | 1 | 5 | Proximity, 30-1200 mm. Unlike an HC-SR04 it is I2C, tiny, and unbothered by soft or angled surfaces |

## 6. Wiring and assembly

| # | Part | Qty | ~EUR | Why |
|---|------|-----|------|-----|
| 6.1 | Silicone wire 14 AWG, red/black, 2 m | 1 | 8 | Battery to fuse to switch to UBECs. 14 AWG for 20 A |
| 6.2 | Silicone wire 20-22 AWG, assorted, 10 m | 1 | 8 | Servo branches and distribution |
| 6.3 | Dupont jumper wires, F-F, 20 cm | 1 pack | 4 | I2C and sensor runs |
| 6.4 | JST-XH / screw terminal assortment | 1 | 6 | Serviceable joints - you will take this apart |
| 6.5 | Heat shrink assortment | 1 | 5 | |
| 6.6 | Small perfboard or distribution PCB | 1 | 5 | Ground star point and I2C fan-out |
| 6.7 | Cable sleeving / spiral wrap, 3 m | 1 | 5 | 18 servo leads through six moving legs. Do this or they snag |
| 6.8 | M2/M3 screw + standoff assortment | 1 | 10 | Board mounting |

## 7. Optional, and worth it

| # | Part | Qty | ~EUR | Why |
|---|------|-----|------|-----|
| 7.1 | SG90 micro servo + pan/tilt bracket | 2 | 8 | Camera pan/tilt on two spare PCA9685 channels. Looking around without turning the body avoids the most torque-expensive move the robot makes |
| 7.2 | Passive buzzer | 1 | 2 | Boot, WiFi-up, low-battery tones. Debugging a headless robot without sound is miserable |
| 7.3 | WS2812 LED strip, short | 1 | 5 | Status at a glance; doubles as headlights for the camera |
| 7.4 | USB-C cable, data-capable | 1 | 5 | Many bundled cables are charge-only |

## Gotchas that cost a build

**INA219 defaults to `0x40` - the same address as PCA9685 #0.** Bridge its
`A0` solder jumper to move it to `0x44` before wiring, or neither device works
and the fault looks like a dead bus.

**Bridge `A0` on the second PCA9685 only.** Untouched, both boards answer at
`0x40` and the right legs mirror the left ones.

**Never power servos through the ESP32's 5 V pin.** It cannot source the
current; the brownout resets the board mid-step.

**Watch the PCA9685's V+ traces.** They handle roughly 10 A total, which nine
DT996s can approach on a simultaneous move. Splitting the legs across two
boards with a UBEC each keeps you inside it. If you still see brownouts, feed
the servos from a separate distribution bus and take only the signal pin from
the PCA9685.

**Redundant I2C pull-ups.** Seven breakouts each carrying 10 k pull-ups put
about 1.4 k on the bus. That still works at 400 kHz, but if the bus is flaky,
desolder the pull-ups on all but one board.

**Servos are 6 V parts.** 2S LiPo is 8.4 V fully charged. Straight through,
that cooks them.

## Rough total

| | EUR |
|---|-----|
| Servos (20) | 90-120 |
| Everything else | 150-190 |
| Charger and LiPo safety, if new to LiPo | +35 |
| **Total** | **~240-345** |
