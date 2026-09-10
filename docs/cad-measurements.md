# CAD leg, measured

First leg modelled in SolidWorks on 2026-09-10 and measured from the STL/STEP
exports in `CAD/export/`. Everything here is measured, not estimated.

## Link lengths

Joint axes were found by fitting cylinders to the exported meshes, which share
one assembly frame. The axes are the **bores in the driven parts** - each sits
about 7 mm from the servo case's own rounded end (r = 10.37 mm), which is the
feature a naive fit latches onto instead.

The leg is drawn in a vertical plane with **Y up**: the coxa axis is vertical,
the femur and knee axes horizontal along Z.

| Link | Axis to axis | Placeholder it replaces | Ratio |
|------|--------------|-------------------------|-------|
| COXA | 59.8 mm | 30 mm | 2.0x |
| FEMUR | 139.9 mm | 60 mm | 2.3x |
| TIBIA | 259.0 mm | 90 mm | 2.9x |
| **Total reach** | **458.7 mm** | 180 mm | **2.5x** |

Joint axes in assembly coordinates:

| Joint | Direction | Passes through |
|-------|-----------|----------------|
| coxa | Y (vertical) | x = 40.2, z = 63.5 |
| femur | Z (horizontal) | x = 100.0, y = 104.6 |
| knee | Z (horizontal) | x = 168.5, y = 226.6 |
| foot tip | - | (295.4, 0.9, 62.8) |

## Masses

STL volume is exact; mass depends on what you print it at. A 3D-printed part
is never solid - perimeters and skins are, the middle is not - so effective
density lands near `0.3 + 0.7 x infill`.

| Part | Volume | Solid PLA | at 30 % infill |
|------|--------|-----------|----------------|
| leg1 (tibia) | 133.65 cm3 | 165.7 g | 76 g |
| base | 21.38 cm3 | 26.5 g | 12 g |
| articulation 1 | 16.26 cm3 | 20.2 g | 9 g |
| articulation 2 | 8.34 cm3 | 10.3 g | 5 g |
| **printed per leg** | **179.6 cm3** | 222.7 g | **102 g** |
| 3x DT996 servo | - | **165 g** | 165 g |
| **total per leg** | | | **267 g** |

The servo STL reports 41.6 g at PLA density. Ignore it - a DT996 is motor,
metal gears and a PCB, and weighs **55 g**. Only printed parts may take the
plastic density.

## The problem: this leg is too big for DT996 servos

Torque from `tools/scale_torque.py`, calibrated against the 0.83 N.m the
Phase-1 physics sim measured while climbing.

| Leg size | Reach | All-up | Climb torque | vs 1.12 N.m usable |
|----------|-------|--------|--------------|--------------------|
| **100 % (as drawn)** | 459 mm | 1.90 kg | **4.96 N.m** | **4.4x over** |
| 60 % | 275 mm | 1.42 kg | 2.22 N.m | 2.0x over |
| 50 % | 229 mm | 1.37 kg | 1.78 N.m | 1.6x over |
| 40 % | 183 mm | 1.33 kg | 1.38 N.m | 1.2x over |
| 33 % | 150 mm | 1.31 kg | 1.12 N.m | at the limit |

**The DT996 ceiling is 150 mm reach - 33 % of the leg as drawn.**

The cause is leverage, not weight. All-up mass barely moves between these rows
(1.31 to 1.90 kg) because 18 servos are a fixed 990 g whatever the frame does.
What changes is the lever arm: torque at the coxa is roughly body weight times
horizontal reach, and reach here went up 4.8x against the simulated design.

Note the *reach* multiplier is 4.8x while the *link chain* is only 2.5x longer.
Reach is the horizontal distance from the coxa axis to the foot in the stance
pose, and the simulated robot stands with its legs tucked (95 mm of a possible
180 mm). A leg drawn stretched out is far more demanding than its link lengths
alone suggest.

## Ways forward

| Option | Leg | Servos | Servo cost (18) | Result |
|--------|-----|--------|-----------------|--------|
| Scale down | 33 % | DT996 as planned | ~110 EUR | 150 mm reach, ~41 cm span - matches the sim exactly |
| Middle | 45 % | DS3225 (25 kg.cm) | ~270 EUR | 206 mm reach, ~55 cm span, 1.58 vs 1.84 N.m usable |
| Full size | 100 % | 67 kg.cm class | ~990 EUR | 459 mm reach, ~1 m span |

The middle option is the interesting one: 45 % of the drawn leg on DS3225s
keeps a robot half a metre across for about 160 EUR more than the DT996 plan.
Going full size means serial-bus servos and roughly nine times the servo
budget - a different class of machine.

Whatever is chosen, `robot_config.py` still holds the 30/60/90 placeholder.
Nothing downstream - URDF, gait, torque budget - reflects the CAD yet.
