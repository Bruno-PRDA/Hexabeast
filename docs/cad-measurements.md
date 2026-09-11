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

## The problem: this leg is proportioned for stronger servos

> **Correction (2026-09-11).** An earlier version of this file said 4.4x over
> budget. That was wrong. `scale_torque.py`'s `reach` is the **stance lever
> arm** - the horizontal coxa-axis-to-foot distance while standing - and I fed
> it the fully-extended link chain instead, overstating torque by about 1.9x.
> The corrected figures are below.

Torque is `C x mass x g x stance_reach`, calibrated against the 0.83 N.m the
Phase-1 sim measured while climbing. Two things follow.

**The limit is stance reach, not link length.** At ~1.34 kg all-up, the DT996
ceiling is **147 mm of stance reach**, whatever the links measure - a longer
leg can always stand more tucked, trading ground clearance for torque.

**The leg as drawn** stands at 242 mm reach with a sim-like tuck, needing
2.61 N.m against 1.12 N.m usable: **2.3x over**, not 4.4x.

## Uniform scaling is the wrong operation

A DT996 is 40.7 x 19.7 x 42.9 mm at every scale. The leg is a mix of *length*
features, which should scale, and *servo interface* features, which must not.
Shrinking both together fails from below long before torque becomes the issue:

| Scale | Links (mm) | Stance | Torque | articulation 1 | Verdict |
|-------|-----------|--------|--------|----------------|---------|
| 33 % | 20 / 46 / 86 | 80 mm | 0.59 N.m (1.9x margin) | **27 mm** | unbuildable - must bolt a 41 mm servo |
| 40 % | 24 / 56 / 104 | 97 mm | 0.73 N.m | **33 mm** | unbuildable |
| 50 % | 30 / 70 / 130 | 121 mm | 0.94 N.m | 41 mm | exactly servo-length, no wall |
| 55 % | 33 / 77 / 142 | 132 mm | ~1.09 N.m | 45 mm | workable |
| 60 % | 36 / 84 / 155 | 145 mm | 1.17 N.m | 49 mm | over torque |

At 33 % the *torque is comfortable* - 0.59 N.m with 1.9x margin. What fails is
geometry: articulation 1 carries the femur servo and currently spans 81.7 mm
around it. At 27 mm it is smaller than the servo it must hold.

The usable window for uniform scaling is roughly 55-58 %, which is thin. The
better move is to scale the link *lengths* to the torque budget and redraw the
brackets at servo size.

## Decided: 42 % of the CAD

Scaled on 2026-09-11, keeping the DT996 servos. `robot_config.py` now holds:

| | Value | Was (placeholder) |
|---|-------|-------------------|
| Links | 25 / 59 / 109 mm | 30 / 60 / 90 |
| Proportions | 1 : 2.36 : 4.36 | 1 : 2 : 3 |
| Link chain | 193 mm | 180 mm |
| Stance reach | 102 mm | 95 mm |
| Stand height | 81 mm | 75 mm |
| Foot span | 344 mm | 330 mm |
| All-up mass | 1.61 kg | 1.32 kg |
| Servo centres | 0 / 45 / -105 deg | 0 / 22 / -94 |

The proportions are the CAD's to within a percent, so this is the same leg at
42 %, not a different design. Torque lands at 0.54 N.m standing and 0.94 N.m
climbing against 1.12 N.m usable - a **1.19x margin**.

Why 42 % and not the 33 % first asked for: at 33 % the torque was never the
problem (0.59 N.m, 1.9x margin) - `articulation 1` fell to 27 mm while still
needing to bolt on a 40.7 mm servo. 42 % puts it at 34 mm, which still means
**redrawing the brackets around the servo** rather than scaling them. The link
*lengths* scale; the *servo interfaces* cannot.

Worst-case joint excursions from centre are coxa 44, femur 62, tibia 68 deg,
all inside the 80 deg of usable travel - so the horns can be fitted at the
centres above and every walking pose is reachable.

### What still has to change in CAD

- `articulation 1` and `articulation 2`: redraw at 42 % **length**, servo
  mounting features at full size.
- `leg1` (tibia): 259 -> 109 mm. This one is pure length, so it scales cleanly.
- The body does not exist yet. `LEGS` still assumes hip axes 70 mm off centre;
  that is the number to design the chassis around, or change and re-run.

Nothing about the servo calibration survives a geometry change - re-run
`python tools/check_ik.py` and copy the new midpoints into `JOINT_CENTER_DEG`
before fitting a single horn.
