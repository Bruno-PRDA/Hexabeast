# Generated leg parts

Produced by `python tools/gen_leg_cad.py` from `robot_config.py`. Do not edit
by hand — regenerate.

| Part | Spans | Printed (30 % infill) |
|------|-------|-----------------------|
| `coxa_link` | coxa axis -> femur servo axis, 25 mm | ~5.3 g |
| `femur_link` | femur axis -> knee servo axis, 59 mm | ~7.0 g |
| `tibia` | knee axis -> foot tip, 109 mm | ~8.2 g |
| `body` | six coxa mounts, electronics, battery, mast socket | ~67 g |
| `camera_mast` | ESP32-S3 camera at 110 mm, tilted 15 deg down | ~15 g |
| `assembly.stl` | the lot at stance pose - for viewing, not printing | - |

Print six each of the three leg parts, one body, one mast: **205 g** of
plastic. With 18 servos and the battery that is **1.51 kg** all-up.

`.step` imports into SolidWorks as a solid body — editable geometry, but no
feature tree. `.stl` goes straight to the slicer.

Axis spacing is verified from the exported STLs, not just asserted by the
script: 25.02 / 58.97 / 109.01 mm against the 25 / 59 / 109 in the config.

**Before printing six of these**, check the servo mounting pattern against a
real DT996 with calipers. The hole spacing here (49.5 x 10 mm, 4.3 mm holes)
is the published MG996R figure; your own servo model has no holes in it, so
there was nothing to measure.

## Two things the assembly check found

**The camera mast is 105 mm because of the knees, not the body.** A front leg's
knee rises to 80 mm above the plate at the top of its swing. The first draft
put the camera at 62 mm, which would have had legs sweeping through the shot
every stride. 105 mm clears the worst case by 30 mm.

**Each foot lands 11.9 mm off the ideal.** The femur servo bolts to a wall, so
its shaft cannot sit on the leg's centre plane - the offset is the servo's half
width plus clearance plus half the wall. Seen from the coxa axis it is a
rotation, not a reach error: the stance hexagon is turned 6.65 deg, and the true
radius is 102.69 mm against a nominal 102. Trim each coxa by -6.65 deg and the
feet return to the model's radial line, leaving 0.7 mm of reach error - under
1 %, so torque and gait are untouched.

All six legs are offset the same rotational sense, which keeps the stance
rotationally symmetric and balanced. The payoff is that all six legs are the
**same printed part**, not mirrored pairs.

Print flat on the bed, no supports needed. Three perimeters minimum — the
walls are 3 mm, which is what carries the bending load at the coxa joint.
