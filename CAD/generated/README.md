# Generated leg parts

Produced by `python tools/gen_leg_cad.py` from `robot_config.py`. Do not edit
by hand — regenerate.

| Part | Spans | Printed (30 % infill) |
|------|-------|-----------------------|
| `coxa_link` | coxa axis -> femur servo axis, 25 mm | ~5.3 g |
| `femur_link` | femur axis -> knee servo axis, 59 mm | ~7.0 g |
| `tibia` | knee axis -> foot tip, 109 mm | ~8.2 g |

`.step` imports into SolidWorks as a solid body — editable geometry, but no
feature tree. `.stl` goes straight to the slicer.

Axis spacing is verified from the exported STLs, not just asserted by the
script: 25.02 / 58.97 / 109.01 mm against the 25 / 59 / 109 in the config.

**Before printing six of these**, check the servo mounting pattern against a
real DT996 with calipers. The hole spacing here (49.5 x 10 mm, 4.3 mm holes)
is the published MG996R figure; your own servo model has no holes in it, so
there was nothing to measure.

Print flat on the bed, no supports needed. Three perimeters minimum — the
walls are 3 mm, which is what carries the bending load at the coxa joint.
