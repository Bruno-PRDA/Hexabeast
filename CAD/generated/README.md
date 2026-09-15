# Generated leg parts

Produced by `python tools/gen_leg_cad.py` from `robot_config.py`. Do not edit
by hand — regenerate.

| Part | Spans | Printed (30 % infill) |
|------|-------|-----------------------|
| `coxa_link` | coxa axis -> femur servo axis, 25 mm | ~5.3 g |
| `femur_link` | femur axis -> knee servo axis, 59 mm | ~6.5 g |
| `tibia` | knee axis -> foot tip, 109 mm | ~8.6 g |
| `body` | six coxa mounts, electronics, battery, mast socket | ~70 g |
| `abdomen` | dome over the rear, houses the UBECs | ~12 g |
| `head` | LCD face, proximity sensor and camera, tilted 15 deg down | ~19 g |
| `assembly.stl` | the lot at stance pose - for viewing, not printing | - |

Print six each of the three leg parts, one body, one dome, one head: **223 g**
of plastic. With 18 servos and the battery that is **1.53 kg** all-up.

`.step` imports into SolidWorks as a solid body — editable geometry, but no
feature tree. `.stl` goes straight to the slicer.

Axis spacing is verified from the exported STLs, not just asserted by the
script: 25.02 / 58.97 / 109.01 mm against the 25 / 59 / 109 in the config.

**Before printing six of these**, check the servo mounting pattern against a
real DT996 with calipers. The hole spacing here (49.5 x 10 mm, 4.3 mm holes)
is the published MG996R figure; your own servo model has no holes in it, so
there was nothing to measure.

## Two things the assembly check found

**The head height is set by the knees, not the body.** A front leg's
knee rises to 80 mm above the plate at the top of its swing. The first draft
put the camera at 62 mm, which would have had legs sweeping through the shot
every stride. The camera sits at 115 mm, clearing the worst case by 35 mm; the LCD face is
at 86 mm and the proximity sensor at 60 mm.

**Styling is packaging, not decoration.** The plate is a faceted profile down
the centreline, drawn out to a point at each hip and lightened with triangular
voids; the tibia and femur are Warren trusses between two edge rails. Every
station of the profile is set by what sits under it - the tail by where the
dome's wall ring lands, the waist by the servo driver mounted across it, the
thorax by the hip spacing - so the outline follows the packaging rather than
being drawn over it.

The voids are not placed by eye. A triangular lattice proposes candidates and
each has to earn its place: miss every hole, cutout and bearing face by 4 mm,
and lie wholly inside the outline eroded by 6.5 mm. The lattice's own phase is
swept, which matters more than it sounds like it should - across the 16 offsets
tried, the number that fits runs from 0 to 10 per half-plate. 20 voids survive,
taking 22 cm2 out of the plate, and adding a component later just makes some of
them vanish.

**A hole that doesn't cut looks exactly like a hole that does.** build123d
extrudes a face along its own normal, so a clockwise polygon builds its prism
*below* the plane - subtract it and nothing is removed. Half the truss bays
alternate winding by construction and every mirrored void flips it again, so
most of them were quietly doing nothing and the parts still rendered perfectly.
Winding is now normalised in one place, and both generators probe for material
where each void should be.

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
walls are 3 mm, and with the middle of each link trussed out it is the rails
that carry the bending load.
