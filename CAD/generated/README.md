# Generated leg parts

Produced by `python tools/gen_leg_cad.py` from `robot_config.py`. Do not edit
by hand — regenerate.

| Part | Spans | Printed (30 % infill) |
|------|-------|-----------------------|
| `coxa_link` | coxa axis -> femur servo axis, 25 mm | ~5.3 g |
| `femur_link` | femur axis -> knee servo axis, 59 mm | ~6.5 g |
| `tibia` | knee axis -> foot tip, 109 mm | ~8.6 g |
| `body` | six coxa mounts, electronics, battery, mast socket | ~69 g |
| `abdomen` | dome over the rear, houses the UBECs | ~12 g |
| `head` | LCD face, proximity sensor and camera, tilted 15 deg down | ~21 g |
| `assembly.stl` | the lot at stance pose - for viewing, not printing | - |

Print six each of the three leg parts, one body, one dome, one head: **224 g**
of plastic. With 18 servos and the battery that is **1.53 kg** all-up.

## Open: the camera board does not fit the turret

`gen_robot_cad.py --check` warns about this on every run, deliberately. A
Freenove ESP32-S3-WROOM CAM is **57.1 x 28.1 mm**; the turret it is drawn on is
30 x 24. It was modelled 45 x 27 x 20, which is why nothing complained before.
Three ways out, none of them free:

- **Grow the turret** to about 62 x 34. Simplest, but it puts a panel bigger
  than the face on top of a 72 mm neck - the worst possible place for mass on
  a walking robot.
- **Board under the plate, camera on a flex extension.** The belly has room at
  x 20..70. The camera is a parallel DVP bus at 20 MHz, so a long flex is a
  real signal-integrity risk.
- **Swap to a XIAO ESP32S3 Sense** (21 x 17.5 mm, camera onboard), which fits
  the turret as drawn. A BOM change.

`.step` imports into SolidWorks as a solid body — editable geometry, but no
feature tree. `.stl` goes straight to the slicer.

Axis spacing is verified from the exported STLs, not just asserted by the
script: 25.02 / 58.97 / 109.01 mm against the 25 / 59 / 109 in the config.

**The servo tab holes are slots, and that is not a nicety.** MG996R clones put
the long pitch anywhere between 47.8 and 49.5 mm. The original 4.3 mm round
hole on a 49.5 nominal only reached down to 48.85, so a servo from the bottom
of that spread would simply not have bolted on. They are now 5.1 x 3.4 mm slots
on a 48.65 nominal, M3, which covers the whole range - a caliper check is a
sanity check rather than a gate.

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

**A hole is a subtraction, so the check could not see a single screw.** The
interference check compared solids, and holes carry no fastener - so the shunt
and the I2C expander both sat directly under fastener tails with about 1.5 mm
of air, and nothing flagged it. Screws, nuts and proud heads are now modelled
as solids from the same hole list the plate is drilled from. The sliver
threshold came down from 60 mm3 to 5 mm3 at the same time: 60 was not small
enough to be safe, and was hiding a 40 mm3 overlap where the OLED and the ToF
occupied the same space.

**Things on the head were placed in the body's frame, not the face's.** The
face tilts 15 degrees nose-down; 22 mm below its centre that swings it 5.7 mm
backwards. The boards were positioned at a fixed offset as though the head
stood upright, which put the proximity sensor 0.9 mm inside the slab it bolts
behind. Everything on the face now goes through `head_frame()`, the same chain
`make_head()` builds the face on.

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
