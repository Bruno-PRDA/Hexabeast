#!/usr/bin/env python3
"""Generate the leg's printed parts from robot_config.py.

    python tools/gen_leg_cad.py                 # STEP + STL into CAD/generated/
    python tools/gen_leg_cad.py --check         # build and report, no export

Why generate rather than model by hand: the axis-to-axis distances ARE the
kinematics. The first hand-drawn leg missed them by 2.5x, and the mistake only
surfaced after measuring the export. Here the spacing is a parameter, so the
parts cannot disagree with the simulator, and a geometry change regenerates
them.

What this is NOT: a SolidWorks feature tree. STEP imports as a solid body, so
treat these as a dimensionally-correct starting point rather than something to
edit parametrically in SolidWorks.

THE RULE THAT SHAPES EVERYTHING HERE: a hobby servo bolts to a flat plate
PERPENDICULAR to its output shaft, through four tabs, with the body passing
through a rectangular cutout. It cannot be straddled by a U-bracket, because
the shaft runs parallel to the cheeks rather than through them. So every joint
in this leg is a plate with a servo cutout, and the only question is which way
that plate faces.

  coxa_link   driven about a VERTICAL axis by the coxa servo's horn. Carries the
              femur servo on a vertical wall, so that shaft is horizontal.
  femur_link  driven about a horizontal axis. The knee servo's axis is parallel
              to its own, so it mounts on a flat plate - the simple case.
  tibia       driven by the knee horn, reaching to the foot. No servo.
"""
import argparse
import math
import os
import sys

from build123d import (Align, Box, Cylinder, Pos, Rot, Vector, export_step,
                       export_stl)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "hexapod_gait"))
from hexapod_gait import robot_config as cfg  # noqa: E402

OUT = os.path.join(ROOT, "CAD", "generated")

# --- Servo envelope ----------------------------------------------------------
# The user's own model (CAD/export/Assemblage1 - SERVO-1.STL) is a simplified
# solid, 54.5 x 46.5 x 20.0 mm with no mounting holes, so the hole pattern below
# is the published MG996R one. VERIFY IT WITH CALIPERS before printing six legs.
SERVO_BODY_L = 40.7      # body alone, between the tabs
SERVO_W = 20.0           # body width (thin axis)
SERVO_H = 46.5           # along the shaft, from the user's model
SERVO_SHAFT_OFF = 10.0   # shaft axis from body centre, along the length
SERVO_HOLE_D = 4.3       # M4 clearance
SERVO_HOLE_PITCH_L = 49.5
SERVO_HOLE_PITCH_W = 10.0

# --- Horn interface ----------------------------------------------------------
HORN_BOSS_D = 24.0
HORN_BORE_D = 9.0        # clearance over the splined boss
HORN_SCREW_D = 2.2       # self-tapping M2 into the horn
HORN_SCREW_R = 7.0
HORN_SCREW_N = 4

# --- Print settings ----------------------------------------------------------
WALL = 3.0
PLATE = 5.0
CLEAR = 0.4
FOOT_D = 16.0
FEMUR_AXIS_Z = 22.0      # femur axis above the coxa horn face
FOOT_DROP = 6.0


def servo_cut(through=120.0):
    """Everything a servo removes from its mounting plate.

    Origin on the SHAFT AXIS, shaft along +Z, body length along +X. Putting the
    origin on the shaft is what makes link lengths trivially right: place this
    at distance d and the servo's axis is at exactly d.
    """
    cx = SERVO_SHAFT_OFF
    cut = Pos(cx, 0, 0) * Box(SERVO_BODY_L + 2 * CLEAR, SERVO_W + 2 * CLEAR, through)
    for sx in (-1, 1):
        for sy in (-1, 1):
            cut += Pos(cx + sx * SERVO_HOLE_PITCH_L / 2,
                       sy * SERVO_HOLE_PITCH_W / 2, 0) * \
                Cylinder(SERVO_HOLE_D / 2, through)
    return cut


def horn_boss(height):
    """SOLID boss bolting to a servo horn. Origin on the joint axis, axis +Z.

    Additive only. The bore is cut separately, at the end, because this boss
    usually lands inside a plate - subtract the bore here and the plate fills
    it straight back in. Add everything, then cut.
    """
    return Cylinder(HORN_BOSS_D / 2, height,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))


def horn_cut(through=120.0):
    """Bore and screw holes for a horn. Subtract LAST, from the whole part."""
    cut = Cylinder(HORN_BORE_D / 2, through,
                   align=(Align.CENTER, Align.CENTER, Align.CENTER))
    for i in range(HORN_SCREW_N):
        a = 2 * math.pi * i / HORN_SCREW_N
        cut += Pos(HORN_SCREW_R * math.cos(a), HORN_SCREW_R * math.sin(a), 0) * \
            Cylinder(HORN_SCREW_D / 2, through,
                     align=(Align.CENTER, Align.CENTER, Align.CENTER))
    return cut


def make_coxa_link(d=None):
    """Coxa axis vertical at the origin; femur servo's shaft horizontal at x=d.

    The awkward part of the leg: the driving axis is vertical and the driven
    one horizontal, so the horn plate and the servo plate are perpendicular.
    """
    d = d if d is not None else cfg.COXA * 1000
    plate_len = d + HORN_BOSS_D / 2 + 6
    wall_y = SERVO_W / 2 + CLEAR + WALL / 2     # wall centreline

    # Horn plate, lying flat, driven from below about +Z.
    part = Pos(plate_len / 2 - HORN_BOSS_D / 2, 0, 0) * Box(
        plate_len, 2 * wall_y + WALL, PLATE,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    part += horn_boss(PLATE)

    # Vertical wall carrying the femur servo, normal along Y so the shaft ends
    # up horizontal. Tall enough to take the 4-hole pattern either side.
    wall_h = FEMUR_AXIS_Z + SERVO_HOLE_PITCH_W / 2 + 8
    wall_x0 = d - SERVO_HOLE_PITCH_L / 2 - 8 + SERVO_SHAFT_OFF
    wall_x1 = d + SERVO_HOLE_PITCH_L / 2 + 8 + SERVO_SHAFT_OFF
    part += Pos((wall_x0 + wall_x1) / 2, -wall_y, 0) * Box(
        wall_x1 - wall_x0, WALL, wall_h,
        align=(Align.CENTER, Align.CENTER, Align.MIN))

    # A gusset, because a bare L-bracket hinges at the corner and this joint
    # carries the whole leg's bending moment.
    part += Pos(d * 0.5, -wall_y, PLATE) * Box(
        d + 10, WALL, 10, align=(Align.CENTER, Align.CENTER, Align.MIN))

    # Rot(-90,0,0) sends +Z to +Y: the servo's shaft becomes horizontal.
    cut = Pos(d, -wall_y, FEMUR_AXIS_Z) * Rot(-90, 0, 0) * servo_cut()
    return part - cut - horn_cut()


def make_femur_link(d=None):
    """Driven about +Z at the origin; the knee servo's axis is parallel, at x=d.

    Both axes parallel means one flat plate serves as horn interface and servo
    mount - the easy joint.
    """
    d = d if d is not None else cfg.FEMUR * 1000
    width = max(SERVO_W + 2 * WALL, HORN_BOSS_D)
    length = d + SERVO_SHAFT_OFF + SERVO_HOLE_PITCH_L / 2 + 8 + HORN_BOSS_D / 2

    # Waisted rather than a plain slab: full width at both ends where the horn
    # and the servo need material, pinched in the middle where the bending
    # moment is lowest. Costs nothing, and it is most of what makes a leg read
    # as a leg rather than a bracket.
    part = None
    n = 10
    for k in range(n):
        x0, x1 = length * k / n, length * (k + 1) / n
        u = (x0 + x1) / 2 / length
        w = width * (1.0 - 0.30 * math.sin(math.pi * u))
        seg = Pos((x0 + x1) / 2 - HORN_BOSS_D / 2, 0, 0) * Box(
            x1 - x0 + 0.02, w, PLATE, align=(Align.CENTER, Align.CENTER, Align.MIN))
        part = seg if part is None else part + seg
    part += horn_boss(PLATE)
    # Side rails: a flat plate this long is weak in bending about Y, and stiffness
    # here costs a gram.
    for sy in (-1, 1):
        part += Pos(d / 2, sy * (width / 2 - WALL / 2), PLATE) * Box(
            d, WALL, 7.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return part - (Pos(d, 0, 0) * servo_cut()) - horn_cut()


def make_tibia(d=None):
    """Driven about +Z at the origin, reaching d to the foot.

    Tapered: the bending moment falls off toward the tip, and mass out there is
    what the knee servo pays for on every step.
    """
    d = d if d is not None else cfg.TIBIA * 1000
    part = horn_boss(PLATE)
    w0 = max(SERVO_W + 2 * WALL, HORN_BOSS_D)
    n = 12
    for i in range(n):
        x0, x1 = d * i / n, d * (i + 1) / n
        w = w0 * (1.0 - 0.55 * (x0 / d))
        part += Pos((x0 + x1) / 2, 0, PLATE / 2) * Box(
            x1 - x0 + 0.02, w, PLATE,
            align=(Align.CENTER, Align.CENTER, Align.CENTER))
    # Spine along the top, for stiffness where the moment is highest.
    part += Pos(d * 0.35, 0, PLATE) * Box(
        d * 0.7, WALL, 6.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # Foot pad; rubber goes on this.
    part += Pos(d, 0, PLATE / 2 - FOOT_DROP / 2) * Cylinder(
        FOOT_D / 2, PLATE + FOOT_DROP,
        align=(Align.CENTER, Align.CENTER, Align.CENTER))
    return part - horn_cut()


PARTS = {
    "coxa_link": (make_coxa_link, "coxa axis -> femur servo axis", cfg.COXA),
    "femur_link": (make_femur_link, "femur axis -> knee servo axis", cfg.FEMUR),
    "tibia": (make_tibia, "knee axis -> foot tip", cfg.TIBIA),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="build and report, no export")
    args = ap.parse_args()

    print(f"links from robot_config.py: {cfg.COXA*1000:.0f} / {cfg.FEMUR*1000:.0f}"
          f" / {cfg.TIBIA*1000:.0f} mm\n")
    if not args.check:
        os.makedirs(OUT, exist_ok=True)

    ok = True
    total = 0.0
    for name, (fn, what, span) in PARTS.items():
        part = fn()
        bb = part.bounding_box()
        vol = part.volume / 1000.0
        mass = vol * 1.24 * 0.46            # PLA, ~30 % infill
        total += mass
        print(f"{name:12s} {what}")
        print(f"{'':12s} span {span*1000:6.1f} mm   bbox "
              f"{bb.size.X:5.1f} x {bb.size.Y:5.1f} x {bb.size.Z:5.1f} mm")
        print(f"{'':12s} {vol:6.2f} cm3   ~{mass:5.1f} g printed")
        if bb.size.X < span * 1000:
            print(f"{'':12s} ERROR: part is shorter than the link it spans")
            ok = False
        if vol < 1.0:
            print(f"{'':12s} ERROR: almost no material - a cut has eaten the part")
            ok = False
        # Can it actually bolt to a horn? A boss that lands inside a plate has
        # its bore filled straight back in unless the cut comes last, and the
        # part looks perfectly fine until someone tries to assemble it.
        solid = part.solids()[0]
        if solid.is_inside(Vector(0, 0, PLATE / 2)):
            print(f"{'':12s} ERROR: horn bore is solid - cut order wrong")
            ok = False
        if solid.is_inside(Vector(HORN_SCREW_R, 0, PLATE / 2)):
            print(f"{'':12s} ERROR: horn screw holes are solid")
            ok = False
        if not solid.is_inside(Vector((HORN_BORE_D / 2 + HORN_SCREW_R) / 2, 0, PLATE / 2)):
            print(f"{'':12s} ERROR: no material left in the horn boss")
            ok = False
        if not args.check:
            export_step(part, os.path.join(OUT, f"{name}.step"))
            export_stl(part, os.path.join(OUT, f"{name}.stl"))
        print()

    print(f"printed mass per leg ~{total:.0f} g, six legs ~{6*total:.0f} g")
    # Two servos ride on the leg links - the femur servo on the coxa link and the
    # knee servo on the femur link. The coxa servo belongs to the body, so it
    # sits in MASS_BODY, not here.
    assumed = 1000 * (cfg.MASS_COXA + cfg.MASS_FEMUR + cfg.MASS_TIBIA) - 2 * 55
    print(f"robot_config assumes {assumed:.0f} g of plastic per leg"
          f" (link masses less the 2 servos they carry)")
    if not args.check:
        print(f"\nwritten to {os.path.relpath(OUT, ROOT)}")
    print("\nServo hole pattern is the published MG996R one. Check it against a")
    print("real DT996 with calipers before printing all six legs.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
