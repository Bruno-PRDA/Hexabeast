#!/usr/bin/env python3
"""Generate the whole robot: body, head, and a stance-pose assembly.

    python tools/gen_robot_cad.py               # parts + assembly into CAD/generated/
    python tools/gen_robot_cad.py --check       # build and verify, no export

Leg parts come from gen_leg_cad.py; this adds everything they bolt to and
assembles the lot at the standing pose so the result can be looked at and,
more usefully, measured.

The assembly is not decoration. It is built by chaining the REAL mechanical
offsets - wall thicknesses, servo heights, horn faces - and then asking where
each foot actually lands. The kinematic model assumes an ideal planar leg, so
any gap between the two is a gap between the robot and its own simulator.

STYLING. The body is overlapping circular lobes down the centreline rather than
a rectangle, which buys a spider's silhouette almost for free: narrow at the
head, broad across the leg roots, pinched at the waist, bulbous behind. Each
lobe still earns its place - the abdomen is sized by the LiPo under it, the
thorax by the hip spacing - so the shape follows the packaging instead of being
laid on top of it.
"""
import argparse
import math
import os
import sys

from build123d import (Align, Axis, Box, Cylinder, Pos, Rot, Sphere, export_step,
                       export_stl, fillet, scale)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "hexapod_gait"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from hexapod_gait import leg_ik, robot_config as cfg  # noqa: E402
import gen_leg_cad as leg  # noqa: E402

OUT = os.path.join(ROOT, "CAD", "generated")

# --- Body --------------------------------------------------------------------
BODY_T = 5.0
HIP_PAD_R = 32.0
EDGE_FILLET = 7.0        # softens the cusps where lobes meet

# (x, y, radius) down the centreline, front to back.
#
# The radii are not free. Two circles that barely overlap meet at a razor cusp,
# and no fillet fits into it - the first attempt had the middle hip pads
# overlapping the thorax by about 1 mm and the whole rounding operation failed.
# Every lobe here overlaps its neighbours, and every hip pad overlaps the spine,
# by at least 8 mm. That is what makes the outline roundable.
BODY_LOBES = [
    (86.0, 0.0, 24.0),    # head end - the neck bolts here
    (58.0, 0.0, 40.0),
    (22.0, 0.0, 54.0),    # thorax, widest where the front legs load it
    (-8.0, 0.0, 46.0),
    (-32.0, 0.0, 24.0),   # waist - the pinch is what reads as a spider
    (-58.0, 0.0, 42.0),
    (-82.0, 0.0, 46.0),   # abdomen, sized by the 2S LiPo beneath it
    (-108.0, 0.0, 30.0),  # abdomen tip
]

# --- Electronics footprints --------------------------------------------------
PCA_HOLE_X, PCA_HOLE_Y = 55.88, 19.05  # 2.2 x 0.75 inch - the board is imperial
PCA_SCREW_D = 2.8
ESP_HOLE_X, ESP_HOLE_Y = 40.0, 20.0     # VERIFY - Freenove and XIAO differ
ESP_SCREW_D = 2.4
# BATTERY. The pack hangs UNDER the plate, not inside the abdomen dome. A 2S
# pack is 90 x 34 x 20 mm and the dome's cavity is 85 x 75 at plate level, but
# the ellipsoid tapers to 70 mm long at the pack's 20 mm top face, so the front
# and rear top corners fall outside the shell in every orientation. Forcing it
# would need ABDOMEN_A ~60 and ABDOMEN_H ~50, putting the dome front at x=-14 -
# through the waist, over the rear driver, and into the 15 mm the rear legs need
# to sweep past. The dome's job is the UBECs. Underslung the pack fits with real
# margin and sits 16 mm further forward, which helps the centre of gravity.
#
# Packs have NO mounting holes - none, in any brand - so retention is straps.
# Each strap gets its own LOCAL pair of slots rather than one pair at each end:
# slots 70 mm apart would run the strap's top length straight across the dome
# floor, where the UBECs and dome bolts live.
BATT_X = -62.0                          # pack centre, spans x -17 to -107
BATT_Z = -10.0                          # centre, i.e. 0 to -20 below the plate
BATT_STRAP_X = (-28.0, -100.0)          # strap stations
BATT_STRAP_Y = 21.0                     # just outside the 34 mm pack
BATT_SLOT_W, BATT_SLOT_L = 20.0, 4.0    # strap width along X, thickness along Y

# --- Abdomen dome ------------------------------------------------------------
# A shell over the rear of the body, housing the battery. Built as spheres on
# the same spine lobes as the plate, squashed in Z - so the dome follows the
# body's outline by construction rather than being a separate shape that has to
# be kept in sync with it.
ABDOMEN_CX = -74.0       # centre of the ovoid
ABDOMEN_A = 45.0         # semi-axis along the body
ABDOMEN_B = 40.0         # semi-axis across it - the rear hips are at y=55, so
                         # this leaves 15 mm for the rear legs to sweep past
ABDOMEN_H = 38.0         # height above the plate
ABDOMEN_WALL = 2.5
ABDOMEN_BOLT_D = 3.4

# --- Head --------------------------------------------------------------------
# Height is set by the KNEES, not the body. A front leg's knee rises to 80 mm
# above the plate at the top of its swing; a camera at 62 mm would have legs
# sweeping through the shot every stride.
# NECK_H is the head CENTRE; the camera turret rides HEAD_H/2 + CAM_RISE above
# that, so the neck itself is shorter than the camera height it produces.
NECK_H = 72.0
NECK_W = 24.0
NECK_T = 8.0
NECK_SCREW_D = 3.4
NECK_FOOT_L = 26.0
HEAD_TILT = 15.0         # nose-down, so the robot can see its own feet

HEAD_W, HEAD_H, HEAD_T = 46.0, 54.0, 8.0
HEAD_FILLET = 8.0

# 1.3" SSD1306 OLED - the face. I2C 0x3C, so it costs no pins.
LCD_WIN_W, LCD_WIN_H = 30.0, 17.0       # visible glass
LCD_HOLE_X, LCD_HOLE_Y = 30.5, 28.0
LCD_SCREW_D = 2.4
LCD_Z = 9.0                             # above the head centre

# VL53L0X time-of-flight. I2C 0x29, also free.
PROX_WIN_D = 5.0
PROX_HOLE_PITCH = 20.0
PROX_SCREW_D = 2.4
PROX_Z = -17.0

CAM_LENS_D = 11.0
CAM_HOLE_Y = ESP_HOLE_Y
CAM_RISE = 11.0                         # turret above the face's top edge


def _hips():
    """(x, y, yaw_deg) per leg, millimetres - straight from robot_config."""
    return [(m[0] * 1000, m[1] * 1000, yaw) for _n, m, yaw, _g in cfg.LEGS]


def _try_fillet(part, radius, axis=Axis.Z):
    """Fillet edges along `axis`, backing off until one radius fits.

    OpenCASCADE refuses a fillet it cannot fit, and a body built from
    overlapping lobes has cusps of wildly varying angle - one tight corner
    rejects the whole operation. Retry smaller rather than give up: a 3 mm
    round still reads as organic, and losing the part is worse than losing
    4 mm of softness.
    """
    for r in (radius, radius * 0.7, radius * 0.45, radius * 0.3):
        try:
            out = fillet(part.edges().filter_by(axis), radius=r)
            if r < radius:
                print(f"  (fillet backed off to r={r:.1f})")
            return out
        except Exception:                                      # noqa: BLE001
            continue
    print(f"  (fillet r={radius} skipped - no radius fitted)")
    return part


def _spine(step=7.0):
    """BODY_LOBES interpolated into closely-spaced circles.

    Smoothness comes from spacing, not from filleting. OpenCASCADE would not
    round this outline at any radius - a single awkward cusp rejects the whole
    batch, and max_fillet could not even converge - so the shape is made smooth
    by construction instead. Circles 7 mm apart with a smoothstepped radius
    leave cusps too shallow to see, and it cannot fail.
    """
    out = []
    for (x0, y0, r0), (x1, y1, r1) in zip(BODY_LOBES, BODY_LOBES[1:]):
        span = math.dist((x0, y0), (x1, y1))
        n = max(1, int(span / step))
        for i in range(n):
            u = i / n
            s = u * u * (3.0 - 2.0 * u)          # smoothstep, so radius eases
            out.append((x0 + (x1 - x0) * u, y0 + (y1 - y0) * u, r0 + (r1 - r0) * s))
    out.append(BODY_LOBES[-1])
    return out


def make_body():
    """Body plate: six coxa mounts, electronics, battery, neck socket."""
    part = None
    for x, y, r in _spine():
        lobe = Pos(x, y, 0) * Cylinder(r, BODY_T,
                                       align=(Align.CENTER, Align.CENTER, Align.MIN))
        part = lobe if part is None else part + lobe
    for hx, hy, _yaw in _hips():
        part += Pos(hx, hy, 0) * Cylinder(
            HIP_PAD_R, BODY_T, align=(Align.CENTER, Align.CENTER, Align.MIN))

    cuts = None
    for hx, hy, yaw in _hips():
        # +180 so the servo body points inward, not off the edge.
        c = Pos(hx, hy, 0) * Rot(0, 0, yaw + 180) * leg.servo_cut()
        cuts = c if cuts is None else cuts + c

    # Two PCA9685s ACROSS the thorax, not along it: the board is 62.5 mm long
    # and the pinched waist only 48 mm wide, so lengthwise they would hang off
    # the edge. Turned 90 degrees they sit in the widest part of the body.
    for cx in (8.0, -18.0):
        for hx in (-PCA_HOLE_Y / 2, PCA_HOLE_Y / 2):
            for hy in (-PCA_HOLE_X / 2, PCA_HOLE_X / 2):
                cuts += Pos(cx + hx, hy, 0) * Cylinder(PCA_SCREW_D / 2, 60)

    for hx in (-ESP_HOLE_X / 2, ESP_HOLE_X / 2):   # ESP32-S3, thorax front
        for hy in (-ESP_HOLE_Y / 2, ESP_HOLE_Y / 2):
            cuts += Pos(45.0 + hx, hy, 0) * Cylinder(ESP_SCREW_D / 2, 60)

    for bx in BATT_STRAP_X:                  # battery straps, two local pairs
        for sy in (-1, 1):
            cuts += Pos(bx, sy * BATT_STRAP_Y, 0) * Box(
                BATT_SLOT_W, BATT_SLOT_L, 60)

    for sy in (-1, 1):                       # neck socket, front edge
        cuts += Pos(82.0, sy * 8.0, 0) * Cylinder(NECK_SCREW_D / 2, 60)

    return part - cuts


def _ellipsoid(a, b, c, cx):
    """Ellipsoid with semi-axes (a, b, c), centred at (cx, 0, 0)."""
    return Pos(cx, 0, 0) * scale(Sphere(1.0), by=(a, b, c))


def make_abdomen():
    """Domed battery shell, open underneath, bolting down to the plate.

    One ellipsoid, not a union of lobes. The first attempt followed the plate's
    spine lobes with ten overlapping spheres; OpenCASCADE fused them into four
    disjoint solids rather than one, and clipping to z >= 0 then silently
    dropped the front half. A spider's abdomen is a smooth ovoid regardless, so
    the single primitive is both more robust and more accurate.
    """
    outer = _ellipsoid(ABDOMEN_A, ABDOMEN_B, ABDOMEN_H, ABDOMEN_CX)
    outer = outer & (Pos(0, 0, 400) * Box(1200, 1200, 800))     # keep z >= 0

    # The inner shell is NOT clipped. Two solids trimmed at exactly z = 0 share
    # a coincident planar face, and the boolean returns an empty solid; letting
    # this one run below zero avoids that and opens the underside, which is what
    # a battery shell wants anyway.
    inner = _ellipsoid(ABDOMEN_A - ABDOMEN_WALL, ABDOMEN_B - ABDOMEN_WALL,
                       ABDOMEN_H - ABDOMEN_WALL, ABDOMEN_CX)
    part = outer - inner

    cuts = None

    def add(c):
        nonlocal cuts
        cuts = c if cuts is None else cuts + c

    # Bolts down into the plate, fore and aft on the centre-line flanks.
    for bx in (ABDOMEN_CX + ABDOMEN_A * 0.45, ABDOMEN_CX - ABDOMEN_A * 0.45):
        for sy in (-1, 1):
            add(Pos(bx, sy * ABDOMEN_B * 0.70, 0) * Cylinder(ABDOMEN_BOLT_D / 2, 60))

    # Cable pass-through at the front, so the battery leads reach the drivers.
    add(Pos(ABDOMEN_CX + ABDOMEN_A, 0, 10.0) * Box(20, 24, 16))

    # Vents - the UBECs live under here and they get warm.
    for i in range(5):
        add(Pos(ABDOMEN_CX + 26.0 - i * 13.0, 0, ABDOMEN_H * 0.62) *
            Box(4.5, 44, 44))

    return part - cuts


def make_head():
    """Neck plus the face: LCD, proximity sensor and camera on one part.

    Everything the robot looks at you with. The LCD is the face, the ToF sensor
    sits below it, and the camera goes above - roughly where a jumping spider
    keeps its big median eyes, which is most of the charm.
    """
    part = Pos(0, 0, NECK_H / 2) * Box(
        NECK_T, NECK_W, NECK_H, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    part += Pos(NECK_FOOT_L / 2 - NECK_T / 2, 0, BODY_T / 2) * Box(
        NECK_FOOT_L, NECK_W, BODY_T, align=(Align.CENTER, Align.CENTER, Align.CENTER))

    face = Box(HEAD_T, HEAD_W, HEAD_H, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    face = _try_fillet(face, HEAD_FILLET, Axis.X)     # round the face outline

    cuts = Pos(0, 0, LCD_Z) * Box(40, LCD_WIN_W, LCD_WIN_H)
    for hy in (-LCD_HOLE_X / 2, LCD_HOLE_X / 2):
        for hz in (-LCD_HOLE_Y / 2, LCD_HOLE_Y / 2):
            cuts += Pos(0, hy, LCD_Z + hz) * Cylinder(LCD_SCREW_D / 2, 40,
                                                      rotation=(0, 90, 0))
    cuts += Pos(0, 0, PROX_Z) * Cylinder(PROX_WIN_D / 2, 40, rotation=(0, 90, 0))
    for sy in (-1, 1):
        cuts += Pos(0, sy * PROX_HOLE_PITCH / 2, PROX_Z) * \
            Cylinder(PROX_SCREW_D / 2, 40, rotation=(0, 90, 0))

    part += Pos(0, 0, NECK_H) * Rot(0, HEAD_TILT, 0) * (face - cuts)

    # Camera turret, above the face like a spider's median eyes.
    turret = Box(HEAD_T, 30.0, 24.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    turret = _try_fillet(turret, 6.0, Axis.X)
    tcut = Cylinder(CAM_LENS_D / 2, 40, rotation=(0, 90, 0))
    for hy in (-CAM_HOLE_Y / 2, CAM_HOLE_Y / 2):
        tcut += Pos(0, hy, 0) * Cylinder(ESP_SCREW_D / 2, 40, rotation=(0, 90, 0))
    part += Pos(0, 0, NECK_H + HEAD_H / 2 + CAM_RISE) * Rot(0, HEAD_TILT, 0) * \
        (turret - tcut)

    foot_cuts = None
    for sy in (-1, 1):
        c = Pos(NECK_FOOT_L - NECK_T / 2 - 10.0, sy * 8.0, BODY_T / 2) * \
            Cylinder(NECK_SCREW_D / 2, 40)
        foot_cuts = c if foot_cuts is None else foot_cuts + c
    return part - foot_cuts


# --- Electrical components ---------------------------------------------------
# Solids representing what actually gets bolted on, so the assembly shows
# whether it all physically fits rather than only where the brackets are.
# Dimensions are millimetres (length, width, height); see docs/bom.md.
#
#   name                L      W      H     where
COMPONENTS = {
    "pca9685":       (62.23, 25.4, 10.1),
    "esp32s3cam":    (45.0, 27.0, 20.0),
    "lipo2s":        (90.0, 34.0, 20.0),    # 2200 mAh class - see BATTERY note
    "oled13":        (35.5, 33.5, 5.0),
    "vl53l0x":       (25.0, 11.0, 3.5),
    "mpu6050":       (21.2, 15.6, 3.5),
    "pcf8574":       (47.63, 15.24, 15.0),
    "ina219":        (25.4, 20.32, 10.0),
    "ubec":          (43.1, 32.3, 12.5),
}


def _block(name, loc):
    """One component as a placed box, origin at its centre."""
    l, w, h = COMPONENTS[name]
    return loc * Box(l, w, h)


def make_electronics():
    """Every electrical part, positioned in the body frame.

    Returns [(label, solid)]. The servos matter most: eighteen 40.7 x 20 x 46.5
    blocks are over half the robot's mass and most of its packaging problem, and
    until they are drawn it is easy to believe there is room for things there
    is not.
    """
    out = []
    sl, sw, sh = leg.SERVO_BODY_L, leg.SERVO_W, leg.SERVO_H
    # Servo body relative to its own shaft: shaft on the top face, offset along
    # the length, body hanging below.
    body = Pos(leg.SERVO_SHAFT_OFF, 0, -sh / 2) * Box(sl, sw, sh)

    a = stance_angles()
    for i, (hx, hy, yaw) in enumerate(_hips()):
        name = cfg.LEGS[i][0]
        # Coxa servo: shaft up through the plate, body hanging into the
        # ground clearance, pointing inward.
        out.append((f"{name}_coxa_servo",
                    Pos(hx, hy, BODY_T) * Rot(0, 0, yaw + 180) * body))
        cl, fl, kl = leg_transforms(hx, hy, yaw, a, BODY_T)
        out.append((f"{name}_femur_servo", fl * body))
        out.append((f"{name}_knee_servo", kl * body))

    # Servo drivers, across the thorax - see the note in make_body(). Spaced so
    # the 25.4 mm boards clear each other.
    out.append(("pca9685_A", _block("pca9685", Pos(12.0, 0, BODY_T + 8.0) * Rot(0, 0, 90))))
    out.append(("pca9685_B", _block("pca9685", Pos(-16.0, 0, BODY_T + 8.0) * Rot(0, 0, 90))))

    # The small I2C boards go UNDER the plate. There is 59 mm of ground
    # clearance and the coxa servos hang down in a ring at radius 70-93, so the
    # middle of the underside is empty. The IMU wants to be at the body centre
    # anyway - that is where its readings mean what the model thinks they mean -
    # and the foot-switch expander wants to be where the leg wiring arrives.
    # The IMU goes at the body centre - that is where its readings mean what the
    # model assumes - and the battery now occupies x -107..-17, so x=0 is the
    # one place under the plate that is both central and free.
    out.append(("mpu6050", _block("mpu6050", Pos(0.0, 0.0, -3.0))))
    # The expander moves off the battery and out to the side, still under the
    # plate, where the leg wiring arrives.
    out.append(("pcf8574", _block("pcf8574", Pos(12.0, -30.0, -9.0))))

    # Power, inside the abdomen dome, with the shunt next to the pack.
    out.append(("lipo2s", _block("lipo2s", Pos(BATT_X, 0, BATT_Z))))
    # Both UBECs side by side in the dome. At 43.1 x 32.3 they are the largest
    # things in there; the ellipsoid is 70 mm wide at their 12.5 mm top face, so
    # +-17 mm centres is what clears the wall.
    for sy in (-1, 1):
        out.append((f"ubec_{'L' if sy > 0 else 'R'}",
                    _block("ubec", Pos(ABDOMEN_CX, sy * 16.5, BODY_T + 6.5))))
    # The shunt does NOT go in the dome: the two UBECs take 43 of its 85 mm of
    # inner length, and anywhere else in there either fouls them or runs out
    # through the ellipsoid's taper. Under the plate beside the IMU instead,
    # close enough to the pack's forward leads to keep the sense wiring short.
    out.append(("ina219", _block("ina219", Pos(0.0, 26.0, -6.0))))

    # On the head. The ESP32-S3 CAM board carries the camera itself, so it goes
    # in the turret rather than on the thorax - the lens cannot be anywhere else.
    head_x = 82.0 - NECK_FOOT_L + NECK_T
    head_face = Rot(0, 90 + HEAD_TILT, 0)
    out.append(("esp32s3cam", _block("esp32s3cam",
                Pos(head_x - 16.0, 0, BODY_T + NECK_H + HEAD_H / 2 + CAM_RISE) * head_face)))
    out.append(("oled13", _block("oled13",
                Pos(head_x - 8.5, 0, BODY_T + NECK_H + LCD_Z) * head_face)))
    out.append(("vl53l0x", _block("vl53l0x",
                Pos(head_x - 11.0, 0, BODY_T + NECK_H + PROX_Z) * head_face)))
    return out


# --- assembly ----------------------------------------------------------------

def stance_angles():
    """Joint angles while standing - identical for all six legs by symmetry."""
    return leg_ik.solve(cfg.REACH, 0.0, -cfg.STAND_HEIGHT)


def leg_transforms(hx, hy, yaw, angles, z_top):
    """Locations for (coxa_link, femur_link, tibia) of one leg, in body frame."""
    c, f, t = (math.degrees(a) for a in angles)
    wall_y = leg.SERVO_W / 2 + leg.CLEAR + leg.WALL / 2
    coxa_loc = Pos(hx, hy, z_top) * Rot(0, 0, yaw + c)
    femur_loc = coxa_loc * Pos(cfg.COXA * 1000, -wall_y, leg.FEMUR_AXIS_Z) * \
        Rot(-90, 0, 0) * Rot(0, 0, -f)
    knee_loc = femur_loc * Pos(cfg.FEMUR * 1000, 0, 0) * Rot(0, 0, -t)
    return coxa_loc, femur_loc, knee_loc


def foot_in_body(hx, hy, yaw, angles, z_top):
    """Where this leg's foot tip lands, by the CAD chain rather than the model."""
    _c, _f, knee = leg_transforms(hx, hy, yaw, angles, z_top)
    return (knee * Pos(cfg.TIBIA * 1000, 0, 0)).position


def knee_peak():
    """Highest a knee reaches while walking - what sets the camera height."""
    from hexapod_gait import gait
    femur_axis = BODY_T + leg.FEMUR_AXIS_Z
    peak = 0.0
    for i in range(100):
        lift = gait.foot_offset((0, 0, -cfg.STAND_HEIGHT), (0.10, 0), 0.0, i / 100, 0)[2]
        f = leg_ik.solve(cfg.REACH, 0.0, -cfg.STAND_HEIGHT + lift)[1]
        peak = max(peak, femur_axis + cfg.FEMUR * 1000 * math.sin(f))
    return peak


def make_assembly(electronics=True):
    parts = [("body", make_body()),
             ("abdomen", Pos(0, 0, BODY_T) * make_abdomen()),
             ("head", Pos(82.0 - NECK_FOOT_L + NECK_T, 0, BODY_T) * make_head())]
    if electronics:
        parts += make_electronics()
    a = stance_angles()
    coxa_p, femur_p, tibia_p = (leg.make_coxa_link(), leg.make_femur_link(),
                                leg.make_tibia())
    for i, (hx, hy, yaw) in enumerate(_hips()):
        cl, fl, kl = leg_transforms(hx, hy, yaw, a, BODY_T)
        parts += [(f"L{i}_coxa", cl * coxa_p), (f"L{i}_femur", fl * femur_p),
                  (f"L{i}_tibia", kl * tibia_p)]
    return parts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="build and verify, no export")
    args = ap.parse_args()

    body, head, abdomen = make_body(), make_head(), make_abdomen()
    for name, p in (("body", body), ("abdomen", abdomen), ("head", head)):
        bb = p.bounding_box()
        print(f"{name:6s} {bb.size.X:5.0f} x {bb.size.Y:5.0f} x {bb.size.Z:5.0f} mm   "
              f"{p.volume/1000:6.1f} cm3   ~{p.volume/1000*1.24*0.46:5.1f} g")

    a = stance_angles()
    worst = 0.0
    for i, (hx, hy, yaw) in enumerate(_hips()):
        got = foot_in_body(hx, hy, yaw, a, BODY_T)
        yr = math.radians(yaw)
        # The kinematic origin is the FEMUR AXIS plane, not the body plate:
        # leg_ik measures STAND_HEIGHT from where the leg's vertical motion
        # starts. The plate sits FEMUR_AXIS_Z above it.
        z0 = BODY_T + leg.FEMUR_AXIS_Z
        want = (hx + math.cos(yr) * cfg.REACH * 1000,
                hy + math.sin(yr) * cfg.REACH * 1000,
                z0 - cfg.STAND_HEIGHT * 1000)
        worst = max(worst, math.dist((got.X, got.Y, got.Z), want))
    wall_y = leg.SERVO_W / 2 + leg.CLEAR + leg.WALL / 2
    print(f"\nfoot vs model: worst {worst:.1f} mm - the femur servo's wall offset "
          f"({wall_y:.1f} mm),")
    print(f"  a {math.degrees(math.atan2(wall_y, cfg.REACH*1000)):.2f} deg stance "
          f"rotation rather than a reach error. Trim it out at the coxa.")

    knee = knee_peak()
    cam = BODY_T + NECK_H + HEAD_H / 2 + CAM_RISE
    lcd = BODY_T + NECK_H + LCD_Z
    print(f"\nknee peak while walking {knee:.0f} mm")
    print(f"camera {cam:.0f} mm  -> clears by {cam-knee:.0f} mm"
          f"   {'OK' if cam - knee > 15 else 'TOO LOW'}")
    print(f"LCD face {lcd:.0f} mm, proximity sensor {BODY_T+NECK_H+PROX_Z:.0f} mm")

    # Does any component intersect another, or the structure it mounts to?
    # A rendered assembly hides this completely - overlapping solids just look
    # like one solid.
    print("\ncomponent interference")
    comps = make_electronics()
    structure = [("body", body), ("abdomen", Pos(0, 0, BODY_T) * abdomen),
                 ("head", Pos(82.0 - NECK_FOOT_L + NECK_T, 0, BODY_T) * make_head())]
    clashes = 0
    for i, (na, pa) in enumerate(comps):
        for nb, pb in structure + comps[i + 1:]:
            try:
                ov = (pa & pb).volume
            except Exception:
                continue
            if ov > 60.0:                 # ignore slivers; 60 mm3 is a 4 mm cube
                print(f"  {na} <-> {nb}: {ov/1000:.2f} cm3")
                clashes += 1
    print(f"  {clashes} interference(s) over 0.06 cm3"
          if clashes else "  none over 0.06 cm3")

    in_dome = sum(p.volume for n, p in comps if n.startswith('ubec'))
    cavity = ((4/3) * math.pi * (ABDOMEN_A - ABDOMEN_WALL) *
              (ABDOMEN_B - ABDOMEN_WALL) * (ABDOMEN_H - ABDOMEN_WALL) / 2)
    print(f"\nin the dome: {in_dome/1000:.0f} cm3 of UBEC in a "
          f"{cavity/1000:.0f} cm3 cavity")
    print(f"under the plate: 90 x 34 x 20 pack centred at x={BATT_X:.0f}, "
          f"{59 - 20 - 2:.0f} mm still clear beneath it")

    if not args.check:
        os.makedirs(OUT, exist_ok=True)
        export_step(body, os.path.join(OUT, "body.step"))
        export_stl(body, os.path.join(OUT, "body.stl"))
        export_step(head, os.path.join(OUT, "head.step"))
        export_stl(head, os.path.join(OUT, "head.stl"))
        export_step(abdomen, os.path.join(OUT, "abdomen.step"))
        export_stl(abdomen, os.path.join(OUT, "abdomen.stl"))
        asm = None
        for _n, p in make_assembly():
            asm = p if asm is None else asm + p
        export_stl(asm, os.path.join(OUT, "assembly.stl"))
        struct = None
        for _n, p in make_assembly(electronics=False):
            struct = p if struct is None else struct + p
        export_stl(struct, os.path.join(OUT, "assembly_printed_only.stl"))
        print(f"\nwritten to {os.path.relpath(OUT, ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
