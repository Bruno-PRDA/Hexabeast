#!/usr/bin/env python3
"""Generate the whole robot: body, camera mast, and a stance-pose assembly.

    python tools/gen_robot_cad.py               # parts + assembly into CAD/generated/
    python tools/gen_robot_cad.py --check       # build and verify, no export

Leg parts come from gen_leg_cad.py; this adds everything they bolt to and
assembles the lot at the standing pose so the result can be looked at and,
more usefully, measured.

The assembly is not decoration. It is built by chaining the REAL mechanical
offsets - wall thicknesses, servo heights, horn faces - and then asking where
each foot actually lands. The kinematic model assumes an ideal planar leg, so
any gap between the two is a gap between the robot and its own simulator.
"""
import argparse
import math
import os
import sys

from build123d import (Align, Box, Cylinder, Pos, Rot, Vector, export_step,
                       export_stl)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "hexapod_gait"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from hexapod_gait import leg_ik, robot_config as cfg  # noqa: E402
import gen_leg_cad as leg  # noqa: E402

OUT = os.path.join(ROOT, "CAD", "generated")

# --- Body --------------------------------------------------------------------
BODY_T = 5.0             # plate thickness
HIP_PAD_R = 30.0         # material around each coxa servo cutout
CORE_L = 150.0           # central slab, carries the electronics
CORE_W = 104.0

# --- Electronics footprints --------------------------------------------------
# PCA9685 breakout: 62.5 x 25.4 mm, 4 holes on a 57.2 x 20.3 pitch.
PCA_HOLE_X, PCA_HOLE_Y = 57.2, 20.3
PCA_SCREW_D = 2.8        # M2.5 clearance
# ESP32-S3 CAM carrier. VERIFY - boards differ; Freenove and XIAO are not alike.
ESP_HOLE_X, ESP_HOLE_Y = 40.0, 20.0
ESP_SCREW_D = 2.4
# Battery strap slots for a 2S LiPo.
BATT_SLOT_W, BATT_SLOT_L, BATT_SLOT_SEP = 4.0, 26.0, 70.0

# --- Camera mast -------------------------------------------------------------
# The knees are what set this, not the body. A front leg's knee rises to 80 mm
# above the plate at the top of its swing - stand the camera at 62 mm, as the
# first draft did, and the legs sweep up through the shot every stride. 105 mm
# clears the worst case by 25 mm.
MAST_H = 105.0           # camera eye height above the body plate
MAST_W = 26.0
MAST_T = 8.0             # thicker, because it is now tall enough to wobble
CAM_TILT = 15.0          # degrees nose-down; the robot should see its own feet
CAM_PLATE_W, CAM_PLATE_H, CAM_PLATE_T = 34.0, 32.0, 4.0
CAM_LENS_D = 11.0
MAST_SCREW_D = 3.4       # M3 into the body
MAST_FOOT_L = 26.0


def _hips():
    """(x, y, yaw_deg) per leg, millimetres - straight from robot_config."""
    return [(m[0] * 1000, m[1] * 1000, yaw) for _n, m, yaw, _g in cfg.LEGS]


def make_body():
    """Body plate: six coxa servo mounts, electronics, battery, mast socket.

    A coxa servo's shaft is VERTICAL, so the plate it bolts to is horizontal -
    the easy orientation. Each servo body points INWARD so the plate stays
    compact and the legs get a clear sweep outside it.
    """
    part = Pos(0, 0, 0) * Box(CORE_L, CORE_W, BODY_T,
                              align=(Align.CENTER, Align.CENTER, Align.MIN))
    for hx, hy, _yaw in _hips():
        part += Pos(hx, hy, 0) * Cylinder(
            HIP_PAD_R, BODY_T, align=(Align.CENTER, Align.CENTER, Align.MIN))

    cuts = None
    for hx, hy, yaw in _hips():
        # +180 so the servo body extends toward the centre, not off the edge.
        c = Pos(hx, hy, 0) * Rot(0, 0, yaw + 180) * leg.servo_cut()
        cuts = c if cuts is None else cuts + c

    # Two PCA9685s, one either side of the battery.
    for sy in (-1, 1):
        for hx in (-PCA_HOLE_X / 2, PCA_HOLE_X / 2):
            for hy in (-PCA_HOLE_Y / 2, PCA_HOLE_Y / 2):
                cuts += Pos(hx, sy * 36.0 + hy, 0) * Cylinder(PCA_SCREW_D / 2, 60)

    # ESP32-S3 carrier, centre-front.
    for hx in (-ESP_HOLE_X / 2, ESP_HOLE_X / 2):
        for hy in (-ESP_HOLE_Y / 2, ESP_HOLE_Y / 2):
            cuts += Pos(52.0 + hx, hy, 0) * Cylinder(ESP_SCREW_D / 2, 60)

    # Battery strap slots, centre, under the pack.
    for sx in (-1, 1):
        cuts += Pos(sx * BATT_SLOT_SEP / 2, 0, 0) * Box(
            BATT_SLOT_W, BATT_SLOT_L, 60)

    # Mast socket: two M3 into the front edge.
    for sy in (-1, 1):
        cuts += Pos(CORE_L / 2 - 10.0, sy * 9.0, 0) * Cylinder(MAST_SCREW_D / 2, 60)

    return part - cuts


def make_camera_mast():
    """Post carrying the ESP32-S3 camera, tilted nose-down.

    Height is the point: the camera has to see over the front legs, which swing
    up to STEP_HEIGHT above the body plane during their swing phase.
    """
    part = Pos(0, 0, MAST_H / 2) * Box(MAST_T, MAST_W, MAST_H,
                                       align=(Align.CENTER, Align.CENTER, Align.CENTER))
    # Foot flange, bolting to the body's front edge.
    part += Pos(MAST_FOOT_L / 2 - MAST_T / 2, 0, BODY_T / 2) * Box(
        MAST_FOOT_L, MAST_W, BODY_T, align=(Align.CENTER, Align.CENTER, Align.CENTER))

    head = Pos(0, 0, 0) * Box(CAM_PLATE_T, CAM_PLATE_W, CAM_PLATE_H,
                              align=(Align.CENTER, Align.CENTER, Align.CENTER))
    lens = Cylinder(CAM_LENS_D / 2, 40, rotation=(0, 90, 0))
    holes = None
    for hy in (-ESP_HOLE_Y / 2, ESP_HOLE_Y / 2):
        for hz in (-ESP_HOLE_X / 2, ESP_HOLE_X / 2):
            h = Pos(0, hy, hz) * Cylinder(ESP_SCREW_D / 2, 40, rotation=(0, 90, 0))
            holes = h if holes is None else holes + h
    head = head - lens - holes
    part += Pos(0, 0, MAST_H) * Rot(0, CAM_TILT, 0) * head

    cuts = None
    for sy in (-1, 1):
        c = Pos(MAST_FOOT_L - MAST_T / 2 - 10.0, sy * 9.0, BODY_T / 2) * \
            Cylinder(MAST_SCREW_D / 2, 40)
        cuts = c if cuts is None else cuts + c
    return part - cuts


# --- assembly ----------------------------------------------------------------

def stance_angles():
    """Joint angles while standing - identical for all six legs by symmetry."""
    return leg_ik.solve(cfg.REACH, 0.0, -cfg.STAND_HEIGHT)


def leg_transforms(hx, hy, yaw, angles, z_top):
    """Locations for (coxa_link, femur_link, tibia) of one leg, in body frame.

    Chains the mechanical offsets the parts actually have, which is what makes
    the foot check below meaningful: an ideal model would put every axis on the
    leg's centre plane, and the real bracket cannot.
    """
    c, f, t = (math.degrees(a) for a in angles)
    wall_y = leg.SERVO_W / 2 + leg.CLEAR + leg.WALL / 2

    coxa_loc = Pos(hx, hy, z_top) * Rot(0, 0, yaw + c)
    # The femur axis sits on the wall face, offset sideways and raised.
    femur_loc = coxa_loc * Pos(cfg.COXA * 1000, -wall_y, leg.FEMUR_AXIS_Z) * \
        Rot(-90, 0, 0) * Rot(0, 0, -f)
    knee_loc = femur_loc * Pos(cfg.FEMUR * 1000, 0, 0) * Rot(0, 0, -t)
    return coxa_loc, femur_loc, knee_loc


def foot_in_body(hx, hy, yaw, angles, z_top):
    """Where this leg's foot tip lands, by the CAD chain rather than the model."""
    _c, _f, knee = leg_transforms(hx, hy, yaw, angles, z_top)
    return (knee * Pos(cfg.TIBIA * 1000, 0, 0)).position


def make_assembly():
    parts = []
    body = make_body()
    parts.append(("body", body))
    parts.append(("mast", Pos(CORE_L / 2 - MAST_FOOT_L + leg.WALL, 0, BODY_T) *
                  make_camera_mast()))

    a = stance_angles()
    coxa_p, femur_p, tibia_p = leg.make_coxa_link(), leg.make_femur_link(), leg.make_tibia()
    for i, (hx, hy, yaw) in enumerate(_hips()):
        cl, fl, kl = leg_transforms(hx, hy, yaw, a, BODY_T)
        parts.append((f"L{i}_coxa", cl * coxa_p))
        parts.append((f"L{i}_femur", fl * femur_p))
        parts.append((f"L{i}_tibia", kl * tibia_p))
    return parts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="build and verify, no export")
    args = ap.parse_args()

    body = make_body()
    mast = make_camera_mast()
    bb = body.bounding_box()
    mb = mast.bounding_box()
    print(f"body  {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm   "
          f"{body.volume/1000:.1f} cm3   ~{body.volume/1000*1.24*0.46:.0f} g")
    print(f"mast  {mb.size.X:.0f} x {mb.size.Y:.0f} x {mb.size.Z:.0f} mm   "
          f"{mast.volume/1000:.1f} cm3   ~{mast.volume/1000*1.24*0.46:.0f} g")

    # Does the CAD chain put the feet where the kinematics says?
    print("\nfoot position: CAD assembly vs the kinematic model")
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
        d = math.dist((got.X, got.Y, got.Z), want)
        worst = max(worst, d)
        print(f"  {cfg.LEGS[i][0]:3s} CAD ({got.X:7.1f},{got.Y:7.1f},{got.Z:7.1f})"
              f"   model ({want[0]:7.1f},{want[1]:7.1f},{want[2]:7.1f})   off {d:5.1f} mm")
    print(f"\nworst foot error {worst:.1f} mm")
    if worst > 2.0:
        print("The bracket puts each joint axis off the leg's centre plane; the IK")
        print("assumes an ideal planar leg. See the note printed below.")

    if not args.check:
        os.makedirs(OUT, exist_ok=True)
        export_step(body, os.path.join(OUT, "body.step"))
        export_stl(body, os.path.join(OUT, "body.stl"))
        export_step(mast, os.path.join(OUT, "camera_mast.step"))
        export_stl(mast, os.path.join(OUT, "camera_mast.stl"))
        asm = None
        for _n, p in make_assembly():
            asm = p if asm is None else asm + p
        export_stl(asm, os.path.join(OUT, "assembly.stl"))
        print(f"\nwritten to {os.path.relpath(OUT, ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
