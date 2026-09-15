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

STYLING. The plate is a faceted profile - straight edges between stations down
the centreline - drawn out to a point at each hip and lightened with triangular
voids. It used to be a union of circular lobes, which gave a spider's outline
but rendered as a pile of bubbles; facets, points and voids are what make a
machined-looking chassis, and they are what the reference robot has.

Every station still earns its place - the abdomen is sized by the dome over it,
the waist by the servo driver that has to clear it, the nose by the neck - so
the shape follows the packaging rather than being laid on top of it.
"""
import argparse
import math
import os
import sys

from build123d import (Align, Axis, Box, Cylinder, Polygon, Pos, Rot, Sphere,
                       Vector, export_step, export_stl, extrude, fillet, scale)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "hexapod_gait"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from hexapod_gait import leg_ik, robot_config as cfg  # noqa: E402
import gen_leg_cad as leg  # noqa: E402

OUT = os.path.join(ROOT, "CAD", "generated")

# --- Body --------------------------------------------------------------------
BODY_T = 5.0
HIP_PAD_R = 32.0
HIP_PAD_N = 8            # facets per hip pad - a disc reads as a bubble

# (x, half-width) down the body, front to back, with straight facets between.
#
# These are not free numbers. Working back to front: the tail has to reach the
# x=-119 point where the dome's wall ring lands; the abdomen has to stay wider
# than that ellipse all the way round; the WAIST is 30 rather than the 24 it
# would like to be, because the rear PCA9685 is mounted across the body and its
# outer mounting holes sit at y=27.9 with the board itself overhanging to 31.1;
# the thorax is set by the hip spacing; the nose has to carry the neck foot.
# Anything narrower than this list is a hole through something.
BODY_PROFILE = [
    (96.0, 18.0),     # nose - blunt, the neck bolts at x=82
    (86.0, 24.0),
    (58.0, 40.0),
    (22.0, 54.0),     # thorax, widest where the front legs load it
    (-8.0, 46.0),
    (-32.0, 30.0),    # waist - the pinch is what still reads as a spider
    (-46.0, 40.0),    # rear coxa pad: a straight facet from the waist to the
                      # abdomen cuts inside the arc it replaced and clipped the
                      # rear servos' inboard bolt holes. check_servo_pads() found
                      # it; this station is what it takes to ring them.
    (-58.0, 42.0),
    (-82.0, 46.0),    # abdomen, outside the dome's 40 mm half-width
    (-108.0, 34.0),
    (-126.0, 14.0),   # tail
]

# Eroding a polygon properly means offsetting every edge along its own normal.
# Scaling the half-widths instead is only exact where an edge runs along x; on a
# sloped facet it erodes by cos(slope) of what was asked. So over-erode by the
# steepest facet in the profile - derived, not typed, because the number changed
# the moment a station was added and a stale constant here leaves a knife edge
# at the tail. Erosion only feeds the void filter, so erring long costs a void.
PROFILE_ERODE_K = max(
    1.0 / math.cos(math.atan2(abs(h1 - h0), abs(x1 - x0)))
    for (x0, h0), (x1, h1) in zip(BODY_PROFILE, BODY_PROFILE[1:]))

# --- Skeletal styling --------------------------------------------------------
# The plate is lightened the same way the leg links are: triangular voids inside
# a continuous rim. Placing them by eye on a lobed outline crossed by six servo
# cutouts, four bolt patterns, two strap slots and a dome footprint is how you
# put a hole through something you forgot about - so they are NOT placed by eye.
# A regular lattice proposes candidates and each one has to earn its place: it
# must miss every functional feature by VOID_KEEPOUT and lie wholly inside the
# outline eroded by VOID_RIM. Whatever survives is correct by construction, and
# adding a new component later just makes some voids disappear.
VOID_SIDE = 16.0         # triangle side
VOID_WEB = 6.0           # material between neighbouring voids
VOID_RIM = 6.5           # material left at the outline
VOID_KEEPOUT = 4.0       # clearance from any hole, cutout or bearing face
VOID_PHASES = 4          # lattice offsets tried; the one that fits most wins

# Hip arms. Drawing each pad out to a point along its own leg direction turns
# the plate into a star. They sit under the coxa link, which swings above the
# plate, so the extra reach cannot foul anything. ARM_HALF has to stay wide
# enough to ring the servo's outboard bolt pair, which lands 14.75 mm out from
# the coxa axis at +-5 mm across - check_servo_pads() in main() proves it does.
ARM_R = 42.0             # arm tip, from the coxa axis
ARM_HALF = 20.0          # arm half-width at the coxa axis

# --- Electronics footprints --------------------------------------------------
PCA_HOLE_X, PCA_HOLE_Y = 55.88, 19.05  # 2.2 x 0.75 inch - the board is imperial
PCA_SCREW_D = 2.8
# The camera board's hole pattern. Used by the HEAD TURRET only: the plate used
# to carry a copy of it at x=45, for a board that does not sit there and, on the
# Freenove, has no mounting holes at all. Drilling a pattern for an absent part
# is how a plate ends up with holes nobody can explain, so it is gone.
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

# HEAD_H is 62, not the 54 it started at, because the face has to carry the
# OLED and the ToF stacked. The ToF was modelled 6.8 mm narrower than the real
# GY-530; at its true 17.78 mm the two boards need more face than 54 mm has,
# and they overlapped by 0.04 cm3 - just under the old 0.06 cm3 sliver
# threshold, so the interference check reported "none" on a real collision.
HEAD_W, HEAD_H, HEAD_T = 46.0, 62.0, 8.0
HEAD_FILLET = 8.0

# 1.3" SSD1306 OLED - the face. I2C 0x3C, so it costs no pins.
LCD_WIN_W, LCD_WIN_H = 30.0, 17.0       # visible glass
LCD_HOLE_X, LCD_HOLE_Y = 30.5, 28.0
LCD_SCREW_D = 2.4
LCD_Z = 6.0                             # above the head centre

# VL53L0X time-of-flight. I2C 0x29, also free.
PROX_WIN_D = 5.0
PROX_HOLE_PITCH = 20.0
PROX_SCREW_D = 2.4
PROX_Z = -22.0

CAM_LENS_D = 11.0
CAM_HOLE_Y = ESP_HOLE_Y
TURRET_W, TURRET_H = 30.0, 24.0         # the panel the camera board bolts to
CAM_RISE = 11.0                         # turret above the face's top edge

# Where the head part is bolted to the plate. It was spelled out four separate
# times; head_frame() below is what everything on the face is now placed in.
HEAD_X = 82.0 - NECK_FOOT_L + NECK_T


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


def _ngon(r, n, phase_deg):
    """Regular n-gon of circumradius r, first vertex at `phase_deg`."""
    return [(r * math.cos(math.radians(phase_deg + 360.0 * k / n)),
             r * math.sin(math.radians(phase_deg + 360.0 * k / n))) for k in range(n)]


def _plate_solid(shrink=0.0, arms=True):
    """The plate's outline as a solid, optionally eroded by `shrink`.

    The eroded copy is what the void filter tests against: "wholly inside the
    plate less 6.5 mm" IS the rim condition, so asking it this way measures the
    rim against the real outline instead of against an offset worked out by hand.

    Arms are left out of the eroded copy on purpose. Eroding a sharp point
    correctly means pulling its tip back by shrink/sin(half-angle) - 2.3x here -
    and getting that subtly wrong puts a void in a 3 mm spike. Dropping them
    instead just means no voids in the arms, which is where they were never
    wanted.
    """
    k = PROFILE_ERODE_K * shrink
    stations = [(x, hw - k) for x, hw in BODY_PROFILE if hw - k > 0.0]
    if not stations:
        raise ValueError("erosion consumed the whole plate")
    # Front and back are faces too: pull the end stations in along x as well.
    stations[0] = (stations[0][0] - k, stations[0][1])
    stations[-1] = (stations[-1][0] + k, stations[-1][1])

    pts = [(x, hw) for x, hw in stations] + [(x, -hw) for x, hw in reversed(stations)]
    part = extrude(Polygon(*leg.ccw(pts), align=None), amount=BODY_T)

    for hx, hy, yaw in _hips():
        # Phase the octagon so a FLAT faces outward along the leg, which is the
        # edge the arm grows out of.
        pad = _ngon(HIP_PAD_R - 1.082 * shrink, HIP_PAD_N, yaw + 180.0 / HIP_PAD_N)
        part += Pos(hx, hy, 0) * extrude(Polygon(*leg.ccw(pad), align=None), amount=BODY_T)
        if arms:
            tri = Polygon(*leg.ccw([(0.0, ARM_HALF), (0.0, -ARM_HALF), (ARM_R, 0.0)]),
                          align=None)
            part += Pos(hx, hy, 0) * Rot(0, 0, yaw) * extrude(tri, amount=BODY_T)
    return part


PCA_CX = (12.0, -16.0)   # driver board centres - see the note in make_electronics
PCA_STANDOFF = 8.0       # board underside above the plate


def _plate_holes():
    """Every round hole through the plate: (x, y, diameter, protrudes).

    `protrudes` says which side of the plate the fastener sticks out of, which
    is the whole reason this list exists separately from the cut geometry: a
    hole is a subtraction and carries no screw, so nothing downstream can see
    the nut under it. Everything bolted from above leaves a tail below; the
    coxa servos, which hang beneath, leave heads above as well.

    One list feeds both the cuts and the fastener solids, so the two cannot
    drift - and the PCA9685 pattern being drilled 4 mm from where the boards
    were actually modelled is exactly the drift this prevents.
    """
    out = []
    for hx, hy, yaw in _hips():
        base = Pos(hx, hy, 0) * Rot(0, 0, yaw + 180)
        for sx in (-1, 1):
            for sy in (-1, 1):
                p = (base * Pos(leg.SERVO_SHAFT_OFF + sx * leg.SERVO_HOLE_PITCH_L / 2,
                                sy * leg.SERVO_HOLE_PITCH_W / 2, 0)).position
                out.append((p.X, p.Y, leg.SERVO_HOLE_D, "both"))
    for cx in PCA_CX:
        for hx in (-PCA_HOLE_Y / 2, PCA_HOLE_Y / 2):
            for hy in (-PCA_HOLE_X / 2, PCA_HOLE_X / 2):
                out.append((cx + hx, hy, PCA_SCREW_D, "below"))
    for sy in (-1, 1):                       # neck socket, front edge
        out.append((82.0, sy * 8.0, NECK_SCREW_D, "below"))
    # The dome bolts down into the plate. The plate had no holes for them at
    # all - make_abdomen drilled its own half of the joint and nothing drilled
    # the other, so the dome was bolted to solid plastic.
    for bx in (ABDOMEN_CX + ABDOMEN_A * 0.45, ABDOMEN_CX - ABDOMEN_A * 0.45):
        for sy in (-1, 1):
            out.append((bx, sy * ABDOMEN_B * 0.70, ABDOMEN_BOLT_D, "below"))
    return out


def fastener_solids():
    """The screws, nuts and heads the holes imply, as solids you can clash on."""
    out = None
    for x, y, d, side in _plate_holes():
        r = max(d + 4.0, FASTENER_FLAT) / 2.0
        if side in ("below", "both"):
            c = Pos(x, y, -FASTENER_TAIL / 2) * Cylinder(r, FASTENER_TAIL)
            out = c if out is None else out + c
        if side in ("above", "both"):
            c = Pos(x, y, BODY_T + FASTENER_HEAD / 2) * Cylinder(r, FASTENER_HEAD)
            out = c if out is None else out + c
    return out


def _body_cuts(margin=0.0):
    """Every functional cut in the plate, optionally grown by `margin`.

    One function serves twice: at margin 0 it IS the cut list, and at
    VOID_KEEPOUT it is the no-go region for lightening. They cannot drift apart,
    which is the whole point - a keepout maintained separately from the cuts it
    is meant to protect is a keepout that silently goes stale.
    """
    m = margin
    cuts = None

    def add(c):
        nonlocal cuts
        cuts = c if cuts is None else cuts + c

    # The servo body pockets. +180 so the case points inward, not off the edge.
    for hx, hy, yaw in _hips():
        base = Pos(hx, hy, 0) * Rot(0, 0, yaw + 180)
        add(base * Pos(leg.SERVO_SHAFT_OFF, 0, 0) * Box(
            leg.SERVO_BODY_L + 2 * leg.CLEAR + 2 * m,
            leg.SERVO_W + 2 * leg.CLEAR + 2 * m, 120))
        # The servo tab holes are SLOTS, along the body - see leg.servo_cut().
        for sx in (-1, 1):
            for sy in (-1, 1):
                add(base * Pos(leg.SERVO_SHAFT_OFF + sx * leg.SERVO_HOLE_PITCH_L / 2,
                               sy * leg.SERVO_HOLE_PITCH_W / 2, 0) *
                    leg.slot_cut(leg.SERVO_SLOT_L + 2 * m, leg.SERVO_HOLE_D + 2 * m))

    # Every other round hole comes from _plate_holes(), which is also what the
    # fastener solids are built from. Two PCA9685s go ACROSS the thorax, not
    # along it: the board is 62.2 mm long and the waist only 60 mm wide.
    for x, y, d, _side in _plate_holes():
        if d == leg.SERVO_HOLE_D:
            continue                          # already cut, as a slot
        add(Pos(x, y, 0) * Cylinder(d / 2 + m, 60))

    for bx in BATT_STRAP_X:                  # battery straps, two local pairs
        for sy in (-1, 1):
            add(Pos(bx, sy * BATT_STRAP_Y, 0) * Box(
                BATT_SLOT_W + 2 * m, BATT_SLOT_L + 2 * m, 60))

    if m > 0.0:
        # Bearing faces. These are not cuts, so they only exist in the keepout:
        # the dome's wall lands on the plate all the way round its ellipse, and
        # the neck's foot lands on the front. A void under either would have the
        # part standing on air.
        def ellipse(a, b):
            return Pos(ABDOMEN_CX, 0, 0) * scale(Cylinder(1.0, 60), by=(a, b, 1.0))
        add(ellipse(ABDOMEN_A + m, ABDOMEN_B + m) -
            ellipse(ABDOMEN_A - ABDOMEN_WALL - m, ABDOMEN_B - ABDOMEN_WALL - m))
        add(Pos(82.0 - NECK_FOOT_L / 2, 0, 0) * Box(
            NECK_FOOT_L + 2 * m, NECK_W + 2 * m, 60))
    return cuts


def _void_candidates(ox, oy):
    """Triangle vertex lists for one lattice phase, y >= 0 half only."""
    s, ht = VOID_SIDE, VOID_SIDE * math.sqrt(3.0) / 2.0
    dx, dy = s + VOID_WEB, ht + VOID_WEB
    out = []
    row = 0
    while (row + 0.5) * dy + oy < 95.0:
        yc = (row + 0.5) * dy + oy
        col = -9
        while col * dx + ox < 115.0:
            for up in (True, False):
                xc = col * dx + ox + (0.0 if up else dx / 2.0)
                g = 1.0 if up else -1.0
                out.append([(xc - s / 2, yc - g * ht / 3), (xc + s / 2, yc - g * ht / 3),
                            (xc, yc + g * 2 * ht / 3)])
            col += 1
        row += 1
    return out


_VOID_CACHE = {}


def _lightening_voids(report=False):
    """Triangular lattice over the plate, filtered down to what actually fits.

    Only the y >= 0 half is proposed and each survivor is mirrored, so the plate
    comes out symmetric even though the acceptance test is numerical.

    The lattice's origin is swept, and it matters far more than it sounds like
    it should: across the 16 phases tried, the number of voids that fit runs
    from 0 to 10. A plate this crowded leaves only a handful of legal pockets,
    and whether a triangle lands in one is almost entirely down to phase. There
    is no picking that by eye, so it is searched.

    Cheap point-in-solid tests throw out the candidates that are simply off the
    body - four fifths of them - so only the survivors cost a boolean.
    """
    key = (VOID_SIDE, VOID_WEB, VOID_RIM, VOID_KEEPOUT)
    if key in _VOID_CACHE:
        return _VOID_CACHE[key][0]

    inner = _plate_solid(shrink=VOID_RIM, arms=False)
    inner_s = inner.solids()[0]
    keep = _body_cuts(margin=VOID_KEEPOUT)
    z = BODY_T / 2

    best = (-1.0, None, None)
    step = (VOID_SIDE + VOID_WEB) / VOID_PHASES
    for pi in range(VOID_PHASES):
        for pj in range(VOID_PHASES):
            ox, oy = pi * step, pj * step
            fit, area = [], 0.0
            for pts in _void_candidates(ox, oy):
                cx = sum(p[0] for p in pts) / 3.0
                cy = sum(p[1] for p in pts) / 3.0
                if not inner_s.is_inside(Vector(cx, cy, z)):
                    continue
                if not all(inner_s.is_inside(Vector(px, py, z)) for px, py in pts):
                    continue
                probe = extrude(Polygon(*leg.ccw(pts), align=None), amount=BODY_T)
                if (probe - inner).volume > 1.0 or (probe & keep).volume > 1.0:
                    continue
                fit.append(pts)
                area += probe.volume / BODY_T
            if area > best[0]:
                best = (area, fit, (ox, oy))

    _area, fit, phase = best
    out = None
    for pts in fit:
        for sy in (1.0, -1.0):
            cut = Pos(0, 0, -30) * extrude(
                Polygon(*leg.ccw([(px, sy * py) for px, py in pts]), align=None),
                amount=60)
            out = cut if out is None else out + cut
    _VOID_CACHE[key] = (out, len(fit), _area, phase, fit)
    if report:
        print(f"lightening: {2 * len(fit)} voids, {2 * _area / 100:.1f} cm2 of "
              f"plate removed (lattice phase {phase[0]:.1f}, {phase[1]:.1f} mm)")
    return out


def check_servo_pads(ring=2.5):
    """Is every coxa servo bolt hole fully ringed by plate?

    A faceted outline with a point at each hip is easy to draw too narrow, and
    the failure is quiet: the bolt hole opens onto the edge, the render still
    looks fine, and you find out when the servo has three screws instead of
    four. Probe all the way round each hole rather than trusting the profile
    arithmetic.
    """
    solid = _plate_solid().solids()[0]
    bad = []
    for hx, hy, yaw in _hips():
        base = Pos(hx, hy, 0) * Rot(0, 0, yaw + 180)
        for sx in (-1, 1):
            for sy in (-1, 1):
                c = (base * Pos(leg.SERVO_SHAFT_OFF + sx * leg.SERVO_HOLE_PITCH_L / 2,
                                sy * leg.SERVO_HOLE_PITCH_W / 2, BODY_T / 2)).position
                # Probe around the SLOT, not a round hole: the cut is
                # SERVO_SLOT_L long, so a radius based on the 3.4 mm width
                # would sample inside material the slot has removed.
                r = leg.SERVO_SLOT_L / 2 + ring
                for k in range(8):
                    a = math.pi * k / 4
                    p = Vector(c.X + r * math.cos(a), c.Y + r * math.sin(a), c.Z)
                    if not solid.is_inside(p):
                        bad.append((hx, hy, sx, sy))
                        break
    return bad


def check_voids_cut(body):
    """Is there still material where each lightening void should be?

    The voids are built, filtered and subtracted without anyone ever looking at
    the result, so a void that silently fails to cut - a flipped winding puts
    the prism below the plate - leaves a heavier part that renders perfectly.
    Probe for it instead.
    """
    solid = body.solids()[0]
    missed = 0
    for pts in _VOID_CACHE.get((VOID_SIDE, VOID_WEB, VOID_RIM, VOID_KEEPOUT),
                               (None, None, None, None, []))[4]:
        for sy in (1.0, -1.0):
            cx = sum(p[0] for p in pts) / 3.0
            cy = sy * sum(p[1] for p in pts) / 3.0
            if solid.is_inside(Vector(cx, cy, BODY_T / 2)):
                missed += 1
    return missed


def make_body():
    """Body plate: six coxa mounts, electronics, battery, neck socket."""
    part = _plate_solid()
    voids = _lightening_voids()
    cuts = _body_cuts()
    if voids is not None:
        cuts += voids
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
    turret = Box(HEAD_T, TURRET_W, TURRET_H,
                 align=(Align.CENTER, Align.CENTER, Align.CENTER))
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
    "esp32s3cam":    (57.1, 28.1, 7.0),     # Freenove fnk0085: 50.9 PCB + 6.2 of
                                            # WROOM module overhanging one end
    "lipo2s":        (90.0, 34.0, 20.0),    # 2200 mAh class - see BATTERY note
    "oled13":        (35.4, 33.5, 3.9),
    "vl53l0x":       (25.4, 17.78, 4.6),    # GY-530. Was modelled 25 x 11 x 3.5 -
                                            # 6.8 mm too narrow, which is what let
                                            # it sit inside the OLED undetected
    "mpu6050":       (21.0, 16.0, 3.3),
    "pcf8574":       (47.63, 15.24, 15.0),
    "ina219":        (25.4, 20.32, 10.0),
    "ubec":          (43.1, 32.3, 12.5),
}

# Fastener tails. A hole is a SUBTRACTION, so nothing in the interference check
# can see the screw that goes through it, the nut on the far side, or the head
# standing proud - and the space under a plate is exactly where both the small
# I2C boards and every fastener tail want to be. Model them as solids: below the
# plate for anything bolted down from above, above it for the coxa servo heads.
FASTENER_TAIL = 6.0      # nut + thread protruding under the plate
FASTENER_HEAD = 4.0      # cap head standing proud above it
FASTENER_FLAT = 7.0      # across flats of an M3/M4 nut or cap head
CLASH_MIN = 5.0          # mm3 - a 1.7 mm cube; see the note in main()


def _block(name, loc):
    """One component as a placed box, origin at its centre."""
    l, w, h = COMPONENTS[name]
    return loc * Box(l, w, h)


def head_frame():
    """The FACE's own frame: origin at the face centre, tilted with it.

    This is the same chain make_head() builds the face on, so anything placed
    through it tilts with the face instead of beside it. The boards used to be
    positioned in the body frame at a fixed x offset, as though the head stood
    upright; 15 degrees of nose-down over 22 mm of drop swings the face back
    5.7 mm, which is how the ToF ended up 0.9 mm inside the slab it was supposed
    to be bolted behind.
    """
    return Pos(HEAD_X, 0, BODY_T + NECK_H) * Rot(0, HEAD_TILT, 0)


def _on_face(name, z_local, extra_gap=0.0):
    """A board lying flat on the BACK of the face, landscape, centred at z."""
    h = COMPONENTS[name][2]
    return _block(name, head_frame() *
                  Pos(-(HEAD_T / 2 + h / 2 + extra_gap), 0, z_local) *
                  Rot(0, 90, 0) * Rot(0, 0, 90))


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

    # Servo drivers, across the thorax - see the note in make_body(). Centres
    # come from PCA_CX, which is now also what gets drilled: the plate used to
    # be drilled at 8 and -18 while the boards were modelled at 12 and -16, so
    # bolting them to the real holes would have left 0.6 mm between them.
    #
    # PCA_STANDOFF is 8 mm, not the 3 mm that a board resting on the plate
    # implies. Two things need the height: the middle legs' coxa bolt heads
    # stand proud right under the board's edge, and the forward battery strap
    # runs through its slot at x=-28 - directly beneath the rear board, where a
    # 3 mm gap would mean unbolting a driver to re-tension the pack.
    zc = BODY_T + PCA_STANDOFF + COMPONENTS["pca9685"][2] / 2
    for name, cx in (("pca9685_A", PCA_CX[0]), ("pca9685_B", PCA_CX[1])):
        out.append((name, _block("pca9685", Pos(cx, 0, zc) * Rot(0, 0, 90))))

    # The small I2C boards go UNDER the plate. There is 59 mm of ground
    # clearance and the coxa servos hang down in a ring at radius 70-93, so the
    # middle of the underside is empty. The IMU wants to be at the body centre
    # anyway - that is where its readings mean what the model thinks they mean -
    # and the foot-switch expander wants to be where the leg wiring arrives.
    # The IMU goes at the body centre - that is where its readings mean what the
    # model assumes - and the battery now occupies x -107..-17, so x=0 is the
    # one place under the plate that is both central and free.
    out.append(("mpu6050", _block("mpu6050", Pos(0.0, 0.0, -3.0))))
    # The expander sits athwart in the forward belly. It was at (12, -30), which
    # put it straight under three of the rear driver's four mounting screws with
    # 1.5 mm of air - no solid-on-solid overlap, so the old interference check
    # saw nothing, but the standoff nuts would have landed on the board.
    out.append(("pcf8574", _block("pcf8574", Pos(30.0, 24.0, -8.0) * Rot(0, 0, 90))))

    # Power, inside the abdomen dome, with the shunt next to the pack.
    out.append(("lipo2s", _block("lipo2s", Pos(BATT_X, 0, BATT_Z))))
    # ONE UBEC in the dome, not two. Side by side at +-16.5 they cleared each
    # other by 0.7 mm and each pushed a corner through the shell, because the
    # ellipsoid's inner half-width falls from 32.5 mm at the centre to 29.5 mm
    # at the boards' fore and aft ends while a 32.3 mm board needs 32.3 mm all
    # the way along. Tandem needs 86 mm of an 85 mm cavity. Raising ABDOMEN_B
    # would eat the 15 mm the rear legs need to sweep past.
    out.append(("ubec_L", _block("ubec", Pos(ABDOMEN_CX, 0.0, BODY_T + 6.5))))
    # UBEC #2 goes athwart in the forward belly instead, mirroring the expander.
    out.append(("ubec_R", _block("ubec", Pos(30.0, -24.0, -11.5) * Rot(0, 0, 90))))
    # The shunt does NOT go in the dome, and not in the y = 26..36 band on either
    # flank either: all eight driver screws land on y = +-27.94 and the middle
    # legs' inboard coxa nuts reach y = 31.7, so that whole strip is fastener.
    # Forward on the centreline, ahead of the IMU, is clear.
    out.append(("ina219", _block("ina219", Pos(62.0, 0.0, -7.0))))

    # On the head, placed in the FACE's frame so they tilt with it. Both are
    # LANDSCAPE: the OLED window (30 across Y by 17 in Z) and its 30.5 mm hole
    # pitch describe a board lying that way, as does the ToF's 20 mm pitch, but
    # the blocks were modelled turned 90 degrees from the features that mount
    # them. The features are right, so the blocks were what moved.
    out.append(("oled13", _on_face("oled13", LCD_Z)))
    out.append(("vl53l0x", _on_face("vl53l0x", PROX_Z)))
    # The camera board rides behind the turret, which is where the lens has to
    # be. It is a 57.1 mm board on a 24 mm turret - see check_head_mounted().
    out.append(("esp32s3cam", _block("esp32s3cam", head_frame() *
                Pos(-(HEAD_T / 2 + COMPONENTS["esp32s3cam"][2] / 2), 0,
                    HEAD_H / 2 + CAM_RISE) * Rot(0, 90, 0) * Rot(0, 0, 90))))
    return out


# Which panel each head board bolts to, and how big that panel is. A board
# BEHIND a panel is correct - that is how a face-mounted sensor works - so a
# bounding-box test against the whole head is useless here and flags the ToF
# for sitting where it belongs. What matters is whether the board's footprint
# fits the panel it screws to.
FACE_PANELS = {
    "oled13":     ("face", HEAD_W, HEAD_H),
    "vl53l0x":    ("face", HEAD_W, HEAD_H),
    "esp32s3cam": ("turret", TURRET_W, TURRET_H),
}


def check_head_mounted(margin=2.0):
    """Does each head board's footprint fit the panel it bolts to?

    Overlap checks cannot catch a board that misses its bracket entirely: two
    solids that do not touch look exactly like a part correctly mounted beside
    it. Compare footprints against panels instead.
    """
    bad = []
    for name, (panel, pw, ph) in FACE_PANELS.items():
        l, w, _h = COMPONENTS[name]         # l across the panel, w up it
        over = max(l - (pw - 2 * margin), w - (ph - 2 * margin))
        if over > 0.0:
            bad.append((name, panel, l, w, pw, ph, over))
    return bad


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
             ("head", Pos(HEAD_X, 0, BODY_T) * make_head())]
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

    _lightening_voids(report=True)
    bad = check_servo_pads()
    if bad:
        print(f"ERROR: {len(bad)} coxa bolt hole(s) break out of the plate edge - "
              f"widen ARM_HALF or BODY_PROFILE")
        for hx, hy, sx, sy in bad:
            print(f"  hip ({hx:.0f}, {hy:.0f}) hole ({sx:+d}, {sy:+d})")
    else:
        print("all 24 coxa bolt holes ringed by plate")

    unmounted = check_head_mounted()
    for name, panel, l, w, pw, ph, over in unmounted:
        print(f"WARNING: {name} is {l:.1f} x {w:.1f} on a {pw:.0f} x {ph:.0f} "
              f"{panel} - overhangs by {over:.1f} mm")
    if not unmounted:
        print("every head board fits the panel it bolts to")

    body, head, abdomen = make_body(), make_head(), make_abdomen()
    missed = check_voids_cut(body)
    if missed:
        print(f"ERROR: {missed} lightening void(s) did not cut - material is still "
              f"there. Check polygon winding.")
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
                 ("head", Pos(HEAD_X, 0, BODY_T) * make_head())]
    clashes = 0
    for i, (na, pa) in enumerate(comps):
        for nb, pb in structure + comps[i + 1:]:
            try:
                ov = (pa & pb).volume
            except Exception:
                continue
            if ov > CLASH_MIN:
                print(f"  {na} <-> {nb}: {ov/1000:.3f} cm3")
                clashes += 1
    print(f"  {clashes} interference(s) over {CLASH_MIN/1000:.3f} cm3"
          if clashes else f"  none over {CLASH_MIN/1000:.3f} cm3")

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
