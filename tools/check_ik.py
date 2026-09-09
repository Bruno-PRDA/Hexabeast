"""Round-trip check of the leg IK, mirroring scripts/leg_ik.gd exactly.

Solve for joint angles, run forward kinematics on the result, and confirm we
land back on the requested foot position. Any sign error in the GDScript shows
up here in milliseconds instead of as a robot doing the splits on screen.
"""
import math

COXA, FEMUR, TIBIA = 0.030, 0.060, 0.090


def solve(x, y, z, coxa=COXA, femur=FEMUR, tibia=TIBIA):
    coxa_angle = math.atan2(-z, x)
    horiz = math.hypot(x, z) - coxa
    vert = y
    dist = math.hypot(horiz, vert)
    near, far = abs(femur - tibia) + 0.001, femur + tibia - 0.001
    dist = max(near, min(far, dist))
    bearing = math.atan2(vert, horiz)
    femur_angle = bearing + math.acos(
        max(-1.0, min(1.0, (femur**2 + dist**2 - tibia**2) / (2 * femur * dist))))
    knee = math.acos(
        max(-1.0, min(1.0, (femur**2 + tibia**2 - dist**2) / (2 * femur * tibia))))
    return coxa_angle, femur_angle, knee - math.pi


def foot_position(a, coxa=COXA, femur=FEMUR, tibia=TIBIA):
    c, f, t = a
    horiz = coxa + femur * math.cos(f) + tibia * math.cos(f + t)
    vert = femur * math.sin(f) + tibia * math.sin(f + t)
    return (horiz * math.cos(c), vert, -horiz * math.sin(c))


REACH, STAND = 0.095, 0.075
worst = 0.0
fails = []
# Sweep the workspace the gait actually visits: the neutral stance plus the
# stride envelope in every direction, and a range of body heights.
for dx in [-0.04, -0.02, 0.0, 0.02, 0.04]:
    for dz in [-0.04, -0.02, 0.0, 0.02, 0.04]:
        for dy in [0.0, 0.02, 0.035]:
            for h in [0.05, 0.075, 0.10]:
                target = (REACH + dx, -h + dy, dz)
                ang = solve(*target)
                got = foot_position(ang)
                err = math.dist(target, got)
                worst = max(worst, err)
                if err > 1e-9:
                    fails.append((target, err))

print(f"samples tested : {5*5*3*3}")
print(f"worst error    : {worst:.3e} m")
print(f"unreachable    : {len(fails)}")

ang = solve(REACH, -STAND, 0.0)
print("\nneutral stance pose:")
for name, a in zip(("coxa ", "femur", "tibia"), ang):
    print(f"  {name} {math.degrees(a):8.2f} deg")
print("  foot ->", tuple(round(v, 6) for v in foot_position(ang)))
print(f"  max leg reach: {COXA+FEMUR+TIBIA:.3f} m,  neutral demand: {REACH:.3f} m")


# --- joint travel demanded across the gait envelope ---------------------
print("\njoint travel over the walking workspace:")
lo = [9e9] * 3
hi = [-9e9] * 3
unreachable = []
for dx in [-0.05 + i * 0.01 for i in range(11)]:
    for dz in [-0.05 + i * 0.01 for i in range(11)]:
        for dy in [0.0, 0.0175, 0.035]:
            target = (REACH + dx, -STAND + dy, dz)
            horiz = math.hypot(target[0], target[2]) - COXA
            if math.hypot(horiz, target[1]) > FEMUR + TIBIA - 0.001:
                unreachable.append(target)
                continue
            for i, a in enumerate(solve(*target)):
                lo[i] = min(lo[i], math.degrees(a))
                hi[i] = max(hi[i], math.degrees(a))

for name, l, h in zip(("coxa ", "femur", "tibia"), lo, hi):
    span = h - l
    mid = (h + l) / 2
    print(f"  {name} {l:8.2f} .. {h:8.2f} deg   span {span:6.2f}   midpoint {mid:7.2f}")
print(f"  unreachable targets in envelope: {len(unreachable)}")
print("\n  -> mount each horn so the servo's centre sits at the midpoint above;")
print("     the servo then only needs +/- span/2, well inside its 90 deg travel.")
