#!/usr/bin/env python3
"""Measure exported CAD and report the numbers robot_config.py needs.

    python tools/measure_cad.py                     # everything in CAD/export/
    python tools/measure_cad.py --density 1240      # PLA; PETG 1270, ABS 1040, alu 2700

Two formats, because each answers a different question:

STL  is a closed triangle mesh, so its volume is exact (divergence theorem)
     and mass follows from the material density. This is where the real
     MASS_COXA / MASS_FEMUR / MASS_TIBIA come from.

STEP keeps analytic surfaces, so every cylinder - every servo shaft, bearing
     bore and pivot - is stored with its true axis. Grouping those axes by
     direction and measuring the perpendicular distance between them gives
     the axis-to-axis link lengths, which is what the kinematics needs and
     what a bounding box cannot tell you.

Nothing here needs SolidWorks, or a CAD kernel.
"""
import argparse
import glob
import math
import os
import re
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "hexapod_gait"))

DENSITY_PLA = 1240.0  # kg/m^3


# --- STL ---------------------------------------------------------------------

def read_stl(path):
    """Triangles as [(v0, v1, v2), ...] in millimetres. Handles both flavours."""
    with open(path, "rb") as fh:
        head = fh.read(5)
        fh.seek(0)
        data = fh.read()
    # An ASCII STL starts with "solid", but so do some binary ones - trust the
    # length instead: binary is exactly 84 + 50*n bytes.
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + 50 * n:
            tris = []
            for i in range(n):
                off = 84 + 50 * i
                f = struct.unpack_from("<12f", data, off)
                tris.append((f[3:6], f[6:9], f[9:12]))
            return tris
    if head[:5] != b"solid":
        raise ValueError(f"{path}: neither valid binary nor ASCII STL")
    nums = re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)
    verts = [tuple(float(c) for c in v) for v in nums]
    return [tuple(verts[i:i + 3]) for i in range(0, len(verts) - 2, 3)]


def stl_properties(tris):
    """Volume (mm^3), surface area (mm^2), bounding box, centroid."""
    vol6 = 0.0
    area2 = 0.0
    cx = cy = cz = 0.0
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for a, b, c in tris:
        # Signed tetrahedron volume against the origin; closed meshes sum exactly.
        d = (a[0] * (b[1] * c[2] - b[2] * c[1])
             - a[1] * (b[0] * c[2] - b[2] * c[0])
             + a[2] * (b[0] * c[1] - b[1] * c[0]))
        vol6 += d
        # Centroid of a solid is the volume-weighted mean of tetra centroids.
        cx += d * (a[0] + b[0] + c[0])
        cy += d * (a[1] + b[1] + c[1])
        cz += d * (a[2] + b[2] + c[2])
        u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
        n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        area2 += math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2)
        for p in (a, b, c):
            for i in range(3):
                lo[i] = min(lo[i], p[i])
                hi[i] = max(hi[i], p[i])
    vol = vol6 / 6.0
    centroid = (cx / (4 * vol6), cy / (4 * vol6), cz / (4 * vol6)) if vol6 else (0, 0, 0)
    return abs(vol), area2 / 2.0, lo, hi, centroid


# --- STEP --------------------------------------------------------------------

ENTITY = re.compile(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\((.*)\)\s*$", re.S)


def parse_step(path):
    """{id: (TYPE, raw_args)} from the DATA section."""
    text = open(path, "r", encoding="utf-8", errors="replace").read()
    if "DATA;" in text:
        text = text.split("DATA;", 1)[1]
    ents = {}
    for stmt in text.split(";"):
        stmt = stmt.strip()
        m = ENTITY.match(stmt)
        if m:
            ents[int(m.group(1))] = (m.group(2), m.group(3))
    return ents


def _refs(args):
    return [int(x) for x in re.findall(r"#(\d+)", args)]


def _floats(args):
    return [float(x) for x in re.findall(r"-?\d+\.\d*(?:[eE][-+]?\d+)?", args)]


def step_axes(ents):
    """Cylindrical surfaces as (origin, direction, radius), millimetres."""
    def triple(eid):
        t, a = ents.get(eid, (None, None))
        if t in ("CARTESIAN_POINT", "DIRECTION"):
            f = _floats(a)
            return tuple(f[:3]) if len(f) >= 3 else None
        return None

    out = []
    for _eid, (typ, args) in ents.items():
        if typ != "CYLINDRICAL_SURFACE":
            continue
        rs = _refs(args)
        fs = _floats(args)
        if not rs or not fs:
            continue
        placement = ents.get(rs[0])
        if not placement or placement[0] != "AXIS2_PLACEMENT_3D":
            continue
        prefs = _refs(placement[1])
        if len(prefs) < 2:
            continue
        origin, direction = triple(prefs[0]), triple(prefs[1])
        if origin and direction:
            out.append((origin, direction, fs[-1]))
    return out


def step_bbox(ents):
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for _eid, (typ, args) in ents.items():
        if typ != "CARTESIAN_POINT":
            continue
        f = _floats(args)
        if len(f) >= 3:
            for i in range(3):
                lo[i] = min(lo[i], f[i])
                hi[i] = max(hi[i], f[i])
    return lo, hi


def group_axes(axes, ang_tol=0.02, pos_tol=0.3):
    """Collapse coaxial cylinders into distinct axes, biggest bore first.

    A single shaft shows up as many cylindrical faces at different radii;
    what matters is the line they share.
    """
    groups = []
    for origin, direction, radius in axes:
        n = math.sqrt(sum(c * c for c in direction))
        if n < 1e-9:
            continue
        d = tuple(c / n for c in direction)
        placed = False
        for g in groups:
            dot = abs(sum(a * b for a, b in zip(d, g["dir"])))
            if dot < 1 - ang_tol:
                continue
            w = tuple(a - b for a, b in zip(origin, g["origin"]))
            proj = sum(a * b for a, b in zip(w, g["dir"]))
            perp = math.sqrt(max(0.0, sum(c * c for c in w) - proj * proj))
            if perp <= pos_tol:
                g["radii"].append(radius)
                g["count"] += 1
                placed = True
                break
        if not placed:
            groups.append({"origin": origin, "dir": d, "radii": [radius], "count": 1})
    groups.sort(key=lambda g: max(g["radii"]), reverse=True)
    return groups


def axis_distance(a, b):
    """Perpendicular distance between two parallel axes - the link length."""
    w = tuple(p - q for p, q in zip(b["origin"], a["origin"]))
    proj = sum(x * y for x, y in zip(w, a["dir"]))
    return math.sqrt(max(0.0, sum(c * c for c in w) - proj * proj))


# --- report ------------------------------------------------------------------

def _find(root, exts):
    """Files with any of `exts`, case-insensitively, without duplicates.

    Windows filesystems are case-insensitive, so globbing "*.stl" and "*.STL"
    returns the same file twice - which silently doubles the reported total mass.
    """
    seen = {}
    for ext in exts:
        for pattern in (ext, ext.upper()):
            for path in glob.glob(os.path.join(root, "**", f"*.{pattern}"), recursive=True):
                seen.setdefault(os.path.normcase(os.path.realpath(path)), path)
    return sorted(seen.values())


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?", default=os.path.join(ROOT, "CAD", "export"))
    ap.add_argument("--density", type=float, default=DENSITY_PLA,
                    help="kg/m^3 for mass from STL volume (default PLA 1240)")
    ap.add_argument("--max-axes", type=int, default=6, help="axes to list per STEP file")
    args = ap.parse_args()

    stls = _find(args.path, ("stl",))
    steps = _find(args.path, ("step", "stp"))

    if not stls and not steps:
        print(f"nothing found under {args.path}")
        print("Export from SolidWorks: File > Save As > STEP AP214 (assembly),")
        print("and STL for each part, into CAD/export/.")
        return 1

    if stls:
        print("=" * 74)
        print(f"STL - volume and mass at {args.density:.0f} kg/m3")
        print("=" * 74)
        total = 0.0
        for path in stls:
            try:
                tris = read_stl(path)
                vol, area, lo, hi, cen = stl_properties(tris)
            except Exception as exc:                      # noqa: BLE001
                print(f"{os.path.basename(path):28s} FAILED: {exc}")
                continue
            grams = vol * 1e-9 * args.density * 1000.0
            total += grams
            print(f"{os.path.basename(path):28s} {len(tris):>7} tri  "
                  f"{vol / 1000.0:8.2f} cm3  {grams:7.1f} g")
            print(f"{'':28s} bbox {hi[0] - lo[0]:6.1f} x {hi[1] - lo[1]:6.1f} x "
                  f"{hi[2] - lo[2]:6.1f} mm   CoM ({cen[0]:.1f}, {cen[1]:.1f}, {cen[2]:.1f})")
        print(f"{'TOTAL':28s} {'':>7}      {'':8}      {total:7.1f} g")

    for path in steps:
        print()
        print("=" * 74)
        print(f"STEP - {os.path.basename(path)}")
        print("=" * 74)
        ents = parse_step(path)
        lo, hi = step_bbox(ents)
        print(f"entities {len(ents)}   bbox {hi[0] - lo[0]:.1f} x {hi[1] - lo[1]:.1f} x "
              f"{hi[2] - lo[2]:.1f} mm")
        groups = group_axes(step_axes(ents))
        if not groups:
            print("no cylindrical surfaces found")
            continue
        print(f"\n{len(groups)} distinct cylindrical axes (largest bore first):")
        shown = groups[:args.max_axes]
        for i, g in enumerate(shown):
            print(f"  [{i}] r={max(g['radii']):5.2f} mm  at ({g['origin'][0]:7.1f},"
                  f" {g['origin'][1]:7.1f}, {g['origin'][2]:7.1f})"
                  f"  dir ({g['dir'][0]:+.2f}, {g['dir'][1]:+.2f}, {g['dir'][2]:+.2f})"
                  f"  faces {g['count']}")
        print("\nperpendicular distance between parallel axes - candidate link lengths:")
        found = False
        for i in range(len(shown)):
            for j in range(i + 1, len(shown)):
                if abs(sum(a * b for a, b in zip(shown[i]["dir"], shown[j]["dir"]))) < 0.98:
                    continue
                d = axis_distance(shown[i], shown[j])
                if d > 1.0:
                    print(f"  [{i}] -> [{j}]   {d:7.2f} mm")
                    found = True
        if not found:
            print("  (no parallel axis pairs - joint axes may be perpendicular here)")

    print("\nAxis-to-axis distances are what robot_config.py's COXA / FEMUR / TIBIA")
    print("mean. Part bounding boxes are always larger and are not interchangeable.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
