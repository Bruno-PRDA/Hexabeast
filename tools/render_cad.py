#!/usr/bin/env python3
"""Render exported STLs to a PNG so the assembly can be looked at.

    python tools/render_cad.py                       # CAD/export -> cad_views.png
    python tools/render_cad.py --out foo.png --size 700

Four orthographic views - isometric, top, front, side - with each part in its
own colour and a legend. Flat shading, painter's algorithm; this is for
understanding layout and spotting a joint axis, not for pretty pictures.
"""
import argparse
import glob
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from measure_cad import read_stl  # noqa: E402

PALETTE = [
    (66, 133, 244), (219, 68, 55), (244, 180, 0), (15, 157, 88),
    (171, 71, 188), (0, 172, 193), (255, 112, 67), (120, 144, 156),
]
LIGHT = np.array([0.4, 0.5, 0.75])
LIGHT = LIGHT / np.linalg.norm(LIGHT)

# name, forward (into screen), up
VIEWS = [
    ("isometric", (-0.6, -0.6, -0.53), (0.0, 0.0, 1.0)),
    ("top  (X right, Y up)", (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)),
    ("front (X right, Z up)", (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    ("side  (Y right, Z up)", (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
]


def load(path_glob):
    parts = []
    seen = {}
    for pattern in ("*.stl", "*.STL"):
        for p in glob.glob(os.path.join(path_glob, pattern)):
            seen.setdefault(os.path.normcase(os.path.realpath(p)), p)
    for p in sorted(seen.values()):
        tris = np.asarray(read_stl(p), dtype=np.float64)  # (n, 3, 3)
        if len(tris) == 0:
            continue
        name = os.path.basename(p)
        for junk in ("Assemblage1 - ", ".STL", ".stl"):
            name = name.replace(junk, "")
        parts.append((name, tris))
    return parts


def basis(forward, up):
    f = np.array(forward, dtype=np.float64)
    f /= np.linalg.norm(f)
    u = np.array(up, dtype=np.float64)
    r = np.cross(u, f)
    if np.linalg.norm(r) < 1e-9:
        r = np.cross(np.array([1.0, 0.0, 0.0]), f)
    r /= np.linalg.norm(r)
    u = np.cross(f, r)
    u /= np.linalg.norm(u)
    return r, u, f


def render_view(parts, forward, up, size, label):
    r, u, f = basis(forward, up)
    allv = np.concatenate([t.reshape(-1, 3) for _n, t in parts])
    sx, sy = allv @ r, allv @ u
    # numpy 2 removed ndarray.ptp(); the free function is the portable spelling.
    pad = 0.06 * max(np.ptp(sx), np.ptp(sy))
    x0, x1 = sx.min() - pad, sx.max() + pad
    y0, y1 = sy.min() - pad, sy.max() + pad
    scale = min(size / (x1 - x0), (size - 26) / (y1 - y0))
    ox, oy = (size - (x1 - x0) * scale) / 2, (size - 26 - (y1 - y0) * scale) / 2

    img = Image.new("RGB", (size, size), (250, 250, 252))
    draw = ImageDraw.Draw(img)

    faces = []
    for idx, (_name, tris) in enumerate(parts):
        v = tris.reshape(-1, 3)
        px = (v @ r - x0) * scale + ox
        py = (size - 26) - ((v @ u - y0) * scale + oy)
        depth = (tris.mean(axis=1) @ f)
        n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        ln = np.linalg.norm(n, axis=1, keepdims=True)
        n = n / np.where(ln < 1e-12, 1.0, ln)
        shade = np.clip(np.abs(n @ LIGHT), 0.0, 1.0) * 0.65 + 0.35
        base = np.array(PALETTE[idx % len(PALETTE)], dtype=np.float64)
        for i in range(len(tris)):
            faces.append((depth[i], px[3 * i:3 * i + 3], py[3 * i:3 * i + 3], base * shade[i]))

    faces.sort(key=lambda t: -t[0])          # painter's: far first
    for _d, px, py, col in faces:
        draw.polygon([(px[0], py[0]), (px[1], py[1]), (px[2], py[2])],
                     fill=tuple(int(c) for c in col))

    draw.rectangle([0, size - 26, size, size], fill=(255, 255, 255))
    draw.text((8, size - 19), f"{label}    {(x1 - x0):.0f} x {(y1 - y0):.0f} mm",
              fill=(40, 40, 44))
    draw.rectangle([0, 0, size - 1, size - 1], outline=(210, 210, 214))
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "CAD", "export"))
    ap.add_argument("--out", default="cad_views.png")
    ap.add_argument("--size", type=int, default=620)
    args = ap.parse_args()

    parts = load(args.path)
    if not parts:
        print(f"no STL found under {args.path}")
        return 1
    print(f"{len(parts)} parts, {sum(len(t) for _n, t in parts)} triangles")

    s = args.size
    sheet = Image.new("RGB", (s * 2, s * 2 + 30), (255, 255, 255))
    for i, (label, fwd, up) in enumerate(VIEWS):
        sheet.paste(render_view(parts, fwd, up, s, label), ((i % 2) * s, (i // 2) * s))

    draw = ImageDraw.Draw(sheet)
    x = 8
    for idx, (name, _t) in enumerate(parts):
        col = PALETTE[idx % len(PALETTE)]
        draw.rectangle([x, s * 2 + 10, x + 12, s * 2 + 22], fill=col)
        draw.text((x + 17, s * 2 + 11), name, fill=(40, 40, 44))
        x += 24 + 7 * len(name)
    sheet.save(args.out)
    print("wrote", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
