#!/usr/bin/env python3
"""CPU twin of GameScene.cpp's buildEndurance(): parses the ten .swpart
files, applies the SAME blueprint poses, and renders the assembly with
matplotlib. Not a product renderer — a check that the trigonometry in the
blueprint puts thirty-five parts where the film puts them, runnable in a
container with no GPU.

Keep the constants below in step with buildEndurance(); they are the same
five numbers, and the joint check at the bottom is the same arithmetic.
"""
import json
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = os.path.join(os.path.dirname(__file__), "..", "..", "Assets", "Parts")

PARTS = {
    200: "en1_ring_habitat.swpart",
    201: "en2_ring_engine.swpart",
    202: "en3_command_pod.swpart",
    203: "en4_ring_spoke.swpart",
    204: "en5_ranger.swpart",
    205: "en6_lander.swpart",
    206: "en7_cargo_pod.swpart",
    207: "en8_cryo_module.swpart",
    208: "en9_core_hub.swpart",
    209: "en10_core_spoke.swpart",
}

(HABITAT, ENGINE, COMMAND, TUNNEL, RANGER, LANDER, CARGO, CRYO, HUB,
 SPOKE) = range(200, 210)

APOTHEM = 29.4
MODULE_HALF_LEN = 6.2
MODULE_HALF_RAD = 2.4
CARGO_HALF_RAD = 2.6
HUB_RADIUS = 3.4
SPOKE_HALF_LEN = 11.8
COUNT = 12
TAN_HALF = math.tan(math.pi / COUNT)
SEC_HALF = 1.0 / math.cos(math.pi / COUNT)

KIND = {0: ENGINE, 3: ENGINE, 6: ENGINE, 9: ENGINE,
        1: CARGO, 4: CARGO, 7: CARGO, 10: CARGO,
        2: HABITAT, 8: HABITAT, 5: COMMAND, 11: CRYO}

CRAFT = [(1, RANGER, 0.85, False), (7, RANGER, 0.85, False),
         (4, LANDER, 1.5, True), (10, LANDER, 1.5, True)]


def euler_xyz(deg):
    rx, ry, rz = [math.radians(d) for d in deg]
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    mx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    my = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    mz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return mz @ my @ mx


def box_faces(half):
    hx, hy, hz = half
    v = np.array([[sx * hx, sy * hy, sz * hz]
                  for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    idx = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4),
           (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [v[list(i)] for i in idx]


def cylinder_faces(r0, half_len, r1=None, segments=12):
    r1 = r0 if r1 is None else r1
    faces, ring0, ring1 = [], [], []
    for s in range(segments):
        a0 = 2 * math.pi * s / segments
        a1 = 2 * math.pi * (s + 1) / segments
        p00 = [r0 * math.cos(a0), r0 * math.sin(a0), -half_len]
        p01 = [r0 * math.cos(a1), r0 * math.sin(a1), -half_len]
        p10 = [r1 * math.cos(a0), r1 * math.sin(a0), half_len]
        p11 = [r1 * math.cos(a1), r1 * math.sin(a1), half_len]
        faces.append(np.array([p00, p01, p11, p10]))
        ring0.append(p00)
        ring1.append(p10)
    faces.append(np.array(ring0))
    faces.append(np.array(ring1))
    return faces


def ellipsoid_faces(radii, segments=14, stacks=7):
    rx, ry, rz = radii
    faces = []
    for t in range(stacks):
        th0, th1 = math.pi * t / stacks, math.pi * (t + 1) / stacks
        for s in range(segments):
            a0 = 2 * math.pi * s / segments
            a1 = 2 * math.pi * (s + 1) / segments
            quad = [[rx * math.sin(th) * math.cos(a),
                     ry * math.sin(th) * math.sin(a),
                     rz * math.cos(th)]
                    for th, a in ((th0, a0), (th0, a1), (th1, a1), (th1, a0))]
            faces.append(np.array(quad))
    return faces


def shape_faces(shape):
    kind, size = shape["kind"], shape["size"]
    if kind == "box":
        return box_faces(size)
    if kind == "cylinder":
        return cylinder_faces(size[0], size[1])
    if kind == "cone":
        return cylinder_faces(size[0], size[1], size[2])
    if kind == "sphere":
        return ellipsoid_faces(size)
    if kind == "tube":
        return cylinder_faces(size[0], size[1])
    return []


def orient(x, y, z):
    return np.column_stack([x, y, z])


def build_blueprint():
    """Mirrors buildEndurance(): ring in the XY plane, axis +Z, nose -Z."""
    axis = np.array([0.0, 0.0, 1.0])
    bp = [(HUB, np.zeros(3), np.eye(3))]
    for i in range(COUNT):
        th = i * 2 * math.pi / COUNT
        radial = np.array([math.cos(th), math.sin(th), 0.0])
        tangent = np.array([-math.sin(th), math.cos(th), 0.0])
        bp.append((KIND[i], radial * APOTHEM, orient(radial, -axis, tangent)))

        vth = th + math.pi / COUNT
        vradial = np.array([math.cos(vth), math.sin(vth), 0.0])
        vtangent = np.array([-math.sin(vth), math.cos(vth), 0.0])
        bp.append((TUNNEL, vradial * (APOTHEM * SEC_HALF),
                   orient(vradial, -axis, vtangent)))

    for k in range(6):
        th = (k * 2) * 2 * math.pi / COUNT
        radial = np.array([math.cos(th), math.sin(th), 0.0])
        tangent = np.array([-math.sin(th), math.cos(th), 0.0])
        bp.append((SPOKE, radial * (HUB_RADIUS + SPOKE_HALF_LEN),
                   orient(axis, -tangent, radial)))

    for module, definition, stand_off, roof in CRAFT:
        th = module * 2 * math.pi / COUNT
        radial = np.array([math.cos(th), math.sin(th), 0.0])
        tangent = np.array([-math.sin(th), math.cos(th), 0.0])
        rot = (orient(tangent, -radial, axis) if roof
               else orient(-tangent, radial, axis))
        bp.append((definition, radial * (APOTHEM + CARGO_HALF_RAD + stand_off), rot))
    return bp


def main():
    defs = {}
    for pid, name in PARTS.items():
        with open(os.path.join(ROOT, name), encoding="utf-8") as f:
            defs[pid] = json.load(f)

    needed = APOTHEM * TAN_HALF - MODULE_HALF_LEN
    authored = abs(defs[TUNNEL]["hitboxes"][0]["halfExtents"][2])
    reach = HUB_RADIUS + 2 * SPOKE_HALF_LEN
    print(f"ring {2 * (APOTHEM + MODULE_HALF_RAD):.1f} m across; "
          f"tunnel needs {needed:.3f} m half-length, authored {authored:.3f} m "
          f"({'closes' if abs(needed - authored) < 0.05 else 'GAPS'}); "
          f"spoke reaches {reach:.2f} m, inner face at "
          f"{APOTHEM - MODULE_HALF_RAD:.2f} m "
          f"({'meets' if abs(reach - (APOTHEM - MODULE_HALF_RAD)) < 0.05 else 'MISSES'})")

    light = np.array([0.5, 0.62, 0.6])
    light /= np.linalg.norm(light)
    # elev/azim are matplotlib's, whose up is +Z; the ring lies in XZ, so
    # "down the ring axis" is elev 0 and "edge on" is elev 90.
    views = [("down the ring axis", 89, -90, None), ("edge on", 1, -90, None),
             ("oblique", 26, -54, None),
             ("the core hub, on six spokes", 34, -60, (0.0, 0.0, 0.0, 15.0))]
    fig = plt.figure(figsize=(13.5, 12), facecolor="black")
    for vi, (label, elev, azim, focus) in enumerate(views):
        ax = fig.add_subplot(2, 2, vi + 1, projection="3d", facecolor="black")
        polys, colors = [], []
        for pid, pos, rot in build_blueprint():
            for shape in defs[pid]["shapes"]:
                if not shape["visible"]:
                    continue
                srot = rot @ euler_xyz(shape["rotationDeg"])
                spos = pos + rot @ np.array(shape["position"])
                for face in shape_faces(shape):
                    polys.append((face @ srot.T) + spos)
                    c, glow = shape["color"], shape["emissive"]
                    colors.append([min(1.0, ch + glow * 0.9) for ch in c])
        shaded = []
        for face, color in zip(polys, colors):
            n = np.cross(face[1] - face[0], face[2] - face[0])
            length = np.linalg.norm(n)
            lam = 0.32 + 0.68 * abs(n @ light) / length if length > 1e-9 else 0.6
            shaded.append([min(1.0, ch * lam) for ch in color])
        ax.add_collection3d(Poly3DCollection(polys, facecolors=shaded,
                                             edgecolors="none"))
        cx, cy, cz, half = focus if focus is not None else (0.0, 0.0, 0.0, 34.0)
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)
        ax.set_zlim(cz - half, cz + half)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()
        ax.set_title(label, color="white", fontsize=11)
    out = os.path.join(os.path.dirname(__file__), "endurance_preview.png")
    fig.tight_layout()
    fig.savefig(out, dpi=115, facecolor="black")
    print("wrote", out)


if __name__ == "__main__":
    main()
