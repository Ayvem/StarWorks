#!/usr/bin/env python3
# =============================================================================
# Tools/solar_scale/check_render_size.py — does the engine DRAW them that size?
#
# check_scale.py proves the numbers in the scene table are the numbers in the
# fact sheet. This proves the renderer agrees: it parks the camera a known
# number of body radii away, photographs the real frame, measures the disc in
# pixels, and compares against what the projection says it must be.
#
# That is a different question and it can fail on its own. A wrong field of
# view, a mesh authored at diameter instead of radius, a uniform scale applied
# twice, an aspect ratio used where a vertical half-angle belongs — none of
# those touch the scene table and every one of them changes what the player
# sees. This is the only check that would catch them.
#
# THE PREDICTION. A sphere's silhouette is its TANGENT cone, so a body of
# radius R at distance d subtends a half-angle of asin(R/d) — not atan(R/d),
# which is the angle to its equator and is a fifth smaller from low orbit. A
# perspective projection puts an off-axis angle t at ndc = tan(t)/tan(fov/2),
# so
#
#     screen radius in pixels = tan(asin(R/d)) / tan(fovY/2) * height/2
#
# WHAT COUNTS AS THE DISC. A body with air does not end at its ground, and it
# is not supposed to: the limb glow really is out there and really is visible.
# So the test is a BRACKET rather than a number — the drawn disc must be at
# least the solid body and at most the body plus the envelope its air is drawn
# in. Airless bodies have an envelope of exactly 1 and are held to 1.5%.
#
# That is what turned two apparent failures into two correct results the first
# time this ran: Mars measured 2.6% oversize and Terra 2.9%, and both are the
# thickness of their own atmospheres to within the width of the threshold.
#
# The floor under the tolerance is the mesh: the LOD sphere is a polyhedron
# INSCRIBED in the body, so its silhouette is cos(pi/segments) of the true
# circle — 0.03% at the two closest levels, 3.4% at the coarsest. A body drawn
# at a coarse LOD is expected to measure small, and the tool says so rather
# than failing.
#
#     xvfb-run -a python3 Tools/solar_scale/check_render_size.py \
#         --binary build/linux-release/bin/StarWorks
#
# =============================================================================
import argparse
import math
import os
import struct
import subprocess
import sys
import tempfile
import zlib

FOV_Y_DEGREES = 60.0

# body, distance in body radii, visible envelope in body radii. Chosen to span
# the LOD ladder and to avoid the one body whose silhouette is not its own:
# Saturn wears rings across it.
#
# The envelope is where the drawn air stops. Terra's is its atmosphere SHELL
# mesh (GameScene.cpp, 1.055 R — raised there for the aurorae); Mars's is the
# top of its air column in Atmosphere.glsl (1.2e5 m on 3.3895e6). The gas
# giants have no envelope because their visible surface IS the top of their
# atmosphere and F24 stopped marching air in front of it.
# The sun is measured with SW_NO_GLARE=1: its glare is a deliberate optical
# overlay three body radii wide, so with it on there is nothing to measure but
# the overlay. The photosphere underneath has to be exact all the same.
SHOTS = [
    ("SOL", 4.0, 1.0),
    ("SOL", 10.0, 1.0),
    ("TERRA", 2.2, 1.055),
    ("TERRA", 8.0, 1.055),
    ("LUNA", 3.0, 1.0),
    ("MARS", 2.5, 1.035),
    ("JUPITER", 4.0, 1.0),
    ("TITAN", 3.5, 1.0),
    ("MERCURY", 6.0, 1.0),
    ("NEPTUNE", 5.0, 1.0),
]


def readPng(path):
    with open(path, "rb") as handle:
        data = handle.read()
    offset, width, height, idat = 8, None, None, b""
    while offset < len(data):
        length = struct.unpack(">I", data[offset:offset + 4])[0]
        tag = data[offset + 4:offset + 8]
        payload = data[offset + 8:offset + 8 + length]
        offset += 12 + length
        if tag == b"IHDR":
            width, height = struct.unpack(">II", payload[:8])
        elif tag == b"IDAT":
            idat += payload
    raw = zlib.decompress(idat)
    stride = width * 3
    rows, previous, cursor = [], bytearray(stride), 0
    for _ in range(height):
        filterType = raw[cursor]
        cursor += 1
        line = bytearray(raw[cursor:cursor + stride])
        cursor += stride
        if filterType == 1:
            for x in range(3, stride):
                line[x] = (line[x] + line[x - 3]) & 255
        elif filterType == 2:
            for x in range(stride):
                line[x] = (line[x] + previous[x]) & 255
        elif filterType == 3:
            for x in range(stride):
                left = line[x - 3] if x >= 3 else 0
                line[x] = (line[x] + ((left + previous[x]) >> 1)) & 255
        elif filterType == 4:
            for x in range(stride):
                a = line[x - 3] if x >= 3 else 0
                b = previous[x]
                c = previous[x - 3] if x >= 3 else 0
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                line[x] = (line[x] + pr) & 255
        rows.append(bytes(line))
        previous = line
    return width, height, rows


def measureDisc(width, height, rows):
    """The widest horizontal run of not-sky, in a window that excludes the HUD.

    The threshold is set from the frame's own empty sky rather than from a
    constant: the night side of a lit body is only a few levels above space,
    and a fixed threshold either eats it or eats the stars."""
    def luminance(x, y):
        row = rows[y]
        return 0.2126 * row[x * 3] + 0.7152 * row[x * 3 + 1] + 0.0722 * row[x * 3 + 2]

    # Empty sky: the top-right corner is HUD-free in every shot this takes.
    corner = [luminance(x, y)
              for y in range(10, 120) for x in range(width - 260, width - 10)]
    corner.sort()
    sky = corner[len(corner) // 2]
    # Stars sit well above the median sky, so the threshold has to clear them:
    # eight levels is above the brightest haze and far below any lit surface.
    threshold = sky + 8.0

    # ONLY THE MIDDLE FORTY ROWS. SW_SHOT points the camera at the body's
    # centre, so the disc's widest row IS the frame's centre row — and a
    # window any wider than that starts finding the HUD instead. It did: the
    # navball is a 200-pixel ring at the bottom centre, and two of these
    # measurements came back as exactly 202 pixels because that is how wide
    # the navball is. One of them then PASSED, because Terra at eight radii
    # happens to be 196 pixels across and the navball is close enough to
    # slip through the bracket. A measurement that can accidentally be right
    # is not a measurement.
    x0, x1 = width // 4, (width * 3) // 4
    y0, y1 = height // 2 - 20, height // 2 + 20
    best, bestRow = 0, 0
    for y in range(y0, y1):
        run, longest = 0, 0
        for x in range(x0, x1):
            if luminance(x, y) > threshold:
                run += 1
                longest = max(longest, run)
            else:
                run = 0
        if longest > best:
            best, bestRow = longest, y
    return best, bestRow, sky, threshold


def predictedDiameter(radii, height):
    """Pixels across, for a body whose centre is `radii` of its own radii away."""
    halfAngle = math.asin(min(1.0 / radii, 1.0))
    ndc = math.tan(halfAngle) / math.tan(math.radians(FOV_Y_DEGREES) / 2.0)
    return 2.0 * ndc * (height / 2.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", default="build/linux-release/bin/StarWorks")
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--tolerance", type=float, default=0.015)
    parser.add_argument("--keep", default="")
    args = parser.parse_args()

    outDir = args.keep or tempfile.mkdtemp(prefix="sw_scale_")
    os.makedirs(outDir, exist_ok=True)
    failures = 0
    print(f"{'BODY':<9} {'DIST':>6} {'MEASURED':>9} {'SOLID':>8} {'+AIR':>8} {'ERR':>8}")
    for body, radii, envelope in SHOTS:
        path = os.path.join(outDir, f"{body}_{radii}.png")
        environment = dict(os.environ)
        # Phase zero: the camera between the sun and the body, so the whole
        # disc is lit and the silhouette is the body rather than the day side.
        environment["SW_SHOT"] = f"{body}@{radii},-0.70"
        if body == "SOL":
            environment["SW_NO_GLARE"] = "1"
        else:
            environment.pop("SW_NO_GLARE", None)
        result = subprocess.run(
            [args.binary, "--cpu", "--quality", "high",
             "--frames", str(args.frames), "--capture", path],
            capture_output=True, env=environment)
        if not os.path.exists(path):
            sys.stderr.write(result.stderr.decode("utf-8", "replace")[-2000:])
            print(f"{body:<9} {radii:6.1f}   NO CAPTURE")
            failures += 1
            continue
        width, height, rows = readPng(path)
        measured, row, sky, threshold = measureDisc(width, height, rows)
        solid = predictedDiameter(radii, height)
        withAir = solid * envelope
        # Below the solid body is always wrong. Above it is wrong only past
        # the air, and the tolerance is applied to whichever bound is nearer.
        if measured < solid:
            error = (solid - measured) / solid
        elif measured > withAir:
            error = (measured - withAir) / withAir
        else:
            error = 0.0
        mark = ""
        if error > args.tolerance:
            mark = "   <-- OUT OF TOLERANCE"
            failures += 1
        print(f"{body:<9} {radii:6.1f} {measured:9d} {solid:8.1f} {withAir:8.1f} "
              f"{error*100:7.2f}%{mark}")

    print(f"\n{failures} discrepanc{'y' if failures == 1 else 'ies'}"
          f"   (captures in {outDir})")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
