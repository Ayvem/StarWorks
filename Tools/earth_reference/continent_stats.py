#!/usr/bin/env python3
# =============================================================================
# Tools/earth_reference/continent_stats.py — is this planet shaped like a
# planet, measured against the only one we have a photograph of.
#
# "The continents look like confetti" is a judgement, and judgements do not
# converge. This turns it into three numbers, taken from Earth's real
# land/sea mask and then from the procedural field, on the SAME equal-area
# grid:
#
#   1. LAND FRACTION.
#   2. WHERE THE LAND IS. The share of all land held by the five biggest
#      landmasses — on Earth, 95.8%. This is the number that catches
#      confetti: a field can have a perfectly Earth-like land fraction and
#      still be a thousand islands, and only this number says so.
#   3. HOW MANY PIECES, and how many of them are negligible.
#
# The Earth figures below were measured with the `global-land-mask` package
# (`pip install global-land-mask`), whose bundled raster IS the reference
# image; --earth re-measures them from scratch if it is installed, so the
# constants can never quietly drift from their source.
#
#     python3 Tools/earth_reference/continent_stats.py \
#         --glm build/linux-release/_deps/glm-src [--style 0] [--earth]
# =============================================================================
import argparse
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                "glsl_parity"))
from glsl_to_cpp import transpile  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# Measured with global-land-mask on a 720 x 1440 equal-area grid. See --earth.
EARTH = {
    "land_fraction": 0.289,
    "top5_share": 0.958,
    "largest_share": 0.543,
}

HARNESS = r"""
} // namespace glsl

#include <cstdio>
#include <cstdlib>
#include <cmath>

int main(int argc, char** argv)
{
    const int style = std::atoi(argv[1]);
    const int rows = std::atoi(argv[2]);
    const int cols = 2 * rows;
    glsl::TerrainParams t = glsl::terrainPreset(style);
    // EQUAL AREA: uniform in sin(latitude), so a cell near the pole covers
    // the same ground as one at the equator and a polar cap cannot flatter
    // the statistics.
    for (int r = 0; r < rows; ++r)
    {
        const float sinLat = -1.0f + 2.0f * (float(r) + 0.5f) / float(rows);
        const float cosLat = std::sqrt(std::max(0.0f, 1.0f - sinLat * sinLat));
        for (int c = 0; c < cols; ++c)
        {
            const float lon = 6.283185307f * (float(c) + 0.5f) / float(cols);
            const vec3 dir(cosLat * std::cos(lon), sinLat, cosLat * std::sin(lon));
            std::putchar(glsl::terrainLandFraction(t, dir) > 0.0f ? 1 : 0);
        }
    }
    return 0;
}
"""


def components(mask):
    """Connected land masses, 8-connected, with longitude wrapping around."""
    from scipy import ndimage
    rows, cols = mask.shape
    labels, count = ndimage.label(mask, structure=np.ones((3, 3), dtype=int))
    # The grid is a cylinder: a continent crossing the date line is ONE
    # continent. Union the labels that touch across the seam.
    parent = list(range(count + 1))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)

    for r in range(rows):
        for dr in (-1, 0, 1):
            rr = r + dr
            if rr < 0 or rr >= rows:
                continue
            a, b = labels[r, 0], labels[rr, cols - 1]
            if a and b:
                union(a, b)
    merged = np.array([find(i) for i in range(count + 1)])
    sizes = np.bincount(merged[labels.ravel()])[1:]
    sizes = np.sort(sizes[sizes > 0])[::-1]
    return sizes


def report(name, mask):
    total = mask.size
    land = int(mask.sum())
    sizes = components(mask)
    share = sizes / max(land, 1)
    print(f"{name}")
    print(f"  land fraction   {land / total * 100:6.2f} %"
          f"      (Earth {EARTH['land_fraction'] * 100:.1f} %)")
    print(f"  largest mass    {share[0] * 100:6.2f} % of land"
          f"  (Earth {EARTH['largest_share'] * 100:.1f} %)")
    print(f"  top 5 hold      {share[:5].sum() * 100:6.2f} % of land"
          f"  (Earth {EARTH['top5_share'] * 100:.1f} %)")
    print(f"  pieces          {len(sizes)}, of which "
          f"{int((share < 0.001).sum())} below 0.1 % of land")
    print(f"  largest eight   {np.round(share[:8] * 100, 2)}")
    return {"land_fraction": land / total, "top5_share": float(share[:5].sum()),
            "largest_share": float(share[0])}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--glm", required=True)
    parser.add_argument("--style", type=int, default=0)
    parser.add_argument("--rows", type=int, default=360)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--earth", action="store_true",
                        help="re-measure the Earth reference (needs "
                             "global-land-mask)")
    args = parser.parse_args()

    if args.earth:
        from global_land_mask import globe
        rows = args.rows
        sinlat = -1.0 + 2.0 * (np.arange(rows) + 0.5) / rows
        lat = np.degrees(np.arcsin(sinlat))
        lon = -180.0 + 360.0 * (np.arange(2 * rows) + 0.5) / (2 * rows)
        LON, LAT = np.meshgrid(lon, lat)
        report("EARTH (global-land-mask)", globe.is_land(LAT, LON))
        print()

    program = transpile(["Noise.glsl", "Terrain.glsl"]) + HARNESS
    workdir = tempfile.mkdtemp(prefix="sw_continents_")
    source = os.path.join(workdir, "stats.cpp")
    exe = os.path.join(workdir, "stats")
    with open(source, "w", encoding="utf-8") as handle:
        handle.write(program)
    build = subprocess.run([args.compiler, "-std=c++20", "-O2", "-I", args.glm,
                            source, "-o", exe], capture_output=True, text=True)
    if build.returncode != 0:
        sys.stderr.write(build.stdout + build.stderr)
        return 2
    run = subprocess.run([exe, str(args.style), str(args.rows)],
                         capture_output=True)
    mask = np.frombuffer(run.stdout, dtype=np.uint8).reshape(args.rows,
                                                             2 * args.rows)
    report(f"STYLE {args.style} (the shipped field)", mask.astype(bool))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
