#!/usr/bin/env python3
# =============================================================================
# Tools/glsl_parity/check_parity.py — the CPU/GPU divergence detector.
#
# StarWorks has one founding invariant: WHAT THE RENDERER SHOWS IS WHAT THE
# PHYSICS COLLIDES WITH. That holds only as long as Shaders/Noise.glsl and
# Shaders/Terrain.glsl remain exact ports of Engine/Source/Math/Noise.hpp and
# Engine/Source/Planet/Terrain.hpp. Reviewing that by eye does not scale.
#
# So this script MECHANICALLY TRANSPILES the two GLSL files into C++ (the
# shared subset is deliberately small: floats, vec3/ivec3, uint, structs,
# for-loops), compiles them next to the real engine headers, and evaluates
# both implementations over tens of thousands of directions on every body
# preset. Any difference in the land/sea decision is a hard failure; any
# elevation difference above a millimetre is a hard failure.
#
# Usage (from the repository root):
#     python3 Tools/glsl_parity/check_parity.py --glm <path-to-glm-include>
#
# The GLM include directory is the one CMake fetched, e.g.
#     build/linux-release/_deps/glm-src   (Linux)
#     build/windows/_deps/glm-src         (Windows)
# =============================================================================
import argparse
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

from glsl_to_cpp import transpile

HARNESS = r"""
} // namespace glsl

// --- the real engine headers ------------------------------------------------
#include "Math/Noise.hpp"
#include "Planet/Terrain.hpp"

#include <cmath>
#include <cstdio>

namespace {

glsl::TerrainParams toGlsl(const sw::planet::TerrainComponent& t)
{
    glsl::TerrainParams p;
    p.seed = t.seed;
    p.octaves = t.octaves;
    p.frequency = t.frequency;
    p.amplitude = t.amplitude;
    p.seaLevelFraction = t.seaLevelFraction;
    p.reliefFrequency = t.reliefFrequency;
    p.reliefOctaves = t.reliefOctaves;
    p.ridgeWeight = t.ridgeWeight;
    p.billowWeight = t.billowWeight;
    p.plainsWeight = t.plainsWeight;
    p.warpStrength = t.warpStrength;
    p.erosion = t.erosion;
    p.terraceStrength = t.terraceStrength;
    p.terraceCount = t.terraceCount;
    p.beltThreshold = t.beltThreshold;
    p.oceanDepth = t.oceanDepth;
    p.detailWeight = t.detailWeight;
    p.detailFrequency = t.detailFrequency;
    p.noiseOffset = t.noiseOffset;
    return p;
}

// Deterministic direction sampler (a cheap LCG + spherical mapping): no
// std::random, so the same points are checked on every machine.
vec3 sampleDirection(std::uint32_t i)
{
    std::uint32_t s = i * 2654435761u + 12345u;
    s ^= s >> 15; s *= 2246822519u; s ^= s >> 13;
    const float u = static_cast<float>(s & 0xFFFFFFu) / 16777216.0f;
    s ^= s >> 16; s *= 3266489917u; s ^= s >> 11;
    const float v = static_cast<float>(s & 0xFFFFFFu) / 16777216.0f;
    const float z = 2.0f * u - 1.0f;
    const float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
    const float phi = 6.28318530718f * v;
    return vec3(r * std::cos(phi), z, r * std::sin(phi));
}

struct Result { double maxElevationDelta = 0.0; long long signMismatches = 0;
                double maxMaskDelta = 0.0; };

Result compare(const sw::planet::TerrainComponent& cpu, int style, int samples)
{
    const glsl::TerrainParams gpu = toGlsl(cpu);
    Result r;
    for (int i = 0; i < samples; ++i)
    {
        const vec3 dir = glm::normalize(sampleDirection(static_cast<std::uint32_t>(i) +
                                                        static_cast<std::uint32_t>(style) * 7919u));
        const float maskCpu = sw::planet::terrainMask(cpu, dir);
        const float maskGpu = glsl::terrainMask(gpu, dir);
        r.maxMaskDelta = std::max<double>(r.maxMaskDelta, std::fabs(maskCpu - maskGpu));

        const float eCpu = sw::planet::terrainElevationSigned(cpu, dir);
        const float eGpu = glsl::terrainElevationSigned(gpu, dir);
        r.maxElevationDelta = std::max<double>(r.maxElevationDelta, std::fabs(eCpu - eGpu));
        if ((eCpu > 0.0f) != (eGpu > 0.0f)) { ++r.signMismatches; }
    }
    return r;
}

} // namespace

int main()
{
    struct Case { const char* name; sw::planet::TerrainComponent terrain; int style; };
    const Case cases[] = {
        {"Terra",     sw::planet::presetTerra(),        0},
        {"Luna",      sw::planet::presetLuna(),         1},
        {"Mars",      sw::planet::presetMars(),         2},
        // The landable solar system (styles 3-13): every preset that
        // collision can sample must match the shader that draws it.
        {"Mercury",   sw::planet::terrainPreset(3),     3},
        {"Io",        sw::planet::terrainPreset(4),     4},
        {"Europa",    sw::planet::terrainPreset(5),     5},
        {"Ganymede",  sw::planet::terrainPreset(6),     6},
        {"Callisto",  sw::planet::terrainPreset(7),     7},
        {"Titan",     sw::planet::terrainPreset(8),     8},
        {"Enceladus", sw::planet::terrainPreset(9),     9},
        {"Rhea",      sw::planet::terrainPreset(10),   10},
        {"Titania",   sw::planet::terrainPreset(11),   11},
        {"Oberon",    sw::planet::terrainPreset(12),   12},
        {"Triton",    sw::planet::terrainPreset(13),   13},
    };
    constexpr int kSamples = 20000;
    int failures = 0;
    for (const Case& c : cases)
    {
        const Result r = compare(c.terrain, c.style, kSamples);
        const bool ok = r.signMismatches == 0 && r.maxElevationDelta <= 1.0e-3 &&
                        r.maxMaskDelta == 0.0;
        std::printf("%-6s  mask delta %.3e  elevation delta %.6f m  land/sea mismatches %lld  %s\n",
                    c.name, r.maxMaskDelta, r.maxElevationDelta, r.signMismatches,
                    ok ? "OK" : "FAIL");
        if (!ok) { ++failures; }
    }
    // The GLSL preset tables must also match the C++ presets field by field.
    for (const Case& c : cases)
    {
        const glsl::TerrainParams a = toGlsl(c.terrain);
        const glsl::TerrainParams b = glsl::terrainPreset(c.style);
        const bool same =
            a.seed == b.seed && a.octaves == b.octaves && a.frequency == b.frequency &&
            a.amplitude == b.amplitude && a.seaLevelFraction == b.seaLevelFraction &&
            a.reliefFrequency == b.reliefFrequency && a.reliefOctaves == b.reliefOctaves &&
            a.ridgeWeight == b.ridgeWeight && a.billowWeight == b.billowWeight && a.plainsWeight == b.plainsWeight &&
            a.warpStrength == b.warpStrength && a.erosion == b.erosion &&
            a.terraceStrength == b.terraceStrength && a.terraceCount == b.terraceCount &&
            a.beltThreshold == b.beltThreshold && a.oceanDepth == b.oceanDepth && a.detailWeight == b.detailWeight &&
            a.detailFrequency == b.detailFrequency &&
            a.noiseOffset == b.noiseOffset;
        std::printf("%-6s  preset table                                             %s\n",
                    c.name, same ? "OK" : "FAIL");
        if (!same) { ++failures; }
    }
    std::printf("%s\n", failures == 0 ? "PARITY OK" : "PARITY FAILED");
    return failures == 0 ? 0 : 1;
}
"""


def main() -> int:
    parser = argparse.ArgumentParser(description="CPU/GPU terrain parity check")
    parser.add_argument("--glm", required=True, help="GLM include directory")
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--keep", action="store_true", help="keep the generated C++")
    args = parser.parse_args()

    program = transpile(["Noise.glsl", "Terrain.glsl"]) + HARNESS

    workdir = tempfile.mkdtemp(prefix="sw_parity_")
    cpp = os.path.join(workdir, "parity.cpp")
    # THE SUFFIX IS NOT COSMETIC ON WINDOWS. g++ writes "parity.exe" whatever
    # you pass to -o, so a harness that then launches "parity" compiles
    # perfectly and dies with a file-not-found — which reads as a broken
    # parity check rather than a missing four characters.
    exe = os.path.join(workdir, "parity" + (".exe" if os.name == "nt" else ""))
    with open(cpp, "w", encoding="utf-8") as handle:
        handle.write(program)

    compile_cmd = [
        args.compiler, "-std=c++20", "-O2",
        # No FMA contraction: the shader compiler does not contract these
        # expressions either, and contraction would hide a real divergence.
        "-ffp-contract=off",
        "-I", os.path.join(REPO, "Engine", "Source"),
        "-I", args.glm,
        cpp, "-o", exe,
    ]
    result = subprocess.run(compile_cmd, capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write("Transpiled GLSL failed to compile as C++:\n")
        sys.stderr.write(result.stdout + result.stderr)
        sys.stderr.write(f"\nGenerated source kept at {cpp}\n")
        return 2

    run = subprocess.run([exe], capture_output=True, text=True)
    sys.stdout.write(run.stdout)
    sys.stderr.write(run.stderr)
    if args.keep:
        sys.stdout.write(f"Generated source: {cpp}\n")
    return run.returncode


if __name__ == "__main__":
    sys.exit(main())
