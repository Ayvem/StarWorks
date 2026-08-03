#!/usr/bin/env python3
# =============================================================================
# Tools/planet_preview/render_preview.py — see the shader without a GPU.
#
# The planetary look now lives in Shaders/PlanetSurface.glsl. Reviewing a
# change to it used to mean "build, launch, fly to Terra, squint". This tool
# transpiles that module (plus Noise/Terrain) to C++ with the same machinery
# the parity check uses, ray-casts the planet on the CPU with the SAME
# shading path the fragment shader runs — footprint-driven LOD included —
# and writes a PNG.
#
# It is a review and non-regression instrument, not a renderer: no HUD, no
# rings, no vessels. It DOES draw the cloud deck and the atmosphere, and
# that line used to say it did not — which cost two sessions of chasing a
# bug that was a correct cloud field all along. What it shows, exactly, is
# the terrain
# shading: relief, self-shadowing, biomes, LOD behaviour with distance.
#
#     python3 Tools/planet_preview/render_preview.py \
#         --glm build/linux-release/_deps/glm-src --out captures/
#
# =============================================================================
import argparse
import os
import struct
import subprocess
import sys
import tempfile
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                "glsl_parity"))
from glsl_to_cpp import transpile  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

HARNESS = r"""
} // namespace glsl

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

// The M24 cinematic grade, mirrored from Mesh.frag so the preview shows the
// tone the game actually ships.
static vec3 hable(vec3 x)
{
    const float A = 0.15f, B = 0.50f, C = 0.10f, D = 0.20f, E = 0.02f, F = 0.30f;
    return ((x * (A * x + C * B) + D * E) / (x * (A * x + B) + D * F)) - E / F;
}
static vec3 gradeCinematic(vec3 color)
{
    const vec3 mapped = hable(color) / hable(vec3(4.0f)).x;
    vec3 graded = (mapped - 0.42f) * 0.98f + 0.42f;
    const float luminance = glm::dot(graded, vec3(0.2126f, 0.7152f, 0.0722f));
    graded = glm::mix(vec3(luminance), graded, 0.82f);
    const vec3 lift = vec3(0.010f, 0.012f, 0.016f);
    graded = lift + graded * (1.0f - lift);
    return glm::clamp(graded, 0.0f, 1.0f);
}

int main(int argc, char** argv)
{
    // argv: width height altitude quality style sunAz sunEl radius lat lon time
    const int width = std::atoi(argv[1]);
    const int height = std::atoi(argv[2]);
    const float altitude = static_cast<float>(std::atof(argv[3]));
    const float quality = static_cast<float>(std::atof(argv[4]));
    const int style = std::atoi(argv[5]);
    const float sunAzimuth = static_cast<float>(std::atof(argv[6]));
    const float sunElevation = static_cast<float>(std::atof(argv[7]));
    const float radius = static_cast<float>(std::atof(argv[8]));
    const float latitude = static_cast<float>(std::atof(argv[9]));
    const float longitude = static_cast<float>(std::atof(argv[10]));
    const float time = (argc > 11) ? static_cast<float>(std::atof(argv[11])) : 0.0f;
    // Tilt off nadir, toward the sun's azimuth: looking straight down can
    // never show the sun glitter (its mirror point is tens of degrees away)
    // nor the limb. This is the shot the reference frame is.
    const float tilt = (argc > 12) ? static_cast<float>(std::atof(argv[12])) : 0.0f;

    // Camera on the local vertical of (latitude, longitude), looking down.
    const vec3 centre = glm::normalize(vec3(std::cos(latitude) * std::cos(longitude),
                                            std::sin(latitude),
                                            std::cos(latitude) * std::sin(longitude)));
    const vec3 eye = centre * (radius + altitude);
    const vec3 east = glm::normalize(glm::cross(
        (std::fabs(centre.y) < 0.95f) ? vec3(0.0f, 1.0f, 0.0f) : vec3(1.0f, 0.0f, 0.0f),
        centre));
    const vec3 north = glm::cross(centre, east);
    // Sun azimuth is measured in the local (east, north) frame.
    const vec3 azimuthDir = east * std::cos(sunAzimuth) + north * std::sin(sunAzimuth);
    const vec3 forward =
        glm::normalize(-centre * std::cos(tilt) + azimuthDir * std::sin(tilt));
    // At zero tilt the view IS the local vertical and cross(forward, centre)
    // vanishes — pick any tangent then, or the whole frame comes out NaN.
    vec3 rightRaw = glm::cross(forward, centre);
    if (glm::length(rightRaw) < 1.0e-4f)
    {
        rightRaw = glm::cross(forward, north);
    }
    const vec3 right = glm::normalize(rightRaw);
    const vec3 up = glm::cross(right, forward);

    const float fov = 60.0f * 3.14159265f / 180.0f;
    const float tanHalf = std::tan(fov * 0.5f);
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    const float pixelAngle = 2.0f * tanHalf / static_cast<float>(height);

    // Sun direction, given as an elevation above the LOCAL horizon and an
    // azimuth around it: a low elevation is exactly the raking light that
    // makes relief read (and the one the reference frame uses).
    const vec3 sunDirBody = glm::normalize(centre * std::sin(sunElevation) +
                                           azimuthDir * std::cos(sunElevation));

    const vec3 sunColor = vec3(1.0f, 0.96f, 0.9f);
    const vec3 ambient = vec3(0.020f, 0.024f, 0.035f);
    const float diffuseWrap = 0.22f;

    std::vector<unsigned char> pixels(static_cast<size_t>(width) * height * 3, 0);
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const float ndcX = (2.0f * (x + 0.5f) / width - 1.0f) * tanHalf * aspect;
            const float ndcY = (1.0f - 2.0f * (y + 0.5f) / height) * tanHalf;
            const vec3 rayDir = glm::normalize(forward + right * ndcX + up * ndcY);

            // Ray / sphere intersection (the sea-level sphere).
            const float b = glm::dot(eye, rayDir);
            const float c = glm::dot(eye, eye) - radius * radius;
            const float disc = b * b - c;
            vec3 color = vec3(0.004f, 0.005f, 0.010f); // space
            float hitDistance = 0.0f;
            if (disc > 0.0f)
            {
                const float t = -b - std::sqrt(disc);
                if (t > 0.0f)
                {
                    hitDistance = t;
                    const vec3 hit = eye + rayDir * t;
                    const vec3 dirBody = glm::normalize(hit);
                    // Screen footprint on the surface, in body radians: the
                    // pixel cone widens with distance and with grazing
                    // incidence — the same quantity fwidth() gives the GPU.
                    const float cosIncidence =
                        std::max(0.05f, std::fabs(glm::dot(dirBody, -rayDir)));
                    const float footprint = pixelAngle * t / (radius * cosIncidence);

                    vec3 albedo;
                    float specular = 0.0f, gloss = 0.0f, selfShadow = 1.0f;
                    float water = 0.0f;
                    vec3 normalBody;
                    glsl::planetShading(style, dirBody, quality, radius, sunDirBody,
                                        footprint, time, albedo, specular, gloss,
                                        normalBody, selfShadow, water);

                    // Mirrors the gas-giant limb darkening in Mesh.frag.
                    if (style >= 20)
                    {
                        const float facing =
                            std::max(glm::dot(dirBody, -rayDir), 0.0f);
                        albedo *= 0.35f + 0.65f * std::pow(facing, 0.45f);
                    }
                    const float wrapped =
                        (glm::dot(normalBody, sunDirBody) + diffuseWrap) /
                        (1.0f + diffuseWrap);
                    const float lambert = std::max(wrapped, 0.0f);
                    vec3 lit = albedo * (ambient + sunColor * (lambert * selfShadow));
                    if (specular > 0.001f && lambert > 0.0f)
                    {
                        const vec3 halfway = glm::normalize(sunDirBody - rayDir);
                        const float exponent = glm::mix(12.0f, 260.0f, gloss);
                        const float highlight = std::pow(
                            std::max(glm::dot(normalBody, halfway), 0.0f), exponent);
                        const float grazing =
                            1.0f - std::max(glm::dot(normalBody, -rayDir), 0.0f);
                        const float fresnel =
                            (water > 0.5f)
                                ? 0.02f + 0.98f * std::pow(grazing, 5.0f)
                                : (0.35f + 0.65f * std::pow(grazing, 3.0f)) * specular;
                        const float energy =
                            (water > 0.5f) ? (exponent + 8.0f) / 25.13f : 1.0f;
                        lit += sunColor * (highlight * fresnel * energy * selfShadow);
                    }
                    color = lit;
                }
            }

            // ---- cloud deck (M28): the shell, in front of everything ------
            // Mirrors the Mesh.frag cloud branch: same coverage function,
            // same thickness sample, same silver lining, alpha-blended over
            // whatever the ground path produced.
            if (style == 0)
            {
                const float shellRadius = radius * glsl::kCloudShellRatio;
                const float sb = glm::dot(eye, rayDir);
                const float sc = glm::dot(eye, eye) - shellRadius * shellRadius;
                const float sdisc = sb * sb - sc;
                if (sdisc > 0.0f)
                {
                    const float st = -sb - std::sqrt(sdisc);
                    if (st > 0.0f)
                    {
                        const vec3 dirBody = glm::normalize(eye + rayDir * st);
                        const float cosIncidence = std::max(
                            0.05f, std::fabs(glm::dot(dirBody, -rayDir)));
                        const float footprint =
                            pixelAngle * st / (shellRadius * cosIncidence);
                        const float octaves = glm::clamp(
                            std::log2(1.0f / std::max(footprint * 40.0f, 1.0e-6f)),
                            3.0f, 6.0f);
                        float cumulus = 0.0f, cirrus = 0.0f;
                        const float coverage = glsl::cloudCoverage(
                            dirBody, time, octaves, cumulus, cirrus);
                        if (coverage >= 0.003f)
                        {
                            const float sunUp = glm::dot(sunDirBody, dirBody);
                            const vec3 tangentSun =
                                glm::normalize(sunDirBody - dirBody * sunUp);
                            const float ahead = glsl::cloudCumulus(
                                glm::normalize(dirBody + tangentSun * 0.0022f),
                                time, octaves);
                            const float core =
                                glm::clamp(cumulus * 0.55f + ahead * 0.75f, 0.0f, 1.0f);
                            const float dayWrap =
                                glm::clamp(sunUp * 0.75f + 0.32f, 0.0f, 1.0f);
                            vec3 cloudColor =
                                glm::mix(vec3(0.36f, 0.40f, 0.50f),
                                         vec3(1.02f, 1.00f, 0.98f),
                                         dayWrap * (1.0f - 0.65f * core));
                            const float forward =
                                glm::clamp(glm::dot(-(-rayDir), sunDirBody), 0.0f, 1.0f);
                            const float phase = std::pow(forward, 8.0f);
                            const float thinEdge = (1.0f - cumulus) * cumulus * 4.0f;
                            cloudColor += vec3(1.0f, 0.96f, 0.88f) *
                                          (phase * thinEdge * 1.6f * dayWrap);
                            cloudColor += ambient;
                            const float alpha = glm::clamp(
                                coverage * (0.20f + 0.80f * dayWrap) + cirrus * 0.25f,
                                0.0f, 1.0f);
                            color = glm::mix(color, cloudColor, alpha);
                        }
                    }
                }
            }
            // ---- atmosphere (M29): the same integral the shader runs ------
            // On the ground it is aerial perspective (extinction + in-scatter
            // between the camera and the fragment); where the ray misses the
            // planet it IS the sky, or the limb seen from space.
            {
                glsl::AtmosphereParams air = glsl::atmospherePreset(style);
                const vec3 centre = -eye; // camera-relative planet centre
                vec3 transmittance = vec3(1.0f);
                const float maxDistance = hitDistance > 0.0f ? hitDistance : 1.0e12f;
                const vec3 scattered = glsl::atmosphereScatter(
                    rayDir, maxDistance, centre, radius, sunDirBody, air, 8,
                    transmittance);
                color = color * transmittance + scattered * sunColor;
            }

            // The engine renders into an sRGB swapchain: the shader writes
            // LINEAR values and the presentation engine encodes them. The
            // preview has to do that last step itself or every capture comes
            // out about twice too dark.
            const vec3 linear = gradeCinematic(color);
            const vec3 graded = vec3(
                linear.x <= 0.0031308f ? linear.x * 12.92f
                                       : 1.055f * std::pow(linear.x, 1.0f / 2.4f) - 0.055f,
                linear.y <= 0.0031308f ? linear.y * 12.92f
                                       : 1.055f * std::pow(linear.y, 1.0f / 2.4f) - 0.055f,
                linear.z <= 0.0031308f ? linear.z * 12.92f
                                       : 1.055f * std::pow(linear.z, 1.0f / 2.4f) - 0.055f);
            const size_t index = (static_cast<size_t>(y) * width + x) * 3;
            pixels[index + 0] = static_cast<unsigned char>(graded.x * 255.0f + 0.5f);
            pixels[index + 1] = static_cast<unsigned char>(graded.y * 255.0f + 0.5f);
            pixels[index + 2] = static_cast<unsigned char>(graded.z * 255.0f + 0.5f);
        }
    }
    std::fwrite(pixels.data(), 1, pixels.size(), stdout);
    return 0;
}
"""


def write_png(path, width, height, rgb):
    """Minimal PNG writer (no third-party dependency)."""
    raw = bytearray()
    stride = width * 3
    for y in range(height):
        raw.append(0)
        raw.extend(rgb[y * stride:(y + 1) * stride])

    def chunk(tag, data):
        payload = tag + data
        return (struct.pack(">I", len(data)) + payload +
                struct.pack(">I", zlib.crc32(payload) & 0xFFFFFFFF))

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) +
           chunk(b"IDAT", zlib.compress(bytes(raw), 9)) + chunk(b"IEND", b""))
    with open(path, "wb") as handle:
        handle.write(png)


# name, style, altitude m, quality, sun azimuth, sun elevation, lat, lon
# name, style, altitude m, quality, sun azimuth, sun elevation, lat, lon, tilt
SHOTS = [
    ("terra-orbit-high",       0, 4.0e5, 2.0, 0.6, 0.55, -0.50, 2.03, 0.0),
    ("terra-orbit-medium",     0, 4.0e5, 1.0, 0.6, 0.55, -0.50, 2.03, 0.0),
    ("terra-orbit-flat",       0, 4.0e5, 0.0, 0.6, 0.55, -0.50, 2.03, 0.0),
    ("terra-ranges-raking",    0, 1.5e5, 2.0, 0.4, 0.16, -0.50, 2.03, 0.0),
    ("terra-ranges-noshadow",  0, 1.5e5, 1.0, 0.4, 0.16, -0.50, 2.03, 0.0),
    ("terra-coast",            0, 2.5e5, 2.0, 0.9, 0.45, -0.35, 3.96, 0.0),
    # Tilt toward the sun's azimuth by (90 deg - sun elevation): that is
    # where a mirror sea sends the sun back to the camera.
    ("terra-ocean-glint",      0, 3.0e5, 2.0, 0.0, 0.75, -0.20, 2.35, 0.82),
    ("terra-surf",             0, 2.0e4, 2.0, 0.5, 0.60, -0.340, 2.061, 0.35),
    ("terra-polar",            0, 3.0e5, 2.0, 0.7, 0.25, -1.20, 4.59, 0.0),
    ("terra-limb",             0, 5.0e5, 2.0, 0.0, 0.35, -0.50, 2.03, 1.32),
    ("terra-sunset-ground",    0, 2.0e3, 2.0, 0.0, 0.05, -0.50, 2.03, 1.50),
    # Reproduces the reported frame: 400 km, sun high behind the camera,
    # looking down at open water — the worst case for in-scattered haze.
    ("terra-orbit-ocean",      0, 4.0e5, 2.0, 0.0, 1.30, -0.20, 2.35, 0.0),
    # The night side and the terminator: the case that exposed the airmass
    # bug (a sun below the horizon read as "no air in the way" and lit the
    # dark hemisphere like noon).
    ("terra-night",            0, 4.0e5, 2.0, 0.0, -0.25, -0.50, 2.03, 0.0),
    ("terra-terminator",       0, 6.0e6, 2.0, 0.0, 0.02, -0.50, 2.03, 0.0),
    # F20 — THE AURORA. The oval rings the MAGNETIC axis (0.1908, 0.9816, 0),
    # which puts its pole at latitude 1.3788, longitude 0, and the ring itself
    # 18.5 degrees off it. All three shots need the sun UNDER the horizon:
    # above -8 degrees the shader correctly draws nothing.
    #   polar  the whole oval from above, the shape check
    #   orbit  low pass over the ring, the structure check
    #   limb   from the side and below, the one that proves the curtains
    #          stand UP instead of lying on the ground like paint
    ("terra-aurora-polar",     0, 8.0e6, 2.0, 0.0, -0.60,  1.3788, 0.0,  0.0),
    ("terra-aurora-orbit",     0, 1.5e6, 2.0, -1.5708, -0.90, 1.3000, 0.0, 0.85),
    ("terra-aurora-limb",      0, 4.0e5, 2.0, 1.5708, -0.90, 0.6000, 0.0, 1.32),
    # Landing scale: 900 m up, looking toward the horizon. This is the shot
    # that exposed a mountain summit rendering as a perfectly level mesa.
    ("terra-landing",          0, 9.0e2, 2.0, 0.0, 0.45, -0.646, 2.007, 1.15),
    ("terra-landing-plain",    0, 6.0e2, 2.0, 0.0, 0.25, -0.20, 4.10, 1.20),
    ("terra-far",              0, 6.0e6, 2.0, 0.6, 0.55, -0.50, 2.03, 0.0),
    ("luna-orbit",             1, 1.2e5, 2.0, 0.5, 0.20, 0.30, 1.20, 0.0),
    ("mars-orbit",             2, 2.5e5, 2.0, 0.5, 0.25, -0.20, 0.60, 0.0),
    # The solar system (F14): one orbit shot per new world.
    ("mercury-orbit",          3, 2.0e5, 2.0, 0.5, 0.30, 0.10, 1.10, 0.0),
    ("io-orbit",               4, 1.5e5, 2.0, 0.5, 0.35, -0.20, 0.80, 0.0),
    ("europa-orbit",           5, 1.0e5, 2.0, 0.5, 0.30, 0.15, 2.10, 0.0),
    ("ganymede-orbit",         6, 2.0e5, 2.0, 0.5, 0.30, -0.10, 1.40, 0.0),
    ("callisto-orbit",         7, 2.0e5, 2.0, 0.5, 0.30, 0.25, 3.10, 0.0),
    ("titan-orbit",            8, 1.5e5, 2.0, 0.5, 0.30, -0.15, 0.50, 0.0),
    ("enceladus-orbit",        9, 4.0e4, 2.0, 0.5, 0.30, -0.60, 1.90, 0.0),
    ("rhea-orbit",            10, 8.0e4, 2.0, 0.5, 0.30, 0.20, 2.60, 0.0),
    ("titania-orbit",         11, 8.0e4, 2.0, 0.5, 0.30, -0.30, 0.90, 0.0),
    ("oberon-orbit",          12, 8.0e4, 2.0, 0.5, 0.30, 0.10, 4.20, 0.0),
    ("triton-orbit",          13, 1.0e5, 2.0, 0.5, 0.30, -0.40, 2.90, 0.0),
    ("jupiter-approach",      20, 3.0e7, 2.0, 0.5, 0.40, -0.20, 5.10, 0.0),
    ("saturn-approach",       21, 2.5e7, 2.0, 0.5, 0.40, 0.15, 1.70, 0.0),
    ("uranus-approach",       22, 1.2e7, 2.0, 0.5, 0.40, 0.30, 0.30, 0.0),
    ("neptune-approach",      23, 1.2e7, 2.0, 0.5, 0.40, -0.25, 3.60, 0.0),
    ("venus-approach",        24, 3.0e6, 2.0, 0.5, 0.40, 0.10, 2.40, 0.0),
    # THE REPORTED FRAME: 10 357 km over Saturn, which is where the banding
    # has to hold up and where the lattice artefact showed itself.
    ("saturn-24r",             21, 8.15e7, 2.0, 0.6, 0.55, 0.0, 0.0, 0.0),
    ("saturn-close",          21, 1.0357e7, 2.0, 0.5, 0.40, 0.15, 1.70, 0.0),
    ("jupiter-close",         20, 1.2e7, 2.0, 0.5, 0.40, -0.20, 5.10, 0.0),
    ("uranus-close",          22, 4.0e6, 2.0, 0.5, 0.40, 0.30, 0.30, 0.0),
    ("neptune-close",         23, 4.0e6, 2.0, 0.5, 0.40, -0.25, 3.60, 0.0),
    # ...and the same three with the sun high, so the banding is judged on
    # the material rather than on a grazing terminator.
    ("saturn-noon",           21, 1.0357e7, 2.0, 0.5, 1.20, 0.15, 1.70, 0.0),
    ("jupiter-noon",          20, 1.2e7, 2.0, 0.5, 1.20, -0.20, 5.10, 0.0),
    ("neptune-noon",          23, 4.0e6, 2.0, 0.5, 1.20, -0.25, 3.60, 0.0),
    # THE WHOLE DISC. Everything above frames a piece of a planet; these
    # frame the planet, which is the view the game spends most of its time
    # in and the one where a limb, a terminator or a polar cap either works
    # or does not.
    ("saturn-full",           21, 2.0e8, 2.0, 0.5, 0.90, 0.15, 1.70, 0.0),
    ("saturn-distant",        21, 8.0e8, 2.0, 0.5, 0.90, 0.15, 1.70, 0.0),
    ("jupiter-full",          20, 2.4e8, 2.0, 0.5, 0.90, -0.20, 5.10, 0.0),
    ("uranus-full",           22, 9.0e7, 2.0, 0.5, 0.90, 0.30, 0.30, 0.0),
    ("neptune-full",          23, 9.0e7, 2.0, 0.5, 0.90, -0.25, 3.60, 0.0),
    ("terra-full",             0, 2.2e7, 2.0, 0.6, 0.90, -0.50, 2.03, 0.0),
    # THE ENDURANCE'S OWN VIEW: it parks at a = 4.0e8 m from Saturn's centre,
    # which is 341 800 km of altitude over a 58 232 km planet. Reported as
    # showing a display bug at exactly this range.
    ("saturn-endurance",      21, 3.418e8, 2.0, 0.5, 0.90, 0.15, 1.70, 0.0),
    ("saturn-endurance-lo",   21, 3.418e8, 0.0, 0.5, 0.90, 0.15, 1.70, 0.0),
    ("mars-full",              2, 1.2e7, 2.0, 0.5, 0.90, -0.20, 0.60, 0.0),
]

RADII = {0: 6.371e6, 1: 1.7374e6, 2: 3.3895e6,
         3: 2.4397e6, 4: 1.8216e6, 5: 1.5608e6, 6: 2.6341e6, 7: 2.4103e6,
         8: 2.5747e6, 9: 2.521e5, 10: 7.638e5, 11: 7.884e5, 12: 7.614e5,
         13: 1.3534e6,
         20: 6.9911e7, 21: 5.8232e7, 22: 2.5362e7, 23: 2.4622e7, 24: 6.0518e6}


def main() -> int:
    parser = argparse.ArgumentParser(description="CPU preview of the planet shader")
    parser.add_argument("--glm", required=True)
    parser.add_argument("--out", default="captures")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=640)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--only", default="", help="render a single shot by name")
    args = parser.parse_args()

    program = transpile(["Noise.glsl", "Terrain.glsl", "Clouds.glsl",
                         "PlanetSurface.glsl", "Atmosphere.glsl"]) + HARNESS
    workdir = tempfile.mkdtemp(prefix="sw_preview_")
    cpp = os.path.join(workdir, "preview.cpp")
    # ".exe" on Windows, where g++ appends it regardless of -o; without
    # it the harness compiles and then cannot find what it just built.
    exe = os.path.join(workdir, "preview" + (".exe" if os.name == "nt" else ""))
    with open(cpp, "w", encoding="utf-8") as handle:
        handle.write(program)
    build = subprocess.run(
        [args.compiler, "-std=c++20", "-O2", "-ffp-contract=off",
         "-I", args.glm, cpp, "-o", exe],
        capture_output=True, text=True)
    if build.returncode != 0:
        sys.stderr.write(build.stdout + build.stderr)
        sys.stderr.write(f"\nGenerated source kept at {cpp}\n")
        return 2

    os.makedirs(args.out, exist_ok=True)
    for (name, style, altitude, quality, azimuth, elevation, latitude, longitude,
         tilt) in SHOTS:
        if args.only and args.only != name:
            continue
        run = subprocess.run(
            [exe, str(args.width), str(args.height), str(altitude), str(quality),
             str(style), str(azimuth), str(elevation), str(RADII[style]),
             str(latitude), str(longitude), "120.0", str(tilt)],
            capture_output=True)
        if run.returncode != 0:
            sys.stderr.write(run.stderr.decode("utf-8", "replace"))
            return 3
        path = os.path.join(args.out, name + ".png")
        write_png(path, args.width, args.height, run.stdout)
        print(f"{path}  ({style=} altitude={altitude:.0f} m quality={quality})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
