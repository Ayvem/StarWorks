// ============================================================================
// Shaders/Terrain.glsl — GLSL TWIN of Engine/Source/Planet/Terrain.hpp.
//
// Same composition, same constants, same presets. The CPU header is the
// source of truth (it is what physics collides with); this is the port the
// fragment shader samples. Tools/glsl_parity/check_parity.py transpiles
// both this file and Noise.glsl to C++ and diffs them against the headers —
// a divergence here means the player would see a mountain the ship flies
// through, so it fails the check.
// ============================================================================
#ifndef SW_TERRAIN_GLSL
#define SW_TERRAIN_GLSL

#include "Noise.glsl"

struct TerrainParams
{
    uint seed;
    int octaves;
    float frequency;
    float amplitude;
    float seaLevelFraction;
    float reliefFrequency;
    int reliefOctaves;
    float ridgeWeight;
    float billowWeight;
    float plainsWeight;
    float warpStrength;
    float erosion;
    float terraceStrength;
    float terraceCount;
    float beltThreshold;
    float oceanDepth;
    float detailWeight;
    float detailFrequency;
    vec3 noiseOffset;
};

// Fixed decorrelation offsets for the relief layers (mirror of the
// kReliefOffset / kBillowOffset / kBeltOffset / kAbyssOffset constants).
const vec3 kReliefOffset = vec3(13.77, 5.29, 21.13);
const vec3 kBillowOffset = vec3(2.19, 17.63, 9.41);
const vec3 kBeltOffset = vec3(31.07, 23.51, 3.89);
const vec3 kAbyssOffset = vec3(41.23, 7.19, 29.77);
const vec3 kDetailOffset = vec3(5.13, 61.07, 18.29);

/// The continental mask: the v1 fBm, unchanged — this is what decides land
/// versus sea, and nothing below may change its sign.
float terrainMask(TerrainParams terrain, vec3 dir)
{
    return fbm3(dir * terrain.frequency + terrain.noiseOffset, terrain.octaves,
                terrain.seed);
}

/// Signed land fraction: > 0 inland, 0 at the shoreline, < 0 at sea.
float terrainLandFraction(TerrainParams terrain, vec3 dir)
{
    float mask = terrainMask(terrain, dir);
    return (mask - terrain.seaLevelFraction) / (1.0 - terrain.seaLevelFraction);
}

/// The heightfield body, with an EXPLICIT ridged-octave count (the CPU twin
/// takes the same argument). M26 feeds it a footprint-derived count: fewer
/// octaves far away — no shimmer — converging to the exact CPU function as
/// the camera comes down.
float terrainElevationSignedLod(TerrainParams terrain, vec3 dir, int reliefOctaves)
{
    float land = terrainLandFraction(terrain, dir);

    if (land <= 0.0)
    {
        // ---- bathymetry: the sea floor ---------------------------------
        float deep = -land;
        float shelf = smoothstepf(0.0, 0.05, deep);
        float basin = smoothstepf(0.03, 0.40, deep);
        vec3 warpedSea = warpDomain3(
            dir * (terrain.frequency * terrain.reliefFrequency * 0.5) +
                kAbyssOffset,
            terrain.seed + 911u, terrain.warpStrength);
        float seaRidge = ridged3(warpedSea, 3, terrain.seed + 977u);
        float depth = terrain.oceanDepth * (0.22 * shelf + 0.78 * basin);
        depth -= terrain.oceanDepth * 0.45 * seaRidge * basin;
        return -depth;
    }

    // ---- land ----------------------------------------------------------
    float coast = smoothstepf(0.0, 0.10, land);

    float orogeny =
        fbm3(dir * (terrain.frequency * 0.8) + kBeltOffset, 3, terrain.seed + 601u);
    float belt = smoothstepf(terrain.beltThreshold,
                             terrain.beltThreshold + 0.22, orogeny);

    vec3 warped = warpDomain3(
        dir * (terrain.frequency * terrain.reliefFrequency) + kReliefOffset,
        terrain.seed + 7u, terrain.warpStrength);
    // Squared: the raw ridged multifractal averages ~0.46, which turns every
    // belt into a high plateau. Squaring keeps the crests and drops the mass
    // between them — ranges rising out of plains.
    float rawRidge = ridged3(warped, reliefOctaves, terrain.seed + 31u);
    float ridge = rawRidge * rawRidge;
    float billow = billow3(warped * 0.45 + kBillowOffset, 3, terrain.seed + 53u);

    float h = 0.42 * land * land + terrain.ridgeWeight * belt * ridge +
              terrain.plainsWeight * (1.0 - belt) * ridge +
              terrain.billowWeight * billow * (0.35 + 0.65 * land);

    // Erosion: mix(h, h*h, erosion) written without a pow() call.
    h = h + terrain.erosion * (h * h - h);

    // Benches / terraces inside the ranges.
    float scaled = h * terrain.terraceCount;
    float floored = floor(scaled);
    float stepped =
        (floored + smoothstepf(0.35, 1.0, scaled - floored)) / terrain.terraceCount;
    h = h + terrain.terraceStrength * belt * (stepped - h);

    // SOFT CEILING, not a clamp — see the CPU twin: a hard clamp turned
    // every overshooting summit into a level mesa with no gradient at all.
    if (h > 0.75)
    {
        h = 0.75 + (h - 0.75) / (1.0 + (h - 0.75) * 4.0);
    }

    // ---- the ground you actually stand on (see the CPU twin) ------------
    float detailFade = smoothstepf(11.0, 13.0, float(reliefOctaves));
    if (detailFade > 0.0 && terrain.detailWeight > 0.0)
    {
        float rough = ridged3(dir * terrain.detailFrequency + kDetailOffset,
                              (reliefOctaves >= 14) ? 4 : 3, terrain.seed + 4111u);
        h += terrain.detailWeight * detailFade * (0.25 + 0.75 * belt) *
             (rough - 0.30);
    }

    h = max(h, 0.0);
    return h * terrain.amplitude * coast;
}

/// SIGNED elevation in meters at FULL detail: positive above sea level,
/// negative on the sea floor (shelf -> abyssal plain -> mid-ocean ridges).
float terrainElevationSigned(TerrainParams terrain, vec3 dir)
{
    return terrainElevationSignedLod(terrain, dir, terrain.reliefOctaves);
}

/// Elevation above the sea-level sphere; oceans read 0 (the sphere IS the
/// water surface — the same clamp physics applies).
float terrainElevation(TerrainParams terrain, vec3 dir)
{
    float elevation = terrainElevationSigned(terrain, dir);
    return (elevation > 0.0) ? elevation : 0.0;
}

// ----------------------------------------------------------------------------
// Body presets — mirror of presetTerra/presetLuna/presetMars.
// Style ids: 0 = Terra, 1 = Luna, 2 = Mars.
// ----------------------------------------------------------------------------
TerrainParams terrainPreset(int style)
{
    TerrainParams t;
    if (style == 1) // Luna
    {
        t.seed = 4242u;
        t.octaves = 4;
        t.frequency = 3.1;
        t.amplitude = 8000.0;
        t.seaLevelFraction = 0.0;
        t.reliefFrequency = 2.2;
        t.reliefOctaves = 15;
        t.ridgeWeight = 0.45;
        t.billowWeight = 0.78;
        t.plainsWeight = 0.42;
        t.warpStrength = 0.18;
        t.erosion = 0.15;
        t.terraceStrength = 0.0;
        t.terraceCount = 5.0;
        t.beltThreshold = 0.38;
        t.oceanDepth = 0.0;
        t.detailWeight = 0.030;
        t.detailFrequency = 3400.0;
        t.noiseOffset = vec3(2.9, 8.1, 0.4);
    }
    else if (style == 2) // Mars
    {
        t.seed = 900u;
        t.octaves = 5;
        t.frequency = 2.8;
        t.amplitude = 16000.0;
        t.seaLevelFraction = 0.0;
        t.reliefFrequency = 2.0;
        t.reliefOctaves = 16;
        t.ridgeWeight = 1.15;
        t.billowWeight = 0.34;
        t.plainsWeight = 0.46;
        t.warpStrength = 0.72;
        t.erosion = 0.72;
        t.terraceStrength = 0.18;
        t.terraceCount = 9.0;
        t.beltThreshold = 0.40;
        t.oceanDepth = 0.0;
        t.detailWeight = 0.018;
        t.detailFrequency = 5200.0;
        t.noiseOffset = vec3(5.5, 3.3, 9.9);
    }
    else // Terra
    {
        t.seed = 1337u;
        t.octaves = 5;
        t.frequency = 2.3;
        t.amplitude = 9000.0;
        t.seaLevelFraction = 0.50;
        t.reliefFrequency = 2.6;
        t.reliefOctaves = 16;
        t.ridgeWeight = 1.35;
        t.billowWeight = 0.28;
        t.plainsWeight = 0.34;
        t.warpStrength = 0.35;
        t.erosion = 0.55;
        t.terraceStrength = 0.12;
        t.terraceCount = 7.0;
        t.beltThreshold = 0.44;
        t.oceanDepth = 4200.0;
        t.detailWeight = 0.022;
        t.detailFrequency = 7960.0;
        t.noiseOffset = vec3(7.31, 1.17, 4.73);
    }
    return t;
}

#endif // SW_TERRAIN_GLSL
