#pragma once

// ============================================================================
// Planet/Terrain.hpp
// Procedural planetary terrain as an ANALYTIC HEIGHTFIELD (header-only).
//
// The elevation is a pure function of (body-fixed direction, parameters):
// no stored grid, no streaming state. That single property is what makes
// terrain collision exact and cheap — physics samples the very same
// function the renderer displaces its patches with, at any point, at any
// resolution, and a landed base sits on the same mountain after a save,
// a warp, or a year of drift.
//
// Directions are in the BODY'S ROTATING FRAME (the terrain spins with the
// planet). Elevation is meters ABOVE the sea-level sphere (bodyRadius);
// oceans clamp to 0 for everything that touches the surface — the sphere
// itself is the water surface — while `terrainElevationSigned` exposes the
// real NEGATIVE sea floor the renderer needs to color deep water.
//
// ---------------------------------------------------------------------------
// M25 — HEIGHTFIELD v2: the same world, finally shaped like a world.
//
// v1 was one 5-octave fBm: round continents, round hills, no crest, no
// valley, nothing an oblique sun could carve. v2 keeps that fBm EXACTLY as
// it was — it is the continental mask, so every coastline, the world map
// and every site already placed on it are bit-for-bit preserved — and adds
// relief ON TOP of it:
//
//   continental mask (v1 fbm, untouched)  -> where is land, where is sea
//   orogeny belt (low-frequency fbm)      -> where mountain RANGES run
//   domain warp (1 pass, 2 octaves)       -> kills the value-noise grid look
//   ridged multifractal (warped)          -> crests, spurs, cols
//   billow (warped, low frequency)        -> foothills, basins, dunes
//   erosion curve + terraces              -> deep valleys, benched slopes
//   coastal envelope                      -> flat plains that meet the sea
//                                            at exactly 0 (C0 continuous)
//   bathymetry (ocean side)               -> shelf, abyssal plain, ridges
//
// Cost: ~22 value-noise samples per land point (v1: 5). Collision calls it
// a handful of times per tick; the 65x65 terrain patch rebuild pays it once
// per rebuild (measured well under the frame budget, see docs).
//
// MIRROR CONTRACT: Shaders/Terrain.glsl is the line-by-line GLSL twin of
// this file, and Shaders/Noise.glsl of Math/Noise.hpp. Both are verified by
// Tools/glsl_parity/check_parity.py. What the physics collides with is what
// the player sees — that invariant is the reason this file exists.
// ============================================================================

#include "Core/Types.hpp"
#include "Math/Noise.hpp"

#include <algorithm>
#include <type_traits>

namespace sw::planet
{
    struct TerrainComponent
    {
        // ---- continental mask (v1, UNCHANGED: coastlines are preserved) ----
        u32 seed = 1337;
        i32 octaves = 5;
        f32 frequency = 2.3f;    // fBm frequency on the unit sphere
        f32 amplitude = 9000.0f; // peak elevation scale, meters
        /// fBm value at the coastline; below it the terrain is ocean.
        /// 0 = no ocean (airless worlds).
        f32 seaLevelFraction = 0.50f;

        // ---- M25 relief shaping (all dimensionless, all per body) ----------
        /// Multiplier on `frequency` for the ridged/billow relief layers.
        f32 reliefFrequency = 2.6f;
        /// 16 octaves puts the finest ridge at ~30 m: that is what makes a
        /// LANDING SITE a landscape (gullies, benches, boulder-scale folds)
        /// instead of a smooth slope. Nobody pays for all of them at once —
        /// the shader takes what its pixel footprint can resolve, the patch
        /// mesh what its cell size can represent, and collision the lot
        /// (a handful of samples per tick).
        i32 reliefOctaves = 16;
        /// Share of `amplitude` given to sharp ridged crests...
        f32 ridgeWeight = 1.35f;
        /// ...and to rounded billow lobes (foothills, basins).
        f32 billowWeight = 0.28f;
        /// Relief OUTSIDE the mountain belts. Without it the plains are a
        /// smooth sheet — geologically wrong (there is no such thing as flat
        /// ground at this scale) and visually the flattest thing in the
        /// frame. It reuses the ridged field already computed, so it costs
        /// nothing beyond a multiply.
        f32 plainsWeight = 0.34f;
        /// Domain-warp displacement, in noise-space units.
        f32 warpStrength = 0.35f;
        /// Valley compression: 0 = raw noise, 1 = fully squared profile.
        f32 erosion = 0.55f;
        /// Terrace (bench) strength and count, applied inside mountain belts.
        f32 terraceStrength = 0.12f;
        f32 terraceCount = 7.0f;
        /// Orogeny mask edge: LOWER means mountains cover more of the land.
        f32 beltThreshold = 0.44f;
        /// Abyssal depth in meters (0 for airless/oceanless worlds).
        f32 oceanDepth = 4200.0f;
        /// LANDING-SCALE DETAIL, as a fraction of `amplitude`.
        ///
        /// The octave cascade cannot supply this on its own: with gain 0.55
        /// and lacunarity 2.07 an octave's slope only grows 14% per step, so
        /// by the time the wavelength is down to a few hundred metres its
        /// amplitude is a metre or two. Measured on a mountain site, a 3 km
        /// square came out at 1 degree of average slope — a billiard table
        /// you happen to have landed on at 8 km of altitude. This term is a
        /// separate ridged field at ~800 m wavelength with its own budget,
        /// four times stronger inside the mountain belts than on the plains.
        f32 detailWeight = 0.022f;
        /// Wavelength of that detail, expressed as a frequency on the unit
        /// sphere (7960 ~ 800 m on a body of Terra's size).
        f32 detailFrequency = 7960.0f;

        Vec3 noiseOffset{7.31f, 1.17f, 4.73f}; // decorrelates worlds
    };
    static_assert(std::is_trivially_copyable_v<TerrainComponent>);

    // Fixed decorrelation offsets for the relief layers. Constants (not
    // per-body parameters) so the GLSL mirror has nothing to receive.
    inline constexpr Vec3 kReliefOffset{13.77f, 5.29f, 21.13f};
    inline constexpr Vec3 kBillowOffset{2.19f, 17.63f, 9.41f};
    inline constexpr Vec3 kBeltOffset{31.07f, 23.51f, 3.89f};
    inline constexpr Vec3 kAbyssOffset{41.23f, 7.19f, 29.77f};
    inline constexpr Vec3 kDetailOffset{5.13f, 61.07f, 18.29f};

    inline constexpr Vec3 kCoastOffset{19.87f, 43.31f, 11.59f};

    /// THE CONTINENTS, and why this is not one call to fBm any more (F18).
    ///
    /// A plain fBm thresholded at sea level carries the same structure at
    /// every scale, so the coastline SHREDS: from orbit the planet was
    /// thousands of islands and no continents at all. Geology does not work
    /// that way. The plates are a handful of very large, very low-frequency
    /// things, and everything finer only decorates their edges.
    ///
    /// So there are two fields. A LOW-frequency one of three octaves pushed
    /// toward plateaus and basins by a smoothstep — that is the plate, and
    /// what matters about it is that its gradient at the shoreline is STEEP.
    /// That steepness is what makes the second field behave: finer noise
    /// displaces the coast by tens of kilometres instead of punching holes
    /// through the middle of a continent.
    ///
    /// EVERY CONSTANT BELOW WAS FOUND BY SEARCH, NOT BY EYE. Earth's real
    /// land/sea mask was measured on an equal-area grid and gave three
    /// numbers to aim at — 28.9% land, the largest landmass holding 54.3%
    /// of it, the top five holding 95.8% — and 360 parameter combinations
    /// were scored against them. This one measures 26.6% / 53.1% / 100%:
    /// five continents whose size distribution is Earth's. What it still
    /// lacks is Earth's fringe of small islands, which hold 4% of the land
    /// and are the reason that last figure is 100 rather than 96.
    ///
    /// > seaLevelFraction is land. Nothing else in this file may change the
    /// sign of `landFraction` — that is the coastline contract.
    [[nodiscard]] inline f32 terrainMask(const TerrainComponent& terrain,
                                         const Vec3& unitDirectionBodyFrame)
    {
        const Vec3 p =
            unitDirectionBodyFrame * terrain.frequency + terrain.noiseOffset;
        const f32 plate = math::fbm3(p * 0.55f, 3, terrain.seed);
        const f32 shaped = math::smoothstepf(0.38f, 0.62f, plate);
        const f32 coast =
            math::fbm3(p * 3.4f + kCoastOffset, 4, terrain.seed + 4441u);
        return shaped * 0.78f + coast * 0.22f;
    }

    /// Signed land fraction: > 0 inland, 0 at the shoreline, < 0 at sea.
    [[nodiscard]] inline f32 terrainLandFraction(const TerrainComponent& terrain,
                                                 const Vec3& unitDirectionBodyFrame)
    {
        const f32 mask = terrainMask(terrain, unitDirectionBodyFrame);
        return (mask - terrain.seaLevelFraction) /
               (1.0f - terrain.seaLevelFraction);
    }

    /// The heightfield body, with an EXPLICIT ridged-octave count so callers
    /// that do not need full detail (far LOD globe meshes) can pay less. The
    /// GPU uses the same entry point with a footprint-derived count — that is
    /// the anti-shimmer LOD of M26, and it converges to this exact function
    /// as the camera comes down.
    [[nodiscard]] inline f32 terrainElevationSignedLod(
        const TerrainComponent& terrain, const Vec3& unitDirectionBodyFrame,
        i32 reliefOctaves)
    {
        const Vec3 dir = unitDirectionBodyFrame;
        const f32 land = terrainLandFraction(terrain, dir);

        if (land <= 0.0f)
        {
            // ---- bathymetry: the sea floor ---------------------------------
            const f32 deep = -land; // 0 at the shore -> 1 in the far basin
            const f32 shelf = math::smoothstepf(0.0f, 0.05f, deep);
            const f32 basin = math::smoothstepf(0.03f, 0.40f, deep);
            const Vec3 warped = math::warpDomain3(
                dir * (terrain.frequency * terrain.reliefFrequency * 0.5f) +
                    kAbyssOffset,
                terrain.seed + 911u, terrain.warpStrength);
            const f32 ridge = math::ridged3(warped, 3, terrain.seed + 977u);
            f32 depth = terrain.oceanDepth * (0.22f * shelf + 0.78f * basin);
            depth -= terrain.oceanDepth * 0.45f * ridge * basin;
            return -depth;
        }

        // ---- land ----------------------------------------------------------
        // Coastal envelope: relief fades to exactly 0 at the shoreline, so
        // beaches are flat and the land/water seam has no cliff.
        const f32 coast = math::smoothstepf(0.0f, 0.10f, land);

        // Where the ranges run. A low-frequency fBm thresholded into belts:
        // mountains are a geological EVENT on part of a continent, not a
        // uniform coat of noise.
        const f32 orogeny = math::fbm3(dir * (terrain.frequency * 0.8f) +
                                           kBeltOffset,
                                       3, terrain.seed + 601u);
        const f32 belt = math::smoothstepf(terrain.beltThreshold,
                                           terrain.beltThreshold + 0.22f,
                                           orogeny);

        const Vec3 warped = math::warpDomain3(
            dir * (terrain.frequency * terrain.reliefFrequency) + kReliefOffset,
            terrain.seed + 7u, terrain.warpStrength);
        // Squared: the raw ridged multifractal averages ~0.46, which turns
        // every belt into a high PLATEAU. Squaring keeps the crests (1 stays
        // 1) and drops the mass between them — ranges rising out of plains,
        // which is what a mountain belt actually looks like.
        const f32 rawRidge = math::ridged3(warped, reliefOctaves, terrain.seed + 31u);
        const f32 ridge = rawRidge * rawRidge;
        const f32 billow =
            math::billow3(warped * 0.45f + kBillowOffset, 3, terrain.seed + 53u);

        // Continental plateau (the v1 land^2 shape, at reduced weight) plus
        // the two relief layers.
        f32 h = 0.42f * land * land + terrain.ridgeWeight * belt * ridge +
                terrain.plainsWeight * (1.0f - belt) * ridge +
                terrain.billowWeight * billow * (0.35f + 0.65f * land);

        // Erosion: pull the profile toward its square — valleys deepen and
        // widen, peaks keep their height. (mix(h, h*h, erosion) without a
        // pow() call, so the GLSL twin is exact.)
        h = h + terrain.erosion * (h * h - h);

        // Benches / terraces inside the ranges: a soft staircase, the
        // signature of stratified rock under erosion.
        const f32 scaled = h * terrain.terraceCount;
        const f32 floored = std::floor(scaled);
        const f32 stepped =
            (floored + math::smoothstepf(0.35f, 1.0f, scaled - floored)) /
            terrain.terraceCount;
        h = h + terrain.terraceStrength * belt * (stepped - h);

        // SOFT CEILING, not a clamp. `clamp(h, 0, 1)` pinned every point
        // where the weights overshoot to exactly the amplitude — which is
        // how a 9 km summit came out as a perfectly level mesa, gradient
        // zero, unlandable and unlit. This knee compresses toward 1 without
        // ever reaching it, so a peak stays a peak and keeps its slopes.
        if (h > 0.75f)
        {
            h = 0.75f + (h - 0.75f) / (1.0f + (h - 0.75f) * 4.0f);
        }

        // ---- the ground you actually stand on ---------------------------
        // Applied AFTER the soft ceiling on purpose: this is local ground
        // texture, not part of the macro shape, and running it through the
        // knee would flatten it precisely on the summits where it matters
        // most. Only sampled when the caller asked for enough octaves to
        // resolve it (the patch mesh near the ground, and collision, always
        // do; a globe pixel covering ten kilometres never does), fading in
        // over three octaves so no LOD boundary pops.
        const f32 detailFade =
            math::smoothstepf(11.0f, 13.0f, static_cast<f32>(reliefOctaves));
        if (detailFade > 0.0f && terrain.detailWeight > 0.0f)
        {
            const f32 rough = math::ridged3(
                dir * terrain.detailFrequency + kDetailOffset,
                (reliefOctaves >= 14) ? 4 : 3, terrain.seed + 4111u);
            h += terrain.detailWeight * detailFade * (0.25f + 0.75f * belt) *
                 (rough - 0.30f);
        }

        h = (h < 0.0f) ? 0.0f : h;
        return h * terrain.amplitude * coast;
    }

    /// SIGNED elevation in meters at FULL detail: positive above sea level,
    /// NEGATIVE on the sea floor (continental shelf -> abyssal plain ->
    /// mid-ocean ridges). This is the reference the GPU must converge to.
    [[nodiscard]] inline f32 terrainElevationSigned(
        const TerrainComponent& terrain, const Vec3& unitDirectionBodyFrame)
    {
        return terrainElevationSignedLod(terrain, unitDirectionBodyFrame,
                                         terrain.reliefOctaves);
    }

    /// Elevation (meters above the sea-level sphere) at a unit direction in
    /// the body's rotating frame. Oceans read 0: the sea-level sphere IS the
    /// surface you land on, float on and collide with.
    [[nodiscard]] inline f64 terrainElevation(const TerrainComponent& terrain,
                                              const Vec3& unitDirectionBodyFrame)
    {
        const f32 elevation =
            terrainElevationSigned(terrain, unitDirectionBodyFrame);
        return (elevation > 0.0f) ? static_cast<f64>(elevation) : 0.0;
    }

    // ------------------------------------------------------------------------
    // Body presets — ONE definition, shared by the game (globe colors, patch,
    // collision, site placement) and mirrored in Shaders/Terrain.glsl for the
    // per-fragment path. Style ids match the game's SurfaceStyle enum:
    // 0 = Terra, 1 = Luna, 2 = Mars.
    // ------------------------------------------------------------------------

    /// Terra: ocean world, folded ranges along the continental belts.
    [[nodiscard]] inline TerrainComponent presetTerra()
    {
        TerrainComponent t{};
        t.seed = 1337u;
        t.octaves = 5;
        t.frequency = 2.3f;
        t.amplitude = 9000.0f;
        t.seaLevelFraction = 0.46f; // F18: 26.6% land, Earth is 28.9%
        t.reliefFrequency = 2.6f;
        t.reliefOctaves = 16;
        t.ridgeWeight = 1.35f;
        t.billowWeight = 0.28f;
        t.plainsWeight = 0.34f;
        t.warpStrength = 0.35f;
        t.erosion = 0.55f;
        t.terraceStrength = 0.12f;
        t.terraceCount = 7.0f;
        t.beltThreshold = 0.44f;
        t.oceanDepth = 4200.0f;
        t.detailWeight = 0.022f;
        t.detailFrequency = 7960.0f;
        t.noiseOffset = {7.31f, 1.17f, 4.73f};
        return t;
    }

    /// Luna: airless, no ocean. Billow-dominated (basin rims, crater fields),
    /// weak ridges, no terraces — nothing erodes without air and water.
    [[nodiscard]] inline TerrainComponent presetLuna()
    {
        TerrainComponent t{};
        t.seed = 4242u;
        t.octaves = 4;
        t.frequency = 3.1f;
        t.amplitude = 8000.0f;
        t.seaLevelFraction = 0.0f; // airless: no ocean, terrain everywhere
        t.reliefFrequency = 2.2f;
        t.reliefOctaves = 15;
        t.ridgeWeight = 0.45f;
        t.billowWeight = 0.78f;
        t.plainsWeight = 0.42f;
        t.warpStrength = 0.18f;
        t.erosion = 0.15f;
        t.terraceStrength = 0.0f;
        t.terraceCount = 5.0f;
        t.beltThreshold = 0.38f;
        t.oceanDepth = 0.0f;
        t.detailWeight = 0.030f;   // no erosion to smooth it: regolith and rubble
        t.detailFrequency = 3400.0f;
        t.noiseOffset = {2.9f, 8.1f, 0.4f};
        return t;
    }

    /// Mars: dry, strongly warped — long canyon systems and one enormous
    /// bulge of highlands, sharp rims, deep valleys.
    [[nodiscard]] inline TerrainComponent presetMars()
    {
        TerrainComponent t{};
        t.seed = 900u;
        t.octaves = 5;
        t.frequency = 2.8f;
        t.amplitude = 16000.0f;
        t.seaLevelFraction = 0.0f;
        t.reliefFrequency = 2.0f;
        t.reliefOctaves = 16;
        t.ridgeWeight = 1.15f;
        t.billowWeight = 0.34f;
        t.plainsWeight = 0.46f;
        t.warpStrength = 0.72f; // canyons: the warp IS the geology here
        t.erosion = 0.72f;
        t.terraceStrength = 0.18f;
        t.terraceCount = 9.0f;
        t.beltThreshold = 0.40f;
        t.oceanDepth = 0.0f;
        t.detailWeight = 0.018f;   // amplitude is 16 km here: same metres, less fraction
        t.detailFrequency = 5200.0f;
        t.noiseOffset = {5.5f, 3.3f, 9.9f};
        return t;
    }

    /// The solar system's small worlds (styles 3-13), one parameterised
    /// family. Every number that distinguishes an icy moon from a volcanic
    /// one is an argument, so the GLSL mirror (Shaders/Terrain.glsl) carries
    /// the SAME helper with the SAME arguments and the parity tool has one
    /// function to check instead of eleven.
    [[nodiscard]] inline TerrainComponent presetSmallWorld(
        u32 seed, f32 amplitude, f32 frequency, f32 ridgeWeight, f32 billowWeight,
        f32 plainsWeight, f32 warpStrength, f32 erosion, f32 detailWeight,
        f32 detailFrequency, Vec3 noiseOffset)
    {
        TerrainComponent t{};
        t.seed = seed;
        t.octaves = 4;
        t.frequency = frequency;
        t.amplitude = amplitude;
        t.seaLevelFraction = 0.0f; // airless (or as good as): no ocean
        t.reliefFrequency = 2.3f;
        t.reliefOctaves = 15;
        t.ridgeWeight = ridgeWeight;
        t.billowWeight = billowWeight;
        t.plainsWeight = plainsWeight;
        t.warpStrength = warpStrength;
        t.erosion = erosion;
        t.terraceStrength = 0.0f;
        t.terraceCount = 5.0f;
        t.beltThreshold = 0.40f;
        t.oceanDepth = 0.0f;
        t.detailWeight = detailWeight;
        t.detailFrequency = detailFrequency;
        t.noiseOffset = noiseOffset;
        return t;
    }

    /// Preset by style id — MIRRORED in Shaders/Terrain.glsl, byte for byte.
    /// 0 Terra, 1 Luna, 2 Mars, then the solar system's landable worlds:
    /// 3 Mercury, 4 Io, 5 Europa, 6 Ganymede, 7 Callisto, 8 Titan,
    /// 9 Enceladus, 10 Rhea, 11 Titania, 12 Oberon, 13 Triton.
    /// Gas worlds (style >= 20) have no ground and never reach this.
    [[nodiscard]] inline TerrainComponent terrainPreset(i32 style)
    {
        switch (style)
        {
        case 1: return presetLuna();
        case 2: return presetMars();
        // Mercury: lunar shapes, hotter and smoother — shallow intercrater
        // plains, lobate scarps standing in for the weak ridges.
        case 3:
            return presetSmallWorld(7100u, 7000.0f, 3.0f, 0.55f, 0.72f, 0.42f, 0.20f,
                                    0.20f, 0.028f, 3600.0f, {11.2f, 4.4f, 7.7f});
        // Io: tidal volcanism — real mountains, sharp ridges, deep warp.
        case 4:
            return presetSmallWorld(4400u, 9000.0f, 2.6f, 1.30f, 0.30f, 0.46f, 0.50f,
                                    0.40f, 0.020f, 5000.0f, {3.1f, 9.2f, 5.5f});
        // Europa: the smoothest solid body in the system — a shell of ice
        // over an ocean, its relief measured in hundreds of metres.
        case 5:
            return presetSmallWorld(3550u, 900.0f, 2.2f, 0.25f, 0.55f, 0.50f, 0.25f,
                                    0.10f, 0.040f, 3000.0f, {8.8f, 2.2f, 6.1f});
        // Ganymede: grooved terrain — ridges win over billow lobes.
        case 6:
            return presetSmallWorld(6600u, 2500.0f, 2.7f, 0.85f, 0.45f, 0.44f, 0.30f,
                                    0.25f, 0.026f, 4200.0f, {1.9f, 6.6f, 3.3f});
        // Callisto: the oldest surface there is — billow craters, nothing else.
        case 7:
            return presetSmallWorld(7700u, 3000.0f, 3.1f, 0.35f, 0.85f, 0.40f, 0.15f,
                                    0.12f, 0.030f, 3800.0f, {9.4f, 1.1f, 5.9f});
        // Titan: dune fields under the haze — low, long-wavelength, soft.
        case 8:
            return presetSmallWorld(8800u, 1800.0f, 2.0f, 0.40f, 0.50f, 0.55f, 0.35f,
                                    0.45f, 0.034f, 2600.0f, {4.7f, 7.3f, 2.8f});
        // Enceladus: young bright ice, softened craters, the tiger stripes
        // live in the palette rather than the heightfield.
        case 9:
            return presetSmallWorld(9900u, 1200.0f, 2.5f, 0.45f, 0.60f, 0.46f, 0.22f,
                                    0.18f, 0.038f, 3200.0f, {6.2f, 3.9f, 8.4f});
        case 10: // Rhea
            return presetSmallWorld(10100u, 2200.0f, 3.0f, 0.40f, 0.78f, 0.42f, 0.16f,
                                    0.14f, 0.030f, 3600.0f, {2.4f, 8.9f, 4.6f});
        case 11: // Titania
            return presetSmallWorld(11100u, 2400.0f, 2.8f, 0.55f, 0.65f, 0.44f, 0.22f,
                                    0.20f, 0.028f, 3900.0f, {7.8f, 5.1f, 1.6f});
        case 12: // Oberon
            return presetSmallWorld(12100u, 2600.0f, 2.9f, 0.45f, 0.75f, 0.42f, 0.18f,
                                    0.16f, 0.028f, 3700.0f, {5.3f, 2.7f, 9.1f});
        // Triton: cantaloupe terrain — dense shallow dimples, geyser plains.
        case 13:
            return presetSmallWorld(13100u, 1400.0f, 2.6f, 0.35f, 0.70f, 0.52f, 0.30f,
                                    0.28f, 0.036f, 3000.0f, {8.1f, 6.4f, 3.7f});
        default: return presetTerra();
        }
    }
} // namespace sw::planet
