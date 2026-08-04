#pragma once

// ============================================================================
// Planet/Deposits.hpp
// Ore deposits as an ANALYTIC FIELD (header-only), exactly like the terrain.
//
// `oreDensity(params, direction, resource)` is a pure function of where you
// are and what you are looking for. No stored map, no spawn tables, no save
// data: a deposit is not an object that exists, it is a property of a place.
// The consequences are the same ones that made the heightfield worth doing:
//
//   * the SCANNER displays the very function the MINER exploits, so a survey
//     cannot lie and a mine cannot be built on a deposit that was not there;
//   * a save stores nothing about deposits, and a world reloaded a year
//     later has its ore in the same rocks;
//   * the GPU can draw the same field one day (survey overlay) with the same
//     bit-exact port discipline Noise.glsl/Terrain.glsl already follow.
//
// Density is in [0,1]: the fraction of the local rock that is worth mining.
// A miner's yield is its nominal rate times the density under its feet, so
// siting a mine well IS the gameplay.
// ============================================================================

#include "Core/Types.hpp"
#include "Math/Noise.hpp"
#include "Planet/Terrain.hpp"
#include "Resources/ResourceTypes.hpp"

#include <algorithm>
#include <type_traits>

namespace sw::planet
{
    /// Per-body geology. Trivially copyable: it rides on the body entity
    /// next to its TerrainComponent, and is saved with it.
    struct DepositComponent
    {
        u32 seed = 0x5EED0Eu;
        /// Frequency of the ore fields on the unit sphere. Higher means
        /// smaller, more scattered deposits.
        f32 frequency = 9.0f;
        /// Fraction of the body that carries metal at all. Raise it for an
        /// ore-rich world, drop it for a barren one.
        f32 metalRichness = 0.55f;
        /// Fraction of the body that carries water ice, before the polar
        /// weighting below is applied.
        f32 iceRichness = 0.35f;
        /// How strongly ice is confined to the poles. 0 = ice everywhere,
        /// 1 = only in the permanently shadowed high latitudes. This single
        /// number is what makes a lunar polar site the obvious first colony.
        f32 icePolarBias = 0.85f;
        /// THE FLOOR UNDER COPPER AND ICE: what the barren rock between the
        /// patches carries anyway.
        ///
        /// Ore in patches is the interesting half of the design and it has a
        /// failure mode that is not interesting at all: land somewhere with no
        /// copper and no ice within reach and the colony cannot start, through
        /// no decision the player made. Copper gates the electronics and ice
        /// gates the fuel and the air, so either gap is a dead end, and the
        /// only cure available to a player who cannot yet build anything is to
        /// abandon the site.
        ///
        /// A tenth of the rock everywhere makes every landing viable and
        /// changes nothing about why you would move: a mine on baseline ground
        /// runs at a TENTH of the rate of one on a real deposit, because yield
        /// is the nominal rate times the density underfoot. Expansion stops
        /// being a requirement and becomes an optimisation, which is where it
        /// belongs.
        ///
        /// Iron has no floor: at 0.55 richness it is already almost everywhere,
        /// and giving the abundant resource a floor too would flatten the map.
        f32 baselineDensity = 0.10f;
    };
    static_assert(std::is_trivially_copyable_v<DepositComponent>);

    inline constexpr Vec3 kIronOffset{12.41f, 3.77f, 51.09f};
    inline constexpr Vec3 kCopperOffset{47.13f, 22.91f, 8.63f};
    inline constexpr Vec3 kIceOffset{31.59f, 44.07f, 17.23f};

    /// Density in [0,1] of `resource` at a unit direction in the body's
    /// rotating frame. Anything the ground does not carry returns 0.
    [[nodiscard]] inline f32 oreDensity(const DepositComponent& deposits,
                                        const Vec3& unitDirectionBodyFrame,
                                        res::Resource resource)
    {
        const Vec3 dir = unitDirectionBodyFrame;

        // Metals: two decorrelated fields, thresholded so ore comes in
        // PATCHES with barren rock between them. The ridged fold is what
        // gives a deposit a core and an edge instead of a soft blob — a
        // survey should have somewhere obvious to put the mine.
        auto metal = [&](const Vec3& offset, u32 seedShift) {
            const f32 field = math::fbm3(dir * deposits.frequency + offset, 4,
                                         deposits.seed + seedShift);
            const f32 edge = 1.0f - deposits.metalRichness;
            const f32 patch = math::smoothstepf(edge, edge + 0.14f, field);
            if (patch <= 0.0f)
            {
                return 0.0f;
            }
            // Grade inside the patch: richer at the core, poorer at the rim.
            const f32 grade = math::ridged3(dir * (deposits.frequency * 2.3f) + offset,
                                            3, deposits.seed + seedShift + 77u);
            return patch * (0.35f + 0.65f * grade * grade);
        };

        switch (resource)
        {
        case res::Resource::IronOre:
            return metal(kIronOffset, 0u);
        case res::Resource::CopperOre:
            // The floor is a MAXIMUM against the field, not an addition to it:
            // a rich patch is exactly as rich as it was, and only the barren
            // rock between patches comes up to the baseline.
            return std::max(metal(kCopperOffset, 911u), deposits.baselineDensity);
        case res::Resource::WaterIce:
        {
            // Ice survives where the sun does not reach: the polar term is a
            // hard geographical fact, not a bonus.
            const f32 latitude = std::abs(dir.y);
            const f32 polar =
                math::smoothstepf(0.55f, 0.93f, latitude) * deposits.icePolarBias +
                (1.0f - deposits.icePolarBias);
            const f32 field =
                math::fbm3(dir * (deposits.frequency * 0.7f) + kIceOffset, 4,
                           deposits.seed + 4242u);
            const f32 edge = 1.0f - deposits.iceRichness;
            const f32 patch = math::smoothstepf(edge, edge + 0.12f, field);
            // Same floor as copper, and it matters more here: the polar term
            // multiplies ice down to nothing across most of a body by design,
            // so without a floor an equatorial landing has no water at all.
            return std::max(patch * polar, deposits.baselineDensity);
        }
        default:
            return 0.0f; // refined goods are made, not dug
        }
    }

    /// The richest resource under a point, and its density. Used by the
    /// survey overlay and by the build validator ("no mine off a deposit").
    [[nodiscard]] inline res::Resource bestDeposit(const DepositComponent& deposits,
                                                   const Vec3& unitDirectionBodyFrame,
                                                   f32& outDensity)
    {
        const res::Resource candidates[] = {res::Resource::IronOre,
                                            res::Resource::CopperOre,
                                            res::Resource::WaterIce};
        res::Resource best = res::Resource::Count;
        outDensity = 0.0f;
        for (const res::Resource candidate : candidates)
        {
            const f32 density = oreDensity(deposits, unitDirectionBodyFrame, candidate);
            if (density > outDensity)
            {
                outDensity = density;
                best = candidate;
            }
        }
        return best;
    }

    // ---- siting helpers ----------------------------------------------------
    // These live here rather than in Terrain.hpp on purpose: Terrain.hpp is
    // the CPU half of a bit-exact CPU/GPU pair (Shaders/Terrain.glsl), and
    // every function in it has a GLSL twin the parity harness checks. Siting
    // is a CPU-only question — nothing on the GPU needs to know where a good
    // building plot is — so it belongs on this side of the line.

    /// Steepest ground slope (as a tangent) across a box of `halfSpanM`
    /// metres either side of `unitDirection`. This is the number a building
    /// footprint actually cares about: a plot can sit on a mountainside and
    /// still be locally flat, and it can sit at 200 m elevation and be a
    /// cliff edge.
    [[nodiscard]] inline f32 terrainLocalSlope(const TerrainComponent& terrain,
                                               const Vec3& unitDirection,
                                               f64 bodyRadiusM, f32 halfSpanM = 60.0f)
    {
        const Vec3 centre = glm::normalize(unitDirection);
        const Vec3 reference = (std::abs(centre.y) < 0.9f) ? Vec3{0.0f, 1.0f, 0.0f}
                                                           : Vec3{1.0f, 0.0f, 0.0f};
        const Vec3 east = glm::normalize(glm::cross(reference, centre));
        const Vec3 north = glm::cross(centre, east);
        const f32 step = halfSpanM / static_cast<f32>(bodyRadiusM);
        const f64 middle = terrainElevation(terrain, centre);

        f32 worst = 0.0f;
        for (i32 iy = -1; iy <= 1; ++iy)
        {
            for (i32 ix = -1; ix <= 1; ++ix)
            {
                if (ix == 0 && iy == 0)
                {
                    continue;
                }
                const Vec3 corner =
                    glm::normalize(centre + east * (static_cast<f32>(ix) * step) +
                                   north * (static_cast<f32>(iy) * step));
                const f64 drop =
                    std::abs(terrainElevation(terrain, corner) - middle);
                worst = std::max(worst,
                                 static_cast<f32>(drop / static_cast<f64>(halfSpanM)));
            }
        }
        return worst;
    }

    /// THE EQUATORIAL SURVEY: the best land-and-ore site ON the equator.
    ///
    /// Latitude is a permanent tax on every mission a base ever flies. An
    /// equatorial pad is handed the planet's full rotational speed for free
    /// (465 m/s on Terra) and needs no plane change to reach the equatorial
    /// orbits everything else in the system uses — a base at 24 degrees pays
    /// both, forever, on every launch. So the starting site is not "the best
    /// ground anywhere", it is "the best ground on the line that costs
    /// nothing to leave from".
    ///
    /// Two passes, both exactly on the equator (`y == 0`, latitude 0.000):
    /// a full sweep of the ring to find a continent with ore, then a fine
    /// sweep to find flat, buildable ground inside it.
    ///
    /// `maxSlopeTangent` is measured across the whole 120 m site footprint,
    /// so it is far stricter than any single building's own limit: the point
    /// is to stand the WHOLE factory on one plateau, not to find six plots
    /// that each happen to pass.
    [[nodiscard]] inline Vec3 surveyEquatorialSite(const TerrainComponent& terrain,
                                                   const DepositComponent& deposits,
                                                   res::Resource resource,
                                                   f64 bodyRadiusM, f32& outDensity,
                                                   f32 maxSlopeTangent = 0.04f,
                                                   f64 minElevation = 40.0)
    {
        constexpr f32 kTwoPi = 6.28318530717958647692f;

        auto equatorial = [](f32 angle) {
            return Vec3{std::sin(angle), 0.0f, std::cos(angle)};
        };
        // `land` is checked before `oreDensity` because the elevation call is
        // the expensive one only when it succeeds — and most of a water world
        // is water.
        auto scoreOf = [&](const Vec3& candidate, bool requireFlat) -> f32 {
            if (terrainElevation(terrain, candidate) <= minElevation)
            {
                return -1.0f;
            }
            if (requireFlat &&
                terrainLocalSlope(terrain, candidate, bodyRadiusM) > maxSlopeTangent)
            {
                return -1.0f;
            }
            return oreDensity(deposits, candidate, resource);
        };

        // ---- pass 1: the whole ring, ~28 km apart ------------------------
        constexpr u32 kRingSamples = 1440;
        f32 bestScore = -1.0f;
        f32 bestAngle = 0.0f;
        for (u32 i = 0; i < kRingSamples; ++i)
        {
            const f32 angle =
                kTwoPi * static_cast<f32>(i) / static_cast<f32>(kRingSamples);
            const f32 score = scoreOf(equatorial(angle), /*requireFlat=*/false);
            if (score > bestScore)
            {
                bestScore = score;
                bestAngle = angle;
            }
        }
        if (bestScore < 0.0f)
        {
            outDensity = 0.0f; // an all-ocean equator: the caller decides
            return Vec3{0.0f, 0.0f, 1.0f};
        }

        // ---- pass 2: +/- 0.6 deg around it, ~350 m apart, flat ground -----
        constexpr i32 kFineSpan = 30;
        constexpr f32 kFineStep = 0.000055f; // rad — about 350 m on Terra
        Vec3 best = equatorial(bestAngle);
        f32 refined = -1.0f;
        for (i32 i = -kFineSpan; i <= kFineSpan; ++i)
        {
            const Vec3 candidate =
                equatorial(bestAngle + static_cast<f32>(i) * kFineStep);
            const f32 score = scoreOf(candidate, /*requireFlat=*/true);
            if (score > refined)
            {
                refined = score;
                best = candidate;
            }
        }
        // No buildable plot in range: keep the coarse pick rather than
        // pretending the survey failed. The build validator still has the
        // last word on any individual footprint.
        outDensity = (refined >= 0.0f) ? refined : bestScore;
        return best;
    }

    /// THE LOCAL SURVEY. Scans a square patch of directions around `centre`
    /// and returns the one carrying the most `resource` on dry, buildable
    /// ground.
    ///
    /// This is deliberately the ONLY siting routine: the scene builder uses
    /// it to found the starting outpost, F2's build cursor will use it to
    /// show the player where to dig, and the tests use it to prove that the
    /// place it picks is actually mineable. A survey that ran different
    /// arithmetic from the mine would be a lie the player could not check.
    ///
    /// `stepRadians` is the angular spacing between samples; `span` samples
    /// are taken either side of centre on both axes.
    [[nodiscard]] inline Vec3 surveySite(const TerrainComponent& terrain,
                                         const DepositComponent& deposits,
                                         const Vec3& centreDirection,
                                         res::Resource resource, f32& outDensity,
                                         i32 span = 8, f32 stepRadians = 0.006f,
                                         f64 minElevation = 40.0)
    {
        const Vec3 centre = glm::normalize(centreDirection);
        // A tangent frame that never degenerates, whatever centre is.
        const Vec3 reference = (std::abs(centre.y) < 0.9f) ? Vec3{0.0f, 1.0f, 0.0f}
                                                           : Vec3{1.0f, 0.0f, 0.0f};
        const Vec3 east = glm::normalize(glm::cross(reference, centre));
        const Vec3 north = glm::cross(centre, east);

        Vec3 best = centre;
        outDensity = -1.0f;
        for (i32 iy = -span; iy <= span; ++iy)
        {
            for (i32 ix = -span; ix <= span; ++ix)
            {
                const Vec3 candidate = glm::normalize(
                    centre + east * (static_cast<f32>(ix) * stepRadians) +
                    north * (static_cast<f32>(iy) * stepRadians));
                // Dry land only: an ocean floor is not a build site.
                if (terrainElevation(terrain, candidate) <= minElevation)
                {
                    continue;
                }
                const f32 density = oreDensity(deposits, candidate, resource);
                if (density > outDensity)
                {
                    outDensity = density;
                    best = candidate;
                }
            }
        }
        if (outDensity < 0.0f)
        {
            outDensity = 0.0f; // nothing but sea in range: caller decides
        }
        return best;
    }

    // ---- per-body presets, mirroring the terrain ones ----------------------

    /// Terra: metal-rich, ice only in the polar caps (there is liquid water
    /// everywhere else, which is a different resource entirely).
    [[nodiscard]] inline DepositComponent depositsTerra()
    {
        DepositComponent d{};
        d.seed = 0x5EED0Eu;
        d.frequency = 9.0f;
        d.metalRichness = 0.55f;
        d.iceRichness = 0.30f;
        d.icePolarBias = 0.90f;
        return d;
    }

    /// Luna: poorer in metal, but the permanently shadowed polar craters
    /// hold the ice that makes propellant — the reason to go there.
    [[nodiscard]] inline DepositComponent depositsLuna()
    {
        DepositComponent d{};
        d.seed = 0xA11CEu;
        d.frequency = 11.0f;
        d.metalRichness = 0.42f;
        d.iceRichness = 0.55f;
        d.icePolarBias = 0.95f;
        return d;
    }

    /// Mars: iron everywhere (the planet is rust), ice at the caps and in
    /// the mid-latitude ground.
    [[nodiscard]] inline DepositComponent depositsMars()
    {
        DepositComponent d{};
        d.seed = 0x3A25u;
        d.frequency = 8.0f;
        d.metalRichness = 0.68f;
        d.iceRichness = 0.45f;
        d.icePolarBias = 0.70f;
        return d;
    }

    /// Preset by style id (0 Terra, 1 Luna, 2 Mars) — same convention as
    /// terrainPreset.
    [[nodiscard]] inline DepositComponent depositPreset(i32 style)
    {
        if (style == 1)
        {
            return depositsLuna();
        }
        if (style == 2)
        {
            return depositsMars();
        }
        return depositsTerra();
    }
} // namespace sw::planet
