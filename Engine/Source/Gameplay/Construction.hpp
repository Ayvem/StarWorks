#pragma once

// ============================================================================
// Gameplay/Construction.hpp
// MAY THIS BUILDING STAND HERE?
//
// F2 lets the player put machines on a procedural planet, and everything that
// makes that feel solid rather than arbitrary is in this one question. The
// rules are not invented here: every one of them is a field the .swpart
// already carries, so a machine's siting requirements are edited in Part
// Studio next to its geometry, and the ghost the player sees is refused for
// a reason the file states.
//
//   * ON LAND      — `terrainElevation` clamps at sea level, so an ocean is
//                    not "flat ground at altitude zero", it is water.
//   * FLAT ENOUGH  — `maxSlopeTangent`, measured across the building's OWN
//                    footprint rather than at its centre pixel.
//   * ON THE ORE   — `minOreDensity`, read from the same analytic field the
//                    miner will be paid on. A survey cannot lie and neither
//                    can this.
//   * ROOM FOR IT  — no overlap with what is already built.
//
// It lives in the engine, tested, because it has three callers that must
// agree to the metre: the ghost the player aims with, the placement that
// commits it, and the scene builder that lays down the starting outpost.
// Three implementations of "is this legal" would be three different games.
//
// Directions are UNIT VECTORS in the body's rotating frame — the same frame
// SurfaceAnchorComponent stores, so nothing has to be converted twice.
// ============================================================================

#include "Core/Types.hpp"
#include "Gameplay/Parts.hpp"
#include "Planet/Deposits.hpp"
#include "Planet/Terrain.hpp"

#include <span>
#include <string_view>

namespace sw::build
{
    enum class Verdict : u8
    {
        Ok = 0,
        NoDefinition, // not a building at all
        Underwater,
        TooSteep,
        NotEnoughOre,
        Overlapping,
        OutOfRange, // beyond the player's reach
        NoGround,   // the aiming ray never hit anything
        Count
    };

    [[nodiscard]] inline std::string_view verdictText(Verdict verdict)
    {
        switch (verdict)
        {
        case Verdict::Ok: return "OK";
        case Verdict::NoDefinition: return "NOT A BUILDING";
        case Verdict::Underwater: return "UNDERWATER";
        case Verdict::TooSteep: return "GROUND TOO STEEP";
        case Verdict::NotEnoughOre: return "NOT ENOUGH ORE";
        case Verdict::Overlapping: return "BLOCKED BY A BUILDING";
        case Verdict::OutOfRange: return "OUT OF RANGE";
        case Verdict::NoGround: return "NO GROUND IN SIGHT";
        default: return "?";
        }
    }

    /// What is already standing: a centre direction and the radius its
    /// footprint sweeps on the ground.
    struct Footprint
    {
        Vec3 direction{0.0f, 0.0f, 1.0f}; // body frame, unit
        f32 radiusM = 0.0f;
    };

    /// Radius of a rectangular footprint — the half-diagonal, so a rotated
    /// building can never sneak a corner into its neighbour.
    [[nodiscard]] inline f32 footprintRadius(const parts::BuildingSpec& spec)
    {
        const f32 x = static_cast<f32>(spec.footprintM[0]) * 0.5f;
        const f32 y = static_cast<f32>(spec.footprintM[1]) * 0.5f;
        return std::sqrt(x * x + y * y);
    }

    /// Ground distance, in metres, between two directions on a body.
    ///
    /// Via the CHORD, not the dot product, and in f64. Two buildings fourteen
    /// metres apart on a 6,371 km sphere subtend 2.2e-6 radians: their dot
    /// product is 1 - 2.4e-12, which in f32 rounds to exactly 1.0, and acos
    /// then reports a distance of ZERO. Every building would overlap every
    /// other one, everywhere, and the ghost would never turn green. The
    /// chord keeps its precision all the way down to millimetres.
    [[nodiscard]] inline f64 groundDistance(const Vec3& a, const Vec3& b,
                                            f64 bodyRadiusM)
    {
        const glm::dvec3 u = glm::normalize(glm::dvec3(a));
        const glm::dvec3 v = glm::normalize(glm::dvec3(b));
        const f64 chord = glm::length(u - v);
        return 2.0 * bodyRadiusM * std::asin(std::clamp(chord * 0.5, 0.0, 1.0));
    }

    /// WHERE THE PLAYER IS LOOKING. Marches a ray against the heightfield in
    /// the body's rotating frame and returns the direction of the first hit.
    ///
    /// It is a march rather than a solve because the heightfield is a fBm
    /// stack with no closed-form intersection — but it is a march with a
    /// bisection tail, so the hit lands on the surface to the centimetre
    /// instead of on whatever step happened to cross it.
    ///
    /// `originLocal` is body-frame metres from the centre; `directionLocal`
    /// need not be normalised.
    [[nodiscard]] inline bool raycastTerrain(const planet::TerrainComponent& terrain,
                                             f64 bodyRadiusM,
                                             const WorldVec3& originLocal,
                                             const WorldVec3& directionLocal,
                                             f64 maxDistanceM, WorldVec3& outHitLocal)
    {
        const f64 length = glm::length(directionLocal);
        if (length < 1.0e-12 || maxDistanceM <= 0.0)
        {
            return false;
        }
        const WorldVec3 step = directionLocal / length;

        // Height above the ground under a point — positive outside the body.
        auto clearance = [&](const WorldVec3& point) {
            const f64 radius = glm::length(point);
            if (radius < 1.0)
            {
                return -bodyRadiusM;
            }
            const Vec3 direction = Vec3(point / radius);
            return radius - (bodyRadiusM + planet::terrainElevation(terrain, direction));
        };

        f64 travelled = 0.0;
        f64 previous = clearance(originLocal);
        if (previous <= 0.0)
        {
            outHitLocal = originLocal; // already underground: that IS the hit
            return true;
        }

        for (u32 iteration = 0; iteration < 160 && travelled < maxDistanceM; ++iteration)
        {
            // Advance by a fraction of the current clearance: far from the
            // ground the strides are long, close to it they shrink. The
            // fraction is well under 1 because a heightfield is not a true
            // distance field — a full-clearance step can jump a ridge.
            const f64 advance =
                glm::clamp(previous * 0.45, 0.5, std::max(0.5, maxDistanceM * 0.1));
            const f64 next = std::min(travelled + advance, maxDistanceM);
            const f64 value = clearance(originLocal + step * next);
            if (value <= 0.0)
            {
                // Bisect the bracket [travelled, next] onto the surface.
                f64 low = travelled;
                f64 high = next;
                for (u32 refine = 0; refine < 32; ++refine)
                {
                    const f64 middle = (low + high) * 0.5;
                    if (clearance(originLocal + step * middle) > 0.0)
                    {
                        low = middle;
                    }
                    else
                    {
                        high = middle;
                    }
                }
                outHitLocal = originLocal + step * high;
                return true;
            }
            travelled = next;
            previous = value;
            if (travelled >= maxDistanceM)
            {
                break;
            }
        }
        return false;
    }

    /// The whole rule set, in the order a player wants to hear it: the
    /// disqualifying facts about the GROUND first, then what is in the way.
    ///
    /// `existing` is every footprint already standing on this body; pass an
    /// empty span to skip the overlap test (the scene builder does, because
    /// it lays its own outpost out by hand).
    [[nodiscard]] inline Verdict validatePlacement(
        const planet::TerrainComponent& terrain, const planet::DepositComponent& deposits,
        f64 bodyRadiusM, const parts::PartDefinition& definition, const Vec3& direction,
        std::span<const Footprint> existing)
    {
        if (!parts::isBuilding(definition))
        {
            return Verdict::NoDefinition;
        }
        const parts::BuildingSpec& spec = definition.building;
        const Vec3 unit = glm::normalize(direction);

        if (planet::terrainElevation(terrain, unit) <= 0.0 && terrain.oceanDepth > 0.0)
        {
            return Verdict::Underwater;
        }

        const f32 radius = footprintRadius(spec);
        // Slope across the building's OWN footprint. A 16 m miner cares about
        // 16 m of ground, not about the pixel under its centre.
        const f32 slope = planet::terrainLocalSlope(terrain, unit, bodyRadiusM,
                                                    std::max(radius, 2.0f));
        if (static_cast<f64>(slope) > spec.maxSlopeTangent)
        {
            return Verdict::TooSteep;
        }

        if (spec.minOreDensity > 0.0)
        {
            f32 density = 0.0f;
            (void)planet::bestDeposit(deposits, unit, density);
            if (static_cast<f64>(density) < spec.minOreDensity)
            {
                return Verdict::NotEnoughOre;
            }
        }

        // BELTS ARE EXEMPT, both ways. A conveyor is a LINEAR structure a
        // metre wide, and a footprint circle is simply the wrong shape for
        // one: a row of 2 m segments overlaps itself by that measure, and a
        // belt has to reach a machine's mouth, which is inside the machine's
        // own circle. Excluding them is honest about the model's limits;
        // F6 can do better when belts become real transport. Everything
        // else still has to find room.
        if (spec.category != factory::BuildingCategory::Conveyor)
        {
            for (const Footprint& other : existing)
            {
                if (groundDistance(unit, other.direction, bodyRadiusM) <
                    static_cast<f64>(radius + other.radiusM))
                {
                    return Verdict::Overlapping;
                }
            }
        }
        return Verdict::Ok;
    }
} // namespace sw::build
