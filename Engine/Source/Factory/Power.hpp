#pragma once

// ============================================================================
// Factory/Power.hpp
// THE GRID: how much electricity a site has, and who gets it.
//
// F3's premise is that energy is the constraint that makes a factory a place
// rather than a spreadsheet. Electrolysis costs 480 kW and it is the only
// road to propellant; solar output is the REAL sun at the REAL local hour;
// and a lunar night is fourteen days long. Those three facts together are
// the entire reason a polar site with ice is worth founding and a site on
// the lunar equator is a trap.
//
// So sunlight here is not a switch. It is the elevation of the actual star
// above the actual local horizon, with the actual other bodies allowed to
// get in the way — the same star the renderer draws and the same geometry
// the shader's eclipse test uses. A site does not "have power at daytime",
// it has the power its panels are physically receiving.
//
// Both functions below are pure: no world, no components, no clock. That is
// deliberate — a 14-day night has to be testable in a millisecond.
// ============================================================================

#include "Factory/Recipes.hpp"
#include "Math/Math.hpp"

#include <algorithm>
#include <span>

namespace sw::factory
{
    /// A body that can get between a site and its star.
    struct Occluder
    {
        WorldVec3 centre{0.0};
        f64 radius = 0.0;
    };

    /// Fraction of full sunlight landing on a flat panel: 1 with the star
    /// overhead, 0 at and below the horizon, 0 in eclipse.
    ///
    /// `siteUp` is the local vertical (unit). Occluders may safely include
    /// the site's OWN body: with the star above the horizon the closest
    /// approach to that body lies behind the panel, so it can never shadow
    /// itself — night is the elevation term's job, and only its job.
    [[nodiscard]] inline f64 solarFactor(const WorldVec3& siteWorld, const Vec3& siteUp,
                                         const WorldVec3& starWorld,
                                         std::span<const Occluder> occluders)
    {
        const WorldVec3 toStar = starWorld - siteWorld;
        const f64 distance = glm::length(toStar);
        if (distance < 1.0)
        {
            return 0.0;
        }
        const WorldVec3 direction = toStar / distance;

        // Lambert on a horizontal panel. Below the horizon there is no
        // sunlight to attenuate, so everything else is skipped.
        const f64 elevation = glm::dot(WorldVec3(glm::normalize(siteUp)), direction);
        if (elevation <= 0.0)
        {
            return 0.0;
        }

        for (const Occluder& occluder : occluders)
        {
            if (occluder.radius <= 0.0)
            {
                continue;
            }
            const WorldVec3 toCentre = occluder.centre - siteWorld;
            const f64 along = glm::dot(toCentre, direction);
            if (along <= 0.0 || along >= distance)
            {
                continue; // behind the panel, or past the star
            }
            const f64 miss = glm::length(toCentre - direction * along);
            if (miss < occluder.radius)
            {
                return 0.0; // eclipsed
            }
        }
        return elevation;
    }

    /// One machine's claim on the grid.
    struct PowerClaim
    {
        f64 demandKw = 0.0;
        /// Lower is served FIRST. A brownout should stop the smelters and
        /// leave the mines digging, not the other way round: ore keeps its
        /// value overnight, a half-melted charge does not.
        u32 priority = 0;
    };

    /// Splits `availableKw` across the claims, filling `outSatisfaction` with
    /// each one's share of its own demand, in [0, 1].
    ///
    /// Strict priority BANDS, proportional inside a band. The band that runs
    /// out is the one that gets a fraction; everything below it gets nothing.
    /// That is what makes a brownout legible — you can see which tier of the
    /// factory the grid stopped at.
    inline void allocatePower(std::span<const PowerClaim> claims, f64 availableKw,
                              std::span<f64> outSatisfaction)
    {
        for (usize i = 0; i < outSatisfaction.size(); ++i)
        {
            outSatisfaction[i] = 0.0;
        }
        if (claims.empty())
        {
            return;
        }

        f64 remaining = std::max(0.0, availableKw);
        // Walk the bands in order. There are a handful of categories, so
        // sorting would cost more than sweeping for the next lowest.
        u32 band = 0;
        bool searching = true;
        while (searching)
        {
            searching = false;
            u32 next = 0;
            bool found = false;
            for (const PowerClaim& claim : claims)
            {
                if (claim.priority >= band && (!found || claim.priority < next))
                {
                    next = claim.priority;
                    found = true;
                }
            }
            if (!found)
            {
                break;
            }
            band = next;

            f64 wanted = 0.0;
            for (const PowerClaim& claim : claims)
            {
                if (claim.priority == band)
                {
                    wanted += std::max(0.0, claim.demandKw);
                }
            }
            const f64 share =
                (wanted <= 1.0e-12) ? 1.0 : std::clamp(remaining / wanted, 0.0, 1.0);
            for (usize i = 0; i < claims.size() && i < outSatisfaction.size(); ++i)
            {
                if (claims[i].priority == band)
                {
                    outSatisfaction[i] = (claims[i].demandKw <= 0.0) ? 1.0 : share;
                }
            }
            remaining = std::max(0.0, remaining - wanted);
            band += 1;
            searching = true;
        }
    }

    /// Default service order. Data today, a per-building field the moment a
    /// player wants their smelter to win.
    [[nodiscard]] inline u32 defaultPowerPriority(BuildingCategory category)
    {
        switch (category)
        {
        case BuildingCategory::Hub:
        case BuildingCategory::Beacon:
            return 0; // keeping the lights on is cheap and keeps you oriented
        case BuildingCategory::Miner:
            return 1; // ore keeps overnight
        case BuildingCategory::Refinery:
            return 2;
        case BuildingCategory::Fabricator:
            return 3;
        default:
            return 4;
        }
    }
} // namespace sw::factory
