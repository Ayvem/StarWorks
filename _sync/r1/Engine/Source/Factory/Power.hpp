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
// Every function below is pure: no world, no components, no clock. That is
// deliberate — a 14-day night has to be testable in a millisecond.
// ============================================================================

#include "Factory/Recipes.hpp"
#include "Math/Math.hpp"

#include <algorithm>
#include <cmath>
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

    /// THE SUNLIGHT A PANEL RECEIVES OVER A WHOLE TICK, not at the instant
    /// the tick happens to begin.
    ///
    /// `solarFactor` above answers a question about a MOMENT, and the grid
    /// used to ask it once and then bill a whole tick's worth of kilowatts
    /// against the answer. At 1x that is right to a part in a thousand — a
    /// 0.2 s tick is 0.05 degrees of a planet's rotation. Under warp it is a
    /// lie with a factor of infinity in it: the Automation lane hands the
    /// grid one tick standing for eight hours, and eight hours of a 24-hour
    /// world is a third of a rotation. Sample that at local noon and the
    /// base generates eight solid hours at full output, dawn to dusk to dawn
    /// included; sample it at local midnight and the same eight hours
    /// generate nothing and the batteries die. Which of the two you got
    /// depended on where the tick boundary happened to land, so the same
    /// simulated day produced different amounts of electricity depending on
    /// how fast the player had been warping — and a base that survived the
    /// night at 1x could be found flat on arrival at 1000x.
    ///
    /// So the factor is INTEGRATED across the interval instead. The mean of
    /// the instantaneous factor over [0, dt] is exactly the right thing to
    /// bill: energy is the integral of power, so power * dt is only honest
    /// when the power is the average one. And because the mean over a long
    /// interval is the sum of the means over the pieces it splits into, one
    /// bulk tick and N small ticks covering the same span reach the same
    /// answer — which is the property the whole lane system exists for.
    ///
    /// What moves during the interval is the BODY: `angularVelocity` is its
    /// spin as axis * rate (rad/s, world frame), exactly as
    /// `phys::GravitySourceComponent` stores it, and the panel is bolted to
    /// the ground, so the site sweeps around `spinCentre` on that axis. The
    /// star's own motion and the occluders' are deliberately NOT swept: a
    /// year is three orders of magnitude longer than a day, and an eclipse
    /// that begins mid-tick is a rarer and much smaller error than the one
    /// this function exists to remove.
    ///
    /// The integral has no closed form — `solarFactor` clamps at the horizon
    /// and steps to zero in eclipse — so it is a composite MIDPOINT rule
    /// over sub-steps of at most `kSolarSampleAngle` radians of rotation.
    /// Midpoint rather than endpoint because the endpoint rule is what the
    /// bug was: it evaluates the interval at one edge of it.
    ///
    /// A stationary site (no spin, or a body that does not turn) costs
    /// exactly one evaluation and returns exactly what `solarFactor` would.
    ///
    /// MEASURED, for a 180 kW panel on an equatorial Terra site (omega =
    /// 7.29e-5 rad/s) over one 86,400 s span, against the same span walked
    /// in one-second ticks:
    ///
    ///   ONE 86,400 s tick, sampled at local noon      4320.00 kWh (+212.5%)
    ///   ONE 86,400 s tick, sampled at local midnight     0.00 kWh (-100.0%)
    ///   ONE 86,400 s tick, integrated                 1382.22 kWh (-0.0013%)
    ///
    /// Worst case over tick sizes 600 / 3,600 / 14,400 / 28,800 / 86,400 s
    /// crossed with seven start phases spread around the day: 0.0017%
    /// integrated, against 212.5% sampled.
    ///
    /// `kSolarMaxSamples` bounds the COST, not the accuracy: past it the
    /// sub-steps simply get coarser than a degree, and since the residual
    /// then averages out over whole rotations it stays small anyway — a tick
    /// spanning a whole YEAR (3.1536e7 s, 366 rotations resolved by 512
    /// samples) measured 0.026% off the finely stepped answer.
    inline constexpr f64 kSolarSampleAngle = 0.02;  // ~1.15 degrees per sample
    inline constexpr u32 kSolarMaxSamples = 512;

    [[nodiscard]] inline f64 averageSolarFactor(const WorldVec3& siteWorld,
                                                const Vec3& siteUp,
                                                const WorldVec3& starWorld,
                                                std::span<const Occluder> occluders,
                                                const WorldVec3& spinCentre,
                                                const WorldVec3& angularVelocity,
                                                f64 seconds)
    {
        const f64 rate = glm::length(angularVelocity);
        const f64 sweep = rate * std::max(0.0, seconds);
        if (!(sweep > 1.0e-6))
        {
            // Nothing turned (and NaN lands here too): the moment IS the
            // interval.
            return solarFactor(siteWorld, siteUp, starWorld, occluders);
        }

        const WorldVec3 axis = angularVelocity / rate;
        const WorldVec3 arm = siteWorld - spinCentre;
        const WorldVec3 up{siteUp};

        const f64 wanted = std::ceil(sweep / kSolarSampleAngle);
        const u32 samples = static_cast<u32>(
            std::clamp(wanted, 1.0, static_cast<f64>(kSolarMaxSamples)));

        f64 total = 0.0;
        for (u32 i = 0; i < samples; ++i)
        {
            const f64 angle =
                sweep * ((static_cast<f64>(i) + 0.5) / static_cast<f64>(samples));
            const glm::dquat turn = glm::angleAxis(angle, axis);
            total += solarFactor(spinCentre + turn * arm, Vec3(turn * up), starWorld,
                                 occluders);
        }
        return total / static_cast<f64>(samples);
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
