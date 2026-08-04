// ============================================================================
// SurveyTests.cpp — the orbital survey's contract (F43).
//
// The survey is the one piece of geology that is STATE rather than a function:
// Deposits.hpp says what is in the ground everywhere and always, and this says
// whether anyone has looked. The overlay draws the first only where the second
// permits, so what these tests have to pin is the second — where a track
// reveals, and, much more importantly, where it does NOT.
//
// Three promises:
//
//   1. THE GRID IS HONEST — every direction lands in a cell, every cell
//      centre lands back in itself, and no cell is unreachable.
//   2. COVERAGE IS A FUNCTION OF THE PATH, not of how often anyone sampled
//      it. This is the property markSurveyArc exists for and the one that
//      broke in flight: at x100 warp the craft crosses eight degrees between
//      frames while the instrument sees six, and a survey that should close
//      in a dozen revolutions read one per cent after nine.
//   3. THE ORBIT YOU CHOOSE IS THE MISSION — a polar orbit closes the map and
//      an equatorial one cannot, ever, no matter how long it flies.
//
// The arming rules (a stable orbit, the instrument switched on) live in
// StarWorksGame and are checked in flight, not here: this binary links the
// engine only, and the engine half is the arithmetic.
// ============================================================================

#include "TestFramework.hpp"

#include <Planet/Deposits.hpp>
#include <Planet/OreRamp.hpp>
#include <Planet/Survey.hpp>
#include <UI/GlobePick.hpp>

#include <algorithm>
#include <cmath>

namespace
{
    using sw::planet::SurveyComponent;

    /// The swath the OS-1 sweeps, mirroring kSwathRadians in GameSurvey.cpp.
    /// Six degrees: a little over one latitude cell.
    constexpr sw::f32 kSwath = 0.105f;

    constexpr sw::f64 kPi = 3.14159265358979323846;

    /// The point under a craft in a circular orbit, in the SPINNING body's
    /// frame — which is the frame the ore field is a function of, and the
    /// reason a satellite in a fixed plane still sweeps out a band.
    sw::Vec3 subSatellite(sw::f64 timeSeconds, sw::f64 inclination,
                          sw::f64 orbitPeriod, sw::f64 dayLength)
    {
        const sw::f64 u = 2.0 * kPi * timeSeconds / orbitPeriod;
        // In-plane basis: P on the equator, Q tilted by the inclination.
        const sw::f64 x = std::cos(u);
        const sw::f64 y = std::sin(u) * std::sin(inclination);
        const sw::f64 z = std::sin(u) * std::cos(inclination);
        // Undo the body's rotation about its axis to land in the body frame.
        const sw::f64 spin = 2.0 * kPi * timeSeconds / dayLength;
        const sw::f64 c = std::cos(-spin);
        const sw::f64 s = std::sin(-spin);
        return glm::normalize(sw::Vec3{static_cast<sw::f32>(x * c + z * s),
                                       static_cast<sw::f32>(y),
                                       static_cast<sw::f32>(-x * s + z * c)});
    }

    struct Flight
    {
        SurveyComponent survey{};
        sw::f32 fraction = 0.0f;
    };

    /// Flies `revolutions` of a circular orbit, sampling every `stepSeconds`.
    /// `paintArcs` picks the shipped behaviour (paint the arc between
    /// samples) or the one it replaced (paint the sample point only).
    Flight fly(sw::f64 inclination, sw::f64 revolutions, sw::f64 stepSeconds,
               bool paintArcs = true)
    {
        constexpr sw::f64 kOrbitPeriod = 5400.0; // ~400 km over Terra
        constexpr sw::f64 kDayLength = 86400.0;
        Flight flight{};
        const sw::f64 duration = revolutions * kOrbitPeriod;
        sw::Vec3 previous = subSatellite(0.0, inclination, kOrbitPeriod, kDayLength);
        sw::planet::markSurveySwath(flight.survey, previous, kSwath);
        for (sw::f64 t = stepSeconds; t <= duration; t += stepSeconds)
        {
            const sw::Vec3 point =
                subSatellite(t, inclination, kOrbitPeriod, kDayLength);
            if (paintArcs)
            {
                sw::planet::markSurveyArc(flight.survey, previous, point, kSwath);
            }
            else
            {
                sw::planet::markSurveySwath(flight.survey, point, kSwath);
            }
            previous = point;
        }
        flight.fraction = sw::planet::surveyFraction(flight.survey);
        return flight;
    }

    /// Deterministic direction sampler — no std::random, so a failure is
    /// reproducible on every machine.
    sw::Vec3 sampleDirection(sw::u32 i)
    {
        sw::u32 s = i * 2654435761u + 12345u;
        s ^= s >> 15;
        s *= 2246822519u;
        s ^= s >> 13;
        const sw::f32 u = static_cast<sw::f32>(s & 0xFFFFFFu) / 16777216.0f;
        s ^= s >> 16;
        s *= 3266489917u;
        s ^= s >> 11;
        const sw::f32 v = static_cast<sw::f32>(s & 0xFFFFFFu) / 16777216.0f;
        const sw::f32 z = 2.0f * u - 1.0f;
        const sw::f32 r = std::sqrt(std::max(0.0f, 1.0f - z * z));
        const sw::f32 phi = 6.28318530718f * v;
        return glm::normalize(sw::Vec3{r * std::cos(phi), z, r * std::sin(phi)});
    }

    sw::u32 cellsIn(const SurveyComponent& survey)
    {
        sw::u32 count = 0;
        for (sw::u32 cell = 0; cell < SurveyComponent::kCells; ++cell)
        {
            count += sw::planet::surveyed(survey, cell) ? 1u : 0u;
        }
        return count;
    }

    /// How many cells of `subset` are missing from `superset`.
    sw::u32 missingFrom(const SurveyComponent& subset, const SurveyComponent& superset)
    {
        sw::u32 missing = 0;
        for (sw::u32 cell = 0; cell < SurveyComponent::kCells; ++cell)
        {
            if (sw::planet::surveyed(subset, cell) &&
                !sw::planet::surveyed(superset, cell))
            {
                ++missing;
            }
        }
        return missing;
    }

    /// The highest |latitude| any surveyed cell reaches, in radians.
    sw::f32 reachedLatitude(const SurveyComponent& survey)
    {
        sw::f32 worst = 0.0f;
        for (sw::u32 cell = 0; cell < SurveyComponent::kCells; ++cell)
        {
            if (!sw::planet::surveyed(survey, cell))
            {
                continue;
            }
            const sw::Vec3 dir = sw::planet::surveyCellDirection(cell);
            worst = std::max(worst, std::asin(std::abs(dir.y)));
        }
        return worst;
    }
} // namespace

// ---------------------------------------------------------------------------

SW_TEST(SurveyCellsTileTheSphereWithoutGapsOrOverlap)
{
    // Every cell centre must land back in its own cell. A grid that rounds a
    // centre into its neighbour would paint the overlay one cell away from
    // the ore it describes, and nothing downstream could tell.
    for (sw::u32 cell = 0; cell < SurveyComponent::kCells; ++cell)
    {
        SW_CHECK(sw::planet::surveyCell(sw::planet::surveyCellDirection(cell)) == cell);
    }

    // And no cell is unreachable from a direction: sweep the sphere finely
    // and confirm every one of the 2048 gets hit.
    SurveyComponent reached{};
    for (sw::u32 iy = 0; iy < 400; ++iy)
    {
        const sw::f32 v = (static_cast<sw::f32>(iy) + 0.5f) / 400.0f;
        const sw::f32 latitude = v * static_cast<sw::f32>(kPi) -
                                 static_cast<sw::f32>(kPi) * 0.5f;
        for (sw::u32 ix = 0; ix < 800; ++ix)
        {
            const sw::f32 u = (static_cast<sw::f32>(ix) + 0.5f) / 800.0f;
            const sw::f32 longitude = u * 2.0f * static_cast<sw::f32>(kPi) -
                                      static_cast<sw::f32>(kPi);
            const sw::f32 ring = std::cos(latitude);
            const sw::Vec3 dir{ring * std::cos(longitude), std::sin(latitude),
                               ring * std::sin(longitude)};
            sw::planet::markSurveyed(reached, sw::planet::surveyCell(dir));
        }
    }
    SW_CHECK(cellsIn(reached) == SurveyComponent::kCells);
    SW_CHECK(sw::planet::surveyFraction(reached) >= 0.999f);

    // A fresh component knows nothing. Obvious, and the thing a save that
    // loaded garbage would break first.
    const SurveyComponent fresh{};
    SW_CHECK(cellsIn(fresh) == 0u);
    SW_CHECK(sw::planet::surveyFraction(fresh) == 0.0f);
}

SW_TEST(CoverageFollowsThePathAndNotTheSampleRate)
{
    // The same polar orbit, flown three ways: sampled every five seconds (a
    // real frame rate), every three hundred (what x1000 warp does to a sixty
    // hertz loop — a eighteenth of a revolution between looks), and every
    // six hundred (twice that again).
    const Flight fine = fly(kPi * 0.5, 6.0, 5.0);
    const Flight coarse = fly(kPi * 0.5, 6.0, 300.0);

    // Same flight, same map — not approximately, EXACTLY: sixty times fewer
    // looks at the same six revolutions and not one cell differs, because
    // what is painted is the arc the craft flew and not the instant anyone
    // happened to look at it.
    SW_CHECK(fine.fraction == coarse.fraction);
    SW_CHECK(missingFrom(fine.survey, coarse.survey) == 0u);
    SW_CHECK(missingFrom(coarse.survey, fine.survey) == 0u);

    // Pushed further it degrades gracefully rather than falling apart: over
    // ten minutes the great circle between two samples cuts the corner off a
    // track that is really a spiral, and a handful of cells at the turns go
    // unpainted. That is honest, and it is under one per cent.
    const Flight extreme = fly(kPi * 0.5, 6.0, 600.0);
    SW_CHECK(std::abs(fine.fraction - extreme.fraction) < 0.02f);
    SW_CHECK(missingFrom(fine.survey, extreme.survey) <
             SurveyComponent::kCells / 100u);

    // And the regression this replaced, kept as the control: painting only
    // the sample points leaves a dotted line, and the same six revolutions
    // reveal less than half the ground. In flight it was far worse than
    // this — nine revolutions at x100 read one per cent — because the frame
    // rate under load is not a constant.
    const Flight dotted = fly(kPi * 0.5, 6.0, 600.0, /*paintArcs=*/false);
    SW_CHECK(dotted.fraction < extreme.fraction * 0.5f);
}

SW_TEST(APolarOrbitClosesTheMapAndAnEquatorialOneNeverCan)
{
    // A polar orbit over a spinning planet is the survey orbit: every
    // revolution comes back over ground the day has turned underneath it.
    const Flight polar = fly(kPi * 0.5, 24.0, 10.0);
    SW_CHECK(polar.fraction > 0.90f);

    // An equatorial one sees a belt and nothing else, however long it flies.
    // Thirty revolutions is a whole planetary day: it closes the belt and
    // gets no closer to the poles than the swath it carries.
    const Flight equatorial = fly(0.0, 30.0, 10.0);
    SW_CHECK(equatorial.fraction < 0.20f);
    SW_CHECK(reachedLatitude(equatorial.survey) < kSwath * 2.0f);

    // Explicitly: the poles stay dark. This is the whole reason the choice
    // of orbit is a decision the player makes rather than a formality.
    SW_CHECK(!sw::planet::surveyed(equatorial.survey, sw::Vec3{0.0f, 1.0f, 0.0f}));
    SW_CHECK(!sw::planet::surveyed(equatorial.survey, sw::Vec3{0.0f, -1.0f, 0.0f}));

    // The belt IS closed, though — an equatorial survey is not a failure,
    // it is a complete survey of one sixth of a planet.
    for (sw::u32 i = 0; i < 64; ++i)
    {
        const sw::f32 longitude =
            2.0f * static_cast<sw::f32>(kPi) * static_cast<sw::f32>(i) / 64.0f;
        const sw::Vec3 dir{std::cos(longitude), 0.0f, std::sin(longitude)};
        SW_CHECK(sw::planet::surveyed(equatorial.survey, dir));
    }
}

SW_TEST(AnUnsurveyedBodyRevealsNothingAndOneSwathRevealsOnlyItsSwath)
{
    // One pass of the instrument reveals a disc a few cells across, and the
    // rest of the planet stays unknown. The overlay reads this bit and
    // nothing else, so this is the gate that makes a survey worth flying.
    SurveyComponent survey{};
    const sw::Vec3 spot = glm::normalize(sw::Vec3{0.3f, 0.2f, 0.9f});
    const sw::u32 revealed = sw::planet::markSurveySwath(survey, spot, kSwath);
    SW_CHECK(revealed > 0u);
    SW_CHECK(revealed < 20u);
    SW_CHECK(sw::planet::surveyed(survey, spot));
    SW_CHECK(!sw::planet::surveyed(survey, -spot));

    // Marking the same spot again costs nothing and reveals nothing: the
    // return value is what a progress readout wants.
    SW_CHECK(sw::planet::markSurveySwath(survey, spot, kSwath) == 0u);

    // A jump too large to be flight — a warp hand-off, a new sphere of
    // influence — is not painted as an arc. It marks where the craft now IS
    // and does not draw a line across half a planet it never crossed.
    SurveyComponent jumped{};
    const sw::Vec3 far = glm::normalize(sw::Vec3{-0.9f, 0.1f, -0.4f});
    sw::planet::markSurveyArc(jumped, spot, far, kSwath);
    SW_CHECK(sw::planet::surveyed(jumped, far));
    SW_CHECK(!sw::planet::surveyed(jumped, spot));
}

SW_TEST(TheBaselineFloorIsAlwaysBelowWhatTheOverlayCallsWorthMoving)
{
    // F43's other half: copper and ice carry a tenth of the rock EVERYWHERE
    // so that no landing site is a dead end. The overlay must still be worth
    // reading, which means the floor has to sit below the display threshold
    // and the real patches well above it — otherwise the map says "ore
    // everywhere", which is true and useless.
    const auto terra = sw::planet::depositsTerra();
    sw::f32 poorest = 1.0f;
    sw::f32 richest = 0.0f;
    for (sw::u32 cell = 0; cell < SurveyComponent::kCells; ++cell)
    {
        const sw::Vec3 dir = sw::planet::surveyCellDirection(cell);
        sw::f32 density = 0.0f;
        const sw::res::Resource best = sw::planet::bestDeposit(terra, dir, density);
        SW_CHECK(best != sw::res::Resource::Count); // nowhere is barren now
        poorest = std::min(poorest, density);
        richest = std::max(richest, density);
    }
    SW_CHECK(poorest >= terra.baselineDensity);
    SW_CHECK(poorest <= terra.baselineDensity + 1.0e-4f);
    SW_CHECK(richest > poorest * 5.0f);

    // And the threshold the map draws at (0.28, kWorthMoving in
    // GameSurvey.cpp) falls between them, so baseline ground is left blank
    // and a real deposit is not.
    constexpr sw::f32 kWorthMoving = 0.28f;
    SW_CHECK(terra.baselineDensity < kWorthMoving);
    SW_CHECK(richest > kWorthMoving);
}

// ---------------------------------------------------------------------------
// F44: the geology screen's two pieces of arithmetic.
//
// The screen itself lives in the game layer and is looked at in captures; what
// is HERE is the part a picture cannot check — a colour ramp that must be
// ordered, and a click on a globe that must land on the hemisphere the player
// was looking at.
// ---------------------------------------------------------------------------

SW_TEST(TheOreRampIsOrderedAndOneHuePerResource)
{
    const sw::res::Resource channels[] = {sw::res::Resource::IronOre,
                                          sw::res::Resource::CopperOre,
                                          sw::res::Resource::WaterIce};
    for (const sw::res::Resource channel : channels)
    {
        // MONOTONE, which is the whole contract of a sequential ramp: more ore
        // is never darker. A rainbow passes every other check and fails this
        // one, which is why this is the test and not a discussion.
        sw::f32 previous = -1.0f;
        for (sw::u32 step = 0; step <= 40; ++step)
        {
            const sw::f32 density = static_cast<sw::f32>(step) / 40.0f;
            const sw::f32 luminance =
                sw::planet::rampLuminance(sw::planet::oreRampColor(channel, density));
            SW_CHECK(luminance > previous);
            previous = luminance;
        }
        // The ends are worth pinning too: the foot has to recede toward a
        // black screen and the head has to be unmistakably bright, or the
        // whole range compresses into "some colour".
        SW_CHECK(sw::planet::rampLuminance(sw::planet::oreRampColor(channel, 0.0f)) <
                 0.15f);
        SW_CHECK(sw::planet::rampLuminance(sw::planet::oreRampColor(channel, 1.0f)) >
                 0.70f);
        // Out of range must not wrap round to the other end of the ramp.
        SW_CHECK(sw::planet::oreRampColor(channel, -3.0f) ==
                 sw::planet::oreRampColor(channel, 0.0f));
        SW_CHECK(sw::planet::oreRampColor(channel, 9.0f) ==
                 sw::planet::oreRampColor(channel, 1.0f));
    }

    // ONE HUE PER RESOURCE. At the same density the three channels must be
    // three visibly different colours — the ramp carries the magnitude, the
    // hue carries which map you are looking at.
    for (const sw::f32 density : {0.35f, 0.6f, 0.85f})
    {
        const sw::Vec3 iron = sw::planet::oreRampColor(sw::res::Resource::IronOre, density);
        const sw::Vec3 copper =
            sw::planet::oreRampColor(sw::res::Resource::CopperOre, density);
        const sw::Vec3 ice = sw::planet::oreRampColor(sw::res::Resource::WaterIce, density);
        SW_CHECK(glm::length(iron - copper) > 0.25f);
        SW_CHECK(glm::length(iron - ice) > 0.25f);
        SW_CHECK(glm::length(copper - ice) > 0.25f);
        // ...and each is what it says: iron reddest, copper greenest, ice bluest.
        SW_CHECK(iron.r > iron.g && iron.r > iron.b);
        SW_CHECK(copper.g > copper.r && copper.g > copper.b);
        SW_CHECK(ice.b > ice.r && ice.b > ice.g);
    }
}

SW_TEST(AClickOnTheGlobeLandsOnTheSideTheCameraCanSee)
{
    // The round trip the beacon depends on: put the camera somewhere, aim a
    // ray at a point on the sphere, and get that point back. A solve that took
    // the far root would return a plausible latitude on the wrong hemisphere,
    // and the mark would simply be somewhere else.
    for (sw::u32 i = 0; i < 200; ++i)
    {
        const sw::f32 yaw = static_cast<sw::f32>(i) * 0.31f;
        const sw::f32 pitch = std::sin(static_cast<sw::f32>(i) * 0.17f) * 1.4f;
        const sw::f32 distance = 1.4f + static_cast<sw::f32>(i % 7) * 0.8f;
        const sw::Vec3 eye = sw::ui::orbitCameraOffset(yaw, pitch, distance);
        // A target somewhere on the visible cap.
        const sw::Vec3 forward = glm::normalize(-eye);
        const sw::Vec3 reference =
            (std::abs(forward.y) < 0.9f) ? sw::Vec3{0.0f, 1.0f, 0.0f} : sw::Vec3{1.0f, 0.0f, 0.0f};
        const sw::Vec3 right = glm::normalize(glm::cross(forward, reference));
        const sw::Vec3 up = glm::cross(right, forward);
        const sw::f32 tilt = 0.5f * std::cos(static_cast<sw::f32>(i));
        const sw::Vec3 target = glm::normalize(-forward + right * tilt +
                                               up * (0.4f * std::sin(static_cast<sw::f32>(i))));
        sw::Vec3 picked{};
        SW_CHECK(sw::ui::pickUnitSphere(eye, target - eye, picked));
        SW_CHECK(glm::length(picked - target) < 1.0e-3f);
        // And the pick is on the near side, always.
        SW_CHECK(glm::dot(picked, glm::normalize(eye)) > 0.0f);
    }

    // A ray beside the globe hits nothing — the sky is not a place to put a
    // beacon.
    const sw::Vec3 eye{0.0f, 0.0f, 3.0f};
    sw::Vec3 picked{};
    SW_CHECK(!sw::ui::pickUnitSphere(eye, sw::Vec3{0.9f, 0.0f, -1.0f}, picked));
    SW_CHECK(!sw::ui::pickUnitSphere(eye, sw::Vec3{0.0f, 0.0f, 1.0f}, picked)); // away
    SW_CHECK(!sw::ui::pickUnitSphere(eye, sw::Vec3{0.0f, 0.0f, 0.0f}, picked)); // degenerate

    // Framing a direction and then clicking the middle of the screen must give
    // that direction back: this is what SW_GEOBEACON's "turn to face it" and
    // the player's own drag both rely on.
    for (sw::u32 i = 0; i < 60; ++i)
    {
        const sw::Vec3 wanted = sampleDirection(i);
        sw::f32 yaw = 0.0f;
        sw::f32 pitch = 0.0f;
        sw::ui::orbitCameraAim(wanted, yaw, pitch);
        const sw::Vec3 camera = sw::ui::orbitCameraOffset(yaw, pitch, 2.3f);
        sw::Vec3 centre{};
        SW_CHECK(sw::ui::pickUnitSphere(camera, -camera, centre));
        SW_CHECK(glm::length(centre - wanted) < 1.0e-3f);
    }
}
