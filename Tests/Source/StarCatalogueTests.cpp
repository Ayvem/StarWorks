// ============================================================================
// StarCatalogueTests.cpp — the solar neighbourhood.
//
// Two kinds of check live here and they are worth telling apart.
//
// The FRAME CONVERSION is testable against arithmetic: the vernal equinox, the
// summer solstice point and the ecliptic pole have exact images under the
// rotation, and any sign error anywhere in equatorialToGame() moves at least
// one of the three. That is the check that matters most, because a sky rotated
// by the obliquity, or mirrored, looks completely convincing and is completely
// wrong.
//
// The DATA is testable only for consistency — nothing in this file can tell
// you whether Ross 128 is really 11.0074 light-years away. What it can tell
// you is that the distance in the table and the length of the vector agree,
// that no planet is inside its own star, that the hard-coded host indices in
// the planet table still point at the stars whose names they were written for,
// and that every pair of systems has genuinely interstellar space between
// them — which is not decoration, it is the condition the top of the warp
// ladder unlocks on.
// ============================================================================

#include "TestFramework.hpp"

#include <Physics/PhysicsComponents.hpp>
#include <Space/StarCatalogue.hpp>

#include <algorithm>
#include <cmath>
#include <string>

using namespace sw;
using namespace sw::space;

// ---- the frame conversion --------------------------------------------------

SW_TEST(TheVernalEquinoxIsTheXAxisAndTheEclipticPoleIsY)
{
    // RA 0h, Dec 0: the one direction the equatorial and ecliptic frames
    // share. It has to come out as exactly +X, at exactly the distance asked
    // for, with nothing in Y or Z.
    const WorldVec3 equinox = equatorialToGame(0.0, 0.0, 1.0);
    SW_CHECK(std::abs(equinox.x - 1.0) < 1.0e-12);
    SW_CHECK(std::abs(equinox.y) < 1.0e-12);
    SW_CHECK(std::abs(equinox.z) < 1.0e-12);

    // The ecliptic north pole sits at RA 18h, Dec +66.5607 degrees (that is
    // 90 minus the obliquity). It has to come out as +Y — that is the whole
    // point of the relabelling.
    const f64 poleDec = 90.0 - kObliquityJ2000 * (180.0 / 3.141592653589793);
    const WorldVec3 pole = equatorialToGame(18.0, poleDec, 1.0);
    SW_CHECK(std::abs(pole.x) < 1.0e-9);
    SW_CHECK(std::abs(pole.y - 1.0) < 1.0e-9);
    SW_CHECK(std::abs(pole.z) < 1.0e-9);

    // The summer solstice point, RA 6h at exactly the obliquity, is 90 degrees
    // along the ecliptic from the equinox: in the XZ plane, zero Y. If the
    // rotation were applied with the wrong sign it would land at Y = +/-0.73.
    const WorldVec3 solstice =
        equatorialToGame(6.0, kObliquityJ2000 * (180.0 / 3.141592653589793), 1.0);
    SW_CHECK(std::abs(solstice.y) < 1.0e-9);
    SW_CHECK(std::abs(glm::length(solstice) - 1.0) < 1.0e-12);
}

SW_TEST(TheSkyIsNotMirrored)
{
    // Right-handedness, stated as a fact about three real directions rather
    // than as a determinant: ecliptic longitude runs from the equinox toward
    // the solstice point, and their cross product must be the NORTH pole. Get
    // the relabelling wrong by a reflection and this comes out as the SOUTH
    // pole while every individual distance and angle stays perfect.
    const WorldVec3 equinox = equatorialToGame(0.0, 0.0, 1.0);
    const WorldVec3 solstice =
        equatorialToGame(6.0, kObliquityJ2000 * (180.0 / 3.141592653589793), 1.0);
    const f64 poleDec = 90.0 - kObliquityJ2000 * (180.0 / 3.141592653589793);
    const WorldVec3 pole = equatorialToGame(18.0, poleDec, 1.0);
    const WorldVec3 cross = glm::cross(equinox, solstice);
    SW_CHECK(glm::dot(cross, pole) > 0.999);
}

SW_TEST(EveryCataloguedDistanceIsTheLengthOfItsOwnVector)
{
    for (const SystemRecord& system : systems())
    {
        const f64 expected = system.distanceLightYears * kLightYear;
        const f64 actual = glm::length(system.position);
        // RELATIVE, and it has to be: one part in 1e13 of a hundred quadrillion
        // metres is a kilometre, and a double only carries sixteen digits, so
        // an absolute metre-level tolerance here is a tolerance tighter than
        // the number type. Six of these failed on a one-metre bound and every
        // one of them was correct arithmetic.
        SW_CHECK(std::abs(actual - expected) < std::max(1.0, expected * 1.0e-13));
    }
}

// ---- the neighbourhood, as a shape ----------------------------------------

SW_TEST(ProximaIsTheNearestSystemAndSitsBesideAlphaCentauri)
{
    // Sol is at the origin by construction, so the nearest OTHER anchor to it
    // must be Proxima. If the obliquity rotation were wrong this would still
    // pass — distances from the origin are rotation-invariant — which is why
    // the second half of this test is the one that bites.
    f64 best = 1.0e300;
    std::string bestName;
    for (const SystemRecord& system : systems())
    {
        if (system.distanceLightYears <= 0.0)
        {
            continue;
        }
        const f64 distance = glm::length(system.position);
        if (distance < best)
        {
            best = distance;
            bestName = system.name;
        }
    }
    SW_CHECK_EQ(bestName, std::string("PROXIMA CENTAURI"));

    // Proxima and Alpha Centauri are a bound pair a fifth of a light-year
    // apart — 13 000 AU, half a million years to the orbit. Their catalogue
    // entries are 0.098 ly apart in DISTANCE and 2.2 degrees apart on the sky,
    // and only a correct 3D placement turns those two numbers into 0.21 ly.
    // This is the test that a mirrored or mis-rotated sky fails.
    const WorldVec3* proxima = nullptr;
    const WorldVec3* alpha = nullptr;
    for (const SystemRecord& system : systems())
    {
        if (std::string(system.name) == "PROXIMA CENTAURI") { proxima = &system.position; }
        if (std::string(system.name) == "ALPHA CENTAURI") { alpha = &system.position; }
    }
    SW_CHECK(proxima != nullptr && alpha != nullptr);
    const f64 separation = glm::length(*proxima - *alpha) / kLightYear;
    SW_CHECK(separation > 0.15 && separation < 0.28);
}

SW_TEST(EverySystemHasInterstellarSpaceAroundIt)
{
    // The SOI no longer gates the warp ladder — it was two light-years wide
    // and useless for that — but it is still what says which system's planets
    // are loaded, so two touching spheres would mean a pair of systems that
    // could never be told apart. The catalogue takes 45% of the distance to
    // the NEAREST neighbour rather than a flat radius, and the pair that
    // forces the rule is Alpha Centauri and Proxima, a fifth of a light-year
    // apart and genuinely bound to each other.
    for (const SystemRecord& a : systems())
    {
        for (const SystemRecord& b : systems())
        {
            if (&a == &b)
            {
                continue;
            }
            const f64 distance = glm::length(a.position - b.position);
            const f64 reach = stars()[a.firstStar].soiRadius +
                              stars()[b.firstStar].soiRadius;
            SW_CHECK(reach < distance);
        }
    }
}

SW_TEST(TheContainingSystemTestAgreesWithTheSoiRadii)
{
    SW_CHECK_EQ(containingSystem(WorldVec3{0.0}), 0);
    // Just inside Sol's own sphere, and just outside it.
    const f64 solSoi = stars()[0].soiRadius;
    SW_CHECK_EQ(containingSystem(WorldVec3{solSoi * 0.99, 0.0, 0.0}), 0);
    SW_CHECK_EQ(containingSystem(WorldVec3{solSoi * 1.01, 0.0, 0.0}), -1);
    // Halfway to Proxima is interstellar space, by construction.
    const u32 proxima = nearestSystem(systems()[1].position);
    SW_CHECK_EQ(proxima, 1u);
    SW_CHECK_EQ(containingSystem(systems()[1].position * 0.5), -1);
    SW_CHECK_EQ(containingSystem(systems()[1].position), 1);
}

// ---- the data, checked for self-consistency -------------------------------

SW_TEST(EveryPlanetsHostIsTheStarItWasWrittenFor)
{
    // The planet table indexes stars by NUMBER, because a name lookup at load
    // time is a lookup that can fail silently. The number is only safe as long
    // as something checks it, and this is that something: insert one star into
    // the middle of the table and every planet below it changes host without a
    // single compiler warning.
    struct Expectation
    {
        const char* planet;
        const char* host;
    };
    const Expectation kExpected[] = {
        {"PROXIMA b", "PROXIMA"},         {"PROXIMA d", "PROXIMA"},
        {"BARNARD b", "BARNARD"},         {"BARNARD e", "BARNARD"},
        {"GJ 411 b", "LALANDE 21185"},    {"GJ 411 c", "LALANDE 21185"},
        {"AEGIR", "EPSILON ERIDANI"},     {"GJ 887 d", "LACAILLE 9352"},
        {"ROSS 128 b", "ROSS 128"},       {"GL 725 A b", "STRUVE 2398 A"},
        {"GL 725 B c", "STRUVE 2398 B"},  {"GJ 15 A b", "GROOMBRIDGE 34 A"},
        {"EPSILON INDI Ab", "EPSILON INDI A"},
        {"TAU CETI f", "TAU CETI"},       {"GJ 1061 d", "GJ 1061"},
    };
    for (const Expectation& expectation : kExpected)
    {
        bool found = false;
        for (const PlanetRecord& planet : planets())
        {
            if (std::string(planet.name) != expectation.planet)
            {
                continue;
            }
            found = true;
            SW_CHECK_EQ(std::string(stars()[planet.starIndex].name),
                        std::string(expectation.host));
        }
        SW_CHECK(found);
    }
}

SW_TEST(NoPlanetIsInsideItsStarAndEveryOneCanBeOrbited)
{
    for (const PlanetRecord& planet : planets())
    {
        const StarRecord& host = stars()[planet.starIndex];
        // Outside the photosphere, by a margin.
        SW_CHECK(planet.sma > host.radius * 2.0);
        // A sphere of influence you can actually fly inside: bigger than the
        // planet, smaller than its own orbit. Both bounds have failed before
        // in this codebase for moons, which is why they are here for planets.
        SW_CHECK(planet.soiRadius > planet.radius);
        SW_CHECK(planet.soiRadius < planet.sma);
        SW_CHECK(planet.radius > 0.0 && planet.mu > 0.0);
    }
}

SW_TEST(EveryStarHasAMassARadiusAndASystem)
{
    for (const StarRecord& star : stars())
    {
        SW_CHECK(star.radius > 0.0);
        SW_CHECK(star.mu > 0.0);
        SW_CHECK(star.soiRadius > star.radius);
        SW_CHECK(star.systemIndex < systems().size());
        // A companion's orbit has to clear both stars.
        if (star.sma > 0.0)
        {
            const StarRecord& primary = stars()[systems()[star.systemIndex].firstStar];
            SW_CHECK(star.sma > (star.radius + primary.radius) * 4.0);
        }
    }
    // Every system's star span covers it exactly once.
    usize counted = 0;
    for (const SystemRecord& system : systems())
    {
        SW_CHECK(system.starCount >= 1);
        counted += system.starCount;
        for (u32 i = 0; i < system.starCount; ++i)
        {
            SW_CHECK_EQ(stars()[system.firstStar + i].systemIndex,
                        static_cast<u32>(&system - systems().data()));
        }
    }
    SW_CHECK_EQ(counted, stars().size());
}

SW_TEST(TheSurfaceStyleLadderPutsTheSolarSystemBackWhereItBelongs)
{
    // The rule is only defensible if, fed the solar system's own numbers, it
    // hands back the solar system's own bodies.
    SW_CHECK_EQ(styleForWorld(0.055, 440.0), 3);  // Mercury
    SW_CHECK_EQ(styleForWorld(1.0, 255.0), 2);    // Terra -> the temperate band
    SW_CHECK_EQ(styleForWorld(0.107, 210.0), 2);  // Mars
    SW_CHECK_EQ(styleForWorld(0.025, 134.0), 7);  // Callisto
    SW_CHECK_EQ(styleForWorld(0.008, 102.0), 6);  // Europa -> the icy band
    SW_CHECK_EQ(styleForWorld(0.0000018, 38.0), 13); // Triton
    SW_CHECK_EQ(styleForWorld(317.8, 110.0), 20); // Jupiter
    SW_CHECK_EQ(styleForWorld(95.2, 81.0), 22);   // Saturn's mass is giant-band
    SW_CHECK_EQ(styleForWorld(17.1, 47.0), 22);   // Neptune -> ice giant

    // And Terra's own equilibrium temperature really is 255 K under this
    // formula, which is the calibration the whole ladder hangs on.
    const f64 terra = equilibriumTemperature(1.0, kAstronomicalUnit);
    SW_CHECK(std::abs(terra - 255.2) < 0.5);
}

SW_TEST(TheHabitableZoneWorldsComeOutTemperate)
{
    // Not a physics claim — a claim that the derived temperatures are not
    // nonsense. The four worlds the literature calls habitable-zone should
    // land in the band between the freezing and boiling points of water, give
    // or take the greenhouse this model does not have.
    const char* kZone[] = {"PROXIMA b", "GJ 887 d", "GL 725 B c", "GJ 1061 d"};
    for (const char* name : kZone)
    {
        bool found = false;
        for (const PlanetRecord& planet : planets())
        {
            if (std::string(planet.name) != name)
            {
                continue;
            }
            found = true;
            SW_CHECK(planet.equilibriumTemperature > 190.0);
            SW_CHECK(planet.equilibriumTemperature < 300.0);
            SW_CHECK(planet.atmosphereTopAltitude > 0.0);
        }
        SW_CHECK(found);
    }
}

SW_TEST(AStarsPositionIsItsAnchorPlusItsOwnOrbit)
{
    for (u32 i = 0; i < stars().size(); ++i)
    {
        const StarRecord& star = stars()[i];
        const WorldVec3 anchor = systems()[star.systemIndex].position;
        const WorldVec3 position = starPositionAt(i, 0.0);
        const f64 offset = glm::length(position - anchor);
        if (star.sma <= 0.0)
        {
            SW_CHECK(offset < 1.0e-6); // the primary IS the anchor
            continue;
        }
        // A companion is somewhere on its ellipse: between periapsis and
        // apoapsis, and nowhere else.
        SW_CHECK(offset > star.sma * (1.0 - star.eccentricity) * 0.999);
        SW_CHECK(offset < star.sma * (1.0 + star.eccentricity) * 1.001);
    }
}

// ---- the two rules that changed when the neighbourhood arrived -------------

SW_TEST(TheWarpLadderOpensInThreeBandsMeasuredFromTheStar)
{
    using namespace sw::phys;
    // Nothing nearby: the altitude ladder is at its own top and has nothing
    // more to say, so distance from the STAR decides.
    constexpr f64 kFree = kSystemWarpCeiling;

    // Among the planets, the in-system ceiling stands.
    SW_CHECK_EQ(maxWarpForSpace(1.5e11, kFree), kSystemWarpCeiling);  // Terra
    SW_CHECK_EQ(maxWarpForSpace(4.5e12, kFree), kSystemWarpCeiling);  // Neptune
    SW_CHECK_EQ(maxWarpForSpace(4.999e12, kFree), kSystemWarpCeiling);
    // Five billion kilometres: past Neptune, in the Kuiper belt.
    SW_CHECK_EQ(maxWarpForSpace(5.0e12, kFree), kDeepSpaceWarpCeiling);
    SW_CHECK_EQ(maxWarpForSpace(4.9e13, kFree), kDeepSpaceWarpCeiling);
    // Fifty billion: ten times the outermost orbit.
    SW_CHECK_EQ(maxWarpForSpace(5.0e13, kFree), kInterstellarWarpCeiling);
    SW_CHECK_EQ(maxWarpForSpace(9.9e14, kFree), kInterstellarWarpCeiling);
    // A thousand billion: a tenth of a light-year, and nothing out there.
    SW_CHECK_EQ(maxWarpForSpace(1.0e15, kFree), kVoidWarpCeiling);
    SW_CHECK_EQ(maxWarpForSpace(4.0e16, kFree), kVoidWarpCeiling);

    // AND THE ALTITUDE LADDER STILL WINS WHENEVER IT BINDS. A craft in the
    // air, or low over a Kuiper body six billion kilometres out, is in the
    // ladder's territory and must not be handed x100M because of where the
    // SUN is.
    SW_CHECK_EQ(maxWarpForSpace(6.0e12, 5.0), 5.0);      // in atmosphere
    SW_CHECK_EQ(maxWarpForSpace(6.0e12, 1.0e4), 1.0e4);  // low over a body
    SW_CHECK_EQ(maxWarpForSpace(4.0e16, 5.0), 5.0);      // even out there

    // THE BANDS AND THE RUNGS AGREE. warpRadiusForRate answers "how far out
    // does this rung need to be", and it is what the refusal message reads, so
    // a rung whose radius disagreed with the ceiling that granted it would put
    // a wrong number on the HUD.
    SW_CHECK_EQ(warpRadiusForRate(1.0e7), 0.0);
    SW_CHECK_EQ(warpRadiusForRate(1.0e8), kDeepSpaceWarpRadius);
    SW_CHECK_EQ(warpRadiusForRate(1.0e9), kInterstellarWarpRadius);
    SW_CHECK_EQ(warpRadiusForRate(1.0e10), kVoidWarpRadius);
    const f64 rungs[] = {1.0e8, 1.0e9, 1.0e10};
    for (const f64 rate : rungs)
    {
        const f64 radius = warpRadiusForRate(rate);
        // Exactly AT its radius the rung is granted, and a hair inside it is
        // not. An off-by-one in either direction here is a rung that can never
        // be selected, or one that can be selected everywhere.
        SW_CHECK(maxWarpForSpace(radius, kFree) >= rate);
        SW_CHECK(maxWarpForSpace(radius * 0.999, kFree) < rate);
    }
}

SW_TEST(EveryCrossingSpendsMostOfItselfAtTheTopRung)
{
    // The bands are only useful if a real journey actually reaches them. The
    // nearest anchor is Proxima at 4.0e16 metres, so a crossing passes the
    // top radius — a thousand billion kilometres — after two and a half per
    // cent of itself and spends the rest at x10B. That is what makes these
    // departure rungs rather than curiosities, and it is also why the sphere
    // of influence was the wrong boundary: at 1.9e16 metres it would have
    // opened the top rung with only sixty per cent of the trip left.
    for (const SystemRecord& system : systems())
    {
        if (system.distanceLightYears <= 0.0)
        {
            continue;
        }
        const f64 distance = glm::length(system.position);
        SW_CHECK(distance > sw::phys::kVoidWarpRadius * 20.0);
        SW_CHECK(sw::phys::kVoidWarpRadius < stars()[0].soiRadius * 0.1);
    }
}

// ============================================================================
// WHAT EACH STAR LOOKS LIKE
//
// "Il faut que les geantes rouges soient geantes et rouges et les naines
// rouges soient petites et rouges et les etoiles bleues soient bleues."
//
// The catalogue's physics was already right — every radius and temperature
// here is interferometric or asteroseismic — and the RENDERING of it was not,
// for one reason that a table of numbers finds instantly and an eye does not:
// the colour ladder held sRGB-ENCODED values and they were used as linear
// multipliers. 2800 K carried 0.68 in green where the linear locus says 0.444,
// and 0.68 IS 0.444 gamma-encoded. Every star in the game was pulled toward
// white by exactly one missing conversion, which is why a red dwarf came out
// pale peach and Sirius came out very nearly white.
//
// These assertions are about what the eye is supposed to be able to say.
// ============================================================================
SW_TEST(TheColourLadderPutsEveryStarWhereItsNameSaysItIs)
{
    using namespace sw::space;

    // SOL IS NEUTRAL BY CONSTRUCTION. The locus is divided by its own value at
    // 5772 K, which is what makes every other colour a statement ABOUT THE
    // SUN — the comparison the words red dwarf and blue giant were coined
    // from — and what keeps twenty milestones of solar-system tuning valid.
    const sw::Vec3 sol = blackbodyColor(5772.0f);
    SW_CHECK(std::abs(sol.r - 1.0f) < 1.0e-3f);
    SW_CHECK(std::abs(sol.g - 1.0f) < 1.0e-3f);
    SW_CHECK(std::abs(sol.b - 1.0f) < 1.0e-3f);

    // A RED DWARF IS RED. Barnard's Star at 3195 K: blue well under a third,
    // green well under three quarters. The old ladder gave it (1, 0.73, 0.52),
    // which is a peach.
    const sw::Vec3 barnard = blackbodyColor(3195.0f);
    SW_CHECK(barnard.r > 0.99f);
    SW_CHECK(barnard.b < 0.32f);
    SW_CHECK(barnard.g < 0.70f);

    // A BLUE STAR IS BLUE. Sirius B at 25 000 K must be blue by a wide margin,
    // not a hint of one: red no more than half of blue.
    const sw::Vec3 siriusB = blackbodyColor(25000.0f);
    SW_CHECK(siriusB.b > 0.99f);
    SW_CHECK(siriusB.r < 0.50f);
    SW_CHECK(siriusB.r < siriusB.g);

    // ...and Sirius A at 9845 K, an A1V, is blue-white: on the blue side,
    // clearly, but not as far as a 25 000 K dwarf.
    const sw::Vec3 siriusA = blackbodyColor(9845.0f);
    SW_CHECK(siriusA.b > siriusA.r + 0.30f);
    SW_CHECK(siriusA.r > siriusB.r);

    // MONOTONE IN TEMPERATURE, over the whole range the catalogue uses. Blue
    // may only rise with temperature and the red/blue balance may only fall:
    // a ladder that wobbles puts a K5 dwarf bluer than a G8 somewhere, and
    // nobody would ever find that by looking.
    sw::f32 previousBlue = -1.0f;
    sw::f32 previousBalance = 1.0e9f;
    for (sw::f32 kelvin = 900.0f; kelvin <= 26000.0f; kelvin *= 1.05f)
    {
        const sw::Vec3 c = blackbodyColor(kelvin);
        SW_CHECK(c.b >= previousBlue - 1.0e-5f);
        const sw::f32 balance = c.r / std::max(c.b, 1.0e-4f);
        SW_CHECK(balance <= previousBalance + 1.0e-3f);
        previousBlue = c.b;
        previousBalance = balance;
        // Normalised: the brightest channel is always exactly 1, because the
        // magnitude carries the brightness and this carries only the hue.
        SW_CHECK(std::abs(std::max({c.r, c.g, c.b}) - 1.0f) < 1.0e-4f);
    }

    // AND NOTHING IS GREEN. Below about 500 K a blackbody's visible tail is
    // numerically nothing, and dividing nothing by the solar locus amplifies
    // the rounding: WISE 0855 at 276 K came out (1.00, 0.90, 0.00), a bright
    // yellow-green for an object that emits no visible light at all. The floor
    // holds it at the deepest red instead.
    for (sw::f32 kelvin = 50.0f; kelvin < 1200.0f; kelvin += 17.0f)
    {
        const sw::Vec3 c = blackbodyColor(kelvin);
        SW_CHECK(c.r > 0.99f);      // red always leads down here
        SW_CHECK(c.g < 0.20f);
        SW_CHECK(c.b < 0.05f);
    }
}

SW_TEST(EveryStarsSizeAndColourAgreeWithItsSpectralClass)
{
    using namespace sw::space;
    // THE CATALOGUE, WALKED. Every M dwarf must be small AND red, every white
    // dwarf tiny, every A star blue-white — checked against the designation
    // each record already carries, so a future edit that mistypes a radius or
    // a temperature fails here instead of looking slightly wrong in a frame
    // nobody photographs.
    sw::usize mDwarfs = 0;
    sw::usize whiteDwarfs = 0;
    sw::usize hotStars = 0;
    for (const StarRecord& star : stars())
    {
        const sw::Vec3 hue = blackbodyColor(static_cast<sw::f32>(star.temperature));
        const sw::f64 solarRadii = star.radius / kSunRadius;
        const std::string_view type(star.designation);

        // A WHITE DWARF is an Earth inside a solar mass, and it is hot.
        if (type.starts_with("D"))
        {
            ++whiteDwarfs;
            SW_CHECK(solarRadii < 0.03);        // Earth-sized, not star-sized
            SW_CHECK(star.temperature > 7000.0);
            SW_CHECK(hue.b >= hue.r);           // on the blue side of neutral
            continue;
        }
        // AN M DWARF is small and red. Both halves, because the failure this
        // test exists for made them the right size and the wrong colour.
        if (type.starts_with("M"))
        {
            ++mDwarfs;
            SW_CHECK(solarRadii < 0.65);
            SW_CHECK(star.temperature < 4000.0);
            SW_CHECK(hue.r > 0.99f);
            SW_CHECK(hue.b < 0.45f); // unmistakably red, not peach
            continue;
        }
        // A/B: blue-white, and bigger than the Sun on the main sequence.
        if (type.starts_with("A") || type.starts_with("B"))
        {
            ++hotStars;
            SW_CHECK(star.temperature > 7500.0);
            SW_CHECK(hue.b > hue.r + 0.25f);
            SW_CHECK(solarRadii > 1.0);
            continue;
        }
        // L/T/Y: brown dwarfs and colder. Planet-sized and barely glowing.
        if (type.starts_with("L") || type.starts_with("T") || type.starts_with("Y"))
        {
            SW_CHECK(solarRadii < 0.15);
            SW_CHECK(star.temperature < 1500.0);
            SW_CHECK(hue.b < 0.02f);
            continue;
        }
        // Everything else is F, G or K: between the two, and neither extreme.
        SW_CHECK(star.temperature > 4000.0 && star.temperature < 7500.0);
        SW_CHECK(solarRadii > 0.4 && solarRadii < 3.0);
    }
    // The catalogue really does contain each family — a walk that matched
    // nothing would pass every assertion above.
    SW_CHECK(mDwarfs >= 15);
    SW_CHECK_EQ(whiteDwarfs, static_cast<sw::usize>(2)); // Sirius B, Procyon B
    SW_CHECK(hotStars >= 1);                             // Sirius A
}

SW_TEST(ACompanionSunIsOneWhenItIsOneAndNotWhenItIsNot)
{
    using namespace sw::space;
    // THE THRESHOLD, CHECKED AGAINST THE CATALOGUE'S OWN NUMBERS rather than
    // against a remembered example. kSunIrradianceRatio decides whether a star
    // is drawn as a SUN — glare, disc and all — or as one billboard among the
    // background; these are the two cases that have to come out differently or
    // the constant is wrong.
    constexpr f64 kAu = 1.495978707e11;
    auto irradiance = [](f64 luminosity, f64 metres) {
        return luminosity / (metres * metres);
    };

    // Alpha Centauri: A at 1.25 AU (its habitable zone), B at its 23.3 AU
    // mean separation. B is the second sun in that sky and must be drawn as
    // one.
    const f64 alphaA = irradiance(1.5059, 1.25 * kAu);
    const f64 alphaB = irradiance(0.4981, 23.30 * kAu);
    SW_CHECK(alphaB / alphaA > kSunIrradianceRatio);

    // ...and from Proxima b, thirteen thousand AU away, the same pair is a
    // pair of bright STARS. Eight orders below the cut, so no threshold near
    // this one could confuse the two cases.
    const f64 proxima = irradiance(0.001567, 0.0485 * kAu); // Proxima b's orbit
    const f64 alphaFromProxima = irradiance(1.5059, 13000.0 * kAu);
    SW_CHECK(alphaFromProxima / proxima < kSunIrradianceRatio * 1.0e-3);

    // A WIDE COMPANION STAYS A POINT. A second Sol a thousand AU out, seen
    // from a world at 1 AU, is magnitude -11 — the brightest thing in that
    // sky after the sun, and still not a second daylight.
    const f64 wide = irradiance(1.0, 1000.0 * kAu) / irradiance(1.0, 1.0 * kAu);
    SW_CHECK(wide < kSunIrradianceRatio);

    // AND THE RATIO IS SCALE-FREE, which is the property that stops a binary
    // from POPPING as you leave it: move ten times further from both and the
    // decision is unchanged, so two suns stay two suns all the way out.
    const f64 farA = irradiance(1.5059, 10.0 * 1.25 * kAu);
    const f64 farB = irradiance(0.4981, std::sqrt(std::pow(10.0 * 1.25, 2.0) +
                                                  std::pow(23.30, 2.0)) * kAu);
    SW_CHECK(farB / farA > kSunIrradianceRatio);
}
