#pragma once

// ============================================================================
// Space/StarCatalogue.hpp
// The solar neighbourhood: every star system whose distance is under twelve
// light-years, with the planets we actually know about.
//
// WHY IT LIVES IN THE ENGINE. Because it is DATA WITH A RULE, and the rule is
// the part that can be wrong. Right ascension and declination are angles on
// the equator of a planet this game does not privilege; the simulation runs
// in an ecliptic frame with +Y north and the reference plane XZ (see
// Physics/Kepler.hpp). Turning one into the other is a rotation by the
// obliquity, and a rotation applied in the wrong direction still produces a
// tidy-looking sky — just a sky rotated 47 degrees off the real one. That is
// exactly the class of mistake a test catches and an eye does not, so the
// conversion is here, next to the numbers, and Tests/Source/SpaceTests.cpp
// checks it against positions computed by hand.
//
// THE SYSTEMS DO NOT MOVE. Every star in this table has a proper motion of
// several arcseconds a year — Barnard's Star has ten, the largest known — and
// none of it is modelled. Over a career's worth of game time that is a fixed
// error of a few light-hours on a four-light-year baseline, and modelling it
// would mean the map you learn is not the map you fly. The positions are
// J2000 and they stay J2000.
//
// WHAT IS AND IS NOT A PLANET. The table carries CONFIRMED planets only. That
// is a real editorial line and it moves: Proxima c was a planet in 2020 and is
// not one now, Kapteyn b was famous and is an artefact of the star's rotation,
// Tau Ceti b/c/d/e did not survive their own follow-up. Anything still argued
// about is left out and named in the comment beside its star, so that adding
// it later is a decision rather than an oversight.
// ============================================================================

#include "Core/Types.hpp"
#include "Math/Math.hpp"

#include <span>

namespace sw::space
{
    /// One light-year in metres (IAU: the Julian year times c).
    inline constexpr f64 kLightYear = 9.4607304725808e15;
    /// One astronomical unit in metres (IAU 2012 definition, exact).
    inline constexpr f64 kAstronomicalUnit = 1.495978707e11;
    /// Gravitational parameters used to turn catalogue masses into physics.
    inline constexpr f64 kMuSun = 1.32712440018e20;
    inline constexpr f64 kMuEarth = 3.986004418e14;
    inline constexpr f64 kMuJupiterPlanet = 1.26686534e17;
    inline constexpr f64 kSunRadius = 6.957e8;
    inline constexpr f64 kEarthRadius = 6.371e6;
    inline constexpr f64 kJupiterRadius = 7.1492e7;
    /// Obliquity of the ecliptic at J2000 (IAU 2006), radians.
    inline constexpr f64 kObliquityJ2000 = 0.409092600600583;

    /// A star. `systemIndex` groups the components of a multiple star; the
    /// first star of a system is its PRIMARY and sits at the system's anchor,
    /// the others orbit it.
    ///
    /// A companion's elements are the elements of the RELATIVE orbit, applied
    /// as if the primary were fixed. The real pair turns about a barycentre
    /// between them, so the primary is drawn stiller than it is — for Alpha
    /// Centauri that is a ten-AU wobble over eighty years, invisible against a
    /// four-light-year baseline and worth the simplification: it keeps the
    /// system's anchor a CONSTANT, which is what the floating origin needs.
    /// How much of the dominant star's irradiance a companion must deliver at
    /// the camera before it is drawn as a SUN — glare, disc and all — rather
    /// than as one billboard among the background stars.
    ///
    /// A RATIO rather than an absolute brightness, because both members of a
    /// pair dim together as you leave: the ratio is very nearly constant all
    /// the way out, so a binary keeps two suns at every range instead of
    /// popping from glare to point the moment it crosses a fixed threshold.
    /// Alpha Cen B seen from A's habitable zone lands at 6.3e-4; Alpha Cen A
    /// seen from Proxima's planet at 1.3e-8.
    inline constexpr f64 kSunIrradianceRatio = 1.0e-4;

    /// 20 000 K down to 3 300 K — converted to linear sRGB and normalized
    /// so its brightest channel is 1: the magnitude carries the
    /// brightness, this only has to carry the hue.
    ///
    /// The class MIX is the Bright Star Catalogue's, and it is nothing
    /// like the galaxy's. Three quarters of all stars are M dwarfs and not
    /// one of them can be seen without a telescope; what fills a real sky
    /// is hot stars near enough to see and cool GIANTS far enough away to
    /// be anywhere. That is where the 24% K comes from, and it is the
    /// reason a sky drawn from the true stellar population comes out red.
    /// Blackbody colour from an effective temperature, normalised so the
    /// brightest channel is 1 — the brightness is carried separately, this
    /// is only the hue.
    ///
    /// The starfield's `starClassColor` below picks a spectral class out of
    /// a random draw, which is what nine thousand anonymous stars need. The
    /// catalogue's thirty-six are not anonymous: each one has a MEASURED
    /// effective temperature, from 276 K for the rogue world WISE 0855 to
    /// 25 000 K for Sirius B, and the same ladder read by temperature
    /// instead of by dice puts every one of them where it belongs. Sirius B
    /// is the reason this exists — a white dwarf is off the end of the main
    /// sequence and no spectral-class draw would ever produce it.
    [[nodiscard]] inline Vec3 blackbodyColor(f32 kelvin)
    {
        // Anchors on the Planckian locus, converted to sRGB primaries and
        // scaled so max(r,g,b) = 1. Between them, linear in log T — which
        // is how colour actually moves along the locus.
        struct Anchor
        {
            f32 kelvin;
            Vec3 color;
        };
        // ANCHORED AT SOL, AND IN LINEAR LIGHT. Both of those were
        // wrong and the second one was a colour-space bug.
        //
        // THE GAMMA. These values are multiplied into vColor.rgb, which
        // the soft-emissive branch hands to gradeCinematic and writes to
        // an sRGB swapchain — so the shader's domain is LINEAR and every
        // number here is a linear multiplier. The old ladder held sRGB-
        // ENCODED values: 2800 K carried 0.68 in green where the linear
        // locus says 0.444, and 0.68 IS 0.444 gamma-encoded, to three
        // decimals. Every star in the game was therefore pulled toward
        // white by exactly the amount sRGB encoding lifts a mid-tone,
        // which is why a red dwarf came out pale peach and Sirius came out
        // very nearly white. The whole ladder was one missing conversion.
        //
        // THE ANCHOR. Normalising each temperature against ITSELF says
        // "this is what a 3000 K body looks like to nobody in particular";
        // dividing by the locus at 5772 K first says "this is what it
        // looks like NEXT TO THE SUN", which is the comparison the words
        // red dwarf and blue giant were coined from, and the adaptation
        // state of the only eye that will ever read this screen. It also
        // pins Sol at exactly (1,1,1), so every frame of the solar system
        // tuned over the last twenty milestones is untouched by this.
        //
        // Computed from Planck's law through the CIE 1931 observer into
        // linear sRGB (D65), divided by the same at 5772 K, renormalised
        // so the brightest channel is 1 — the magnitude carries the
        // brightness, this only carries the hue.
        //
        // THE FLOOR IS 800 K and it is not cosmetic. Below about 500 K a
        // blackbody's visible tail is numerically nothing, and dividing
        // nothing by the solar locus amplifies the rounding: WISE 0855 at
        // 276 K came out (1.00, 0.90, 0.00), a bright YELLOW-GREEN for an
        // object that emits no visible light at all. Held at the deepest
        // red instead, which is what the last thing you can see looks
        // like.
        const Anchor ladder[] = {
            {800.0f, {1.000f, 0.000f, 0.000f}},   // the last visible ember
            {1300.0f, {1.000f, 0.110f, 0.000f}},  // a brown dwarf
            {2000.0f, {1.000f, 0.303f, 0.009f}},  // a T dwarf
            {2800.0f, {1.000f, 0.505f, 0.142f}},  // late M
            {3600.0f, {1.000f, 0.676f, 0.345f}},  // early M
            {4600.0f, {1.000f, 0.847f, 0.644f}},  // K
            {5772.0f, {1.000f, 1.000f, 1.000f}},  // Sol, by construction
            {7500.0f, {0.680f, 0.787f, 1.000f}},  // F/A
            {10000.0f, {0.501f, 0.650f, 1.000f}}, // A
            {25000.0f, {0.312f, 0.478f, 1.000f}}, // a hot white dwarf
        };
        constexpr i32 kCount = static_cast<i32>(std::size(ladder));
        if (kelvin <= ladder[0].kelvin) { return ladder[0].color; }
        if (kelvin >= ladder[kCount - 1].kelvin) { return ladder[kCount - 1].color; }
        for (i32 i = 0; i + 1 < kCount; ++i)
        {
            if (kelvin <= ladder[i + 1].kelvin)
            {
                const f32 t =
                    (std::log(kelvin) - std::log(ladder[i].kelvin)) /
                    (std::log(ladder[i + 1].kelvin) - std::log(ladder[i].kelvin));
                return glm::mix(ladder[i].color, ladder[i + 1].color, t);
            }
        }
        return ladder[kCount - 1].color;
    }

    struct StarRecord
    {
        const char* name = "";
        const char* designation = ""; // catalogue name, for the map
        u32 systemIndex = 0;
        f64 radius = 0.0;      // metres
        f64 mu = 0.0;          // m^3/s^2
        f64 luminosity = 0.0;  // solar luminosities, bolometric
        f64 temperature = 0.0; // effective temperature, K
        // Relative orbit about the system primary. Zero for the primary.
        f64 sma = 0.0; // metres
        f64 eccentricity = 0.0;
        f64 inclination = 0.0;   // radians, from the game's XZ plane
        f64 ascendingNode = 0.0; // radians about +Y from +X
        f64 meanAnomaly = 0.0;   // radians at epoch
        f64 soiRadius = 0.0;     // metres, filled in by the catalogue builder
    };

    /// A planet of one of those stars. `starIndex` indexes stars().
    struct PlanetRecord
    {
        const char* name = "";
        u32 starIndex = 0;
        f64 radius = 0.0; // metres
        f64 mu = 0.0;     // m^3/s^2
        f64 sma = 0.0;    // metres
        f64 eccentricity = 0.0;
        f64 inclination = 0.0;   // radians
        f64 ascendingNode = 0.0; // radians
        f64 meanAnomaly = 0.0;   // radians at epoch
        f64 soiRadius = 0.0;     // metres
        f64 equilibriumTemperature = 0.0; // K, albedo 0.3, derived
        i32 surfaceStyle = 0;  // chosen from the temperature, see styleForWorld
        f64 atmosphereTopAltitude = 0.0; // metres, 0 = airless
        bool massIsMinimum = true;       // almost all of them: radial velocity
    };

    /// A system: a name, an anchor position, and the stars that belong to it.
    struct SystemRecord
    {
        const char* name = "";
        f64 rightAscensionHours = 0.0;
        f64 declinationDegrees = 0.0;
        f64 distanceLightYears = 0.0;
        WorldVec3 position{}; // metres, game frame, from the three above
        u32 firstStar = 0;
        u32 starCount = 0;
    };

    /// Equatorial J2000 (RA in hours, declination in degrees) plus a distance,
    /// turned into the game's ecliptic frame: +Y is the ecliptic north pole,
    /// +X points at the vernal equinox, and the XZ plane is the ecliptic.
    ///
    /// The middle step is the one worth writing down. An equatorial direction
    /// is rotated onto the ecliptic about the +X axis (the equinox is the one
    /// direction the two frames share) by the obliquity, and then the axes are
    /// relabelled — ecliptic Z, the pole, becomes game Y. The relabelling is a
    /// PROPER rotation (determinant +1), which is why it is written as
    /// (x, z, -y) and not the tidier-looking (x, z, y): the tidy one is a
    /// reflection and would hand back a mirror-image sky.
    [[nodiscard]] WorldVec3 equatorialToGame(f64 rightAscensionHours,
                                             f64 declinationDegrees,
                                             f64 distanceMetres);

    /// Equilibrium temperature of a body at `smaMetres` from a star of
    /// `luminosity` solar luminosities, for a Bond albedo of 0.3.
    [[nodiscard]] f64 equilibriumTemperature(f64 luminosity, f64 smaMetres);

    /// Which of the game's surface styles a world of this mass and
    /// temperature gets. Everything past a few Earth masses is an envelope
    /// rather than a ground; below that the choice is temperature alone, and
    /// the ladder runs from molten through desert to nitrogen ice.
    ///
    /// This is a GUESS AND IT IS LABELLED ONE. Nobody has photographed any of
    /// these worlds; what is known is a mass, a period and a star. Deriving
    /// the look from those two numbers by a fixed rule is at least honest,
    /// reproducible, and gives the neighbourhood the variety it really has —
    /// as opposed to picking each one by hand, which would encode nothing but
    /// the author's taste while looking exactly as authoritative.
    [[nodiscard]] i32 styleForWorld(f64 massEarths, f64 equilibriumKelvin);

    /// The catalogue. Built once, on first use.
    [[nodiscard]] std::span<const SystemRecord> systems();
    [[nodiscard]] std::span<const StarRecord> stars();
    [[nodiscard]] std::span<const PlanetRecord> planets();

    /// Absolute position of a star, metres, game frame: its system's anchor
    /// plus its own relative orbit evaluated at `timeSeconds`.
    [[nodiscard]] WorldVec3 starPositionAt(u32 starIndex, f64 timeSeconds);

    /// Index of the system whose anchor is nearest `position` (absolute,
    /// game frame). Never negative — the catalogue is never empty.
    [[nodiscard]] u32 nearestSystem(const WorldVec3& position);

    /// Index of the system the point is INSIDE, by the primary's SOI, or -1
    /// for interstellar space. This is the test the deep-space time warp is
    /// gated on: above ten million, a rung is a rung between the stars.
    [[nodiscard]] i32 containingSystem(const WorldVec3& position);

    /// Sol's own entry, which is index 0 and sits at the origin of the frame.
    inline constexpr u32 kSolSystem = 0;
} // namespace sw::space
