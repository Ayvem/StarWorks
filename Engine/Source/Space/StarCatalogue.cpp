// ============================================================================
// Space/StarCatalogue.cpp
// The numbers, and the three rules that turn them into a playable frame.
// ============================================================================

#include "Space/StarCatalogue.hpp"

#include "Physics/Kepler.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>

namespace sw::space
{
    namespace
    {
        constexpr f64 solarRadii(f64 x) { return x * kSunRadius; }
        constexpr f64 solarMasses(f64 x) { return x * kMuSun; }
        constexpr f64 earthRadii(f64 x) { return x * kEarthRadius; }
        constexpr f64 earthMasses(f64 x) { return x * kMuEarth; }
        constexpr f64 jupiterRadii(f64 x) { return x * kJupiterRadius; }
        constexpr f64 jupiterMasses(f64 x) { return x * kMuJupiterPlanet; }
        constexpr f64 au(f64 x) { return x * kAstronomicalUnit; }
        constexpr f64 lightYears(f64 x) { return x * kLightYear; }

        /// A deterministic angle in [0, 2*pi) from a name. Used ONLY for the
        /// things the catalogue genuinely does not contain: the orientation of
        /// a system's plane against Sol's ecliptic, and where each planet is
        /// in its orbit at t = 0.
        ///
        /// Radial velocity measures a period and a minimum mass. It does not
        /// measure an inclination, a node, or a phase — for almost every world
        /// in this table those three numbers are simply unknown. Leaving them
        /// at zero would be a choice too, and a worse one: every system in the
        /// neighbourhood would share Sol's ecliptic and every planet in it
        /// would start lined up on the +X axis, which is a thing no sky has
        /// ever looked like. A hash of the name is arbitrary but it is STABLE:
        /// the same sky every launch, the same sky in the save file, and no
        /// table of invented numbers pretending to be measurements.
        [[nodiscard]] f64 hashAngle(const char* text, u32 salt)
        {
            u64 h = 1469598103934665603ull ^ (static_cast<u64>(salt) * 1099511628211ull);
            for (const char* p = text; *p != '\0'; ++p)
            {
                h ^= static_cast<u64>(static_cast<unsigned char>(*p));
                h *= 1099511628211ull;
            }
            h ^= h >> 33;
            h *= 0xff51afd7ed558ccdull;
            h ^= h >> 33;
            return 6.283185307179586 * (static_cast<f64>(h >> 11) /
                                        static_cast<f64>(1ull << 53));
        }

        // ------------------------------------------------------------------
        // THE SYSTEMS. Right ascension and declination are J2000; distances
        // are the best modern parallaxes (Gaia DR3 where its astrometry is
        // clean, Hipparcos or dedicated orbital solutions where Gaia's RUWE
        // says the star's own binary motion corrupted the fit — Sirius,
        // Procyon and Epsilon Eridani are all in that second group).
        // ------------------------------------------------------------------
        struct SystemSeed
        {
            const char* name;
            f64 raHours;
            f64 decDegrees;
            f64 distanceLy;
        };

        constexpr f64 hms(f64 h, f64 m, f64 s) { return h + m / 60.0 + s / 3600.0; }
        constexpr f64 dms(f64 d, f64 m, f64 s)
        {
            // The sign belongs to the whole angle, not to the degrees field:
            // -00 30 00 is half a degree SOUTH and a naive d + m/60 would put
            // it half a degree north. Every declination in this table between
            // zero and minus one degree would have been mirrored.
            const f64 magnitude = ((d < 0.0) ? -d : d) + m / 60.0 + s / 3600.0;
            return (d < 0.0) ? -magnitude : magnitude;
        }

        const SystemSeed kSystemSeeds[] = {
            // Sol is index 0 and its "distance" is zero: it IS the frame.
            {"SOL", 0.0, 0.0, 0.0},
            {"PROXIMA CENTAURI", hms(14, 29, 42.95), dms(-62, 40, 46.2), 4.2465},
            {"ALPHA CENTAURI", hms(14, 39, 36.49), dms(-60, 50, 2.4), 4.3441},
            {"BARNARD'S STAR", hms(17, 57, 48.50), dms(4, 41, 36.1), 5.9629},
            {"LUHMAN 16", hms(10, 49, 18.77), dms(-53, 19, 9.9), 6.5102},
            {"WISE 0855-0714", hms(8, 55, 10.83), dms(-7, 14, 42.5), 7.4295},
            {"WOLF 359", hms(10, 56, 28.92), dms(7, 0, 53.0), 7.8558},
            {"LALANDE 21185", hms(11, 3, 20.19), dms(35, 58, 11.6), 8.3044},
            {"SIRIUS", hms(6, 45, 8.92), dms(-16, 42, 58.0), 8.6080},
            {"LUYTEN 726-8", hms(1, 39, 1.38), dms(-17, 57, 2.6), 8.7695},
            {"ROSS 154", hms(18, 49, 49.36), dms(-23, 50, 10.5), 9.7063},
            {"ROSS 248", hms(23, 41, 55.04), dms(44, 10, 38.8), 10.3057},
            {"EPSILON ERIDANI", hms(3, 32, 55.84), dms(-9, 27, 29.7), 10.4749},
            {"LACAILLE 9352", hms(23, 5, 52.04), dms(-35, 51, 11.1), 10.7241},
            {"ROSS 128", hms(11, 47, 44.40), dms(0, 48, 16.4), 11.0074},
            {"EZ AQUARII", hms(22, 38, 33.70), dms(-15, 17, 57.0), 11.1090},
            {"61 CYGNI", hms(21, 6, 53.94), dms(38, 44, 57.9), 11.4043},
            {"PROCYON", hms(7, 39, 18.12), dms(5, 13, 30.0), 11.4618},
            {"STRUVE 2398", hms(18, 42, 46.70), dms(59, 37, 49.4), 11.4909},
            {"GROOMBRIDGE 34", hms(0, 18, 22.89), dms(44, 1, 22.6), 11.6191},
            {"DX CANCRI", hms(8, 29, 49.35), dms(26, 46, 33.6), 11.6797},
            {"EPSILON INDI", hms(22, 3, 21.65), dms(-56, 47, 9.5), 11.8670},
            {"TAU CETI", hms(1, 44, 4.08), dms(-15, 56, 14.9), 11.9118},
            {"GJ 1061", hms(3, 35, 59.70), dms(-44, 30, 45.7), 11.9839},
        };

        // ------------------------------------------------------------------
        // THE STARS. Radii and masses are interferometric or asteroseismic
        // where they exist (Alpha Centauri A and B, Sirius A, Procyon A), and
        // otherwise from empirical M-dwarf relations. `sma` is the RELATIVE
        // orbit about the system's primary.
        // ------------------------------------------------------------------
        struct StarSeed
        {
            const char* name;
            const char* designation;
            u32 system;
            f64 radiusSolar;
            f64 massSolar;
            f64 luminositySolar;
            f64 temperature;
            f64 smaAu;
            f64 eccentricity;
        };

        const StarSeed kStarSeeds[] = {
            {"SOL", "G2V", 0, 1.0, 1.0, 1.0, 5772.0, 0.0, 0.0},
            {"PROXIMA", "M5.5Ve", 1, 0.1542, 0.1221, 0.001567, 2992.0, 0.0, 0.0},
            // The pair is a real 79.762-year orbit: a = 23.30 AU, e = 0.519.
            {"ALPHA CEN A", "G2V", 2, 1.2175, 1.0788, 1.5059, 5804.0, 0.0, 0.0},
            {"ALPHA CEN B", "K1V", 2, 0.8591, 0.9092, 0.4981, 5207.0, 23.30, 0.519},
            {"BARNARD", "M4V", 3, 0.187, 0.162, 0.00340, 3195.0, 0.0, 0.0},
            // Two brown dwarfs, the nearest of their kind. 27-year orbit.
            {"LUHMAN 16 A", "L7.5", 4, 0.102, 0.0338, 2.2e-5, 1305.0, 0.0, 0.0},
            {"LUHMAN 16 B", "T0.5", 4, 0.102, 0.0281, 2.1e-5, 1320.0, 3.50, 0.343},
            // Not a star at all: a rogue world of a few Jupiter masses and 276
            // kelvin, the coldest object of its kind ever found. It is in the
            // table because it is the fourth-nearest thing to Sol, and because
            // arriving at something that emits no light is worth doing once.
            {"WISE 0855", "Y4", 5, 0.1074, 0.0057, 6.03e-8, 276.0, 0.0, 0.0},
            {"WOLF 359", "M6V", 6, 0.144, 0.110, 0.00106, 2749.0, 0.0, 0.0},
            {"LALANDE 21185", "M2V", 7, 0.392, 0.389, 0.02194, 3547.0, 0.0, 0.0},
            // Sirius B is a white dwarf: a solar mass inside an Earth-sized
            // ball, twenty-five thousand kelvin, and a surface gravity four
            // hundred thousand times Terra's. 50.13-year orbit, e = 0.591.
            {"SIRIUS A", "A1V", 8, 1.7144, 2.063, 24.74, 9845.0, 0.0, 0.0},
            {"SIRIUS B", "DA2", 8, 0.008098, 1.018, 0.02448, 25000.0, 19.80, 0.591},
            {"BL CETI", "M5.5V", 9, 0.165, 0.122, 0.00147, 2784.0, 0.0, 0.0},
            {"UV CETI", "M6V", 9, 0.159, 0.116, 0.00125, 2728.0, 5.50, 0.620},
            {"ROSS 154", "M3.5Ve", 10, 0.200, 0.177, 0.004015, 3340.0, 0.0, 0.0},
            {"ROSS 248", "M5.5V", 11, 0.190, 0.145, 0.0022, 2930.0, 0.0, 0.0},
            {"EPSILON ERIDANI", "K2V", 12, 0.742, 0.82, 0.326, 5085.0, 0.0, 0.0},
            {"LACAILLE 9352", "M0.5V", 13, 0.474, 0.479, 0.0368, 3672.0, 0.0, 0.0},
            {"ROSS 128", "M4V", 14, 0.198, 0.176, 0.00366, 3189.0, 0.0, 0.0},
            // A triple: A and C are a 3.79-day spectroscopic pair, B goes
            // round both of them in 2.25 years.
            {"EZ AQR A", "M5Ve", 15, 0.175, 0.1216, 0.00078, 2900.0, 0.0, 0.0},
            {"EZ AQR C", "M dwarf", 15, 0.140, 0.0957, 0.00012, 2600.0, 0.030, 0.010},
            {"EZ AQR B", "M dwarf", 15, 0.210, 0.1145, 0.0019, 2650.0, 1.180, 0.437},
            // Bessel's star: the first parallax ever measured, 1838.
            {"61 CYGNI A", "K5V", 16, 0.667, 0.6771, 0.150, 4398.0, 0.0, 0.0},
            {"61 CYGNI B", "K7V", 16, 0.594, 0.6289, 0.097, 4174.0, 84.00, 0.480},
            {"PROCYON A", "F5IV-V", 17, 2.043, 1.478, 7.049, 6582.0, 0.0, 0.0},
            {"PROCYON B", "DQZ", 17, 0.01234, 0.592, 0.00049, 7740.0, 15.00, 0.407},
            {"STRUVE 2398 A", "M3V", 18, 0.351, 0.330, 0.01552, 3433.0, 0.0, 0.0},
            {"STRUVE 2398 B", "M3.5V", 18, 0.280, 0.250, 0.00916, 3379.0, 56.00, 0.220},
            {"GROOMBRIDGE 34 A", "M1.5V", 19, 0.385, 0.393, 0.02249, 3601.0, 0.0, 0.0},
            {"GROOMBRIDGE 34 B", "M3.5V", 19, 0.180, 0.150, 8.5e-4, 3304.0, 146.00, 0.300},
            {"DX CANCRI", "M6.5Ve", 20, 0.1235, 0.106, 0.00073, 2840.0, 0.0, 0.0},
            {"EPSILON INDI A", "K5Ve", 21, 0.713, 0.782, 0.21, 4649.0, 0.0, 0.0},
            // The B pair is fifteen hundred AU out — a third of a light-day.
            {"EPSILON INDI Ba", "T1V", 21, 0.0805, 0.0639, 2.04e-5, 1312.0, 1460.0, 0.100},
            {"EPSILON INDI Bb", "T6V", 21, 0.0825, 0.0508, 5.97e-6, 972.0, 1462.4, 0.100},
            {"TAU CETI", "G8V", 22, 0.793, 0.800, 0.488, 5320.0, 0.0, 0.0},
            {"GJ 1061", "M5.5V", 23, 0.152, 0.125, 0.001641, 2977.0, 0.0, 0.0},
        };

        // ------------------------------------------------------------------
        // THE PLANETS. Confirmed only. `massEarths` is a MINIMUM mass for
        // everything found by radial velocity, which is everything here bar
        // Epsilon Indi Ab (imaged) and Epsilon Eridani b (astrometry pins the
        // inclination). `radiusEarths` is measured for none of them — not one
        // of these worlds transits — so it is a mass-radius relation, and the
        // relation is written down in massRadius() below rather than baked
        // into the table, so that it can be argued with.
        //
        // LEFT OUT ON PURPOSE, each of them once announced: Proxima c (the
        // 1928-day signal did not survive reanalysis or NIRPS), Alpha Cen A
        // "S1" (a 2025 JWST candidate seen once and not recovered), Wolf 359 b
        // and c (b unconfirmed since 2019, c is the star's rotation), Kapteyn b
        // (the star's rotation over three), Tau Ceti b/c/d/e (none recovered),
        // Alpha Cen Bb (refuted 2015), Epsilon Eridani c (abandoned).
        // ------------------------------------------------------------------
        struct PlanetSeed
        {
            const char* name;
            u32 star;
            f64 massEarths;
            f64 smaAu;
            f64 eccentricity;
            f64 radiusEarthsOverride; // 0 = derive from the mass
            f64 airTopKm;             // 0 = airless
        };

        const PlanetSeed kPlanetSeeds[] = {
            // -- Proxima: two confirmed, one of them in the habitable zone --
            {"PROXIMA d", 1, 0.260, 0.02881, 0.0, 0.0, 0.0},
            {"PROXIMA b", 1, 1.055, 0.04848, 0.0, 0.0, 90.0},
            // -- Barnard's Star: four sub-Earths, MAROON-X + ESPRESSO 2025 --
            {"BARNARD d", 4, 0.263, 0.0188, 0.04, 0.0, 0.0},
            {"BARNARD b", 4, 0.299, 0.0229, 0.03, 0.0, 0.0},
            {"BARNARD c", 4, 0.335, 0.0274, 0.08, 0.0, 0.0},
            {"BARNARD e", 4, 0.193, 0.0381, 0.04, 0.0, 0.0},
            // -- Lalande 21185 -------------------------------------------
            {"GJ 411 b", 9, 2.69, 0.07879, 0.063, 0.0, 0.0},
            {"GJ 411 c", 9, 13.6, 2.94, 0.132, 0.0, 0.0},
            // -- Epsilon Eridani: a true Jupiter, inclination pinned -------
            {"AEGIR", 16, 317.8, 3.53, 0.06, 13.8, 0.0},
            // -- Lacaille 9352: four, and the fourth is in the zone --------
            {"GJ 887 e", 17, 1.46, 0.0417, 0.0, 0.0, 0.0},
            {"GJ 887 b", 17, 3.90, 0.0683, 0.14, 0.0, 0.0},
            {"GJ 887 c", 17, 6.50, 0.121, 0.17, 0.0, 0.0},
            {"GJ 887 d", 17, 6.10, 0.212, 0.25, 0.0, 240.0},
            // -- Ross 128 --------------------------------------------------
            {"ROSS 128 b", 18, 1.40, 0.0496, 0.116, 0.0, 110.0},
            // -- Struve 2398: one each, and B's is in the zone -------------
            {"GL 725 A b", 26, 2.78, 0.068, 0.0, 0.0, 0.0},
            {"GL 725 B c", 27, 3.40, 0.139, 0.0, 0.0, 200.0},
            // -- Groombridge 34 -------------------------------------------
            {"GJ 15 A b", 28, 3.03, 0.072, 0.094, 0.0, 0.0},
            {"GJ 15 A c", 28, 36.0, 5.40, 0.270, 0.0, 0.0},
            // -- Epsilon Indi: the nearest imaged planet, 275 K ------------
            {"EPSILON INDI Ab", 31, 2066.0, 15.76, 0.25, 11.6, 0.0},
            // -- Tau Ceti: the three that survived the 2017 reanalysis -----
            {"TAU CETI g", 34, 1.75, 0.133, 0.06, 0.0, 0.0},
            {"TAU CETI h", 34, 1.83, 0.243, 0.23, 0.0, 0.0},
            {"TAU CETI f", 34, 3.93, 1.334, 0.16, 0.0, 160.0},
            // -- GJ 1061: three, the outer one in the zone ----------------
            {"GJ 1061 b", 35, 1.11, 0.021, 0.05, 0.0, 0.0},
            {"GJ 1061 c", 35, 1.81, 0.0342, 0.02, 0.0, 0.0},
            {"GJ 1061 d", 35, 1.67, 0.054, 0.04, 0.0, 120.0},
        };

        /// Radius from mass, for worlds nobody has seen transit. Two regimes
        /// and one break: rock and iron compress slowly (R ~ M^0.27, Zeng's
        /// Earth-composition relation), and past about six Earth masses a
        /// planet keeps its hydrogen and the radius runs away (R ~ M^0.59,
        /// Chen & Kipping's sub-Neptune branch) until degeneracy flattens it
        /// at roughly one Jupiter radius. It is not a measurement and the
        /// table does not pretend it is.
        [[nodiscard]] f64 massRadius(f64 massEarths)
        {
            if (massEarths <= 6.0)
            {
                return std::pow(massEarths, 0.27);
            }
            if (massEarths <= 120.0)
            {
                const f64 breakRadius = std::pow(6.0, 0.27);
                return breakRadius * std::pow(massEarths / 6.0, 0.59);
            }
            return 11.2; // one Jupiter: degeneracy holds the radius flat
        }

        struct Catalogue
        {
            std::vector<SystemRecord> systems;
            std::vector<StarRecord> stars;
            std::vector<PlanetRecord> planets;
        };

        const Catalogue& catalogue()
        {
            static const Catalogue built = [] {
                Catalogue c{};

                // ---- systems: the anchors ------------------------------
                for (const SystemSeed& seed : kSystemSeeds)
                {
                    SystemRecord record{};
                    record.name = seed.name;
                    record.rightAscensionHours = seed.raHours;
                    record.declinationDegrees = seed.decDegrees;
                    record.distanceLightYears = seed.distanceLy;
                    record.position = equatorialToGame(seed.raHours, seed.decDegrees,
                                                       lightYears(seed.distanceLy));
                    c.systems.push_back(record);
                }

                // ---- stars ---------------------------------------------
                for (const StarSeed& seed : kStarSeeds)
                {
                    StarRecord record{};
                    record.name = seed.name;
                    record.designation = seed.designation;
                    record.systemIndex = seed.system;
                    record.radius = solarRadii(seed.radiusSolar);
                    record.mu = solarMasses(seed.massSolar);
                    record.luminosity = seed.luminositySolar;
                    record.temperature = seed.temperature;
                    record.sma = au(seed.smaAu);
                    record.eccentricity = seed.eccentricity;
                    // The pair's plane: the catalogue has one for a handful of
                    // these and nothing for the rest, so all of them get the
                    // system's own deterministic plane. Same sky every launch.
                    record.inclination =
                        0.5 * hashAngle(c.systems[seed.system].name, 11u);
                    record.ascendingNode = hashAngle(c.systems[seed.system].name, 12u);
                    record.meanAnomaly = hashAngle(seed.name, 13u);
                    c.stars.push_back(record);
                }

                // Each system's star span, in table order (the seeds are
                // grouped by system and the primary is first).
                for (u32 i = 0; i < c.stars.size(); ++i)
                {
                    SystemRecord& system = c.systems[c.stars[i].systemIndex];
                    if (system.starCount == 0)
                    {
                        system.firstStar = i;
                    }
                    ++system.starCount;
                }

                // ---- planets --------------------------------------------
                for (const PlanetSeed& seed : kPlanetSeeds)
                {
                    const StarRecord& host = c.stars[seed.star];
                    PlanetRecord record{};
                    record.name = seed.name;
                    record.starIndex = seed.star;
                    record.mu = earthMasses(seed.massEarths);
                    record.radius = earthRadii(seed.radiusEarthsOverride > 0.0
                                                   ? seed.radiusEarthsOverride
                                                   : massRadius(seed.massEarths));
                    record.sma = au(seed.smaAu);
                    record.eccentricity = seed.eccentricity;
                    // Coplanar within a system, because real systems are, and
                    // the plane is the system's — see the note on hashAngle.
                    record.inclination =
                        0.5 * hashAngle(c.systems[host.systemIndex].name, 11u);
                    record.ascendingNode =
                        hashAngle(c.systems[host.systemIndex].name, 12u);
                    record.meanAnomaly = hashAngle(seed.name, 7u);
                    record.equilibriumTemperature =
                        equilibriumTemperature(host.luminosity, record.sma);
                    record.surfaceStyle =
                        styleForWorld(seed.massEarths, record.equilibriumTemperature);
                    record.atmosphereTopAltitude = seed.airTopKm * 1000.0;
                    record.soiRadius =
                        record.sma * std::pow(record.mu / host.mu, 0.4);
                    c.planets.push_back(record);
                }

                // ---- spheres of influence --------------------------------
                // A COMPANION's SOI is the ordinary formula against its
                // primary. A PRIMARY's is the system's, and that is a
                // different question with no textbook answer: what bounds a
                // star's gravitational reach is the Galaxy on one side and its
                // nearest neighbour on the other. Both are used, whichever
                // is smaller.
                //
                // The galactic bound is the Hill radius against the mass
                // interior to Sol's orbit, which for Sol is a shade under two
                // light-years and scales as the cube root of the mass. The
                // neighbour bound is 45% of the distance to the nearest other
                // anchor, which keeps a gap of genuinely interstellar space
                // between every pair of systems in the table — and that gap is
                // where the top of the warp ladder unlocks, so it has to
                // exist even between Alpha Centauri and Proxima, which are
                // only a fifth of a light-year apart and really are bound to
                // each other.
                for (u32 i = 0; i < c.stars.size(); ++i)
                {
                    StarRecord& star = c.stars[i];
                    const SystemRecord& system = c.systems[star.systemIndex];
                    if (star.sma > 0.0)
                    {
                        const StarRecord& primary = c.stars[system.firstStar];
                        star.soiRadius = star.sma * std::pow(star.mu / primary.mu, 0.4);
                        continue;
                    }
                    f64 nearest = 1.0e30;
                    for (const SystemRecord& other : c.systems)
                    {
                        if (&other == &system)
                        {
                            continue;
                        }
                        nearest = std::min(nearest,
                                           glm::length(other.position - system.position));
                    }
                    const f64 galactic =
                        lightYears(1.9) * std::cbrt(star.mu / kMuSun);
                    star.soiRadius = std::min(galactic, 0.45 * nearest);
                }
                return c;
            }();
            return built;
        }
    } // namespace

    WorldVec3 equatorialToGame(f64 rightAscensionHours, f64 declinationDegrees,
                               f64 distanceMetres)
    {
        const f64 ra = rightAscensionHours * (3.141592653589793 / 12.0);
        const f64 dec = declinationDegrees * (3.141592653589793 / 180.0);
        // Equatorial unit vector: +x at the equinox, +z at the celestial pole.
        const f64 ex = std::cos(dec) * std::cos(ra);
        const f64 ey = std::cos(dec) * std::sin(ra);
        const f64 ez = std::sin(dec);
        // Rotate about +x by the obliquity: the equinox is the shared axis.
        const f64 cosE = std::cos(kObliquityJ2000);
        const f64 sinE = std::sin(kObliquityJ2000);
        const f64 lx = ex;
        const f64 ly = ey * cosE + ez * sinE;
        const f64 lz = -ey * sinE + ez * cosE;
        // Relabel to the game's frame: +Y is the pole. (x, z, -y) keeps the
        // determinant at +1 — see the header.
        return WorldVec3{lx, lz, -ly} * distanceMetres;
    }

    f64 equilibriumTemperature(f64 luminosity, f64 smaMetres)
    {
        if (smaMetres <= 0.0)
        {
            return 0.0;
        }
        // T = 278.6 K * (L/Lsun)^(1/4) / sqrt(a/AU), times (1-A)^(1/4) for a
        // Bond albedo of 0.3 — 278.6 * 0.916 = 255.2, which is Terra's own
        // 255 K and the reason that number is the one everybody quotes.
        const f64 auDistance = smaMetres / kAstronomicalUnit;
        return 255.2 * std::pow(luminosity, 0.25) / std::sqrt(auDistance);
    }

    i32 styleForWorld(f64 massEarths, f64 equilibriumKelvin)
    {
        // Past about ten Earth masses a world keeps its hydrogen and what you
        // see is an envelope, not a ground. Which envelope is temperature:
        // ammonia clouds are white, methane absorbs red and goes blue.
        if (massEarths > 10.0)
        {
            if (massEarths > 150.0)
            {
                return (equilibriumKelvin > 95.0) ? 20 : 21; // Jupiter, Saturn
            }
            return (equilibriumKelvin > 95.0) ? 23 : 22; // Neptune, Uranus
        }
        // A ground, and the thresholds are the solar system's OWN equilibrium
        // temperatures — Mercury 440, Mars 210, Callisto 134, Ganymede 110,
        // Europa 102, Enceladus 75, Triton 38 — so every band is anchored to a
        // body the game already draws at that temperature rather than to a
        // round number. The bands are the midpoints between them.
        if (equilibriumKelvin > 700.0)
        {
            return 4; // Io: hot enough that the surface is molten in places
        }
        if (equilibriumKelvin > 340.0)
        {
            return 3; // Mercury: baked rock, no air to move the heat around
        }
        if (equilibriumKelvin > 195.0)
        {
            return 2; // Mars: iron oxide and dust, the temperate case
        }
        if (equilibriumKelvin > 120.0)
        {
            return 7; // Callisto: old cratered ice and rock
        }
        if (equilibriumKelvin > 95.0)
        {
            return 6; // Ganymede: grooved terrain
        }
        if (equilibriumKelvin > 70.0)
        {
            return 5; // Europa: cracked water ice
        }
        if (equilibriumKelvin > 45.0)
        {
            return 9; // Enceladus: clean bright ice
        }
        return 13; // Triton: nitrogen frost
    }

    std::span<const SystemRecord> systems() { return catalogue().systems; }
    std::span<const StarRecord> stars() { return catalogue().stars; }
    std::span<const PlanetRecord> planets() { return catalogue().planets; }

    WorldVec3 starPositionAt(u32 starIndex, f64 timeSeconds)
    {
        const Catalogue& c = catalogue();
        const StarRecord& star = c.stars[starIndex];
        const WorldVec3 anchor = c.systems[star.systemIndex].position;
        if (star.sma <= 0.0)
        {
            return anchor;
        }
        const StarRecord& primary = c.stars[c.systems[star.systemIndex].firstStar];
        const phys::KeplerOrbit orbit = phys::kepler::fromElements(
            primary.mu, star.sma, star.eccentricity, star.inclination,
            star.ascendingNode, 0.0, star.meanAnomaly, 0.0);
        WorldVec3 relative{};
        phys::kepler::evaluate(orbit, timeSeconds, relative);
        return anchor + relative;
    }

    u32 nearestSystem(const WorldVec3& position)
    {
        const Catalogue& c = catalogue();
        u32 best = 0;
        f64 bestDistanceSq = 1.0e300;
        for (u32 i = 0; i < c.systems.size(); ++i)
        {
            const WorldVec3 delta = position - c.systems[i].position;
            const f64 distanceSq = glm::dot(delta, delta);
            if (distanceSq < bestDistanceSq)
            {
                bestDistanceSq = distanceSq;
                best = i;
            }
        }
        return best;
    }

    i32 containingSystem(const WorldVec3& position)
    {
        const Catalogue& c = catalogue();
        for (u32 i = 0; i < c.systems.size(); ++i)
        {
            const StarRecord& primary = c.stars[c.systems[i].firstStar];
            const WorldVec3 delta = position - c.systems[i].position;
            if (glm::dot(delta, delta) <= primary.soiRadius * primary.soiRadius)
            {
                return static_cast<i32>(i);
            }
        }
        return -1;
    }
} // namespace sw::space
