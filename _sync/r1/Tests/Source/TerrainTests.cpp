// ============================================================================
// TerrainTests.cpp — the heightfield v2 contract (M25).
//
// The terrain function is the single most load-bearing function in the
// project: physics collides with it, the renderer draws it, saved bases
// stand on it. These tests pin down the four promises it makes.
//
//   1. DETERMINISM — same direction, same parameters, same meters, forever.
//   2. COASTLINES ARE PRESERVED — v2 adds relief on top of the v1
//      continental mask and may NOT move a single shoreline: the world map,
//      the launch site and every save ever written depend on it.
//   3. SEA LEVEL IS SOLID — oceans clamp to 0 for everything that touches
//      the surface, while the signed form exposes the real sea floor.
//   4. IT IS A LANDSCAPE — real crests, real valleys, bounded slopes.
// ============================================================================

#include "TestFramework.hpp"

#include <Planet/Terrain.hpp>

#include <cmath>

namespace
{
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
        return glm::normalize(
            sw::Vec3{r * std::cos(phi), z, r * std::sin(phi)});
    }

    /// The v1 (pre-M25) elevation, reproduced here verbatim. It is the
    /// reference the coastline test compares against: v2 is allowed to
    /// change every altitude on the planet, and no land/sea decision.
    sw::f32 legacyElevationV1(const sw::planet::TerrainComponent& terrain,
                              const sw::Vec3& dir)
    {
        const sw::f32 n = sw::math::fbm3(dir * terrain.frequency + terrain.noiseOffset,
                                         terrain.octaves, terrain.seed);
        const sw::f32 land =
            (n - terrain.seaLevelFraction) / (1.0f - terrain.seaLevelFraction);
        return (land <= 0.0f) ? 0.0f : land * land * terrain.amplitude;
    }

    constexpr int kSamples = 10000;
} // namespace

SW_TEST(TerrainIsDeterministic)
{
    const sw::planet::TerrainComponent terra = sw::planet::presetTerra();
    for (int i = 0; i < 200; ++i)
    {
        const sw::Vec3 dir = sampleDirection(static_cast<sw::u32>(i));
        const sw::f64 a = sw::planet::terrainElevation(terra, dir);
        const sw::f64 b = sw::planet::terrainElevation(terra, dir);
        SW_CHECK_EQ(a, b);
    }
    // A different seed is a different world.
    sw::planet::TerrainComponent other = terra;
    other.seed = terra.seed + 1u;
    int differences = 0;
    for (int i = 0; i < 200; ++i)
    {
        const sw::Vec3 dir = sampleDirection(static_cast<sw::u32>(i));
        if (sw::planet::terrainElevation(terra, dir) !=
            sw::planet::terrainElevation(other, dir))
        {
            ++differences;
        }
    }
    SW_CHECK(differences > 150);
}

SW_TEST(TerrainV2PreservesEveryCoastline)
{
    // THE M25 CONTRACT: relief is added on top of the v1 continental mask,
    // so the land/sea classification is bit-identical to v1 everywhere.
    const sw::planet::TerrainComponent worlds[] = {sw::planet::presetTerra(),
                                                  sw::planet::presetLuna(),
                                                  sw::planet::presetMars()};
    for (const sw::planet::TerrainComponent& terrain : worlds)
    {
        int mismatches = 0;
        for (int i = 0; i < kSamples; ++i)
        {
            const sw::Vec3 dir = sampleDirection(static_cast<sw::u32>(i));
            const bool landV1 = legacyElevationV1(terrain, dir) > 0.0f;
            const bool landV2 = sw::planet::terrainElevation(terrain, dir) > 0.0;
            if (landV1 != landV2)
            {
                ++mismatches;
            }
        }
        SW_CHECK_EQ(mismatches, 0);
    }
}

SW_TEST(TerrainSeaLevelIsSolidAndTheSeaFloorIsBelowIt)
{
    const sw::planet::TerrainComponent terra = sw::planet::presetTerra();
    int oceanPoints = 0;
    for (int i = 0; i < kSamples; ++i)
    {
        const sw::Vec3 dir = sampleDirection(static_cast<sw::u32>(i));
        const sw::f32 signedElevation =
            sw::planet::terrainElevationSigned(terra, dir);
        const sw::f64 clamped = sw::planet::terrainElevation(terra, dir);

        // Nothing the surface touches is ever below sea level...
        SW_CHECK(clamped >= 0.0);
        // ...and above water the two forms agree exactly.
        if (signedElevation > 0.0f)
        {
            SW_CHECK_EQ(clamped, static_cast<sw::f64>(signedElevation));
        }
        else
        {
            ++oceanPoints;
            SW_CHECK_EQ(clamped, 0.0);
            // Bathymetry stays inside the declared abyssal depth.
            SW_CHECK(signedElevation >= -terra.oceanDepth * 1.001f);
        }
        // Peaks stay inside the declared amplitude.
        SW_CHECK(signedElevation <= terra.amplitude * 1.001f);
    }
    // Terra is an ocean world: a good third of it must be water.
    SW_CHECK(oceanPoints > kSamples / 4);
}

SW_TEST(TerrainAirlessWorldsHaveNoOcean)
{
    const sw::planet::TerrainComponent worlds[] = {sw::planet::presetLuna(),
                                                  sw::planet::presetMars()};
    for (const sw::planet::TerrainComponent& terrain : worlds)
    {
        SW_CHECK_EQ(terrain.seaLevelFraction, 0.0f);
        SW_CHECK_EQ(terrain.oceanDepth, 0.0f);
        for (int i = 0; i < 2000; ++i)
        {
            const sw::Vec3 dir = sampleDirection(static_cast<sw::u32>(i));
            SW_CHECK(sw::planet::terrainElevationSigned(terrain, dir) >= 0.0f);
        }
    }
}

SW_TEST(TerrainV2IsARealLandscape)
{
    // v1 could not produce this: its land^2 profile put almost everything in
    // the lowest tenth of the amplitude. v2 must show mountains (a
    // meaningful share of high ground) AND keep them rare.
    const sw::planet::TerrainComponent terra = sw::planet::presetTerra();
    int high = 0;
    int veryHigh = 0;
    int land = 0;
    sw::f32 peak = 0.0f;
    for (int i = 0; i < kSamples; ++i)
    {
        const sw::Vec3 dir = sampleDirection(static_cast<sw::u32>(i));
        const sw::f32 elevation = sw::planet::terrainElevationSigned(terra, dir);
        if (elevation <= 0.0f)
        {
            continue;
        }
        ++land;
        peak = std::max(peak, elevation);
        if (elevation > 0.30f * terra.amplitude)
        {
            ++high;
        }
        if (elevation > 0.60f * terra.amplitude)
        {
            ++veryHigh;
        }
    }
    SW_CHECK(land > 1000);
    SW_CHECK(peak > 0.70f * terra.amplitude); // real summits exist
    SW_CHECK(high > land / 50);               // ranges cover part of the land
    SW_CHECK(veryHigh < land / 3);            // but the world is not a spike field
}

SW_TEST(TerrainIsContinuousAndWalkable)
{
    // No cliffs of the "infinite wall" kind: over a 200 m step on the ground
    // the elevation must change by less than that step times a sane slope
    // bound (tan 80 deg ~ 5.7). This is what keeps a rover, a landing gear
    // and the collision solver honest.
    const sw::planet::TerrainComponent terra = sw::planet::presetTerra();
    constexpr sw::f64 kBodyRadius = 6.371e6;
    constexpr sw::f64 kStepMeters = 200.0;
    const sw::f32 epsilon = static_cast<sw::f32>(kStepMeters / kBodyRadius);
    sw::f64 worstSlope = 0.0;
    for (int i = 0; i < 3000; ++i)
    {
        const sw::Vec3 dir = sampleDirection(static_cast<sw::u32>(i) + 991u);
        const sw::Vec3 reference =
            std::abs(dir.y) < 0.95f ? sw::Vec3{0, 1, 0} : sw::Vec3{1, 0, 0};
        const sw::Vec3 tangent = glm::normalize(glm::cross(reference, dir));
        const sw::f64 here = sw::planet::terrainElevation(terra, dir);
        const sw::f64 there = sw::planet::terrainElevation(
            terra, glm::normalize(dir + tangent * epsilon));
        worstSlope = std::max(worstSlope, std::abs(there - here) / kStepMeters);
    }
    SW_CHECK(worstSlope < 5.7);
}

SW_TEST(TerrainPresetsAreStableAndTheLaunchSiteDoesNotMove)
{
    // "The pad stays where it was" is a v2 acceptance criterion: the launch
    // site sits at +Z in Terra's body frame and its GROUND HEIGHT must be
    // exactly what v1 gave, or every saved vessel standing on it would sink
    // or float after this milestone.
    const sw::planet::TerrainComponent terra = sw::planet::presetTerra();
    const sw::Vec3 padDirection{0.0f, 0.0f, 1.0f};
    SW_CHECK_EQ(sw::planet::terrainElevation(terra, padDirection),
                static_cast<sw::f64>(legacyElevationV1(terra, padDirection)));

    // Presets are addressable by style id exactly as the GLSL twin expects.
    SW_CHECK_EQ(sw::planet::terrainPreset(0).seed, sw::planet::presetTerra().seed);
    SW_CHECK_EQ(sw::planet::terrainPreset(1).seed, sw::planet::presetLuna().seed);
    SW_CHECK_EQ(sw::planet::terrainPreset(2).seed, sw::planet::presetMars().seed);
}

// ---------------------------------------------------------------------------
// 5. THE DRAWN GROUND IS THE GROUND YOU STAND ON
//
// Collision samples the analytic heightfield exactly; the renderer draws a
// grid of samples with flat triangles stretched between them. Those two
// surfaces agree only as well as the grid is fine — and where they disagree
// the player sees the thing they cannot un-see, a rocket half-submerged in a
// hillside it is, as far as physics is concerned, resting neatly on top of.
//
// The heightfield is a RIDGED fractal: it has creases, so the error of a
// chord across one cell falls roughly with the cell width rather than with
// its square. That is what makes this a resolution contract and not a
// tuning knob, and it is why the terrain patch's level of detail must be
// chosen from height above the GROUND and never from height above the sea.
// Standing on Terra's 1,100 m launch plateau, the sea-level reading asked
// for a 137 m grid; the ground-level reading asks for a 15.6 m one.
// ---------------------------------------------------------------------------

namespace
{
    /// Worst signed gap, in metres, between the analytic surface and a mesh
    /// of `cells` square that samples it and interpolates linearly between.
    /// Negative means the true ground is BELOW the drawn one — you sink.
    struct MeshGap
    {
        sw::f64 sink = 0.0;  // metres the collider sits below the mesh
        sw::f64 hover = 0.0; // ...and above it
    };

    [[nodiscard]] MeshGap meshGap(const sw::planet::TerrainComponent& terrain,
                                  const sw::Vec3& centreDir, sw::f64 extent,
                                  sw::u32 cells, sw::f64 radius)
    {
        const sw::Vec3 reference =
            (std::abs(centreDir.y) < 0.99f) ? sw::Vec3{0, 1, 0} : sw::Vec3{1, 0, 0};
        const sw::Vec3 east = glm::normalize(glm::cross(reference, centreDir));
        const sw::Vec3 north = glm::cross(centreDir, east);

        const auto surfaceRadius = [&](sw::f64 u, sw::f64 v) {
            const sw::WorldVec3 raw = sw::WorldVec3(centreDir) * radius +
                                      sw::WorldVec3(east) * u + sw::WorldVec3(north) * v;
            const sw::Vec3 dir = sw::Vec3(glm::normalize(raw));
            return radius + sw::planet::terrainElevation(terrain, dir);
        };

        const sw::u32 verts = cells + 1;
        std::vector<sw::f64> grid(static_cast<sw::usize>(verts) * verts);
        for (sw::u32 j = 0; j < verts; ++j)
        {
            for (sw::u32 i = 0; i < verts; ++i)
            {
                grid[static_cast<sw::usize>(j) * verts + i] =
                    surfaceRadius((static_cast<sw::f64>(i) / cells * 2.0 - 1.0) * extent,
                                  (static_cast<sw::f64>(j) / cells * 2.0 - 1.0) * extent);
            }
        }

        MeshGap out{};
        for (sw::u32 j = 0; j < cells; ++j)
        {
            for (sw::u32 i = 0; i < cells; ++i)
            {
                const sw::f64 r00 = grid[static_cast<sw::usize>(j) * verts + i];
                const sw::f64 r10 = grid[static_cast<sw::usize>(j) * verts + i + 1];
                const sw::f64 r01 = grid[static_cast<sw::usize>(j + 1) * verts + i];
                const sw::f64 r11 = grid[static_cast<sw::usize>(j + 1) * verts + i + 1];
                for (sw::u32 b = 1; b < 3; ++b)
                {
                    for (sw::u32 a = 1; a < 3; ++a)
                    {
                        const sw::f64 fa = static_cast<sw::f64>(a) / 3.0;
                        const sw::f64 fb = static_cast<sw::f64>(b) / 3.0;
                        const sw::f64 drawn = (r00 * (1 - fa) + r10 * fa) * (1 - fb) +
                                              (r01 * (1 - fa) + r11 * fa) * fb;
                        const sw::f64 truth = surfaceRadius(
                            ((static_cast<sw::f64>(i) + fa) / cells * 2.0 - 1.0) * extent,
                            ((static_cast<sw::f64>(j) + fb) / cells * 2.0 - 1.0) *
                                extent);
                        out.sink = std::min(out.sink, truth - drawn);
                        out.hover = std::max(out.hover, truth - drawn);
                    }
                }
            }
        }
        return out;
    }
} // namespace

SW_TEST(TheDrawnGroundFollowsTheColliderWhenTheCellsAreFineEnough)
{
    const sw::planet::TerrainComponent terra = sw::planet::presetTerra();
    constexpr sw::f64 kTerraRadius = 6.371e6;
    // A patch of real, creased terrain — not a smooth plain, which would
    // pass at any resolution and prove nothing.
    const sw::Vec3 site = glm::normalize(sw::Vec3(0.31f, 0.74f, 0.60f));

    // 15.6 m cells: what the patch builds at landing extent.
    const MeshGap fine = meshGap(terra, site, 500.0, 64, kTerraRadius);
    SW_CHECK(-fine.sink < 0.6);
    SW_CHECK(fine.hover < 2.6);

    // 137 m cells: what the level of detail asked for while it was reading
    // altitude from SEA level on an 1,100 m plateau. Measured across the
    // whole patch this is metres of disagreement, and a 2.4 m rocket
    // disappears into it.
    const MeshGap coarse = meshGap(terra, site, 4400.0, 64, kTerraRadius);
    SW_CHECK(-coarse.sink > 1.5);
    SW_CHECK(-coarse.sink > -fine.sink * 3.0);

    // The relationship is what matters and it must stay monotone: halving
    // the cell must not make the agreement worse.
    const MeshGap middle = meshGap(terra, site, 1000.0, 64, kTerraRadius);
    SW_CHECK(-fine.sink <= -middle.sink + 0.05);
    SW_CHECK(-middle.sink <= -coarse.sink + 0.05);
}
