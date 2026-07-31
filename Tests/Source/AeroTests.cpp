// ============================================================================
// AeroTests.cpp — the offline wind tunnel and the tables it produces.
//
// The forge is a numerical solver, and the only honest way to trust one is
// to point it at shapes whose answer is already in a textbook. A flat plate
// normal to the flow is Cd 1.17; a sphere is 0.47 subcritical and 0.92
// hypersonic; a slender cone is a fraction of either. If the solver lands
// on those, the fin it solves next is probably right too.
//
// The rest of the file guards the properties the RUNTIME depends on: the
// depth buffer really does hide the far side of a body, a table survives a
// trip through its own file, sampling wraps in azimuth without a seam, and a
// part standing in another part's wake stops collecting air.
// ============================================================================

#include "TestFramework.hpp"

#include <Core/FileSystem.hpp>
#include <ECS/World.hpp>
#include <Gameplay/AeroForge.hpp>
#include <Gameplay/Parts.hpp>
#include <Gameplay/VesselAerodynamics.hpp>
#include <Physics/Aerodynamics.hpp>
#include <Scene/TransformComponents.hpp>

#include <glm/gtx/quaternion.hpp> // glm::rotation, for attitude fixtures

#include <cmath>
#include <filesystem>
#include <vector>

using namespace sw;
using namespace sw::aero;

namespace
{
    /// A part made of exactly one authored primitive, at the origin.
    [[nodiscard]] parts::PartDefinition shapePart(parts::ShapeKind kind, const Vec3& size,
                                                  const Vec3& position = Vec3(0.0f),
                                                  const Vec3& rotationDeg = Vec3(0.0f))
    {
        parts::PartDefinition definition{};
        definition.id = 9001;
        definition.name = "probe";
        parts::PartShape shape{};
        shape.kind = kind;
        shape.size = size;
        shape.position = position;
        shape.rotationDeg = rotationDeg;
        shape.segments = 48; // a fine tessellation: the shape, not the facets
        shape.visible = true;
        shape.collider = true;
        definition.shapes.push_back(shape);
        return definition;
    }

    struct Measurement
    {
        f64 areaM2 = 0.0;
        f64 dragCoefficient = 0.0;
        f64 crossCoefficient = 0.0;
        Vec3 forceM2{0.0f};
        Vec3 momentM3{0.0f};
    };

    [[nodiscard]] Measurement blow(const parts::PartDefinition& definition, const Vec3& flow,
                                   bool liftingSurface = false, u32 resolution = 256)
    {
        ForgeSettings settings{};
        settings.resolution = resolution;
        settings.liftingSurface = liftingSurface;
        const std::vector<SkinTriangle> skin = partSkin(definition);
        const Vec3 direction = glm::normalize(flow);
        const SolvedDirection solved = solveDirection(skin, direction, settings);

        Measurement out{};
        out.areaM2 = solved.projectedAreaM2;
        out.forceM2 = solved.forceM2;
        out.momentM3 = solved.momentM3;
        if (out.areaM2 > 0.0)
        {
            const f32 along = glm::dot(solved.forceM2, direction);
            out.dragCoefficient = static_cast<f64>(along) / out.areaM2;
            out.crossCoefficient =
                static_cast<f64>(glm::length(solved.forceM2 - direction * along)) /
                out.areaM2;
        }
        return out;
    }

    [[nodiscard]] bool almost(f64 a, f64 b, f64 tolerance)
    {
        return std::abs(a - b) <= tolerance;
    }
} // namespace

// ---------------------------------------------------------------------------
// The solver, against shapes with published answers
// ---------------------------------------------------------------------------

SW_TEST(AFlatPlateMeetsTheTextbook)
{
    // 1 m x 1 m plate, 2 cm thick, square to the flow.
    const parts::PartDefinition plate =
        shapePart(parts::ShapeKind::Box, Vec3(0.5f, 0.5f, 0.01f));
    const Measurement measured = blow(plate, Vec3(0.0f, 0.0f, 1.0f));

    // The silhouette really is one square metre.
    SW_CHECK(almost(measured.areaM2, 1.0, 0.02));
    // Impact pressure on the front (1.0) plus base suction behind (0.20).
    // The measured figure for a square plate is 1.17.
    SW_CHECK(almost(measured.dragCoefficient, 1.20, 0.06));
    // Square to the flow: no side force at all.
    SW_CHECK(measured.crossCoefficient < 0.01);
    // ...and no moment, because the plate is centred on the origin.
    SW_CHECK(glm::length(measured.momentM3) < 0.01f);
}

SW_TEST(ASphereIsHalfThePlate)
{
    const parts::PartDefinition sphere =
        shapePart(parts::ShapeKind::Sphere, Vec3(0.5f, 0.5f, 0.5f));
    const Measurement measured = blow(sphere, Vec3(0.0f, 0.0f, 1.0f));

    SW_CHECK(almost(measured.areaM2, math::kPi * 0.25, 0.02)); // pi r^2, r = 0.5
    // Newtonian theory gives exactly Cp_max/2 over a sphere; with the wake
    // term that is 0.60, between the subcritical 0.47 and the hypersonic
    // 0.92 a real sphere shows at the two ends of its range.
    SW_CHECK(almost(measured.dragCoefficient, 0.60, 0.05));
    SW_CHECK(measured.crossCoefficient < 0.02);
}

SW_TEST(ASharpNoseIsCheaperThanABluntOne)
{
    // Same base radius, same length: one a cone, one a cylinder.
    const parts::PartDefinition cone =
        shapePart(parts::ShapeKind::Cone, Vec3(0.0f, 1.0f, 0.5f));
    const parts::PartDefinition cylinder =
        shapePart(parts::ShapeKind::Cylinder, Vec3(0.5f, 1.0f, 0.0f));

    // The cone's apex is at -Z, so the air must arrive from -Z to meet it
    // point-first: flow travels toward +Z.
    const Measurement pointed = blow(cone, Vec3(0.0f, 0.0f, 1.0f));
    const Measurement blunt = blow(cylinder, Vec3(0.0f, 0.0f, 1.0f));

    SW_CHECK(almost(pointed.areaM2, blunt.areaM2, 0.03)); // same silhouette
    // A 14-degree half-angle cone: sin^2 of that is 0.06, and the flat base
    // behind it collects the wake. The bluff cylinder pays full price.
    SW_CHECK(pointed.dragCoefficient < blunt.dragCoefficient * 0.4);
    SW_CHECK(pointed.dragCoefficient > 0.15); // the base is still there
}

SW_TEST(TheSolverNeverSeesTheFarSide)
{
    // Nose-on, a body twice as long is NOT twice as draggy: everything
    // behind the front face is in its own shadow. Only skin friction, which
    // scales with length, may differ — and it is a few per cent.
    const parts::PartDefinition shortBox =
        shapePart(parts::ShapeKind::Box, Vec3(0.5f, 0.5f, 0.5f));
    const parts::PartDefinition longBox =
        shapePart(parts::ShapeKind::Box, Vec3(0.5f, 0.5f, 2.0f));

    const Measurement small = blow(shortBox, Vec3(0.0f, 0.0f, 1.0f));
    const Measurement large = blow(longBox, Vec3(0.0f, 0.0f, 1.0f));

    SW_CHECK(almost(small.areaM2, large.areaM2, 0.02));
    SW_CHECK(almost(small.dragCoefficient, large.dragCoefficient, 0.06));
}

SW_TEST(AFinLiftsAcrossTheFlowAndThenStalls)
{
    // A thin plate lying in the XZ plane — its normal is Y, so a flow tilted
    // out of that plane must push it back toward the plane.
    //
    // ONE SQUARE METRE OF PLANFORM. Every force below is therefore also the
    // coefficient, which is what makes them comparable with a textbook.
    const parts::PartDefinition fin =
        shapePart(parts::ShapeKind::Box, Vec3(0.5f, 0.01f, 0.5f));

    const Measurement straight = blow(fin, Vec3(0.0f, 0.0f, 1.0f), true);
    SW_CHECK(straight.crossCoefficient < 0.02); // edge-on: nothing across

    const auto atIncidence = [&fin](f32 degrees) {
        const f32 alpha = math::toRadians(degrees);
        return blow(fin, Vec3(0.0f, std::sin(alpha), std::cos(alpha)), true);
    };

    const Measurement gentle = atIncidence(6.0f);
    const Measurement peak = atIncidence(12.0f);
    const Measurement stalled = atIncidence(35.0f);

    // WHICH WAY. The air arrives with a +Y component, so the plate is pushed
    // along +Y: the lift is on the downwind side, which is what makes a fin
    // at the tail a restoring force rather than a runaway one.
    SW_CHECK(gentle.forceM2.y > 0.0f);
    SW_CHECK(peak.forceM2.y > 0.0f);

    // NEAR-LINEAR before the stall, which is the whole reason the solver
    // carries a linear term: doubling the incidence roughly doubles the
    // force. Impact pressure alone would have QUADRUPLED it, from a base a
    // thirtieth as large, and a fin built on that never stabilises anything.
    const f32 gentleLift = std::abs(gentle.forceM2.y);
    const f32 peakLift = std::abs(peak.forceM2.y);
    SW_CHECK(gentleLift > 0.15f);
    SW_CHECK(peakLift > gentleLift * 1.6f);
    SW_CHECK(peakLift < gentleLift * 2.6f);

    // ...AND A STALL AFTER IT. Past the angle at which the flow can stay
    // attached the linear term fades out and impact pressure has the surface
    // to itself, so the force ACROSS the wind falls even though the plate is
    // now presenting far more area to it.
    SW_CHECK(std::abs(stalled.forceM2.y) < peakLift);
}

SW_TEST(TheForgeTellsAWingFromABody)
{
    // The decision that keeps the linear term where it belongs. It is made
    // from proportions alone: nothing here knows what a fin is called.
    ForgeSettings settings{};
    settings.resolution = 64;
    settings.thetaCount = 5;
    settings.phiCount = 4;

    const parts::PartDefinition plate =
        shapePart(parts::ShapeKind::Box, Vec3(0.9f, 0.05f, 0.7f));
    const parts::PartDefinition tank =
        shapePart(parts::ShapeKind::Cylinder, Vec3(1.2f, 2.1f, 0.0f));

    const f32 alpha = math::toRadians(10.0f);
    const Vec3 flow(0.0f, std::sin(alpha), std::cos(alpha));

    // Referred to each part's own frontal area, the plate must produce far
    // more force ACROSS the wind than the tank does — the tank is a body,
    // and a body at ten degrees mostly just pushes air out of the way.
    const AeroTable plateTable = forgePart(plate, settings);
    const AeroTable tankTable = forgePart(tank, settings);
    SW_CHECK(plateTable.valid());
    SW_CHECK(tankTable.valid());

    const Measurement plateWing = blow(plate, flow, true);
    const Measurement plateBody = blow(plate, flow, false);
    SW_CHECK(plateWing.crossCoefficient > plateBody.crossCoefficient * 3.0);

    // And the tank's solved table must be the BODY one — if the forge had
    // called it a wing, every rocket in the game would fly like a dart made
    // of wings.
    const AeroSample tankSolved = sample(tankTable, glm::normalize(flow));
    const Measurement tankBody = blow(tank, flow, false);
    SW_CHECK(glm::length(tankSolved.forceM2 - tankBody.forceM2) <
             glm::length(tankBody.forceM2) * 0.25f);
}

SW_TEST(TheMomentIsTakenAboutThePartOrigin)
{
    // The same plate, moved a metre along +X. The force must not change;
    // the moment must become exactly r x F.
    const parts::PartDefinition centred =
        shapePart(parts::ShapeKind::Box, Vec3(0.5f, 0.5f, 0.01f));
    const parts::PartDefinition offset = shapePart(
        parts::ShapeKind::Box, Vec3(0.5f, 0.5f, 0.01f), Vec3(1.0f, 0.0f, 0.0f));

    const Measurement a = blow(centred, Vec3(0.0f, 0.0f, 1.0f));
    const Measurement b = blow(offset, Vec3(0.0f, 0.0f, 1.0f));

    SW_CHECK(glm::length(a.forceM2 - b.forceM2) < 0.02f);
    const Vec3 expected = glm::cross(Vec3(1.0f, 0.0f, 0.0f), b.forceM2);
    SW_CHECK(glm::length(b.momentM3 - expected) < 0.03f);
    SW_CHECK(glm::length(b.momentM3) > 0.5f); // and it is a real moment
}

SW_TEST(AnAxisymmetricPartAnswersTheSameAllTheWayRound)
{
    const parts::PartDefinition tank =
        shapePart(parts::ShapeKind::Cylinder, Vec3(0.6f, 1.5f, 0.0f));
    const Measurement alongX = blow(tank, Vec3(1.0f, 0.0f, 0.0f));
    const Measurement alongY = blow(tank, Vec3(0.0f, 1.0f, 0.0f));

    SW_CHECK(almost(alongX.areaM2, alongY.areaM2, 0.02));
    SW_CHECK(almost(alongX.dragCoefficient, alongY.dragCoefficient, 0.02));
}

// ---------------------------------------------------------------------------
// The table: sampling and the file
// ---------------------------------------------------------------------------

SW_TEST(SamplingReturnsTheSolvedNodeExactly)
{
    AeroTable table{};
    table.partId = 1;
    table.thetaCount = 5;
    table.phiCount = 4;
    table.samples.resize(20);
    for (u32 t = 0; t < 5; ++t)
    {
        for (u32 p = 0; p < 4; ++p)
        {
            table.samples[t * 4 + p].forceM2 =
                Vec3(static_cast<f32>(t), static_cast<f32>(p), 0.0f);
        }
    }
    SW_CHECK(table.valid());

    // theta node 2 of 5 is 90 degrees; phi node 1 of 4 is 90 degrees.
    const AeroSample s = sample(table, Vec3(0.0f, 1.0f, 0.0f));
    SW_CHECK(almost(s.forceM2.x, 2.0, 1.0e-4));
    SW_CHECK(almost(s.forceM2.y, 1.0, 1.0e-4));
}

SW_TEST(SamplingHasNoSeamInAzimuth)
{
    // The last phi node wraps onto the first. A table read just before and
    // just after the seam must agree — a discontinuity here is a kick the
    // player feels as the vehicle rolls through north.
    AeroTable table{};
    table.partId = 1;
    table.thetaCount = 3;
    table.phiCount = 8;
    table.samples.resize(24);
    for (u32 t = 0; t < 3; ++t)
    {
        for (u32 p = 0; p < 8; ++p)
        {
            const f32 phi = 2.0f * math::kPi * static_cast<f32>(p) / 8.0f;
            table.samples[t * 8 + p].forceM2 = Vec3(std::cos(phi), std::sin(phi), 1.0f);
        }
    }

    const f32 epsilon = 1.0e-3f;
    const AeroSample before = sample(
        table, Vec3(std::cos(-epsilon), std::sin(-epsilon), 0.0f));
    const AeroSample after =
        sample(table, Vec3(std::cos(epsilon), std::sin(epsilon), 0.0f));
    SW_CHECK(glm::length(before.forceM2 - after.forceM2) < 0.01f);
}

SW_TEST(ATableSurvivesItsOwnFile)
{
    const parts::PartDefinition fin =
        shapePart(parts::ShapeKind::Box, Vec3(0.4f, 0.02f, 0.6f));
    ForgeSettings settings{};
    settings.resolution = 64;
    settings.thetaCount = 7;
    settings.phiCount = 6;
    const AeroTable written = forgePart(fin, settings);
    SW_CHECK(written.valid());

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "sw_aero_roundtrip.aero.json";
    SW_CHECK(saveAeroTable(written, path));

    AeroTable read{};
    SW_CHECK(loadAeroTable(path, read));
    SW_CHECK_EQ(read.partId, written.partId);
    SW_CHECK_EQ(read.thetaCount, written.thetaCount);
    SW_CHECK_EQ(read.phiCount, written.phiCount);
    SW_CHECK_EQ(read.samples.size(), written.samples.size());

    f32 worst = 0.0f;
    for (usize i = 0; i < read.samples.size(); ++i)
    {
        worst = std::max(worst, glm::length(read.samples[i].forceM2 -
                                            written.samples[i].forceM2));
        worst = std::max(worst, glm::length(read.samples[i].momentM3 -
                                            written.samples[i].momentM3));
    }
    // Six significant digits: the file is smaller than the double it came
    // from and closer than anything the physics can tell apart.
    SW_CHECK(worst < 1.0e-5f);
    std::filesystem::remove(path);
}

SW_TEST(TheShippedCatalogueHasBeenThroughTheTunnel)
{
    const std::filesystem::path assets =
        FileSystem::executableDirectory() / "Assets" / "Parts";
    const std::vector<AeroTable> tables = loadAeroTables(assets);
    SW_CHECK(tables.size() >= 9); // one per vessel part
    for (const AeroTable& table : tables)
    {
        SW_CHECK(table.valid());
        SW_CHECK(table.partId != 0);
        SW_CHECK(table.maxAreaM2 > 0.0);
        // Head-on, every shipped part pushes DOWNWIND. A table with the
        // sign flipped would accelerate a rocket through the atmosphere,
        // which is a fault worth one line of test.
        const AeroSample nose = sample(table, Vec3(0.0f, 0.0f, 1.0f));
        SW_CHECK(nose.forceM2.z > 0.0f);
    }
}

// ---------------------------------------------------------------------------
// The air
// ---------------------------------------------------------------------------

SW_TEST(TheAirThinsOutAndCoolsWithHeight)
{
    phys::AtmosphereComponent terra{};
    SW_CHECK(almost(density(terra, 0.0), 1.225, 1.0e-6));
    SW_CHECK(almost(density(terra, terra.scaleHeight), 1.225 / std::exp(1.0), 1.0e-6));
    SW_CHECK_EQ(density(terra, terra.topAltitude + 1.0), 0.0);
    SW_CHECK(density(terra, -50.0) == density(terra, 0.0)); // below sea level

    SW_CHECK(temperature(terra, 0.0) > temperature(terra, 10000.0));
    SW_CHECK(speedOfSound(terra, 0.0) > 300.0);
    SW_CHECK(speedOfSound(terra, 0.0) < 350.0);
    // Colder air upstairs carries sound more slowly — which is why a rocket
    // goes supersonic at a lower true airspeed the higher it climbs.
    SW_CHECK(speedOfSound(terra, 11000.0) < speedOfSound(terra, 0.0));
}

SW_TEST(DragSpikesThroughTheSoundBarrier)
{
    SW_CHECK(almost(machDragFactor(0.0), 1.0, 1.0e-9));
    SW_CHECK(almost(machDragFactor(0.5), 1.0, 1.0e-9));
    SW_CHECK(machDragFactor(1.1) > 1.4);
    SW_CHECK(machDragFactor(1.15) > machDragFactor(0.9));
    SW_CHECK(machDragFactor(4.0) < machDragFactor(1.15)); // the shock lies down
    SW_CHECK(machDragFactor(50.0) > 1.0);                 // and never goes away
    // Monotone between the knots — a jagged curve would make the throttle
    // fight itself around Mach 1.
    f64 previous = machDragFactor(0.0);
    for (f64 m = 0.0; m <= 1.15; m += 0.01)
    {
        const f64 current = machDragFactor(m);
        SW_CHECK(current >= previous - 1.0e-9);
        previous = current;
    }
}

SW_TEST(TheWindLeavesTheGroundAlone)
{
    phys::AtmosphereComponent terra{};
    const WorldVec3 up{0.0, 1.0, 0.0};
    const WorldVec3 east{1.0, 0.0, 0.0};

    SW_CHECK(glm::length(windVelocity(terra, 0.0, up, east, 0.0)) < 0.01);
    const f64 jet = glm::length(windVelocity(terra, 1.3 * terra.scaleHeight, up, east, 0.0));
    SW_CHECK(jet > 20.0);
    SW_CHECK(jet < 45.0);
    // Above the atmosphere there is nothing to blow.
    SW_CHECK_EQ(glm::length(windVelocity(terra, terra.topAltitude, up, east, 0.0)), 0.0);
    // Always horizontal: a vertical wind would lift a rocket off the pad.
    const WorldVec3 sample = windVelocity(terra, 5000.0, up, east, 1234.0);
    SW_CHECK(std::abs(glm::dot(sample, up)) < 1.0e-9);
    // Deterministic: the same second gives the same air, forever.
    SW_CHECK_EQ(glm::length(windVelocity(terra, 5000.0, up, east, 1234.0) - sample), 0.0);
}

// ---------------------------------------------------------------------------
// Occlusion and damping
// ---------------------------------------------------------------------------

SW_TEST(APartInAnothersWakeStopsCollectingAir)
{
    // Two identical boxes in line along Z, the air travelling toward +Z:
    // the one at -Z is in front and the one at +Z is behind it.
    std::vector<OccluderBox> boxes;
    boxes.push_back(OccluderBox{Vec3(0.0f, 0.0f, -1.0f), Vec3(0.5f), Quat(1, 0, 0, 0), 0});
    boxes.push_back(OccluderBox{Vec3(0.0f, 0.0f, 1.0f), Vec3(0.5f), Quat(1, 0, 0, 0), 1});

    const Vec3 flow(0.0f, 0.0f, 1.0f);
    SW_CHECK(almost(exposure(boxes, 0, flow), 1.0, 1.0e-6));   // nothing ahead of it
    SW_CHECK(exposure(boxes, 1, flow) < 0.2f);               // entirely shadowed
    SW_CHECK(exposure(boxes, 1, flow) > 0.0f);               // but never nothing

    // Turn the vehicle broadside and both boxes are in clean air.
    const Vec3 sideways(1.0f, 0.0f, 0.0f);
    SW_CHECK(almost(exposure(boxes, 0, sideways), 1.0, 1.0e-6));
    SW_CHECK(almost(exposure(boxes, 1, sideways), 1.0, 1.0e-6));
}

SW_TEST(AHalfShadowIsHalfExposed)
{
    // A wide part with a narrow part in front of it: some of the net gets
    // through and some does not.
    std::vector<OccluderBox> boxes;
    boxes.push_back(
        OccluderBox{Vec3(0.0f, 0.0f, -2.0f), Vec3(0.4f, 2.0f, 0.2f), Quat(1, 0, 0, 0), 0});
    boxes.push_back(
        OccluderBox{Vec3(0.0f, 0.0f, 0.0f), Vec3(2.0f, 2.0f, 0.5f), Quat(1, 0, 0, 0), 1});

    const f32 shaded = exposure(boxes, 1, Vec3(0.0f, 0.0f, 1.0f));
    SW_CHECK(shaded > 0.2f);
    SW_CHECK(shaded < 0.9f);
}

SW_TEST(TheRayFindsTheNearFaceOfABox)
{
    const OccluderBox box{Vec3(0.0f), Vec3(1.0f), Quat(1, 0, 0, 0), 0};
    f32 entry = 0.0f;
    SW_CHECK(rayHitsBox(box, Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f), 10.0f, entry));
    SW_CHECK(almost(entry, 4.0, 1.0e-4));
    SW_CHECK(!rayHitsBox(box, Vec3(5.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f), 10.0f, entry));
    // Too short a ray is a miss, not a hit at the far end.
    SW_CHECK(!rayHitsBox(box, Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f), 3.0f, entry));
}

SW_TEST(DampingAlwaysOpposesTheRotation)
{
    const Vec3 spin(0.3f, -0.7f, 0.1f);
    const Vec3 moment = dampingMoment(1.225, 200.0, 4.0, 3.0, spin);
    SW_CHECK(glm::dot(moment, spin) < 0.0f);
    // Faster air damps harder; still air does not damp at all.
    SW_CHECK(glm::length(dampingMoment(1.225, 400.0, 4.0, 3.0, spin)) >
             glm::length(moment));
    SW_CHECK_EQ(glm::length(dampingMoment(0.0, 400.0, 4.0, 3.0, spin)), 0.0f);
    SW_CHECK_EQ(glm::length(dampingMoment(1.225, 0.0, 4.0, 3.0, spin)), 0.0f);
    // The lever arm is squared: a fin twice as far back damps four times as
    // hard, which is the same reason it stabilises four times as hard.
    SW_CHECK(almost(glm::length(dampingMoment(1.225, 200.0, 4.0, 6.0, spin)) /
                      glm::length(moment),
                  4.0, 0.01));
}

SW_TEST(InertiaGrowsWithTheBoxItIsMadeOf)
{
    // A slender rod: hard to spin end over end, easy to roll.
    const Vec3 rod = boxInertia(1000.0, Vec3(0.5f, 0.5f, 5.0f));
    SW_CHECK(rod.x > rod.z * 10.0f);
    SW_CHECK(almost(rod.x, rod.y, 1.0e-3));
    // m/12 (y^2 + z^2) with y = 1 m, z = 10 m.
    SW_CHECK(almost(rod.x, 1000.0 / 12.0 * (1.0 + 100.0), 1.0));
    SW_CHECK(almost(rod.z, 1000.0 / 12.0 * (1.0 + 1.0), 1.0));
}

// ---------------------------------------------------------------------------
// THE WHOLE PIPELINE: a rocket, in air, with fins.
//
// Everything above tests a piece. This tests the claim: that a vehicle built
// out of tabulated parts weathercocks when its fins are at the back and
// tumbles when they are at the front, with nothing anywhere in the engine
// having been told what a fin is for.
// ---------------------------------------------------------------------------

SW_TEST(FinsAtTheTailStabiliseAndFinsAtTheNoseDoNot)
{
    const std::filesystem::path assets =
        FileSystem::executableDirectory() / "Assets" / "Parts";
    SW_CHECK(parts::loadCatalog(assets));
    SW_CHECK(loadTables(assets) >= 9);

    constexpr f64 kTerraRadius = 6.371e6;

    const auto flyWithFinsAt = [&](f32 finZ) {
        ecs::World world;
        parts::VesselAssemblySystem assembly;
        VesselAerodynamicsSystem aerodynamics;

        const ecs::Entity terra = world.createEntity();
        TransformComponent terraTransform{};
        world.addComponent(terra, terraTransform);
        phys::GravitySourceComponent gravity{};
        gravity.mu = 3.986004418e14;
        gravity.bodyRadius = kTerraRadius;
        world.addComponent(terra, gravity);
        world.addComponent(terra, phys::AtmosphereComponent{});

        const ecs::Entity root = world.createEntity();
        world.addComponent(root, TransformComponent{});
        world.addComponent(root, parts::VesselComponent{});
        world.addComponent(root, phys::DynamicBodyComponent{});
        world.addComponent(root, AeroStateComponent{});

        const auto addPart = [&](u32 id, const Vec3& position, const Quat& rotation) {
            const ecs::Entity part = world.createEntity();
            parts::PartComponent component{};
            component.definitionId = id;
            component.vessel = root;
            component.localPosition = position;
            component.localRotation = rotation;
            world.addComponent(part, component);
            world.addComponent(part, TransformComponent{});
        };
        const Quat identity(1.0f, 0.0f, 0.0f, 0.0f);
        addPart(parts::kPartCoreStructural, Vec3(0.0f, 0.0f, -4.0f), identity);
        addPart(parts::kPartFuelTankMedium, Vec3(0.0f, 0.0f, 0.0f), identity);
        addPart(parts::kPartEngineVector, Vec3(0.0f, 0.0f, 3.2f), identity);
        for (int i = 0; i < 4; ++i)
        {
            const Quat spin =
                glm::angleAxis(static_cast<f32>(i) * math::kHalfPi, Vec3(0, 0, 1));
            addPart(parts::kPartFinBasic, spin * Vec3(1.6f, 0.0f, finZ), spin);
        }

        // 260 m/s straight up at 6 km, nose pitched six degrees off the
        // flight path — a rocket that has just been nudged by a gust.
        TransformComponent& transform = world.getComponent<TransformComponent>(root);
        phys::DynamicBodyComponent& body =
            world.getComponent<phys::DynamicBodyComponent>(root);
        const WorldVec3 up{0.0, 1.0, 0.0};
        transform.position = up * (kTerraRadius + 6000.0);
        body.velocity = up * 260.0;
        transform.rotation =
            glm::normalize(glm::angleAxis(math::toRadians(6.0f), Vec3(1, 0, 0)) *
                           glm::rotation(Vec3(0, 0, -1), Vec3(up)));

        assembly.update(world, 0.02f);
        aerodynamics.update(world, 0.02f);
        return world.getComponent<AeroStateComponent>(root);
    };

    const AeroStateComponent tail = flyWithFinsAt(3.0f);
    const AeroStateComponent nose = flyWithFinsAt(-3.6f);

    // Both are flying, fast, in real air.
    SW_CHECK_EQ(tail.inAtmosphere, 1u);
    SW_CHECK(tail.dynamicPressurePa > 1.0e4);
    SW_CHECK(tail.machNumber > 0.5 && tail.machNumber < 1.2);
    SW_CHECK(tail.angleOfAttackRad > 0.05); // it really is at an angle
    SW_CHECK(tail.dragN > 0.0);             // and the air really is slowing it

    // THE POINT. Fins behind the balance point turn the nose back into the
    // wind; the same fins in front of it turn the nose further away. The
    // pitch axis here is x, and the disturbance was a positive rotation
    // about it, so a restoring moment is a negative one.
    SW_CHECK(tail.angularAccelRadS2.x < -0.1f);
    SW_CHECK(nose.angularAccelRadS2.x > 0.1f);

    // ...and the reason is where the pressure acts. Tail fins pull the
    // centre of pressure BEHIND the centre of mass (+Z is the tail);
    // nose fins drag it forward.
    SW_CHECK(tail.centreOfPressure.z > nose.centreOfPressure.z);
}

SW_TEST(AStackHidesBehindItsOwnNose)
{
    const std::filesystem::path assets =
        FileSystem::executableDirectory() / "Assets" / "Parts";
    SW_CHECK(parts::loadCatalog(assets));
    SW_CHECK(loadTables(assets) >= 9);

    constexpr f64 kTerraRadius = 6.371e6;

    const auto dragOfTanks = [&](int count) {
        ecs::World world;
        parts::VesselAssemblySystem assembly;
        VesselAerodynamicsSystem aerodynamics;

        const ecs::Entity terra = world.createEntity();
        world.addComponent(terra, TransformComponent{});
        phys::GravitySourceComponent gravity{};
        gravity.mu = 3.986004418e14;
        gravity.bodyRadius = kTerraRadius;
        world.addComponent(terra, gravity);
        world.addComponent(terra, phys::AtmosphereComponent{});

        const ecs::Entity root = world.createEntity();
        world.addComponent(root, TransformComponent{});
        world.addComponent(root, parts::VesselComponent{});
        world.addComponent(root, phys::DynamicBodyComponent{});
        world.addComponent(root, AeroStateComponent{});
        for (int i = 0; i < count; ++i)
        {
            const ecs::Entity part = world.createEntity();
            parts::PartComponent component{};
            component.definitionId = parts::kPartFuelTankMedium;
            component.vessel = root;
            component.localPosition = Vec3(0.0f, 0.0f, static_cast<f32>(i) * 4.2f);
            world.addComponent(part, component);
            world.addComponent(part, TransformComponent{});
        }

        TransformComponent& transform = world.getComponent<TransformComponent>(root);
        phys::DynamicBodyComponent& body =
            world.getComponent<phys::DynamicBodyComponent>(root);
        const WorldVec3 up{0.0, 1.0, 0.0};
        transform.position = up * (kTerraRadius + 6000.0);
        body.velocity = up * 260.0;
        transform.rotation = glm::rotation(Vec3(0, 0, -1), Vec3(up));

        assembly.update(world, 0.02f);
        aerodynamics.update(world, 0.02f);
        return world.getComponent<AeroStateComponent>(root).dragN;
    };

    const f64 one = dragOfTanks(1);
    const f64 five = dragOfTanks(5);
    SW_CHECK(one > 1.0e4);
    // Five tanks nose to tail are not five tanks' worth of drag: four of
    // them are standing in the first one's shadow. Some growth is right —
    // the stack is longer, so there is more skin in the wind — but nothing
    // like five times.
    SW_CHECK(five > one);
    SW_CHECK(five < one * 1.8);
}

// ---------------------------------------------------------------------------
// THE AIR, ASKED A QUESTION IT HAS NO ANSWER TO
// ---------------------------------------------------------------------------

namespace
{
    [[nodiscard]] bool allFinite(const Vec3& v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    [[nodiscard]] bool allFinite(const WorldVec3& v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    /// A table whose entries say which node they came from, so a lookup can
    /// be checked against the node it was supposed to land on.
    [[nodiscard]] AeroTable labelledTable()
    {
        AeroTable table{};
        table.partId = 1;
        table.thetaCount = 5;
        table.phiCount = 4;
        table.samples.resize(20);
        for (u32 t = 0; t < table.thetaCount; ++t)
        {
            for (u32 p = 0; p < table.phiCount; ++p)
            {
                table.samples[t * table.phiCount + p].forceM2 =
                    Vec3(static_cast<f32>(t), static_cast<f32>(p), 0.0f);
                table.samples[t * table.phiCount + p].momentM3 =
                    Vec3(0.0f, static_cast<f32>(t), static_cast<f32>(p));
            }
        }
        return table;
    }
} // namespace

SW_TEST(SamplingStillAirIsNotANaN)
{
    // A VESSEL THAT IS NOT MOVING is the commonest input this function gets:
    // every rocket on every pad, every tick, until the clamps release. Its
    // part-frame relative wind is the zero vector, and normalizing that is
    // 0/0 in all three lanes. The NaN did not stop at the direction either —
    // theta and phi came out NaN, the node indices came from a NaN cast
    // (implementation-defined, so not even reliably out of range), and the
    // sample went into the vessel's running force and moment for the tick.
    const AeroTable table = labelledTable();
    const AeroSample still = sample(table, Vec3(0.0f));
    SW_CHECK(allFinite(still.forceM2));
    SW_CHECK(allFinite(still.momentM3));

    // And the answer is not merely finite, it is the right one: head-on,
    // theta = 0, which is the fallback flowDirection() already uses for a
    // part with no airspeed. Node (0, 0) of the labelled table.
    SW_CHECK(almost(still.forceM2.x, 0.0, 1.0e-5));
    SW_CHECK(almost(still.forceM2.y, 0.0, 1.0e-5));
    const AeroSample headOn = sample(table, Vec3(0.0f, 0.0f, 1.0f));
    SW_CHECK(glm::length(still.forceM2 - headOn.forceM2) < 1.0e-5f);
    SW_CHECK(glm::length(still.momentM3 - headOn.momentM3) < 1.0e-5f);

    // A denormally small flow is a direction; it must still be read as one.
    const AeroSample crawling = sample(table, Vec3(0.0f, 1.0e-6f, 0.0f));
    SW_CHECK(allFinite(crawling.forceM2));
    SW_CHECK(almost(crawling.forceM2.x, 2.0, 1.0e-4)); // 90 degrees: theta node 2
}

SW_TEST(NoAerodynamicEntryPointAnswersADegenerateQuestionWithANaN)
{
    // Same rule as the physics side: nothing in this header may hand back a
    // NaN for an empty object or a zero vector. A NaN force is added to a
    // vessel total, a NaN moment is divided by an inertia and integrated, and
    // by the time anyone notices, the vehicle has no position at all.
    const AeroTable emptyTable{};
    const AeroSample fromNothing = sample(emptyTable, Vec3(0.0f));
    SW_CHECK(allFinite(fromNothing.forceM2));
    SW_CHECK(allFinite(fromNothing.momentM3));

    const phys::AtmosphereComponent air{};
    SW_CHECK(std::isfinite(density(air, 0.0)));
    SW_CHECK(std::isfinite(density(air, -1.0e6)));
    SW_CHECK(std::isfinite(temperature(air, -1.0e6)));
    SW_CHECK(std::isfinite(speedOfSound(air, 0.0)));
    SW_CHECK(std::isfinite(machDragFactor(0.0)));
    SW_CHECK(std::isfinite(machDragFactor(-1.0)));
    SW_CHECK(std::isfinite(dynamicPressure(0.0, 0.0)));

    // An atmosphere with no thickness at all.
    phys::AtmosphereComponent none{};
    none.scaleHeight = 0.0;
    none.surfaceDensity = 0.0;
    none.topAltitude = 0.0;
    SW_CHECK(std::isfinite(density(none, 0.0)));
    SW_CHECK(std::isfinite(temperature(none, 0.0)));
    SW_CHECK(std::isfinite(speedOfSound(none, 0.0)));

    // ...and that case never reaches the scale height at all, which is worth
    // saying because it was doing duty as the test of it: topAltitude = 0
    // means density() returns at the altitude guard one line earlier. The
    // division is real and needs a ceiling ABOVE the ground to be reached.
    // MEASURED without the scale-height guard: -nan at altitude 0.
    phys::AtmosphereComponent noScale{};
    noScale.scaleHeight = 0.0;
    noScale.surfaceDensity = 1.225;
    noScale.topAltitude = 1.0e5;
    SW_CHECK(std::isfinite(density(noScale, 0.0)));
    SW_CHECK_EQ(density(noScale, 0.0), 0.0); // no scale height, no air
    SW_CHECK(std::isfinite(density(noScale, 1.0e4)));
    // The wind divides by it twice more: the jet's width, and its falloff.
    // MEASURED without the guard: (nan, nan, nan) at altitude 0.
    const WorldVec3 deadWind = windVelocity(noScale, 0.0, WorldVec3{0.0, 1.0, 0.0},
                                            WorldVec3{1.0, 0.0, 0.0}, 0.0);
    SW_CHECK(allFinite(deadWind));
    SW_CHECK_EQ(glm::length(deadWind), 0.0);

    // The other sign of the same mistake, and the reason "is it a number" is
    // not on its own enough: a NEGATIVE scale height is air that gets
    // THICKER with height. Near the ground it is perfectly finite and
    // perfectly wrong — MEASURED 1.377938 kg/m^3 at 1 km on -8500 m, above
    // the 1.225 at sea level — and it overflows to +inf higher up.
    phys::AtmosphereComponent inverted{};
    inverted.scaleHeight = -8500.0;
    inverted.surfaceDensity = 1.225;
    inverted.topAltitude = 1.0e7;
    SW_CHECK_EQ(density(inverted, 1.0e3), 0.0);
    SW_CHECK(std::isfinite(density(inverted, 7.0e6))); // exp(823) unguarded
    SW_CHECK_EQ(density(inverted, 7.0e6), 0.0);

    SW_CHECK(allFinite(windVelocity(air, 0.0, WorldVec3{0.0, 1.0, 0.0},
                                    WorldVec3{1.0, 0.0, 0.0}, 0.0)));
    SW_CHECK(allFinite(windVelocity(air, 1.0e9, WorldVec3{0.0}, WorldVec3{0.0}, 0.0)));

    SW_CHECK(allFinite(flowDirection(Vec3(0.0f))));
    SW_CHECK(almost(glm::length(flowDirection(Vec3(0.0f))), 1.0, 1.0e-6));

    SW_CHECK(allFinite(dampingMoment(0.0, 0.0, 0.0, 0.0, Vec3(0.0f))));
    SW_CHECK(allFinite(dampingMoment(1.225, 200.0, 4.0, 3.0, Vec3(0.0f))));
    SW_CHECK(allFinite(boxInertia(0.0, Vec3(0.0f))));

    // Occlusion, with no boxes at all and with a flow that is not a
    // direction: `perpendicular()` normalizes a cross product that would be
    // the zero vector if the flow were.
    const std::vector<OccluderBox> nothing;
    SW_CHECK(std::isfinite(exposure(nothing, 0, Vec3(0.0f, 0.0f, 1.0f))));
    std::vector<OccluderBox> one(1);
    one[0].halfExtents = Vec3(0.0f);
    SW_CHECK(std::isfinite(exposure(one, 0, Vec3(0.0f))));
    SW_CHECK(std::isfinite(exposure(one, 0, Vec3(0.0f, 0.0f, 1.0f))));

    f32 near = 0.0f;
    OccluderBox flat{};
    flat.halfExtents = Vec3(0.0f);
    (void)rayHitsBox(flat, Vec3(0.0f), Vec3(0.0f), 0.0f, near);
    SW_CHECK(std::isfinite(near));
}
