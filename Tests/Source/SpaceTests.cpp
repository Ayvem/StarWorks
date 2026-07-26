// ============================================================================
// SpaceTests.cpp — the hierarchical star system: celestial index recursion,
// SOI primary selection, celestial motion, and the patched-conics
// trajectory prediction (encounters, impacts, SOI exits — the KSP rules).
// ============================================================================

#include "TestFramework.hpp"

#include <ECS/World.hpp>
#include <Physics/PhysicsSystems.hpp>
#include <Scene/TransformComponents.hpp>
#include <Simulation/Simulation.hpp>
#include <Space/CelestialIndex.hpp>
#include <Space/CelestialSystems.hpp>
#include <Space/TrajectoryPrediction.hpp>

#include <cmath>

using namespace sw;
using namespace sw::space;

namespace
{
    constexpr f64 kMuSol = 1.32712440018e20;
    constexpr f64 kMuTerra = 3.986004418e14;
    constexpr f64 kMuLuna = 4.9048695e12;
    constexpr f64 kTerraSma = 1.496e11;
    constexpr f64 kLunaSma = 3.844e8;
    constexpr f64 kTerraSoi = 9.24e8;
    constexpr f64 kLunaSoi = 6.61e7;
    constexpr f64 kTerraRadius = 6.371e6;
    constexpr f64 kLunaRadius = 1.7374e6;
    constexpr f64 kLeoRadius = 6.771e6;
    constexpr f64 kPi = 3.14159265358979323846;

    struct SystemFixture
    {
        ecs::World world;
        ecs::Entity sol{};
        ecs::Entity terra{};
        ecs::Entity luna{};
        phys::KeplerOrbit terraOrbit{};
        phys::KeplerOrbit lunaOrbit{};

        explicit SystemFixture(f64 lunaM0 = 0.0)
        {
            sol = world.createEntity();
            world.addComponent(sol, TransformComponent{});
            world.addComponent(sol, PreviousTransformComponent{});
            world.addComponent(sol, phys::GravitySourceComponent{kMuSol, 6.9634e8});
            world.addComponent(sol, makeCelestialBody("SOL"));

            terraOrbit =
                phys::kepler::fromElements(kMuSol, kTerraSma, 0.0167, 0.0, 0.0, 0.0,
                                           /*M0=*/0.0, /*epoch=*/0.0);
            terra = world.createEntity();
            TransformComponent terraTransform{};
            phys::kepler::evaluate(terraOrbit, 0.0, terraTransform.position);
            world.addComponent(terra, terraTransform);
            world.addComponent(terra, PreviousTransformComponent{});
            phys::GravitySourceComponent terraGravity{kMuTerra, kTerraRadius};
            terraGravity.soiRadius = kTerraSoi;
            world.addComponent(terra, terraGravity);
            world.addComponent(terra, makeCelestialBody("TERRA", sol, &terraOrbit));

            lunaOrbit = phys::kepler::fromElements(kMuTerra, kLunaSma, 0.0, 0.0, 0.0,
                                                   0.0, lunaM0, /*epoch=*/0.0);
            luna = world.createEntity();
            TransformComponent lunaTransform{};
            phys::kepler::evaluate(lunaOrbit, 0.0, lunaTransform.position);
            lunaTransform.position += terraTransform.position;
            world.addComponent(luna, lunaTransform);
            world.addComponent(luna, PreviousTransformComponent{});
            phys::GravitySourceComponent lunaGravity{kMuLuna, kLunaRadius};
            lunaGravity.soiRadius = kLunaSoi;
            world.addComponent(luna, lunaGravity);
            world.addComponent(luna, makeCelestialBody("LUNA", terra, &lunaOrbit));
        }
    };
} // namespace

SW_TEST(CelestialIndexRecursionAndSoi)
{
    SystemFixture fixture;
    CelestialIndex index;
    index.rebuild(fixture.world);
    SW_CHECK_EQ(index.size(), static_cast<usize>(3));

    const i32 solIndex = index.indexOf(fixture.sol);
    const i32 terraIndex = index.indexOf(fixture.terra);
    const i32 lunaIndex = index.indexOf(fixture.luna);
    SW_CHECK(solIndex >= 0 && terraIndex >= 0 && lunaIndex >= 0);
    // Topological order: parents strictly before children.
    SW_CHECK(solIndex < terraIndex && terraIndex < lunaIndex);
    SW_CHECK_EQ(index.childrenOf(static_cast<usize>(terraIndex)).size(),
                static_cast<usize>(1));

    // Recursive world states at an arbitrary future time: Luna's world
    // position must be Terra's world position + Luna's relative evaluation.
    const f64 t = 123456.0;
    WorldVec3 terraPosition{};
    WorldVec3 terraVelocity{};
    index.stateAt(terraIndex, t, terraPosition, &terraVelocity);
    WorldVec3 terraExpected{};
    WorldVec3 terraVelocityExpected{};
    phys::kepler::evaluate(fixture.terraOrbit, t, terraExpected, &terraVelocityExpected);
    SW_CHECK(glm::length(terraPosition - terraExpected) < 1.0);
    SW_CHECK(glm::length(terraVelocity - terraVelocityExpected) < 1.0e-6);

    WorldVec3 lunaPosition{};
    WorldVec3 lunaVelocity{};
    index.stateAt(lunaIndex, t, lunaPosition, &lunaVelocity);
    WorldVec3 lunaRelative{};
    WorldVec3 lunaRelativeVelocity{};
    phys::kepler::evaluate(fixture.lunaOrbit, t, lunaRelative, &lunaRelativeVelocity);
    SW_CHECK(glm::length(lunaPosition - (terraExpected + lunaRelative)) < 1.0);
    SW_CHECK(glm::length(lunaVelocity -
                         (terraVelocityExpected + lunaRelativeVelocity)) < 1.0e-6);

    // SOI selection: LEO -> Terra, near Luna -> Luna (NOT the heavier
    // Terra, NOT the vastly heavier Sol), interplanetary -> Sol.
    SW_CHECK_EQ(index.soiPrimaryAt(terraPosition + WorldVec3{kLeoRadius, 0.0, 0.0}, t),
                terraIndex);
    SW_CHECK_EQ(index.soiPrimaryAt(lunaPosition + WorldVec3{1.0e7, 0.0, 0.0}, t),
                lunaIndex);
    SW_CHECK_EQ(index.soiPrimaryAt(terraPosition * 0.5, t), solIndex);
}

SW_TEST(CelestialMotionSystemMovesTheHierarchy)
{
    SystemFixture fixture;
    sim::Simulation simulation({{"Physics", 50.0f, 4}});
    sim::SimulationLane& lane = *simulation.findLane("Physics");
    lane.scheduler().addSystem(std::make_unique<CelestialMotionSystem>(lane));

    for (int i = 0; i < 10; ++i)
    {
        simulation.advance(fixture.world, 0.02f, nullptr);
    }
    const f64 t = simulation.findLane("Physics")->presentSeconds();

    // Terra's transform follows the analytic orbit; its stamped world
    // velocity matches; Luna rides on top.
    WorldVec3 terraExpected{};
    WorldVec3 terraVelocityExpected{};
    phys::kepler::evaluate(fixture.terraOrbit, t, terraExpected, &terraVelocityExpected);
    const auto& terraTransform =
        fixture.world.getComponent<TransformComponent>(fixture.terra);
    SW_CHECK(glm::length(terraTransform.position - terraExpected) < 1.0);
    const auto& terraGravity =
        fixture.world.getComponent<phys::GravitySourceComponent>(fixture.terra);
    SW_CHECK(glm::length(terraGravity.worldVelocity - terraVelocityExpected) < 1.0e-6);
    // Terra moves ~30 km/s: after 0.2 s it must have traveled ~6 km.
    SW_CHECK(glm::length(terraVelocityExpected) > 2.9e4);

    WorldVec3 lunaRelative{};
    phys::kepler::evaluate(fixture.lunaOrbit, t, lunaRelative);
    const auto& lunaTransform =
        fixture.world.getComponent<TransformComponent>(fixture.luna);
    SW_CHECK(glm::length(lunaTransform.position - (terraExpected + lunaRelative)) < 1.0);

    // Interpolation support: previous is one tick behind, not mirrored.
    const auto& terraPrevious =
        fixture.world.getComponent<PreviousTransformComponent>(fixture.terra);
    const f64 tickTravel = glm::length(terraTransform.position - terraPrevious.position);
    SW_CHECK(tickTravel > 100.0 && tickTravel < 2000.0); // ~600 m per 0.02 s
}

SW_TEST(TrajectoryPredictionDetectsLunaEncounter)
{
    // A Hohmann transfer aimed at Luna: periapsis at LEO (+X of Terra),
    // apoapsis at Luna's distance. Luna's phase is chosen so it arrives at
    // the apoapsis point exactly when the ship does.
    const f64 transferSma = 0.5 * (kLeoRadius + kLunaSma);
    const f64 timeOfFlight = kPi * std::sqrt(transferSma * transferSma * transferSma /
                                             kMuTerra);
    const f64 lunaMeanMotion = std::sqrt(kMuTerra / (kLunaSma * kLunaSma * kLunaSma));
    // Ship apoapsis is at true anomaly pi from +X; Luna starts behind it.
    const f64 lunaM0 = kPi - lunaMeanMotion * timeOfFlight;

    SystemFixture fixture(lunaM0);
    CelestialIndex index;
    index.rebuild(fixture.world);
    const i32 terraIndex = index.indexOf(fixture.terra);
    const i32 lunaIndex = index.indexOf(fixture.luna);

    // Ship state at t=0: LEO periapsis of the transfer ellipse, in WORLD
    // frame (Terra-relative + Terra's own state).
    const f64 periapsisSpeed =
        std::sqrt(kMuTerra * (2.0 / kLeoRadius - 1.0 / transferSma));
    WorldVec3 terraPosition{};
    WorldVec3 terraVelocity{};
    index.stateAt(terraIndex, 0.0, terraPosition, &terraVelocity);
    const WorldVec3 shipPosition = terraPosition + WorldVec3{kLeoRadius, 0.0, 0.0};
    // Prograde consistent with fromElements' basisQ convention (-Z at +X).
    const WorldVec3 shipVelocity =
        terraVelocity + WorldVec3{0.0, 0.0, -periapsisSpeed};

    PredictionSettings settings{};
    settings.horizonSeconds = timeOfFlight * 1.5;
    std::vector<TrajectorySegment> segments;
    predictTrajectory(index, shipPosition, shipVelocity, 0.0, settings, segments);

    // Segment 0: elliptic around Terra, ends with a LUNA ENCOUNTER.
    SW_CHECK(segments.size() >= 2);
    SW_CHECK_EQ(segments[0].primaryIndex, terraIndex);
    SW_CHECK(!segments[0].orbit.isHyperbolic());
    SW_CHECK(segments[0].endReason == SegmentEnd::Encounter);
    SW_CHECK_EQ(segments[0].eventBodyIndex, lunaIndex);
    // The encounter happens on approach, before the nominal arrival time.
    SW_CHECK(segments[0].endTime > 0.5 * timeOfFlight);
    SW_CHECK(segments[0].endTime < timeOfFlight);

    // At the refined encounter time the ship sits ON Luna's SOI sphere.
    WorldVec3 shipRelative{};
    phys::kepler::evaluate(segments[0].orbit, segments[0].endTime, shipRelative);
    WorldVec3 lunaRelative{};
    phys::kepler::evaluate(fixture.lunaOrbit, segments[0].endTime, lunaRelative);
    const f64 separation = glm::length(shipRelative - lunaRelative);
    SW_CHECK(std::abs(separation - kLunaSoi) / kLunaSoi < 1.0e-3);

    // Segment 1 is the patch INSIDE Luna's SOI, starting where 0 ended.
    SW_CHECK_EQ(segments[1].primaryIndex, lunaIndex);
    SW_CHECK_EQ(segments[1].startTime, segments[0].endTime);
    // A fast arrival relative to a light moon: the flyby arc is hyperbolic.
    SW_CHECK(segments[1].orbit.isHyperbolic());
}

SW_TEST(TrajectoryPredictionDetectsImpactAndSoiExit)
{
    SystemFixture fixture;
    CelestialIndex index;
    index.rebuild(fixture.world);
    const i32 solIndex = index.indexOf(fixture.sol);
    const i32 terraIndex = index.indexOf(fixture.terra);

    WorldVec3 terraPosition{};
    WorldVec3 terraVelocity{};
    index.stateAt(terraIndex, 0.0, terraPosition, &terraVelocity);

    // ---- impact: a suborbital lob — half circular speed at 400 km -------
    {
        const f64 speed = 0.5 * phys::kepler::circularOrbitSpeed(kMuTerra, kLeoRadius);
        PredictionSettings settings{};
        settings.horizonSeconds = 6.0 * 3600.0;
        std::vector<TrajectorySegment> segments;
        predictTrajectory(index, terraPosition + WorldVec3{kLeoRadius, 0.0, 0.0},
                          terraVelocity + WorldVec3{0.0, 0.0, -speed}, 0.0, settings,
                          segments);

        SW_CHECK_EQ(segments.size(), static_cast<usize>(1));
        SW_CHECK(segments[0].endReason == SegmentEnd::Impact);
        SW_CHECK_EQ(segments[0].eventBodyIndex, terraIndex);
        // The refined impact point is on the surface.
        WorldVec3 impactRelative{};
        phys::kepler::evaluate(segments[0].orbit, segments[0].endTime, impactRelative);
        SW_CHECK(std::abs(glm::length(impactRelative) - kTerraRadius) / kTerraRadius <
                 1.0e-3);
    }

    // ---- SOI exit: 1.4x escape speed straight out of Terra ---------------
    {
        const f64 escape = std::sqrt(2.0 * kMuTerra / kLeoRadius);
        PredictionSettings settings{};
        settings.horizonSeconds = 30.0 * 86400.0;
        std::vector<TrajectorySegment> segments;
        predictTrajectory(index, terraPosition + WorldVec3{kLeoRadius, 0.0, 0.0},
                          terraVelocity + WorldVec3{0.0, 0.0, -1.4 * escape}, 0.0,
                          settings, segments);

        SW_CHECK(segments.size() >= 2);
        SW_CHECK_EQ(segments[0].primaryIndex, terraIndex);
        SW_CHECK(segments[0].orbit.isHyperbolic());
        SW_CHECK(segments[0].endReason == SegmentEnd::SoiExit);
        SW_CHECK_EQ(segments[0].eventBodyIndex, solIndex);
        // At the exit time the ship is on Terra's SOI sphere.
        WorldVec3 exitRelative{};
        phys::kepler::evaluate(segments[0].orbit, segments[0].endTime, exitRelative);
        SW_CHECK(std::abs(glm::length(exitRelative) - kTerraSoi) / kTerraSoi < 1.0e-3);
        // The follow-up patch orbits Sol and coasts to the horizon (bound
        // heliocentric ellipse: Terra escape is far below Sol escape).
        SW_CHECK_EQ(segments[1].primaryIndex, solIndex);
        SW_CHECK(!segments[1].orbit.isHyperbolic());
        SW_CHECK(segments[1].endReason == SegmentEnd::Horizon);
    }
}

SW_TEST(StateOnPredictionAndManeuverDv)
{
    SystemFixture fixture;
    CelestialIndex index;
    index.rebuild(fixture.world);
    const i32 terraIndex = index.indexOf(fixture.terra);

    // Circular LEO around Terra, world frame.
    WorldVec3 terraPosition{};
    WorldVec3 terraVelocity{};
    index.stateAt(terraIndex, 0.0, terraPosition, &terraVelocity);
    const f64 circular = phys::kepler::circularOrbitSpeed(kMuTerra, kLeoRadius);
    const WorldVec3 position = terraPosition + WorldVec3{kLeoRadius, 0.0, 0.0};
    const WorldVec3 velocity = terraVelocity + WorldVec3{0.0, 0.0, -circular};

    PredictionSettings settings{};
    std::vector<TrajectorySegment> segments;
    predictTrajectory(index, position, velocity, 0.0, settings, segments);
    SW_CHECK(!segments.empty());

    // stateOnPrediction at a future time must equal the direct composition
    // of the segment conic with Terra's analytic world state.
    const f64 probeTime = 987.0;
    WorldVec3 predictedPosition{};
    WorldVec3 predictedVelocity{};
    SW_CHECK(stateOnPrediction(index, segments, probeTime, predictedPosition,
                               predictedVelocity));
    WorldVec3 terraAtProbe{};
    WorldVec3 terraVelocityAtProbe{};
    index.stateAt(terraIndex, probeTime, terraAtProbe, &terraVelocityAtProbe);
    WorldVec3 relative{};
    WorldVec3 relativeVelocity{};
    phys::kepler::evaluate(segments[0].orbit, probeTime, relative, &relativeVelocity);
    SW_CHECK(glm::length(predictedPosition - (terraAtProbe + relative)) < 1.0);
    SW_CHECK(glm::length(predictedVelocity -
                         (terraVelocityAtProbe + relativeVelocity)) < 1.0e-6);

    // A prograde maneuver dv at that state must raise the apoapsis on the
    // opposite side while keeping the burn point as the new periapsis.
    const WorldVec3 relV = predictedVelocity - terraVelocityAtProbe;
    const WorldVec3 prograde = relV / glm::length(relV);
    const f64 dv = 500.0;
    std::vector<TrajectorySegment> burned;
    predictTrajectory(index, predictedPosition, predictedVelocity + prograde * dv,
                      probeTime, settings, burned);
    SW_CHECK(!burned.empty());
    const f64 newApoapsis = phys::kepler::apoapsis(burned[0].orbit);
    const f64 newPeriapsis = phys::kepler::periapsis(burned[0].orbit);
    SW_CHECK(newApoapsis > kLeoRadius * 1.15);              // clearly raised
    SW_CHECK(std::abs(newPeriapsis - kLeoRadius) < 2.0e4);  // burn point stays
}

SW_TEST(RailsFollowTheirPrimaryAroundTheStar)
{
    SystemFixture fixture;
    sim::Simulation simulation({{"Physics", 50.0f, 4}});
    auto& scheduler = simulation.findLane("Physics")->scheduler();
    scheduler.addSystem(std::make_unique<CelestialMotionSystem>(
        *simulation.findLane("Physics")));
    scheduler.addSystem(
        std::make_unique<phys::RailsSystem>(*simulation.findLane("Physics")));

    // A station on rails around TERRA (primary-relative elements).
    const phys::KeplerOrbit stationOrbit = phys::kepler::fromElements(
        kMuTerra, kLeoRadius, 0.0, 0.0, 0.0, 0.0, /*M0=*/1.0, 0.0);
    const ecs::Entity station = fixture.world.createEntity();
    {
        TransformComponent transform{};
        phys::kepler::evaluate(stationOrbit, 0.0, transform.position);
        transform.position +=
            fixture.world.getComponent<TransformComponent>(fixture.terra).position;
        fixture.world.addComponent(station, transform);
        fixture.world.addComponent(station, PreviousTransformComponent{});
        phys::OnRailsComponent rails{};
        rails.orbit = stationOrbit;
        rails.primary = fixture.terra;
        fixture.world.addComponent(station, rails);
    }

    for (int i = 0; i < 25; ++i)
    {
        simulation.advance(fixture.world, 0.02f, nullptr);
    }
    const f64 t = simulation.findLane("Physics")->presentSeconds();

    // World position = Terra's analytic world position + relative conic:
    // the station follows its planet around the star.
    WorldVec3 terraExpected{};
    phys::kepler::evaluate(fixture.terraOrbit, t, terraExpected);
    WorldVec3 stationRelative{};
    phys::kepler::evaluate(stationOrbit, t, stationRelative);
    const auto& stationTransform =
        fixture.world.getComponent<TransformComponent>(station);
    SW_CHECK(glm::length(stationTransform.position - (terraExpected + stationRelative)) <
             1.0);
}
