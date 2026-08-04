// ============================================================================
// SpaceTests.cpp — the hierarchical star system: celestial index recursion,
// SOI primary selection, celestial motion, and the patched-conics
// trajectory prediction (encounters, impacts, SOI exits — the KSP rules).
// ============================================================================

#include "TestFramework.hpp"

#include <ECS/World.hpp>
#include <Physics/PhysicsSystems.hpp>
#include <Scene/Camera.hpp>
#include <Scene/TransformComponents.hpp>
#include <Simulation/Simulation.hpp>
#include <Space/CelestialIndex.hpp>
#include <Space/CelestialSystems.hpp>
#include <Space/TrajectoryPrediction.hpp>

#include <cmath>
#include <tuple>

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

// ============================================================================
// A TRAJECTORY RUNS TO ITS END, NOT TO A CLOCK.
//
// The plan used to stop after a fixed six days. On a parking orbit that was
// ninety-six revolutions of line drawn on top of itself; on a heliocentric
// transfer it was one and a half degrees of arc — a stub hanging in space
// that told a pilot nothing about where they were going. What ends a
// trajectory is an EVENT: an impact, an encounter, an escape, or a full
// revolution that meets none of them.
// ============================================================================
SW_TEST(TrajectoryRunsToAnEventOrARevolution)
{
    SystemFixture fixture;
    CelestialIndex index;
    index.rebuild(fixture.world);
    const i32 solIndex = index.indexOf(fixture.sol);
    const i32 terraIndex = index.indexOf(fixture.terra);

    WorldVec3 terraPosition{};
    WorldVec3 terraVelocity{};
    index.stateAt(terraIndex, 0.0, terraPosition, &terraVelocity);

    // ---- a parking orbit CLOSES, after exactly one revolution -----------
    {
        const f64 circular = phys::kepler::circularOrbitSpeed(kMuTerra, kLeoRadius);
        std::vector<TrajectorySegment> segments;
        predictTrajectory(index, terraPosition + WorldVec3{kLeoRadius, 0.0, 0.0},
                          terraVelocity + WorldVec3{0.0, 0.0, -circular}, 0.0,
                          PredictionSettings{}, segments);

        SW_CHECK_EQ(segments.size(), static_cast<usize>(1));
        SW_CHECK(segments[0].endReason == SegmentEnd::Closed);
        const f64 period = phys::kepler::period(segments[0].orbit);
        SW_CHECK(period > 5000.0 && period < 6000.0); // ~92 min at 400 km
        SW_CHECK(std::abs(segments[0].endTime - period) < 1.0);

        // ...and the line joins up: the last point of the arc is the first.
        WorldVec3 first{};
        WorldVec3 last{};
        phys::kepler::evaluate(segments[0].orbit, segments[0].startTime, first);
        phys::kepler::evaluate(segments[0].orbit, segments[0].endTime, last);
        SW_CHECK(glm::length(first - last) < 1.0);
    }

    // ---- a heliocentric orbit draws its WHOLE year, not six days --------
    {
        // A craft coasting beside Terra, outside every sphere of influence:
        // its primary is Sol and its period is about a year.
        const WorldVec3 position = terraPosition * 1.2;
        const f64 speed = phys::kepler::circularOrbitSpeed(kMuSol,
                                                           glm::length(position));
        const WorldVec3 velocity =
            glm::normalize(WorldVec3{-position.z, 0.0, position.x}) * speed;
        std::vector<TrajectorySegment> segments;
        predictTrajectory(index, position, velocity, 0.0, PredictionSettings{},
                          segments);

        SW_CHECK(!segments.empty());
        SW_CHECK_EQ(segments[0].primaryIndex, solIndex);
        SW_CHECK(segments[0].endReason == SegmentEnd::Closed);
        const f64 period = phys::kepler::period(segments[0].orbit);
        // 1.2 AU: a year and a third — 480 days. The old six-day horizon
        // covered 1.2% of it, which is the bug this test exists for.
        SW_CHECK(period > 300.0 * 86400.0);
        SW_CHECK(std::abs(segments[0].endTime - period) < 60.0);
        SW_CHECK(segments[0].endTime > 50.0 * (6.0 * 86400.0));
    }

    // ---- an escape still finds the edge of the sphere of influence ------
    // The scan step is now derived from the hyperbola's own time scale
    // rather than from the horizon, so this is the check that the finer
    // resolution did not cost the coarse reach.
    {
        const f64 escape = std::sqrt(2.0 * kMuTerra / kLeoRadius);
        std::vector<TrajectorySegment> segments;
        predictTrajectory(index, terraPosition + WorldVec3{kLeoRadius, 0.0, 0.0},
                          terraVelocity + WorldVec3{0.0, 0.0, -1.4 * escape}, 0.0,
                          PredictionSettings{}, segments);

        SW_CHECK(segments.size() >= 2);
        SW_CHECK(segments[0].endReason == SegmentEnd::SoiExit);
        WorldVec3 exitRelative{};
        phys::kepler::evaluate(segments[0].orbit, segments[0].endTime, exitRelative);
        SW_CHECK(std::abs(glm::length(exitRelative) - kTerraSoi) / kTerraSoi < 1.0e-3);
        // ...and what it escapes onto is a heliocentric orbit drawn whole.
        SW_CHECK_EQ(segments[1].primaryIndex, solIndex);
        SW_CHECK(segments[1].endReason == SegmentEnd::Closed ||
                 segments[1].endReason == SegmentEnd::Encounter);
    }

    // ---- the horizon is still a HARD cap, which the node planner needs --
    {
        const f64 circular = phys::kepler::circularOrbitSpeed(kMuTerra, kLeoRadius);
        PredictionSettings settings{};
        settings.horizonSeconds = 600.0; // ten minutes, mid-revolution
        std::vector<TrajectorySegment> segments;
        predictTrajectory(index, terraPosition + WorldVec3{kLeoRadius, 0.0, 0.0},
                          terraVelocity + WorldVec3{0.0, 0.0, -circular}, 0.0, settings,
                          segments);
        SW_CHECK_EQ(segments.size(), static_cast<usize>(1));
        SW_CHECK(segments[0].endReason == SegmentEnd::Horizon);
        SW_CHECK(std::abs(segments[0].endTime - 600.0) < 1.0e-6);
    }
}

// ============================================================================
// THE MANEUVER STEP LADDER, AND THE TRAP IN IT.
//
// A node spans five orders of magnitude: a tenth of a metre per second to
// trim a rendezvous, three and a half kilometres of it to leave for Mars.
// The modifiers pick the rung — and Control means two different things
// depending on whether Shift is with it, which is exactly the kind of rule
// that gets written in the wrong order once and then hands a player 0.1 m/s
// when they asked for 1000.
// ============================================================================
SW_TEST(ManeuverStepsClimbWithTheModifiers)
{
    // The rungs themselves.
    SW_CHECK_EQ(maneuverStep(false, false, false).deltaVMps, 1.0);
    SW_CHECK_EQ(maneuverStep(true, false, false).deltaVMps, 10.0);   // shift
    SW_CHECK_EQ(maneuverStep(false, false, true).deltaVMps, 100.0);  // alt
    SW_CHECK_EQ(maneuverStep(true, true, false).deltaVMps, 1000.0);  // ctrl+shift
    SW_CHECK_EQ(maneuverStep(false, true, false).deltaVMps, 0.1);    // ctrl (fine)

    // THE TRAP: control+shift is the COARSEST step, not the finest one.
    // Tested for on its own because reading the flags in the obvious order
    // gets this backwards.
    SW_CHECK(maneuverStep(true, true, false).deltaVMps >
             maneuverStep(false, true, false).deltaVMps * 9999.0);

    // Time moves on the SAME ladder — ten seconds at the base rung, and a
    // burn a hundred times bigger is planned a hundred times further out.
    for (const auto& [shift, control, alt] : {std::tuple{false, false, false},
                                              std::tuple{true, false, false},
                                              std::tuple{false, false, true},
                                              std::tuple{true, true, false},
                                              std::tuple{false, true, false}})
    {
        const ManeuverStep step = maneuverStep(shift, control, alt);
        SW_CHECK(std::abs(step.seconds - step.deltaVMps * 10.0) < 1.0e-12);
        SW_CHECK(step.deltaVMps > 0.0);
    }

    // Monotone in the direction a player would expect: every modifier that
    // means "bigger" is bigger than no modifier at all.
    SW_CHECK(maneuverStep(true, false, false).deltaVMps >
             maneuverStep(false, false, false).deltaVMps);
    SW_CHECK(maneuverStep(false, false, true).deltaVMps >
             maneuverStep(true, false, false).deltaVMps);
    SW_CHECK(maneuverStep(true, true, false).deltaVMps >
             maneuverStep(false, false, true).deltaVMps);

    // ...and one tap of the coarsest rung is a real transfer burn: Terra
    // escape from low orbit is about 3.2 km/s, so three taps and a bit.
    const f64 escapeBurn = 3200.0;
    SW_CHECK(maneuverStep(true, true, false).deltaVMps * 4.0 > escapeBurn);
}

// ============================================================================
// PICKING A MOMENT OFF THE MAP.
//
// Dragging a maneuver node is one question asked every frame: which moment
// of this trajectory is nearest the pixel under the cursor? Get it wrong by
// half an orbit and the burn jumps to the far side of the planet under the
// player's hand.
//
// The trap is the camera. A perspective divide with w < 0 does not fail —
// it mirrors the point through the origin, so the half of the orbit BEHIND
// the camera lands on screen looking perfectly plausible, on the wrong
// side. This test puts the camera inside the orbit, where half the plan is
// behind it, and checks the pick stays on the visible half.
// ============================================================================
SW_TEST(TheMapCanPickAMomentOffTheTrajectory)
{
    SystemFixture fixture;
    CelestialIndex index;
    index.rebuild(fixture.world);
    const i32 terraIndex = index.indexOf(fixture.terra);

    WorldVec3 terraPosition{};
    WorldVec3 terraVelocity{};
    index.stateAt(terraIndex, 0.0, terraPosition, &terraVelocity);

    // A circular parking orbit, predicted whole (one closed revolution).
    const f64 circular = phys::kepler::circularOrbitSpeed(kMuTerra, kLeoRadius);
    std::vector<TrajectorySegment> plan;
    predictTrajectory(index, terraPosition + WorldVec3{kLeoRadius, 0.0, 0.0},
                      terraVelocity + WorldVec3{0.0, 0.0, -circular}, 0.0,
                      PredictionSettings{}, plan);
    SW_CHECK(!plan.empty());
    SW_CHECK(plan[0].endReason == SegmentEnd::Closed);
    const f64 period = plan[0].endTime - plan[0].startTime;

    // The real camera, with the real Vulkan-convention projection: looking
    // straight down at Terra from well outside the orbit, as the map does.
    Camera camera;
    camera.setPerspective(1.047f, 1.0e5f, 2.0e12f); // 60 degrees
    camera.setAspectRatio(16.0f / 9.0f);
    const WorldVec3 eye = terraPosition + WorldVec3{0.0, 4.0e7, 0.0};
    camera.setPosition(eye);
    {
        const Vec3 forward = Vec3(glm::normalize(terraPosition - eye));
        const Vec3 right = glm::normalize(glm::cross(forward, Vec3{0, 0, 1}));
        const Vec3 up = glm::cross(right, forward);
        camera.setOrientation(glm::quat_cast(Mat3{right, up, -forward}));
    }
    const Mat4 viewProjection = camera.viewProjectionCameraRelative();

    // Take a known moment, put it on screen, and ask for it back.
    auto ndcOf = [&](f64 when) {
        WorldVec3 relative{};
        phys::kepler::evaluate(plan[0].orbit, when, relative);
        const Vec3 cameraRelative =
            Vec3((index.positionAt(terraIndex, 0.0) + relative) - camera.position());
        const Vec4 clip = viewProjection * Vec4(cameraRelative, 1.0f);
        SW_CHECK(clip.w > 0.0f);
        return Vec2{clip.x / clip.w, clip.y / clip.w};
    };

    for (const f64 fraction : {0.13, 0.37, 0.62, 0.88})
    {
        const f64 wanted = plan[0].startTime + period * fraction;
        f64 picked = 0.0;
        f32 distance = 0.0f;
        SW_CHECK(timeNearestScreenPoint(index, plan, viewProjection, camera.position(),
                                        0.0, ndcOf(wanted), 320, picked, distance));
        // Within one sample of the drawn line — that IS the resolution the
        // player is pointing at.
        SW_CHECK(std::abs(picked - wanted) < period / 160.0);
        SW_CHECK(distance < 0.01f);
    }

    // ---- and now the trap: the camera INSIDE the orbit -------------------
    // Half the plan is behind it. A point sampled from the near half must
    // still win, and nothing from the far half may be handed back.
    {
        Camera inside;
        inside.setPerspective(1.047f, 100.0f, 1.0e9f);
        inside.setAspectRatio(16.0f / 9.0f);
        inside.setPosition(terraPosition);
        // Looking along +X, so the orbit's +X side is in front and the -X
        // side is squarely behind.
        inside.setOrientation(glm::quat_cast(
            Mat3{Vec3{0, 0, -1}, Vec3{0, 1, 0}, Vec3{-1, 0, 0}}));
        const Mat4 insideVp = inside.viewProjectionCameraRelative();

        f64 picked = 0.0;
        f32 distance = 0.0f;
        // Aim at the middle of the screen: the visible arc crosses it.
        SW_CHECK(timeNearestScreenPoint(index, plan, insideVp, inside.position(), 0.0,
                                        Vec2{0.0f, 0.0f}, 320, picked, distance));
        WorldVec3 relative{};
        phys::kepler::evaluate(plan[0].orbit, picked, relative);
        // Whatever it picked is IN FRONT of the camera, never the mirrored
        // point from behind it.
        SW_CHECK(glm::dot(relative, WorldVec3(inside.forward())) > 0.0);
    }
}

// ============================================================================
// HOW CLOSE DO I PASS, AND WHERE WILL IT BE BY THEN?
//
// The number that turns a trajectory into a transfer. Two plans that look
// identical on a map — one passing 400 000 km from Luna, one passing 4 000
// — differ by the entire mission, and the only way to tell them apart is to
// minimise the separation of two analytic functions of time.
//
// The second half of the answer matters as much as the first: the body is
// NOT where the map shows it now. A marker for the closest approach has to
// say where the target will have MOVED TO, on the orbit ring the player is
// looking at.
// ============================================================================
SW_TEST(ClosestApproachFindsTheRealMinimum)
{
    SystemFixture fixture;
    CelestialIndex index;
    index.rebuild(fixture.world);
    const i32 terraIndex = index.indexOf(fixture.terra);
    const i32 lunaIndex = index.indexOf(fixture.luna);

    WorldVec3 terraPosition{};
    WorldVec3 terraVelocity{};
    index.stateAt(terraIndex, 0.0, terraPosition, &terraVelocity);

    // An ordinary parking orbit, and Luna a long way off.
    const f64 circular = phys::kepler::circularOrbitSpeed(kMuTerra, kLeoRadius);
    std::vector<TrajectorySegment> plan;
    predictTrajectory(index, terraPosition + WorldVec3{kLeoRadius, 0.0, 0.0},
                      terraVelocity + WorldVec3{0.0, 0.0, -circular}, 0.0,
                      PredictionSettings{}, plan);
    SW_CHECK(!plan.empty());

    const ClosestApproach approach = closestApproachToBody(index, plan, lunaIndex);
    SW_CHECK(approach.valid);

    // ---- it IS the minimum, and it is self-consistent --------------------
    auto separationAt = [&](f64 when) {
        WorldVec3 relative{};
        phys::kepler::evaluate(plan[0].orbit, when, relative);
        const WorldVec3 ours = index.positionAt(plan[0].primaryIndex, when) + relative;
        return glm::length(index.positionAt(lunaIndex, when) - ours);
    };
    SW_CHECK(std::abs(separationAt(approach.timeSeconds) - approach.distanceM) < 1.0);
    const f64 span = plan[0].endTime - plan[0].startTime;
    for (int i = 0; i <= 400; ++i)
    {
        const f64 when = plan[0].startTime + span * (static_cast<f64>(i) / 400.0);
        // No sampled moment beats it by more than a metre.
        SW_CHECK(separationAt(when) > approach.distanceM - 1.0);
    }

    // ...and the relative speed is the real one, not the orbital speed.
    {
        WorldVec3 relative{};
        WorldVec3 relativeVelocity{};
        phys::kepler::evaluate(plan[0].orbit, approach.timeSeconds, relative,
                               &relativeVelocity);
        WorldVec3 primaryPosition{};
        WorldVec3 primaryVelocity{};
        index.stateAt(plan[0].primaryIndex, approach.timeSeconds, primaryPosition,
                      &primaryVelocity);
        WorldVec3 lunaPosition{};
        WorldVec3 lunaVelocity{};
        index.stateAt(lunaIndex, approach.timeSeconds, lunaPosition, &lunaVelocity);
        const f64 expected =
            glm::length((primaryVelocity + relativeVelocity) - lunaVelocity);
        SW_CHECK(std::abs(approach.relativeSpeedMps - expected) < 1.0e-6);
    }

    // ---- WHERE IT WILL BE, in the frame the map draws --------------------
    // Luna's future position comes back relative to TERRA, so adding
    // Terra's position NOW puts the marker on the ring that is on screen —
    // not 7.8 million kilometres along Terra's own year, where Luna will
    // really be.
    SW_CHECK_EQ(approach.targetPrimaryIndex, terraIndex);
    const f64 ringRadius = glm::length(approach.targetRelativePosition);
    SW_CHECK(std::abs(ringRadius - kLunaSma) / kLunaSma < 1.0e-6);
    // Our own marker is on OUR primary in the same way.
    SW_CHECK_EQ(approach.primaryIndex, terraIndex);
    SW_CHECK(std::abs(glm::length(approach.relativePosition) - kLeoRadius) / kLeoRadius <
             1.0e-6);
}

SW_TEST(ClosestApproachSeesTheTransferHit)
{
    // The same aimed Hohmann as the encounter test: Luna is phased to
    // arrive at the ship's apoapsis exactly when the ship does. A plan that
    // reports a near-miss for THAT is a plan nobody could fly.
    const f64 transferSma = 0.5 * (kLeoRadius + kLunaSma);
    const f64 timeOfFlight =
        kPi * std::sqrt(transferSma * transferSma * transferSma / kMuTerra);
    const f64 lunaMeanMotion = std::sqrt(kMuTerra / (kLunaSma * kLunaSma * kLunaSma));
    SystemFixture fixture(kPi - lunaMeanMotion * timeOfFlight);
    CelestialIndex index;
    index.rebuild(fixture.world);
    const i32 terraIndex = index.indexOf(fixture.terra);
    const i32 lunaIndex = index.indexOf(fixture.luna);

    WorldVec3 terraPosition{};
    WorldVec3 terraVelocity{};
    index.stateAt(terraIndex, 0.0, terraPosition, &terraVelocity);
    const f64 periapsisSpeed =
        std::sqrt(kMuTerra * (2.0 / kLeoRadius - 1.0 / transferSma));

    std::vector<TrajectorySegment> plan;
    predictTrajectory(index, terraPosition + WorldVec3{kLeoRadius, 0.0, 0.0},
                      terraVelocity + WorldVec3{0.0, 0.0, -periapsisSpeed}, 0.0,
                      PredictionSettings{}, plan);
    SW_CHECK(plan.size() >= 2);

    const ClosestApproach approach = closestApproachToBody(index, plan, lunaIndex);
    SW_CHECK(approach.valid);
    // Well inside Luna's sphere of influence, near the nominal arrival.
    SW_CHECK(approach.distanceM < kLunaSoi);
    SW_CHECK(approach.timeSeconds > 0.5 * timeOfFlight);
    SW_CHECK(approach.timeSeconds < 1.5 * timeOfFlight);
    // The closest point is on the patch INSIDE Luna's SOI, so the answer is
    // measured from Luna itself — which is what makes an impact readable as
    // an impact.
    SW_CHECK_EQ(approach.primaryIndex, lunaIndex);
    SW_CHECK(std::abs(glm::length(approach.relativePosition) - approach.distanceM) < 1.0);
    // Approaching from a transfer, the relative speed is kilometres per
    // second — this is a flyby, not a rendezvous.
    SW_CHECK(approach.relativeSpeedMps > 500.0);
}

// ============================================================================
// THE READOUT THAT TELLS YOU WHEN TO STOP BURNING.
//
// A maneuver's dv display has exactly one job: reach zero at the moment the
// burn is complete. It did not — it sat at the full planned dv from the
// first second of the burn to the last — and the reason is worth keeping a
// test for, because the wrong formula is the obvious one.
//
// Recompute the target each frame from the CURRENT trajectory and it moves
// with the ship: "my velocity at the node, on the orbit I am now on, plus
// the planned dv" changes by exactly as much as the burn changed the ship.
// The difference never shrinks.
//
// The fix measures against the COASTING velocity of the plan frozen before
// ignition — which also subtracts gravity, worth a kilometre per second
// over a two-minute burn in low orbit.
// ============================================================================
SW_TEST(TheBurnReadoutCountsDownToZero)
{
    SystemFixture fixture;
    CelestialIndex index;
    index.rebuild(fixture.world);
    const i32 terraIndex = index.indexOf(fixture.terra);

    WorldVec3 terraPosition{};
    WorldVec3 terraVelocity{};
    index.stateAt(terraIndex, 0.0, terraPosition, &terraVelocity);
    WorldVec3 position = terraPosition + WorldVec3{kLeoRadius, 0.0, 0.0};
    WorldVec3 velocity =
        terraVelocity +
        WorldVec3{0.0, 0.0, -phys::kepler::circularOrbitSpeed(kMuTerra, kLeoRadius)};

    // The plan, frozen before ignition: coast, and 100 m/s prograde at T+60.
    std::vector<TrajectorySegment> coast;
    predictTrajectory(index, position, velocity, 0.0, PredictionSettings{}, coast);
    constexpr f64 kNodeTime = 60.0;
    constexpr f64 kPlannedDv = 100.0;
    WorldVec3 nodePosition{};
    WorldVec3 nodeVelocity{};
    SW_CHECK(stateOnPrediction(index, coast, kNodeTime, nodePosition, nodeVelocity));
    WorldVec3 primaryPosition{};
    WorldVec3 primaryVelocity{};
    index.stateAt(terraIndex, kNodeTime, primaryPosition, &primaryVelocity);
    const WorldVec3 prograde =
        glm::normalize(nodeVelocity - primaryVelocity);
    const WorldVec3 plannedDv = prograde * kPlannedDv;

    // Before ignition it reads the planned burn exactly — no drift while
    // the ship coasts toward the node, which the naive formula also got
    // wrong (it read 535 m/s for this very case).
    SW_CHECK(std::abs(glm::length(remainingBurn(index, coast, plannedDv, velocity, 0.0)) -
                      kPlannedDv) < 1.0e-6);

    // ---- fly it: gravity, then gravity plus thrust -----------------------
    constexpr f64 kDt = 0.02;   // the physics tick
    constexpr f64 kAccel = 8.0; // m/s^2
    f64 applied = 0.0;
    f64 readingAtIgnition = 0.0;
    f64 worstNaive = 0.0;
    for (f64 t = 0.0; t < 74.0; t += kDt)
    {
        WorldVec3 radial = position - index.positionAt(terraIndex, t);
        const f64 distance = glm::length(radial);
        WorldVec3 acceleration = -radial * (kMuTerra / (distance * distance * distance));
        const bool burning = (t >= kNodeTime) && (applied < kPlannedDv);
        if (burning)
        {
            acceleration += prograde * kAccel;
            applied += kAccel * kDt;
        }
        velocity += acceleration * kDt;
        position += velocity * kDt;

        if (burning)
        {
            if (readingAtIgnition == 0.0)
            {
                readingAtIgnition =
                    glm::length(remainingBurn(index, coast, plannedDv, velocity, t));
            }
            // THE NAIVE FORMULA, computed alongside so the test fails if
            // anyone ever "simplifies" back to it: recompute the plan from
            // the current state and take the difference.
            std::vector<TrajectorySegment> live;
            predictTrajectory(index, position, velocity, t, PredictionSettings{}, live);
            WorldVec3 livePosition{};
            WorldVec3 liveVelocity{};
            if (stateOnPrediction(index, live, std::max(kNodeTime, t), livePosition,
                                  liveVelocity))
            {
                WorldVec3 p{};
                WorldVec3 v{};
                index.stateAt(terraIndex, std::max(kNodeTime, t), p, &v);
                const WorldVec3 naiveTarget =
                    liveVelocity + glm::normalize(liveVelocity - v) * kPlannedDv;
                worstNaive =
                    std::max(worstNaive, glm::length(naiveTarget - velocity));
            }
        }
    }

    // It started the burn at the full planned dv...
    SW_CHECK(std::abs(readingAtIgnition - kPlannedDv) < 1.0);
    // ...and finished at zero, having burned exactly the plan.
    const f64 finalReading =
        glm::length(remainingBurn(index, coast, plannedDv, velocity, 74.0));
    SW_CHECK(finalReading < 1.0);
    // The naive formula, meanwhile, never came down off the full dv — this
    // is the bug, measured, so that it cannot come back unnoticed.
    SW_CHECK(worstNaive > 0.99 * kPlannedDv);
}

// ============================================================================
// THE RANGE CAP ON A PLAN THAT NEVER ENDS
//
// A hyperbolic escape from the outermost body has no sphere of influence to
// leave and no revolution to close, so the scan ran to the twenty-year horizon
// — a line reaching a hundred billion kilometres, sampled thousands of times
// and then split into thousands of screen-space boxes, because every chord of
// it spans four orders of magnitude of camera distance. Placing a maneuver
// node on one visibly hitched the frame.
//
// Two things have to hold for the fix to be a fix rather than a truncation:
// the capped plan must STOP at the range asked for, and a plan that has a real
// event before that range must be completely unaffected.
// ============================================================================
SW_TEST(AnEscapeTrajectoryStopsAtTheRangeCapAndABoundOneDoesNot)
{
    ecs::World world;
    const ecs::Entity sol = world.createEntity();
    {
        TransformComponent transform{};
        world.addComponent(sol, transform);
        phys::GravitySourceComponent gravity{kMuSol, 6.957e8};
        world.addComponent(sol, gravity); // no SOI: a lone star owns all space
        world.addComponent(sol, makeCelestialBody("SOL"));
    }
    CelestialIndex index;
    index.rebuild(world);

    // Well clear of the sun and comfortably above escape speed there.
    const WorldVec3 position{kTerraSma, 0.0, 0.0};
    const f64 escapeSpeed = std::sqrt(2.0 * kMuSol / kTerraSma);
    const WorldVec3 velocity{0.0, 0.0, escapeSpeed * 1.4};

    PredictionSettings capped{};
    capped.maxRangeMeters = 1.0e13;
    std::vector<TrajectorySegment> plan;
    predictTrajectory(index, position, velocity, 0.0, capped, plan);

    SW_CHECK(plan.size() == 1);
    SW_CHECK(plan[0].endReason == SegmentEnd::RangeLimit);
    // It stopped AT the cap, not somewhere near it: the event time is refined
    // by bisection, so the radius there is the cap to within a part in 1e4.
    WorldVec3 end{};
    phys::kepler::evaluate(plan[0].orbit, plan[0].endTime, end);
    const f64 endRadius = glm::length(end);
    SW_CHECK(std::abs(endRadius - capped.maxRangeMeters) < capped.maxRangeMeters * 1.0e-4);
    // And it got there in a finite, sensible time rather than running to the
    // twenty-year horizon.
    SW_CHECK(plan[0].endTime > 0.0);
    SW_CHECK(plan[0].endTime < capped.horizonSeconds);

    // A BOUND ORBIT INSIDE THE CAP IS UNTOUCHED. Terra's own circular orbit
    // never reaches 1e13 m, so the cap must not appear anywhere in its plan —
    // it still closes after one revolution, as it always did.
    const f64 circular = std::sqrt(kMuSol / kTerraSma);
    std::vector<TrajectorySegment> bound;
    predictTrajectory(index, position, WorldVec3{0.0, 0.0, circular}, 0.0, capped, bound);
    SW_CHECK(bound.size() == 1);
    SW_CHECK(bound[0].endReason == SegmentEnd::Closed);
}

// ============================================================================
// ...AND THE ARC IS WALKED BY ANOMALY WHICHEVER SIDE OF PERIAPSIS IT STARTS
//
// This is the freeze the player reported: "when I accelerate a lot (+50000
// delta v) the game freezes and then judders, and the counter still says 200
// fps". Two hundred frames a second and a freeze cannot both be true of the
// same frame, so it was never a frame-rate problem — it was one main-thread
// call, four times a second, that took a sixth of a second.
//
// The call was this one. The anomaly walk that bounds a hyperbolic escape was
// refused whenever the craft was still falling toward periapsis, because
// acosh has one branch and the code read a positive anomaly as "outbound".
// A burn made anywhere on the sunward side of an orbit gives exactly that
// state, so the plan fell back on walking twenty years of time at the
// hyperbola's own hour-and-a-half step: sixty-six thousand samples with every
// planet probed at each. Measured 125 ms leaving Terra at +50 km/s, against
// 9 ms for the same burn a few degrees later.
//
// The assertion is the sample count rather than a stopwatch, because that is
// the quantity that actually differs — both versions return the same segments,
// ending in the same place, for the same reason.
// ============================================================================
SW_TEST(AnInboundHyperbolaIsScannedAsCheaplyAsAnOutboundOne)
{
    ecs::World world;
    const ecs::Entity sol = world.createEntity();
    {
        TransformComponent transform{};
        world.addComponent(sol, transform);
        world.addComponent(sol, phys::GravitySourceComponent{kMuSol, 6.957e8});
        world.addComponent(sol, makeCelestialBody("SOL"));
    }
    CelestialIndex index;
    index.rebuild(world);

    // Three astronomical units out, well above escape speed, and pointed
    // INWARD: this arc falls to periapsis at 0.9 au before it climbs out to
    // the range cap. Nothing about it is unusual — it is what a burn made on
    // the day side of any orbit produces.
    const f64 radius = 3.0 * kTerraSma;
    const f64 escapeSpeed = std::sqrt(2.0 * kMuSol / radius);
    const WorldVec3 position{radius, 0.0, 0.0};
    const WorldVec3 inward = glm::normalize(WorldVec3{-0.9, 0.0, 0.436});
    const WorldVec3 outward = glm::normalize(WorldVec3{0.9, 0.0, 0.436});

    PredictionSettings settings{};
    settings.maxRangeMeters = 1.0e13;
    std::vector<TrajectorySegment> plan;
    PredictionStats inboundStats{};
    PredictionStats outboundStats{};
    predictTrajectory(index, position, inward * (escapeSpeed * 1.4), 0.0, settings, plan,
                      &inboundStats);
    SW_CHECK(plan.size() == 1);
    // It still says the same thing it always did: the arc runs out of room.
    SW_CHECK(plan[0].endReason == SegmentEnd::RangeLimit);
    WorldVec3 end{};
    phys::kepler::evaluate(plan[0].orbit, plan[0].endTime, end);
    SW_CHECK(std::abs(glm::length(end) - settings.maxRangeMeters) <
             settings.maxRangeMeters * 1.0e-4);
    // ...and it goes THROUGH periapsis to get there, which is the part the old
    // window could not express.
    WorldVec3 periapsisSide{};
    phys::kepler::evaluate(plan[0].orbit, plan[0].endTime * 0.05, periapsisSide);
    SW_CHECK(glm::length(periapsisSide) < radius);

    predictTrajectory(index, position, outward * (escapeSpeed * 1.4), 0.0, settings, plan,
                      &outboundStats);
    SW_CHECK(plan.size() == 1);
    SW_CHECK(plan[0].endReason == SegmentEnd::RangeLimit);

    // THE POINT. One extra revolution's worth of samples is the whole budget
    // an anomaly walk is allowed; the uniform-in-time fallback spent sixteen
    // times that on this very arc.
    SW_CHECK(outboundStats.samples <= settings.samplesPerRevolution + 1);
    SW_CHECK(inboundStats.samples <= settings.samplesPerRevolution + 1);
}

// ============================================================================
// AND THE ARC IS WALKED BY ANOMALY, NOT BY TIME
//
// The step for a hyperbola is a fraction of its own time scale — about an hour
// for a solar escape — while the arc it has to cover is years of it. Sampled
// uniformly in time that is seventy thousand probes with every child body
// tested at each, four times a second, for a plan the player creates by
// pressing one key. Measured at 341 ms per call before any of this, 138 ms
// with the range cap alone, and 8 ms once the samples were spaced by
// hyperbolic anomaly instead.
//
// The saving is only real if the coverage survives it, and that is what this
// checks: a hyperbolic flyby whose exit and encounter were found before must
// still find them, to the same precision.
// ============================================================================
SW_TEST(SpacingSamplesByAnomalyStillCatchesTheSoiExit)
{
    ecs::World world;
    const ecs::Entity sol = world.createEntity();
    {
        TransformComponent transform{};
        world.addComponent(sol, transform);
        world.addComponent(sol, phys::GravitySourceComponent{kMuSol, 6.957e8});
        world.addComponent(sol, makeCelestialBody("SOL"));
    }
    const phys::KeplerOrbit terraOrbit =
        phys::kepler::fromElements(kMuSol, kTerraSma, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const ecs::Entity terra = world.createEntity();
    {
        TransformComponent transform{};
        phys::kepler::evaluate(terraOrbit, 0.0, transform.position);
        world.addComponent(terra, transform);
        phys::GravitySourceComponent gravity{kMuTerra, 6.371e6};
        gravity.soiRadius = 9.24e8;
        world.addComponent(terra, gravity);
        world.addComponent(terra, makeCelestialBody("TERRA", sol, &terraOrbit));
    }
    CelestialIndex index;
    index.rebuild(world);

    // A craft leaving Terra on a hyperbola: 300 km up, well over escape speed.
    WorldVec3 terraPosition{};
    WorldVec3 terraVelocity{};
    index.stateAt(index.indexOf(terra), 0.0, terraPosition, &terraVelocity);
    const f64 radius = 6.371e6 + 3.0e5;
    const f64 escape = std::sqrt(2.0 * kMuTerra / radius);
    const WorldVec3 position = terraPosition + WorldVec3{radius, 0.0, 0.0};
    const WorldVec3 velocity = terraVelocity + WorldVec3{0.0, 0.0, escape * 1.25};

    PredictionSettings settings{};
    std::vector<TrajectorySegment> plan;
    predictTrajectory(index, position, velocity, 0.0, settings, plan);

    // It leaves Terra's sphere of influence and hands off to Sol.
    SW_CHECK(plan.size() >= 2);
    SW_CHECK(plan[0].endReason == SegmentEnd::SoiExit);
    // ...AT the sphere of influence, to within a part in ten thousand. This is
    // the number the anomaly walk could have blunted and did not.
    WorldVec3 exitPoint{};
    phys::kepler::evaluate(plan[0].orbit, plan[0].endTime, exitPoint);
    SW_CHECK(std::abs(glm::length(exitPoint) - 9.24e8) < 9.24e8 * 1.0e-4);
}

// ============================================================================
// F49 — AND THE TARGET DOES NOT HAVE TO BE A WORLD
//
// A rendezvous is with a craft far more often than with a planet, and the
// minimiser never cared which: all it asks is where the target will be. A
// body answers from its catalogue; a craft answers from a conic fitted to its
// state vectors, which is the same orbit the map already draws round it.
// ============================================================================
SW_TEST(ACraftIsAsTargetableAsAWorld)
{
    SystemFixture fixture;
    CelestialIndex index;
    index.rebuild(fixture.world);
    const i32 terraIndex = index.indexOf(fixture.terra);

    WorldVec3 terraPosition{};
    WorldVec3 terraVelocity{};
    index.stateAt(terraIndex, 0.0, terraPosition, &terraVelocity);

    // A station in a circular orbit, and a ship in the SAME orbit a quarter
    // of a revolution behind it: the classic rendezvous geometry, where the
    // separation is constant and the two never meet without a burn.
    const f64 radius = kLeoRadius + 200.0e3;
    const f64 speed = phys::kepler::circularOrbitSpeed(kMuTerra, radius);
    const WorldVec3 stationPosition = terraPosition + WorldVec3{radius, 0.0, 0.0};
    const WorldVec3 stationVelocity = terraVelocity + WorldVec3{0.0, 0.0, -speed};
    const TargetPath station =
        craftTargetPath(index, terraIndex, stationPosition, stationVelocity, 0.0);
    SW_CHECK(station.hasOrbit);
    SW_CHECK(station.primaryIndex == terraIndex);
    SW_CHECK(station.bodyRadius == 0.0); // you dock with it, you do not crash into it

    std::vector<TrajectorySegment> plan;
    predictTrajectory(index, terraPosition + WorldVec3{0.0, 0.0, radius},
                      terraVelocity + WorldVec3{speed, 0.0, 0.0}, 0.0,
                      PredictionSettings{}, plan);
    SW_CHECK(!plan.empty());

    const ClosestApproach approach = closestApproachToPath(index, plan, station);
    SW_CHECK(approach.valid);
    // A quarter turn apart on the same ring: the closest they ever come is
    // the chord, r*sqrt(2), and no burn in this test changes that.
    const f64 chord = radius * 1.41421356;
    SW_CHECK(std::abs(approach.distanceM - chord) < chord * 0.02);
    // Same ring, same speed: the separation never closes, so the approach is
    // not a near miss and the relative speed is the chord's, not zero.
    SW_CHECK(approach.relativeSpeedMps > 1.0);
    // The marker is placed in the target's own frame, so the map draws it on
    // the ring rather than out in the world.
    SW_CHECK(approach.targetPrimaryIndex == terraIndex);
    SW_CHECK(std::abs(glm::length(approach.targetRelativePosition) - radius) <
             radius * 0.01);

    // A CRAFT SITTING STILL IN NO FRAME AT ALL still answers, rather than
    // inventing an orbit: this is the state a landed craft is in for the
    // instant before the ground rotates it.
    const TargetPath adrift = craftTargetPath(index, -1, stationPosition,
                                              WorldVec3{0.0}, 0.0);
    SW_CHECK(!adrift.hasOrbit);
    SW_CHECK(glm::length(adrift.staticPosition - stationPosition) < 1.0);

    // ...and a body's path through the same entry point is the body's own
    // orbit, so the two kinds of target cannot drift apart in behaviour.
    const TargetPath luna = bodyTargetPath(index, index.indexOf(fixture.luna));
    SW_CHECK(luna.hasOrbit);
    SW_CHECK(luna.primaryIndex == terraIndex);
    SW_CHECK(luna.bodyRadius > 1.0e6);
    const ClosestApproach viaPath = closestApproachToPath(index, plan, luna);
    const ClosestApproach viaBody =
        closestApproachToBody(index, plan, index.indexOf(fixture.luna));
    SW_CHECK(viaPath.valid == viaBody.valid);
    SW_CHECK(std::abs(viaPath.distanceM - viaBody.distanceM) < 1.0);
}
