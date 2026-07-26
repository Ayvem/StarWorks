// ============================================================================
// PhysicsTests.cpp — Kepler orbits, Newtonian integrator and the
// simulation bubble (rails <-> dynamic conversions).
// ============================================================================

#include "TestFramework.hpp"

#include <ECS/CommandBuffer.hpp>
#include <ECS/World.hpp>
#include <Physics/Kepler.hpp>
#include <Physics/PhysicsSystems.hpp>
#include <Scene/TransformComponents.hpp>
#include <Simulation/Simulation.hpp>

#include <glm/gtx/quaternion.hpp> // glm::rotation, for attitude fixtures

#include <cmath>
#include <limits>

namespace
{
    constexpr sw::f64 kMuTerra = 3.986004418e14;
    constexpr sw::f64 kLeoRadius = 6.771e6; // 400 km altitude

    [[nodiscard]] sw::f64 relativeError(const sw::WorldVec3& a, const sw::WorldVec3& b,
                                        sw::f64 scale)
    {
        return glm::length(a - b) / scale;
    }
} // namespace

using namespace sw;
using namespace sw::phys;

SW_TEST(KeplerCircularOrbitGeometry)
{
    const KeplerOrbit orbit =
        kepler::fromElements(kMuTerra, kLeoRadius, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Radius stays constant; a quarter period advances the anomaly by 90°.
    const f64 period = kepler::period(orbit);
    SW_CHECK(std::abs(period - 5546.0) < 10.0); // real LEO period ~92.4 min

    WorldVec3 p0{};
    WorldVec3 pQuarter{};
    WorldVec3 v0{};
    kepler::evaluate(orbit, 0.0, p0, &v0);
    kepler::evaluate(orbit, period * 0.25, pQuarter);

    SW_CHECK(std::abs(glm::length(p0) - kLeoRadius) / kLeoRadius < 1.0e-12);
    SW_CHECK(std::abs(glm::length(pQuarter) - kLeoRadius) / kLeoRadius < 1.0e-9);
    // Perpendicular position vectors after a quarter of a circular orbit.
    SW_CHECK(std::abs(glm::dot(glm::normalize(p0), glm::normalize(pQuarter))) < 1.0e-6);
    // Circular speed = sqrt(mu/r) ~ 7.67 km/s in LEO.
    SW_CHECK(std::abs(glm::length(v0) - kepler::circularOrbitSpeed(kMuTerra, kLeoRadius)) <
             1.0e-6);
    // Full period returns to the start.
    WorldVec3 pFull{};
    kepler::evaluate(orbit, period, pFull);
    SW_CHECK(relativeError(pFull, p0, kLeoRadius) < 1.0e-9);
}

SW_TEST(KeplerStateVectorRoundTrip)
{
    // An inclined, eccentric orbit built from elements...
    const KeplerOrbit original =
        kepler::fromElements(kMuTerra, 8.0e6, 0.3, 0.4, 1.1, 2.2, 0.7, /*epoch=*/100.0);

    // ...sampled mid-flight into state vectors...
    const f64 sampleTime = 1234.5;
    WorldVec3 position{};
    WorldVec3 velocity{};
    kepler::evaluate(original, sampleTime, position, &velocity);

    // ...must reconstruct an orbit with identical future motion.
    KeplerOrbit rebuilt{};
    SW_CHECK(kepler::fromStateVectors(kMuTerra, position, velocity, sampleTime, rebuilt));
    SW_CHECK(std::abs(rebuilt.semiMajorAxis - original.semiMajorAxis) /
                 original.semiMajorAxis <
             1.0e-9);
    SW_CHECK(std::abs(rebuilt.eccentricity - original.eccentricity) < 1.0e-9);

    for (const f64 probeTime : {sampleTime, sampleTime + 500.0, sampleTime + 4321.0})
    {
        WorldVec3 expected{};
        WorldVec3 actual{};
        kepler::evaluate(original, probeTime, expected);
        kepler::evaluate(rebuilt, probeTime, actual);
        SW_CHECK(relativeError(actual, expected, original.semiMajorAxis) < 1.0e-8);
    }
}

SW_TEST(KeplerRejectsUnboundTrajectoriesByDefault)
{
    KeplerOrbit orbit{};
    const WorldVec3 position{kLeoRadius, 0.0, 0.0};
    // Twice escape velocity: clearly hyperbolic.
    const f64 escape = std::sqrt(2.0 * kMuTerra / kLeoRadius);
    SW_CHECK(!kepler::fromStateVectors(kMuTerra, position, {0.0, 0.0, 2.0 * escape}, 0.0,
                                       orbit));
}

SW_TEST(KeplerHyperbolicOrbit)
{
    // A hyperbolic flyby: 1.3x escape speed, tangential, opted in.
    const WorldVec3 position{kLeoRadius, 0.0, 0.0};
    const f64 escape = std::sqrt(2.0 * kMuTerra / kLeoRadius);
    const WorldVec3 velocity{0.0, 0.0, 1.3 * escape};

    KeplerOrbit orbit{};
    SW_CHECK(kepler::fromStateVectors(kMuTerra, position, velocity, /*epoch=*/50.0, orbit,
                                      /*allowHyperbolic=*/true));
    SW_CHECK(orbit.isHyperbolic());
    SW_CHECK(orbit.semiMajorAxis < 0.0);
    SW_CHECK(kepler::apoapsis(orbit) == std::numeric_limits<f64>::infinity());
    SW_CHECK(kepler::periapsis(orbit) > 0.0 && kepler::periapsis(orbit) <= kLeoRadius);

    // The evaluation must reproduce the epoch state exactly...
    WorldVec3 p0{};
    WorldVec3 v0{};
    kepler::evaluate(orbit, 50.0, p0, &v0);
    SW_CHECK(relativeError(p0, position, kLeoRadius) < 1.0e-9);
    SW_CHECK(glm::length(v0 - velocity) / glm::length(velocity) < 1.0e-9);

    // ...and conserve specific orbital energy far along the escape leg
    // (also backwards in time, on the inbound branch).
    const f64 energy0 = 0.5 * glm::dot(velocity, velocity) - kMuTerra / kLeoRadius;
    for (const f64 t : {-20000.0, 1000.0, 5000.0, 200000.0})
    {
        WorldVec3 p{};
        WorldVec3 v{};
        kepler::evaluate(orbit, 50.0 + t, p, &v);
        const f64 energy = 0.5 * glm::dot(v, v) - kMuTerra / glm::length(p);
        SW_CHECK(std::abs(energy - energy0) / std::abs(energy0) < 1.0e-9);
        SW_CHECK(glm::length(p) >= kepler::periapsis(orbit) * 0.999999);
    }

    // Asymptotic speed: v_inf = sqrt(mu / |a|).
    WorldVec3 pFar{};
    WorldVec3 vFar{};
    kepler::evaluate(orbit, 50.0 + 5.0e6, pFar, &vFar);
    const f64 vInf = std::sqrt(kMuTerra / std::abs(orbit.semiMajorAxis));
    SW_CHECK(std::abs(glm::length(vFar) - vInf) / vInf < 1.0e-3);
}

SW_TEST(GravityIntegratorHoldsACircularOrbit)
{
    ecs::World world;

    // Terra at the origin.
    {
        const ecs::Entity terra = world.createEntity();
        world.addComponent(terra, TransformComponent{});
        world.addComponent(terra, GravitySourceComponent{kMuTerra});
    }

    // Dynamic body with exact circular velocity.
    const ecs::Entity body = world.createEntity();
    {
        TransformComponent transform{};
        transform.position = {0.0, 0.0, kLeoRadius};
        world.addComponent(body, transform);
        const f64 speed = kepler::circularOrbitSpeed(kMuTerra, kLeoRadius);
        world.addComponent(body, DynamicBodyComponent{{speed, 0.0, 0.0}, 1000.0});
    }

    GravityIntegrationSystem gravity;
    constexpr f32 kDt = 0.02f; // Physics lane step
    constexpr int kTicks = 5000; // 100 simulated seconds
    for (int i = 0; i < kTicks; ++i)
    {
        gravity.update(world, kDt);
    }

    // Semi-implicit Euler on a circular orbit: radius drift must stay tiny.
    const f64 radius = glm::length(world.getComponent<TransformComponent>(body).position);
    SW_CHECK(std::abs(radius - kLeoRadius) / kLeoRadius < 5.0e-4);

    // Specific orbital energy must match the circular value closely.
    const auto& dynamicBody = world.getComponent<DynamicBodyComponent>(body);
    const f64 energy =
        0.5 * glm::dot(dynamicBody.velocity, dynamicBody.velocity) - kMuTerra / radius;
    const f64 expectedEnergy = -kMuTerra / (2.0 * kLeoRadius);
    SW_CHECK(std::abs(energy - expectedEnergy) / std::abs(expectedEnergy) < 1.0e-3);
}

SW_TEST(AtmosphereDragAndSolidGround)
{
    ecs::World world;

    // Small test body: radius 1 km, weak gravity, thick atmosphere.
    {
        const ecs::Entity body = world.createEntity();
        world.addComponent(body, TransformComponent{});
        world.addComponent(body, GravitySourceComponent{1.0e6, 1000.0}); // ~1 m/s^2
        world.addComponent(body, AtmosphereComponent{1.225, 500.0, 5000.0});
    }

    // Falling object released 200 m above the surface.
    const ecs::Entity probe = world.createEntity();
    {
        TransformComponent transform{};
        transform.position = {0.0, 1200.0, 0.0};
        world.addComponent(probe, transform);
        DynamicBodyComponent body{};
        body.ballisticFactor = 0.01;
        world.addComponent(probe, body);
    }

    GravityIntegrationSystem gravity;
    SurfaceInteractionSystem::Config config{};
    SurfaceInteractionSystem surface(config);

    constexpr f32 kDt = 0.02f;
    f64 maxFallSpeed = 0.0;
    bool landed = false;
    for (int tick = 0; tick < 5000 && !landed; ++tick) // up to 100 s
    {
        gravity.update(world, kDt);
        surface.update(world, kDt);
        const auto& body = world.getComponent<DynamicBodyComponent>(probe);
        maxFallSpeed = std::max(maxFallSpeed, glm::length(body.velocity));
        landed = body.isGrounded != 0;
    }

    SW_CHECK(landed);
    // Drag must keep the fall well below the vacuum impact speed (~20 m/s
    // for 200 m at 1 m/s^2): terminal velocity here is a few m/s.
    SW_CHECK(maxFallSpeed < 15.0);

    // Resting on the surface: radius equals the body radius, speed ~ 0.
    const auto& transform = world.getComponent<TransformComponent>(probe);
    SW_CHECK(std::abs(glm::length(transform.position) - 1000.0) < 1.0e-6);
    for (int tick = 0; tick < 250; ++tick) // 5 more seconds at rest
    {
        gravity.update(world, kDt);
        surface.update(world, kDt);
    }
    SW_CHECK(glm::length(world.getComponent<DynamicBodyComponent>(probe).velocity) < 0.05);
    SW_CHECK(world.getComponent<DynamicBodyComponent>(probe).isGrounded != 0);
}

SW_TEST(TerrainCollisionLandsOnTheHeightfield)
{
    ecs::World world;

    // A Terra-like body with procedural terrain.
    planet::TerrainComponent terrain{};
    terrain.seed = 1337u;
    terrain.octaves = 5;
    terrain.frequency = 2.3f;
    terrain.amplitude = 9000.0f;
    terrain.seaLevelFraction = 0.50f;
    constexpr f64 kRadius = 6.371e6;
    {
        const ecs::Entity body = world.createEntity();
        world.addComponent(body, TransformComponent{});
        world.addComponent(body, GravitySourceComponent{kMuTerra, kRadius});
        world.addComponent(body, terrain);
    }

    // Find a direction with REAL elevation (a mountain), then drop a probe
    // 500 m above that mountain top.
    Vec3 landDir{0.0f, 0.0f, 1.0f};
    f64 bestElevation = 0.0;
    for (u32 i = 0; i < 400; ++i)
    {
        const f32 a = static_cast<f32>(i) * 0.137f;
        const f32 b = static_cast<f32>(i) * 0.071f;
        const Vec3 dir = glm::normalize(
            Vec3{std::cos(a) * std::cos(b), std::sin(b), std::sin(a) * std::cos(b)});
        const f64 elevation = planet::terrainElevation(terrain, dir);
        if (elevation > bestElevation)
        {
            bestElevation = elevation;
            landDir = dir;
        }
    }
    SW_CHECK(bestElevation > 500.0); // the noise really makes mountains

    const ecs::Entity probe = world.createEntity();
    {
        TransformComponent transform{};
        transform.position = WorldVec3(landDir) * (kRadius + bestElevation + 500.0);
        world.addComponent(probe, transform);
        world.addComponent(probe, DynamicBodyComponent{{0.0, 0.0, 0.0}, 500.0});
    }

    GravityIntegrationSystem gravity;
    SurfaceInteractionSystem::Config config{};
    SurfaceInteractionSystem surface(config);
    constexpr f32 kDt = 0.02f;
    bool landed = false;
    for (int tick = 0; tick < 4000 && !landed; ++tick)
    {
        gravity.update(world, kDt);
        surface.update(world, kDt);
        landed = world.getComponent<DynamicBodyComponent>(probe).isGrounded != 0;
    }
    SW_CHECK(landed);

    // At rest ON the mountain: radius = sea level + local elevation, NOT
    // the bare sphere. (The clamp point moved slightly downhill during the
    // fall; compare against the elevation AT the resting direction.)
    const auto& transform = world.getComponent<TransformComponent>(probe);
    const f64 restRadius = glm::length(transform.position);
    const Vec3 restDir = Vec3(glm::normalize(transform.position));
    const f64 restElevation = planet::terrainElevation(terrain, restDir);
    SW_CHECK(restElevation > 100.0);
    SW_CHECK(std::abs(restRadius - (kRadius + restElevation)) < 1.0);
}

SW_TEST(GroundCoRotatesWithTheSpinningBody)
{
    ecs::World world;

    // A spinning Terra-like body, translating through space as well.
    const WorldVec3 bodyVelocity{100.0, 0.0, 0.0};
    const WorldVec3 spin{0.0, 7.2921e-5, 0.0}; // Earth sidereal rate, +Y
    constexpr f64 kRadius = 6.371e6;
    {
        const ecs::Entity terra = world.createEntity();
        world.addComponent(terra, TransformComponent{});
        GravitySourceComponent source{kMuTerra, kRadius};
        source.worldVelocity = bodyVelocity;
        source.angularVelocity = spin;
        world.addComponent(terra, source);
    }

    // An object dropped AT REST (world frame) on the equator (+X).
    const ecs::Entity probe = world.createEntity();
    {
        TransformComponent transform{};
        transform.position = {kRadius, 0.0, 0.0};
        world.addComponent(probe, transform);
        world.addComponent(probe, DynamicBodyComponent{{0.0, 0.0, 0.0}, 1000.0});
    }

    SurfaceInteractionSystem::Config config{};
    SurfaceInteractionSystem surface(config);
    constexpr f32 kDt = 0.02f;
    for (int tick = 0; tick < 500; ++tick) // 10 s of ground friction
    {
        surface.update(world, kDt);
    }

    // Friction must drag it INTO the rotating surface frame: final velocity
    // equals body translation + omega x r (the local surface velocity,
    // ~465 m/s eastward for Earth), NOT zero.
    const WorldVec3 expected =
        bodyVelocity + glm::cross(spin, WorldVec3{kRadius, 0.0, 0.0});
    const auto& body = world.getComponent<DynamicBodyComponent>(probe);
    SW_CHECK(glm::length(body.velocity - expected) < 0.05);
    SW_CHECK(body.isGrounded != 0);
    SW_CHECK(glm::length(expected - bodyVelocity) > 400.0); // spin really counted
}

SW_TEST(BubbleConvertsRailsToDynamicAndBack)
{
    ecs::World world;
    ecs::EntityCommandBuffer commands;
    sim::Simulation simulation({{"Logistics", 10.0f, 4}});

    // Terra (at the world origin; default SOI is effectively infinite).
    const ecs::Entity terra = world.createEntity();
    world.addComponent(terra, TransformComponent{});
    world.addComponent(terra, GravitySourceComponent{kMuTerra});

    // On-rails object at +Z LEO, Terra-relative orbit, distinctive payload.
    const KeplerOrbit orbit = kepler::fromElements(
        kMuTerra, kLeoRadius, 0.0, 0.0, 0.0, 0.0, /*M0=*/4.71238898038468986, 0.0);
    const ecs::Entity object = world.createEntity();
    {
        TransformComponent transform{};
        kepler::evaluate(orbit, 0.0, transform.position);
        world.addComponent(object, transform);
        world.addComponent(object, PreviousTransformComponent{});
        OnRailsComponent rails{};
        rails.orbit = orbit;
        rails.primary = terra;
        rails.dynamicMass = 7777.0; // must survive the round trips below
        world.addComponent(object, rails);
    }

    SimulationBubbleSystem::Config config{};
    config.enterRadius = 1.0e4;
    config.exitRadius = 1.5e4;

    sim::SimulationLane& lane = *simulation.findLane("Logistics");
    auto rails = std::make_unique<RailsSystem>(lane);
    auto bubble = std::make_unique<SimulationBubbleSystem>(commands, lane, config);
    auto* bubblePtr = bubble.get();
    auto& scheduler = lane.scheduler();
    scheduler.addSystem(std::move(rails));
    scheduler.addSystem(std::move(bubble));

    // ---- focus near the object: it must become dynamic --------------------
    const WorldVec3 objectPosition =
        world.getComponent<TransformComponent>(object).position;
    bubblePtr->setFocus(objectPosition + WorldVec3{500.0, 0.0, 0.0});

    simulation.advance(world, 0.1f, nullptr); // one Logistics tick
    commands.playback(world);

    SW_CHECK(world.hasComponent<DynamicBodyComponent>(object));
    SW_CHECK(!world.hasComponent<OnRailsComponent>(object));
    // Continuity: the hand-off position equals the ANALYTIC position at the
    // conversion time (the object legitimately moved ~767 m during the
    // 0.1 s Logistics tick), and the velocity is the circular speed.
    const auto& converted = world.getComponent<DynamicBodyComponent>(object);
    SW_CHECK(std::abs(glm::length(converted.velocity) -
                      kepler::circularOrbitSpeed(kMuTerra, kLeoRadius)) < 1.0);
    SW_CHECK_EQ(converted.mass, 7777.0); // payload restored, not defaulted
    WorldVec3 expectedAtConversion{};
    kepler::evaluate(orbit, simulation.findLane("Logistics")->presentSeconds(), expectedAtConversion);
    SW_CHECK(relativeError(world.getComponent<TransformComponent>(object).position,
                           expectedAtConversion, kLeoRadius) < 1.0e-9);

    // ---- focus far away: it must go back on rails ---------------------------
    bubblePtr->setFocus(objectPosition + WorldVec3{1.0e6, 0.0, 0.0});
    simulation.advance(world, 0.1f, nullptr);
    commands.playback(world);

    SW_CHECK(!world.hasComponent<DynamicBodyComponent>(object));
    SW_CHECK(world.hasComponent<OnRailsComponent>(object));

    // The rebuilt rails orbit must continue the motion seamlessly: its
    // evaluation now must match the object's current transform. The primary
    // and the dynamic payload must round-trip too.
    const auto& backOnRails = world.getComponent<OnRailsComponent>(object);
    SW_CHECK(backOnRails.primary == terra);
    SW_CHECK_EQ(backOnRails.dynamicMass, 7777.0);
    WorldVec3 railsPosition{};
    kepler::evaluate(backOnRails.orbit, simulation.findLane("Logistics")->presentSeconds(), railsPosition);
    SW_CHECK(relativeError(railsPosition,
                           world.getComponent<TransformComponent>(object).position,
                           kLeoRadius) < 1.0e-6);

    // ---- force-rails (time warp): dynamic INSIDE the bubble still railifies
    bubblePtr->setFocus(world.getComponent<TransformComponent>(object).position);
    simulation.advance(world, 0.1f, nullptr); // converts back to dynamic
    commands.playback(world);
    SW_CHECK(world.hasComponent<DynamicBodyComponent>(object));

    bubblePtr->setForceRails(true);
    simulation.advance(world, 0.1f, nullptr);
    commands.playback(world);
    SW_CHECK(world.hasComponent<OnRailsComponent>(object));
    SW_CHECK(!world.hasComponent<DynamicBodyComponent>(object));

    // While forced, nothing may come off the rails even at distance zero.
    simulation.advance(world, 0.1f, nullptr);
    commands.playback(world);
    SW_CHECK(world.hasComponent<OnRailsComponent>(object));

    // Releasing force-rails lets the bubble re-acquire it — with the same
    // dynamic payload it had before the warp (a 7777 kg ship must not come
    // back as a 1000 kg default).
    bubblePtr->setForceRails(false);
    bubblePtr->setFocus(world.getComponent<TransformComponent>(object).position);
    simulation.advance(world, 0.1f, nullptr);
    commands.playback(world);
    SW_CHECK(world.hasComponent<DynamicBodyComponent>(object));
    SW_CHECK_EQ(world.getComponent<DynamicBodyComponent>(object).mass, 7777.0);
}

SW_TEST(LandedCraftAnchorsInsteadOfRailing)
{
    // A craft SITTING ON A PLANET must never be converted to Kepler rails
    // (the ground state is a degenerate ellipse that flings it skyward on
    // the first rail tick — the pre-M21 launch-pad-warp bug). The bubble
    // system anchors it to the surface instead, and releases it back to a
    // co-rotating dynamic body when the bubble focus returns.
    ecs::World world;
    ecs::EntityCommandBuffer commands;
    sim::Simulation simulation({{"Logistics", 10.0f, 4}});

    // Terra-like body spinning about +Y.
    const ecs::Entity terra = world.createEntity();
    {
        TransformComponent transform{};
        world.addComponent(terra, transform);
        world.addComponent(terra, PreviousTransformComponent{});
        GravitySourceComponent gravity{3.986e14, 6.371e6};
        gravity.angularVelocity = {0.0, 7.2921e-5, 0.0};
        world.addComponent(terra, gravity);
    }

    // Landed craft on the equator, exactly co-rotating (ground speed 0).
    const ecs::Entity craft = world.createEntity();
    const WorldVec3 surfacePosition{6.372e6, 0.0, 0.0}; // ~1 km terrain
    {
        TransformComponent transform{};
        transform.position = surfacePosition;
        transform.rotation = glm::angleAxis(0.7f, glm::normalize(Vec3{0.2f, 1, 0.3f}));
        world.addComponent(craft, transform);
        world.addComponent(craft, PreviousTransformComponent{});
        DynamicBodyComponent body{};
        body.velocity = glm::cross(WorldVec3{0.0, 7.2921e-5, 0.0}, surfacePosition);
        body.mass = 4321.0;
        world.addComponent(craft, body);
    }
    const Quat originalRotation = world.getComponent<TransformComponent>(craft).rotation;

    SimulationBubbleSystem::Config config{};
    config.enterRadius = 1.0e4;
    config.exitRadius = 1.5e4;
    sim::SimulationLane& lane = *simulation.findLane("Logistics");
    auto anchors = std::make_unique<SurfaceAnchorSystem>();
    auto bubble = std::make_unique<SimulationBubbleSystem>(commands, lane, config);
    auto* bubblePtr = bubble.get();
    lane.scheduler().addSystem(std::move(anchors));
    lane.scheduler().addSystem(std::move(bubble));

    // WARP: everything railifies... except the landed craft, which anchors.
    bubblePtr->setFocus(surfacePosition);
    bubblePtr->setForceRails(true);
    simulation.advance(world, 0.1f, nullptr);
    commands.playback(world);

    SW_CHECK(!world.hasComponent<OnRailsComponent>(craft));
    SW_CHECK(!world.hasComponent<DynamicBodyComponent>(craft));
    SW_CHECK(world.hasComponent<SurfaceAnchorComponent>(craft));
    const auto& anchor = world.getComponent<SurfaceAnchorComponent>(craft);
    SW_CHECK(anchor.body == terra);
    SW_CHECK_EQ(anchor.autoRelease, static_cast<u8>(1));
    SW_CHECK(std::abs(anchor.dynamicMass - 4321.0) < 1.0e-9);

    // Anchored through several warp ticks: the craft stays glued to the
    // (rotating) surface — its body-frame position never drifts.
    for (int i = 0; i < 5; ++i)
    {
        simulation.advance(world, 0.1f, nullptr);
        commands.playback(world);
    }
    SW_CHECK(world.hasComponent<SurfaceAnchorComponent>(craft));
    const WorldVec3 anchoredLocal =
        world.getComponent<SurfaceAnchorComponent>(craft).localPosition;
    SW_CHECK(glm::length(anchoredLocal - surfacePosition) < 1.0);

    // Warp ends with the focus here: the craft wakes up DYNAMIC again,
    // co-rotating with the ground, same mass, same attitude.
    bubblePtr->setForceRails(false);
    simulation.advance(world, 0.1f, nullptr);
    commands.playback(world);

    SW_CHECK(world.hasComponent<DynamicBodyComponent>(craft));
    SW_CHECK(!world.hasComponent<SurfaceAnchorComponent>(craft));
    const auto& released = world.getComponent<DynamicBodyComponent>(craft);
    SW_CHECK(std::abs(released.mass - 4321.0) < 1.0e-9);
    const WorldVec3 expectedVelocity =
        glm::cross(WorldVec3{0.0, 7.2921e-5, 0.0},
                   world.getComponent<TransformComponent>(craft).position);
    SW_CHECK(glm::length(released.velocity - expectedVelocity) < 0.5);
    const Quat releasedRotation = world.getComponent<TransformComponent>(craft).rotation;
    SW_CHECK(std::abs(glm::dot(releasedRotation, originalRotation)) > 0.999f);
}

// ============================================================================
// THE BURIED-TO-THE-WAIST BUG.
//
// Ground contact used to snap a body's ORIGIN onto the terrain, which is
// only correct for an object of zero size. Everything else sank up to its
// origin: a rocket modelled around its centre went half its length into the
// rock, and the EVA capsule — a 2 m body centred on itself — put the
// player's waist at ground level. GroundHullComponent is the missing half
// of the question, and these are its two promises.
// ============================================================================

SW_TEST(GroundClearanceProjectsTheHullAtAnyAttitude)
{
    // A rocket: 1.5 m across, 20 m long along its model +Z (the tail axis),
    // centred on its own middle.
    GroundHullComponent rocket{};
    rocket.centre = Vec3{0.0f, 0.0f, 0.0f};
    rocket.halfExtents = Vec3{1.5f, 1.5f, 10.0f};

    const Vec3 up{0.0f, 1.0f, 0.0f};

    // Standing on its tail: model +Z points down, so it rests on its full
    // half length. A bounding SPHERE would have said the same 10 m here —
    // and then floated it 10 m up when it fell over.
    const Quat tailDown = glm::rotation(Vec3{0.0f, 0.0f, 1.0f}, -up);
    SW_CHECK(std::abs(groundClearance(rocket, tailDown, up) - 10.0) < 1.0e-4);

    // Lying on its side: it rests on its flank, 1.5 m, not on 10 m.
    const Quat onItsSide = glm::rotation(Vec3{0.0f, 0.0f, 1.0f}, Vec3{1.0f, 0.0f, 0.0f});
    SW_CHECK(std::abs(groundClearance(rocket, onItsSide, up) - 1.5) < 1.0e-4);

    // An off-centre hull follows its centre: a lander whose box sits 3 m
    // below its origin reaches 3 m further down.
    GroundHullComponent lander = rocket;
    lander.centre = Vec3{0.0f, 0.0f, 3.0f}; // +Z is down when tail-down
    SW_CHECK(std::abs(groundClearance(lander, tailDown, up) - 13.0) < 1.0e-4);

    // A body with no hull at all keeps the old behaviour exactly: origin on
    // the ground. Nothing regresses for the asteroid or a bare probe.
    SW_CHECK_EQ(groundClearance(GroundHullComponent{}, tailDown, up), 0.0);
}

SW_TEST(ALandedBodyRestsOnItsHullNotItsOrigin)
{
    ecs::World world;
    constexpr f64 kRadius = 6.371e6;
    {
        const ecs::Entity body = world.createEntity();
        world.addComponent(body, TransformComponent{});
        world.addComponent(body, GravitySourceComponent{kMuTerra, kRadius});
    }

    // Two probes dropped side by side on a bare sphere (no terrain, so the
    // ground is exactly kRadius): one shapeless, one 2 m tall and centred
    // on itself — the EVA capsule's geometry. They fall along +Y with an
    // identity attitude, so the capsule's tall axis IS the local vertical.
    const Vec3 dropDirection{0.0f, 1.0f, 0.0f};
    auto dropProbe = [&](bool withHull) {
        const ecs::Entity probe = world.createEntity();
        TransformComponent transform{};
        transform.position = WorldVec3(dropDirection) * (kRadius + 40.0);
        world.addComponent(probe, transform);
        world.addComponent(probe, DynamicBodyComponent{{0.0, 0.0, 0.0}, 500.0});
        if (withHull)
        {
            world.addComponent(probe, GroundHullComponent{Vec3{0.0f},
                                                          Vec3{0.5f, 1.0f, 0.5f}});
        }
        return probe;
    };
    const ecs::Entity bare = dropProbe(false);
    const ecs::Entity standing = dropProbe(true);

    GravityIntegrationSystem gravity;
    SurfaceInteractionSystem surface(SurfaceInteractionSystem::Config{});
    for (int tick = 0; tick < 1000; ++tick)
    {
        gravity.update(world, 0.02f);
        surface.update(world, 0.02f);
    }

    SW_CHECK(world.getComponent<DynamicBodyComponent>(bare).isGrounded != 0);
    SW_CHECK(world.getComponent<DynamicBodyComponent>(standing).isGrounded != 0);

    const f64 bareRadius =
        glm::length(world.getComponent<TransformComponent>(bare).position);
    const f64 standingRadius =
        glm::length(world.getComponent<TransformComponent>(standing).position);

    // The shapeless probe still puts its origin exactly on the ground.
    SW_CHECK(std::abs(bareRadius - kRadius) < 0.01);
    // The 2 m body stands ON the ground: its origin ends up 1 m higher, so
    // its bottom — not its middle — is the thing touching.
    SW_CHECK(std::abs(standingRadius - (kRadius + 1.0)) < 0.01);
}
