#include "Physics/PhysicsSystems.hpp"

#include "Physics/Aerodynamics.hpp"

#include <glm/gtx/component_wise.hpp>

#include "Core/Log.hpp"
#include "Physics/HullCollision.hpp"
#include "ECS/World.hpp"
#include "Simulation/Simulation.hpp"

#include <algorithm>
#include <cmath>

namespace sw::phys
{
    // ------------------------------------------------------------------------
    // GravityIntegrationSystem
    // ------------------------------------------------------------------------
    void GravityIntegrationSystem::update(ecs::World& world, f32 deltaSeconds)
    {
        // Collect gravity sources (a handful of celestial bodies).
        m_sources.clear();
        world.forEach<TransformComponent, GravitySourceComponent>(
            [this](ecs::Entity, TransformComponent& transform,
                   GravitySourceComponent& source) {
                m_sources.push_back({transform.position, source.mu});
            });

        const f64 dt = static_cast<f64>(deltaSeconds);

        // Semi-implicit (symplectic) Euler: v then p — stable for orbits.
        world.forEach<TransformComponent, DynamicBodyComponent>(
            [this, dt](ecs::Entity, TransformComponent& transform,
                       DynamicBodyComponent& body) {
                WorldVec3 acceleration{0.0};
                for (const Source& source : m_sources)
                {
                    const WorldVec3 toSource = source.position - transform.position;
                    const f64 distanceSq = glm::dot(toSource, toSource);
                    if (distanceSq > 1.0)
                    {
                        const f64 distance = std::sqrt(distanceSq);
                        acceleration += toSource * (source.mu / (distanceSq * distance));
                    }
                }
                body.gravityMps2 = acceleration;
                body.velocity += acceleration * dt;
                transform.position += body.velocity * dt;
            });
    }

    // ------------------------------------------------------------------------
    // SurfaceAnchorSystem
    // ------------------------------------------------------------------------
    void SurfaceAnchorSystem::update(ecs::World& world, f32 /*deltaSeconds*/)
    {
        world.forEach<TransformComponent, PreviousTransformComponent,
                      SurfaceAnchorComponent>(
            [&world](ecs::Entity, TransformComponent& transform,
                     PreviousTransformComponent& previous,
                     SurfaceAnchorComponent& anchor) {
                const auto* body =
                    world.tryGetComponent<TransformComponent>(anchor.body);
                if (body == nullptr)
                {
                    return; // body destroyed: anchor idles (structure floats)
                }

                // Rotate the body-fixed local position into world space.
                //
                // The rotation MUST come from the body's f64 spin state, not
                // from its f32 quaternion: |localPosition| is a planet radius
                // (6.371e6 m), and f32's seven digits quantise that lever arm
                // to about 1.2 m. Worse, the quantisation MOVES as the planet
                // turns — up to 0.77 m between consecutive frames — so every
                // building, pad and mast anchored to the ground shimmered
                // while the camera, an honest f64 world position, held still.
                //
                // Orientation keeps the f32 path: a 1e-7 rad error across a
                // 20 m structure is a nanometre.
                const auto* source =
                    world.tryGetComponent<GravitySourceComponent>(anchor.body);
                const glm::dquat bodyRotation =
                    (source != nullptr) ? spinRotation(*source)
                                        : glm::dquat(body->rotation);
                transform.position = body->position + bodyRotation * anchor.localPosition;
                transform.rotation =
                    Quat(bodyRotation * glm::dquat(anchor.localRotation));

                // Interpolate IN LOCKSTEP with the body: previous is derived
                // from the body's own previous pose. Mirroring current here
                // would make anchored structures jitter against terrain that
                // glides between physics ticks (the planet moves ~km/frame
                // on its orbit).
                if (const auto* bodyPrevious =
                        world.tryGetComponent<PreviousTransformComponent>(anchor.body))
                {
                    const glm::dquat previousRotation =
                        (source != nullptr)
                            ? spinRotation(*source, source->spinAnglePrevious)
                            : glm::dquat(bodyPrevious->rotation);
                    previous.position =
                        bodyPrevious->position + previousRotation * anchor.localPosition;
                    previous.rotation =
                        Quat(previousRotation * glm::dquat(anchor.localRotation));
                }
                else
                {
                    previous.position = transform.position;
                    previous.rotation = transform.rotation;
                }
            });
    }

    // ------------------------------------------------------------------------
    // SurfaceInteractionSystem
    // ------------------------------------------------------------------------
    void SurfaceInteractionSystem::update(ecs::World& world, f32 deltaSeconds)
    {
        const f64 dt = static_cast<f64>(deltaSeconds);

        m_surfaces.clear();
        world.forEach<TransformComponent, GravitySourceComponent>(
            [this, &world](ecs::Entity entity, TransformComponent& transform,
                           GravitySourceComponent& source) {
                if (source.bodyRadius <= 0.0)
                {
                    return;
                }
                Surface surface{};
                surface.center = transform.position;
                surface.velocity = source.worldVelocity;
                surface.angularVelocity = source.angularVelocity;
                surface.rotation = transform.rotation;
                // The terrain is sampled in the body's rotating frame, and a
                // f32 quaternion slides that sample point ~0.6 m across the
                // ground as the planet turns — the resting height then
                // wobbles with the local slope. Same f64 spin, same reason.
                surface.rotation64 = spinRotation(source);
                surface.radius = source.bodyRadius;
                surface.mu = source.mu;
                if (const auto* atmosphere =
                        world.tryGetComponent<AtmosphereComponent>(entity))
                {
                    surface.hasAtmosphere = true;
                    surface.atmosphere = *atmosphere;
                }
                if (const auto* terrain =
                        world.tryGetComponent<planet::TerrainComponent>(entity))
                {
                    surface.hasTerrain = true;
                    surface.terrain = *terrain;
                }
                m_surfaces.push_back(surface);
            });

        world.forEach<TransformComponent, DynamicBodyComponent>(
            [this, dt, &world](ecs::Entity entity, TransformComponent& transform,
                               DynamicBodyComponent& body) {
                body.isGrounded = 0;

                // How far below its ORIGIN this body reaches, at its current
                // attitude. Without a hull the answer is zero and the origin
                // rests on the ground — which is what buried everything up
                // to its waist before GroundHullComponent existed.
                const auto* hull = world.tryGetComponent<GroundHullComponent>(entity);

                // WHO OWNS THE DRAG. A part-built vessel carries an
                // AeroStateComponent, and the aerodynamics system has
                // already given it real forces from its parts' tables —
                // attitude-dependent, with moments. Applying the old
                // isotropic ballistic factor on top would charge it for the
                // same air twice. Everything else (the EVA suit, a crate, an
                // asteroid) still uses the simple model, which is the right
                // answer for a body with no parts to look up.
                const bool tabulatedAero =
                    world.tryGetComponent<aero::AeroStateComponent>(entity) != nullptr;

                for (const Surface& surface : m_surfaces)
                {
                    const WorldVec3 radial = transform.position - surface.center;
                    const f64 distance = glm::length(radial);
                    if (distance <= 1.0)
                    {
                        continue; // degenerate (the body itself)
                    }
                    const WorldVec3 up = radial / distance;

                    // The ground under this point: sea level + procedural
                    // terrain elevation, sampled in the body's ROTATING
                    // frame (mountains spin with their planet). Physics and
                    // rendering share the exact same heightfield function.
                    const f64 clearance =
                        (hull != nullptr)
                            ? groundClearance(*hull, transform.rotation, Vec3(up))
                            : 0.0;

                    f64 groundRadius = surface.radius;
                    if (surface.hasTerrain)
                    {
                        const glm::dquat toBodyFrame = glm::inverse(surface.rotation64);
                        const Vec3 centreDirection =
                            glm::normalize(Vec3(toBodyFrame * up));
                        groundRadius +=
                            planet::terrainElevation(surface.terrain, centreDirection);

                        // ON A SLOPE, the ground under the CENTRE is not the
                        // ground the object touches: the uphill side of the
                        // footprint hits first. Sampling only the centre
                        // buries that side — a 8 m-wide rocket on a 0.3
                        // slope sinks its uphill legs 1.2 m into the hill.
                        //
                        // So the footprint's horizontal extremes are sampled
                        // too and the HIGHEST wins. Max, not average: an
                        // object may rest above the ground, never inside it.
                        const f64 reach = footprintReach(hull, transform.rotation,
                                                         Vec3(up));
                        if (reach > 1.0)
                        {
                            const WorldVec3 reference =
                                (std::abs(up.y) < 0.9) ? WorldVec3{0.0, 1.0, 0.0}
                                                       : WorldVec3{1.0, 0.0, 0.0};
                            const WorldVec3 east =
                                glm::normalize(glm::cross(reference, up));
                            const WorldVec3 north = glm::cross(up, east);
                            const f64 step = reach / std::max(surface.radius, 1.0);
                            const WorldVec3 offsets[4] = {east * step, -east * step,
                                                          north * step, -north * step};
                            for (const WorldVec3& offset : offsets)
                            {
                                const Vec3 direction = glm::normalize(
                                    Vec3(toBodyFrame * glm::normalize(up + offset)));
                                groundRadius = std::max(
                                    groundRadius,
                                    surface.radius + planet::terrainElevation(
                                                         surface.terrain, direction));
                            }
                        }
                    }
                    // The radius the body's ORIGIN sits at when it is
                    // resting: the ground, plus its own hull.
                    const f64 restRadius = groundRadius + clearance;
                    const f64 altitude = distance - restRadius;

                    // Everything below is measured in the LOCAL SURFACE
                    // frame: translation of the body PLUS its spin at this
                    // point. The atmosphere co-rotates too (that is what
                    // makes a landed ship stay put on a spinning planet —
                    // and what will make eastward launches cheaper).
                    const WorldVec3 surfaceVelocity =
                        surface.velocity + glm::cross(surface.angularVelocity, radial);
                    WorldVec3 relativeVelocity = body.velocity - surfaceVelocity;

                    // ---- atmosphere: quadratic drag, exponential density --
                    // (pressure altitude is measured from SEA level).
                    const f64 seaAltitude = distance - surface.radius;
                    if (surface.hasAtmosphere && !tabulatedAero &&
                        seaAltitude < surface.atmosphere.topAltitude)
                    {
                        const f64 density =
                            surface.atmosphere.surfaceDensity *
                            std::exp(-std::max(seaAltitude, 0.0) /
                                     surface.atmosphere.scaleHeight);
                        const f64 speed = glm::length(relativeVelocity);
                        if (speed > 1.0e-6)
                        {
                            const f64 deceleration = 0.5 * density * speed * speed *
                                                     body.ballisticFactor;
                            // Never reverse the velocity within one step.
                            const f64 dv = std::min(deceleration * dt, speed);
                            relativeVelocity -= (relativeVelocity / speed) * dv;
                        }
                    }

                    // ---- solid ground (terrain-aware) ---------------------
                    if (altitude <= 0.0)
                    {
                        transform.position = surface.center + up * restRadius;

                        const f64 radialSpeed = glm::dot(relativeVelocity, up);
                        if (radialSpeed < 0.0)
                        {
                            if (-radialSpeed > m_config.crashSpeedThreshold)
                            {
                                SW_LOG_WARN("Physics",
                                            "Surface impact at {:.1f} m/s — that was a "
                                            "crash, not a landing",
                                            -radialSpeed);
                            }
                            else if (-radialSpeed > 0.5)
                            {
                                SW_LOG_INFO("Physics", "Touchdown at {:.1f} m/s",
                                            -radialSpeed);
                            }
                            relativeVelocity -= up * radialSpeed; // absorb impact
                        }

                        // Tangential friction until rest (in the body frame).
                        const f64 decay = std::max(
                            0.0, 1.0 - m_config.groundFrictionPerSecond * dt);
                        relativeVelocity *= decay;
                        body.isGrounded = 1;

                        // ---- and the SAME for the rotation -----------------
                        //
                        // Ground contact used to touch the velocity and stop
                        // there, so a vehicle that came down spinning kept
                        // spinning on the dirt for ever — the one thing that
                        // makes a landed craft read as a prop rather than an
                        // object. Two forces were missing, and they are the
                        // same two the linear case already had:
                        //
                        //   GRAVITY, which topples a body whose centre of
                        //   mass has passed outside the feet it is standing
                        //   on, and does exactly nothing while it has not;
                        //   FRICTION, which rubs the spin off against the
                        //   ground the way it already rubs off the slide.
                        if (hull != nullptr)
                        {
                            // The mass distribution, if anything knows it.
                            // The aerodynamics pass computes a real tensor
                            // from the parts and their fuel; anything else
                            // gets the honest box approximation of its own
                            // hull, which is what its hull is.
                            Vec3 centreOfMass = hull->centre;
                            Vec3 inertia =
                                aero::boxInertia(body.mass, hull->halfExtents);
                            if (const auto* air =
                                    world.tryGetComponent<aero::AeroStateComponent>(
                                        entity);
                                air != nullptr &&
                                glm::compMin(air->inertiaKgM2) > 1.0f)
                            {
                                centreOfMass = air->centreOfMass;
                                inertia = air->inertiaKgM2;
                            }

                            const f64 gravity =
                                surface.mu / std::max(distance * distance, 1.0);
                            body.angularVelocity +=
                                topplingAcceleration(*hull, transform.rotation,
                                                     Vec3(up), centreOfMass, inertia,
                                                     body.mass, gravity) *
                                static_cast<f32>(dt);

                            const f32 spinDecay = std::max(
                                0.0f, 1.0f - static_cast<f32>(
                                                 m_config.groundAngularFrictionPerSecond *
                                                 dt));
                            body.angularVelocity *= spinDecay;
                            if (glm::length(body.angularVelocity) < 1.0e-4f)
                            {
                                body.angularVelocity = Vec3(0.0f); // settled
                            }
                        }
                    }

                    body.velocity = surfaceVelocity + relativeVelocity;
                }
            });
    }

    // ------------------------------------------------------------------------
    // RailsSystem
    // ------------------------------------------------------------------------
    void RailsSystem::update(ecs::World& world, f32 /*deltaSeconds*/)
    {
        // AT FULL CLOCK PRECISION, like the celestials: a railed craft parked
        // beside a planet has to hold still RELATIVE to it, and two analytic
        // positions evaluated at differently-rounded times drift apart by
        // metres once the session has run for an interstellar crossing.
        f64 whole = 0.0;
        f64 fraction = 0.0;
        m_timebase.presentSecondsSplit(whole, fraction);

        world.forEach<TransformComponent, PreviousTransformComponent, OnRailsComponent>(
            [whole, fraction, &world](ecs::Entity, TransformComponent& transform,
                           PreviousTransformComponent& previous, OnRailsComponent& rails) {
                WorldVec3 relative{};
                if (!kepler::evaluateSplit(rails.orbit, whole, fraction, relative))
                {
                    // Not an orbit. Snapping the entity to the primary's
                    // centre — which is what a zeroed relative position does
                    // — is a worse answer than leaving the transform alone,
                    // and it is the answer that used to be given silently.
                    return;
                }

                // World position = primary's CURRENT position + the relative
                // conic. The primary was moved earlier this tick by the
                // CelestialMotionSystem at the same simulation time, so the
                // composition is exact.
                WorldVec3 primaryPosition{0.0};
                if (const auto* primaryTransform =
                        world.tryGetComponent<TransformComponent>(rails.primary))
                {
                    primaryPosition = primaryTransform->position;
                }

                previous.position = transform.position; // interpolate
                previous.rotation = transform.rotation;
                transform.position = primaryPosition + relative;
            });
    }

    // ------------------------------------------------------------------------
    // SimulationBubbleSystem
    // ------------------------------------------------------------------------
    SimulationBubbleSystem::SimulationBubbleSystem(ecs::EntityCommandBuffer& commands,
                                                   const sim::SimulationLane& timebase,
                                                   const Config& config)
        : m_commands(commands)
        , m_timebase(timebase)
        , m_config(config)
    {
        SW_ASSERT(m_config.exitRadius > m_config.enterRadius,
                  "Bubble hysteresis requires exitRadius ({}) > enterRadius ({})",
                  m_config.exitRadius, m_config.enterRadius);
    }

    const SimulationBubbleSystem::BodySnapshot*
    SimulationBubbleSystem::selectPrimary(const WorldVec3& position) const
    {
        const BodySnapshot* best = nullptr;
        for (const BodySnapshot& body : m_bodies)
        {
            const WorldVec3 delta = position - body.position;
            if (glm::dot(delta, delta) < body.soiRadius * body.soiRadius &&
                (best == nullptr || body.soiRadius < best->soiRadius))
            {
                best = &body;
            }
        }
        if (best == nullptr) // outside every SOI: fall back to the heaviest
        {
            for (const BodySnapshot& body : m_bodies)
            {
                if (best == nullptr || body.mu > best->mu)
                {
                    best = &body;
                }
            }
        }
        return best;
    }

    const SimulationBubbleSystem::BodySnapshot*
    SimulationBubbleSystem::findBody(ecs::Entity entity) const
    {
        for (const BodySnapshot& body : m_bodies)
        {
            if (body.entity == entity)
            {
                return &body;
            }
        }
        return nullptr;
    }

    void SimulationBubbleSystem::update(ecs::World& world, f32 /*deltaSeconds*/)
    {
        const f64 time = m_timebase.presentSeconds();
        const WorldVec3 focus = m_focus;

        // Snapshot every gravity source: positions for SOI selection,
        // world velocities for frame-relative conversions.
        m_bodies.clear();
        world.forEach<TransformComponent, GravitySourceComponent>(
            [this](ecs::Entity entity, TransformComponent& transform,
                   GravitySourceComponent& source) {
                m_bodies.push_back({entity, transform.position, source.worldVelocity,
                                    source.mu, source.soiRadius});
            });
        if (m_bodies.empty())
        {
            return; // no gravity source: nothing to convert against
        }

        const f64 enterSq = m_config.enterRadius * m_config.enterRadius;
        const f64 exitSq = m_config.exitRadius * m_config.exitRadius;
        const bool forceRails = m_forceRails;

        // ---- rails -> dynamic (entering the bubble) --------------------------
        world.forEach<TransformComponent, OnRailsComponent>(
            [&](ecs::Entity entity, TransformComponent& transform, OnRailsComponent& rails) {
                if (forceRails)
                {
                    return; // warp: nothing leaves the rails
                }
                const WorldVec3 delta = transform.position - focus;
                if (glm::dot(delta, delta) >= enterSq)
                {
                    return;
                }

                // Exact state vectors at conversion time, composed with the
                // primary's own state: the hand-off is continuous in both
                // position and velocity, in the WORLD frame.
                WorldVec3 relative{};
                WorldVec3 relativeVelocity{};
                if (!kepler::evaluate(rails.orbit, time, relative, &relativeVelocity))
                {
                    // The orbit is degenerate and has no velocity to give.
                    // Converting anyway handed the new DynamicBody a NaN,
                    // which the integrator then carried for ever and which
                    // showed up as a vessel that had silently left the world.
                    // Leaving the entity on rails costs one missed hand-off.
                    return;
                }

                WorldVec3 position = relative;
                WorldVec3 velocity = relativeVelocity;
                if (const BodySnapshot* primary = findBody(rails.primary))
                {
                    position += primary->position;
                    velocity += primary->velocity;
                }

                const f64 mass = rails.dynamicMass;
                const f64 ballistic = rails.dynamicBallisticFactor;
                m_commands.defer([entity, position, velocity, mass,
                                  ballistic](ecs::World& w) {
                    if (!w.isAlive(entity) ||
                        !w.hasComponent<OnRailsComponent>(entity))
                    {
                        return;
                    }
                    auto& t = w.getComponent<TransformComponent>(entity);
                    t.position = position;
                    if (auto* prev = w.tryGetComponent<PreviousTransformComponent>(entity))
                    {
                        prev->position = position;
                        prev->rotation = t.rotation;
                    }
                    w.removeComponent<OnRailsComponent>(entity);
                    DynamicBodyComponent dynamicBody{};
                    dynamicBody.velocity = velocity;
                    dynamicBody.mass = mass;
                    dynamicBody.ballisticFactor = ballistic;
                    w.addComponent(entity, dynamicBody);
                });
            });

        // ---- anchored -> dynamic (bubble focus returned, no warp) ---------------
        world.forEach<TransformComponent, SurfaceAnchorComponent>(
            [&](ecs::Entity entity, TransformComponent& transform,
                SurfaceAnchorComponent& anchor) {
                if (forceRails || anchor.autoRelease == 0)
                {
                    return; // hand-built structures stay anchored forever
                }
                const WorldVec3 delta = transform.position - focus;
                if (glm::dot(delta, delta) >= enterSq)
                {
                    return;
                }
                // Wake up co-rotating with the ground it stood on.
                WorldVec3 velocity{0.0};
                if (const auto* gravity =
                        world.tryGetComponent<GravitySourceComponent>(anchor.body))
                {
                    if (const auto* bodyTransform =
                            world.tryGetComponent<TransformComponent>(anchor.body))
                    {
                        velocity = gravity->worldVelocity +
                                   glm::cross(gravity->angularVelocity,
                                              transform.position -
                                                  bodyTransform->position);
                    }
                }
                SW_LOG_INFO("Physics", "Bubble: entity {} anchor RELEASED -> dynamic",
                            entity.index);
                const f64 mass = anchor.dynamicMass > 0.0 ? anchor.dynamicMass : 1.0e3;
                const f64 ballistic = anchor.dynamicBallisticFactor;
                m_commands.defer([entity, velocity, mass, ballistic](ecs::World& w) {
                    if (!w.isAlive(entity) ||
                        !w.hasComponent<SurfaceAnchorComponent>(entity))
                    {
                        return;
                    }
                    w.removeComponent<SurfaceAnchorComponent>(entity);
                    DynamicBodyComponent dynamicBody{};
                    dynamicBody.velocity = velocity;
                    dynamicBody.mass = mass;
                    dynamicBody.ballisticFactor = ballistic;
                    w.addComponent(entity, dynamicBody);
                });
            });

        // ---- dynamic -> rails (leaving the bubble) -----------------------------
        world.forEach<TransformComponent, DynamicBodyComponent>(
            [&](ecs::Entity entity, TransformComponent& transform,
                DynamicBodyComponent& body) {
                // Gravity sources themselves are never bubble-managed.
                if (world.hasComponent<GravitySourceComponent>(entity))
                {
                    return;
                }
                const WorldVec3 delta = transform.position - focus;
                // Warp railifies EVERYTHING; otherwise only bubble leavers.
                if (!forceRails && glm::dot(delta, delta) <= exitSq)
                {
                    return;
                }

                // Elements RELATIVE to the SOI primary. Hyperbolic arcs are
                // allowed on rails (evaluate() handles them analytically);
                // only near-parabolic states stay truly simulated.
                const BodySnapshot* primary = selectPrimary(transform.position);
                if (primary == nullptr)
                {
                    return;
                }

                // LANDED craft NEVER ride Kepler rails: a ground state is a
                // degenerate ellipse that would fling them skyward on the
                // first rail tick. They become SURFACE ANCHORS instead —
                // frozen in the body-fixed frame, co-rotating exactly, and
                // released back to dynamic when the bubble focus returns.
                const auto* primaryGravity =
                    world.tryGetComponent<GravitySourceComponent>(primary->entity);
                const auto* primaryTransform =
                    world.tryGetComponent<TransformComponent>(primary->entity);
                if (primaryGravity != nullptr && primaryTransform != nullptr &&
                    primaryGravity->bodyRadius > 0.0)
                {
                    const WorldVec3 relative = transform.position - primary->position;
                    const f64 altitude =
                        glm::length(relative) - primaryGravity->bodyRadius;
                    const WorldVec3 surfaceVelocity =
                        primary->velocity +
                        glm::cross(primaryGravity->angularVelocity, relative);
                    const f64 groundSpeed =
                        glm::length(body.velocity - surfaceVelocity);
                    if (altitude < 2.5e4 && groundSpeed < 5.0)
                    {
                        SW_LOG_INFO("Physics",
                                    "Bubble: entity {} LANDED (alt {:.0f} m, ground "
                                    "speed {:.2f}) -> surface anchor",
                                    entity.index, altitude, groundSpeed);
                        // f64 again: this INVERTS a planet-radius offset
                        // into the body frame, so an f32 rotation would bake
                        // a metre of error into where the base was built.
                        const glm::dquat inverseRotation =
                            glm::inverse(spinRotation(*primaryGravity));
                        SurfaceAnchorComponent anchor{};
                        anchor.body = primary->entity;
                        anchor.localPosition =
                            inverseRotation *
                            (transform.position - primaryTransform->position);
                        anchor.localRotation =
                            Quat(inverseRotation * glm::dquat(transform.rotation));
                        anchor.dynamicMass = body.mass;
                        anchor.dynamicBallisticFactor = body.ballisticFactor;
                        anchor.autoRelease = 1;
                        m_commands.defer([entity, anchor](ecs::World& w) {
                            if (!w.isAlive(entity) ||
                                !w.hasComponent<DynamicBodyComponent>(entity))
                            {
                                return;
                            }
                            w.removeComponent<DynamicBodyComponent>(entity);
                            w.addComponent(entity, anchor);
                        });
                        return;
                    }
                }
                KeplerOrbit orbit{};
                if (!kepler::fromStateVectors(primary->mu,
                                              transform.position - primary->position,
                                              body.velocity - primary->velocity, time,
                                              orbit, /*allowHyperbolic=*/true))
                {
                    return; // degenerate state: keep it truly simulated
                }

                OnRailsComponent rails{};
                rails.orbit = orbit;
                rails.primary = primary->entity;
                rails.dynamicMass = body.mass;
                rails.dynamicBallisticFactor = body.ballisticFactor;
                m_commands.defer([entity, rails](ecs::World& w) {
                    if (!w.isAlive(entity) ||
                        !w.hasComponent<DynamicBodyComponent>(entity))
                    {
                        return;
                    }
                    w.removeComponent<DynamicBodyComponent>(entity);
                    w.addComponent(entity, rails);
                });
            });
    }

    // ------------------------------------------------------------------------
    // HullCollisionSystem
    // ------------------------------------------------------------------------
    void HullCollisionSystem::update(ecs::World& world, f32 deltaSeconds)
    {
        (void)deltaSeconds;

        // ---- gather ------------------------------------------------------
        // One pass over every solid thing, keeping only what the broad phase
        // needs: where it is, how far it reaches, and which way it faces.
        struct Solid
        {
            ecs::Entity entity{};
            WorldVec3 position{0.0};
            Quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
            const HullComponent* hull = nullptr;
            f64 radius = 0.0;
        };
        std::vector<Solid> solids;
        std::vector<usize> movers;
        world.forEach<TransformComponent, HullComponent>(
            [&](ecs::Entity entity, TransformComponent& transform, HullComponent& hull) {
                if (hull.count == 0)
                {
                    return;
                }
                if (world.hasComponent<HullMoverComponent>(entity))
                {
                    movers.push_back(solids.size());
                }
                solids.push_back({entity, transform.position, transform.rotation, &hull,
                                  static_cast<f64>(hull.radius)});
            });
        m_hullCount = static_cast<u32>(solids.size());
        m_narrowPairs = 0;
        if (movers.empty())
        {
            return;
        }

        for (const usize moverIndex : movers)
        {
            Solid& mover = solids[moverIndex];
            const auto* limit =
                world.tryGetComponent<HullMoverComponent>(mover.entity);
            const f32 maxPush = (limit != nullptr) ? limit->maxPushM : 1.5f;

            // The mover's boxes, ONCE, in a frame centred on itself.
            Obb moverBoxes[kMaxHullBoxes]{};
            for (u32 i = 0; i < mover.hull->count; ++i)
            {
                moverBoxes[i] = makeObb(mover.rotation * mover.hull->boxes[i].centre,
                                        mover.hull->boxes[i].halfExtents, mover.rotation);
            }

            Vec3 push{0.0f};
            for (usize other = 0; other < solids.size(); ++other)
            {
                if (other == moverIndex)
                {
                    continue;
                }
                const Solid& blocker = solids[other];
                // BROAD PHASE. One subtraction, one comparison, and the
                // overwhelming majority of a base never gets any further.
                const WorldVec3 offset = blocker.position - mover.position;
                const f64 reach = mover.radius + blocker.radius;
                if (glm::dot(offset, offset) > reach * reach)
                {
                    continue;
                }
                m_narrowPairs += 1;

                // NARROW PHASE, in f32 metres around the mover — never in
                // world coordinates, where a planet radius would eat the
                // centimetres this is trying to resolve.
                const Vec3 relative = Vec3(offset);
                for (u32 b = 0; b < blocker.hull->count; ++b)
                {
                    const Obb blockerBox =
                        makeObb(relative + blocker.rotation * blocker.hull->boxes[b].centre,
                                blocker.hull->boxes[b].halfExtents, blocker.rotation);
                    for (u32 m = 0; m < mover.hull->count; ++m)
                    {
                        Vec3 axis{0.0f};
                        f32 depth = 0.0f;
                        if (!obbPenetration(moverBoxes[m], blockerBox, axis, depth))
                        {
                            continue;
                        }
                        // Accumulate the deepest push per direction rather
                        // than summing: standing in a corner, two walls each
                        // asking for 10 cm is one 10 cm step out, not 20.
                        const f32 along = glm::dot(push, axis);
                        if (depth > along)
                        {
                            push += axis * (depth - along);
                        }
                    }
                }
            }

            if (glm::dot(push, push) <= 1.0e-10f)
            {
                continue;
            }
            const f32 length = glm::length(push);
            if (length > maxPush)
            {
                push *= maxPush / length;
            }
            auto* transform = world.tryGetComponent<TransformComponent>(mover.entity);
            if (transform == nullptr)
            {
                continue;
            }
            transform->position += WorldVec3(push);

            // THE VELOCITY IS NOT TOUCHED, and that is deliberate.
            //
            // The obvious next line is "remove the component heading into
            // the wall". It launched the player a hundred metres, because
            // `velocity` is a WORLD velocity: standing on Terra it already
            // carries ~30 km/s of orbital motion plus 465 m/s of the
            // planet's rotation. Projecting that onto a wall normal and
            // subtracting it is not a small correction, it is a kilometres-
            // per-second impulse, and the wall fires the player off like a
            // catapult.
            //
            // What would be correct is the velocity RELATIVE to the blocker
            // — and the blocker's carrier velocity is a question about the
            // planet it is anchored to, which this system deliberately knows
            // nothing about. It does not need to: the walker SETS its
            // tangential velocity from input every tick in the local surface
            // frame (see CapsuleMovementSystem), so nothing accumulates and
            // pushing the position back out is the whole job. Walk into a
            // wall and you simply stop.
            //
            // The general rule this cost us, restated: on a planet, a world
            // velocity is mostly carrier motion. Anything that reasons about
            // how fast something is moving RELATIVE to the ground has to
            // subtract that carrier first, or it is doing arithmetic on
            // 30 km/s it did not mean to touch.
        }
    }
} // namespace sw::phys
