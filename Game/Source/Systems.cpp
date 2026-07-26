#include "Systems.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace game
{
    void SnapshotSystem::update(sw::ecs::World& world, sw::f32 /*deltaSeconds*/)
    {
        world.forEach<TransformComponent, PreviousTransformComponent,
                      sw::phys::DynamicBodyComponent>(
            [](sw::ecs::Entity, const TransformComponent& transform,
               PreviousTransformComponent& previous, sw::phys::DynamicBodyComponent&) {
                previous.position = transform.position;
                previous.rotation = transform.rotation;
            });
    }

    void SpinSystem::update(sw::ecs::World& world, sw::f32 deltaSeconds)
    {
        world.forEach<TransformComponent, SpinComponent, sw::phys::DynamicBodyComponent>(
            [deltaSeconds](sw::ecs::Entity, TransformComponent& transform,
                           const SpinComponent& spin, sw::phys::DynamicBodyComponent&) {
                const sw::Quat step =
                    glm::angleAxis(spin.radiansPerSecond * deltaSeconds, spin.axis);
                transform.rotation = glm::normalize(step * transform.rotation);
            });
    }

    void CelestialSpinSystem::update(sw::ecs::World& world, sw::f32 /*deltaSeconds*/)
    {
        // Positions (and previous.position) belong to the engine's
        // CelestialMotionSystem — this system only owns the rotation.
        //
        // ANALYTIC, like everything else on rails: angle = rate x time,
        // evaluated at the lane's present. Integrating it instead made the
        // day cycle depend on how many ticks the frame could afford, which
        // under warp is "not many" — the world spun around the star with a
        // planet that had barely turned, and a save reloaded with the
        // terrain in a different place than it went out.
        const sw::f64 now = m_timebase.presentSeconds();
        world.forEach<TransformComponent, PreviousTransformComponent, SpinComponent,
                      sw::phys::GravitySourceComponent>(
            [now](sw::ecs::Entity, TransformComponent& transform,
                  PreviousTransformComponent& previous, const SpinComponent& spin,
                  sw::phys::GravitySourceComponent& source) {
                previous.rotation = transform.rotation;
                // fmod keeps f32 precision intact after a year of spinning:
                // 7.29e-5 rad/s reaches 2,300 radians in a single orbit.
                const sw::f64 angle =
                    std::fmod(static_cast<sw::f64>(spin.radiansPerSecond) * now,
                              6.283185307179586);
                transform.rotation = glm::angleAxis(static_cast<sw::f32>(angle),
                                                    glm::normalize(spin.axis));
                // The SAME rotation, kept in f64. Anything sitting 6,371 km
                // from the axis is positioned from this one, not from the
                // quaternion above: f32 quantises that lever arm to about
                // 1.2 m and the error moves as the planet turns, which is
                // what made the whole surface shimmer.
                source.spinAxis = sw::WorldVec3(glm::normalize(spin.axis));
                source.spinAnglePrevious = source.spinAngle;
                source.spinAngle = angle;
            });
    }

    void CloudLayerSystem::update(sw::ecs::World& world, sw::f32 deltaSeconds)
    {
        world.forEach<TransformComponent, PreviousTransformComponent,
                      CloudLayerComponent>(
            [&world, deltaSeconds](sw::ecs::Entity, TransformComponent& transform,
                                   PreviousTransformComponent& previous,
                                   CloudLayerComponent& layer) {
                const auto* body =
                    world.tryGetComponent<TransformComponent>(layer.body);
                if (body == nullptr)
                {
                    return;
                }
                // Follow the body's center IN LOCKSTEP (previous from the
                // body's previous — same rule as surface anchors).
                if (const auto* bodyPrevious =
                        world.tryGetComponent<PreviousTransformComponent>(layer.body))
                {
                    previous.position = bodyPrevious->position;
                }
                else
                {
                    previous.position = transform.position;
                }
                previous.rotation = transform.rotation;
                transform.position = body->position;

                // A shell with no spin of its own is GLUED to the body's
                // rotation (M28: the cloud deck). Its weather then lives in
                // ONE place — the shader's analytic coverage, advected by the
                // world clock — which is what lets a ground fragment sample
                // the very cloud above it and know that it stands in shade.
                if (layer.radiansPerSecond == 0.0f)
                {
                    transform.rotation = body->rotation;
                    return;
                }
                const sw::Quat step = glm::angleAxis(
                    layer.radiansPerSecond * deltaSeconds, layer.spinAxis);
                transform.rotation = glm::normalize(step * transform.rotation);
            });
    }

    void SolarChargeSystem::update(sw::ecs::World& world, sw::f32 deltaSeconds)
    {
        std::vector<sw::ecs::Entity> chargedVessels;
        world.forEach<sw::parts::PartComponent, sw::factory::InventoryComponent>(
            [&](sw::ecs::Entity, sw::parts::PartComponent& part,
                sw::factory::InventoryComponent& inventory) {
                const auto* definition = sw::parts::findDefinition(part.definitionId);
                if (definition == nullptr ||
                    definition->type != sw::parts::PartType::Battery)
                {
                    return;
                }
                for (const sw::ecs::Entity done : chargedVessels)
                {
                    if (done == part.vessel)
                    {
                        return; // one battery per vessel takes the charge
                    }
                }
                const auto* vessel =
                    world.tryGetComponent<sw::parts::VesselComponent>(part.vessel);
                if (vessel == nullptr || vessel->solarChargeRateKw <= 0.0)
                {
                    return;
                }
                sw::factory::inventoryAdd(inventory, sw::res::Resource::ElectricCharge,
                                          vessel->solarChargeRateKw *
                                              static_cast<sw::f64>(deltaSeconds));
                chargedVessels.push_back(part.vessel);
            });
    }

    void SasSystem::update(sw::ecs::World& world, sw::f32 /*deltaSeconds*/)
    {
        // Snapshot gravity sources for SOI-primary velocity lookup.
        struct BodyInfo
        {
            sw::WorldVec3 center;
            sw::WorldVec3 velocity;
            sw::f64 soiRadius;
        };
        std::vector<BodyInfo> bodies;
        world.forEach<TransformComponent, sw::phys::GravitySourceComponent>(
            [&](sw::ecs::Entity, TransformComponent& transform,
                sw::phys::GravitySourceComponent& source) {
                bodies.push_back(
                    {transform.position, source.worldVelocity, source.soiRadius});
            });
        if (bodies.empty())
        {
            return;
        }

        world.forEach<TransformComponent, sw::phys::DynamicBodyComponent, ShipComponent,
                      ShipControlsComponent, SasComponent>(
            [&](sw::ecs::Entity, TransformComponent& transform,
                sw::phys::DynamicBodyComponent& body, ShipComponent& ship,
                const ShipControlsComponent& controls, const SasComponent& sas) {
                if (sas.mode == SasComponent::kOff ||
                    glm::dot(controls.rotationInput, controls.rotationInput) > 1.0e-6f ||
                    controls.killRotation != 0)
                {
                    return; // off, or the pilot is flying by hand
                }

                const BodyInfo* primary = nullptr;
                for (const BodyInfo& candidate : bodies)
                {
                    const sw::WorldVec3 d = transform.position - candidate.center;
                    if (glm::dot(d, d) < candidate.soiRadius * candidate.soiRadius &&
                        (primary == nullptr || candidate.soiRadius < primary->soiRadius))
                    {
                        primary = &candidate;
                    }
                }
                if (primary == nullptr)
                {
                    return;
                }
                const sw::WorldVec3 relativeVelocity = body.velocity - primary->velocity;
                const sw::f64 speed = glm::length(relativeVelocity);
                if (speed < 1.0)
                {
                    return; // no meaningful prograde
                }
                sw::Vec3 target = sw::Vec3(relativeVelocity / speed);
                if (sas.mode == SasComponent::kRetrograde)
                {
                    target = -target;
                }

                // Proportional attitude controller: command an angular
                // velocity along the shortest-arc axis, rate-limited. The
                // ThrustSystem integrates it (and clamps to the ship's
                // maximum turn rate).
                const sw::Vec3 forward = transform.rotation * sw::math::kWorldForward;
                const sw::Vec3 axis = glm::cross(forward, target);
                const sw::f32 sinAngle = glm::length(axis);
                const sw::f32 cosAngle = glm::dot(forward, target);
                const sw::f32 angle = std::atan2(sinAngle, cosAngle);
                if (sinAngle < 1.0e-5f || angle < 2.0e-3f)
                {
                    ship.angularVelocity *= 0.85f; // aligned: settle
                    return;
                }
                const sw::Vec3 axisWorld = axis / sinAngle;
                const sw::f32 rate = std::min(angle * 1.2f, ship.maxAngularSpeed);
                // Body-frame command (ThrustSystem integrates body-frame).
                ship.angularVelocity =
                    glm::inverse(transform.rotation) * (axisWorld * rate);
            });
    }

    void ThrustSystem::update(sw::ecs::World& world, sw::f32 deltaSeconds)
    {
        world.forEach<TransformComponent, sw::phys::DynamicBodyComponent, ShipComponent,
                      ShipControlsComponent>(
            [deltaSeconds, &world](sw::ecs::Entity entity, TransformComponent& transform,
                                   sw::phys::DynamicBodyComponent& body,
                                   ShipComponent& ship,
                                   const ShipControlsComponent& controls) {
                // ---- attitude (RCS): integrate body-frame angular velocity --
                ship.angularVelocity += controls.rotationInput * ship.angularAccel *
                                        deltaSeconds;
                if (controls.killRotation != 0)
                {
                    const sw::f32 decay =
                        std::max(0.0f, 1.0f - 3.0f * deltaSeconds);
                    ship.angularVelocity *= decay;
                }
                const sw::f32 angularSpeed = glm::length(ship.angularVelocity);
                if (angularSpeed > ship.maxAngularSpeed)
                {
                    ship.angularVelocity *= ship.maxAngularSpeed / angularSpeed;
                }
                if (angularSpeed > 1.0e-6f)
                {
                    const sw::Vec3 axis = ship.angularVelocity / angularSpeed;
                    const sw::Quat step =
                        glm::angleAxis(angularSpeed * deltaSeconds, axis);
                    // Body-frame rotation: post-multiply.
                    transform.rotation = glm::normalize(transform.rotation * step);
                }

                // ---- throttle limiter (Shift/Ctrl held) ---------------------
                ship.throttle = std::clamp(
                    ship.throttle + controls.throttleDelta * 0.5f * deltaSeconds, 0.0f,
                    1.0f);

                // ---- main engine: F = ma along the ship's forward (-Z) ------
                if (controls.thrustAxis != 0.0f && ship.throttle > 0.0f)
                {
                    sw::f64 thrustNewtons = ship.mainThrustNewtons;

                    // Part-built vessel: thrust comes from its ENGINES and
                    // burns real FUEL out of its tanks — run dry and the
                    // engine dies. Mass loss happens by itself: the
                    // assembly pass re-weighs the vessel from its cargo.
                    if (const auto* vessel =
                            world.tryGetComponent<sw::parts::VesselComponent>(entity);
                        vessel != nullptr && vessel->partCount > 0)
                    {
                        thrustNewtons = vessel->maxThrustNewtons;
                        const sw::f64 throttleFactor =
                            static_cast<sw::f64>(ship.throttle) *
                            std::abs(static_cast<sw::f64>(controls.thrustAxis));
                        const sw::f64 fuelNeeded = vessel->maxMassFlowKgps *
                                                   throttleFactor *
                                                   static_cast<sw::f64>(deltaSeconds);
                        sw::f64 fuelBurned = 0.0;
                        if (fuelNeeded > 0.0)
                        {
                            world.forEach<sw::parts::PartComponent,
                                          sw::factory::InventoryComponent>(
                                [&](sw::ecs::Entity, sw::parts::PartComponent& part,
                                    sw::factory::InventoryComponent& inventory) {
                                    if (part.vessel != entity ||
                                        fuelBurned >= fuelNeeded)
                                    {
                                        return;
                                    }
                                    fuelBurned += sw::factory::inventoryRemove(
                                        inventory, sw::res::Resource::Fuel,
                                        fuelNeeded - fuelBurned);
                                });
                            // Partial tank: thrust scales with what burned.
                            thrustNewtons *= fuelBurned / fuelNeeded;
                        }
                    }

                    const sw::Vec3 forward = transform.rotation * sw::math::kWorldForward;
                    const sw::f64 acceleration =
                        static_cast<sw::f64>(controls.thrustAxis) *
                        static_cast<sw::f64>(ship.throttle) * thrustNewtons / body.mass;
                    body.velocity += sw::WorldVec3(forward) * acceleration *
                                     static_cast<sw::f64>(deltaSeconds);
                }
            });
    }

    void CapsuleMovementSystem::update(sw::ecs::World& world, sw::f32 deltaSeconds)
    {
        // Snapshot the gravity sources; each capsule picks its SOI primary
        // (deepest containing sphere of influence) for "up" and for the
        // surface-relative walking frame.
        struct BodyInfo
        {
            sw::WorldVec3 center;
            sw::WorldVec3 velocity;
            sw::WorldVec3 angularVelocity;
            sw::f64 soiRadius;
        };
        std::vector<BodyInfo> bodies;
        world.forEach<TransformComponent, sw::phys::GravitySourceComponent>(
            [&](sw::ecs::Entity, TransformComponent& transform,
                sw::phys::GravitySourceComponent& source) {
                bodies.push_back({transform.position, source.worldVelocity,
                                  source.angularVelocity, source.soiRadius});
            });
        if (bodies.empty())
        {
            return;
        }

        world.forEach<TransformComponent, sw::phys::DynamicBodyComponent, CapsuleComponent,
                      ShipControlsComponent>(
            [&](sw::ecs::Entity, TransformComponent& transform,
                sw::phys::DynamicBodyComponent& body, CapsuleComponent& capsule,
                const ShipControlsComponent& controls) {
                const BodyInfo* primary = nullptr;
                for (const BodyInfo& candidate : bodies)
                {
                    const sw::WorldVec3 d = transform.position - candidate.center;
                    if (glm::dot(d, d) < candidate.soiRadius * candidate.soiRadius &&
                        (primary == nullptr || candidate.soiRadius < primary->soiRadius))
                    {
                        primary = &candidate;
                    }
                }
                if (primary == nullptr)
                {
                    return; // deep space: no meaningful "up"
                }

                const sw::WorldVec3 radial = transform.position - primary->center;
                const sw::f64 distance = glm::length(radial);
                if (distance <= 1.0)
                {
                    return;
                }
                const sw::Vec3 up = sw::Vec3(radial / distance);

                // The heading is NOT integrated here any more: the camera
                // owns it (StarWorksGame::updateChaseCamera writes it from
                // the mouse look), so the suit always faces where the player
                // is looking. A/D became a SIDESTEP instead of a turn — you
                // can now circle a building while still watching it.

                // Local tangent frame from up + heading.
                sw::Vec3 reference =
                    (std::abs(up.y) < 0.99f) ? sw::Vec3{0, 1, 0} : sw::Vec3{1, 0, 0};
                const sw::Vec3 east = glm::normalize(glm::cross(reference, up));
                const sw::Vec3 north = glm::cross(up, east);
                const sw::Vec3 heading =
                    north * std::cos(capsule.headingRadians) +
                    east * std::sin(capsule.headingRadians);

                // Keep the capsule upright, facing its heading.
                const sw::Vec3 right = glm::normalize(glm::cross(heading, up));
                transform.rotation =
                    glm::normalize(glm::quat_cast(sw::Mat3{right, up, -heading}));

                // Walking: grounded only. The tangential velocity is SET in
                // the LOCAL SURFACE frame (body translation + spin at this
                // point): the planet races around its star AND rotates, and
                // walking must never strip that carrier velocity.
                if (body.isGrounded != 0 &&
                    (controls.thrustAxis != 0.0f || controls.strafeAxis != 0.0f))
                {
                    // Forward + sideways, clamped so walking diagonally is
                    // not 41% faster than walking straight.
                    sw::Vec3 step = heading * controls.thrustAxis +
                                    right * controls.strafeAxis;
                    const sw::f32 magnitude = glm::length(step);
                    if (magnitude > 1.0f)
                    {
                        step /= magnitude;
                    }
                    const sw::Vec3 walk = step * capsule.walkSpeed;
                    const sw::WorldVec3 surfaceVelocity =
                        primary->velocity +
                        glm::cross(primary->angularVelocity, radial);
                    const sw::WorldVec3 relative = body.velocity - surfaceVelocity;
                    const sw::f64 radialSpeed = glm::dot(relative, sw::WorldVec3(up));
                    body.velocity = surfaceVelocity + sw::WorldVec3(up) * radialSpeed +
                                    sw::WorldVec3(walk);
                }
            });
    }

    void StatsSystem::update(sw::ecs::World& world, sw::f32 /*deltaSeconds*/)
    {
        // Every 30 ticks of the 1 Hz World lane ≈ every 30 simulated seconds.
        if ((m_ticks++ % 30) != 0)
        {
            return;
        }

        sw::f64 totalMined = 0.0;
        sw::f64 totalRefined = 0.0;
        sw::f64 storedOre = 0.0;
        sw::f64 storedIron = 0.0;
        world.forEach<sw::factory::MinerComponent>(
            [&](sw::ecs::Entity, sw::factory::MinerComponent& miner) {
                totalMined += miner.totalMined;
            });
        world.forEach<sw::factory::RefineryComponent>(
            [&](sw::ecs::Entity, sw::factory::RefineryComponent& refinery) {
                totalRefined += refinery.totalRefined;
            });
        world.forEach<sw::factory::InventoryComponent>(
            [&](sw::ecs::Entity, sw::factory::InventoryComponent& inventory) {
                storedOre += sw::factory::inventoryCount(inventory,
                                                         sw::res::Resource::IronOre);
                storedIron +=
                    sw::factory::inventoryCount(inventory, sw::res::Resource::Iron);
            });

        SW_LOG_INFO("Game",
                    "[World lane] tick {}: {} entities ({} dynamic, {} rails) | factory: "
                    "mined {:.1f} u, refined {:.1f} u, stored ore {:.1f} u / iron {:.1f} u",
                    m_ticks - 1, world.aliveCount(),
                    world.count<sw::phys::DynamicBodyComponent>(),
                    world.count<sw::phys::OnRailsComponent>(), totalMined, totalRefined,
                    storedOre, storedIron);
    }
} // namespace game
