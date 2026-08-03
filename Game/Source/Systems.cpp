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

    void RailsSpinSystem::update(sw::ecs::World& world, sw::f32 /*deltaSeconds*/)
    {
        // The split clock, for the reason CelestialSpinSystem gives.
        sw::f64 whole = 0.0;
        sw::f64 fraction = 0.0;
        m_timebase.presentSecondsSplit(whole, fraction);
        world.forEach<TransformComponent, PreviousTransformComponent, SpinComponent,
                      sw::phys::OnRailsComponent>(
            [whole, fraction](sw::ecs::Entity, TransformComponent& transform,
                  PreviousTransformComponent& previous, const SpinComponent& spin,
                  sw::phys::OnRailsComponent&) {
                // fmod before the cast: 0.586 rad/s reaches a million
                // radians in a fortnight, and an f32 angle that large has
                // lost the fraction of a turn that IS the answer.
                // A STOPPED RING KEEPS ITS ATTITUDE. Rate zero means "this
                // craft is not spinning", not "snap it back to where its
                // spin would have started" — which is what writing the
                // closed form unconditionally would do the moment a pilot
                // de-spun the ring and then left it.
                if (spin.radiansPerSecond == 0.0f)
                {
                    return;
                }
                const sw::f64 rate = static_cast<sw::f64>(spin.radiansPerSecond);
                const sw::f64 turn = 6.283185307179586 / std::abs(rate);
                const sw::f64 angle =
                    std::fmod(rate * (std::fmod(whole, turn) + fraction),
                              6.283185307179586);
                // THE AXIS IS WHERE THE CRAFT'S OWN +Z IS HELD in the world,
                // and the spin is about it. A closed form has no attitude to
                // build on — so the base is derived, by the shortest arc
                // that takes the model axis onto the world one, and the ring
                // can be parked in any plane without a component to store it.
                const sw::Vec3 worldAxis = glm::normalize(spin.axis);
                const sw::Vec3 modelAxis{0.0f, 0.0f, 1.0f};
                const sw::f32 alignment = glm::dot(modelAxis, worldAxis);
                sw::Quat base{1.0f, 0.0f, 0.0f, 0.0f};
                if (alignment < -0.9999f)
                {
                    base = glm::angleAxis(glm::pi<sw::f32>(),
                                          sw::Vec3{1.0f, 0.0f, 0.0f}); // antiparallel
                }
                else if (alignment < 0.9999f)
                {
                    const sw::Vec3 cross = glm::cross(modelAxis, worldAxis);
                    base = glm::normalize(
                        sw::Quat{1.0f + alignment, cross.x, cross.y, cross.z});
                }
                // Last tick's attitude, for the interpolator. RailsSystem
                // already copied it this tick; writing the same value again
                // costs nothing and keeps this system correct on its own.
                previous.rotation = transform.rotation;
                transform.rotation =
                    glm::angleAxis(static_cast<sw::f32>(angle), worldAxis) * base;
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
        // THE SPLIT CLOCK, for the same reason the positions use it: the
        // fmod below keeps the ANGLE small but it cannot recover precision the
        // multiply has already thrown away, and `rate * now` at now = 3e11 is
        // 2.2e7 radians with an ulp of 3.7e-9 — 2.4 cm on Terra's surface,
        // moving every tick. Reducing the TIME first instead of the angle
        // keeps the product small from the start.
        sw::f64 whole = 0.0;
        sw::f64 fraction = 0.0;
        m_timebase.presentSecondsSplit(whole, fraction);
        world.forEach<TransformComponent, PreviousTransformComponent, SpinComponent,
                      sw::phys::GravitySourceComponent>(
            [whole, fraction](sw::ecs::Entity, TransformComponent& transform,
                  PreviousTransformComponent& previous, const SpinComponent& spin,
                  sw::phys::GravitySourceComponent& source) {
                previous.rotation = transform.rotation;
                // The rotation PERIOD reduces the whole seconds exactly, and
                // the fraction rides on top with its full precision.
                const sw::f64 rate = static_cast<sw::f64>(spin.radiansPerSecond);
                const sw::f64 turn =
                    (rate != 0.0) ? 6.283185307179586 / std::abs(rate) : 0.0;
                const sw::f64 reduced =
                    (turn > 0.0) ? std::fmod(whole, turn) + fraction : whole + fraction;
                const sw::f64 angle = std::fmod(rate * reduced, 6.283185307179586);
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

    void SasSystem::update(sw::ecs::World& world, sw::f32 deltaSeconds)
    {
        // Snapshot gravity sources for SOI-primary velocity lookup.
        struct BodyInfo
        {
            sw::WorldVec3 center;
            sw::WorldVec3 velocity;
            /// The body's SPIN. Without it there is no surface frame, and
            /// without a surface frame the retrograde the autopilot flies
            /// and the retrograde the navball draws are different vectors.
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

                // ---- STABILITY: point nowhere, just stop turning ----------
                //
                // First, and before anything looks for a planet, because
                // holding still is the one thing that means the same in deep
                // space as it does in an atmosphere.
                //
                // It spends the RCS's own authority and not a unit more —
                // `angularAccel * dt` per tick, taken straight off the spin
                // vector. That bound is the whole honesty of the mode: it
                // settles a wobble in a second or two, and against a real
                // aerodynamic tumble it loses, visibly, which is exactly
                // what a set of attitude thrusters does. A mode that simply
                // wrote zero would make the atmosphere stop mattering.
                if (sas.mode == SasComponent::kStability)
                {
                    const sw::f32 rate = glm::length(body.angularVelocity);
                    if (rate < 1.0e-5f)
                    {
                        body.angularVelocity = sw::Vec3(0.0f);
                        return;
                    }
                    const sw::f32 authority =
                        std::max(ship.angularAccel, 0.0f) * deltaSeconds;
                    if (authority >= rate)
                    {
                        body.angularVelocity = sw::Vec3(0.0f);
                    }
                    else
                    {
                        body.angularVelocity -=
                            (body.angularVelocity / rate) * authority;
                    }
                    return;
                }

                if (bodies.empty())
                {
                    return;
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
                sw::Vec3 target{0.0f};
                if (sas.mode == SasComponent::kNode)
                {
                    // The burn vector, handed down by the flight plan. It
                    // shrinks as the burn is flown, and when it reaches zero
                    // there is nothing left to point at — hold, do not spin
                    // toward whatever a normalised zero would produce.
                    const sw::f32 length = glm::length(sas.targetDirection);
                    if (length < 1.0e-6f)
                    {
                        body.angularVelocity *= 0.85f;
                        return;
                    }
                    target = sas.targetDirection / length;
                }
                else
                {
                    // THE SAME FRAME THE NAVBALL IS DRAWING IN.
                    //
                    // Orbital prograde is measured against the primary's
                    // translation alone; SURFACE prograde also subtracts the
                    // ground's motion under you, which on Terra's equator is
                    // 465 m/s. Coming down, those two are not a refinement
                    // apart — they can be tens of degrees apart, and holding
                    // the orbital one while the marker on the navball shows
                    // the other is what made a landing a fight against the
                    // instrument instead of against gravity.
                    sw::WorldVec3 reference = primary->velocity;
                    if (sas.surfaceRelative != 0)
                    {
                        reference += glm::cross(primary->angularVelocity,
                                                transform.position - primary->center);
                    }
                    const sw::WorldVec3 relativeVelocity = body.velocity - reference;
                    const sw::f64 speed = glm::length(relativeVelocity);
                    if (speed < 1.0)
                    {
                        return; // no meaningful prograde
                    }
                    target = sw::Vec3(relativeVelocity / speed);
                    if (sas.mode == SasComponent::kRetrograde)
                    {
                        target = -target;
                    }
                    else if (sas.mode != SasComponent::kPrograde)
                    {
                        // ---- THE ORBIT'S OTHER FOUR DIRECTIONS ------------
                        //
                        // Built from the SAME velocity prograde is, which is
                        // the point: the frame toggle moves all six markers
                        // together or none of them, and an autopilot holding
                        // a normal computed in one frame while the navball
                        // draws it in the other is the landing bug of F6 all
                        // over again with a different vector.
                        //
                        // NORMAL is the angular momentum r x v — the axis the
                        // orbit turns about, so a burn along it rotates the
                        // plane and changes nothing else. RADIAL OUT is
                        // prograde x normal, which for a circular orbit is
                        // straight up and for an eccentric one is the in-plane
                        // direction perpendicular to the velocity, pointing
                        // away from the focus. That is the definition worth
                        // having: a radial burn moves the line of apsides
                        // without touching the period.
                        sw::Vec3 prograde{};
                        sw::Vec3 normal{};
                        sw::Vec3 radialOut{};
                        if (!sw::phys::orbitalFrame(transform.position - primary->center,
                                                    relativeVelocity, prograde, normal,
                                                    radialOut))
                        {
                            // Straight up or straight down: the orbit has no
                            // plane. Hold rather than point somewhere invented.
                            body.angularVelocity *= 0.85f;
                            return;
                        }
                        switch (sas.mode)
                        {
                        case SasComponent::kNormal: target = normal; break;
                        case SasComponent::kAntiNormal: target = -normal; break;
                        case SasComponent::kRadialOut: target = radialOut; break;
                        case SasComponent::kRadialIn: target = -radialOut; break;
                        default: break;
                        }
                    }
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
                    body.angularVelocity *= 0.85f; // aligned: settle
                    return;
                }
                const sw::Vec3 axisWorld = axis / sinAngle;
                const sw::f32 rate = std::min(angle * 1.2f, ship.maxAngularSpeed);
                // Body-frame command (ThrustSystem integrates body-frame).
                body.angularVelocity =
                    glm::inverse(transform.rotation) * (axisWorld * rate);
            });
    }

    void ThrustSystem::update(sw::ecs::World& world, sw::f32 deltaSeconds)
    {
        world.forEach<TransformComponent, sw::phys::DynamicBodyComponent, ShipComponent,
                      ShipControlsComponent>(
            [deltaSeconds, &world, this](sw::ecs::Entity entity,
                                         TransformComponent& transform,
                                         sw::phys::DynamicBodyComponent& body,
                                         ShipComponent& ship,
                                         const ShipControlsComponent& controls) {
                // ---- attitude (RCS): COMMAND the body-frame angular rate ----
                //
                // This system no longer turns the ship: it only says how
                // fast the ship should be turning. AngularIntegrationSystem,
                // which runs immediately after, is the single author of
                // rotation-from-angular-velocity for every dynamic body in
                // the world. See its comment for why that had to move.
                //
                // The spin has TWO authors: this system and the atmosphere.
                // So the rate limit has to tell them apart. It caps what the
                // PILOT may command — a ship cannot out-turn its own
                // thrusters — while leaving alone any rate the vehicle
                // ALREADY had, because the air is under no obligation to
                // respect an RCS rating. Clamping the total would have made
                // a rocket flipping in the airstream quietly stop flipping
                // the moment it passed 0.8 rad/s.
                const sw::f32 inherited = glm::length(body.angularVelocity);
                body.angularVelocity += controls.rotationInput * ship.angularAccel *
                                        deltaSeconds;
                if (controls.killRotation != 0)
                {
                    const sw::f32 decay =
                        std::max(0.0f, 1.0f - 3.0f * deltaSeconds);
                    body.angularVelocity *= decay;
                }
                const sw::f32 commanded = glm::length(body.angularVelocity);
                const sw::f32 limit = std::max(ship.maxAngularSpeed, inherited);
                if (commanded > limit)
                {
                    body.angularVelocity *= limit / commanded;
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
                        // Creative: full thrust, nothing leaves the tanks.
                        // The skip is HERE, not a fake refill afterwards —
                        // a refill would still jiggle the vessel's mass
                        // through the assembly pass every tick.
                        if (fuelNeeded > 0.0 && !m_infiniteFuel)
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

                    // ---- WHERE THE ENGINES PUSH, AND HOW HARD THEY TWIST --
                    //
                    // A force has a point of application. The old model put
                    // the whole of the vessel's thrust at its centre of mass
                    // along the hull's nose, which is exactly right for a
                    // symmetric rocket and a lie for everything else: shut
                    // three of the Endurance's four propulsion modules and it
                    // went on in a straight line, because the three that were
                    // left had been summed into a scalar that had forgotten
                    // where they were bolted.
                    //
                    // VesselAssemblySystem now sums them as VECTORS at their
                    // own positions, so the direction and the torque both fall
                    // out of the geometry. Nothing special-cases a symmetric
                    // craft: its arms cancel.
                    const sw::f64 command = static_cast<sw::f64>(controls.thrustAxis) *
                                            static_cast<sw::f64>(ship.throttle);
                    sw::Vec3 forward = transform.rotation * sw::math::kWorldForward;
                    const auto* vessel =
                        world.tryGetComponent<sw::parts::VesselComponent>(entity);
                    if (vessel != nullptr && vessel->partCount > 0 &&
                        glm::length(vessel->thrustForceN) > 1.0f)
                    {
                        // The engines' own summed direction, in the world.
                        forward = glm::normalize(transform.rotation *
                                                 vessel->thrustForceN);
                        // ...and the twist they leave behind. Body-frame:
                        // AngularIntegrationSystem integrates body-frame, and
                        // the torque was summed in the vessel's own frame, so
                        // no rotation belongs on this line at all.
                        //
                        // A THROTTLE THAT RUNS DRY MUST NOT KEEP TWISTING, so
                        // the torque is scaled by the same fraction the thrust
                        // was: `thrustNewtons` has already been cut by the
                        // fuel that actually burned.
                        const sw::f64 delivered =
                            (vessel->maxThrustNewtons > 1.0)
                                ? thrustNewtons / vessel->maxThrustNewtons
                                : 1.0;
                        const sw::Vec3 torque =
                            vessel->thrustTorqueNm *
                            static_cast<sw::f32>(command * delivered);
                        const sw::Vec3 inertia = glm::max(vessel->inertiaKgM2,
                                                          sw::Vec3{1.0f});
                        body.angularVelocity += (torque / inertia) * deltaSeconds;
                    }
                    const sw::f64 acceleration =
                        command * thrustNewtons / body.mass;
                    body.velocity += sw::WorldVec3(forward) * acceleration *
                                     static_cast<sw::f64>(deltaSeconds);
                }
            });
    }

    void AngularIntegrationSystem::update(sw::ecs::World& world, sw::f32 deltaSeconds)
    {
        world.forEach<TransformComponent, sw::phys::DynamicBodyComponent>(
            [deltaSeconds, &world](sw::ecs::Entity entity, TransformComponent& transform,
                                   sw::phys::DynamicBodyComponent& body) {
                // A WALKING SUIT DOES NOT TUMBLE.
                //
                // The capsule carries a hull, so the ground writes toppling
                // spin into it like it does for any other solid — but its
                // attitude is AUTHORED, not simulated: CapsuleMovementSystem
                // stands it upright and points it where the camera looks.
                // Integrating here and having that overwritten a few systems
                // later is work nobody reads, and the moment the two ever
                // ran in the other order the player would be lying on the
                // floor. Skipping is the honest statement of who owns it.
                if (world.tryGetComponent<CapsuleComponent>(entity) != nullptr)
                {
                    return;
                }

                const sw::f32 angularSpeed = glm::length(body.angularVelocity);
                if (angularSpeed <= 1.0e-6f)
                {
                    return;
                }
                const sw::Vec3 axis = body.angularVelocity / angularSpeed;
                const sw::Quat step = glm::angleAxis(angularSpeed * deltaSeconds, axis);
                // Body-frame rotation: post-multiply.
                const sw::Quat before = transform.rotation;
                transform.rotation = glm::normalize(transform.rotation * step);

                // A VEHICLE TURNS ABOUT ITS BALANCE POINT, not about
                // whichever part its builder happened to start from. The
                // transform's origin is the root part, so the position
                // is shifted by exactly as much as the rotation moved
                // the centre of mass — leaving that point where it was.
                // On a 20 m rocket, turning about the nose instead of
                // the middle swings the tail ten metres, and that is the
                // difference between a flip that looks like physics and
                // one that looks like a bug.
                if (const auto* vessel =
                        world.tryGetComponent<sw::parts::VesselComponent>(entity);
                    vessel != nullptr && vessel->partCount > 0)
                {
                    transform.position +=
                        sw::WorldVec3(before * vessel->centreOfMass) -
                        sw::WorldVec3(transform.rotation * vessel->centreOfMass);
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

                // JUMPING, before walking, and only with both feet on the
                // ground. It adds a RADIAL speed in the SURFACE frame and
                // touches nothing else: a jump taken at a run keeps the run,
                // and — the part a naive impulse gets wrong — keeps the
                // planet's own 465 m/s of spin underneath it.
                if (controls.jump != 0 && body.isGrounded != 0)
                {
                    const sw::WorldVec3 surfaceVelocity =
                        primary->velocity + glm::cross(primary->angularVelocity, radial);
                    body.velocity = sw::phys::surfaceJumpVelocity(
                        body.velocity, surfaceVelocity, sw::WorldVec3(up),
                        static_cast<sw::f64>(capsule.jumpSpeed));
                    body.isGrounded = 0;
                }

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
