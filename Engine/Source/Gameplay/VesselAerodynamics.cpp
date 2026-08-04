#include "Gameplay/VesselAerodynamics.hpp"

#include "Core/Log.hpp"
#include "ECS/World.hpp"
#include "Gameplay/PartGeometry.hpp"
#include "Scene/TransformComponents.hpp"

#include <glm/gtx/component_wise.hpp>

#include <algorithm>
#include <cmath>

namespace sw::aero
{
    namespace
    {
        std::vector<AeroTable>& registry()
        {
            static std::vector<AeroTable> tables;
            return tables;
        }

        /// The tumble a structure is assumed not to survive commanding, and
        /// therefore the rate no amount of aerodynamic moment may exceed.
        /// It exists to stop a numerical blow-up becoming a spinning ship,
        /// not to model anything.
        constexpr f32 kTumbleLimit = 8.0f; // rad/s
    } // namespace

    void setTables(std::vector<AeroTable> tables)
    {
        std::sort(tables.begin(), tables.end(),
                  [](const AeroTable& a, const AeroTable& b) { return a.partId < b.partId; });
        registry() = std::move(tables);
    }

    const AeroTable* findTable(u32 partId)
    {
        const std::vector<AeroTable>& tables = registry();
        const auto found = std::lower_bound(
            tables.begin(), tables.end(), partId,
            [](const AeroTable& table, u32 id) { return table.partId < id; });
        return (found != tables.end() && found->partId == partId) ? &*found : nullptr;
    }

    usize tableCount() { return registry().size(); }

    usize loadTables(const std::filesystem::path& directory)
    {
        setTables(loadAeroTables(directory));
        return registry().size();
    }

    // ------------------------------------------------------------------------
    void VesselAerodynamicsSystem::update(ecs::World& world, f32 deltaSeconds)
    {
        m_vesselCount = 0;
        const f64 dt = static_cast<f64>(deltaSeconds);
        if (dt <= 0.0 || registry().empty())
        {
            return;
        }

        // ---- who has air ----------------------------------------------------
        m_bodies.clear();
        world.forEach<TransformComponent, phys::GravitySourceComponent,
                      phys::AtmosphereComponent>(
            [this](ecs::Entity, TransformComponent& transform,
                   phys::GravitySourceComponent& source,
                   phys::AtmosphereComponent& atmosphere) {
                m_bodies.push_back(AirBody{transform.position, source.worldVelocity,
                                           source.angularVelocity, source.bodyRadius,
                                           atmosphere});
            });
        if (m_bodies.empty())
        {
            return;
        }

        world.forEach<TransformComponent, phys::DynamicBodyComponent,
                      parts::VesselComponent, AeroStateComponent>(
            [this, &world, dt](ecs::Entity entity, TransformComponent& transform,
                               phys::DynamicBodyComponent& body,
                               parts::VesselComponent& vessel, AeroStateComponent& state) {
                state = AeroStateComponent{};
                state.centreOfMass = vessel.centreOfMass;
                state.inertiaKgM2 = vessel.inertiaKgM2;
                if (vessel.partCount == 0)
                {
                    return;
                }

                // ---- which atmosphere, and how high ----------------------
                const AirBody* air = nullptr;
                f64 altitude = 0.0;
                WorldVec3 radial{0.0};
                for (const AirBody& candidate : m_bodies)
                {
                    const WorldVec3 offset = transform.position - candidate.centre;
                    const f64 height = glm::length(offset) - candidate.radius;
                    if (height < candidate.atmosphere.topAltitude &&
                        (air == nullptr || height < altitude))
                    {
                        air = &candidate;
                        altitude = height;
                        radial = offset;
                    }
                }
                if (air == nullptr || glm::length(radial) < 1.0)
                {
                    return; // vacuum: the state stays zeroed
                }
                const f64 densityKgM3 = density(air->atmosphere, altitude);
                if (densityKgM3 <= 0.0)
                {
                    return;
                }

                // ---- the air's own motion (THE CARRIER-VELOCITY RULE) ----
                // The atmosphere co-rotates with the planet. What matters is
                // the vessel's speed THROUGH it, never its world speed —
                // which on Terra's surface is already 30 km/s of orbit.
                const WorldVec3 up = glm::normalize(radial);
                const WorldVec3 surfaceVelocity =
                    air->velocity + glm::cross(air->angularVelocity, radial);
                WorldVec3 east = glm::cross(air->angularVelocity, radial);
                if (glm::length(east) < 1.0e-6)
                {
                    const WorldVec3 reference = (std::abs(up.y) < 0.9) ? WorldVec3{0, 1, 0}
                                                                       : WorldVec3{1, 0, 0};
                    east = glm::cross(reference, up);
                }
                east = glm::normalize(east);
                const WorldVec3 airVelocity =
                    surfaceVelocity +
                    windVelocity(air->atmosphere, altitude, up, east, m_timeSeconds);

                const WorldVec3 relative = body.velocity - airVelocity;
                const f64 speed = glm::length(relative);
                if (speed < 0.05)
                {
                    state.densityKgM3 = densityKgM3;
                    state.inAtmosphere = 1;
                    return; // becalmed: no force worth the rounding error
                }

                const f64 mach = speed / std::max(speedOfSound(air->atmosphere, altitude), 1.0);
                const f64 q = dynamicPressure(densityKgM3, speed) * machDragFactor(mach);

                // ---- the flow, in the vessel's own frame ------------------
                const Quat inverseRotation = glm::inverse(transform.rotation);
                const Vec3 flowVessel =
                    glm::normalize(inverseRotation * Vec3(-relative / speed));

                // ---- gather the parts and their boxes ---------------------
                m_parts.clear();
                m_boxes.clear();
                world.forEach<parts::PartComponent>(
                    [this, entity, &world](ecs::Entity partEntity,
                                           parts::PartComponent& part) {
                        if (part.vessel != entity)
                        {
                            return;
                        }
                        const parts::PartDefinition* definition =
                            parts::findDefinition(part.definitionId);
                        if (definition == nullptr)
                        {
                            return;
                        }
                        const u32 index = static_cast<u32>(m_parts.size());
                        PartEntry entry{findTable(part.definitionId), part.localPosition,
                                        part.localRotation};
                        entry.fairing =
                            world.tryGetComponent<parts::FairingComponent>(partEntity);
                        m_parts.push_back(entry);

                        auto cached = m_hullCache.find(part.definitionId);
                        if (cached == m_hullCache.end())
                        {
                            cached = m_hullCache
                                         .emplace(part.definitionId,
                                                  parts::effectiveHull(*definition))
                                         .first;
                        }
                        for (const parts::HitBox& box : cached->second)
                        {
                            m_boxes.push_back(OccluderBox{
                                part.localPosition + part.localRotation * box.center,
                                box.halfExtents, part.localRotation, index});
                        }
                    });
                if (m_parts.empty())
                {
                    return;
                }

                // ---- WHO IS INSIDE A SHELL ---------------------------------
                //
                // Done once, here, rather than per part per tick inside the
                // sum: the number of fairings on a craft is nought or one and
                // the number of parts is dozens, so the loop that costs
                // anything is the one over parts.
                //
                // A shielded part contributes NOTHING — not a reduced
                // exposure, nothing. `exposure()` deliberately never returns
                // zero, because a part in another part's wake still feels base
                // pressure and a stage that goes weightless in the airstream
                // reads as a bug; a part inside a sealed shroud is a different
                // fact, and the honest number for it is zero.
                for (usize p = 0; p < m_parts.size(); ++p)
                {
                    const PartEntry& shell = m_parts[p];
                    if (shell.fairing == nullptr || shell.fairing->closed == 0 ||
                        shell.fairing->jettisoned != 0)
                    {
                        continue;
                    }
                    const Quat inverseShell = glm::inverse(shell.localRotation);
                    for (usize q = 0; q < m_parts.size(); ++q)
                    {
                        if (q == p || m_parts[q].shielded)
                        {
                            continue;
                        }
                        const Vec3 inShellFrame =
                            inverseShell * (m_parts[q].localPosition - shell.localPosition);
                        m_parts[q].shielded =
                            parts::fairingEncloses(*shell.fairing, inShellFrame);
                    }
                }

                // ---- sum the parts ---------------------------------------
                //
                // Each part answers in ITS frame, about ITS origin. Rotating
                // the force into the vessel frame is one quaternion; moving
                // the moment onto the vessel's balance point is one cross
                // product. Everything after that is addition — which is the
                // whole reason the tables store forces and moments and not
                // accelerations.
                Vec3 force(0.0f);
                Vec3 moment(0.0f);
                Vec3 pressureWeighted(0.0f);
                f32 pressureWeight = 0.0f;
                f64 exposedArea = 0.0;

                // Scratch that lives on the system, not on the stack: two
                // heap allocations per vessel per tick is two too many at
                // fifty ticks a second.
                m_exposure.assign(m_parts.size(), 1.0f);
                std::vector<f32>& partExposure = m_exposure;
                {
                    // One box per part is enough to ask the question; a part
                    // with several is represented by its first, which is the
                    // one the hull builder put the bulk in.
                    m_firstBox.assign(m_parts.size(), static_cast<u32>(m_boxes.size()));
                    std::vector<u32>& firstBox = m_firstBox;
                    for (u32 b = 0; b < m_boxes.size(); ++b)
                    {
                        u32& slot = firstBox[m_boxes[b].ownerIndex];
                        if (slot == m_boxes.size())
                        {
                            slot = b;
                        }
                    }
                    for (usize p = 0; p < m_parts.size(); ++p)
                    {
                        if (firstBox[p] < m_boxes.size())
                        {
                            partExposure[p] = exposure(m_boxes, firstBox[p], flowVessel);
                        }
                    }
                }

                for (usize p = 0; p < m_parts.size(); ++p)
                {
                    const PartEntry& part = m_parts[p];
                    if (part.shielded)
                    {
                        continue; // inside a shroud: the air never reaches it
                    }
                    if (part.fairing != nullptr && part.fairing->closed != 0 &&
                        part.fairing->jettisoned == 0)
                    {
                        // THE SHELL ANSWERS FROM ITS OWN PANELS. Same three
                        // terms the offline forge integrates per pixel on every
                        // other part — impact, base pressure, skin friction —
                        // summed per flat quad instead, which a fairing is made
                        // of and a nose cone is not.
                        const Quat inverseFairing = glm::inverse(part.localRotation);
                        const Vec3 flowPart =
                            glm::normalize(inverseFairing * flowVessel);
                        Vec3 shellForce{0.0f};
                        Vec3 shellMoment{0.0f};
                        parts::fairingAero(*part.fairing, flowPart, shellForce,
                                           shellMoment);
                        const f32 scale = static_cast<f32>(q);
                        const Vec3 rotatedForce = (part.localRotation * shellForce) * scale;
                        const Vec3 rotatedMoment =
                            (part.localRotation * shellMoment) * scale;
                        const Vec3 lever = part.localPosition - vessel.centreOfMass;
                        force += rotatedForce;
                        moment += rotatedMoment + glm::cross(lever, rotatedForce);
                        exposedArea += parts::fairingAreaM2(*part.fairing) * 0.5;
                        const f32 shellWeight = glm::length(rotatedForce);
                        pressureWeighted += part.localPosition * shellWeight;
                        pressureWeight += shellWeight;
                        continue;
                    }
                    if (part.table == nullptr)
                    {
                        continue; // no table: this part is aerodynamically absent
                    }
                    const Quat inversePart = glm::inverse(part.localRotation);
                    const Vec3 flowPart = glm::normalize(inversePart * flowVessel);
                    const AeroSample solved = sample(*part.table, flowPart);

                    const f32 scale = static_cast<f32>(q) * partExposure[p];
                    const Vec3 partForce = (part.localRotation * solved.forceM2) * scale;
                    const Vec3 partMoment = (part.localRotation * solved.momentM3) * scale;
                    const Vec3 lever = part.localPosition - vessel.centreOfMass;

                    force += partForce;
                    moment += partMoment + glm::cross(lever, partForce);
                    exposedArea += static_cast<f64>(partExposure[p]) * part.table->maxAreaM2;

                    // THE CENTRE OF PRESSURE IS ABOUT THE SIDEWAYS FORCE,
                    // not the total one. Weighting by everything puts it on
                    // the nose of every vehicle ever built, because the nose
                    // is where the drag is — and a rocket that is measurably
                    // stable then reads as though it ought to tumble. What
                    // decides stability is where the force ACROSS the
                    // airstream acts, so that is what gets averaged.
                    const Vec3 crossForce =
                        partForce - flowVessel * glm::dot(partForce, flowVessel);
                    const f32 weight = glm::length(crossForce);
                    pressureWeighted += part.localPosition * weight;
                    pressureWeight += weight;
                }

                if (pressureWeight > 1.0e-4f)
                {
                    state.centreOfPressure = pressureWeighted / pressureWeight;
                }
                else
                {
                    state.centreOfPressure = vessel.centreOfMass;
                }

                // ---- damping ---------------------------------------------
                //
                // The restoring moment above says which way the vehicle
                // turns; this says how quickly it stops. The lever arm is
                // the distance between the balance point and the pressure
                // point — the same one that produced the restoring moment,
                // so a stable vehicle is damped and an unstable one is not.
                //
                // The floor is the vehicle's OWN size rather than a constant:
                // flying exactly into the wind the two points coincide, and
                // a lever arm of nothing would leave a long rocket with no
                // pitch damping at all at precisely the attitude it spends
                // most of its flight in.
                const f64 lever = std::max(
                    0.25 * static_cast<f64>(glm::compMax(vessel.halfExtents)),
                    static_cast<f64>(
                        glm::length(state.centreOfPressure - vessel.centreOfMass)));
                moment += dampingMoment(densityKgM3, speed,
                                        std::max(exposedArea, 0.1), lever,
                                        body.angularVelocity);

                // ---- apply -----------------------------------------------
                const f64 mass = std::max(body.mass, 1.0);
                const WorldVec3 worldForce = WorldVec3(transform.rotation * force);
                // A single step may never turn the vessel around inside its
                // own airstream: an atmosphere slows a body down, it does
                // not fling it upwind.
                WorldVec3 deltaV = worldForce * (dt / mass);
                const f64 along = glm::dot(deltaV, -relative / speed);
                if (along > speed)
                {
                    deltaV *= speed / along;
                }
                body.velocity += deltaV;

                const Vec3 angularAccel = moment / vessel.inertiaKgM2;
                body.angularVelocity += angularAccel * static_cast<f32>(dt);
                const f32 spin = glm::length(body.angularVelocity);
                if (spin > kTumbleLimit)
                {
                    body.angularVelocity *= kTumbleLimit / spin;
                }

                // ---- what the pilot gets to see --------------------------
                state.forceN = worldForce;
                state.momentNm = moment;
                state.angularAccelRadS2 = angularAccel;
                state.dynamicPressurePa = q;
                state.densityKgM3 = densityKgM3;
                state.machNumber = mach;
                state.airspeedMps = speed;
                state.inAtmosphere = 1;

                const Vec3 flowUnit = flowVessel;
                const f32 dragComponent = glm::dot(force, flowUnit);
                state.dragN = static_cast<f64>(dragComponent);
                state.liftN = static_cast<f64>(glm::length(force - flowUnit * dragComponent));

                // ANGLE OF ATTACK: between where the nose points (-Z) and
                // where the vessel is actually going. It is the number that
                // decides whether a reentry is a controlled one.
                const Vec3 nose(0.0f, 0.0f, -1.0f);
                const Vec3 travel = -flowVessel; // the way the vessel moves
                state.angleOfAttackRad =
                    std::acos(std::clamp(glm::dot(nose, travel), -1.0f, 1.0f));

                m_vesselCount += 1;
            });
    }
} // namespace sw::aero
