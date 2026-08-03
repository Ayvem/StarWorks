// ============================================================================
// GameFlight.cpp — Flight: controls, warp, autopilot support, maneuver nodes, reentry, camera.
// Split out of StarWorksGame.cpp; same class, one theme per translation unit.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"
#include "Systems.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <format>
#include <limits>

namespace game
{

    sw::f32 StarWorksGame::heatingFactorFor(sw::ecs::Entity entity) const
    {
        auto& world = const_cast<sw::ecs::World&>(m_world);
        const auto* body = world.tryGetComponent<sw::phys::DynamicBodyComponent>(entity);
        const auto* transform = world.tryGetComponent<TransformComponent>(entity);
        if (body == nullptr || transform == nullptr || m_celestialIndex.size() == 0)
        {
            return 0.0f;
        }
        const sw::i32 primaryIndex = m_celestialIndex.soiPrimaryAt(
            transform->position, m_physicsLane->presentSeconds());
        if (primaryIndex < 0)
        {
            return 0.0f;
        }
        const auto& primary =
            m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
        const auto* atmosphere =
            world.tryGetComponent<sw::phys::AtmosphereComponent>(primary.entity);
        const auto* source =
            world.tryGetComponent<sw::phys::GravitySourceComponent>(primary.entity);
        const auto* primaryTransform =
            world.tryGetComponent<TransformComponent>(primary.entity);
        if (atmosphere == nullptr || source == nullptr || primaryTransform == nullptr)
        {
            return 0.0f;
        }

        const sw::WorldVec3 radial = transform->position - primaryTransform->position;
        const sw::f64 altitude = glm::length(radial) - primary.bodyRadius;
        if (altitude > atmosphere->topAltitude)
        {
            return 0.0f;
        }
        const sw::f64 density =
            atmosphere->surfaceDensity *
            std::exp(-std::max(altitude, 0.0) / atmosphere->scaleHeight);
        // Air moves with the planet: translation + spin at this point.
        const sw::WorldVec3 airVelocity =
            source->worldVelocity + glm::cross(source->angularVelocity, radial);
        const sw::f64 speed = glm::length(body->velocity - airVelocity);
        const sw::f64 q = density * speed * speed * speed;
        if (q <= kHeatGlowStart)
        {
            return 0.0f;
        }
        return std::clamp(
            static_cast<sw::f32>(std::log10(q / kHeatGlowStart)) / kHeatLogRange, 0.0f,
            1.0f);
    }

    void StarWorksGame::updateReentryEffects(sw::f32 deltaSeconds)
    {
        m_shipHeat = heatingFactorFor(m_shipEntity);
        m_capsuleHeat =
            m_capsuleEntity.isNull() ? 0.0f : heatingFactorFor(m_capsuleEntity);

        // ---- age & move existing particles (visual, render-frame rate) ------
        for (sw::usize i = 0; i < m_particles.size();)
        {
            ReentryParticle& particle = m_particles[i];
            particle.life -= deltaSeconds;
            if (particle.life <= 0.0f)
            {
                m_particles[i] = m_particles.back();
                m_particles.pop_back();
                continue;
            }
            particle.position += particle.velocity * static_cast<sw::f64>(deltaSeconds);
            ++i;
        }

        // ---- spawn plasma behind hot craft -----------------------------------
        auto spawnFor = [this, deltaSeconds](sw::ecs::Entity entity, sw::f32 heat) {
            if (heat < 0.05f)
            {
                return;
            }
            const auto* transform = m_world.tryGetComponent<TransformComponent>(entity);
            const auto* body =
                m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(entity);
            if (transform == nullptr || body == nullptr)
            {
                return;
            }

            // Spawn from the INTERPOLATED pose — the craft is RENDERED
            // there. Spawning at the raw physics-tick position offset the
            // whole cloud from the ship by up to a tick of motion (~600 m
            // of planetary travel): the "same offset on every particle" bug.
            sw::WorldVec3 spawnOrigin = transform->position;
            if (const auto* previous =
                    m_world.tryGetComponent<PreviousTransformComponent>(entity))
            {
                spawnOrigin =
                    glm::mix(previous->position, transform->position,
                             static_cast<sw::f64>(m_physicsLane->alpha()));
            }

            // The wake trails opposite the motion THROUGH THE AIR. The raw
            // world velocity is dominated by the planet's own 30 km/s
            // orbital motion and would point the trail the wrong way.
            sw::WorldVec3 airVelocity{0.0};
            const sw::i32 primaryIndex = m_celestialIndex.soiPrimaryAt(
                transform->position, m_physicsLane->presentSeconds());
            if (primaryIndex >= 0)
            {
                const auto& primary =
                    m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
                const auto* source =
                    m_world.tryGetComponent<sw::phys::GravitySourceComponent>(
                        primary.entity);
                const auto* primaryTransform =
                    m_world.tryGetComponent<TransformComponent>(primary.entity);
                if (source != nullptr && primaryTransform != nullptr)
                {
                    airVelocity =
                        source->worldVelocity +
                        glm::cross(source->angularVelocity,
                                   transform->position - primaryTransform->position);
                }
            }
            const sw::WorldVec3 relativeVelocity = body->velocity - airVelocity;
            const sw::f64 speed = glm::length(relativeVelocity);
            if (speed < 1.0)
            {
                return;
            }
            const sw::WorldVec3 backward = -relativeVelocity / speed;

            m_particleSpawnDebt += (40.0f + 280.0f * heat) * deltaSeconds;
            auto random01 = [this] { return hash01(m_particleSeed++); };
            while (m_particleSpawnDebt >= 1.0f && m_particles.size() < kMaxParticles)
            {
                m_particleSpawnDebt -= 1.0f;
                ReentryParticle particle{};
                const sw::WorldVec3 jitter{random01() - 0.5f, random01() - 0.5f,
                                           random01() - 0.5f};
                // The plasma is shed along the wake, BEHIND the craft: a
                // glowing tail rather than a cloud around the camera.
                particle.position = spawnOrigin +
                                    backward * (12.0 + 160.0 * random01()) +
                                    jitter * 7.0;
                particle.velocity = body->velocity +
                                    backward * (60.0 + 260.0 * random01()) +
                                    jitter * 30.0;
                particle.maxLife = 0.35f + 0.75f * random01();
                particle.life = particle.maxLife;
                particle.size = (0.20f + 0.45f * random01()) * (0.5f + heat);
                // Long incandescent STREAK along the airflow.
                particle.streakDirection = sw::Vec3(backward);
                particle.stretch = 5.0f + 6.0f * heat;
                particle.kind = 0;
                m_particles.push_back(particle);
            }
        };
        spawnFor(m_shipEntity, m_shipHeat);
        if (!m_capsuleEntity.isNull())
        {
            spawnFor(m_capsuleEntity, m_capsuleHeat);
        }

        // ---- engine exhaust jet ----------------------------------------------
        const auto* ship = m_world.tryGetComponent<ShipComponent>(m_shipEntity);
        const auto* controls =
            m_world.tryGetComponent<ShipControlsComponent>(m_shipEntity);
        const auto* shipBody =
            m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(m_shipEntity);
        const auto* shipTransform =
            m_world.tryGetComponent<TransformComponent>(m_shipEntity);
        if (ship != nullptr && controls != nullptr && shipBody != nullptr &&
            shipTransform != nullptr && controls->thrustAxis != 0.0f &&
            ship->throttle > 0.01f)
        {
            const sw::f32 power = ship->throttle * std::abs(controls->thrustAxis);
            m_particleSpawnDebt += 220.0f * power * deltaSeconds;
            auto random01 = [this] { return hash01(m_particleSeed++); };
            // The plume leaves through the nozzle, OPPOSITE the thrust:
            // forward burn -> jet out the back (+Z body), retro -> the front.
            const sw::f32 jetSign = (controls->thrustAxis > 0.0f) ? 1.0f : -1.0f;
            const sw::Vec3 jetDir =
                shipTransform->rotation * sw::Vec3{0.0f, 0.0f, jetSign};
            sw::WorldVec3 exhaustOrigin = shipTransform->position;
            if (const auto* previous =
                    m_world.tryGetComponent<PreviousTransformComponent>(m_shipEntity))
            {
                exhaustOrigin =
                    glm::mix(previous->position, shipTransform->position,
                             static_cast<sw::f64>(m_physicsLane->alpha()));
            }
            const sw::WorldVec3 nozzle = exhaustOrigin + sw::WorldVec3(jetDir) * 9.5;
            while (m_particleSpawnDebt >= 1.0f && m_particles.size() < kMaxParticles)
            {
                m_particleSpawnDebt -= 1.0f;
                ReentryParticle particle{};
                const sw::WorldVec3 jitter{random01() - 0.5f, random01() - 0.5f,
                                           random01() - 0.5f};
                particle.position = nozzle + jitter * 1.2;
                particle.velocity = shipBody->velocity +
                                    sw::WorldVec3(jetDir) * (70.0 + 90.0 * random01()) +
                                    jitter * 7.0;
                particle.maxLife = 0.22f + 0.30f * random01();
                particle.life = particle.maxLife;
                particle.size = 0.28f + 0.35f * random01();
                particle.streakDirection = jetDir;
                particle.stretch = 3.0f;
                particle.kind = 1;
                m_particles.push_back(particle);
            }
        }
    }

    void StarWorksGame::collectParticles(const sw::Camera& activeCamera)
    {
        const sw::WorldVec3 cameraPosition = activeCamera.position();
        for (const ReentryParticle& particle : m_particles)
        {
            const sw::f32 lifeFraction = particle.life / particle.maxLife; // 1 -> 0
            const sw::f32 heatColor = lifeFraction * lifeFraction;

            sw::Vec4 tint{};
            if (particle.kind == 0) // plasma: white-hot -> deep red
            {
                tint = {1.0f, 0.25f + 0.65f * heatColor, 0.05f + 0.45f * heatColor,
                        1.0f};
            }
            else // exhaust: blue-white flame -> faint amber
            {
                tint = {0.55f + 0.40f * heatColor, 0.60f + 0.35f * heatColor,
                        0.75f + 0.25f * heatColor, 1.0f};
            }
            const sw::f32 size =
                particle.size * (1.0f + 1.6f * (1.0f - lifeFraction));

            // Soft round BILLBOARD (radial alpha falloff), stretched along
            // the streak direction projected onto the view plane — plasma
            // reads as glowing gas, exhaust as a flame, no hard box edges.
            const sw::Vec3 relative = sw::Vec3(particle.position - cameraPosition);
            const sw::f32 depth = glm::length(relative);
            const sw::Vec3 toCam = depth > 1.0e-4f ? -relative / depth
                                                   : sw::Vec3{0.0f, 0.0f, 1.0f};
            sw::Vec3 stretchAxis =
                particle.streakDirection -
                toCam * glm::dot(particle.streakDirection, toCam);
            const sw::f32 stretchLength = glm::length(stretchAxis);
            if (stretchLength > 1.0e-4f)
            {
                stretchAxis /= stretchLength;
            }
            else
            {
                const sw::Vec3 reference =
                    std::abs(toCam.y) < 0.99f ? sw::Vec3{0, 1, 0} : sw::Vec3{1, 0, 0};
                stretchAxis = glm::normalize(glm::cross(reference, toCam));
            }
            const sw::Vec3 minorAxis = glm::cross(toCam, stretchAxis);
            const sw::Mat4 basis{sw::Vec4(stretchAxis, 0.0f), sw::Vec4(minorAxis, 0.0f),
                                 sw::Vec4(toCam, 0.0f), sw::Vec4(relative, 1.0f)};

            sw::DrawItem item{};
            item.mesh = &m_meshes[m_particleGlowMeshIndex];
            item.transform =
                basis * glm::scale(sw::Mat4{1.0f},
                                   sw::Vec3{size * particle.stretch, size, 1.0f});
            item.boundsCenter = relative;
            item.boundsRadius = size * particle.stretch;
            item.tint = tint;
            m_drawItems.push_back(item);
        }
    }

    sw::i32 StarWorksGame::controlledPrimaryIndex() const
    {
        if (m_celestialIndex.size() == 0)
        {
            return -1;
        }
        const auto& transform = const_cast<sw::ecs::World&>(m_world)
                                    .getComponent<TransformComponent>(controlledEntity());
        return m_celestialIndex.soiPrimaryAt(transform.position,
                                             m_physicsLane->presentSeconds());
    }

    void StarWorksGame::updateManeuverNodeInput()
    {
        // Map view only: no conflict with flight keys (Shift/Ctrl throttle).
        if (input().wasKeyPressed(sw::KeyCode::N))
        {
            m_nodeActive = !m_nodeActive;
            if (m_nodeActive)
            {
                m_nodeTime = m_physicsLane->presentSeconds() + 120.0;
                m_nodePrograde = 0.0;
                m_nodeNormal = 0.0;
                m_nodeRadial = 0.0;
            }
            m_lastPredictionSeconds = -1.0e9; // recompute now
            SW_LOG_INFO("Game", "Maneuver node {}", m_nodeActive ? "created" : "deleted");
        }
        if (!m_nodeActive)
        {
            return;
        }

        // Both sides of the keyboard: a player holding right shift is asking
        // for the same thing as one holding left shift.
        const bool shift = input().isKeyDown(sw::KeyCode::LeftShift) ||
                           input().isKeyDown(sw::KeyCode::RightShift);
        const bool control = input().isKeyDown(sw::KeyCode::LeftControl) ||
                             input().isKeyDown(sw::KeyCode::RightControl);
        const bool alt = input().isKeyDown(sw::KeyCode::LeftAlt) ||
                         input().isKeyDown(sw::KeyCode::RightAlt);
        const sw::space::ManeuverStep step = sw::space::maneuverStep(shift, control, alt);

        bool edited = false;
        auto adjust = [&](sw::KeyCode plus, sw::KeyCode minus, sw::f64& value,
                          sw::f64 amount) {
            if (input().wasKeyPressed(plus)) { value += amount; edited = true; }
            if (input().wasKeyPressed(minus)) { value -= amount; edited = true; }
        };
        adjust(sw::KeyCode::L, sw::KeyCode::J, m_nodeTime, step.seconds);
        adjust(sw::KeyCode::I, sw::KeyCode::K, m_nodePrograde, step.deltaVMps);
        adjust(sw::KeyCode::O, sw::KeyCode::U, m_nodeNormal, step.deltaVMps);
        adjust(sw::KeyCode::Y, sw::KeyCode::H, m_nodeRadial, step.deltaVMps);
        m_nodeTime = std::max(m_nodeTime, m_physicsLane->presentSeconds() + 5.0);
        if (edited)
        {
            m_lastPredictionSeconds = -1.0e9; // instant visual feedback
        }
    }

    // ------------------------------------------------------------------------
    // PICKING A TARGET OFF THE MAP
    //
    // Click a body and it is the target; click it again and it is not. A
    // click on empty space changes nothing — losing a target because you
    // clicked to look at something else would be a small betrayal every
    // time it happened.
    //
    // The hit test is against the body's CENTRE on screen, not its drawn
    // disc: at map zoom most bodies are a few pixels across, and asking the
    // player to hit a four-pixel sphere is asking them to hunt.
    // ------------------------------------------------------------------------
    void StarWorksGame::updateTargetPick()
    {
        if (!m_mapView || m_editorMode || m_nodeDragging ||
            !input().wasMouseButtonPressed(sw::MouseButton::Left))
        {
            return;
        }
        sw::f32 cursorX = 0.0f;
        sw::f32 cursorY = 0.0f;
        if (!hudCursor(cursorX, cursorY))
        {
            return;
        }
        for (const HudButton& button : m_hudButtons)
        {
            if (cursorX >= button.x0 && cursorX <= button.x1 && cursorY >= button.y0 &&
                cursorY <= button.y1)
            {
                return; // a panel has first claim on the click
            }
        }

        const sw::Vec2 cursor{cursorX, cursorY};
        const sw::f64 time = m_physicsLane->presentSeconds();
        const sw::Mat4 viewProjection = m_mapCamera.viewProjectionCameraRelative();
        const sw::WorldVec3 cameraPosition = m_mapCamera.position();
        constexpr sw::f32 kPickRadiusNdc = 0.06f;

        sw::i32 picked = -1;
        sw::f32 pickedDistance = kPickRadiusNdc;
        for (sw::usize i = 0; i < m_celestialIndex.size(); ++i)
        {
            const sw::WorldVec3 world =
                m_celestialIndex.positionAt(static_cast<sw::i32>(i), time);
            const sw::ui::MarkerPlacement placement = sw::ui::placeScreenMarker(
                viewProjection, sw::Vec3(world - cameraPosition), m_mapCamera.right(),
                m_mapCamera.up(), m_mapCamera.forward());
            if (placement.offScreen)
            {
                continue; // clamped to the border: that is a pointer, not a body
            }
            const sw::f32 distance = glm::length(placement.ndc - cursor);
            if (distance < pickedDistance)
            {
                pickedDistance = distance;
                picked = static_cast<sw::i32>(i);
            }
        }
        if (picked < 0)
        {
            return; // empty space: keep whatever was targeted
        }

        m_targetIndex = (picked == m_targetIndex) ? -1 : picked;
        m_approach = {};
        m_nodeApproach = {};
        m_lastPredictionSeconds = -1.0e9; // answer the new question now
        if (m_targetIndex >= 0)
        {
            SW_LOG_INFO("Game", "TARGET: {}",
                        m_celestialIndex.body(static_cast<sw::usize>(m_targetIndex)).name);
        }
        else
        {
            SW_LOG_INFO("Game", "TARGET cleared");
        }
    }

    // ------------------------------------------------------------------------
    // DRAGGING THE NODE ALONG ITS ORBIT
    //
    // The keys move the node in fixed steps; the mouse moves it where you
    // point. Grab the violet marker and the node's TIME follows the pixel
    // under the cursor — the burn slides round the orbit and the planned
    // trajectory redraws under your hand, which is the only way to answer
    // "where on this orbit should I burn?" by looking rather than counting.
    //
    // It runs AFTER the camera, next to the ground cursor, for the reason
    // that rule exists: a pick is a ray from THIS frame's eye, and doing it
    // in the key handler aims it one frame behind the map you can see.
    // ------------------------------------------------------------------------
    void StarWorksGame::updateNodeDrag()
    {
        if (!m_mapView || !m_nodeActive || m_editorMode)
        {
            m_nodeDragging = false;
            return;
        }
        sw::f32 cursorX = 0.0f;
        sw::f32 cursorY = 0.0f;
        if (!hudCursor(cursorX, cursorY))
        {
            m_nodeDragging = false;
            return;
        }
        const sw::Vec2 cursor{cursorX, cursorY};

        if (!input().isMouseButtonDown(sw::MouseButton::Left))
        {
            m_nodeDragging = false;
        }

        const sw::f64 time = m_physicsLane->presentSeconds();
        const sw::Mat4 viewProjection = m_mapCamera.viewProjectionCameraRelative();
        const sw::WorldVec3 cameraPosition = m_mapCamera.position();

        // ---- grabbing it ----------------------------------------------------
        if (!m_nodeDragging && input().wasMouseButtonPressed(sw::MouseButton::Left) &&
            m_nodePrimaryIndex >= 0)
        {
            // Not if a HUD button is under the cursor: the panel gets first
            // claim on a click, or the map's own buttons would be unusable
            // whenever a node happened to be near them.
            bool overButton = false;
            for (const HudButton& button : m_hudButtons)
            {
                overButton = overButton ||
                             (cursorX >= button.x0 && cursorX <= button.x1 &&
                              cursorY >= button.y0 && cursorY <= button.y1);
            }
            const sw::WorldVec3 nodeWorld =
                m_celestialIndex.positionAt(m_nodePrimaryIndex, time) +
                m_nodeRelativePosition;
            const sw::ui::MarkerPlacement placement = sw::ui::placeScreenMarker(
                viewProjection, sw::Vec3(nodeWorld - cameraPosition),
                m_mapCamera.right(), m_mapCamera.up(), m_mapCamera.forward());
            // The marker is about 0.05 NDC across; twice that is a target a
            // hand can hit without hunting for it.
            constexpr sw::f32 kGrabRadiusNdc = 0.05f;
            if (!overButton && !placement.offScreen &&
                glm::length(placement.ndc - cursor) < kGrabRadiusNdc)
            {
                m_nodeDragging = true;
            }
        }

        // ---- and moving it --------------------------------------------------
        if (!m_nodeDragging)
        {
            return;
        }
        sw::f64 pickedTime = 0.0;
        sw::f32 pickedDistance = 0.0f;
        if (!sw::space::timeNearestScreenPoint(m_celestialIndex, m_prediction,
                                               viewProjection, cameraPosition, time,
                                               cursor, kPredictionDisplaySamples,
                                               pickedTime, pickedDistance))
        {
            return;
        }
        // The node is stuck to its LINE, not to the cursor. Drag the mouse
        // off into empty space and the nearest sample is still somewhere on
        // the plan — possibly half an orbit away — so a pick that lands
        // nowhere near the pointer is ignored rather than obeyed.
        constexpr sw::f32 kMaxPickNdc = 0.30f;
        if (pickedDistance > kMaxPickNdc)
        {
            return;
        }
        // The same floor the keys respect: a burn cannot be scheduled in the
        // past, and one five seconds out is already unflyable.
        const sw::f64 clamped = std::max(pickedTime, time + 5.0);
        if (std::abs(clamped - m_nodeTime) > 1.0e-6)
        {
            m_nodeTime = clamped;
            m_lastPredictionSeconds = -1.0e9; // redraw the plan under the hand
        }
    }

    void StarWorksGame::refreshPrediction()
    {
        // The flight plan is a pure function of current state — recomputing
        // a few times per second is plenty (and each run costs ~ms).
        const sw::f64 now = clock().totalSeconds();
        if (now - m_lastPredictionSeconds < kPredictionRefreshSeconds)
        {
            return;
        }
        m_lastPredictionSeconds = now;

        const sw::ecs::Entity entity = controlledEntity();
        const auto& transform = m_world.getComponent<TransformComponent>(entity);

        // Resting on a surface: no meaningful ballistic trajectory — and
        // the conic of a landed craft "impacts" permanently, which is
        // noise, not information.
        if (const auto* body =
                m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(entity);
            body != nullptr && body->isGrounded != 0)
        {
            m_prediction.clear();
            m_nodePrediction.clear();
            m_approach = {};
            m_nodeApproach = {};
            return;
        }

        const sw::f64 startTime = m_physicsLane->presentSeconds();
        // THE PRE-BURN PLAN IS DRAWN WHOLE, node or no node.
        //
        // It used to stop AT the node, on the reasoning that the post-burn
        // path takes over there. But the node is DRAGGED along that line
        // now, and a line that ends at the thing you are dragging gives you
        // nowhere to drag it to. Drawing both — the current orbit entire,
        // the planned one branching off the marker — is also what KSP does,
        // and for the same reason.
        const sw::space::PredictionSettings settings{};
        sw::space::predictTrajectory(m_celestialIndex, transform.position,
                                     controlledVelocity(), startTime, settings,
                                     m_prediction);

        // ---- how close does this plan pass the target? ---------------------
        m_approach = {};
        m_nodeApproach = {};
        if (m_targetIndex >= 0 &&
            static_cast<sw::usize>(m_targetIndex) < m_celestialIndex.size())
        {
            m_approach = sw::space::closestApproachToBody(m_celestialIndex, m_prediction,
                                                          m_targetIndex);
        }

        // ---- the planned burn: dv applied in the orbital frame at the node ----
        m_nodePrediction.clear();
        m_nodePrimaryIndex = -1;
        if (!m_nodeActive)
        {
            return;
        }
        sw::WorldVec3 nodePosition{};
        sw::WorldVec3 nodeVelocity{};
        if (!sw::space::stateOnPrediction(m_celestialIndex, m_prediction, m_nodeTime,
                                          nodePosition, nodeVelocity))
        {
            return;
        }
        const sw::i32 nodePrimary =
            m_celestialIndex.soiPrimaryAt(nodePosition, m_nodeTime);
        if (nodePrimary < 0)
        {
            return;
        }
        sw::WorldVec3 primaryPosition{};
        sw::WorldVec3 primaryVelocity{};
        m_celestialIndex.stateAt(nodePrimary, m_nodeTime, primaryPosition,
                                 &primaryVelocity);
        const sw::WorldVec3 relativePosition = nodePosition - primaryPosition;
        const sw::WorldVec3 relativeVelocity = nodeVelocity - primaryVelocity;
        const sw::f64 relativeSpeed = glm::length(relativeVelocity);
        if (relativeSpeed < 1.0e-6)
        {
            return;
        }

        // KSP orbital frame at the node: prograde along the motion, normal
        // along the orbit's angular momentum, radial completing (outward).
        const sw::WorldVec3 prograde = relativeVelocity / relativeSpeed;
        sw::WorldVec3 normal = glm::cross(relativePosition, relativeVelocity);
        const sw::f64 normalLength = glm::length(normal);
        if (normalLength < 1.0e-9)
        {
            return; // radial trajectory: no orbital frame
        }
        normal /= normalLength;
        const sw::WorldVec3 radialOut = glm::cross(prograde, normal);

        const sw::WorldVec3 dv = prograde * m_nodePrograde + normal * m_nodeNormal +
                                 radialOut * m_nodeRadial;

        // ---- THE LOCK ------------------------------------------------------
        // Close to the node, freeze the plan: the coasting trajectory as it
        // was BEFORE the burn, and the dv vector in world terms. From here
        // the burn is flown against a target that does not move with the
        // ship, which is the only way the remaining dv can reach zero.
        // Editing the node re-takes the lock; drifting out of the window
        // drops it.
        const bool inWindow = (m_nodeTime - startTime) <= kBurnLockSeconds;
        const bool lockMatches = m_burnLocked && m_burnNodeTime == m_nodeTime &&
                                 m_burnPrograde == m_nodePrograde &&
                                 m_burnNormal == m_nodeNormal &&
                                 m_burnRadial == m_nodeRadial;
        if (inWindow && !lockMatches)
        {
            m_burnLocked = true;
            m_burnCoast = m_prediction; // the path NOT taken, from here on
            m_burnDvWorld = dv;
            m_burnNodeTime = m_nodeTime;
            m_burnPrograde = m_nodePrograde;
            m_burnNormal = m_nodeNormal;
            m_burnRadial = m_nodeRadial;
        }
        else if (!inWindow)
        {
            m_burnLocked = false;
        }

        // What is LEFT of the burn is what the planned trajectory should be
        // drawn from: as the player flies it, the plan converges onto the
        // orbit they are actually achieving instead of drifting away from it.
        const sw::WorldVec3 remaining =
            m_burnLocked ? remainingBurnVector() : dv;
        m_nodePostBurnVelocity = nodeVelocity + remaining;
        m_nodePrimaryIndex = nodePrimary;
        m_nodeRelativePosition = relativePosition;

        sw::space::PredictionSettings nodeSettings{}; // full horizon again
        sw::space::predictTrajectory(m_celestialIndex, nodePosition,
                                     m_nodePostBurnVelocity, m_nodeTime, nodeSettings,
                                     m_nodePrediction);
        if (m_targetIndex >= 0)
        {
            // What the PLANNED burn would achieve, which is the number the
            // player is actually dialling the node for.
            m_nodeApproach = sw::space::closestApproachToBody(
                m_celestialIndex, m_nodePrediction, m_targetIndex);
        }
    }

    // THE BURN STILL TO FLY.
    //
    // Locked: the planned dv minus what has actually been applied, where
    // "applied" is measured against the COASTING velocity from the frozen
    // pre-burn plan. That subtraction is what makes the number honest —
    // gravity changes the ship's velocity by a kilometre per second over a
    // two-minute burn in low orbit, and counting that as thrust would have
    // the readout hit zero while the engine still had work to do.
    //
    // Unlocked (the node is minutes away): there is nothing to count down
    // yet, so it is simply the planned burn.
    sw::WorldVec3 StarWorksGame::remainingBurnVector()
    {
        if (!m_nodeActive)
        {
            return sw::WorldVec3{0.0};
        }
        if (m_burnLocked)
        {
            return sw::space::remainingBurn(m_celestialIndex, m_burnCoast, m_burnDvWorld,
                                            controlledVelocity(),
                                            m_physicsLane->presentSeconds());
        }
        return m_nodePostBurnVelocity - controlledVelocity();
    }

    void StarWorksGame::toggleEva()
    {
        // ON FOOT IS HOME. The suit is created with the world and never
        // destroyed; this only decides which of the two things the player's
        // hands are on. Boarding therefore needs a vessel to board, and
        // there may not be one — the world starts with none.
        if (m_capsuleEntity.isNull())
        {
            return;
        }
        if (m_evaMode)
        {
            if (m_shipEntity.isNull() || !m_world.isAlive(m_shipEntity))
            {
                SW_LOG_INFO("Game", "No vessel to board — order one at the VAB");
                return;
            }
            m_evaMode = false;
            SW_LOG_INFO("Game", "Aboard: controlling vessel {}", m_shipEntity.index);
            return;
        }

        // Stepping out: put the suit beside the vessel, CO-MOVING with it.
        // Anything else and the ship leaves at orbital speed the instant the
        // player's feet touch vacuum.
        if (!m_shipEntity.isNull() && m_world.isAlive(m_shipEntity))
        {
            const auto& shipTransform =
                m_world.getComponent<TransformComponent>(m_shipEntity);
            const sw::WorldVec3 shipVelocity = controlledVelocity();
            auto& transform = m_world.getComponent<TransformComponent>(m_capsuleEntity);
            transform.position =
                shipTransform.position +
                sw::WorldVec3(shipTransform.rotation * sw::Vec3{12.0f, 0.0f, 0.0f});
            transform.rotation = shipTransform.rotation;
            if (auto* previous =
                    m_world.tryGetComponent<PreviousTransformComponent>(m_capsuleEntity))
            {
                previous->position = transform.position;
                previous->rotation = transform.rotation;
            }
            if (auto* body =
                    m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(m_capsuleEntity))
            {
                body->velocity = shipVelocity;
                body->angularVelocity = sw::Vec3{0.0f};
            }
        }
        m_evaMode = true;
        SW_LOG_INFO("Game", "EVA: on foot");
    }

    // STAGING: fire the ship's next decoupler, nose-most last.
    void StarWorksGame::fireNextDecoupler()
    {
        sw::ecs::Entity decoupler{};
        m_world.forEach<sw::parts::PartComponent>(
            [&](sw::ecs::Entity entity, sw::parts::PartComponent& part) {
                if (part.vessel != m_shipEntity || !decoupler.isNull())
                {
                    return;
                }
                const auto* definition = sw::parts::findDefinition(part.definitionId);
                if (definition != nullptr &&
                    definition->type == sw::parts::PartType::Decoupler)
                {
                    decoupler = entity;
                }
            });
        if (decoupler.isNull())
        {
            return;
        }
        const sw::ecs::Entity separated = sw::parts::decoupleAt(m_world, decoupler);
        if (!separated.isNull())
        {
            // splitVessel already gave it Transform/Previous/body.
            m_world.addComponent(separated, BoundsComponent{0.1f});
            m_world.addComponent(separated,
                                 MapMarkerComponent{{0.6f, 0.6f, 0.6f, 1.0f}});
            SW_LOG_INFO("Game", "STAGING: decoupler fired, stage separated");
        }
    }

    void StarWorksGame::updateWarp()
    {
        // X0 IS A WARP RATE. Stopping time is the bottom of the same ladder
        // every other rate lives on, so it is reached the same way: step
        // down from x1 and time stops; step up and it starts again, at x1.
        // Nothing else on the keyboard can pause the game by accident.
        // ---- WARPING TO THE NODE -------------------------------------------
        // The rung is chosen so one real second never advances more than
        // half the time that is left: the approach slows down of its own
        // accord and the last rung is x1, which is what stops the overshoot
        // a fixed ladder plus a human reaction time cannot avoid.
        // ---- SYNC WARP: catching another player's temporality -------------
        // Same servo as the node, one difference that matters: it is allowed
        // past the altitude ladder. Closing a three-hour gap from a 200 km
        // orbit at x100 would take a real hour, and nobody would use it.
        bool bypassAltitudeCap = false;
        if (m_syncWarpTo > 0.0)
        {
            const sw::f64 remaining = m_syncWarpTo - m_physicsLane->presentSeconds();
            if (remaining <= 0.0 || !netActive() || !warpAllowed())
            {
                if (remaining > 0.0)
                {
                    SW_LOG_INFO("Game", "Sync warp stopped: {}",
                                netActive() ? warpBlockReason() : "session ended");
                }
                else
                {
                    SW_LOG_INFO("Game", "Sync warp: caught up");
                }
                m_syncWarpTo = 0.0;
                m_syncWarpPlayer = 0;
                m_warpIndex = 0;
            }
            else
            {
                sw::u32 want = 0;
                for (sw::u32 i = 0; i < kWarpSteps; ++i)
                {
                    if (static_cast<sw::f64>(kWarpLadder[i]) <= remaining * 0.5)
                    {
                        want = i;
                    }
                }
                m_warpIndex = want;
                bypassAltitudeCap = true;
            }
        }
        if (m_syncWarpTo > 0.0 && (input().wasKeyPressed(sw::KeyCode::Period) ||
                                   input().wasKeyPressed(sw::KeyCode::Comma)))
        {
            m_syncWarpTo = 0.0;
            m_syncWarpPlayer = 0;
            SW_LOG_INFO("Game", "Sync warp cancelled");
        }

        if (m_warpToSeconds > 0.0)
        {
            if (!m_nodeActive)
            {
                m_warpToSeconds = 0.0; // the node it was aiming at is gone
                m_warpIndex = 0;
            }
            else
            {
                const sw::f64 remaining =
                    m_warpToSeconds - m_physicsLane->presentSeconds();
                if (remaining <= 0.0)
                {
                    m_warpToSeconds = 0.0;
                    m_warpIndex = 0;
                    SW_LOG_INFO("Game", "Warp to node: arrived");
                }
                else
                {
                    sw::u32 want = 0;
                    for (sw::u32 i = 0; i < kWarpSteps; ++i)
                    {
                        if (static_cast<sw::f64>(kWarpLadder[i]) <= remaining * 0.5)
                        {
                            want = i;
                        }
                    }
                    m_warpIndex = want;
                }
            }
        }
        // Any manual warp key takes the controls back.
        if (m_warpToSeconds > 0.0 && (input().wasKeyPressed(sw::KeyCode::Period) ||
                                      input().wasKeyPressed(sw::KeyCode::Comma)))
        {
            m_warpToSeconds = 0.0;
            SW_LOG_INFO("Game", "Warp to node cancelled");
        }

        if (keyPressed(sw::KeyCode::Period))
        {
            if (m_simulation.isPaused())
            {
                m_simulation.setPaused(false); // x0 -> x1
                SW_LOG_INFO("Game", "Simulation resumed");
            }
            else if (m_warpIndex + 1 < kWarpSteps)
            {
                ++m_warpIndex;
            }
        }
        if (keyPressed(sw::KeyCode::Comma) && !m_simulation.isPaused())
        {
            if (m_warpIndex > 0)
            {
                --m_warpIndex;
            }
            else
            {
                m_simulation.setPaused(true); // x1 -> x0
                SW_LOG_INFO("Game", "Simulation paused (warp x0)");
            }
        }

        // WHERE YOU ARE DECIDES HOW FAST TIME MAY RUN, and every body gets
        // a vote — the nearest one wins, judged in ITS OWN terms: its air,
        // its radius. A ladder in kilometres would have called Saturn's
        // cloud tops deep space, and an atmosphere constant tuned on Terra
        // would have put the step-down 4 000 km under Saturn's.
        const sw::WorldVec3 focusPosition =
            m_world.getComponent<TransformComponent>(controlledEntity()).position;
        sw::f32 maxAllowed = kWarpLadder[kWarpSteps - 1];
        sw::f64 nearestAltitude = 1.0e18;
        m_world.forEach<TransformComponent, sw::phys::GravitySourceComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                sw::phys::GravitySourceComponent& source) {
                const sw::f64 altitude =
                    glm::length(focusPosition - transform.position) - source.bodyRadius;
                sw::f64 air = 0.0;
                if (const auto* atmosphere =
                        m_world.tryGetComponent<sw::phys::AtmosphereComponent>(entity))
                {
                    air = atmosphere->topAltitude;
                }
                maxAllowed = std::min(
                    maxAllowed, maxWarpForAltitude(altitude, air, source.bodyRadius));
                nearestAltitude = std::min(nearestAltitude, altitude);
            });

        // ...AND THEN THE SAME QUESTION ONE SCALE UP. The altitude ladder
        // tops out at ten million and knows nothing about stars, so out
        // between them it has to be overruled rather than consulted: the two
        // interstellar rungs open only where there is no star's sphere of
        // influence to be inside of. That is a question about the ABSOLUTE
        // position — the origin travels with the ship, so the transform alone
        // cannot answer it.
        const sw::WorldVec3 absoluteFocus = absolutePosition(focusPosition);
        const sw::f64 distanceToStar = glm::length(
            absoluteFocus -
            sw::space::systems()[sw::space::nearestSystem(absoluteFocus)].position);
        const sw::f32 wasRequesting = kWarpLadder[m_warpIndex];
        maxAllowed = static_cast<sw::f32>(sw::phys::maxWarpForSpace(
            distanceToStar, static_cast<sw::f64>(maxAllowed)));
        // WHICH DISTANCE THE REFUSED RUNG WANTED, derived rather than written
        // out three times. There are three star-scale bands now and there will
        // be no fourth by accident: the message asks the ladder what the rung
        // needed, so adding a rung cannot leave a message saying the old
        // number.
        const sw::f64 neededRadius =
            sw::phys::warpRadiusForRate(static_cast<sw::f64>(wasRequesting));
        const bool wasStarBound =
            wasRequesting > maxAllowed &&
            maxAllowed >= static_cast<sw::f32>(sw::phys::kSystemWarpCeiling) &&
            neededRadius > 0.0;

        if (!bypassAltitudeCap)
        {
            const bool wasAboveCeiling = kWarpLadder[m_warpIndex] > maxAllowed;
            while (m_warpIndex > 0 && kWarpLadder[m_warpIndex] > maxAllowed)
            {
                --m_warpIndex;
            }
            if (wasAboveCeiling)
            {
                m_warpToSeconds = 0.0;
                SW_LOG_INFO("Game", "Warp limited to x{:g} (altitude {:.0f} km)",
                            kWarpLadder[m_warpIndex], nearestAltitude / 1000.0);
                // SAY IT, AND ONLY NOW — the frame the ceiling actually took
                // something away. A warning drawn from a standing condition
                // lives on the HUD for the whole game and tells nobody
                // anything.
                m_warpRefusedUntil = clock().totalSeconds() + 4.0;
                // The order is NEAREST CAUSE FIRST. Sitting on a launch pad
                // inside Sol's SOI at the top rung, all three are true at
                // once, and "INSIDE A STAR'S SOI" would be a strange thing to
                // read while looking at the sky through an atmosphere.
                m_warpRefusedReason =
                    (maxAllowed <= kMaxAtmosphericWarp)
                        ? std::string("IN ATMOSPHERE")
                    : wasStarBound
                        ? std::format("NEEDS {:.0f} BN KM FROM THE STAR",
                                      neededRadius / 1.0e9)
                        : std::string("TOO CLOSE TO THE BODY");
            }

            // ON FOOT, RAILS WARP IS NOT OFFERED AT ALL — and that is a
            // MOVEMENT rule before it is a physics one.
            //
            // Above physics warp the integrator is switched off and a landed body
            // becomes a surface anchor. For a rocket that is exactly right.
            // For the player it meant keeping the camera and losing the
            // legs: updateShipControls returns early above it, so W, A, S, D
            // and Space all stopped working, with nothing on screen saying
            // why. A person standing on a planet has nothing to fast-forward
            // through anyway — and the one case that genuinely needs to skip
            // hours on foot, catching another player's temporality, sets
            // bypassAltitudeCap and is deliberately not caught here.
            if (m_evaMode && kWarpLadder[m_warpIndex] > kMaxPhysicsWarp)
            {
                while (m_warpIndex > 0 && kWarpLadder[m_warpIndex] > kMaxPhysicsWarp)
                {
                    --m_warpIndex;
                }
                m_warpToSeconds = 0.0;
                m_warpRefusedUntil = clock().totalSeconds() + 4.0;
                m_warpRefusedReason = "ON FOOT - BOARD A CRAFT TO WARP";
            }
        }

        m_simulation.setTimeScale(kWarpLadder[m_warpIndex]);
        // Physics warp (<= x100 since F16): everything stays truly simulated — the
        // Physics lane is STRICT so a slow machine slows the sim instead
        // of desynchronizing it (the pre-M21 launch-pad fling). Rails
        // warp (above it): analytic orbits only, exact at any speed; drops
        // move the whole world coherently, so strictness is lifted.
        const bool physicsWarp = kWarpLadder[m_warpIndex] <= kMaxPhysicsWarp;
        m_physicsLane->setStrictCatchUp(physicsWarp);
        m_bubbleSystem->setForceRails(!physicsWarp);
        // ...and the budget that makes physics warp mean anything. Sixteen
        // ticks a frame is x19 and no more, so the lane is handed exactly
        // the catch-up it needs for the rung selected — and handed it back
        // when the rung drops, because a big budget is only wanted while
        // somebody is spending it (a hitch at x1 should cost one hitch, not
        // a second and a half of simulation). Under rails warp the number is
        // irrelevant: nothing is being integrated.
        m_physicsLane->setMaxTicksPerFrame(
            physicsWarp ? physicsTickBudget(kWarpLadder[m_warpIndex])
                        : physicsTickBudget(1.0f));
    }

    void StarWorksGame::updateShipControls()
    {
        // THE JUMP IS A LATCH, NOT AN EDGE, and this is why.
        //
        // `ShipControlsComponent` is cleared and rewritten here, once per
        // RENDERED FRAME. It is consumed by CapsuleMovementSystem, which
        // runs on the physics lane at a FIXED fifty hertz. Above sixty frames
        // a second most frames tick that lane zero times — so a jump written
        // as a one-frame edge was cleared again before any tick could see it,
        // and roughly one press in three did nothing at all.
        //
        // So the request stays set until a physics tick has actually run
        // (cleared in onUpdate, after `advance`). Pressing twice inside one
        // tick still jumps once: the system sets isGrounded to 0 as it goes,
        // and a second tick finds no ground to push off.
        const bool jumpRequested = m_jumpRequested;

        // Idle both control blocks, then feed the controlled one. There may
        // be NO SHIP: the world starts without one and stays that way until
        // a VAB builds a design and a pad rolls it out.
        ShipControlsComponent* shipControls =
            m_shipEntity.isNull()
                ? nullptr
                : m_world.tryGetComponent<ShipControlsComponent>(m_shipEntity);
        if (shipControls != nullptr)
        {
            *shipControls = {};
        }
        ShipControlsComponent* capsuleControls = nullptr;
        if (!m_capsuleEntity.isNull())
        {
            capsuleControls =
                m_world.tryGetComponent<ShipControlsComponent>(m_capsuleEntity);
            if (capsuleControls != nullptr)
            {
                *capsuleControls = {};
            }
        }

        if (!m_shipMode || m_mapView || m_editorMode ||
            kWarpLadder[m_warpIndex] > kMaxPhysicsWarp)
        {
            return; // engines only work while the world is truly simulated
        }

        ShipControlsComponent* controlsPtr =
            (m_evaMode && capsuleControls != nullptr) ? capsuleControls : shipControls;
        if (controlsPtr == nullptr)
        {
            return; // nothing under this player's hands
        }
        ShipControlsComponent& controls = *controlsPtr;

        const bool walking = m_evaMode && capsuleControls != nullptr;

        if (input().isKeyDown(sw::KeyCode::W)) { controls.thrustAxis += 1.0f; }
        if (input().isKeyDown(sw::KeyCode::S)) { controls.thrustAxis -= 1.0f; }
        if (input().isKeyDown(sw::KeyCode::Up)) { controls.rotationInput.x -= 1.0f; }
        if (input().isKeyDown(sw::KeyCode::Down)) { controls.rotationInput.x += 1.0f; }
        if (walking)
        {
            // ON FOOT the left/right keys SIDESTEP. Turning belongs to the
            // mouse, because the suit faces wherever the camera looks — a
            // walker that steers with keys and looks with the mouse makes
            // you fight two controls to go one direction.
            if (input().isKeyDown(sw::KeyCode::A)) { controls.strafeAxis -= 1.0f; }
            if (input().isKeyDown(sw::KeyCode::D)) { controls.strafeAxis += 1.0f; }
        }
        else
        {
            if (input().isKeyDown(sw::KeyCode::A)) { controls.rotationInput.y += 1.0f; }
            if (input().isKeyDown(sw::KeyCode::D)) { controls.rotationInput.y -= 1.0f; }
        }
        if (input().isKeyDown(sw::KeyCode::Q)) { controls.rotationInput.z += 1.0f; }
        if (input().isKeyDown(sw::KeyCode::E)) { controls.rotationInput.z -= 1.0f; }
        controls.killRotation = input().isKeyDown(sw::KeyCode::X) ? 1u : 0u;
        // JUMP, on foot only. The walker system consumes it on the first
        // tick that sees it and leaves the ground, so a held key does not
        // hover and a slow frame running four physics ticks does not jump
        // four times.
        if (walking && jumpRequested)
        {
            controls.jump = 1u;
        }
        // Throttle limiter (ship only; the capsule ignores it).
        if (input().isKeyDown(sw::KeyCode::LeftShift)) { controls.throttleDelta += 1.0f; }
        if (input().isKeyDown(sw::KeyCode::LeftControl))
        {
            controls.throttleDelta -= 1.0f;
        }
        // SW_THRUST=<0..1>: the throttle a capture cannot press. It belongs
        // HERE, at the end of the one function that decides the controls,
        // because that is the only place a value survives — written anywhere
        // downstream it is overwritten by this function on the next frame
        // before the animation that reads it ever runs.
        if (const char* thrustSpec = std::getenv("SW_THRUST");
            thrustSpec != nullptr && !walking && !m_shipEntity.isNull())
        {
            const sw::f32 level = glm::clamp(
                static_cast<sw::f32>(std::strtod(thrustSpec, nullptr)), 0.0f, 1.0f);
            controls.thrustAxis = 1.0f;
            if (auto* ship = m_world.tryGetComponent<ShipComponent>(m_shipEntity))
            {
                ship->throttle = level;
            }
        }
    }

    void StarWorksGame::updateChaseCamera(sw::f32 deltaSeconds)
    {
        const sw::ecs::Entity target = controlledEntity();
        const auto& transform = m_world.getComponent<TransformComponent>(target);
        const auto& previous = m_world.getComponent<PreviousTransformComponent>(target);

        const sw::f32 alpha = m_physicsLane->alpha();
        const sw::WorldVec3 position =
            glm::mix(previous.position, transform.position, static_cast<sw::f64>(alpha));
        const sw::Quat rotation = glm::slerp(previous.rotation, transform.rotation, alpha);

        // ---- THE CAMERA FRAME ------------------------------------------------
        // The craft's own rotation does NOT appear anywhere below, and that
        // is the whole design. A chase camera that inherits the vehicle's
        // attitude turns every roll, every RCS twitch and every SAS
        // correction into a camera move: the world swings around you while
        // you are trying to read it, and you cannot look at anything for
        // longer than the autopilot leaves the nose still. So the view has
        // its own orientation, and only the mouse changes it.
        //
        // ONE automatic behaviour survives, because it is the one that is
        // about the WORLD rather than about the vehicle: close to a body,
        // "up" means up. The camera levels on the local horizon — and it
        // stops there. It does not also follow where the rocket is pointing;
        // it just stops being upside down.
        sw::Vec3 radialUp{0.0f, 1.0f, 0.0f};
        sw::Quat horizonFrame{1.0f, 0.0f, 0.0f, 0.0f};
        bool wantHorizonLock = false;
        const sw::i32 primaryIndex = controlledPrimaryIndex();
        if (primaryIndex >= 0)
        {
            const auto& primary =
                m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
            const sw::WorldVec3 primaryPosition = m_celestialIndex.positionAt(
                primaryIndex, m_physicsLane->presentSeconds());
            const sw::WorldVec3 radial = position - primaryPosition;
            const sw::f64 distance = glm::length(radial);
            if (distance > 1.0)
            {
                radialUp = sw::Vec3(radial / distance);
                // Low relative to the BODY'S OWN SIZE — 3% of its radius, so
                // it means the same thing on a moon as on a planet.
                wantHorizonLock =
                    (distance - primary.bodyRadius) < 0.03 * primary.bodyRadius;

                // The frame's heading has to come from something that does
                // not move with the craft, or "level" would still track the
                // nose. NORTH does: the body's own spin axis, projected onto
                // the local horizontal.
                sw::Vec3 axis{0.0f, 1.0f, 0.0f};
                if (const auto* source =
                        m_world.tryGetComponent<sw::phys::GravitySourceComponent>(
                            primary.entity);
                    source != nullptr && glm::length(source->spinAxis) > 1.0e-12)
                {
                    axis = sw::Vec3(glm::normalize(source->spinAxis));
                }
                sw::Vec3 north = axis - radialUp * glm::dot(axis, radialUp);
                if (glm::length(north) < 1.0e-3f)
                {
                    // Straight over a pole: any horizontal direction will do,
                    // as long as it is a CONTINUOUS choice.
                    const sw::Vec3 reference =
                        (std::abs(radialUp.x) < 0.9f) ? sw::Vec3{1.0f, 0.0f, 0.0f}
                                                      : sw::Vec3{0.0f, 0.0f, 1.0f};
                    north = reference - radialUp * glm::dot(reference, radialUp);
                }
                north = glm::normalize(north);
                const sw::Vec3 east = glm::normalize(glm::cross(north, radialUp));
                horizonFrame = glm::quat_cast(sw::Mat3{east, radialUp, -north});
            }
        }
        const sw::f32 blendTarget = wantHorizonLock ? 1.0f : 0.0f;
        m_groundCamBlend +=
            (blendTarget - m_groundCamBlend) * std::min(1.0f, deltaSeconds * 2.0f);
        const sw::f32 blend = m_groundCamBlend;

        // Away from a body the reference is INERTIAL — the world axes. It
        // does not drift, it does not spin, and a camera parked in it stays
        // parked. Near one it becomes the horizon frame, eased across so
        // crossing the threshold is not a snap.
        const sw::Quat offsetRotation =
            glm::slerp(sw::Quat{1.0f, 0.0f, 0.0f, 0.0f}, horizonFrame, blend);

        // ---- user camera control: right-drag orbits, wheel zooms, C resets ----
        // ON FOOT the horizontal drag does not orbit the camera around the
        // player — it TURNS THE PLAYER, and the camera follows from behind.
        // That is what "you always walk where you are looking" means: there
        // is only one heading in the world, and the mouse owns it.
        CapsuleComponent* walker =
            m_evaMode ? m_world.tryGetComponent<CapsuleComponent>(target) : nullptr;
        if (input().isMouseButtonDown(sw::MouseButton::Right))
        {
            if (walker != nullptr)
            {
                walker->headingRadians += input().mouseDeltaX() * 0.0045f;
                m_chaseYaw = 0.0f; // the body turned instead
            }
            else
            {
                m_chaseYaw -= input().mouseDeltaX() * 0.0045f;
            }
            m_chasePitch =
                std::clamp(m_chasePitch - input().mouseDeltaY() * 0.0045f, -1.35f, 1.35f);
        }
        else if (walker != nullptr && m_chaseYaw != 0.0f)
        {
            // Coming back from the cockpit with a yawed camera: hand the
            // offset over to the body once, so the view does not sit at an
            // angle to the direction W walks in.
            walker->headingRadians -= m_chaseYaw;
            m_chaseYaw = 0.0f;
        }
        if (const sw::f32 scroll = input().scrollDeltaY(); scroll != 0.0f)
        {
            m_chaseZoom = std::clamp(
                m_chaseZoom * std::pow(1.18f, -scroll), 0.35f, 12.0f);
        }
        if (input().wasKeyPressed(sw::KeyCode::C))
        {
            m_chaseYaw = 0.0f;
            m_chasePitch = 0.0f;
            m_chaseZoom = 1.0f;
        }
        const sw::Quat userOrbit =
            glm::angleAxis(m_chaseYaw, sw::Vec3{0.0f, 1.0f, 0.0f}) *
            glm::angleAxis(m_chasePitch, sw::Vec3{1.0f, 0.0f, 0.0f});

        // ---- ON FOOT: FIRST PERSON -------------------------------------
        // A factory is built at arm's length. Watching your own back while
        // you place a machine puts the thing you are aiming at behind your
        // own shoulders, so EVA looks out of the suit's visor instead: the
        // camera sits at the head, the body's heading IS the view direction
        // (the mouse already turns it), and the pitch is a free look.
        if (walker != nullptr)
        {
            // The suit is a 2 m body centred on its transform; eyes just
            // under the top of it.
            constexpr sw::f32 kEyeHeight = 0.72f;
            const sw::WorldVec3 eye = position + sw::WorldVec3(radialUp * kEyeHeight);
            m_camera.setPosition(eye);

            // Heading comes from the body (rotation's -Z), pitch from the
            // mouse. Both are applied about the LOCAL vertical / local
            // right, so the horizon stays level on a round world.
            const sw::Vec3 bodyForward =
                glm::normalize(rotation * sw::math::kWorldForward);
            sw::Vec3 flat = bodyForward - radialUp * glm::dot(bodyForward, radialUp);
            if (glm::length(flat) < 1.0e-4f)
            {
                flat = rotation * sw::Vec3{1.0f, 0.0f, 0.0f};
            }
            flat = glm::normalize(flat);
            const sw::Vec3 lookRight = glm::normalize(glm::cross(flat, radialUp));
            const sw::Vec3 forward =
                glm::normalize(glm::angleAxis(m_chasePitch, lookRight) * flat);
            const sw::Vec3 up = glm::cross(lookRight, forward);
            m_camera.setOrientation(glm::quat_cast(sw::Mat3{lookRight, up, -forward}));
            return;
        }

        const sw::Vec3 chaseOffset = sw::Vec3{0.0f, 12.0f, 42.0f} * m_chaseZoom;
        const sw::WorldVec3 cameraPosition =
            position + sw::WorldVec3(offsetRotation * (userOrbit * chaseOffset));
        m_camera.setPosition(cameraPosition);

        const sw::Vec3 forward = glm::normalize(sw::Vec3(position - cameraPosition));
        // Camera up comes from the REFERENCE frame, never from the craft.
        sw::Vec3 targetUp = offsetRotation * sw::Vec3{0.0f, 1.0f, 0.0f};
        if (std::abs(glm::dot(forward, targetUp)) > 0.999f)
        {
            targetUp = offsetRotation * sw::Vec3{0.0f, 0.0f, 1.0f}; // straight down
        }
        const sw::Vec3 right = glm::normalize(glm::cross(forward, targetUp));
        const sw::Vec3 up = glm::cross(right, forward);
        m_camera.setOrientation(glm::quat_cast(sw::Mat3{right, up, -forward}));
    }

    void StarWorksGame::cyclePilotedVessel()
    {
        std::vector<sw::ecs::Entity> pilotable;
        m_world.forEach<ShipComponent>([&](sw::ecs::Entity entity, ShipComponent&) {
            pilotable.push_back(entity);
        });
        if (pilotable.empty())
        {
            SW_LOG_INFO("Game", "No vessel exists yet — order one at the VAB");
            return;
        }

        // ON FOOT, `P` BOARDS THE NEAREST ONE. Cycling through a list is the
        // right verb when you are already flying and want the other rocket;
        // it is the wrong one when you are standing next to exactly one.
        if (m_evaMode || m_shipEntity.isNull() || !m_world.isAlive(m_shipEntity))
        {
            const sw::WorldVec3 here =
                m_world.getComponent<TransformComponent>(controlledEntity()).position;
            sw::ecs::Entity best{};
            sw::f64 bestDistance = 0.0;
            for (const sw::ecs::Entity candidate : pilotable)
            {
                const auto* transform =
                    m_world.tryGetComponent<TransformComponent>(candidate);
                if (transform == nullptr)
                {
                    continue;
                }
                const sw::f64 distance = glm::length(transform->position - here);
                if (best.isNull() || distance < bestDistance)
                {
                    best = candidate;
                    bestDistance = distance;
                }
            }
            if (best.isNull())
            {
                return;
            }
            m_shipEntity = best;
            m_evaMode = false;
            m_sasMode = 0;
            despinBoardedVessel();
            SW_LOG_INFO("Game", "Boarded vessel {} ({:.0f} m away)", m_shipEntity.index,
                        bestDistance);
            return;
        }

        if (pilotable.size() < 2)
        {
            return;
        }
        sw::usize next = 0;
        for (sw::usize i = 0; i < pilotable.size(); ++i)
        {
            if (pilotable[i] == m_shipEntity)
            {
                next = (i + 1) % pilotable.size();
            }
        }
        m_shipEntity = pilotable[next];
        m_sasMode = 0;
        despinBoardedVessel();
        SW_LOG_INFO("Game", "PILOTING vessel {}", m_shipEntity.index);
    }

    // ------------------------------------------------------------------------
    // A RING THAT SPINS FOR GRAVITY DOES NOT SPIN FOR ITS PILOT.
    //
    // The Endurance cruises at 5.6 rpm because that is what puts a floor
    // under its crew — and it is also 0.59 rad/s of roll, which is a ship
    // nobody can fly and a camera nobody can watch. So taking the controls
    // de-spins it, the way its crew does before every manoeuvre in the
    // film: the rate is zeroed on the SpinComponent, which stops both spin
    // systems at once and rides in the save like any other state.
    //
    // It stays stopped. Nothing here spins it back up, and that is honest
    // rather than lazy — restarting the ring is a crew action this game
    // does not have yet, and pretending otherwise would mean a ship that
    // silently starts rolling the moment you look away.
    // ------------------------------------------------------------------------
    void StarWorksGame::despinBoardedVessel()
    {
        auto* spin = m_world.tryGetComponent<SpinComponent>(m_shipEntity);
        if (spin == nullptr || spin->radiansPerSecond == 0.0f)
        {
            return;
        }
        SW_LOG_INFO("Game", "Ring de-spun for manoeuvring ({:.2f} rpm -> 0)",
                    spin->radiansPerSecond * 60.0f / glm::two_pi<sw::f32>());
        spin->radiansPerSecond = 0.0f;
        if (auto* body = m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(
                m_shipEntity))
        {
            body->angularVelocity = sw::WorldVec3{0.0};
        }
    }

    void StarWorksGame::refreshFlightState()
    {
        m_flight = FlightState{};

        const sw::ecs::Entity flown = controlledEntity();
        if (flown.isNull() || !m_world.isAlive(flown))
        {
            return;
        }

        // STANDING ON SOMETHING. Two spellings of the same fact: a live body
        // resting on terrain, or — once rails warp has already converted it —
        // a surface anchor, which has no DynamicBodyComponent at all. Testing
        // only the first would make the gate slam shut the instant the warp
        // it permitted took effect.
        if (const auto* body = m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(flown))
        {
            m_flight.grounded = body->isGrounded != 0;
        }
        if (m_world.tryGetComponent<sw::phys::SurfaceAnchorComponent>(flown) != nullptr)
        {
            m_flight.grounded = true;
        }
        // A HOP IS NOT A FLIGHT — see the gravity-sized latch further down,
        // which needs the primary this only records the footing for.
        const bool onFoot =
            m_evaMode && !m_capsuleEntity.isNull() && flown == m_capsuleEntity;
        if (onFoot && m_flight.grounded)
        {
            m_lastFootingSeconds = clock().totalSeconds();
        }

        m_flight.primaryIndex = controlledPrimaryIndex();
        if (m_flight.primaryIndex < 0)
        {
            return;
        }

        const auto& primary =
            m_celestialIndex.body(static_cast<sw::usize>(m_flight.primaryIndex));
        const sw::f64 time = m_physicsLane->presentSeconds();
        sw::WorldVec3 primaryPosition{0.0};
        sw::WorldVec3 primaryVelocity{0.0};
        m_celestialIndex.stateAt(m_flight.primaryIndex, time, primaryPosition,
                                 &primaryVelocity);

        const sw::WorldVec3 position =
            m_world.getComponent<TransformComponent>(flown).position;
        m_flight.altitude = glm::length(position - primaryPosition) - primary.bodyRadius;

        if (const auto* air =
                m_world.tryGetComponent<sw::phys::AtmosphereComponent>(primary.entity))
        {
            m_flight.atmosphereTop = air->topAltitude;
        }

        // A HOP IS NOT A FLIGHT.
        //
        // `isGrounded` is exact and momentary: it goes to zero the instant
        // the feet leave the dirt. The warp gate read it directly, so
        // pressing Space slammed the whole gate shut for the length of a
        // jump — and the panel put a red WARP LOCKED under the pilot list
        // for it. Measured on Terra: a 4.5 m/s jump peaks at 1.77 m and is
        // is off the ground for 0.88 s.
        //
        // The question the gate really asks is "could this safely go on
        // rails?", and for a person a metre and a half above their own
        // planet the honest answer is yes. So the footing is REMEMBERED for
        // as long as a jump can physically last.
        //
        // That window is computed from the local gravity and not written
        // down, because a constant would be wrong nearly everywhere: the
        // same legs and the same key give 0.92 s of air on Terra, 2.4 s on
        // Mars and 5.6 s on Luna. A body genuinely in flight — thrown clear,
        // stepping off a cliff — outlasts its own hang time and the gate
        // closes exactly as it should.
        if (onFoot && !m_flight.grounded && m_lastFootingSeconds > 0.0)
        {
            const sw::f64 surfaceGravity =
                primary.mu / (primary.bodyRadius * primary.bodyRadius);
            sw::f32 jumpSpeed = CapsuleComponent{}.jumpSpeed;
            if (const auto* capsule = m_world.tryGetComponent<CapsuleComponent>(flown))
            {
                jumpSpeed = capsule->jumpSpeed;
            }
            // The rule lives in the engine (phys::jumpHangSeconds) so it can
            // be tested without a window; this only feeds it the situation.
            const sw::f64 hangSeconds = sw::phys::jumpHangSeconds(
                static_cast<sw::f64>(jumpSpeed), surfaceGravity);
            if (clock().totalSeconds() - m_lastFootingSeconds < hangSeconds)
            {
                m_flight.grounded = true;
            }
        }

        if (m_flight.grounded)
        {
            // LEANING, AND WHETHER THAT IS A PROBLEM. Measured against the
            // same statics the ground contact uses, so the readout and the
            // physics can never disagree.
            const sw::Vec3 up = glm::normalize(sw::Vec3(position - primaryPosition));
            const auto& transform = m_world.getComponent<TransformComponent>(flown);
            const sw::Vec3 nose = transform.rotation * sw::Vec3{0.0f, 0.0f, -1.0f};
            m_flight.leanDegrees = static_cast<sw::f32>(
                std::acos(std::clamp(static_cast<sw::f64>(glm::dot(nose, up)), -1.0, 1.0)) *
                180.0 / 3.14159265358979);

            const auto* hull = m_world.tryGetComponent<sw::phys::GroundHullComponent>(flown);
            const auto* vessel = m_world.tryGetComponent<sw::parts::VesselComponent>(flown);
            const auto* body = m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(flown);
            if (hull != nullptr && vessel != nullptr && body != nullptr)
            {
                const sw::Vec3 topple = sw::phys::topplingAcceleration(
                    *hull, transform.rotation, up, vessel->centreOfMass,
                    vessel->inertiaKgM2, body->mass,
                    primary.mu / (primary.bodyRadius * primary.bodyRadius));
                m_flight.tipping = glm::length(topple) > 1.0e-6f;
            }
            return; // no orbit to speak of, and none needed
        }

        sw::phys::KeplerOrbit orbit{};
        if (!sw::phys::kepler::fromStateVectors(primary.mu, position - primaryPosition,
                                                controlledVelocity() - primaryVelocity,
                                                time, orbit, /*allowHyperbolic=*/true))
        {
            return;
        }
        m_flight.periapsisAltitude = sw::phys::kepler::periapsis(orbit) - primary.bodyRadius;
        m_flight.closedOrbit = !orbit.isHyperbolic();
        if (m_flight.closedOrbit)
        {
            m_flight.apoapsisAltitude =
                sw::phys::kepler::apoapsis(orbit) - primary.bodyRadius;
        }
    }

    /// The SYNC-warp gate only (F17): catching another player's clock skips
    /// hours on rails, and that is the one warp still worth refusing on the
    /// shape of the trajectory. The ordinary ladder is altitude alone.
    bool StarWorksGame::warpAllowed() const
    {
        // The rule itself lives in the engine (phys::warpPermitted) so it can
        // be tested without a window; this only feeds it this frame's state.
        return sw::phys::warpPermitted(m_flight.grounded, m_flight.closedOrbit,
                                       m_flight.periapsisAltitude,
                                       m_flight.atmosphereTop);
    }

    const char* StarWorksGame::warpBlockReason() const
    {
        if (warpAllowed())
        {
            return "";
        }
        if (!m_flight.closedOrbit)
        {
            return "NO CLOSED ORBIT";
        }
        return "PERIAPSIS IN AIR";
    }
} // namespace game
