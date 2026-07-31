// ============================================================================
// GameFactoryUi.cpp — Factory-facing UI: build ghosts, overlays, the build catalogue, the E panel.
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

    // The cable preview: the span you are about to get, hung exactly as
    // `hangCable` will hang it, in the colour of the verdict. Same curve
    // function as the real thing, so what you are shown IS what you build.
    void StarWorksGame::collectCableGhost(const sw::Camera& activeCamera)
    {
        if (m_cableSource.isNull() || m_buildCursor.target.isNull() ||
            m_cableMeshIndex == 0xFFFFFFFFu)
        {
            return;
        }
        sw::WorldVec3 from{};
        sw::WorldVec3 to{};
        const sw::factory::CableVerdict verdict =
            planCable(m_cableSource, m_buildCursor.target, from, to);
        if (glm::length(to - from) < 1.0e-6)
        {
            return;
        }
        CableComponent preview{};
        preview.body = m_buildCursor.body;
        hangCable(preview, from, to);

        sw::WorldVec3 bodyPosition{};
        glm::dquat bodyRotation{};
        bodyRenderPose(m_buildCursor.body, bodyPosition, bodyRotation);
        const sw::Vec4 tint = (verdict == sw::factory::CableVerdict::Ok)
                                  ? sw::Vec4{0.35f, 1.0f, 0.45f, 0.55f}
                                  : sw::Vec4{1.0f, 0.35f, 0.30f, 0.45f};

        for (sw::u32 i = 0; i + 1 < preview.pointCount; ++i)
        {
            const sw::WorldVec3 a = bodyPosition + bodyRotation * preview.points[i];
            const sw::WorldVec3 b = bodyPosition + bodyRotation * preview.points[i + 1];
            const sw::WorldVec3 delta = b - a;
            const sw::f64 length = glm::length(delta);
            if (length < 1.0e-3)
            {
                continue;
            }
            const sw::Vec3 forward = sw::Vec3(delta / length);
            const sw::Vec3 hint = sw::Vec3(
                glm::normalize(bodyRotation * glm::normalize(a - bodyPosition)));
            sw::Vec3 right = glm::cross(forward, hint);
            if (glm::length(right) < 1.0e-4f)
            {
                right = glm::cross(forward, sw::Vec3{0.0f, 0.0f, 1.0f});
            }
            right = glm::normalize(right);
            const sw::Vec3 realUp = glm::cross(right, forward);

            const sw::Vec3 relative = sw::Vec3((a + b) * 0.5 - activeCamera.position());
            sw::DrawItem item{};
            item.mesh = &m_meshes[m_cableMeshIndex];
            item.transform =
                glm::translate(sw::Mat4{1.0f}, relative) *
                glm::mat4_cast(glm::quat_cast(sw::Mat3{right, realUp, -forward})) *
                glm::scale(sw::Mat4{1.0f},
                           sw::Vec3{2.2f, 2.2f,
                                    static_cast<sw::f32>(length) / m_cableSegmentM});
            item.boundsCenter = relative;
            item.boundsRadius = static_cast<sw::f32>(length);
            item.tint = tint;
            item.transparent = true;
            m_drawItems.push_back(item);
        }
    }

    void StarWorksGame::collectBuildGhost(const sw::Camera& activeCamera)
    {
        if (!m_buildCursor.active || m_heldBuilding == 0)
        {
            return;
        }
        const auto* held = sw::parts::findDefinition(m_heldBuilding);
        const auto meshIt = m_partMeshIds.find(m_heldBuilding);
        if (held == nullptr || meshIt == m_partMeshIds.end())
        {
            return;
        }
        const bool beltMode =
            held->building.category == sw::factory::BuildingCategory::Conveyor;
        if (beltMode && m_beltPreview.empty())
        {
            return; // nothing picked yet, or nothing to show
        }
        // A CABLE has no ground ghost to draw: it is not placed on a spot,
        // it is strung between two nodes. Its preview is the span itself,
        // drawn below once both ends are known.
        if (held->building.category == sw::factory::BuildingCategory::Cable)
        {
            collectCableGhost(activeCamera);
            return;
        }
        const auto* gravity =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(m_buildCursor.body);
        const auto* terrain =
            m_world.tryGetComponent<sw::planet::TerrainComponent>(m_buildCursor.body);
        const auto* bodyTransform =
            m_world.tryGetComponent<TransformComponent>(m_buildCursor.body);
        if (gravity == nullptr || terrain == nullptr || bodyTransform == nullptr)
        {
            return;
        }

        // The RENDERED pose — the ghost has to sit on the ground the player
        // can see, which is the interpolated one.
        sw::WorldVec3 bodyPosition{};
        glm::dquat spin{};
        bodyRenderPose(m_buildCursor.body, bodyPosition, spin);

        const bool ok = beltMode ? (m_beltVerdict == sw::build::Verdict::Ok)
                                 : (m_buildCursor.verdict == sw::build::Verdict::Ok);
        const sw::Vec4 tint = ok ? sw::Vec4{0.35f, 1.0f, 0.45f, 0.45f}
                                 : sw::Vec4{1.0f, 0.35f, 0.30f, 0.40f};

        auto pushGhost = [&](const sw::Vec3& up, sw::f32 yaw) {
            const sw::f64 elevation = sw::planet::terrainElevation(*terrain, up);
            const sw::WorldVec3 world =
                bodyPosition +
                spin * (sw::WorldVec3(up) * (gravity->bodyRadius + elevation));
            const sw::Vec3 relative = sw::Vec3(world - activeCamera.position());
            sw::DrawItem item{};
            item.mesh = &m_meshes[meshIt->second];
            item.transform =
                glm::translate(sw::Mat4{1.0f}, relative) *
                glm::mat4_cast(sw::Quat(spin) * standUpFor(up) *
                               glm::angleAxis(yaw, sw::Vec3{0.0f, 1.0f, 0.0f}));
            item.boundsCenter = relative;
            item.boundsRadius = sw::parts::partBoundsRadius(*held) + 0.5f;
            item.tint = tint;
            item.transparent = true;
            m_drawItems.push_back(item);
        };

        if (beltMode)
        {
            // The WHOLE RUN, previewed. What you are shown in green is what
            // planBelt will hand to placeBuilding — the same tiles, from the
            // same call — so a belt cannot come out different when you click.
            for (const BeltTile& tile : m_beltPreview)
            {
                pushGhost(tile.direction, tile.yawRadians);
            }
            return;
        }
        pushGhost(m_buildCursor.direction, m_buildCursor.yawRadians);
    }

    // ------------------------------------------------------------------------
    // CARGO ON THE BELTS
    //
    // No item entities, no second simulation: the crates are a closed-form
    // function of the lane's present time and the link's MEASURED
    // throughput. That is the same discipline the orbits and the planet
    // spin already follow, and it buys the same three things — it is exact
    // under time warp, it costs nothing when nobody is looking, and it
    // cannot drift away from the matter it depicts.
    //
    // Spacing IS the flow: crates/second = flow / unitsPerCrate, so a belt
    // fed by a starving mine visibly thins out and a stopped one empties.
    // ------------------------------------------------------------------------
    // ------------------------------------------------------------------------
    // THE CABLES
    //
    // A span is one entity, not a row of them — you do not demolish a metre
    // of wire — so unlike the belt deck there is no per-tile entity for the
    // world pass to draw, and the curve is stroked here.
    //
    // It goes through `bodyRenderPose` for the same reason the build ghost
    // and the belt cargo do: the endpoints are in the body's ROTATING frame,
    // and transforming them with the planet's tick pose instead of the pose
    // it is being DRAWN at would hang every wire up to 595 m off its poles,
    // resetting every tick. That rule has now cost three features; it is
    // cheaper to obey it than to rediscover it.
    // ------------------------------------------------------------------------
    // F2 — WHAT YOU ACTUALLY BUMP INTO.
    //
    // Every solid hull, drawn as the boxes it is. Green for the things that
    // stand still, amber for the things that get pushed out of them, so the
    // player's own box is never confused with the world's. Belts and cables
    // draw nothing, because they have no hull — which is itself the fastest
    // way to confirm they are walk-through.
    void StarWorksGame::collectHullOverlay(const sw::Camera& activeCamera)
    {
        if (!m_showHitboxes || m_hullBoxMeshIndex == 0xFFFFFFFFu)
        {
            return;
        }
        const sw::WorldVec3 cameraPosition = activeCamera.position();
        // THE RENDERED POSE, not the tick pose. A building's TransformComponent
        // is where it was at the last physics step; the mesh over it is drawn
        // at the interpolated pose, and on Terra one step of orbital motion is
        // 595 m. Reading the raw transform here drew every box swimming beside
        // its own machine and snapping back every tick — the same mistake that
        // has now cost the conveyor cargo, the build ghost and the cables, so
        // this uses the identical mix the mesh pass uses rather than something
        // that merely looks equivalent.
        const sw::f32 alpha = m_physicsLane->alpha();
        const sw::f64 alpha64 = static_cast<sw::f64>(alpha);

        m_world.forEach<TransformComponent, PreviousTransformComponent,
                        sw::phys::HullComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                PreviousTransformComponent& previous, sw::phys::HullComponent& hull) {
                const sw::WorldVec3 position =
                    glm::mix(previous.position, transform.position, alpha64);
                const sw::Quat rotation =
                    glm::slerp(previous.rotation, transform.rotation, alpha);

                const sw::WorldVec3 offset = position - cameraPosition;
                // Same broad phase as the collision itself: a box a
                // kilometre away is not a box you are inspecting.
                if (glm::dot(offset, offset) > 1.0e6)
                {
                    return;
                }
                const bool mover =
                    m_world.hasComponent<sw::phys::HullMoverComponent>(entity);
                const sw::Vec4 tint = mover ? sw::Vec4{1.0f, 0.72f, 0.20f, 0.30f}
                                            : sw::Vec4{0.30f, 1.0f, 0.55f, 0.22f};
                for (sw::u32 i = 0; i < hull.count; ++i)
                {
                    const sw::Vec3 relative =
                        sw::Vec3(offset) + rotation * hull.boxes[i].centre;
                    sw::DrawItem item{};
                    item.mesh = &m_meshes[m_hullBoxMeshIndex];
                    item.transform =
                        glm::translate(sw::Mat4{1.0f}, relative) *
                        glm::mat4_cast(rotation) *
                        // The cube is 1 m across, so its half extent is 0.5:
                        // scaling by the full extent is what makes the drawn
                        // box the same size as the one being tested.
                        glm::scale(sw::Mat4{1.0f},
                                   glm::max(hull.boxes[i].halfExtents * 2.0f,
                                            sw::Vec3{0.02f}));
                    item.boundsCenter = relative;
                    item.boundsRadius = glm::length(hull.boxes[i].halfExtents) + 0.5f;
                    item.tint = tint;
                    item.transparent = true;
                    m_drawItems.push_back(item);
                }
            });
    }

    void StarWorksGame::collectCables(const sw::Camera& activeCamera)
    {
        if (m_cableMeshIndex == 0xFFFFFFFFu)
        {
            return;
        }
        const sw::WorldVec3 cameraPosition = activeCamera.position();

        m_world.forEach<sw::factory::PowerLinkComponent, CableComponent>(
            [&](sw::ecs::Entity, sw::factory::PowerLinkComponent& link,
                CableComponent& cable) {
            if (cable.pointCount < 2 || cable.body.isNull())
            {
                return;
            }
            // Is this wire's grid short? Read it off either end — they are
            // on the same grid by construction, that being what a cable is.
            if (const auto* power =
                    m_world.tryGetComponent<sw::factory::PowerComponent>(link.a))
            {
                cable.starved = power->gridProducedKw + 1.0e-9 < power->gridConsumedKw;
            }
            sw::WorldVec3 bodyPosition{};
            glm::dquat bodyRotation{};
            bodyRenderPose(cable.body, bodyPosition, bodyRotation);

            // A wire is 11 cm across. Past a couple of kilometres it is not
            // a pixel, and stroking twelve segments of it is pure cost.
            const sw::WorldVec3 anchor = bodyPosition + bodyRotation * cable.points[0];
            if (glm::length(anchor - cameraPosition) > 3000.0)
            {
                return;
            }

            // A browning-out grid dims its own wires. It is the cheapest
            // possible answer to "why is my smelter stopped" that does not
            // involve opening a panel.
            const sw::Vec4 tint = cable.starved ? sw::Vec4{0.62f, 0.32f, 0.26f, 1.0f}
                                                : sw::Vec4{1.0f, 1.0f, 1.0f, 1.0f};

            for (sw::u32 i = 0; i + 1 < cable.pointCount; ++i)
            {
                const sw::WorldVec3 a = bodyPosition + bodyRotation * cable.points[i];
                const sw::WorldVec3 b = bodyPosition + bodyRotation * cable.points[i + 1];
                const sw::WorldVec3 delta = b - a;
                const sw::f64 length = glm::length(delta);
                if (length < 1.0e-3)
                {
                    continue;
                }
                // The part's cylinder runs along its own Z, and model -Z is
                // "forward" everywhere in this codebase, so the segment is
                // aimed the way a belt tile is and stretched to fit.
                const sw::Vec3 forward = sw::Vec3(delta / length);
                const sw::Vec3 hint = sw::Vec3(
                    glm::normalize(bodyRotation * glm::normalize(a - bodyPosition)));
                sw::Vec3 right = glm::cross(forward, hint);
                if (glm::length(right) < 1.0e-4f)
                {
                    right = glm::cross(forward, sw::Vec3{0.0f, 0.0f, 1.0f});
                }
                right = glm::normalize(right);
                const sw::Vec3 realUp = glm::cross(right, forward);

                const sw::Vec3 relative = sw::Vec3((a + b) * 0.5 - cameraPosition);
                sw::DrawItem item{};
                item.mesh = &m_meshes[m_cableMeshIndex];
                item.transform =
                    glm::translate(sw::Mat4{1.0f}, relative) *
                    glm::mat4_cast(glm::quat_cast(sw::Mat3{right, realUp, -forward})) *
                    glm::scale(sw::Mat4{1.0f},
                               sw::Vec3{1.0f, 1.0f,
                                        static_cast<sw::f32>(length) / m_cableSegmentM});
                item.boundsCenter = relative;
                item.boundsRadius = static_cast<sw::f32>(length);
                item.tint = tint;
                m_drawItems.push_back(item);
            }
        });
    }

    void StarWorksGame::collectConveyors(const sw::Camera& activeCamera)
    {
        const sw::WorldVec3 cameraPosition = activeCamera.position();
        // EVERY POSE ON SCREEN IS INTERPOLATED, and cargo is no exception.
        // The first version read the belt's TICK pose while the deck under
        // it was drawn at the interpolated one, and the two are a full
        // physics step apart — which for a planet moving 30 km/s around its
        // star is up to 595 metres. The crates were not floating up: they
        // were being drawn where the belt had been (or would be) one tick
        // away, in whatever direction Terra's orbit happened to point.
        const sw::f32 alpha = m_physicsLane->alpha();
        const sw::f64 alpha64 = static_cast<sw::f64>(alpha);
        // The cargo's own phase gets the same treatment: `presentSeconds` is
        // quantised to the tick, so using it raw makes crates advance in
        // 5 cm hops instead of gliding.
        const sw::f64 now = m_physicsLane->presentSeconds() +
                            alpha64 * static_cast<sw::f64>(m_physicsLane->stepSeconds());

        m_world.forEach<TransformComponent, PreviousTransformComponent,
                        ConveyorComponent>(
            [&](sw::ecs::Entity, TransformComponent& transform,
                PreviousTransformComponent& previous, ConveyorComponent& conveyor) {
                if (conveyor.pointCount < 2 || conveyor.lengthM < 1.0f)
                {
                    return;
                }
                // The belt's pose THIS FRAME — the same mix the deck mesh is
                // drawn with, so the cargo can only ever be on the deck.
                const sw::WorldVec3 beltPosition =
                    glm::mix(previous.position, transform.position, alpha64);
                const glm::dquat beltRotation{
                    glm::slerp(previous.rotation, transform.rotation, alpha)};

                // Belts are metres wide: past a few kilometres they are not
                // even a pixel, and their cargo certainly is not.
                const sw::f64 distance = glm::length(beltPosition - cameraPosition);
                if (distance > 4000.0)
                {
                    return;
                }

                // The path in world space, rebuilt from the belt's own pose:
                // its anchor already carries the body's f64 rotation, so the
                // cargo rides exactly where the deck was drawn.
                const sw::WorldVec3 origin = conveyor.points[0];

                // Model -Z is the direction of travel and +Y is up: one
                // frame for the deck tiles and the crates alike.
                auto placeAlong = [&](sw::f64 arcLength, sw::f32 rideHeight,
                                      sw::f32 stretchZ, sw::u32 meshIndex,
                                      const sw::Vec4& tint, sw::f32 boundsRadius) {
                    sw::WorldVec3 local{};
                    sw::Vec3 heading{};
                    sw::factory::conveyorPointAt(conveyor.points, conveyor.pointCount,
                                                 arcLength, local, heading);
                    // `local` and `up` are BODY-FRAME; the ride height is
                    // added there, before the rotation, or the part is
                    // lifted along a world axis instead of the local one.
                    const glm::dvec3 up = glm::normalize(local);
                    const sw::WorldVec3 world =
                        beltPosition +
                        beltRotation * (local - origin + up * static_cast<sw::f64>(
                                                                  rideHeight));
                    const sw::Vec3 relative = sw::Vec3(world - cameraPosition);
                    const sw::Vec3 forward =
                        sw::Vec3(glm::normalize(beltRotation * glm::dvec3(heading)));
                    const sw::Vec3 worldUp =
                        sw::Vec3(glm::normalize(beltRotation * up));
                    const sw::Vec3 right = glm::normalize(glm::cross(forward, worldUp));
                    const sw::Vec3 realUp = glm::cross(right, forward);

                    sw::DrawItem item{};
                    item.mesh = &m_meshes[meshIndex];
                    item.transform =
                        glm::translate(sw::Mat4{1.0f}, relative) *
                        glm::mat4_cast(
                            glm::quat_cast(sw::Mat3{right, realUp, -forward})) *
                        glm::scale(sw::Mat4{1.0f}, sw::Vec3{1.0f, 1.0f, stretchZ});
                    item.boundsCenter = relative;
                    item.boundsRadius = boundsRadius;
                    item.tint = tint;
                    m_drawItems.push_back(item);
                };

                // The DECK is not drawn here any more: since F2 a belt is a
                // row of ordinary CV-1 building entities, and they are drawn
                // by the same pass as every other mesh in the world. What is
                // left for this function is the one thing no entity holds —
                // the cargo, which is a function of time and flow.
                // ---- and only NOW, what is riding it ---------------------
                // The deck above is drawn UNCONDITIONALLY, because a belt is
                // a structure: it exists whether or not goods are on it.
                // That was the bug — the whole conveyor sat behind the flow
                // gate below, so every time the link's source ran dry for a
                // tick the belt itself blinked out of the world.
                const sw::u32 cargoMesh = (conveyor.cargoMesh != 0xFFFFFFFFu)
                                              ? conveyor.cargoMesh
                                              : m_cargoMeshIndex;
                if (cargoMesh == 0xFFFFFFFFu)
                {
                    return;
                }
                const auto* link =
                    m_world.tryGetComponent<sw::factory::ItemLinkComponent>(
                        conveyor.link);
                // Everything this belt's source is putting on it, whatever
                // that is: two gases down one run are two streams of crates.
                const sw::f64 flow =
                    (link != nullptr)
                        ? sw::factory::linkFlowFrom(*link, conveyor.source)
                        : 0.0;
                if (flow <= 1.0e-6)
                {
                    return; // stopped: an empty belt is the honest picture
                }
                const sw::f64 cratesPerSecond =
                    flow / std::max(static_cast<sw::f64>(conveyor.unitsPerCrate), 1.0e-6);
                const sw::f64 spacing =
                    static_cast<sw::f64>(conveyor.speedMps) / cratesPerSecond;
                if (spacing < 0.35)
                {
                    return; // shoulder to shoulder: draw the deck, not confetti
                }
                const sw::i32 crates =
                    std::min(32, static_cast<sw::i32>(conveyor.lengthM / spacing));
                if (crates <= 0)
                {
                    return;
                }
                const sw::f64 travelled =
                    std::fmod(now * static_cast<sw::f64>(conveyor.speedMps),
                              static_cast<sw::f64>(conveyor.lengthM));

                // ---- THE CARGO: the CR-1 prop, tinted per resource -------
                // It rides ON the deck, so its height is the deck's own top
                // surface — read off the CV-1's collider box, not guessed.
                for (sw::i32 c = 0; c < crates; ++c)
                {
                    const sw::f64 s =
                        std::fmod(travelled + static_cast<sw::f64>(c) * spacing,
                                  static_cast<sw::f64>(conveyor.lengthM));
                    placeAlong(s, m_conveyorDeckHeightM, 1.0f, cargoMesh,
                               {conveyor.cargoColor.r, conveyor.cargoColor.g,
                                conveyor.cargoColor.b, 1.0f},
                               (cargoMesh == m_vehicleCargoMeshIndex) ? 4.4f : 0.6f);
                }
            });
    }

    // ------------------------------------------------------------------------
    // THE BUILD MENU (F)
    //
    // Satisfactory's lesson, applied: the catalogue of things you can put on
    // the ground is a first-class screen, not a submenu of a vehicle editor.
    // The VAB (B) assembles ROCKETS out of parts; this assembles a FACTORY
    // out of buildings, and the two never share a palette because a refinery
    // is not something you bolt to a fuel tank.
    //
    // What it does today is arm a definition — `m_heldBuilding` — and show
    // what that building costs and needs. F2 turns that armed id into a
    // ghost on the terrain and a placement; nothing here will have to move
    // when it does, because the id IS the contract.
    // ------------------------------------------------------------------------
    void StarWorksGame::collectBuildMenu()
    {
        m_hudButtons.clear();

        std::vector<const sw::parts::PartDefinition*> buildings;
        for (const sw::parts::PartDefinition& definition : sw::parts::catalog())
        {
            if (sw::parts::isBuilding(definition))
            {
                buildings.push_back(&definition);
            }
        }

        constexpr sw::f32 kLeft = -0.62f;
        constexpr sw::f32 kRight = 0.62f;
        constexpr sw::f32 kTop = -0.56f;
        constexpr sw::f32 kRowHeight = 0.076f;
        constexpr sw::f32 kRowGap = 0.008f;
        constexpr sw::f32 kHeaderH = 0.090f;
        constexpr sw::f32 kFooterH = 0.115f;
        constexpr sw::f32 kPad = 0.018f;

        sw::f32 cursorX = -2.0f;
        sw::f32 cursorY = -2.0f;
        const bool haveCursor = hudCursor(cursorX, cursorY);
        auto hovering = [&](sw::f32 y) {
            return haveCursor && cursorX >= kLeft + kPad && cursorX <= kRight - kPad &&
                   cursorY >= y && cursorY <= y + kRowHeight;
        };

        const sw::f32 listTop = kTop + kHeaderH;
        const sw::f32 bottom =
            listTop + static_cast<sw::f32>(buildings.size()) * (kRowHeight + kRowGap) +
            kFooterH;
        hudPanel(kLeft, kTop, kRight, bottom, hud::kPanel);
        hudQuad(kLeft, kTop, kRight, listTop - 0.006f, hud::kHeader);
        hudText("BUILD", kLeft + 0.025f, kTop + 0.028f, 0.052f, hud::kTitle);
        hudText("CLICK TO ARM   F CLOSE", kRight - 0.34f, kTop + 0.040f, 0.028f,
                hud::kTextDim);

        sw::f32 rowY = listTop;
        for (sw::usize i = 0; i < buildings.size(); ++i)
        {
            const sw::parts::PartDefinition& definition = *buildings[i];
            const sw::parts::BuildingSpec& spec = definition.building;
            const bool armed = definition.id == m_heldBuilding;
            const bool hot = hovering(rowY);

            const sw::Vec4 fill = armed ? (hot ? hud::kRowOnHover : hud::kRowOn)
                                        : (hot ? hud::kRowHover
                                               : ((i % 2 == 0) ? hud::kRow : hud::kRowAlt));
            hudQuad(kLeft + kPad, rowY, kRight - kPad, rowY + kRowHeight, fill);
            // The category chip: a colour band down the left edge of the row.
            hudQuad(kLeft + kPad, rowY, kLeft + kPad + 0.012f, rowY + kRowHeight,
                    hud::categoryColor(spec.category));

            hudText(hud::caps(definition.name), kLeft + kPad + 0.028f, rowY + 0.012f,
                    0.036f, armed ? hud::kTitle : hud::kText);
            hudText(hud::caps(std::string(sw::factory::categoryName(spec.category))),
                    kLeft + kPad + 0.028f, rowY + 0.050f, 0.024f,
                    hud::categoryColor(spec.category));

            // The spec, read straight off the .swpart: how much ground it
            // needs, and what it does to the grid.
            std::string summary =
                std::format("{:.0f}X{:.0f} M   {}", spec.footprintM[0], spec.footprintM[1],
                            hud::powerText(spec.powerKw));
            if (spec.inventoryVolumeM3 > 0.0)
            {
                summary += std::format("   {:.0f} M3", spec.inventoryVolumeM3);
            }
            if (spec.minOreDensity > 0.0)
            {
                summary += std::format("   ORE {:.2f}", spec.minOreDensity);
            }
            hudText(summary, kRight - 0.44f, rowY + 0.020f, 0.028f,
                    armed ? hud::kText : hud::kTextDim);
            const sw::usize recipes =
                sw::factory::recipesForCategory(spec.category).size();
            if (recipes > 0)
            {
                hudText(std::format("{} RECIPES   SLOPE {:.2f}", recipes,
                                    spec.maxSlopeTangent),
                        kRight - 0.44f, rowY + 0.052f, 0.024f, hud::kTextDim);
            }

            m_hudButtons.push_back({kLeft + kPad, rowY, kRight - kPad, rowY + kRowHeight,
                                    400u + static_cast<sw::u32>(definition.id)});
            rowY += kRowHeight + kRowGap;
        }

        if (buildings.empty())
        {
            hudText("NO BUILDINGS IN THE CATALOG", kLeft + 0.04f, rowY + 0.02f, 0.034f,
                    hud::kBad);
            return;
        }

        // The footer: what is in your hand, and what it wants next.
        rowY += 0.012f;
        hudQuad(kLeft + kPad, rowY, kRight - kPad, bottom - 0.014f, hud::kHeader);
        if (const auto* held = sw::parts::findDefinition(m_heldBuilding);
            held != nullptr && sw::parts::isBuilding(*held))
        {
            const bool belt = held->building.category ==
                              sw::factory::BuildingCategory::Conveyor;
            const bool cable =
                held->building.category == sw::factory::BuildingCategory::Cable;
            hudText(std::format("ARMED   {}", hud::caps(held->name)), kLeft + 0.04f,
                    rowY + 0.014f, 0.036f, hud::kOk);
            hudText(belt    ? "PICK AN OUTPUT, THEN AN INPUT"
                    : cable ? "PICK TWO POWER NODES"
                            : "LOOK AT THE GROUND AND CLICK   WHEEL ROTATES",
                    kLeft + 0.04f, rowY + 0.058f, 0.028f, hud::kTextDim);
        }
        else
        {
            hudText("NOTHING ARMED", kLeft + 0.04f, rowY + 0.014f, 0.036f, hud::kTextDim);
            hudText("PICK A BUILDING ABOVE", kLeft + 0.04f, rowY + 0.058f, 0.028f,
                    hud::kTextDim);
        }
    }

    // ======================= THE MACHINE PANEL (E) =========================
    //
    // F3's premise is that a building is a GENERIC executor and its recipe is
    // a choice. Somewhere that choice has to be made, and the honest place is
    // standing in front of the machine: E, near it, and the panel is the
    // machine's own front plate — what it is doing, why it is not, what the
    // grid is giving it, what is in the bin, and the list of jobs its
    // category knows how to do.
    //
    // Everything on it is READ from the components, nothing is cached. A
    // panel that agreed with the simulation only at the moment it opened
    // would be worse than no panel.
    // ------------------------------------------------------------------------
    void StarWorksGame::toggleConfigMenu()
    {
        if (!m_configTarget.isNull())
        {
            m_configTarget = {};
            return;
        }
        // On foot, near a building, nothing else in the way.
        if (!m_evaMode || m_mapView || m_editorMode || m_buildMenu ||
            m_capsuleEntity.isNull())
        {
            return;
        }
        const auto* player = m_world.tryGetComponent<TransformComponent>(m_capsuleEntity);
        if (player == nullptr)
        {
            return;
        }

        // NEAR ENOUGH, AND IN FRONT OF YOU — asked of the machine's actual
        // HULL rather than of its centre. The old rule took the nearest
        // centre inside 18 m, which is wrong twice over: a 16 m solar field
        // you are standing on has a centre 8 m away and loses to a silo
        // behind your shoulder, and a machine you are touching can have its
        // centre out of range entirely. A ray from the eye against the
        // hitboxes answers the real question exactly, and reuses the very
        // boxes you cannot walk through.
        (void)player;
        const sw::ecs::Entity best = hullUnderCrosshair(kConfigRangeM);
        if (best.isNull())
        {
            SW_LOG_INFO("Game", "E: nothing in front of you within {:.0f} m",
                        kConfigRangeM);
            m_configTarget = {};
            return;
        }

        // A ROCKET IS SOMETHING YOU LOOK AT AND PRESS E ON, exactly like a
        // machine. `P` still cycles between vessels once you are flying, but
        // getting IN should not be a key that means something else — you are
        // standing in front of the thing, which is the whole gesture.
        if (const auto* part = m_world.tryGetComponent<sw::parts::PartComponent>(best))
        {
            const sw::ecs::Entity vessel = part->vessel;
            if (!vessel.isNull() && m_world.isAlive(vessel) &&
                m_world.hasComponent<ShipComponent>(vessel))
            {
                m_shipEntity = vessel;
                m_evaMode = false;
                m_sasMode = 0;
                m_configTarget = {};
                SW_LOG_INFO("Game", "Boarded vessel {}", vessel.index);
                return;
            }
            m_configTarget = {}; // a part of something that is not a craft
            return;
        }
        m_configTarget = best;
    }

    void StarWorksGame::collectConfigMenu()
    {
        m_hudButtons.clear();

        const auto* building =
            m_world.tryGetComponent<sw::factory::BuildingComponent>(m_configTarget);
        const auto* definition =
            (building != nullptr) ? sw::parts::findDefinition(building->definitionId)
                                  : nullptr;
        if (building == nullptr || definition == nullptr)
        {
            m_configTarget = {}; // demolished under our feet
            return;
        }
        auto* state =
            m_world.tryGetComponent<sw::factory::RecipeStateComponent>(m_configTarget);
        auto* power = m_world.tryGetComponent<sw::factory::PowerComponent>(m_configTarget);
        const auto* inventory =
            m_world.tryGetComponent<sw::factory::InventoryComponent>(m_configTarget);

        const std::vector<sw::u32> recipes =
            sw::factory::recipesForCategory(building->category);

        // THE VAB'S PANEL IS THIS PANEL. An assembly hall runs no recipes;
        // what it offers instead is the list of designs on disk, priced.
        // Everything else — the state tab, the grid figures, the bin, the
        // priority control — is the same machine panel it always was, which
        // is the whole reason the VAB needed no screen of its own.
        auto* assembly =
            m_world.tryGetComponent<sw::factory::AssemblyComponent>(m_configTarget);
        const bool hall = (assembly != nullptr);
        const std::span<const sw::parts::ShipBlueprint> designs =
            hall ? sw::parts::blueprintCatalog()
                 : std::span<const sw::parts::ShipBlueprint>{};
        // A panel is only a screen tall. Eight designs is already a taller
        // list than any other panel in the game; past that the list is
        // capped and SAYS it is capped, rather than growing off the bottom
        // of the window where the rows cannot be clicked.
        constexpr sw::usize kMaxDesignRows = 8;
        const sw::usize shownDesigns = std::min(designs.size(), kMaxDesignRows);
        const sw::usize listRows = hall ? shownDesigns : recipes.size();
        const bool listEmpty = (listRows == 0);

        constexpr sw::f32 kLeft = -0.68f;
        constexpr sw::f32 kRight = 0.68f;
        constexpr sw::f32 kTop = -0.62f;
        constexpr sw::f32 kRowHeight = 0.072f;
        constexpr sw::f32 kRowGap = 0.008f;
        constexpr sw::f32 kPad = 0.018f;
        constexpr sw::f32 kHeaderH = 0.086f;
        // A hall and a pad each carry one line no other machine has — what
        // is on the slipway, and whether the deck is clear — so their stats
        // block is taller by exactly that line. Leaving it at 0.168 drew
        // that line OUTSIDE its own background, which is the same class of
        // fault as the RECIPE label through the STOP row.
        const bool extraStatsLine =
            hall || building->category == sw::factory::BuildingCategory::Pad;
        const sw::f32 kStatsH = extraStatsLine ? 0.208f : 0.168f;
        constexpr sw::f32 kFooterH = 0.098f;

        sw::f32 cursorX = -2.0f;
        sw::f32 cursorY = -2.0f;
        const bool haveCursor = hudCursor(cursorX, cursorY);
        auto hovering = [&](sw::f32 y, sw::f32 x1) {
            return haveCursor && cursorX >= kLeft + kPad && cursorX <= x1 &&
                   cursorY >= y && cursorY <= y + kRowHeight;
        };

        const sw::f32 listTop = kTop + kHeaderH + kStatsH;
        // The list's own header eats `kLabelH` before the first row: at
        // 0.032 the word RECIPE was drawn straight through the STOP row
        // under it, because a label needs its own height AND the gap.
        constexpr sw::f32 kLabelH = 0.050f;
        // THE CATALOGUE COLUMN. A hall's panel is split: the list of saved
        // designs on the left, and on the right the one that is selected —
        // drawn, priced and weighed — with the button that actually builds
        // it. A name and a tonne figure is not enough to choose a rocket by.
        constexpr sw::f32 kCatalogueSplit = 0.10f;   // where the column starts
        constexpr sw::f32 kPreviewH = 0.40f;         // the 3D box
        constexpr sw::f32 kStatLine = 0.044f;
        // Includes the label strip the column starts below, which the list
        // also pays for — leaving it out put the PRODUCE button through the
        // footer, which a mock of the layout showed before the code ran.
        const sw::f32 columnHeight =
            kLabelH + kPreviewH + 0.014f + 5.0f * kStatLine + 0.014f + kRowHeight;

        const sw::f32 listHeight =
            listEmpty ? 0.06f
                      : (kLabelH +
                         static_cast<sw::f32>(listRows + 1) * (kRowHeight + kRowGap));
        const sw::f32 bodyHeight = hall ? std::max(listHeight, columnHeight) : listHeight;
        const sw::f32 bottom = listTop + bodyHeight + kFooterH;
        hudPanel(kLeft, kTop, kRight, bottom, hud::kPanel);

        // ---- HEADER: who this is -------------------------------------------
        hudQuad(kLeft, kTop, kRight, kTop + kHeaderH - 0.006f, hud::kHeader);
        hudQuad(kLeft, kTop, kLeft + 0.014f, kTop + kHeaderH - 0.006f,
                hud::categoryColor(building->category));
        hudText(hud::caps(definition->name), kLeft + 0.032f, kTop + 0.026f, 0.048f,
                hud::kTitle);
        hudText(hud::caps(std::string(sw::factory::categoryName(building->category))),
                kRight - 0.34f, kTop + 0.020f, 0.026f,
                hud::categoryColor(building->category));
        hudText("E CLOSE", kRight - 0.34f, kTop + 0.052f, 0.026f, hud::kTextDim);

        // ---- STATS: what it is doing, and on what power ---------------------
        sw::f32 rowY = kTop + kHeaderH + 0.006f;
        hudQuad(kLeft + kPad, rowY, kRight - kPad, rowY + kStatsH - 0.020f,
                hud::kRow);
        rowY += 0.016f;

        const char* stateText = "IDLE";
        sw::Vec4 stateColor = hud::kTextDim;
        // A hall reports its OWN state: it is not running a recipe, it is
        // paying for a rocket, and the two are never both true.
        const sw::u32 shownState = hall            ? assembly->state
                                   : (state != nullptr) ? state->state
                                                        : 0u;
        if (hall || state != nullptr)
        {
            switch (shownState)
            {
            case sw::factory::RecipeStateComponent::kRunning:
                stateText = "RUNNING";
                stateColor = hud::kOk;
                break;
            case sw::factory::RecipeStateComponent::kStarved:
                stateText = "STARVED";
                stateColor = hud::kWarn;
                break;
            case sw::factory::RecipeStateComponent::kBlocked:
                stateText = "BLOCKED";
                stateColor = hud::kWarn;
                break;
            case sw::factory::RecipeStateComponent::kNoPower:
                stateText = "NO POWER";
                stateColor = hud::kBad;
                break;
            default:
                break;
            }
        }
        // The state gets its own coloured tab: it is the one thing on this
        // panel you should be able to read from across the room.
        hudQuad(kLeft + kPad, rowY - 0.006f, kLeft + kPad + 0.008f, rowY + 0.040f,
                stateColor);
        hudText(stateText, kLeft + kPad + 0.020f, rowY, 0.042f, stateColor);

        if (power != nullptr)
        {
            const bool short_ = power->satisfaction < 0.999;
            hudText(std::format("SUPPLY {:.0f}%", power->satisfaction * 100.0),
                    kLeft + 0.32f, rowY + 0.004f, 0.034f,
                    short_ ? hud::kBad : hud::kOk);
            hudText(std::format("PRIORITY {}", power->priority), kRight - 0.30f,
                    rowY + 0.004f, 0.034f, hud::kText);
        }
        rowY += 0.050f;

        if (power != nullptr)
        {
            // Two labelled columns, so 0 reads as a zero rather than as the
            // word PASSIVE (which belongs on a catalogue row showing ONE
            // number, not on a line that already says which side is which).
            hudText(std::format("THIS  MAKES {:.0f} KW  DRAWS {:.0f} KW",
                                power->actualProducedKw, power->consumedKw),
                    kLeft + kPad + 0.014f, rowY, 0.028f, hud::kTextDim);
            // The second column starts where the FIRST one can no longer
            // reach: a 180 kW draw is three digits, and at 0.32 the two
            // sentences were printed through each other.
            hudText(std::format("GRID {}  MAKES {:.0f} KW  DRAWS {:.0f} KW",
                                power->gridId, power->gridProducedKw,
                                power->gridConsumedKw),
                    kLeft + 0.62f, rowY, 0.028f,
                    (power->gridProducedKw + 1.0e-9 < power->gridConsumedKw)
                        ? hud::kWarn
                        : hud::kTextDim);
        }
        rowY += 0.034f;

        if (inventory != nullptr)
        {
            std::string stock;
            for (const sw::factory::InventorySlot& slot : inventory->slots)
            {
                if (slot.resource == sw::res::Resource::Count || slot.units <= 0.0)
                {
                    continue;
                }
                if (!stock.empty()) { stock += "   "; }
                stock += std::format("{} {:.0f}",
                                     hud::caps(std::string(
                                         sw::res::definition(slot.resource).name)),
                                     slot.units);
            }
            const sw::f64 used = sw::factory::inventoryVolume(*inventory);
            hudText(stock.empty()
                        ? std::format("BIN EMPTY   0 / {:.0f} M3",
                                      inventory->volumeCapacityM3)
                        : std::format("{}   {:.1f} / {:.0f} M3", stock, used,
                                      inventory->volumeCapacityM3),
                    kLeft + kPad + 0.014f, rowY, 0.028f, hud::kText);
        }

        // ---- THE ORDER, at a hall --------------------------------------------
        // The one line that is not on any other machine's panel: what is on
        // the slipway and how much of its metal has arrived. It goes where
        // the recipe's throughput would be, because it is the same question.
        if (hall)
        {
            rowY += 0.032f;
            if (assembly->blueprint[0] == '\0')
            {
                hudText("NO ORDER   PICK A DESIGN BELOW", kLeft + kPad + 0.014f, rowY,
                        0.028f, hud::kTextDim);
            }
            else
            {
                const sw::f64 progress = sw::factory::assemblyProgress(*assembly);
                hudText(std::format("BUILDING {}   {:.0f}%   IRON {:.0f}/{:.0f}   "
                                    "COPPER {:.0f}/{:.0f}   DONE {}",
                                    hud::caps(std::string(assembly->blueprint)),
                                    progress * 100.0, assembly->ironPaidKg,
                                    assembly->ironNeededKg, assembly->copperPaidKg,
                                    assembly->copperNeededKg, assembly->completed),
                        kLeft + kPad + 0.014f, rowY, 0.028f, hud::kText);
            }
        }

        // ---- THE DECK, at a pad ---------------------------------------------
        // The one thing a pad can tell you that no other building can: why
        // the crate sitting in its bin has not become a rocket.
        if (building->category == sw::factory::BuildingCategory::Pad)
        {
            rowY += 0.032f;
            const sw::f64 crates =
                (inventory != nullptr)
                    ? sw::factory::inventoryCount(*inventory, sw::res::Resource::Vehicle)
                    : 0.0;
            const bool occupied = padIsOccupied(m_configTarget);
            hudText(occupied ? std::format("DECK OCCUPIED   {:.0f} WAITING   LAUNCH OR "
                                           "MOVE THE VESSEL",
                                           crates)
                    : (crates >= 1.0)
                        ? std::format("DECK CLEAR   {:.0f} WAITING   ROLLING OUT", crates)
                        : std::string("DECK CLEAR   NOTHING WAITING"),
                    kLeft + kPad + 0.014f, rowY, 0.028f,
                    occupied ? hud::kWarn : hud::kOk);
        }

        // ---- THE JOB LIST ----------------------------------------------------
        rowY = listTop;
        if (listEmpty)
        {
            hudText(hall ? "NO DESIGNS SAVED   BUILD ONE IN THE HANGAR (B)"
                         : "THIS BUILDING RUNS NO RECIPES",
                    kLeft + kPad + 0.014f, rowY + 0.030f, 0.032f, hud::kTextDim);
        }
        else if (hall)
        {
            hudText(shownDesigns < designs.size()
                        ? std::format("DESIGN   SHOWING {} OF {}", shownDesigns,
                                      designs.size())
                        : std::string("DESIGN"),
                    kLeft + kPad + 0.004f, rowY + 0.006f, 0.026f, hud::kTextDim);
            rowY += kLabelH;

            const std::string_view current{assembly->blueprint};

            // CLEAR, first, for the same reason STOP is: a hall with no order
            // draws its idle load and nothing else, and on a battery night
            // that is a choice worth having.
            const sw::f32 listRight = kCatalogueSplit - 0.010f;
            {
                const bool selected = current.empty();
                const bool hot = hovering(rowY, listRight);
                hudQuad(kLeft + kPad, rowY, listRight, rowY + kRowHeight,
                        selected ? hud::kRowStop
                                 : (hot ? hud::kRowHover : hud::kRow));
                hudText("CLEAR ORDER", kLeft + kPad + 0.018f, rowY + 0.018f, 0.034f,
                        selected ? hud::kTitle : hud::kText);
                m_hudButtons.push_back(
                    {kLeft + kPad, rowY, listRight, rowY + kRowHeight, 899u});
                rowY += kRowHeight + kRowGap;
            }

            for (sw::usize i = 0; i < shownDesigns; ++i)
            {
                const sw::parts::ShipBlueprint& design = designs[i];
                const sw::parts::BillOfMaterials bill =
                    sw::parts::blueprintCost(design);
                const bool buildable = sw::parts::blueprintIsBuildable(design);
                const bool building_ = (current == design.name);
                const bool picked = (m_vabSelection == static_cast<sw::i32>(i));
                const bool hot = hovering(rowY, listRight);
                // TWO DIFFERENT STATES, two different colours: the one being
                // BUILT (green) and the one you are LOOKING at (highlight).
                // They are usually the same row and must not be assumed to
                // be — reading a catalogue is not placing an order.
                hudQuad(kLeft + kPad, rowY, listRight, rowY + kRowHeight,
                        building_ ? (hot ? hud::kRowOnHover : hud::kRowOn)
                        : picked  ? hud::kRowHover
                        : hot     ? hud::kRowHover
                                  : ((i % 2 == 0) ? hud::kRow : hud::kRowAlt));
                if (picked && !building_)
                {
                    hudQuad(kLeft + kPad, rowY, kLeft + kPad + 0.010f, rowY + kRowHeight,
                            hud::kTitle);
                }
                hudText(hud::caps(design.name), kLeft + kPad + 0.022f, rowY + 0.010f,
                        0.032f, (building_ || picked) ? hud::kTitle : hud::kText);
                hudText(buildable ? std::format("{:.1f} T", bill.totalKg() / 1000.0)
                                  : "NO PARTS",
                        kLeft + kPad + 0.022f, rowY + 0.042f, 0.024f,
                        buildable ? hud::kTextDim : hud::kBad);
                m_hudButtons.push_back({kLeft + kPad, rowY, listRight,
                                        rowY + kRowHeight,
                                        900u + static_cast<sw::u32>(i)});
                rowY += kRowHeight + kRowGap;
            }

            // ---- THE CATALOGUE ENTRY: what you are about to build --------
            {
                if (m_vabSelection < 0 ||
                    m_vabSelection >= static_cast<sw::i32>(shownDesigns))
                {
                    m_vabSelection = shownDesigns > 0 ? 0 : -1;
                }
                const sw::f32 columnLeft = kCatalogueSplit;
                const sw::f32 columnRight = kRight - kPad;
                sw::f32 columnY = listTop + kLabelH;

                hudQuad(columnLeft, columnY, columnRight, columnY + kPreviewH,
                        sw::Vec4{0.06f, 0.10f, 0.15f, 0.98f});

                if (m_vabSelection >= 0)
                {
                    const sw::parts::ShipBlueprint& picked =
                        designs[static_cast<sw::usize>(m_vabSelection)];
                    // Wall clock, not simulation time: the model keeps
                    // turning while the game is paused, which is exactly when
                    // somebody is reading this panel.
                    hudDesignPreview(picked, columnLeft + 0.010f, columnY + 0.010f,
                                     columnRight - 0.010f, columnY + kPreviewH - 0.010f,
                                     static_cast<sw::f32>(clock().totalSeconds()) * 0.55f);
                    columnY += kPreviewH + 0.014f;

                    const sw::parts::BillOfMaterials bill =
                        sw::parts::blueprintCost(picked);
                    const bool buildable = sw::parts::blueprintIsBuildable(picked);
                    const sw::f64 iron =
                        (inventory != nullptr)
                            ? sw::factory::inventoryCount(*inventory,
                                                          sw::res::Resource::Iron)
                            : 0.0;
                    const sw::f64 copper =
                        (inventory != nullptr)
                            ? sw::factory::inventoryCount(*inventory,
                                                          sw::res::Resource::Copper)
                            : 0.0;

                    hudText(hud::caps(picked.name), columnLeft + 0.006f, columnY, 0.040f,
                            hud::kTitle);
                    columnY += kStatLine;
                    hudText(std::format("{} PARTS   {:.1f} T DRY", picked.parts.size(),
                                        bill.totalKg() / 1000.0),
                            columnLeft + 0.006f, columnY, 0.030f, hud::kText);
                    columnY += kStatLine;
                    // Each metal against what is IN THE BIN, because that is
                    // the question the player is actually asking.
                    hudText(std::format("IRON    {:.0f} KG   HAVE {:.0f}", bill.ironKg,
                                        iron),
                            columnLeft + 0.006f, columnY, 0.030f,
                            (iron >= bill.ironKg) ? hud::kOk : hud::kWarn);
                    columnY += kStatLine;
                    hudText(std::format("COPPER  {:.0f} KG   HAVE {:.0f}", bill.copperKg,
                                        copper),
                            columnLeft + 0.006f, columnY, 0.030f,
                            (copper >= bill.copperKg) ? hud::kOk : hud::kWarn);
                    columnY += kStatLine;
                    const sw::f64 seconds =
                        (assembly->buildRateKgPerSecond > 0.0)
                            ? bill.totalKg() / assembly->buildRateKgPerSecond
                            : 0.0;
                    hudText(std::format("BUILD   {:.0f} S AT FULL POWER", seconds),
                            columnLeft + 0.006f, columnY, 0.030f, hud::kTextDim);
                    columnY += kStatLine + 0.014f;

                    // ---- PRODUCE -------------------------------------------
                    const bool hot = haveCursor && cursorX >= columnLeft &&
                                     cursorX <= columnRight && cursorY >= columnY &&
                                     cursorY <= columnY + kRowHeight;
                    hudQuad(columnLeft, columnY, columnRight, columnY + kRowHeight,
                            !buildable ? sw::Vec4{0.12f, 0.14f, 0.18f, 0.95f}
                            : hot      ? hud::kRowOnHover
                                       : hud::kRowOn);
                    hudText(buildable ? "PRODUCE" : "PARTS MISSING",
                            columnLeft + 0.026f, columnY + 0.018f, 0.036f,
                            buildable ? hud::kTitle : hud::kBad);
                    if (buildable)
                    {
                        m_hudButtons.push_back({columnLeft, columnY, columnRight,
                                                columnY + kRowHeight, 898u});
                    }
                }
                else
                {
                    hudText("PICK A DESIGN", columnLeft + 0.020f, columnY + 0.18f, 0.034f,
                            hud::kTextDim);
                }
            }
        }
        else
        {
            hudText("RECIPE", kLeft + kPad + 0.004f, rowY + 0.006f, 0.026f,
                    hud::kTextDim);
            rowY += kLabelH;

            const sw::u32 current = (state != nullptr) ? state->recipeId : 0u;

            // STOP is a job like any other: an idle machine still pays its
            // idle draw, and on a battery night that is a choice worth having.
            {
                const bool selected = (sw::factory::findRecipe(current) == nullptr);
                const bool hot = hovering(rowY, kRight - kPad);
                hudQuad(kLeft + kPad, rowY, kRight - kPad, rowY + kRowHeight,
                        selected ? hud::kRowStop
                                 : (hot ? hud::kRowHover : hud::kRow));
                hudText("STOP", kLeft + kPad + 0.018f, rowY + 0.018f, 0.036f,
                        selected ? hud::kTitle : hud::kText);
                hudText("IDLE DRAW ONLY", kLeft + 0.30f, rowY + 0.022f, 0.026f,
                        hud::kTextDim);
                m_hudButtons.push_back(
                    {kLeft + kPad, rowY, kRight - kPad, rowY + kRowHeight, 600u});
                rowY += kRowHeight + kRowGap;
            }

            sw::usize index = 0;
            for (const sw::u32 id : recipes)
            {
                const sw::factory::RecipeDefinition* recipe = sw::factory::findRecipe(id);
                if (recipe == nullptr)
                {
                    continue;
                }
                const bool selected = (id == current);
                const bool hot = hovering(rowY, kRight - kPad);
                hudQuad(kLeft + kPad, rowY, kRight - kPad, rowY + kRowHeight,
                        selected ? (hot ? hud::kRowOnHover : hud::kRowOn)
                                 : (hot ? hud::kRowHover
                                        : ((index % 2 == 0) ? hud::kRow : hud::kRowAlt)));
                hudText(hud::caps(recipe->name), kLeft + kPad + 0.018f, rowY + 0.018f,
                        0.036f, selected ? hud::kTitle : hud::kText);

                // The recipe read as a sentence: what goes in, what comes out,
                // what it costs. The whole fuel chain is legible from these.
                std::string flow;
                for (const sw::factory::Ingredient& in : recipe->inputs)
                {
                    if (in.resource == sw::res::Resource::Count ||
                        in.unitsPerSecond <= 0.0)
                    {
                        continue;
                    }
                    if (!flow.empty()) { flow += " + "; }
                    flow += std::format("{:.2f} {}", in.unitsPerSecond,
                                        hud::caps(std::string(
                                            sw::res::definition(in.resource).name)));
                }
                if (flow.empty()) { flow = "GROUND"; }
                flow += " > ";
                bool firstOut = true;
                for (const sw::factory::Ingredient& out : recipe->outputs)
                {
                    if (out.resource == sw::res::Resource::Count ||
                        out.unitsPerSecond <= 0.0)
                    {
                        continue;
                    }
                    if (!firstOut) { flow += " + "; }
                    firstOut = false;
                    flow += std::format("{:.2f} {}", out.unitsPerSecond,
                                        hud::caps(std::string(
                                            sw::res::definition(out.resource).name)));
                }
                hudText(flow, kLeft + 0.26f, rowY + 0.024f, 0.026f,
                        selected ? hud::kText : hud::kTextDim);
                hudText(std::format("{:.0f} KW", recipe->powerKw), kRight - 0.13f,
                        rowY + 0.024f, 0.028f,
                        (recipe->powerKw >= 400.0) ? hud::kWarn : hud::kTextDim);

                m_hudButtons.push_back({kLeft + kPad, rowY, kRight - kPad,
                                        rowY + kRowHeight, 610u + id});
                rowY += kRowHeight + kRowGap;
                ++index;
            }
        }

        // ---- FOOTER: the priority control -----------------------------------
        if (power != nullptr)
        {
            rowY += 0.012f;
            const sw::f32 buttonRight = kLeft + 0.40f;
            const bool hot = hovering(rowY, buttonRight);
            hudQuad(kLeft + kPad, rowY, buttonRight, rowY + kRowHeight,
                    hot ? hud::kRowHover : hud::kRowAlt);
            hudText(std::format("GRID PRIORITY  {}", power->priority),
                    kLeft + kPad + 0.018f, rowY + 0.020f, 0.032f, hud::kText);
            m_hudButtons.push_back(
                {kLeft + kPad, rowY, buttonRight, rowY + kRowHeight, 601u});
            hudText("CLICK TO CYCLE. LOWER IS SERVED FIRST", kLeft + 0.43f,
                    rowY + 0.022f, 0.026f, hud::kTextDim);
        }
    }
} // namespace game
