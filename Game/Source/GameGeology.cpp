// ============================================================================
// GameGeology.cpp — F44: the geology screen.
//
// The survey (F43) put the ore on the map as one diamond per cell, coloured by
// whichever resource happened to WIN there. Measured on Terra, that display
// showed iron on 582 cells, copper on 586, and ice on ONE — because ice is
// almost never the richest thing under a place, and "the best of three" hides
// the two it did not pick. A player looking for the ice that makes fuel saw a
// planet with no ice on it.
//
// So this is not a bigger overlay. It is a different instrument, and the
// difference is the one every real resource map makes: you look at the IRON
// map, then at the COPPER map. One channel at a time, one hue per channel,
// and the brightness carries how much — which is the only encoding that can
// show a magnitude honestly, because a reader can order two brightnesses of
// one colour and cannot order two hues.
//
// It is a screen rather than a panel for the reason the design office is:
// what it shows is a WHOLE WORLD, and "where on this planet is the copper" has
// nothing to do with where the ship is pointing this second.
//
// And it ends in a BEACON, because a map that cannot be acted on is a
// picture. A click on the globe puts a marked place on the ground — anchored
// in the body's rotating frame, saved with the world, and drawn by the same
// reticle that finds the starting outpost from orbit. The chain the player
// asked for runs end to end: survey it, look at it, mark it, land on it.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"

#include <UI/GlobePick.hpp>
#include "Systems.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <format>
#include <vector>

namespace game
{
    namespace
    {
        struct Channel
        {
            sw::res::Resource resource;
            const char* label;  // the button
            const char* symbol; // the beacon's name
        };
        constexpr Channel kChannels[] = {
            {sw::res::Resource::IronOre, "FER", "FE"},
            {sw::res::Resource::CopperOre, "CUIVRE", "CU"},
            {sw::res::Resource::WaterIce, "GLACE", "H2O"},
        };
        constexpr sw::usize kChannelCount = std::size(kChannels);

        // Button ids. 1500-1999 belongs to this screen (UI/HudRoute.hpp).
        constexpr sw::u32 kIdClose = 1500;
        constexpr sw::u32 kIdChannel = 1501; // + channel index
        constexpr sw::u32 kIdBody = 1520;    // + row index

        /// The globe's mesh. 129 x 257 vertices: at the size the screen draws
        /// it, one quad is about three pixels, which is the point at which a
        /// per-vertex gradient stops being a gradient of quads.
        constexpr sw::u32 kGlobeRings = 128;
        constexpr sw::u32 kGlobeSegments = 256;

        /// The graticule, baked into the vertex colours rather than drawn.
        /// Thirty degrees is the spacing every planetary map uses, and it is
        /// what turns a flat-shaded disc into a sphere you can read a position
        /// off. Drawing it as geometry would have cost a thousand draw items.
        constexpr sw::f32 kGraticuleDegrees = 30.0f;
        constexpr sw::f32 kGraticuleWidthDeg = 0.45f;

        constexpr sw::f32 kPi = 3.14159265358979f;

        [[nodiscard]] sw::usize channelIndex(sw::res::Resource resource)
        {
            for (sw::usize i = 0; i < kChannelCount; ++i)
            {
                if (kChannels[i].resource == resource) { return i; }
            }
            return 0;
        }

        /// Ground colour where nobody has looked yet.
        ///
        /// NOT a step at the bottom of the ramp, and that is the whole reason
        /// this function exists: "no data" and "almost none" are different
        /// facts, and a reader cannot tell a dark ramp step from an unknown
        /// one. So unsurveyed ground keeps its RELIEF, desaturated to a grey,
        /// and the ramp colours are then the only saturated things on the
        /// globe. The boundary of the survey draws itself.
        [[nodiscard]] sw::Vec3 unknownGroundColor(const sw::planet::TerrainComponent* terrain,
                                                  sw::f32 signedElevation)
        {
            const sw::f32 amplitude =
                (terrain != nullptr) ? std::max(terrain->amplitude, 1.0f) : 1.0f;
            const sw::f32 height = glm::clamp(signedElevation / amplitude, 0.0f, 1.0f);
            return glm::mix(sw::Vec3{0.19f, 0.20f, 0.19f},
                            sw::Vec3{0.34f, 0.34f, 0.33f}, height);
        }

        /// Water nobody has surveyed. Surveyed water still gets the ramp —
        /// see the coastline note in rebuildGeologyGlobe.
        constexpr sw::Vec3 kSeaColor{0.07f, 0.10f, 0.15f};
    } // namespace

    // ------------------------------------------------------------------------
    // Opening and closing
    // ------------------------------------------------------------------------
    void StarWorksGame::enterGeology()
    {
        m_geologyMode = true;
        m_pausedBeforeGeology = m_simulation.isPaused();
        m_simulation.setPaused(true);
        const std::vector<sw::ecs::Entity> bodies = geologyBodies();
        // Open on the body under the ship if it is one of them, because the
        // planet you are orbiting is the one you were just surveying.
        m_geologyBody = bodies.empty() ? sw::ecs::Entity{} : bodies.front();
        if (const sw::i32 primary = controlledPrimaryIndex(); primary >= 0)
        {
            const sw::ecs::Entity here =
                m_celestialIndex.body(static_cast<sw::usize>(primary)).entity;
            if (std::find(bodies.begin(), bodies.end(), here) != bodies.end())
            {
                m_geologyBody = here;
            }
        }
        m_geologyYaw = 0.6f;
        m_geologyPitch = 0.25f;
        m_geologyDistance = 2.3f;
        m_geologyGlobeDirty = true;
        SW_LOG_INFO("Game", "GEOLOGY: open ({} surveyed bod{})", bodies.size(),
                    bodies.size() == 1 ? "y" : "ies");
    }

    void StarWorksGame::exitGeology()
    {
        m_geologyMode = false;
        m_simulation.setPaused(m_pausedBeforeGeology);
        SW_LOG_INFO("Game", "GEOLOGY: closed");
    }

    // ------------------------------------------------------------------------
    // WHAT A SATELLITE HAS STARTED TO LOOK AT
    //
    // The gate is coverage, not geology: a body carries a SurveyComponent only
    // once an armed instrument has passed over it, so the list IS the record of
    // where the player has flown one. Nothing that has never been looked at
    // appears — which is the point of having built the instrument.
    // ------------------------------------------------------------------------
    std::vector<sw::ecs::Entity> StarWorksGame::geologyBodies() const
    {
        struct Row
        {
            sw::ecs::Entity entity;
            sw::f64 distance;
        };
        std::vector<Row> rows;
        auto& world = const_cast<sw::ecs::World&>(m_world);
        const sw::WorldVec3 here =
            world.getComponent<TransformComponent>(controlledEntity()).position;
        world.forEach<TransformComponent, sw::planet::DepositComponent,
                      sw::planet::SurveyComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                sw::planet::DepositComponent&, sw::planet::SurveyComponent& survey) {
                if (sw::planet::surveyFraction(survey) <= 0.0f)
                {
                    return;
                }
                rows.push_back({entity, glm::length(transform.position - here)});
            });
        // NEAREST FIRST. A list of worlds ordered by anything else is a list
        // the player has to read; ordered by distance, the one they are
        // standing on or orbiting is the first row.
        std::sort(rows.begin(), rows.end(),
                  [](const Row& a, const Row& b) { return a.distance < b.distance; });
        std::vector<sw::ecs::Entity> bodies;
        bodies.reserve(rows.size());
        for (const Row& row : rows) { bodies.push_back(row.entity); }
        return bodies;
    }

    // ------------------------------------------------------------------------
    // The globe
    // ------------------------------------------------------------------------
    void StarWorksGame::rebuildGeologyGlobe()
    {
        m_geologyGlobeDirty = false;
        if (m_geologyBody.isNull() || !m_world.isAlive(m_geologyBody))
        {
            return;
        }
        const auto* deposits =
            m_world.tryGetComponent<sw::planet::DepositComponent>(m_geologyBody);
        const auto* survey =
            m_world.tryGetComponent<sw::planet::SurveyComponent>(m_geologyBody);
        const auto* terrain =
            m_world.tryGetComponent<sw::planet::TerrainComponent>(m_geologyBody);
        if (deposits == nullptr || survey == nullptr)
        {
            return;
        }
        const sw::f64 started = clock().totalSeconds();
        sw::MeshData sphere = sw::PrimitiveFactory::makeUvSphere(
            1.0f, kGlobeRings, kGlobeSegments, {1.0f, 1.0f, 1.0f, 1.0f});
        // Elevation first, for every vertex, because the COASTLINE needs a
        // vertex's neighbours and not just itself.
        std::vector<sw::f32> elevations(sphere.vertices.size(), 1.0f);
        if (terrain != nullptr)
        {
            for (sw::usize i = 0; i < sphere.vertices.size(); ++i)
            {
                elevations[i] = sw::planet::terrainElevationSignedLod(
                    *terrain, glm::normalize(sphere.vertices[i].position), 5);
            }
        }
        const sw::u32 stride = kGlobeSegments + 1;
        for (sw::usize i = 0; i < sphere.vertices.size(); ++i)
        {
            sw::Vertex& vertex = sphere.vertices[i];
            const sw::Vec3 direction = glm::normalize(vertex.position);
            const sw::f32 elevation = elevations[i];
            const bool known = sw::planet::surveyed(*survey, direction);
            // THE RAMP RUNS OVER WATER TOO, and it took a picture to settle
            // that. Painting only the land was defensible — a drill does not
            // go on a sea floor — and on Terra, which is seven tenths ocean,
            // it produced a survey that had plainly worked and a globe with
            // nothing on it. Worse, it broke the encoding: brightness meant
            // "how much ore" in some places and "is this land" in others.
            //
            // So the fill stays a pure function of the density everywhere,
            // and the thing the player actually needs from the geography —
            // can I put something down here — is drawn as a COASTLINE
            // instead. A contour adds a line; it does not overwrite a value.
            sw::Vec3 color =
                known ? sw::planet::oreRampColor(m_geologyChannel,
                                     sw::planet::oreDensity(*deposits, direction,
                                                            m_geologyChannel))
                      : ((elevation <= 0.0f) ? kSeaColor
                                             : unknownGroundColor(terrain, elevation));
            if (terrain != nullptr && known)
            {
                const sw::u32 row = static_cast<sw::u32>(i) / stride;
                const sw::u32 column = static_cast<sw::u32>(i) % stride;
                const bool land = elevation > 0.0f;
                auto differs = [&](sw::u32 r, sw::u32 c) {
                    if (r > kGlobeRings || c > kGlobeSegments) { return false; }
                    return (elevations[static_cast<sw::usize>(r) * stride + c] > 0.0f) !=
                           land;
                };
                if ((row > 0 && differs(row - 1, column)) ||
                    (column > 0 && differs(row, column - 1)) ||
                    differs(row + 1, column) || differs(row, column + 1))
                {
                    color *= 0.30f;
                }
            }
            // The graticule. Every thirty degrees, dimmed rather than drawn
            // over: a line that darkens the map cannot be mistaken for data,
            // and a sphere without one is a disc.
            const sw::f32 longitude =
                std::atan2(direction.z, direction.x) * 180.0f / kPi + 180.0f;
            const sw::f32 latitude =
                std::asin(glm::clamp(direction.y, -1.0f, 1.0f)) * 180.0f / kPi + 90.0f;
            auto onLine = [](sw::f32 degrees) {
                const sw::f32 offset =
                    std::abs(std::remainder(degrees, kGraticuleDegrees));
                return offset < kGraticuleWidthDeg;
            };
            if (onLine(longitude) || onLine(latitude))
            {
                color *= 0.55f;
            }
            vertex.color = {color.r, color.g, color.b, 1.0f};
        }
        // TWO SLOTS, SWAPPED. Replacing the mesh that was drawn last frame is
        // what used to force a full waitIdle; the terrain patch builder learnt
        // that once and this is the same lesson, not a new one.
        m_geologyGlobeSlot ^= 1u;
        m_meshes[m_geologyGlobeMesh[m_geologyGlobeSlot]] =
            renderer().createMesh(sphere);
        SW_LOG_INFO("Game", "GEOLOGY: globe rebuilt ({}) in {:.0f} ms",
                    kChannels[channelIndex(m_geologyChannel)].label,
                    (clock().totalSeconds() - started) * 1000.0);
    }

    // ------------------------------------------------------------------------
    // Input: orbit the globe, switch channel, drop a beacon
    // ------------------------------------------------------------------------
    void StarWorksGame::updateGeology()
    {
        if (keyPressed(sw::KeyCode::Escape))
        {
            exitGeology();
            return;
        }
        // Right-drag orbits and the wheel zooms, exactly as in the design
        // office. Left is the placing button on both screens.
        if (input().isMouseButtonDown(sw::MouseButton::Right))
        {
            m_geologyYaw -= input().mouseDeltaX() * 0.006f;
            m_geologyPitch = std::clamp(m_geologyPitch - input().mouseDeltaY() * 0.006f,
                                        -1.45f, 1.45f);
        }
        if (const sw::f32 scroll = input().scrollDeltaY(); scroll != 0.0f)
        {
            m_geologyDistance =
                std::clamp(m_geologyDistance * std::pow(1.12f, -scroll), 1.35f, 7.0f);
        }
        const sw::Vec3 offset = sw::ui::orbitCameraOffset(m_geologyYaw, m_geologyPitch,
                                                          m_geologyDistance);
        m_geologyCamera.setPosition(sw::WorldVec3(offset));
        const sw::Vec3 forward = glm::normalize(-offset);
        sw::Vec3 reference{0.0f, 1.0f, 0.0f};
        if (std::abs(glm::dot(forward, reference)) > 0.995f)
        {
            reference = sw::Vec3{1.0f, 0.0f, 0.0f};
        }
        const sw::Vec3 right = glm::normalize(glm::cross(forward, reference));
        const sw::Vec3 up = glm::cross(right, forward);
        m_geologyCamera.setOrientation(glm::quat_cast(sw::Mat3{right, up, -forward}));
        m_geologyCamera.setAspectRatio(renderer().aspectRatio());

        if (m_geologyGlobeDirty)
        {
            rebuildGeologyGlobe();
        }
    }

    bool StarWorksGame::geologyCursorDirection(sw::Vec3& outDirection)
    {
        sw::u32 width = 0;
        sw::u32 height = 0;
        window().framebufferSize(width, height);
        if (width == 0 || height == 0)
        {
            return false;
        }
        const sw::f32 ndcX = input().mouseX() / static_cast<sw::f32>(width) * 2.0f - 1.0f;
        const sw::f32 ndcY = input().mouseY() / static_cast<sw::f32>(height) * 2.0f - 1.0f;
        const sw::Mat4 inverse =
            glm::inverse(m_geologyCamera.viewProjectionCameraRelative());
        const sw::Vec4 nearPoint = inverse * sw::Vec4{ndcX, ndcY, 0.9f, 1.0f};
        const sw::Vec4 farPoint = inverse * sw::Vec4{ndcX, ndcY, 0.1f, 1.0f};
        const sw::Vec3 a = sw::Vec3(nearPoint) / nearPoint.w;
        const sw::Vec3 b = sw::Vec3(farPoint) / farPoint.w;
        // The globe is a unit sphere at the origin and the camera is the only
        // thing that moves, so the pick's answer IS the body-frame direction —
        // no frames left to get wrong.
        return sw::ui::pickUnitSphere(sw::Vec3(m_geologyCamera.position()),
                                      glm::normalize(b - a), outDirection);
    }

    // ------------------------------------------------------------------------
    // BEACONS: the map turned into a place
    //
    // Nothing here is new machinery, and that is deliberate. A beacon is a
    // TransformComponent, a SurfaceAnchorComponent and a BeaconComponent — the
    // three the starting outpost's own mast already carries — so it is placed
    // every tick by the anchor system in the body's rotating frame, drawn by
    // collectBeacons with its label and its distance in both the cockpit and
    // the map, and saved with the world without one line being added to the
    // save schema. The whole feature is a place to press.
    // ------------------------------------------------------------------------
    void StarWorksGame::geologyToggleBeacon(const sw::Vec3& bodyDirection)
    {
        if (m_geologyBody.isNull() || !m_world.isAlive(m_geologyBody))
        {
            return;
        }
        const auto* gravity =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(m_geologyBody);
        const auto* bodyTransform =
            m_world.tryGetComponent<TransformComponent>(m_geologyBody);
        if (gravity == nullptr || bodyTransform == nullptr)
        {
            return;
        }

        // A CLICK ON A BEACON REMOVES IT. Placing and clearing are the same
        // gesture because a map you can only add to fills up with the sites
        // you have already rejected.
        const sw::f32 kSameSpot = std::cos(0.035f); // ~220 km on Terra
        sw::ecs::Entity remove{};
        m_world.forEach<sw::phys::SurfaceAnchorComponent, sw::factory::BeaconComponent>(
            [&](sw::ecs::Entity entity, sw::phys::SurfaceAnchorComponent& anchor,
                sw::factory::BeaconComponent&) {
                if (!remove.isNull() || anchor.body != m_geologyBody)
                {
                    return;
                }
                const sw::f64 length = glm::length(anchor.localPosition);
                if (!(length > 0.0))
                {
                    return;
                }
                const sw::Vec3 direction = sw::Vec3(anchor.localPosition / length);
                if (glm::dot(direction, bodyDirection) >= kSameSpot)
                {
                    remove = entity;
                }
            });
        if (!remove.isNull())
        {
            m_world.destroyEntity(remove);
            SW_LOG_INFO("Game", "GEOLOGY: beacon cleared");
            return;
        }

        const auto* terrain =
            m_world.tryGetComponent<sw::planet::TerrainComponent>(m_geologyBody);
        const sw::f64 elevation =
            (terrain != nullptr) ? sw::planet::terrainElevation(*terrain, bodyDirection)
                                 : 0.0;
        const sw::WorldVec3 localPosition =
            sw::WorldVec3(glm::dvec3(bodyDirection) * (gravity->bodyRadius + elevation));
        const glm::dquat bodyRotation = sw::phys::spinRotation(*gravity);

        const sw::ecs::Entity entity = m_world.createEntity();
        TransformComponent transform{};
        transform.position = bodyTransform->position + bodyRotation * localPosition;
        m_world.addComponent(entity, transform);
        m_world.addComponent(entity,
                             PreviousTransformComponent{transform.position,
                                                        transform.rotation});
        sw::phys::SurfaceAnchorComponent anchor{};
        anchor.body = m_geologyBody;
        anchor.localPosition = localPosition;
        m_world.addComponent(entity, anchor);

        sw::factory::BeaconComponent beacon{};
        // THE LABEL IS THE READING. A waypoint called WAYPOINT 3 tells you
        // nothing when you find it again four flights later; "CU 0.74" is the
        // reason it was dropped.
        const auto* deposits =
            m_world.tryGetComponent<sw::planet::DepositComponent>(m_geologyBody);
        const sw::f32 density =
            (deposits != nullptr)
                ? sw::planet::oreDensity(*deposits, bodyDirection, m_geologyChannel)
                : 0.0f;
        const auto* survey =
            m_world.tryGetComponent<sw::planet::SurveyComponent>(m_geologyBody);
        // A MARK ON GROUND NOBODY HAS LOOKED AT IS STILL A USEFUL MARK — it is
        // just not a reading. Calling it "FE 0.00" would be the display
        // inventing a measurement out of the absence of one, which is the
        // exact failure this whole screen exists to undo.
        if (survey != nullptr && sw::planet::surveyed(*survey, bodyDirection))
        {
            std::snprintf(beacon.label, sizeof(beacon.label), "%s %.2f",
                          kChannels[channelIndex(m_geologyChannel)].symbol,
                          static_cast<double>(density));
        }
        else
        {
            std::snprintf(beacon.label, sizeof(beacon.label), "%s", "REPERE");
        }
        // Visible from anywhere a descent is planned from: the default range
        // is a thousand kilometres, which is below the orbit the survey was
        // flown from, so the mark you just placed would not be there when you
        // went back to the map to aim at it.
        beacon.rangeM = 2.0e7;
        beacon.nearRangeM = 60.0;
        m_world.addComponent(entity, beacon);
        SW_LOG_INFO("Game", "GEOLOGY: beacon '{}' placed", beacon.label);
    }

    // ------------------------------------------------------------------------
    // Drawing
    // ------------------------------------------------------------------------
    void StarWorksGame::collectGeologyItems()
    {
        m_drawItems.clear();
        if (m_geologyGlobeDirty)
        {
            rebuildGeologyGlobe();
        }
        if (!m_geologyBody.isNull() && m_world.isAlive(m_geologyBody))
        {
            const sw::Vec3 eye = sw::Vec3(m_geologyCamera.position());
            sw::DrawItem globe{};
            globe.mesh = &m_meshes[m_geologyGlobeMesh[m_geologyGlobeSlot]];
            globe.transform = glm::translate(sw::Mat4{1.0f}, -eye);
            globe.boundsCenter = -eye;
            globe.boundsRadius = 1.0f;
            // UNLIT. A lit globe hides half its data in its own night, and the
            // brightness on the lit half then means two things at once — how
            // much ore is there, and how square-on the ground is to a lamp.
            // The emissive path draws the vertex colour and nothing else,
            // which is what a map is.
            globe.tint = {1.0f, 1.0f, 1.0f, 2.0f};
            m_drawItems.push_back(globe);

            // The beacons already on this world, and the outpost's own mast
            // among them: a site that is already taken is information.
            m_world.forEach<sw::phys::SurfaceAnchorComponent,
                            sw::factory::BeaconComponent>(
                [&](sw::ecs::Entity, sw::phys::SurfaceAnchorComponent& anchor,
                    sw::factory::BeaconComponent&) {
                    if (anchor.body != m_geologyBody)
                    {
                        return;
                    }
                    const sw::f64 length = glm::length(anchor.localPosition);
                    if (!(length > 0.0))
                    {
                        return;
                    }
                    const sw::Vec3 direction = sw::Vec3(anchor.localPosition / length);
                    const sw::Vec3 relative = direction * 1.012f - eye;
                    sw::DrawItem mark{};
                    mark.mesh = &m_meshes[m_markerMeshIndex];
                    mark.transform = glm::translate(sw::Mat4{1.0f}, relative) *
                                     glm::scale(sw::Mat4{1.0f}, sw::Vec3{0.022f});
                    mark.boundsCenter = relative;
                    mark.boundsRadius = 0.022f;
                    mark.tint = {1.0f, 0.85f, 0.35f, 2.0f};
                    m_drawItems.push_back(mark);
                });
        }
        collectGeologyUi();
    }

    void StarWorksGame::collectGeologyUi()
    {
        hudBeginButtons(); // this screen renders through its own path

        constexpr sw::f32 kLeft = -0.98f;
        constexpr sw::f32 kListRight = -0.60f;
        constexpr sw::f32 kRowHeight = 0.082f;
        constexpr sw::f32 kRowGap = 0.008f;

        sw::f32 cursorX = 0.0f;
        sw::f32 cursorY = 0.0f;
        const bool haveCursor = hudCursor(cursorX, cursorY);
        auto hovering = [&](sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1) {
            return haveCursor && cursorX >= x0 && cursorX <= x1 && cursorY >= y0 &&
                   cursorY <= y1;
        };

        // ---- title ----------------------------------------------------------
        hudPanel(kLeft, -0.97f, 0.98f, -0.90f, hud::kHeader);
        hudText("GEOLOGIE  -  RELEVES ORBITAUX", kLeft + 0.02f, -0.955f, 0.040f,
                hud::kTitle);
        {
            const sw::f32 x0 = 0.86f;
            const bool hot = hovering(x0, -0.965f, 0.965f, -0.905f);
            hudQuad(x0, -0.965f, 0.965f, -0.905f, hot ? hud::kRowHover : hud::kRow);
            hudText("F4", x0 + 0.028f, -0.955f, 0.038f, hud::kText);
            m_hudButtons.push_back({x0, -0.965f, 0.965f, -0.905f, kIdClose});
        }

        // ---- the worlds a satellite has looked at ----------------------------
        const std::vector<sw::ecs::Entity> bodies = geologyBodies();
        hudPanel(kLeft, -0.87f, kListRight, -0.822f, hud::kHeader);
        hudText("MONDES RELEVES", kLeft + 0.02f, -0.862f, 0.030f, hud::kTextDim);
        sw::f32 rowY = -0.822f + kRowGap;
        for (sw::usize i = 0; i < bodies.size() && i < 12; ++i)
        {
            const sw::ecs::Entity entity = bodies[i];
            const bool picked = entity == m_geologyBody;
            const bool hot = hovering(kLeft, rowY, kListRight, rowY + kRowHeight);
            hudQuad(kLeft, rowY, kListRight, rowY + kRowHeight,
                    picked ? (hot ? hud::kRowOnHover : hud::kRowOn)
                    : hot  ? hud::kRowHover
                           : ((i % 2 == 0) ? hud::kRow : hud::kRowAlt));
            const sw::i32 index = m_celestialIndex.indexOf(entity);
            const char* name =
                (index >= 0) ? m_celestialIndex.body(static_cast<sw::usize>(index)).name
                             : "?";
            hudText(name, kLeft + 0.022f, rowY + 0.006f, 0.034f, hud::kText);
            if (const auto* survey =
                    m_world.tryGetComponent<sw::planet::SurveyComponent>(entity))
            {
                const sw::f32 percent = sw::planet::surveyFraction(*survey) * 100.0f;
                hudText(percent < 1.0f ? std::string("MOINS DE 1% RELEVE")
                                       : std::format("{:.0f}% RELEVE", percent),
                        kLeft + 0.022f, rowY + 0.048f, 0.026f, hud::kTextDim);
            }
            m_hudButtons.push_back({kLeft, rowY, kListRight, rowY + kRowHeight,
                                    kIdBody + static_cast<sw::u32>(i)});
            rowY += kRowHeight + kRowGap;
        }
        if (bodies.empty())
        {
            hudText("AUCUN. METTEZ UN OS-1 SUR UNE", kLeft + 0.022f, rowY + 0.01f,
                    0.028f, hud::kWarn);
            hudText("ORBITE STABLE ET ARMEZ-LE.", kLeft + 0.022f, rowY + 0.048f,
                    0.028f, hud::kWarn);
        }

        // ---- the channel: one resource at a time -----------------------------
        constexpr sw::f32 kChannelY = 0.80f;
        constexpr sw::f32 kChannelHeight = 0.070f;
        sw::f32 channelX = kLeft;
        for (sw::usize i = 0; i < kChannelCount; ++i)
        {
            const sw::f32 x1 = channelX + 0.215f;
            const bool picked = kChannels[i].resource == m_geologyChannel;
            const bool hot = hovering(channelX, kChannelY, x1, kChannelY + kChannelHeight);
            hudQuad(channelX, kChannelY, x1, kChannelY + kChannelHeight,
                    picked ? (hot ? hud::kRowOnHover : hud::kRowOn)
                    : hot  ? hud::kRowHover
                           : hud::kRow);
            // The swatch is the ramp's own top step, so the button and the
            // globe cannot disagree about what colour this channel is.
            const sw::Vec3 swatch = sw::planet::oreRampColor(kChannels[i].resource, 0.62f);
            hudQuad(channelX + 0.014f, kChannelY + 0.016f, channelX + 0.052f,
                    kChannelY + kChannelHeight - 0.016f,
                    {swatch.r, swatch.g, swatch.b, 1.0f});
            hudText(kChannels[i].label, channelX + 0.068f, kChannelY + 0.018f, 0.034f,
                    picked ? hud::kTitle : hud::kText);
            m_hudButtons.push_back({channelX, kChannelY, x1, kChannelY + kChannelHeight,
                                    kIdChannel + static_cast<sw::u32>(i)});
            channelX = x1 + 0.012f;
        }

        // ---- the legend, with numbers on it ----------------------------------
        //
        // A gradient with no scale is a mood. These are the same numbers the
        // mine multiplies its rate by, so the map and the pay slip agree.
        constexpr sw::f32 kLegendLeft = 0.34f;
        constexpr sw::f32 kLegendRight = 0.97f;
        constexpr sw::f32 kLegendTop = 0.800f;
        constexpr sw::f32 kLegendBottom = 0.836f;
        constexpr sw::u32 kLegendSteps = 48;
        for (sw::u32 i = 0; i < kLegendSteps; ++i)
        {
            const sw::f32 t0 = static_cast<sw::f32>(i) / static_cast<sw::f32>(kLegendSteps);
            const sw::f32 t1 =
                static_cast<sw::f32>(i + 1) / static_cast<sw::f32>(kLegendSteps);
            const sw::Vec3 color = sw::planet::oreRampColor(m_geologyChannel, (t0 + t1) * 0.5f);
            hudQuad(glm::mix(kLegendLeft, kLegendRight, t0), kLegendTop,
                    glm::mix(kLegendLeft, kLegendRight, t1) + 0.002f, kLegendBottom,
                    {color.r, color.g, color.b, 1.0f});
        }
        hudText("TENEUR   GRIS : NON RELEVE   TRAIT : LITTORAL", kLegendLeft, kLegendTop - 0.040f, 0.028f,
                hud::kTextDim);
        for (sw::u32 tick = 0; tick <= 4; ++tick)
        {
            const sw::f32 t = static_cast<sw::f32>(tick) / 4.0f;
            hudText(std::format("{:.2f}", t),
                    glm::mix(kLegendLeft, kLegendRight, t) - 0.028f,
                    kLegendBottom + 0.008f, 0.026f, hud::kTextDim);
        }

        // ---- what is under the cursor ----------------------------------------
        //
        // The three channels at once, in figures. The globe answers "where",
        // and this answers the question the globe cannot: how much, exactly,
        // of everything, at this one point.
        sw::Vec3 direction{};
        const bool onGlobe = geologyCursorDirection(direction);
        const auto* deposits =
            m_geologyBody.isNull()
                ? nullptr
                : m_world.tryGetComponent<sw::planet::DepositComponent>(m_geologyBody);
        const auto* survey =
            m_geologyBody.isNull()
                ? nullptr
                : m_world.tryGetComponent<sw::planet::SurveyComponent>(m_geologyBody);
        hudPanel(kLeft, 0.88f, 0.98f, 0.965f, hud::kPanel);
        if (onGlobe && deposits != nullptr && survey != nullptr)
        {
            const sw::f32 latitude =
                std::asin(glm::clamp(direction.y, -1.0f, 1.0f)) * 180.0f / kPi;
            const sw::f32 longitude = std::atan2(direction.z, direction.x) * 180.0f / kPi;
            const auto* terrain =
                m_world.tryGetComponent<sw::planet::TerrainComponent>(m_geologyBody);
            const bool sea =
                terrain != nullptr &&
                sw::planet::terrainElevationSignedLod(*terrain, direction, 5) <= 0.0f;
            if (sw::planet::surveyed(*survey, direction))
            {
                // ALL THREE, IN FIGURES. This is the line that answers the
                // question the globe cannot: a channel shows where the copper
                // is, and only a number says that this exact place also holds
                // the iron and the ice a base needs to stand on its own.
                std::string line = std::format("{:+.1f} LAT  {:+.1f} LON   ", latitude,
                                               longitude);
                for (const Channel& channel : kChannels)
                {
                    line += std::format(
                        "{} {:.2f}   ", channel.symbol,
                        sw::planet::oreDensity(*deposits, direction, channel.resource));
                }
                hudText(line, kLeft + 0.02f, 0.895f, 0.034f,
                        sea ? hud::kTextDim : hud::kText);
                hudText(sea ? "MER : RIEN A FORER ICI"
                            : "CLIC GAUCHE : POSER OU RETIRER UNE BALISE",
                        kLeft + 0.02f, 0.932f, 0.026f,
                        sea ? hud::kWarn : hud::kTextDim);
            }
            else
            {
                hudText(std::format("{:+.1f} LAT  {:+.1f} LON   NON RELEVE", latitude,
                                    longitude),
                        kLeft + 0.02f, 0.895f, 0.034f, hud::kWarn);
                hudText("AUCUN SATELLITE N'EST PASSE ICI", kLeft + 0.02f, 0.932f, 0.026f,
                        hud::kTextDim);
            }
        }
        else
        {
            hudText("CLIC DROIT : TOURNER   MOLETTE : ZOOM", kLeft + 0.02f, 0.895f,
                    0.034f, hud::kTextDim);
            hudText("SURVOLEZ LE GLOBE POUR LIRE LES TENEURS", kLeft + 0.02f, 0.932f,
                    0.026f, hud::kTextDim);
        }
    }
} // namespace game
