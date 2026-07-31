#include "StarWorksGame.hpp"

#include "Systems.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <format>

#include "GameInternal.hpp"

namespace game
{
    StarWorksGame::StarWorksGame(const sw::ApplicationConfig& config)
        : sw::Application(config)
        , m_cameraController(m_camera)
    {
        // Far planes sized for the full system: the Sun must render from
        // Mars (3.8e11 m away when opposed). Reverse-Z keeps the precision.
        m_camera.setPerspective(sw::math::toRadians(60.0f), 0.5f, 1.0e13f);
        // Shading tier (M26): HIGH on a real GPU, LOW under a software
        // rasterizer. It gates the per-fragment planet path's octave budget
        // and its terrain self-shadowing march.
        renderer().setQuality(config.renderQuality);
        m_mapCamera.setPerspective(sw::math::toRadians(60.0f), 1.0e5f, 2.0e12f);
        m_glyphMeshIndex.fill(0xFFFFFFFFu);

        // DATA-DRIVEN PARTS: the shipped .swpart files replace the built-in
        // fallback before any mesh or vessel is built. Part Studio edits
        // these same files.
        // NOTHING HEAVY HAPPENS HERE ANY MORE.
        //
        // All of it — four catalogues off disk, the whole scene, the meshes —
        // used to run inside this constructor, which meant the window existed
        // but drew nothing until it was done. A slow disk and a hang look
        // exactly alike from there, and the player has no way to tell them
        // apart. The work is now a PLAN, run one step per frame with a bar
        // over it, so the first frame appears immediately and every frame
        // after it says what is happening.
        //
        // The one exception is the glyph meshes, built right here: the bar
        // has to draw text on the very first frame, and it cannot wait for a
        // step that has not run yet.
        buildGlyphMeshes();
        buildBootPlan();

    }

    sw::u32 StarWorksGame::registerMesh(sw::Mesh mesh)
    {
        m_meshes.push_back(std::move(mesh));
        return static_cast<sw::u32>(m_meshes.size() - 1);
    }

    void StarWorksGame::buildGlyphMeshes()
    {
        for (const char* c = kGlyphCharset; *c != '\0'; ++c)
        {
            const sw::MeshData glyph = sw::ui::buildGlyphMesh(*c);
            if (!glyph.empty())
            {
                m_glyphMeshIndex[static_cast<sw::usize>(*c)] =
                    registerMesh(renderer().createMesh(glyph));
            }
        }
    }

    void StarWorksGame::buildNavballMeshes()
    {
        m_navRingMeshIndex =
            registerMesh(renderer().createMesh(buildNavRingMesh(48, 0.06f)));
        m_navLineMeshIndex = registerMesh(renderer().createMesh(buildNavBarMesh()));
        m_navDiamondMeshIndex =
            registerMesh(renderer().createMesh(buildNavDiamondMesh()));
    }

    sw::ecs::Entity StarWorksGame::controlledEntity() const
    {
        // ON FOOT IS THE NORMAL STATE. There may be no vessel at all — the
        // game now starts with none — so the suit is the fallback and not
        // the exception. Everything downstream (the HUD, the camera, the
        // terrain patch, the simulation bubble) dereferences this without
        // checking, which is only safe because it is never null once the
        // scene is built.
        if (m_evaMode && !m_capsuleEntity.isNull())
        {
            return m_capsuleEntity;
        }
        if (!m_shipEntity.isNull() && m_world.isAlive(m_shipEntity))
        {
            return m_shipEntity;
        }
        return m_capsuleEntity;
    }

    sw::WorldVec3 StarWorksGame::controlledVelocity() const
    {
        const sw::ecs::Entity entity = controlledEntity();
        auto& world = const_cast<sw::ecs::World&>(m_world);
        if (const auto* body =
                world.tryGetComponent<sw::phys::DynamicBodyComponent>(entity))
        {
            return body->velocity;
        }
        if (const auto* rails = world.tryGetComponent<sw::phys::OnRailsComponent>(entity))
        {
            sw::WorldVec3 position{};
            sw::WorldVec3 velocity{};
            sw::phys::kepler::evaluate(rails->orbit, m_physicsLane->presentSeconds(),
                                       position, &velocity);
            // The orbit is primary-relative: add the primary's own motion.
            if (const auto* primary = world.tryGetComponent<sw::phys::GravitySourceComponent>(
                    rails->primary))
            {
                velocity += primary->worldVelocity;
            }
            return velocity;
        }
        return {0.0, 0.0, 0.0};
    }

    StarWorksGame::~StarWorksGame()
    {
        // See the header: the terrain-patch job writes into members of THIS
        // object, and the pool that runs it belongs to the base class.
        threadPool().waitIdle();
    }

    void StarWorksGame::onUpdate(sw::f32 deltaSeconds)
    {
        // ---- the shell, before anything the game does ----------------------
        // Booting and the menu are not the game with a flag set; they are
        // states in which the game is NOT running, so they return early
        // rather than being threaded through every system below.
        if (m_shell == Shell::Booting)
        {
            updateBoot();
            return;
        }
        if (m_shell == Shell::Menu)
        {
            // The title screen renders the live world behind the menu, from
            // its own slowly-orbiting camera. Only without a session: a pause
            // menu keeps the player's frozen view — that IS their game.
            if (!m_hasSession)
            {
                updateMenuCamera(deltaSeconds);
            }
            updateTextField();
            handleHudClicks();
            // ESC backs out one level, and out of the menu entirely only if
            // there is a game to go back to.
            if (keyPressed(sw::KeyCode::Escape))
            {
                if (m_menuPage != MenuPage::Root) { openMenu(MenuPage::Root); }
                else if (m_hasSession) { continueGame(); }
            }
            return;
        }

        // The address field takes the keyboard while it has focus — including
        // ESC, which cancels it rather than quitting the game. Everything
        // below asks through keyPressed(), which is false while typing.
        updateTextField();

        // ESC OPENS THE MENU; it does not quit.
        //
        // It used to call requestClose() on the spot — no confirmation, no
        // chance to save, and the reflex that closes a dialog in every other
        // program threw away the session. In the hangar it still belongs to
        // the editor (drop / put back the held part).
        if (keyPressed(sw::KeyCode::Escape) && !m_editorMode)
        {
            openMenu(MenuPage::Root);
            return;
        }

        // --- mode toggles -------------------------------------------------------
        if (keyPressed(sw::KeyCode::M))
        {
            m_mapView = !m_mapView;
            SW_LOG_INFO("Game", "Star map {}", m_mapView ? "opened" : "closed");
        }
        if (keyPressed(sw::KeyCode::G) && !m_mapView)
        {
            toggleEva();
        }
        // F3, not a letter: N already creates a maneuver node on the map,
        // and every other letter in reach is a flight control. F2 shows the
        // hulls, F5/F9 save and load — the panel keys are a family.
        if (keyPressed(sw::KeyCode::F3))
        {
            m_netPanel = !m_netPanel;
            if (!m_netPanel)
            {
                m_netAddressFocused = false;
            }
        }
        if (keyPressed(sw::KeyCode::V))
        {
            m_speedSurfaceRelative = !m_speedSurfaceRelative;
        }
        if (keyPressed(sw::KeyCode::Tab))
        {
            m_shipMode = !m_shipMode;
            if (!m_shipMode)
            {
                const sw::Vec3 forward = m_camera.forward();
                const sw::f32 yaw = std::atan2(-forward.x, -forward.z);
                const sw::f32 pitch = std::asin(std::clamp(forward.y, -1.0f, 1.0f));
                m_cameraController.setPose(m_camera.position(), yaw, pitch);
            }
            SW_LOG_INFO("Game", "{}", m_shipMode ? "Chase camera" : "Free camera");
        }
        // SPACE is the ACTION key, not the pause key.
        //
        // Pausing is a warp rate — x0 — and it belongs on the warp control
        // with every other rate: press the warp-down key at x1 and time
        // stops. Space was doing the one thing a pilot's thumb should never
        // do by accident, on the key every game in the genre uses to stage.
        // On foot it jumps; in a rocket it fires the next decoupler.
        if (keyPressed(sw::KeyCode::Space) && !m_editorMode && !m_mapView)
        {
            if (m_evaMode)
            {
                m_jumpRequested = true; // consumed by updateShipControls
            }
            else
            {
                fireNextDecoupler();
            }
        }
        if (m_mapView)
        {
            updateManeuverNodeInput();
        }

        // ---- vessel editor (B) + staging (Z) -----------------------------------
        // THE HANGAR OPENS ON FOOT. It used to require a cockpit, which
        // made sense while the player began in one; now that on foot is the
        // normal state, refusing there would put the design tool behind a
        // vessel you can only obtain by using the design tool.
        if (keyPressed(sw::KeyCode::B) && !m_mapView)
        {
            if (m_editorMode) { exitEditor(); }
            else { enterEditor(); }
        }
        if (m_editorMode)
        {
            updateEditor();
        }
        // Z stays as the second name for the same action — the muscle
        // memory of everyone who has flown this build so far.
        // Staging belongs to whoever is IN the rocket. On foot it would fire
        // a decoupler on a vessel the player is not aboard and may not even
        // be able to see.
        if (keyPressed(sw::KeyCode::Z) && !m_editorMode && !m_mapView && !m_evaMode)
        {
            fireNextDecoupler();
        }

        // ---- docking: ports of different vessels, close and slow ---------------
        if (!m_editorMode && clock().totalSeconds() - m_lastDockCheckSeconds > 0.5)
        {
            m_lastDockCheckSeconds = clock().totalSeconds();
            struct Port { sw::ecs::Entity part; sw::ecs::Entity vessel; sw::WorldVec3 pos; };
            std::vector<Port> ports;
            m_world.forEach<sw::parts::PartComponent, TransformComponent>(
                [&](sw::ecs::Entity entity, sw::parts::PartComponent& part,
                    TransformComponent& transform) {
                    const auto* definition =
                        sw::parts::findDefinition(part.definitionId);
                    if (definition != nullptr &&
                        definition->type == sw::parts::PartType::DockingPort)
                    {
                        ports.push_back({entity, part.vessel, transform.position});
                    }
                });
            for (sw::usize a = 0; a < ports.size(); ++a)
            {
                for (sw::usize b = a + 1; b < ports.size(); ++b)
                {
                    if (ports[a].vessel == ports[b].vessel ||
                        glm::length(ports[a].pos - ports[b].pos) > 4.0)
                    {
                        continue;
                    }
                    // The player's vessel absorbs the other one.
                    const bool shipIsA = ports[a].vessel == m_shipEntity;
                    if (sw::parts::dockVessels(m_world,
                                               shipIsA ? ports[a].part : ports[b].part,
                                               shipIsA ? ports[b].part : ports[a].part))
                    {
                        SW_LOG_INFO("Game", "DOCKING: vessels merged");
                    }
                }
            }
        }

        // ---- SAS: T cycles OFF -> SAS -> PGD -> RTG -> NODE; buttons too ------
        // NODE is skipped when there is no node: cycling onto a mode that
        // cannot point anywhere would look like the key had stopped working.
        if (keyPressed(sw::KeyCode::T))
        {
            const sw::u32 ring[5] = {SasComponent::kOff, SasComponent::kStability,
                                     SasComponent::kPrograde, SasComponent::kRetrograde,
                                     SasComponent::kNode};
            const sw::u32 count = m_nodeActive ? 5u : 4u;
            sw::u32 index = 0;
            for (sw::u32 i = 0; i < count; ++i)
            {
                if (ring[i] == m_sasMode)
                {
                    index = i;
                    break;
                }
            }
            m_sasMode = ring[(index + 1) % count];
        }
        if (keyPressed(sw::KeyCode::P) && !m_editorMode)
        {
            cyclePilotedVessel(); // fly any built vessel
        }
        // F opens the BUILDING catalogue. Not in the hangar, which is the
        // rocket editor and has its own palette.
        if (keyPressed(sw::KeyCode::F) && !m_editorMode)
        {
            m_buildMenu = !m_buildMenu;
            m_configTarget = {}; // one panel at a time
        }
        // F2 shows the collision hulls. Not a debug flag hidden behind a
        // rebuild: the hitboxes are authored by hand now, and an authoring
        // mistake and an engine mistake look identical until you can see
        // the boxes.
        if (keyPressed(sw::KeyCode::F2))
        {
            m_showHitboxes = !m_showHitboxes;
            SW_LOG_INFO("Game", "HITBOXES {}", m_showHitboxes ? "SHOWN" : "HIDDEN");
        }
        // E opens the MACHINE panel of whatever you are standing next to.
        // On foot only: E is also the ship's roll axis, and a pilot pressing
        // it means roll. (`m_evaMode` is checked inside.)
        if (keyPressed(sw::KeyCode::E) && m_evaMode && !m_editorMode)
        {
            // Closing needs nothing; OPENING casts a ray from the camera, so
            // it has to wait for this frame's camera — the same reason the
            // ground cursor aims last. Answering here would aim at where you
            // were looking one frame ago.
            if (!m_configTarget.isNull())
            {
                m_configTarget = {};
            }
            else
            {
                m_configRequested = true;
            }
        }
        handleHudClicks();
        if (auto* sas = m_world.tryGetComponent<SasComponent>(m_shipEntity))
        {
            if (m_sasMode == SasComponent::kNode && !m_nodeActive)
            {
                m_sasMode = SasComponent::kOff; // the node it held is gone
            }
            sas->mode = m_sasMode;
            // ONE TOGGLE, THREE INSTRUMENTS. The speed readout, the navball's
            // prograde marker and the autopilot all read the same flag, so
            // pressing V cannot leave the autopilot flying toward a direction
            // the marker is no longer drawing.
            sas->surfaceRelative = m_speedSurfaceRelative ? 1u : 0u;
            // The burn still to fly, handed down every frame. It is a WORLD
            // vector and it shrinks as the burn is flown, so the autopilot
            // keeps the nose on the part of the burn that is left rather
            // than on the direction the whole burn started in.
            sas->targetDirection = sw::Vec3(remainingBurnVector());
        }

        // --- save / load ----------------------------------------------------------
        if (input().wasKeyPressed(sw::KeyCode::F5))
        {
            try
            {
                saveGame();
            }
            catch (const sw::Exception& e)
            {
                SW_LOG_ERROR("Game", "Save failed: {}", e.message());
            }
        }
        if (input().wasKeyPressed(sw::KeyCode::F9))
        {
            try
            {
                loadGame();
            }
            catch (const sw::Exception& e)
            {
                SW_LOG_ERROR("Game", "Load failed: {}", e.message());
            }
        }

        // The gate the warp control reads. Computed here, once, so the HUD
        // and the control cannot disagree about whether this craft is in a
        // stable orbit — they run at opposite ends of the frame.
        refreshFlightState();
        updateWarp();

        // --- per-mode camera & controls ------------------------------------------
        if (m_mapView)
        {
            const sw::f32 scroll = input().scrollDeltaY();
            if (scroll != 0.0f)
            {
                m_mapHeightMeters = std::clamp(
                    m_mapHeightMeters * std::pow(1.3, static_cast<sw::f64>(-scroll)),
                    kMapMinHeight, kMapMaxHeight);
            }
            // Right-drag orbits the map camera around the focus (yaw +
            // tilt); default stays near top-down.
            if (input().isMouseButtonDown(sw::MouseButton::Right))
            {
                m_mapYaw -= input().mouseDeltaX() * 0.005f;
                m_mapPitch = std::clamp(m_mapPitch - input().mouseDeltaY() * 0.005f,
                                        0.12f, 1.53f);
            }

            // KSP-style framing: the map centers on the SOI primary of the
            // controlled craft — Terra view at LEO, Sol view once you leave
            // Terra's sphere of influence.
            sw::WorldVec3 mapCenter{0.0};
            if (const sw::i32 primaryIndex = controlledPrimaryIndex(); primaryIndex >= 0)
            {
                mapCenter = m_celestialIndex.positionAt(primaryIndex,
                                                        m_physicsLane->presentSeconds());
            }
            const sw::f64 cosPitch = std::cos(m_mapPitch);
            const sw::WorldVec3 offsetDir{cosPitch * std::sin(m_mapYaw),
                                          std::sin(m_mapPitch),
                                          cosPitch * std::cos(m_mapYaw)};
            const sw::WorldVec3 cameraPos = mapCenter + offsetDir * m_mapHeightMeters;
            m_mapCamera.setPosition(cameraPos);
            const sw::Vec3 forward = glm::normalize(sw::Vec3(mapCenter - cameraPos));
            const sw::Vec3 right =
                glm::normalize(glm::cross(forward, sw::Vec3{0, 1, 0}));
            const sw::Vec3 up = glm::cross(right, forward);
            m_mapCamera.setOrientation(glm::quat_cast(sw::Mat3{right, up, -forward}));
            m_mapCamera.setAspectRatio(renderer().aspectRatio());
        }
        else if (!m_shipMode)
        {
            m_cameraController.update(input(), window(), deltaSeconds);
        }
        m_camera.setAspectRatio(renderer().aspectRatio());

        updateShipControls();

        m_bubbleSystem->setFocus(
            m_world.getComponent<TransformComponent>(controlledEntity()).position);

        // The wind is a closed form in simulation time — handing the system
        // the clock rather than letting it read one is what keeps two runs
        // of the same launch identical, at any time scale.
        if (m_aerodynamics != nullptr)
        {
            m_aerodynamics->setTimeSeconds(m_physicsLane->presentSeconds());
        }

        const sw::u64 physicsTicksBefore = m_physicsLane->tickCount();
        m_simulation.advance(m_world, deltaSeconds, &threadPool());
        m_commands.playback(m_world);
        if (m_physicsLane->tickCount() != physicsTicksBefore)
        {
            // A tick ran, so whatever was latched has been acted on.
            m_jumpRequested = false;
        }
        // After the clock has moved: the session reports THIS player's new
        // instant, and whatever the timeline says is now due is released.
        updateNetwork(deltaSeconds);

        // Fresh hierarchy snapshot for the map, HUD and flight plan.
        m_celestialIndex.rebuild(m_world);
        refreshPrediction();
        updateTerrainPatch();
        updateGrassField();

        updateReentryEffects(deltaSeconds);
        // A crate of rocket that has arrived becomes a rocket. Once a frame
        // is plenty: the crate is not going anywhere, and unpacking it
        // creates entities, which no simulation lane is allowed to do.
        updateLaunchPads();

        if (m_shipMode && !m_mapView)
        {
            updateChaseCamera(deltaSeconds);
        }

        // THE GROUND CURSOR AIMS LAST, on purpose. It casts a ray from the
        // camera at the ground, so it needs THIS frame's camera and THIS
        // frame's interpolated planet — not last frame's, and not the raw
        // tick pose. Running it up with the key handling put a whole frame
        // between where you were looking and where the ghost landed.
        updateBuildCursor();
        // ...and the maneuver node's mouse grab, which is a pick against the
        // map exactly as it is being drawn this frame — and the target
        // pick, which is the same kind of question asked of the bodies.
        updateNodeDrag();
        updateTargetPick();
        // ...and E, for the same reason: it is a ray from THIS frame's eye.
        if (m_configRequested)
        {
            m_configRequested = false;
            toggleConfigMenu();
        }

        // --- periodic statistics ------------------------------------------------
        const sw::f64 now = clock().totalSeconds();
        if (now - m_lastStatsLogSeconds > 5.0)
        {
            m_lastStatsLogSeconds = now;
            const sw::RenderStats& stats = renderer().stats();
            SW_LOG_INFO("Game",
                        "render: {} submitted, {} culled, {} instances in {} draw calls | "
                        "sim: {} dynamic / {} rails, warp x{:g} ({:.0f} FPS, prepare "
                        "{:.1f} ms)",
                        stats.itemsSubmitted, stats.itemsCulled, stats.instancesDrawn,
                        stats.drawCalls, m_world.count<sw::phys::DynamicBodyComponent>(),
                        m_world.count<sw::phys::OnRailsComponent>(),
                        kWarpLadder[m_warpIndex], clock().smoothedFps(),
                        stats.cpuPrepareMs);
        }
    }

    sw::u32 StarWorksGame::selectLodLevel(sw::f64 distance, sw::f64 worldRadius) const
    {
        const sw::f64 clampedDistance = std::max(distance, worldRadius * 1.0001);
        const sw::f64 angularDiameter = 2.0 * std::atan(worldRadius / clampedDistance);
        const sw::f32 fraction =
            static_cast<sw::f32>(angularDiameter) / m_camera.verticalFov();

        for (sw::u32 level = 0; level < CelestialLodComponent::kLodLevels - 1; ++level)
        {
            if (fraction >= kLodScreenFractions[level])
            {
                return level;
            }
        }
        return CelestialLodComponent::kLodLevels - 1;
    }

    // ========================================================================
    // MULTIPLAYER
    // ========================================================================

    bool StarWorksGame::keyPressed(sw::KeyCode key)
    {
        return !m_netAddressFocused && input().wasKeyPressed(key);
    }

    // ------------------------------------------------------------------------
    // A BODY'S POSE AS IT IS BEING DRAWN
    //
    // Anything positioned relative to a planet has to use the pose that
    // planet is RENDERED at, not the one it was last simulated at. The
    // difference is a physics step of orbital motion — 595 m for Terra — and
    // it resets every tick, so the symptom is always the same: a thing that
    // should be nailed to the ground swings hundreds of metres and snaps
    // back. It cost us the conveyor cargo once already; the build ghost is
    // the second time, so it is a function now.
    // ------------------------------------------------------------------------
    void StarWorksGame::bodyRenderPose(sw::ecs::Entity body, sw::WorldVec3& outPosition,
                                       glm::dquat& outRotation)
    {
        outPosition = sw::WorldVec3{0.0};
        outRotation = glm::dquat{1.0, 0.0, 0.0, 0.0};
        const auto* transform = m_world.tryGetComponent<TransformComponent>(body);
        if (transform == nullptr)
        {
            return;
        }
        const sw::f64 alpha = static_cast<sw::f64>(m_physicsLane->alpha());
        outPosition = transform->position;
        if (const auto* previous =
                m_world.tryGetComponent<PreviousTransformComponent>(body))
        {
            outPosition = glm::mix(previous->position, transform->position, alpha);
        }
        if (const auto* gravity =
                m_world.tryGetComponent<sw::phys::GravitySourceComponent>(body))
        {
            outRotation = sw::phys::spinRotationAt(*gravity, alpha);
        }
        else
        {
            outRotation = glm::dquat(transform->rotation);
        }
    }

    void StarWorksGame::onRender()
    {
        // BOOTING DRAWS ONLY THE BAR. There is no scene to render yet — the
        // step that builds it has not run — so this is not a choice about
        // taste, it is the only thing that can be drawn.
        if (m_shell == Shell::Booting)
        {
            m_drawItems.clear();
            m_hudButtons.clear();
            collectShellHud();
            renderer().renderFrame(m_camera, m_drawItems);
            return;
        }
        if (m_editorMode)
        {
            // Belt and braces: the hangar returns early, so if a menu were
            // ever opened from in here it would be invisible AND would have
            // frozen the update loop — a soft lock. Nothing opens one today
            // (ESC in the editor belongs to the held part), which is exactly
            // why this is worth two lines rather than a comment saying it
            // cannot happen.
            if (m_shell == Shell::Menu)
            {
                m_hudButtons.clear();
                collectShellHud();
            }
            // The hangar is its own little world: fixed light, no eclipse,
            // soft studio ambient (no fog).
            renderer().setSunPosition({60.0f, 90.0f, 40.0f});
            renderer().setShadowSpheres({});
            renderer().setAtmosphere({0.0f, 0.0f, 0.0f}, 0.0f,
                                     {0.055f, 0.06f, 0.07f});
            collectHangarItems();
            renderer().renderFrame(m_hangarCamera, m_drawItems);
            return;
        }
        // The title screen swaps in the menu's backdrop camera; everything
        // downstream (sun, shadow spheres, fog) reads `activeCamera` and
        // needs no other change. A pause menu keeps the player's own view.
        const bool menuBackdrop = m_shell == Shell::Menu && !m_hasSession;
        const sw::Camera& activeCamera =
            menuBackdrop ? m_menuCamera : (m_mapView ? m_mapCamera : m_camera);
        const sw::WorldVec3 cameraPosition = activeCamera.position();

        // Light comes from Sol's actual position (camera-relative), and
        // every celestial body except the star casts an analytic shadow —
        // no sunlight behind a planet.
        if (const auto* sol = m_world.tryGetComponent<TransformComponent>(m_solEntity))
        {
            renderer().setSunPosition(sw::Vec3(sol->position - cameraPosition));
        }
        std::array<sw::Renderer::ShadowSphere, sw::Renderer::kMaxShadowSpheres>
            occluders{};
        sw::u32 occluderCount = 0;
        for (sw::usize i = 0;
             i < m_celestialIndex.size() &&
             occluderCount < sw::Renderer::kMaxShadowSpheres;
             ++i)
        {
            const auto& body = m_celestialIndex.body(i);
            if (body.entity == m_solEntity)
            {
                continue;
            }
            if (const auto* bodyTransform =
                    m_world.tryGetComponent<TransformComponent>(body.entity))
            {
                occluders[occluderCount++] = {
                    sw::Vec3(bodyTransform->position - cameraPosition),
                    static_cast<sw::f32>(body.bodyRadius)};
            }
        }
        renderer().setShadowSpheres(
            std::span<const sw::Renderer::ShadowSphere>(occluders.data(),
                                                        occluderCount));

        // ---- AERIAL PERSPECTIVE (M21): fog + sky light from the camera's ------
        // position inside an atmosphere. Color tracks the local sun
        // elevation — blue at noon, amber at the terminator, black at night.
        sw::Vec3 fogColor{0.0f};
        sw::f32 fogDensity = 0.0f;
        sw::Vec3 skyAmbient{0.0f};
        m_skyDayFactor = 0.0f;

        // ---- M29: WHICH AIR are we looking through? ---------------------------
        // The nearest body that has an atmosphere, always — not just when the
        // camera is inside it. From orbit that same body supplies the limb;
        // from the ground, the sky and the aerial perspective. One integral,
        // three uses, so the descent never switches models.
        if (m_mapView)
        {
            // The star map is a schematic: no air, no haze over the orbits.
            renderer().setAtmosphereBody(sw::Vec3{0.0f}, 0.0f, 0);
        }
        else
        {
            sw::f32 bestRadius = 0.0f;
            sw::Vec3 bestCentre{0.0f};
            sw::i32 bestStyle = 0;
            sw::f64 bestDistance = std::numeric_limits<sw::f64>::max();
            for (sw::usize i = 0; i < m_celestialIndex.size(); ++i)
            {
                const auto& body = m_celestialIndex.body(i);
                if (m_world.tryGetComponent<sw::phys::AtmosphereComponent>(
                        body.entity) == nullptr)
                {
                    continue;
                }
                const auto* bodyTransform =
                    m_world.tryGetComponent<TransformComponent>(body.entity);
                if (bodyTransform == nullptr)
                {
                    continue;
                }
                const sw::f64 distance =
                    glm::length(bodyTransform->position - cameraPosition);
                if (distance >= bestDistance)
                {
                    continue;
                }
                bestDistance = distance;
                bestCentre = sw::Vec3(bodyTransform->position - cameraPosition);
                bestRadius = static_cast<sw::f32>(body.bodyRadius);
                bestStyle = 0;
                if (const auto* lod =
                        m_world.tryGetComponent<CelestialLodComponent>(body.entity);
                    lod != nullptr && lod->surfaceStyle >= 0)
                {
                    bestStyle = lod->surfaceStyle;
                }
            }
            renderer().setAtmosphereBody(bestCentre, bestRadius, bestStyle);
        }

        if (!m_mapView)
        {
            for (sw::usize i = 0; i < m_celestialIndex.size(); ++i)
            {
                const auto& body = m_celestialIndex.body(i);
                const auto* atmosphereComponent =
                    m_world.tryGetComponent<sw::phys::AtmosphereComponent>(body.entity);
                const auto* bodyTransform =
                    m_world.tryGetComponent<TransformComponent>(body.entity);
                if (atmosphereComponent == nullptr || bodyTransform == nullptr)
                {
                    continue;
                }
                const sw::WorldVec3 radial = cameraPosition - bodyTransform->position;
                const sw::f64 altitude = glm::length(radial) - body.bodyRadius;
                if (altitude > atmosphereComponent->topAltitude)
                {
                    continue;
                }
                const sw::f32 density = static_cast<sw::f32>(
                    std::exp(-std::max(altitude, 0.0) /
                             atmosphereComponent->scaleHeight));
                const sw::Vec3 up = sw::Vec3(glm::normalize(radial));
                sw::Vec3 sunDir{0.0f, 1.0f, 0.0f};
                if (const auto* sol =
                        m_world.tryGetComponent<TransformComponent>(m_solEntity))
                {
                    sunDir = sw::Vec3(glm::normalize(sol->position - cameraPosition));
                }
                const sw::f32 elevation = glm::dot(up, sunDir);
                const sw::f32 day =
                    glm::clamp((elevation + 0.06f) / 0.30f, 0.0f, 1.0f);
                const sw::Vec3 daySky{0.28f, 0.46f, 0.78f};
                const sw::Vec3 sunsetSky{0.82f, 0.40f, 0.16f};
                const sw::Vec3 nightSky{0.010f, 0.012f, 0.022f};
                sw::Vec3 sky = glm::mix(sunsetSky, daySky,
                                        glm::clamp((elevation - 0.03f) / 0.30f,
                                                   0.0f, 1.0f));
                sky = glm::mix(nightSky, sky, day);
                fogColor = sky;
                fogDensity = density * 1.2e-4f;
                skyAmbient = sky * (0.38f * density * day);
                m_skyDayFactor = day * density;
                break;
            }
        }
        renderer().setAtmosphere(fogColor, fogDensity, skyAmbient);
        // World clock for animated shading (cloud advection, waves). It is
        // the SIMULATION's clock, not the wall clock: the deck drifts over a
        // ground that now turns analytically with the same time, so warping
        // speeds the weather up exactly as it speeds the day up. Wrapped to
        // a day so f32 keeps millisecond resolution after a long session.
        renderer().setTimeSeconds(static_cast<sw::f32>(
            std::fmod(m_physicsLane->presentSeconds(), 86400.0)));

        collectDrawItems(activeCamera, menuBackdrop ? false : m_mapView);
        renderer().renderFrame(activeCamera, m_drawItems);
    }
} // namespace game
