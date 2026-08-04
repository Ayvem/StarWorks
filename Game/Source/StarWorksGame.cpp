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
        // THE FAR PLANE IS TWELVE LIGHT-YEARS NOW, and it had to be: at 1.0e13
        // metres — Neptune's orbit and a bit — every star in the catalogue was
        // BEHIND IT and clipped away, so flying to Sirius meant flying toward
        // an empty patch of sky that stayed empty until the moment of arrival.
        // Reverse-Z is what makes this free: depth is stored with the near
        // plane at 1.0 and the far at 0.0 in an f32 buffer, so precision is
        // set by the near plane and moving the far one out by four orders of
        // magnitude costs a fraction of a bit.
        m_camera.setPerspective(sw::math::toRadians(60.0f), 0.5f, 1.5e17f);
        parseDebugShot();
        // Shading tier (M26): HIGH on a real GPU, LOW under a software
        // rasterizer. It gates the per-fragment planet path's octave budget
        // and its terrain self-shadowing march.
        renderer().setQuality(config.renderQuality);
        // Far plane past the whole system as seen from anywhere in it: the
        // map's max height is 1.3e13 m (it has to frame Neptune at 4.5e12),
        // and a far plane below the zoom ceiling clips the entire scene into
        // an empty view the moment you zoom past it.
        m_mapCamera.setPerspective(sw::math::toRadians(60.0f), 1.0e5f, 4.0e17f);
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
        // A NEW FRAME'S BUTTON TABLE MAY BE OPENED AGAIN. handleHudClicks
        // below reads the table THIS frame's render has not built yet, which
        // is the previous frame's — a one-frame lag that has always been
        // there and is invisible at sixty hertz. What matters here is only
        // that the latch guarding "one clear per frame" is released on the
        // frame boundary, and this is it.
        m_hudButtonsOpen = false;
        // ...and the same boundary closes the previous frame's stopwatch. The
        // completed set is kept because the probe that reads it runs during
        // RENDER, by which time this frame's accumulators are half full.
        m_phaseLastMs = m_phaseMs;
        m_phaseMs = {};
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
        if (keyPressed(sw::KeyCode::Escape) && !m_editorMode && !m_geologyMode)
        {
            openMenu(MenuPage::Root);
            return;
        }

        // --- F44: THE GEOLOGY SCREEN --------------------------------------------
        //
        // F4 and not a letter, for the reason F3 is not one: every letter in
        // reach is a flight control, and F2 shows the hulls while F5/F9 save
        // and load — the panel keys are a family and this is one of them.
        //
        // It returns EARLY, which the hangar does not. The hangar leaves the
        // cockpit reachable underneath because it grew out of it; this screen
        // is a map of a planet and nothing on it should ever fire a decoupler.
        // One return is a smaller promise to keep than eleven `&& !m_geologyMode`.
        if (keyPressed(sw::KeyCode::F4) && !m_editorMode)
        {
            if (m_geologyMode) { exitGeology(); }
            else { enterGeology(); }
        }
        if (m_geologyMode)
        {
            updateGeology();
            handleHudClicks();
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
        if (keyPressed(sw::KeyCode::Tab) && m_mapView)
        {
            // IN THE MAP, Tab cycles the camera's focus body: AUTO (the
            // SOI primary, as always) -> Sol -> Terra -> Luna -> ... and
            // back to AUTO. Shift+Tab walks the other way. Framing the
            // TARGET while dragging a node is what makes an arrival
            // readable — the encounter markers sit on the body you are
            // looking at instead of a pixel three screens from home.
            const auto count = static_cast<sw::i32>(m_celestialIndex.size());
            if (count > 0)
            {
                const bool backwards = input().isKeyDown(sw::KeyCode::LeftShift) ||
                                       input().isKeyDown(sw::KeyCode::RightShift);
                // The cycle runs over [-1, count): -1 is AUTO.
                m_mapFocusIndex = backwards
                                      ? (m_mapFocusIndex < 0 ? count - 1
                                                             : m_mapFocusIndex - 1)
                                      : (m_mapFocusIndex + 1 >= count
                                             ? -1
                                             : m_mapFocusIndex + 1);
                // THE ZOOM MUST FOLLOW THE BODY, or the cycle only appears
                // to work for the small ones. A height tuned for Terra
                // (6e7 m) lands INSIDE Sol (radius 7e8) and inside Jupiter —
                // an empty or garbled view that reads as "Tab skipped it".
                // Arriving on a body reframes to at least four radii; going
                // back to a small one keeps your zoom (zooming in is a
                // choice, being inside a star is not).
                if (m_mapFocusIndex >= 0)
                {
                    const sw::f64 radius =
                        m_celestialIndex.body(static_cast<sw::usize>(m_mapFocusIndex))
                            .bodyRadius;
                    m_mapHeightMeters =
                        std::clamp(std::max(m_mapHeightMeters, radius * 4.0),
                                   kMapMinHeight, kMapMaxHeight);
                }
            }
        }
        else if (keyPressed(sw::KeyCode::Tab))
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

        // ---- docking: two faces that touch, look at each other, and are slow --
        if (!m_editorMode && !m_geologyMode)
        {
            updateDocking();
        }

        // ---- SAS: T cycles OFF -> SAS -> PGD -> RTG -> RAD/NML -> NODE -------
        // NODE is skipped when there is no node: cycling onto a mode that
        // cannot point anywhere would look like the key had stopped working.
        if (keyPressed(sw::KeyCode::T))
        {
            // ...and now through the orbit's other four directions as well.
            // NODE stays LAST so that skipping it when there is no node is
            // still a matter of shortening the ring rather than of leaving a
            // hole in the middle of it.
            const sw::u32 ring[9] = {SasComponent::kOff,       SasComponent::kStability,
                                     SasComponent::kPrograde,  SasComponent::kRetrograde,
                                     SasComponent::kRadialOut, SasComponent::kRadialIn,
                                     SasComponent::kNormal,    SasComponent::kAntiNormal,
                                     SasComponent::kNode};
            const sw::u32 count = m_nodeActive ? 9u : 8u;
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
                // The zoom floor is per-body: 1.5 radii of whatever the
                // camera is orbiting. The global constant alone (2e7 m) is
                // INSIDE Sol, Jupiter and Saturn — wheel in far enough over
                // a giant and the map showed its interior.
                sw::f64 minHeight = kMapMinHeight;
                if (m_mapFocusIndex >= 0 &&
                    m_mapFocusIndex < static_cast<sw::i32>(m_celestialIndex.size()))
                {
                    minHeight = std::max(
                        minHeight,
                        m_celestialIndex.body(static_cast<sw::usize>(m_mapFocusIndex))
                                .bodyRadius *
                            1.5);
                }
                else if (const sw::i32 primary = controlledPrimaryIndex(); primary >= 0)
                {
                    minHeight = std::max(
                        minHeight,
                        m_celestialIndex.body(static_cast<sw::usize>(primary))
                                .bodyRadius *
                            1.5);
                }
                m_mapHeightMeters = std::clamp(
                    m_mapHeightMeters * std::pow(1.3, static_cast<sw::f64>(-scroll)),
                    minHeight, kMapMaxHeight);
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
            // Terra's sphere of influence. Unless Tab picked a body: then
            // the camera rides THAT one, wherever the craft is.
            sw::WorldVec3 mapCenter{0.0};
            if (m_mapFocusIndex >= 0 &&
                m_mapFocusIndex < static_cast<sw::i32>(m_celestialIndex.size()))
            {
                mapCenter = m_celestialIndex.positionAt(
                    m_mapFocusIndex, m_physicsLane->presentSeconds());
            }
            else if (const sw::i32 primaryIndex = controlledPrimaryIndex();
                     primaryIndex >= 0)
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
        {
            PhaseTimer phase(m_phaseMs[kPhaseSimulation]);
            m_simulation.advance(m_world, deltaSeconds, &threadPool());
            m_commands.playback(m_world);
        }
        if (m_physicsLane->tickCount() != physicsTicksBefore)
        {
            // A tick ran, so whatever was latched has been acted on.
            m_jumpRequested = false;
        }
        // After the clock has moved: the session reports THIS player's new
        // instant, and whatever the timeline says is now due is released.
        updateNetwork(deltaSeconds);

        // Fresh hierarchy snapshot for the map, HUD and flight plan.
        {
            PhaseTimer phase(m_phaseMs[kPhaseCelestialIndex]);
            m_celestialIndex.rebuild(m_world);
        }
        // AFTER the playback and BEFORE anything reads a position this frame:
        // the origin may move here, and every cached absolute position in the
        // game is shifted with it (see rebaseOrigin).
        {
            PhaseTimer phase(m_phaseMs[kPhaseStreaming]);
            updateSystemStreaming();
        }
        updateThrottleAnimations();
        {
            PhaseTimer phase(m_phaseMs[kPhasePrediction]);
            refreshPrediction();
        }
        {
            PhaseTimer phase(m_phaseMs[kPhaseTerrain]);
            updateTerrainPatch();
        }
        // The instrument, after the flight state it reads and the celestial
        // index it looks the primary up in.
        updateSurvey();
        {
            PhaseTimer phase(m_phaseMs[kPhaseGrass]);
            updateGrassField();
        }
        {
            PhaseTimer phase(m_phaseMs[kPhaseReentry]);
            updateReentryEffects(deltaSeconds);
        }
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
            hudBeginButtons(); // the boot bar is its own screen
            collectShellHud();
            renderer().renderFrame(m_camera, m_drawItems);
            return;
        }
        if (m_geologyMode)
        {
            // The debug hooks live in applyDebugJump and this branch returns
            // before the frame ever reaches it — so a capture that opened this
            // screen could never press anything ON it. One-shot flags make the
            // second call free.
            applyDebugJump();
            // No star, no shadows, no air: the globe is drawn unlit and the
            // background is the black a map is read on, not a sky.
            renderer().setSunPosition({0.0f, 0.0f, 0.0f});
            renderer().setShadowSpheres({});
            renderer().setAtmosphere({0.0f, 0.0f, 0.0f}, 0.0f, {0.0f, 0.0f, 0.0f});
            collectGeologyItems();
            renderer().renderFrame(m_geologyCamera, m_drawItems);
            return;
        }
        if (m_editorMode)
        {
            // ...and the same for the hangar, for the same reason the geology
            // screen does it above: this branch returns before the frame
            // reaches the hooks, so a capture that opened the design office
            // could never press anything IN it — including a palette shelf,
            // whose button only exists once the panel has been collected once.
            applyDebugJump();
            // Belt and braces: the hangar returns early, so if a menu were
            // ever opened from in here it would be invisible AND would have
            // frozen the update loop — a soft lock. Nothing opens one today
            // (ESC in the editor belongs to the held part), which is exactly
            // why this is worth two lines rather than a comment saying it
            // cannot happen.
            if (m_shell == Shell::Menu)
            {
                hudSeizeButtons();
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
        // A capture run overrides the camera here, AFTER every system that
        // wanted a say in it and BEFORE anything reads it. No-op unless
        // SW_SHOT is set.
        applyDebugJump();
        applyDebugShot();
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
        // THE SUN IS WHICHEVER STAR IS BRIGHTEST FROM HERE. Between the
        // systems that is still Sol for the first two light-years and then it
        // is not, and every shader that says "sunlight" means this one.
        m_lightStar = dominantStar(cameraPosition);
        if (m_lightStar.isNull())
        {
            m_lightStar = m_solEntity;
        }
        if (const auto* sun = m_world.tryGetComponent<TransformComponent>(m_lightStar))
        {
            renderer().setSunPosition(sw::Vec3(sun->position - cameraPosition));
        }
        // THE EIGHT NEAREST BODIES, not the first eight in the index. There
        // are nineteen of them and the shader takes eight, and the index is
        // ordered by construction — Sol, Terra, Luna, Mars and its rocks,
        // Mercury, Venus, Jupiter — so the cut fell before Saturn, always.
        // The one body in the game with a RING SYSTEM was therefore the one
        // body that could not cast a shadow, and Saturn's rings ran straight
        // through its own shadow cone as if the planet were not there. That
        // shadow is in every photograph of the planet ever taken.
        //
        // Nineteen distances and a partial sort, once a frame.
        struct ScoredOccluder
        {
            sw::f64 distanceSquared;
            sw::Renderer::ShadowSphere sphere;
        };
        std::vector<ScoredOccluder> scored;
        scored.reserve(m_celestialIndex.size());
        for (sw::usize i = 0; i < m_celestialIndex.size(); ++i)
        {
            const auto& body = m_celestialIndex.body(i);
            // NO STAR CASTS A SHADOW, and it used to say `m_solEntity`.
            // With thirty-six stars in the world that let Proxima cast one on
            // Proxima b: the shadow cone's apex sits inside a source that is
            // also the light, so the test came out lit or unlit essentially at
            // random per pixel and the planet was a salt-and-pepper mess of
            // exactly two colours — its own day side and its own night side,
            // interleaved. It measured as fourteen times Mars's high-frequency
            // noise, which is how it was found rather than argued about.
            if (m_world.tryGetComponent<StarVisualComponent>(body.entity) != nullptr ||
                body.bodyRadius <= 0.0)
            {
                continue;
            }
            const auto* bodyTransform =
                m_world.tryGetComponent<TransformComponent>(body.entity);
            if (bodyTransform == nullptr)
            {
                continue;
            }
            const sw::WorldVec3 offset = bodyTransform->position - cameraPosition;
            // RANKED BY ANGULAR SIZE, not by distance: a shadow matters when
            // the thing casting it is big in the sky, and Luna at 400 000 km
            // beats Jupiter at five astronomical units on both counts. The
            // key is (distance / radius) squared, smallest first.
            const sw::f64 range = glm::length(offset);
            const sw::f64 key = (range * range) / (body.bodyRadius * body.bodyRadius);
            scored.push_back({key,
                              {sw::Vec3(offset), static_cast<sw::f32>(body.bodyRadius)}});
        }
        const sw::usize keep =
            std::min<sw::usize>(scored.size(), sw::Renderer::kMaxShadowSpheres);
        std::partial_sort(scored.begin(),
                          scored.begin() + static_cast<std::ptrdiff_t>(keep),
                          scored.end(),
                          [](const ScoredOccluder& a, const ScoredOccluder& b) {
                              return a.distanceSquared < b.distanceSquared;
                          });
        std::array<sw::Renderer::ShadowSphere, sw::Renderer::kMaxShadowSpheres>
            occluders{};
        const auto occluderCount = static_cast<sw::u32>(keep);
        for (sw::usize i = 0; i < keep; ++i)
        {
            occluders[i] = scored[i].sphere;
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
            // ...BUT NOT FROM ANOTHER PLANET. "The nearest body that has an
            // atmosphere, always" is right in spirit and wrong in f32: the
            // shader's ray-sphere test computes `dot(c, c) - r * r`, and from
            // Jupiter, Terra's centre is 7.8e11 m away. That squares to 6e23,
            // where a float's spacing is 7e16 — nine million times the 4e13
            // that `r * r` is trying to subtract. The discriminant is then
            // pure quantization noise WITH A RANDOM SIGN, so a fragment on
            // Jupiter would decide it was looking through Terra's air, and the
            // structured garbage that came back was a grid over the disc.
            //
            // Two hundred radii is past any air (Terra's ends at 1.05, the
            // limb is readable to perhaps 20) and keeps every squared term
            // inside four million radii squared, where f32 still has digits.
            // It also skips the whole march for every fragment in deep space,
            // which is most of them.
            constexpr sw::f64 kAirVisibleRadii = 200.0;
            if (bestRadius <= 0.0f || bestDistance > bestRadius * kAirVisibleRadii)
            {
                bestRadius = 0.0f;
                bestCentre = sw::Vec3{0.0f};
                bestStyle = 0;
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

        {
            PhaseTimer phase(m_phaseMs[kPhaseScene]);
            collectDrawItems(activeCamera, menuBackdrop ? false : m_mapView);
        }
        {
            PhaseTimer phase(m_phaseMs[kPhaseRender]);
            renderer().renderFrame(activeCamera, m_drawItems);
        }
    }
} // namespace game
