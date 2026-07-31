// ============================================================================
// GameShell.cpp — The shell around the game: boot plan, main menu, named save slots UI.
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

    // ------------------------------------------------------------------------
    // The shell: booting, the menu, and saves that have names
    // ------------------------------------------------------------------------

    void StarWorksGame::bootWireSystems()
    {
        // EVERYTHING HERE NEEDS THE SCENE TO EXIST, which is why it is a boot
        // step and not constructor code any more.
        //
        // Two ways it depended on the world, and only one of them announced
        // itself. The camera's parking spot reads Terra's transform, and with
        // the scene deferred that became getComponent on a null entity — a
        // loud assert on the first run, which is the good kind of failure.
        // The other was silent: PowerGridSystem is CONSTRUCTED with the Sol
        // entity, so building the lanes before the scene handed the whole
        // power grid a null star and every solar panel on every base would
        // have quietly produced nothing, for ever, with no error anywhere.
        // The assert is what made me look; the second one is what mattered.
        // The FREE camera's parking spot (Tab). It used to look at the
        // orbital station, which no longer exists; it now sits above the
        // outpost the player is standing in, so pressing Tab on the first
        // frame does not fly the view to empty space.
        {
            const sw::Vec3 siteUp = terraStartSite();
            const sw::WorldVec3 terraCentre =
                m_world.getComponent<TransformComponent>(m_terraEntity).position;
            m_cameraController.setPose(
                terraCentre + sw::WorldVec3(siteUp) * (kTerraRadius + 220.0), 0.0f,
                -0.35f);
        }

        // ---- simulation lanes ---------------------------------------------------
        m_physicsLane = m_simulation.findLane("Physics");
        m_physicsLane->setStrictCatchUp(true); // never desync live physics
        auto& physics = m_physicsLane->scheduler();
        // Celestials move FIRST: everything else this tick (gravity, rails,
        // anchors) reads their up-to-date positions/velocities.
        physics.addSystem(
            std::make_unique<sw::space::CelestialMotionSystem>(*m_physicsLane));
        // ...and they TURN first, for the same reason. The spin is analytic,
        // so it can be evaluated the instant the tick's present time is
        // known — and everything downstream that samples the ground reads
        // the body's rotating frame. Running it late (it used to sit after
        // the surface systems) left ground CONTACT sampling the heightfield
        // through a one-tick-stale attitude: 1.46e-6 rad, which on Terra is
        // 9.3 m of ground. Flat ground did not care. On slopes steeper than
        // 0.25 that offset is worth up to 13.6 m of elevation, which is
        // exactly how you walk into a mountainside.
        physics.addSystem(std::make_unique<CelestialSpinSystem>(*m_physicsLane));
        physics.addSystem(std::make_unique<SnapshotSystem>());
        // Parts -> vessel aggregates (mass falls as fuel burns).
        physics.addSystem(std::make_unique<sw::parts::VesselAssemblySystem>());
        // Rails ride at Physics rate too: their primaries move ~30 km/s, so
        // a 10 Hz refresh would visibly step (the closed-form solve is cheap).
        physics.addSystem(std::make_unique<sw::phys::RailsSystem>(*m_physicsLane));
        physics.addSystem(std::make_unique<sw::phys::GravityIntegrationSystem>());
        physics.addSystem(std::make_unique<SasSystem>()); // before Thrust: it commands
        // ROTATION IS INTEGRATED BEFORE THRUST, and the order is load-bearing.
        //
        // ThrustSystem used to do both itself, in one pass, in this order:
        // command the angular rate, TURN the ship by it, then fire the engine
        // along the attitude that turn produced. Splitting the integration out
        // and running it *after* thrust silently changed that — the engine
        // began firing along the previous tick's attitude, a one-tick lag in
        // thrust direction on every burn. Nothing failed and nothing warned:
        // an adversarial re-read of the split caught it by recovering the old
        // line table out of a stale object file and comparing the statement
        // order.
        //
        // Registering the integrator FIRST puts the turn back ahead of the
        // burn, and extends it to every dynamic body rather than only the
        // thing the player is flying — so crates and debris can topple too.
        physics.addSystem(std::make_unique<AngularIntegrationSystem>());
        physics.addSystem(std::make_unique<ThrustSystem>());
        // AERODYNAMICS, after thrust and before the ground. Thrust first,
        // because a rocket's own acceleration is part of the flow it meets
        // this tick; the ground last, because whatever the air did to a
        // vehicle about to land must not survive the touchdown.
        {
            auto aerodynamics = std::make_unique<sw::aero::VesselAerodynamicsSystem>();
            m_aerodynamics = aerodynamics.get();
            physics.addSystem(std::move(aerodynamics));
        }
        sw::phys::SurfaceInteractionSystem::Config surfaceConfig{};
        physics.addSystem(
            std::make_unique<sw::phys::SurfaceInteractionSystem>(surfaceConfig));
        physics.addSystem(std::make_unique<CapsuleMovementSystem>());
        physics.addSystem(std::make_unique<SpinSystem>());
        // Atmosphere/cloud shells follow their planet (own drift spin).
        physics.addSystem(std::make_unique<CloudLayerSystem>());
        // After the celestial spin: surface bases co-rotate with their body.
        physics.addSystem(std::make_unique<sw::phys::SurfaceAnchorSystem>());
        // Parts ride their vessel (lockstep interpolation), last.
        physics.addSystem(std::make_unique<sw::parts::PartAttachmentSystem>());
        // ...and only NOW is every solid thing where it is going to be, so
        // this is the one place the walker can be pushed back out of what it
        // walked into. Before the anchors and the part attachment it would
        // be resolving against last tick's building positions, which on a
        // spinning planet is 595 m of lie.
        m_hullCollision = new sw::phys::HullCollisionSystem();
        physics.addSystem(
            std::unique_ptr<sw::phys::HullCollisionSystem>(m_hullCollision));

        auto& automation = m_simulation.findLane("Automation")->scheduler();
        automation.addSystem(std::make_unique<SolarChargeSystem>());
        // F3 — THE GRID RUNS FIRST. Every building's `satisfaction` for this
        // tick is decided here, from the real sun and the site's batteries,
        // and the executor below spends it. The other order would run the
        // factory on last tick's weather.
        automation.addSystem(std::make_unique<sw::factory::PowerGridSystem>(m_solEntity));
        // The generic recipe executor: ONE system for every building the
        // player will ever place. The two below stay for the asteroid rig
        // and the orbital station, which are craft, not buildings.
        automation.addSystem(std::make_unique<sw::factory::ProductionSystem>());
        // ...and the assembly hall, which is the executor's sibling: same
        // lane, same power, same bulk catch-up, but its bill of materials
        // comes from a design the player drew rather than from a recipe.
        automation.addSystem(std::make_unique<sw::factory::AssemblySystem>());
        automation.addSystem(std::make_unique<sw::factory::MinerSystem>());
        automation.addSystem(std::make_unique<sw::factory::RefinerySystem>());

        auto& logistics = m_simulation.findLane("Logistics")->scheduler();
        logistics.addSystem(std::make_unique<sw::factory::TransferSystem>());
        sw::phys::SimulationBubbleSystem::Config bubbleConfig{};
        bubbleConfig.enterRadius = kBubbleEnterRadius;
        bubbleConfig.exitRadius = kBubbleExitRadius;
        auto bubble = std::make_unique<sw::phys::SimulationBubbleSystem>(
            m_commands, *m_physicsLane, bubbleConfig);
        m_bubbleSystem = bubble.get();
        logistics.addSystem(std::move(bubble));

        m_simulation.findLane("World")->scheduler().addSystem(
            std::make_unique<StatsSystem>());

        SW_LOG_INFO("Game", "Milestone 10 scene ready: {} entities", m_world.aliveCount());
        SW_LOG_INFO("Game",
                    "Controls: Tab pilot/free | G EVA (first person: mouse turns you, "
                    "A/D strafe) | F build menu; LCLICK build, WHEEL rotate, R demolish; "
                    "belts: LCLICK output then input | B hangar | M map | P next ship | V "
                    "speed ORB/SRF | Shift/Ctrl throttle | ,/. warp | W/S A/D arrows Q/E "
                    "X | Space pause | Esc quit");
    }

    void StarWorksGame::buildBootPlan()
    {
        // Order matters and is not arbitrary: the part catalogue is what
        // every mesh and every vessel is built from, aero tables and recipes
        // are read by name out of that catalogue, and the scene needs all
        // three. Each entry is one frame, so the labels are what the player
        // reads while they wait — they name the WORK, not the function.
        m_bootSteps = {
            {"LOADING PARTS", &StarWorksGame::bootLoadParts},
            {"LOADING AERODYNAMICS", &StarWorksGame::bootLoadAero},
            {"LOADING RECIPES", &StarWorksGame::bootLoadRecipes},
            {"LOADING DESIGNS", &StarWorksGame::bootLoadDesigns},
            {"BUILDING THE SYSTEM", &StarWorksGame::bootBuildScene},
            {"STARTING THE SIMULATION", &StarWorksGame::bootWireSystems},
            {"BUILDING INSTRUMENTS", &StarWorksGame::bootBuildInstruments},
            {"PREPARING SAVES", &StarWorksGame::bootPrepareSaves},
        };
        m_bootCursor = 0;
        m_bootLabel = m_bootSteps.empty() ? "READY" : m_bootSteps.front().label;
    }

    void StarWorksGame::bootLoadParts()
    {
        sw::parts::loadCatalog(sw::FileSystem::executableDirectory() / "Assets" / "Parts");
    }

    void StarWorksGame::bootLoadAero()
    {
        sw::aero::loadTables(sw::FileSystem::executableDirectory() / "Assets" / "Parts");
    }

    void StarWorksGame::bootLoadRecipes()
    {
        sw::factory::loadRecipeCatalog(sw::FileSystem::executableDirectory() / "Assets" /
                                       "Recipes");
    }

    void StarWorksGame::bootLoadDesigns()
    {
        sw::parts::loadBlueprintCatalog(sw::FileSystem::executableDirectory() / "Assets" /
                                        "Ships");
    }

    void StarWorksGame::bootBuildScene()
    {
        buildScene();
    }

    void StarWorksGame::bootBuildInstruments()
    {
        buildNavballMeshes();
        m_hangarFloorMeshIndex = registerMesh(renderer().createMesh(
            sw::PrimitiveFactory::makeGridPlane(40.0f, 20, {0.2f, 0.3f, 0.38f, 1.0f})));
        m_hangarCamera.setPerspective(sw::math::toRadians(55.0f), 0.2f, 500.0f);

        // The title screen's backdrop camera, and its readability scrim.
        m_menuCamera.setPerspective(sw::math::toRadians(50.0f), 0.5f, 1.0e13f);
        {
            // A full-screen vertical gradient: darkest at the top where the
            // title sits over open space, nearly clear across the middle so
            // the planet reads, a touch darker again at the foot for the
            // build line. hudQuad cannot do this — its unit quad is one flat
            // colour — so the gradient lives in vertex alpha and the HUD
            // shader's `vColor * tint` does the rest.
            sw::MeshData scrim;
            const sw::Vec3 normal{0.0f, 0.0f, 1.0f};
            const sw::Vec3 shade{0.010f, 0.014f, 0.026f};
            const auto rowAt = [&](sw::f32 y, sw::f32 alpha) {
                scrim.vertices.push_back({{-1.0f, y, 0.0f}, normal,
                                          {shade.x, shade.y, shade.z, alpha}, {}});
                scrim.vertices.push_back({{1.0f, y, 0.0f}, normal,
                                          {shade.x, shade.y, shade.z, alpha}, {}});
            };
            rowAt(-1.00f, 0.72f); // top (NDC y grows downward)
            rowAt(-0.34f, 0.34f); // under the title
            rowAt(0.42f, 0.24f);  // across the buttons
            rowAt(1.00f, 0.42f);  // foot
            for (sw::u32 band = 0; band < 3; ++band)
            {
                const sw::u32 a = band * 2;
                scrim.indices.insert(scrim.indices.end(),
                                     {a, a + 1, a + 2, a + 1, a + 3, a + 2});
            }
            m_menuScrimMeshIndex = registerMesh(renderer().createMesh(scrim));
        }
    }

    void StarWorksGame::bootPrepareSaves()
    {
        buildSaveSchema();

        // CAN THIS WORLD BE SAVED? Asked now, on the world the scene builder
        // just produced, rather than discovered when the player presses F5
        // hours later and gets one line saying "Save failed".
        if (const std::vector<std::string> missing =
                sw::save::unsaveableComponents(m_world, m_saveSchema);
            !missing.empty())
        {
            for (const std::string& entry : missing)
            {
                SW_LOG_ERROR("Save",
                             "{} is in the world but not in the save schema — saving is "
                             "IMPOSSIBLE until it is registered in buildSaveSchema",
                             entry);
            }
        }
        m_celestialIndex.rebuild(m_world);
        refreshSaveSlots();
    }

    void StarWorksGame::updateBoot()
    {
        if (m_bootCursor >= m_bootSteps.size())
        {
            m_shell = Shell::Menu;
            m_menuPage = MenuPage::Root;
            // The backdrop camera must be posed before the menu's FIRST
            // rendered frame — onUpdate has already run this frame, so an
            // unposed camera would show one frame of garbage from origin.
            updateMenuCamera(0.0f);
            // Nothing ticks behind a menu. The world exists — it was just
            // built — but a player reading a menu is not playing, and a
            // simulation that ran underneath one would age the save they are
            // about to load over it.
            m_simulation.setPaused(true);
            SW_LOG_INFO("Game", "Ready");
            return;
        }

        // ONE STEP PER FRAME, and the label is updated BEFORE the work runs,
        // so the bar names the thing the player is currently waiting for
        // rather than the thing that just finished.
        const BootStep step = m_bootSteps[m_bootCursor];
        m_bootLabel = step.label;
        ++m_bootCursor;
        (this->*step.run)();
        if (m_bootCursor < m_bootSteps.size())
        {
            m_bootLabel = m_bootSteps[m_bootCursor].label;
        }
    }

    void StarWorksGame::newGame()
    {
        if (m_hasSession)
        {
            // A world was already played or loaded, so it has to go before a
            // new one is built. Everything derived from it is rebuilt below;
            // the mesh table is deliberately NOT reset, because meshes are
            // GPU resources keyed by index and the scene builder registers
            // its own afresh.
            netLeave();
            m_world.clearForRestore();
            m_shipEntity = {};
            m_capsuleEntity = {};
            m_nodeActive = false;
            m_syncWarpTo = 0.0;
            m_warpIndex = 0;
            m_prediction.clear();
            m_nodePrediction.clear();
            buildScene();
            m_celestialIndex.rebuild(m_world);
            m_lastPredictionSeconds = -1.0e9;
            m_grassCenterDir = sw::Vec3(0.0f);
        }
        m_simulation.setPaused(false);
        m_hasSession = true;
        m_shell = Shell::Playing;
        m_shellStatus.clear();
        SW_LOG_INFO("Game", "New game");
    }

    void StarWorksGame::continueGame()
    {
        m_simulation.setPaused(false);
        m_hasSession = true;
        m_shell = Shell::Playing;
    }

    void StarWorksGame::openMenu(MenuPage page)
    {
        m_shell = Shell::Menu;
        m_menuPage = page;
        m_simulation.setPaused(true);
        m_saveNameFocused = false;
        if (page == MenuPage::Load || page == MenuPage::Save)
        {
            refreshSaveSlots();
        }
    }

    void StarWorksGame::handleShellClick(sw::u32 id)
    {
        m_shellStatus.clear();
        switch (id)
        {
            case 2000: continueGame(); return;
            case 2001: newGame(); return;
            case 2002: openMenu(MenuPage::Load); return;
            case 2003:
                // Offer a name rather than an empty box: the common case is
                // "just save it", and a player who wants to name it can type
                // over a suggestion faster than they can invent one.
                if (m_saveName.empty())
                {
                    m_saveName = std::format(
                        "SAVE {}", static_cast<sw::u64>(m_simulation.simulatedSeconds()));
                }
                openMenu(MenuPage::Save);
                return;
            case 2004: openMenu(MenuPage::Settings); return;
            case 2005: window().requestClose(); return;
            case 2010: m_saveNameFocused = !m_saveNameFocused; return;
            case 2011:
            {
                // A name typed by a human goes into a filename, so it is
                // sanitised rather than trusted: a slash would silently write
                // somewhere else, and a colon is not a filename on Windows.
                std::string file;
                for (const char c : m_saveName)
                {
                    const bool safe = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                                      (c >= '0' && c <= '9') || c == ' ' || c == '-' ||
                                      c == '_';
                    file.push_back(safe ? c : '_');
                }
                while (!file.empty() && file.back() == ' ') { file.pop_back(); }
                if (file.empty()) { file = "SAVE"; }
                try
                {
                    saveGameTo(savesDirectory() / (file + ".sav"));
                    m_shellStatus = std::format("SAVED AS {}", file);
                    refreshSaveSlots();
                    m_menuPage = MenuPage::Root;
                }
                catch (const sw::Exception& e)
                {
                    m_shellStatus = "SAVE FAILED - SEE THE LOG";
                    SW_LOG_ERROR("Game", "Save failed: {}", e.what());
                }
                m_saveNameFocused = false;
                return;
            }
            case 2099: openMenu(MenuPage::Root); return;
            default: break;
        }
        if (id >= 2100)
        {
            const auto index = static_cast<sw::usize>(id - 2100u);
            if (index >= m_saveSlots.size()) { return; }
            const SaveSlot slot = m_saveSlots[index];
            try
            {
                loadGameFrom(slot.path);
                m_hasSession = true;
                m_shell = Shell::Playing;
                m_simulation.setPaused(false);
                SW_LOG_INFO("Game", "Loaded '{}'", slot.path.string());
            }
            catch (const sw::Exception& e)
            {
                // A refused save leaves the world it was loading INTO in an
                // unknown state, so we do not pretend it is playable: stay in
                // the menu and say so.
                m_shellStatus = std::format("COULD NOT LOAD {}", hud::caps(slot.name));
                SW_LOG_ERROR("Game", "Load failed: {}", e.what());
            }
        }
    }

    void StarWorksGame::updateMenuCamera(sw::f32 deltaSeconds)
    {
        const auto* terra = m_world.tryGetComponent<TransformComponent>(m_terraEntity);
        if (terra == nullptr)
        {
            return;
        }
        // A slow drift, on WALL time: the simulation is paused behind the
        // menu, and this motion is presentation, not state. A full lap takes
        // about thirteen minutes — visible if you sit there, never dizzying.
        m_menuOrbitAngle += 0.008f * deltaSeconds;

        const sw::WorldVec3 center = terra->position;
        sw::Vec3 toSun{1.0f, 0.0f, 0.0f};
        if (const auto* sol = m_world.tryGetComponent<TransformComponent>(m_solEntity))
        {
            toSun = sw::Vec3(glm::normalize(sol->position - center));
        }

        // Park the camera near the terminator, biased to the day side: the
        // lit limb with a dark edge is the most photogenic angle a planet
        // has, and it is stable because the azimuth is measured FROM the
        // sun rather than from a fixed longitude.
        const sw::Vec3 pole = sw::math::kWorldUp;
        sw::Vec3 sunAzimuth = toSun - pole * glm::dot(toSun, pole);
        sunAzimuth = glm::length(sunAzimuth) > 1.0e-4f ? glm::normalize(sunAzimuth)
                                                       : sw::Vec3{1.0f, 0.0f, 0.0f};
        const sw::Vec3 east = glm::normalize(glm::cross(pole, sunAzimuth));
        const sw::f32 azimuth = 0.95f + m_menuOrbitAngle; // ~54 deg off subsolar
        sw::Vec3 direction =
            sunAzimuth * std::cos(azimuth) + east * std::sin(azimuth);
        direction = glm::normalize(direction + pole * 0.20f); // a little north

        const sw::f64 orbitRadius = kTerraRadius * 2.05; // ~6,700 km up
        const sw::WorldVec3 position = center + sw::WorldVec3(direction) * orbitRadius;

        // Aim ABOVE the planet's centre so the globe sits in the lower half
        // of the frame and the title floats over open space and stars.
        const sw::WorldVec3 target = center + sw::WorldVec3(pole) * (kTerraRadius * 1.15);
        const sw::Vec3 forward = glm::normalize(sw::Vec3(target - position));

        m_menuCamera.setAspectRatio(renderer().aspectRatio());
        m_menuCamera.setPosition(position);
        m_menuCamera.setOrientation(glm::quatLookAt(forward, pole));
    }

    void StarWorksGame::hudTextCentered(std::string_view text, sw::f32 centerX,
                                        sw::f32 y, sw::f32 heightNdc,
                                        const sw::Vec4& color)
    {
        // Visible width = (n-1) advances plus the last glyph's own width.
        const sw::f32 aspect = renderer().aspectRatio();
        const sw::f32 advance = sw::ui::kGlyphAdvance * heightNdc / aspect;
        const sw::f32 glyphWidth = (5.0f / 7.0f) * heightNdc / aspect;
        const sw::f32 width =
            text.empty() ? 0.0f
                         : advance * static_cast<sw::f32>(text.size() - 1) + glyphWidth;
        hudText(text, centerX - width * 0.5f, y, heightNdc, color);
    }

    void StarWorksGame::hudTitle(sw::f32 centerX, sw::f32 topY, sw::f32 heightNdc)
    {
        // Passes in submission order (text keeps submission order within its
        // layer): a faint blue glow as four SAME-SIZE offset copies, a hard
        // dark drop shadow for contrast against the lit limb, then the face.
        // The glow must not be a scaled-up copy: a bigger glyph has a bigger
        // advance, so its letters drift out of register with the face's as
        // the string runs on — offset copies share the advance and stay
        // aligned under every letter. The 5x7 glyphs read as deliberate
        // blockwork at this size, which suits an industrial game.
        constexpr std::string_view kTitle = "STARWORKS";
        const sw::f32 gx = heightNdc * 0.030f;
        const sw::f32 gy = heightNdc * 0.038f;
        const sw::Vec4 glow{0.40f, 0.68f, 1.00f, 0.11f};
        hudTextCentered(kTitle, centerX - gx, topY, heightNdc, glow);
        hudTextCentered(kTitle, centerX + gx, topY, heightNdc, glow);
        hudTextCentered(kTitle, centerX, topY - gy, heightNdc, glow);
        hudTextCentered(kTitle, centerX, topY + gy, heightNdc, glow);
        hudTextCentered(kTitle, centerX + 0.006f, topY + 0.008f, heightNdc,
                        {0.00f, 0.00f, 0.00f, 0.85f});
        hudTextCentered(kTitle, centerX, topY, heightNdc, {0.95f, 0.97f, 1.00f, 1.0f});
    }

    void StarWorksGame::collectBootBar()
    {
        // A BAR THAT MEASURES SOMETHING. Its width is steps-done over
        // steps-total, not a timer: a fake bar that reaches 90 % and waits
        // is worse than no bar, because it turns "this is slow" into "this
        // is broken" at exactly the moment the player is deciding whether to
        // kill the process.
        const sw::f32 total = std::max(1.0f, static_cast<sw::f32>(m_bootSteps.size()));
        const sw::f32 done = static_cast<sw::f32>(m_bootCursor) / total;

        hudQuad(-1.0f, -1.0f, 1.0f, 1.0f, sw::Vec4{0.035f, 0.045f, 0.06f, 1.0f});
        hudTitle(0.0f, -0.30f, 0.17f);

        constexpr sw::f32 kX0 = -0.42f;
        constexpr sw::f32 kX1 = 0.42f;
        constexpr sw::f32 kY = 0.10f;
        constexpr sw::f32 kH = 0.030f;
        hudQuad(kX0 - 0.004f, kY - 0.004f, kX1 + 0.004f, kY + kH + 0.004f,
                sw::Vec4{0.10f, 0.13f, 0.17f, 1.0f});
        hudQuad(kX0, kY, kX0 + (kX1 - kX0) * done, kY + kH, hud::kOk);

        hudText(hud::caps(m_bootLabel), kX0, kY + kH + 0.030f, 0.036f, hud::kText);
        hudText(std::format("{}/{}", m_bootCursor, m_bootSteps.size()), kX1 - 0.06f,
                kY + kH + 0.030f, 0.036f, hud::kTextDim);
    }

    void StarWorksGame::collectShellHud()
    {
        if (m_shell == Shell::Booting)
        {
            collectBootBar();
            return;
        }

        // THE BACKDROP IS THE GAME. On the title screen the world renders
        // behind this from the menu's own orbiting camera, so the cover is a
        // vertical gradient scrim — darkest over the space the title floats
        // in, nearly clear over the planet — not a wash that hides the very
        // image the screen is built around. A pause menu keeps the heavier
        // wash: its backdrop is a frozen mid-game frame, and half-visible
        // clickable-looking machinery under a menu invites clicks that will
        // not answer.
        const bool titleScreen = !m_hasSession;
        if (titleScreen && m_menuScrimMeshIndex != 0xFFFFFFFFu)
        {
            sw::DrawItem scrim{};
            scrim.mesh = &m_meshes[m_menuScrimMeshIndex];
            scrim.transform = sw::Mat4{1.0f};
            scrim.screenSpace = true;
            scrim.tint = {1.0f, 1.0f, 1.0f, 1.0f};
            m_drawItems.push_back(scrim);
        }
        else if (!titleScreen)
        {
            hudQuad(-1.0f, -1.0f, 1.0f, 1.0f, sw::Vec4{0.02f, 0.03f, 0.045f, 0.90f});
        }

        // THE TITLE, big and centred — the identity of the screen on the
        // title screen, a header on the pause menu.
        const sw::f32 titleHeight = titleScreen ? 0.235f : 0.130f;
        const sw::f32 titleTop = titleScreen ? -0.80f : -0.88f;
        hudTitle(0.0f, titleTop, titleHeight);
        if (titleScreen)
        {
            if (m_menuPage == MenuPage::Root)
            {
                hudTextCentered("AN INDUSTRIAL SPACE PROGRAM", 0.0f,
                                titleTop + titleHeight + 0.045f, 0.032f,
                                {0.62f, 0.74f, 0.88f, 0.92f});
            }
            hudTextCentered("PRE-ALPHA BUILD - CUSTOM ENGINE", 0.0f, 0.93f, 0.024f,
                            {0.55f, 0.65f, 0.78f, 0.65f});
        }

        sw::f32 cursorX = -2.0f;
        sw::f32 cursorY = -2.0f;
        const bool haveCursor = hudCursor(cursorX, cursorY);
        // The root page is a tight centred column of short verbs; the sub
        // pages hold tabular rows (a save's name, age and size) and get a
        // wider one.
        const bool rootPage = m_menuPage == MenuPage::Root;
        const sw::f32 kX0 = rootPage ? -0.26f : -0.36f;
        const sw::f32 kX1 = -kX0;
        constexpr sw::f32 kRow = 0.082f;
        constexpr sw::f32 kGap = 0.018f;
        sw::f32 y = rootPage ? -0.16f : -0.44f;

        const auto row = [&](std::string_view label, sw::u32 id, bool enabled) {
            const bool hot = enabled && haveCursor && cursorX >= kX0 && cursorX <= kX1 &&
                             cursorY >= y && cursorY <= y + kRow;
            const sw::Vec4 base = titleScreen ? sw::Vec4{0.05f, 0.09f, 0.14f, 0.70f}
                                              : sw::Vec4(hud::kRow);
            hudQuad(kX0, y, kX1, y + kRow,
                    !enabled ? sw::Vec4{0.06f, 0.07f, 0.09f, titleScreen ? 0.45f : 0.95f}
                    : hot    ? hud::kRowOnHover
                             : base);
            // A label longer than its row shrinks to fit rather than
            // spilling — measured with the same advance the renderer uses,
            // the discipline every HUD overflow in this project has taught.
            sw::f32 textHeight = rootPage ? 0.042f : 0.038f;
            const sw::f32 available = (kX1 - kX0) - 0.070f;
            const sw::f32 aspect = renderer().aspectRatio();
            const auto widthOf = [&](sw::f32 h) {
                return label.empty()
                           ? 0.0f
                           : (sw::ui::kGlyphAdvance * static_cast<sw::f32>(label.size() - 1) +
                              5.0f / 7.0f) *
                                 h / aspect;
            };
            if (widthOf(textHeight) > available)
            {
                textHeight *= available / widthOf(textHeight);
            }
            const sw::f32 textY = y + (kRow - textHeight) * 0.5f - 0.006f;
            if (rootPage)
            {
                hudTextCentered(label, 0.0f, textY, textHeight,
                                enabled ? hud::kText : hud::kTextDim);
            }
            else
            {
                hudText(label, kX0 + 0.040f, textY, textHeight,
                        enabled ? hud::kText : hud::kTextDim);
            }
            if (enabled)
            {
                m_hudButtons.push_back({kX0, y, kX1, y + kRow, id});
            }
            y += kRow + kGap;
        };

        switch (m_menuPage)
        {
            case MenuPage::Root:
                if (m_hasSession) { row("CONTINUE", 2000u, true); }
                // A dash, not parentheses: the 5x7 charset has no '(' — the
                // old label was silently rendering "NEW GAME  ABANDONS...".
                row(m_hasSession ? "NEW GAME - ABANDONS THIS ONE" : "NEW GAME", 2001u,
                    true);
                row("LOAD GAME", 2002u, !m_saveSlots.empty());
                if (m_hasSession) { row("SAVE GAME", 2003u, true); }
                row("SETTINGS", 2004u, true);
                row("QUIT", 2005u, true);
                break;

            case MenuPage::Load:
                hudTextCentered("LOAD GAME", 0.0f, -0.53f, 0.048f, hud::kTitle);
                if (m_saveSlots.empty())
                {
                    hudText("NO SAVES YET", kX0 + 0.045f, y + 0.026f, 0.040f,
                            hud::kTextDim);
                    y += kRow + kGap;
                }
                for (sw::usize i = 0; i < m_saveSlots.size() && i < 8; ++i)
                {
                    const SaveSlot& slot = m_saveSlots[i];
                    row(std::format("{}   {}   {} KB", hud::caps(slot.name), slot.when,
                                    slot.bytes / 1024),
                        2100u + static_cast<sw::u32>(i), true);
                }
                row("BACK", 2099u, true);
                break;

            case MenuPage::Save:
            {
                hudTextCentered("SAVE GAME", 0.0f, -0.53f, 0.048f, hud::kTitle);
                // The name field, on the same machinery as the multiplayer
                // address box: click to focus, type, ESC to cancel.
                const bool hot = haveCursor && cursorX >= kX0 && cursorX <= kX1 &&
                                 cursorY >= y && cursorY <= y + kRow;
                hudQuad(kX0, y, kX1, y + kRow,
                        m_saveNameFocused ? hud::kRowOn : (hot ? hud::kRowHover : hud::kRow));
                hudText(m_saveNameFocused ? m_saveName + "-" : m_saveName,
                        kX0 + 0.045f, y + 0.026f, 0.044f, hud::kText);
                m_hudButtons.push_back({kX0, y, kX1, y + kRow, 2010u});
                y += kRow + kGap;
                row("WRITE IT", 2011u, !m_saveName.empty());
                row("BACK", 2099u, true);
                break;
            }

            case MenuPage::Settings:
                hudTextCentered("SETTINGS", 0.0f, -0.53f, 0.048f, hud::kTitle);
                // DELIBERATELY EMPTY, and it says so. A settings page that
                // silently has nothing in it reads as a page that failed to
                // load; one that admits it is a promise.
                hudText("NOTHING TO CONFIGURE YET.", kX0, y + 0.020f, 0.038f,
                        hud::kTextDim);
                hudText("QUALITY AND LOGGING ARE COMMAND-LINE FLAGS FOR NOW",
                        kX0, y + 0.070f, 0.030f, hud::kTextDim);
                y += kRow * 2.0f;
                row("BACK", 2099u, true);
                break;
        }

        if (!m_shellStatus.empty())
        {
            hudTextCentered(hud::caps(m_shellStatus), 0.0f, y + 0.030f, 0.034f,
                            hud::kTextDim);
        }
    }
} // namespace game
