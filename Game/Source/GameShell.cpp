// ============================================================================
// GameShell.cpp — The shell around the game: boot plan, main menu, named save slots UI.
// Split out of StarWorksGame.cpp; same class, one theme per translation unit.
// ============================================================================

#include "StarWorksGame.hpp"

#include <cstdio>

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
        // BEFORE the assembly system, which reads the phases to decide how
        // much power a half-open panel makes.
        physics.addSystem(std::make_unique<sw::parts::PartAnimationSystem>());
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
        {
            auto thrust = std::make_unique<ThrustSystem>();
            m_thrustSystem = thrust.get();
            physics.addSystem(std::move(thrust));
        }
        applyCreativeMode(); // the wiring must reflect the mode from frame one
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
        // ...and its railed twin: a ring ship spins for gravity whether or
        // not anybody is close enough for it to be simulated. Before
        // PartAttachmentSystem, so the ring's parts ride this tick's angle.
        physics.addSystem(std::make_unique<RailsSpinSystem>(*m_physicsLane));
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
        m_menuCamera.setPerspective(sw::math::toRadians(50.0f), 0.5f, 1.5e17f);
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

    void StarWorksGame::parseDebugShot()
    {
        if (const char* noGlare = std::getenv("SW_NO_GLARE");
            noGlare != nullptr && *noGlare != '\0' && *noGlare != '0')
        {
            m_debugNoGlare = true;
        }
        const char* spec = std::getenv("SW_SHOT");
        if (spec == nullptr || *spec == '\0')
        {
            return;
        }
        std::string text(spec);
        // `BODY@RADII[,map][,YAW]`. Everything after the body name is
        // optional; a malformed field leaves its default rather than
        // refusing, because this is a development aid and the fastest
        // failure mode for one is to still take the picture.
        const auto at = text.find('@');
        m_debugShotBody = text.substr(0, at);
        for (char& c : m_debugShotBody)
        {
            c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        }
        if (at != std::string::npos)
        {
            std::string rest = text.substr(at + 1);
            std::string field;
            bool first = true;
            bool yawTaken = false;
            while (!rest.empty())
            {
                const auto comma = rest.find(',');
                field = rest.substr(0, comma);
                rest = (comma == std::string::npos) ? std::string() : rest.substr(comma + 1);
                if (first)
                {
                    m_debugShotRadii = std::strtod(field.c_str(), nullptr);
                    first = false;
                }
                else if (field == "map")
                {
                    m_debugShotMap = true;
                }
                else if (!yawTaken)
                {
                    m_debugShotYaw = static_cast<sw::f32>(std::strtod(field.c_str(), nullptr));
                    yawTaken = true;
                }
                else
                {
                    // ELEVATION, the fourth field, and it exists because the
                    // interesting face of a thing is not always its top. An
                    // engine's nozzles point down its own -Y; a camera nailed
                    // above the waterline photographs the roof of it and
                    // proves nothing about whether the flame lit.
                    m_debugShotPitch =
                        static_cast<sw::f32>(std::strtod(field.c_str(), nullptr));
                }
            }
        }
        if (m_debugShotRadii <= 0.0)
        {
            m_debugShotRadii = 6.0;
        }
        SW_LOG_INFO("Game", "SW_SHOT: body '{}' at {} radii{}{}", m_debugShotBody,
                    m_debugShotRadii, m_debugShotMap ? " (map)" : "",
                    m_debugShotYaw != 0.0f ? " yawed" : "");
    }

    void StarWorksGame::applyDebugJump()
    {
        // SW_JUMP=<SYSTEM NAME>: teleport the controlled craft to that
        // system's primary, ten thousand stellar radii out.
        //
        // It exists because the interstellar half of the game is otherwise
        // UNTESTABLE from a capture. A system's planets are built the frame
        // the craft enters the star's sphere of influence, and getting there
        // legitimately is four light-years of travel — so without this there
        // is no way to photograph Proxima b, and a rendering path nobody can
        // photograph is a rendering path nobody has checked. Same reasoning
        // as SW_SHOT, one scale up.
        if (m_shell != Shell::Playing)
        {
            return;
        }
        // THE PROBES RUN ABOVE THE JUMP LATCH, and that is not tidiness.
        // SW_JUMP sets m_debugJumped on its first frame and every later call
        // returns here — so a probe placed below it, waiting thirty frames for
        // the camera to be put where it belongs, was never reached in any run
        // that also jumped. Which is every run where the question is about
        // another star.
        if (const char* sasSpec = std::getenv("SW_SAS"); sasSpec != nullptr)
        {
            // SW_SAS=<mode> ENGAGES AN AUTOPILOT MODE. A capture has no
            // keyboard and no mouse, so without this the four new hold modes
            // and the four navball markers that go with them have no picture
            // — and a marker whose sign nobody has looked at is exactly the
            // kind of thing that ships pointing the wrong way.
            m_sasMode = static_cast<sw::u32>(std::strtoul(sasSpec, nullptr, 10));
        }
        if (std::getenv("SW_GROUNDPROBE") != nullptr && !m_debugGroundProbed)
        {
            // HOW SMOOTHLY THE WORLD MOVES, in the shipping engine, as a
            // function of how much simulated time the session has run.
            //
            // It samples the PLANET rather than the walker, and that is a
            // correction rather than a convenience: SW_CLOCK winds the clock
            // before the scene is built, but the scene is still PLACED at spin
            // angle zero, so the first tick rotates the ground out from under
            // a walker carrying the wrong 464 m/s — a ballistic hop that
            // measures the hook and not the engine. The body's own per-frame
            // step has no such artefact, and it is exactly the quantity the
            // split clock exists to protect: an orbit is smooth, so what is
            // left after removing the mean step IS the numerical noise.
            const sw::ecs::Entity subject = controlledEntity();
            const auto* transform =
                m_world.tryGetComponent<TransformComponent>(subject);
            sw::i32 primary =
                (transform != nullptr)
                    ? m_celestialIndex.soiPrimaryAt(transform->position,
                                                    m_physicsLane->presentSeconds())
                    : -1;
            if (++m_groundWarmup < 60u)
            {
                // let the boot finish
            }
            else if (primary >= 0 && m_groundSamples.size() < 400)
            {
                const auto& body =
                    m_celestialIndex.body(static_cast<sw::usize>(primary));
                const sw::WorldVec3 here =
                    m_world.getComponent<TransformComponent>(body.entity).position;
                const sw::WorldVec3 previous =
                    m_world.getComponent<PreviousTransformComponent>(body.entity).position;
                m_groundSamples.push_back(glm::length(here - previous));
            }
            else if (m_groundSamples.size() >= 400)
            {
                m_debugGroundProbed = true;
                if (std::FILE* probe = std::fopen("/tmp/sw_groundprobe.txt", "w"))
                {
                    sw::f64 lo = 1.0e30, hi = -1.0e30, sum = 0.0;
                    sw::usize counted = 0;
                    for (const sw::f64 step : m_groundSamples)
                    {
                        if (!(step > 0.0)) { continue; } // frames with no tick
                        lo = std::min(lo, step);
                        hi = std::max(hi, step);
                        sum += step;
                        ++counted;
                    }
                    std::fprintf(probe,
                                 "simulated time %.6g s\n"
                                 "%zu ticks of the primary's own motion\n"
                                 "mean step %.6f m\n"
                                 "step jitter (max-min) %.6f m\n",
                                 m_simulation.simulatedSeconds(), counted,
                                 counted > 0 ? sum / static_cast<sw::f64>(counted) : 0.0,
                                 counted > 0 ? hi - lo : 0.0);
                    std::fclose(probe);
                }
            }
        }
        if (std::getenv("SW_STARPROBE") != nullptr && !m_debugStarProbed &&
            ++m_debugProbeDelay > 30u)
        {
            // WHAT EVERY STAR WILL LOOK LIKE, in one table, from the values
            // that actually decide it — because "check that the red dwarfs are
            // small and red" is a question about thirty-six objects and eyes
            // are the wrong instrument for thirty-six of anything.
            m_debugStarProbed = true;
            if (std::FILE* probe = std::fopen("/tmp/sw_starprobe.txt", "w"))
            {
                std::fprintf(probe,
                             "%-18s %-8s %8s %8s %10s %6s %6s %6s %s\n", "NAME",
                             "CLASS", "R/Rsol", "T(K)", "L/Lsol", "R", "G", "B",
                             "READS AS");
                for (sw::u32 i = 0; i < sw::space::stars().size(); ++i)
                {
                    const sw::space::StarRecord& record = sw::space::stars()[i];
                    const sw::Vec3 hue =
                        blackbodyColor(static_cast<sw::f32>(record.temperature));
                    // The name the EYE would give it: hue by which channel
                    // leads and by how far, size by radius against Sol.
                    // Neutral FIRST: Sol is exactly (1,1,1) by construction
                    // and a "blue is at least red" test calls that blue.
                    const sw::f32 chroma = hue.r - hue.b;
                    const char* colourWord =
                        (std::abs(chroma) < 0.06f) ? "WHITE"
                        : (chroma < -0.12f)        ? "BLUE"
                        : (chroma < 0.0f)          ? "BLUE-WHITE"
                        : (hue.b > 0.72f)          ? "YELLOW"
                        : (hue.b > 0.42f)          ? "ORANGE"
                                                   : "RED";
                    const char* sizeWord =
                        (record.radius <= 0.03 * sw::space::kSunRadius) ? "TINY"
                        : (record.radius < 0.6 * sw::space::kSunRadius)  ? "SMALL"
                        : (record.radius < 1.6 * sw::space::kSunRadius)  ? "SUNLIKE"
                        : (record.radius < 10.0 * sw::space::kSunRadius) ? "LARGE"
                                                                        : "GIANT";
                    std::fprintf(probe, "%-18s %-8s %8.4f %8.0f %10.3g %6.2f %6.2f "
                                        "%6.2f %s %s\n",
                                 record.name, record.designation,
                                 record.radius / sw::space::kSunRadius,
                                 record.temperature, record.luminosity,
                                 static_cast<double>(hue.r), static_cast<double>(hue.g),
                                 static_cast<double>(hue.b), sizeWord, colourWord);
                }
                // ...and which of them are SUNS from where the camera is, which
                // is the other half of the report.
                std::vector<sw::ecs::Entity> suns;
                collectSunsHere(m_camera.position(), suns);
                // WITH THE NUMBER THAT DECIDED IT. A list of names says which
                // stars passed and nothing about why, and the first run of
                // this probe reported twenty-seven suns in a sky that has two
                // — a list alone could not tell a broken threshold from a
                // camera parked somewhere unexpected.
                auto irradiance = [&](sw::ecs::Entity e) -> double {
                    const auto* t = m_world.tryGetComponent<TransformComponent>(e);
                    const auto* v = m_world.tryGetComponent<StarVisualComponent>(e);
                    if (t == nullptr || v == nullptr) { return -1.0; }
                    const sw::WorldVec3 d = t->position - m_camera.position();
                    return static_cast<double>(v->luminosity) /
                           std::max(glm::dot(d, d), 1.0);
                };
                const double dominant = irradiance(m_lightStar);
                std::fprintf(probe, "\ncamera at %.4g, %.4g, %.4g m\n",
                             m_camera.position().x, m_camera.position().y,
                             m_camera.position().z);
                std::fprintf(probe, "suns here: %zu   (dominant irradiance %.4g)\n",
                             suns.size(), dominant);
                for (const sw::ecs::Entity sun : suns)
                {
                    const auto* visual = m_world.tryGetComponent<StarVisualComponent>(sun);
                    if (visual != nullptr)
                    {
                        const double e = irradiance(sun);
                        std::fprintf(probe, "  %-18s ratio %10.3g%s\n",
                                     sw::space::stars()[visual->catalogueIndex].name,
                                     dominant > 0.0 ? e / dominant : -1.0,
                                     sun == m_lightStar ? "  (dominant, keeps the flare)"
                                                        : "");
                    }
                }
                std::fclose(probe);
            }
        }
        if (m_debugJumped)
        {
            return;
        }
        // SW_TARGET=<BODY> and SW_ESCAPE=1 ride along with SW_JUMP: between
        // them they can put the game into the one state the interstellar
        // guidance panel exists for — escaping a star with another star
        // selected — which is otherwise hours of flying away.
        if (const char* targetSpec = std::getenv("SW_TARGET"); targetSpec != nullptr)
        {
            std::string wantedBody(targetSpec);
            for (char& c : wantedBody)
            {
                c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            }
            for (sw::usize i = 0; i < m_celestialIndex.size(); ++i)
            {
                std::string candidate = m_celestialIndex.body(i).name;
                for (char& c : candidate)
                {
                    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
                }
                if (candidate == wantedBody)
                {
                    m_targetIndex = static_cast<sw::i32>(i);
                    SW_LOG_INFO("Game", "SW_TARGET: {}", candidate);
                    break;
                }
            }
        }
        if (const char* warpSpec = std::getenv("SW_WARP"); warpSpec != nullptr)
        {
            // Ask for a rung and let the ladder refuse it. There is no other
            // way to photograph a gate: the clamp only runs when something has
            // been requested, and a capture has no keyboard.
            m_warpIndex = std::min(static_cast<sw::u32>(std::strtoul(warpSpec, nullptr, 10)),
                                   kWarpSteps - 1u);
            m_simulation.setPaused(false);
        }
        if (const char* escapeSpec = std::getenv("SW_ESCAPE");
            escapeSpec != nullptr && *escapeSpec != '0')
        {
            // AND OUT OF THE EVA SUIT. The game starts on foot, and on foot
            // the ladder is capped at x100 whatever the star says — so every
            // capture of the interstellar rungs came back reading "WARP X100"
            // and looked like the gate was broken when it was the pilot who
            // was standing outside.
            m_evaMode = false;
            if (auto* body = m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(
                    controlledEntity()))
            {
                const sw::WorldVec3 sun =
                    m_world.getComponent<TransformComponent>(m_lightStar.isNull()
                                                                 ? m_solEntity
                                                                 : m_lightStar)
                        .position;
                // SW_ESCAPE=<billions of km> out and moving. The default of
                // six is past the deep-space boundary; the other two warp
                // bands are at fifty and a thousand, and being able to stand
                // in each of them is the only way to photograph the ladder.
                const sw::f64 billions = std::max(std::strtod(escapeSpec, nullptr), 6.0);
                const sw::WorldVec3 outward{0.0, 0.0, 1.0};
                m_world.getComponent<TransformComponent>(controlledEntity()).position =
                    sun + outward * (billions * 1.0e12);
                body->velocity = outward * 1.2e5; // 120 km/s: unambiguously gone
                body->isGrounded = 0;
                m_debugJumped = true;
                SW_LOG_INFO("Game", "SW_ESCAPE: {} bn km out, 120 km/s outbound",
                            billions);
                return;
            }
        }
        if (std::getenv("SW_BOARD") != nullptr && m_shipEntity.isNull())
        {
            // TAKE THE CONTROLS. A new world starts the player ON FOOT, and
            // m_shipEntity stays null until somebody presses P — so every
            // capture of the part menu was aimed at an astronaut standing next
            // to an outpost, matched no part on any ship, and came back empty
            // in a way that read as a broken pick. It was a pilot who had not
            // boarded.
            cyclePilotedVessel();
            SW_LOG_INFO("Game", "SW_BOARD: piloting vessel {}", m_shipEntity.index);
        }
        if (const char* menuSpec = std::getenv("SW_PARTMENU"); menuSpec != nullptr)
        {
            // Open an operable part's menu, and optionally work it.
            // SW_PARTMENU=<n>[,<definitionId>]: n=1 opens the menu, n>=2 also
            // toggles animation n-2, and the optional id says WHICH kind of
            // part to open it on — without that the first animated module on
            // the ring wins, and the ring's first module is not necessarily
            // the one the picture is about. A capture has no mouse, so
            // without this hook the one thing the whole feature is for cannot
            // be photographed.
            char* cursor = nullptr;
            const sw::u32 toggle = static_cast<sw::u32>(std::strtoul(menuSpec, &cursor, 10));
            const sw::u32 wantedDefinition =
                (cursor != nullptr && *cursor == ',')
                    ? static_cast<sw::u32>(std::strtoul(cursor + 1, nullptr, 10))
                    : 0u;
            m_world.forEach<sw::parts::PartComponent,
                            sw::parts::PartAnimationComponent>(
                [&](sw::ecs::Entity entity, sw::parts::PartComponent& part,
                    sw::parts::PartAnimationComponent&) {
                    if (!m_menuPart.isNull() || part.vessel != m_shipEntity ||
                        (wantedDefinition != 0u && part.definitionId != wantedDefinition))
                    {
                        return;
                    }
                    m_menuPart = entity;
                });
            if (!m_menuPart.isNull() && toggle >= 2 && !m_debugMenuToggled)
            {
                // ONCE. Called every frame it flips the target every frame and
                // the panel never moves at all — which looks exactly like an
                // animation system that does not work.
                m_debugMenuToggled = true;
                // THROUGH THE BUTTON THAT IS ACTUALLY THERE.
                //
                // This hook called togglePartAnimation directly, then a
                // synthesised HudButton, and BOTH stepped over a broken step:
                // first the routing, then the fact that the row had already
                // been cleared out of the table by a later collector. A test
                // that manufactures its own input cannot see either.
                //
                // So it searches the table the click handler reads, and says
                // so loudly when the row is not in it — because "the button is
                // missing" and "the button did nothing" are the two failures
                // that look identical from outside and have nothing in common.
                const sw::u32 wantedButton = 900u + (toggle - 2u);
                const HudButton* row = nullptr;
                for (const HudButton& candidate : m_hudButtons)
                {
                    if (candidate.id == wantedButton) { row = &candidate; break; }
                }
                if (row == nullptr)
                {
                    if (std::FILE* miss = std::fopen("/tmp/sw_buttonprobe.txt", "w"))
                    {
                        std::fprintf(miss,
                                     "BUTTON %u IS NOT IN THE TABLE: %zu buttons, ids",
                                     wantedButton, m_hudButtons.size());
                        for (const HudButton& candidate : m_hudButtons)
                        {
                            std::fprintf(miss, " %u", candidate.id);
                        }
                        std::fprintf(miss, "\n");
                        std::fclose(miss);
                    }
                    m_debugMenuToggled = false; // try again next frame
                }
                else
                {
                    if (std::FILE* hit = std::fopen("/tmp/sw_buttonprobe.txt", "w"))
                    {
                        std::fprintf(hit, "pressed button %u of %zu\n", wantedButton,
                                     m_hudButtons.size());
                        std::fclose(hit);
                    }
                    static_cast<void>(applyHudClick(*row));
                }
            }
        }
        if (std::getenv("SW_PICKPROBE") != nullptr && !m_debugProbed &&
            ++m_debugProbeDelay > 30u)
        {
            // WHAT THE PICK ACTUALLY SEES. Three rounds of fixing this by
            // reasoning produced three plausible corrections and no working
            // click, which is the point at which reasoning has to stop and a
            // measurement has to start.
            m_debugProbed = true;
            // Straight to a file, flushed. The ordinary log lives in a buffer
            // that a capture run's teardown throws away — three probes came
            // back empty before that was noticed.
            std::FILE* probe = std::fopen("/tmp/sw_pickprobe.txt", "w");
            // THE CATALOGUE FIRST. The probe's first run reported "0 of 35
            // parts have animations", which has two completely different
            // causes — a catalogue that never loaded, or a catalogue that
            // loaded and simply has nothing animated on THIS ship — and the
            // count alone cannot tell them apart. So it now says which.
            if (probe != nullptr)
            {
                std::fprintf(probe, "catalogue: %zu definitions\n",
                             sw::parts::catalog().size());
                for (const sw::parts::PartDefinition& definition : sw::parts::catalog())
                {
                    if (!definition.animations.empty())
                    {
                        std::fprintf(probe, "  animated definition %u '%s': %zu\n",
                                     definition.id, definition.name.c_str(),
                                     definition.animations.size());
                    }
                }
            }
            sw::u32 parts = 0;
            sw::u32 animated = 0;
            sw::u32 stateful = 0;
            const sw::WorldVec3 eye = m_camera.position();
            const sw::Vec3 forward = m_camera.forward();
            m_world.forEach<TransformComponent, sw::parts::PartComponent>(
                [&](sw::ecs::Entity entity, TransformComponent&,
                    sw::parts::PartComponent& part) {
                    ++parts;
                    const auto* definition = sw::parts::findDefinition(part.definitionId);
                    if (definition == nullptr || definition->animations.empty())
                    {
                        return;
                    }
                    ++animated;
                    const bool hasState =
                        m_world.tryGetComponent<sw::parts::PartAnimationComponent>(
                            entity) != nullptr;
                    stateful += hasState ? 1u : 0u;
                    // AT THE DRAWN POSE, the same one the pick uses. Aiming the
                    // probe at the simulated pose reported "aimedPick nothing"
                    // for every module — which is not a broken pick, it is a
                    // hundred and ninety metres of interpolation lag measuring
                    // itself.
                    const sw::Vec3 relative = sw::Vec3(renderPosition(entity) - eye);
                    const sw::f32 radius =
                        std::max(sw::parts::partBoundsRadius(*definition), 0.75f);
                    if (probe != nullptr)
                    {
                        // AIMED AT THIS PART, not straight ahead. A ray down
                        // the middle of the screen missed all six modules and
                        // looked like a broken pick; it was a camera pointed at
                        // the hub, which is the hole in the middle of a ring.
                        // What the pick has to answer is "if the cursor is ON
                        // the part, is the part what comes back" — so the ray
                        // is aimed at the part and the ANSWER is its own name.
                        const sw::ecs::Entity aimed =
                            pickPartAlongRay(eye, glm::normalize(relative));
                        const char* hitName = "nothing";
                        if (!aimed.isNull())
                        {
                            const auto* hitDefinition = sw::parts::findDefinition(
                                m_world.getComponent<sw::parts::PartComponent>(aimed)
                                    .definitionId);
                            hitName = (hitDefinition != nullptr)
                                          ? hitDefinition->name.c_str()
                                          : "?";
                        }
                        // AND WHAT ITS ANIMATIONS ARE DOING. The nozzle is
                        // driven by the throttle and gated by the hand switch,
                        // and a picture of an engine seen from the wrong side
                        // cannot tell a cold nozzle from an unlit one.
                        const auto* live =
                            m_world.tryGetComponent<sw::parts::PartAnimationComponent>(
                                entity);
                        char phases[128] = {};
                        int written = 0;
                        for (sw::u32 i = 0; live != nullptr && i < live->count; ++i)
                        {
                            written += std::snprintf(
                                phases + written,
                                sizeof(phases) - static_cast<sw::usize>(written),
                                " [%u %.2f->%.2f]", i,
                                static_cast<double>(live->phase[i]),
                                static_cast<double>(live->target[i]));
                        }
                        std::fprintf(probe,
                                     "part '%s' range %.1f m radius %.2f m state %s "
                                     "offAxis %.1f deg aimedPick '%s'%s",
                                     definition->name.c_str(),
                                     static_cast<double>(glm::length(relative)),
                                     static_cast<double>(radius),
                                     hasState ? "YES" : "NO",
                                     static_cast<double>(glm::degrees(std::acos(
                                         glm::clamp(glm::dot(glm::normalize(relative),
                                                             forward),
                                                    -1.0f, 1.0f)))),
                                     hitName, aimed == entity ? " SELF" : "");
                        std::fprintf(probe, "%s\n", phases);
                    }
                });
            if (probe != nullptr)
            {
                std::fprintf(probe,
                             "%u part entities, %u with animations, %u with live state; "
                             "camera %.1f m from the ship\n",
                             parts, animated, stateful,
                             m_shipEntity.isNull()
                                 ? -1.0
                                 : glm::length(m_world
                                                   .getComponent<TransformComponent>(
                                                       m_shipEntity)
                                                   .position -
                                               eye));
                // AND THE PICK ITSELF, down the middle of the screen. The
                // cursor is at whatever the window manager left it at in a
                // headless run, so this is the pick's GEOMETRY with the one
                // input it cannot supply substituted: if a part answers here
                // and not under the mouse, the fault is the cursor, and if
                // nothing answers here the fault is the ray.
                const sw::ecs::Entity centre =
                    pickPartAlongRay(m_camera.position(), m_camera.forward());
                std::fprintf(probe, "centre-screen pick: %s\n",
                             centre.isNull() ? "nothing"
                                             : sw::parts::findDefinition(
                                                   m_world
                                                       .getComponent<
                                                           sw::parts::PartComponent>(centre)
                                                       .definitionId)
                                                   ->name.c_str());
                std::fclose(probe);
            }
        }
        const char* spec = std::getenv("SW_JUMP");
        if (spec == nullptr || *spec == '\0')
        {
            return;
        }
        m_debugJumped = true;
        std::string wanted(spec);
        for (char& c : wanted)
        {
            c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        }
        sw::i32 target = -1;
        for (sw::u32 i = 0; i < sw::space::systems().size(); ++i)
        {
            if (wanted == sw::space::systems()[i].name)
            {
                target = static_cast<sw::i32>(i);
                break;
            }
        }
        if (target < 0)
        {
            SW_LOG_WARN("Game", "SW_JUMP: no system named '{}'", wanted);
            return;
        }
        const sw::space::SystemRecord& system =
            sw::space::systems()[static_cast<sw::u32>(target)];
        const sw::space::StarRecord& primary = sw::space::stars()[system.firstStar];
        const sw::WorldVec3 destination =
            system.position + sw::WorldVec3{primary.radius * 400.0, 0.0, 0.0};
        // Move first, THEN let the streaming update notice: the origin shift
        // and the planet build both key off where the craft is.
        m_world.getComponent<TransformComponent>(controlledEntity()).position =
            localPosition(destination);
        updateSystemStreaming();
        m_world.getComponent<TransformComponent>(controlledEntity()).position =
            localPosition(destination);
        m_camera.setPosition(localPosition(destination));
        SW_LOG_INFO("Game", "SW_JUMP: {} ({:.3f} ly)", system.name,
                    system.distanceLightYears);
    }

    void StarWorksGame::applyDebugShot()
    {
        if (m_debugShotBody.empty() || m_shell != Shell::Playing)
        {
            return;
        }
        // SW_SHOT=SHIP@<radii> points at the craft rather than at a world.
        // Photographing a rocket used to be impossible: the shot camera looks
        // up a CELESTIAL body by name and a rocket is not one, so anything
        // that happens on a vessel — a solar wing swinging out, a nozzle
        // lighting up — had no picture and therefore no check.
        sw::WorldVec3 shipTarget{};
        bool shipShot = false;
        sw::ecs::Entity framed{};
        if (m_debugShotBody == "SHIP")
        {
            framed = m_shipEntity.isNull() ? controlledEntity() : m_shipEntity;
        }
        else if (m_debugShotBody == "SUNS")
        {
            // SW_SHOT=SUNS@<metres> FRAMES THE PAIR. Neither of the other two
            // modes can photograph a binary: a body shot needs a name in the
            // celestial index and the catalogue's stars are not in it under
            // theirs, and a ship shot points at the craft, which is parked
            // beside ONE of the two. So this asks collectSunsHere which stars
            // are suns from here, takes the two brightest, and stands off
            // their midpoint along a line PERPENDICULAR to the one joining
            // them — the one direction that guarantees both are in frame and
            // separated in it, rather than one hiding behind the other.
            std::vector<sw::ecs::Entity> suns;
            collectSunsHere(m_camera.position(), suns);
            if (suns.size() < 2)
            {
                framed = suns.empty() ? m_lightStar : suns.front();
            }
            else
            {
                const auto* a = m_world.tryGetComponent<TransformComponent>(suns[0]);
                const auto* b = m_world.tryGetComponent<TransformComponent>(suns[1]);
                if (a != nullptr && b != nullptr)
                {
                    const sw::WorldVec3 midpoint = (a->position + b->position) * 0.5;
                    sw::Vec3 axis = sw::Vec3(b->position - a->position);
                    const sw::f32 span = glm::length(axis);
                    axis = (span > 1.0f) ? axis / span : sw::Vec3{1.0f, 0.0f, 0.0f};
                    const sw::Vec3 reference = (std::abs(axis.y) < 0.9f)
                                                   ? sw::Vec3{0.0f, 1.0f, 0.0f}
                                                   : sw::Vec3{1.0f, 0.0f, 0.0f};
                    const sw::Vec3 offset =
                        glm::normalize(glm::cross(axis, reference));
                    const sw::WorldVec3 eye =
                        midpoint + sw::WorldVec3(offset) * m_debugShotRadii;
                    m_camera.setPosition(eye);
                    m_camera.setOrientation(glm::quatLookAt(
                        glm::normalize(sw::Vec3(midpoint - eye)),
                        sw::Vec3{0.0f, 1.0f, 0.0f}));
                    m_mapView = m_debugShotMap;
                    return;
                }
            }
        }
        else if (m_debugShotBody == "PART")
        {
            // SW_SHOT=PART@<metres> frames the part whose menu is open, not
            // the vessel. A sixty-four-metre ring photographed from its own
            // origin is a hole with hardware round the edge: the one module
            // the picture is about is thirty metres off to one side and half
            // the frame away from where the camera is looking.
            framed = m_menuPart;
        }
        if (!framed.isNull() || m_debugShotBody == "SHIP" ||
            m_debugShotBody == "PART" || m_debugShotBody == "SUNS")
        {
            const auto* transform =
                framed.isNull() ? nullptr
                                : m_world.tryGetComponent<TransformComponent>(framed);
            if (transform == nullptr)
            {
                return; // nothing to frame yet — try again next frame
            }
            // AT THE POSE IT IS DRAWN AT, not the pose it is simulated at —
            // see renderPosition. The shot camera used the raw one, and at
            // nine kilometres a second that is up to two hundred metres of
            // disagreement, which is why a ship framed from twenty-four
            // metres came back sitting in the corner of the picture.
            shipTarget = renderPosition(framed);
            shipShot = true;
        }
        sw::i32 found = -1;
        for (sw::usize i = 0; !shipShot && i < m_celestialIndex.size(); ++i)
        {
            std::string candidate = m_celestialIndex.body(i).name;
            for (char& c : candidate)
            {
                c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            }
            if (m_debugShotBody == candidate)
            {
                found = static_cast<sw::i32>(i);
                break;
            }
        }
        if (found < 0 && !shipShot)
        {
            return;
        }
        const sw::f64 seconds = m_physicsLane->presentSeconds();
        const sw::WorldVec3 target =
            shipShot ? shipTarget : m_celestialIndex.positionAt(found, seconds);
        // A vessel has no body radius, so the "radii" field means METRES for
        // a ship shot — which is the unit a rocket is measured in anyway.
        const sw::f64 radius =
            shipShot ? 1.0
                     : m_celestialIndex.body(static_cast<sw::usize>(found)).bodyRadius;

        // THE SUN OVER THE CAMERA'S SHOULDER, and then turned off it. A body
        // photographed from exactly in front of the sun is a flat disc with
        // no terminator on it; forty degrees round puts the shadow where a
        // shape can be read from it, which is the whole point of the picture.
        sw::WorldVec3 sunPosition{0.0};
        if (const auto* sol = m_world.tryGetComponent<TransformComponent>(m_solEntity))
        {
            sunPosition = sol->position;
        }
        sw::Vec3 toSun = sw::Vec3(sunPosition - target);
        if (glm::length(toSun) < 1.0f)
        {
            toSun = sw::Vec3{1.0f, 0.0f, 0.0f};
        }
        toSun = glm::normalize(toSun);
        const sw::Vec3 sideways = glm::normalize(
            glm::cross(toSun, std::abs(toSun.y) < 0.9f ? sw::Vec3{0, 1, 0}
                                                       : sw::Vec3{1, 0, 0}));
        const sw::Vec3 up = glm::cross(sideways, toSun);
        if (shipShot)
        {
            // A SHIP IS NOT A PLANET and cannot be framed like one. The offset
            // below is computed from the SUN's direction, which on a launch
            // pad at dawn points along the ground and puts the camera under
            // it: three captures in a row came back as the inside of a
            // terrain patch. A vessel is framed from its own frame instead —
            // off one shoulder, slightly above, looking at it.
            const auto* shipTransform =
                m_world.tryGetComponent<TransformComponent>(framed);
            const sw::Quat orientation =
                (shipTransform != nullptr) ? shipTransform->rotation : sw::Quat{1, 0, 0, 0};
            const sw::Vec3 right = orientation * sw::Vec3{1.0f, 0.0f, 0.0f};
            const sw::Vec3 up = orientation * sw::Vec3{0.0f, 1.0f, 0.0f};
            const sw::Vec3 back = orientation * sw::Vec3{0.0f, 0.0f, 1.0f};
            const sw::f32 yaw = m_debugShotYaw;
            const sw::Vec3 direction = glm::normalize(
                right * std::cos(yaw) + back * std::sin(yaw) + up * m_debugShotPitch);
            const sw::WorldVec3 eye =
                target + sw::WorldVec3(direction) * m_debugShotRadii;
            m_camera.setPosition(eye);
            m_camera.setOrientation(glm::quatLookAt(
                glm::normalize(sw::Vec3(target - eye)), sw::Vec3{0.0f, 1.0f, 0.0f}));
            m_mapView = m_debugShotMap;
            return;
        }
        const sw::f32 turn = 0.70f + m_debugShotYaw; // ~40 degrees, plus the caller's
        const sw::Vec3 offset = glm::normalize(toSun * std::cos(turn) +
                                               sideways * std::sin(turn) + up * 0.22f);
        const sw::WorldVec3 eye =
            target + sw::WorldVec3(offset) * (radius * m_debugShotRadii);

        const sw::Vec3 forward = glm::normalize(sw::Vec3(target - eye));
        sw::Vec3 reference{0.0f, 1.0f, 0.0f};
        if (std::abs(glm::dot(forward, reference)) > 0.99f)
        {
            reference = sw::Vec3{1.0f, 0.0f, 0.0f};
        }
        const sw::Vec3 right = glm::normalize(glm::cross(forward, reference));
        const sw::Vec3 viewUp = glm::cross(right, forward);
        const sw::Quat orientation = glm::quat_cast(sw::Mat3{right, viewUp, -forward});

        sw::Camera& camera = m_debugShotMap ? m_mapCamera : m_camera;
        camera.setPosition(eye);
        camera.setOrientation(orientation);
        camera.setAspectRatio(renderer().aspectRatio());
        m_mapView = m_debugShotMap;
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
            // A CAPTURE RUN SKIPS THE MENU. See applyDebugShot: SW_SHOT names
            // a body to point the camera at, and it exists so that a change
            // to the look of a planet can be LOOKED AT, headless, in the real
            // renderer, instead of being argued about from a CPU preview that
            // only ever runs the per-fragment path.
            if (!m_debugShotBody.empty())
            {
                newGame();
            }
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

    void StarWorksGame::applyCreativeMode()
    {
        if (m_thrustSystem != nullptr)
        {
            m_thrustSystem->setInfiniteFuel(m_creativeMode);
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
            m_mapFocusIndex = -1; // a new world, the old focus means nothing
            buildScene();
            m_celestialIndex.rebuild(m_world);
            m_lastPredictionSeconds = -1.0e9;
            m_grassCenterDir = sw::Vec3(0.0f);
        }
        m_simulation.setPaused(false);
        m_hasSession = true;
        m_shell = Shell::Playing;
        m_shellStatus.clear();
        applyCreativeMode();
        SW_LOG_INFO("Game", "New game{}", m_creativeMode ? " (CREATIVE)" : "");
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
            case 2006: m_creativeMode = !m_creativeMode; return;
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
        // menu, and this motion is presentation, not state. It OSCILLATES
        // (+-20 deg around the photogenic angle) rather than orbiting: a
        // full lap spends half its time on the night side, where a title
        // screen does not look moody, it looks broken.
        m_menuOrbitAngle += deltaSeconds;

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
        const sw::f32 azimuth =
            0.95f + 0.35f * std::sin(m_menuOrbitAngle * 0.012f); // ~54 +-20 deg
        sw::Vec3 direction =
            sunAzimuth * std::cos(azimuth) + east * std::sin(azimuth);
        direction = glm::normalize(direction + pole * 0.20f); // a little north

        const sw::f64 orbitRadius = kTerraRadius * 2.05; // ~6,700 km up // ~6,700 km up
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
                // The mode the NEXT new game starts in. Mid-session the row
                // only reports: the mode is part of the save, and flipping a
                // survival world to creative from the pause menu is the kind
                // of decision a save file should not have made behind it.
                row(m_creativeMode ? "MODE - CREATIVE - NO FUEL BURN"
                                   : "MODE - SURVIVAL",
                    2006u, !m_hasSession);
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
