// ============================================================================
// GameSaveLoad.cpp — Save schema, quicksave/quickload, named saves on disk.
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

    void StarWorksGame::buildSaveSchema()
    {
        // Stable names + versions. Bump a version whenever the struct layout
        // changes; the loader refuses mismatches instead of guessing.
        m_saveSchema.registerComponent<sw::TransformComponent>("sw.Transform", 1);
        m_saveSchema.registerComponent<sw::PreviousTransformComponent>(
            "sw.PreviousTransform", 1);
        // v2: + the body-frame angular velocity, which moved down here from
        // the game's ship when the atmosphere became able to spin things.
        m_saveSchema.registerComponent<sw::phys::DynamicBodyComponent>("phys.DynamicBody",
                                                                       2);
        m_saveSchema.registerComponent<sw::phys::GroundHullComponent>("phys.GroundHull",
                                                                      1);
        // THE COLLISION HULLS, and the tag that says who gets pushed out of
        // them. Both are added by the scene builder — every building carries
        // a HullComponent straight from its .swpart's hitboxes, and the EVA
        // suit carries the mover tag — so both are in the world from the
        // first frame, and a save could not be written without them.
        // Trivially copyable fixed-size arrays, like every other component
        // here: the snapshot memcpy's whole columns.
        m_saveSchema.registerComponent<sw::phys::HullComponent>("phys.Hull", 1);
        m_saveSchema.registerComponent<sw::phys::HullMoverComponent>("phys.HullMover", 1);
        // v2: primary-relative orbit + primary handle + dynamic payload.
        m_saveSchema.registerComponent<sw::phys::OnRailsComponent>("phys.OnRails", 2);
        // v4: + the f64 spin state (axis, angle, previous angle) that keeps
        // planet-radius offsets from shimmering.
        m_saveSchema.registerComponent<sw::phys::GravitySourceComponent>(
            "phys.GravitySource", 4);
        m_saveSchema.registerComponent<sw::space::CelestialBodyComponent>(
            "space.CelestialBody", 1);
        m_saveSchema.registerComponent<sw::phys::AtmosphereComponent>("phys.Atmosphere", 1);
        m_saveSchema.registerComponent<sw::phys::SurfaceAnchorComponent>(
            "phys.SurfaceAnchor", 2); // v2: local rotation + auto-release payload
        m_saveSchema.registerComponent<sw::planet::TerrainComponent>("planet.Terrain", 3);
        m_saveSchema.registerComponent<sw::planet::DepositComponent>("planet.Deposits",
                                                                    1);
        // v2: + surfaceRelative, and kStability joined the modes.
        m_saveSchema.registerComponent<SasComponent>("game.Sas", 2);
        m_saveSchema.registerComponent<sw::parts::PartComponent>("parts.Part", 1);
        // A panel caught halfway through opening reloads halfway through
        // opening. Unregistered, this would not be a lost animation — it would
        // be a game that cannot write a save at all.
        m_saveSchema.registerComponent<sw::parts::PartAnimationComponent>(
            "parts.PartAnimation", 1);
        m_saveSchema.registerComponent<sw::parts::PartFlexComponent>(
            "sw.PartFlex", 1);
        // v2: + centre of mass, inertia and hull extents — what the
        // aerodynamics needs to turn a moment into a rotation.
        m_saveSchema.registerComponent<sw::parts::VesselComponent>("parts.Vessel", 2);
        // F6 — the air's answer. Recomputed every tick, so it is saved only
        // to keep the component ON the entity across a reload: a vessel that
        // came back without one would silently fall back to isotropic drag.
        m_saveSchema.registerComponent<sw::aero::AeroStateComponent>("aero.State", 1);
        m_saveSchema.registerComponent<sw::parts::JointComponent>("parts.Joint", 1);
        m_saveSchema.registerComponent<sw::factory::InventoryComponent>("factory.Inventory",
                                                                        1);
        m_saveSchema.registerComponent<sw::factory::MinerComponent>("factory.Miner", 1);
        m_saveSchema.registerComponent<sw::factory::RefineryComponent>("factory.Refinery",
                                                                       1);
        // v2: an ARRAY of channels — a machine can be fed more than one
        // good, which is what the fuel chain's synthesiser needs.
        m_saveSchema.registerComponent<sw::factory::ItemLinkComponent>("factory.ItemLink",
                                                                       2);
        // F1 — the data-driven industry.
        m_saveSchema.registerComponent<sw::factory::BuildingComponent>("factory.Building",
                                                                       1);
        m_saveSchema.registerComponent<sw::factory::RecipeStateComponent>(
            "factory.RecipeState", 1);
        // F3 — the grid. v2: + actualProducedKw, priority. v3: + gridId and
        // the grid's books, which arrived with the cables.
        m_saveSchema.registerComponent<sw::factory::PowerComponent>("factory.Power", 3);
        // v2 of Site: + batteryFlowKw.
        m_saveSchema.registerComponent<sw::factory::SiteComponent>("factory.Site", 2);
        m_saveSchema.registerComponent<sw::factory::BatteryComponent>("factory.Battery", 1);
        // THE CABLES. The link is what is stored — its two ends — because
        // unlike a belt there is no intermediate object to derive it from.
        // The CableComponent's curve is NOT authoritative: rebuildPowerNetwork
        // re-hangs it from the endpoints after every load.
        m_saveSchema.registerComponent<sw::factory::PowerLinkComponent>(
            "factory.PowerLink", 1);
        m_saveSchema.registerComponent<CableComponent>("game.Cable", 1);
        // v2: + nearRangeM (the pointer steps aside once you have arrived).
        m_saveSchema.registerComponent<sw::factory::BeaconComponent>("factory.Beacon", 2);
        // THE ASSEMBLY HALL'S TWO COMPONENTS, and the reason F5 was failing
        // outright. Both are added to every VAB — so to the starting outpost,
        // on the first frame of a new game — and neither was registered here.
        // save::saveWorld walks the archetypes and throws "Save schema is
        // missing a component" on the first one it cannot name, which meant
        // NO save could ever be written: not a corrupted file, no file at
        // all, and only a single "Save failed" line in the log to show for
        // it. A component that is added to a live entity and not registered
        // is a broken save, always; see SaveTests.
        m_saveSchema.registerComponent<sw::factory::AssemblyComponent>("factory.Assembly",
                                                                       1);
        m_saveSchema.registerComponent<sw::factory::VehicleQueueComponent>(
            "factory.VehicleQueue", 1);
        m_saveSchema.registerComponent<BoundsComponent>("game.Bounds", 1);
        m_saveSchema.registerComponent<SpinComponent>("game.Spin", 1);
        m_saveSchema.registerComponent<MeshComponent>("game.Mesh", 2); // v2: transparent
        m_saveSchema.registerComponent<CloudLayerComponent>("game.CloudLayer", 1);
        m_saveSchema.registerComponent<CelestialLodComponent>("game.CelestialLod",
                                                              2); // v2: surfaceStyle
        // v2: angular velocity moved to phys.DynamicBody.
        m_saveSchema.registerComponent<ShipComponent>("game.Ship", 2);
        // v2: + strafeAxis (the EVA sidestep).
        m_saveSchema.registerComponent<ShipControlsComponent>("game.ShipControls", 2);
        m_saveSchema.registerComponent<CapsuleComponent>("game.Capsule", 1);
        m_saveSchema.registerComponent<MapMarkerComponent>("game.MapMarker", 1);
        // Every star in the twelve-light-year catalogue carries one, and the
        // save refuses to write a world containing a component it does not
        // know — which is how this one was found, loudly, on the first frame
        // after the catalogue went in.
        m_saveSchema.registerComponent<StarVisualComponent>("game.StarVisual", 1);
        m_saveSchema.registerComponent<ConveyorComponent>("game.Conveyor", 2); // v2: source
    }

    std::filesystem::path StarWorksGame::savesDirectory() const
    {
        return sw::FileSystem::executableDirectory() / "Saves";
    }

    std::filesystem::path StarWorksGame::quickSavePath() const
    {
        // Beside the executable, where it has always been: an existing
        // quicksave must keep working across this change.
        return sw::FileSystem::executableDirectory() / "starworks.sav";
    }

    void StarWorksGame::refreshSaveSlots()
    {
        m_saveSlots.clear();
        std::error_code errorCode;

        const auto describe = [](const std::filesystem::path& path, bool quick) {
            SaveSlot slot;
            slot.quick = quick;
            slot.path = path;
            slot.name = quick ? std::string("QUICKSAVE") : path.stem().string();
            std::error_code code;
            slot.bytes = static_cast<sw::u64>(std::filesystem::file_size(path, code));
            const auto written = std::filesystem::last_write_time(path, code);
            if (!code)
            {
                // file_time_type has no portable calendar conversion before
                // C++20's clock_cast, and MSVC and libstdc++ disagree about
                // which epoch it counts from — so the age is shown relative
                // to now, which needs no epoch at all and is what somebody
                // picking a save actually wants to know.
                const auto age = decltype(written)::clock::now() - written;
                const auto minutes =
                    std::chrono::duration_cast<std::chrono::minutes>(age).count();
                if (minutes < 1) { slot.when = "JUST NOW"; }
                else if (minutes < 60) { slot.when = std::format("{} MIN AGO", minutes); }
                else if (minutes < 60 * 24)
                {
                    slot.when = std::format("{} H AGO", minutes / 60);
                }
                else { slot.when = std::format("{} D AGO", minutes / (60 * 24)); }
            }
            return slot;
        };

        if (std::filesystem::exists(quickSavePath(), errorCode))
        {
            m_saveSlots.push_back(describe(quickSavePath(), true));
        }
        if (std::filesystem::is_directory(savesDirectory(), errorCode))
        {
            for (const auto& entry :
                 std::filesystem::directory_iterator(savesDirectory(), errorCode))
            {
                if (entry.is_regular_file(errorCode) && entry.path().extension() == ".sav")
                {
                    m_saveSlots.push_back(describe(entry.path(), false));
                }
            }
        }
        // Newest first: the one you want is almost always the last one you
        // made. The quicksave sorts with the rest rather than being pinned,
        // because a quicksave from last week is not more interesting than a
        // named save from five minutes ago.
        std::sort(m_saveSlots.begin(), m_saveSlots.end(),
                  [](const SaveSlot& a, const SaveSlot& b) {
                      std::error_code code;
                      return std::filesystem::last_write_time(a.path, code) >
                             std::filesystem::last_write_time(b.path, code);
                  });
    }

    void StarWorksGame::saveGame()
    {
        saveGameTo(quickSavePath());
    }

    void StarWorksGame::loadGame()
    {
        loadGameFrom(quickSavePath());
    }

    void StarWorksGame::saveGameTo(const std::filesystem::path& path)
    {
        sw::ser::BinaryWriter writer;
        writer.write<sw::u32>(0x53575347); // "SWSG"
        // Game save version. Component CONTENT changes ride on the
        // per-component versions in the schema; this number moves when the
        // SESSION block below changes shape. v10: + the creative-mode flag.
        // v11: + the origin system, which is the one number that decides what
        // every position in the file MEANS.
        writer.write<sw::u32>(11);

        sw::save::saveWorld(m_world, m_saveSchema, writer);
        sw::save::saveSimulation(m_simulation, writer);

        // Player/session state.
        writer.write(m_shipEntity);
        writer.write(m_capsuleEntity);
        writer.write(static_cast<sw::u8>(m_evaMode ? 1 : 0));
        writer.write(static_cast<sw::u8>(m_shipMode ? 1 : 0));
        writer.write(static_cast<sw::u8>(m_speedSurfaceRelative ? 1 : 0));
        writer.write(m_warpIndex);
        // WHICH STAR THE ORIGIN IS ON. Without it a save made at Barnard's
        // Star reloads with every position interpreted against Sol — the ship
        // would be six light-years from where it was parked, and nothing in
        // the file would look wrong. Sol is zero, so a save written before the
        // catalogue existed reads back as Sol, which is where it was.
        writer.write(m_originSystem);
        writer.write(m_mapHeightMeters);
        writer.write(m_camera.position());
        writer.write(m_camera.orientation());
        // Maneuver node (v4).
        writer.write(static_cast<sw::u8>(m_nodeActive ? 1 : 0));
        writer.write(m_nodeTime);
        writer.write(m_nodePrograde);
        writer.write(m_nodeNormal);
        writer.write(m_nodeRadial);
        writer.write(static_cast<sw::u8>(m_creativeMode ? 1 : 0)); // v10

        // The directory may not exist yet — a named save is the first thing
        // that ever writes into it.
        std::error_code errorCode;
        std::filesystem::create_directories(path.parent_path(), errorCode);
        sw::FileSystem::writeBinaryFile(path, writer.bytes());
        SW_LOG_INFO("Game", "Saved to '{}' ({} KB, {} entities, t={:.1f}s)", path.string(),
                    writer.size() / 1024, m_world.aliveCount(),
                    m_simulation.simulatedSeconds());
    }

    void StarWorksGame::loadGameFrom(const std::filesystem::path& path)
    {
        const std::vector<sw::u8> bytes = sw::FileSystem::readBinaryFile(path);
        sw::ser::BinaryReader reader(bytes);

        if (reader.read<sw::u32>() != 0x53575347)
        {
            SW_THROW("'{}' is not a StarWorks save", path.string());
        }
        const sw::u32 version = reader.read<sw::u32>();
        if (version < 9 || version > 11)
        {
            SW_THROW("Unsupported save version {}", version);
        }

        sw::save::loadWorld(m_world, m_saveSchema, reader);
        sw::save::loadSimulation(m_simulation, reader);
        m_celestialIndex.rebuild(m_world);
        m_lastPredictionSeconds = -1.0e9; // stale flight plan: recompute
        m_mapFocusIndex = -1;             // view state, not save state: AUTO

        m_shipEntity = reader.read<sw::ecs::Entity>();
        m_capsuleEntity = reader.read<sw::ecs::Entity>();
        m_evaMode = reader.read<sw::u8>() != 0;
        m_shipMode = reader.read<sw::u8>() != 0;
        m_speedSurfaceRelative = reader.read<sw::u8>() != 0;
        m_warpIndex = reader.read<sw::u32>();
        // v11 and up. Older files predate interstellar travel entirely, so
        // their positions can only ever have meant Sol.
        m_originSystem = (version >= 11) ? reader.read<sw::u32>()
                                         : sw::space::kSolSystem;
        if (m_originSystem >= sw::space::systems().size())
        {
            m_originSystem = sw::space::kSolSystem;
        }
        m_mapHeightMeters = reader.read<sw::f64>();
        m_camera.setPosition(reader.read<sw::WorldVec3>());
        m_camera.setOrientation(reader.read<sw::Quat>());
        m_nodeActive = reader.read<sw::u8>() != 0;
        m_nodeTime = reader.read<sw::f64>();
        m_nodePrograde = reader.read<sw::f64>();
        m_nodeNormal = reader.read<sw::f64>();
        m_nodeRadial = reader.read<sw::f64>();
        // v9 saves predate the mode and were all survival by definition.
        m_creativeMode = (version >= 10) && reader.read<sw::u8>() != 0;
        applyCreativeMode();
        if (const auto* sas = m_world.tryGetComponent<SasComponent>(m_shipEntity))
        {
            m_sasMode = sas->mode;
        }
        // Force a terrain patch rebuild on the next frame.
        m_lastTerrainRebuildSeconds = -1.0e9;
        m_terrainBody = {};

        if (!m_shipMode)
        {
            const sw::Vec3 forward = m_camera.forward();
            m_cameraController.setPose(
                m_camera.position(), std::atan2(-forward.x, -forward.z),
                std::asin(std::clamp(forward.y, -1.0f, 1.0f)));
        }

        // The two DERIVED networks. Neither is stored — the belts' chains
        // come from where the mouths ended up, the grids from the cables —
        // so a loaded world has to re-derive both before the first tick, or
        // it runs one frame with an empty factory and a dead grid.
        rebuildConveyorNetwork();
        rebuildPowerNetwork();
        rebuildHulls();

        SW_LOG_INFO("Game", "Loaded '{}': {} entities, t={:.1f}s, warp x{:g}",
                    path.string(), m_world.aliveCount(), m_simulation.simulatedSeconds(),
                    kWarpLadder[m_warpIndex]);
    }
} // namespace game
