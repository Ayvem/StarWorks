// ============================================================================
// GameSaveLoad.cpp — Save schema, quicksave and quickload.
// Split out of StarWorksGame.cpp; same class, one theme per translation unit.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"
#include "Systems.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
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
        // Solid-object hulls. The contents are DERIVED — loadGame() calls
        // rebuildHulls() and re-derives every box from the part catalogue —
        // but Snapshot (rightly) refuses to save a world holding any
        // unregistered component, so they are registered like everything else.
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
        // The assembly hall's order and its queue of crated designs. These were
        // added in F5 but never registered — and Snapshot refuses to save a
        // world holding an unregistered component, so F5 (quicksave) failed the
        // moment a VAB existed, which is from the starting outpost onward.
        m_saveSchema.registerComponent<sw::factory::AssemblyComponent>("factory.Assembly", 1);
        m_saveSchema.registerComponent<sw::factory::VehicleQueueComponent>(
            "factory.VehicleQueue", 1);
        // THE CABLES. The link is what is stored — its two ends — because
        // unlike a belt there is no intermediate object to derive it from.
        // The CableComponent's curve is NOT authoritative: rebuildPowerNetwork
        // re-hangs it from the endpoints after every load.
        m_saveSchema.registerComponent<sw::factory::PowerLinkComponent>(
            "factory.PowerLink", 1);
        m_saveSchema.registerComponent<CableComponent>("game.Cable", 1);
        // v2: + nearRangeM (the pointer steps aside once you have arrived).
        m_saveSchema.registerComponent<sw::factory::BeaconComponent>("factory.Beacon", 2);
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
        m_saveSchema.registerComponent<ConveyorComponent>("game.Conveyor", 2); // v2: source
    }

    void StarWorksGame::saveGame()
    {
        sw::ser::BinaryWriter writer;
        writer.write<sw::u32>(0x53575347); // "SWSG"
        // Game save version. Component CONTENT changes ride on the per-component
        // versions in the schema above; this number only moves when the session
        // block below (player state, warp, map height...) changes shape.
        writer.write<sw::u32>(9);

        sw::save::saveWorld(m_world, m_saveSchema, writer);
        sw::save::saveSimulation(m_simulation, writer);

        // Player/session state.
        writer.write(m_shipEntity);
        writer.write(m_capsuleEntity);
        writer.write(static_cast<sw::u8>(m_evaMode ? 1 : 0));
        writer.write(static_cast<sw::u8>(m_shipMode ? 1 : 0));
        writer.write(static_cast<sw::u8>(m_speedSurfaceRelative ? 1 : 0));
        writer.write(m_warpIndex);
        writer.write(m_mapHeightMeters);
        writer.write(m_camera.position());
        writer.write(m_camera.orientation());
        // Maneuver node (v4).
        writer.write(static_cast<sw::u8>(m_nodeActive ? 1 : 0));
        writer.write(m_nodeTime);
        writer.write(m_nodePrograde);
        writer.write(m_nodeNormal);
        writer.write(m_nodeRadial);

        const auto path = sw::FileSystem::executableDirectory() / "starworks.sav";
        sw::FileSystem::writeBinaryFile(path, writer.bytes());
        SW_LOG_INFO("Game", "Saved to '{}' ({} KB, {} entities, t={:.1f}s)", path.string(),
                    writer.size() / 1024, m_world.aliveCount(),
                    m_simulation.simulatedSeconds());
    }

    void StarWorksGame::loadGame()
    {
        const auto path = sw::FileSystem::executableDirectory() / "starworks.sav";
        const std::vector<sw::u8> bytes = sw::FileSystem::readBinaryFile(path);
        sw::ser::BinaryReader reader(bytes);

        if (reader.read<sw::u32>() != 0x53575347)
        {
            SW_THROW("'{}' is not a StarWorks save", path.string());
        }
        if (const sw::u32 version = reader.read<sw::u32>(); version != 9)
        {
            SW_THROW("Unsupported save version {}", version);
        }

        sw::save::loadWorld(m_world, m_saveSchema, reader);
        sw::save::loadSimulation(m_simulation, reader);
        m_celestialIndex.rebuild(m_world);
        m_lastPredictionSeconds = -1.0e9; // stale flight plan: recompute

        m_shipEntity = reader.read<sw::ecs::Entity>();
        m_capsuleEntity = reader.read<sw::ecs::Entity>();
        m_evaMode = reader.read<sw::u8>() != 0;
        m_shipMode = reader.read<sw::u8>() != 0;
        m_speedSurfaceRelative = reader.read<sw::u8>() != 0;
        m_warpIndex = reader.read<sw::u32>();
        m_mapHeightMeters = reader.read<sw::f64>();
        m_camera.setPosition(reader.read<sw::WorldVec3>());
        m_camera.setOrientation(reader.read<sw::Quat>());
        m_nodeActive = reader.read<sw::u8>() != 0;
        m_nodeTime = reader.read<sw::f64>();
        m_nodePrograde = reader.read<sw::f64>();
        m_nodeNormal = reader.read<sw::f64>();
        m_nodeRadial = reader.read<sw::f64>();
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
