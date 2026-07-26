#pragma once

// ============================================================================
// StarWorksGame.hpp
// The game layer. Milestone 10: the hierarchical star system.
//
//  - Sol -> Terra (-> Luna) / Mars, every orbit real-scale and analytic.
//    Celestials move on parent-relative Kepler rails (CelestialMotionSystem);
//    the station, ship and asteroid live in Terra's SOI and follow it
//    around the Sun.
//  - KSP-style PATCHED-CONICS flight plan: the map draws the controlled
//    craft's trajectory as colored patches around each successive primary,
//    with encounter / SOI-exit / impact markers, and the HUD calls out the
//    next event ("ENC LUNA T-…"). Powered by space::predictTrajectory.
//  - The map (M) recenters on the current SOI primary and zooms from LEO
//    out to the whole Sol system.
//  - HUD speed/altitude are measured relative to the current SOI primary.
// ============================================================================

#include "Components.hpp"

#include <Engine.hpp>

#include <array>
#include <atomic>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace game
{
    class StarWorksGame final : public sw::Application
    {
    public:
        explicit StarWorksGame(const sw::ApplicationConfig& config);
        /// Waits for the terrain-patch job before any member it writes into
        /// can be destroyed. The thread pool lives in the base class, so it
        /// outlives these members — without this, a build finishing during
        /// shutdown would write into freed memory.
        ~StarWorksGame() override;

    protected:
        void onUpdate(sw::f32 deltaSeconds) override;
        void onRender() override;

    private:
        [[nodiscard]] sw::u32 registerMesh(sw::Mesh mesh);
        /// surfaceStyle: -1 flat color, otherwise a SurfaceStyle id (see
        /// colorizeCelestial in the .cpp) painting continents/craters/rust.
        [[nodiscard]] CelestialLodComponent makeSphereLodSet(const sw::Vec4& color,
                                                             sw::i32 surfaceStyle = -1);
        void buildScene();
        /// Puts one building on a body: the ONE path, used by the scene
        /// builder and by the player's ground build cursor alike.
        /// `direction` is a unit vector in the body's rotating frame.
        sw::ecs::Entity placeBuilding(sw::u32 definitionId, sw::ecs::Entity body,
                                      const sw::Vec3& direction, sw::f32 yawRadians,
                                      sw::u32 recipeId, sw::ecs::Entity site,
                                      const sw::Vec4& marker = {});
        void buildGlyphMeshes();
        void updateWarp();
        void updateShipControls();
        void toggleEva();
        void updateChaseCamera(sw::f32 deltaSeconds);
        void collectDrawItems(const sw::Camera& activeCamera, bool mapView);
        void collectMapTrajectories(const sw::Camera& activeCamera);
        void collectHud();
        /// Screen pointers for every navigation beacon in the world: a
        /// reticle at the beacon, its name, and the live distance under it.
        /// Always drawn in map view; in flight only inside the beacon's own
        /// range. See BeaconComponent.
        void collectBeacons(const sw::Camera& activeCamera, bool mapView);
        void collectNavball();
        /// The building catalogue (F): a clickable list of every .swpart
        /// with an industrial block, with the selected one's specs.
        void collectBuildMenu();
        /// Map view: the button that steps through the vessels you own.
        void collectMapButtons();
        /// THE CONVEYOR NETWORK, derived from geometry.
        ///
        /// Belt segments are ordinary buildings — the player places them one
        /// by one like everything else. What makes a RUN of them a working
        /// link is that their conveyor-out and conveyor-in ports meet, and
        /// that is a question about where things are, not about what the
        /// player intended. This walks the ports after every build and
        /// demolition and rebuilds every chain it finds.
        void rebuildConveyorNetwork();
        /// The recipe a freshly placed building starts on: the first one its
        /// category can run. F4 will let the player choose.
        [[nodiscard]] static sw::u32 defaultRecipeFor(
            sw::factory::BuildingCategory category);
        /// A body's position and rotation AS IT IS BEING DRAWN this frame —
        /// interpolated between ticks, spin in f64. Anything placed relative
        /// to a planet must use this and not the raw tick pose: the gap is a
        /// physics step of orbital motion, 595 m for Terra.
        void bodyRenderPose(sw::ecs::Entity body, sw::WorldVec3& outPosition,
                            glm::dquat& outRotation);
        /// One planned belt tile: where it stands and which way it faces.
        struct BeltTile
        {
            sw::Vec3 direction{0.0f, 0.0f, 1.0f};
            sw::f32 yawRadians = 0.0f;
        };
        /// Plans the run of belt tiles between two machines' conveyor mouths:
        /// the ONE routine, used by the preview, by the commit and by the
        /// scene builder's starting outpost.
        [[nodiscard]] sw::build::Verdict planBelt(sw::ecs::Entity body,
                                                  sw::ecs::Entity from,
                                                  sw::ecs::Entity to,
                                                  std::vector<BeltTile>& outTiles);
        /// The body-frame position of one of a building's conveyor mouths.
        [[nodiscard]] bool conveyorPortOf(sw::ecs::Entity entity, sw::parts::NodeType type,
                                          sw::WorldVec3& outLocal);
        /// F2: aims the ground build cursor and acts on click / R.
        void updateBuildCursor();
        /// The ghost, and the one line that says why it is red.
        void collectBuildGhost(const sw::Camera& activeCamera);
        /// Every footprint standing on `body`, for the overlap rule.
        [[nodiscard]] std::vector<sw::build::Footprint> footprintsOn(
            sw::ecs::Entity body);
        /// The site a new building at `direction` should join: the nearest
        /// hub on the same body, or null if there is none in reach.
        [[nodiscard]] sw::ecs::Entity siteNear(sw::ecs::Entity body,
                                               const sw::Vec3& direction);
        /// Belt decks (tiled CV-1 parts) and the crates riding them,
        /// positioned analytically from the lane clock and each link's
        /// measured throughput.
        void collectConveyors(const sw::Camera& activeCamera);
        void collectParticles(const sw::Camera& activeCamera);
        void refreshPrediction();
        /// Atmospheric heating 0..1 for a dynamic craft (0 in vacuum).
        [[nodiscard]] sw::f32 heatingFactorFor(sw::ecs::Entity entity) const;
        /// Reentry glow tint + plasma particle spawning/aging (visual only,
        /// render-frame rate).
        void updateReentryEffects(sw::f32 deltaSeconds);
        void buildNavballMeshes();
        /// World position of the controlled craft's SOI primary (index into
        /// m_celestialIndex; -1 when the index is empty).
        [[nodiscard]] sw::i32 controlledPrimaryIndex() const;
        void hudText(std::string_view text, sw::f32 x, sw::f32 y, sw::f32 heightNdc,
                     const sw::Vec4& color);
        [[nodiscard]] sw::u32 selectLodLevel(sw::f64 distance, sw::f64 worldRadius) const;
        [[nodiscard]] sw::ecs::Entity controlledEntity() const;
        [[nodiscard]] sw::WorldVec3 controlledVelocity() const;

        void buildSaveSchema();
        void saveGame();
        void loadGame();

        sw::Camera m_camera;
        sw::FreeCameraController m_cameraController;

        // Star map view (M): orbiting system camera + zoom height.
        sw::Camera m_mapCamera;
        /// Default zoom frames the LEO traffic; wheel out to see Luna.
        sw::f64 m_mapHeightMeters = 6.0e7;
        sw::f32 m_mapYaw = 0.0f;
        sw::f32 m_mapPitch = 1.52f; // ~top-down; right-drag tilts/orbits
        bool m_mapView = false;

        // Control target: ship, or EVA capsule (G). Tab = pilot/free camera.
        sw::ecs::Entity m_shipEntity{};
        sw::ecs::Entity m_capsuleEntity{}; // null until first EVA
        bool m_shipMode = true;
        bool m_evaMode = false;

        // HUD state.
        bool m_speedSurfaceRelative = false; // V toggles ORB / SRF
        std::array<sw::u32, 128> m_glyphMeshIndex{};
        sw::u32 m_capsuleMeshIndex = 0;
        sw::u32 m_markerMeshIndex = 0;
        /// Belt deck and cargo crate: both are ordinary .swpart definitions
        /// (CV-1 and CR-1), so their look is edited in Part Studio like
        /// everything else. Cached mesh slots, resolved once at startup.
        sw::u32 m_conveyorMeshIndex = 0xFFFFFFFFu;
        sw::u32 m_cargoMeshIndex = 0xFFFFFFFFu;
        /// Length of one CV-1 along its own Z, read from its collider box —
        /// the belt tiles at exactly this spacing, so a longer segment part
        /// means fewer, longer tiles with no code change.
        sw::f32 m_conveyorSegmentM = 2.0f;
        /// Top of the CV-1's deck in its own frame — where cargo rides.
        sw::f32 m_conveyorDeckHeightM = 0.66f;

        // Artificial horizon (bottom-center instrument).
        sw::u32 m_navRingMeshIndex = 0;
        sw::u32 m_navLineMeshIndex = 0;
        sw::u32 m_navDiamondMeshIndex = 0;

        // Visual environment (Milestone 12/13, overhauled in M21).
        sw::u32 m_starfieldMeshIndex = 0;
        sw::u32 m_sunHaloMeshIndex = 0;
        sw::u32 m_sunCoreMeshIndex = 0;
        sw::u32 m_particleGlowMeshIndex = 0; // soft radial-falloff billboard
        sw::u32 m_flareMeshIndex = 0;        // lens-flare ghost disc
        sw::f32 m_skyDayFactor = 0.0f;       // 0 space/night -> 1 full daylight

        // Terrain patch (Milestone 15): a local heightfield mesh under the
        // craft, extent scaled by altitude (that IS the LOD), rebuilt as the
        // craft moves.
        //
        // The build itself runs on the THREAD POOL: at landing density it is
        // ~9,400 evaluations of a 16-octave heightfield, which on the main
        // thread is a visible stutter every time the craft crosses its own
        // patch. The job only reads its captured parameters and writes the
        // pending mesh; the main thread uploads it on a later frame, so the
        // patch on screen is never the one being written.
        void updateTerrainPatch();
        void buildTerrainPatch(const sw::planet::TerrainComponent& terrain,
                               sw::i32 surfaceStyle, const sw::Vec3& centerDir,
                               sw::f64 extent, sw::f64 radius);
        enum class TerrainJob : sw::u32 { Idle, Running, Ready };
        std::atomic<TerrainJob> m_terrainJob{TerrainJob::Idle};
        sw::MeshData m_terrainPendingMesh;
        sw::WorldVec3 m_terrainPendingOrigin{0.0};
        sw::Vec3 m_terrainPendingCenterDir{0.0f};
        sw::f64 m_terrainPendingExtent = 0.0;
        sw::ecs::Entity m_terrainPendingBody{};
        sw::u32 m_terrainMeshSlot = 0xFFFFFFFFu;
        /// Two GPU meshes, used in turn: replacing the one drawn last frame
        /// would need a full device idle to be safe (see updateTerrainPatch).
        sw::u32 m_terrainMeshSlots[2] = {0xFFFFFFFFu, 0xFFFFFFFFu};
        sw::u32 m_terrainSlotIndex = 0;
        bool m_terrainVisible = false;
        sw::ecs::Entity m_terrainBody{};
        sw::Vec3 m_terrainCenterDir{0.0f};  // body-frame patch center
        sw::f64 m_terrainExtent = 0.0;      // half-size, meters
        sw::WorldVec3 m_terrainOriginLocal{0.0}; // body-frame patch origin
        sw::f64 m_lastTerrainRebuildSeconds = -1.0e9;

        // The HANGAR (B) — the VAB. A fully separate view; the design is a
        // BLUEPRINT (plain data, not live entities). KSP-style MOUSE
        // construction: pick a part from the palette (it follows the
        // cursor), STACK nodes snap magnetically, RADIAL parts glue to any
        // collider surface under the cursor, symmetry clones around the
        // stack axis, click a placed part to grab its whole subtree.
        struct OpenAttachPoint
        {
            sw::i32 partIndex = -1; // blueprint index
            sw::u8 pointIndex = 0;
            sw::Vec3 vesselPosition{0.0f}; // node pose in the vessel frame
            sw::Vec3 vesselDirection{0.0f, 0.0f, 1.0f};
            sw::f32 size = 0.6f;
        };
        struct BlueprintPart
        {
            sw::u32 definitionId = 0;
            sw::Vec3 localPosition{0.0f};
            sw::Quat localRotation{1.0f, 0.0f, 0.0f, 0.0f};
            sw::i32 parentIndex = -1;  // index into the blueprint
            sw::u8 parentPoint = 0;    // parent node index; 255 = SURFACE attach
            sw::u8 childPoint = 0;     // child node index used by the joint
            sw::i32 symmetryGroup = -1;
        };
        struct GhostState
        {
            bool active = false;  // a pose exists under the cursor
            bool valid = false;   // and it passes validation
            sw::Vec3 position{0.0f};
            sw::Quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
            sw::i32 parentIndex = -1;
            sw::u8 parentPoint = 255; // 255 = surface attach
            sw::u8 childPoint = 0;
        };
        void enterEditor();
        void exitEditor();
        void updateEditor();
        void collectEditorUi();
        void collectHangarItems();
        void hangarNewBlueprint();
        void hangarLoadNextVessel();
        void hangarBuild();
        [[nodiscard]] std::vector<OpenAttachPoint> openAttachPoints();
        /// Cursor ray in the BLUEPRINT frame (hangar display rotation undone).
        void editorCursorRay(sw::Vec3& outOrigin, sw::Vec3& outDirection);
        void computeGhost();   // ghost pose + validation for the held part
        void commitGhost();    // place the held part (and symmetry clones)
        void grabPartAt(sw::usize index); // lift a subtree into the hand
        [[nodiscard]] sw::f64 partWetMassKg(sw::u32 definitionId) const;
        [[nodiscard]] sw::ecs::Entity instantiateBlueprint(sw::ecs::Entity existingRoot);
        void cyclePilotedVessel();
        bool m_editorMode = false;
        bool m_pausedBeforeEditor = false;
        std::vector<BlueprintPart> m_blueprint;
        sw::ecs::Entity m_hangarSource{}; // vessel the blueprint was loaded from
        // The hand: a fresh part from the palette, or a grabbed subtree
        // (stored relative to its root part; backup allows ESC restore).
        sw::u32 m_heldDefinition = 0; // 0 = empty hand
        std::vector<BlueprintPart> m_heldSubtree;
        std::vector<BlueprintPart> m_blueprintBackup;
        sw::Quat m_heldRotation{1.0f, 0.0f, 0.0f, 0.0f};
        GhostState m_ghost;
        sw::u32 m_symmetryCount = 1; // 1/2/3/4/6/8, radial placements only
        sw::i32 m_symmetryNextGroup = 0;
        bool m_showCenters = true; // CoM / thrust markers

        // ---- THE BUILD MENU (F) — groundwork for F2 --------------------------
        // A Satisfactory-style catalogue of BUILDINGS, opened on foot or from
        // the cockpit. Picking one arms it: `m_heldBuilding` is the definition
        // id F2's ground placement will read, and the same id is what a saved
        // hotbar will one day store. The menu itself is only a view over the
        // .swpart catalogue — nothing about a building is described twice.
        bool m_buildMenu = false;
        sw::u32 m_heldBuilding = 0; // 0 = empty hand
        sw::u32 m_buildMenuPage = 0;

        // ---- F2: THE GROUND BUILD CURSOR ------------------------------------
        // With a building armed, on foot, the ghost follows where you LOOK —
        // a ray against the same heightfield the collider reads — inside a
        // reach you have to walk to extend. Everything about whether it may
        // stand there comes from build::validatePlacement, so the green you
        // see and the placement that commits agree by construction.
        struct BuildCursor
        {
            bool active = false;
            sw::ecs::Entity body{};             // what you are standing on
            sw::Vec3 direction{0.0f, 0.0f, 1.0f}; // body frame, unit
            sw::f32 yawRadians = 0.0f;
            sw::f64 rangeM = 0.0;
            sw::build::Verdict verdict = sw::build::Verdict::NoGround;
            /// The building under the cursor: what R demolishes, and what
            /// the belt tool picks as an endpoint.
            sw::ecs::Entity target{};
        };
        BuildCursor m_buildCursor;
        sw::f32 m_buildYaw = 0.0f; // sticky between placements

        // ---- the BELT TOOL --------------------------------------------------
        // A conveyor is not placed tile by tile: you pick the machine that
        // SHIPS and the machine that RECEIVES, and the run is laid between
        // their mouths. That is the operation the player actually has in
        // mind — "feed this from that" — and the segments are its result,
        // not its input. They are still ordinary buildings afterwards, and
        // the network is still derived from where their ports ended up, so
        // nothing about the rest of the system had to change.
        sw::ecs::Entity m_beltSource{}; // picked, waiting for a destination
        std::vector<BeltTile> m_beltPreview;
        sw::build::Verdict m_beltVerdict = sw::build::Verdict::NoGround;
        /// Longest run the tool will lay in one go. A belt across a continent
        /// is a thousand entities and almost certainly a misclick.
        static constexpr sw::f64 kMaxBeltLengthM = 250.0;
        /// Reach, in metres. Satisfactory's lesson: a factory is built by
        /// walking around it, so the cursor stops where your arm does.
        static constexpr sw::f64 kBuildRangeM = 30.0;
        sw::Camera m_hangarCamera;
        sw::f32 m_hangarYaw = 0.6f;
        sw::f32 m_hangarPitch = 0.25f;
        sw::f32 m_hangarDistance = 28.0f;
        sw::u32 m_hangarFloorMeshIndex = 0;
        std::unordered_map<sw::u32, sw::u32> m_partMeshIds; // catalog id -> mesh slot

        // SAS + clickable HUD buttons.
        sw::u32 m_sasMode = 0; // mirrors the ship's SasComponent
        struct HudButton
        {
            sw::f32 x0, y0, x1, y1; // NDC (y down)
            sw::u32 id;
        };
        std::vector<HudButton> m_hudButtons; // rebuilt each frame
        void collectSasButtons();
        void handleHudClicks();

        // Visual particles: reentry plasma streaks and engine exhaust.
        struct ReentryParticle
        {
            sw::WorldVec3 position{0.0};
            sw::WorldVec3 velocity{0.0};
            sw::f32 life = 0.0f;    // seconds remaining
            sw::f32 maxLife = 1.0f;
            sw::f32 size = 1.0f;    // meters (cross-section)
            sw::Vec3 streakDirection{0.0f, 0.0f, 1.0f}; // elongation axis
            sw::f32 stretch = 1.0f; // length multiplier along the axis
            sw::u8 kind = 0;        // 0 = plasma, 1 = engine exhaust
        };
        std::vector<ReentryParticle> m_particles;
        sw::f32 m_shipHeat = 0.0f;
        sw::f32 m_capsuleHeat = 0.0f;
        sw::f32 m_particleSpawnDebt = 0.0f;
        sw::u32 m_particleSeed = 0x9E3779B9u;

        // Chase camera reference frame: 0 = INERTIAL (the world axes), 1 =
        // the local HORIZON frame. Eased between the two so crossing the
        // altitude threshold levels the view instead of snapping it.
        //
        // The craft's own attitude is not one of the options, on purpose: a
        // camera that inherits it turns every roll and every SAS correction
        // into a camera move. Levelling on the horizon is the one automatic
        // behaviour left, because it is about the WORLD, not the vehicle.
        sw::f32 m_groundCamBlend = 0.0f;
        // User orbiting of the chase camera (right-drag; sticky, C resets)
        // and wheel zoom (distance multiplier on the base chase offset).
        // These are the ONLY things that aim the chase camera.
        sw::f32 m_chaseYaw = 0.0f;
        sw::f32 m_chasePitch = 0.0f;
        sw::f32 m_chaseZoom = 1.0f;

        // Time warp (','/'.').
        sw::u32 m_warpIndex = 0;

        // Celestial hierarchy snapshot + patched-conics flight plan.
        sw::space::CelestialIndex m_celestialIndex;
        std::vector<sw::space::TrajectorySegment> m_prediction;
        sw::f64 m_lastPredictionSeconds = -1.0e9;
        sw::f64 m_lastDockCheckSeconds = 0.0;

        // Maneuver node (KSP-style planned burn). Edited in map view:
        // N create/delete, J/L time, I/K prograde, U/O normal, H/Y radial
        // (Shift x10, Ctrl x0.1).
        bool m_nodeActive = false;
        sw::f64 m_nodeTime = 0.0;      // absolute simulation seconds
        sw::f64 m_nodePrograde = 0.0;  // m/s
        sw::f64 m_nodeNormal = 0.0;
        sw::f64 m_nodeRadial = 0.0;
        std::vector<sw::space::TrajectorySegment> m_nodePrediction;
        sw::WorldVec3 m_nodePostBurnVelocity{0.0}; // world, at node time
        sw::i32 m_nodePrimaryIndex = -1;
        sw::WorldVec3 m_nodeRelativePosition{0.0}; // primary-relative, node time
        void updateManeuverNodeInput();
        sw::ecs::Entity m_solEntity{};
        sw::ecs::Entity m_terraEntity{};

        std::vector<sw::Mesh> m_meshes;
        sw::ecs::World m_world;
        sw::sim::Simulation m_simulation;
        sw::sim::SimulationLane* m_physicsLane = nullptr; // cached, owned by m_simulation
        sw::ecs::EntityCommandBuffer m_commands;
        sw::phys::SimulationBubbleSystem* m_bubbleSystem = nullptr;

        std::vector<sw::DrawItem> m_drawItems;
        sw::f64 m_lastStatsLogSeconds = 0.0;

        // Save/load (F5/F9): component schema with stable names.
        sw::save::Schema m_saveSchema;
    };
} // namespace game
