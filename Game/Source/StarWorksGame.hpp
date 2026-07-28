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
        /// The maneuver node, dragged along its own orbit with the mouse.
        /// Runs after the map camera: a pick is a ray from THIS frame's eye.
        void updateNodeDrag();
        bool m_nodeDragging = false;
        /// SPACE, and Z: fire the ship's next decoupler.
        void fireNextDecoupler();
        /// SPACE on foot: one jump, taken by the next physics tick. An edge
        /// rather than a held state, so the key cannot hover.
        bool m_jumpRequested = false;
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
        /// Map view: warp forward to one minute before the maneuver node.
        void collectWarpToNodeButton();
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
        /// ...between two SPECIFIC mouths, which is what the tool uses once
        /// a machine has more than one of a kind.
        [[nodiscard]] sw::build::Verdict planBelt(sw::ecs::Entity body,
                                                  sw::ecs::Entity from,
                                                  sw::u32 fromPortIndex,
                                                  sw::ecs::Entity to, sw::u32 toPortIndex,
                                                  std::vector<BeltTile>& outTiles);
        /// The body-frame position of one of a building's conveyor mouths.
        /// The mouth of `type` at `index`, in the body frame. A machine may
        /// have several of a kind — see parts::conveyorNodes.
        [[nodiscard]] bool conveyorPortOf(sw::ecs::Entity entity, sw::parts::NodeType type,
                                          sw::u32 index, sw::WorldVec3& outLocal);
        /// The free mouth nearest where the player is aiming.
        [[nodiscard]] sw::u32 chooseConveyorPort(sw::ecs::Entity entity,
                                                 sw::parts::NodeType type,
                                                 const sw::WorldVec3& aimLocal,
                                                 bool& outAny);
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
        /// ONE SEGMENT OF A TRAJECTORY LINE: a unit box stretched between
        /// two samples of a conic. A row of dots reads as a dotted line at
        /// any zoom, which is fine for a marker and useless for an orbit —
        /// you cannot tell a trajectory that stops from one whose dots have
        /// simply spread out. Drawn this way the line is continuous, and
        /// where it ends is information.
        sw::u32 m_orbitLineMeshIndex = 0;
        /// Belt deck and cargo crate: both are ordinary .swpart definitions
        /// (CV-1 and CR-1), so their look is edited in Part Studio like
        /// everything else. Cached mesh slots, resolved once at startup.
        sw::u32 m_conveyorMeshIndex = 0xFFFFFFFFu;
        sw::u32 m_cargoMeshIndex = 0xFFFFFFFFu;
        sw::u32 m_vehicleCargoMeshIndex = 0xFFFFFFFFu;
        /// Length of one CV-1 along its own Z, read from its collider box —
        /// the belt tiles at exactly this spacing, so a longer segment part
        /// means fewer, longer tiles with no code change.
        sw::f32 m_conveyorSegmentM = 2.0f;
        /// Top of the CV-1's deck in its own frame — where cargo rides.
        sw::f32 m_conveyorDeckHeightM = 0.66f;
        /// Owned by the Physics lane; kept for its per-tick pair counts,
        /// which the HUD shows so a base that has become expensive says so.
        sw::phys::HullCollisionSystem* m_hullCollision = nullptr;
        /// Owned by the Physics lane; kept so the HUD can read the air.
        sw::aero::VesselAerodynamicsSystem* m_aerodynamics = nullptr;
        /// Turns a definition's authored hitboxes into the solid shape an
        /// entity carries. Conveyor decks and cables get none — you step
        /// over a belt, you do not climb it.
        [[nodiscard]] static bool hullFor(const sw::parts::PartDefinition& definition,
                                          sw::phys::HullComponent& outHull);
        /// What the player is looking at, by their actual hull rather than
        /// by distance to a centre: the E panel's question.
        [[nodiscard]] sw::ecs::Entity hullUnderCrosshair(sw::f64 maxDistanceM);
        /// Re-derives every entity's solid shape from its .swpart. Hulls are
        /// derived data, so they are not saved; a load rebuilds them the way
        /// it rebuilds the belt chains and the power grids.
        void rebuildHulls();
        /// E was pressed: resolve it AFTER the camera has been updated, for
        /// the same reason the ground cursor aims last.
        bool m_configRequested = false;
        /// F2: draw every solid hull as the box it actually is. Seeing the
        /// collision shape is the only way to tell an authoring mistake from
        /// an engine one — a hull that matches the model looks like nothing
        /// at all, which is exactly why it has to be lookable-at.
        bool m_showHitboxes = false;
        sw::u32 m_hullBoxMeshIndex = 0xFFFFFFFFu;
        void collectHullOverlay(const sw::Camera& activeCamera);
        /// The player's own hull, read off EV-1's hitbox at startup.
        sw::phys::GroundHullComponent m_capsuleHull{{0.0f, 0.0f, 0.0f},
                                                     {0.5f, 1.0f, 0.5f}};
        /// The CW-1 wire span, same contract as the belt tile.
        sw::u32 m_cableMeshIndex = 0xFFFFFFFFu;
        sw::f32 m_cableSegmentM = 2.0f;

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
        /// Turns `m_blueprint` into live entities. `pad`, when it is a real
        /// launch pad, is where the vessel is born — standing on that pad's
        /// deck, co-rotating with the planet under it. A null pad keeps the
        /// old surveyed place next to the outpost, which is what the
        /// hangar's BUILD test shortcut still uses.
        [[nodiscard]] sw::ecs::Entity instantiateBlueprint(sw::ecs::Entity existingRoot,
                                                           sw::ecs::Entity pad = {});
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
        /// What ONE belt can move, units per second, per good it carries.
        /// The rated capacity of a conveyor: a machine that out-produces it
        /// backs up, which is the bottleneck a factory is built around.
        static constexpr sw::f64 kConveyorRateUnitsPerSecond = 3.0;
        sw::ecs::Entity m_beltSource{}; // picked, waiting for a destination
        /// Which of the source's out mouths the pending run leaves by, and
        /// which of the destination's in mouths the preview is aiming at.
        sw::u32 m_beltSourcePort = 0;
        sw::u32 m_beltDestinationPort = 0;

        // ---- THE CABLE TOOL --------------------------------------------------
        // Same two clicks as the belt, a different question: not "feed this
        // from that" but "put these on the same grid". The rules are in
        // factory::validateCable — one wire per building, unlimited at a
        // pole — so the HUD's refusal and the commit's refusal are the same
        // sentence from the same function.
        sw::ecs::Entity m_cableSource{};
        sw::factory::CableVerdict m_cableVerdict = sw::factory::CableVerdict::NoPowerNode;
        /// Longest span. A wire is a straight line through the air, and one
        /// long enough to cross a valley would go through the hill.
        static constexpr sw::f64 kMaxCableLengthM = 120.0;
        /// How far a span droops, as a fraction of its length. Enough to read
        /// as a hanging wire, not enough to reach the ground.
        static constexpr sw::f64 kCableSagFraction = 0.045;
        bool powerNodeOf(sw::ecs::Entity entity, sw::WorldVec3& outLocal);
        void rebuildPowerNetwork();
        void hangCable(CableComponent& cable, const sw::WorldVec3& from,
                       const sw::WorldVec3& to);
        /// Both ends' rules, in the terms factory::validateCable wants them.
        [[nodiscard]] sw::factory::CableVerdict planCable(sw::ecs::Entity from,
                                                          sw::ecs::Entity to,
                                                          sw::WorldVec3& outFrom,
                                                          sw::WorldVec3& outTo);
        void layCable(sw::ecs::Entity body, sw::ecs::Entity from, sw::ecs::Entity to);
        void collectCables(const sw::Camera& activeCamera);
        void collectCableGhost(const sw::Camera& activeCamera);
        std::vector<BeltTile> m_beltPreview;
        sw::build::Verdict m_beltVerdict = sw::build::Verdict::NoGround;
        /// Longest run the tool will lay in one go. A belt across a continent
        /// is a thousand entities and almost certainly a misclick.
        static constexpr sw::f64 kMaxBeltLengthM = 250.0;
        /// Reach, in metres. Satisfactory's lesson: a factory is built by
        /// walking around it, so the cursor stops where your arm does.
        static constexpr sw::f64 kBuildRangeM = 30.0;
        // ---- F3: THE MACHINE PANEL (E) --------------------------------------
        // Walk up to a building, press E, and you are looking at what it is
        // doing: its state, its share of the grid, what is in its bin, and
        // the list of recipes its CATEGORY can run. Choosing one is the only
        // way a player builds the fuel chain, so this panel is not chrome —
        // it is the interface to the entire production system.
        //
        // It holds an ENTITY, not a copy of anything. A machine demolished
        // while its panel is open simply stops resolving, and the panel
        // closes itself on the next frame.
        sw::ecs::Entity m_configTarget{};
        /// How close you have to stand. Short on purpose: the panel is a
        /// control on the machine, not a remote.
        static constexpr sw::f64 kConfigRangeM = 18.0;
        void collectConfigMenu();
        void toggleConfigMenu();
        void applyRecipeChoice(sw::ecs::Entity entity, sw::u32 recipeId);

        // ---- F5: THE VAB AND THE PAD ----------------------------------------
        // The loop the whole factory has been building towards: a design is
        // saved in the hangar, ORDERED at an assembly hall, paid for in iron
        // and copper carried there on belts, crated, shipped to a pad, and
        // stood up on it as a real vessel with real fuel in its tanks.
        //
        // The engine owns the parts of that a test can hold still — the bill
        // of materials, the .swship file, the metal arithmetic. What lives
        // here is the part that needs the world: turning a saved design back
        // into entities, standing on a particular pad, on a spinning planet.
        /// Orders `design` at `hall`, costed from the catalogue.
        void orderVehicle(sw::ecs::Entity hall, const sw::parts::ShipBlueprint& design);
        /// The pads' own tick: unpack an arrived crate into a vessel.
        void updateLaunchPads();
        /// True while a vessel is standing on (or lifting off) this pad. A
        /// pad holds ONE rocket; the next crate waits on the belt.
        [[nodiscard]] bool padIsOccupied(sw::ecs::Entity pad);
        /// A saved design as the hangar's working list. The joints come with
        /// it, so a vessel built from a file is jointed as it was drawn.
        [[nodiscard]] static std::vector<BlueprintPart> partsFromDesign(
            const sw::parts::ShipBlueprint& design);
        [[nodiscard]] sw::parts::ShipBlueprint designFromParts(
            std::string_view name) const;
        /// Writes the current design to Assets/Ships and registers it, so it
        /// is orderable at a VAB without a restart.
        void hangarSaveShip();
        /// Pours up to `availableUnits` of fuel into a vessel's tanks;
        /// returns what actually went in.
        sw::f64 fuelVessel(sw::ecs::Entity vessel, sw::f64 availableUnits);

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
        /// A filled screen-space rectangle in NDC. Every panel, row and chip
        /// in the game goes through this one call.
        void hudQuad(sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1,
                     const sw::Vec4& color);
        /// A panel with a hairline edge — the edge is what stops a dark list
        /// from bleeding into a dark planet.
        void hudPanel(sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1,
                      const sw::Vec4& fill);
        /// The cursor in the same NDC the buttons are laid out in, so a row
        /// can light up under it. False when there is no window yet.
        [[nodiscard]] bool hudCursor(sw::f32& outX, sw::f32& outY);

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

        // ---- FLYING the burn -------------------------------------------------
        // The remaining dv has to COUNT DOWN while the engine is lit, or the
        // readout that tells you when to stop tells you nothing.
        //
        // It did not, and the reason is worth writing down: the target was
        // recomputed every refresh as "my velocity at the node, on my
        // CURRENT trajectory, plus the planned dv". Burning changes the
        // current trajectory, so the target moved with the ship and the
        // difference stayed pinned at the full dv forever.
        //
        // So near the node the plan is FROZEN: the pre-burn trajectory and
        // the dv vector are captured once, and what remains is the plan
        // minus what has actually been applied — measured against the
        // COASTING velocity from that frozen plan, which subtracts gravity's
        // own contribution over the burn (a kilometre per second in low
        // orbit; not something to hand-wave).
        static constexpr sw::f64 kBurnLockSeconds = 120.0;
        bool m_burnLocked = false;
        sw::WorldVec3 m_burnDvWorld{0.0};
        std::vector<sw::space::TrajectorySegment> m_burnCoast;
        sw::f64 m_burnNodeTime = 0.0; // what the lock was taken for
        sw::f64 m_burnPrograde = 0.0;
        sw::f64 m_burnNormal = 0.0;
        sw::f64 m_burnRadial = 0.0;
        /// The burn still to fly, as a world vector. Zero-length when there
        /// is no node. Used by the readout, the navball marker and the SAS.
        [[nodiscard]] sw::WorldVec3 remainingBurnVector();
        /// WARP TO THE NODE: absolute simulation time to stop at, or 0.
        sw::f64 m_warpToSeconds = 0.0;
        void updateManeuverNodeInput();

        // ---- THE TARGET, and how close we get to it -------------------------
        // Click a body on the map and it becomes the target. From then on
        // the plan answers the question a transfer is actually about: how
        // close do I pass, when, and WHERE WILL IT BE by then — because the
        // body you are aiming at is not where it is now, and the whole
        // difficulty of a transfer is aiming at where it is going to be.
        //
        // Both markers are drawn in the map's own frame (primary-relative),
        // so the target's future position lands on the orbit ring you can
        // see rather than in empty space along Terra's year.
        sw::i32 m_targetIndex = -1; // CelestialIndex body, -1 = none
        sw::space::ClosestApproach m_approach{};
        /// The same, for the trajectory AFTER the planned burn.
        sw::space::ClosestApproach m_nodeApproach{};
        void updateTargetPick();

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
