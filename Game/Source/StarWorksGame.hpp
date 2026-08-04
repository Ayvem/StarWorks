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
#include <filesystem>
#include <memory>
#include <string>
#include <atomic>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace game
{
    class ThrustSystem; // Systems.hpp; the game keeps a pointer (creative mode)

    /// The hangar palette's shelves (F49). Declared here rather than beside
    /// its helpers in GameInternal.hpp only because the editor has to REMEMBER
    /// which one is open, and the state lives on the game.
    enum class PaletteGroup : sw::u32
    {
        Command = 0,   // pods, cores, the instrument that flies with them
        Propulsion,    // tanks and engines
        Structure,     // what holds a rocket together and lets it let go
        Power,         // batteries and panels
        Aero,          // the things that only matter inside an atmosphere
        Endurance,     // the F15 kit: its own world, and it says so
        Count,         // ...and every shelf shut, which is a legal state
    };

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
        [[nodiscard]] CelestialLodComponent makeSphereLodSet(
            const sw::Vec4& color, sw::i32 surfaceStyle = -1,
            sw::f64 bodyRadiusMeters = 0.0);
        void buildScene();
        /// F15: assembles the ENDURANCE from its ten catalogue parts and
        /// parks it, spinning, on rails around Saturn — a flyable vessel,
        /// not scenery. Called by buildScene; a no-op when the shipped
        /// catalogue (and its 200-range ids) is missing.
        void buildEndurance();
        /// Puts one building on a body: the ONE path, used by the scene
        /// builder and by the player's ground build cursor alike.
        /// `direction` is a unit vector in the body's rotating frame.
        sw::ecs::Entity placeBuilding(sw::u32 definitionId, sw::ecs::Entity body,
                                      const sw::Vec3& direction, sw::f32 yawRadians,
                                      sw::u32 recipeId, sw::ecs::Entity site,
                                      const sw::Vec4& marker = {});
        void buildGlyphMeshes();
        void updateWarp();
        /// The SYNC-warp gate (F17): the multiplayer catch-up is the one
        /// warp that skips hours and bypasses the altitude ladder, so it is
        /// the one still refused on the shape of the trajectory.
        [[nodiscard]] bool warpAllowed() const;
        /// Why not, in three words, for the panel.
        [[nodiscard]] const char* warpBlockReason() const;
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
        /// The recipe a freshly placed building starts on. A MINE starts on
        /// the ore that is actually under it — see the note at the definition,
        /// and the thirty-nine per cent of legal sites that used to produce a
        /// machine which could never dig anything.
        [[nodiscard]] static sw::u32 defaultRecipeFor(
            sw::factory::BuildingCategory category,
            const sw::planet::DepositComponent* deposits = nullptr,
            const sw::Vec3& up = sw::Vec3{0.0f, 1.0f, 0.0f});
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
        /// Which body the map camera orbits. -1 = AUTO: follow the
        /// controlled craft's SOI primary (the default, and what the map
        /// always did). Tab cycles through every celestial body so an
        /// arrival can be framed from the TARGET's side while the node is
        /// being dragged; Shift+Tab cycles backwards, and leaving AUTO
        /// changes nothing about the craft — this is a camera, not a frame
        /// of physics.
        sw::i32 m_mapFocusIndex = -1;
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
        sw::u32 m_sunAureoleMeshIndex = 0;
        sw::u32 m_particleGlowMeshIndex = 0; // soft radial-falloff billboard
        sw::u32 m_flareMeshIndex = 0;        // lens-flare ghost disc
        sw::f32 m_skyDayFactor = 0.0f;       // 0 space/night -> 1 full daylight

        // Terrain patch (Milestone 15): a local heightfield mesh under the
        // craft, extent scaled by altitude (that IS the LOD), rebuilt as the
        // craft moves.
        //
        // The build itself runs on the THREAD POOL: at landing density it is
        // tens of thousands of evaluations of a 16-octave heightfield —
        // ~37,000 at the 192-cell landing grid — which on the main thread
        // is a visible stutter every time the craft crosses its own patch. The job only reads its captured parameters and writes the
        // pending mesh; the main thread uploads it on a later frame, so the
        // patch on screen is never the one being written.
        void updateTerrainPatch();
        void buildTerrainPatch(const sw::planet::TerrainComponent& terrain,
                               sw::i32 surfaceStyle, const sw::Vec3& centerDir,
                               sw::f64 extent, sw::f64 radius, sw::u32 cellCount,
                               const sw::Vec3& sunDirBody);
        enum class TerrainJob : sw::u32 { Idle, Running, Ready };
        std::atomic<TerrainJob> m_terrainJob{TerrainJob::Idle};
        sw::MeshData m_terrainPendingMesh;
        sw::WorldVec3 m_terrainPendingOrigin{0.0};
        sw::Vec3 m_terrainPendingCenterDir{0.0f};
        sw::f64 m_terrainPendingExtent = 0.0;
        sw::u32 m_terrainPendingCells = 0;
        sw::Vec3 m_terrainPendingEast{0.0f};
        sw::Vec3 m_terrainPendingNorth{0.0f};
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

        // ---- THE GRASS, on its own clock and its own meshes -----------------
        //
        // The field has to follow a walking player at the scale of a field —
        // forty metres, not the four hundred and fifty the ground is happy
        // with. Riding on the terrain patch made every one of those a 5 MB
        // upload on the main thread, and `uploadToBuffer` SUBMITS AND THEN
        // WAITS ON A FENCE: the wait drains whatever the graphics queue is
        // already holding, so the hitch is up to a whole frame of GPU work.
        // Eleven times more often, that is a visible spike every ten seconds
        // of walking.
        //
        // So the grass is its own geometry, built from the ground grid the
        // patch already computed, cut into chunks, and uploaded ONE CHUNK PER
        // FRAME. The stall is divided by the chunk count and spread over that
        // many frames, and the field on screen is never incomplete: the new
        // set fills the spare slots while the old one keeps drawing, and they
        // swap only when the last chunk has landed.
        static constexpr sw::u32 kGrassChunks = 6;
        void buildGrassField(const std::vector<sw::Vertex>& groundGrid, sw::u32 cellCount,
                             const sw::Vec3& centerDir, const sw::Vec3& east,
                             const sw::Vec3& north, sw::f64 extent, sw::f64 radius,
                             const sw::Vec3& fieldDir);
        std::atomic<TerrainJob> m_grassJob{TerrainJob::Idle};
        sw::MeshData m_grassPending[kGrassChunks];
        /// Two sets of chunk slots: one drawn, one being filled.
        sw::u32 m_grassSlots[2][kGrassChunks] = {
            {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu},
            {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu}};
        /// A chunk can come back empty — bare rock, open water, or simply
        /// nothing green in its sixth of the field. Its SLOT is kept (so the
        /// mesh table does not grow a new entry on every rebuild) and only
        /// its validity is cleared.
        bool m_grassChunkValid[2][kGrassChunks] = {};
        sw::u32 m_grassSet = 0;          // which set is on screen
        sw::u32 m_grassLiveCount = 0;    // chunks of that set worth drawing
        sw::u32 m_grassUploadCursor = 0; // next chunk to hand to the GPU
        sw::Vec3 m_grassCenterDir{0.0f};
        sw::Vec3 m_grassPendingCenterDir{0.0f};
        /// The patch origin the chunk vertices are relative to — the grass
        /// is drawn with the terrain's own transform, so it must share it.
        sw::WorldVec3 m_grassOriginLocal{0.0};
        sw::WorldVec3 m_grassPendingOriginLocal{0.0};
        sw::ecs::Entity m_grassBody{};
        /// The ground grid the last patch produced, kept so the grass can be
        /// re-centred without rebuilding the terrain: the field then stands
        /// on exactly the surface that is drawn, not on a second sampling of
        /// the heightfield that would put it half a metre out.
        std::vector<sw::Vertex> m_terrainGridVertices;
        sw::u32 m_terrainGridCells = 0;
        sw::Vec3 m_terrainGridEast{0.0f};
        sw::Vec3 m_terrainGridNorth{0.0f};
        void updateGrassField();

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
            /// F48: the shell drawn on this part, when it is a fairing base.
            /// Empty on everything else, which is every other part there is.
            sw::parts::FairingComponent fairing{};
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
        [[nodiscard]] std::vector<OpenAttachPoint> openAttachPoints();
        /// Cursor ray in the BLUEPRINT frame (hangar display rotation undone).
        void editorCursorRay(sw::Vec3& outOrigin, sw::Vec3& outDirection);
        void computeGhost();   // ghost pose + validation for the held part
        /// How many copies the current placement makes: the symmetry count on
        /// a fresh radial part, one otherwise. Judging, committing and drawing
        /// all ask this, so they cannot disagree about what is being placed.
        [[nodiscard]] sw::u32 ghostCloneCount() const;
        [[nodiscard]] std::vector<sw::i32> ghostParents(sw::u32 cloneCount) const;
        /// Why the ghost is red, for the hook that photographs it: the
        /// two reasons look identical on screen and are not the same bug.
        bool m_ghostCollides = false;
        bool m_ghostOverloaded = false;
        sw::i32 m_ghostBlockedBy = -1;
        /// SW_PLACE=<defId>[,<symmetry>[,<azimuthDeg>[,<height>]]]: put a part
        /// on the design without a mouse, by aiming the editor's own ray at
        /// the stack and pressing the real commit. Shouts when the placement
        /// is refused, because a hook that quietly places nothing photographs
        /// the same empty deck a broken editor would.
        void debugPlacePart(const char* spec);
        bool m_editorRayScripted = false;
        sw::Vec3 m_editorRayOrigin{0.0f};
        sw::Vec3 m_editorRayDirection{0.0f, 0.0f, 1.0f};
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
        /// Stops the piloted vessel's gravity spin (F15). A no-op on
        /// anything that was not spinning, which is every rocket.
        void despinBoardedVessel();
        // ---- F48: DRAWING A FAIRING -----------------------------------------
        /// Placing a fairing base drops straight into this, as KSP does: the
        /// wall follows the cursor, a left click plants a ring, a right click
        /// takes one back, and closing the nose ends it.
        void beginFairing(sw::usize blueprintIndex);
        void endFairing(bool keep);
        void updateFairing();
        /// True when the click was the tool's rather than the placement's.
        bool fairingClick();
        [[nodiscard]] bool fairingCursor(sw::Vec2& outRing);
        void rebuildFairingPreview();
        void collectFairingPreview(const sw::WorldVec3& cameraPosition,
                                   const sw::Quat& display);
        void collectFairingUi();
        /// Builds and registers the shell mesh for ONE fairing part, and hangs
        /// it on a child entity of the craft — the base keeps its own mesh, and
        /// the shell is the thing that later flies away in pieces.
        void buildFairingShellMesh(sw::ecs::Entity fairingPart,
                                   const sw::parts::FairingComponent& fairing);
        /// The release: the shell becomes `sides` panels of debris, each with
        /// a shove outward and a few seconds to live, and the payload feels the
        /// wind from that instant.
        void jettisonFairing(sw::ecs::Entity fairingPart);
        /// SW_FAIRING=<h:r,h:r,…>: draw one, without a mouse. The rings are
        /// fed to the SAME cursor the ray writes and clicked in with the SAME
        /// handler a left button press calls — a hook that built a profile by
        /// filling in the array would photograph a shape the tool cannot
        /// actually draw.
        void debugFairingScript(const char* spec);
        bool m_fairingDrawing = false;
        sw::usize m_fairingIndex = 0;
        sw::parts::FairingComponent m_fairingDraft{};
        sw::Vec2 m_fairingCursor{0.0f};
        bool m_fairingCursorValid = false;
        bool m_fairingCanClose = false;
        bool m_fairingPreviewDirty = false;
        sw::u32 m_fairingPreviewSlots[2] = {0xFFFFFFFFu, 0xFFFFFFFFu};
        sw::u32 m_fairingPreviewSlot = 0;
        sw::u32 m_fairingPreviewIndexCount = 0;
        std::vector<sw::Vec2> m_fairingScript;
        bool m_fairingScripted = false;

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
        /// Which palette shelf is open. `Count` means all of them are shut,
        /// which is a legal state: five headers and nothing else is the
        /// fastest way to see what the room can build.
        PaletteGroup m_paletteGroup = PaletteGroup::Command;
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
        /// Writes and registers the design; returns the name it was given,
        /// or empty on failure.
        std::string hangarSaveShip();
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
        // ------------------------------------------------------------------
        // THE SHELL: what the player is looking at before, and around, a game
        //
        // The game used to be the only thing there was. It opened straight
        // into a world that had already been built during the constructor —
        // a second or two of a black window with no way to tell a slow load
        // from a hang — and ESC quit it on the spot, with no way back and no
        // chance to save. Everything below is that missing frame around the
        // game, and it is a STATE MACHINE rather than a set of flags because
        // "loading" and "menu" and "playing" are mutually exclusive by
        // nature, and flags let you be two of them at once.
        // ------------------------------------------------------------------
        enum class Shell : sw::u8
        {
            Booting, // work is being done, a bar says how much is left
            Menu,    // nothing is simulated; the player is choosing
            Playing,
        };
        enum class MenuPage : sw::u8
        {
            Root,
            Load,
            Save,     // typing a name for a long save
            Settings, // deliberately empty for now
        };

        Shell m_shell = Shell::Booting;
        MenuPage m_menuPage = MenuPage::Root;
        /// True once a world has been played or loaded. It decides whether
        /// the root page offers CONTINUE, and whether NEW GAME has to throw
        /// a world away before building one.
        bool m_hasSession = false;
        /// One unit of start-up work: a name to show and the thing to do.
        /// Split up so the bar MEASURES something — a bar driven by a timer
        /// while the main thread blocks is a decoration, not information.
        struct BootStep
        {
            const char* label;
            void (StarWorksGame::*run)();
        };
        std::vector<BootStep> m_bootSteps;
        sw::usize m_bootCursor = 0;
        /// The step whose name is on screen: the one ABOUT to run, because a
        /// step that has finished is not what the player is waiting for.
        std::string m_bootLabel = "STARTING";
        void buildBootPlan();
        void bootLoadParts();
        void bootLoadAero();
        void bootLoadRecipes();
        void bootLoadDesigns();
        void bootBuildScene();
        void bootWireSystems();
        void bootBuildInstruments();
        void bootPrepareSaves();
        void updateBoot();
        void collectShellHud();
        void handleShellClick(sw::u32 id);
        void collectBootBar();
        /// The title screen's backdrop camera: a slow orbit of Terra along
        /// the terminator, so the menu sits over a live image of the game
        /// rather than a black wash. Runs on WALL time — the simulation is
        /// paused behind the menu, and the drift is presentation, not state.
        void updateMenuCamera(sw::f32 deltaSeconds);
        /// hudText, centred on `centerX` (metrics from the glyph advance).
        void hudTextCentered(std::string_view text, sw::f32 centerX, sw::f32 y,
                             sw::f32 heightNdc, const sw::Vec4& color);
        /// The big three-pass title (halo, shadow, face), centred.
        void hudTitle(sw::f32 centerX, sw::f32 topY, sw::f32 heightNdc);
        void newGame();
        void continueGame();
        void openMenu(MenuPage page);
        /// Saves found on disk, newest first. Rebuilt when the Load page is
        /// opened rather than every frame: it touches the filesystem.
        struct SaveSlot
        {
            std::string name;      // what the player typed, or "QUICKSAVE"
            std::filesystem::path path;
            sw::u64 bytes = 0;
            std::string when;      // local time, as text
            bool quick = false;
        };
        std::vector<SaveSlot> m_saveSlots;
        void refreshSaveSlots();
        /// The long save's name, typed on the Save page. Shares the text
        /// field machinery the multiplayer address box already uses.
        std::string m_saveName;
        bool m_saveNameFocused = false;
        /// What the last save or load did, shown under the menu's buttons.
        std::string m_shellStatus;
        /// CREATIVE MODE: engines burn no fuel. Chosen on the title screen,
        /// fixed for the session afterwards (it rides in the save, v10+).
        bool m_creativeMode = false;
        /// The thrust system instance, kept so the mode can reach it (same
        /// pattern as m_aerodynamics / m_bubbleSystem).
        ThrustSystem* m_thrustSystem = nullptr;
        /// The assembly hall's system, kept for the same reason: creative mode
        /// waives its bill of materials.
        sw::factory::AssemblySystem* m_assemblySystem = nullptr;
        /// Pushes m_creativeMode into the systems that act on it.
        void applyCreativeMode();
        /// The title screen's own camera (see updateMenuCamera). Used as the
        /// render camera whenever the menu is up with no session behind it;
        /// a pause menu keeps the player's frozen view instead.
        sw::Camera m_menuCamera;
        /// Where the backdrop orbit currently is, radians about Terra's axis.
        sw::f32 m_menuOrbitAngle = 0.0f;
        /// Full-screen vertical gradient (dark up top for the title, clear
        /// at the bottom so the planet stays visible). Vertex alpha does the
        /// gradient; hudQuad cannot, its unit quad is one flat colour.
        sw::u32 m_menuScrimMeshIndex = 0xFFFFFFFFu;
        [[nodiscard]] std::filesystem::path savesDirectory() const;
        [[nodiscard]] std::filesystem::path quickSavePath() const;
        void saveGameTo(const std::filesystem::path& path);
        void loadGameFrom(const std::filesystem::path& path);

        struct HudButton
        {
            sw::f32 x0, y0, x1, y1; // NDC (y down)
            sw::u32 id;
        };
        std::vector<HudButton> m_hudButtons; // rebuilt each frame
        /// True once this frame's button table has been opened. Guards the
        /// rule that exactly ONE collector clears — see hudBeginButtons.
        bool m_hudButtonsOpen = false;
        /// Opens the frame's button table: clears it, once, before anything
        /// is collected. Every collector after this one APPENDS.
        void hudBeginButtons();
        /// A modal taking the screen over: throws away what is already
        /// collected so nothing behind it stays clickable. Deliberately a
        /// different name, because confusing the two is the bug this pair
        /// exists to prevent.
        void hudSeizeButtons();
        void collectSasButtons();
        void handleHudClicks();
        /// One matched button: does what it means. Returns true when the
        /// click is finished, false to fall through to the hangar's 3D pick.
        /// The routing itself lives in sw::ui::routeHudClick.
        bool applyHudClick(const HudButton& button);

        // ====================================================================
        // MULTIPLAYER
        //
        // Every player owns their own clock. One of them warping does not
        // drag anyone else forward — that is the point of warp — so two
        // players in one session can legitimately be hours apart, and the
        // panel treats that difference as a first-class reading rather than
        // a fault. What crosses the gap is a STAMPED event: it carries the
        // instant it happened at, and a player who has not reached that
        // instant holds it until they do (Network/Timeline.hpp).
        // ====================================================================

        /// What the piloted craft is doing, computed once per frame BEFORE
        /// anything reads it. The warp gate and the HUD have to agree, and
        /// the gate runs earlier in the frame than the HUD does.
        struct FlightState
        {
            sw::i32 primaryIndex = -1;
            sw::f64 altitude = 0.0;
            sw::f64 periapsisAltitude = 0.0;
            sw::f64 apoapsisAltitude = 0.0;
            sw::f64 atmosphereTop = 0.0;
            bool grounded = false;
            bool closedOrbit = false;
            /// Standing on something: how far the nose leans off the local
            /// vertical, and whether the ground is still holding it up.
            ///
            /// This is on the HUD because the honest answer to "why is my
            /// rocket leaning and not falling over" is usually "because it
            /// is inside its own support polygon", and a player has no way
            /// to see that. A number and one word turn a bug report into a
            /// reading.
            sw::f32 leanDegrees = 0.0f;
            bool tipping = false;
        };
        FlightState m_flight{};
        void refreshFlightState();

        // ---- F43: THE ORBITAL SURVEY ---------------------------------------
        /// Fills in the coverage under the ground track when an OS-1 is armed
        /// and the orbit is one an instrument can work from.
        void updateSurvey();
        [[nodiscard]] bool surveyArmed() const;
        [[nodiscard]] bool surveyOrbitStable() const;
        /// The colour a surveyed place is worth marking in, or false when
        /// what is under it is not worth moving for.
        [[nodiscard]] static bool surveyTint(const sw::planet::DepositComponent& deposits,
                                             const sw::Vec3& bodyDirection,
                                             sw::Vec4& outColor);
        std::string m_surveyStatus;
        sw::f32 m_surveyFraction = 0.0f;
        /// Where the last sample was taken, so the arc between two samples
        /// can be painted whole. Reset whenever the primary changes, because
        /// an arc from one planet to another is not a ground track.
        sw::ecs::Entity m_surveyPrimary{};
        sw::Vec3 m_surveyPrevious{0.0f};

        // ---- F44: THE GEOLOGY SCREEN ----------------------------------------
        //
        // A screen and not a HUD panel, for the same reason the design office
        // is: what it shows is a WHOLE WORLD, and the question it answers —
        // where on this planet is the copper — has nothing to do with where
        // the ship is pointing this second. F4 opens it, F4 closes it, and
        // the simulation holds still while it is up.
        void enterGeology();
        void exitGeology();
        void updateGeology();
        void collectGeologyItems();
        void collectGeologyUi();
        /// Every body a satellite has started to look at, nearest first.
        [[nodiscard]] std::vector<sw::ecs::Entity> geologyBodies() const;
        /// Rebuilds the ore globe for the selected body and channel. Cheap
        /// enough to do on a channel switch and far too expensive to do per
        /// frame, so it is driven by a dirty flag.
        void rebuildGeologyGlobe();
        /// The direction under the cursor on the globe, in the body's frame.
        /// False when the cursor misses the sphere.
        [[nodiscard]] bool geologyCursorDirection(sw::Vec3& outDirection);
        /// Drops a beacon on the ground at a body-frame direction, or removes
        /// the one already within a few cells of it.
        void geologyToggleBeacon(const sw::Vec3& bodyDirection);
        bool m_geologyMode = false;
        bool m_pausedBeforeGeology = false;
        sw::ecs::Entity m_geologyBody{};
        sw::res::Resource m_geologyChannel = sw::res::Resource::IronOre;
        sw::f32 m_geologyYaw = 0.6f;
        sw::f32 m_geologyPitch = 0.25f;
        sw::f32 m_geologyDistance = 2.3f;
        /// Two slots, swapped on every rebuild: replacing the mesh that was
        /// drawn last frame is what used to force a full waitIdle, and the
        /// terrain patch builder learnt that the expensive way.
        sw::u32 m_geologyGlobeMesh[2] = {0, 0};
        sw::u32 m_geologyGlobeSlot = 0;
        bool m_geologyGlobeDirty = true;
        sw::Camera m_geologyCamera;


        /// Absolute simulation time a SYNC warp is aiming at, or 0.
        /// Distinct from m_warpToSeconds (the node) because it bypasses the
        /// altitude ladder: catching up to another player is the one warp
        /// whose destination is not negotiable.
        sw::f64 m_syncWarpTo = 0.0;
        /// Who we are catching up to, for the panel's benefit.
        sw::u32 m_syncWarpPlayer = 0;

        bool m_netPanel = false;
        std::unique_ptr<sw::net::Host> m_netHost;
        std::unique_ptr<sw::net::Client> m_netClient;
        /// The client's mirror of the host's world. Deliberately a SEPARATE
        /// world from m_world: this build does not yet merge a remote world
        /// into the local one, and decoding into the live world would fight
        /// the local simulation for every entity.
        sw::ecs::World m_netMirror;
        std::string m_netAddress = "127.0.0.1:7777";
        bool m_netAddressFocused = false;
        std::string m_netStatus;
        /// A timeout is diagnosed once per attempt, not once per frame — the
        /// state is sticky and the log would otherwise fill with it.
        bool m_netTimeoutLogged = false;
        /// What the firewall step did the last time HOST was pressed, so the
        /// panel can say so without asking the operating system every frame.
        sw::platform::FirewallRequest m_netFirewall =
            sw::platform::FirewallRequest::Unsupported;
        /// Wall clock of the last tick the suit had both feet down. The warp
        /// gate reads it so a jump — 1.6 s of legitimate air — does not count
        /// as having left the planet. See refreshFlightState.
        sw::f64 m_lastFootingSeconds = -1.0;
        /// Wall clock until which the warp refusal is worth saying, and what
        /// to say. A gate that is silent until it actually refuses something
        /// is the difference between an explanation and a permanent warning.
        sw::f64 m_warpRefusedUntil = -1.0;
        std::string m_warpRefusedReason;
        /// Sampled once, when hosting starts. A Public network profile makes
        /// the firewall rule inert without removing it, so it has to be said
        /// out loud or it is invisible.
        bool m_netPublicNetwork = false;
        sw::f64 m_netLastBeaconAt = -1.0;
        sw::u64 m_netEventsApplied = 0;

        /// A key press, unless a text field has the keyboard. One place to
        /// ask, so adding a field cannot silently leave a gameplay key live
        /// underneath it.
        [[nodiscard]] bool keyPressed(sw::KeyCode key);

        void updateNetwork(sw::f32 deltaSeconds);
        void updateTextField();
        void collectNetPanel();
        void netHost();
        void netJoin();
        void netLeave();
        /// Starts a sync warp toward `targetSeconds` if the craft is allowed
        /// to warp at all.
        void netSyncTo(sw::u32 playerId, sw::f64 targetSeconds);
        [[nodiscard]] bool netActive() const
        {
            return m_netHost != nullptr || m_netClient != nullptr;
        }
        /// Everyone in the session, host first, from whichever end we are.
        [[nodiscard]] std::vector<sw::net::PlayerView> netRoster() const;
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

        /// A DESIGN, DRAWN SOLID INSIDE A PANEL. Real part geometry, framed
        /// to the rectangle, spinning about its own vertical — the VAB's
        /// catalogue entry, so ordering a rocket does not mean ordering a
        /// name and a mass.
        ///
        /// There is no depth buffer in the HUD pass, so hidden surfaces are
        /// removed the two ways that need none: back faces are culled by the
        /// pipeline (correct on its own for a convex part, which is what
        /// almost every part is), and the parts are sorted back-to-front
        /// among themselves here.
        void hudDesignPreview(const sw::parts::ShipBlueprint& design, sw::f32 x0,
                              sw::f32 y0, sw::f32 x1, sw::f32 y1, sw::f32 spinRadians);

        /// Which saved design the VAB panel is showing, as an index into
        /// sw::parts::blueprintCatalog(). -1 = none picked yet.
        sw::i32 m_vabSelection = -1;

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
        // ---- F46: DOCKING ---------------------------------------------------
        /// Looks for a capture between every pair of free ports on different
        /// craft, and merges the two when one clears all four limits.
        void updateDocking();
        /// Releases the docking joint on the part whose menu is open.
        void undockPart(sw::ecs::Entity portPart);
        /// Moves the root-level components a vessel needs to fly onto the
        /// survivor of a merge, before the absorbed root is destroyed.
        void adoptVesselRoot(sw::ecs::Entity survivor, sw::ecs::Entity absorbed);
        /// What the pilot is being told about the nearest approach: empty when
        /// no port of the controlled craft is near another craft's.
        std::string m_dockStatus;
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
        /// ...or a CRAFT (F49). The two are exclusive: picking one clears the
        /// other, because "the target" is one thing and a HUD with two of them
        /// would have to say which it meant on every line.
        sw::ecs::Entity m_targetVessel{};
        sw::space::ClosestApproach m_approach{};
        /// The same, for the trajectory AFTER the planned burn.
        sw::space::ClosestApproach m_nodeApproach{};
        void updateTargetPick();
        [[nodiscard]] bool isTargetableCraft(sw::ecs::Entity entity) const;
        [[nodiscard]] std::string targetName() const;
        /// Where the target is right now, or false when there is not one (or
        /// it has stopped existing).
        [[nodiscard]] bool targetWorldPosition(sw::WorldVec3& outPosition) const;
        /// How the target moves, for the closest-approach solver. Clears a
        /// craft target that has been staged, docked or destroyed away.
        [[nodiscard]] bool targetPath(sw::space::TargetPath& outPath);

        sw::ecs::Entity m_solEntity{};
        sw::ecs::Entity m_terraEntity{};

        // ---- THE FLOATING ORIGIN --------------------------------------------
        // Every position in the ECS is relative to the anchor of the system
        // this index names, and index 0 — Sol — is the identity, so a save
        // written before any of this existed still loads.
        //
        // IT IS NOT A CONVENIENCE, IT IS THE ONLY WAY THE NUMBERS FIT. A double
        // carries about sixteen digits, so at Proxima's 4.0e16 metres the gap
        // between two representable positions is EIGHT METRES. Held in absolute
        // coordinates a craft in orbit there would jitter by the length of
        // itself, a landing would be a coin flip, and the terrain — which
        // subtracts two nearby world positions to find a patch origin — would
        // dissolve. Rebased onto the star it is standing next to, the same
        // craft is at 1e7 metres and the gap is a NANOMETRE.
        //
        // What makes it cheap is that the renderer was already camera-relative:
        // everything reaching the GPU is a difference, so a shift the camera
        // shares is invisible. What makes it dangerous is the handful of places
        // that cache an absolute position across frames — see rebaseOrigin().
        sw::u32 m_originSystem = sw::space::kSolSystem;

        /// A local (ECS) position in absolute catalogue coordinates.
        [[nodiscard]] sw::WorldVec3 absolutePosition(const sw::WorldVec3& local) const
        {
            return sw::space::systems()[m_originSystem].position + local;
        }
        /// ...and back again.
        [[nodiscard]] sw::WorldVec3 localPosition(const sw::WorldVec3& absolute) const
        {
            return absolute - sw::space::systems()[m_originSystem].position;
        }
        /// Moves the world origin to another system's anchor, shifting every
        /// absolute position in the game by the difference. Safe only between
        /// ticks, on the main thread.
        void rebaseOrigin(sw::u32 systemIndex);

        /// One entity per catalogue star, index-parallel to sw::space::stars().
        /// Every one of them exists for the whole session: you have to be able
        /// to SEE where you are going, and a star four light-years away costs
        /// one transform and one distance computation a tick.
        std::vector<sw::ecs::Entity> m_starEntities;
        void buildCatalogueStars();
        /// The star that lights this point — brightest, not nearest.
        [[nodiscard]] sw::ecs::Entity dominantStar(const sw::WorldVec3& position) const;
        /// Refreshed once per frame in onRender: the star whose light the
        /// shaders are given, whose glare gets the three-layer treatment, and
        /// whose radius the shadow tests are written against. Everywhere the
        /// code used to say "Sol" and mean "the sun", it now says this.
        sw::ecs::Entity m_lightStar{};
        /// Every star drawn as a SUN this frame — the dominant one plus any
        /// companion delivering at least kSunIrradianceRatio of its
        /// irradiance. Rebuilt each frame; the billboard path skips these.
        std::vector<sw::ecs::Entity> m_sunsHere;
        void collectSunsHere(const sw::WorldVec3& position,
                             std::vector<sw::ecs::Entity>& out) const;
        /// The three-layer glare, the photosphere disc and (for the dominant
        /// star only) the lens-flare chain, for ONE star.
        void collectStarVisual(sw::ecs::Entity star, const sw::Camera& activeCamera,
                               bool mapView, bool withFlare);

        // ---- PART ANIMATION -------------------------------------------------
        /// Mesh slot for one animated group of one part, keyed by definition
        /// id and animation index. A part's STATIC group stays in
        /// m_partMeshIds, so every part that animates nothing is untouched.
        std::unordered_map<sw::u64, sw::u32> m_partGroupMeshIds;
        /// The motions themselves, derived once at boot from the catalogue —
        /// which shapes ride together and whose pose they follow. Keyed by
        /// definition id, because it is a property of the DEFINITION and not
        /// of any instance of it.
        std::unordered_map<sw::u32, std::vector<sw::parts::PartMotionGroup>> m_partMotions;
        [[nodiscard]] static constexpr sw::u64 partGroupKey(sw::u32 definitionId,
                                                            sw::u32 group)
        {
            return (static_cast<sw::u64>(definitionId) << 16) | group;
        }
        /// Adds the live animation state to a freshly spawned part, if its
        /// definition has any. One call, from every path that makes a part.
        void attachPartAnimation(sw::ecs::Entity part, sw::u32 definitionId);
        /// Emits the extra draw items for a part's moving groups.
        void collectAnimatedGroups(sw::ecs::Entity entity,
                                   const sw::parts::PartComponent& part,
                                   const sw::Mat4& partModel, const sw::Vec3& relative,
                                   sw::f32 boundsRadius, const sw::Vec4& tint);
        /// Where an entity is DRAWN this frame: the physics lane's alpha mix
        /// of its previous and current transforms, which is the pose the
        /// renderer and the chase camera both use. Anything that has to agree
        /// with what the pilot can see — a click target, a screen anchor —
        /// must ask this and not the raw transform.
        [[nodiscard]] sw::WorldVec3 renderPosition(sw::ecs::Entity entity) const;
        /// Right-click in flight: the part under the cursor, or null.
        [[nodiscard]] sw::ecs::Entity pickPartUnderCursor(const sw::Camera& camera) const;
        /// The same search with the ray handed in: the operable part nearest
        /// along it, or null. Split out so a headless run can aim it, since a
        /// capture has no cursor to aim it with.
        [[nodiscard]] sw::ecs::Entity pickPartAlongRay(const sw::WorldVec3& eye,
                                                       const sw::Vec3& direction) const;
        /// The part whose menu is open, and where on screen it sits.
        sw::ecs::Entity m_menuPart{};
        sw::Vec2 m_menuAnchor{0.0f, 0.0f};
        void collectPartMenu(const sw::Camera& camera);
        void togglePartAnimation(sw::u32 index);
        /// Feeds the throttle into every throttle-triggered animation.
        void updateThrottleAnimations();
        /// How far the mouse travelled while the right button was down. A
        /// right-DRAG turns the camera; a right-CLICK opens a part's menu.
        sw::f32 m_rightDragPixels = 0.0f;
        sw::f32 m_rightHeldSeconds = 0.0f;
        /// Until when to say that the last click found nothing operable.
        sw::f64 m_partMenuMissUntil = 0.0;
        /// Draws every catalogue star that is NOT the local sun: one soft
        /// billboard each, sized and lit off its apparent magnitude.
        void collectDistantStars(const sw::Camera& activeCamera);
        /// Instantiates one system's planets. Idempotent: a system is built at
        /// most once per session.
        void loadSystemPlanets(sw::u32 systemIndex);
        /// Once per frame: hands the origin to the nearest star and builds the
        /// planets of whichever system the craft has entered.
        void updateSystemStreaming();
        /// SW_JUMP=<SYSTEM>: teleport the craft to another star, once, at boot.
        void applyDebugJump();

        // ---- INTERSTELLAR GUIDANCE ------------------------------------------
        // Between the stars there is no orbit to read. A conic drawn around a
        // star you have already escaped is a straight line to within the width
        // of the screen, and the map's whole vocabulary — periapsis, encounter,
        // closest approach — has nothing to say about a four-light-year
        // crossing. What a pilot actually needs there is one number repeated
        // three times: how far off the required heading am I, about each axis.
        struct InterstellarGuidance
        {
            bool valid = false;
            sw::u32 systemIndex = 0;
            const char* systemName = "";
            sw::f64 distanceMeters = 0.0;
            /// The rotation that takes the current heading onto the required
            /// one, as an axis-angle vector in DEGREES about world X, Y and Z.
            /// Three numbers that go to zero together and each of which says
            /// which way to turn.
            sw::Vec3 deviationDegrees{0.0f};
            sw::f32 totalDegrees = 0.0f;
            sw::f64 closingSpeedMps = 0.0;
            sw::f64 etaSeconds = -1.0; // negative: not closing
        };
        [[nodiscard]] InterstellarGuidance interstellarGuidance() const;
        bool m_debugJumped = false;
        bool m_debugMenuToggled = false;
        bool m_debugProbed = false;
        bool m_debugStarProbed = false;
        bool m_debugClockSet = false;
        bool m_debugGroundProbed = false;
        bool m_debugThrustProbed = false;
        bool m_debugEnginesSet = false;
        /// What the flight plan costs. It is recomputed on the MAIN THREAD, so
        /// this is directly a hitch the player feels — and it is invisible to
        /// a frame-rate counter, because the frames between two refreshes are
        /// as fast as they ever were.
        sw::f64 m_worstPredictionMs = 0.0;
        sw::f64 m_lastPredictionMs = 0.0;
        bool m_debugFrameProbed = false;
        bool m_debugBurned = false;
        sw::u32 m_debugBurnDelay = 0;
        bool m_debugPredSwept = false;
        sw::u32 m_debugSweepDelay = 0;
        bool m_debugSpawned = false;
        bool m_debugHangarOpened = false;
        bool m_debugFairingDrawn = false;
        bool m_debugPlaced = false;
        bool m_debugPaletteOpened = false;
        bool m_debugCraftTargeted = false;
        sw::u32 m_debugTargetDelay = 0;
        sw::u32 m_debugStagesFired = 0;
        sw::u32 m_debugStageDelay = 0;
        bool m_debugDockSpawned = false;
        sw::u32 m_debugDockDelay = 0;
        bool m_debugMachineOpened = false;
        sw::u32 m_debugMachineRow = 0;
        sw::u32 m_debugMachineDelay = 0;
        bool m_debugGeologyOpened = false;
        bool m_debugGeologyChannel = false;
        bool m_debugGeologyBeacon = false;
        sw::u32 m_debugGeologyDelay = 0;
        bool m_debugOrbited = false;
        bool m_debugSurveyed = false;
        sw::u32 m_debugSurveyDelay = 0;
        sw::u32 m_debugOrbitDelay = 0;
        bool m_debugAnimProbed = false;
        sw::u32 m_debugAnimDelay = 0;
        std::vector<sw::f32> m_frameSamples;
        std::vector<sw::f32> m_predictionSamples;
        /// WHERE THE FRAME WENT. One accumulator per phase of the frame,
        /// reset on the frame boundary; `m_phaseLastMs` is the completed
        /// previous frame, which is the one `clock().deltaSeconds()` also
        /// describes — so the probe compares two numbers about the same
        /// frame rather than two numbers about neighbouring ones.
        enum FramePhase : sw::u32
        {
            kPhaseSimulation = 0,
            kPhaseCelestialIndex,
            kPhaseStreaming,
            kPhasePrediction,
            kPhaseTerrain,
            kPhaseGrass,
            kPhaseReentry,
            kPhaseScene,
            kPhaseRender,
            kPhaseCount,
        };
        static constexpr std::array<const char*, kPhaseCount> kPhaseNames{
            "simulation", "celestial-index", "streaming", "prediction",
            "terrain",    "grass",           "reentry",   "scene-collect",
            "render"};
        std::array<sw::f64, kPhaseCount> m_phaseMs{};
        std::array<sw::f64, kPhaseCount> m_phaseLastMs{};
        std::array<sw::f64, kPhaseCount> m_phaseWorstMs{};
        std::array<sw::f64, kPhaseCount> m_phaseTotalMs{};
        std::array<sw::f64, kPhaseCount> m_phaseWorstFrameMs{};
        /// SW_GROUNDPROBE: the walker's distance from its body's centre, one
        /// sample per frame. Standing still, this should be a constant.
        std::vector<sw::f64> m_groundSamples;
        sw::u32 m_groundWarmup = 0;
        /// Frames the probe waits before firing. SW_SHOT moves the camera in
        /// the call AFTER the probe's, so a probe on frame one measures the
        /// eye where the boot left it — 1.6 billion km from the ship it is
        /// meant to be photographing — and reports a pick that misses
        /// everything for a reason that has nothing to do with the pick.
        sw::u32 m_debugProbeDelay = 0;
        /// One flag per catalogue system. Sol's is set at scene build.
        std::vector<bool> m_systemLoaded;

        // ---- SW_SHOT: the capture camera -----------------------------------
        // Set the environment variable SW_SHOT to `BODY@RADII[,map][,yaw]` —
        // for example `SATURN@6.9` — and the build skips the menu, starts a
        // world, and parks the camera that many body radii from the named
        // body with the sun over its shoulder. With --frames and --capture
        // that is a screenshot of the REAL renderer, which is the only thing
        // that can see a bug in the vertex path, a mesh, or a blend mode.
        // Empty (the normal case) and none of it happens.
        std::string m_debugShotBody;
        sw::f64 m_debugShotRadii = 6.0;
        sw::f32 m_debugShotYaw = 0.0f;
        /// SW_SHOT's fourth field: how far above the subject the ship/part
        /// camera sits, as a multiple of the standoff. Negative looks up at it.
        sw::f32 m_debugShotPitch = 0.35f;
        bool m_debugShotMap = false;
        /// Frames the shot camera has run for. SW_SHOT's `map` token waits on
        /// this: the part menu only exists in the cockpit, so a capture has to
        /// arm what it photographs BEFORE it opens the map.
        sw::u32 m_debugShotFrames = 0;
        /// SW_NO_GLARE=1 suppresses the sun's three glare layers, leaving the
        /// photosphere alone. It exists so Tools/solar_scale/check_render_size
        /// can MEASURE the sun's disc: the glare is a deliberate optical
        /// overlay wider than the body, so with it on there is nothing to
        /// measure but the overlay.
        bool m_debugNoGlare = false;
        void parseDebugShot();
        void applyDebugShot();

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
