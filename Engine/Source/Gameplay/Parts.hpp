#pragma once

// ============================================================================
// Gameplay/Parts.hpp
// THE PART SYSTEM — the foundation every future construction feature
// (rocket assembly, staging, docking, factories-on-wheels) builds on.
//
// Architecture, in three layers:
//
//  1. PartDefinition / PartCatalog — STATIC data. Every part type's mass,
//     cost, volume, resource capacities, strength, aerodynamics and attach
//     points live in a code-defined catalog (data files later; the lookup
//     API won't change). Definitions are addressed by a STABLE id.
//
//  2. PartComponent — one ECS ENTITY PER PART INSTANCE, referencing its
//     definition by id and its VESSEL by entity handle, with a vessel-local
//     pose. Entity-per-part is the property that makes the future free:
//     decoupling = reparenting part entities to a new vessel root; docking
//     = pointing them at a merged one; breaking = destroying one entity.
//     Resources carried by a part (fuel, charge) are an ordinary
//     factory::InventoryComponent ON the part entity — tanks are cargo.
//
//  3. Systems. VesselAssemblySystem aggregates parts into the vessel's
//     VesselComponent every physics tick (dry mass + carried resource mass
//     -> DynamicBody.mass, so the rocket equation simply EMERGES as fuel
//     burns; thrust, mass flow, drag area -> ballistic factor).
//     PartAttachmentSystem glues part transforms to their vessel's pose
//     (lockstep previous-transforms: parts interpolate with their vessel).
// ============================================================================

#include "ECS/System.hpp"
#include "Factory/FactoryComponents.hpp"
#include "Physics/PhysicsComponents.hpp"
#include "Resources/ResourceTypes.hpp"
#include "Scene/TransformComponents.hpp"

#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace sw::parts
{
    enum class PartType : u8
    {
        FuelTank = 0,
        Engine,
        Wing,
        Battery,
        SolarPanel,
        DockingPort,
        Decoupler,
        CargoBay,
        Structural,

        Count
    };

    // ---- geometry: parts are COMPOSED PRIMITIVES ------------------------------
    // A part's look AND its collision come from the same list of authored
    // primitives (Part Studio writes them, the game reads them). A shape can
    // be render-only (greebles), collider-only (simplified hull) or both.
    // Local frame convention: -Z is the NOSE direction of the rocket stack,
    // +Z the tail; radial shapes grow on X/Y.

    enum class ShapeKind : u8
    {
        Box = 0,   // size = half extents (x, y, z)
        Cylinder,  // axis Z; size.x = radius, size.y = half length
        Cone,      // axis Z frustum; size.x = radius at -Z, size.z = radius at +Z,
                   // size.y = half length (size.z = 0 -> pointed nose)
        Sphere,    // ellipsoid; size = per-axis radii
        Tube,      // axis Z ring; size.x = outer radius, size.z = inner radius,
                   // size.y = half length
    };

    struct PartShape
    {
        ShapeKind kind = ShapeKind::Box;
        Vec3 position{0.0f};      // part-local, meters
        Vec3 rotationDeg{0.0f};   // euler XYZ, degrees (tool-friendly)
        Vec3 size{0.5f};          // semantics per kind (see ShapeKind)
        Vec3 color{0.8f};         // linear RGB
        f32 emissive = 0.0f;      // 0 = lit, 1 = fully self-lit
        f32 specular = 0.32f;     // Blinn-Phong strength (0 = matte)
        f32 gloss = 0.60f;        // highlight tightness (0 = broad)
        u32 segments = 24;        // radial tessellation of round kinds
        bool visible = true;      // rendered
        bool collider = false;    // part of the collision hull

        // ---- ANIMATION: the same shape, posed twice -------------------------
        // `animation` is an index into PartDefinition::animations, or -1 for a
        // shape that never moves. When it moves, the fields above are its pose
        // at phase 0 and the three below are its pose at phase 1; everything
        // in between is interpolated.
        //
        // TWO POSES AND NOT A CURVE, on purpose. A hinge, a piston, a nozzle
        // lighting up — every animation a part needs is one rest state and one
        // working state. Keyframes would buy sequences nobody has asked for and
        // cost an authoring tool nobody would enjoy using; two poses can be
        // authored by MOVING THE THING, which is the whole of Part Studio's
        // existing vocabulary.
        i32 animation = -1;
        Vec3 endPosition{0.0f};    // part-local, at phase 1
        Vec3 endRotationDeg{0.0f}; // euler XYZ, at phase 1
        /// Self-illumination at phase 1. Negative means "same as `emissive`",
        /// which is what every shape that only moves wants. It exists for the
        /// one animation that does not move at all: an engine bell whose glow
        /// cone goes from dark to white with the throttle.
        f32 endEmissive = -1.0f;
    };

    /// Node types follow KSP: STACK nodes joint along the rocket axis and
    /// only mate with an opposing stack node; RADIAL nodes let the part be
    /// glued onto another part's SURFACE anywhere.
    enum class NodeType : u8
    {
        Stack = 0,
        Radial,
        /// CONVEYOR PORTS. A belt does not attach to a hull the way a fin
        /// does — it arrives at a specific mouth, facing a specific way, and
        /// it has a DIRECTION: goods come out of one machine and go into the
        /// next. Making that a node type rather than a convention means the
        /// mouths are authored in Part Studio, on the geometry, and the
        /// build validator can say "this belt runs backwards" instead of
        /// discovering it at runtime.
        ConveyorIn,
        ConveyorOut,
        /// THE POWER CONNECTION. Where a cable may be hooked onto this
        /// building, in its own frame — usually the top of a mast or a
        /// junction box on the roof, because a wire that meets a machine at
        /// ankle height looks like a trip hazard rather than a supply.
        ///
        /// A building without one takes no cable at all, which is the honest
        /// way to say "this thing does not touch the grid": a conveyor tile
        /// has no power node and so can never be wired to anything.
        Power,
        Count
    };

    [[nodiscard]] std::string_view nodeTypeName(NodeType type);
    [[nodiscard]] bool nodeTypeFromName(std::string_view name, NodeType& outType);

    /// True for the two conveyor port types.
    [[nodiscard]] inline bool isConveyorNode(NodeType type)
    {
        return type == NodeType::ConveyorIn || type == NodeType::ConveyorOut;
    }

    /// ONE BOX OF A PART'S HITBOX.
    ///
    /// Until now a part's collision hull was inferred from the shapes marked
    /// `collider` — the same primitives that draw it. That conflates two
    /// different jobs. What a part LOOKS like wants cones, tubes, greebles
    /// and forty segments; what a part BUMPS INTO wants as few boxes as will
    /// do, because every one of them is tested against every other part.
    /// Worse, it meant you could not fix a hull without changing the model.
    ///
    /// So the hull is now authored: a LIST of boxes, axis-aligned in the
    /// part's own frame — that is what makes it an AABB, and what makes it
    /// cheap. A part is not one box (a rocket with fins is not a crate), so
    /// the hull is their union, and Part Studio draws and edits them the way
    /// it already does shapes and nodes.
    ///
    /// A definition with NO hitboxes falls back to its collider shapes,
    /// exactly as before — every .swpart written before this loads and
    /// behaves unchanged.
    struct HitBox
    {
        Vec3 center{0.0f};
        Vec3 halfExtents{0.5f};
    };

    /// A named attachment location on a part, in the part's local frame.
    /// Authored ON the collider surface by Part Studio — never inside.
    struct AttachNode
    {
        std::string name;                  // "top", "bottom", "radial"...
        Vec3 position{0.0f};               // meters, part-local
        Vec3 direction{0.0f, 0.0f, 1.0f};  // outward normal of the joint
        NodeType type = NodeType::Stack;
        f32 size = 0.6f;                   // visual + snap radius, meters
    };

    /// One resource capacity slot of a definition.
    struct ResourceCapacity
    {
        res::Resource resource = res::Resource::Count; // Count == unused
        f64 units = 0.0;                               // filled at build time
    };

    inline constexpr u32 kMaxResourceCapacities = 2;

    /// Static description of a part model — everything the game will ever
    /// ask about a part that is not per-instance state. DATA-DRIVEN: the
    /// canonical source is an .swpart JSON file (authored in Part Studio);
    /// The INDUSTRIAL block (F1): what turns a piece of authored geometry
    /// into a building the player can plant on a planet. Absent (`valid ==
    /// false`) on every rocket part, so a .swpart written before F1 loads
    /// unchanged and the VAB palette can simply skip the ones that have it.
    struct BuildingSpec
    {
        bool valid = false;
        factory::BuildingCategory category = factory::BuildingCategory::Storage;
        /// Ground footprint in metres — the box the placement validator
        /// tests for slope, overlap and clearance.
        f64 footprintM[2] = {8.0, 8.0};
        /// Steady electrical balance: POSITIVE produces (solar), negative is
        /// the idle draw. A running recipe adds its own on top.
        f64 powerKw = 0.0;
        /// Storage the building carries. 0 means it holds nothing itself.
        f64 inventoryVolumeM3 = 0.0;
        /// Steepest ground it can be built on, as a TANGENT (0.25 = 14 deg).
        f64 maxSlopeTangent = 0.25;
        /// Miners only: the deposit density below which siting is refused.
        f64 minOreDensity = 0.0;
    };

    // ---- ANIMATIONS -----------------------------------------------------------
    /// What drives an animation's phase.
    enum class AnimationTrigger : u8
    {
        /// The pilot, through the part's right-click menu. Phase walks from 0
        /// to 1 over `durationSeconds` and back.
        Toggle = 0,
        /// The throttle, continuously. Phase IS the throttle, so a nozzle
        /// brightens as the engine is opened up rather than snapping on.
        Throttle,
        Count,
    };

    /// What the animation is called in the pilot's menu, as a pair of verbs.
    /// A closed list rather than free text: the HUD font has no lower case and
    /// forty glyphs, and every part in the game does one of these four things.
    enum class AnimationVerbs : u8
    {
        OpenClose = 0, // solar arrays, cargo bays
        OnOff,         // engines, lights
        ExtendRetract, // landing gear, ladders
        DeployStow,    // antennae, radiators
        Count,
    };

    /// What a fully-retracted animation TAKES AWAY. This is the difference
    /// between an animation and a decoration: a stowed panel that still makes
    /// power is a moving picture, not a mechanism.
    enum class AnimationGates : u8
    {
        Nothing = 0,
        Power,  // chargeRateKw is scaled by the phase
        Thrust, // thrustNewtons is scaled by the phase
        Count,
    };

    struct PartAnimation
    {
        static constexpr usize kNameCapacity = 20;

        char name[kNameCapacity] = {}; // shown in the pilot's menu
        AnimationTrigger trigger = AnimationTrigger::Toggle;
        AnimationVerbs verbs = AnimationVerbs::OpenClose;
        AnimationGates gates = AnimationGates::Nothing;
        f32 durationSeconds = 3.0f;
        /// Where a freshly built part starts. A solar wing folds into its
        /// fairing; a landing gear is down on the pad.
        bool startsOpen = false;
    };

    /// A rigid motion between two poses, expressed the way it LOOKS rather
    /// than the way it was authored.
    ///
    /// Two poses interpolated the obvious way — slerp the rotation, lerp the
    /// position — is wrong for a hinge, and wrongest exactly where hinges are
    /// used. A solar panel swinging ninety degrees about a mount at its root
    /// has both ends of its travel correct and its MIDDLE cutting the corner:
    /// the panel shrinks toward the hub and springs back out, which reads as a
    /// telescope rather than a hinge.
    ///
    /// Every rigid motion is a rotation about some axis through some point,
    /// plus a slide along that axis: Chasles' theorem. Recovering that triple
    /// from the two poses costs one eigen-solve of a 3x3 and turns the same
    /// authored data into an arc. The pivot is only determined perpendicular
    /// to the axis — sliding it along the axis changes nothing — so the
    /// minimum-norm solution is taken, which is the pivot a person would point
    /// at if asked where the hinge was.
    struct HingeMotion
    {
        Vec3 pivot{0.0f};
        Vec3 axis{0.0f, 1.0f, 0.0f};
        f32 angleRadians = 0.0f;
        Vec3 slide{0.0f}; // pure translation (the whole motion when angle = 0)
    };

    /// Recovers the hinge that takes (positionA, rotationA) to
    /// (positionB, rotationB).
    [[nodiscard]] HingeMotion hingeBetween(const Vec3& positionA, const Quat& rotationA,
                                           const Vec3& positionB, const Quat& rotationB);

    /// The pose at `phase` in [0,1] along that hinge.
    void poseAlongHinge(const HingeMotion& hinge, const Vec3& positionA,
                        const Quat& rotationA, f32 phase, Vec3& outPosition,
                        Quat& outRotation);

    /// loadCatalog() fills the registry from a directory of them.
    struct PartDefinition
    {
        u32 id = 0;                     // STABLE id (saved in instances)
        PartType type = PartType::Structural;
        std::string name;

        // ---- bulk properties -------------------------------------------------
        f64 dryMassKg = 100.0;
        f64 costCredits = 100.0;
        f64 volumeM3 = 1.0;

        // ---- resources carried (tanks, batteries...) --------------------------
        ResourceCapacity capacities[kMaxResourceCapacities]{};

        // ---- structure ---------------------------------------------------------
        f64 crashToleranceMps = 12.0;   // impact speed the part survives
        f64 breakingForceN = 2.0e5;     // joint strength (future structural sim)

        // ---- aerodynamics ------------------------------------------------------
        /// THE FALLBACK, and only that, since F6. A part with a solved
        /// `.aero.json` beside it is flown from the table — direction by
        /// direction, with moments — and never touches this number. What it
        /// still buys is a vessel of parts nobody has run the forge on: the
        /// old isotropic model, one Cd*A summed over the stack, which is a
        /// worse answer than the table and a much better one than nothing.
        f64 dragCoefficientArea = 0.8;  // Cd * A, m^2
        /// DEAD, and left in place for the files. Lift is no longer a number
        /// somebody types next to a wing: it comes out of the geometry, in
        /// the forge, from the shape of the surface and the angle it meets
        /// the air at. Kept so that every .swpart ever written still loads,
        /// and as a marker of what the aero pass replaced.
        f64 liftCoefficient = 0.0;

        // ---- function-specific -------------------------------------------------
        f64 thrustNewtons = 0.0;        // engines
        f64 specificImpulseS = 0.0;     // engines (fuel flow = F / (Isp*g0))
        f64 chargeRateKw = 0.0;         // solar panels: kJ/s generated

        // ---- industry (F1): present only on buildings --------------------------
        BuildingSpec building{};
        /// Set by `"prop": true`. See isProp().
        bool prop = false;

        // ---- geometry & connexions ----------------------------------------------
        std::vector<PartShape> shapes;
        std::vector<AttachNode> nodes;
        /// The COLLISION HULL, as boxes. Empty means "derive it from the
        /// collider shapes", which is what every part did before hitboxes
        /// existed.
        std::vector<HitBox> hitboxes;
        /// At most four: a part with five things to deploy is two parts.
        std::vector<PartAnimation> animations;
    };

    inline constexpr usize kMaxPartAnimations = 4;

    /// The catalog. Before loadCatalog() succeeds it holds a minimal
    /// BUILT-IN fallback (same stable ids, box geometry) so tests and a
    /// game with missing assets keep working. Lookup by STABLE id.
    [[nodiscard]] std::span<const PartDefinition> catalog();
    [[nodiscard]] const PartDefinition* findDefinition(u32 id);

    // ---- .swpart files (JSON) ---------------------------------------------------
    /// Replaces the registry with every *.swpart in `directory` (sorted by
    /// id). Returns false (and keeps the previous catalog) when the
    /// directory has no valid part files. Errors are logged per file.
    bool loadCatalog(const std::filesystem::path& directory);
    /// Parses one part file into `out`. Returns false and logs on error.
    [[nodiscard]] bool loadPartFile(const std::filesystem::path& path, PartDefinition& out);
    /// Writes the canonical JSON form (Part Studio's save).
    [[nodiscard]] bool savePartFile(const PartDefinition& definition,
                                    const std::filesystem::path& path);

    // Stable catalog ids (never renumber).
    inline constexpr u32 kPartFuelTankMedium = 1;
    inline constexpr u32 kPartEngineVector = 2;
    inline constexpr u32 kPartFinBasic = 3;
    inline constexpr u32 kPartBatteryPack = 4;
    inline constexpr u32 kPartSolarWing = 5;
    inline constexpr u32 kPartDockingRing = 6;
    inline constexpr u32 kPartDecouplerFlat = 7;
    inline constexpr u32 kPartCargoBaySmall = 8;
    inline constexpr u32 kPartCoreStructural = 9;

    // Buildings share the same catalogue and the same stable-id space: they
    // are parts with an industrial block, edited in the same Part Studio.
    inline constexpr u32 kBuildingHub = 100;
    inline constexpr u32 kBuildingMiner = 101;
    inline constexpr u32 kBuildingRefinery = 102;
    inline constexpr u32 kBuildingStorage = 103;
    inline constexpr u32 kBuildingSolarFarm = 104;
    inline constexpr u32 kBuildingBeacon = 105;
    inline constexpr u32 kBuildingConveyor = 106; // one belt segment, tiled
    inline constexpr u32 kPropConveyorCrate = 107; // what rides the belt
    inline constexpr u32 kBuildingBatteryBank = 108; // joules for the night
    inline constexpr u32 kBuildingPowerPole = 109;   // the only place a grid branches
    inline constexpr u32 kBuildingCable = 110;       // one span of wire
    inline constexpr u32 kPropEvaSuit = 111;         // the player, on foot
    inline constexpr u32 kBuildingVab = 112;         // where rockets are made
    inline constexpr u32 kBuildingLaunchPad = 113;   // ...and where they stand
    inline constexpr u32 kPropVehicleCradle = 114;   // a rocket, riding a belt

    // The ENDURANCE family (F15): the ring ship parked at Saturn. Each piece
    // is an ordinary vessel part — the ship is the ASSEMBLY, authored as a
    // blueprint in GameScene, not as one monolithic model. Ids 200+ so the
    // rocket/building ranges keep room to grow.
    //
    // The eight of them are the film's own parts list: twelve modules of
    // FIVE kinds (four propulsion, four cargo pods, two habitats, one cryo
    // bay, one command module) strung on twelve connecting tunnels, plus
    // the two support craft. There is no hub and there are no spokes —
    // the Endurance is a bare ring, and that is most of its silhouette.
    inline constexpr u32 kPartEnduranceHabitat = 200; // crew quarters + arrays
    inline constexpr u32 kPartEnduranceEngine = 201;  // 3 plasma engines each
    inline constexpr u32 kPartEnduranceCommand = 202; // cockpit, comms, cupola
    inline constexpr u32 kPartEnduranceTunnel = 203;  // module-to-module link
    inline constexpr u32 kPartEnduranceRanger = 204;  // the lifting body
    inline constexpr u32 kPartEnduranceLander = 205;  // the heavy-lift ferry
    inline constexpr u32 kPartEnduranceCargo = 206;   // detachable, becomes base
    inline constexpr u32 kPartEnduranceCryo = 207;    // hypersleep + sick bay
    inline constexpr u32 kPartEnduranceCoreHub = 208; // six ports, ring centre
    inline constexpr u32 kPartEnduranceSpoke = 209;   // core-to-ring strut

    /// A PROP: authored geometry the GAME places, never the player. Conveyor
    /// cargo is the first one — a crate is not something you pick out of a
    /// palette, it is something a belt is carrying. It is still a .swpart,
    /// so its look is edited in the same tool as everything else.
    [[nodiscard]] inline bool isProp(const PartDefinition& definition)
    {
        return definition.prop;
    }

    /// The parts a player may stack onto a VESSEL: not buildings, not props.
    [[nodiscard]] inline bool isVesselPart(const PartDefinition& definition)
    {
        return !definition.building.valid && !definition.prop;
    }

    /// The first conveyor port of a definition matching `type`, or nullptr.
    [[nodiscard]] const AttachNode* findConveyorNode(const PartDefinition& definition,
                                                     NodeType type);

    /// EVERY port of a type, in authored order.
    ///
    /// A machine may have more than one mouth of the same kind, and the
    /// order is the contract: out port i ships the recipe's output i. That
    /// is what lets an electrolyser put hydrogen on one belt and oxygen on
    /// another instead of both down a single run — which is the difference
    /// between a fuel chain you can lay out and one you cannot.
    [[nodiscard]] std::vector<const AttachNode*> conveyorNodes(
        const PartDefinition& definition, NodeType type);

    /// How many mouths of this kind the definition has.
    [[nodiscard]] u32 conveyorNodeCount(const PartDefinition& definition, NodeType type);

    /// Where a cable hooks onto this definition, or nullptr if it does not
    /// take one. Same lookup as the conveyor mouths — one node type, one
    /// place it is authored, one place it is read.
    [[nodiscard]] inline const AttachNode* findPowerNode(const PartDefinition& definition)
    {
        return findConveyorNode(definition, NodeType::Power);
    }

    /// SOLID THINGS. Everything a player can bump into carries a collision
    /// hull; belts and cables deliberately do not. You step over a conveyor
    /// deck and duck under a wire, and making the two things a player walks
    /// among most into obstacles would turn a factory floor into an assault
    /// course. It is a property of the CATEGORY rather than of the file, so
    /// a new belt part cannot forget the rule — and it is separate from
    /// SHAPE: a belt still has hitboxes, the renderer and the build
    /// validator want them.
    [[nodiscard]] inline bool isSolid(const PartDefinition& definition)
    {
        if (!definition.building.valid)
        {
            return true; // rocket parts and props are always solid
        }
        return definition.building.category != factory::BuildingCategory::Conveyor &&
               definition.building.category != factory::BuildingCategory::Cable;
    }

    /// True when this definition describes a planetary building rather than
    /// a rocket part — the VAB palette filters on it.
    [[nodiscard]] inline bool isBuilding(const PartDefinition& definition)
    {
        return definition.building.valid;
    }

    // ---- per-instance components ------------------------------------------------

    struct PartComponent
    {
        u32 definitionId = 0;
        ecs::Entity vessel{};        // the vessel root this part belongs to
        Vec3 localPosition{0.0f};    // vessel-frame pose
        Quat localRotation{1.0f, 0.0f, 0.0f, 0.0f};
        f32 integrity = 1.0f;        // 1 = intact (future damage model)
    };

    /// On the vessel ROOT entity; refreshed by VesselAssemblySystem.
    struct VesselComponent
    {
        f64 dryMassKg = 0.0;
        f64 totalMassKg = 0.0;        // dry + carried resources
        f64 totalCostCredits = 0.0;
        f64 maxThrustNewtons = 0.0;   // all engines at full throttle
        f64 maxMassFlowKgps = 0.0;    // fuel burn at full throttle
        f64 dragCoefficientArea = 0.0;
        f64 solarChargeRateKw = 0.0;
        u32 partCount = 0;
        /// WHERE THE VESSEL BALANCES, in its own frame, and how hard it is
        /// to turn about each of its own axes.
        ///
        /// Both are recomputed every tick, and that is the point: they move
        /// as the tanks drain. A rocket whose centre of mass creeps forward
        /// as it burns grows MORE stable on the way up; one that staged
        /// badly can find its fins ahead of its balance point and flip.
        /// Neither is expressible with a mass and a drag number, and both
        /// fall out of keeping these two fields honest.
        Vec3 centreOfMass{0.0f};
        Vec3 inertiaKgM2{1.0f}; // diagonal, about the centre of mass
        /// Half extents of the vessel's hull, vessel frame. Aerodynamic
        /// damping needs a length, and this is the honest one.
        Vec3 halfExtents{0.5f};
    };

    // ---- joints ------------------------------------------------------------------
    // A joint is its OWN ENTITY between two parts — not an implicit parent
    // pointer. That is what keeps the future honest: an impact stronger
    // than breakForceN destroys the joint entity and the vessel falls
    // apart along real structural lines; decoupling and undocking are the
    // same operation triggered politely.

    enum class JointType : u8
    {
        Stack = 0,
        Radial,
        Docking,
    };

    struct JointComponent
    {
        ecs::Entity partA{}; // the side that stays "up" the tree (root-ward)
        ecs::Entity partB{};
        u8 attachPointA = 0; // indices into each part's definition points
        u8 attachPointB = 0;
        JointType type = JointType::Stack;
        f64 strengthN = 2.0e5;   // static load limit (editor validation)
        f64 breakForceN = 2.0e5; // dynamic impact limit (structural failure)
    };

    /// THE LIVE STATE OF ONE PART'S ANIMATIONS, on the part entity.
    ///
    /// Fixed-size and trivially copyable because every component in this
    /// engine has to be — the save is a column memcpy — and four is the cap
    /// because a part with five things to deploy is two parts.
    ///
    /// `phase` is what is drawn; `target` is where the pilot last told it to
    /// go. Keeping both is what makes a panel caught halfway through opening
    /// close again from where it is rather than snapping shut, and it is also
    /// the whole of the save state: a game reloaded mid-deployment carries on.
    struct PartAnimationComponent
    {
        static constexpr u32 kMaxAnimations = 4;

        f32 phase[kMaxAnimations]{};
        f32 target[kMaxAnimations]{};
        u32 count = 0;
    };

    static_assert(std::is_trivially_copyable_v<PartAnimationComponent>);

    /// How much of a gated capability this part currently has: 1 when no
    /// animation gates it, otherwise the phase of the one that does.
    [[nodiscard]] f64 animationGate(const PartDefinition& definition,
                                    const PartAnimationComponent* state,
                                    AnimationGates gate);

    /// Advances every part's animation phases toward their targets.
    ///
    /// INTEGRATED, not analytic, and this is the one place in the codebase
    /// that goes against the house rule. Everything else that moves with time
    /// — spin, conveyors, orbits — is a closed form of the clock, so that warp
    /// and save/load are exact. An animation cannot be: its start time is
    /// whenever the pilot clicked, which is not a quantity the world knows,
    /// and a three-second door under x1000 warp would open and shut four
    /// hundred times between two frames. Under warp it simply snaps, which is
    /// also what a pilot would see: at a thousand times real time, three
    /// seconds is three milliseconds.
    class PartAnimationSystem final : public ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override
        {
            return "PartAnimationSystem";
        }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .write<PartAnimationComponent>()
                .read<PartComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;
    };

    static_assert(std::is_trivially_copyable_v<PartComponent>);
    static_assert(std::is_trivially_copyable_v<VesselComponent>);
    static_assert(std::is_trivially_copyable_v<JointComponent>);

    // ---- vessel operations (free functions; structural — call from the
    // main thread between simulation ticks, never inside a system) -------------

    /// Creates the joint entity linking two parts of the same vessel.
    ecs::Entity connectParts(ecs::World& world, ecs::Entity partA, ecs::Entity partB,
                             u8 attachPointA, u8 attachPointB, JointType type,
                             f64 strengthN, f64 breakForceN);

    /// Splits `parts` (and everything only connected through them) off
    /// `vessel` into a NEW vessel root cloned from the old one (pose,
    /// velocity + a gentle separation impulse along +Z of the vessel).
    /// Returns the new root (null if nothing detached).
    ecs::Entity splitVessel(ecs::World& world, ecs::Entity vessel,
                            std::span<const ecs::Entity> partsToDetach);

    /// KSP-style decoupling: severs the decoupler's TAIL-side joint (the
    /// one leading away from the root part) and splits the disconnected
    /// component into a new vessel. Returns the new vessel root.
    ecs::Entity decoupleAt(ecs::World& world, ecs::Entity decouplerPart);

    /// Docking: merges portB's whole vessel INTO portA's vessel (parts are
    /// re-localized into A's frame, B's root entity is destroyed) and
    /// joins the two ports. portA's vessel survives — pass the player's
    /// side as A. Returns false if the ports belong to the same vessel.
    bool dockVessels(ecs::World& world, ecs::Entity portPartA, ecs::Entity portPartB);

    // ---- systems -----------------------------------------------------------------

    /// Physics lane, EARLY (before thrust/gravity): sums every part of every
    /// vessel into its VesselComponent and pushes mass/drag into the
    /// vessel's DynamicBody. Runs every tick — part counts are small and
    /// this is what makes fuel burn lighten the rocket.
    class VesselAssemblySystem final : public ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override
        {
            return "VesselAssemblySystem";
        }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .write<VesselComponent>()
                .write<phys::DynamicBodyComponent>()
                .write<phys::GroundHullComponent>()
                .read<PartComponent>()
                .read<factory::InventoryComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;
    };

    /// Physics lane, LATE (after integration/rails): parts ride their
    /// vessel rigidly — world pose = vessel pose composed with the local
    /// pose, previous derived from the vessel's previous (interpolation in
    /// lockstep, the same rule as surface anchors).
    class PartAttachmentSystem final : public ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override
        {
            return "PartAttachmentSystem";
        }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .write<TransformComponent>()
                .write<PreviousTransformComponent>()
                .read<PartComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;
    };
} // namespace sw::parts
