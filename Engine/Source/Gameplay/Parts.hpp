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
        f64 dragCoefficientArea = 0.8;  // Cd * A, m^2 (feeds atmospheric drag)
        f64 liftCoefficient = 0.0;      // wings; used by the future aero pass

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
    };

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
