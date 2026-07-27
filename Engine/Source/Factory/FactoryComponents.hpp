#pragma once

// ============================================================================
// Factory/FactoryComponents.hpp
// Industrial automation components. All machines work on INVENTORIES: a
// fixed number of typed slots bounded by real VOLUME (m^3), so denser
// refined materials pack tighter than raw ore — matter is conserved and
// physical from the first machine.
//
//  MinerComponent    : extracts a resource into the entity's own inventory.
//  RefineryComponent : converts input units to output units (with losses)
//                      inside the entity's own inventory.
//  ItemLinkComponent : pulls a resource from ANOTHER entity's inventory
//                      into this one (drone/conveyor abstraction; becomes a
//                      real conveyor entity chain in a later milestone).
//
// Machines run in the Automation lane (5 Hz), transfers in the Logistics
// lane (10 Hz) — production keeps working under time warp at the lanes'
// real-time budget, and never depends on rendering.
// ============================================================================

#include "Core/Types.hpp"
#include "ECS/Entity.hpp"
#include "Factory/Recipes.hpp"
#include "Resources/ResourceTypes.hpp"

#include <string_view>
#include <type_traits>

namespace sw::factory
{
    inline constexpr u32 kInventorySlots = 8;

    struct InventorySlot
    {
        res::Resource resource = res::Resource::Count; // Count == empty slot
        f64 units = 0.0;
    };

    struct InventoryComponent
    {
        InventorySlot slots[kInventorySlots]{};
        f64 volumeCapacityM3 = 10.0;
    };

    struct MinerComponent
    {
        res::Resource output = res::Resource::IronOre;
        f64 unitsPerSecond = 1.0;
        f64 totalMined = 0.0; // lifetime statistics
    };

    struct RefineryComponent
    {
        res::Resource input = res::Resource::IronOre;
        res::Resource output = res::Resource::Iron;
        f64 inputUnitsPerSecond = 1.0;
        f64 conversionRatio = 0.9; // output units per input unit
        f64 totalRefined = 0.0;    // lifetime statistics
    };

    /// ONE feed into a machine: where from, what, how fast at most.
    struct LinkChannel
    {
        ecs::Entity source{};
        res::Resource resource = res::Resource::Count; // Count == unused
        f64 unitsPerSecond = 0.0;
        /// What ACTUALLY moved last tick, units per second. A link is rarely
        /// running at its rated speed — the source starves, the destination
        /// fills — and the difference is the single most useful number about
        /// a factory. The conveyor renderer spaces its cargo by it, so a belt
        /// you look at is running as fast as it looks like it is running,
        /// and F4's UI will graph the same field.
        f64 flowUnitsPerSecond = 0.0;
    };

    /// Eight, to match kMaxMachinePorts: a machine with eight mouths must
    /// be able to be fed by eight belts.
    inline constexpr u32 kMaxLinkChannels = 8;

    /// The feeds into ONE machine.
    ///
    /// It is an array and not a single link because the fuel chain demands
    /// it: the synthesiser takes hydrogen AND oxygen, from the same
    /// electrolyser, down two separate belts. A one-link component made that
    /// chain — the chain the whole of F3 exists to enable — unbuildable.
    /// Four channels is `kMaxRecipeIngredients`, which is not a coincidence:
    /// a machine never needs more feeds than its recipe has inputs.
    struct ItemLinkComponent
    {
        LinkChannel channels[kMaxLinkChannels]{};
    };

    /// Adds a feed. Returns the channel index, or -1 when full. An existing
    /// channel with the same source AND resource is UPDATED rather than
    /// duplicated — two belts between the same pair carrying the same goods
    /// are one logistics link that happens to be drawn twice.
    i32 linkAddChannel(ItemLinkComponent& link, ecs::Entity source,
                       res::Resource resource, f64 unitsPerSecond);

    /// Measured throughput of the channel carrying `resource`, or 0.
    [[nodiscard]] f64 linkFlow(const ItemLinkComponent& link, res::Resource resource);

    /// Measured throughput of everything arriving from one source — what a
    /// single belt between two machines is actually moving, however many
    /// goods it happens to be carrying.
    [[nodiscard]] f64 linkFlowFrom(const ItemLinkComponent& link, ecs::Entity source);

    /// A component carrying exactly one feed — the common case, and what
    /// every call site used before machines needed two.
    [[nodiscard]] inline ItemLinkComponent makeItemLink(ecs::Entity source,
                                                        res::Resource resource,
                                                        f64 unitsPerSecond)
    {
        ItemLinkComponent link{};
        link.channels[0] = {source, resource, unitsPerSecond, 0.0};
        return link;
    }

    // ------------------------------------------------------------------------
    // F1 — the data-driven industry.
    //
    // The three components above (Miner/Refinery/ItemLink) are the ORIGINAL
    // hard-coded machines and stay for the asteroid rig and the orbital
    // station. Everything the player builds on a planet uses the four below
    // instead: a building knows what it IS, a recipe state knows what it is
    // DOING, a power component knows what it costs, and a site owns the
    // books. The behaviour lives in ONE executor, not in one system per
    // machine type.
    // ------------------------------------------------------------------------

    /// A structure standing on a body, belonging to a site.
    struct BuildingComponent
    {
        u32 definitionId = 0;   // .swpart catalogue id (geometry + specs)
        ecs::Entity site{};     // the hub that owns it
        BuildingCategory category = BuildingCategory::Storage;
        /// Deposit density under the footprint, sampled ONCE at build time
        /// from the analytic ore field. A miner's yield is its recipe rate
        /// times this — which is the whole reason siting a mine matters.
        f32 groundDensity = 0.0f;
    };

    /// What this building is currently running, and how that is going.
    struct RecipeStateComponent
    {
        static constexpr u32 kIdle = 0;    // no recipe selected
        static constexpr u32 kRunning = 1;
        static constexpr u32 kStarved = 2; // missing inputs
        static constexpr u32 kBlocked = 3; // output full
        static constexpr u32 kNoPower = 4;
        u32 recipeId = 0;
        u32 state = kIdle;
        /// Lifetime output in units — the number the UI graphs and the tests
        /// balance against the inputs consumed.
        f64 producedUnits = 0.0;
        f64 consumedUnits = 0.0;
    };

    /// Electrical books of one building, written by the site grid every
    /// Automation tick (F3).
    struct PowerComponent
    {
        /// Nameplate output. For SOLAR this is what the panel makes with the
        /// star overhead; the grid multiplies it by the real sun.
        f64 producedKw = 0.0;
        f64 consumedKw = 0.0;
        f64 satisfaction = 1.0; // 0..1 — the fraction of demand actually met
        /// What the panel is ACTUALLY making right now, after elevation and
        /// eclipse. 0 at night, which is the whole point of F3.
        f64 actualProducedKw = 0.0;
        /// Lower is served first in a brownout. Seeded from the category
        /// (factory::defaultPowerPriority) and editable per building.
        u32 priority = 0;
        /// WHICH GRID this building is on — the connected component of the
        /// cable graph it belongs to, recomputed by `rebuildPowerNetwork`
        /// after every build and demolition. A building nobody has wired up
        /// gets a grid of its own, which is exactly right: it runs on
        /// whatever it makes itself, and usually that is nothing.
        u32 gridId = 0;
        /// That grid's books last tick. Duplicated onto every member so the
        /// machine panel is a pure read of one component — 16 bytes against
        /// a lookup through a table that would have to exist somewhere.
        f64 gridProducedKw = 0.0;
        f64 gridConsumedKw = 0.0;
    };

    /// A CABLE: one span of wire between two power nodes.
    ///
    /// Unlike the conveyor network, this one is DECLARED rather than derived,
    /// and the difference is real. A belt is a row of tiles you can see, and
    /// what they connect follows from where their mouths ended up — the
    /// geometry is the statement. A cable has no intermediate object: the
    /// span IS the statement, so it is what gets stored. What is derived
    /// from it, every time, is the GRID (see Factory/PowerNetwork.hpp).
    struct PowerLinkComponent
    {
        ecs::Entity a{};
        ecs::Entity b{};
    };

    /// A BATTERY BANK. It is a building whose inventory holds ElectricCharge
    /// (1 unit = 1 kJ), and whose job is the fourteen-day lunar night: charge
    /// while the sun is up, carry the site through the dark, and — when it
    /// cannot — let the factory stop, honestly, until dawn.
    struct BatteryComponent
    {
        /// How fast it can take charge in and give it back, kW. A bank that
        /// could dump its whole store in a tick would make the grid a
        /// step function instead of a curve.
        f64 maxChargeKw = 400.0;
        f64 maxDischargeKw = 400.0;
        /// Last tick's flow, positive charging. Purely for the UI.
        f64 flowKw = 0.0;
    };

    /// A NAVIGATION BEACON: the one building whose product is being found.
    ///
    /// A surveyed site can be anywhere on a 6,371 km sphere, and from the
    /// air one patch of ground looks like every other patch of ground. The
    /// beacon is the answer: it carries a name and a range, and the game
    /// layer draws a pointer at it — on the map always, and in the cockpit
    /// once you are inside `rangeM` — with the live distance under it.
    ///
    /// It is a BUILDING, not a HUD setting: it is placed, it is saved, it
    /// draws idle power, and in F2 the player plants one wherever they want
    /// to be able to find their way back to.
    struct BeaconComponent
    {
        char label[16]{};
        /// How close the player must be for the pointer to appear in the
        /// first-person view. The map ignores it — a map that hid the thing
        /// you were looking for would not be a map.
        f64 rangeM = 1000000.0; // 1000 km: findable from orbit
        /// ...and how close is TOO close. Standing in the middle of the base
        /// you can see the base; a reticle and a distance readout pinned over
        /// it are then pure obstruction. The map ignores this one too.
        f64 nearRangeM = 500.0;
    };

    /// THE VAB, as the simulation sees it (F5).
    ///
    /// An assembly hall is not a recipe machine, and trying to make it one
    /// was the wrong shape: a recipe is a fixed list of goods per second,
    /// while a rocket's bill of materials depends on the DESIGN the player
    /// composed this morning. So the hall holds an order — a name and the
    /// metal that order costs — and the executor for it is the simplest
    /// thing in the factory: pour iron and copper in at a rate until the
    /// bill is paid, then a vehicle exists.
    ///
    /// The bill is stored in KILOGRAMS and not looked up, deliberately. The
    /// factory layer knows nothing about parts, hangars or blueprints; the
    /// game layer costs the design once, when the player picks it, and hands
    /// the number down. One unit of Iron is one kilogram of iron, so the
    /// arithmetic below never has to convert anything.
    struct AssemblyComponent
    {
        static constexpr u32 kNameChars = 24;
        /// The ordered design. Empty means nothing is on the slipway.
        char blueprint[kNameChars]{};
        /// What that design costs, from parts::blueprintCost.
        f64 ironNeededKg = 0.0;
        f64 copperNeededKg = 0.0;
        /// ...and what has actually gone into it so far. This is the only
        /// state that matters: pause the hall, change nothing, come back in
        /// a fortnight and the half-built rocket is still half built.
        f64 ironPaidKg = 0.0;
        f64 copperPaidKg = 0.0;
        /// How fast metal can be worked, kilograms per second, at full
        /// power. A twelve-tonne airframe at 40 kg/s is five minutes of
        /// factory time — long enough to be a supply problem, short enough
        /// to watch.
        f64 buildRateKgPerSecond = 40.0;
        /// Lifetime vehicles finished here.
        u32 completed = 0;
        /// Same vocabulary as RecipeStateComponent, so one panel reads both.
        u32 state = RecipeStateComponent::kIdle;
    };

    /// How much of the order is done, 0..1. Progress on the METAL, which is
    /// the honest measure: a rocket that is missing its copper is not
    /// ninety-five per cent finished just because the iron all arrived.
    [[nodiscard]] f64 assemblyProgress(const AssemblyComponent& assembly);

    /// Sets the order and clears the slipway. Passing an empty name idles
    /// the hall.
    void assemblyOrder(AssemblyComponent& assembly, std::string_view name, f64 ironKg,
                       f64 copperKg);

    /// WHAT ARRIVED. A `Resource::Vehicle` unit is one rocket, but a unit
    /// carries no identity — a belt cannot tell the pad whether the crate
    /// that just landed on it holds a lander or a heavy lifter.
    ///
    /// So the names travel alongside, in a queue on the HALL that built
    /// them: it pushes a name when it finishes a hull, and the pad pops one
    /// when it unpacks a crate — reaching the right queue through the belt's
    /// own link channel, which already names the machine at the far end.
    ///
    /// It works because a belt is first-in first-out and because the crates
    /// on one belt all came from one hall. Route vehicles through a silo and
    /// the identity is lost, which is the honest consequence of a unit not
    /// being a thing: build the pad's feed straight off the VAB.
    inline constexpr u32 kVehicleQueueSlots = 8;

    struct VehicleQueueComponent
    {
        char names[kVehicleQueueSlots][AssemblyComponent::kNameChars]{};
        u32 count = 0;
    };

    /// Appends a name. Returns false when the queue is full — the hall then
    /// stays BLOCKED rather than shipping a rocket nobody can identify.
    bool vehicleQueuePush(VehicleQueueComponent& queue, std::string_view name);
    /// The oldest name, or empty when there is none.
    [[nodiscard]] std::string_view vehicleQueueFront(const VehicleQueueComponent& queue);
    /// Drops the oldest name.
    void vehicleQueuePop(VehicleQueueComponent& queue);

    /// The hub entity: a site's identity and its aggregate books.
    struct SiteComponent
    {
        char name[16]{};
        ecs::Entity body{};      // the celestial body it stands on
        f64 producedKw = 0.0;    // last tick, summed over the site
        f64 consumedKw = 0.0;
        /// Net battery flow last tick, positive charging. Negative means the
        /// site is living off its banks — which on a lunar night is the most
        /// important number on the screen.
        f64 batteryFlowKw = 0.0;
        u32 buildingCount = 0;
    };

    static_assert(std::is_trivially_copyable_v<InventoryComponent>);
    static_assert(std::is_trivially_copyable_v<BuildingComponent>);
    static_assert(std::is_trivially_copyable_v<RecipeStateComponent>);
    static_assert(std::is_trivially_copyable_v<PowerComponent>);
    static_assert(std::is_trivially_copyable_v<BatteryComponent>);
    static_assert(std::is_trivially_copyable_v<PowerLinkComponent>);
    static_assert(std::is_trivially_copyable_v<SiteComponent>);
    static_assert(std::is_trivially_copyable_v<AssemblyComponent>);
    static_assert(std::is_trivially_copyable_v<VehicleQueueComponent>);
    static_assert(std::is_trivially_copyable_v<BeaconComponent>);
    static_assert(std::is_trivially_copyable_v<MinerComponent>);
    static_assert(std::is_trivially_copyable_v<RefineryComponent>);
    static_assert(std::is_trivially_copyable_v<ItemLinkComponent>);

    // ---- inventory operations (free functions, unit-tested) -----------------

    /// Total occupied volume in m^3.
    [[nodiscard]] f64 inventoryVolume(const InventoryComponent& inventory);

    /// Units of one resource currently stored.
    [[nodiscard]] f64 inventoryCount(const InventoryComponent& inventory,
                                     res::Resource resource);

    /// Adds up to `units`; bounded by volume capacity and free slots.
    /// Returns the units actually accepted.
    f64 inventoryAdd(InventoryComponent& inventory, res::Resource resource, f64 units);

    /// Removes up to `units`. Returns the units actually removed.
    f64 inventoryRemove(InventoryComponent& inventory, res::Resource resource, f64 units);

    /// How many more units of `resource` would fit — volume AND slots.
    /// The recipe executor needs this BEFORE it consumes anything: a
    /// machine that eats its input and then discovers the output bin is
    /// full has destroyed matter.
    [[nodiscard]] f64 inventoryFreeUnits(const InventoryComponent& inventory,
                                         res::Resource resource);
} // namespace sw::factory
