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

    struct ItemLinkComponent
    {
        ecs::Entity source{};
        res::Resource resource = res::Resource::IronOre;
        f64 unitsPerSecond = 1.0;
        /// What ACTUALLY moved last tick, units per second. A link is rarely
        /// running at its rated speed — the source starves, the destination
        /// fills — and the difference is the single most useful number about
        /// a factory. The conveyor renderer spaces its cargo by it, so a belt
        /// you look at is running as fast as it looks like it is running,
        /// and F4's UI will graph the same field.
        f64 flowUnitsPerSecond = 0.0;
    };

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

    /// Electrical books of one building. The site grid fills `satisfaction`
    /// each tick (F3); until then everything runs at full power.
    struct PowerComponent
    {
        f64 producedKw = 0.0;
        f64 consumedKw = 0.0;
        f64 satisfaction = 1.0; // 0..1 — the fraction of demand actually met
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

    /// The hub entity: a site's identity and its aggregate books.
    struct SiteComponent
    {
        char name[16]{};
        ecs::Entity body{};      // the celestial body it stands on
        f64 producedKw = 0.0;    // last tick, summed over the site
        f64 consumedKw = 0.0;
        u32 buildingCount = 0;
    };

    static_assert(std::is_trivially_copyable_v<InventoryComponent>);
    static_assert(std::is_trivially_copyable_v<BuildingComponent>);
    static_assert(std::is_trivially_copyable_v<RecipeStateComponent>);
    static_assert(std::is_trivially_copyable_v<PowerComponent>);
    static_assert(std::is_trivially_copyable_v<SiteComponent>);
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
