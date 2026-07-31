// ============================================================================
// FactoryTests.cpp — inventories, machines and the full production chain.
// ============================================================================

#include "TestFramework.hpp"

#include <ECS/World.hpp>
#include <Factory/FactorySystems.hpp>
#include <Simulation/Simulation.hpp>

#include <cmath>

using namespace sw;
using namespace sw::factory;
using sw::res::Resource;

SW_TEST(InventoryVolumeBoundsAndStacking)
{
    InventoryComponent inventory{};
    inventory.volumeCapacityM3 = 1.0; // 1 m^3

    // Iron ore: 1/2500 m^3 per unit -> capacity 2500 units.
    const f64 accepted = inventoryAdd(inventory, Resource::IronOre, 3000.0);
    SW_CHECK(std::abs(accepted - 2500.0) < 1.0e-6);
    SW_CHECK(std::abs(inventoryCount(inventory, Resource::IronOre) - 2500.0) < 1.0e-6);
    SW_CHECK(inventoryAdd(inventory, Resource::IronOre, 1.0) == 0.0); // full

    // Removing frees volume; the same stack is reused (one slot).
    SW_CHECK(std::abs(inventoryRemove(inventory, Resource::IronOre, 500.0) - 500.0) <
             1.0e-6);
    SW_CHECK(std::abs(inventoryAdd(inventory, Resource::IronOre, 400.0) - 400.0) < 1.0e-6);

    // Removing more than stored returns only what exists.
    const f64 removed = inventoryRemove(inventory, Resource::IronOre, 1.0e9);
    SW_CHECK(std::abs(removed - 2400.0) < 1.0e-6);
    SW_CHECK(inventoryCount(inventory, Resource::IronOre) == 0.0);
}

SW_TEST(RefineryConservesMatterAndStallsWhenFull)
{
    ecs::World world;
    const ecs::Entity plant = world.createEntity();

    InventoryComponent inventory{};
    inventory.volumeCapacityM3 = 1.0;
    world.addComponent(plant, inventory);
    world.addComponent(plant, RefineryComponent{Resource::IronOre, Resource::Iron,
                                                /*in/s*/ 10.0, /*ratio*/ 0.9, 0.0});
    auto& inv = world.getComponent<InventoryComponent>(plant);
    inventoryAdd(inv, Resource::IronOre, 100.0);

    RefinerySystem refinery;
    refinery.update(world, 1.0f); // one second: consume 10 ore -> 9 iron

    SW_CHECK(std::abs(inventoryCount(inv, Resource::IronOre) - 90.0) < 1.0e-9);
    SW_CHECK(std::abs(inventoryCount(inv, Resource::Iron) - 9.0) < 1.0e-9);
    SW_CHECK(std::abs(world.getComponent<RefineryComponent>(plant).totalRefined - 9.0) <
             1.0e-9);

    // Starved refinery does nothing.
    inventoryRemove(inv, Resource::IronOre, 1.0e9);
    const f64 ironBefore = inventoryCount(inv, Resource::Iron);
    refinery.update(world, 1.0f);
    SW_CHECK(inventoryCount(inv, Resource::Iron) == ironBefore);
}

// A REFINERY MAY NOT INVENT MATTER, AND MAY NOT POISON ITS OWN BIN.
//
// `conversionRatio` is the one production number in the factory that nothing
// validates on the way in. The recipe loader refuses a .swrecipe whose
// outputs outweigh its inputs, but this component is not a recipe: it is
// written by the asteroid rig, restored by the save loader, and reachable by
// anything that can set an f64. The three values that hurt are the three
// pinned here — a ratio above unity (matter from nowhere), a ratio at or
// below zero (the divide on the consumption line), and NaN, which is by far
// the worst of them because `inventoryAdd` has no defence against it and one
// tick turns every slot in the bin into a quiet NaN that then spreads down
// the belt to everything downstream.
SW_TEST(RefineryRatioIsBoundedAndACorruptRatioIsInert)
{
    // A ratio of 2 asks for two kilograms of iron out of one of ore. Iron ore
    // and iron are both 1 kg per unit, so the honest ceiling is 1.0 and the
    // machine is held to it: ten units in, at most ten units out.
    {
        ecs::World world;
        const ecs::Entity plant = world.createEntity();
        InventoryComponent inventory{};
        inventory.volumeCapacityM3 = 100.0;
        world.addComponent(plant, inventory);
        world.addComponent(plant, RefineryComponent{Resource::IronOre, Resource::Iron,
                                                    10.0, 2.0, 0.0});
        auto& inv = world.getComponent<InventoryComponent>(plant);
        inventoryAdd(inv, Resource::IronOre, 100.0);

        RefinerySystem refinery;
        refinery.update(world, 1.0f);

        const f64 ore = inventoryCount(inv, Resource::IronOre);
        const f64 iron = inventoryCount(inv, Resource::Iron);
        const f64 consumed = 100.0 - ore;
        SW_CHECK(std::abs(consumed - 10.0) < 1.0e-9);
        SW_CHECK(std::abs(iron - 10.0) < 1.0e-9); // NOT 20
        // The clamp is written back, so the panel and the save file quote the
        // number the simulation actually honoured.
        SW_CHECK(std::abs(world.getComponent<RefineryComponent>(plant).conversionRatio -
                          1.0) < 1.0e-12);
    }

    // Zero, negative and NaN are INERT: the hopper is untouched, nothing is
    // produced, and — the part that matters — no slot in the bin has been
    // turned into a NaN that would spread through every inventory the belts
    // touch.
    const f64 nan = std::nan("");
    for (const f64 ratio : {0.0, -0.5, nan})
    {
        ecs::World world;
        const ecs::Entity again = world.createEntity();
        InventoryComponent inventory{};
        inventory.volumeCapacityM3 = 100.0;
        world.addComponent(again, inventory);
        world.addComponent(again, RefineryComponent{Resource::IronOre, Resource::Iron,
                                                    10.0, ratio, 0.0});
        auto& inv = world.getComponent<InventoryComponent>(again);
        inventoryAdd(inv, Resource::IronOre, 100.0);
        RefinerySystem refinery;
        refinery.update(world, 1.0f);

        SW_CHECK(std::isfinite(inventoryCount(inv, Resource::IronOre)));
        SW_CHECK(std::isfinite(inventoryVolume(inv)));
        SW_CHECK(std::abs(inventoryCount(inv, Resource::IronOre) - 100.0) < 1.0e-12);
        SW_CHECK(inventoryCount(inv, Resource::Iron) == 0.0);
        const auto& after = world.getComponent<RefineryComponent>(again);
        SW_CHECK(after.totalRefined == 0.0);
        SW_CHECK(std::isfinite(after.conversionRatio));
        SW_CHECK(after.conversionRatio == 0.0); // normalised, not left as NaN
    }
}

SW_TEST(FullProductionChainThroughSimulationLanes)
{
    // Miner (asteroid) --link--> refinery --link--> storage, driven by the
    // real Automation (5 Hz) and Logistics (10 Hz) lanes for 60 simulated
    // seconds, fed in uneven frame slices.
    ecs::World world;
    sim::Simulation simulation({{"Logistics", 10.0f, 8}, {"Automation", 5.0f, 8}});

    const ecs::Entity rock = world.createEntity();
    {
        InventoryComponent hopper{};
        hopper.volumeCapacityM3 = 10.0;
        world.addComponent(rock, hopper);
        world.addComponent(rock, MinerComponent{Resource::IronOre, 2.0, 0.0});
    }
    const ecs::Entity refinery = world.createEntity();
    {
        InventoryComponent tanks{};
        tanks.volumeCapacityM3 = 10.0;
        world.addComponent(refinery, tanks);
        world.addComponent(refinery, RefineryComponent{Resource::IronOre, Resource::Iron,
                                                       1.0, 0.9, 0.0});
        world.addComponent(refinery, ItemLinkComponent{rock, Resource::IronOre, 1.5});
    }
    const ecs::Entity depot = world.createEntity();
    {
        InventoryComponent silo{};
        silo.volumeCapacityM3 = 10.0;
        world.addComponent(depot, silo);
        world.addComponent(depot, ItemLinkComponent{refinery, Resource::Iron, 1.0});
    }

    simulation.findLane("Automation")->scheduler().addSystem(std::make_unique<MinerSystem>());
    simulation.findLane("Automation")
        ->scheduler()
        .addSystem(std::make_unique<RefinerySystem>());
    simulation.findLane("Logistics")->scheduler().addSystem(std::make_unique<TransferSystem>());

    for (int i = 0; i < 60 * 8; ++i) // 60 s in 0.125 s frames (exact fp)
    {
        simulation.advance(world, 0.125f, nullptr);
    }

    // Mining: 2 u/s * 60 s = 120 mined. The chain is limited by the
    // 1 u/s refinery input over ~60 s => ~54 iron produced (0.9 yield),
    // most of it hauled to the depot by the 1 u/s link.
    const f64 mined = world.getComponent<MinerComponent>(rock).totalMined;
    const f64 refined = world.getComponent<RefineryComponent>(refinery).totalRefined;
    const f64 depotIron =
        inventoryCount(world.getComponent<InventoryComponent>(depot), Resource::Iron);

    // 5 Hz step (0.2 s) is not exact in binary floating point: the last
    // tick of the hour may land just past the total, so 299 or 300 ticks
    // are both correct (119.6 or 120.0 units).
    SW_CHECK(mined >= 119.5 && mined <= 120.0 + 1.0e-9);
    SW_CHECK(refined > 50.0 && refined <= 54.0 + 1.0e-9);
    SW_CHECK(depotIron > 45.0 && depotIron <= refined);

    // Matter conservation: every mined unit is ore somewhere or was
    // converted at exactly ratio 0.9 (no losses to bugs).
    f64 oreEverywhere = 0.0;
    world.forEach<InventoryComponent>([&](ecs::Entity, InventoryComponent& inventory) {
        oreEverywhere += inventoryCount(inventory, Resource::IronOre);
    });
    const f64 oreConsumed = refined / 0.9;
    SW_CHECK(std::abs(oreEverywhere + oreConsumed - mined) < 1.0e-6);
}
