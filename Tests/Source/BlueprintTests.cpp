// ============================================================================
// BlueprintTests.cpp — a design is a file, and a rocket costs metal.
//
// F5 turns the hangar from a toy into an industry, and there are exactly
// three claims in that which can be checked without a screen:
//
//   1. THE BILL IS EXACT. Iron plus copper equals the part's dry mass, to
//      the last gram, for every part in the catalogue and every fraction
//      anyone might set later. That is what lets an assembly hall claim to
//      conserve matter: twelve tonnes of metal in, a twelve-tonne airframe
//      out, and no rounding hiding in between.
//
//   2. THE SPLIT MEANS SOMETHING. Electrical parts are mostly copper and
//      structure is mostly iron — that is the whole reason the copper chain
//      exists, and a change that quietly made a fuel tank copper-rich would
//      make the second half of the factory pointless.
//
//   3. A DESIGN SURVIVES DISK. Poses, rotations AND JOINTS. A .swship that
//      forgot its parent/child pairs would load as a pile of parts flying in
//      formation — it would even look right until something pushed on it.
//
// ...and one thing that needs a world: the hall itself, which must consume
// exactly the bill and not a gram more, stall honestly when a metal runs
// out, and hand the design's name to the belt alongside the crate.
// ============================================================================

#include "TestFramework.hpp"

#include <Core/FileSystem.hpp>
#include <ECS/World.hpp>
#include <Factory/FactoryComponents.hpp>
#include <Factory/FactorySystems.hpp>
#include <Gameplay/Blueprint.hpp>
#include <Gameplay/Parts.hpp>

#include <cmath>
#include <filesystem>

using namespace sw;
using namespace sw::parts;

namespace
{
    [[nodiscard]] ShipBlueprint threePartStack()
    {
        ShipBlueprint design{};
        design.name = "TEST STACK";
        BlueprintPartRecord core{};
        core.definitionId = kPartCoreStructural;
        design.parts.push_back(core);

        BlueprintPartRecord tank{};
        tank.definitionId = kPartFuelTankMedium;
        tank.localPosition = {0.0f, 0.0f, 3.4f};
        tank.parentIndex = 0;
        tank.parentPoint = 1;
        tank.childPoint = 0;
        design.parts.push_back(tank);

        BlueprintPartRecord engine{};
        engine.definitionId = kPartEngineVector;
        engine.localPosition = {0.0f, 0.0f, 6.6f};
        engine.localRotation = glm::angleAxis(0.5f, Vec3{0.0f, 1.0f, 0.0f});
        engine.parentIndex = 1;
        engine.parentPoint = 1;
        engine.childPoint = 0;
        engine.symmetryGroup = 3;
        design.parts.push_back(engine);
        return design;
    }
} // namespace

SW_TEST(EveryPartsBillIsExactlyItsDryMass)
{
    // Whatever the catalogue holds — built-in fallback or the shipped
    // files — no part may cost more or less metal than it weighs.
    for (const PartDefinition& definition : catalog())
    {
        const BillOfMaterials bill = partCost(definition);
        SW_CHECK(bill.ironKg >= 0.0 && bill.copperKg >= 0.0);
        SW_CHECK(std::abs(bill.totalKg() - definition.dryMassKg) < 1.0e-9);
    }
}

SW_TEST(ElectricalPartsAreCopperAndStructureIsIron)
{
    // The SHAPE of the split, not its exact numbers: a battery is mostly
    // copper, a fuel tank is mostly iron, and an engine sits between them.
    SW_CHECK(copperFraction(PartType::Battery) > 0.5);
    SW_CHECK(copperFraction(PartType::SolarPanel) > 0.5);
    SW_CHECK(copperFraction(PartType::FuelTank) < 0.1);
    SW_CHECK(copperFraction(PartType::Structural) < 0.1);
    SW_CHECK(copperFraction(PartType::Engine) > copperFraction(PartType::FuelTank));
    SW_CHECK(copperFraction(PartType::Engine) < copperFraction(PartType::Battery));

    // ...and the design's bill is the sum of its parts', which is the only
    // reason the panel can price a rocket before it is built.
    const ShipBlueprint design = threePartStack();
    BillOfMaterials expected{};
    for (const BlueprintPartRecord& record : design.parts)
    {
        const PartDefinition* definition = findDefinition(record.definitionId);
        SW_CHECK(definition != nullptr);
        if (definition == nullptr) { return; }
        const BillOfMaterials part = partCost(*definition);
        expected.ironKg += part.ironKg;
        expected.copperKg += part.copperKg;
    }
    const BillOfMaterials bill = blueprintCost(design);
    SW_CHECK(std::abs(bill.ironKg - expected.ironKg) < 1.0e-9);
    SW_CHECK(std::abs(bill.copperKg - expected.copperKg) < 1.0e-9);
    // The whole thing weighs what it costs.
    SW_CHECK(std::abs(bill.totalKg() - blueprintDryMassKg(design)) < 1.0e-9);
}

SW_TEST(ADesignSurvivesDiskWithItsJoints)
{
    const ShipBlueprint design = threePartStack();
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "sw_test_design.swship";
    SW_CHECK(saveBlueprintFile(design, path));

    ShipBlueprint loaded{};
    SW_CHECK(loadBlueprintFile(path, loaded));
    SW_CHECK(loaded.name == design.name);
    SW_CHECK(loaded.parts.size() == design.parts.size());
    if (loaded.parts.size() != design.parts.size()) { return; }
    for (usize i = 0; i < loaded.parts.size(); ++i)
    {
        const BlueprintPartRecord& a = design.parts[i];
        const BlueprintPartRecord& b = loaded.parts[i];
        SW_CHECK(a.definitionId == b.definitionId);
        SW_CHECK(glm::length(a.localPosition - b.localPosition) < 1.0e-5f);
        SW_CHECK(std::abs(glm::dot(a.localRotation, b.localRotation)) > 0.99999f);
        // THE JOINTS. Losing these loads a pile of parts, not a rocket.
        SW_CHECK(a.parentIndex == b.parentIndex);
        SW_CHECK(a.parentPoint == b.parentPoint);
        SW_CHECK(a.childPoint == b.childPoint);
        SW_CHECK(a.symmetryGroup == b.symmetryGroup);
    }
    std::error_code error{};
    std::filesystem::remove(path, error);

    // A design naming a part this build does not have is not buildable, and
    // says so rather than costing zero-mass metal.
    ShipBlueprint broken{};
    broken.name = "GHOST";
    BlueprintPartRecord missing{};
    missing.definitionId = 999999;
    broken.parts.push_back(missing);
    SW_CHECK(!blueprintIsBuildable(broken));
    SW_CHECK(blueprintCost(broken).totalKg() < 1.0e-12);
    SW_CHECK(!blueprintIsBuildable(ShipBlueprint{})); // and nor is nothing
}

SW_TEST(AHallConsumesExactlyTheBillAndCratesOneVehicle)
{
    ecs::World world;
    const ecs::Entity hall = world.createEntity();

    const ShipBlueprint design = threePartStack();
    const BillOfMaterials bill = blueprintCost(design);

    factory::AssemblyComponent assembly{};
    factory::assemblyOrder(assembly, design.name, bill.ironKg, bill.copperKg);
    assembly.buildRateKgPerSecond = 100.0;
    world.addComponent(hall, assembly);
    world.addComponent(hall, factory::VehicleQueueComponent{});

    factory::InventoryComponent inventory{};
    inventory.volumeCapacityM3 = 200.0; // a crate is 60 m^3 of it
    // TWICE the bill, so a shortage cannot be what stops it.
    factory::inventoryAdd(inventory, res::Resource::Iron, bill.ironKg * 2.0);
    factory::inventoryAdd(inventory, res::Resource::Copper, bill.copperKg * 2.0);
    world.addComponent(hall, inventory);

    factory::AssemblySystem system;
    // Bulk catch-up: one long tick and many short ones must agree, so the
    // hall behaves the same at warp as it does at 1x.
    for (int i = 0; i < 200; ++i)
    {
        system.update(world, 1.0f);
    }

    const auto& after = world.getComponent<factory::InventoryComponent>(hall);
    const auto& built = world.getComponent<factory::AssemblyComponent>(hall);
    // A hall does not stop after one: it keeps building the order until the
    // metal runs out, so what must hold is that the CRATES and the FINISHED
    // count agree — one unit in the bin per hull, never a fraction of one.
    SW_CHECK(built.completed >= 1);
    SW_CHECK(std::abs(factory::inventoryCount(after, res::Resource::Vehicle) -
                      static_cast<f64>(built.completed)) < 1.0e-9);
    // ...and the metal ledger closes exactly: everything missing from the
    // bin is either in a finished hull or on the slipway. Metal that went
    // anywhere else would be metal destroyed.
    const f64 ironSpent =
        bill.ironKg * 2.0 - factory::inventoryCount(after, res::Resource::Iron);
    const f64 copperSpent =
        bill.copperKg * 2.0 - factory::inventoryCount(after, res::Resource::Copper);
    const f64 hullsPaid = static_cast<f64>(built.completed);
    SW_CHECK(std::abs(ironSpent - (bill.ironKg * hullsPaid + built.ironPaidKg)) < 1.0e-6);
    SW_CHECK(std::abs(copperSpent - (bill.copperKg * hullsPaid + built.copperPaidKg)) <
             1.0e-6);

    // The design's NAME left with the crate: that is how the pad knows what
    // it is unpacking.
    const auto& queue = world.getComponent<factory::VehicleQueueComponent>(hall);
    SW_CHECK(queue.count >= 1);
    if (queue.count == 0) { return; }
    SW_CHECK(factory::vehicleQueueFront(queue) == design.name);
}

SW_TEST(AHallWithNoCopperStallsInsteadOfHoardingIron)
{
    ecs::World world;
    const ecs::Entity hall = world.createEntity();

    factory::AssemblyComponent assembly{};
    factory::assemblyOrder(assembly, "HEAVY", 1000.0, 200.0);
    assembly.buildRateKgPerSecond = 60.0;
    world.addComponent(hall, assembly);

    factory::InventoryComponent inventory{};
    inventory.volumeCapacityM3 = 200.0;
    factory::inventoryAdd(inventory, res::Resource::Iron, 5000.0);
    // ...and NO copper at all.
    world.addComponent(hall, inventory);

    factory::AssemblySystem system;
    for (int i = 0; i < 100; ++i)
    {
        system.update(world, 1.0f);
    }

    const auto& built = world.getComponent<factory::AssemblyComponent>(hall);
    const auto& after = world.getComponent<factory::InventoryComponent>(hall);
    SW_CHECK(built.state == factory::RecipeStateComponent::kStarved);
    SW_CHECK(built.completed == 0);
    SW_CHECK(factory::inventoryCount(after, res::Resource::Vehicle) < 1.0e-12);
    // The iron it took is the iron the FINISHED share of the bill would use,
    // not every gram in the bin: 1000 kg of iron against 200 kg of copper is
    // five parts iron to one, and with no copper the pour stops at the iron
    // that matches — the rest stays where a smelter can send it elsewhere.
    SW_CHECK(built.ironPaidKg <= 1000.0 + 1.0e-6);
    SW_CHECK(factory::inventoryCount(after, res::Resource::Iron) > 4000.0 - 1.0e-6);
}

SW_TEST(AHallWithNoRoomForTheCrateBlocksRatherThanLosingIt)
{
    ecs::World world;
    const ecs::Entity hall = world.createEntity();

    factory::AssemblyComponent assembly{};
    factory::assemblyOrder(assembly, "SMALL", 100.0, 20.0);
    assembly.buildRateKgPerSecond = 1000.0;
    world.addComponent(hall, assembly);

    factory::InventoryComponent inventory{};
    inventory.volumeCapacityM3 = 5.0; // nowhere near a 60 m^3 cradle
    factory::inventoryAdd(inventory, res::Resource::Iron, 100.0);
    factory::inventoryAdd(inventory, res::Resource::Copper, 20.0);
    world.addComponent(hall, inventory);

    factory::AssemblySystem system;
    for (int i = 0; i < 10; ++i)
    {
        system.update(world, 1.0f);
    }

    const auto& built = world.getComponent<factory::AssemblyComponent>(hall);
    const auto& after = world.getComponent<factory::InventoryComponent>(hall);
    SW_CHECK(built.state == factory::RecipeStateComponent::kBlocked);
    SW_CHECK(built.completed == 0);
    // Not a gram was worked, because there was never anywhere to put the
    // result. A machine that consumed its input first would have destroyed
    // the metal outright.
    SW_CHECK(std::abs(factory::inventoryCount(after, res::Resource::Iron) - 100.0) <
             1.0e-9);
    SW_CHECK(std::abs(factory::inventoryCount(after, res::Resource::Copper) - 20.0) <
             1.0e-9);
}

SW_TEST(ACrateRidesTheBeltFromTheHallToThePad)
{
    // The logistics half of the loop, with no game layer in it: a hall, a
    // link standing in for the belt, and a pad at the far end. What the pad
    // must end up with is a whole vehicle unit AND a way back to the name.
    ecs::World world;
    const ecs::Entity hall = world.createEntity();
    const ecs::Entity pad = world.createEntity();

    factory::AssemblyComponent assembly{};
    factory::assemblyOrder(assembly, "LANDER", 400.0, 100.0);
    assembly.buildRateKgPerSecond = 250.0;
    world.addComponent(hall, assembly);
    world.addComponent(hall, factory::VehicleQueueComponent{});

    factory::InventoryComponent hallBin{};
    hallBin.volumeCapacityM3 = 200.0;
    factory::inventoryAdd(hallBin, res::Resource::Iron, 400.0);
    factory::inventoryAdd(hallBin, res::Resource::Copper, 100.0);
    world.addComponent(hall, hallBin);

    factory::InventoryComponent padBin{};
    padBin.volumeCapacityM3 = 300.0;
    world.addComponent(pad, padBin);
    world.addComponent(pad, factory::makeItemLink(hall, res::Resource::Vehicle, 3.0));

    factory::AssemblySystem hallSystem;
    factory::TransferSystem belt;
    for (int i = 0; i < 20; ++i)
    {
        hallSystem.update(world, 0.5f);
        belt.update(world, 0.5f);
    }

    // One rocket, whole, at the pad — not a fraction of one, and not two.
    const auto& arrived = world.getComponent<factory::InventoryComponent>(pad);
    SW_CHECK(std::abs(factory::inventoryCount(arrived, res::Resource::Vehicle) - 1.0) <
             1.0e-9);
    // The hall's bin is empty again, so nothing was duplicated in transit.
    const auto& shipped = world.getComponent<factory::InventoryComponent>(hall);
    SW_CHECK(factory::inventoryCount(shipped, res::Resource::Vehicle) < 1.0e-9);
    // ...and the name is still on the hall's queue, which is where the pad
    // reads it from through this very link.
    const auto& queue = world.getComponent<factory::VehicleQueueComponent>(hall);
    SW_CHECK(factory::vehicleQueueFront(queue) == "LANDER");
}

SW_TEST(TheVehicleQueueIsFirstInFirstOut)
{
    factory::VehicleQueueComponent queue{};
    SW_CHECK(factory::vehicleQueueFront(queue).empty());
    SW_CHECK(factory::vehicleQueuePush(queue, "ALPHA"));
    SW_CHECK(factory::vehicleQueuePush(queue, "BRAVO"));
    SW_CHECK(factory::vehicleQueueFront(queue) == "ALPHA");
    factory::vehicleQueuePop(queue);
    SW_CHECK(factory::vehicleQueueFront(queue) == "BRAVO");
    factory::vehicleQueuePop(queue);
    SW_CHECK(queue.count == 0);
    factory::vehicleQueuePop(queue); // popping nothing is not a crash

    // It is bounded, and says so rather than overwriting the oldest hull.
    for (u32 i = 0; i < factory::kVehicleQueueSlots; ++i)
    {
        SW_CHECK(factory::vehicleQueuePush(queue, "SHIP"));
    }
    SW_CHECK(!factory::vehicleQueuePush(queue, "ONE TOO MANY"));

    // A name longer than the field is truncated, never written past its end.
    factory::VehicleQueueComponent tight{};
    SW_CHECK(factory::vehicleQueuePush(
        tight, "A NAME FAR LONGER THAN TWENTY THREE CHARACTERS"));
    SW_CHECK(factory::vehicleQueueFront(tight).size() ==
             factory::AssemblyComponent::kNameChars - 1);
}

SW_TEST(TheShippedDesignsLoadAndArePriced)
{
    // The catalogue on disk, on the same contract as .swpart and .swrecipe.
    const std::filesystem::path directory =
        FileSystem::executableDirectory() / "Assets" / "Ships";
    if (!std::filesystem::is_directory(directory))
    {
        return; // a build with no shipped designs is allowed
    }
    SW_CHECK(loadCatalog(FileSystem::executableDirectory() / "Assets" / "Parts"));
    SW_CHECK(loadBlueprintCatalog(directory));
    SW_CHECK(!blueprintCatalog().empty());
    for (const ShipBlueprint& design : blueprintCatalog())
    {
        SW_CHECK(!design.name.empty());
        SW_CHECK(!design.parts.empty());
        // Every part it names exists, so the VAB can actually build it...
        SW_CHECK(blueprintIsBuildable(design));
        // ...and it costs what it weighs.
        const BillOfMaterials bill = blueprintCost(design);
        SW_CHECK(bill.totalKg() > 0.0);
        SW_CHECK(std::abs(bill.totalKg() - blueprintDryMassKg(design)) < 1.0e-6);
        // Every joint points BACKWARDS, at a part already placed. A forward
        // reference would be a joint to a part that does not exist yet.
        for (usize i = 0; i < design.parts.size(); ++i)
        {
            SW_CHECK(design.parts[i].parentIndex < static_cast<i32>(i));
        }
    }
}

// ============================================================================
// F47: CREATIVE MODE PAYS NOTHING FOR THE METAL — and waits the same.
//
// A creative session that still had to mine, smelt and belt twelve tonnes
// before it could fly a design is creative in name only: the mode exists to
// put a craft in the air, and the craft is the one thing the factory stood
// between the player and. What it does NOT waive is the labour, because a hall
// that finished instantly would stop being a machine.
// ============================================================================
SW_TEST(ACreativeHallBuildsOnAnEmptyBinAndStillTakesTheTime)
{
    const ShipBlueprint design = threePartStack();
    const BillOfMaterials bill = blueprintCost(design);
    SW_CHECK(bill.totalKg() > 1.0); // the fixture has to cost something

    auto makeHall = [&](ecs::World& world) {
        const ecs::Entity hall = world.createEntity();
        factory::AssemblyComponent assembly{};
        factory::assemblyOrder(assembly, design.name, bill.ironKg, bill.copperKg);
        assembly.buildRateKgPerSecond = 100.0;
        world.addComponent(hall, assembly);
        world.addComponent(hall, factory::VehicleQueueComponent{});
        factory::InventoryComponent inventory{};
        // Room for every crate the rate could possibly finish, so that what
        // this test measures is the RATE and not the size of the bin: a
        // 60 m^3 crate fills a 200 m^3 apron in three hulls and the hall goes
        // BLOCKED, which is correct and is a different fact.
        inventory.volumeCapacityM3 = 1.0e6;
        world.addComponent(hall, inventory); // EMPTY: not a gram of metal
        return hall;
    };

    // Survival: an empty bin builds nothing, forever, and says why.
    {
        ecs::World world;
        const ecs::Entity hall = makeHall(world);
        factory::AssemblySystem system;
        for (int i = 0; i < 200; ++i)
        {
            system.update(world, 1.0f);
        }
        const auto& built = world.getComponent<factory::AssemblyComponent>(hall);
        SW_CHECK_EQ(built.completed, 0u);
        SW_CHECK_EQ(built.state, factory::RecipeStateComponent::kStarved);
    }

    // Creative: the same hall, the same empty bin, and a rocket comes out.
    {
        ecs::World world;
        const ecs::Entity hall = makeHall(world);
        factory::AssemblySystem system;
        system.setFreeMaterials(true);

        // ...AND THE TIME IS THE SAME. One second of a 100 kg/s hall cannot
        // finish a bill of several tonnes, free or not: if this ever passes,
        // the mode has turned the hall into a spawner.
        system.update(world, 1.0f);
        SW_CHECK_EQ(world.getComponent<factory::AssemblyComponent>(hall).completed, 0u);
        SW_CHECK(factory::assemblyProgress(
                     world.getComponent<factory::AssemblyComponent>(hall)) > 0.0);

        for (int i = 0; i < 200; ++i)
        {
            system.update(world, 1.0f);
        }
        const auto& built = world.getComponent<factory::AssemblyComponent>(hall);
        const auto& after = world.getComponent<factory::InventoryComponent>(hall);
        SW_CHECK(built.completed >= 1u);
        SW_CHECK_EQ(built.state, factory::RecipeStateComponent::kRunning);
        // One crate per hull, exactly as in survival.
        SW_CHECK(std::abs(factory::inventoryCount(after, res::Resource::Vehicle) -
                          static_cast<f64>(built.completed)) < 1.0e-9);
        // AND THE BIN IS UNTOUCHED. Free means the metal is never taken, not
        // that it is taken from a stock allowed to go negative.
        SW_CHECK_EQ(factory::inventoryCount(after, res::Resource::Iron), 0.0);
        SW_CHECK_EQ(factory::inventoryCount(after, res::Resource::Copper), 0.0);
        // The number of hulls is set by the RATE and nothing else: 201 seconds
        // of budget at 100 kg/s, divided by the bill.
        const f64 possible = std::floor(201.0 * 100.0 / bill.totalKg());
        SW_CHECK(static_cast<f64>(built.completed) <= possible);
        SW_CHECK(static_cast<f64>(built.completed) >= possible - 1.0);
    }

    // ...and metal that IS in the bin stays there. A creative hall standing on
    // a working supply chain must not quietly drain it.
    {
        ecs::World world;
        const ecs::Entity hall = makeHall(world);
        auto& inventory = world.getComponent<factory::InventoryComponent>(hall);
        factory::inventoryAdd(inventory, res::Resource::Iron, 500.0);
        factory::AssemblySystem system;
        system.setFreeMaterials(true);
        for (int i = 0; i < 200; ++i)
        {
            system.update(world, 1.0f);
        }
        SW_CHECK(world.getComponent<factory::AssemblyComponent>(hall).completed >= 1u);
        SW_CHECK_EQ(factory::inventoryCount(
                        world.getComponent<factory::InventoryComponent>(hall),
                        res::Resource::Iron),
                    500.0);
    }
}
