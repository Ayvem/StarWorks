// ============================================================================
// IndustryTests.cpp — the F1 contract: data-driven industry that cannot
// cheat physics.
//
// Three promises are pinned here.
//
//   1. MATTER IS CONSERVED. Every recipe in the catalogue — built-in or
//      loaded from Assets/Recipes — is weighed: outputs may never exceed
//      inputs, and the difference must be the loss the recipe DECLARES.
//      A balance pass that quietly invents iron fails the build.
//   2. DEPOSITS ARE A FUNCTION, not a spawn table: same direction, same
//      seed, same density, forever; bounded to [0,1]; ice concentrated at
//      the poles because that is what makes a polar site worth founding.
//   3. THE EXECUTOR IS WARP-EXACT. One tick of an hour and three thousand
//      ticks of 1.2 s must produce the same goods to the unit, because the
//      Automation lane will hand it either one.
// ============================================================================

#include "TestFramework.hpp"

#include <ECS/World.hpp>
#include <Factory/FactoryComponents.hpp>
#include <Factory/FactorySystems.hpp>
#include <Factory/Recipes.hpp>
#include <Factory/Conveyor.hpp>
#include <Planet/Deposits.hpp>

#include <cmath>

namespace
{
    /// A building that runs one recipe, with room for its goods.
    sw::ecs::Entity makeMachine(sw::ecs::World& world,
                                sw::factory::BuildingCategory category, sw::u32 recipeId,
                                sw::f64 volumeM3, sw::f32 groundDensity = 1.0f)
    {
        const sw::ecs::Entity entity = world.createEntity();
        sw::factory::BuildingComponent building{};
        building.category = category;
        building.groundDensity = groundDensity;
        world.addComponent(entity, building);

        sw::factory::RecipeStateComponent state{};
        state.recipeId = recipeId;
        world.addComponent(entity, state);

        sw::factory::InventoryComponent inventory{};
        inventory.volumeCapacityM3 = volumeM3;
        world.addComponent(entity, inventory);
        return entity;
    }

    sw::Vec3 sampleDirection(sw::u32 i)
    {
        sw::u32 s = i * 2654435761u + 12345u;
        s ^= s >> 15;
        s *= 2246822519u;
        s ^= s >> 13;
        const sw::f32 u = static_cast<sw::f32>(s & 0xFFFFFFu) / 16777216.0f;
        s ^= s >> 16;
        s *= 3266489917u;
        s ^= s >> 11;
        const sw::f32 v = static_cast<sw::f32>(s & 0xFFFFFFu) / 16777216.0f;
        const sw::f32 z = 2.0f * u - 1.0f;
        const sw::f32 r = std::sqrt(std::max(0.0f, 1.0f - z * z));
        const sw::f32 phi = 6.28318530718f * v;
        return glm::normalize(sw::Vec3{r * std::cos(phi), z, r * std::sin(phi)});
    }
} // namespace

SW_TEST(RecipesConserveMatter)
{
    // THE industrial invariant. Mass out <= mass in, and what is missing is
    // exactly the loss the recipe declares (slag, vented volatiles).
    for (const sw::factory::RecipeDefinition& recipe : sw::factory::recipeCatalog())
    {
        const sw::f64 inputMass = sw::factory::recipeInputMassKgps(recipe);
        const sw::f64 outputMass = sw::factory::recipeOutputMassKgps(recipe);
        if (inputMass <= 0.0)
        {
            // Extraction: the deposit is the input, so there is nothing to
            // balance here — but it must still declare no loss.
            SW_CHECK_EQ(recipe.massLossFraction, 0.0);
            SW_CHECK(outputMass > 0.0);
            continue;
        }
        SW_CHECK(outputMass <= inputMass * 1.0001);
        const sw::f64 declared = inputMass * recipe.massLossFraction;
        const sw::f64 actual = inputMass - outputMass;
        SW_CHECK(std::abs(actual - declared) <= inputMass * 0.02);
    }
}

SW_TEST(RecipeCatalogueIsAddressable)
{
    SW_CHECK(!sw::factory::recipeCatalog().empty());
    for (const sw::factory::RecipeDefinition& recipe : sw::factory::recipeCatalog())
    {
        SW_CHECK(recipe.id != 0);
        SW_CHECK(sw::factory::findRecipe(recipe.id) != nullptr);
    }
    // Category routing is what pairs a machine with what it may run.
    const std::vector<sw::u32> mining =
        sw::factory::recipesForCategory(sw::factory::BuildingCategory::Miner);
    SW_CHECK(!mining.empty());
    for (const sw::u32 id : mining)
    {
        SW_CHECK_EQ(sw::factory::findRecipe(id)->requiredCategory,
                    sw::factory::BuildingCategory::Miner);
    }
    // Electrolysis is the chain the whole energy system exists for.
    const auto* electrolysis = sw::factory::findRecipe(sw::factory::kRecipeElectrolysis);
    SW_CHECK(electrolysis != nullptr);
    SW_CHECK(electrolysis->powerKw > 100.0);
}

SW_TEST(DepositsAreAnAnalyticField)
{
    const sw::planet::DepositComponent worlds[] = {sw::planet::depositsTerra(),
                                                   sw::planet::depositsLuna(),
                                                   sw::planet::depositsMars()};
    for (const sw::planet::DepositComponent& deposits : worlds)
    {
        int rich = 0;
        for (int i = 0; i < 4000; ++i)
        {
            const sw::Vec3 dir = sampleDirection(static_cast<sw::u32>(i));
            const sw::f32 iron =
                sw::planet::oreDensity(deposits, dir, sw::res::Resource::IronOre);
            // Determinism: the same question twice is the same answer.
            SW_CHECK_EQ(iron, sw::planet::oreDensity(deposits, dir,
                                                     sw::res::Resource::IronOre));
            SW_CHECK(iron >= 0.0f && iron <= 1.0f);
            // Refined goods are made, never dug.
            SW_CHECK_EQ(sw::planet::oreDensity(deposits, dir, sw::res::Resource::Iron),
                        0.0f);
            if (iron > 0.25f)
            {
                ++rich;
            }
        }
        // Ore comes in PATCHES: worth prospecting for, not everywhere.
        SW_CHECK(rich > 100);
        SW_CHECK(rich < 3200);
    }
}

SW_TEST(DepositsPutTheIceAtThePoles)
{
    const sw::planet::DepositComponent luna = sw::planet::depositsLuna();
    sw::f64 polar = 0.0;
    sw::f64 equatorial = 0.0;
    int polarCount = 0;
    int equatorialCount = 0;
    for (int i = 0; i < 6000; ++i)
    {
        const sw::Vec3 dir = sampleDirection(static_cast<sw::u32>(i) + 31u);
        const sw::f32 ice =
            sw::planet::oreDensity(luna, dir, sw::res::Resource::WaterIce);
        if (std::abs(dir.y) > 0.90f)
        {
            polar += ice;
            ++polarCount;
        }
        else if (std::abs(dir.y) < 0.30f)
        {
            equatorial += ice;
            ++equatorialCount;
        }
    }
    SW_CHECK(polarCount > 50 && equatorialCount > 50);
    // The reason a lunar polar site is the obvious first colony.
    SW_CHECK(polar / polarCount > 4.0 * (equatorial / equatorialCount));
}

SW_TEST(ProductionRunsRecipesAndConservesMatter)
{
    sw::ecs::World world;
    const sw::ecs::Entity smelter = makeMachine(
        world, sw::factory::BuildingCategory::Refinery, sw::factory::kRecipeSmeltIron,
        20.0);
    auto& inventory = world.getComponent<sw::factory::InventoryComponent>(smelter);
    sw::factory::inventoryAdd(inventory, sw::res::Resource::IronOre, 300.0);

    sw::factory::ProductionSystem production;
    for (int tick = 0; tick < 50; ++tick)
    {
        production.update(world, 1.0f);
    }

    const auto& state = world.getComponent<sw::factory::RecipeStateComponent>(smelter);
    const auto& after = world.getComponent<sw::factory::InventoryComponent>(smelter);
    const sw::f64 ore = sw::factory::inventoryCount(after, sw::res::Resource::IronOre);
    const sw::f64 iron = sw::factory::inventoryCount(after, sw::res::Resource::Iron);

    // 50 s at 3 units/s of ore, unless the volume ran out first.
    SW_CHECK(iron > 0.0);
    SW_CHECK_EQ(state.state, sw::factory::RecipeStateComponent::kRunning);
    // Matter: what left the ore stack became iron plus the declared slag.
    const sw::f64 consumed = 300.0 - ore;
    const sw::f64 expectedIron = consumed * 0.6; // 1.8 out of 3.0
    SW_CHECK(std::abs(iron - expectedIron) < 1.0e-6);
    SW_CHECK(std::abs(state.producedUnits - iron) < 1.0e-6);
    SW_CHECK(std::abs(state.consumedUnits - consumed) < 1.0e-6);
}

SW_TEST(ProductionIsExactUnderTimeWarp)
{
    // The Automation lane hands this system whatever backlog the frame could
    // afford: one long tick under warp, many short ones at x1. Both must
    // produce the same goods — that is what "warp-exact" means.
    auto run = [](sw::f32 step, int ticks) {
        sw::ecs::World world;
        const sw::ecs::Entity mine =
            makeMachine(world, sw::factory::BuildingCategory::Miner,
                        sw::factory::kRecipeMineIronOre, 1.0e6, 0.5f);
        sw::factory::ProductionSystem production;
        for (int i = 0; i < ticks; ++i)
        {
            production.update(world, step);
        }
        return sw::factory::inventoryCount(
            world.getComponent<sw::factory::InventoryComponent>(mine),
            sw::res::Resource::IronOre);
    };

    // Step sizes exact in binary, so the comparison is about the executor
    // and not about f32 rounding: 7200 x 0.5 s is one hour to the bit.
    const sw::f64 slow = run(0.5f, 7200);
    const sw::f64 bulk = run(3600.0f, 1); // the same hour, one bulk tick
    SW_CHECK(std::abs(slow - bulk) < 1.0e-6);
    // Half density in the ground, half the yield: 1 u/s * 0.5 * 3600 s.
    SW_CHECK(std::abs(bulk - 1800.0) < 1.0e-6);
}

SW_TEST(ProductionStopsInsteadOfDestroyingMatter)
{
    sw::ecs::World world;
    // Electrolysis is the right machine to jam: a litre of water becomes
    // hydrogen that needs sixty times the room, so a small tank fills long
    // before the water runs out. (Smelting could never block — iron packs
    // tighter than the ore it came from, so it FREES volume as it runs.)
    const sw::ecs::Entity plant = makeMachine(
        world, sw::factory::BuildingCategory::Refinery,
        sw::factory::kRecipeElectrolysis, 0.30);
    auto& inventory = world.getComponent<sw::factory::InventoryComponent>(plant);
    const sw::f64 water =
        sw::factory::inventoryAdd(inventory, sw::res::Resource::Water, 100.0);

    sw::factory::ProductionSystem production;
    for (int tick = 0; tick < 400; ++tick)
    {
        production.update(world, 1.0f);
    }

    const auto& state = world.getComponent<sw::factory::RecipeStateComponent>(plant);
    const auto& after = world.getComponent<sw::factory::InventoryComponent>(plant);
    const sw::f64 remainingWater =
        sw::factory::inventoryCount(after, sw::res::Resource::Water);
    const sw::f64 hydrogen =
        sw::factory::inventoryCount(after, sw::res::Resource::Hydrogen);
    const sw::f64 oxygen = sw::factory::inventoryCount(after, sw::res::Resource::Oxygen);

    // It jammed rather than consuming water it had nowhere to put.
    SW_CHECK_EQ(state.state, sw::factory::RecipeStateComponent::kBlocked);
    SW_CHECK(remainingWater > 0.0);
    // And every gram is accounted for: water in = hydrogen + oxygen out.
    const sw::f64 consumed = water - remainingWater;
    SW_CHECK(consumed > 0.0);
    SW_CHECK(std::abs((hydrogen + oxygen) - consumed) < 1.0e-6);

    // A starved machine reports it, and touches nothing.
    sw::factory::inventoryRemove(world.getComponent<sw::factory::InventoryComponent>(plant),
                                 sw::res::Resource::Water, 1.0e9);
    production.update(world, 1.0f);
    SW_CHECK_EQ(world.getComponent<sw::factory::RecipeStateComponent>(plant).state,
                sw::factory::RecipeStateComponent::kStarved);
}

SW_TEST(MinersOnlyYieldWhatTheGroundHolds)
{
    sw::ecs::World world;
    const sw::ecs::Entity barren =
        makeMachine(world, sw::factory::BuildingCategory::Miner,
                    sw::factory::kRecipeMineIronOre, 100.0, 0.0f);
    sw::factory::ProductionSystem production;
    production.update(world, 10.0f);

    SW_CHECK_EQ(sw::factory::inventoryCount(
                    world.getComponent<sw::factory::InventoryComponent>(barren),
                    sw::res::Resource::IronOre),
                0.0);
    SW_CHECK_EQ(world.getComponent<sw::factory::RecipeStateComponent>(barren).state,
                sw::factory::RecipeStateComponent::kStarved);

    // A recipe the building's category cannot run is simply not run.
    const sw::ecs::Entity mismatched =
        makeMachine(world, sw::factory::BuildingCategory::Storage,
                    sw::factory::kRecipeSmeltIron, 100.0);
    auto& inventory = world.getComponent<sw::factory::InventoryComponent>(mismatched);
    sw::factory::inventoryAdd(inventory, sw::res::Resource::IronOre, 50.0);
    production.update(world, 10.0f);
    SW_CHECK_EQ(sw::factory::inventoryCount(
                    world.getComponent<sw::factory::InventoryComponent>(mismatched),
                    sw::res::Resource::Iron),
                0.0);
    SW_CHECK_EQ(world.getComponent<sw::factory::RecipeStateComponent>(mismatched).state,
                sw::factory::RecipeStateComponent::kIdle);
}

// The survey is what the scene builder uses to found the starting outpost,
// and what F2's build cursor will use to tell the player where to dig. If it
// returns a spot the MN-1 cannot legally be built on, the game opens on a
// mine that never turns — so the contract is checked here, on the real
// Terra presets, not on a fixture.
SW_TEST(SurveyFindsGroundAMineCanActuallyWork)
{
    const sw::planet::TerrainComponent terrain = sw::planet::presetTerra();
    const sw::planet::DepositComponent deposits = sw::planet::depositsTerra();
    constexpr sw::f64 kTerraRadius = 6.371e6;

    // THE EXACT SWEEP THE GAME FOUNDS ITS OUTPOST WITH.
    sw::f32 grade = 0.0f;
    const sw::Vec3 site = sw::planet::surveyEquatorialSite(
        terrain, deposits, sw::res::Resource::IronOre, kTerraRadius, grade);

    // ON THE EQUATOR, exactly. A base pays its latitude on every launch it
    // ever makes — in rotation speed it is not given, and in the plane
    // change it has to fly. This is not "near the equator": y is zero.
    SW_CHECK_EQ(site.y, 0.0f);
    // Dry land, well above the shore, and richer than the MN-1's minimum.
    SW_CHECK(sw::planet::terrainElevation(terrain, site) > 40.0);
    SW_CHECK(grade > 0.15f);
    // It reports the density the miner will actually be paid on.
    SW_CHECK_EQ(grade,
                sw::planet::oreDensity(deposits, site, sw::res::Resource::IronOre));

    // BUILDABLE: every building of the shipped layout — and the launch pad
    // 120 m east of it — must stand on ground flatter than the steepest
    // slope the buildings accept. This is the check that would have caught
    // a site perched on a ridge.
    const sw::Vec3 east = glm::normalize(glm::cross(sw::Vec3{0.0f, 1.0f, 0.0f}, site));
    const sw::Vec3 north = glm::cross(site, east);
    struct Plot
    {
        sw::f32 eastM;
        sw::f32 northM;
    };
    const Plot layout[] = {{0.0f, 0.0f},    {34.0f, 0.0f},  {34.0f, -30.0f},
                           {0.0f, -30.0f},  {-34.0f, -15.0f}, {-14.0f, 22.0f},
                           {120.0f, 0.0f}};
    for (const Plot& plot : layout)
    {
        const sw::Vec3 direction = glm::normalize(
            site + east * (plot.eastM / static_cast<sw::f32>(kTerraRadius)) +
            north * (plot.northM / static_cast<sw::f32>(kTerraRadius)));
        SW_CHECK(sw::planet::terrainElevation(terrain, direction) > 40.0);
        SW_CHECK(sw::planet::terrainLocalSlope(terrain, direction, kTerraRadius, 20.0f) <
                 0.12f); // the strictest limit in the catalogue (SL-1)
    }

    // Determinism: a world reloaded a year later founds the same outpost.
    sw::f32 again = 0.0f;
    const sw::Vec3 repeat = sw::planet::surveyEquatorialSite(
        terrain, deposits, sw::res::Resource::IronOre, kTerraRadius, again);
    SW_CHECK_EQ(repeat.x, site.x);
    SW_CHECK_EQ(repeat.y, site.y);
    SW_CHECK_EQ(repeat.z, site.z);
    SW_CHECK_EQ(again, grade);

    // The LOCAL survey (what F2's build cursor will use) still reports an
    // honest zero rather than lying when its patch holds nothing but sea:
    // +Z on Terra is open ocean.
    sw::f32 wet = 1.0f;
    const sw::Vec3 nowhere =
        sw::planet::surveySite(terrain, deposits, sw::Vec3{0.0f, 0.0f, 1.0f},
                               sw::res::Resource::IronOre, wet);
    SW_CHECK_EQ(wet, 0.0f);
    SW_CHECK_EQ(nowhere.z, 1.0f);
}

// The starting outpost end to end: a mine feeds a smelter over a link, at
// the rates the shipped .swrecipe files declare, and the mass that comes out
// is the mass that went in minus the slag the recipe admits to.
SW_TEST(OutpostChainSmeltsWhatItMinesAndLosesOnlyItsSlag)
{
    sw::ecs::World world;
    const sw::ecs::Entity miner =
        makeMachine(world, sw::factory::BuildingCategory::Miner,
                    sw::factory::kRecipeMineIronOre, 500.0, 1.0f);
    const sw::ecs::Entity refinery =
        makeMachine(world, sw::factory::BuildingCategory::Refinery,
                    sw::factory::kRecipeSmeltIron, 500.0);
    // The conveyor abstraction the scene wires between them.
    world.addComponent(refinery, sw::factory::makeItemLink(
                                     miner, sw::res::Resource::IronOre, 3.0));

    sw::factory::ProductionSystem production;
    sw::factory::TransferSystem transfer;
    for (sw::u32 tick = 0; tick < 600; ++tick)
    {
        production.update(world, 0.2f); // Automation lane, 5 Hz
        transfer.update(world, 0.2f);
    }

    const auto& minerState = world.getComponent<sw::factory::RecipeStateComponent>(miner);
    const auto& refineryState =
        world.getComponent<sw::factory::RecipeStateComponent>(refinery);
    const auto& minerBin = world.getComponent<sw::factory::InventoryComponent>(miner);
    const auto& refineryBin =
        world.getComponent<sw::factory::InventoryComponent>(refinery);

    // 120 s at 1.0 unit/s: the mine produced its nominal rate.
    SW_CHECK(std::abs(minerState.producedUnits - 120.0) < 1.0e-4);
    // The smelter ran, and ran on ore it did not dig itself.
    SW_CHECK(refineryState.producedUnits > 0.0);
    SW_CHECK(std::abs(refineryState.consumedUnits * (1.8 / 3.0) -
                      refineryState.producedUnits) < 1.0e-4);

    // MATTER: every gram mined is still somewhere, or is declared slag.
    const sw::f64 oreMassPerUnit =
        sw::res::definition(sw::res::Resource::IronOre).massPerUnitKg;
    const sw::f64 ironMassPerUnit =
        sw::res::definition(sw::res::Resource::Iron).massPerUnitKg;
    const sw::f64 mined = minerState.producedUnits * oreMassPerUnit;
    const sw::f64 held =
        (sw::factory::inventoryCount(minerBin, sw::res::Resource::IronOre) +
         sw::factory::inventoryCount(refineryBin, sw::res::Resource::IronOre)) *
        oreMassPerUnit;
    const sw::f64 smelted =
        sw::factory::inventoryCount(refineryBin, sw::res::Resource::Iron) *
        ironMassPerUnit;
    const sw::f64 slag =
        refineryState.consumedUnits * oreMassPerUnit *
        sw::factory::findRecipe(sw::factory::kRecipeSmeltIron)->massLossFraction;
    SW_CHECK(std::abs(held + smelted + slag - mined) < 1.0e-4);
}

// A BELT MUST NOT GO THROUGH THE GROUND, and its cargo must ride the deck
// that was drawn. Both come from the same two functions, which is the point:
// F2 lets the player draw belts by hand and F6 turns them into transport, and
// a second implementation of "where does the deck go" would be a conveyor
// whose items are somewhere its rails are not.
SW_TEST(ConveyorDecksFollowTheGroundAndCarryTheirCargoOnIt)
{
    const sw::planet::TerrainComponent terrain = sw::planet::presetTerra();
    constexpr sw::f64 kRadius = 6.371e6;

    // Lay a 40 m belt across the roughest dry ground we can find — the case
    // where a straight chord in space would cut through a rise.
    sw::Vec3 site{0.0f, 0.0f, 1.0f};
    sw::f32 steepest = 0.0f;
    for (sw::u32 i = 0; i < 2000; ++i)
    {
        const sw::f32 a = static_cast<sw::f32>(i) * 0.0137f;
        const sw::f32 b = static_cast<sw::f32>(i) * 0.0071f;
        const sw::Vec3 direction = glm::normalize(
            sw::Vec3{std::cos(a) * std::cos(b), std::sin(b), std::sin(a) * std::cos(b)});
        if (sw::planet::terrainElevation(terrain, direction) <= 100.0)
        {
            continue;
        }
        const sw::f32 slope =
            sw::planet::terrainLocalSlope(terrain, direction, kRadius, 20.0f);
        if (slope > steepest)
        {
            steepest = slope;
            site = direction;
        }
    }
    SW_CHECK(steepest > 0.2f);

    const sw::Vec3 east = glm::normalize(glm::cross(sw::Vec3{0.0f, 1.0f, 0.0f}, site));
    auto anchorAt = [&](sw::f32 metres) {
        const sw::Vec3 direction =
            glm::normalize(site + east * (metres / static_cast<sw::f32>(kRadius)));
        return sw::WorldVec3(direction) *
               (kRadius + sw::planet::terrainElevation(terrain, direction));
    };

    constexpr sw::f64 kClearance = 1.05;
    sw::WorldVec3 points[sw::factory::kMaxConveyorPoints]{};
    sw::u32 count = sw::factory::kMaxConveyorPoints;
    const sw::f64 length = sw::factory::buildConveyorPath(
        terrain, kRadius, anchorAt(-20.0f), anchorAt(20.0f), kClearance, points, count);

    SW_CHECK_EQ(count, sw::factory::kMaxConveyorPoints);
    // A deck that CLIMBS is longer than the ground distance it spans — that
    // is the whole point of following the terrain rather than cutting a
    // chord through it — but only by what the slope justifies.
    SW_CHECK(length >= 40.0);
    SW_CHECK(length < 40.0 * (1.0 + static_cast<sw::f64>(steepest)) + 2.0);

    // NOWHERE along the deck — not at the sample points, not between them —
    // may the belt be below the ground it crosses.
    for (sw::u32 i = 0; i + 1 < count; ++i)
    {
        for (sw::u32 k = 0; k <= 8; ++k)
        {
            const sw::WorldVec3 middle =
                glm::mix(points[i], points[i + 1], static_cast<sw::f64>(k) / 8.0);
            const sw::Vec3 direction = sw::Vec3(glm::normalize(middle));
            const sw::f64 ground =
                kRadius + sw::planet::terrainElevation(terrain, direction);
            SW_CHECK(glm::length(middle) >= ground);
        }
    }

    // CARGO RIDES THE DECK. Walking the arc length must stay on the
    // polyline, advance monotonically, and wrap without a jump.
    sw::f64 previousAlong = -1.0;
    for (sw::u32 step = 0; step <= 200; ++step)
    {
        const sw::f64 arc = length * static_cast<sw::f64>(step) / 200.0;
        sw::WorldVec3 position{};
        sw::Vec3 heading{};
        sw::factory::conveyorPointAt(points, count, arc, position, heading);

        // On the deck: within a hair of the nearest segment.
        sw::f64 nearest = 1.0e9;
        for (sw::u32 i = 0; i + 1 < count; ++i)
        {
            const sw::WorldVec3 segment = points[i + 1] - points[i];
            const sw::f64 lengthSquared = glm::dot(segment, segment);
            const sw::f64 t = glm::clamp(
                glm::dot(position - points[i], segment) / lengthSquared, 0.0, 1.0);
            nearest = std::min(nearest, glm::length(position - (points[i] + segment * t)));
        }
        SW_CHECK(nearest < 1.0e-6);

        // Monotone along the belt, and always pointing forward.
        const sw::f64 along = glm::dot(position - points[0],
                                       glm::normalize(points[count - 1] - points[0]));
        SW_CHECK(along >= previousAlong - 1.0e-6);
        previousAlong = along;
        SW_CHECK(std::abs(glm::length(heading) - 1.0f) < 1.0e-4f);
    }

    // Above the ground the whole way, with the clearance it was asked for.
    sw::WorldVec3 head{};
    sw::Vec3 direction{};
    sw::factory::conveyorPointAt(points, count, length * 0.5, head, direction);
    const sw::f64 ground =
        kRadius + sw::planet::terrainElevation(terrain, sw::Vec3(glm::normalize(head)));
    SW_CHECK(glm::length(head) - ground > 0.5);
}

// A LINK'S REPORTED THROUGHPUT MUST BE AN AVERAGE, NOT A SNAPSHOT.
//
// The belts read this number to space their cargo, so it is a rendering
// input as much as a statistic — and the instantaneous rate is useless as
// one. A link rated well above what feeds it empties its source on the
// first tick and finds it bare on the next: the raw rate alternates between
// "everything" and "nothing" at the lane frequency, and everything reading
// it flickers in step. That is exactly what made the conveyors blink.
SW_TEST(LinkThroughputIsTheAverageItActuallyMoves)
{
    sw::ecs::World world;

    // A mine making 0.85 units/s feeding a link RATED at 3.0 — the shipped
    // outpost's own mismatch, and the case that oscillated.
    const sw::ecs::Entity mine =
        makeMachine(world, sw::factory::BuildingCategory::Miner,
                    sw::factory::kRecipeMineIronOre, 500.0, 0.85f);
    const sw::ecs::Entity depot =
        makeMachine(world, sw::factory::BuildingCategory::Storage, 0u, 500.0);
    world.addComponent(depot, sw::factory::makeItemLink(
                                  mine, sw::res::Resource::IronOre, 3.0));

    sw::factory::ProductionSystem production;
    sw::factory::TransferSystem transfer;

    // Warm up past the average's memory, then watch it for a while.
    sw::f64 lowest = 1.0e9;
    sw::f64 highest = 0.0;
    // The lanes are stepped at their REAL relative rates — Automation 5 Hz,
    // Logistics 10 Hz — so one production tick is two transfer ticks. Get
    // that ratio wrong and the "measured" rate is measured against the wrong
    // clock, which is its own way of lying about a factory.
    for (sw::u32 tick = 0; tick < 600; ++tick)
    {
        production.update(world, 0.2f); // Automation, 5 Hz
        transfer.update(world, 0.1f);   // Logistics, 10 Hz
        transfer.update(world, 0.1f);
        if (tick >= 300)
        {
            const sw::f64 flow =
                sw::factory::linkFlow(
                    world.getComponent<sw::factory::ItemLinkComponent>(depot),
                    sw::res::Resource::IronOre);
            lowest = std::min(lowest, flow);
            highest = std::max(highest, flow);
        }
    }

    // It settles on what the mine really produces, 0.85 units/s...
    SW_CHECK(std::abs(highest - 0.85) < 0.05);
    // ...and it STAYS there. A snapshot would swing between 0 and 3.0; the
    // average must not move by more than a few percent.
    SW_CHECK(highest - lowest < 0.05);
    SW_CHECK(lowest > 0.5); // and never reads "stopped" on a running belt

    // A link whose source dries up decays to zero rather than freezing at
    // its last reading — an abandoned belt must eventually look abandoned.
    world.getComponent<sw::factory::RecipeStateComponent>(mine).recipeId = 0u;
    sw::factory::inventoryRemove(
        world.getComponent<sw::factory::InventoryComponent>(mine),
        sw::res::Resource::IronOre, 1.0e9);
    for (sw::u32 tick = 0; tick < 400; ++tick)
    {
        transfer.update(world, 0.1f);
    }
    SW_CHECK(sw::factory::linkFlow(
                 world.getComponent<sw::factory::ItemLinkComponent>(depot),
                 sw::res::Resource::IronOre) < 1.0e-3);
}
