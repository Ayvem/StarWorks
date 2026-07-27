// ============================================================================
// PowerTests.cpp — the F3 contract: a factory runs on the sun that is
// actually in the sky.
//
// Four promises are pinned here.
//
//   1. SUNLIGHT IS GEOMETRY. `solarFactor` is Lambert against the REAL star
//      direction at the REAL local vertical, zero below the horizon, zero
//      behind an occluding body — and a site is never shadowed by the very
//      body it is standing on, which is the one bug this shape of test
//      exists to catch.
//   2. A BROWNOUT IS LEGIBLE. `allocatePower` serves strict priority bands
//      and splits proportionally inside the band that runs out. You can read
//      off which tier of the factory the grid stopped at.
//   3. THE EXECUTOR TELLS THE TRUTH. STARVED / BLOCKED / NO POWER are
//      distinguishable, and a machine that is not being served produces
//      nothing rather than producing on credit.
//   4. THE FOURTEEN-DAY NIGHT. A lunar site simulated through one full
//      28-day rotation charges by day, lives off its banks after dusk,
//      stops when they are flat, and resumes at dawn — and the totals it
//      reaches under warp (one-hour ticks) match the ones it reaches in
//      real time (one-minute ticks). That last clause is the whole reason
//      rates are per-second and the lanes catch up in bulk.
// ============================================================================

#include "TestFramework.hpp"

#include <ECS/World.hpp>
#include <Factory/FactoryComponents.hpp>
#include <Factory/FactorySystems.hpp>
#include <Factory/Power.hpp>
#include <Factory/PowerNetwork.hpp>
#include <Factory/Recipes.hpp>
#include <Physics/PhysicsComponents.hpp>
#include <Scene/TransformComponents.hpp>

#include <cmath>
#include <utility>
#include <vector>

namespace
{
    using sw::f64;
    using sw::u32;
    using sw::usize;

    constexpr f64 kMoonRadius = 1.7374e6;      // Luna, m
    constexpr f64 kStarDistance = 1.496e11;    // 1 AU, m
    constexpr f64 kRotationPeriod = 28.0 * 86400.0; // 14 days lit, 14 dark

    /// A rotation that stands the model's +Y on `up`. The grid reads the
    /// panel's world up out of exactly this, so the test builds it the same
    /// way placeBuilding does.
    sw::Quat standUp(const sw::Vec3& up)
    {
        const sw::Vec3 y{0.0f, 1.0f, 0.0f};
        const sw::f32 d = glm::clamp(glm::dot(y, up), -1.0f, 1.0f);
        if (d > 0.9999f)
        {
            return sw::Quat{1.0f, 0.0f, 0.0f, 0.0f};
        }
        if (d < -0.9999f)
        {
            return glm::angleAxis(3.14159265f, sw::Vec3{1.0f, 0.0f, 0.0f});
        }
        return glm::angleAxis(std::acos(d), glm::normalize(glm::cross(y, up)));
    }

    /// The local vertical of an equatorial site `seconds` into the rotation.
    sw::WorldVec3 siteUpAt(f64 seconds)
    {
        const f64 angle = 6.283185307179586 * (seconds / kRotationPeriod);
        return sw::WorldVec3{std::cos(angle), 0.0, std::sin(angle)};
    }

    struct Site
    {
        sw::ecs::Entity hub{};
        sw::ecs::Entity solar{};
        sw::ecs::Entity battery{};
        sw::ecs::Entity miner{};
    };

    sw::ecs::Entity addBuilding(sw::ecs::World& world, sw::ecs::Entity hub,
                                sw::factory::BuildingCategory category, f64 producedKw,
                                f64 consumedKw)
    {
        const sw::ecs::Entity entity = world.createEntity();
        world.addComponent(entity, sw::TransformComponent{});

        sw::factory::BuildingComponent building{};
        building.category = category;
        building.site = hub;
        building.groundDensity = 1.0f;
        world.addComponent(entity, building);

        sw::factory::PowerComponent power{};
        power.producedKw = producedKw;
        power.consumedKw = consumedKw;
        power.priority = sw::factory::defaultPowerPriority(category);
        world.addComponent(entity, power);
        return entity;
    }

    /// A lunar outpost: one solar field, one battery bank, one ice mine.
    Site buildOutpost(sw::ecs::World& world, f64 batteryVolumeM3)
    {
        Site site{};
        site.hub = world.createEntity();
        sw::factory::SiteComponent hub{};
        world.addComponent(site.hub, hub);

        site.solar = addBuilding(world, site.hub, sw::factory::BuildingCategory::Solar,
                                 180.0, 0.0);

        site.battery = addBuilding(world, site.hub,
                                   sw::factory::BuildingCategory::Battery, 0.0, 0.0);
        world.addComponent(site.battery, sw::factory::BatteryComponent{});
        sw::factory::InventoryComponent bank{};
        bank.volumeCapacityM3 = batteryVolumeM3;
        world.addComponent(site.battery, bank);

        site.miner = addBuilding(world, site.hub, sw::factory::BuildingCategory::Miner,
                                 0.0, 30.0);
        sw::factory::RecipeStateComponent state{};
        state.recipeId = sw::factory::kRecipeMineWaterIce;
        world.addComponent(site.miner, state);
        sw::factory::InventoryComponent bin{};
        bin.volumeCapacityM3 = 1.0e9; // never the bottleneck in this test
        world.addComponent(site.miner, bin);

        return site;
    }

    /// Spins the outpost to `seconds` into the lunar day and steps one tick.
    void stepOutpost(sw::ecs::World& world, const Site& site,
                     sw::factory::PowerGridSystem& grid,
                     sw::factory::ProductionSystem& production, f64 seconds, f64 dt)
    {
        const sw::WorldVec3 up = siteUpAt(seconds);
        const sw::Quat rotation = standUp(sw::Vec3(up));
        for (const sw::ecs::Entity entity : {site.solar, site.battery, site.miner})
        {
            auto& transform = world.getComponent<sw::TransformComponent>(entity);
            transform.position = up * kMoonRadius;
            transform.rotation = rotation;
        }
        grid.update(world, static_cast<sw::f32>(dt));
        production.update(world, static_cast<sw::f32>(dt));
    }

    f64 charge(sw::ecs::World& world, sw::ecs::Entity battery)
    {
        return sw::factory::inventoryCount(
            world.getComponent<sw::factory::InventoryComponent>(battery),
            sw::res::Resource::ElectricCharge);
    }
} // namespace

// ---------------------------------------------------------------------------
// 1. SUNLIGHT IS GEOMETRY
// ---------------------------------------------------------------------------
SW_TEST(SolarFactorIsTheStarActuallyInTheSky)
{
    const sw::WorldVec3 star{kStarDistance, 0.0, 0.0};

    // Noon: the star straight up.
    const sw::WorldVec3 noonSite{kMoonRadius, 0.0, 0.0};
    SW_CHECK(std::abs(sw::factory::solarFactor(noonSite, sw::Vec3{1.0f, 0.0f, 0.0f}, star,
                                               {}) -
                      1.0) < 1.0e-6);

    // Midnight: the star straight down. Not "dim" — zero.
    const sw::WorldVec3 nightSite{-kMoonRadius, 0.0, 0.0};
    SW_CHECK(sw::factory::solarFactor(nightSite, sw::Vec3{-1.0f, 0.0f, 0.0f}, star, {}) ==
             0.0);

    // Terminator: grazing incidence, and no negative light just past it.
    const sw::WorldVec3 dawnSite{0.0, 0.0, kMoonRadius};
    SW_CHECK(sw::factory::solarFactor(dawnSite, sw::Vec3{0.0f, 0.0f, 1.0f}, star, {}) <
             1.0e-4);

    // A 60-degree sun gives exactly cos(60) = 0.5.
    const sw::Vec3 tilted{0.5f, 0.0f, 0.8660254f};
    SW_CHECK(std::abs(sw::factory::solarFactor(sw::WorldVec3(tilted) * kMoonRadius, tilted,
                                               star, {}) -
                      0.5) < 1.0e-5);

    // A SITE IS NEVER SHADOWED BY ITS OWN BODY. The grid hands the whole
    // gravity-source list in, moon included; with the star up, the moon's
    // closest approach to the ray is behind the panel.
    const sw::factory::Occluder self{sw::WorldVec3{0.0}, kMoonRadius};
    const sw::factory::Occluder list[] = {self};
    SW_CHECK(std::abs(sw::factory::solarFactor(noonSite, sw::Vec3{1.0f, 0.0f, 0.0f}, star,
                                               list) -
                      1.0) < 1.0e-6);

    // ...but a body that IS in the way puts the lights out. A planet of the
    // moon's own size, parked halfway to the star, on the ray.
    const sw::factory::Occluder eclipse[] = {
        {sw::WorldVec3{kStarDistance * 0.5, 0.0, 0.0}, kMoonRadius}};
    SW_CHECK(sw::factory::solarFactor(noonSite, sw::Vec3{1.0f, 0.0f, 0.0f}, star,
                                      eclipse) == 0.0);

    // ...and one just off the ray does not. Offset by two radii.
    const sw::factory::Occluder miss[] = {
        {sw::WorldVec3{kStarDistance * 0.5, 0.0, kMoonRadius * 2.0}, kMoonRadius}};
    SW_CHECK(sw::factory::solarFactor(noonSite, sw::Vec3{1.0f, 0.0f, 0.0f}, star, miss) >
             0.99);
}

// ---------------------------------------------------------------------------
// 1b. THE GRID IS THE CABLES
//
// A grid is a connected component of the cable graph and nothing else. The
// two rules that give it shape — one wire per building, unlimited at a pole —
// are what make a base's electrical layout a decision the player makes rather
// than an adjacency they cannot see.
// ---------------------------------------------------------------------------
SW_TEST(CablesDecideWhoIsOnWhoseGrid)
{
    using sw::factory::PowerLink;

    // Six things, wired: 0-1-2 in a chain, 3-4 as a pair, 5 on its own.
    const PowerLink wired[] = {{0, 1}, {1, 2}, {3, 4}};
    const std::vector<u32> grid = sw::factory::traceGrids(6, wired);
    SW_CHECK_EQ(grid[0], grid[1]);
    SW_CHECK_EQ(grid[1], grid[2]);
    SW_CHECK_EQ(grid[3], grid[4]);
    SW_CHECK(grid[0] != grid[3]);
    // A building nobody wired up is its OWN grid — it runs on what it makes,
    // which for a smelter is nothing. That is the honest answer, and it is
    // why a solar farm you forgot to connect powers nothing.
    SW_CHECK(grid[5] != grid[0]);
    SW_CHECK(grid[5] != grid[3]);
    SW_CHECK_EQ(grid[5], 5u);

    // The id is the SMALLEST member index, so it does not depend on the
    // order the cables happen to be walked in — demolish an unrelated span
    // across the base and the grid you were looking at keeps its number.
    const PowerLink reversed[] = {{3, 4}, {1, 2}, {0, 1}};
    SW_CHECK(sw::factory::traceGrids(6, reversed) == grid);
    SW_CHECK_EQ(grid[2], 0u);

    // Links pointing at things that no longer exist are ignored, not
    // trusted: a cable can outlive its endpoint by one rebuild.
    const PowerLink dangling[] = {{0, 1}, {2, 99}};
    const std::vector<u32> survived = sw::factory::traceGrids(3, dangling);
    SW_CHECK_EQ(survived[0], survived[1]);
    SW_CHECK_EQ(survived[2], 2u);

    // ---- the two rules ------------------------------------------------
    SW_CHECK_EQ(sw::factory::maxCablesFor(sw::factory::BuildingCategory::Miner), 1u);
    SW_CHECK_EQ(sw::factory::maxCablesFor(sw::factory::BuildingCategory::Solar), 1u);
    SW_CHECK(sw::factory::maxCablesFor(sw::factory::BuildingCategory::Pole) > 1000u);

    using V = sw::factory::CableVerdict;
    const auto check = [](sw::factory::BuildingCategory a,
                          sw::factory::BuildingCategory b, u32 onA, u32 onB, u32 gridA,
                          u32 gridB, f64 length) {
        return sw::factory::validateCable(true, true, a, b, onA, onB, gridA, gridB,
                                          length, 120.0);
    };
    const auto miner = sw::factory::BuildingCategory::Miner;
    const auto solar = sw::factory::BuildingCategory::Solar;
    const auto pole = sw::factory::BuildingCategory::Pole;

    // Two fresh buildings: fine, once.
    SW_CHECK_EQ(check(miner, solar, 0, 0, 0, 1, 30.0), V::Ok);
    // ...and only once. The second wire onto either end is refused, which
    // is the whole reason the pole is worth building.
    SW_CHECK_EQ(check(miner, solar, 1, 0, 0, 1, 30.0), V::EndpointFull);
    SW_CHECK_EQ(check(miner, solar, 0, 1, 0, 1, 30.0), V::EndpointFull);
    // A POLE takes as many as you like.
    SW_CHECK_EQ(check(pole, miner, 7, 0, 0, 1, 30.0), V::Ok);
    SW_CHECK_EQ(check(pole, pole, 40, 40, 0, 1, 30.0), V::Ok);
    // A second path between things already joined does nothing but hang a
    // wire in the way.
    SW_CHECK_EQ(check(pole, pole, 3, 3, 4, 4, 30.0), V::AlreadyWired);
    // Two poles that are BOTH still unwired share no grid yet even if the
    // union-find has not run: the degree check is what distinguishes them.
    SW_CHECK_EQ(check(pole, pole, 0, 0, 0, 0, 30.0), V::Ok);
    SW_CHECK_EQ(check(pole, miner, 1, 0, 0, 1, 500.0), V::TooLong);
    SW_CHECK_EQ(sw::factory::validateCable(false, true, pole, miner, 0, 0, 0, 1, 10.0,
                                           120.0),
                V::NoPowerNode);

    // Every refusal says something a player can act on.
    for (const V verdict : {V::SameNode, V::AlreadyWired, V::EndpointFull,
                            V::NoPowerNode, V::TooLong})
    {
        SW_CHECK(!sw::factory::cableVerdictText(verdict).empty());
    }
}

SW_TEST(ACableHangsBetweenItsTwoNodesAndSagsInBetween)
{
    // A 40 m span across an equatorial site on the moon.
    const sw::WorldVec3 up{1.0, 0.0, 0.0};
    const sw::WorldVec3 from{kMoonRadius + 10.0, 0.0, -20.0};
    const sw::WorldVec3 to{kMoonRadius + 10.0, 0.0, 20.0};
    constexpr f64 kSag = 0.045;

    // It MEETS its poles. A wire that starts half a metre under the
    // insulator reads as broken, and the ends are the one part of the curve
    // the player looks at closely.
    SW_CHECK(glm::length(sw::factory::cablePointAt(from, to, sw::Vec3(up), kSag, 0.0) -
                         from) < 1.0e-9);
    SW_CHECK(glm::length(sw::factory::cablePointAt(from, to, sw::Vec3(up), kSag, 1.0) -
                         to) < 1.0e-9);

    // ...and it hangs in between, by 4 * 0.25 * 40 * 0.045 = 1.8 m at the
    // middle, straight DOWN the local vertical rather than along a world
    // axis — on a sphere those are not the same direction.
    const sw::WorldVec3 middle =
        sw::factory::cablePointAt(from, to, sw::Vec3(up), kSag, 0.5);
    const f64 droop = (kMoonRadius + 10.0) - middle.x;
    SW_CHECK(std::abs(droop - 1.8) < 1.0e-9);
    SW_CHECK(std::abs(middle.z) < 1.0e-9); // symmetric

    // The sag is a fraction of the SPAN, so a short jumper between adjacent
    // machines does not droop into the ground.
    const sw::WorldVec3 shortTo{kMoonRadius + 10.0, 0.0, -16.0};
    const sw::WorldVec3 shortMid =
        sw::factory::cablePointAt(from, shortTo, sw::Vec3(up), kSag, 0.5);
    SW_CHECK((kMoonRadius + 10.0) - shortMid.x < 0.2);

    // Off the ends, the curve stops rather than flying off: a caller that
    // samples past 1 gets the far pole, not a wire in the sky.
    SW_CHECK(glm::length(sw::factory::cablePointAt(from, to, sw::Vec3(up), kSag, 4.0) -
                         to) < 1.0e-9);
}

// ---------------------------------------------------------------------------
// 2. A BROWNOUT IS LEGIBLE
// ---------------------------------------------------------------------------
SW_TEST(PowerAllocationServesPriorityBandsThenSharesWhatIsLeft)
{
    // Two miners (band 1) and two refineries (band 2).
    const sw::factory::PowerClaim claims[] = {
        {30.0, 1}, {30.0, 1}, {100.0, 2}, {300.0, 2}};
    std::vector<f64> out(4, -1.0);

    // Plenty: everybody runs.
    sw::factory::allocatePower(claims, 1000.0, out);
    for (const f64 share : out)
    {
        SW_CHECK(std::abs(share - 1.0) < 1.0e-12);
    }

    // Enough for the mines and half the smelting: the mines are untouched
    // and the refineries share what is left IN PROPORTION to what they asked
    // for — 260 kW spread over a 400 kW appetite is 65% each.
    sw::factory::allocatePower(claims, 60.0 + 260.0, out);
    SW_CHECK(std::abs(out[0] - 1.0) < 1.0e-12);
    SW_CHECK(std::abs(out[1] - 1.0) < 1.0e-12);
    SW_CHECK(std::abs(out[2] - 0.65) < 1.0e-12);
    SW_CHECK(std::abs(out[3] - 0.65) < 1.0e-12);

    // Barely anything: the FIRST band browns out and the second gets nothing
    // at all. The factory stops at a readable place, not everywhere at once.
    sw::factory::allocatePower(claims, 30.0, out);
    SW_CHECK(std::abs(out[0] - 0.5) < 1.0e-12);
    SW_CHECK(std::abs(out[1] - 0.5) < 1.0e-12);
    SW_CHECK(out[2] == 0.0);
    SW_CHECK(out[3] == 0.0);

    // Nothing at all.
    sw::factory::allocatePower(claims, 0.0, out);
    for (const f64 share : out)
    {
        SW_CHECK(share == 0.0);
    }

    // A claim that asks for nothing is always satisfied — otherwise a
    // passive building would report NO POWER forever.
    const sw::factory::PowerClaim freeloader[] = {{0.0, 4}};
    std::vector<f64> single(1, -1.0);
    sw::factory::allocatePower(freeloader, 0.0, single);
    SW_CHECK(std::abs(single[0] - 1.0) < 1.0e-12);

    // The default order is the one the spec asks for: mines outlive smelters.
    SW_CHECK(sw::factory::defaultPowerPriority(sw::factory::BuildingCategory::Miner) <
             sw::factory::defaultPowerPriority(sw::factory::BuildingCategory::Refinery));
    SW_CHECK(sw::factory::defaultPowerPriority(sw::factory::BuildingCategory::Hub) <
             sw::factory::defaultPowerPriority(sw::factory::BuildingCategory::Miner));
}

// ---------------------------------------------------------------------------
// 3. THE EXECUTOR TELLS THE TRUTH
// ---------------------------------------------------------------------------
SW_TEST(ExecutorStatesAreDistinguishableAndNoPowerProducesNothing)
{
    sw::ecs::World world;

    auto machine = [&world](sw::factory::BuildingCategory category, u32 recipeId,
                            f64 volumeM3, f64 satisfaction) {
        const sw::ecs::Entity entity = world.createEntity();
        sw::factory::BuildingComponent building{};
        building.category = category;
        building.groundDensity = 1.0f;
        world.addComponent(entity, building);
        sw::factory::RecipeStateComponent state{};
        state.recipeId = recipeId;
        world.addComponent(entity, state);
        sw::factory::InventoryComponent inventory{};
        inventory.volumeCapacityM3 = volumeM3;
        world.addComponent(entity, inventory);
        sw::factory::PowerComponent power{};
        power.satisfaction = satisfaction;
        world.addComponent(entity, power);
        return entity;
    };

    // NO POWER: a well-fed smelter with a dead grid.
    const sw::ecs::Entity dark =
        machine(sw::factory::BuildingCategory::Refinery, sw::factory::kRecipeSmeltIron,
                100.0, 0.0);
    sw::factory::inventoryAdd(world.getComponent<sw::factory::InventoryComponent>(dark),
                              sw::res::Resource::IronOre, 50.0);

    // STARVED: full power, empty hopper.
    const sw::ecs::Entity hungry =
        machine(sw::factory::BuildingCategory::Refinery, sw::factory::kRecipeSmeltIron,
                100.0, 1.0);

    // BLOCKED: full power, full input, no room for the product. Electrolysis
    // is the case that matters — its two gases take MORE volume than the
    // water they came from, so the bin can genuinely fill.
    // 594 units of water is 0.594 m^3; the bin holds 0.595. Electrolysis
    // nets +0.0087 m^3 per second of running, so one second does not fit.
    const sw::ecs::Entity jammed =
        machine(sw::factory::BuildingCategory::Refinery,
                sw::factory::kRecipeElectrolysis, 0.595, 1.0);
    sw::factory::inventoryAdd(world.getComponent<sw::factory::InventoryComponent>(jammed),
                              sw::res::Resource::Water, 594.0);

    // OK, at half power: it runs, at half rate.
    const sw::ecs::Entity brownedOut =
        machine(sw::factory::BuildingCategory::Refinery, sw::factory::kRecipeSmeltIron,
                100.0, 0.5);
    sw::factory::inventoryAdd(
        world.getComponent<sw::factory::InventoryComponent>(brownedOut),
        sw::res::Resource::IronOre, 50.0);

    sw::factory::ProductionSystem production;
    production.update(world, 1.0f);

    using State = sw::factory::RecipeStateComponent;
    SW_CHECK_EQ(world.getComponent<State>(dark).state, State::kNoPower);
    SW_CHECK_EQ(world.getComponent<State>(hungry).state, State::kStarved);
    SW_CHECK_EQ(world.getComponent<State>(jammed).state, State::kBlocked);
    SW_CHECK_EQ(world.getComponent<State>(brownedOut).state, State::kRunning);

    // NOTHING is produced without power, and nothing is CONSUMED either: a
    // machine that ate its ore and made no iron would be destroying matter
    // every night.
    SW_CHECK(world.getComponent<State>(dark).producedUnits == 0.0);
    SW_CHECK(world.getComponent<State>(dark).consumedUnits == 0.0);
    SW_CHECK(
        std::abs(sw::factory::inventoryCount(
                     world.getComponent<sw::factory::InventoryComponent>(dark),
                     sw::res::Resource::IronOre) -
                 50.0) < 1.0e-12);

    // Half the grid is half the rate, exactly. SmeltIron eats 3 u/s.
    SW_CHECK(std::abs(world.getComponent<State>(brownedOut).consumedUnits - 1.5) <
             1.0e-9);
}

// ---------------------------------------------------------------------------
// 3b. THE FUEL CHAIN
//
// WaterIce -> Water -> H2 + O2 -> Fuel. It is the chain that gives a lunar
// or martian site a reason to exist, and it is also the one that stresses
// every piece of F3 at once: a mine whose yield is the ground, a melter, an
// electrolyser that eats 480 kW, and a synthesiser fed TWO gases down one
// belt. If any of those four pieces is wrong there is no propellant at the
// end, and the mass will not balance.
// ---------------------------------------------------------------------------
SW_TEST(TheFuelChainTurnsIceIntoPropellantWithoutLosingAGram)
{
    sw::ecs::World world;

    auto machine = [&world](sw::factory::BuildingCategory category, u32 recipeId,
                            f64 volumeM3) {
        const sw::ecs::Entity entity = world.createEntity();
        sw::factory::BuildingComponent building{};
        building.category = category;
        building.groundDensity = 1.0f;
        world.addComponent(entity, building);
        sw::factory::RecipeStateComponent state{};
        state.recipeId = recipeId;
        world.addComponent(entity, state);
        sw::factory::InventoryComponent inventory{};
        inventory.volumeCapacityM3 = volumeM3;
        world.addComponent(entity, inventory);
        sw::factory::PowerComponent power{}; // satisfaction 1: the grid is elsewhere
        world.addComponent(entity, power);
        return entity;
    };

    const sw::ecs::Entity mine = machine(sw::factory::BuildingCategory::Miner,
                                         sw::factory::kRecipeMineWaterIce, 200.0);
    const sw::ecs::Entity melter = machine(sw::factory::BuildingCategory::Refinery,
                                           sw::factory::kRecipeMeltWater, 200.0);
    const sw::ecs::Entity electrolyser = machine(
        sw::factory::BuildingCategory::Refinery, sw::factory::kRecipeElectrolysis, 400.0);
    const sw::ecs::Entity synthesiser =
        machine(sw::factory::BuildingCategory::Refinery,
                sw::factory::kRecipeSynthesizeFuel, 400.0);

    world.addComponent(melter, sw::factory::makeItemLink(
                                   mine, sw::res::Resource::WaterIce, 3.0));
    world.addComponent(electrolyser,
                       sw::factory::makeItemLink(melter, sw::res::Resource::Water, 3.0));
    // ONE belt out of the electrolyser, carrying BOTH gases — the case the
    // channel array exists for.
    sw::factory::ItemLinkComponent gases{};
    SW_CHECK(sw::factory::linkAddChannel(gases, electrolyser, sw::res::Resource::Hydrogen,
                                         3.0) == 0);
    SW_CHECK(sw::factory::linkAddChannel(gases, electrolyser, sw::res::Resource::Oxygen,
                                         3.0) == 1);
    world.addComponent(synthesiser, gases);

    sw::factory::ProductionSystem production;
    sw::factory::TransferSystem transfer;
    for (u32 tick = 0; tick < 3000; ++tick)
    {
        production.update(world, 0.2f); // Automation, 5 Hz
        transfer.update(world, 0.1f);   // Logistics, 10 Hz — two per production
        transfer.update(world, 0.1f);
    }

    using State = sw::factory::RecipeStateComponent;
    auto& fuelBin = world.getComponent<sw::factory::InventoryComponent>(synthesiser);
    const f64 fuel = sw::factory::inventoryCount(fuelBin, sw::res::Resource::Fuel);
    SW_CHECK(fuel > 0.0); // there IS propellant at the end of the chain

    // ...and the chain is HYDROGEN-limited, which is a real fact about it
    // and not a bug: electrolysis splits water into 0.111 H2 and 0.889 O2
    // per unit, the synthesiser wants 0.25 and 0.75, so the oxygen piles up
    // in the synthesiser's bin while the hydrogen is eaten on arrival. A
    // player reading STARVED off this machine's panel is being told the
    // truth about their factory's shape.
    SW_CHECK_EQ(world.getComponent<State>(synthesiser).state, State::kStarved);
    SW_CHECK(sw::factory::inventoryCount(fuelBin, sw::res::Resource::Oxygen) > fuel);
    SW_CHECK(sw::factory::inventoryCount(fuelBin, sw::res::Resource::Hydrogen) < 1.0);

    // BOTH gases arrived. A chain that only ever moved hydrogen would still
    // make some fuel out of the oxygen it started with — which it has none
    // of — so this is the assertion that the two-channel belt works.
    const auto& link = world.getComponent<sw::factory::ItemLinkComponent>(synthesiser);
    SW_CHECK(sw::factory::linkFlow(link, sw::res::Resource::Hydrogen) > 0.0);
    SW_CHECK(sw::factory::linkFlow(link, sw::res::Resource::Oxygen) > 0.0);

    // MATTER. Every gram of ice the mine dug is still in the system, as ice,
    // water, gas or propellant, minus only what the recipes DECLARE as loss.
    f64 held = 0.0;
    for (const sw::ecs::Entity entity : {mine, melter, electrolyser, synthesiser})
    {
        const auto& bin = world.getComponent<sw::factory::InventoryComponent>(entity);
        for (const sw::factory::InventorySlot& slot : bin.slots)
        {
            if (slot.resource != sw::res::Resource::Count)
            {
                held += slot.units * sw::res::definition(slot.resource).massPerUnitKg;
            }
        }
    }
    const f64 dug = world.getComponent<State>(mine).producedUnits *
                    sw::res::definition(sw::res::Resource::WaterIce).massPerUnitKg;
    f64 declaredLoss = 0.0;
    const std::pair<sw::ecs::Entity, u32> stages[] = {
        {melter, sw::factory::kRecipeMeltWater},
        {electrolyser, sw::factory::kRecipeElectrolysis},
        {synthesiser, sw::factory::kRecipeSynthesizeFuel}};
    for (const auto& [entity, recipeId] : stages)
    {
        const sw::factory::RecipeDefinition* recipe = sw::factory::findRecipe(recipeId);
        // consumedUnits is the sum over inputs; weigh it through the recipe's
        // own input mix so the loss is in kilograms, not units.
        const f64 inputUnitsPerSecond = [&] {
            f64 total = 0.0;
            for (const sw::factory::Ingredient& in : recipe->inputs)
            {
                if (in.resource != sw::res::Resource::Count)
                {
                    total += in.unitsPerSecond;
                }
            }
            return total;
        }();
        const f64 seconds = (inputUnitsPerSecond > 0.0)
                                ? world.getComponent<State>(entity).consumedUnits /
                                      inputUnitsPerSecond
                                : 0.0;
        declaredLoss += seconds * sw::factory::recipeMassLossKgps(*recipe);
    }
    SW_CHECK(std::abs(held + declaredLoss - dug) < 1.0e-6 * std::max(1.0, dug));
}

// ---------------------------------------------------------------------------
// 4. THE FOURTEEN-DAY NIGHT
// ---------------------------------------------------------------------------
SW_TEST(LunarNightDrainsTheBanksStopsTheMineAndResumesAtDawn)
{
    sw::ecs::World world;
    // 300 m^3 of ElectricCharge is 2 GJ — about eighteen hours of mining,
    // which is nothing against a fourteen-day night. That is the point: no
    // bank you can build carries you through, so the site STOPS, honestly.
    const Site site = buildOutpost(world, 300.0);
    sw::factory::PowerGridSystem grid{sw::ecs::Entity{}};
    sw::factory::ProductionSystem production;

    // No star entity: the system reads the origin, which is where this test
    // puts the moon's centre... so give it a real one.
    const sw::ecs::Entity star = world.createEntity();
    sw::TransformComponent starTransform{};
    starTransform.position = sw::WorldVec3{kStarDistance, 0.0, 0.0};
    world.addComponent(star, starTransform);
    world.addComponent(star, sw::phys::GravitySourceComponent{});
    grid.setStar(star);

    constexpr f64 kDt = 600.0; // ten-minute ticks: the Automation lane warping
    const usize steps = static_cast<usize>(kRotationPeriod / kDt);

    // The site starts at local NOON (its up points straight at the star) and
    // the phases follow from that: dusk a quarter turn later, midnight at a
    // half, dawn at three quarters. Fourteen days of each.
    using State = sw::factory::RecipeStateComponent;
    f64 peakCharge = 0.0;
    bool sawNoPower = false;
    bool ranAgainAfterDawn = false;
    // The moment the sun ACTUALLY goes out, which is what makes the night's
    // arithmetic exact: from here on, every joule the mine spends came out
    // of the banks and nowhere else.
    bool nightfall = false;
    f64 chargeAtNightfall = 0.0;
    f64 minedAtNightfall = 0.0;
    f64 minedAtMidnight = 0.0;

    for (usize step = 0; step < steps; ++step)
    {
        const f64 t = static_cast<f64>(step) * kDt;
        stepOutpost(world, site, grid, production, t, kDt);

        const f64 stored = charge(world, site.battery);
        peakCharge = std::max(peakCharge, stored);
        const State& state = world.getComponent<State>(site.miner);
        const auto& books = world.getComponent<sw::factory::SiteComponent>(site.hub);

        // Morning, well before dusk: full sun, banks topped off, digging.
        if (step == static_cast<usize>(kRotationPeriod * 0.02 / kDt))
        {
            SW_CHECK_EQ(state.state, State::kRunning);
            SW_CHECK(stored > 1.9e6); // essentially full
            SW_CHECK(books.producedKw > 170.0);
            SW_CHECK(books.consumedKw == 30.0);
        }
        // NIGHTFALL: the first tick with no sunlight at all.
        if (!nightfall && step > 0 && books.producedKw == 0.0)
        {
            nightfall = true;
            chargeAtNightfall = stored;
            minedAtNightfall = state.producedUnits;
            SW_CHECK(t > kRotationPeriod * 0.24 && t < kRotationPeriod * 0.26);
        }
        // Midnight, half a turn in: the banks are flat and the mine is off.
        if (step == static_cast<usize>(kRotationPeriod * 0.5 / kDt))
        {
            SW_CHECK_EQ(state.state, State::kNoPower);
            SW_CHECK(stored < 1.0); // flat
            SW_CHECK(books.producedKw == 0.0);
            SW_CHECK(books.batteryFlowKw == 0.0); // nothing left to give
            minedAtMidnight = state.producedUnits;
        }
        if (state.state == State::kNoPower)
        {
            sawNoPower = true;
        }
        // After dawn, at three quarters of a turn: the sun is back and so is
        // the mine.
        if (t > kRotationPeriod * 0.78 && state.state == State::kRunning)
        {
            ranAgainAfterDawn = true;
        }
    }

    SW_CHECK(sawNoPower);
    SW_CHECK(nightfall);
    SW_CHECK(ranAgainAfterDawn);
    SW_CHECK(peakCharge > 1.9e6); // the banks did fill, by day

    // DUSK COSTS SOMETHING TOO. The banks reach nightfall about half full,
    // not full: a panel makes 180 kW * cos(elevation), so it drops below the
    // mine's 30 kW roughly ten degrees before the sun actually sets, and the
    // last three quarters of a day of "daylight" is already run on stored
    // charge. Half the bank spent before the night has even started is
    // exactly the sort of thing a player should have to plan around.
    SW_CHECK(chargeAtNightfall > 9.0e5 && chargeAtNightfall < 1.2e6);

    // THE EXACT NIGHT. After nightfall the mine draws 30 kW and nothing else
    // does, so the ore it manages before the banks die is the stored energy
    // divided by that draw, at 1.2 u/s — and not one unit more. The tolerance
    // is a single tick, which is where the banks run dry inside.
    const f64 nightYield = minedAtMidnight - minedAtNightfall;
    const f64 expected = chargeAtNightfall / 30.0 * 1.2;
    SW_CHECK(std::abs(nightYield - expected) < kDt * 1.2);
    SW_CHECK(nightYield > 4.0e4); // ...and it really was half a day's work
}

// The same fourteen-day night at two very different tick rates. The lane
// hands the executor whatever dt the warp factor produces, so an hour-long
// tick and a minute-long tick have to reach the same books — bounded by the
// one thing quantisation genuinely costs, which is where dusk lands inside a
// tick.
SW_TEST(TheNightIsTheSameUnderWarpAsItIsInRealTime)
{
    auto run = [](f64 dt) {
        sw::ecs::World world;
        const Site site = buildOutpost(world, 300.0);
        const sw::ecs::Entity star = world.createEntity();
        sw::TransformComponent starTransform{};
        starTransform.position = sw::WorldVec3{kStarDistance, 0.0, 0.0};
        world.addComponent(star, starTransform);
        world.addComponent(star, sw::phys::GravitySourceComponent{});
        sw::factory::PowerGridSystem grid{star};
        sw::factory::ProductionSystem production;

        const usize steps = static_cast<usize>(kRotationPeriod / dt);
        for (usize step = 0; step < steps; ++step)
        {
            stepOutpost(world, site, grid, production, static_cast<f64>(step) * dt, dt);
        }
        return world.getComponent<sw::factory::RecipeStateComponent>(site.miner)
            .producedUnits;
    };

    const f64 realTime = run(60.0);   // one-minute ticks
    const f64 warped = run(3600.0);   // one-hour ticks: 60x
    SW_CHECK(realTime > 1.0e6);       // a fortnight of daylight is a lot of ice

    // The two runs may only disagree by the daylight the coarse ticks
    // straddle at dawn and at dusk: two ticks, at the mine's full rate.
    const f64 allowed = 2.0 * 3600.0 * 1.2;
    SW_CHECK(std::abs(realTime - warped) < allowed);
}
