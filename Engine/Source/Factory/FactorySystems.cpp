#include "Factory/FactorySystems.hpp"

#include "ECS/World.hpp"
#include "Physics/PhysicsComponents.hpp"
#include "Scene/TransformComponents.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace sw::factory
{
    void MinerSystem::update(ecs::World& world, f32 deltaSeconds)
    {
        const f64 dt = static_cast<f64>(deltaSeconds);
        world.forEach<MinerComponent, InventoryComponent>(
            [dt](ecs::Entity, MinerComponent& miner, InventoryComponent& inventory) {
                const f64 mined =
                    inventoryAdd(inventory, miner.output, miner.unitsPerSecond * dt);
                miner.totalMined += mined;
            });
    }

    void RefinerySystem::update(ecs::World& world, f32 deltaSeconds)
    {
        const f64 dt = static_cast<f64>(deltaSeconds);
        world.forEach<RefineryComponent, InventoryComponent>(
            [dt](ecs::Entity, RefineryComponent& refinery, InventoryComponent& inventory) {
                const f64 wantedInput = refinery.inputUnitsPerSecond * dt;
                const f64 availableInput =
                    std::min(wantedInput, inventoryCount(inventory, refinery.input));
                if (availableInput <= 0.0)
                {
                    return; // starved
                }

                // Reserve output space FIRST: never destroy matter.
                const f64 wantedOutput = availableInput * refinery.conversionRatio;
                const f64 acceptedOutput =
                    inventoryAdd(inventory, refinery.output, wantedOutput);
                if (acceptedOutput <= 0.0)
                {
                    return; // output full: stall
                }

                const f64 consumedInput = acceptedOutput / refinery.conversionRatio;
                inventoryRemove(inventory, refinery.input, consumedInput);
                refinery.totalRefined += acceptedOutput;
            });
    }

    void TransferSystem::update(ecs::World& world, f32 deltaSeconds)
    {
        const f64 dt = static_cast<f64>(deltaSeconds);
        if (dt <= 0.0)
        {
            return;
        }
        // THE THROUGHPUT IS AN AVERAGE, NOT A SNAPSHOT — and that is not a
        // cosmetic choice.
        //
        // A link rated 3 units/s pulling from a mine that makes 0.85 empties
        // its source on the first tick and finds it bare on the next: the
        // instantaneous rate alternates between "everything" and "nothing"
        // at the lane's 10 Hz. Anything reading it — the belt's cargo
        // spacing, F4's future graphs — flickers in step. What the player
        // actually wants to know is what the link MOVES, which is the mean.
        //
        // Exponential moving average with a `tau`-second memory. It is
        // dt-weighted, so it stays correct when the lane hands it eight
        // hours of bulk catch-up: a step longer than the memory simply
        // lands on the instantaneous value, which over that step IS the
        // average.
        constexpr f64 kFlowMemorySeconds = 4.0;
        const f64 blend = (dt >= kFlowMemorySeconds)
                              ? 1.0
                              : (1.0 - std::exp(-dt / kFlowMemorySeconds));

        world.forEach<ItemLinkComponent, InventoryComponent>(
            [&world, dt, blend](ecs::Entity, ItemLinkComponent& link,
                                InventoryComponent& destination) {
                // Every FEED into this machine, independently. A synthesiser
                // waiting on oxygen must still be taking hydrogen in, or the
                // chain deadlocks on whichever gas the belt happened to be
                // built for first.
                for (LinkChannel& channel : link.channels)
                {
                    if (channel.resource == res::Resource::Count ||
                        channel.unitsPerSecond <= 0.0)
                    {
                        continue;
                    }
                    auto* source =
                        world.tryGetComponent<InventoryComponent>(channel.source);
                    if (source == nullptr)
                    {
                        continue; // source gone: channel idles, flow frozen
                    }

                    f64 moved = 0.0;
                    const f64 wanted =
                        std::min(channel.unitsPerSecond * dt,
                                 inventoryCount(*source, channel.resource));
                    if (wanted > 0.0)
                    {
                        // Accept first (volume-bound), then take exactly that.
                        const f64 accepted =
                            inventoryAdd(destination, channel.resource, wanted);
                        if (accepted > 0.0)
                        {
                            inventoryRemove(*source, channel.resource, accepted);
                        }
                        moved = accepted;
                    }
                    channel.flowUnitsPerSecond +=
                        (moved / dt - channel.flowUnitsPerSecond) * blend;
                }
            });
    }

    // ------------------------------------------------------------------------
    // PowerGridSystem
    //
    // Per site, once a tick:
    //   1. what the panels are ACTUALLY making — the real star's elevation
    //      over this exact patch of ground, zero in eclipse;
    //   2. what every machine is asking for;
    //   3. the shortfall taken from the batteries, or the surplus put back;
    //   4. the allocation, in priority bands, into each `satisfaction`.
    //
    // Everything downstream — whether a smelter runs, how fast, whether it
    // reports NO POWER — falls out of step 4 and nothing else. There is no
    // "is it night" flag anywhere: night is what step 1 returns.
    // ------------------------------------------------------------------------
    void PowerGridSystem::update(ecs::World& world, f32 deltaSeconds)
    {
        const f64 dt = static_cast<f64>(deltaSeconds);
        if (dt <= 0.0)
        {
            return;
        }

        // The star, and every body that could get in front of it.
        WorldVec3 starPosition{0.0};
        if (const auto* transform = world.tryGetComponent<TransformComponent>(m_star))
        {
            starPosition = transform->position;
        }
        std::vector<Occluder> occluders;
        world.forEach<TransformComponent, phys::GravitySourceComponent>(
            [&](ecs::Entity entity, TransformComponent& transform,
                phys::GravitySourceComponent& source) {
                if (entity == m_star || source.bodyRadius <= 0.0)
                {
                    return;
                }
                occluders.push_back({transform.position, source.bodyRadius});
            });

        // ---- gather, PER GRID ------------------------------------------
        // Not per site. A site is a name and a set of books; a GRID is what
        // the cables actually joined together, and electricity only knows
        // about the second one. `power.gridId` is written by the game's
        // rebuildPowerNetwork after every build and demolition, so this
        // system never walks the cable graph itself — it just reads the
        // component it was handed.
        struct Member
        {
            ecs::Entity entity{};
            PowerClaim claim{};
        };
        struct Grid
        {
            f64 producedKw = 0.0;
            f64 demandKw = 0.0;
            std::vector<Member> members;
            std::vector<ecs::Entity> batteries;
        };
        std::unordered_map<u32, Grid> grids;

        world.forEach<BuildingComponent, PowerComponent, TransformComponent>(
            [&](ecs::Entity entity, BuildingComponent& building, PowerComponent& power,
                TransformComponent& transform) {
                Grid& grid = grids[power.gridId];

                // SOLAR: the nameplate times the sun that is actually there.
                power.actualProducedKw = power.producedKw;
                if (building.category == BuildingCategory::Solar &&
                    power.producedKw > 0.0)
                {
                    // The panel's world-space up. A building's transform
                    // rotation already stands its model +Y on the local
                    // vertical (see placeBuilding), so this IS the local
                    // vertical — no need to go back to the anchor for it.
                    const Vec3 worldUp = transform.rotation * Vec3{0.0f, 1.0f, 0.0f};
                    power.actualProducedKw =
                        power.producedKw *
                        solarFactor(transform.position, worldUp, starPosition, occluders);
                }
                grid.producedKw += power.actualProducedKw;

                // DEMAND. `consumedKw` already carries the idle draw plus the
                // loaded recipe's own appetite (placeBuilding sums them), so
                // a machine with nothing to do still keeps its heaters on —
                // which is why an idle site still drains its batteries.
                const f64 demand = power.consumedKw;
                power.satisfaction = 0.0;
                grid.demandKw += demand;
                grid.members.push_back({entity, {demand, power.priority}});

                if (building.category == BuildingCategory::Battery)
                {
                    grid.batteries.push_back(entity);
                }
            });

        // ---- balance and allocate --------------------------------------
        for (auto& [gridId, grid] : grids)
        {
            f64 available = grid.producedKw;

            // Batteries close the gap, in both directions, bounded by their
            // rated power AND by what they are actually holding.
            for (const ecs::Entity entity : grid.batteries)
            {
                auto* battery = world.tryGetComponent<BatteryComponent>(entity);
                auto* store = world.tryGetComponent<InventoryComponent>(entity);
                if (battery == nullptr || store == nullptr)
                {
                    continue;
                }
                battery->flowKw = 0.0;
                if (available < grid.demandKw)
                {
                    // DISCHARGE. 1 unit of ElectricCharge is 1 kJ, so kW * s
                    // is units directly — no conversion constant to get wrong.
                    const f64 wantedKw =
                        std::min(grid.demandKw - available, battery->maxDischargeKw);
                    const f64 drawn =
                        inventoryRemove(*store, res::Resource::ElectricCharge,
                                        wantedKw * dt);
                    const f64 deliveredKw = drawn / dt;
                    available += deliveredKw;
                    battery->flowKw = -deliveredKw;
                }
                else if (available > grid.demandKw)
                {
                    const f64 spareKw =
                        std::min(available - grid.demandKw, battery->maxChargeKw);
                    const f64 stored = inventoryAdd(
                        *store, res::Resource::ElectricCharge, spareKw * dt);
                    const f64 takenKw = stored / dt;
                    available -= takenKw;
                    battery->flowKw = takenKw;
                }
            }

            std::vector<PowerClaim> claims;
            claims.reserve(grid.members.size());
            for (const Member& member : grid.members)
            {
                claims.push_back(member.claim);
            }
            std::vector<f64> satisfaction(claims.size(), 0.0);
            allocatePower(claims, available, satisfaction);

            for (usize i = 0; i < grid.members.size(); ++i)
            {
                if (auto* power =
                        world.tryGetComponent<PowerComponent>(grid.members[i].entity))
                {
                    power->satisfaction = satisfaction[i];
                    // The grid's books, copied onto every member. The machine
                    // panel wants to say "this GRID makes 180 kW and wants
                    // 210" while standing in front of one smelter, and the
                    // only alternative to duplicating two doubles is a table
                    // keyed by grid id that would have to live somewhere and
                    // be kept in step with this loop.
                    power->gridProducedKw = grid.producedKw;
                    power->gridConsumedKw = grid.demandKw;
                }
            }

            // The SITE's books are a different question — "how is my base
            // doing" rather than "what is this wire carrying" — so they are
            // summed separately, below, over the site rather than the grid.
        }

        // ---- the site books, for the UI and the logs --------------------
        // A site can hold several grids (that is the point of the mechanic),
        // and it can hold buildings on none of them. So its totals are a
        // plain sum over its own members, not a copy of any one grid's.
        std::unordered_map<ecs::Entity, SiteComponent*> sites;
        world.forEach<SiteComponent>([&](ecs::Entity entity, SiteComponent& site) {
            site.producedKw = 0.0;
            site.consumedKw = 0.0;
            site.batteryFlowKw = 0.0;
            site.buildingCount = 0;
            sites[entity] = &site;
        });
        world.forEach<BuildingComponent, PowerComponent>(
            [&](ecs::Entity entity, BuildingComponent& building, PowerComponent& power) {
                const auto it = sites.find(building.site);
                if (it == sites.end())
                {
                    return;
                }
                it->second->producedKw += power.actualProducedKw;
                it->second->consumedKw += power.consumedKw;
                it->second->buildingCount += 1;
                if (const auto* battery = world.tryGetComponent<BatteryComponent>(entity))
                {
                    it->second->batteryFlowKw += battery->flowKw;
                }
            });
    }

    void ProductionSystem::update(ecs::World& world, f32 deltaSeconds)
    {
        const f64 dt = static_cast<f64>(deltaSeconds);
        if (dt <= 0.0)
        {
            return;
        }

        world.forEach<BuildingComponent, RecipeStateComponent, InventoryComponent>(
            [dt, &world](ecs::Entity entity, BuildingComponent& building,
                         RecipeStateComponent& state, InventoryComponent& inventory) {
                const RecipeDefinition* recipe = findRecipe(state.recipeId);
                if (recipe == nullptr || recipe->requiredCategory != building.category)
                {
                    state.state = RecipeStateComponent::kIdle;
                    return;
                }

                // A mine yields what the ground holds. The density was
                // sampled from the analytic ore field when the building was
                // placed, so a badly sited mine is permanently poor — and a
                // survey before building is worth doing.
                f64 scale = 1.0;
                if (building.category == BuildingCategory::Miner)
                {
                    scale = static_cast<f64>(building.groundDensity);
                    if (scale <= 0.0)
                    {
                        state.state = RecipeStateComponent::kStarved;
                        return;
                    }
                }

                // Power. The site grid writes `satisfaction` (F3); with no
                // power component a machine simply runs.
                f64 satisfaction = 1.0;
                if (const auto* power = world.tryGetComponent<PowerComponent>(entity))
                {
                    satisfaction = std::clamp(power->satisfaction, 0.0, 1.0);
                }
                if (satisfaction <= 0.0)
                {
                    state.state = RecipeStateComponent::kNoPower;
                    return;
                }

                const f64 wantedSeconds = dt * scale;

                // ---- how much of that can we ACTUALLY run? ---------------
                // Two limits, both computed before a single unit moves.
                f64 fraction = 1.0;
                bool starved = false;
                for (const Ingredient& input : recipe->inputs)
                {
                    if (input.resource == res::Resource::Count ||
                        input.unitsPerSecond <= 0.0)
                    {
                        continue;
                    }
                    const f64 wanted = input.unitsPerSecond * wantedSeconds;
                    const f64 available = inventoryCount(inventory, input.resource);
                    if (available < wanted)
                    {
                        fraction = std::min(fraction, available / wanted);
                        starved = true;
                    }
                }
                // Output room is a VOLUME question, not a per-resource one:
                // electrolysis makes two gases that compete for the same
                // tank, and asking each of them separately whether it fits
                // said yes to both — then the second one silently did not
                // fit and the water that became it was already gone. Matter
                // died in that gap; IndustryTests found it.
                //
                // The inputs are removed BEFORE the outputs are added, so
                // what they free counts. A recipe whose products pack
                // tighter than its feedstock (smelting) can never block.
                bool blocked = false;
                f64 netVolumePerSecond = 0.0;
                for (const Ingredient& output : recipe->outputs)
                {
                    if (output.resource == res::Resource::Count)
                    {
                        continue;
                    }
                    netVolumePerSecond += output.unitsPerSecond *
                                          res::definition(output.resource).volumePerUnitM3;
                    // Each product also needs somewhere to sit.
                    if (inventoryFreeUnits(inventory, output.resource) <= 0.0)
                    {
                        fraction = 0.0;
                        blocked = true;
                    }
                }
                for (const Ingredient& input : recipe->inputs)
                {
                    if (input.resource == res::Resource::Count)
                    {
                        continue;
                    }
                    netVolumePerSecond -= input.unitsPerSecond *
                                          res::definition(input.resource).volumePerUnitM3;
                }
                if (netVolumePerSecond > 0.0)
                {
                    const f64 freeVolume = std::max(
                        0.0, inventory.volumeCapacityM3 - inventoryVolume(inventory));
                    const f64 wantedVolume = netVolumePerSecond * wantedSeconds;
                    if (freeVolume < wantedVolume)
                    {
                        fraction = std::min(fraction, freeVolume / wantedVolume);
                        blocked = true;
                    }
                }
                // The MATERIAL limit and the ELECTRICAL limit are different
                // diagnoses and the panel shows them differently: a machine
                // on half power is RUNNING, at half speed, and saying NO
                // POWER at it would send the player looking for a fault that
                // is not there. NO POWER means stopped — see the early
                // return above, which is the only place it comes from.
                const f64 materialFraction = std::clamp(fraction, 0.0, 1.0);
                fraction = std::clamp(materialFraction * satisfaction, 0.0, 1.0);

                if (fraction <= 1.0e-12)
                {
                    state.state = blocked ? RecipeStateComponent::kBlocked
                                          : RecipeStateComponent::kStarved;
                    return;
                }

                // ---- move the matter -------------------------------------
                const f64 seconds = wantedSeconds * fraction;
                for (const Ingredient& input : recipe->inputs)
                {
                    if (input.resource == res::Resource::Count)
                    {
                        continue;
                    }
                    state.consumedUnits += inventoryRemove(
                        inventory, input.resource, input.unitsPerSecond * seconds);
                }
                for (const Ingredient& output : recipe->outputs)
                {
                    if (output.resource == res::Resource::Count)
                    {
                        continue;
                    }
                    state.producedUnits += inventoryAdd(
                        inventory, output.resource, output.unitsPerSecond * seconds);
                }

                state.state = (materialFraction >= 0.999)
                                  ? RecipeStateComponent::kRunning
                                  : (blocked ? RecipeStateComponent::kBlocked
                                             : RecipeStateComponent::kStarved);
            });
    }

} // namespace sw::factory
