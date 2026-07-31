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
    namespace
    {
        /// Is there ANYWHERE in this inventory for `resource` to go — an
        /// existing stack of it, or an empty slot?
        ///
        /// Deliberately not a volume question. `inventoryFreeUnits` answers
        /// both at once, which is exactly wrong for a machine that is about
        /// to consume its inputs first: the slot layout is a fact about the
        /// bin right now, while the free volume is a fact about the bin
        /// AFTER the feedstock leaves it. Mixing the two made a full bin
        /// look permanently jammed.
        [[nodiscard]] bool inventoryHasSlotFor(const InventoryComponent& inventory,
                                               res::Resource resource)
        {
            if (resource == res::Resource::Count)
            {
                return false;
            }
            for (const InventorySlot& slot : inventory.slots)
            {
                if (slot.resource == resource || slot.resource == res::Resource::Count)
                {
                    return true;
                }
            }
            return false;
        }
    } // namespace

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
                // THE RATIO IS NOT TRUSTED, because nothing checks it on the
                // way in. `loadRecipeFile` refuses a recipe that creates
                // matter, but this component is not a recipe: it is written
                // by the asteroid rig, by the save loader, and by anything
                // that can set an f64 — and a save file with a corrupt or
                // absent field lands here as 0, as garbage, or as NaN.
                //
                // Two separate hazards, so two separate guards.
                //
                //  * ZERO OR NEGATIVE is INERT. The consumption below divides
                //    by this number, so zero is an infinity that empties the
                //    hopper into nothing, and a negative ratio adds negative
                //    output (a no-op) and then REMOVES a negative amount of
                //    input, which `inventoryRemove` refuses — leaving a plant
                //    that runs forever on ore it never eats. Neither is a
                //    refinery. A machine set up to convert nothing converts
                //    nothing.
                //  * TOO LARGE CREATES MATTER. One unit in may not become
                //    more KILOGRAMS out than it arrived with, which is the
                //    same rule `RecipesConserveMatter` holds every recipe in
                //    the catalogue to. Iron ore and iron are both 1 kg per
                //    unit, so the ceiling there is 1.0; a resource pair with
                //    different unit masses gets the mass-honest ceiling
                //    rather than a hard-coded one.
                //
                // The clamp is written BACK, not applied privately at the use
                // site, so that the machine panel, the save file and the
                // arithmetic below all quote the same number. A ratio the
                // simulation refuses to honour must not keep being displayed.
                if (!(refinery.conversionRatio > 0.0)) // false for NaN too
                {
                    refinery.conversionRatio = 0.0;
                    return; // inert, not a divide-by-zero
                }
                const f64 inputMassPerUnit =
                    res::definition(refinery.input).massPerUnitKg;
                const f64 outputMassPerUnit =
                    res::definition(refinery.output).massPerUnitKg;
                if (outputMassPerUnit > 0.0)
                {
                    refinery.conversionRatio =
                        std::min(refinery.conversionRatio,
                                 inputMassPerUnit / outputMassPerUnit);
                }
                const f64 ratio = refinery.conversionRatio;

                const f64 wantedInput = refinery.inputUnitsPerSecond * dt;
                const f64 availableInput =
                    std::min(wantedInput, inventoryCount(inventory, refinery.input));
                if (availableInput <= 0.0)
                {
                    return; // starved
                }

                // Reserve output space FIRST: never destroy matter.
                const f64 wantedOutput = availableInput * ratio;
                const f64 acceptedOutput =
                    inventoryAdd(inventory, refinery.output, wantedOutput);
                if (acceptedOutput <= 0.0)
                {
                    return; // output full: stall
                }

                const f64 consumedInput = acceptedOutput / ratio;
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

                    // WHICH WAY THE GROUND UNDER THE PANEL IS TURNING, so the
                    // sunlight can be averaged over the tick rather than
                    // sampled at the instant it starts (see
                    // averageSolarFactor — this is the F3 bug that made a
                    // warping player's base generate a whole day of power
                    // from a moment that happened to be noon).
                    //
                    // The route is site -> body because that is the only
                    // statement of what a building is STANDING ON that the
                    // factory layer has; a hall with no site, or a site on no
                    // body, is treated as not turning, which is exactly right
                    // for an orbital platform and harmless for anything else.
                    WorldVec3 spinCentre{0.0};
                    WorldVec3 spin{0.0};
                    if (const auto* site =
                            world.tryGetComponent<SiteComponent>(building.site))
                    {
                        if (const auto* body =
                                world.tryGetComponent<phys::GravitySourceComponent>(
                                    site->body))
                        {
                            spin = body->angularVelocity;
                            if (const auto* centre =
                                    world.tryGetComponent<TransformComponent>(site->body))
                            {
                                spinCentre = centre->position;
                            }
                        }
                    }

                    power.actualProducedKw =
                        power.producedKw *
                        averageSolarFactor(transform.position, worldUp, starPosition,
                                           occluders, spinCentre, spin, dt);
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
                    // Each product also needs a SLOT to sit in — and that is
                    // the only per-product question left, because the volume
                    // is the net one below.
                    //
                    // Asking `inventoryFreeUnits` here instead was a deadlock,
                    // and a nasty one because it looked like caution. It
                    // measures the room that exists NOW, before the inputs
                    // are removed; a smelter whose bin has filled to the brim
                    // with its own ore therefore has zero free units of iron,
                    // reports BLOCKED and stops — forever, because the only
                    // thing that could ever free that volume is the smelt it
                    // just refused to run. Ore packs at 2,500 kg/m^3 and iron
                    // at 7,870, so the smelt would have freed volume rather
                    // than needing any: the machine starved to death in front
                    // of a full larder. The net-volume test below asks the
                    // question that actually matters — will the products fit
                    // in what is free PLUS what the feedstock vacates — and a
                    // slot is a separate, genuinely current fact.
                    if (!inventoryHasSlotFor(inventory, output.resource))
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

    // ------------------------------------------------------------------------
    // THE ASSEMBLY HALL
    // ------------------------------------------------------------------------
    void AssemblySystem::update(ecs::World& world, f32 deltaSeconds)
    {
        const f64 dt = static_cast<f64>(deltaSeconds);
        if (dt <= 0.0)
        {
            return;
        }

        world.forEach<AssemblyComponent, InventoryComponent>(
            [dt, &world](ecs::Entity entity, AssemblyComponent& assembly,
                         InventoryComponent& inventory) {
                const f64 needed = assembly.ironNeededKg + assembly.copperNeededKg;
                if (assembly.blueprint[0] == '\0' || needed <= 0.0)
                {
                    assembly.state = RecipeStateComponent::kIdle;
                    return;
                }

                f64 satisfaction = 1.0;
                if (const auto* power = world.tryGetComponent<PowerComponent>(entity))
                {
                    satisfaction = std::clamp(power->satisfaction, 0.0, 1.0);
                }
                if (satisfaction <= 0.0)
                {
                    assembly.state = RecipeStateComponent::kNoPower;
                    return;
                }

                // THE TICK'S WHOLE BUDGET, spent until it is gone.
                //
                // This used to work one hull per call and drop whatever was
                // left over, which is a bug the Automation lane's bulk
                // catch-up turns into a disaster rather than a rounding
                // error: at warp the lane hands this system a single tick
                // standing for eight hours.
                //
                // MEASURED, on a 40 kg/s hall with a 12-tonne bill (9,600 kg
                // of iron and 2,400 of copper) handed one 28,800 s tick: the
                // budget is 1,152,000 kg, which is ninety-six airframes. The
                // old code finished ONE, spent 12,000 kg, and dropped the
                // other 1,140,000 kg of budget on the floor — the factory
                // ran at 1/96th speed for exactly as long as the player
                // warped, which is the one thing warp is supposed to be free
                // of. With the loop, that single tick finishes 96 hulls and
                // spends 921,600.000000 kg of iron, which is what 28,800
                // ticks of one second spend, to the last printed digit.
                //
                // So it is a loop, and the loop's exit conditions are the
                // real ones: the budget runs out, the metal runs out, the
                // bin has no room for another crate, or the apron is full.
                // Everything below the loop is per-hull and unchanged; what
                // changed is that it may happen more than once.
                f64 budget = assembly.buildRateKgPerSecond * dt * satisfaction;
                if (!(budget > 0.0)) // false for NaN, which must not reach the cast
                {
                    budget = 0.0;
                }

                // HOW MANY HULLS THIS TICK COULD POSSIBLY FINISH — the bound
                // that makes the loop provably terminate. The metal that can
                // be worked is what is already on the slipway (strictly less
                // than one bill) plus this tick's budget, so one bill's worth
                // of budget is one extra hull and the carry-over is the +1.
                // Without a bound, a degenerate order whose bill is smaller
                // than the completion tolerance would finish for free, reset
                // the slipway and finish again, and the tick would never end.
                constexpr f64 kGram = 1.0e-3;
                const f64 hullsFromBudget =
                    std::floor(budget / std::max(needed, 2.0 * kGram));
                const u32 hullLimit =
                    static_cast<u32>(std::min(hullsFromBudget, 1.0e6)) + 1u;

                bool starved = false;
                bool blocked = false;
                u32 finished = 0;
                while (finished < hullLimit)
                {
                    // ROOM FOR THE FINISHED HULL, asked before a gram of
                    // metal is worked. A hall that spent twelve tonnes of
                    // iron and then found nowhere to put the rocket would
                    // have destroyed it.
                    if (inventoryFreeUnits(inventory, res::Resource::Vehicle) < 1.0)
                    {
                        blocked = true;
                        break;
                    }

                    const f64 remainingIron =
                        std::max(0.0, assembly.ironNeededKg - assembly.ironPaidKg);
                    const f64 remainingCopper =
                        std::max(0.0, assembly.copperNeededKg - assembly.copperPaidKg);
                    const f64 remaining = remainingIron + remainingCopper;

                    if (remaining > 1.0e-9)
                    {
                        if (budget <= 0.0)
                        {
                            break; // out of tick, not out of anything real
                        }
                        // Both metals at once, in the proportion of what is
                        // LEFT to pay. Pouring the iron first would let a
                        // hall with no copper drain every smelter in the base
                        // and then stop one gram short, which reads as a
                        // supply fault where there is none.
                        const f64 wanted = std::min(budget, remaining);
                        const f64 wantIron = wanted * (remainingIron / remaining);
                        const f64 wantCopper = wanted - wantIron;

                        const f64 gotIron =
                            inventoryRemove(inventory, res::Resource::Iron, wantIron);
                        const f64 gotCopper =
                            inventoryRemove(inventory, res::Resource::Copper, wantCopper);
                        assembly.ironPaidKg += gotIron;
                        assembly.copperPaidKg += gotCopper;
                        budget -= gotIron + gotCopper;
                        starved = (gotIron + 1.0e-9 < wantIron) ||
                                  (gotCopper + 1.0e-9 < wantCopper);
                    }

                    // ---- is it finished? ---------------------------------
                    if (assembly.ironPaidKg + kGram < assembly.ironNeededKg ||
                        assembly.copperPaidKg + kGram < assembly.copperNeededKg)
                    {
                        break; // still on the slipway: starved, or out of tick
                    }

                    auto* queue = world.tryGetComponent<VehicleQueueComponent>(entity);
                    if (queue != nullptr && queue->count >= kVehicleQueueSlots)
                    {
                        // Eight unclaimed hulls on the apron: stop, rather
                        // than ship a rocket whose design nobody recorded.
                        blocked = true;
                        break;
                    }
                    const f64 crated =
                        inventoryAdd(inventory, res::Resource::Vehicle, 1.0);
                    if (crated < 1.0)
                    {
                        // Should not happen — the room was checked above —
                        // but if it ever does, the metal stays on the
                        // slipway rather than evaporating.
                        inventoryRemove(inventory, res::Resource::Vehicle, crated);
                        blocked = true;
                        break;
                    }
                    if (queue != nullptr)
                    {
                        vehicleQueuePush(*queue, std::string_view(assembly.blueprint));
                    }
                    assembly.ironPaidKg = 0.0;
                    assembly.copperPaidKg = 0.0;
                    ++assembly.completed;
                    ++finished;
                }

                // BLOCKED outranks STARVED, and both outrank RUNNING: a hall
                // that finished four hulls and then ran out of somewhere to
                // put the fifth is blocked NOW, and that is what the panel
                // must say. Reporting the four it managed would send the
                // player looking somewhere else entirely.
                assembly.state = blocked  ? RecipeStateComponent::kBlocked
                                 : starved ? RecipeStateComponent::kStarved
                                           : RecipeStateComponent::kRunning;
            });
    }

} // namespace sw::factory
