#include "Factory/FactorySystems.hpp"

#include "ECS/World.hpp"

#include <algorithm>

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
        world.forEach<ItemLinkComponent, InventoryComponent>(
            [&world, dt](ecs::Entity, ItemLinkComponent& link,
                         InventoryComponent& destination) {
                auto* source = world.tryGetComponent<InventoryComponent>(link.source);
                if (source == nullptr)
                {
                    return; // source gone (destroyed): link idles
                }

                const f64 wanted = std::min(link.unitsPerSecond * dt,
                                            inventoryCount(*source, link.resource));
                if (wanted <= 0.0)
                {
                    link.flowUnitsPerSecond = 0.0;
                    return;
                }
                // Accept first (volume-bound), then take exactly that much.
                const f64 accepted = inventoryAdd(destination, link.resource, wanted);
                if (accepted > 0.0)
                {
                    inventoryRemove(*source, link.resource, accepted);
                }
                // Measured throughput, not the rating: this is what the belt
                // is drawn moving, and what tells you at a glance whether
                // the chain upstream is keeping up.
                link.flowUnitsPerSecond = (dt > 0.0) ? accepted / dt : 0.0;
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
                fraction = std::clamp(fraction * satisfaction, 0.0, 1.0);

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

                state.state = (fraction >= 0.999)
                                  ? RecipeStateComponent::kRunning
                                  : (blocked ? RecipeStateComponent::kBlocked
                                             : (starved ? RecipeStateComponent::kStarved
                                                        : RecipeStateComponent::kNoPower));
            });
    }

} // namespace sw::factory
