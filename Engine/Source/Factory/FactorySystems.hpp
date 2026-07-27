#pragma once

// ============================================================================
// Factory/FactorySystems.hpp
// Production and logistics systems. See FactoryComponents.hpp for the data
// model. Lane placement (game decides): Miner/Refinery in Automation
// (5 Hz), Transfer in Logistics (10 Hz).
// ============================================================================

#include "ECS/System.hpp"
#include "Factory/FactoryComponents.hpp"
#include "Factory/Power.hpp"

namespace sw::factory
{
    /// Extracts the miner's output resource into its own inventory.
    class MinerSystem final : public ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override { return "MinerSystem"; }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .write<MinerComponent>()
                .write<InventoryComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;
    };

    /// Converts input units to output units inside the same inventory.
    /// Output volume is reserved before input is consumed: a full tank
    /// stalls the refinery instead of destroying matter.
    class RefinerySystem final : public ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override { return "RefinerySystem"; }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .write<RefineryComponent>()
                .write<InventoryComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;
    };

    /// Pulls resources across entity links (drone/conveyor abstraction).
    class TransferSystem final : public ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override { return "TransferSystem"; }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .read<ItemLinkComponent>()
                .write<InventoryComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;
    };
    /// THE production executor (F1).
    ///
    /// One system for every machine the player builds, driven entirely by
    /// the recipe catalogue. It runs in the Automation lane, which uses BULK
    /// CATCH-UP: `deltaSeconds` may be one tick or an hour of warped time,
    /// and because every rate is units-per-second the result is identical
    /// either way. That property is tested, not assumed.
    ///
    /// The order of operations is the part that matters: it works out what
    /// fraction of a full second it can actually run — limited by the inputs
    /// present AND by the room left for the outputs — and only then moves
    /// any matter. A machine that consumed its input and then found the
    /// output bin full would have destroyed matter, which is the one thing
    /// this codebase does not do.
    /// THE SITE GRID (Automation lane, BEFORE the production executor).
    ///
    /// Every tick it answers one question per site — how much electricity is
    /// there, and who gets it — and writes the answer into each building's
    /// `satisfaction`, which is the only thing the executor reads. That
    /// separation is the point: production does not know about the sun, and
    /// the grid does not know about recipes.
    ///
    /// Solar output is the REAL star at the REAL local hour: elevation above
    /// the site's own horizon, zero in eclipse. A fourteen-day lunar night is
    /// therefore not a special case in the code — it is what this arithmetic
    /// says when you point it at the Moon.
    ///
    /// Needs the star: construct it with the entity everything orbits.
    class PowerGridSystem final : public ecs::System
    {
    public:
        explicit PowerGridSystem(ecs::Entity star) : m_star(star) {}

        [[nodiscard]] std::string_view name() const override
        {
            return "PowerGridSystem";
        }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .write<PowerComponent>()
                .write<BatteryComponent>()
                .write<SiteComponent>()
                .write<InventoryComponent>() // batteries hold ElectricCharge
                .read<BuildingComponent>()
                .read<RecipeStateComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;

        void setStar(ecs::Entity star) { m_star = star; }

    private:
        ecs::Entity m_star{};
    };

    class ProductionSystem final : public ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override
        {
            return "ProductionSystem";
        }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .write<InventoryComponent>()
                .write<RecipeStateComponent>()
                .read<BuildingComponent>()
                .read<PowerComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;
    };

} // namespace sw::factory
