#pragma once

// ============================================================================
// Systems.hpp — game-side ECS systems (Physics-lane companions to the
// engine's physics systems).
//
// Simulation-cost policy (Milestone 6): per-tick work only touches entities
// that are REALLY simulated. Snapshot and Spin therefore require
// DynamicBodyComponent in their queries — thousands of on-rails debris cost
// nothing here. Celestial bodies (a handful) have their own spin system.
// ============================================================================

#include "Components.hpp"

namespace game
{
    /// Physics lane, first: snapshots dynamic transforms so the renderer
    /// can interpolate between the previous and current tick.
    class SnapshotSystem final : public sw::ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override { return "SnapshotSystem"; }

        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}
                .read<TransformComponent>()
                .read<sw::phys::DynamicBodyComponent>()
                .write<PreviousTransformComponent>();
        }

        void update(sw::ecs::World& world, sw::f32 deltaSeconds) override;
    };

    /// Self-rotation of really-simulated (dynamic) objects.
    class SpinSystem final : public sw::ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override { return "SpinSystem"; }

        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}
                .write<TransformComponent>()
                .read<SpinComponent>()
                .read<sw::phys::DynamicBodyComponent>();
        }

        void update(sw::ecs::World& world, sw::f32 deltaSeconds) override;
    };

    /// Rotation of celestial bodies (the day cycle), ON RAILS.
    ///
    /// The angle is a pure function of the lane's present time, not an
    /// integration — for exactly the reason the orbits are analytic. Under
    /// time warp the Physics lane keeps strict fixed steps and DROPS the
    /// backlog it cannot afford, so an incremental spin quietly falls
    /// behind: the planet's position jumped around the star while its
    /// surface barely turned. A closed form is also exact across a save,
    /// and puts a landed base back under the same star it went to sleep
    /// under. Also maintains the Previous snapshot for interpolation.
    class CelestialSpinSystem final : public sw::ecs::System
    {
    public:
        /// `timebase`: the lane this system runs in (Physics) — the same
        /// present the celestial POSITIONS are evaluated at, so a body's
        /// spin and its orbit never disagree by a tick.
        explicit CelestialSpinSystem(const sw::sim::SimulationLane& timebase)
            : m_timebase(timebase)
        {
        }

        [[nodiscard]] std::string_view name() const override
        {
            return "CelestialSpinSystem";
        }

        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}
                .write<TransformComponent>()
                .write<PreviousTransformComponent>()
                .read<SpinComponent>()
                .write<sw::phys::GravitySourceComponent>();
        }

        void update(sw::ecs::World& world, sw::f32 deltaSeconds) override;

    private:
        const sw::sim::SimulationLane& m_timebase;
    };

    /// Keeps atmosphere/cloud shells centered on their body, spinning at
    /// their own rate (clouds drift over the ground). Runs after the
    /// celestial motion so the center is this tick's.
    class CloudLayerSystem final : public sw::ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override
        {
            return "CloudLayerSystem";
        }

        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}
                .write<TransformComponent>()
                .write<PreviousTransformComponent>()
                .read<CloudLayerComponent>();
        }

        void update(sw::ecs::World& world, sw::f32 deltaSeconds) override;
    };

    /// Automation lane: solar wings trickle-charge their vessel's
    /// batteries (flat rate while deployed; eclipse awareness later).
    class SolarChargeSystem final : public sw::ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override
        {
            return "SolarChargeSystem";
        }

        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}
                .write<sw::factory::InventoryComponent>()
                .read<sw::parts::PartComponent>()
                .read<sw::parts::VesselComponent>();
        }

        void update(sw::ecs::World& world, sw::f32 deltaSeconds) override;
    };

    /// SAS: drives the craft's angular velocity to align its nose with the
    /// prograde/retrograde direction (relative to the SOI primary). Pauses
    /// whenever the pilot touches the rotation controls. Runs just before
    /// ThrustSystem, which integrates the commanded angular velocity.
    class SasSystem final : public sw::ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override { return "SasSystem"; }

        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}
                .write<ShipComponent>()
                .read<TransformComponent>()
                .read<sw::phys::DynamicBodyComponent>()
                .read<sw::phys::GravitySourceComponent>()
                .read<ShipControlsComponent>()
                .read<SasComponent>();
        }

        void update(sw::ecs::World& world, sw::f32 deltaSeconds) override;
    };

    /// Physics lane: applies pilot thrust and attitude control to ships.
    /// Runs after gravity so the pilot's acceleration adds on top of the
    /// gravitational one within the same tick.
    class ThrustSystem final : public sw::ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override { return "ThrustSystem"; }

        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}
                .write<TransformComponent>()
                .write<sw::phys::DynamicBodyComponent>()
                .write<ShipComponent>()
                .write<sw::factory::InventoryComponent>() // engines burn fuel
                .read<sw::parts::PartComponent>()
                .read<sw::parts::VesselComponent>()
                .read<ShipControlsComponent>();
        }

        void update(sw::ecs::World& world, sw::f32 deltaSeconds) override;
    };

    /// Physics lane: grounded capsule locomotion (EVA). Runs after the
    /// surface system so isGrounded reflects the current tick.
    class CapsuleMovementSystem final : public sw::ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override
        {
            return "CapsuleMovementSystem";
        }

        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}
                .write<TransformComponent>()
                .write<sw::phys::DynamicBodyComponent>()
                .write<CapsuleComponent>()
                .read<ShipControlsComponent>()
                .read<sw::phys::GravitySourceComponent>();
        }

        void update(sw::ecs::World& world, sw::f32 deltaSeconds) override;
    };

    /// World lane (1 Hz): periodic simulation statistics to the log.
    class StatsSystem final : public sw::ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override { return "StatsSystem"; }

        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}
                .read<sw::factory::InventoryComponent>()
                .read<sw::factory::MinerComponent>()
                .read<sw::factory::RefineryComponent>();
        }

        void update(sw::ecs::World& world, sw::f32 deltaSeconds) override;

    private:
        sw::u64 m_ticks = 0;
    };
} // namespace game
