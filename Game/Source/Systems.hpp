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

    /// SELF-ROTATION OF A RAILED CRAFT — a ring ship's artificial gravity.
    ///
    /// A station that spins for gravity is not a dynamic body. The
    /// Endurance rides a Kepler orbit like any other unattended vessel,
    /// and RailsSystem writes only its POSITION — which leaves its
    /// attitude free, and free is exactly what a 5.6 rpm ring needs.
    ///
    /// ANALYTIC, like the day cycle and for the same reason: under warp
    /// the Physics lane keeps strict steps and DROPS the backlog it cannot
    /// afford, so an integrated spin quietly falls behind the orbit it is
    /// riding — and a save reloads with the ring at an angle it never
    /// passed through. angle = rate x present is exact at any warp and
    /// across a save.
    ///
    /// The hand-off costs nothing: when the bubble wakes the vessel into a
    /// DynamicBody, this system stops seeing it and SpinSystem picks it up
    /// mid-turn, integrating on from the attitude this one left behind.
    class RailsSpinSystem final : public sw::ecs::System
    {
    public:
        explicit RailsSpinSystem(const sw::sim::SimulationLane& timebase)
            : m_timebase(timebase)
        {
        }

        [[nodiscard]] std::string_view name() const override
        {
            return "RailsSpinSystem";
        }

        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}
                .write<TransformComponent>()
                .write<PreviousTransformComponent>()
                .read<SpinComponent>()
                .read<sw::phys::OnRailsComponent>();
        }

        void update(sw::ecs::World& world, sw::f32 deltaSeconds) override;

    private:
        const sw::sim::SimulationLane& m_timebase;
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
                .write<sw::phys::DynamicBodyComponent>() // the craft's own spin
                .read<TransformComponent>()
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

        /// CREATIVE MODE: engines produce full thrust and burn nothing.
        /// A flag on the system rather than per-entity state, because the
        /// mode belongs to the SESSION (it is chosen at NEW GAME and rides
        /// in the save), not to any one vessel — and the host owns it in
        /// multiplayer for the same reason it owns everything else.
        void setInfiniteFuel(bool on) { m_infiniteFuel = on; }

        void update(sw::ecs::World& world, sw::f32 deltaSeconds) override;

    private:
        bool m_infiniteFuel = false;
    };

    /// Physics lane: turns angular velocity into ATTITUDE, for every
    /// dynamic body — not just the ones somebody is flying.
    ///
    /// This used to live inside ThrustSystem, whose query is
    /// <Transform, DynamicBody, Ship, ShipControls>. That gate was the bug:
    /// ThrustSystem was the only thing in the build that integrated
    /// DynamicBodyComponent::angularVelocity into a rotation, so a crate, a
    /// piece of debris, a dropped fuel tank — anything with a body and a
    /// hull but no cockpit — had the ground's toppling torque written into
    /// its angular velocity by SurfaceInteractionSystem every single tick
    /// and then never turned by so much as a milliradian. The spin
    /// accumulated, the friction rubbed it off again, and the box sat there
    /// perfectly upright on its corner for ever.
    ///
    /// MEASURED, driving the engine's own SurfaceInteractionSystem for 20 s
    /// on an airless 1737 km world. A 1.2 m crate tipped 30 degrees onto an
    /// edge: it carried 0.11771 rad/s of toppling spin and turned 0.00
    /// degrees, and now settles back towards its base, 30.00 -> 23.26
    /// degrees, with 0.00589 rad/s left. A 2.4 m crate tipped 50 degrees,
    /// which is well past the angle it can recover from: 0.17148 rad/s and
    /// 0.00 degrees spent, against 50.00 -> 91.00 degrees now — it lies
    /// down, which is the whole point. (Those magnitudes belong to the
    /// ground model's toppling maths; what this system owns is the 0.00.)
    ///
    /// A SHIP is unaffected to the last digit: one second of full pitch
    /// input leaves it at 0.500000 rad/s having turned 0.254166 rad, both
    /// before the split and after. Integrated once, by exactly as much.
    ///
    /// Splitting it out rather than widening ThrustSystem's query keeps the
    /// two jobs honest: ThrustSystem COMMANDS a rate (RCS input, kill
    /// rotation, the ship's turn-rate limit), this system SPENDS it. It is
    /// registered immediately after ThrustSystem, and the scheduler cannot
    /// merge the two into one parallel stage because they disagree about
    /// TransformComponent — so a ship is integrated at exactly the same
    /// point in the tick it always was, exactly once.
    class AngularIntegrationSystem final : public sw::ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override
        {
            return "AngularIntegrationSystem";
        }

        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}
                .write<TransformComponent>()
                .read<sw::phys::DynamicBodyComponent>()
                .read<sw::parts::VesselComponent>() // turn about the balance point
                .read<CapsuleComponent>();          // ...but never the walker
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
