#pragma once

// ============================================================================
// Physics/PhysicsSystems.hpp
// Engine systems implementing the two-regime physics architecture.
//
//  GravityIntegrationSystem (Physics lane, 50 Hz)
//      Semi-implicit Euler in f64 over every DynamicBody: only the handful
//      of objects inside the simulation bubble ever pay this cost.
//
//  RailsSystem (Physics lane, 50 Hz)
//      Writes the CURRENT analytic position of every on-rails object into
//      its transform: primary's world position + primary-relative Kepler
//      evaluation. Previous = last tick's position, so rails objects
//      interpolate — mandatory now that their primaries (planets) race
//      around the star at tens of km/s. Still the "not really simulated"
//      tier: one closed-form solve, no integration ever.
//
//  SimulationBubbleSystem (Logistics lane, 10 Hz)
//      Classifies objects by distance to the focus (the player/camera):
//      rails objects entering the bubble become dynamic (state vectors from
//      the analytic orbit — position/velocity continuous); dynamic objects
//      leaving it are converted back to rails (elements from state vectors,
//      relative to their SOI primary — the deepest body whose sphere of
//      influence contains them, never simply the heaviest one: near Luna
//      you orbit Luna, not the Sun). Hysteresis (exit > enter radius)
//      prevents churn at the boundary. Conversions are structural, so they
//      go through the command buffer and apply at playback.
// ============================================================================

#include "ECS/CommandBuffer.hpp"
#include "ECS/System.hpp"
#include "Physics/PhysicsComponents.hpp"
#include "Planet/Terrain.hpp"
#include "Scene/TransformComponents.hpp"

#include <vector>

namespace sw::sim
{
    class SimulationLane;
} // namespace sw::sim

namespace sw::phys
{
    class GravityIntegrationSystem final : public ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override
        {
            return "GravityIntegrationSystem";
        }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .write<TransformComponent>()
                .write<DynamicBodyComponent>()
                .read<GravitySourceComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;

    private:
        struct Source
        {
            WorldVec3 position;
            f64 mu;
        };
        std::vector<Source> m_sources; // scratch, rebuilt each tick
    };

    /// Keeps surface-anchored entities glued to their (rotating) body:
    /// world position = body position + body rotation * local position.
    /// Runs after the celestial spin update. Previous mirrors current
    /// (anchored structures don't interpolate — the spin is imperceptible
    /// per frame). Anchored entities must NOT also be DynamicBody/OnRails.
    class SurfaceAnchorSystem final : public ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override
        {
            return "SurfaceAnchorSystem";
        }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .write<TransformComponent>()
                .write<PreviousTransformComponent>()
                .read<SurfaceAnchorComponent>()
                .read<GravitySourceComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;
    };

    /// Atmosphere drag + solid ground (Physics lane, after gravity/thrust).
    ///
    ///  - Drag: quadratic, exponential density profile, applied inside a
    ///    body's atmosphere; deceleration never reverses the velocity.
    ///  - Ground: the body surface is solid. Bodies below it are clamped to
    ///    the surface, inward radial velocity is absorbed (a hard impact is
    ///    logged as a crash, a gentle one as a landing) and tangential
    ///    velocity decays under friction until the object rests. Sets
    ///    DynamicBody::isGrounded for movement/controller code.
    ///
    /// All velocities are measured RELATIVE to the body's own world-frame
    /// velocity (GravitySource::worldVelocity): the atmosphere and the
    /// ground travel with the planet, so a ship landed on a world moving
    /// 30 km/s around its star stays landed.
    ///
    /// Current simplification (documented): the surface does not co-rotate
    /// with the body's spin — friction stops objects in the body's
    /// TRANSLATING frame, not its rotating one.
    class SurfaceInteractionSystem final : public ecs::System
    {
    public:
        struct Config
        {
            f64 crashSpeedThreshold = 8.0; // m/s vertical: crash vs landing
            f64 groundFrictionPerSecond = 2.5;
        };

        explicit SurfaceInteractionSystem(const Config& config) : m_config(config) {}

        [[nodiscard]] std::string_view name() const override
        {
            return "SurfaceInteractionSystem";
        }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .write<TransformComponent>()
                .write<DynamicBodyComponent>()
                .read<GravitySourceComponent>()
                .read<AtmosphereComponent>()
                .read<GroundHullComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;

    private:
        struct Surface
        {
            WorldVec3 center;
            WorldVec3 velocity;        // translation of the body
            WorldVec3 angularVelocity; // spin: surface moves at v + w x r
            Quat rotation;             // body orientation (terrain is body-fixed)
            glm::dquat rotation64;     // the same, exact: see GravitySource
            f64 radius;                // sea level
            bool hasAtmosphere;
            bool hasTerrain;
            AtmosphereComponent atmosphere;
            planet::TerrainComponent terrain;
        };

        Config m_config;
        std::vector<Surface> m_surfaces; // scratch, rebuilt each tick
    };

    class RailsSystem final : public ecs::System
    {
    public:
        /// `timebase` is the lane whose per-tick "present" drives the
        /// analytic evaluation — the lane this system runs in. Using the
        /// master clock instead would put rails objects up to one step
        /// ahead of the integrated world (fatal once primaries move).
        explicit RailsSystem(const sim::SimulationLane& timebase)
            : m_timebase(timebase)
        {
        }

        [[nodiscard]] std::string_view name() const override { return "RailsSystem"; }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .write<TransformComponent>()
                .write<PreviousTransformComponent>()
                .read<OnRailsComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;

    private:
        const sim::SimulationLane& m_timebase;
    };

    class SimulationBubbleSystem final : public ecs::System
    {
    public:
        struct Config
        {
            f64 enterRadius = 1.0e4; // rails -> dynamic below this distance
            f64 exitRadius = 1.5e4;  // dynamic -> rails beyond this distance
        };

        /// `timebase`: the PHYSICS lane — conversions must produce state
        /// consistent with the transforms written by that lane's ticks.
        SimulationBubbleSystem(ecs::EntityCommandBuffer& commands,
                               const sim::SimulationLane& timebase, const Config& config);

        /// World-space focus of the bubble (typically the camera/player).
        void setFocus(const WorldVec3& focus) { m_focus = focus; }

        /// Time-warp mode: while true, NOTHING is truly simulated — every
        /// dynamic body (except gravity sources) is converted to rails and
        /// no rails object may enter the bubble. Analytic orbits are exact
        /// at any time scale; integration is not.
        void setForceRails(bool force) { m_forceRails = force; }
        [[nodiscard]] bool forceRails() const { return m_forceRails; }

        [[nodiscard]] std::string_view name() const override
        {
            return "SimulationBubbleSystem";
        }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            // Read-only over component data; conversions are deferred
            // through the command buffer.
            return ecs::SystemAccess{}
                .read<TransformComponent>()
                .read<OnRailsComponent>()
                .read<DynamicBodyComponent>()
                .read<SurfaceAnchorComponent>()
                .read<GravitySourceComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;

    private:
        /// Per-update snapshot of every gravity source, for SOI selection
        /// and frame-relative conversions.
        struct BodySnapshot
        {
            ecs::Entity entity{};
            WorldVec3 position{0.0};
            WorldVec3 velocity{0.0};
            f64 mu = 0.0;
            f64 soiRadius = 0.0;
        };

        /// The SOI primary of a world position: containing body with the
        /// smallest SOI (deepest in the hierarchy). Falls back to the
        /// largest mu if no SOI contains the point. nullptr when empty.
        [[nodiscard]] const BodySnapshot* selectPrimary(const WorldVec3& position) const;
        [[nodiscard]] const BodySnapshot* findBody(ecs::Entity entity) const;

        ecs::EntityCommandBuffer& m_commands;
        const sim::SimulationLane& m_timebase;
        Config m_config;
        WorldVec3 m_focus{0.0};
        bool m_forceRails = false;
        std::vector<BodySnapshot> m_bodies; // scratch, rebuilt each update
    };

    // ------------------------------------------------------------------------
    // HullCollisionSystem — you cannot walk through a refinery.
    //
    // Every solid thing carries its authored hitboxes as a HullComponent;
    // the ones that get pushed out of the others also carry a
    // HullMoverComponent. This runs late in the Physics lane, after motion
    // has already happened, and moves the movers back out — the standard
    // discrete order, and the one that keeps the walker's own controller
    // simple: it may step wherever it likes, and this puts it right.
    //
    // The broad phase is a bounding-sphere test on positions the entities
    // already carry, so a base of a hundred buildings costs a hundred f64
    // subtractions before anything expensive happens. The narrow phase is
    // box-against-box, and only ever for the handful within arm's reach.
    // ------------------------------------------------------------------------
    class HullCollisionSystem final : public ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override
        {
            return "HullCollisionSystem";
        }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            // POSITION ONLY. It does not write velocities on purpose — see
            // the comment in update(): a world velocity on a planet is
            // mostly carrier motion, and taking a bite out of it is a
            // kilometres-per-second impulse.
            return ecs::SystemAccess{}
                .write<TransformComponent>()
                .read<HullComponent>()
                .read<HullMoverComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;

        /// Solid things seen last tick — how much work the pair test did,
        /// for the HUD and for proving the broad phase is doing its job.
        [[nodiscard]] u32 lastHullCount() const { return m_hullCount; }
        [[nodiscard]] u32 lastNarrowPairs() const { return m_narrowPairs; }

    private:
        u32 m_hullCount = 0;
        u32 m_narrowPairs = 0;
    };
} // namespace sw::phys
