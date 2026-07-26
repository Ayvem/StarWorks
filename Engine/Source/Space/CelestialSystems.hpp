#pragma once

// ============================================================================
// Space/CelestialSystems.hpp
//
//  CelestialMotionSystem (Physics lane, 50 Hz — FIRST system of the lane)
//      Moves every celestial body along its parent-relative Kepler orbit:
//      world position = recursive analytic evaluation up to the root star,
//      always at the CURRENT master simulation time (so celestial state is
//      exact at any time warp, exactly like generic rails objects).
//
//      Unlike the rails tier, celestials get REAL interpolation (previous =
//      position at the previous tick): a planet moves ~30 km/s, and the
//      terrain under a landed ship must glide, not step, between frames.
//
//      Also stamps GravitySource::worldVelocity — the body-relative frame
//      that atmosphere drag, ground friction and orbit conversions measure
//      against.
// ============================================================================

#include "ECS/System.hpp"
#include "Physics/PhysicsComponents.hpp"
#include "Scene/TransformComponents.hpp"
#include "Space/CelestialIndex.hpp"

namespace sw::sim
{
    class SimulationLane;
} // namespace sw::sim

namespace sw::space
{
    class CelestialMotionSystem final : public ecs::System
    {
    public:
        /// `timebase`: the lane this system runs in (Physics). Celestial
        /// positions MUST be evaluated at the lane's per-tick present, not
        /// the master clock — otherwise the planets run up to one step
        /// ahead of every integrated body, and the ground itself jitters
        /// under anything landed on it.
        explicit CelestialMotionSystem(const sim::SimulationLane& timebase)
            : m_timebase(timebase)
        {
        }

        [[nodiscard]] std::string_view name() const override
        {
            return "CelestialMotionSystem";
        }

        [[nodiscard]] ecs::SystemAccess access() const override
        {
            return ecs::SystemAccess{}
                .write<TransformComponent>()
                .write<PreviousTransformComponent>()
                .write<phys::GravitySourceComponent>()
                .read<CelestialBodyComponent>();
        }

        void update(ecs::World& world, f32 deltaSeconds) override;

    private:
        const sim::SimulationLane& m_timebase;
        CelestialIndex m_index; // rebuilt every tick (bodies are few)
    };
} // namespace sw::space
