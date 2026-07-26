#pragma once

// ============================================================================
// Simulation/Simulation.hpp
// Multi-rate, fixed-timestep simulation core.
//
// The simulation is organized in LANES, each with its own fixed frequency
// and its own SystemScheduler:
//     Physics 50 Hz, Logistics 10 Hz, Automation 5 Hz, Economy 2 Hz,
//     World 1 Hz   (defaultLanes(); fully configurable).
//
// Guarantees:
//  - Rendering NEVER drives simulation: the frame loop feeds wall-clock time
//    into advance(); lanes tick zero or more times with their exact fixed
//    step. A fast renderer just interpolates more smoothly; a slow one
//    triggers catch-up ticks (bounded by maxTicksPerFrame to avoid the
//    spiral of death — excess time is dropped and logged).
//  - Determinism: lanes run in configuration order; inside a lane, systems
//    run through the conflict-free stage scheduler. Same inputs, same ticks,
//    same results — regardless of frame rate or thread timing.
//  - Pause / time scale: scaled at the entry point, so every lane stays
//    mutually consistent. Rendering keeps running while paused — the
//    simulation is simply not fed any time.
//
// Rendering reads simulation state via interpolation: lane.alpha() is the
// normalized progress toward the next tick (see the game's render collect).
// ============================================================================

#include "Core/Types.hpp"
#include "ECS/SystemScheduler.hpp"

#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace sw
{
    class ThreadPool;
} // namespace sw

namespace sw::ecs
{
    class World;
} // namespace sw::ecs

namespace sw::sim
{
    struct LaneConfig
    {
        std::string name;
        f32 frequencyHz = 50.0f;
        /// Catch-up bound per advance() call; excess backlog is dropped
        /// (with a warning) or bulk-processed, see bulkCatchUp.
        u32 maxTicksPerFrame = 8;
        /// If true, backlog beyond the tick bound is consumed in ONE extra
        /// tick whose dt is the whole remaining backlog. Correct for
        /// rate-based systems (production, transfers: amount = rate * dt),
        /// which therefore stay EXACT under extreme time warp. Never enable
        /// it for numerical integration lanes (Physics): integrators need
        /// their fixed step — that lane drops backlog instead, and time
        /// warp puts everything on rails anyway.
        bool bulkCatchUp = false;
    };

    // WHY strict catch-up exists (M21): dropping a fixed-step lane's
    // backlog silently desynchronizes it from the master clock — analytic
    // systems (celestial motion, rails) evaluate at presentSeconds() and
    // JUMP over the dropped interval while integrated bodies do not. For a
    // craft standing on a moving planet that mismatch is a battering ram:
    // the ground teleports into the hull every frame and launches it (the
    // launch-pad x5-warp fling on slow machines). While a lane is STRICT,
    // Simulation::advance() slows the MASTER clock down to what the lane
    // can actually consume — time stays globally consistent and the sim
    // simply runs below the requested time scale on weak hardware. The
    // game keeps Physics strict during physics warp (<= x5, integration
    // live) and relaxes it during rails warp (everything analytic: a drop
    // moves the whole world coherently, and 10000x needs the drops).

    class Simulation;

    class SimulationLane
    {
    public:
        SimulationLane(const LaneConfig& config, const Simulation& owner);

        /// See the strict catch-up note above LaneConfig.
        void setStrictCatchUp(bool strict) { m_strictCatchUp = strict; }
        [[nodiscard]] bool strictCatchUp() const { return m_strictCatchUp; }
        /// Scaled seconds this lane can still absorb THIS frame without
        /// dropping backlog (fixed-step lanes; bulk lanes are unbounded).
        [[nodiscard]] f64 remainingCapacitySeconds() const;

        SimulationLane(const SimulationLane&) = delete;
        SimulationLane& operator=(const SimulationLane&) = delete;

        [[nodiscard]] const std::string& name() const { return m_config.name; }
        [[nodiscard]] f32 frequencyHz() const { return m_config.frequencyHz; }
        [[nodiscard]] f32 stepSeconds() const { return m_stepSeconds; }
        [[nodiscard]] u64 tickCount() const { return m_tickCount; }

        /// Normalized [0,1) progress of the accumulator toward the next tick.
        /// Use it to interpolate render state between two simulation ticks.
        [[nodiscard]] f32 alpha() const
        {
            return static_cast<f32>(m_accumulator / m_stepSeconds);
        }

        /// THE TIME THIS LANE CONSIDERS "NOW": master simulated seconds
        /// minus the not-yet-ticked residue in the accumulator. Everything
        /// analytic that must stay consistent with this lane's per-tick
        /// state (celestial positions vs integrated bodies!) evaluates at
        /// THIS time, never at Simulation::simulatedSeconds() directly:
        /// the master clock runs up to a full step AHEAD of the lane
        /// within a frame — 600 m of planetary motion — and jumps further
        /// ahead whenever backlog is dropped. Called mid-tick, it returns
        /// exactly the current tick's target time; dropped backlog moves
        /// it forward consistently (the lane's world skips as one).
        [[nodiscard]] f64 presentSeconds() const;

        /// Systems of this lane (register at startup, before the first tick).
        [[nodiscard]] ecs::SystemScheduler& scheduler() { return m_scheduler; }

        // --- serialization support ------------------------------------------
        [[nodiscard]] f64 accumulatorSeconds() const { return m_accumulator; }
        void restoreState(u64 tickCount, f64 accumulatorSeconds)
        {
            m_tickCount = tickCount;
            m_accumulator = accumulatorSeconds;
        }

    private:
        friend class Simulation;

        /// Accumulates scaled time and runs 0..maxTicksPerFrame fixed steps.
        /// warnOnDrop silences the backlog warning when drops are expected
        /// (time warp).
        void advance(f64 scaledDeltaSeconds, ecs::World& world, ThreadPool* pool,
                     bool warnOnDrop);

        LaneConfig m_config;
        bool m_strictCatchUp = false;
        const Simulation* m_owner = nullptr;
        f64 m_stepSeconds = 0.0;
        f64 m_accumulator = 0.0;
        u64 m_tickCount = 0;
        u64 m_droppedTimeEvents = 0;
        ecs::SystemScheduler m_scheduler;
    };

    class Simulation
    {
    public:
        explicit Simulation(std::vector<LaneConfig> laneConfigs = defaultLanes());

        Simulation(const Simulation&) = delete;
        Simulation& operator=(const Simulation&) = delete;

        /// Physics 50 / Logistics 10 / Automation 5 / Economy 2 / World 1.
        [[nodiscard]] static std::vector<LaneConfig> defaultLanes();

        /// Feeds one frame of wall-clock time into every lane, in order.
        void advance(ecs::World& world, f32 frameDeltaSeconds, ThreadPool* pool);

        [[nodiscard]] usize laneCount() const { return m_lanes.size(); }
        [[nodiscard]] SimulationLane& lane(usize index) { return *m_lanes[index]; }
        /// nullptr if no lane has that name.
        [[nodiscard]] SimulationLane* findLane(std::string_view name);

        void setPaused(bool paused) { m_paused = paused; }
        [[nodiscard]] bool isPaused() const { return m_paused; }

        /// 1.0 = real time; clamped to [0, 100000] (time-warp range).
        /// At high scales, fixed-step lanes hit their catch-up bounds and
        /// drop backlog by design: analytic (on-rails) state follows
        /// simulatedSeconds() and stays exact; per-tick systems simply run
        /// at their real-time budget. Engage rails mode before warping.
        void setTimeScale(f32 scale);
        [[nodiscard]] f32 timeScale() const { return m_timeScale; }

        /// Total scaled seconds fed into the lanes (excludes paused frames).
        [[nodiscard]] f64 simulatedSeconds() const { return m_simulatedSeconds; }
        /// Serialization support: restores the master simulation clock.
        void setSimulatedSeconds(f64 seconds) { m_simulatedSeconds = seconds; }

    private:
        std::vector<std::unique_ptr<SimulationLane>> m_lanes;
        f64 m_simulatedSeconds = 0.0;
        f32 m_timeScale = 1.0f;
        bool m_paused = false;
    };
} // namespace sw::sim
