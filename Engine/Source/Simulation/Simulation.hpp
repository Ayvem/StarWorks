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

#include <cmath>
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

        /// THE CATCH-UP BUDGET, RAISED ON DEMAND.
        ///
        /// A fixed-step lane's time scale ceiling is arithmetic: it can
        /// integrate `maxTicksPerFrame` steps per rendered frame and no
        /// more, so the fastest real-time multiple it can hold is
        /// ticks x frequency / fps. Sixteen 50 Hz ticks at 60 fps is x19 —
        /// which is why physics warp used to stop at x5.
        ///
        /// It is a KNOB rather than a constant because the two things it
        /// trades are wanted at different moments. A big budget buys a high
        /// physics warp; a small one bounds how much simulation a single
        /// hitch frame can burn at x1, which is the behaviour you want when
        /// nobody asked to fast-forward. So the game raises it for the warp
        /// rung it is on and lowers it again afterwards.
        ///
        /// Nothing here can desynchronise: with strict catch-up the master
        /// clock is already held to what the lane consumes, so a budget the
        /// machine cannot afford simply runs the world slower than asked.
        void setMaxTicksPerFrame(u32 ticks)
        {
            m_config.maxTicksPerFrame = (ticks > 0u) ? ticks : 1u;
        }
        [[nodiscard]] u32 maxTicksPerFrame() const { return m_config.maxTicksPerFrame; }
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

        /// The same instant, carried at full precision: exact whole seconds
        /// plus a fraction in [0, 1). Anything ANALYTIC — a Kepler orbit, a
        /// planet's spin — must evaluate at this and not at presentSeconds(),
        /// which cannot hold a fraction of a second once the session has run
        /// for an interstellar crossing. See Simulation::wholeSeconds().
        void presentSecondsSplit(f64& outWhole, f64& outFraction) const;

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

        /// THE TIME SCALE ACTUALLY DELIVERED, smoothed over about a second.
        ///
        /// `timeScale()` is what the pilot asked for; this is what the
        /// hardware paid. They differ whenever a strict lane clamps the
        /// master clock — which is the DESIGNED behaviour of physics warp on
        /// a machine that cannot integrate that fast, and which was until
        /// now completely invisible: the HUD said x100 while the world moved
        /// at forty. Reporting it is the difference between "warp is broken"
        /// and "this machine gives you x40 of the x100 you asked for".
        [[nodiscard]] f32 achievedTimeScale() const { return m_achievedTimeScale; }

        /// Total scaled seconds fed into the lanes (excludes paused frames).
        [[nodiscard]] f64 simulatedSeconds() const
        {
            return m_wholeSeconds + m_fractionSeconds;
        }
        /// THE CLOCK, SPLIT, AND THE REASON IT HAS TO BE.
        ///
        /// Everything analytic — every planet's position, every rail, every
        /// planet's rotation — is a function of ABSOLUTE simulated time, and
        /// an interstellar crossing costs about 3e11 seconds of it. A double
        /// at 3e11 has a step of 61 microseconds; Terra moves 29.8 km in a
        /// second, so each tick's time is snapped to a grid 1.8 m wide. That
        /// is not drift, which would be invisible — it is NOISE, different
        /// every tick, and it is exactly what a player standing on the ground
        /// after a trip to Proxima Centauri sees as the whole world
        /// vibrating. Measured: 0.10 m of it after three centuries of
        /// simulated time, 2.2 m after one crossing, 35 m after ten.
        ///
        /// So the clock is carried as an exact INTEGER second count plus a
        /// fraction in [0, 1). The whole part is exact to 2^53 seconds — two
        /// hundred and eighty-five million years — and the fraction keeps its
        /// full seventeen digits however large the whole part grows. Reducing
        /// the mean anomaly modulo the orbital period instead was measured
        /// first and is NOT enough: it fixes the anomaly's ulp and leaves the
        /// time's, which is the larger of the two (1.82 m of the 2.18 m).
        [[nodiscard]] f64 wholeSeconds() const { return m_wholeSeconds; }
        [[nodiscard]] f64 fractionSeconds() const { return m_fractionSeconds; }
        /// Serialization support: restores the master simulation clock.
        void setSimulatedSeconds(f64 seconds)
        {
            m_wholeSeconds = std::floor(seconds);
            m_fractionSeconds = seconds - m_wholeSeconds;
        }

    private:
        std::vector<std::unique_ptr<SimulationLane>> m_lanes;
        /// Exact integer seconds, and the fraction below one. See
        /// wholeSeconds() for why this is not one double.
        f64 m_wholeSeconds = 0.0;
        f64 m_fractionSeconds = 0.0;
        f32 m_timeScale = 1.0f;
        f32 m_achievedTimeScale = 1.0f;
        bool m_paused = false;
    };
} // namespace sw::sim
