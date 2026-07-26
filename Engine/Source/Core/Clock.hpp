#pragma once

// ============================================================================
// Core/Clock.hpp
// Frame timing built on std::chrono::steady_clock (monotonic).
//
// The clock produces the per-frame delta time used by variable-rate code
// (camera, animation). It deliberately caps very large deltas (debugger
// pauses, window drags) so downstream code never sees a multi-second step.
//
// The future multi-rate Simulation module (50/10/5/2/1 Hz) will build its
// fixed-step accumulators on top of this clock; rendering will never drive
// simulation directly.
// ============================================================================

#include "Core/Types.hpp"

#include <chrono>

namespace sw
{
    class Clock
    {
    public:
        Clock();

        /// Restarts the clock (total time and frame counter reset).
        void reset();

        /// Advances one frame; computes the new delta time.
        void tick();

        /// Seconds elapsed between the two most recent tick() calls (capped).
        [[nodiscard]] f32 deltaSeconds() const { return m_deltaSeconds; }

        /// Uncapped delta of the last frame — for profiling/diagnostics.
        [[nodiscard]] f64 rawDeltaSeconds() const { return m_rawDeltaSeconds; }

        /// Seconds since construction or reset().
        [[nodiscard]] f64 totalSeconds() const;

        /// Number of tick() calls since construction or reset().
        [[nodiscard]] u64 frameIndex() const { return m_frameIndex; }

        /// Exponentially smoothed frames-per-second estimate.
        [[nodiscard]] f32 smoothedFps() const { return m_smoothedFps; }

        /// Upper bound applied to deltaSeconds(). Default: 0.25 s.
        void setMaxDeltaSeconds(f32 seconds) { m_maxDeltaSeconds = seconds; }

    private:
        using SteadyClock = std::chrono::steady_clock;
        using TimePoint = SteadyClock::time_point;

        TimePoint m_startTime{};
        TimePoint m_lastTickTime{};
        f64 m_rawDeltaSeconds = 0.0;
        f32 m_deltaSeconds = 0.0f;
        f32 m_maxDeltaSeconds = 0.25f;
        f32 m_smoothedFps = 0.0f;
        u64 m_frameIndex = 0;
    };
} // namespace sw
