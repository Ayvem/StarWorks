#include "Core/Clock.hpp"

#include <algorithm>

namespace sw
{
    Clock::Clock()
    {
        reset();
    }

    void Clock::reset()
    {
        m_startTime = SteadyClock::now();
        m_lastTickTime = m_startTime;
        m_rawDeltaSeconds = 0.0;
        m_deltaSeconds = 0.0f;
        m_smoothedFps = 0.0f;
        m_frameIndex = 0;
    }

    void Clock::tick()
    {
        const TimePoint now = SteadyClock::now();
        const std::chrono::duration<f64> delta = now - m_lastTickTime;
        m_lastTickTime = now;

        m_rawDeltaSeconds = delta.count();
        m_deltaSeconds = std::min(static_cast<f32>(m_rawDeltaSeconds), m_maxDeltaSeconds);
        ++m_frameIndex;

        // The very first delta after reset() measures loop startup, not a
        // frame (microseconds -> absurd instant FPS). Seeding the moving
        // average with it poisons the display for hundreds of frames, so
        // frame 1 never contributes.
        if (m_rawDeltaSeconds > 0.0 && m_frameIndex >= 2)
        {
            const f32 instantFps = static_cast<f32>(1.0 / m_rawDeltaSeconds);
            // Exponential moving average; alpha tuned for a readable display.
            constexpr f32 kAlpha = 0.05f;
            m_smoothedFps = (m_smoothedFps == 0.0f)
                                ? instantFps
                                : m_smoothedFps + kAlpha * (instantFps - m_smoothedFps);
        }
    }

    f64 Clock::totalSeconds() const
    {
        const std::chrono::duration<f64> total = SteadyClock::now() - m_startTime;
        return total.count();
    }
} // namespace sw
