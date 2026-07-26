#include "Simulation/Simulation.hpp"

#include "Core/Assert.hpp"
#include "Core/Log.hpp"

#include <algorithm>
#include <cmath>

namespace sw::sim
{
    namespace
    {
        constexpr const char* kLogCat = "Simulation";
    } // namespace

    // ------------------------------------------------------------------------
    // SimulationLane
    // ------------------------------------------------------------------------
    SimulationLane::SimulationLane(const LaneConfig& config, const Simulation& owner)
        : m_config(config)
        , m_owner(&owner)
    {
        SW_ASSERT(config.frequencyHz > 0.0f, "Lane '{}' needs a positive frequency",
                  config.name);
        SW_ASSERT(config.maxTicksPerFrame > 0, "Lane '{}' needs maxTicksPerFrame >= 1",
                  config.name);
        m_stepSeconds = 1.0 / static_cast<f64>(m_config.frequencyHz);
    }

    f64 SimulationLane::remainingCapacitySeconds() const
    {
        if (m_config.bulkCatchUp)
        {
            return 1.0e30; // bulk lanes consume anything in one tick
        }
        const f64 capacity = static_cast<f64>(m_config.maxTicksPerFrame) * m_stepSeconds +
                             0.5 * m_stepSeconds - m_accumulator;
        // Never stall completely: one step per frame minimum keeps the
        // simulation moving even on absurdly slow frames.
        return std::max(capacity, m_stepSeconds);
    }

    f64 SimulationLane::presentSeconds() const
    {
        return m_owner->simulatedSeconds() - m_accumulator;
    }

    void SimulationLane::advance(f64 scaledDeltaSeconds, ecs::World& world, ThreadPool* pool,
                                 bool warnOnDrop)
    {
        m_accumulator += scaledDeltaSeconds;

        u32 ticksThisFrame = 0;
        const f32 fixedDt = static_cast<f32>(m_stepSeconds);
        while (m_accumulator >= m_stepSeconds && ticksThisFrame < m_config.maxTicksPerFrame)
        {
            // Consume the step BEFORE running: presentSeconds() must
            // report THIS tick's target time to the systems inside it
            // (analytic celestial evaluation depends on it).
            m_accumulator -= m_stepSeconds;
            ++m_tickCount;
            ++ticksThisFrame;
            m_scheduler.run(world, fixedDt, pool);
        }

        if (m_accumulator >= m_stepSeconds)
        {
            if (m_config.bulkCatchUp)
            {
                // Rate-based lane: consume the whole backlog in one big
                // tick — exact for amount = rate * dt systems, and the
                // mechanism that keeps factories honest under time warp.
                const f64 backlog = m_accumulator;
                m_accumulator = 0.0;
                ++m_tickCount;
                m_scheduler.run(world, static_cast<f32>(backlog), pool);
            }
            else
            {
                // Fixed-step lane: drop the backlog instead of spiraling.
                // The simulation lags wall clock but the app stays
                // responsive; log sparsely (first event, then every 64th).
                const f64 dropped =
                    m_accumulator - std::fmod(m_accumulator, m_stepSeconds);
                m_accumulator = std::fmod(m_accumulator, m_stepSeconds);
                if (warnOnDrop && (m_droppedTimeEvents++ & 63) == 0)
                {
                    SW_LOG_WARN(kLogCat,
                                "Lane '{}' dropped {:.3f}s of backlog (frame too slow for "
                                "{} Hz x{} catch-up ticks)",
                                m_config.name, dropped, m_config.frequencyHz,
                                m_config.maxTicksPerFrame);
                }
            }
        }
    }

    // ------------------------------------------------------------------------
    // Simulation
    // ------------------------------------------------------------------------
    Simulation::Simulation(std::vector<LaneConfig> laneConfigs)
    {
        SW_ASSERT(!laneConfigs.empty(), "Simulation needs at least one lane");
        m_lanes.reserve(laneConfigs.size());
        for (const LaneConfig& config : laneConfigs)
        {
            m_lanes.push_back(std::make_unique<SimulationLane>(config, *this));
            SW_LOG_INFO(kLogCat, "Lane '{}' at {} Hz (step {:.4f}s, catch-up {} ticks/frame)",
                        config.name, config.frequencyHz,
                        1.0 / static_cast<f64>(config.frequencyHz), config.maxTicksPerFrame);
        }
    }

    std::vector<LaneConfig> Simulation::defaultLanes()
    {
        // Physics: strict fixed step (numerical integration). 16 catch-up
        // ticks/frame covers PHYSICS WARP (full simulation up to x5 time
        // scale — drag, thrust and collisions stay live) at 20+ FPS. All
        // other lanes are rate-based and bulk-consume their backlog, which
        // keeps production/logistics/economy EXACT under extreme time warp.
        return {
            {"Physics", 50.0f, 16, false},
            {"Logistics", 10.0f, 4, true},
            {"Automation", 5.0f, 4, true},
            {"Economy", 2.0f, 2, true},
            {"World", 1.0f, 2, true},
        };
    }

    void Simulation::advance(ecs::World& world, f32 frameDeltaSeconds, ThreadPool* pool)
    {
        if (m_paused || frameDeltaSeconds <= 0.0f)
        {
            return;
        }

        f64 scaled = static_cast<f64>(frameDeltaSeconds) * m_timeScale;
        // STRICT lanes must never drop backlog (see the LaneConfig note):
        // the master clock slows to what they can actually consume.
        for (const std::unique_ptr<SimulationLane>& lane : m_lanes)
        {
            if (lane->strictCatchUp())
            {
                scaled = std::min(scaled, lane->remainingCapacitySeconds());
            }
        }
        m_simulatedSeconds += scaled;

        // Backlog drops are the EXPECTED regime during time warp; only warn
        // about them when running near real time (a genuine slow frame).
        const bool warnOnDrop = m_timeScale <= 2.0f;

        // Fixed lane order == configuration order: deterministic.
        for (const std::unique_ptr<SimulationLane>& lane : m_lanes)
        {
            lane->advance(scaled, world, pool, warnOnDrop);
        }
    }

    SimulationLane* Simulation::findLane(std::string_view name)
    {
        for (const std::unique_ptr<SimulationLane>& lane : m_lanes)
        {
            if (lane->name() == name)
            {
                return lane.get();
            }
        }
        return nullptr;
    }

    void Simulation::setTimeScale(f32 scale)
    {
        const f32 clamped = std::clamp(scale, 0.0f, 100000.0f);
        if (clamped != m_timeScale)
        {
            m_timeScale = clamped;
            SW_LOG_INFO(kLogCat, "Time scale set to x{:g}", m_timeScale);
        }
    }
} // namespace sw::sim
