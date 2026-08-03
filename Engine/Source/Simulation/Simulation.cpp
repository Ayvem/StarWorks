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

    void SimulationLane::presentSecondsSplit(f64& outWhole, f64& outFraction) const
    {
        // The lane's present is the master clock minus its own un-ticked
        // residue, and the subtraction has to happen in the FRACTION so the
        // whole part stays an exact integer. The accumulator is under one
        // step — a fiftieth of a second on the physics lane — so the borrow
        // is at most one.
        outWhole = m_owner->wholeSeconds();
        outFraction = m_owner->fractionSeconds() - m_accumulator;
        const f64 borrow = std::floor(outFraction);
        outWhole += borrow;
        outFraction -= borrow;
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
        // Physics: strict fixed step (numerical integration). Sixteen
        // catch-up ticks/frame is the RESTING budget — enough for x5 at
        // 20 FPS, and small enough that one hitch frame at x1 cannot burn a
        // second of simulation. The game raises it (setMaxTicksPerFrame) for
        // whatever physics-warp rung the pilot selects and lowers it again
        // afterwards; see the note on that setter. All other lanes are
        // rate-based and bulk-consume their backlog, which keeps
        // production/logistics/economy EXACT under extreme time warp.
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
        // CARRIED, NOT ACCUMULATED INTO ONE DOUBLE. Adding 0.02 to a double
        // already holding 3e11 loses the 0.02 into the rounding; keeping the
        // whole seconds exact and the fraction separate keeps every tick's
        // time to seventeen digits however long the session has run. See
        // Simulation::wholeSeconds() for what that is worth in metres.
        m_fractionSeconds += scaled;
        const f64 carry = std::floor(m_fractionSeconds);
        m_wholeSeconds += carry;
        m_fractionSeconds -= carry;

        // WHAT THE HARDWARE ACTUALLY PAID. `scaled` has just been clamped to
        // what the strict lanes can integrate, so its ratio to the frame is
        // the time scale the world really ran at. Smoothed over roughly a
        // second (the frame-rate jitter underneath it is not information)
        // and only meaningful while running: a paused frame returns early.
        {
            const f64 delivered = scaled / static_cast<f64>(frameDeltaSeconds);
            const f32 smoothing =
                std::min(1.0f, frameDeltaSeconds / 1.0f); // ~1 s time constant
            m_achievedTimeScale +=
                (static_cast<f32>(delivered) - m_achievedTimeScale) * smoothing;
        }

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
        // The ceiling is INTERPLANETARY, not orbital. At x100 000 a Mars
        // transfer is still three real hours; the warp ladder now goes to
        // ten million so that it is a minute — and a limit that silently
        // ignored the top of the ladder is the worst kind of bug, the one
        // where the button works and nothing happens.
        //
        // What makes the number safe is that above physics warp nothing is
        // integrated: every orbit is analytic, and the rate-based lanes
        // bulk-consume whatever interval they are handed, exactly.
        //
        // AND THEN THE LADDER LEFT THE SYSTEM. Four light-years is 4.0e16
        // metres; at a hundred kilometres a second that is thirteen thousand
        // years, and ten million is still a fortnight of sitting there. A
        // billion makes the crossing seven minutes and ten billion makes it
        // forty seconds, which is the difference between a destination and a
        // number on a map. The ceiling here is the ladder's top rung and
        // nothing else — WHERE that rung may be selected is a separate
        // question, answered by maxWarpForSpace().
        const f32 clamped = std::clamp(scale, 0.0f, 1.0e10f);
        if (clamped != m_timeScale)
        {
            m_timeScale = clamped;
            SW_LOG_INFO(kLogCat, "Time scale set to x{:g}", m_timeScale);
        }
    }
} // namespace sw::sim
