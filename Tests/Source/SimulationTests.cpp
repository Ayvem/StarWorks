// ============================================================================
// SimulationTests.cpp — unit tests for the multi-rate fixed-step simulation.
// Test lanes use power-of-two frequencies so every accumulator arithmetic
// step is exact in floating point and tick counts can be asserted exactly.
// ============================================================================

#include "TestFramework.hpp"

#include <ECS/World.hpp>
#include <Simulation/Simulation.hpp>

namespace
{
    struct TickProbe
    {
        int value = 0;
    };

    /// Counts its own updates and verifies it always receives the fixed step.
    class CountingSystem final : public sw::ecs::System
    {
    public:
        CountingSystem(sw::f32 expectedDt, int* counter, bool* dtAlwaysFixed)
            : m_expectedDt(expectedDt)
            , m_counter(counter)
            , m_dtAlwaysFixed(dtAlwaysFixed)
        {
        }

        [[nodiscard]] std::string_view name() const override { return "CountingSystem"; }
        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}.write<TickProbe>();
        }

        void update(sw::ecs::World&, sw::f32 deltaSeconds) override
        {
            ++(*m_counter);
            if (deltaSeconds != m_expectedDt)
            {
                *m_dtAlwaysFixed = false;
            }
        }

    private:
        sw::f32 m_expectedDt;
        int* m_counter;
        bool* m_dtAlwaysFixed;
    };
} // namespace

using sw::sim::LaneConfig;
using sw::sim::Simulation;

SW_TEST(LanesTickAtTheirExactFixedRates)
{
    sw::ecs::World world;
    Simulation simulation({{"fast", 32.0f, 64}, {"slow", 4.0f, 8}});

    int fastTicks = 0;
    int slowTicks = 0;
    bool fastDtFixed = true;
    bool slowDtFixed = true;
    simulation.findLane("fast")->scheduler().addSystem(
        std::make_unique<CountingSystem>(1.0f / 32.0f, &fastTicks, &fastDtFixed));
    simulation.findLane("slow")->scheduler().addSystem(
        std::make_unique<CountingSystem>(1.0f / 4.0f, &slowTicks, &slowDtFixed));

    // 1.0 second fed as 4 x 0.25 (exact in binary floating point).
    for (int i = 0; i < 4; ++i)
    {
        simulation.advance(world, 0.25f, nullptr);
    }

    SW_CHECK_EQ(fastTicks, 32);
    SW_CHECK_EQ(slowTicks, 4);
    SW_CHECK(fastDtFixed); // systems always see exactly 1/32 s
    SW_CHECK(slowDtFixed);
    SW_CHECK_EQ(simulation.findLane("fast")->tickCount(), 32ull);
}

SW_TEST(TickCountIsIndependentOfFrameSlicing)
{
    // The same total time, delivered in different frame sizes, must produce
    // the same number of ticks — rendering rate never affects simulation.
    auto runSliced = [](sw::f32 slice, int slices) {
        sw::ecs::World world;
        Simulation simulation({{"lane", 16.0f, 128}});
        int ticks = 0;
        bool fixed = true;
        simulation.findLane("lane")->scheduler().addSystem(
            std::make_unique<CountingSystem>(1.0f / 16.0f, &ticks, &fixed));
        for (int i = 0; i < slices; ++i)
        {
            simulation.advance(world, slice, nullptr);
        }
        return ticks;
    };

    const int coarse = runSliced(0.5f, 4);    // 2.0 s in 4 big frames
    const int fine = runSliced(0.03125f, 64); // 2.0 s in 64 small frames
    SW_CHECK_EQ(coarse, 32);
    SW_CHECK_EQ(fine, 32);
}

SW_TEST(CatchUpIsBoundedAndBacklogDropped)
{
    sw::ecs::World world;
    Simulation simulation({{"lane", 32.0f, 4}}); // at most 4 catch-up ticks

    int ticks = 0;
    bool fixed = true;
    simulation.findLane("lane")->scheduler().addSystem(
        std::make_unique<CountingSystem>(1.0f / 32.0f, &ticks, &fixed));

    // A 1-second hitch would mean 32 ticks; the bound caps it at 4 and the
    // rest of the backlog is dropped (accumulator < one step afterwards).
    simulation.advance(world, 1.0f, nullptr);
    SW_CHECK_EQ(ticks, 4);
    SW_CHECK(simulation.findLane("lane")->alpha() < 1.0f);

    // Next frame within the bound (0.125 s = exactly 4 ticks at 32 Hz)
    // keeps ticking normally.
    simulation.advance(world, 0.125f, nullptr);
    SW_CHECK_EQ(ticks, 8); // 4 + 4
}

SW_TEST(BulkCatchUpConsumesAllBacklogExactly)
{
    /// Sums every delta it is given — models rate-based (factory) systems.
    class TimeSumSystem final : public sw::ecs::System
    {
    public:
        explicit TimeSumSystem(sw::f32* sum) : m_sum(sum) {}
        [[nodiscard]] std::string_view name() const override { return "TimeSumSystem"; }
        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}.write<TickProbe>();
        }
        void update(sw::ecs::World&, sw::f32 deltaSeconds) override
        {
            *m_sum += deltaSeconds;
        }

    private:
        sw::f32* m_sum;
    };

    sw::ecs::World world;
    // Bulk lane: 4 bounded ticks + one catch-all tick per advance.
    Simulation simulation({{"lane", 32.0f, 4, true}});
    sw::f32 processedSeconds = 0.0f;
    simulation.findLane("lane")->scheduler().addSystem(
        std::make_unique<TimeSumSystem>(&processedSeconds));

    // A massive warp-style frame: 10 s in one advance. Fixed ticks cover
    // 4/32 s; the bulk tick must process the exact remainder — no time is
    // ever lost in a bulk lane.
    simulation.advance(world, 10.0f, nullptr);
    SW_CHECK_EQ(processedSeconds, 10.0f); // exact in binary fp

    simulation.advance(world, 0.125f, nullptr);
    SW_CHECK_EQ(processedSeconds, 10.125f);
}

SW_TEST(PauseAndTimeScale)
{
    sw::ecs::World world;
    Simulation simulation({{"lane", 32.0f, 64}});

    int ticks = 0;
    bool fixed = true;
    simulation.findLane("lane")->scheduler().addSystem(
        std::make_unique<CountingSystem>(1.0f / 32.0f, &ticks, &fixed));

    simulation.setPaused(true);
    simulation.advance(world, 10.0f, nullptr);
    SW_CHECK_EQ(ticks, 0); // paused: no time enters the lanes

    simulation.setPaused(false);
    simulation.setTimeScale(2.0f);
    simulation.advance(world, 0.5f, nullptr); // 0.5 s wall = 1.0 s simulated
    SW_CHECK_EQ(ticks, 32);
    SW_CHECK(fixed); // time scale changes tick COUNT, never tick SIZE
}

SW_TEST(DefaultLanesMatchTheDesign)
{
    Simulation simulation; // default config
    SW_CHECK_EQ(simulation.laneCount(), 5u);
    SW_CHECK(simulation.findLane("Physics") != nullptr);
    SW_CHECK_EQ(simulation.findLane("Physics")->frequencyHz(), 50.0f);
    SW_CHECK_EQ(simulation.findLane("Logistics")->frequencyHz(), 10.0f);
    SW_CHECK_EQ(simulation.findLane("Automation")->frequencyHz(), 5.0f);
    SW_CHECK_EQ(simulation.findLane("Economy")->frequencyHz(), 2.0f);
    SW_CHECK_EQ(simulation.findLane("World")->frequencyHz(), 1.0f);
}

// ============================================================================
// TEN MILLION TIMES REAL TIME.
//
// The two new warp rungs exist for one job: crossing to another planet
// without sitting through three real hours of it. What has to survive that
// is not the physics — above x5 nothing is integrated, every orbit is
// analytic — but the BOOKS. The rate-based lanes must still consume exactly
// the interval that passed, because a factory that loses an hour of its
// backlog at warp is a factory that quietly destroys matter.
//
// One rendered frame at x10 000 000 is about two days of simulated time.
// ============================================================================
SW_TEST(ExtremeWarpLosesNoTimeInTheRateBasedLanes)
{
    class TimeSumSystem final : public sw::ecs::System
    {
    public:
        explicit TimeSumSystem(sw::f64* sum) : m_sum(sum) {}
        [[nodiscard]] std::string_view name() const override { return "TimeSumSystem"; }
        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}.write<TickProbe>();
        }
        void update(sw::ecs::World&, sw::f32 deltaSeconds) override
        {
            *m_sum += static_cast<sw::f64>(deltaSeconds);
        }

    private:
        sw::f64* m_sum;
    };

    sw::ecs::World world;
    Simulation simulation; // the real lane set
    sw::f64 automationSeconds = 0.0;
    sw::f64 logisticsSeconds = 0.0;
    simulation.findLane("Automation")
        ->scheduler()
        .addSystem(std::make_unique<TimeSumSystem>(&automationSeconds));
    simulation.findLane("Logistics")
        ->scheduler()
        .addSystem(std::make_unique<TimeSumSystem>(&logisticsSeconds));

    // Rails warp: Physics is allowed to drop backlog (the whole world moves
    // analytically and coherently), the rate-based lanes are not.
    simulation.findLane("Physics")->setStrictCatchUp(false);
    simulation.setTimeScale(1.0e7f);

    constexpr sw::f32 kFrame = 1.0f / 60.0f;
    constexpr int kFrames = 120; // two seconds of wall clock
    for (int i = 0; i < kFrames; ++i)
    {
        simulation.advance(world, kFrame, nullptr);
    }

    const sw::f64 expected =
        static_cast<sw::f64>(kFrames) * static_cast<sw::f64>(kFrame) * 1.0e7;
    // 231 simulated DAYS in two seconds of wall clock, and both lanes have
    // seen every second of it.
    SW_CHECK(expected > 200.0 * 86400.0);
    SW_CHECK(std::abs(automationSeconds - expected) / expected < 1.0e-6);
    SW_CHECK(std::abs(logisticsSeconds - expected) / expected < 1.0e-6);
    // The master clock agrees: the simulated time IS the warped time.
    SW_CHECK(std::abs(simulation.simulatedSeconds() - expected) / expected < 1.0e-6);
}
