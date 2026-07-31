// ============================================================================
// SimulationTests.cpp — unit tests for the multi-rate fixed-step simulation.
// Wherever a tick count or a clock value is asserted EXACTLY, the lane runs
// at a power-of-two frequency and the frames are power-of-two seconds, so the
// accumulator arithmetic is exact in binary floating point and no assertion
// rides on a rounding residue. The one test that uses the SHIPPED lane set
// (50 / 10 / 5 / 2 / 1 Hz, fed 1/60 s frames — none of which is representable)
// asserts relative tolerances only, never equality.
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

// ============================================================================
// STRICT CATCH-UP — the M21 launch-pad fling.
//
// A craft standing on a moving planet is held there by two different clocks:
// the ground is ANALYTIC (evaluated at lane.presentSeconds()) and the hull is
// INTEGRATED (advanced one fixed step per tick). Those two agree only as long
// as presentSeconds() moves at exactly the rate the lane actually integrates.
// A fixed-step lane that drops backlog breaks that: the master clock — and
// with it presentSeconds() — jumps over the dropped interval while the hull
// does not, so the terrain teleports INTO the hull and the contact solver
// throws the craft off the pad. That was the x5-warp fling on slow machines.
//
// strictCatchUp is the fix: while a lane is strict, Simulation::advance()
// clamps the master clock to what that lane can actually consume this frame
// (SimulationLane::remainingCapacitySeconds()). Nothing is ever dropped, so
// the analytic clock can never outrun the integration; the price is that the
// simulation runs BELOW the requested time scale on hardware that cannot keep
// up, which is exactly the trade the game wants during physics warp.
//
// The tests below pin that guarantee from the outside — tick bound, clock
// lock, mode difference and exact time accounting — so it cannot regress into
// "the code happens to still compile".
//
// The lanes used here are power-of-two frequencies (32 Hz / 4 ticks, step
// 0.031250 s, unless a test says otherwise), so every number in this section
// is exact in binary floating point — no assertion depends on which side of
// zero a rounding residue lands. All figures below were MEASURED against
// this build.
// ============================================================================
namespace
{
    /// Records what a lane actually ran: how many ticks, and the exact total
    /// of the dt values its systems were handed.
    class TickRecorder final : public sw::ecs::System
    {
    public:
        [[nodiscard]] std::string_view name() const override { return "TickRecorder"; }
        [[nodiscard]] sw::ecs::SystemAccess access() const override
        {
            return sw::ecs::SystemAccess{}.write<TickProbe>();
        }

        void update(sw::ecs::World&, sw::f32 deltaSeconds) override
        {
            ++ticks;
            integratedSeconds += static_cast<sw::f64>(deltaSeconds);
        }

        int ticks = 0;
        sw::f64 integratedSeconds = 0.0;
    };

    constexpr sw::f32 kStrictHz = 32.0f;
    constexpr sw::u32 kStrictMaxTicks = 4;
    constexpr sw::f64 kStrictStep = 1.0 / 32.0; // 0.031250 s, exact

    /// One 32 Hz / 4-tick lane with a recorder attached, strict or not.
    TickRecorder* attachRecorder(Simulation& simulation, bool strict)
    {
        auto recorder = std::make_unique<TickRecorder>();
        TickRecorder* raw = recorder.get();
        sw::sim::SimulationLane* lane = simulation.findLane("lane");
        lane->setStrictCatchUp(strict);
        lane->scheduler().addSystem(std::move(recorder));
        return raw;
    }
} // namespace

SW_TEST(StrictCatchUpBoundsTheBurstOnAHitchFrame)
{
    sw::ecs::World world;
    Simulation simulation({{"lane", kStrictHz, kStrictMaxTicks, false}});
    TickRecorder* recorder = attachRecorder(simulation, true);
    sw::sim::SimulationLane* lane = simulation.findLane("lane");

    // A 100-second frame. Unbounded catch-up would be 3200 ticks; the bound
    // the strict lane actually honours is maxTicksPerFrame, and the master
    // clock is held to what those ticks cover. MEASURED: 4 ticks, master
    // clock +0.140625 s = (4 + 0.5) steps — the extra half step is the slack
    // remainingCapacitySeconds() leaves so alpha() keeps a sane interpolation
    // residue instead of snapping to 0 every hitch.
    simulation.advance(world, 100.0f, nullptr);
    SW_CHECK_EQ(recorder->ticks, 4);
    SW_CHECK_EQ(simulation.simulatedSeconds(), 0.140625);
    SW_CHECK_EQ(lane->accumulatorSeconds(), 0.015625); // half a step
    SW_CHECK_EQ(lane->alpha(), 0.5f);

    // And it holds frame after frame: a machine that is permanently 800x too
    // slow does not accumulate a debt that later explodes into one giant
    // burst. MEASURED steady state: exactly 4 ticks and +0.125 s of master
    // clock per frame, i.e. the sim runs at the lane's real-time budget.
    for (int frame = 0; frame < 10; ++frame)
    {
        const int before = recorder->ticks;
        const sw::f64 clockBefore = simulation.simulatedSeconds();
        simulation.advance(world, 100.0f, nullptr);
        SW_CHECK_EQ(recorder->ticks - before, 4);
        SW_CHECK_EQ(simulation.simulatedSeconds() - clockBefore, 0.125);
    }
    SW_CHECK_EQ(recorder->ticks, 44);              // 4 + 10 x 4
    SW_CHECK_EQ(simulation.simulatedSeconds(), 1.390625); // 0.140625 + 10 x 0.125
}

SW_TEST(StrictCatchUpKeepsTheAnalyticClockLockedToTheIntegration)
{
    // THE fling invariant, stated without reference to the implementation:
    // over any frame, the time the lane says it is AT (presentSeconds(), the
    // clock every analytic body is evaluated at) advances by exactly the time
    // the lane INTEGRATED — one fixed step per tick, no more. If this ever
    // fails, the ground has moved further than the craft standing on it.
    const sw::f32 frameDeltas[] = {0.0078125f, 0.25f, 1.0f, 0.03125f,
                                   100.0f,     0.5f,  10.0f};

    sw::ecs::World world;
    Simulation strictSim({{"lane", kStrictHz, kStrictMaxTicks, false}});
    TickRecorder* strictRec = attachRecorder(strictSim, true);
    sw::sim::SimulationLane* strictLane = strictSim.findLane("lane");

    for (sw::f32 delta : frameDeltas)
    {
        const int before = strictRec->ticks;
        const sw::f64 presentBefore = strictLane->presentSeconds();
        strictSim.advance(world, delta, nullptr);
        const sw::f64 integrated = static_cast<sw::f64>(strictRec->ticks - before) * kStrictStep;
        // MEASURED: exact for every delta above, including the sub-step
        // frames (0 ticks, present does not move at all) and the 100 s hitch.
        SW_CHECK_EQ(strictLane->presentSeconds() - presentBefore, integrated);
    }

    // Same lane, strictness lifted, same 100 s hitch: the analytic clock runs
    // 100 s while the lane integrates 4 steps = 0.125 s. MEASURED overrun
    // factor: 800x. That gap IS the battering ram — 100 s of planetary
    // rotation delivered to a hull that only moved an eighth of a second.
    Simulation looseSim({{"lane", kStrictHz, kStrictMaxTicks, false}});
    TickRecorder* looseRec = attachRecorder(looseSim, false);
    sw::sim::SimulationLane* looseLane = looseSim.findLane("lane");
    looseSim.advance(world, 100.0f, nullptr);
    const sw::f64 looseIntegrated = static_cast<sw::f64>(looseRec->ticks) * kStrictStep;
    SW_CHECK_EQ(looseIntegrated, 0.125);
    SW_CHECK_EQ(looseLane->presentSeconds(), 100.0);
    SW_CHECK(looseLane->presentSeconds() / looseIntegrated >= 800.0);
}

SW_TEST(RailsWarpLiftsStrictnessAndTheMasterClockDoesCatchUp)
{
    // The other half of the contract: lifting strictness is a FEATURE, not
    // an accident. On rails everything is analytic, so a dropped interval
    // moves the whole world coherently and the master clock must be allowed
    // to keep up — that is what makes x10000 warp arrive anywhere. The two
    // modes are fed the identical frame here so the difference is the mode
    // and nothing else.
    auto runOneHugeFrame = [](bool strict) {
        sw::ecs::World world;
        Simulation simulation({{"lane", kStrictHz, kStrictMaxTicks, false}});
        TickRecorder* recorder = attachRecorder(simulation, strict);
        simulation.advance(world, 100.0f, nullptr);
        return std::pair<sw::f64, int>{simulation.simulatedSeconds(), recorder->ticks};
    };

    const auto [strictClock, strictTicks] = runOneHugeFrame(true);
    const auto [looseClock, looseTicks] = runOneHugeFrame(false);

    // MEASURED: strict 0.140625 s of simulated time, loose the full 100 s —
    // a factor of 711. The tick bound is the lane's own maxTicksPerFrame in
    // both modes; what strictness buys is that the clock waits for it.
    SW_CHECK_EQ(strictClock, 0.140625);
    SW_CHECK_EQ(looseClock, 100.0);
    SW_CHECK(looseClock / strictClock > 700.0);
    SW_CHECK_EQ(strictTicks, 4);
    SW_CHECK_EQ(looseTicks, 4);
}

SW_TEST(StrictCatchUpIsInertWhileTheMachineKeepsUpAndBitesTheMomentItDoesNot)
{
    // Two halves of one claim, on ONE lane, because the first half alone is
    // no guard: "strict == loose while the frame fits" stays true if the
    // clamp is deleted outright. The second half is what pins the clamp — the
    // very next frame, one that does NOT fit, must separate the two modes.
    //
    // Lane: 64 Hz / 16 catch-up ticks. Step 1/64 s, budget 16 steps = 0.25 s
    // per frame, and every quantity below is a power of two — nothing here
    // depends on a 1e-16 accumulator residue staying on the positive side of
    // zero (a 50 Hz lane, whose 0.02 s step is NOT representable, cannot be
    // asserted this way).
    struct Run
    {
        sw::f64 inBudgetClock;
        int inBudgetTicks;
        sw::f64 overBudgetClock;
        int overBudgetTicks;
    };

    auto run = [](bool strict) {
        sw::ecs::World world;
        Simulation simulation({{"lane", 64.0f, 16, false}});
        auto recorder = std::make_unique<TickRecorder>();
        TickRecorder* raw = recorder.get();
        sw::sim::SimulationLane* lane = simulation.findLane("lane");
        lane->setStrictCatchUp(strict);
        lane->scheduler().addSystem(std::move(recorder));

        // Eight frames of exactly the budget: the machine keeps up.
        for (int frame = 0; frame < 8; ++frame)
        {
            simulation.advance(world, 0.25f, nullptr);
        }
        Run result{simulation.simulatedSeconds(), raw->ticks, 0.0, 0};

        // One frame the machine cannot keep up with: 1.0 s, four budgets.
        const int ticksBefore = raw->ticks;
        simulation.advance(world, 1.0f, nullptr);
        result.overBudgetClock = simulation.simulatedSeconds();
        result.overBudgetTicks = raw->ticks - ticksBefore;
        return result;
    };

    const Run strict = run(true);
    const Run loose = run(false);

    // Inert inside the budget. MEASURED: both modes 128 ticks and exactly
    // 2.0 s of simulated time — bit for bit identical, so strictness costs
    // the player whose machine keeps up nothing at all.
    SW_CHECK_EQ(strict.inBudgetTicks, 128);
    SW_CHECK_EQ(loose.inBudgetTicks, 128);
    SW_CHECK_EQ(strict.inBudgetClock, 2.0);
    SW_CHECK_EQ(loose.inBudgetClock, 2.0);

    // And decidedly NOT inert outside it. Both modes run the same 16 ticks
    // (the bound is maxTicksPerFrame either way); what differs is the clock.
    // MEASURED: strict advances 0.2578125 s = 16.5 steps — the ticks it can
    // actually cover plus the half-step interpolation slack — while loose
    // takes the whole 1.0 s and drops the 0.75 s it could not integrate.
    SW_CHECK_EQ(strict.overBudgetTicks, 16);
    SW_CHECK_EQ(loose.overBudgetTicks, 16);
    SW_CHECK_EQ(strict.overBudgetClock, 2.2578125);
    SW_CHECK_EQ(loose.overBudgetClock, 3.0);
    SW_CHECK(strict.overBudgetClock < loose.overBudgetClock);
}

SW_TEST(NeitherModeLosesNorDoubleCountsASingleTick)
{
    // Accounting, both modes. Every simulated second must be either
    // integrated by a tick, still sitting in the accumulator, or dropped as
    // a WHOLE number of steps. A fractional remainder here would mean the
    // lane silently gained or lost a fraction of a tick, which is precisely
    // the desynchronization strictness exists to prevent — and half a step
    // of drift per frame is invisible in a screenshot and fatal on a pad.
    const sw::f32 frameDeltas[] = {0.0078125f, 0.25f, 1.0f, 0.03125f,
                                   100.0f,     0.5f,  10.0f};

    for (int mode = 0; mode < 2; ++mode)
    {
        const bool strict = (mode == 0);
        sw::ecs::World world;
        Simulation simulation({{"lane", kStrictHz, kStrictMaxTicks, false}});
        TickRecorder* recorder = attachRecorder(simulation, strict);
        sw::sim::SimulationLane* lane = simulation.findLane("lane");

        for (sw::f32 delta : frameDeltas)
        {
            simulation.advance(world, delta, nullptr);
        }

        // No double counting: the scheduler ran exactly tickCount() times and
        // every one of them was handed the fixed step, never a bulk dt.
        SW_CHECK_EQ(static_cast<sw::u64>(recorder->ticks), lane->tickCount());
        SW_CHECK_EQ(recorder->integratedSeconds,
                    static_cast<sw::f64>(recorder->ticks) * kStrictStep);

        const sw::f64 dropped = simulation.simulatedSeconds() -
                                recorder->integratedSeconds - lane->accumulatorSeconds();
        SW_CHECK(dropped >= 0.0); // time is never invented
        const sw::f64 droppedSteps = dropped / kStrictStep;
        // Whole steps only, in both modes. MEASURED: strict drops 0 steps
        // (clock total 0.671875 s, all of it ticked or pending); loose drops
        // 3556 steps — 111.125 s — out of a 111.789063 s clock, while both
        // modes ran the identical 21 ticks.
        SW_CHECK_EQ(droppedSteps, static_cast<sw::f64>(static_cast<sw::u64>(droppedSteps + 0.5)));

        if (strict)
        {
            // The whole point: a strict lane NEVER drops. Its accounted time
            // is the master clock, exactly.
            SW_CHECK_EQ(dropped, 0.0);
            SW_CHECK_EQ(recorder->integratedSeconds + lane->accumulatorSeconds(),
                        simulation.simulatedSeconds());
            SW_CHECK_EQ(simulation.simulatedSeconds(), 0.671875);
        }
        else
        {
            SW_CHECK(droppedSteps > 3000.0);
        }
    }
}

SW_TEST(TheCapacityFloorKeepsAStarvedStrictLaneMovingAFullStepPerFrame)
{
    // remainingCapacitySeconds() floors what it hands back at one step
    // (std::max(capacity, m_stepSeconds)): a strict lane must never talk the
    // master clock into advancing LESS than the one tick it is about to run,
    // or an already-struggling machine would also see its clock creep, which
    // on a launch pad reads as the world stuttering rather than slowing.
    //
    // The floor only binds when the residue is large relative to the budget,
    // i.e. accumulator > (maxTicksPerFrame - 0.5) steps — with a 1-tick lane
    // that means any residue above half a step. Seeding 3/4 of a step (32 Hz,
    // 0.0234375 s: no tick, pure residue) puts the lane exactly there:
    // raw capacity would be 1.5 - 0.75 = 0.75 steps, the floor lifts it to 1.
    sw::ecs::World world;
    Simulation simulation({{"lane", kStrictHz, 1, false}});
    TickRecorder* recorder = attachRecorder(simulation, true);
    sw::sim::SimulationLane* lane = simulation.findLane("lane");

    simulation.advance(world, 0.0234375f, nullptr); // 3/4 step: no tick yet
    SW_CHECK_EQ(recorder->ticks, 0);
    SW_CHECK_EQ(lane->accumulatorSeconds(), 0.75 * kStrictStep);
    SW_CHECK_EQ(lane->remainingCapacitySeconds(), kStrictStep); // floored

    // Five 100-second hitches on the starved lane. MEASURED: one tick and
    // exactly one step (0.03125 s) of master clock per frame, with the 3/4
    // step residue preserved — a fixed point, not a slow bleed. Without the
    // floor the first frame would only buy 0.75 of a step of clock and eat
    // the residue down to half a step.
    for (int frame = 0; frame < 5; ++frame)
    {
        const int before = recorder->ticks;
        const sw::f64 clockBefore = simulation.simulatedSeconds();
        simulation.advance(world, 100.0f, nullptr);
        SW_CHECK_EQ(recorder->ticks - before, 1);
        SW_CHECK_EQ(simulation.simulatedSeconds() - clockBefore, kStrictStep);
        SW_CHECK_EQ(lane->accumulatorSeconds(), 0.75 * kStrictStep);
    }
    // Strict is still strict: seed + 5 steps of clock, nothing dropped.
    SW_CHECK_EQ(simulation.simulatedSeconds(), 0.1796875);
    SW_CHECK_EQ(recorder->integratedSeconds, 5.0 * kStrictStep);
    SW_CHECK_EQ(recorder->integratedSeconds + lane->accumulatorSeconds(),
                simulation.simulatedSeconds());

    // The floor is a FLOOR, not the capacity: give the same lane two ticks
    // per frame and the real capacity (2.5 - 0.75 = 1.75 steps on the first
    // hitch, then 2 steps once the residue settles at half a step) is what
    // the clock follows. MEASURED: 2 ticks/frame, +0.0546875 s then
    // +0.0625 s of clock.
    Simulation twoTick({{"lane", kStrictHz, 2, false}});
    TickRecorder* twoRec = attachRecorder(twoTick, true);
    twoTick.advance(world, 0.0234375f, nullptr);
    const sw::f64 seedClock = twoTick.simulatedSeconds();
    twoTick.advance(world, 100.0f, nullptr);
    SW_CHECK_EQ(twoRec->ticks, 2);
    SW_CHECK_EQ(twoTick.simulatedSeconds() - seedClock, 1.75 * kStrictStep);
    const sw::f64 afterFirstHitch = twoTick.simulatedSeconds();
    twoTick.advance(world, 100.0f, nullptr);
    SW_CHECK_EQ(twoRec->ticks, 4);
    SW_CHECK_EQ(twoTick.simulatedSeconds() - afterFirstHitch, 2.0 * kStrictStep);
}
