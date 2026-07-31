// ============================================================================
// SchedulerTests.cpp — unit tests for ThreadPool and SystemScheduler.
// ============================================================================

#include "TestFramework.hpp"

#include <Core/ThreadPool.hpp>
#include <ECS/SystemScheduler.hpp>
#include <ECS/World.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <string_view>

namespace
{
    struct CompA
    {
        int value = 0;
    };
    struct CompB
    {
        int value = 0;
    };

    /// Test system whose access set and body are injected.
    class TestSystem final : public sw::ecs::System
    {
    public:
        TestSystem(std::string_view name, sw::ecs::SystemAccess access,
                   std::function<void(sw::ecs::World&)> body)
            : m_name(name)
            , m_access(access)
            , m_body(std::move(body))
        {
        }

        [[nodiscard]] std::string_view name() const override { return m_name; }
        [[nodiscard]] sw::ecs::SystemAccess access() const override { return m_access; }
        void update(sw::ecs::World& world, sw::f32) override { m_body(world); }

    private:
        std::string_view m_name;
        sw::ecs::SystemAccess m_access;
        std::function<void(sw::ecs::World&)> m_body;
    };
} // namespace

using sw::ecs::SystemAccess;
using sw::ecs::SystemScheduler;
using sw::ecs::World;

SW_TEST(ThreadPoolRunsEveryTask)
{
    sw::ThreadPool pool(4);
    std::atomic<int> counter{0};

    constexpr int kTasks = 1000;
    for (int i = 0; i < kTasks; ++i)
    {
        pool.submit([&counter] { counter.fetch_add(1, std::memory_order_relaxed); });
    }
    pool.waitIdle();
    SW_CHECK_EQ(counter.load(), kTasks);
}

SW_TEST(SchedulerStagesRespectConflicts)
{
    SystemScheduler scheduler;

    // A writes CompA; B writes CompB (no conflict with A); C reads CompA
    // (conflicts with A's write -> must open a new stage).
    scheduler.addSystem(std::make_unique<TestSystem>(
        "WriteA", SystemAccess{}.write<CompA>(), [](World&) {}));
    scheduler.addSystem(std::make_unique<TestSystem>(
        "WriteB", SystemAccess{}.write<CompB>(), [](World&) {}));
    scheduler.addSystem(std::make_unique<TestSystem>(
        "ReadA", SystemAccess{}.read<CompA>(), [](World&) {}));

    SW_CHECK_EQ(scheduler.systemCount(), 3u);
    SW_CHECK_EQ(scheduler.stageCount(), 2u); // {WriteA, WriteB}, {ReadA}
}

SW_TEST(SchedulerParallelExecutionIsCorrect)
{
    World world;
    constexpr int kEntities = 2000;
    for (int i = 0; i < kEntities; ++i)
    {
        const auto e = world.createEntity();
        world.addComponent(e, CompA{1});
        world.addComponent(e, CompB{2});
    }

    // Two independent writers run in the same stage, in parallel; a third
    // system depending on both runs after them.
    SystemScheduler scheduler;
    scheduler.addSystem(std::make_unique<TestSystem>(
        "IncA", SystemAccess{}.write<CompA>(), [](World& w) {
            w.forEach<CompA>([](sw::ecs::Entity, CompA& a) { a.value += 10; });
        }));
    scheduler.addSystem(std::make_unique<TestSystem>(
        "IncB", SystemAccess{}.write<CompB>(), [](World& w) {
            w.forEach<CompB>([](sw::ecs::Entity, CompB& b) { b.value += 100; });
        }));
    scheduler.addSystem(std::make_unique<TestSystem>(
        "SumAB", SystemAccess{}.read<CompA>().read<CompB>().write<CompA>(),
        [](World& w) {
            w.forEach<CompA, CompB>(
                [](sw::ecs::Entity, CompA& a, CompB& b) { a.value += b.value; });
        }));

    SW_CHECK_EQ(scheduler.stageCount(), 2u);

    sw::ThreadPool pool(4);
    for (int tick = 0; tick < 3; ++tick)
    {
        scheduler.run(world, 0.016f, &pool);
    }

    // Per tick: a = a + 10 + (b + 100); deterministic regardless of threads.
    int expectedA = 1, expectedB = 2;
    for (int tick = 0; tick < 3; ++tick)
    {
        expectedB += 100;
        expectedA += 10 + expectedB;
    }

    bool allCorrect = true;
    world.forEach<CompA, CompB>([&](sw::ecs::Entity, CompA& a, CompB& b) {
        allCorrect = allCorrect && a.value == expectedA && b.value == expectedB;
    });
    SW_CHECK(allCorrect);
    SW_CHECK_EQ(world.count<CompA>(), static_cast<sw::u32>(kEntities));
}
