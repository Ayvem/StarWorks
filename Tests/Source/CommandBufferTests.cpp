// ============================================================================
// CommandBufferTests.cpp — unit tests for deferred ECS structural changes.
// ============================================================================

#include "TestFramework.hpp"

#include <Core/ThreadPool.hpp>
#include <ECS/CommandBuffer.hpp>
#include <ECS/World.hpp>

namespace
{
    struct Tag
    {
        int value = 0;
    };
    struct Extra
    {
        int value = 0;
    };
} // namespace

using sw::ecs::Entity;
using sw::ecs::EntityCommandBuffer;
using sw::ecs::World;

SW_TEST(CommandsAreDeferredUntilPlayback)
{
    World world;
    EntityCommandBuffer commands;

    const Entity e = world.createEntity();
    world.addComponent(e, Tag{7});

    // Record while "iterating" — nothing happens immediately.
    commands.add(e, Extra{42});
    commands.create([](World& w, Entity fresh) { w.addComponent(fresh, Tag{100}); });
    SW_CHECK(!world.hasComponent<Extra>(e));
    SW_CHECK_EQ(world.aliveCount(), 1u);
    SW_CHECK_EQ(commands.pendingCount(), 2u);

    commands.playback(world);
    SW_CHECK(world.hasComponent<Extra>(e));
    SW_CHECK_EQ(world.getComponent<Extra>(e).value, 42);
    SW_CHECK_EQ(world.aliveCount(), 2u);
    SW_CHECK_EQ(world.count<Tag>(), 2u);
    SW_CHECK_EQ(commands.pendingCount(), 0u); // cleared by playback
}

SW_TEST(CommandsOnDeadEntitiesAreSkipped)
{
    World world;
    EntityCommandBuffer commands;

    const Entity e = world.createEntity();
    world.addComponent(e, Tag{1});

    // Destroy first, then try to touch the same entity: later commands must
    // be skipped, not crash or resurrect anything.
    commands.destroy(e);
    commands.add(e, Extra{5});
    commands.remove<Tag>(e);
    commands.playback(world);

    SW_CHECK(!world.isAlive(e));
    SW_CHECK_EQ(world.aliveCount(), 0u);
}

SW_TEST(DestroyDuringIterationViaCommands)
{
    World world;
    EntityCommandBuffer commands;

    for (int i = 0; i < 10; ++i)
    {
        const Entity e = world.createEntity();
        world.addComponent(e, Tag{i});
    }

    // The canonical pattern: decide during iteration, mutate at playback.
    world.forEach<Tag>([&commands](Entity e, Tag& tag) {
        if (tag.value % 2 == 1)
        {
            commands.destroy(e);
        }
    });
    SW_CHECK_EQ(world.aliveCount(), 10u); // untouched during iteration

    commands.playback(world);
    SW_CHECK_EQ(world.aliveCount(), 5u);

    bool allEven = true;
    world.forEach<Tag>([&allEven](Entity, Tag& tag) { allEven &= (tag.value % 2 == 0); });
    SW_CHECK(allEven);
}

SW_TEST(RecordingIsThreadSafe)
{
    World world;
    EntityCommandBuffer commands;
    sw::ThreadPool pool(4);

    constexpr int kPerTask = 50;
    constexpr int kTasks = 8;
    for (int t = 0; t < kTasks; ++t)
    {
        pool.submit([&commands, t] {
            for (int i = 0; i < kPerTask; ++i)
            {
                const int value = t * kPerTask + i;
                commands.create(
                    [value](World& w, Entity fresh) { w.addComponent(fresh, Tag{value}); });
            }
        });
    }
    pool.waitIdle();
    SW_CHECK_EQ(commands.pendingCount(), static_cast<sw::usize>(kTasks * kPerTask));

    commands.playback(world);
    SW_CHECK_EQ(world.aliveCount(), static_cast<sw::u32>(kTasks * kPerTask));

    // Every recorded value must exist exactly once.
    std::vector<int> seen(kTasks * kPerTask, 0);
    world.forEach<Tag>([&seen](Entity, Tag& tag) { ++seen[static_cast<size_t>(tag.value)]; });
    bool allOnce = true;
    for (const int count : seen)
    {
        allOnce &= (count == 1);
    }
    SW_CHECK(allOnce);
}
