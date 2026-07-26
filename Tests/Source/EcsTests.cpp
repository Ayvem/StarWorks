// ============================================================================
// EcsTests.cpp — unit tests for the archetype ECS.
// ============================================================================

#include "TestFramework.hpp"

#include <ECS/World.hpp>

#include <set>

namespace
{
    struct Position
    {
        float x = 0, y = 0, z = 0;
    };
    struct Velocity
    {
        float x = 0, y = 0, z = 0;
    };
    struct Health
    {
        int value = 0;
    };
} // namespace

using sw::ecs::Entity;
using sw::ecs::World;

SW_TEST(EntityCreateDestroyAndGenerations)
{
    World world;

    const Entity a = world.createEntity();
    const Entity b = world.createEntity();
    SW_CHECK(world.isAlive(a));
    SW_CHECK(world.isAlive(b));
    SW_CHECK(a.index != b.index);
    SW_CHECK_EQ(world.aliveCount(), 2u);

    world.destroyEntity(a);
    SW_CHECK(!world.isAlive(a));
    SW_CHECK_EQ(world.aliveCount(), 1u);

    // Index is recycled with a new generation; the stale handle stays dead.
    const Entity c = world.createEntity();
    SW_CHECK_EQ(c.index, a.index);
    SW_CHECK(c.generation != a.generation);
    SW_CHECK(!world.isAlive(a));
    SW_CHECK(world.isAlive(c));
}

SW_TEST(AddGetRemoveComponents)
{
    World world;
    const Entity e = world.createEntity();

    world.addComponent(e, Position{1, 2, 3});
    SW_CHECK(world.hasComponent<Position>(e));
    SW_CHECK(!world.hasComponent<Velocity>(e));
    SW_CHECK_EQ(world.getComponent<Position>(e).y, 2.0f);

    // Adding a second component migrates the row; values must survive.
    world.addComponent(e, Velocity{10, 20, 30});
    SW_CHECK(world.hasComponent<Position>(e));
    SW_CHECK_EQ(world.getComponent<Position>(e).z, 3.0f);
    SW_CHECK_EQ(world.getComponent<Velocity>(e).x, 10.0f);

    // Removal migrates back; the remaining component must survive.
    world.removeComponent<Velocity>(e);
    SW_CHECK(!world.hasComponent<Velocity>(e));
    SW_CHECK_EQ(world.getComponent<Position>(e).x, 1.0f);
    SW_CHECK(world.tryGetComponent<Velocity>(e) == nullptr);
}

SW_TEST(ComponentValuesSurviveSwapRemove)
{
    World world;

    // Three entities in the same archetype; destroy the middle one.
    const Entity e0 = world.createEntity();
    const Entity e1 = world.createEntity();
    const Entity e2 = world.createEntity();
    world.addComponent(e0, Health{100});
    world.addComponent(e1, Health{200});
    world.addComponent(e2, Health{300});

    world.destroyEntity(e1);

    SW_CHECK_EQ(world.getComponent<Health>(e0).value, 100);
    SW_CHECK_EQ(world.getComponent<Health>(e2).value, 300);
    SW_CHECK_EQ(world.count<Health>(), 2u);
}

SW_TEST(ForEachMatchesExactComponentSets)
{
    World world;

    const Entity posOnly = world.createEntity();
    world.addComponent(posOnly, Position{1, 0, 0});

    const Entity both = world.createEntity();
    world.addComponent(both, Position{2, 0, 0});
    world.addComponent(both, Velocity{5, 0, 0});

    const Entity all3 = world.createEntity();
    world.addComponent(all3, Position{3, 0, 0});
    world.addComponent(all3, Velocity{6, 0, 0});
    world.addComponent(all3, Health{50});

    // Query <Position>: all three.
    std::set<sw::u32> seen;
    world.forEach<Position>([&](Entity e, Position&) { seen.insert(e.index); });
    SW_CHECK_EQ(seen.size(), 3u);

    // Query <Position, Velocity>: two, and integration works via references.
    seen.clear();
    world.forEach<Position, Velocity>([&](Entity e, Position& p, Velocity& v) {
        seen.insert(e.index);
        p.x += v.x;
    });
    SW_CHECK_EQ(seen.size(), 2u);
    SW_CHECK(seen.contains(both.index) && seen.contains(all3.index));
    SW_CHECK_EQ(world.getComponent<Position>(both).x, 7.0f);
    SW_CHECK_EQ(world.getComponent<Position>(all3).x, 9.0f);
    SW_CHECK_EQ(world.getComponent<Position>(posOnly).x, 1.0f); // untouched

    SW_CHECK_EQ((world.count<Position, Velocity, Health>()), 1u);
}

SW_TEST(ManyEntitiesStressAndIntegrity)
{
    World world;
    constexpr int kCount = 10000;

    std::vector<Entity> entities;
    entities.reserve(kCount);
    for (int i = 0; i < kCount; ++i)
    {
        const Entity e = world.createEntity();
        world.addComponent(e, Health{i});
        if (i % 2 == 0)
        {
            world.addComponent(e, Position{static_cast<float>(i), 0, 0});
        }
        entities.push_back(e);
    }

    // Destroy every third entity, then verify the survivors are intact.
    for (int i = 0; i < kCount; i += 3)
    {
        world.destroyEntity(entities[static_cast<size_t>(i)]);
    }

    int survivors = 0;
    for (int i = 0; i < kCount; ++i)
    {
        const Entity e = entities[static_cast<size_t>(i)];
        if (i % 3 == 0)
        {
            SW_CHECK(!world.isAlive(e));
            continue;
        }
        ++survivors;
        if (world.getComponent<Health>(e).value != i)
        {
            SW_CHECK(false); // report once per corruption, not 10k checks
            break;
        }
    }
    SW_CHECK_EQ(world.aliveCount(), static_cast<sw::u32>(survivors));
}
