#include "ECS/CommandBuffer.hpp"

namespace sw::ecs
{
    void EntityCommandBuffer::create(std::function<void(World&, Entity)> init)
    {
        enqueue([init = std::move(init)](World& world) {
            const Entity entity = world.createEntity();
            if (init)
            {
                init(world, entity);
            }
        });
    }

    void EntityCommandBuffer::destroy(Entity entity)
    {
        enqueue([entity](World& world) {
            if (world.isAlive(entity))
            {
                world.destroyEntity(entity);
            }
        });
    }

    void EntityCommandBuffer::playback(World& world)
    {
        // Swap out under the lock so recording can resume immediately and
        // playback itself runs without holding the mutex.
        std::vector<std::function<void(World&)>> commands;
        {
            std::scoped_lock lock(m_mutex);
            commands.swap(m_commands);
        }
        for (const std::function<void(World&)>& command : commands)
        {
            command(world);
        }
    }

    usize EntityCommandBuffer::pendingCount() const
    {
        std::scoped_lock lock(m_mutex);
        return m_commands.size();
    }

    void EntityCommandBuffer::enqueue(std::function<void(World&)> command)
    {
        std::scoped_lock lock(m_mutex);
        m_commands.push_back(std::move(command));
    }
} // namespace sw::ecs
