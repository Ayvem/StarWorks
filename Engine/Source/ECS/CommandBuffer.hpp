#pragma once

// ============================================================================
// ECS/CommandBuffer.hpp
// Deferred structural changes for the ECS.
//
// Systems run in parallel and therefore must NOT create/destroy entities or
// add/remove components directly (structural changes invalidate the columns
// other threads are iterating). Instead they record commands into an
// EntityCommandBuffer — recording is thread-safe — and the owning thread
// calls playback() at a well-defined point (typically right after a
// scheduler stage or a simulation tick).
//
// Semantics:
//  - Commands execute in recording order.
//  - Commands targeting an entity that died before playback (e.g. destroyed
//    by an earlier command) are skipped silently — this is the standard,
//    data-race-free interpretation of deferred destruction.
//
// The std::function storage is simple and correct; a flat POD command queue
// can replace it later without touching any call site.
// ============================================================================

#include "ECS/Entity.hpp"
#include "ECS/World.hpp"

#include <functional>
#include <mutex>
#include <vector>

namespace sw::ecs
{
    class EntityCommandBuffer
    {
    public:
        /// Deferred World::createEntity(); `init` runs at playback with the
        /// fresh entity (add components there).
        void create(std::function<void(World&, Entity)> init);

        void destroy(Entity entity);

        template <typename T>
        void add(Entity entity, T value)
        {
            enqueue([entity, value](World& world) {
                if (world.isAlive(entity) && !world.hasComponent<T>(entity))
                {
                    world.addComponent<T>(entity, value);
                }
            });
        }

        template <typename T>
        void remove(Entity entity)
        {
            enqueue([entity](World& world) {
                if (world.isAlive(entity) && world.hasComponent<T>(entity))
                {
                    world.removeComponent<T>(entity);
                }
            });
        }

        /// Escape hatch for compound structural edits that must apply
        /// atomically at playback (e.g. regime conversions swapping several
        /// components). Same thread-safety as the typed helpers.
        void defer(std::function<void(World&)> command) { enqueue(std::move(command)); }

        /// Owner-thread only. Executes all commands in order, then clears.
        void playback(World& world);

        [[nodiscard]] usize pendingCount() const;

    private:
        void enqueue(std::function<void(World&)> command); // thread-safe

        mutable std::mutex m_mutex;
        std::vector<std::function<void(World&)>> m_commands;
    };
} // namespace sw::ecs
