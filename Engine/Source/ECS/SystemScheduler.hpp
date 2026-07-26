#pragma once

// ============================================================================
// ECS/SystemScheduler.hpp
// Groups registered systems into sequential stages such that systems inside
// one stage have no read/write conflicts, then runs each stage's systems in
// parallel on the ThreadPool.
//
// Staging is order-preserving: a system never jumps ahead of an earlier
// system it conflicts with, so data flow follows registration order and
// results are deterministic regardless of thread timing (systems with
// disjoint access sets are, by construction, order-independent).
// ============================================================================

#include "Core/Types.hpp"
#include "ECS/System.hpp"

#include <memory>
#include <vector>

namespace sw
{
    class ThreadPool;
} // namespace sw

namespace sw::ecs
{
    class World;

    class SystemScheduler
    {
    public:
        /// Registers a system; stages are rebuilt lazily before the next run.
        void addSystem(std::unique_ptr<System> system);

        /// Runs all systems for one tick. pool == nullptr forces fully
        /// sequential execution (useful for debugging determinism issues).
        void run(World& world, f32 deltaSeconds, ThreadPool* pool);

        [[nodiscard]] u32 systemCount() const { return static_cast<u32>(m_systems.size()); }
        [[nodiscard]] u32 stageCount();

    private:
        void rebuildStagesIfNeeded();

        std::vector<std::unique_ptr<System>> m_systems;
        std::vector<std::vector<System*>> m_stages;
        bool m_stagesDirty = false;
    };
} // namespace sw::ecs
