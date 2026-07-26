#include "ECS/SystemScheduler.hpp"

#include "Core/Log.hpp"
#include "Core/ThreadPool.hpp"

#include <string>

namespace sw::ecs
{
    namespace
    {
        constexpr const char* kLogCat = "ECS";
    } // namespace

    void SystemScheduler::addSystem(std::unique_ptr<System> system)
    {
        m_systems.push_back(std::move(system));
        m_stagesDirty = true;
    }

    u32 SystemScheduler::stageCount()
    {
        rebuildStagesIfNeeded();
        return static_cast<u32>(m_stages.size());
    }

    void SystemScheduler::rebuildStagesIfNeeded()
    {
        if (!m_stagesDirty)
        {
            return;
        }
        m_stagesDirty = false;
        m_stages.clear();

        // Order-preserving greedy staging: each system goes into the current
        // stage unless it conflicts with a system already there, in which
        // case a new stage begins.
        std::vector<System*> currentStage;
        std::vector<SystemAccess> currentAccess;

        auto flush = [&] {
            if (!currentStage.empty())
            {
                m_stages.push_back(currentStage);
                currentStage.clear();
                currentAccess.clear();
            }
        };

        for (const std::unique_ptr<System>& system : m_systems)
        {
            const SystemAccess access = system->access();
            bool conflicts = false;
            for (const SystemAccess& existing : currentAccess)
            {
                if (access.conflictsWith(existing))
                {
                    conflicts = true;
                    break;
                }
            }
            if (conflicts)
            {
                flush();
            }
            currentStage.push_back(system.get());
            currentAccess.push_back(access);
        }
        flush();

        if (Log::isLevelEnabled(LogLevel::Debug))
        {
            for (usize i = 0; i < m_stages.size(); ++i)
            {
                std::string names;
                for (const System* system : m_stages[i])
                {
                    names += names.empty() ? "" : ", ";
                    names += system->name();
                }
                SW_LOG_DEBUG(kLogCat, "Stage {}: [{}]", i, names);
            }
        }
    }

    void SystemScheduler::run(World& world, f32 deltaSeconds, ThreadPool* pool)
    {
        rebuildStagesIfNeeded();

        for (const std::vector<System*>& stage : m_stages)
        {
            if (pool == nullptr || stage.size() == 1)
            {
                for (System* system : stage)
                {
                    system->update(world, deltaSeconds);
                }
                continue;
            }

            for (System* system : stage)
            {
                pool->submit([system, &world, deltaSeconds] {
                    system->update(world, deltaSeconds);
                });
            }
            pool->waitIdle();
        }
    }
} // namespace sw::ecs
