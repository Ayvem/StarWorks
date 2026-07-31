#include "Core/ThreadPool.hpp"

#include "Core/Log.hpp"

#include <algorithm>

namespace sw
{
    ThreadPool::ThreadPool(u32 threadCount)
    {
        if (threadCount == 0)
        {
            const u32 hardware = std::thread::hardware_concurrency();
            threadCount = (hardware > 1) ? hardware - 1 : 1;
        }

        m_workers.reserve(threadCount);
        for (u32 i = 0; i < threadCount; ++i)
        {
            m_workers.emplace_back([this] { workerLoop(); });
        }
        SW_LOG_INFO("Core", "ThreadPool started with {} worker threads", threadCount);
    }

    ThreadPool::~ThreadPool()
    {
        {
            std::scoped_lock lock(m_mutex);
            m_stopping = true;
        }
        m_taskAvailable.notify_all();
        for (std::thread& worker : m_workers)
        {
            worker.join();
        }
    }

    void ThreadPool::submit(std::function<void()> task)
    {
        {
            std::scoped_lock lock(m_mutex);
            m_tasks.push_back(std::move(task));
        }
        m_taskAvailable.notify_one();
    }

    void ThreadPool::waitIdle()
    {
        std::unique_lock lock(m_mutex);
        m_allIdle.wait(lock, [this] { return m_tasks.empty() && m_activeTasks == 0; });
    }

    void ThreadPool::workerLoop()
    {
        for (;;)
        {
            std::function<void()> task;
            {
                std::unique_lock lock(m_mutex);
                m_taskAvailable.wait(lock, [this] { return m_stopping || !m_tasks.empty(); });
                if (m_stopping && m_tasks.empty())
                {
                    return;
                }
                task = std::move(m_tasks.front());
                m_tasks.pop_front();
                ++m_activeTasks;
            }

            task();

            {
                std::scoped_lock lock(m_mutex);
                --m_activeTasks;
                if (m_tasks.empty() && m_activeTasks == 0)
                {
                    m_allIdle.notify_all();
                }
            }
        }
    }
} // namespace sw
