#pragma once

// ============================================================================
// Core/ThreadPool.hpp
// Persistent worker pool. Current users: the ECS SystemScheduler (one task
// per system inside a stage). It will grow into the engine job system
// (task graphs, work stealing) — call sites already treat it as opaque.
//
// submit() is thread-safe. waitIdle() blocks until every submitted task has
// finished; it is the only synchronization point callers need.
// ============================================================================

#include "Core/Types.hpp"

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace sw
{
    class ThreadPool
    {
    public:
        /// threadCount == 0 selects (hardware_concurrency - 1), minimum 1.
        explicit ThreadPool(u32 threadCount = 0);
        ~ThreadPool();

        ThreadPool(const ThreadPool&) = delete;
        ThreadPool& operator=(const ThreadPool&) = delete;

        void submit(std::function<void()> task);

        /// Blocks until the queue is empty and no task is running.
        void waitIdle();

        [[nodiscard]] u32 threadCount() const { return static_cast<u32>(m_workers.size()); }

    private:
        void workerLoop();

        std::vector<std::thread> m_workers;
        std::deque<std::function<void()>> m_tasks;
        std::mutex m_mutex;
        std::condition_variable m_taskAvailable;
        std::condition_variable m_allIdle;
        u32 m_activeTasks = 0;
        bool m_stopping = false;
    };
} // namespace sw
