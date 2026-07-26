#pragma once

// ============================================================================
// Core/Log.hpp
// Engine-wide logging.
//
// Design:
//  - Static facility: available everywhere without plumbing a logger object
//    through every constructor. Initialized once by the Application.
//  - Thread-safe: a single mutex protects the sinks. Logging is not expected
//    on hot per-entity paths; systems will aggregate and log summaries.
//  - Two sinks: colored console output and an optional log file.
//  - Category string per message ("Core", "Renderer", "Vulkan", ...), so
//    output stays filterable as the engine grows.
//  - The SW_LOG_* macros check the level *before* evaluating std::format,
//    so disabled log levels cost a single branch.
// ============================================================================

#include "Core/Types.hpp"

#include <format>
#include <string>
#include <string_view>

namespace sw
{
    enum class LogLevel : u8
    {
        Trace = 0,
        Debug,
        Info,
        Warn,
        Error,
        Critical,
        Off,
    };

    class Log
    {
    public:
        struct Config
        {
            LogLevel level = LogLevel::Info;
            /// Empty = no file sink.
            std::string filePath{};
            bool useColors = true;
        };

        Log() = delete;

        /// Must be called once before any logging macro. Safe to call again
        /// (reconfigures the sinks).
        static void initialize(const Config& config);

        /// Flushes and closes the sinks. Logging after shutdown is safe and
        /// falls back to plain stderr.
        static void shutdown();

        static void setLevel(LogLevel level);
        [[nodiscard]] static bool isLevelEnabled(LogLevel level);

        /// Core entry point; prefer the SW_LOG_* macros.
        static void write(LogLevel level, std::string_view category, std::string_view message);
    };
} // namespace sw

// ----------------------------------------------------------------------------
// Logging macros — the only supported way to emit log messages.
// Usage: SW_LOG_INFO("Renderer", "Swapchain recreated: {}x{}", w, h);
// ----------------------------------------------------------------------------
#define SW_LOG_IMPL(level, category, ...)                                          \
    do                                                                             \
    {                                                                              \
        if (::sw::Log::isLevelEnabled(level))                                      \
        {                                                                          \
            ::sw::Log::write(level, category, ::std::format(__VA_ARGS__));         \
        }                                                                          \
    } while (false)

#define SW_LOG_TRACE(category, ...)    SW_LOG_IMPL(::sw::LogLevel::Trace, category, __VA_ARGS__)
#define SW_LOG_DEBUG(category, ...)    SW_LOG_IMPL(::sw::LogLevel::Debug, category, __VA_ARGS__)
#define SW_LOG_INFO(category, ...)     SW_LOG_IMPL(::sw::LogLevel::Info, category, __VA_ARGS__)
#define SW_LOG_WARN(category, ...)     SW_LOG_IMPL(::sw::LogLevel::Warn, category, __VA_ARGS__)
#define SW_LOG_ERROR(category, ...)    SW_LOG_IMPL(::sw::LogLevel::Error, category, __VA_ARGS__)
#define SW_LOG_CRITICAL(category, ...) SW_LOG_IMPL(::sw::LogLevel::Critical, category, __VA_ARGS__)
