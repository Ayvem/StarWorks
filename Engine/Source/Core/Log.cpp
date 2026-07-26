#include "Core/Log.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>

#if defined(_WIN32)
    #define WIN32_LEAN_AND_MEAN
    #include <Windows.h>
#endif

namespace sw
{
    namespace
    {
        struct LogState
        {
            std::mutex mutex;
            LogLevel level = LogLevel::Info;
            bool useColors = true;
            bool initialized = false;
            std::ofstream file;
        };

        LogState& state()
        {
            static LogState s;
            return s;
        }

        constexpr std::array<const char*, 6> kLevelNames = {
            "TRACE", "DEBUG", "INFO ", "WARN ", "ERROR", "CRIT ",
        };

        // ANSI escape sequences per level (empty = default color).
        constexpr std::array<const char*, 6> kLevelColors = {
            "\x1b[90m",        // Trace    — bright black
            "\x1b[36m",        // Debug    — cyan
            "\x1b[32m",        // Info     — green
            "\x1b[33m",        // Warn     — yellow
            "\x1b[31m",        // Error    — red
            "\x1b[41m\x1b[97m" // Critical — white on red
        };
        constexpr const char* kColorReset = "\x1b[0m";

        /// "HH:MM:SS.mmm" wall-clock timestamp.
        std::string makeTimestamp()
        {
            using namespace std::chrono;
            const auto now = system_clock::now();
            const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
            const std::time_t t = system_clock::to_time_t(now);

            std::tm tmBuf{};
#if defined(_WIN32)
            localtime_s(&tmBuf, &t);
#else
            localtime_r(&t, &tmBuf);
#endif
            char buf[16];
            std::strftime(buf, sizeof(buf), "%H:%M:%S", &tmBuf);
            return std::format("{}.{:03}", buf, static_cast<int>(ms.count()));
        }

        void enableWindowsAnsiColors()
        {
#if defined(_WIN32)
            const HANDLE handle = GetStdHandle(STD_OUTPUT_HANDLE);
            if (handle == INVALID_HANDLE_VALUE)
            {
                return;
            }
            DWORD mode = 0;
            if (GetConsoleMode(handle, &mode))
            {
                SetConsoleMode(handle, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
            }
#endif
        }
    } // namespace

    void Log::initialize(const Config& config)
    {
        LogState& s = state();
        std::scoped_lock lock(s.mutex);

        s.level = config.level;
        s.useColors = config.useColors;

        if (s.file.is_open())
        {
            s.file.close();
        }
        if (!config.filePath.empty())
        {
            s.file.open(config.filePath, std::ios::out | std::ios::trunc);
            if (!s.file.is_open())
            {
                std::fprintf(stderr, "[Log] Failed to open log file '%s'\n", config.filePath.c_str());
            }
        }

        if (s.useColors)
        {
            enableWindowsAnsiColors();
        }
        s.initialized = true;
    }

    void Log::shutdown()
    {
        LogState& s = state();
        std::scoped_lock lock(s.mutex);
        if (s.file.is_open())
        {
            s.file.flush();
            s.file.close();
        }
        s.initialized = false;
    }

    void Log::setLevel(LogLevel level)
    {
        LogState& s = state();
        std::scoped_lock lock(s.mutex);
        s.level = level;
    }

    bool Log::isLevelEnabled(LogLevel level)
    {
        // Racy read is acceptable: worst case one message is dropped/kept
        // during a level change; correctness is preserved.
        return static_cast<u8>(level) >= static_cast<u8>(state().level);
    }

    void Log::write(LogLevel level, std::string_view category, std::string_view message)
    {
        if (level == LogLevel::Off)
        {
            return;
        }

        LogState& s = state();
        std::scoped_lock lock(s.mutex);

        const auto idx = static_cast<usize>(level);
        const std::string timestamp = makeTimestamp();
        const std::string line =
            std::format("[{}] [{}] [{}] {}", timestamp, kLevelNames[idx], category, message);

        std::FILE* const stream = (level >= LogLevel::Error) ? stderr : stdout;
        if (s.useColors && s.initialized)
        {
            std::fprintf(stream, "%s%s%s\n", kLevelColors[idx], line.c_str(), kColorReset);
        }
        else
        {
            std::fprintf(stream, "%s\n", line.c_str());
        }

        if (s.file.is_open())
        {
            // Flushed on every write: log volume is low (no per-entity spam by
            // policy) and a crash/kill must never lose the last lines — that is
            // the whole point of a log file.
            s.file << line << std::endl;
        }
    }
} // namespace sw
