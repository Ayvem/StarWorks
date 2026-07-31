#pragma once

// ============================================================================
// Core/Assert.hpp
// Engine assertions.
//
//  - SW_ASSERT(cond, "fmt", ...) : active only in SW_DEBUG builds. Logs a
//    critical message with file/line and breaks into the debugger.
//  - SW_VERIFY(cond, "fmt", ...) : same, but the condition is evaluated in
//    every build (use when the expression has side effects).
// ============================================================================

#include "Core/Log.hpp"

#if defined(_MSC_VER)
    #define SW_DEBUGBREAK() __debugbreak()
#elif defined(__has_builtin)
    #if __has_builtin(__builtin_debugtrap)
        #define SW_DEBUGBREAK() __builtin_debugtrap()
    #else
        #define SW_DEBUGBREAK() __builtin_trap()
    #endif
#else
    #define SW_DEBUGBREAK() __builtin_trap()
#endif

#define SW_ASSERT_IMPL(cond, ...)                                                    \
    do                                                                               \
    {                                                                                \
        if (!(cond))                                                                 \
        {                                                                            \
            SW_LOG_CRITICAL("Assert", "Assertion failed: {} ({}:{}) — {}", #cond,    \
                            __FILE__, __LINE__, ::std::format(__VA_ARGS__));         \
            SW_DEBUGBREAK();                                                         \
        }                                                                            \
    } while (false)

#if defined(SW_DEBUG)
    #define SW_ASSERT(cond, ...) SW_ASSERT_IMPL(cond, __VA_ARGS__)
    #define SW_VERIFY(cond, ...) SW_ASSERT_IMPL(cond, __VA_ARGS__)
#else
    #define SW_ASSERT(cond, ...) ((void)0)
    #define SW_VERIFY(cond, ...)                                                     \
        do                                                                           \
        {                                                                            \
            if (!(cond))                                                             \
            {                                                                        \
                SW_LOG_ERROR("Assert", "Verify failed: {} ({}:{})", #cond,           \
                             __FILE__, __LINE__);                                    \
            }                                                                        \
        } while (false)
#endif
