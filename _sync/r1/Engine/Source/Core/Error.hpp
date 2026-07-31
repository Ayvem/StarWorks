#pragma once

// ============================================================================
// Core/Error.hpp
// Error handling policy of the engine:
//
//  - Unrecoverable initialization/loading failures throw sw::Exception,
//    which carries the source location. The top-level main() catches it,
//    logs it and exits cleanly.
//  - Per-frame / hot paths never throw: they use return codes (e.g. VkResult
//    handling in the renderer) so the frame loop stays predictable.
//  - Programmer errors are caught with SW_ASSERT (Core/Assert.hpp).
// ============================================================================

#include <exception>
#include <format>
#include <string>
#include <utility>

namespace sw
{
    class Exception : public std::exception
    {
    public:
        Exception(std::string message, const char* file, int line)
            : m_message(std::move(message))
            , m_file(file)
            , m_line(line)
        {
            m_fullMessage = std::format("{} ({}:{})", m_message, m_file, m_line);
        }

        [[nodiscard]] const char* what() const noexcept override { return m_fullMessage.c_str(); }
        [[nodiscard]] const std::string& message() const noexcept { return m_message; }
        [[nodiscard]] const char* file() const noexcept { return m_file; }
        [[nodiscard]] int line() const noexcept { return m_line; }

    private:
        std::string m_message;
        std::string m_fullMessage;
        const char* m_file;
        int m_line;
    };
} // namespace sw

/// Throws sw::Exception with std::format semantics and source location.
/// Usage: SW_THROW("Failed to open '{}'", path);
#define SW_THROW(...) throw ::sw::Exception(::std::format(__VA_ARGS__), __FILE__, __LINE__)
