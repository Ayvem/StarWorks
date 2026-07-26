#pragma once

// ============================================================================
// Core/FileSystem.hpp
// Minimal file utilities needed by the early engine. Will grow into the
// Assets module's VFS backend; game code should never hardcode paths.
// ============================================================================

#include "Core/Types.hpp"

#include <filesystem>
#include <vector>

namespace sw
{
    class FileSystem
    {
    public:
        FileSystem() = delete;

        /// Reads an entire file in binary mode. Throws sw::Exception on failure.
        [[nodiscard]] static std::vector<u8> readBinaryFile(const std::filesystem::path& path);

        /// Writes a whole buffer to a file (truncating). Throws on failure.
        static void writeBinaryFile(const std::filesystem::path& path,
                                    const std::vector<u8>& bytes);

        /// Directory containing the running executable.
        [[nodiscard]] static std::filesystem::path executableDirectory();

        /// Resolves a path relative to (in order): the executable directory,
        /// then the current working directory. Throws sw::Exception if the
        /// file exists in neither location.
        [[nodiscard]] static std::filesystem::path resolve(const std::filesystem::path& relative);
    };
} // namespace sw
