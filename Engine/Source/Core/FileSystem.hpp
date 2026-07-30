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

        /// The root of the SOURCE tree, or an empty path when there is none.
        ///
        /// NOTHING IN THIS PROJECT HARDCODES A DRIVE OR A FOLDER. It has to
        /// work from F:\, from G:\, from a USB stick and from a path with a
        /// space in it, so anything that needs the root asks for it here
        /// instead of writing it down.
        ///
        /// Found by walking up from the executable, looking for a directory
        /// carrying BOTH `CMakeLists.txt` AND `Assets/`. The pair is the
        /// marker because either alone matches directories that are not
        /// ours: walking up for `Assets` on its own would happily settle on
        /// `G:\Assets` and start writing into it.
        ///
        /// Returns EMPTY for a packaged build. That is correct and not a
        /// failure — `dist\StarWorks\` deliberately ships no source tree,
        /// and a caller that treated "no root" as an error would break the
        /// shipped game to serve the developer's convenience.
        [[nodiscard]] static std::filesystem::path projectRoot();

        /// Resolves a path relative to (in order): the executable directory,
        /// then the current working directory. Throws sw::Exception if the
        /// file exists in neither location.
        [[nodiscard]] static std::filesystem::path resolve(const std::filesystem::path& relative);
    };
} // namespace sw
