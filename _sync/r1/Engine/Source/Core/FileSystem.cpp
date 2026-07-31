#include "Core/FileSystem.hpp"

#include "Core/Error.hpp"

#include <fstream>

#if defined(_WIN32)
    #define WIN32_LEAN_AND_MEAN
    #include <Windows.h>
#elif defined(__linux__)
    #include <unistd.h>
#endif

namespace sw
{
    std::vector<u8> FileSystem::readBinaryFile(const std::filesystem::path& path)
    {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file.is_open())
        {
            SW_THROW("Failed to open file '{}'", path.string());
        }

        const std::streamsize size = file.tellg();
        if (size < 0)
        {
            SW_THROW("Failed to query size of file '{}'", path.string());
        }

        std::vector<u8> buffer(static_cast<usize>(size));
        file.seekg(0);
        if (size > 0 && !file.read(reinterpret_cast<char*>(buffer.data()), size))
        {
            SW_THROW("Failed to read file '{}'", path.string());
        }
        return buffer;
    }

    void FileSystem::writeBinaryFile(const std::filesystem::path& path,
                                     const std::vector<u8>& bytes)
    {
        std::ofstream file(path, std::ios::binary | std::ios::trunc);
        if (!file.is_open())
        {
            SW_THROW("Failed to open '{}' for writing", path.string());
        }
        if (!bytes.empty() &&
            !file.write(reinterpret_cast<const char*>(bytes.data()),
                        static_cast<std::streamsize>(bytes.size())))
        {
            SW_THROW("Failed to write {} bytes to '{}'", bytes.size(), path.string());
        }
    }

    std::filesystem::path FileSystem::executableDirectory()
    {
#if defined(_WIN32)
        wchar_t buffer[MAX_PATH];
        const DWORD length = GetModuleFileNameW(nullptr, buffer, MAX_PATH);
        if (length > 0 && length < MAX_PATH)
        {
            return std::filesystem::path(buffer).parent_path();
        }
#elif defined(__linux__)
        char buffer[4096];
        const ssize_t length = ::readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
        if (length > 0)
        {
            buffer[length] = '\0';
            return std::filesystem::path(buffer).parent_path();
        }
#endif
        // Fallback: current working directory.
        return std::filesystem::current_path();
    }

    std::filesystem::path FileSystem::projectRoot()
    {
        std::error_code errorCode;
        std::filesystem::path probe =
            std::filesystem::weakly_canonical(executableDirectory(), errorCode);
        if (errorCode)
        {
            probe = executableDirectory();
        }

        // Bounded by the FILESYSTEM rather than by a guessed number of
        // levels: stop when parent_path() stops moving, which is what a root
        // directory does on both Windows and POSIX. A fixed "five levels up"
        // either falls short of a deep build tree or overshoots into the
        // drive root, and both failures are silent.
        for (int guard = 0; guard < 64; ++guard)
        {
            const bool hasLists =
                std::filesystem::exists(probe / "CMakeLists.txt", errorCode);
            const bool hasAssets =
                std::filesystem::is_directory(probe / "Assets", errorCode);
            if (hasLists && hasAssets)
            {
                return probe;
            }
            const std::filesystem::path parent = probe.parent_path();
            if (parent.empty() || parent == probe)
            {
                break;
            }
            probe = parent;
        }
        return {};
    }

    std::filesystem::path FileSystem::resolve(const std::filesystem::path& relative)
    {
        const std::filesystem::path fromExe = executableDirectory() / relative;
        if (std::filesystem::exists(fromExe))
        {
            return fromExe;
        }

        const std::filesystem::path fromCwd = std::filesystem::current_path() / relative;
        if (std::filesystem::exists(fromCwd))
        {
            return fromCwd;
        }

        SW_THROW("Failed to resolve path '{}' (searched '{}' and '{}')",
                 relative.string(), fromExe.string(), fromCwd.string());
    }
} // namespace sw
