// ============================================================================
// PortabilityTests.cpp — the project must work from any folder on any drive.
//
// WHY THIS FILE EXISTS. The whole tree was moved from F:\StarWorks to
// G:\StarWorks and `launch.ps1` broke on its very first line, because that
// line was `cd F:\StarWorks`. One hardcoded path, written once for
// convenience, and the project stops building on the same machine that built
// it yesterday.
//
// The rule that replaces it: NOTHING ASSUMES WHERE THE PROJECT LIVES.
// Scripts resolve from $PSScriptRoot, CMake from ${CMAKE_SOURCE_DIR}, the
// game from its own executable, and anything that genuinely needs the source
// root asks FileSystem::projectRoot() for it.
//
// A rule nobody checks is a rule that lasts a week, so this walks the source
// tree and fails on any drive-letter path that has crept back in. It is a
// test rather than a code review because a code review does not run in CI.
//
// SCOPE, deliberately: scripts, C++ and build files, where an absolute path
// is always a bug. Markdown is excluded — `-Exe C:\path\to\it.exe` in a
// document is an illustration of a path the USER supplies, which is exactly
// what documentation is for.
// ============================================================================

#include "TestFramework.hpp"

#include <Core/FileSystem.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace sw;

namespace
{
    /// A drive-letter path: one letter, a colon, then a separator. Written
    /// out rather than pulled from <regex> so the matcher is obvious and
    /// cannot surprise anyone reading a failure.
    bool looksLikeAbsoluteWindowsPath(const std::string& line, usize colon)
    {
        if (colon == 0 || colon + 1 >= line.size())
        {
            return false;
        }
        const char letter = line[colon - 1];
        const bool isLetter =
            (letter >= 'A' && letter <= 'Z') || (letter >= 'a' && letter <= 'z');
        if (!isLetter)
        {
            return false;
        }
        // A letter directly before the colon is not enough: "sw::net" and a
        // timestamp both have one. The separator right after is what makes
        // it a path.
        const char next = line[colon + 1];
        if (next != '\\' && next != '/')
        {
            return false;
        }
        // The letter must not be part of a longer word — `https://` ends in
        // "s:/" and is not a drive.
        if (colon >= 2)
        {
            const char before = line[colon - 2];
            const bool wordish = (before >= 'A' && before <= 'Z') ||
                                 (before >= 'a' && before <= 'z') ||
                                 (before >= '0' && before <= '9');
            if (wordish)
            {
                return false;
            }
        }
        return true;
    }

    /// A whole-line comment, in every language this tree contains: `//` and
    /// `*` (a continued block) for C++, `#` for PowerShell and CMake.
    ///
    /// Comments are skipped because a comment does not execute — and the
    /// files that EXPLAIN this rule have to be able to name the paths they
    /// are banning. The limitation is deliberate and worth knowing: a path
    /// in a comment TRAILING real code is still flagged, because deciding
    /// where a comment starts on a mixed line means parsing string literals,
    /// and a matcher that needs a parser is a matcher nobody trusts. The fix
    /// when that bites is to put the comment on its own line.
    bool isWholeLineComment(const std::string& line)
    {
        usize i = 0;
        while (i < line.size() && (line[i] == ' ' || line[i] == '\t'))
        {
            ++i;
        }
        if (i >= line.size())
        {
            return false;
        }
        if (line[i] == '#' || line[i] == '*')
        {
            return true;
        }
        if (line[i] == '/' && i + 1 < line.size() &&
            (line[i + 1] == '/' || line[i + 1] == '*'))
        {
            return true;
        }
        // Batch: `rem ` in any case. The trailing space is required — `remove`
        // is not a comment, and batch agrees.
        if (i + 3 < line.size())
        {
            const auto lower = [](char c) {
                return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
            };
            if (lower(line[i]) == 'r' && lower(line[i + 1]) == 'e' &&
                lower(line[i + 2]) == 'm' &&
                (line[i + 3] == ' ' || line[i + 3] == '\t'))
            {
                return true;
            }
        }
        return false;
    }

    bool hasExtension(const std::filesystem::path& path,
                      const std::vector<std::string>& extensions)
    {
        const std::string extension = path.extension().string();
        for (const std::string& candidate : extensions)
        {
            if (extension == candidate)
            {
                return true;
            }
        }
        return false;
    }
} // namespace

SW_TEST(TheProjectRootIsFoundByItsMarkerAndNotByCountingLevels)
{
    const std::filesystem::path root = FileSystem::projectRoot();

    // An empty root is the legitimate answer for a packaged build, where
    // there is no source tree at all. Everything below only applies when we
    // are running from one.
    if (root.empty())
    {
        return;
    }

    // The marker is the PAIR. Either half alone matches folders that are not
    // ours, which is how a five-levels-up walk ends at a drive root.
    SW_CHECK(std::filesystem::exists(root / "CMakeLists.txt"));
    SW_CHECK(std::filesystem::is_directory(root / "Assets"));
    SW_CHECK(std::filesystem::is_directory(root / "Engine" / "Source"));

    // And it must be an ancestor of the running executable, not some other
    // checkout that happened to match first.
    const std::string exePath = FileSystem::executableDirectory().string();
    const std::string rootPath = root.string();
    SW_CHECK(exePath.size() >= rootPath.size());
    SW_CHECK(exePath.compare(0, rootPath.size(), rootPath) == 0);
}

SW_TEST(NoScriptOrSourceFileHardcodesADriveLetterPath)
{
    const std::filesystem::path root = FileSystem::projectRoot();
    if (root.empty())
    {
        return; // packaged build: nothing to scan
    }

    const std::vector<std::string> scanned = {".ps1", ".cmd",   ".bat", ".cpp",
                                              ".hpp", ".cmake", ".json", ".txt"};
    std::vector<std::string> offenders;

    std::error_code errorCode;
    auto walker = std::filesystem::recursive_directory_iterator(
        root, std::filesystem::directory_options::skip_permission_denied, errorCode);
    if (errorCode)
    {
        return;
    }

    for (const auto& entry : walker)
    {
        const std::filesystem::path& path = entry.path();

        // Build trees are full of generated absolute paths and are nobody's
        // source. So is anything fetched by CMake.
        const std::string asText = path.generic_string();
        if (asText.find("/build/") != std::string::npos ||
            asText.find("/_deps/") != std::string::npos ||
            asText.find("/dist/") != std::string::npos ||
            asText.find("/.git/") != std::string::npos)
        {
            continue;
        }
        if (!entry.is_regular_file(errorCode) || !hasExtension(path, scanned))
        {
            continue;
        }

        std::ifstream file(path);
        if (!file.is_open())
        {
            continue;
        }
        std::string line;
        int lineNumber = 0;
        while (std::getline(file, line))
        {
            ++lineNumber;
            if (isWholeLineComment(line))
            {
                continue;
            }
            for (usize i = 0; i < line.size(); ++i)
            {
                if (line[i] == ':' && looksLikeAbsoluteWindowsPath(line, i))
                {
                    offenders.push_back(path.filename().string() + ":" +
                                        std::to_string(lineNumber) + "  " + line);
                    break;
                }
            }
        }
    }

    for (const std::string& offender : offenders)
    {
        std::printf("        hardcoded path -> %s\n", offender.c_str());
    }
    // Zero, and the log above names every one of them so a failure is
    // actionable without re-running anything.
    SW_CHECK_EQ(offenders.size(), usize{0});
}
