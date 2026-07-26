// ============================================================================
// Main.cpp — Part Studio entry point.
// ============================================================================

#include "PartStudioApp.hpp"

#include <cstdio>
#include <cstring>
#include <string>

namespace
{
    sw::ApplicationConfig parseArguments(int argc, char** argv)
    {
        sw::ApplicationConfig config{};
        config.name = "PartStudio";
        config.window.title = "StarWorks Part Studio";
        config.window.width = 1600;
        config.window.height = 900;
        config.log.level = sw::LogLevel::Info;
        for (int i = 1; i < argc; ++i)
        {
            if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc)
            {
                config.maxFrames = static_cast<sw::u64>(std::stoull(argv[++i]));
            }
            else if (std::strcmp(argv[i], "--cpu") == 0)
            {
                config.preferCpuDevice = true;
            }
        }
        return config;
    }
} // namespace

int main(int argc, char** argv)
{
    try
    {
        studio::PartStudioApp app(parseArguments(argc, argv));
        app.run();
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "[FATAL] %s\n", e.what());
        return 1;
    }
    return 0;
}
