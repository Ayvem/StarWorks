// ============================================================================
// Main.cpp — process entry point.
// Kept intentionally tiny: parse arguments, build the config, run the game,
// translate fatal exceptions into a clean exit code.
// ============================================================================

#include "StarWorksGame.hpp"

#include <cstdio>
#include <cstring>
#include <string>

namespace
{
    /// --frames N   : exit after N frames (automated soak tests / CI).
    /// --log-file F : mirror the log into file F.
    /// --cpu        : prefer a software (CPU) Vulkan device (llvmpipe, ...).
    /// --quality Q  : shading tier, low|medium|high (default high; --cpu
    ///                implies low unless --quality follows it).
    sw::ApplicationConfig parseArguments(int argc, char** argv)
    {
        sw::ApplicationConfig config{};
        config.name = "StarWorks";
        config.window.title = "StarWorks";
        config.window.width = 1600;
        config.window.height = 900;
#if defined(SW_DEBUG)
        config.log.level = sw::LogLevel::Debug;
#else
        config.log.level = sw::LogLevel::Info;
#endif

        for (int i = 1; i < argc; ++i)
        {
            if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc)
            {
                config.maxFrames = static_cast<sw::u64>(std::stoull(argv[++i]));
            }
            else if (std::strcmp(argv[i], "--log-file") == 0 && i + 1 < argc)
            {
                config.log.filePath = argv[++i];
            }
            else if (std::strcmp(argv[i], "--cpu") == 0)
            {
                config.preferCpuDevice = true;
                // A software rasterizer cannot afford the HIGH fragment
                // budget; CI captures run LOW by default.
                config.renderQuality = 0;
            }
            else if (std::strcmp(argv[i], "--quality") == 0 && i + 1 < argc)
            {
                const char* value = argv[++i];
                if (std::strcmp(value, "low") == 0) { config.renderQuality = 0; }
                else if (std::strcmp(value, "medium") == 0) { config.renderQuality = 1; }
                else { config.renderQuality = 2; }
            }
        }
        return config;
    }
} // namespace

int main(int argc, char** argv)
{
    try
    {
        game::StarWorksGame gameInstance(parseArguments(argc, argv));
        gameInstance.run();
    }
    catch (const std::exception& e)
    {
        // The logger may or may not be alive at this point; stderr always is.
        std::fprintf(stderr, "[FATAL] %s\n", e.what());
        return 1;
    }
    return 0;
}
