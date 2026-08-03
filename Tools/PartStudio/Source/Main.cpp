// ============================================================================
// Main.cpp — Part Studio entry point.
// ============================================================================

#include "PartStudioApp.hpp"

#include <cstdio>
#include <cstring>
#include <string>

// A CAPTURE PATH FOR THE TOOL, and not only for the game. An animation is
// authored here and played back here, so a picture of the tool with the phase
// slider up is the picture that says whether the feature works — the game's
// own camera cannot easily be pointed at a rocket standing on a pad.
std::string g_openPart;
std::string g_capturePath;
float g_openPhase = 0.0f;

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
            else if (std::strcmp(argv[i], "--part") == 0 && i + 1 < argc)
            {
                g_openPart = argv[++i];
            }
            else if (std::strcmp(argv[i], "--phase") == 0 && i + 1 < argc)
            {
                g_openPhase = std::stof(argv[++i]);
            }
            else if (std::strcmp(argv[i], "--capture") == 0 && i + 1 < argc)
            {
                g_capturePath = argv[++i];
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
        app.applyStartupOptions(g_openPart, g_openPhase, g_capturePath);
        app.run();
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "[FATAL] %s\n", e.what());
        return 1;
    }
    return 0;
}
