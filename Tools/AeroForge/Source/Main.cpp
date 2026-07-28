// ============================================================================
// Tools/AeroForge — the offline wind tunnel.
//
//   AeroForge [<parts directory>] [--out <directory>] [--resolution N]
//             [--theta N] [--phi N] [--part <id>] [--report]
//
// Reads every .swpart it finds, solves each one over the wind-direction
// grid, and writes the matching .aero.json beside it. Buildings are skipped
// — a refinery is not going anywhere near an airstream — unless --all is
// given.
//
// --report prints the drag coefficient the solved table implies head-on and
// side-on, referred to the part's own largest silhouette. That is the number
// to sanity-check against a textbook, and the reason it is printed rather
// than buried: a generated asset nobody ever looks at is a generated asset
// nobody can trust.
// ============================================================================

#include "Core/Log.hpp"
#include "Gameplay/AeroForge.hpp"
#include "Gameplay/Parts.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <span>
#include <cstdio>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    [[nodiscard]] sw::u32 parseCount(std::string_view text, sw::u32 fallback)
    {
        sw::u32 value = 0;
        const auto* first = text.data();
        const auto* last = first + text.size();
        const auto result = std::from_chars(first, last, value);
        return (result.ec == std::errc{} && value > 0) ? value : fallback;
    }

    struct Reading
    {
        sw::f64 areaM2 = 0.0;
        sw::f64 dragCoefficient = 0.0;
        sw::f64 crossCoefficient = 0.0; // lift, referred to the same area
    };

    /// The textbook numbers for one flow direction, referred to the area
    /// the part actually presents IN that direction — which is the only
    /// reference that lets a fin's Cd be compared with a tank's.
    [[nodiscard]] Reading read(std::span<const sw::aero::SkinTriangle> skin,
                               const sw::aero::ForgeSettings& settings, const sw::Vec3& flow)
    {
        const sw::Vec3 direction = glm::normalize(flow);
        const sw::aero::SolvedDirection solved =
            sw::aero::solveDirection(skin, direction, settings);
        Reading out{};
        out.areaM2 = solved.projectedAreaM2;
        if (out.areaM2 <= 0.0)
        {
            return out;
        }
        const sw::f32 along = glm::dot(solved.forceM2, direction);
        out.dragCoefficient = static_cast<sw::f64>(along) / out.areaM2;
        out.crossCoefficient =
            static_cast<sw::f64>(glm::length(solved.forceM2 - direction * along)) /
            out.areaM2;
        return out;
    }
} // namespace

int main(int argc, char** argv)
{
    sw::Log::initialize(sw::Log::Config{sw::LogLevel::Warn, {}, true});

    std::filesystem::path partsDirectory = "Assets/Parts";
    std::filesystem::path outputDirectory;
    sw::aero::ForgeSettings settings{};
    sw::u32 onlyPart = 0;
    bool includeBuildings = false;
    bool report = false;

    for (int i = 1; i < argc; ++i)
    {
        const std::string_view argument = argv[i];
        const auto next = [&](std::string_view fallback) -> std::string_view {
            return (i + 1 < argc) ? std::string_view(argv[++i]) : fallback;
        };
        if (argument == "--out") { outputDirectory = next(""); }
        else if (argument == "--resolution") { settings.resolution = parseCount(next(""), settings.resolution); }
        else if (argument == "--theta") { settings.thetaCount = parseCount(next(""), settings.thetaCount); }
        else if (argument == "--phi") { settings.phiCount = parseCount(next(""), settings.phiCount); }
        else if (argument == "--part") { onlyPart = parseCount(next(""), 0); }
        else if (argument == "--all") { includeBuildings = true; }
        else if (argument == "--report") { report = true; }
        else if (argument.starts_with("--"))
        {
            std::printf("unknown option '%.*s'\n", static_cast<int>(argument.size()),
                        argument.data());
            return 1;
        }
        else { partsDirectory = argument; }
    }
    if (outputDirectory.empty())
    {
        outputDirectory = partsDirectory;
    }

    std::error_code error;
    if (!std::filesystem::is_directory(partsDirectory, error))
    {
        std::printf("no such directory: %s\n", partsDirectory.string().c_str());
        return 1;
    }
    std::filesystem::create_directories(outputDirectory, error);

    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(partsDirectory, error))
    {
        if (entry.path().extension() == ".swpart")
        {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());

    sw::u32 written = 0;
    for (const std::filesystem::path& path : files)
    {
        sw::parts::PartDefinition definition{};
        if (!sw::parts::loadPartFile(path, definition))
        {
            continue;
        }
        if (onlyPart != 0 && definition.id != onlyPart)
        {
            continue;
        }
        if (!includeBuildings && !sw::parts::isVesselPart(definition))
        {
            continue; // a refinery never meets an airstream
        }

        const sw::aero::AeroTable table = sw::aero::forgePart(definition, settings);
        if (!table.valid())
        {
            std::printf("  %-24s FAILED\n", definition.name.c_str());
            continue;
        }
        const std::filesystem::path out =
            outputDirectory / sw::aero::aeroPathFor(path.filename());
        if (!sw::aero::saveAeroTable(table, out))
        {
            continue;
        }
        written += 1;

        if (report)
        {
            const std::vector<sw::aero::SkinTriangle> skin = sw::aero::partSkin(definition);
            // Mirror the forge's own wing-or-body decision so the report
            // describes the table that was actually written.
            sw::Vec3 low(1.0e9f), high(-1.0e9f);
            for (const sw::aero::SkinTriangle& triangle : skin)
            {
                for (const sw::Vec3& point : {triangle.a, triangle.b, triangle.c})
                {
                    low = glm::min(low, point);
                    high = glm::max(high, point);
                }
            }
            const sw::Vec3 extents = high - low;
            const bool lifting =
                std::min({extents.x, extents.y, extents.z}) /
                    std::max({extents.x, extents.y, extents.z}) <
                static_cast<sw::f32>(settings.plateRatio);
            sw::aero::ForgeSettings reportSettings = settings;
            reportSettings.liftingSurface = lifting;
            const Reading nose = read(skin, reportSettings, sw::Vec3(0.0f, 0.0f, 1.0f));
            const Reading side = read(skin, reportSettings, sw::Vec3(1.0f, 0.0f, 0.0f));
            const Reading angled = read(
                skin, reportSettings,
                sw::Vec3(0.0f, std::sin(0.2618f), std::cos(0.2618f))); // 15 deg out of plane
            std::printf("  %-22s %-4s nose %6.3f m^2 Cd %5.3f | side %6.3f m^2 Cd %5.3f"
                        " | 15deg Cl %5.3f\n",
                        definition.name.c_str(), lifting ? "WING" : "body", nose.areaM2,
                        nose.dragCoefficient, side.areaM2, side.dragCoefficient,
                        angled.crossCoefficient);
        }
    }

    std::printf("AeroForge: %u tables written to %s\n", written,
                outputDirectory.string().c_str());
    return (written > 0) ? 0 : 1;
}
