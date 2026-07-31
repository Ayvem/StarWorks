#pragma once

// ============================================================================
// GameInternal.hpp — helpers shared by the StarWorksGame translation units.
//
// This is everything that used to live in StarWorksGame.cpp's anonymous
// namespace: physical constants, HUD palette, mesh builders, small pure
// helpers. It is internal to the Game target — the engine never includes it.
// Functions are inline and constants inline constexpr so that any Game*.cpp
// may include this header without ODR trouble.
// ============================================================================

#include "StarWorksGame.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>

namespace game
{
        /// The autopilot mode, for the log and the HUD. One place, so a mode
        /// added later cannot be printed as "RETROGRADE" by a stale ternary.
        [[nodiscard]] inline const char* sasModeName(sw::u32 mode)
        {
            switch (mode)
            {
            case SasComponent::kStability: return "STABILITY";
            case SasComponent::kPrograde: return "PROGRADE";
            case SasComponent::kRetrograde: return "RETROGRADE";
            case SasComponent::kNode: return "NODE";
            default: return "OFF";
            }
        }

        // ---- real-world dimensions and gravity (meters, m^3/s^2) --------------
        // The hierarchy: Sol -> Terra (-> Luna) / Mars. All values real.
        inline constexpr sw::f64 kMuSol = 1.32712440018e20;
        inline constexpr sw::f64 kSolRadius = 6.9634e8;
        inline constexpr sw::f64 kTerraRadius = 6.371e6;    // Earth
        inline constexpr sw::f64 kMuTerra = 3.986004418e14; // Earth GM
        inline constexpr sw::f64 kTerraSma = 1.496e11;      // 1 AU
        inline constexpr sw::f64 kLunaRadius = 1.7374e6;    // Moon
        inline constexpr sw::f64 kMuLuna = 4.9048695e12;    // Moon GM
        inline constexpr sw::f64 kLunaSma = 3.844e8;        // Earth-Moon distance
        inline constexpr sw::f64 kMarsRadius = 3.3895e6;
        inline constexpr sw::f64 kMuMars = 4.2828e13;
        inline constexpr sw::f64 kMarsSma = 2.2794e11;
        // Sphere-of-influence radii: r = a * (mu / mu_parent)^(2/5).
        inline constexpr sw::f64 kTerraSoi = 9.24e8;
        inline constexpr sw::f64 kLunaSoi = 6.61e7;
        inline constexpr sw::f64 kMarsSoi = 5.77e8;

        inline constexpr sw::f64 kStationAltitude = 4.0e5; // 400 km (LEO)
        inline constexpr sw::f64 kStationOrbitRadius = kTerraRadius + kStationAltitude;
        inline constexpr sw::f64 kStationPhase = 4.71238898038468986; // 3*pi/2
        /// Terra sidereal angular velocity (rad/s) around +Y — used for the
        /// SuRFace-relative speed readout.
        inline constexpr sw::WorldVec3 kTerraAngularVelocity{0.0, 7.2921e-5, 0.0};


        inline constexpr sw::f64 kBubbleEnterRadius = 1.0e4; // 10 km
        inline constexpr sw::f64 kBubbleExitRadius = 1.5e4;  // 15 km (hysteresis)

        // Star map: constant on-screen marker size and zoom limits (up to
        // the full Sol system — Mars orbit is 2.28e11 m).
        inline constexpr sw::f32 kMarkerScreenFraction = 0.016f;
        inline constexpr sw::f64 kMapMinHeight = 2.0e7;
        inline constexpr sw::f64 kMapMaxHeight = 8.0e11;
        // Line segments, not dots: a chord every degree and a half reads as
        // a smooth curve at any zoom the map allows, and the flight plan
        // gets more of them because it is the line being read.
        inline constexpr sw::u32 kTrajectorySamples = 240;
        inline constexpr sw::u32 kPredictionDisplaySamples = 320;
        /// Patch colors: current conic, then each successive patch (KSP
        /// style — the eye follows the hand-offs by color).
        inline constexpr sw::Vec4 kPatchColors[] = {
            {0.35f, 1.0f, 0.55f, 2.0f},  // green: current orbit
            {1.0f, 0.85f, 0.25f, 2.0f},  // yellow: next patch
            {0.95f, 0.45f, 1.0f, 2.0f},  // magenta
            {0.35f, 0.8f, 1.0f, 2.0f},   // cyan
            {1.0f, 0.55f, 0.25f, 2.0f},  // orange
        };
        /// How often the flight plan is recomputed (wall seconds).
        inline constexpr sw::f64 kPredictionRefreshSeconds = 0.25;

        // ---- artificial horizon (navball) ------------------------------------
        inline constexpr sw::f32 kNavballCenterY = 0.62f; // NDC, y grows downward
        inline constexpr sw::f32 kNavballRadius = 0.26f;  // NDC (vertical)
        inline constexpr sw::f32 kHalfPi = 1.5707963267948966f;

        // ---- reentry heating ---------------------------------------------------
        /// Heating proxy q = rho * v_rel^3 (W/m^2-ish). Glow ramps over
        /// [1e7, 1e9] on a log scale: faint at ~100 km on a LEO reentry,
        /// blinding below ~55 km.
        inline constexpr sw::f64 kHeatGlowStart = 1.0e7;
        inline constexpr sw::f32 kHeatLogRange = 2.0f;
        inline constexpr sw::usize kMaxParticles = 320;

        [[nodiscard]] inline sw::MeshData buildNavRingMesh(sw::u32 segments, sw::f32 thickness)
        {
            // Unit-radius ring in the XY plane (z = 0), for the HUD pass.
            sw::MeshData mesh;
            const sw::f32 inner = 1.0f - thickness;
            for (sw::u32 i = 0; i < segments; ++i)
            {
                const sw::f32 a0 =
                    2.0f * 3.14159265f * static_cast<sw::f32>(i) / segments;
                const sw::f32 a1 =
                    2.0f * 3.14159265f * static_cast<sw::f32>(i + 1) / segments;
                const sw::Vec2 d0{std::cos(a0), std::sin(a0)};
                const sw::Vec2 d1{std::cos(a1), std::sin(a1)};

                const sw::u32 base = static_cast<sw::u32>(mesh.vertices.size());
                const sw::Vec4 white{1.0f, 1.0f, 1.0f, 1.0f};
                const sw::Vec3 normal{0.0f, 0.0f, 1.0f};
                mesh.vertices.push_back({{d0.x * inner, d0.y * inner, 0.0f}, normal, white, {}});
                mesh.vertices.push_back({{d0.x, d0.y, 0.0f}, normal, white, {}});
                mesh.vertices.push_back({{d1.x, d1.y, 0.0f}, normal, white, {}});
                mesh.vertices.push_back({{d1.x * inner, d1.y * inner, 0.0f}, normal, white, {}});
                mesh.indices.insert(mesh.indices.end(),
                                    {base, base + 1, base + 2, base, base + 2, base + 3});
            }
            return mesh;
        }

        [[nodiscard]] inline sw::MeshData buildNavBarMesh()
        {
            // Unit bar: x in [-1,1], y in [-1,1] — sized entirely by the
            // instance transform.
            sw::MeshData mesh;
            const sw::Vec4 white{1.0f, 1.0f, 1.0f, 1.0f};
            const sw::Vec3 normal{0.0f, 0.0f, 1.0f};
            mesh.vertices.push_back({{-1.0f, -1.0f, 0.0f}, normal, white, {}});
            mesh.vertices.push_back({{1.0f, -1.0f, 0.0f}, normal, white, {}});
            mesh.vertices.push_back({{1.0f, 1.0f, 0.0f}, normal, white, {}});
            mesh.vertices.push_back({{-1.0f, 1.0f, 0.0f}, normal, white, {}});
            mesh.indices = {0, 1, 2, 0, 2, 3};
            return mesh;
        }

        [[nodiscard]] inline sw::MeshData buildNavDiamondMesh()
        {
            sw::MeshData mesh;
            const sw::Vec4 white{1.0f, 1.0f, 1.0f, 1.0f};
            const sw::Vec3 normal{0.0f, 0.0f, 1.0f};
            mesh.vertices.push_back({{0.0f, -1.0f, 0.0f}, normal, white, {}});
            mesh.vertices.push_back({{1.0f, 0.0f, 0.0f}, normal, white, {}});
            mesh.vertices.push_back({{0.0f, 1.0f, 0.0f}, normal, white, {}});
            mesh.vertices.push_back({{-1.0f, 0.0f, 0.0f}, normal, white, {}});
            mesh.indices = {0, 1, 2, 0, 2, 3};
            return mesh;
        }

        // ---- time warp -----------------------------------------------------------
        // The top two rungs are for LEAVING: at x100 000 a Mars transfer is
        // still three real hours, and nobody sits through that. A million
        // makes it eleven minutes, ten million makes it one — and both are
        // exact, because above physics warp every orbit is analytic and the
        // rate-based lanes bulk-consume whatever interval they are handed.
        inline constexpr sw::f32 kWarpLadder[] = {1.0f,      2.0f,      5.0f,       10.0f,
                                           50.0f,     100.0f,    1000.0f,    10000.0f,
                                           100000.0f, 1000000.0f, 10000000.0f};
        inline constexpr sw::u32 kWarpSteps = static_cast<sw::u32>(std::size(kWarpLadder));

        /// The warp rate as a pilot reads it. "1E+07" is a number a compiler
        /// prints; X10M is a number a person reads.
        [[nodiscard]] inline std::string warpText(sw::f32 rate)
        {
            if (rate >= 1.0e6f)
            {
                return std::format("{:.0f}M", rate / 1.0e6f);
            }
            if (rate >= 1.0e3f)
            {
                return std::format("{:.0f}K", rate / 1.0e3f);
            }
            return std::format("{:.0f}", rate);
        }

        /// PHYSICS WARP: the world stays fully simulated (drag, thrust,
        /// collisions) up to this time scale; beyond it everything rides
        /// analytic rails. Integration at 50 Hz stays stable to x5.
        /// The one event kind this build speaks: "my craft was here, at this
    /// instant". Everything else a player does will join it here, and the
    /// Timeline treats them all the same way.
    inline constexpr sw::u32 kNetEventBeacon = 1;

    inline constexpr sw::f32 kMaxPhysicsWarp = 5.0f;

        [[nodiscard]] inline sw::f32 maxWarpForAltitude(sw::f64 altitudeMeters)
        {
            // Inside the atmosphere, PHYSICS warp is allowed (that is the
            // whole point: reentries at x5 with live drag); rails warp is not.
            if (altitudeMeters < 1.2e5) { return kMaxPhysicsWarp; }
            if (altitudeMeters < 3.0e5) { return 10.0f; }
            if (altitudeMeters < 1.0e6) { return 100.0f; }
            if (altitudeMeters < 5.0e6) { return 1000.0f; }
            if (altitudeMeters < 2.0e7) { return 10000.0f; }
            // ...and the two interplanetary rungs, gated on being genuinely
            // FAR from everything. A million times real time moves a craft
            // 30 000 km per rendered frame at Terra's orbital speed: close
            // to a body that is a jump straight through its sphere of
            // influence, and out here it is a comfortable cruise.
            if (altitudeMeters < 1.0e8) { return 100000.0f; }
            if (altitudeMeters < 1.0e9) { return 1000000.0f; } // beyond Terra's SOI
            return 10000000.0f;
        }

        /// Sphere LOD resolutions, most to least detailed (rings, segments).
        /// LOD0 is dense enough for readable per-vertex continents.
        inline constexpr sw::u32 kLodRings[CelestialLodComponent::kLodLevels] = {150, 80, 32, 14, 8};
        inline constexpr sw::u32 kLodSegments[CelestialLodComponent::kLodLevels] = {225, 120, 48, 21, 12};
        inline constexpr sw::f32 kLodScreenFractions[CelestialLodComponent::kLodLevels - 1] = {
            0.5f, 0.15f, 0.04f, 0.008f};

        inline constexpr const char* kGlyphCharset = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,-+/%:";

        [[nodiscard]] inline sw::MeshData buildMarkerMesh()
        {
            // Markers are drawn EMISSIVE (tint alpha 2.0) — normals unused.
            return sw::PrimitiveFactory::makeOctahedron(1.0f, {1.0f, 1.0f, 1.0f, 1.0f});
        }

        inline sw::f32 hash01(sw::u32 x)
        {
            x ^= 2747636419u;
            x *= 2654435769u;
            x ^= x >> 16;
            x *= 2654435769u;
            x ^= x >> 16;
            return static_cast<sw::f32>(x & 0xFFFFFF) / 16777216.0f;
        }

        // Noise moved into the engine (Math/Noise.hpp): the terrain
        // heightfield, the globe colors and the clouds share ONE function.
        using sw::math::fbm3;

        // Terrain parameter sets live in the ENGINE (Planet/Terrain.hpp) since
        // M25: the globe colors, the physics heightfield, the terrain patch,
        // the site placement AND the GLSL twin (Shaders/Terrain.glsl) all read
        // that one table. A per-body constant defined twice is a coastline
        // that moves between the renderer and the collider.
        using sw::planet::presetLuna;
        using sw::planet::presetMars;
        using sw::planet::presetTerra;

        /// THE STARTING SITE on Terra, surveyed from the analytic fields.
        ///
        /// Until F1 the outpost and the launch pad were both nailed to the
        /// +Z equator because it was a convenient number — and +Z on Terra
        /// is open ocean, so the pad floated on the sea and the mine dug
        /// water. The survey settles it, and it searches THE EQUATOR: a base
        /// pays its latitude on every launch it ever makes, in the rotation
        /// speed it is not given (465 m/s on Terra) and in the plane change
        /// it has to fly to reach an equatorial orbit. A richer site 24
        /// degrees north is not a better site.
        ///
        /// So: sweep the whole equatorial ring for a continent with ore,
        /// then refine to flat, buildable ground inside it, both reading the
        /// very heightfield the collider uses and the very ore field the
        /// miner is paid on. Cached so the outpost, the pad and the tests all
        /// name the same place, and deterministic so a reloaded world does.
        [[nodiscard]] inline const sw::Vec3& terraStartSite()
        {
            static const sw::Vec3 site = [] {
                sw::f32 grade = 0.0f;
                return sw::planet::surveyEquatorialSite(
                    presetTerra(), sw::planet::depositsTerra(),
                    sw::res::Resource::IronOre, kTerraRadius, grade);
            }();
            return site;
        }

        /// Stands a building's model upright at a place on a body: its +Y
        /// onto the local vertical, by the shortest rotation.
        [[nodiscard]] inline sw::Quat standUpFor(const sw::Vec3& up)
        {
            const sw::Vec3 from{0.0f, 1.0f, 0.0f};
            const sw::f32 alignment = glm::dot(from, up);
            if (alignment > 0.99999f)
            {
                return sw::Quat{1.0f, 0.0f, 0.0f, 0.0f};
            }
            if (alignment < -0.99999f)
            {
                return sw::Quat{0.0f, 1.0f, 0.0f, 0.0f}; // 180 deg about X
            }
            const sw::Vec3 axis = glm::cross(from, up);
            return glm::normalize(sw::Quat{1.0f + alignment, axis.x, axis.y, axis.z});
        }

        /// The yaw about the local vertical that points `modelDirection` — a
        /// direction in the building's own frame, such as a conveyor port's
        /// outward normal — along `wantedTangent` on the ground.
        [[nodiscard]] inline sw::f32 yawToFace(const sw::Vec3& up, const sw::Vec3& modelDirection,
                                        const sw::Vec3& wantedTangent)
        {
            const sw::Vec3 have = standUpFor(up) * modelDirection;
            const sw::Vec3 haveFlat = have - up * glm::dot(have, up);
            const sw::Vec3 wantFlat = wantedTangent - up * glm::dot(wantedTangent, up);
            if (glm::length(haveFlat) < 1.0e-4f || glm::length(wantFlat) < 1.0e-4f)
            {
                return 0.0f;
            }
            const sw::Vec3 a = glm::normalize(haveFlat);
            const sw::Vec3 b = glm::normalize(wantFlat);
            return std::atan2(glm::dot(glm::cross(a, b), up),
                              glm::clamp(glm::dot(a, b), -1.0f, 1.0f));
        }

        /// Cargo colour for a resource — ore reads as rock, metal as metal.
        /// One crate on a belt is a few pixels; the colour IS the label.
        [[nodiscard]] inline sw::Vec3 resourceCargoColor(sw::res::Resource resource)
        {
            switch (resource)
            {
            case sw::res::Resource::IronOre:   return {0.42f, 0.28f, 0.20f};
            case sw::res::Resource::CopperOre: return {0.45f, 0.30f, 0.16f};
            case sw::res::Resource::Iron:      return {0.62f, 0.64f, 0.68f};
            case sw::res::Resource::Copper:    return {0.72f, 0.42f, 0.22f};
            case sw::res::Resource::WaterIce:  return {0.70f, 0.82f, 0.90f};
            case sw::res::Resource::Water:     return {0.25f, 0.45f, 0.62f};
            default:                           return {0.55f, 0.58f, 0.62f};
            }
        }

        // ====================== THE HUD PALETTE ==========================
        //
        // Every panel in the game reads from this one block. That is the
        // whole point: the build menu and the machine panel were each
        // choosing their own near-black on near-black, and the result was
        // two screens you could not read and could not compare.
        //
        // The rules the numbers encode:
        //   * A PANEL is nearly opaque. A translucent list over a planet is
        //     a list you are reading through a landscape.
        //   * A ROW is clearly lighter than the panel it sits on, and every
        //     other row is lighter still — zebra striping does more for a
        //     scanned list than any amount of border drawing.
        //   * SELECTION is a hue change, not a brightness change, because
        //     brightness is already carrying the zebra.
        //   * TEXT is near-white for the thing itself and a desaturated blue
        //     for its details — never the panel colour with the alpha turned
        //     down, which is how the old menus became unreadable.
        namespace hud
        {
            inline constexpr sw::Vec4 kPanel{0.03f, 0.05f, 0.08f, 0.96f};
            inline constexpr sw::Vec4 kEdge{0.28f, 0.48f, 0.68f, 0.95f};
            inline constexpr sw::Vec4 kHeader{0.09f, 0.15f, 0.23f, 0.98f};
            inline constexpr sw::Vec4 kRow{0.12f, 0.17f, 0.24f, 0.98f};
            inline constexpr sw::Vec4 kRowAlt{0.18f, 0.25f, 0.34f, 0.98f};
            inline constexpr sw::Vec4 kRowHover{0.24f, 0.35f, 0.48f, 1.0f};
            inline constexpr sw::Vec4 kRowOn{0.10f, 0.45f, 0.28f, 1.0f};
            inline constexpr sw::Vec4 kRowOnHover{0.16f, 0.60f, 0.37f, 1.0f};
            inline constexpr sw::Vec4 kRowStop{0.44f, 0.15f, 0.15f, 1.0f};
            inline constexpr sw::Vec4 kTitle{0.97f, 0.99f, 1.0f, 1.0f};
        /// A signed span of simulation time, at whatever unit makes it
        /// readable. Used for "how far ahead is this player", where the
        /// answer is anywhere from four seconds to four days.
        [[nodiscard]] inline std::string signedDuration(sw::f64 seconds)
        {
            const char* sign = (seconds < 0.0) ? "-" : "+";
            const sw::f64 magnitude = std::abs(seconds);
            if (magnitude >= 86400.0)
            {
                return std::format("{}{:.1f} D", sign, magnitude / 86400.0);
            }
            if (magnitude >= 3600.0)
            {
                return std::format("{}{:.1f} H", sign, magnitude / 3600.0);
            }
            if (magnitude >= 60.0)
            {
                return std::format("{}{:.0f} MIN", sign, magnitude / 60.0);
            }
            return std::format("{}{:.0f} S", sign, magnitude);
        }
            inline constexpr sw::Vec4 kText{0.91f, 0.95f, 1.0f, 1.0f};
            inline constexpr sw::Vec4 kTextDim{0.64f, 0.75f, 0.88f, 1.0f};
            inline constexpr sw::Vec4 kOk{0.42f, 0.95f, 0.55f, 1.0f};
            inline constexpr sw::Vec4 kWarn{1.0f, 0.78f, 0.30f, 1.0f};
            inline constexpr sw::Vec4 kBad{1.0f, 0.46f, 0.40f, 1.0f};

            /// One colour per building family, shown as a chip at the head of
            /// every row. A catalogue of eight machines sorts itself the
            /// moment the eye can group it without reading a word.
            [[nodiscard]] inline sw::Vec4 categoryColor(sw::factory::BuildingCategory category)
            {
                switch (category)
                {
                case sw::factory::BuildingCategory::Miner:
                    return {0.85f, 0.55f, 0.25f, 1.0f};
                case sw::factory::BuildingCategory::Refinery:
                    return {0.90f, 0.35f, 0.30f, 1.0f};
                case sw::factory::BuildingCategory::Storage:
                    return {0.55f, 0.60f, 0.66f, 1.0f};
                case sw::factory::BuildingCategory::Solar:
                    return {0.98f, 0.85f, 0.30f, 1.0f};
                case sw::factory::BuildingCategory::Battery:
                    return {0.35f, 0.85f, 0.55f, 1.0f};
                case sw::factory::BuildingCategory::Pole:
                    return {0.45f, 0.72f, 0.98f, 1.0f};
                case sw::factory::BuildingCategory::Conveyor:
                    return {0.70f, 0.70f, 0.75f, 1.0f};
                case sw::factory::BuildingCategory::Beacon:
                    return {1.0f, 0.78f, 0.28f, 1.0f};
                case sw::factory::BuildingCategory::Hub:
                    return {0.62f, 0.50f, 0.95f, 1.0f};
                case sw::factory::BuildingCategory::Assembly:
                    return {0.98f, 0.55f, 0.85f, 1.0f};
                case sw::factory::BuildingCategory::Pad:
                    return {0.40f, 0.90f, 0.92f, 1.0f};
                default:
                    return {0.60f, 0.66f, 0.74f, 1.0f};
                }
            }

            /// A power balance a player can read. "{:.0f} KW" turned the
            /// belt segment's half-kilowatt into "-0 KW", which says
            /// something false with total confidence; below half a kilowatt
            /// the honest word is PASSIVE.
            [[nodiscard]] inline std::string powerText(sw::f64 kw)
            {
                if (std::abs(kw) < 0.05)
                {
                    return "PASSIVE";
                }
                const char* sign = (kw > 0.0) ? "+" : "-";
                return (std::abs(kw) >= 10.0)
                           ? std::format("{}{:.0f} KW", sign, std::abs(kw))
                           : std::format("{}{:.1f} KW", sign, std::abs(kw));
            }

            /// Uppercased copy: the glyph font has one case, and mixing them
            /// in the source makes the widths lie.
            [[nodiscard]] inline std::string caps(std::string text)
            {
                for (char& c : text)
                {
                    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
                }
                return text;
            }
        } // namespace hud

        /// The VAB palette shows ROCKET parts. Since F1 the catalogue also
        /// holds buildings — same file format, same Part Studio, same stable
        /// id space — and those belong to the ground build mode (F2), not to
        /// a vessel. One filter, used by both the row layout and the click
        /// handler, so an index can never mean two different parts.
        [[nodiscard]] inline std::vector<const sw::parts::PartDefinition*> rocketPartPalette()
        {
            std::vector<const sw::parts::PartDefinition*> palette;
            for (const sw::parts::PartDefinition& definition : sw::parts::catalog())
            {
                if (sw::parts::isVesselPart(definition))
                {
                    palette.push_back(&definition);
                }
            }
            return palette;
        }

        // ---- planetary surface palettes (per-vertex, deterministic) -----------
        enum class SurfaceStyle
        {
            Terra,
            Luna,
            Mars,
        };

        /// CPU TWIN of planetBiome() in Shaders/PlanetSurface.glsl.
        ///
        /// The far LODs carry this palette in their vertex colors; inside 4
        /// body radii the renderer swaps to the per-fragment path, which runs
        /// the same formulas at pixel resolution. Both are driven by the M25
        /// heightfield and by a SLOPE measured at their own sampling scale,
        /// so the swap changes SHARPNESS and nothing else — a mountain does
        /// not move, a coast does not shift, no color pops.
        /// Colours one globe vertex.
        ///
        /// `frequencyLimit` is the highest noise frequency this mesh can
        /// actually REPRESENT — roughly rings / 2pi, the Nyquist limit of the
        /// vertex spacing. It matters more than it sounds: the palette is
        /// full of terms sampled at frequency 11, 42 and higher, and on the
        /// far LODs (the lowest is a 96-vertex sphere) those are not detail,
        /// they are one random number per vertex, smeared across enormous
        /// triangles by Gouraud. That is what made Luna look like it had
        /// weather. Each term is faded toward its mean as it approaches the
        /// limit, so a mesh only ever carries the frequencies it can hold.
        inline void colorizeSurfaceVertex(sw::Vertex& vertex, SurfaceStyle style,
                                   const sw::Vec3& dir, sw::f32 elevation,
                                   sw::f32 slope,
                                   const sw::planet::TerrainComponent& terrain,
                                   sw::f32 frequencyLimit)
        {
            using sw::math::smoothstepf;
            // 1 where the mesh resolves this frequency comfortably, 0 where
            // it would only alias.
            const auto resolve = [frequencyLimit](sw::f32 frequency) {
                return 1.0f -
                       smoothstepf(frequencyLimit * 0.5f, frequencyLimit, frequency);
            };
            const sw::f32 detail =
                (fbm3(dir * 42.0f, 3, 90210u) - 0.5f) * resolve(42.0f);
            const sw::f32 relief =
                glm::clamp(elevation / terrain.amplitude, 0.0f, 1.0f);
            const sw::f32 latitude = std::abs(dir.y);
            const sw::f32 rock = smoothstepf(0.025f, 0.09f, slope);
            const sw::f32 land = sw::planet::terrainLandFraction(terrain, dir);
            sw::Vec3 albedo{0.5f, 0.5f, 0.5f};
            sw::Vec2 material{0.0f, 0.0f}; // uv.x = specular, uv.y = gloss

            switch (style)
            {
            case SurfaceStyle::Terra:
            {
                const sw::f32 iceEdge =
                    0.91f + 0.03f * (fbm3(dir * 6.0f, 3, 555u) - 0.5f);
                if (elevation <= 0.0f && latitude <= iceEdge)
                {
                    const sw::f32 depth = glm::clamp(
                        -elevation / std::max(terrain.oceanDepth, 1.0f), 0.0f, 1.0f);
                    const sw::f32 shelf = 1.0f - smoothstepf(0.02f, 0.30f, depth);
                    const sw::Vec3 abyss{0.008f, 0.032f, 0.105f};
                    const sw::Vec3 open{0.020f, 0.105f, 0.250f};
                    const sw::Vec3 shallows{0.065f, 0.330f, 0.410f};
                    albedo = glm::mix(glm::mix(abyss, open, 1.0f - depth), shallows,
                                      shelf * shelf) *
                             (1.0f + detail * 0.06f);
                    // Far LODs cannot resolve waves: the wide specular lobe
                    // IS the sub-pixel roughness (same reasoning as the
                    // fragment path, which fades toward this value).
                    material = {0.55f, 0.52f};
                    break;
                }

                const sw::f32 wet =
                    fbm3(dir * 1.9f + sw::Vec3{63.11f, 27.43f, 15.91f}, 3,
                         terrain.seed + 3131u);
                const sw::f32 humidity = glm::clamp(
                    wet * 1.9f - 0.30f +
                        0.40f * (1.0f - smoothstepf(0.0f, 0.30f, land)) -
                        0.35f * smoothstepf(0.12f, 0.55f, relief),
                    0.0f, 1.0f);

                const sw::Vec3 desert{0.615f, 0.505f, 0.310f};
                const sw::Vec3 steppe{0.470f, 0.425f, 0.240f};
                const sw::Vec3 grass{0.245f, 0.360f, 0.150f};
                const sw::Vec3 forest{0.120f, 0.245f, 0.105f};
                sw::Vec3 ground =
                    glm::mix(desert, steppe, smoothstepf(0.15f, 0.45f, humidity));
                ground = glm::mix(ground, grass, smoothstepf(0.42f, 0.64f, humidity));
                ground = glm::mix(ground, forest, smoothstepf(0.64f, 0.86f, humidity));

                const sw::f32 beach =
                    (1.0f - smoothstepf(0.0f, 70.0f, elevation)) * (1.0f - rock);
                ground = glm::mix(ground, sw::Vec3{0.720f, 0.660f, 0.460f}, beach);
                ground = glm::mix(ground, sw::Vec3{0.420f, 0.360f, 0.245f},
                                  smoothstepf(0.10f, 0.42f, relief));
                const sw::Vec3 stone =
                    sw::Vec3{0.380f, 0.350f, 0.320f} * (1.0f + detail * 0.28f);
                ground = glm::mix(ground, stone, rock);

                const sw::f32 snowLine = std::max(
                    0.02f, 0.75f * (1.0f - 1.05f * latitude * latitude) - 0.04f * humidity);
                const sw::f32 snow =
                    smoothstepf(snowLine, snowLine + 0.10f, relief + detail * 0.05f) *
                    (1.0f - smoothstepf(0.07f, 0.16f, slope));
                albedo = glm::mix(ground * (1.0f + detail * 0.22f),
                                  sw::Vec3{0.90f, 0.92f, 0.95f}, snow);
                const sw::f32 ice = smoothstepf(iceEdge, iceEdge + 0.015f, latitude);
                albedo = glm::mix(albedo,
                                  sw::Vec3{0.92f, 0.94f, 0.97f} * (1.0f + detail * 0.12f),
                                  ice);
                material = {0.30f * std::max(snow, ice), 0.45f * std::max(snow, ice)};
                break;
            }
            case SurfaceStyle::Luna:
            {
                // Maria over cratered highlands. The shore between them used
                // to be a hard `m < 0.47` step, which on a fractal field
                // draws a crisp wandering edge — from a distance that reads
                // as a weather front, not as a basalt plain. Real maria have
                // soft margins; so does this one now.
                const sw::f32 m =
                    fbm3(dir * 3.1f + sw::Vec3{2.9f, 8.1f, 0.4f}, 4, 4242u);
                const sw::f32 maria =
                    smoothstepf(0.435f, 0.515f, m) * resolve(3.1f) + 0.5f *
                                                                     (1.0f -
                                                                      resolve(3.1f));
                const sw::f32 fine =
                    (fbm3(dir * 11.0f, 3, 4343u) - 0.5f) * resolve(11.0f);
                sw::f32 g = glm::mix(0.235f, 0.415f, maria) + 0.10f * fine +
                            detail * 0.08f + relief * 0.10f;
                g = glm::mix(g, g * 1.18f + 0.03f, rock);
                albedo = {g, g, g * 1.04f};
                break;
            }
            case SurfaceStyle::Mars:
            {
                const sw::f32 capEdge =
                    0.93f + 0.02f * (fbm3(dir * 5.0f, 3, 771u) - 0.5f);
                const sw::Vec3 lowlands{0.360f, 0.170f, 0.090f};
                const sw::Vec3 highlands{0.660f, 0.360f, 0.180f};
                const sw::Vec3 dust{0.720f, 0.520f, 0.330f};
                sw::Vec3 ground =
                    glm::mix(lowlands, highlands, smoothstepf(0.02f, 0.45f, relief));
                ground = glm::mix(ground, dust, smoothstepf(0.45f, 0.85f, relief));
                ground = glm::mix(ground, sw::Vec3{0.230f, 0.150f, 0.110f}, rock);
                albedo = ground * (1.0f + detail * 0.24f);
                const sw::f32 cap = smoothstepf(capEdge, capEdge + 0.015f, latitude);
                albedo = glm::mix(albedo,
                                  sw::Vec3{0.90f, 0.88f, 0.86f} * (1.0f + detail * 0.10f),
                                  cap);
                material = {cap * 0.25f, cap * 0.40f};
                break;
            }
            }

            vertex.color = {albedo.r, albedo.g, albedo.b, 1.0f};
            vertex.uv = material;
        }

        // ---- static starfield: parallax-free orientation reference ------------
        // M21: denser sky with a power-law brightness distribution, color
        // temperatures, a GALACTIC BAND (stars concentrated along a fixed
        // great circle) and a few faint nebulosity glows inside it.
        inline constexpr sw::f32 kStarDomeRadius = 1.0e12f; // inside the 1e13 far plane
        inline constexpr sw::u32 kStarCount = 3400;

        [[nodiscard]] inline sw::MeshData buildStarfieldMesh()
        {
            sw::MeshData mesh;
            mesh.vertices.reserve(kStarCount * 6 + 16 * 10);
            mesh.indices.reserve(kStarCount * 24 + 16 * 24);
            sw::u32 seed = 0xC0FFEEu; // FIXED seed: the sky never changes

            // The Milky Way plane: a fixed, slightly tilted great circle.
            const sw::Vec3 bandNormal = glm::normalize(sw::Vec3{0.22f, 0.94f, 0.26f});
            const sw::Vec3 bandU = glm::normalize(glm::cross(bandNormal, sw::Vec3{0, 0, 1}));
            const sw::Vec3 bandV = glm::cross(bandNormal, bandU);

            const auto pushOctahedron = [&mesh](const sw::Vec3& dir, sw::f32 size,
                                                const sw::Vec4& color) {
                const sw::u32 base = static_cast<sw::u32>(mesh.vertices.size());
                const sw::Vec3 axes[6] = {{size, 0, 0},  {-size, 0, 0}, {0, size, 0},
                                          {0, -size, 0}, {0, 0, size},  {0, 0, -size}};
                for (const sw::Vec3& axis : axes)
                {
                    mesh.vertices.push_back({dir + axis, dir, color, {}});
                }
                const sw::u32 tris[8][3] = {{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4},
                                            {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5}};
                for (const auto& tri : tris)
                {
                    mesh.indices.insert(mesh.indices.end(),
                                        {base + tri[0], base + tri[1], base + tri[2]});
                }
            };

            for (sw::u32 star = 0; star < kStarCount; ++star)
            {
                sw::Vec3 dir;
                const bool inBand = hash01(seed++) < 0.42f;
                if (inBand)
                {
                    // Dense along the band, thin gaussian-ish spread across it.
                    // The two spread samples are drawn in separate statements:
                    // two `seed++` in one expression are unsequenced (UB), and
                    // GCC and MSVC ordered them differently — two compilers,
                    // two skies.
                    const sw::f32 along = 6.2831853f * hash01(seed++);
                    const sw::f32 spreadA = hash01(seed++);
                    const sw::f32 spreadB = hash01(seed++);
                    const sw::f32 spread = (spreadA + spreadB - 1.0f) * 0.22f;
                    dir = glm::normalize(bandU * std::cos(along) +
                                         bandV * std::sin(along) + bandNormal * spread);
                }
                else
                {
                    const sw::f32 z = 2.0f * hash01(seed++) - 1.0f;
                    const sw::f32 phi = 6.2831853f * hash01(seed++);
                    const sw::f32 r = std::sqrt(std::max(0.0f, 1.0f - z * z));
                    dir = {r * std::cos(phi), z, r * std::sin(phi)};
                }

                // Power-law brightness: many faint stars, a handful of beacons.
                const sw::f32 magnitude = hash01(seed++);
                const sw::f32 power = magnitude * magnitude * magnitude;
                const sw::f32 size = 0.0005f + 0.0022f * power;
                const sw::f32 warm = hash01(seed++);
                const sw::f32 brightness =
                    (0.28f + 0.72f * power) * (inBand ? 0.85f : 1.0f);
                // Color temperature: blue-white .. white .. warm .. orange-red.
                sw::Vec3 tempColor;
                if (warm < 0.55f)
                {
                    tempColor = glm::mix(sw::Vec3{0.75f, 0.83f, 1.0f},
                                         sw::Vec3{1.0f, 1.0f, 1.0f}, warm / 0.55f);
                }
                else
                {
                    tempColor = glm::mix(sw::Vec3{1.0f, 1.0f, 1.0f},
                                         sw::Vec3{1.0f, 0.72f, 0.45f},
                                         (warm - 0.55f) / 0.45f);
                }
                pushOctahedron(dir, size,
                               {tempColor.r * brightness, tempColor.g * brightness,
                                tempColor.b * brightness, 1.0f});
            }

            return mesh;
        }

        // ---- sun glow: radial-falloff emissive discs (billboarded) -------------
        /// Vertex alpha rides the emissive encoding: center 2.0 (opaque
        /// self-lit) fading to 1.0 (fully transparent) at the rim.
        [[nodiscard]] inline sw::MeshData buildGlowDiscMesh(const sw::Vec3& centerColor,
                                                     const sw::Vec3& rimColor,
                                                     sw::f32 centerAlpha,
                                                     sw::f32 rimAlpha = 1.0f)
        {
            sw::MeshData mesh;
            const sw::Vec3 normal{0.0f, 0.0f, 1.0f};
            mesh.vertices.push_back(
                {{0.0f, 0.0f, 0.0f}, normal,
                 {centerColor.r, centerColor.g, centerColor.b, centerAlpha}, {}});
            constexpr sw::u32 kSegments = 40;
            for (sw::u32 i = 0; i <= kSegments; ++i)
            {
                const sw::f32 a = 6.2831853f * static_cast<sw::f32>(i) / kSegments;
                mesh.vertices.push_back(
                    {{std::cos(a), std::sin(a), 0.0f}, normal,
                     {rimColor.r, rimColor.g, rimColor.b, rimAlpha}, {}});
            }
            for (sw::u32 i = 1; i <= kSegments; ++i)
            {
                mesh.indices.insert(mesh.indices.end(), {0u, i, i + 1});
            }
            return mesh;
        }

        // ---- atmosphere & cloud shells (transparent pass) ----------------------
        [[nodiscard]] inline sw::MeshData buildAtmosphereShellMesh()
        {
            sw::MeshData mesh = sw::PrimitiveFactory::makeUvSphere(
                1.0f, 40, 60, {0.36f, 0.56f, 0.92f, 1.0f});
            // Slightly stronger veil near the poles-of-view is impossible
            // without shaders; a uniform low alpha reads well from both
            // orbit (blue limb) and the ground (blue sky dome).
            for (sw::Vertex& vertex : mesh.vertices)
            {
                vertex.color.a = 0.30f;
            }
            return mesh;
        }

        [[nodiscard]] inline sw::MeshData buildCloudShellMesh()
        {
            // M28: a bare white shell. Coverage, edges, layers, drift, polar
            // fade and thickness are all decided PER FRAGMENT by
            // Shaders/Clouds.glsl — the vertex path could only ever carry
            // blurred blobs, and could not be sampled by the ground for
            // shadows.
            return sw::PrimitiveFactory::makeUvSphere(1.0f, 56, 84,
                                                      {1.0f, 1.0f, 1.0f, 1.0f});
        }
} // namespace game
