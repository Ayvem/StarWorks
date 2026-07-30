#include "StarWorksGame.hpp"

#include "Systems.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <format>

namespace game
{
    namespace
    {
        /// The autopilot mode, for the log and the HUD. One place, so a mode
        /// added later cannot be printed as "RETROGRADE" by a stale ternary.
        [[nodiscard]] const char* sasModeName(sw::u32 mode)
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
        constexpr sw::f64 kMuSol = 1.32712440018e20;
        constexpr sw::f64 kSolRadius = 6.9634e8;
        constexpr sw::f64 kTerraRadius = 6.371e6;    // Earth
        constexpr sw::f64 kMuTerra = 3.986004418e14; // Earth GM
        constexpr sw::f64 kTerraSma = 1.496e11;      // 1 AU
        constexpr sw::f64 kLunaRadius = 1.7374e6;    // Moon
        constexpr sw::f64 kMuLuna = 4.9048695e12;    // Moon GM
        constexpr sw::f64 kLunaSma = 3.844e8;        // Earth-Moon distance
        constexpr sw::f64 kMarsRadius = 3.3895e6;
        constexpr sw::f64 kMuMars = 4.2828e13;
        constexpr sw::f64 kMarsSma = 2.2794e11;
        // Sphere-of-influence radii: r = a * (mu / mu_parent)^(2/5).
        constexpr sw::f64 kTerraSoi = 9.24e8;
        constexpr sw::f64 kLunaSoi = 6.61e7;
        constexpr sw::f64 kMarsSoi = 5.77e8;

        constexpr sw::f64 kStationAltitude = 4.0e5; // 400 km (LEO)
        constexpr sw::f64 kStationOrbitRadius = kTerraRadius + kStationAltitude;
        constexpr sw::f64 kStationPhase = 4.71238898038468986; // 3*pi/2
        /// Terra sidereal angular velocity (rad/s) around +Y — used for the
        /// SuRFace-relative speed readout.
        constexpr sw::WorldVec3 kTerraAngularVelocity{0.0, 7.2921e-5, 0.0};


        constexpr sw::f64 kBubbleEnterRadius = 1.0e4; // 10 km
        constexpr sw::f64 kBubbleExitRadius = 1.5e4;  // 15 km (hysteresis)

        // Star map: constant on-screen marker size and zoom limits (up to
        // the full Sol system — Mars orbit is 2.28e11 m).
        constexpr sw::f32 kMarkerScreenFraction = 0.016f;
        constexpr sw::f64 kMapMinHeight = 2.0e7;
        constexpr sw::f64 kMapMaxHeight = 8.0e11;
        // Line segments, not dots: a chord every degree and a half reads as
        // a smooth curve at any zoom the map allows, and the flight plan
        // gets more of them because it is the line being read.
        constexpr sw::u32 kTrajectorySamples = 240;
        constexpr sw::u32 kPredictionDisplaySamples = 320;
        /// Patch colors: current conic, then each successive patch (KSP
        /// style — the eye follows the hand-offs by color).
        constexpr sw::Vec4 kPatchColors[] = {
            {0.35f, 1.0f, 0.55f, 2.0f},  // green: current orbit
            {1.0f, 0.85f, 0.25f, 2.0f},  // yellow: next patch
            {0.95f, 0.45f, 1.0f, 2.0f},  // magenta
            {0.35f, 0.8f, 1.0f, 2.0f},   // cyan
            {1.0f, 0.55f, 0.25f, 2.0f},  // orange
        };
        /// How often the flight plan is recomputed (wall seconds).
        constexpr sw::f64 kPredictionRefreshSeconds = 0.25;

        // ---- artificial horizon (navball) ------------------------------------
        constexpr sw::f32 kNavballCenterY = 0.62f; // NDC, y grows downward
        constexpr sw::f32 kNavballRadius = 0.26f;  // NDC (vertical)
        constexpr sw::f32 kHalfPi = 1.5707963267948966f;

        // ---- reentry heating ---------------------------------------------------
        /// Heating proxy q = rho * v_rel^3 (W/m^2-ish). Glow ramps over
        /// [1e7, 1e9] on a log scale: faint at ~100 km on a LEO reentry,
        /// blinding below ~55 km.
        constexpr sw::f64 kHeatGlowStart = 1.0e7;
        constexpr sw::f32 kHeatLogRange = 2.0f;
        constexpr sw::usize kMaxParticles = 320;

        [[nodiscard]] sw::MeshData buildNavRingMesh(sw::u32 segments, sw::f32 thickness)
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

        [[nodiscard]] sw::MeshData buildNavBarMesh()
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

        [[nodiscard]] sw::MeshData buildNavDiamondMesh()
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
        constexpr sw::f32 kWarpLadder[] = {1.0f,      2.0f,      5.0f,       10.0f,
                                           50.0f,     100.0f,    1000.0f,    10000.0f,
                                           100000.0f, 1000000.0f, 10000000.0f};
        constexpr sw::u32 kWarpSteps = static_cast<sw::u32>(std::size(kWarpLadder));

        /// The warp rate as a pilot reads it. "1E+07" is a number a compiler
        /// prints; X10M is a number a person reads.
        [[nodiscard]] std::string warpText(sw::f32 rate)
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
    constexpr sw::u32 kNetEventBeacon = 1;

    constexpr sw::f32 kMaxPhysicsWarp = 5.0f;

        [[nodiscard]] sw::f32 maxWarpForAltitude(sw::f64 altitudeMeters)
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
        constexpr sw::u32 kLodRings[CelestialLodComponent::kLodLevels] = {150, 80, 32, 14, 8};
        constexpr sw::u32 kLodSegments[CelestialLodComponent::kLodLevels] = {225, 120, 48, 21, 12};
        constexpr sw::f32 kLodScreenFractions[CelestialLodComponent::kLodLevels - 1] = {
            0.5f, 0.15f, 0.04f, 0.008f};

        constexpr const char* kGlyphCharset = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,-+/%:";

        [[nodiscard]] sw::MeshData buildMarkerMesh()
        {
            // Markers are drawn EMISSIVE (tint alpha 2.0) — normals unused.
            return sw::PrimitiveFactory::makeOctahedron(1.0f, {1.0f, 1.0f, 1.0f, 1.0f});
        }

        sw::f32 hash01(sw::u32 x)
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
        [[nodiscard]] const sw::Vec3& terraStartSite()
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
        [[nodiscard]] sw::Quat standUpFor(const sw::Vec3& up)
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
        [[nodiscard]] sw::f32 yawToFace(const sw::Vec3& up, const sw::Vec3& modelDirection,
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
        [[nodiscard]] sw::Vec3 resourceCargoColor(sw::res::Resource resource)
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
            constexpr sw::Vec4 kPanel{0.03f, 0.05f, 0.08f, 0.96f};
            constexpr sw::Vec4 kEdge{0.28f, 0.48f, 0.68f, 0.95f};
            constexpr sw::Vec4 kHeader{0.09f, 0.15f, 0.23f, 0.98f};
            constexpr sw::Vec4 kRow{0.12f, 0.17f, 0.24f, 0.98f};
            constexpr sw::Vec4 kRowAlt{0.18f, 0.25f, 0.34f, 0.98f};
            constexpr sw::Vec4 kRowHover{0.24f, 0.35f, 0.48f, 1.0f};
            constexpr sw::Vec4 kRowOn{0.10f, 0.45f, 0.28f, 1.0f};
            constexpr sw::Vec4 kRowOnHover{0.16f, 0.60f, 0.37f, 1.0f};
            constexpr sw::Vec4 kRowStop{0.44f, 0.15f, 0.15f, 1.0f};
            constexpr sw::Vec4 kTitle{0.97f, 0.99f, 1.0f, 1.0f};
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
            constexpr sw::Vec4 kText{0.91f, 0.95f, 1.0f, 1.0f};
            constexpr sw::Vec4 kTextDim{0.64f, 0.75f, 0.88f, 1.0f};
            constexpr sw::Vec4 kOk{0.42f, 0.95f, 0.55f, 1.0f};
            constexpr sw::Vec4 kWarn{1.0f, 0.78f, 0.30f, 1.0f};
            constexpr sw::Vec4 kBad{1.0f, 0.46f, 0.40f, 1.0f};

            /// One colour per building family, shown as a chip at the head of
            /// every row. A catalogue of eight machines sorts itself the
            /// moment the eye can group it without reading a word.
            [[nodiscard]] sw::Vec4 categoryColor(sw::factory::BuildingCategory category)
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
            [[nodiscard]] std::string powerText(sw::f64 kw)
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
            [[nodiscard]] std::string caps(std::string text)
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
        [[nodiscard]] std::vector<const sw::parts::PartDefinition*> rocketPartPalette()
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
        void colorizeSurfaceVertex(sw::Vertex& vertex, SurfaceStyle style,
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
        constexpr sw::f32 kStarDomeRadius = 1.0e12f; // inside the 1e13 far plane
        constexpr sw::u32 kStarCount = 3400;

        [[nodiscard]] sw::MeshData buildStarfieldMesh()
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
                    const sw::f32 along = 6.2831853f * hash01(seed++);
                    const sw::f32 spread =
                        (hash01(seed++) + hash01(seed++) - 1.0f) * 0.22f;
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
        [[nodiscard]] sw::MeshData buildGlowDiscMesh(const sw::Vec3& centerColor,
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
        [[nodiscard]] sw::MeshData buildAtmosphereShellMesh()
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

        [[nodiscard]] sw::MeshData buildCloudShellMesh()
        {
            // M28: a bare white shell. Coverage, edges, layers, drift, polar
            // fade and thickness are all decided PER FRAGMENT by
            // Shaders/Clouds.glsl — the vertex path could only ever carry
            // blurred blobs, and could not be sampled by the ground for
            // shadows.
            return sw::PrimitiveFactory::makeUvSphere(1.0f, 56, 84,
                                                      {1.0f, 1.0f, 1.0f, 1.0f});
        }
    } // namespace

    StarWorksGame::StarWorksGame(const sw::ApplicationConfig& config)
        : sw::Application(config)
        , m_cameraController(m_camera)
    {
        // Far planes sized for the full system: the Sun must render from
        // Mars (3.8e11 m away when opposed). Reverse-Z keeps the precision.
        m_camera.setPerspective(sw::math::toRadians(60.0f), 0.5f, 1.0e13f);
        // Shading tier (M26): HIGH on a real GPU, LOW under a software
        // rasterizer. It gates the per-fragment planet path's octave budget
        // and its terrain self-shadowing march.
        renderer().setQuality(config.renderQuality);
        m_mapCamera.setPerspective(sw::math::toRadians(60.0f), 1.0e5f, 2.0e12f);
        m_glyphMeshIndex.fill(0xFFFFFFFFu);

        // DATA-DRIVEN PARTS: the shipped .swpart files replace the built-in
        // fallback before any mesh or vessel is built. Part Studio edits
        // these same files.
        sw::parts::loadCatalog(sw::FileSystem::executableDirectory() / "Assets" / "Parts");
        // DATA-DRIVEN AERODYNAMICS (F6): every part's `.aero.json` sidecar,
        // solved offline by Tools/AeroForge over the same geometry. A part
        // with no table simply produces no aerodynamic force — which is why
        // buildings do not need one.
        sw::aero::loadTables(sw::FileSystem::executableDirectory() / "Assets" / "Parts");
        // DATA-DRIVEN INDUSTRY (F1): the production chains are .swrecipe
        // files on the same contract — stable ids, a built-in fallback, and
        // a loader that refuses any recipe which would create matter.
        sw::factory::loadRecipeCatalog(sw::FileSystem::executableDirectory() / "Assets" /
                                       "Recipes");
        // DATA-DRIVEN DESIGNS (F5): every .swship in Assets/Ships is a rocket
        // somebody drew in the hangar, and every one of them is orderable at
        // a VAB. Failing to find any is not an error — a new save has none —
        // so the catalogue simply stays empty until the player saves one.
        sw::parts::loadBlueprintCatalog(sw::FileSystem::executableDirectory() /
                                        "Assets" / "Ships");

        buildScene();
        buildGlyphMeshes();
        buildNavballMeshes();
        m_hangarFloorMeshIndex = registerMesh(renderer().createMesh(
            sw::PrimitiveFactory::makeGridPlane(40.0f, 20, {0.2f, 0.3f, 0.38f, 1.0f})));
        m_hangarCamera.setPerspective(sw::math::toRadians(55.0f), 0.2f, 500.0f);
        buildSaveSchema();
        m_celestialIndex.rebuild(m_world);

        // The FREE camera's parking spot (Tab). It used to look at the
        // orbital station, which no longer exists; it now sits above the
        // outpost the player is standing in, so pressing Tab on the first
        // frame does not fly the view to empty space.
        {
            const sw::Vec3 siteUp = terraStartSite();
            const sw::WorldVec3 terraCentre =
                m_world.getComponent<TransformComponent>(m_terraEntity).position;
            m_cameraController.setPose(
                terraCentre + sw::WorldVec3(siteUp) * (kTerraRadius + 220.0), 0.0f,
                -0.35f);
        }

        // ---- simulation lanes ---------------------------------------------------
        m_physicsLane = m_simulation.findLane("Physics");
        m_physicsLane->setStrictCatchUp(true); // never desync live physics
        auto& physics = m_physicsLane->scheduler();
        // Celestials move FIRST: everything else this tick (gravity, rails,
        // anchors) reads their up-to-date positions/velocities.
        physics.addSystem(
            std::make_unique<sw::space::CelestialMotionSystem>(*m_physicsLane));
        // ...and they TURN first, for the same reason. The spin is analytic,
        // so it can be evaluated the instant the tick's present time is
        // known — and everything downstream that samples the ground reads
        // the body's rotating frame. Running it late (it used to sit after
        // the surface systems) left ground CONTACT sampling the heightfield
        // through a one-tick-stale attitude: 1.46e-6 rad, which on Terra is
        // 9.3 m of ground. Flat ground did not care. On slopes steeper than
        // 0.25 that offset is worth up to 13.6 m of elevation, which is
        // exactly how you walk into a mountainside.
        physics.addSystem(std::make_unique<CelestialSpinSystem>(*m_physicsLane));
        physics.addSystem(std::make_unique<SnapshotSystem>());
        // Parts -> vessel aggregates (mass falls as fuel burns).
        physics.addSystem(std::make_unique<sw::parts::VesselAssemblySystem>());
        // Rails ride at Physics rate too: their primaries move ~30 km/s, so
        // a 10 Hz refresh would visibly step (the closed-form solve is cheap).
        physics.addSystem(std::make_unique<sw::phys::RailsSystem>(*m_physicsLane));
        physics.addSystem(std::make_unique<sw::phys::GravityIntegrationSystem>());
        physics.addSystem(std::make_unique<SasSystem>()); // before Thrust: it commands
        physics.addSystem(std::make_unique<ThrustSystem>());
        // AERODYNAMICS, after thrust and before the ground. Thrust first,
        // because a rocket's own acceleration is part of the flow it meets
        // this tick; the ground last, because whatever the air did to a
        // vehicle about to land must not survive the touchdown.
        {
            auto aerodynamics = std::make_unique<sw::aero::VesselAerodynamicsSystem>();
            m_aerodynamics = aerodynamics.get();
            physics.addSystem(std::move(aerodynamics));
        }
        sw::phys::SurfaceInteractionSystem::Config surfaceConfig{};
        physics.addSystem(
            std::make_unique<sw::phys::SurfaceInteractionSystem>(surfaceConfig));
        physics.addSystem(std::make_unique<CapsuleMovementSystem>());
        physics.addSystem(std::make_unique<SpinSystem>());
        // Atmosphere/cloud shells follow their planet (own drift spin).
        physics.addSystem(std::make_unique<CloudLayerSystem>());
        // After the celestial spin: surface bases co-rotate with their body.
        physics.addSystem(std::make_unique<sw::phys::SurfaceAnchorSystem>());
        // Parts ride their vessel (lockstep interpolation), last.
        physics.addSystem(std::make_unique<sw::parts::PartAttachmentSystem>());
        // ...and only NOW is every solid thing where it is going to be, so
        // this is the one place the walker can be pushed back out of what it
        // walked into. Before the anchors and the part attachment it would
        // be resolving against last tick's building positions, which on a
        // spinning planet is 595 m of lie.
        m_hullCollision = new sw::phys::HullCollisionSystem();
        physics.addSystem(
            std::unique_ptr<sw::phys::HullCollisionSystem>(m_hullCollision));

        auto& automation = m_simulation.findLane("Automation")->scheduler();
        automation.addSystem(std::make_unique<SolarChargeSystem>());
        // F3 — THE GRID RUNS FIRST. Every building's `satisfaction` for this
        // tick is decided here, from the real sun and the site's batteries,
        // and the executor below spends it. The other order would run the
        // factory on last tick's weather.
        automation.addSystem(std::make_unique<sw::factory::PowerGridSystem>(m_solEntity));
        // The generic recipe executor: ONE system for every building the
        // player will ever place. The two below stay for the asteroid rig
        // and the orbital station, which are craft, not buildings.
        automation.addSystem(std::make_unique<sw::factory::ProductionSystem>());
        // ...and the assembly hall, which is the executor's sibling: same
        // lane, same power, same bulk catch-up, but its bill of materials
        // comes from a design the player drew rather than from a recipe.
        automation.addSystem(std::make_unique<sw::factory::AssemblySystem>());
        automation.addSystem(std::make_unique<sw::factory::MinerSystem>());
        automation.addSystem(std::make_unique<sw::factory::RefinerySystem>());

        auto& logistics = m_simulation.findLane("Logistics")->scheduler();
        logistics.addSystem(std::make_unique<sw::factory::TransferSystem>());
        sw::phys::SimulationBubbleSystem::Config bubbleConfig{};
        bubbleConfig.enterRadius = kBubbleEnterRadius;
        bubbleConfig.exitRadius = kBubbleExitRadius;
        auto bubble = std::make_unique<sw::phys::SimulationBubbleSystem>(
            m_commands, *m_physicsLane, bubbleConfig);
        m_bubbleSystem = bubble.get();
        logistics.addSystem(std::move(bubble));

        m_simulation.findLane("World")->scheduler().addSystem(
            std::make_unique<StatsSystem>());

        SW_LOG_INFO("Game", "Milestone 10 scene ready: {} entities", m_world.aliveCount());
        SW_LOG_INFO("Game",
                    "Controls: Tab pilot/free | G EVA (first person: mouse turns you, "
                    "A/D strafe) | F build menu; LCLICK build, WHEEL rotate, R demolish; "
                    "belts: LCLICK output then input | B hangar | M map | P next ship | V "
                    "speed ORB/SRF | Shift/Ctrl throttle | ,/. warp | W/S A/D arrows Q/E "
                    "X | Space pause | Esc quit");
    }

    sw::u32 StarWorksGame::registerMesh(sw::Mesh mesh)
    {
        m_meshes.push_back(std::move(mesh));
        return static_cast<sw::u32>(m_meshes.size() - 1);
    }

    CelestialLodComponent StarWorksGame::makeSphereLodSet(const sw::Vec4& color,
                                                          sw::i32 surfaceStyle)
    {
        // RELIEF SHADING (M22): the globe's vertex normals are tilted by
        // the gradient of the SAME analytic heightfield physics collides
        // with — mountain ranges catch the light and throw shadow flanks
        // from orbit, and they are exactly where the terrain patch will
        // put them when you land. Ocean stays flat (elevation clamps to 0,
        // gradient vanishes) and keeps its mirror specular.
        sw::planet::TerrainComponent terrain{};
        sw::f64 bodyRadius = 0.0;
        bool hasRelief = false;
        if (surfaceStyle >= 0)
        {
            switch (static_cast<SurfaceStyle>(surfaceStyle))
            {
            case SurfaceStyle::Terra:
                terrain = presetTerra();
                bodyRadius = kTerraRadius;
                hasRelief = true;
                break;
            case SurfaceStyle::Luna:
                terrain = presetLuna();
                bodyRadius = kLunaRadius;
                hasRelief = true;
                break;
            case SurfaceStyle::Mars:
                terrain = presetMars();
                bodyRadius = kMarsRadius;
                hasRelief = true;
                break;
            }
        }

        CelestialLodComponent lod{};
        lod.surfaceStyle = surfaceStyle;
        for (sw::u32 level = 0; level < CelestialLodComponent::kLodLevels; ++level)
        {
            sw::MeshData sphere = sw::PrimitiveFactory::makeUvSphere(
                1.0f, kLodRings[level], kLodSegments[level], color);
            if (surfaceStyle < 0)
            {
                lod.meshIndex[level] = registerMesh(renderer().createMesh(sphere));
                continue;
            }

            // ONE pass per vertex (M25): the elevation is sampled once and
            // feeds BOTH the palette and the relief normal. The v2
            // heightfield costs ~22 noise samples per point — paying it
            // twice per vertex, on five LODs, would be a visible hitch at
            // world build time.
            //
            // Only the two closest LODs carry relief normals: farther ones
            // subtend a handful of pixels and the sampling would be wasted.
            // Slopes are physically tiny (9 km over thousands of km); an
            // exaggeration factor makes them READ from orbit without moving
            // a single vertex (silhouette and collision stay exact).
            constexpr sw::f32 kSlopeExaggeration = 220.0f;
            const sw::f32 epsilon = 0.004f; // ~25 km sampling arc on Terra
            const bool reliefNormals = hasRelief && level < 2;
            // A 25 km arc cannot resolve a 2 km ridge: sampling the full
            // octave stack here would only alias between vertices. Level 0
            // takes six relief octaves, level 1 five, the rest four.
            const sw::i32 reliefOctavesForLevel =
                std::min(terrain.reliefOctaves,
                         (level == 0) ? 6 : ((level == 1) ? 5 : 4));
            const auto style = static_cast<SurfaceStyle>(surfaceStyle);
            // Nyquist for this tessellation: rings / 2pi. Nothing finer than
            // this may reach the palette, or the mesh turns a texture into
            // noise (see colorizeSurfaceVertex).
            const sw::f32 frequencyLimit =
                static_cast<sw::f32>(kLodRings[level]) / 6.2831853f;
            for (sw::Vertex& vertex : sphere.vertices)
            {
                const sw::Vec3 dir = glm::normalize(vertex.position);
                // Far LODs sample fewer relief octaves — exactly the LOD the
                // shader applies by screen footprint, so the vertex path and
                // the fragment path meet in the middle instead of popping.
                const sw::f32 elevation =
                    hasRelief ? sw::planet::terrainElevationSignedLod(
                                    terrain, dir, reliefOctavesForLevel)
                              : 0.0f;

                sw::f32 slope = 0.0f;
                sw::f32 slopeA = 0.0f;
                sw::f32 slopeB = 0.0f;
                sw::Vec3 tangentA{0.0f};
                sw::Vec3 tangentB{0.0f};
                if (reliefNormals && elevation > 0.0f)
                {
                    const sw::Vec3 reference = std::abs(dir.y) < 0.95f
                                                   ? sw::Vec3{0, 1, 0}
                                                   : sw::Vec3{1, 0, 0};
                    tangentA = glm::normalize(glm::cross(reference, dir));
                    tangentB = glm::cross(dir, tangentA);
                    slopeA = (sw::planet::terrainElevationSignedLod(
                                  terrain, glm::normalize(dir + tangentA * epsilon),
                                  reliefOctavesForLevel) -
                              elevation) /
                             static_cast<sw::f32>(bodyRadius * epsilon);
                    slopeB = (sw::planet::terrainElevationSignedLod(
                                  terrain, glm::normalize(dir + tangentB * epsilon),
                                  reliefOctavesForLevel) -
                              elevation) /
                             static_cast<sw::f32>(bodyRadius * epsilon);
                    slope = glm::length(sw::Vec2{slopeA, slopeB});
                }

                colorizeSurfaceVertex(vertex, style, dir, elevation, slope, terrain,
                                      frequencyLimit);

                if (reliefNormals && elevation > 0.0f)
                {
                    vertex.normal = glm::normalize(
                        dir - (tangentA * slopeA + tangentB * slopeB) *
                                  kSlopeExaggeration);
                }
            }
            lod.meshIndex[level] = registerMesh(renderer().createMesh(sphere));
        }
        return lod;
    }

    void StarWorksGame::buildGlyphMeshes()
    {
        for (const char* c = kGlyphCharset; *c != '\0'; ++c)
        {
            const sw::MeshData glyph = sw::ui::buildGlyphMesh(*c);
            if (!glyph.empty())
            {
                m_glyphMeshIndex[static_cast<sw::usize>(*c)] =
                    registerMesh(renderer().createMesh(glyph));
            }
        }
    }

    void StarWorksGame::buildNavballMeshes()
    {
        m_navRingMeshIndex =
            registerMesh(renderer().createMesh(buildNavRingMesh(48, 0.06f)));
        m_navLineMeshIndex = registerMesh(renderer().createMesh(buildNavBarMesh()));
        m_navDiamondMeshIndex =
            registerMesh(renderer().createMesh(buildNavDiamondMesh()));
    }

    void StarWorksGame::buildScene()
    {
        // ---- meshes -------------------------------------------------------------
        // Sol's colors exceed 1.0 slightly: paired with the emissive tint it
        // reads as a glowing star, not a lit rock.
        const CelestialLodComponent solLod =
            makeSphereLodSet({1.0f, 0.92f, 0.72f, 1.0f});
        const CelestialLodComponent terraLod = makeSphereLodSet(
            {0.21f, 0.33f, 0.48f, 1.0f}, static_cast<sw::i32>(SurfaceStyle::Terra));
        const CelestialLodComponent lunaLod = makeSphereLodSet(
            {0.42f, 0.41f, 0.43f, 1.0f}, static_cast<sw::i32>(SurfaceStyle::Luna));
        const CelestialLodComponent marsLod = makeSphereLodSet(
            {0.62f, 0.32f, 0.18f, 1.0f}, static_cast<sw::i32>(SurfaceStyle::Mars));

        // Environment meshes: the fixed star dome, Terra's atmosphere veil
        // and its drifting cloud shell.
        m_starfieldMeshIndex =
            registerMesh(renderer().createMesh(buildStarfieldMesh()));
        m_sunHaloMeshIndex = registerMesh(renderer().createMesh(buildGlowDiscMesh(
            {1.0f, 0.86f, 0.62f}, {1.0f, 0.5f, 0.22f}, 1.55f)));
        m_sunCoreMeshIndex = registerMesh(renderer().createMesh(buildGlowDiscMesh(
            {1.0f, 0.99f, 0.94f}, {1.0f, 0.86f, 0.55f}, 2.0f)));
        // Soft round billboard for plasma/exhaust + a lens-flare ghost disc.
        m_particleGlowMeshIndex = registerMesh(renderer().createMesh(
            buildGlowDiscMesh({1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, 2.0f)));
        // HUD-path disc: straight alpha (center 1 -> rim 0), tint modulates.
        m_flareMeshIndex = registerMesh(renderer().createMesh(
            buildGlowDiscMesh({1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, 1.0f, 0.0f)));
        const sw::u32 atmosphereMeshId =
            registerMesh(renderer().createMesh(buildAtmosphereShellMesh()));
        const sw::u32 cloudMeshId =
            registerMesh(renderer().createMesh(buildCloudShellMesh()));

        // The F2 overlay's box: a UNIT cube (half extent 0.5), scaled to
        // each hull box. White, so the tint alone decides how it reads.
        m_hullBoxMeshIndex = registerMesh(renderer().createMesh(
            sw::PrimitiveFactory::makeCube(1.0f, {1.0f, 1.0f, 1.0f, 1.0f})));
        // Part meshes, indexed by catalog id (small ids: direct table).
        for (const sw::parts::PartDefinition& definition : sw::parts::catalog())
        {
            m_partMeshIds[definition.id] = registerMesh(
                renderer().createMesh(sw::parts::buildPartMesh(definition)));
        }
        // The BELT and its CARGO are ordinary parts. Their mesh slots and
        // their metrics are read from the catalogue rather than hard-coded,
        // so redrawing CV-1 in Part Studio — longer, wider, taller deck —
        // changes every belt in the world without touching this file.
        if (const auto* belt =
                sw::parts::findDefinition(sw::parts::kBuildingConveyor))
        {
            m_conveyorMeshIndex = m_partMeshIds.at(belt->id);
            constexpr sw::f32 kHuge = 1.0e9f;
            sw::Vec3 low{kHuge, kHuge, kHuge};
            sw::Vec3 high{-kHuge, -kHuge, -kHuge};
            sw::parts::expandPartHullBounds(*belt, sw::Vec3{0.0f},
                                                sw::Quat{1.0f, 0.0f, 0.0f, 0.0f}, low,
                                                high);
            if (low.z <= high.z)
            {
                m_conveyorSegmentM = std::max(0.1f, high.z - low.z);
                m_conveyorDeckHeightM = high.y; // cargo rides on the deck
            }
            SW_LOG_INFO("Game", "Conveyor part: {:.2f} m segment, deck at {:.2f} m",
                        m_conveyorSegmentM, m_conveyorDeckHeightM);
        }
        if (const auto* crate =
                sw::parts::findDefinition(sw::parts::kPropConveyorCrate))
        {
            m_cargoMeshIndex = m_partMeshIds.at(crate->id);
        }
        // ...and the CRADLE a finished rocket rides in. Same contract, a
        // different prop: the belt out of the VAB is meant to look like a
        // different kind of traffic, because it is.
        if (const auto* cradle =
                sw::parts::findDefinition(sw::parts::kPropVehicleCradle))
        {
            m_vehicleCargoMeshIndex = m_partMeshIds.at(cradle->id);
        }
        // The CABLE is the same story: one authored span, repeated along the
        // curve. Its length comes off its own collider, so a thicker or
        // longer CW-1 redrawn in Part Studio re-wires the whole base.
        if (const auto* wire = sw::parts::findDefinition(sw::parts::kBuildingCable))
        {
            m_cableMeshIndex = m_partMeshIds.at(wire->id);
            constexpr sw::f32 kHuge = 1.0e9f;
            sw::Vec3 low{kHuge, kHuge, kHuge};
            sw::Vec3 high{-kHuge, -kHuge, -kHuge};
            sw::parts::expandPartHullBounds(*wire, sw::Vec3{0.0f},
                                                sw::Quat{1.0f, 0.0f, 0.0f, 0.0f}, low,
                                                high);
            if (low.z <= high.z)
            {
                m_cableSegmentM = std::max(0.1f, high.z - low.z);
            }
            SW_LOG_INFO("Game", "Cable part: {:.2f} m span segment", m_cableSegmentM);
        }

        // THE PLAYER IS A PART. First person or not, you are visible to
        // yourself in the map, to a future second player, and in every
        // screenshot taken from the ship — and a capsule primitive said
        // "placeholder" in all of them. EV-1 is an ordinary .swpart prop, so
        // the suit is redrawn in Part Studio like everything else, and its
        // ground hull comes from its own hitbox rather than from a constant
        // in this file that could drift from the model.
        if (const auto* suit = sw::parts::findDefinition(sw::parts::kPropEvaSuit))
        {
            m_capsuleMeshIndex = m_partMeshIds.at(suit->id);
            constexpr sw::f32 kHuge = 1.0e9f;
            sw::Vec3 low{kHuge, kHuge, kHuge};
            sw::Vec3 high{-kHuge, -kHuge, -kHuge};
            sw::parts::expandPartHullBounds(*suit, sw::Vec3{0.0f},
                                            sw::Quat{1.0f, 0.0f, 0.0f, 0.0f}, low, high);
            if (low.y <= high.y)
            {
                m_capsuleHull.centre = (low + high) * 0.5f;
                m_capsuleHull.halfExtents = (high - low) * 0.5f;
            }
            SW_LOG_INFO("Game", "EVA suit hull: centre {:.2f} half {:.2f} x {:.2f}",
                        m_capsuleHull.centre.y, m_capsuleHull.halfExtents.x,
                        m_capsuleHull.halfExtents.y);
        }
        else
        {
            m_capsuleMeshIndex = registerMesh(renderer().createMesh(
                sw::PrimitiveFactory::makeCapsule(0.5f, 0.5f, 12, 16,
                                                  {0.9f, 0.6f, 0.2f, 1.0f})));
        }
        m_markerMeshIndex = registerMesh(renderer().createMesh(buildMarkerMesh()));
        // The trajectory line's own segment: a unit box, stretched along +Z
        // between two samples of a conic and thickened with distance so it
        // stays one pixel wide however far out the map is zoomed.
        m_orbitLineMeshIndex = registerMesh(renderer().createMesh(
            sw::PrimitiveFactory::makeBox({0.5f, 0.5f, 0.5f}, {1.0f, 1.0f, 1.0f, 1.0f})));

        // Sun position and eclipse occluders are camera-relative and set
        // every frame in onRender.

        auto snapshotOf = [](const TransformComponent& transform) {
            return PreviousTransformComponent{transform.position, transform.rotation};
        };

        // ================= THE HIERARCHY: Sol -> Terra/Mars -> Luna ==============
        // Parent-relative Kepler elements, real values. Initial world
        // positions are the analytic evaluation at t=0 — identical to what
        // the CelestialMotionSystem will compute on the first tick.

        // ---- Sol: the static root ------------------------------------------------
        {
            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{}; // world origin
            transform.uniformScale = static_cast<sw::f32>(kSolRadius);
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{1.0f});
            m_world.addComponent(e, solLod);
            m_world.addComponent(e, SpinComponent{{0.0f, 1.0f, 0.0f}, 2.9e-6f});
            sw::phys::GravitySourceComponent gravity{kMuSol, kSolRadius};
            gravity.angularVelocity = {0.0, 2.9e-6, 0.0};
            m_world.addComponent(e, gravity);
            m_world.addComponent(e, sw::space::makeCelestialBody("SOL"));
            m_world.addComponent(e, MapMarkerComponent{{1.0f, 0.85f, 0.3f, 1.0f}});
            m_solEntity = e;
        }

        // ---- Terra: SOLID SURFACE + ATMOSPHERE, on rails around Sol --------------
        const sw::phys::KeplerOrbit terraOrbit = sw::phys::kepler::fromElements(
            kMuSol, kTerraSma, 0.0167, 0.0, 0.0, 0.0, /*M0=*/0.0, /*epoch=*/0.0);
        sw::WorldVec3 terraPos0{};
        sw::phys::kepler::evaluate(terraOrbit, 0.0, terraPos0);
        {
            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            transform.position = terraPos0;
            transform.uniformScale = static_cast<sw::f32>(kTerraRadius);
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{1.0f});
            m_world.addComponent(e, terraLod);
            m_world.addComponent(e, SpinComponent{{0.0f, 1.0f, 0.0f}, 7.2921e-5f});
            sw::phys::GravitySourceComponent gravity{kMuTerra, kTerraRadius};
            gravity.soiRadius = kTerraSoi;
            gravity.angularVelocity = kTerraAngularVelocity; // matches the Spin
            m_world.addComponent(e, gravity);
            m_world.addComponent(e, sw::phys::AtmosphereComponent{1.225, 8500.0, 1.4e5});
            m_world.addComponent(e, presetTerra()); // REAL ground: collision + visuals
            // Geology, analytic like the ground itself: nothing about a
            // deposit is stored, so a survey cannot lie and a save cannot
            // move the ore.
            m_world.addComponent(e, sw::planet::depositsTerra());
            m_world.addComponent(e, sw::space::makeCelestialBody("TERRA", m_solEntity,
                                                                 &terraOrbit));
            m_world.addComponent(e, MapMarkerComponent{{0.35f, 0.65f, 1.0f, 1.0f}});
            m_terraEntity = e;
        }
        const sw::ecs::Entity terraEntity = m_terraEntity;

        // ---- Terra's VISIBLE atmosphere + cloud deck (transparent shells) --------
        {
            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            transform.position = terraPos0;
            // Encloses the 80 km air column Shaders/Atmosphere.glsl marches
            // (a shell smaller than the model would clip the limb).
            transform.uniformScale = static_cast<sw::f32>(kTerraRadius * 1.0130); // ~83 km
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{1.0f});
            MeshComponent mesh{atmosphereMeshId};
            mesh.transparent = 1;
            m_world.addComponent(e, mesh);
            m_world.addComponent(e, CloudLayerComponent{terraEntity, {0, 1, 0}, 0.0f});
        }
        {
            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            transform.position = terraPos0;
            transform.uniformScale = static_cast<sw::f32>(kTerraRadius * 1.005); // ~32 km
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{1.0f});
            MeshComponent mesh{cloudMeshId};
            mesh.transparent = MeshComponent::kCloudDeck;
            m_world.addComponent(e, mesh);
            // M28: the shell is GLUED to Terra's rotation and the drift moved
            // into the shader (Clouds.glsl). That is what lets the ground
            // path reproduce the deck exactly from the world clock and put
            // each shadow under the cloud that casts it.
            m_world.addComponent(e, CloudLayerComponent{terraEntity, {0, 1, 0}, 0.0f});
        }

        // ---- Luna: around TERRA (5.14 deg inclination, real) ---------------------
        const sw::phys::KeplerOrbit lunaOrbit = sw::phys::kepler::fromElements(
            kMuTerra, kLunaSma, 0.0549, 0.0897, 0.0, 0.0, /*M0=*/0.6, /*epoch=*/0.0);
        {
            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            sw::WorldVec3 lunaRel{};
            sw::phys::kepler::evaluate(lunaOrbit, 0.0, lunaRel);
            transform.position = terraPos0 + lunaRel;
            transform.uniformScale = static_cast<sw::f32>(kLunaRadius);
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{1.0f});
            m_world.addComponent(e, lunaLod);
            m_world.addComponent(e, SpinComponent{{0.0f, 1.0f, 0.0f}, 2.66e-6f});
            sw::phys::GravitySourceComponent gravity{kMuLuna, kLunaRadius};
            gravity.soiRadius = kLunaSoi;
            gravity.angularVelocity = {0.0, 2.66e-6, 0.0};
            m_world.addComponent(e, gravity);
            m_world.addComponent(e, presetLuna());
            m_world.addComponent(e, sw::planet::depositsLuna()); // polar ice = propellant
            m_world.addComponent(e, sw::space::makeCelestialBody("LUNA", terraEntity,
                                                                 &lunaOrbit));
            m_world.addComponent(e, MapMarkerComponent{{0.75f, 0.75f, 0.78f, 1.0f}});
        }

        // ---- Mars: second planet, red and far -------------------------------------
        const sw::phys::KeplerOrbit marsOrbit = sw::phys::kepler::fromElements(
            kMuSol, kMarsSma, 0.0934, 0.0323, 0.0, 0.0, /*M0=*/2.0, /*epoch=*/0.0);
        {
            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            sw::phys::kepler::evaluate(marsOrbit, 0.0, transform.position);
            transform.uniformScale = static_cast<sw::f32>(kMarsRadius);
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{1.0f});
            m_world.addComponent(e, marsLod);
            m_world.addComponent(e, SpinComponent{{0.0f, 1.0f, 0.0f}, 7.088e-5f});
            sw::phys::GravitySourceComponent gravity{kMuMars, kMarsRadius};
            gravity.soiRadius = kMarsSoi;
            gravity.angularVelocity = {0.0, 7.088e-5, 0.0};
            m_world.addComponent(e, gravity);
            m_world.addComponent(e, presetMars());
            m_world.addComponent(e, sw::planet::depositsMars());
            m_world.addComponent(e, sw::space::makeCelestialBody("MARS", m_solEntity,
                                                                 &marsOrbit));
            m_world.addComponent(e, MapMarkerComponent{{1.0f, 0.45f, 0.25f, 1.0f}});
        }

        // ============ EVERYTHING ELSE LIVES IN TERRA'S SOI ========================
        // Spawns are TERRA-relative: dynamic bodies add Terra's world
        // position and orbital velocity; rails objects simply reference
        // Terra as their primary (relative elements unchanged).
        sw::WorldVec3 terraVel0{};
        {
            sw::WorldVec3 unused{};
            sw::phys::kepler::evaluate(terraOrbit, 0.0, unused, &terraVel0);
        }
        const sw::WorldVec3 stationCenter =
            terraPos0 + sw::WorldVec3{0.0, 0.0, kStationOrbitRadius};

        // ---- GROUND OUTPOST: the first FACTORY SITE, built from data ------------
        // F1 turns the old hard-coded mining rig into what the player will
        // build in F2: a hub that owns a site, and buildings that are
        // .swpart definitions running .swrecipe recipes. Nothing below
        // hard-codes a rate, a power figure or a chain — it names catalogue
        // ids and lets the data say what they mean.
        //
        // Anchored in Terra's rotating frame: the site rides the planet's
        // rotation and survives save/load at its exact construction site.
        {
            const sw::planet::TerrainComponent terrain = presetTerra();
            const sw::planet::DepositComponent deposits = sw::planet::depositsTerra();

            // ---- the SURVEY: where the ore actually is ---------------------
            // The old rig sat at the equator on +Z because that was a
            // convenient number. A mine's yield is now the analytic deposit
            // density under its feet, so the scene does what a player will
            // do in F2: scan the neighbourhood and site the mine on the best
            // ground it finds. Deterministic — the same world always builds
            // the same outpost.
            const sw::Vec3 siteUp = terraStartSite();

            // Local tangent frame of the site: buildings are laid out in
            // metres on this plane, which is exactly the frame F2's ground
            // build mode will place them in.
            const sw::Vec3 siteEast = glm::normalize(
                glm::cross(sw::Vec3{0.0f, 1.0f, 0.0f}, siteUp));
            const sw::Vec3 siteNorth = glm::cross(siteUp, siteEast);
            // Model space is Y-up; the anchor's local rotation stands the
            // building on the local vertical.
            const sw::Quat standUp = standUpFor(siteUp);

            sw::ecs::Entity hubEntity{};
            // Spawns one building from its catalogue definition: geometry,
            // power, storage and siting rules all come from the .swpart.
            auto spawnBuilding = [&](sw::u32 definitionId, sw::f32 eastMetres,
                                     sw::f32 northMetres, sw::u32 recipeId,
                                     const sw::Vec4& marker, sw::f32 yawRadians = 0.0f) {
                // Offsets are metres on the tangent plane, re-normalised so
                // every building sits on the sphere — and then handed to the
                // very same placement the player's build cursor uses.
                const sw::Vec3 direction = glm::normalize(
                    siteUp + siteEast * (eastMetres / static_cast<sw::f32>(kTerraRadius)) +
                    siteNorth * (northMetres / static_cast<sw::f32>(kTerraRadius)));
                return placeBuilding(definitionId, terraEntity, direction, yawRadians,
                                     recipeId, hubEntity, marker);
            };

            // The hub defines the site; everything else points back at it.
            hubEntity = spawnBuilding(sw::parts::kBuildingHub, 0.0f, 0.0f, 0u, {});
            if (!hubEntity.isNull())
            {
                sw::factory::SiteComponent site{};
                std::snprintf(site.name, sizeof(site.name), "%s", "TERRA ALPHA");
                site.body = terraEntity;
                m_world.addComponent(hubEntity, site);
                m_world.getComponent<sw::factory::BuildingComponent>(hubEntity).site =
                    hubEntity;
            }

            // Each machine is turned so its conveyor mouth faces the belt it
            // feeds: the miner ships south to the smelter, the smelter ships
            // west to the silo. The port directions come from the .swpart,
            // so re-authoring a mouth in Part Studio re-aims the machine.
            const auto* minerPart = sw::parts::findDefinition(sw::parts::kBuildingMiner);
            const auto* refineryPart =
                sw::parts::findDefinition(sw::parts::kBuildingRefinery);
            const auto* storagePart =
                sw::parts::findDefinition(sw::parts::kBuildingStorage);
            auto portDirection = [](const sw::parts::PartDefinition* definition,
                                    sw::parts::NodeType type) {
                const sw::parts::AttachNode* node =
                    (definition != nullptr) ? sw::parts::findConveyorNode(*definition, type)
                                            : nullptr;
                return (node != nullptr) ? node->direction : sw::Vec3{0.0f, 0.0f, 1.0f};
            };

            const sw::ecs::Entity minerEntity = spawnBuilding(
                sw::parts::kBuildingMiner, 34.0f, 0.0f, sw::factory::kRecipeMineIronOre,
                {},
                yawToFace(siteUp, portDirection(minerPart, sw::parts::NodeType::ConveyorOut),
                          -siteNorth));
            const sw::ecs::Entity refineryEntity = spawnBuilding(
                sw::parts::kBuildingRefinery, 34.0f, -30.0f,
                sw::factory::kRecipeSmeltIron, {},
                yawToFace(siteUp, portDirection(refineryPart, sw::parts::NodeType::ConveyorIn),
                          siteNorth));
            const sw::ecs::Entity storageEntity = spawnBuilding(
                sw::parts::kBuildingStorage, 0.0f, -30.0f, 0u, {},
                yawToFace(siteUp, portDirection(storagePart, sw::parts::NodeType::ConveyorIn),
                          siteEast));
            const sw::ecs::Entity solarEntity =
                spawnBuilding(sw::parts::kBuildingSolarFarm, -34.0f, -15.0f, 0u, {});

            // F3: the BANK. A site with panels and no storage is a site that
            // stops every sunset, so the starting outpost is delivered with
            // one — half charged, which makes the first night a decision
            // (run the smelter now, or keep the charge?) instead of a
            // scripted blackout.
            const sw::ecs::Entity batteryEntity =
                spawnBuilding(sw::parts::kBuildingBatteryBank, -34.0f, 14.0f, 0u, {});
            if (!batteryEntity.isNull())
            {
                sw::factory::inventoryAdd(
                    m_world.getComponent<sw::factory::InventoryComponent>(batteryEntity),
                    sw::res::Resource::ElectricCharge, 500000.0); // 500 MJ
            }

            // The BEACON: the site is at a surveyed spot on a 6,371 km
            // sphere, and nothing else here can be seen from the air. Its
            // 25 m lit mast finds you on the ground; the pointer it puts on
            // the map and on the HUD finds you from orbit.
            const sw::ecs::Entity beaconEntity =
                spawnBuilding(sw::parts::kBuildingBeacon, -14.0f, 22.0f, 0u,
                              {1.0f, 0.78f, 0.28f, 1.0f});
            if (!beaconEntity.isNull())
            {
                sw::factory::BeaconComponent beacon{};
                std::snprintf(beacon.label, sizeof(beacon.label), "%s", "TERRA ALPHA");
                beacon.rangeM = 1.0e6;    // 1000 km: visible from low orbit
                beacon.nearRangeM = 500.0; // ...and out of the way once you land
                m_world.addComponent(beaconEntity, beacon);
            }

            // ---- F5: THE VAB AND THE PAD ----------------------------------
            // The far end of the yard, and the reason the near end exists.
            // The hall faces the pad, the pad faces the hall, and the belt
            // between them carries finished rockets — the one belt on this
            // planet whose cargo is a vehicle.
            //
            // The pad stands where new vessels have always appeared (120 m
            // east of the hub), so it is now a REAL building at the place
            // that used to be a computed guess.
            const auto* vabPart = sw::parts::findDefinition(sw::parts::kBuildingVab);
            const auto* padPart =
                sw::parts::findDefinition(sw::parts::kBuildingLaunchPad);
            const sw::ecs::Entity vabEntity = spawnBuilding(
                sw::parts::kBuildingVab, 75.0f, 0.0f, 0u, {},
                yawToFace(siteUp, portDirection(vabPart, sw::parts::NodeType::ConveyorOut),
                          siteEast));
            const sw::ecs::Entity padEntity = spawnBuilding(
                sw::parts::kBuildingLaunchPad, 120.0f, 0.0f, 0u,
                {0.4f, 0.9f, 0.92f, 1.0f},
                yawToFace(siteUp, portDirection(padPart, sw::parts::NodeType::ConveyorIn),
                          -siteEast));
            if (!vabEntity.isNull())
            {
                // Enough metal for a first hull, and NO order standing. The
                // outpost is stocked, not started: the player designs
                // something in the hangar and orders it, which is the whole
                // loop this milestone exists to make mandatory. Seeding an
                // order here would just re-create the starting rocket that
                // was deleted a hundred lines above.
                auto& bin =
                    m_world.getComponent<sw::factory::InventoryComponent>(vabEntity);
                sw::factory::inventoryAdd(bin, sw::res::Resource::Iron, 3000.0);
                sw::factory::inventoryAdd(bin, sw::res::Resource::Copper, 500.0);
            }
            if (!padEntity.isNull())
            {
                // ...and fuel on the pad, so the first rocket rolls out with
                // something in its tanks.
                sw::factory::inventoryAdd(
                    m_world.getComponent<sw::factory::InventoryComponent>(padEntity),
                    sw::res::Resource::Fuel, 16000.0);
            }

            // ---- the BELTS, laid by the very tool the player uses ---------
            // Two clicks' worth of work: pick the machine that ships, pick
            // the one that receives, and `planBelt` produces the run. The
            // starting outpost gets no shortcut — if the tool could not lay
            // this belt, neither could the scene.
            auto layBelt = [&](sw::ecs::Entity from, sw::ecs::Entity to) {
                std::vector<BeltTile> tiles;
                const sw::build::Verdict verdict = planBelt(terraEntity, from, to, tiles);
                if (verdict != sw::build::Verdict::Ok)
                {
                    SW_LOG_WARN("Game", "Starting belt refused: {}",
                                sw::build::verdictText(verdict));
                }
                for (const BeltTile& tile : tiles)
                {
                    placeBuilding(sw::parts::kBuildingConveyor, terraEntity,
                                  tile.direction, tile.yawRadians, 0u, hubEntity, {});
                }
            };
            layBelt(minerEntity, refineryEntity);
            layBelt(refineryEntity, storageEntity);
            layBelt(vabEntity, padEntity);

            // ...and now derive what those rows of segments actually connect.
            rebuildConveyorNetwork();

            // ---- the GRID, wired the way the player would wire it --------
            // One PL-1 in the middle of the yard, and a single span from it
            // to every machine. That is not decoration: without the pole the
            // outpost could not be one grid at all, because a building takes
            // exactly one cable and seven of them cannot form a chain.
            // The starting base gets no exemption from the rule it teaches.
            const sw::ecs::Entity poleEntity =
                spawnBuilding(sw::parts::kBuildingPowerPole, 10.0f, -14.0f, 0u, {});
            // A second pole out by the pad: a span is 120 m at most, and the
            // launch complex is further from the yard than that rule allows
            // in one hop. Poles are how a grid crosses ground, which is the
            // whole reason they exist.
            const sw::ecs::Entity farPoleEntity =
                spawnBuilding(sw::parts::kBuildingPowerPole, 97.0f, -16.0f, 0u, {});
            rebuildPowerNetwork(); // number the nodes before asking about them
            if (!poleEntity.isNull() && !farPoleEntity.isNull())
            {
                layCable(terraEntity, poleEntity, farPoleEntity);
            }
            if (!farPoleEntity.isNull())
            {
                for (const sw::ecs::Entity end : {vabEntity, padEntity})
                {
                    if (end.isNull())
                    {
                        continue;
                    }
                    sw::WorldVec3 from{};
                    sw::WorldVec3 to{};
                    if (planCable(farPoleEntity, end, from, to) !=
                        sw::factory::CableVerdict::Ok)
                    {
                        SW_LOG_WARN("Game", "Launch complex cable refused");
                        continue;
                    }
                    layCable(terraEntity, farPoleEntity, end);
                }
            }
            if (!poleEntity.isNull())
            {
                for (const sw::ecs::Entity end :
                     {hubEntity, minerEntity, refineryEntity, storageEntity, solarEntity,
                      batteryEntity, beaconEntity})
                {
                    if (end.isNull())
                    {
                        continue;
                    }
                    sw::WorldVec3 from{};
                    sw::WorldVec3 to{};
                    const sw::factory::CableVerdict verdict =
                        planCable(poleEntity, end, from, to);
                    if (verdict != sw::factory::CableVerdict::Ok)
                    {
                        SW_LOG_WARN("Game", "Starting cable refused: {}",
                                    sw::factory::cableVerdictText(verdict));
                        continue;
                    }
                    layCable(terraEntity, poleEntity, end);
                }
            }

            if (!hubEntity.isNull())
            {
                auto& site = m_world.getComponent<sw::factory::SiteComponent>(hubEntity);
                m_world.forEach<sw::factory::BuildingComponent,
                                sw::factory::PowerComponent>(
                    [&](sw::ecs::Entity, sw::factory::BuildingComponent& building,
                        sw::factory::PowerComponent& power) {
                        if (building.site != hubEntity)
                        {
                            return;
                        }
                        site.producedKw += power.producedKw;
                        site.consumedKw += power.consumedKw;
                        ++site.buildingCount;
                    });
                SW_LOG_INFO("Game",
                            "Site '{}': {} buildings, {:.0f} kW produced / {:.0f} kW "
                            "demanded, iron grade {:.2f}",
                            site.name, site.buildingCount, site.producedKw,
                            site.consumedKw,
                            !minerEntity.isNull()
                                ? m_world
                                      .getComponent<sw::factory::BuildingComponent>(
                                          minerEntity)
                                      .groundDensity
                                : 0.0f);
            }
        }

        // ---- THE PLAYER: on foot, at the base --------------------------------
        //
        // There is no starting rocket any more, and that is the point. A
        // vessel exists in this world only because a VAB was given a design
        // and the iron and copper to build it; handing the player one at
        // start-up made the entire assembly line decorative.
        //
        // So the player starts as a suit standing on the pad apron at TERRA
        // ALPHA. On foot is the NORMAL state now — piloting is something you
        // board, not something you begin in.
        {
            const sw::ecs::Entity e = m_world.createEntity();

            // Ten metres north of the hub, on the ground, in the same tangent
            // frame every building was laid out in.
            const sw::Vec3 siteUp = terraStartSite();
            const sw::Vec3 siteEast =
                glm::normalize(glm::cross(sw::Vec3{0.0f, 1.0f, 0.0f}, siteUp));
            const sw::Vec3 siteNorth = glm::cross(siteUp, siteEast);
            const sw::Vec3 standDirection = glm::normalize(
                siteUp + siteNorth * (14.0f / static_cast<sw::f32>(kTerraRadius)));

            sw::f64 elevation = 0.0;
            if (const auto* terrain =
                    m_world.tryGetComponent<sw::planet::TerrainComponent>(terraEntity))
            {
                elevation = sw::planet::terrainElevation(*terrain, standDirection);
            }

            TransformComponent transform{};
            transform.position = terraPos0 + sw::WorldVec3(standDirection) *
                                                 (kTerraRadius + elevation + 2.0);
            transform.rotation = standUpFor(standDirection);
            m_world.addComponent(
                e, PreviousTransformComponent{transform.position, transform.rotation});
            m_world.addComponent(e, transform);
            m_world.addComponent(e, BoundsComponent{1.4f});
            m_world.addComponent(e, MeshComponent{m_capsuleMeshIndex});
            m_world.addComponent(e, MapMarkerComponent{{1.0f, 0.8f, 0.2f, 1.0f}});
            m_world.addComponent(e, m_capsuleHull);
            if (const auto* suit = sw::parts::findDefinition(sw::parts::kPropEvaSuit))
            {
                sw::phys::HullComponent hull{};
                if (hullFor(*suit, hull))
                {
                    m_world.addComponent(e, hull);
                    m_world.addComponent(e, sw::phys::HullMoverComponent{});
                }
            }
            m_world.addComponent(e, CapsuleComponent{});
            m_world.addComponent(e, ShipControlsComponent{});

            // THE CARRIER VELOCITY. A body standing on Terra is not still: it
            // is doing Terra's orbit plus Terra's spin at this latitude, some
            // 30 km/s and 465 m/s of it. Start it at rest in the world frame
            // and the ground leaves at half a kilometre a second.
            sw::WorldVec3 spin{0.0};
            if (const auto* gravity =
                    m_world.tryGetComponent<sw::phys::GravitySourceComponent>(terraEntity))
            {
                spin = glm::cross(gravity->angularVelocity,
                                  transform.position - terraPos0);
            }
            sw::phys::DynamicBodyComponent body{};
            body.velocity = terraVel0 + spin;
            body.mass = 120.0;
            body.ballisticFactor = 0.01;
            m_world.addComponent(e, body);

            m_capsuleEntity = e;
            m_evaMode = true;
        }
    }

    void StarWorksGame::buildSaveSchema()
    {
        // Stable names + versions. Bump a version whenever the struct layout
        // changes; the loader refuses mismatches instead of guessing.
        m_saveSchema.registerComponent<sw::TransformComponent>("sw.Transform", 1);
        m_saveSchema.registerComponent<sw::PreviousTransformComponent>(
            "sw.PreviousTransform", 1);
        // v2: + the body-frame angular velocity, which moved down here from
        // the game's ship when the atmosphere became able to spin things.
        m_saveSchema.registerComponent<sw::phys::DynamicBodyComponent>("phys.DynamicBody",
                                                                       2);
        m_saveSchema.registerComponent<sw::phys::GroundHullComponent>("phys.GroundHull",
                                                                      1);
        // v2: primary-relative orbit + primary handle + dynamic payload.
        m_saveSchema.registerComponent<sw::phys::OnRailsComponent>("phys.OnRails", 2);
        // v4: + the f64 spin state (axis, angle, previous angle) that keeps
        // planet-radius offsets from shimmering.
        m_saveSchema.registerComponent<sw::phys::GravitySourceComponent>(
            "phys.GravitySource", 4);
        m_saveSchema.registerComponent<sw::space::CelestialBodyComponent>(
            "space.CelestialBody", 1);
        m_saveSchema.registerComponent<sw::phys::AtmosphereComponent>("phys.Atmosphere", 1);
        m_saveSchema.registerComponent<sw::phys::SurfaceAnchorComponent>(
            "phys.SurfaceAnchor", 2); // v2: local rotation + auto-release payload
        m_saveSchema.registerComponent<sw::planet::TerrainComponent>("planet.Terrain", 3);
        m_saveSchema.registerComponent<sw::planet::DepositComponent>("planet.Deposits",
                                                                    1);
        // v2: + surfaceRelative, and kStability joined the modes.
        m_saveSchema.registerComponent<SasComponent>("game.Sas", 2);
        m_saveSchema.registerComponent<sw::parts::PartComponent>("parts.Part", 1);
        // v2: + centre of mass, inertia and hull extents — what the
        // aerodynamics needs to turn a moment into a rotation.
        m_saveSchema.registerComponent<sw::parts::VesselComponent>("parts.Vessel", 2);
        // F6 — the air's answer. Recomputed every tick, so it is saved only
        // to keep the component ON the entity across a reload: a vessel that
        // came back without one would silently fall back to isotropic drag.
        m_saveSchema.registerComponent<sw::aero::AeroStateComponent>("aero.State", 1);
        m_saveSchema.registerComponent<sw::parts::JointComponent>("parts.Joint", 1);
        m_saveSchema.registerComponent<sw::factory::InventoryComponent>("factory.Inventory",
                                                                        1);
        m_saveSchema.registerComponent<sw::factory::MinerComponent>("factory.Miner", 1);
        m_saveSchema.registerComponent<sw::factory::RefineryComponent>("factory.Refinery",
                                                                       1);
        // v2: an ARRAY of channels — a machine can be fed more than one
        // good, which is what the fuel chain's synthesiser needs.
        m_saveSchema.registerComponent<sw::factory::ItemLinkComponent>("factory.ItemLink",
                                                                       2);
        // F1 — the data-driven industry.
        m_saveSchema.registerComponent<sw::factory::BuildingComponent>("factory.Building",
                                                                       1);
        m_saveSchema.registerComponent<sw::factory::RecipeStateComponent>(
            "factory.RecipeState", 1);
        // F3 — the grid. v2: + actualProducedKw, priority. v3: + gridId and
        // the grid's books, which arrived with the cables.
        m_saveSchema.registerComponent<sw::factory::PowerComponent>("factory.Power", 3);
        // v2 of Site: + batteryFlowKw.
        m_saveSchema.registerComponent<sw::factory::SiteComponent>("factory.Site", 2);
        m_saveSchema.registerComponent<sw::factory::BatteryComponent>("factory.Battery", 1);
        // THE CABLES. The link is what is stored — its two ends — because
        // unlike a belt there is no intermediate object to derive it from.
        // The CableComponent's curve is NOT authoritative: rebuildPowerNetwork
        // re-hangs it from the endpoints after every load.
        m_saveSchema.registerComponent<sw::factory::PowerLinkComponent>(
            "factory.PowerLink", 1);
        m_saveSchema.registerComponent<CableComponent>("game.Cable", 1);
        // v2: + nearRangeM (the pointer steps aside once you have arrived).
        m_saveSchema.registerComponent<sw::factory::BeaconComponent>("factory.Beacon", 2);
        m_saveSchema.registerComponent<BoundsComponent>("game.Bounds", 1);
        m_saveSchema.registerComponent<SpinComponent>("game.Spin", 1);
        m_saveSchema.registerComponent<MeshComponent>("game.Mesh", 2); // v2: transparent
        m_saveSchema.registerComponent<CloudLayerComponent>("game.CloudLayer", 1);
        m_saveSchema.registerComponent<CelestialLodComponent>("game.CelestialLod",
                                                              2); // v2: surfaceStyle
        // v2: angular velocity moved to phys.DynamicBody.
        m_saveSchema.registerComponent<ShipComponent>("game.Ship", 2);
        // v2: + strafeAxis (the EVA sidestep).
        m_saveSchema.registerComponent<ShipControlsComponent>("game.ShipControls", 2);
        m_saveSchema.registerComponent<CapsuleComponent>("game.Capsule", 1);
        m_saveSchema.registerComponent<MapMarkerComponent>("game.MapMarker", 1);
        m_saveSchema.registerComponent<ConveyorComponent>("game.Conveyor", 2); // v2: source
    }

    void StarWorksGame::saveGame()
    {
        sw::ser::BinaryWriter writer;
        writer.write<sw::u32>(0x53575347); // "SWSG"
        writer.write<sw::u32>(9);          // game save version (9: Milestone 23)

        sw::save::saveWorld(m_world, m_saveSchema, writer);
        sw::save::saveSimulation(m_simulation, writer);

        // Player/session state.
        writer.write(m_shipEntity);
        writer.write(m_capsuleEntity);
        writer.write(static_cast<sw::u8>(m_evaMode ? 1 : 0));
        writer.write(static_cast<sw::u8>(m_shipMode ? 1 : 0));
        writer.write(static_cast<sw::u8>(m_speedSurfaceRelative ? 1 : 0));
        writer.write(m_warpIndex);
        writer.write(m_mapHeightMeters);
        writer.write(m_camera.position());
        writer.write(m_camera.orientation());
        // Maneuver node (v4).
        writer.write(static_cast<sw::u8>(m_nodeActive ? 1 : 0));
        writer.write(m_nodeTime);
        writer.write(m_nodePrograde);
        writer.write(m_nodeNormal);
        writer.write(m_nodeRadial);

        const auto path = sw::FileSystem::executableDirectory() / "starworks.sav";
        sw::FileSystem::writeBinaryFile(path, writer.bytes());
        SW_LOG_INFO("Game", "Saved to '{}' ({} KB, {} entities, t={:.1f}s)", path.string(),
                    writer.size() / 1024, m_world.aliveCount(),
                    m_simulation.simulatedSeconds());
    }

    void StarWorksGame::loadGame()
    {
        const auto path = sw::FileSystem::executableDirectory() / "starworks.sav";
        const std::vector<sw::u8> bytes = sw::FileSystem::readBinaryFile(path);
        sw::ser::BinaryReader reader(bytes);

        if (reader.read<sw::u32>() != 0x53575347)
        {
            SW_THROW("'{}' is not a StarWorks save", path.string());
        }
        if (const sw::u32 version = reader.read<sw::u32>(); version != 9)
        {
            SW_THROW("Unsupported save version {}", version);
        }

        sw::save::loadWorld(m_world, m_saveSchema, reader);
        sw::save::loadSimulation(m_simulation, reader);
        m_celestialIndex.rebuild(m_world);
        m_lastPredictionSeconds = -1.0e9; // stale flight plan: recompute

        m_shipEntity = reader.read<sw::ecs::Entity>();
        m_capsuleEntity = reader.read<sw::ecs::Entity>();
        m_evaMode = reader.read<sw::u8>() != 0;
        m_shipMode = reader.read<sw::u8>() != 0;
        m_speedSurfaceRelative = reader.read<sw::u8>() != 0;
        m_warpIndex = reader.read<sw::u32>();
        m_mapHeightMeters = reader.read<sw::f64>();
        m_camera.setPosition(reader.read<sw::WorldVec3>());
        m_camera.setOrientation(reader.read<sw::Quat>());
        m_nodeActive = reader.read<sw::u8>() != 0;
        m_nodeTime = reader.read<sw::f64>();
        m_nodePrograde = reader.read<sw::f64>();
        m_nodeNormal = reader.read<sw::f64>();
        m_nodeRadial = reader.read<sw::f64>();
        if (const auto* sas = m_world.tryGetComponent<SasComponent>(m_shipEntity))
        {
            m_sasMode = sas->mode;
        }
        // Force a terrain patch rebuild on the next frame.
        m_lastTerrainRebuildSeconds = -1.0e9;
        m_terrainBody = {};

        if (!m_shipMode)
        {
            const sw::Vec3 forward = m_camera.forward();
            m_cameraController.setPose(
                m_camera.position(), std::atan2(-forward.x, -forward.z),
                std::asin(std::clamp(forward.y, -1.0f, 1.0f)));
        }

        // The two DERIVED networks. Neither is stored — the belts' chains
        // come from where the mouths ended up, the grids from the cables —
        // so a loaded world has to re-derive both before the first tick, or
        // it runs one frame with an empty factory and a dead grid.
        rebuildConveyorNetwork();
        rebuildPowerNetwork();
        rebuildHulls();

        SW_LOG_INFO("Game", "Loaded '{}': {} entities, t={:.1f}s, warp x{:g}",
                    path.string(), m_world.aliveCount(), m_simulation.simulatedSeconds(),
                    kWarpLadder[m_warpIndex]);
    }

    sw::ecs::Entity StarWorksGame::controlledEntity() const
    {
        // ON FOOT IS THE NORMAL STATE. There may be no vessel at all — the
        // game now starts with none — so the suit is the fallback and not
        // the exception. Everything downstream (the HUD, the camera, the
        // terrain patch, the simulation bubble) dereferences this without
        // checking, which is only safe because it is never null once the
        // scene is built.
        if (m_evaMode && !m_capsuleEntity.isNull())
        {
            return m_capsuleEntity;
        }
        if (!m_shipEntity.isNull() && m_world.isAlive(m_shipEntity))
        {
            return m_shipEntity;
        }
        return m_capsuleEntity;
    }

    sw::WorldVec3 StarWorksGame::controlledVelocity() const
    {
        const sw::ecs::Entity entity = controlledEntity();
        auto& world = const_cast<sw::ecs::World&>(m_world);
        if (const auto* body =
                world.tryGetComponent<sw::phys::DynamicBodyComponent>(entity))
        {
            return body->velocity;
        }
        if (const auto* rails = world.tryGetComponent<sw::phys::OnRailsComponent>(entity))
        {
            sw::WorldVec3 position{};
            sw::WorldVec3 velocity{};
            sw::phys::kepler::evaluate(rails->orbit, m_physicsLane->presentSeconds(),
                                       position, &velocity);
            // The orbit is primary-relative: add the primary's own motion.
            if (const auto* primary = world.tryGetComponent<sw::phys::GravitySourceComponent>(
                    rails->primary))
            {
                velocity += primary->worldVelocity;
            }
            return velocity;
        }
        return {0.0, 0.0, 0.0};
    }

    sw::f32 StarWorksGame::heatingFactorFor(sw::ecs::Entity entity) const
    {
        auto& world = const_cast<sw::ecs::World&>(m_world);
        const auto* body = world.tryGetComponent<sw::phys::DynamicBodyComponent>(entity);
        const auto* transform = world.tryGetComponent<TransformComponent>(entity);
        if (body == nullptr || transform == nullptr || m_celestialIndex.size() == 0)
        {
            return 0.0f;
        }
        const sw::i32 primaryIndex = m_celestialIndex.soiPrimaryAt(
            transform->position, m_physicsLane->presentSeconds());
        if (primaryIndex < 0)
        {
            return 0.0f;
        }
        const auto& primary =
            m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
        const auto* atmosphere =
            world.tryGetComponent<sw::phys::AtmosphereComponent>(primary.entity);
        const auto* source =
            world.tryGetComponent<sw::phys::GravitySourceComponent>(primary.entity);
        const auto* primaryTransform =
            world.tryGetComponent<TransformComponent>(primary.entity);
        if (atmosphere == nullptr || source == nullptr || primaryTransform == nullptr)
        {
            return 0.0f;
        }

        const sw::WorldVec3 radial = transform->position - primaryTransform->position;
        const sw::f64 altitude = glm::length(radial) - primary.bodyRadius;
        if (altitude > atmosphere->topAltitude)
        {
            return 0.0f;
        }
        const sw::f64 density =
            atmosphere->surfaceDensity *
            std::exp(-std::max(altitude, 0.0) / atmosphere->scaleHeight);
        // Air moves with the planet: translation + spin at this point.
        const sw::WorldVec3 airVelocity =
            source->worldVelocity + glm::cross(source->angularVelocity, radial);
        const sw::f64 speed = glm::length(body->velocity - airVelocity);
        const sw::f64 q = density * speed * speed * speed;
        if (q <= kHeatGlowStart)
        {
            return 0.0f;
        }
        return std::clamp(
            static_cast<sw::f32>(std::log10(q / kHeatGlowStart)) / kHeatLogRange, 0.0f,
            1.0f);
    }

    void StarWorksGame::updateReentryEffects(sw::f32 deltaSeconds)
    {
        m_shipHeat = heatingFactorFor(m_shipEntity);
        m_capsuleHeat =
            m_capsuleEntity.isNull() ? 0.0f : heatingFactorFor(m_capsuleEntity);

        // ---- age & move existing particles (visual, render-frame rate) ------
        for (sw::usize i = 0; i < m_particles.size();)
        {
            ReentryParticle& particle = m_particles[i];
            particle.life -= deltaSeconds;
            if (particle.life <= 0.0f)
            {
                m_particles[i] = m_particles.back();
                m_particles.pop_back();
                continue;
            }
            particle.position += particle.velocity * static_cast<sw::f64>(deltaSeconds);
            ++i;
        }

        // ---- spawn plasma behind hot craft -----------------------------------
        auto spawnFor = [this, deltaSeconds](sw::ecs::Entity entity, sw::f32 heat) {
            if (heat < 0.05f)
            {
                return;
            }
            const auto* transform = m_world.tryGetComponent<TransformComponent>(entity);
            const auto* body =
                m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(entity);
            if (transform == nullptr || body == nullptr)
            {
                return;
            }

            // Spawn from the INTERPOLATED pose — the craft is RENDERED
            // there. Spawning at the raw physics-tick position offset the
            // whole cloud from the ship by up to a tick of motion (~600 m
            // of planetary travel): the "same offset on every particle" bug.
            sw::WorldVec3 spawnOrigin = transform->position;
            if (const auto* previous =
                    m_world.tryGetComponent<PreviousTransformComponent>(entity))
            {
                spawnOrigin =
                    glm::mix(previous->position, transform->position,
                             static_cast<sw::f64>(m_physicsLane->alpha()));
            }

            // The wake trails opposite the motion THROUGH THE AIR. The raw
            // world velocity is dominated by the planet's own 30 km/s
            // orbital motion and would point the trail the wrong way.
            sw::WorldVec3 airVelocity{0.0};
            const sw::i32 primaryIndex = m_celestialIndex.soiPrimaryAt(
                transform->position, m_physicsLane->presentSeconds());
            if (primaryIndex >= 0)
            {
                const auto& primary =
                    m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
                const auto* source =
                    m_world.tryGetComponent<sw::phys::GravitySourceComponent>(
                        primary.entity);
                const auto* primaryTransform =
                    m_world.tryGetComponent<TransformComponent>(primary.entity);
                if (source != nullptr && primaryTransform != nullptr)
                {
                    airVelocity =
                        source->worldVelocity +
                        glm::cross(source->angularVelocity,
                                   transform->position - primaryTransform->position);
                }
            }
            const sw::WorldVec3 relativeVelocity = body->velocity - airVelocity;
            const sw::f64 speed = glm::length(relativeVelocity);
            if (speed < 1.0)
            {
                return;
            }
            const sw::WorldVec3 backward = -relativeVelocity / speed;

            m_particleSpawnDebt += (40.0f + 280.0f * heat) * deltaSeconds;
            auto random01 = [this] { return hash01(m_particleSeed++); };
            while (m_particleSpawnDebt >= 1.0f && m_particles.size() < kMaxParticles)
            {
                m_particleSpawnDebt -= 1.0f;
                ReentryParticle particle{};
                const sw::WorldVec3 jitter{random01() - 0.5f, random01() - 0.5f,
                                           random01() - 0.5f};
                // The plasma is shed along the wake, BEHIND the craft: a
                // glowing tail rather than a cloud around the camera.
                particle.position = spawnOrigin +
                                    backward * (12.0 + 160.0 * random01()) +
                                    jitter * 7.0;
                particle.velocity = body->velocity +
                                    backward * (60.0 + 260.0 * random01()) +
                                    jitter * 30.0;
                particle.maxLife = 0.35f + 0.75f * random01();
                particle.life = particle.maxLife;
                particle.size = (0.20f + 0.45f * random01()) * (0.5f + heat);
                // Long incandescent STREAK along the airflow.
                particle.streakDirection = sw::Vec3(backward);
                particle.stretch = 5.0f + 6.0f * heat;
                particle.kind = 0;
                m_particles.push_back(particle);
            }
        };
        spawnFor(m_shipEntity, m_shipHeat);
        if (!m_capsuleEntity.isNull())
        {
            spawnFor(m_capsuleEntity, m_capsuleHeat);
        }

        // ---- engine exhaust jet ----------------------------------------------
        const auto* ship = m_world.tryGetComponent<ShipComponent>(m_shipEntity);
        const auto* controls =
            m_world.tryGetComponent<ShipControlsComponent>(m_shipEntity);
        const auto* shipBody =
            m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(m_shipEntity);
        const auto* shipTransform =
            m_world.tryGetComponent<TransformComponent>(m_shipEntity);
        if (ship != nullptr && controls != nullptr && shipBody != nullptr &&
            shipTransform != nullptr && controls->thrustAxis != 0.0f &&
            ship->throttle > 0.01f)
        {
            const sw::f32 power = ship->throttle * std::abs(controls->thrustAxis);
            m_particleSpawnDebt += 220.0f * power * deltaSeconds;
            auto random01 = [this] { return hash01(m_particleSeed++); };
            // The plume leaves through the nozzle, OPPOSITE the thrust:
            // forward burn -> jet out the back (+Z body), retro -> the front.
            const sw::f32 jetSign = (controls->thrustAxis > 0.0f) ? 1.0f : -1.0f;
            const sw::Vec3 jetDir =
                shipTransform->rotation * sw::Vec3{0.0f, 0.0f, jetSign};
            sw::WorldVec3 exhaustOrigin = shipTransform->position;
            if (const auto* previous =
                    m_world.tryGetComponent<PreviousTransformComponent>(m_shipEntity))
            {
                exhaustOrigin =
                    glm::mix(previous->position, shipTransform->position,
                             static_cast<sw::f64>(m_physicsLane->alpha()));
            }
            const sw::WorldVec3 nozzle = exhaustOrigin + sw::WorldVec3(jetDir) * 9.5;
            while (m_particleSpawnDebt >= 1.0f && m_particles.size() < kMaxParticles)
            {
                m_particleSpawnDebt -= 1.0f;
                ReentryParticle particle{};
                const sw::WorldVec3 jitter{random01() - 0.5f, random01() - 0.5f,
                                           random01() - 0.5f};
                particle.position = nozzle + jitter * 1.2;
                particle.velocity = shipBody->velocity +
                                    sw::WorldVec3(jetDir) * (70.0 + 90.0 * random01()) +
                                    jitter * 7.0;
                particle.maxLife = 0.22f + 0.30f * random01();
                particle.life = particle.maxLife;
                particle.size = 0.28f + 0.35f * random01();
                particle.streakDirection = jetDir;
                particle.stretch = 3.0f;
                particle.kind = 1;
                m_particles.push_back(particle);
            }
        }
    }

    void StarWorksGame::collectParticles(const sw::Camera& activeCamera)
    {
        const sw::WorldVec3 cameraPosition = activeCamera.position();
        for (const ReentryParticle& particle : m_particles)
        {
            const sw::f32 lifeFraction = particle.life / particle.maxLife; // 1 -> 0
            const sw::f32 heatColor = lifeFraction * lifeFraction;

            sw::Vec4 tint{};
            if (particle.kind == 0) // plasma: white-hot -> deep red
            {
                tint = {1.0f, 0.25f + 0.65f * heatColor, 0.05f + 0.45f * heatColor,
                        1.0f};
            }
            else // exhaust: blue-white flame -> faint amber
            {
                tint = {0.55f + 0.40f * heatColor, 0.60f + 0.35f * heatColor,
                        0.75f + 0.25f * heatColor, 1.0f};
            }
            const sw::f32 size =
                particle.size * (1.0f + 1.6f * (1.0f - lifeFraction));

            // Soft round BILLBOARD (radial alpha falloff), stretched along
            // the streak direction projected onto the view plane — plasma
            // reads as glowing gas, exhaust as a flame, no hard box edges.
            const sw::Vec3 relative = sw::Vec3(particle.position - cameraPosition);
            const sw::f32 depth = glm::length(relative);
            const sw::Vec3 toCam = depth > 1.0e-4f ? -relative / depth
                                                   : sw::Vec3{0.0f, 0.0f, 1.0f};
            sw::Vec3 stretchAxis =
                particle.streakDirection -
                toCam * glm::dot(particle.streakDirection, toCam);
            const sw::f32 stretchLength = glm::length(stretchAxis);
            if (stretchLength > 1.0e-4f)
            {
                stretchAxis /= stretchLength;
            }
            else
            {
                const sw::Vec3 reference =
                    std::abs(toCam.y) < 0.99f ? sw::Vec3{0, 1, 0} : sw::Vec3{1, 0, 0};
                stretchAxis = glm::normalize(glm::cross(reference, toCam));
            }
            const sw::Vec3 minorAxis = glm::cross(toCam, stretchAxis);
            const sw::Mat4 basis{sw::Vec4(stretchAxis, 0.0f), sw::Vec4(minorAxis, 0.0f),
                                 sw::Vec4(toCam, 0.0f), sw::Vec4(relative, 1.0f)};

            sw::DrawItem item{};
            item.mesh = &m_meshes[m_particleGlowMeshIndex];
            item.transform =
                basis * glm::scale(sw::Mat4{1.0f},
                                   sw::Vec3{size * particle.stretch, size, 1.0f});
            item.boundsCenter = relative;
            item.boundsRadius = size * particle.stretch;
            item.tint = tint;
            m_drawItems.push_back(item);
        }
    }

    void StarWorksGame::updateTerrainPatch()
    {
        // ---- 1. land a finished build -------------------------------------
        // The job wrote into the pending mesh; nothing on screen referenced
        // it, so the upload is a plain buffer creation with no device idle.
        if (m_terrainJob.load(std::memory_order_acquire) == TerrainJob::Ready)
        {
            // A BUILD THAT PRODUCED NOTHING IS NOT A BUILD TO UPLOAD. The
            // job refuses to hand over a mesh it found a fault in, and a
            // zero-vertex buffer is its own kind of crash — so the landing
            // pad checks rather than assumes, and the patch already on
            // screen simply stays there.
            if (m_terrainPendingMesh.empty())
            {
                m_terrainJob.store(TerrainJob::Idle, std::memory_order_release);
                return;
            }
            if (m_terrainMeshSlots[0] == 0xFFFFFFFFu)
            {
                m_terrainMeshSlots[0] =
                    registerMesh(renderer().createMesh(m_terrainPendingMesh));
                m_terrainMeshSlots[1] =
                    registerMesh(renderer().createMesh(m_terrainPendingMesh));
                m_terrainSlotIndex = 0;
            }
            else
            {
                // DOUBLE BUFFERED: the buffers being destroyed here belong to
                // the build BEFORE last, seconds old and long out of flight.
                // (Replacing the mesh drawn last frame is what used to force
                // a full renderer().waitIdle() — a guaranteed pipeline bubble
                // on every rebuild.)
                m_terrainSlotIndex ^= 1u;
                m_meshes[m_terrainMeshSlots[m_terrainSlotIndex]] =
                    renderer().createMesh(m_terrainPendingMesh);
            }
            m_terrainMeshSlot = m_terrainMeshSlots[m_terrainSlotIndex];
            // KEEP THE GROUND GRID. It is what lets the grass be re-centred
            // without rebuilding the terrain — and, more importantly, what
            // makes the field stand on exactly the surface that is DRAWN.
            // A second sampling of the heightfield would put it up to half a
            // metre out, which on a 0.6 m blade is half the blade.
            m_terrainGridCells = m_terrainPendingCells;
            m_terrainGridEast = m_terrainPendingEast;
            m_terrainGridNorth = m_terrainPendingNorth;
            {
                const sw::usize gridVertices =
                    static_cast<sw::usize>(m_terrainGridCells + 1) *
                    (m_terrainGridCells + 1);
                m_terrainGridVertices.assign(
                    m_terrainPendingMesh.vertices.begin(),
                    m_terrainPendingMesh.vertices.begin() +
                        static_cast<std::ptrdiff_t>(
                            std::min(gridVertices, m_terrainPendingMesh.vertices.size())));
            }
            m_grassCenterDir = sw::Vec3(0.0f); // force the field to re-seed
            m_terrainOriginLocal = m_terrainPendingOrigin;
            m_terrainCenterDir = m_terrainPendingCenterDir;
            m_terrainExtent = m_terrainPendingExtent;
            m_terrainBody = m_terrainPendingBody;
            m_terrainPendingMesh = sw::MeshData{}; // release the CPU copy
            m_terrainJob.store(TerrainJob::Idle, std::memory_order_release);
        }

        const bool wasVisible = m_terrainVisible;
        m_terrainVisible = false;
        const sw::i32 primaryIndex = controlledPrimaryIndex();
        if (primaryIndex < 0)
        {
            return;
        }
        const auto& primary =
            m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
        const auto* terrain =
            m_world.tryGetComponent<sw::planet::TerrainComponent>(primary.entity);
        const auto* bodyTransform =
            m_world.tryGetComponent<TransformComponent>(primary.entity);
        if (terrain == nullptr || bodyTransform == nullptr)
        {
            return;
        }
        const auto& craft = m_world.getComponent<TransformComponent>(controlledEntity());
        const sw::WorldVec3 radial = craft.position - bodyTransform->position;
        const sw::f64 distance = glm::length(radial);
        const sw::f64 seaAltitude = distance - primary.bodyRadius;
        if (distance <= 1.0 || seaAltitude > 1.2e5)
        {
            // Terrain detail only matters below ~120 km. Above that the
            // globe's own per-fragment surface is what you see, and building
            // a patch of analytic heightfield bought nothing but a hitch.
            return;
        }
        m_terrainVisible = m_terrainMeshSlot != 0xFFFFFFFFu;

        if (m_terrainJob.load(std::memory_order_acquire) != TerrainJob::Idle)
        {
            return; // a build is already in flight; keep showing the current one
        }

        // Patch center: the body-frame direction under the craft. Extent
        // scales with altitude — a wide, coarse patch from high up, a
        // tight, dense one near the ground. That scaling IS the LOD.
        // The body frame, from the body's f64 spin state. An f32 quaternion
        // slides this direction by ~1e-7 rad, which at 6,371 km is 0.6 m of
        // ground — enough to move the patch under your feet every frame.
        const auto* bodySpin =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(primary.entity);
        const glm::dquat inverseRotation =
            (bodySpin != nullptr) ? glm::inverse(sw::phys::spinRotation(*bodySpin))
                                  : glm::inverse(glm::dquat(bodyTransform->rotation));
        const sw::Vec3 centerDir =
            sw::Vec3(glm::normalize(inverseRotation * (radial / distance)));
        // HOW HIGH ABOVE THE GROUND — not above the sea.
        //
        // This is the bug that buried the rocket. The patch's resolution is
        // chosen from your altitude, and it was measuring that from SEA
        // LEVEL: standing on an 1,100 m plateau, the game sized the patch
        // for somebody flying at 1,100 m and drew a 6.6 km square in 137 m
        // cells. A mesh that coarse cannot follow a creased fractal, and
        // measured against the collider the drawn ground was off by up to
        // 7.7 m — which is how a 2.4 m rocket ends up half-submerged in a
        // hillside it is, as far as physics is concerned, resting neatly on
        // top of. Terra's terrain reaches 9 km; almost nowhere interesting
        // is at sea level.
        const sw::f64 groundAltitude =
            distance -
            (primary.bodyRadius +
             sw::planet::terrainElevation(*terrain, centerDir));
        // Down to 1.5 km at landing: with 192 cells that is a 15.6 m grid,
        // fine enough for the gullies and benches the 16-octave heightfield
        // carries. (It used to bottom out at 4 km / 125 m cells, which could
        // not show anything smaller than a hill.)
        const sw::f64 extent =
            std::clamp(std::max(groundAltitude, 250.0) * 6.0, 1.5e3, 4.0e5);

        const sw::f64 now = clock().totalSeconds();
        // HOW FAR YOU MAY WALK BEFORE THE PATCH FOLLOWS YOU.
        //
        // Thirty per cent of the extent — 450 m at landing scale — is the
        // right answer for the GROUND, whose vertices are anchored to the
        // planet and therefore identical before and after a re-centre: you
        // cannot see that rebuild happen at all. It is the wrong answer by
        // an order of magnitude for the FIELD standing on it, which only
        // exists within a disc around the patch centre. Walk out of that
        // disc and there is no grass; wait for the patch to re-centre and a
        // whole new field arrives in one frame while the old one leaves in
        // the same one.
        //
        // So a patch that carries plants follows the player at the scale of
        // the field rather than the scale of the terrain. The ground pays
        // for it — a 60 ms rebuild every 40 m instead of every 450 m, which
        // walking is one every ten seconds — and pays it on a worker thread,
        // against a patch already on screen, at most once a second.
        // The grass has its own clock now (see updateGrassField), so the
        // ground is free to go back to the threshold that suits it: its
        // vertices are planet-anchored and a re-centre is invisible.
        const sw::f64 followDistance = extent * 0.30;
        const bool moved =
            static_cast<sw::f64>(glm::distance(centerDir, m_terrainCenterDir)) *
                primary.bodyRadius >
            followDistance;
        const bool rescaled =
            extent > m_terrainExtent * 1.8 || extent < m_terrainExtent * 0.55;
        const bool needRebuild = m_terrainMeshSlot == 0xFFFFFFFFu ||
                                 m_terrainBody != primary.entity || moved || rescaled;
        // Rebuild cadence scales with altitude: near the ground the patch is
        // small and the craft crosses it quickly, high up it spans hundreds
        // of kilometres and nothing moves relative to it.
        const sw::f64 rebuildInterval = std::clamp(seaAltitude / 12000.0, 1.0, 8.0);
        if (!needRebuild ||
            (wasVisible && now - m_lastTerrainRebuildSeconds < rebuildInterval))
        {
            return; // keep showing the current patch
        }
        m_lastTerrainRebuildSeconds = now;

        // The body's palette style (Terra/Luna/Mars) comes from its LOD
        // component — the patch is a close-up of that same world.
        sw::i32 surfaceStyle = 0;
        if (const auto* bodyLod =
                m_world.tryGetComponent<CelestialLodComponent>(primary.entity);
            bodyLod != nullptr && bodyLod->surfaceStyle >= 0)
        {
            surfaceStyle = bodyLod->surfaceStyle;
        }

        // WHERE THE SUN IS, in the body's own frame — because the relief
        // shading below is BAKED, and a baked shadow has to be baked toward
        // something. Terra turns 0.004 degrees in the second between two
        // rebuilds, so a shadow map that is one rebuild old is a shadow map
        // that is right.
        sw::Vec3 sunDirBody{0.0f, 1.0f, 0.0f};
        if (const auto* sunTransform =
                m_world.tryGetComponent<TransformComponent>(m_solEntity))
        {
            const sw::WorldVec3 toSun = sunTransform->position - bodyTransform->position;
            if (glm::length(toSun) > 1.0)
            {
                sunDirBody =
                    sw::Vec3(glm::normalize(inverseRotation * glm::normalize(toSun)));
            }
        }

        // Everything the build needs is captured BY VALUE: the job never
        // touches the world, the renderer or any component.
        const sw::planet::TerrainComponent terrainCopy = *terrain;
        const sw::f64 radius = primary.bodyRadius;
        m_terrainPendingBody = primary.entity;
        m_terrainJob.store(TerrainJob::Running, std::memory_order_release);
        // CELLS ARE NOT A CONSTANT, because what matters is the cell SIZE
        // where somebody is standing. Measured against the collider at
        // landing extent, going from 96 cells to 192 takes the worst gap
        // between the drawn ground and the ground you stand on from 1.25 m
        // down to 0.50 m — and nothing at four hundred kilometres up cares
        // either way, so the big patches keep the cheap grid.
        const sw::u32 cells = (extent <= 2.5e3) ? 192u
                              : (extent <= 2.5e4) ? 128u
                                                  : 96u;
        threadPool().submit([this, terrainCopy, surfaceStyle, centerDir, extent, radius,
                             cells, sunDirBody]() {
            buildTerrainPatch(terrainCopy, surfaceStyle, centerDir, extent, radius,
                              cells, sunDirBody);
            m_terrainJob.store(TerrainJob::Ready, std::memory_order_release);
        });
    }

    void StarWorksGame::buildTerrainPatch(const sw::planet::TerrainComponent& terrain,
                                          sw::i32 surfaceStyle,
                                          const sw::Vec3& centerDir, sw::f64 extent,
                                          sw::f64 radius, sw::u32 cellCount,
                                          const sw::Vec3& sunDirBody)
    {
        // ---- the grid: a tangent plane projected onto the sphere ----------
        const sw::u32 kCells = std::clamp(cellCount, 16u, 256u);
        const sw::u32 kVerts = kCells + 1;
        const sw::Vec3 reference =
            (std::abs(centerDir.y) < 0.99f) ? sw::Vec3{0, 1, 0} : sw::Vec3{1, 0, 0};
        const sw::Vec3 east = glm::normalize(glm::cross(reference, centerDir));
        const sw::Vec3 north = glm::cross(centerDir, east);
        const sw::WorldVec3 origin =
            sw::WorldVec3(centerDir) *
            (radius + sw::planet::terrainElevation(terrain, centerDir));

        // The grid cell size decides how much relief detail is even
        // representable here: sampling sixteen octaves onto a mesh whose
        // cells are kilometres wide only aliases between vertices, and costs
        // the full price of the heightfield at every one of the ~9,400
        // points. Near the ground the cells are 31 m and the count saturates
        // at the body's full stack — which is exactly where it should.
        const sw::f64 cellMetres = 2.0 * extent / kCells;
        // Same convention as the globe LODs: the finest noise frequency a
        // mesh of this spacing can carry, in cycles per radian.
        const sw::f32 patchFrequencyLimit =
            static_cast<sw::f32>(radius / (2.0 * std::max(cellMetres, 1.0e-3)));
        sw::i32 patchOctaves = terrain.reliefOctaves;
        // Octaves are kept until their wavelength drops to a QUARTER of a cell,
        // not twice one. The looser rule saved a couple of samples and cost
        // something worse: collision samples the FULL stack, so the drawn
        // ground sat up to three metres below the ground the lander stood
        // on. At landing extent this now keeps the whole stack — the patch
        // and the collider are the same surface again, to the centimetre.
        while (patchOctaves > 3 &&
               radius / (static_cast<sw::f64>(terrain.frequency) *
                         terrain.reliefFrequency * std::pow(2.07, patchOctaves - 1)) <
                   cellMetres * 0.25)
        {
            --patchOctaves;
        }

        std::vector<sw::WorldVec3> points(kVerts * kVerts);
        std::vector<sw::f32> elevations(kVerts * kVerts);
        /// The SOLID ground height at each vertex — sea level clamped in,
        /// negative sea floor clamped out. This is the surface the relief
        /// shading marches over, and it is the same array the geometry was
        /// built from, so a shadow can never fall on ground that is not
        /// drawn where the shadow says it is.
        std::vector<sw::f32> ground(kVerts * kVerts);
        for (sw::u32 j = 0; j < kVerts; ++j)
        {
            for (sw::u32 i = 0; i < kVerts; ++i)
            {
                const sw::f64 u =
                    (static_cast<sw::f64>(i) / kCells * 2.0 - 1.0) * extent;
                const sw::f64 v =
                    (static_cast<sw::f64>(j) / kCells * 2.0 - 1.0) * extent;
                const sw::WorldVec3 raw = sw::WorldVec3(centerDir) * radius +
                                          sw::WorldVec3(east) * u +
                                          sw::WorldVec3(north) * v;
                const sw::WorldVec3 dir = glm::normalize(raw);
                // SIGNED elevation: the geometry clamps at sea level (the
                // sphere IS the water surface, exactly as collision sees it)
                // but the stored value keeps the negative sea floor, which
                // is what colors deep water.
                const sw::f32 signedElevation = sw::planet::terrainElevationSignedLod(
                    terrain, sw::Vec3(dir), patchOctaves);
                const sw::f64 elevation =
                    (signedElevation > 0.0f) ? static_cast<sw::f64>(signedElevation)
                                             : 0.0;
                points[j * kVerts + i] = dir * (radius + elevation) - origin;
                elevations[j * kVerts + i] = signedElevation;
                ground[j * kVerts + i] = static_cast<sw::f32>(elevation);
            }
        }

        // ---- RELIEF SHADING, baked -----------------------------------------
        //
        // The patch's triangles are 15 m across and its normals are honest,
        // but a lambert term alone leaves rolling ground looking like a
        // painted sheet: nothing casts, nothing pools, and a dip and a rise
        // of the same slope are the same colour. Two terms fix that, and
        // both are computed on the height grid that is already in hand — no
        // extra heightfield evaluations, and no possibility of disagreeing
        // with the surface being drawn.
        //
        //   CAST SHADOW  march toward the sun, one cell at a time, and see
        //                whether the ground ever rises above the ray. This
        //                is what puts a hill's shadow on the valley beside
        //                it, and it is the term that makes a landscape read
        //                as a landscape at low sun.
        //   SKY OCCLUSION  the same march in six directions with no sun in
        //                it: how much of the sky this point can see. Gullies
        //                and the insides of craters darken; ridges do not.
        //
        // Both stay deliberately gentle. This is ground the player has to
        // land on and walk over, not a photograph.
        const sw::f64 cellSpacing = 2.0 * extent / kCells;
        std::vector<sw::f32> shading(kVerts * kVerts, 1.0f);
        {
            const sw::f32 sunUp = glm::dot(sunDirBody, centerDir);
            const sw::Vec3 sunTangent = sunDirBody - centerDir * sunUp;
            const sw::f32 sunTangentLength = glm::length(sunTangent);
            // Direction the shadow ray walks, in GRID cells.
            sw::f32 sunU = 0.0f;
            sw::f32 sunV = 0.0f;
            if (sunTangentLength > 1.0e-5f)
            {
                sunU = glm::dot(sunTangent, east) / sunTangentLength;
                sunV = glm::dot(sunTangent, north) / sunTangentLength;
            }
            // Metres of climb per metre walked toward the sun. A sun on the
            // horizon casts shadows to infinity; cap the slope so the march
            // stays finite and the terminator stays soft.
            const sw::f32 sunSlope =
                (sunTangentLength > 1.0e-5f)
                    ? glm::clamp(sunUp / sunTangentLength, -8.0f, 8.0f)
                    : 8.0f;

            const auto heightAt = [&](sw::i32 i, sw::i32 j) {
                const sw::i32 ci = glm::clamp(i, 0, static_cast<sw::i32>(kCells));
                const sw::i32 cj = glm::clamp(j, 0, static_cast<sw::i32>(kCells));
                return ground[static_cast<sw::usize>(cj) * kVerts + ci];
            };

            constexpr sw::i32 kShadowSteps = 20;
            constexpr sw::i32 kSkySteps = 6;
            // Six azimuths, fixed: enough to tell a hollow from a shoulder,
            // cheap enough to run on every vertex of a 192-cell grid.
            constexpr sw::f32 kSkyDirs[6][2] = {{1.0f, 0.0f},   {0.5f, 0.866f},
                                                {-0.5f, 0.866f}, {-1.0f, 0.0f},
                                                {-0.5f, -0.866f}, {0.5f, -0.866f}};

            for (sw::u32 j = 0; j < kVerts; ++j)
            {
                for (sw::u32 i = 0; i < kVerts; ++i)
                {
                    const sw::usize index = static_cast<sw::usize>(j) * kVerts + i;
                    const sw::f32 base = ground[index];

                    // ---- cast shadow ----------------------------------
                    sw::f32 blocked = 0.0f;
                    if (sunUp > -0.05f)
                    {
                        for (sw::i32 step = 1; step <= kShadowSteps; ++step)
                        {
                            const sw::f32 walk =
                                static_cast<sw::f32>(step) *
                                static_cast<sw::f32>(cellSpacing);
                            const sw::f32 rayHeight = base + walk * sunSlope;
                            const sw::f32 terrainHeight = heightAt(
                                static_cast<sw::i32>(i) +
                                    static_cast<sw::i32>(std::lround(sunU * step)),
                                static_cast<sw::i32>(j) +
                                    static_cast<sw::i32>(std::lround(sunV * step)));
                            // Softened by how far the blocker is: a ridge at
                            // the end of the march throws a vaguer shadow
                            // than the boulder at your feet, which is both
                            // true and what keeps the term from banding.
                            const sw::f32 over = terrainHeight - rayHeight;
                            if (over > 0.0f)
                            {
                                const sw::f32 softness =
                                    2.0f + 0.35f * static_cast<sw::f32>(step) *
                                               static_cast<sw::f32>(cellSpacing);
                                blocked = std::max(blocked,
                                                   glm::clamp(over / softness, 0.0f, 1.0f));
                            }
                        }
                    }
                    else
                    {
                        blocked = 1.0f; // the sun is under this horizon
                    }

                    // ---- sky occlusion ---------------------------------
                    sw::f32 openness = 0.0f;
                    for (const auto& direction : kSkyDirs)
                    {
                        sw::f32 highest = 0.0f; // tangent of the horizon angle
                        for (sw::i32 step = 1; step <= kSkySteps; ++step)
                        {
                            const sw::f32 walk =
                                static_cast<sw::f32>(step) *
                                static_cast<sw::f32>(cellSpacing);
                            const sw::f32 rise =
                                heightAt(static_cast<sw::i32>(i) +
                                             static_cast<sw::i32>(
                                                 std::lround(direction[0] * step)),
                                         static_cast<sw::i32>(j) +
                                             static_cast<sw::i32>(
                                                 std::lround(direction[1] * step))) -
                                base;
                            highest = std::max(highest, rise / walk);
                        }
                        // cos of the horizon angle: 1 = open sky, 0 = a wall.
                        openness += 1.0f / std::sqrt(1.0f + highest * highest);
                    }
                    openness /= 6.0f;

                    // Measured on Terra's roughest ground the term runs
                    // 0.50 .. 1.00; on the launch plain it is flat at 1.00,
                    // because that ground really is flat at fifteen metres
                    // and honest shading of flat ground is no shading. What
                    // makes the plain read is the field standing on it.
                    const sw::f32 sunTerm = 1.0f - 0.55f * blocked;
                    const sw::f32 skyTerm = 0.45f + 0.55f * openness;
                    shading[index] = glm::clamp(sunTerm * skyTerm, 0.22f, 1.0f);
                }
            }
        }

        sw::MeshData mesh;
        // Room for the ground, its rim skirt and a full field of plants, so
        // appending never has to move what is already there. Bounded and
        // stated, because everything below appends to this vector.
        mesh.vertices.reserve(static_cast<sw::usize>(kVerts) * kVerts + 16u * kCells +
                              96000u);
        mesh.vertices.resize(static_cast<sw::usize>(kVerts) * kVerts);
        const auto style = static_cast<SurfaceStyle>(surfaceStyle);
        for (sw::u32 j = 0; j < kVerts; ++j)
        {
            for (sw::u32 i = 0; i < kVerts; ++i)
            {
                const sw::usize index = j * kVerts + i;
                sw::Vertex& vertex = mesh.vertices[index];
                vertex.position = sw::Vec3(points[index]);

                // Finite-difference normal (real slope shading).
                const sw::u32 iPrev = (i > 0) ? i - 1 : i;
                const sw::u32 iNext = (i < kCells) ? i + 1 : i;
                const sw::u32 jPrev = (j > 0) ? j - 1 : j;
                const sw::u32 jNext = (j < kCells) ? j + 1 : j;
                const sw::Vec3 dx =
                    sw::Vec3(points[j * kVerts + iNext] - points[j * kVerts + iPrev]);
                const sw::Vec3 dy =
                    sw::Vec3(points[jNext * kVerts + i] - points[jPrev * kVerts + i]);
                vertex.normal = glm::normalize(glm::cross(dx, dy));

                // Palette: the SAME function the globe LODs and the fragment
                // shader use — walking down from orbit never crosses a color
                // seam. The slope comes straight from the finite-difference
                // normal we just built (tan of the angle to the local
                // vertical), so cliffs are bare rock here too.
                const sw::Vec3 vertexDir =
                    sw::Vec3(glm::normalize(origin + sw::WorldVec3(vertex.position)));
                const sw::f32 cosine =
                    glm::clamp(glm::dot(vertex.normal, vertexDir), 0.05f, 1.0f);
                const sw::f32 slope =
                    std::sqrt(std::max(0.0f, 1.0f - cosine * cosine)) / cosine;
                // The patch's own Nyquist limit: metre-scale cells resolve
                // every frequency in the palette, so nothing is faded here.
                colorizeSurfaceVertex(vertex, style, vertexDir, elevations[index],
                                      slope, terrain, patchFrequencyLimit);
                // The baked relief term multiplies the ALBEDO, so it stacks
                // with the shader's own lambert instead of replacing it: a
                // slope facing the sun is bright, a slope facing the sun
                // from inside somebody else's shadow is not.
                const sw::f32 relief = shading[index];
                vertex.color.r *= relief;
                vertex.color.g *= relief;
                vertex.color.b *= relief;
            }
        }
        mesh.indices.reserve(kCells * kCells * 6);
        for (sw::u32 j = 0; j < kCells; ++j)
        {
            for (sw::u32 i = 0; i < kCells; ++i)
            {
                const sw::u32 a = j * kVerts + i;
                const sw::u32 b = a + 1;
                const sw::u32 c = a + kVerts;
                const sw::u32 d = c + 1;
                // CCW seen from OUTSIDE the planet (+up): front faces out.
                mesh.indices.insert(mesh.indices.end(), {a, b, c, b, d, c});
            }
        }

        // ---- THE RIM SKIRT --------------------------------------------------
        //
        // The patch is a sheet laid over the globe, and the globe is a second
        // ground surface underneath it — 133 km between vertices at this
        // level of detail, so within a few kilometres of the camera it is
        // one enormous flat triangle. It has to stay: beyond the patch's
        // 1.5 km rim it IS the horizon. But it should never be SEEN, and at
        // the rim it was: the sheet simply stopped, and the eye followed the
        // cut straight down onto the surface below.
        //
        // A skirt closes it. One ring of quads dropped from the border
        // vertices, darkened like a cut bank, costing 4 x kCells triangles —
        // half a per cent of the patch. Nothing else about the second
        // surface needs changing, because front-to-back batching plus the
        // early depth test already reject every one of its hidden fragments
        // before it evaluates a noise octave.
        {
            const sw::f32 skirtDrop =
                static_cast<sw::f32>(std::max(200.0, extent * 0.35));
            const auto addSkirt = [&](sw::u32 i, sw::u32 j, sw::u32 iNext, sw::u32 jNext) {
                const sw::usize a = static_cast<sw::usize>(j) * kVerts + i;
                const sw::usize b = static_cast<sw::usize>(jNext) * kVerts + iNext;
                const sw::u32 first = static_cast<sw::u32>(mesh.vertices.size());
                const sw::Vec3 down = -centerDir * skirtDrop;

                // BY VALUE, and this is not a style preference. Reading the
                // two rim vertices through REFERENCES into `mesh.vertices`
                // and then pushing onto that same vector is a use-after-free
                // the moment the push reallocates — which it does on the very
                // first quad, because the vector was sized exactly to the
                // ground grid. It cost a crash the instant a patch with a
                // skirt was built, and it is the reason the grass appeared to
                // be at fault: the grass is simply what the same build
                // produces next.
                const sw::Vertex topA = mesh.vertices[a];
                const sw::Vertex topB = mesh.vertices[b];

                // The rim's outward normal: along the edge, turned a quarter
                // turn about the local vertical. Degenerate edges (two rim
                // vertices at the same place) would normalise a zero vector
                // into NaN and hand the GPU a mesh full of them, so the
                // fallback is stated rather than hoped for.
                sw::Vec3 along = topB.position - topA.position;
                if (glm::dot(along, along) < 1.0e-12f)
                {
                    along = east;
                }
                const sw::Vec3 outward = glm::cross(centerDir, glm::normalize(along));
                const sw::Vec3 normal = (glm::dot(outward, outward) > 1.0e-12f)
                                            ? glm::normalize(outward)
                                            : centerDir;

                for (sw::u32 corner = 0; corner < 4; ++corner)
                {
                    const sw::Vertex& source = (corner % 2 == 0) ? topA : topB;
                    sw::Vertex vertex = source;
                    if (corner >= 2)
                    {
                        vertex.position += down;
                        vertex.color = sw::Vec4(sw::Vec3(source.color) * 0.45f, 1.0f);
                    }
                    vertex.normal = normal;
                    mesh.vertices.push_back(vertex);
                }
                // Both windings: which side of the rim faces the camera
                // depends on which edge of the patch it is.
                mesh.indices.insert(mesh.indices.end(),
                                    {first, first + 2, first + 1, first + 1, first + 2,
                                     first + 3, first, first + 1, first + 2, first + 1,
                                     first + 3, first + 2});
            };
            for (sw::u32 i = 0; i < kCells; ++i)
            {
                addSkirt(i, 0, i + 1, 0);
                addSkirt(i, kCells, i + 1, kCells);
                addSkirt(0, i, 0, i + 1);
                addSkirt(kCells, i, kCells, i + 1);
            }
        }

        // ---- THE GUARD ------------------------------------------------------
        //
        // Everything above APPENDS to one vertex vector, and a geometry bug
        // in an append is invisible to the compiler and silent at runtime
        // until a driver chokes on it. One pass over the finished mesh turns
        // that whole class of fault into a log line and a patch that is
        // simply not shown: 0.3 ms against a 60 ms build, which is nothing
        // for never handing the GPU a NaN.
        {
            sw::usize bad = 0;
            for (const sw::Vertex& vertex : mesh.vertices)
            {
                if (!std::isfinite(vertex.position.x) || !std::isfinite(vertex.position.y) ||
                    !std::isfinite(vertex.position.z) || !std::isfinite(vertex.normal.x) ||
                    !std::isfinite(vertex.normal.y) || !std::isfinite(vertex.normal.z))
                {
                    bad += 1;
                }
            }
            const sw::u32 vertexCount = static_cast<sw::u32>(mesh.vertices.size());
            for (const sw::u32 index : mesh.indices)
            {
                if (index >= vertexCount)
                {
                    bad += 1;
                    break;
                }
            }
            if (bad != 0)
            {
                SW_LOG_ERROR("Terrain",
                             "Patch build produced {} bad vertices or an out-of-range "
                             "index ({} vertices, {} indices) - discarding",
                             bad, vertexCount, mesh.indices.size());
                return; // the previous patch keeps being shown
            }
        }

        m_terrainPendingMesh = std::move(mesh);
        m_terrainPendingOrigin = origin;
        m_terrainPendingCenterDir = centerDir;
        m_terrainPendingExtent = extent;
        m_terrainPendingCells = kCells;
        m_terrainPendingEast = east;
        m_terrainPendingNorth = north;
    }


    // ------------------------------------------------------------------------
    // THE GRASS FIELD
    //
    // Its own geometry, on its own clock, cut into chunks that go to the GPU
    // one per frame. It reads the ground grid the terrain patch already
    // produced, so every blade stands on exactly the surface that is drawn —
    // and so re-centring the field costs nothing on the heightfield.
    // ------------------------------------------------------------------------
    void StarWorksGame::buildGrassField(const std::vector<sw::Vertex>& groundGrid,
                                        sw::u32 cellCount, const sw::Vec3& centerDir,
                                        const sw::Vec3& east, const sw::Vec3& north,
                                        sw::f64 extent, sw::f64 radius,
                                        const sw::Vec3& fieldDir)
    {
        for (sw::MeshData& chunk : m_grassPending)
        {
            chunk.vertices.clear();
            chunk.indices.clear();
            chunk.vertices.reserve(16000);
            chunk.indices.reserve(48000);
        }
        const sw::u32 verts = cellCount + 1;
        if (groundGrid.size() < static_cast<sw::usize>(verts) * verts)
        {
            return;
        }

        const auto hash01 = [](sw::i64 a, sw::i64 b, sw::u32 salt) {
            sw::u64 h = static_cast<sw::u64>(a) * 0x9E3779B97F4A7C15ull;
            h ^= static_cast<sw::u64>(b) * 0xC2B2AE3D27D4EB4Full;
            h ^= static_cast<sw::u64>(salt) * 0x165667B19E3779F9ull;
            h ^= h >> 29;
            h *= 0xBF58476D1CE4E5B9ull;
            h ^= h >> 32;
            return static_cast<sw::f32>(h & 0xFFFFFFull) / 16777215.0f;
        };

        // Where the PLAYER is, in the patch's own tangent chart. The field
        // follows them; the chart does not move.
        const sw::f64 fieldU = static_cast<sw::f64>(glm::dot(fieldDir, east)) * radius;
        const sw::f64 fieldV = static_cast<sw::f64>(glm::dot(fieldDir, north)) * radius;

        // The lattice is anchored to the PLANET: absolute plate-carrée metres,
        // so a tuft keeps its cell — and therefore its jitter, its height and
        // its colour — no matter which patch or which field it lands in.
        const sw::f64 latitude =
            std::asin(glm::clamp(static_cast<sw::f64>(centerDir.y), -1.0, 1.0));
        const sw::f64 longitude = std::atan2(static_cast<sw::f64>(centerDir.x),
                                             static_cast<sw::f64>(centerDir.z));
        const sw::f64 anchorU = longitude * radius * std::cos(latitude) + fieldU;
        const sw::f64 anchorV = latitude * radius + fieldV;

        constexpr sw::f64 kSpacing = 1.5;
        const sw::f64 plantRadius = std::min(extent * 0.20, 260.0);
        const sw::f64 plantFull = 30.0;  // full density inside this
        const sw::f64 plantFade = 0.82;  // height fades over the last 18 %

        // Bilinear read of the ground the patch drew. Positions, colours and
        // normals all come from the same four vertices, so a blade stands on
        // the surface, is lit like it, and is coloured by it.
        const auto sampleGrid = [&](sw::f64 u, sw::f64 v, sw::Vec3& outPoint,
                                    sw::Vec4& outColor, sw::Vec3& outNormal) {
            const sw::f64 fx = (u / extent * 0.5 + 0.5) * cellCount;
            const sw::f64 fy = (v / extent * 0.5 + 0.5) * cellCount;
            if (fx < 0.0 || fy < 0.0 || fx >= cellCount || fy >= cellCount)
            {
                return false;
            }
            const sw::u32 i = static_cast<sw::u32>(fx);
            const sw::u32 j = static_cast<sw::u32>(fy);
            const sw::f32 a = static_cast<sw::f32>(fx - i);
            const sw::f32 b = static_cast<sw::f32>(fy - j);
            const sw::usize i00 = static_cast<sw::usize>(j) * verts + i;
            const sw::usize i10 = i00 + 1;
            const sw::usize i01 = i00 + verts;
            const sw::usize i11 = i01 + 1;
            const auto mix2 = [a, b](auto p00, auto p10, auto p01, auto p11) {
                return (p00 * (1.0f - a) + p10 * a) * (1.0f - b) +
                       (p01 * (1.0f - a) + p11 * a) * b;
            };
            outPoint = mix2(groundGrid[i00].position, groundGrid[i10].position,
                            groundGrid[i01].position, groundGrid[i11].position);
            outColor = mix2(groundGrid[i00].color, groundGrid[i10].color,
                            groundGrid[i01].color, groundGrid[i11].color);
            outNormal = glm::normalize(mix2(groundGrid[i00].normal, groundGrid[i10].normal,
                                            groundGrid[i01].normal, groundGrid[i11].normal));
            return true;
        };

        const sw::i64 uFirst =
            static_cast<sw::i64>(std::floor((anchorU - plantRadius) / kSpacing));
        const sw::i64 uLast =
            static_cast<sw::i64>(std::ceil((anchorU + plantRadius) / kSpacing));
        const sw::i64 vFirst =
            static_cast<sw::i64>(std::floor((anchorV - plantRadius) / kSpacing));
        const sw::i64 vLast =
            static_cast<sw::i64>(std::ceil((anchorV + plantRadius) / kSpacing));

        for (sw::i64 kv = vFirst; kv <= vLast; ++kv)
        {
            for (sw::i64 ku = uFirst; ku <= uLast; ++ku)
            {
                // Cell -> patch-local metres, jittered by a function OF THE
                // CELL, so the jitter travels with the planet and not with
                // the field.
                const sw::f32 jitterU = hash01(ku, kv, 11u) - 0.5f;
                const sw::f32 jitterV = hash01(ku, kv, 23u) - 0.5f;
                const sw::f64 localU = static_cast<sw::f64>(ku) * kSpacing - anchorU +
                                       static_cast<sw::f64>(jitterU) * kSpacing * 0.85;
                const sw::f64 localV = static_cast<sw::f64>(kv) * kSpacing - anchorV +
                                       static_cast<sw::f64>(jitterV) * kSpacing * 0.85;
                const sw::f64 distanceSquared = localU * localU + localV * localV;
                if (distanceSquared > plantRadius * plantRadius)
                {
                    continue;
                }
                // ...and back into the patch's chart, which is offset from the
                // field's by where the player stands in it.
                const sw::f64 u = localU + fieldU;
                const sw::f64 v = localV + fieldV;

                sw::Vec3 base{0.0f};
                sw::Vec4 groundColor{0.0f};
                sw::Vec3 groundNormal{0.0f};
                if (!sampleGrid(u, v, base, groundColor, groundNormal))
                {
                    continue;
                }
                // A cliff face is bare, and it is bare because it is a cliff.
                if (glm::dot(groundNormal, centerDir) < 0.88f)
                {
                    continue;
                }
                // WHAT THE GROUND ITSELF SAYS. Green ground grows things; rock,
                // ice, dune and open water do not — read straight off the
                // palette, so a plant can never appear on a colour that would
                // not support it and no biome table has to be kept in step
                // with the one the terrain already has.
                const sw::f32 green =
                    groundColor.g - 0.5f * (groundColor.r + groundColor.b);
                sw::f32 density = sw::math::smoothstepf(-0.005f, 0.045f, green);
                density *= 0.35f + 0.9f * hash01(ku / 13, kv / 13, 91u);
                // Thinned as an inverse power rather than a ramp: cover within
                // thirty metres, texture beyond, and a total that stays near
                // six thousand tufts however far the field is asked to reach.
                const sw::f64 distance = std::sqrt(distanceSquared);
                if (distance > plantFull)
                {
                    density *= static_cast<sw::f32>(std::pow(plantFull / distance, 1.5));
                }
                if (hash01(ku, kv, 57u) > density)
                {
                    continue;
                }
                // Fade LATE, so the band sits beyond anything a re-centre can
                // move a visible tuft across.
                const sw::f32 edge =
                    1.0f - sw::math::smoothstepf(static_cast<sw::f32>(plantFade), 1.0f,
                                                 static_cast<sw::f32>(distance /
                                                                      plantRadius));
                if (edge <= 0.02f)
                {
                    continue;
                }
                const sw::f32 height =
                    (0.22f + 0.55f * hash01(ku, kv, 131u)) * edge * (0.6f + density);
                if (height < 0.06f)
                {
                    continue;
                }
                const sw::f32 width = height * (0.28f + 0.16f * hash01(ku, kv, 77u));

                const sw::Vec3 leaf =
                    glm::mix(sw::Vec3(groundColor), sw::Vec3(0.20f, 0.34f, 0.12f),
                             0.55f + 0.25f * hash01(ku, kv, 197u));
                const sw::Vec4 rootColor{leaf * 0.55f, 1.0f};
                const sw::Vec4 tipColor{leaf * (1.05f + 0.25f * hash01(ku, kv, 211u)),
                                        1.0f};

                // WHICH CHUNK. By hash, so the six of them are the same size
                // and each is spread over the whole field — the set is only
                // ever shown complete, so this is about balancing the six
                // uploads, not about what appears first.
                const sw::u32 chunkIndex =
                    std::min(static_cast<sw::u32>(hash01(ku, kv, 777u) *
                                                  static_cast<sw::f32>(kGrassChunks)),
                             kGrassChunks - 1u);
                sw::MeshData& chunk = m_grassPending[chunkIndex];

                for (sw::u32 blade = 0; blade < 3; ++blade)
                {
                    const sw::f32 yaw =
                        (hash01(ku, kv, 300u + blade) + static_cast<sw::f32>(blade)) *
                        2.0943951f;
                    const sw::Vec3 lean = east * std::cos(yaw) + north * std::sin(yaw);
                    const sw::Vec3 across = glm::normalize(glm::cross(centerDir, lean));
                    const sw::f32 bend =
                        height * (0.25f + 0.35f * hash01(ku, kv, 400u + blade));

                    const sw::Vec3 root = base;
                    const sw::Vec3 tip = root + centerDir * height + lean * bend;
                    const sw::Vec3 normal =
                        glm::normalize(centerDir * 0.72f + across * 0.28f);

                    const sw::u32 first = static_cast<sw::u32>(chunk.vertices.size());
                    const sw::Vec3 offsets[4] = {root - across * (width * 0.5f),
                                                 root + across * (width * 0.5f),
                                                 tip - across * (width * 0.12f),
                                                 tip + across * (width * 0.12f)};
                    for (sw::u32 corner = 0; corner < 4; ++corner)
                    {
                        sw::Vertex vertex{};
                        vertex.position = offsets[corner];
                        vertex.normal = normal;
                        vertex.color = (corner < 2) ? rootColor : tipColor;
                        vertex.uv = {0.05f, 0.15f}; // matte: leaves do not shine
                        chunk.vertices.push_back(vertex);
                    }
                    // Both windings, so a blade is never invisible from the
                    // side the culler happens to be looking from.
                    chunk.indices.insert(chunk.indices.end(),
                                         {first, first + 1, first + 2, first + 1,
                                          first + 3, first + 2, first + 2, first + 1,
                                          first, first + 2, first + 3, first + 1});
                }
            }
        }
    }

    void StarWorksGame::updateGrassField()
    {
        // ---- 1. land one chunk per frame ------------------------------------
        //
        // ONE. `uploadToBuffer` submits its copy and then waits on a fence,
        // which drains whatever the graphics queue is holding — so every
        // upload costs up to a frame of GPU work whatever its size. Doing six
        // small ones on six frames turns one visible spike into six frames
        // nobody notices, and the field on screen never flickers because the
        // OLD set keeps drawing until the last new chunk has landed.
        if (m_grassJob.load(std::memory_order_acquire) == TerrainJob::Ready)
        {
            const sw::u32 target = m_grassSet ^ 1u;
            if (m_grassUploadCursor < kGrassChunks)
            {
                const sw::u32 index = m_grassUploadCursor;
                m_grassChunkValid[target][index] = !m_grassPending[index].empty();
                if (m_grassChunkValid[target][index])
                {
                    if (m_grassSlots[target][index] == 0xFFFFFFFFu)
                    {
                        m_grassSlots[target][index] =
                            registerMesh(renderer().createMesh(m_grassPending[index]));
                    }
                    else
                    {
                        m_meshes[m_grassSlots[target][index]] =
                            renderer().createMesh(m_grassPending[index]);
                    }
                }
                m_grassUploadCursor += 1;
                return; // one upload per frame, and not one more
            }
            // Every chunk has landed: show the new field and release the CPU
            // copies.
            m_grassSet = target;
            m_grassLiveCount = kGrassChunks;
            m_grassCenterDir = m_grassPendingCenterDir;
            m_grassOriginLocal = m_grassPendingOriginLocal;
            m_grassBody = m_terrainBody;
            for (sw::MeshData& chunk : m_grassPending)
            {
                chunk = sw::MeshData{};
            }
            m_grassJob.store(TerrainJob::Idle, std::memory_order_release);
            return;
        }
        if (m_grassJob.load(std::memory_order_acquire) != TerrainJob::Idle)
        {
            return; // a field is being seeded
        }

        // ---- 2. does the field need to move? --------------------------------
        if (m_terrainGridVertices.empty() || m_terrainGridCells == 0 ||
            m_terrainExtent > 2.5e3)
        {
            m_grassLiveCount = 0; // too high up for a field to mean anything
            return;
        }
        const sw::i32 primaryIndex = controlledPrimaryIndex();
        if (primaryIndex < 0 || m_terrainBody.isNull())
        {
            return;
        }
        const auto& primary =
            m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
        const auto* bodyTransform =
            m_world.tryGetComponent<TransformComponent>(m_terrainBody);
        const auto* bodySpin =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(m_terrainBody);
        if (bodyTransform == nullptr)
        {
            return;
        }
        const auto& craft = m_world.getComponent<TransformComponent>(controlledEntity());
        const sw::WorldVec3 radial = craft.position - bodyTransform->position;
        const sw::f64 distance = glm::length(radial);
        if (distance <= 1.0)
        {
            return;
        }
        const glm::dquat inverseRotation =
            (bodySpin != nullptr) ? glm::inverse(sw::phys::spinRotation(*bodySpin))
                                  : glm::inverse(glm::dquat(bodyTransform->rotation));
        const sw::Vec3 here =
            sw::Vec3(glm::normalize(inverseRotation * (radial / distance)));

        // FORTY METRES, the scale of the field rather than of the terrain.
        // Any further and a walking player leaves the grass behind.
        const sw::f64 travelled =
            static_cast<sw::f64>(glm::distance(here, m_grassCenterDir)) *
            primary.bodyRadius;
        if (m_grassLiveCount != 0 && travelled < 40.0)
        {
            return;
        }

        // ---- 3. seed it -----------------------------------------------------
        // The grid is copied into the job rather than shared: 1.8 MB and a
        // fifth of a millisecond buys the guarantee that a terrain rebuild
        // landing mid-seed cannot pull the ground out from under it.
        const std::vector<sw::Vertex> grid = m_terrainGridVertices;
        const sw::u32 cells = m_terrainGridCells;
        const sw::Vec3 centerDir = m_terrainCenterDir;
        const sw::Vec3 east = m_terrainGridEast;
        const sw::Vec3 north = m_terrainGridNorth;
        const sw::f64 extent = m_terrainExtent;
        const sw::f64 radius = primary.bodyRadius;
        m_grassPendingCenterDir = here;
        m_grassPendingOriginLocal = m_terrainOriginLocal;
        m_grassUploadCursor = 0;
        m_grassJob.store(TerrainJob::Running, std::memory_order_release);
        threadPool().submit([this, grid, cells, centerDir, east, north, extent, radius,
                             here]() {
            buildGrassField(grid, cells, centerDir, east, north, extent, radius, here);
            m_grassJob.store(TerrainJob::Ready, std::memory_order_release);
        });
    }

    sw::i32 StarWorksGame::controlledPrimaryIndex() const
    {
        if (m_celestialIndex.size() == 0)
        {
            return -1;
        }
        const auto& transform = const_cast<sw::ecs::World&>(m_world)
                                    .getComponent<TransformComponent>(controlledEntity());
        return m_celestialIndex.soiPrimaryAt(transform.position,
                                             m_physicsLane->presentSeconds());
    }

    void StarWorksGame::updateManeuverNodeInput()
    {
        // Map view only: no conflict with flight keys (Shift/Ctrl throttle).
        if (input().wasKeyPressed(sw::KeyCode::N))
        {
            m_nodeActive = !m_nodeActive;
            if (m_nodeActive)
            {
                m_nodeTime = m_physicsLane->presentSeconds() + 120.0;
                m_nodePrograde = 0.0;
                m_nodeNormal = 0.0;
                m_nodeRadial = 0.0;
            }
            m_lastPredictionSeconds = -1.0e9; // recompute now
            SW_LOG_INFO("Game", "Maneuver node {}", m_nodeActive ? "created" : "deleted");
        }
        if (!m_nodeActive)
        {
            return;
        }

        // Both sides of the keyboard: a player holding right shift is asking
        // for the same thing as one holding left shift.
        const bool shift = input().isKeyDown(sw::KeyCode::LeftShift) ||
                           input().isKeyDown(sw::KeyCode::RightShift);
        const bool control = input().isKeyDown(sw::KeyCode::LeftControl) ||
                             input().isKeyDown(sw::KeyCode::RightControl);
        const bool alt = input().isKeyDown(sw::KeyCode::LeftAlt) ||
                         input().isKeyDown(sw::KeyCode::RightAlt);
        const sw::space::ManeuverStep step = sw::space::maneuverStep(shift, control, alt);

        bool edited = false;
        auto adjust = [&](sw::KeyCode plus, sw::KeyCode minus, sw::f64& value,
                          sw::f64 amount) {
            if (input().wasKeyPressed(plus)) { value += amount; edited = true; }
            if (input().wasKeyPressed(minus)) { value -= amount; edited = true; }
        };
        adjust(sw::KeyCode::L, sw::KeyCode::J, m_nodeTime, step.seconds);
        adjust(sw::KeyCode::I, sw::KeyCode::K, m_nodePrograde, step.deltaVMps);
        adjust(sw::KeyCode::O, sw::KeyCode::U, m_nodeNormal, step.deltaVMps);
        adjust(sw::KeyCode::Y, sw::KeyCode::H, m_nodeRadial, step.deltaVMps);
        m_nodeTime = std::max(m_nodeTime, m_physicsLane->presentSeconds() + 5.0);
        if (edited)
        {
            m_lastPredictionSeconds = -1.0e9; // instant visual feedback
        }
    }

    // ------------------------------------------------------------------------
    // PICKING A TARGET OFF THE MAP
    //
    // Click a body and it is the target; click it again and it is not. A
    // click on empty space changes nothing — losing a target because you
    // clicked to look at something else would be a small betrayal every
    // time it happened.
    //
    // The hit test is against the body's CENTRE on screen, not its drawn
    // disc: at map zoom most bodies are a few pixels across, and asking the
    // player to hit a four-pixel sphere is asking them to hunt.
    // ------------------------------------------------------------------------
    void StarWorksGame::updateTargetPick()
    {
        if (!m_mapView || m_editorMode || m_nodeDragging ||
            !input().wasMouseButtonPressed(sw::MouseButton::Left))
        {
            return;
        }
        sw::f32 cursorX = 0.0f;
        sw::f32 cursorY = 0.0f;
        if (!hudCursor(cursorX, cursorY))
        {
            return;
        }
        for (const HudButton& button : m_hudButtons)
        {
            if (cursorX >= button.x0 && cursorX <= button.x1 && cursorY >= button.y0 &&
                cursorY <= button.y1)
            {
                return; // a panel has first claim on the click
            }
        }

        const sw::Vec2 cursor{cursorX, cursorY};
        const sw::f64 time = m_physicsLane->presentSeconds();
        const sw::Mat4 viewProjection = m_mapCamera.viewProjectionCameraRelative();
        const sw::WorldVec3 cameraPosition = m_mapCamera.position();
        constexpr sw::f32 kPickRadiusNdc = 0.06f;

        sw::i32 picked = -1;
        sw::f32 pickedDistance = kPickRadiusNdc;
        for (sw::usize i = 0; i < m_celestialIndex.size(); ++i)
        {
            const sw::WorldVec3 world =
                m_celestialIndex.positionAt(static_cast<sw::i32>(i), time);
            const sw::ui::MarkerPlacement placement = sw::ui::placeScreenMarker(
                viewProjection, sw::Vec3(world - cameraPosition), m_mapCamera.right(),
                m_mapCamera.up(), m_mapCamera.forward());
            if (placement.offScreen)
            {
                continue; // clamped to the border: that is a pointer, not a body
            }
            const sw::f32 distance = glm::length(placement.ndc - cursor);
            if (distance < pickedDistance)
            {
                pickedDistance = distance;
                picked = static_cast<sw::i32>(i);
            }
        }
        if (picked < 0)
        {
            return; // empty space: keep whatever was targeted
        }

        m_targetIndex = (picked == m_targetIndex) ? -1 : picked;
        m_approach = {};
        m_nodeApproach = {};
        m_lastPredictionSeconds = -1.0e9; // answer the new question now
        if (m_targetIndex >= 0)
        {
            SW_LOG_INFO("Game", "TARGET: {}",
                        m_celestialIndex.body(static_cast<sw::usize>(m_targetIndex)).name);
        }
        else
        {
            SW_LOG_INFO("Game", "TARGET cleared");
        }
    }

    // ------------------------------------------------------------------------
    // DRAGGING THE NODE ALONG ITS ORBIT
    //
    // The keys move the node in fixed steps; the mouse moves it where you
    // point. Grab the violet marker and the node's TIME follows the pixel
    // under the cursor — the burn slides round the orbit and the planned
    // trajectory redraws under your hand, which is the only way to answer
    // "where on this orbit should I burn?" by looking rather than counting.
    //
    // It runs AFTER the camera, next to the ground cursor, for the reason
    // that rule exists: a pick is a ray from THIS frame's eye, and doing it
    // in the key handler aims it one frame behind the map you can see.
    // ------------------------------------------------------------------------
    void StarWorksGame::updateNodeDrag()
    {
        if (!m_mapView || !m_nodeActive || m_editorMode)
        {
            m_nodeDragging = false;
            return;
        }
        sw::f32 cursorX = 0.0f;
        sw::f32 cursorY = 0.0f;
        if (!hudCursor(cursorX, cursorY))
        {
            m_nodeDragging = false;
            return;
        }
        const sw::Vec2 cursor{cursorX, cursorY};

        if (!input().isMouseButtonDown(sw::MouseButton::Left))
        {
            m_nodeDragging = false;
        }

        const sw::f64 time = m_physicsLane->presentSeconds();
        const sw::Mat4 viewProjection = m_mapCamera.viewProjectionCameraRelative();
        const sw::WorldVec3 cameraPosition = m_mapCamera.position();

        // ---- grabbing it ----------------------------------------------------
        if (!m_nodeDragging && input().wasMouseButtonPressed(sw::MouseButton::Left) &&
            m_nodePrimaryIndex >= 0)
        {
            // Not if a HUD button is under the cursor: the panel gets first
            // claim on a click, or the map's own buttons would be unusable
            // whenever a node happened to be near them.
            bool overButton = false;
            for (const HudButton& button : m_hudButtons)
            {
                overButton = overButton ||
                             (cursorX >= button.x0 && cursorX <= button.x1 &&
                              cursorY >= button.y0 && cursorY <= button.y1);
            }
            const sw::WorldVec3 nodeWorld =
                m_celestialIndex.positionAt(m_nodePrimaryIndex, time) +
                m_nodeRelativePosition;
            const sw::ui::MarkerPlacement placement = sw::ui::placeScreenMarker(
                viewProjection, sw::Vec3(nodeWorld - cameraPosition),
                m_mapCamera.right(), m_mapCamera.up(), m_mapCamera.forward());
            // The marker is about 0.05 NDC across; twice that is a target a
            // hand can hit without hunting for it.
            constexpr sw::f32 kGrabRadiusNdc = 0.05f;
            if (!overButton && !placement.offScreen &&
                glm::length(placement.ndc - cursor) < kGrabRadiusNdc)
            {
                m_nodeDragging = true;
            }
        }

        // ---- and moving it --------------------------------------------------
        if (!m_nodeDragging)
        {
            return;
        }
        sw::f64 pickedTime = 0.0;
        sw::f32 pickedDistance = 0.0f;
        if (!sw::space::timeNearestScreenPoint(m_celestialIndex, m_prediction,
                                               viewProjection, cameraPosition, time,
                                               cursor, kPredictionDisplaySamples,
                                               pickedTime, pickedDistance))
        {
            return;
        }
        // The node is stuck to its LINE, not to the cursor. Drag the mouse
        // off into empty space and the nearest sample is still somewhere on
        // the plan — possibly half an orbit away — so a pick that lands
        // nowhere near the pointer is ignored rather than obeyed.
        constexpr sw::f32 kMaxPickNdc = 0.30f;
        if (pickedDistance > kMaxPickNdc)
        {
            return;
        }
        // The same floor the keys respect: a burn cannot be scheduled in the
        // past, and one five seconds out is already unflyable.
        const sw::f64 clamped = std::max(pickedTime, time + 5.0);
        if (std::abs(clamped - m_nodeTime) > 1.0e-6)
        {
            m_nodeTime = clamped;
            m_lastPredictionSeconds = -1.0e9; // redraw the plan under the hand
        }
    }

    void StarWorksGame::refreshPrediction()
    {
        // The flight plan is a pure function of current state — recomputing
        // a few times per second is plenty (and each run costs ~ms).
        const sw::f64 now = clock().totalSeconds();
        if (now - m_lastPredictionSeconds < kPredictionRefreshSeconds)
        {
            return;
        }
        m_lastPredictionSeconds = now;

        const sw::ecs::Entity entity = controlledEntity();
        const auto& transform = m_world.getComponent<TransformComponent>(entity);

        // Resting on a surface: no meaningful ballistic trajectory — and
        // the conic of a landed craft "impacts" permanently, which is
        // noise, not information.
        if (const auto* body =
                m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(entity);
            body != nullptr && body->isGrounded != 0)
        {
            m_prediction.clear();
            m_nodePrediction.clear();
            m_approach = {};
            m_nodeApproach = {};
            return;
        }

        const sw::f64 startTime = m_physicsLane->presentSeconds();
        // THE PRE-BURN PLAN IS DRAWN WHOLE, node or no node.
        //
        // It used to stop AT the node, on the reasoning that the post-burn
        // path takes over there. But the node is DRAGGED along that line
        // now, and a line that ends at the thing you are dragging gives you
        // nowhere to drag it to. Drawing both — the current orbit entire,
        // the planned one branching off the marker — is also what KSP does,
        // and for the same reason.
        const sw::space::PredictionSettings settings{};
        sw::space::predictTrajectory(m_celestialIndex, transform.position,
                                     controlledVelocity(), startTime, settings,
                                     m_prediction);

        // ---- how close does this plan pass the target? ---------------------
        m_approach = {};
        m_nodeApproach = {};
        if (m_targetIndex >= 0 &&
            static_cast<sw::usize>(m_targetIndex) < m_celestialIndex.size())
        {
            m_approach = sw::space::closestApproachToBody(m_celestialIndex, m_prediction,
                                                          m_targetIndex);
        }

        // ---- the planned burn: dv applied in the orbital frame at the node ----
        m_nodePrediction.clear();
        m_nodePrimaryIndex = -1;
        if (!m_nodeActive)
        {
            return;
        }
        sw::WorldVec3 nodePosition{};
        sw::WorldVec3 nodeVelocity{};
        if (!sw::space::stateOnPrediction(m_celestialIndex, m_prediction, m_nodeTime,
                                          nodePosition, nodeVelocity))
        {
            return;
        }
        const sw::i32 nodePrimary =
            m_celestialIndex.soiPrimaryAt(nodePosition, m_nodeTime);
        if (nodePrimary < 0)
        {
            return;
        }
        sw::WorldVec3 primaryPosition{};
        sw::WorldVec3 primaryVelocity{};
        m_celestialIndex.stateAt(nodePrimary, m_nodeTime, primaryPosition,
                                 &primaryVelocity);
        const sw::WorldVec3 relativePosition = nodePosition - primaryPosition;
        const sw::WorldVec3 relativeVelocity = nodeVelocity - primaryVelocity;
        const sw::f64 relativeSpeed = glm::length(relativeVelocity);
        if (relativeSpeed < 1.0e-6)
        {
            return;
        }

        // KSP orbital frame at the node: prograde along the motion, normal
        // along the orbit's angular momentum, radial completing (outward).
        const sw::WorldVec3 prograde = relativeVelocity / relativeSpeed;
        sw::WorldVec3 normal = glm::cross(relativePosition, relativeVelocity);
        const sw::f64 normalLength = glm::length(normal);
        if (normalLength < 1.0e-9)
        {
            return; // radial trajectory: no orbital frame
        }
        normal /= normalLength;
        const sw::WorldVec3 radialOut = glm::cross(prograde, normal);

        const sw::WorldVec3 dv = prograde * m_nodePrograde + normal * m_nodeNormal +
                                 radialOut * m_nodeRadial;

        // ---- THE LOCK ------------------------------------------------------
        // Close to the node, freeze the plan: the coasting trajectory as it
        // was BEFORE the burn, and the dv vector in world terms. From here
        // the burn is flown against a target that does not move with the
        // ship, which is the only way the remaining dv can reach zero.
        // Editing the node re-takes the lock; drifting out of the window
        // drops it.
        const bool inWindow = (m_nodeTime - startTime) <= kBurnLockSeconds;
        const bool lockMatches = m_burnLocked && m_burnNodeTime == m_nodeTime &&
                                 m_burnPrograde == m_nodePrograde &&
                                 m_burnNormal == m_nodeNormal &&
                                 m_burnRadial == m_nodeRadial;
        if (inWindow && !lockMatches)
        {
            m_burnLocked = true;
            m_burnCoast = m_prediction; // the path NOT taken, from here on
            m_burnDvWorld = dv;
            m_burnNodeTime = m_nodeTime;
            m_burnPrograde = m_nodePrograde;
            m_burnNormal = m_nodeNormal;
            m_burnRadial = m_nodeRadial;
        }
        else if (!inWindow)
        {
            m_burnLocked = false;
        }

        // What is LEFT of the burn is what the planned trajectory should be
        // drawn from: as the player flies it, the plan converges onto the
        // orbit they are actually achieving instead of drifting away from it.
        const sw::WorldVec3 remaining =
            m_burnLocked ? remainingBurnVector() : dv;
        m_nodePostBurnVelocity = nodeVelocity + remaining;
        m_nodePrimaryIndex = nodePrimary;
        m_nodeRelativePosition = relativePosition;

        sw::space::PredictionSettings nodeSettings{}; // full horizon again
        sw::space::predictTrajectory(m_celestialIndex, nodePosition,
                                     m_nodePostBurnVelocity, m_nodeTime, nodeSettings,
                                     m_nodePrediction);
        if (m_targetIndex >= 0)
        {
            // What the PLANNED burn would achieve, which is the number the
            // player is actually dialling the node for.
            m_nodeApproach = sw::space::closestApproachToBody(
                m_celestialIndex, m_nodePrediction, m_targetIndex);
        }
    }

    // THE BURN STILL TO FLY.
    //
    // Locked: the planned dv minus what has actually been applied, where
    // "applied" is measured against the COASTING velocity from the frozen
    // pre-burn plan. That subtraction is what makes the number honest —
    // gravity changes the ship's velocity by a kilometre per second over a
    // two-minute burn in low orbit, and counting that as thrust would have
    // the readout hit zero while the engine still had work to do.
    //
    // Unlocked (the node is minutes away): there is nothing to count down
    // yet, so it is simply the planned burn.
    sw::WorldVec3 StarWorksGame::remainingBurnVector()
    {
        if (!m_nodeActive)
        {
            return sw::WorldVec3{0.0};
        }
        if (m_burnLocked)
        {
            return sw::space::remainingBurn(m_celestialIndex, m_burnCoast, m_burnDvWorld,
                                            controlledVelocity(),
                                            m_physicsLane->presentSeconds());
        }
        return m_nodePostBurnVelocity - controlledVelocity();
    }

    void StarWorksGame::toggleEva()
    {
        // ON FOOT IS HOME. The suit is created with the world and never
        // destroyed; this only decides which of the two things the player's
        // hands are on. Boarding therefore needs a vessel to board, and
        // there may not be one — the world starts with none.
        if (m_capsuleEntity.isNull())
        {
            return;
        }
        if (m_evaMode)
        {
            if (m_shipEntity.isNull() || !m_world.isAlive(m_shipEntity))
            {
                SW_LOG_INFO("Game", "No vessel to board — order one at the VAB");
                return;
            }
            m_evaMode = false;
            SW_LOG_INFO("Game", "Aboard: controlling vessel {}", m_shipEntity.index);
            return;
        }

        // Stepping out: put the suit beside the vessel, CO-MOVING with it.
        // Anything else and the ship leaves at orbital speed the instant the
        // player's feet touch vacuum.
        if (!m_shipEntity.isNull() && m_world.isAlive(m_shipEntity))
        {
            const auto& shipTransform =
                m_world.getComponent<TransformComponent>(m_shipEntity);
            const sw::WorldVec3 shipVelocity = controlledVelocity();
            auto& transform = m_world.getComponent<TransformComponent>(m_capsuleEntity);
            transform.position =
                shipTransform.position +
                sw::WorldVec3(shipTransform.rotation * sw::Vec3{12.0f, 0.0f, 0.0f});
            transform.rotation = shipTransform.rotation;
            if (auto* previous =
                    m_world.tryGetComponent<PreviousTransformComponent>(m_capsuleEntity))
            {
                previous->position = transform.position;
                previous->rotation = transform.rotation;
            }
            if (auto* body =
                    m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(m_capsuleEntity))
            {
                body->velocity = shipVelocity;
                body->angularVelocity = sw::Vec3{0.0f};
            }
        }
        m_evaMode = true;
        SW_LOG_INFO("Game", "EVA: on foot");
    }

    // STAGING: fire the ship's next decoupler, nose-most last.
    void StarWorksGame::fireNextDecoupler()
    {
        sw::ecs::Entity decoupler{};
        m_world.forEach<sw::parts::PartComponent>(
            [&](sw::ecs::Entity entity, sw::parts::PartComponent& part) {
                if (part.vessel != m_shipEntity || !decoupler.isNull())
                {
                    return;
                }
                const auto* definition = sw::parts::findDefinition(part.definitionId);
                if (definition != nullptr &&
                    definition->type == sw::parts::PartType::Decoupler)
                {
                    decoupler = entity;
                }
            });
        if (decoupler.isNull())
        {
            return;
        }
        const sw::ecs::Entity separated = sw::parts::decoupleAt(m_world, decoupler);
        if (!separated.isNull())
        {
            // splitVessel already gave it Transform/Previous/body.
            m_world.addComponent(separated, BoundsComponent{0.1f});
            m_world.addComponent(separated,
                                 MapMarkerComponent{{0.6f, 0.6f, 0.6f, 1.0f}});
            SW_LOG_INFO("Game", "STAGING: decoupler fired, stage separated");
        }
    }

    void StarWorksGame::updateWarp()
    {
        // X0 IS A WARP RATE. Stopping time is the bottom of the same ladder
        // every other rate lives on, so it is reached the same way: step
        // down from x1 and time stops; step up and it starts again, at x1.
        // Nothing else on the keyboard can pause the game by accident.
        // ---- WARPING TO THE NODE -------------------------------------------
        // The rung is chosen so one real second never advances more than
        // half the time that is left: the approach slows down of its own
        // accord and the last rung is x1, which is what stops the overshoot
        // a fixed ladder plus a human reaction time cannot avoid.
        // ---- SYNC WARP: catching another player's temporality -------------
        // Same servo as the node, one difference that matters: it is allowed
        // past the altitude ladder. Closing a three-hour gap from a 200 km
        // orbit at x100 would take a real hour, and nobody would use it.
        bool bypassAltitudeCap = false;
        if (m_syncWarpTo > 0.0)
        {
            const sw::f64 remaining = m_syncWarpTo - m_physicsLane->presentSeconds();
            if (remaining <= 0.0 || !netActive() || !warpAllowed())
            {
                if (remaining > 0.0)
                {
                    SW_LOG_INFO("Game", "Sync warp stopped: {}",
                                netActive() ? warpBlockReason() : "session ended");
                }
                else
                {
                    SW_LOG_INFO("Game", "Sync warp: caught up");
                }
                m_syncWarpTo = 0.0;
                m_syncWarpPlayer = 0;
                m_warpIndex = 0;
            }
            else
            {
                sw::u32 want = 0;
                for (sw::u32 i = 0; i < kWarpSteps; ++i)
                {
                    if (static_cast<sw::f64>(kWarpLadder[i]) <= remaining * 0.5)
                    {
                        want = i;
                    }
                }
                m_warpIndex = want;
                bypassAltitudeCap = true;
            }
        }
        if (m_syncWarpTo > 0.0 && (input().wasKeyPressed(sw::KeyCode::Period) ||
                                   input().wasKeyPressed(sw::KeyCode::Comma)))
        {
            m_syncWarpTo = 0.0;
            m_syncWarpPlayer = 0;
            SW_LOG_INFO("Game", "Sync warp cancelled");
        }

        if (m_warpToSeconds > 0.0)
        {
            if (!m_nodeActive)
            {
                m_warpToSeconds = 0.0; // the node it was aiming at is gone
                m_warpIndex = 0;
            }
            else
            {
                const sw::f64 remaining =
                    m_warpToSeconds - m_physicsLane->presentSeconds();
                if (remaining <= 0.0)
                {
                    m_warpToSeconds = 0.0;
                    m_warpIndex = 0;
                    SW_LOG_INFO("Game", "Warp to node: arrived");
                }
                else
                {
                    sw::u32 want = 0;
                    for (sw::u32 i = 0; i < kWarpSteps; ++i)
                    {
                        if (static_cast<sw::f64>(kWarpLadder[i]) <= remaining * 0.5)
                        {
                            want = i;
                        }
                    }
                    m_warpIndex = want;
                }
            }
        }
        // Any manual warp key takes the controls back.
        if (m_warpToSeconds > 0.0 && (input().wasKeyPressed(sw::KeyCode::Period) ||
                                      input().wasKeyPressed(sw::KeyCode::Comma)))
        {
            m_warpToSeconds = 0.0;
            SW_LOG_INFO("Game", "Warp to node cancelled");
        }

        if (keyPressed(sw::KeyCode::Period))
        {
            if (m_simulation.isPaused())
            {
                m_simulation.setPaused(false); // x0 -> x1
                SW_LOG_INFO("Game", "Simulation resumed");
            }
            else if (m_warpIndex + 1 < kWarpSteps)
            {
                ++m_warpIndex;
            }
        }
        if (keyPressed(sw::KeyCode::Comma) && !m_simulation.isPaused())
        {
            if (m_warpIndex > 0)
            {
                --m_warpIndex;
            }
            else
            {
                m_simulation.setPaused(true); // x1 -> x0
                SW_LOG_INFO("Game", "Simulation paused (warp x0)");
            }
        }

        const sw::WorldVec3 focusPosition =
            m_world.getComponent<TransformComponent>(controlledEntity()).position;
        sw::f64 minAltitude = 1.0e18;
        m_world.forEach<TransformComponent, sw::phys::GravitySourceComponent>(
            [&](sw::ecs::Entity, TransformComponent& transform,
                sw::phys::GravitySourceComponent& source) {
                const sw::f64 altitude =
                    glm::length(focusPosition - transform.position) - source.bodyRadius;
                minAltitude = std::min(minAltitude, altitude);
            });

        if (!bypassAltitudeCap)
        {
            const sw::f32 maxAllowed = maxWarpForAltitude(minAltitude);
            while (m_warpIndex > 0 && kWarpLadder[m_warpIndex] > maxAllowed)
            {
                --m_warpIndex;
                SW_LOG_INFO("Game", "Warp limited to x{:g} (altitude {:.0f} km)",
                            kWarpLadder[m_warpIndex], minAltitude / 1000.0);
            }

            // AND THE REAL GATE. Past physics warp the integrator is switched
            // off and the world goes analytic, which is only an approximation
            // of the truth when the truth is already analytic: a closed orbit
            // clear of the air, or a craft standing still on the ground.
            // A suborbital arc, a reentry or an escape trajectory is not, and
            // warping one used to hand back a vehicle somewhere it could
            // never have reached. The altitude ladder above never caught
            // that: it happily allowed x1000 on a trajectory into the dirt.
            if (kWarpLadder[m_warpIndex] > kMaxPhysicsWarp && !warpAllowed())
            {
                while (m_warpIndex > 0 && kWarpLadder[m_warpIndex] > kMaxPhysicsWarp)
                {
                    --m_warpIndex;
                }
                m_warpToSeconds = 0.0;
            }
        }

        m_simulation.setTimeScale(kWarpLadder[m_warpIndex]);
        // Physics warp (<= x5): everything stays truly simulated — the
        // Physics lane is STRICT so a slow machine slows the sim instead
        // of desynchronizing it (the pre-M21 launch-pad fling). Rails
        // warp (> x5): analytic orbits only, exact at any speed; drops
        // move the whole world coherently, so strictness is lifted.
        const bool physicsWarp = kWarpLadder[m_warpIndex] <= kMaxPhysicsWarp;
        m_physicsLane->setStrictCatchUp(physicsWarp);
        m_bubbleSystem->setForceRails(!physicsWarp);
    }

    void StarWorksGame::updateShipControls()
    {
        // THE JUMP IS A LATCH, NOT AN EDGE, and this is why.
        //
        // `ShipControlsComponent` is cleared and rewritten here, once per
        // RENDERED FRAME. It is consumed by CapsuleMovementSystem, which
        // runs on the physics lane at a FIXED fifty hertz. Above sixty frames
        // a second most frames tick that lane zero times — so a jump written
        // as a one-frame edge was cleared again before any tick could see it,
        // and roughly one press in three did nothing at all.
        //
        // So the request stays set until a physics tick has actually run
        // (cleared in onUpdate, after `advance`). Pressing twice inside one
        // tick still jumps once: the system sets isGrounded to 0 as it goes,
        // and a second tick finds no ground to push off.
        const bool jumpRequested = m_jumpRequested;

        // Idle both control blocks, then feed the controlled one. There may
        // be NO SHIP: the world starts without one and stays that way until
        // a VAB builds a design and a pad rolls it out.
        ShipControlsComponent* shipControls =
            m_shipEntity.isNull()
                ? nullptr
                : m_world.tryGetComponent<ShipControlsComponent>(m_shipEntity);
        if (shipControls != nullptr)
        {
            *shipControls = {};
        }
        ShipControlsComponent* capsuleControls = nullptr;
        if (!m_capsuleEntity.isNull())
        {
            capsuleControls =
                m_world.tryGetComponent<ShipControlsComponent>(m_capsuleEntity);
            if (capsuleControls != nullptr)
            {
                *capsuleControls = {};
            }
        }

        if (!m_shipMode || m_mapView || m_editorMode ||
            kWarpLadder[m_warpIndex] > kMaxPhysicsWarp)
        {
            return; // engines only work while the world is truly simulated
        }

        ShipControlsComponent* controlsPtr =
            (m_evaMode && capsuleControls != nullptr) ? capsuleControls : shipControls;
        if (controlsPtr == nullptr)
        {
            return; // nothing under this player's hands
        }
        ShipControlsComponent& controls = *controlsPtr;

        const bool walking = m_evaMode && capsuleControls != nullptr;

        if (input().isKeyDown(sw::KeyCode::W)) { controls.thrustAxis += 1.0f; }
        if (input().isKeyDown(sw::KeyCode::S)) { controls.thrustAxis -= 1.0f; }
        if (input().isKeyDown(sw::KeyCode::Up)) { controls.rotationInput.x -= 1.0f; }
        if (input().isKeyDown(sw::KeyCode::Down)) { controls.rotationInput.x += 1.0f; }
        if (walking)
        {
            // ON FOOT the left/right keys SIDESTEP. Turning belongs to the
            // mouse, because the suit faces wherever the camera looks — a
            // walker that steers with keys and looks with the mouse makes
            // you fight two controls to go one direction.
            if (input().isKeyDown(sw::KeyCode::A)) { controls.strafeAxis -= 1.0f; }
            if (input().isKeyDown(sw::KeyCode::D)) { controls.strafeAxis += 1.0f; }
        }
        else
        {
            if (input().isKeyDown(sw::KeyCode::A)) { controls.rotationInput.y += 1.0f; }
            if (input().isKeyDown(sw::KeyCode::D)) { controls.rotationInput.y -= 1.0f; }
        }
        if (input().isKeyDown(sw::KeyCode::Q)) { controls.rotationInput.z += 1.0f; }
        if (input().isKeyDown(sw::KeyCode::E)) { controls.rotationInput.z -= 1.0f; }
        controls.killRotation = input().isKeyDown(sw::KeyCode::X) ? 1u : 0u;
        // JUMP, on foot only. The walker system consumes it on the first
        // tick that sees it and leaves the ground, so a held key does not
        // hover and a slow frame running four physics ticks does not jump
        // four times.
        if (walking && jumpRequested)
        {
            controls.jump = 1u;
        }
        // Throttle limiter (ship only; the capsule ignores it).
        if (input().isKeyDown(sw::KeyCode::LeftShift)) { controls.throttleDelta += 1.0f; }
        if (input().isKeyDown(sw::KeyCode::LeftControl))
        {
            controls.throttleDelta -= 1.0f;
        }
    }

    void StarWorksGame::updateChaseCamera(sw::f32 deltaSeconds)
    {
        const sw::ecs::Entity target = controlledEntity();
        const auto& transform = m_world.getComponent<TransformComponent>(target);
        const auto& previous = m_world.getComponent<PreviousTransformComponent>(target);

        const sw::f32 alpha = m_physicsLane->alpha();
        const sw::WorldVec3 position =
            glm::mix(previous.position, transform.position, static_cast<sw::f64>(alpha));
        const sw::Quat rotation = glm::slerp(previous.rotation, transform.rotation, alpha);

        // ---- THE CAMERA FRAME ------------------------------------------------
        // The craft's own rotation does NOT appear anywhere below, and that
        // is the whole design. A chase camera that inherits the vehicle's
        // attitude turns every roll, every RCS twitch and every SAS
        // correction into a camera move: the world swings around you while
        // you are trying to read it, and you cannot look at anything for
        // longer than the autopilot leaves the nose still. So the view has
        // its own orientation, and only the mouse changes it.
        //
        // ONE automatic behaviour survives, because it is the one that is
        // about the WORLD rather than about the vehicle: close to a body,
        // "up" means up. The camera levels on the local horizon — and it
        // stops there. It does not also follow where the rocket is pointing;
        // it just stops being upside down.
        sw::Vec3 radialUp{0.0f, 1.0f, 0.0f};
        sw::Quat horizonFrame{1.0f, 0.0f, 0.0f, 0.0f};
        bool wantHorizonLock = false;
        const sw::i32 primaryIndex = controlledPrimaryIndex();
        if (primaryIndex >= 0)
        {
            const auto& primary =
                m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
            const sw::WorldVec3 primaryPosition = m_celestialIndex.positionAt(
                primaryIndex, m_physicsLane->presentSeconds());
            const sw::WorldVec3 radial = position - primaryPosition;
            const sw::f64 distance = glm::length(radial);
            if (distance > 1.0)
            {
                radialUp = sw::Vec3(radial / distance);
                // Low relative to the BODY'S OWN SIZE — 3% of its radius, so
                // it means the same thing on a moon as on a planet.
                wantHorizonLock =
                    (distance - primary.bodyRadius) < 0.03 * primary.bodyRadius;

                // The frame's heading has to come from something that does
                // not move with the craft, or "level" would still track the
                // nose. NORTH does: the body's own spin axis, projected onto
                // the local horizontal.
                sw::Vec3 axis{0.0f, 1.0f, 0.0f};
                if (const auto* source =
                        m_world.tryGetComponent<sw::phys::GravitySourceComponent>(
                            primary.entity);
                    source != nullptr && glm::length(source->spinAxis) > 1.0e-12)
                {
                    axis = sw::Vec3(glm::normalize(source->spinAxis));
                }
                sw::Vec3 north = axis - radialUp * glm::dot(axis, radialUp);
                if (glm::length(north) < 1.0e-3f)
                {
                    // Straight over a pole: any horizontal direction will do,
                    // as long as it is a CONTINUOUS choice.
                    const sw::Vec3 reference =
                        (std::abs(radialUp.x) < 0.9f) ? sw::Vec3{1.0f, 0.0f, 0.0f}
                                                      : sw::Vec3{0.0f, 0.0f, 1.0f};
                    north = reference - radialUp * glm::dot(reference, radialUp);
                }
                north = glm::normalize(north);
                const sw::Vec3 east = glm::normalize(glm::cross(north, radialUp));
                horizonFrame = glm::quat_cast(sw::Mat3{east, radialUp, -north});
            }
        }
        const sw::f32 blendTarget = wantHorizonLock ? 1.0f : 0.0f;
        m_groundCamBlend +=
            (blendTarget - m_groundCamBlend) * std::min(1.0f, deltaSeconds * 2.0f);
        const sw::f32 blend = m_groundCamBlend;

        // Away from a body the reference is INERTIAL — the world axes. It
        // does not drift, it does not spin, and a camera parked in it stays
        // parked. Near one it becomes the horizon frame, eased across so
        // crossing the threshold is not a snap.
        const sw::Quat offsetRotation =
            glm::slerp(sw::Quat{1.0f, 0.0f, 0.0f, 0.0f}, horizonFrame, blend);

        // ---- user camera control: right-drag orbits, wheel zooms, C resets ----
        // ON FOOT the horizontal drag does not orbit the camera around the
        // player — it TURNS THE PLAYER, and the camera follows from behind.
        // That is what "you always walk where you are looking" means: there
        // is only one heading in the world, and the mouse owns it.
        CapsuleComponent* walker =
            m_evaMode ? m_world.tryGetComponent<CapsuleComponent>(target) : nullptr;
        if (input().isMouseButtonDown(sw::MouseButton::Right))
        {
            if (walker != nullptr)
            {
                walker->headingRadians += input().mouseDeltaX() * 0.0045f;
                m_chaseYaw = 0.0f; // the body turned instead
            }
            else
            {
                m_chaseYaw -= input().mouseDeltaX() * 0.0045f;
            }
            m_chasePitch =
                std::clamp(m_chasePitch - input().mouseDeltaY() * 0.0045f, -1.35f, 1.35f);
        }
        else if (walker != nullptr && m_chaseYaw != 0.0f)
        {
            // Coming back from the cockpit with a yawed camera: hand the
            // offset over to the body once, so the view does not sit at an
            // angle to the direction W walks in.
            walker->headingRadians -= m_chaseYaw;
            m_chaseYaw = 0.0f;
        }
        if (const sw::f32 scroll = input().scrollDeltaY(); scroll != 0.0f)
        {
            m_chaseZoom = std::clamp(
                m_chaseZoom * std::pow(1.18f, -scroll), 0.35f, 12.0f);
        }
        if (input().wasKeyPressed(sw::KeyCode::C))
        {
            m_chaseYaw = 0.0f;
            m_chasePitch = 0.0f;
            m_chaseZoom = 1.0f;
        }
        const sw::Quat userOrbit =
            glm::angleAxis(m_chaseYaw, sw::Vec3{0.0f, 1.0f, 0.0f}) *
            glm::angleAxis(m_chasePitch, sw::Vec3{1.0f, 0.0f, 0.0f});

        // ---- ON FOOT: FIRST PERSON -------------------------------------
        // A factory is built at arm's length. Watching your own back while
        // you place a machine puts the thing you are aiming at behind your
        // own shoulders, so EVA looks out of the suit's visor instead: the
        // camera sits at the head, the body's heading IS the view direction
        // (the mouse already turns it), and the pitch is a free look.
        if (walker != nullptr)
        {
            // The suit is a 2 m body centred on its transform; eyes just
            // under the top of it.
            constexpr sw::f32 kEyeHeight = 0.72f;
            const sw::WorldVec3 eye = position + sw::WorldVec3(radialUp * kEyeHeight);
            m_camera.setPosition(eye);

            // Heading comes from the body (rotation's -Z), pitch from the
            // mouse. Both are applied about the LOCAL vertical / local
            // right, so the horizon stays level on a round world.
            const sw::Vec3 bodyForward =
                glm::normalize(rotation * sw::math::kWorldForward);
            sw::Vec3 flat = bodyForward - radialUp * glm::dot(bodyForward, radialUp);
            if (glm::length(flat) < 1.0e-4f)
            {
                flat = rotation * sw::Vec3{1.0f, 0.0f, 0.0f};
            }
            flat = glm::normalize(flat);
            const sw::Vec3 lookRight = glm::normalize(glm::cross(flat, radialUp));
            const sw::Vec3 forward =
                glm::normalize(glm::angleAxis(m_chasePitch, lookRight) * flat);
            const sw::Vec3 up = glm::cross(lookRight, forward);
            m_camera.setOrientation(glm::quat_cast(sw::Mat3{lookRight, up, -forward}));
            return;
        }

        const sw::Vec3 chaseOffset = sw::Vec3{0.0f, 12.0f, 42.0f} * m_chaseZoom;
        const sw::WorldVec3 cameraPosition =
            position + sw::WorldVec3(offsetRotation * (userOrbit * chaseOffset));
        m_camera.setPosition(cameraPosition);

        const sw::Vec3 forward = glm::normalize(sw::Vec3(position - cameraPosition));
        // Camera up comes from the REFERENCE frame, never from the craft.
        sw::Vec3 targetUp = offsetRotation * sw::Vec3{0.0f, 1.0f, 0.0f};
        if (std::abs(glm::dot(forward, targetUp)) > 0.999f)
        {
            targetUp = offsetRotation * sw::Vec3{0.0f, 0.0f, 1.0f}; // straight down
        }
        const sw::Vec3 right = glm::normalize(glm::cross(forward, targetUp));
        const sw::Vec3 up = glm::cross(right, forward);
        m_camera.setOrientation(glm::quat_cast(sw::Mat3{right, up, -forward}));
    }

    StarWorksGame::~StarWorksGame()
    {
        // See the header: the terrain-patch job writes into members of THIS
        // object, and the pool that runs it belongs to the base class.
        threadPool().waitIdle();
    }

    void StarWorksGame::onUpdate(sw::f32 deltaSeconds)
    {
        // The address field takes the keyboard while it has focus — including
        // ESC, which cancels it rather than quitting the game. Everything
        // below asks through keyPressed(), which is false while typing.
        updateTextField();

        // ESC quits the game — except in the hangar, where it belongs to
        // the editor (drop / put back the held part).
        if (keyPressed(sw::KeyCode::Escape) && !m_editorMode)
        {
            window().requestClose();
            return;
        }

        // --- mode toggles -------------------------------------------------------
        if (keyPressed(sw::KeyCode::M))
        {
            m_mapView = !m_mapView;
            SW_LOG_INFO("Game", "Star map {}", m_mapView ? "opened" : "closed");
        }
        if (keyPressed(sw::KeyCode::G) && !m_mapView)
        {
            toggleEva();
        }
        // F3, not a letter: N already creates a maneuver node on the map,
        // and every other letter in reach is a flight control. F2 shows the
        // hulls, F5/F9 save and load — the panel keys are a family.
        if (keyPressed(sw::KeyCode::F3))
        {
            m_netPanel = !m_netPanel;
            if (!m_netPanel)
            {
                m_netAddressFocused = false;
            }
        }
        if (keyPressed(sw::KeyCode::V))
        {
            m_speedSurfaceRelative = !m_speedSurfaceRelative;
        }
        if (keyPressed(sw::KeyCode::Tab))
        {
            m_shipMode = !m_shipMode;
            if (!m_shipMode)
            {
                const sw::Vec3 forward = m_camera.forward();
                const sw::f32 yaw = std::atan2(-forward.x, -forward.z);
                const sw::f32 pitch = std::asin(std::clamp(forward.y, -1.0f, 1.0f));
                m_cameraController.setPose(m_camera.position(), yaw, pitch);
            }
            SW_LOG_INFO("Game", "{}", m_shipMode ? "Chase camera" : "Free camera");
        }
        // SPACE is the ACTION key, not the pause key.
        //
        // Pausing is a warp rate — x0 — and it belongs on the warp control
        // with every other rate: press the warp-down key at x1 and time
        // stops. Space was doing the one thing a pilot's thumb should never
        // do by accident, on the key every game in the genre uses to stage.
        // On foot it jumps; in a rocket it fires the next decoupler.
        if (keyPressed(sw::KeyCode::Space) && !m_editorMode && !m_mapView)
        {
            if (m_evaMode)
            {
                m_jumpRequested = true; // consumed by updateShipControls
            }
            else
            {
                fireNextDecoupler();
            }
        }
        if (m_mapView)
        {
            updateManeuverNodeInput();
        }

        // ---- vessel editor (B) + staging (Z) -----------------------------------
        // THE HANGAR OPENS ON FOOT. It used to require a cockpit, which
        // made sense while the player began in one; now that on foot is the
        // normal state, refusing there would put the design tool behind a
        // vessel you can only obtain by using the design tool.
        if (keyPressed(sw::KeyCode::B) && !m_mapView)
        {
            if (m_editorMode) { exitEditor(); }
            else { enterEditor(); }
        }
        if (m_editorMode)
        {
            updateEditor();
        }
        // Z stays as the second name for the same action — the muscle
        // memory of everyone who has flown this build so far.
        // Staging belongs to whoever is IN the rocket. On foot it would fire
        // a decoupler on a vessel the player is not aboard and may not even
        // be able to see.
        if (keyPressed(sw::KeyCode::Z) && !m_editorMode && !m_mapView && !m_evaMode)
        {
            fireNextDecoupler();
        }

        // ---- docking: ports of different vessels, close and slow ---------------
        if (!m_editorMode && clock().totalSeconds() - m_lastDockCheckSeconds > 0.5)
        {
            m_lastDockCheckSeconds = clock().totalSeconds();
            struct Port { sw::ecs::Entity part; sw::ecs::Entity vessel; sw::WorldVec3 pos; };
            std::vector<Port> ports;
            m_world.forEach<sw::parts::PartComponent, TransformComponent>(
                [&](sw::ecs::Entity entity, sw::parts::PartComponent& part,
                    TransformComponent& transform) {
                    const auto* definition =
                        sw::parts::findDefinition(part.definitionId);
                    if (definition != nullptr &&
                        definition->type == sw::parts::PartType::DockingPort)
                    {
                        ports.push_back({entity, part.vessel, transform.position});
                    }
                });
            for (sw::usize a = 0; a < ports.size(); ++a)
            {
                for (sw::usize b = a + 1; b < ports.size(); ++b)
                {
                    if (ports[a].vessel == ports[b].vessel ||
                        glm::length(ports[a].pos - ports[b].pos) > 4.0)
                    {
                        continue;
                    }
                    // The player's vessel absorbs the other one.
                    const bool shipIsA = ports[a].vessel == m_shipEntity;
                    if (sw::parts::dockVessels(m_world,
                                               shipIsA ? ports[a].part : ports[b].part,
                                               shipIsA ? ports[b].part : ports[a].part))
                    {
                        SW_LOG_INFO("Game", "DOCKING: vessels merged");
                    }
                }
            }
        }

        // ---- SAS: T cycles OFF -> SAS -> PGD -> RTG -> NODE; buttons too ------
        // NODE is skipped when there is no node: cycling onto a mode that
        // cannot point anywhere would look like the key had stopped working.
        if (keyPressed(sw::KeyCode::T))
        {
            const sw::u32 ring[5] = {SasComponent::kOff, SasComponent::kStability,
                                     SasComponent::kPrograde, SasComponent::kRetrograde,
                                     SasComponent::kNode};
            const sw::u32 count = m_nodeActive ? 5u : 4u;
            sw::u32 index = 0;
            for (sw::u32 i = 0; i < count; ++i)
            {
                if (ring[i] == m_sasMode)
                {
                    index = i;
                    break;
                }
            }
            m_sasMode = ring[(index + 1) % count];
        }
        if (keyPressed(sw::KeyCode::P) && !m_editorMode)
        {
            cyclePilotedVessel(); // fly any built vessel
        }
        // F opens the BUILDING catalogue. Not in the hangar, which is the
        // rocket editor and has its own palette.
        if (keyPressed(sw::KeyCode::F) && !m_editorMode)
        {
            m_buildMenu = !m_buildMenu;
            m_configTarget = {}; // one panel at a time
        }
        // F2 shows the collision hulls. Not a debug flag hidden behind a
        // rebuild: the hitboxes are authored by hand now, and an authoring
        // mistake and an engine mistake look identical until you can see
        // the boxes.
        if (keyPressed(sw::KeyCode::F2))
        {
            m_showHitboxes = !m_showHitboxes;
            SW_LOG_INFO("Game", "HITBOXES {}", m_showHitboxes ? "SHOWN" : "HIDDEN");
        }
        // E opens the MACHINE panel of whatever you are standing next to.
        // On foot only: E is also the ship's roll axis, and a pilot pressing
        // it means roll. (`m_evaMode` is checked inside.)
        if (keyPressed(sw::KeyCode::E) && m_evaMode && !m_editorMode)
        {
            // Closing needs nothing; OPENING casts a ray from the camera, so
            // it has to wait for this frame's camera — the same reason the
            // ground cursor aims last. Answering here would aim at where you
            // were looking one frame ago.
            if (!m_configTarget.isNull())
            {
                m_configTarget = {};
            }
            else
            {
                m_configRequested = true;
            }
        }
        handleHudClicks();
        if (auto* sas = m_world.tryGetComponent<SasComponent>(m_shipEntity))
        {
            if (m_sasMode == SasComponent::kNode && !m_nodeActive)
            {
                m_sasMode = SasComponent::kOff; // the node it held is gone
            }
            sas->mode = m_sasMode;
            // ONE TOGGLE, THREE INSTRUMENTS. The speed readout, the navball's
            // prograde marker and the autopilot all read the same flag, so
            // pressing V cannot leave the autopilot flying toward a direction
            // the marker is no longer drawing.
            sas->surfaceRelative = m_speedSurfaceRelative ? 1u : 0u;
            // The burn still to fly, handed down every frame. It is a WORLD
            // vector and it shrinks as the burn is flown, so the autopilot
            // keeps the nose on the part of the burn that is left rather
            // than on the direction the whole burn started in.
            sas->targetDirection = sw::Vec3(remainingBurnVector());
        }

        // --- save / load ----------------------------------------------------------
        if (input().wasKeyPressed(sw::KeyCode::F5))
        {
            try
            {
                saveGame();
            }
            catch (const sw::Exception& e)
            {
                SW_LOG_ERROR("Game", "Save failed: {}", e.message());
            }
        }
        if (input().wasKeyPressed(sw::KeyCode::F9))
        {
            try
            {
                loadGame();
            }
            catch (const sw::Exception& e)
            {
                SW_LOG_ERROR("Game", "Load failed: {}", e.message());
            }
        }

        // The gate the warp control reads. Computed here, once, so the HUD
        // and the control cannot disagree about whether this craft is in a
        // stable orbit — they run at opposite ends of the frame.
        refreshFlightState();
        updateWarp();

        // --- per-mode camera & controls ------------------------------------------
        if (m_mapView)
        {
            const sw::f32 scroll = input().scrollDeltaY();
            if (scroll != 0.0f)
            {
                m_mapHeightMeters = std::clamp(
                    m_mapHeightMeters * std::pow(1.3, static_cast<sw::f64>(-scroll)),
                    kMapMinHeight, kMapMaxHeight);
            }
            // Right-drag orbits the map camera around the focus (yaw +
            // tilt); default stays near top-down.
            if (input().isMouseButtonDown(sw::MouseButton::Right))
            {
                m_mapYaw -= input().mouseDeltaX() * 0.005f;
                m_mapPitch = std::clamp(m_mapPitch - input().mouseDeltaY() * 0.005f,
                                        0.12f, 1.53f);
            }

            // KSP-style framing: the map centers on the SOI primary of the
            // controlled craft — Terra view at LEO, Sol view once you leave
            // Terra's sphere of influence.
            sw::WorldVec3 mapCenter{0.0};
            if (const sw::i32 primaryIndex = controlledPrimaryIndex(); primaryIndex >= 0)
            {
                mapCenter = m_celestialIndex.positionAt(primaryIndex,
                                                        m_physicsLane->presentSeconds());
            }
            const sw::f64 cosPitch = std::cos(m_mapPitch);
            const sw::WorldVec3 offsetDir{cosPitch * std::sin(m_mapYaw),
                                          std::sin(m_mapPitch),
                                          cosPitch * std::cos(m_mapYaw)};
            const sw::WorldVec3 cameraPos = mapCenter + offsetDir * m_mapHeightMeters;
            m_mapCamera.setPosition(cameraPos);
            const sw::Vec3 forward = glm::normalize(sw::Vec3(mapCenter - cameraPos));
            const sw::Vec3 right =
                glm::normalize(glm::cross(forward, sw::Vec3{0, 1, 0}));
            const sw::Vec3 up = glm::cross(right, forward);
            m_mapCamera.setOrientation(glm::quat_cast(sw::Mat3{right, up, -forward}));
            m_mapCamera.setAspectRatio(renderer().aspectRatio());
        }
        else if (!m_shipMode)
        {
            m_cameraController.update(input(), window(), deltaSeconds);
        }
        m_camera.setAspectRatio(renderer().aspectRatio());

        updateShipControls();

        m_bubbleSystem->setFocus(
            m_world.getComponent<TransformComponent>(controlledEntity()).position);

        // The wind is a closed form in simulation time — handing the system
        // the clock rather than letting it read one is what keeps two runs
        // of the same launch identical, at any time scale.
        if (m_aerodynamics != nullptr)
        {
            m_aerodynamics->setTimeSeconds(m_physicsLane->presentSeconds());
        }

        const sw::u64 physicsTicksBefore = m_physicsLane->tickCount();
        m_simulation.advance(m_world, deltaSeconds, &threadPool());
        m_commands.playback(m_world);
        if (m_physicsLane->tickCount() != physicsTicksBefore)
        {
            // A tick ran, so whatever was latched has been acted on.
            m_jumpRequested = false;
        }
        // After the clock has moved: the session reports THIS player's new
        // instant, and whatever the timeline says is now due is released.
        updateNetwork(deltaSeconds);

        // Fresh hierarchy snapshot for the map, HUD and flight plan.
        m_celestialIndex.rebuild(m_world);
        refreshPrediction();
        updateTerrainPatch();
        updateGrassField();

        updateReentryEffects(deltaSeconds);
        // A crate of rocket that has arrived becomes a rocket. Once a frame
        // is plenty: the crate is not going anywhere, and unpacking it
        // creates entities, which no simulation lane is allowed to do.
        updateLaunchPads();

        if (m_shipMode && !m_mapView)
        {
            updateChaseCamera(deltaSeconds);
        }

        // THE GROUND CURSOR AIMS LAST, on purpose. It casts a ray from the
        // camera at the ground, so it needs THIS frame's camera and THIS
        // frame's interpolated planet — not last frame's, and not the raw
        // tick pose. Running it up with the key handling put a whole frame
        // between where you were looking and where the ghost landed.
        updateBuildCursor();
        // ...and the maneuver node's mouse grab, which is a pick against the
        // map exactly as it is being drawn this frame — and the target
        // pick, which is the same kind of question asked of the bodies.
        updateNodeDrag();
        updateTargetPick();
        // ...and E, for the same reason: it is a ray from THIS frame's eye.
        if (m_configRequested)
        {
            m_configRequested = false;
            toggleConfigMenu();
        }

        // --- periodic statistics ------------------------------------------------
        const sw::f64 now = clock().totalSeconds();
        if (now - m_lastStatsLogSeconds > 5.0)
        {
            m_lastStatsLogSeconds = now;
            const sw::RenderStats& stats = renderer().stats();
            SW_LOG_INFO("Game",
                        "render: {} submitted, {} culled, {} instances in {} draw calls | "
                        "sim: {} dynamic / {} rails, warp x{:g} ({:.0f} FPS, prepare "
                        "{:.1f} ms)",
                        stats.itemsSubmitted, stats.itemsCulled, stats.instancesDrawn,
                        stats.drawCalls, m_world.count<sw::phys::DynamicBodyComponent>(),
                        m_world.count<sw::phys::OnRailsComponent>(),
                        kWarpLadder[m_warpIndex], clock().smoothedFps(),
                        stats.cpuPrepareMs);
        }
    }

    sw::u32 StarWorksGame::selectLodLevel(sw::f64 distance, sw::f64 worldRadius) const
    {
        const sw::f64 clampedDistance = std::max(distance, worldRadius * 1.0001);
        const sw::f64 angularDiameter = 2.0 * std::atan(worldRadius / clampedDistance);
        const sw::f32 fraction =
            static_cast<sw::f32>(angularDiameter) / m_camera.verticalFov();

        for (sw::u32 level = 0; level < CelestialLodComponent::kLodLevels - 1; ++level)
        {
            if (fraction >= kLodScreenFractions[level])
            {
                return level;
            }
        }
        return CelestialLodComponent::kLodLevels - 1;
    }

    void StarWorksGame::hudQuad(sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1,
                                const sw::Vec4& color)
    {
        sw::DrawItem item{};
        item.mesh = &m_meshes[m_navLineMeshIndex]; // unit quad
        item.transform =
            glm::translate(sw::Mat4{1.0f}, {(x0 + x1) * 0.5f, (y0 + y1) * 0.5f, 0.0f}) *
            glm::scale(sw::Mat4{1.0f}, {(x1 - x0) * 0.5f, (y1 - y0) * 0.5f, 1.0f});
        item.screenSpace = true;
        item.tint = color;
        m_drawItems.push_back(item);
    }

    void StarWorksGame::hudPanel(sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1,
                                 const sw::Vec4& fill)
    {
        constexpr sw::f32 kEdge = 0.004f;
        hudQuad(x0 - kEdge, y0 - kEdge, x1 + kEdge, y1 + kEdge, hud::kEdge);
        hudQuad(x0, y0, x1, y1, fill);
    }

    bool StarWorksGame::hudCursor(sw::f32& outX, sw::f32& outY)
    {
        sw::u32 width = 0;
        sw::u32 height = 0;
        window().framebufferSize(width, height);
        if (width == 0 || height == 0)
        {
            return false;
        }
        outX = input().mouseX() / static_cast<sw::f32>(width) * 2.0f - 1.0f;
        outY = input().mouseY() / static_cast<sw::f32>(height) * 2.0f - 1.0f;
        return true;
    }

    void StarWorksGame::hudDesignPreview(const sw::parts::ShipBlueprint& design,
                                         sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1,
                                         sw::f32 spinRadians)
    {
        if (design.parts.empty())
        {
            return;
        }

        // ---- 1. how big is it, and where is its middle --------------------
        sw::Vec3 low{1.0e9f};
        sw::Vec3 high{-1.0e9f};
        for (const sw::parts::BlueprintPartRecord& part : design.parts)
        {
            const auto* definition = sw::parts::findDefinition(part.definitionId);
            if (definition == nullptr)
            {
                continue;
            }
            const sw::f32 reach = sw::parts::partBoundsRadius(*definition);
            low = glm::min(low, part.localPosition - sw::Vec3{reach});
            high = glm::max(high, part.localPosition + sw::Vec3{reach});
        }
        if (low.x > high.x)
        {
            return; // nothing in the catalogue matched a part we have
        }
        const sw::Vec3 centre = (low + high) * 0.5f;
        const sw::Vec3 half = glm::max((high - low) * 0.5f, sw::Vec3{0.05f});

        // ---- 2. the view: proper rotations only ---------------------------
        // Stood upright by the same rotation the hangar uses, turned about
        // that vertical by the caller's angle, and tipped a little so the
        // thing reads as a solid rather than a silhouette.
        // The hangar's own display rotation: +90 deg about X puts the nose
        // (-Z) up. Written out rather than shared because the constant lives
        // in the hangar's translation unit section, further down this file.
        constexpr sw::f32 kPitch = 0.22f;
        const sw::Quat standUpright = glm::angleAxis(1.5707963f, sw::Vec3{1, 0, 0});
        const sw::Quat view = glm::angleAxis(-kPitch, sw::Vec3{1.0f, 0.0f, 0.0f}) *
                              glm::angleAxis(spinRadians, sw::Vec3{0.0f, 1.0f, 0.0f}) *
                              standUpright;

        // ---- 3. fit it to the rectangle -----------------------------------
        // NDC x is compressed by the aspect ratio and NDC y is not, so the
        // width available in "square" units is the half-width TIMES aspect.
        //
        // FIT THE SHAPE, NOT ITS BOUNDING SPHERE. A rocket is long and thin,
        // and a sphere around it is as wide as it is tall — framing by the
        // sphere throws away a third of the height for a width nothing ever
        // occupies. So: the vessel's own axis (+Z, stood upright, so it runs
        // up the screen) sets the vertical extent, its radial size sets the
        // horizontal one, and the pitch mixes a little of each into the
        // other. Worst case over a whole turn, so the model does not pulse
        // as it spins.
        const sw::f32 radial = std::max(half.x, half.y);
        const sw::f32 needHeight =
            half.z * std::cos(kPitch) + radial * std::sin(kPitch);
        const sw::f32 needWidth = radial;

        const sw::f32 aspect = renderer().aspectRatio();
        const sw::f32 halfWidth = std::abs(x1 - x0) * 0.5f;
        const sw::f32 halfHeight = std::abs(y1 - y0) * 0.5f;
        const sw::f32 scale = std::min(halfWidth * aspect / needWidth,
                                       halfHeight / needHeight) *
                              0.92f;
        const sw::Vec3 middle{(x0 + x1) * 0.5f, (y0 + y1) * 0.5f, 0.0f};

        // THE Y IS NEGATED, ONCE. The camera does the same thing in its
        // projection and the front-face convention was settled against it;
        // a preview that skipped the flip would be culled inside out.
        const sw::Mat4 frame =
            glm::translate(sw::Mat4{1.0f}, middle) *
            glm::scale(sw::Mat4{1.0f}, sw::Vec3{scale / aspect, -scale, scale}) *
            glm::mat4_cast(view);

        // ---- 4. back to front ---------------------------------------------
        // The view looks toward -Z of its own space, exactly as the camera
        // does, so the most negative depth is the farthest away.
        struct Ordered
        {
            sw::f32 depth;
            const sw::parts::BlueprintPartRecord* part;
        };
        std::vector<Ordered> ordered;
        ordered.reserve(design.parts.size());
        for (const sw::parts::BlueprintPartRecord& part : design.parts)
        {
            ordered.push_back({(view * (part.localPosition - centre)).z, &part});
        }
        std::sort(ordered.begin(), ordered.end(),
                  [](const Ordered& a, const Ordered& b) { return a.depth < b.depth; });

        // ---- 5. submit -----------------------------------------------------
        for (const Ordered& entry : ordered)
        {
            const auto meshIt = m_partMeshIds.find(entry.part->definitionId);
            if (meshIt == m_partMeshIds.end())
            {
                continue;
            }
            sw::DrawItem item{};
            item.mesh = &m_meshes[meshIt->second];
            item.transform =
                frame *
                glm::translate(sw::Mat4{1.0f}, entry.part->localPosition - centre) *
                glm::mat4_cast(entry.part->localRotation);
            item.screenSpace = true;
            item.hudSolid = true;
            item.hudLayer = static_cast<sw::u8>(sw::ui::HudLayer::Background);
            m_drawItems.push_back(item);
        }
    }

    void StarWorksGame::hudText(std::string_view text, sw::f32 x, sw::f32 y,
                                sw::f32 heightNdc, const sw::Vec4& color)
    {
        const sw::f32 aspect = renderer().aspectRatio();
        const sw::f32 scaleX = (5.0f / 7.0f) * heightNdc / aspect;
        const sw::f32 advance = sw::ui::kGlyphAdvance * heightNdc / aspect;

        sw::f32 penX = x;
        for (const char character : text)
        {
            const auto index = static_cast<sw::usize>(
                static_cast<unsigned char>(std::toupper(character)));
            const sw::u32 meshIndex =
                (index < m_glyphMeshIndex.size()) ? m_glyphMeshIndex[index] : 0xFFFFFFFFu;
            if (meshIndex != 0xFFFFFFFFu)
            {
                sw::DrawItem item{};
                item.mesh = &m_meshes[meshIndex];
                item.transform = glm::translate(sw::Mat4{1.0f}, {penX, y, 0.0f}) *
                                 glm::scale(sw::Mat4{1.0f}, {scaleX, heightNdc, 1.0f});
                item.screenSpace = true;
                // TEXT IS A LAYER, not a submission order to get right: a
                // glyph is never painted over by a panel, whatever else the
                // frame decided to draw. See UI/HudOrder.hpp.
                item.hudLayer = static_cast<sw::u8>(sw::ui::HudLayer::Text);
                item.tint = color;
                m_drawItems.push_back(item);
            }
            penX += advance;
        }
    }

    // ------------------------------------------------------------------------
    // Navigation beacons
    //
    // A surveyed site sits somewhere on 510 million square kilometres of
    // procedural ground, and from 30 km up one valley looks like the next.
    // A beacon fixes that: it draws a reticle at its own position with its
    // name and the LIVE distance under it — on the map always, and in the
    // cockpit once you are inside the beacon's declared range.
    //
    // The distance is measured from the CRAFT YOU CONTROL, not from the
    // camera: in free-cam or on the map the camera can be parked anywhere,
    // and "how far am I" has to mean the pilot, not the viewpoint.
    //
    // A beacon behind you or off the edge of the screen would be useless
    // exactly when you need it most (you are looking for it BECAUSE you
    // cannot see it), so an off-screen beacon is clamped to the border and
    // its reticle turns into an arrow pointing off that edge.
    // ------------------------------------------------------------------------
    void StarWorksGame::collectBeacons(const sw::Camera& activeCamera, bool mapView)
    {
        const sw::WorldVec3 cameraPosition = activeCamera.position();
        const sw::Mat4 viewProjection = activeCamera.viewProjectionCameraRelative();
        const sw::f32 aspect = renderer().aspectRatio();

        sw::WorldVec3 playerPosition = cameraPosition;
        if (const auto* controlled =
                m_world.tryGetComponent<TransformComponent>(controlledEntity()))
        {
            playerPosition = controlled->position;
        }

        auto centredText = [&](std::string_view text, sw::f32 centreX, sw::f32 y,
                               sw::f32 height, const sw::Vec4& color) {
            const sw::f32 advance = sw::ui::kGlyphAdvance * height / aspect;
            const sw::f32 halfWidth =
                advance * static_cast<sw::f32>(text.size()) * 0.5f;
            // A marker near the border would hang its label off the screen;
            // slide the text back on rather than truncating it.
            const sw::f32 clamped =
                glm::clamp(centreX, -0.98f + halfWidth, 0.98f - halfWidth);
            hudText(text, clamped - halfWidth, y, height, color);
        };
        auto bar = [&](sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1,
                       const sw::Vec4& color) {
            sw::DrawItem item{};
            item.mesh = &m_meshes[m_navLineMeshIndex]; // unit quad
            item.transform =
                glm::translate(sw::Mat4{1.0f},
                               {(x0 + x1) * 0.5f, (y0 + y1) * 0.5f, 0.0f}) *
                glm::scale(sw::Mat4{1.0f}, {(x1 - x0) * 0.5f, (y1 - y0) * 0.5f, 1.0f});
            item.screenSpace = true;
            item.tint = color;
            m_drawItems.push_back(item);
        };

        m_world.forEach<TransformComponent, sw::factory::BeaconComponent>(
            [&](sw::ecs::Entity, TransformComponent& transform,
                sw::factory::BeaconComponent& beacon) {
                const sw::f64 distance = glm::length(transform.position - playerPosition);
                if (!mapView &&
                    (distance > beacon.rangeM || distance < beacon.nearRangeM))
                {
                    // Too far to be lit for you, or so close that the pointer
                    // would just be covering the thing it points at.
                    return;
                }

                const sw::Vec3 relative = sw::Vec3(transform.position - cameraPosition);
                // Where the pointer goes — including the behind-you and
                // past-the-edge cases — is one tested engine function
                // (UI/ScreenMarker.hpp), not arithmetic repeated per HUD.
                const sw::ui::MarkerPlacement placement = sw::ui::placeScreenMarker(
                    viewProjection, relative, activeCamera.right(), activeCamera.up(),
                    activeCamera.forward());
                const sw::Vec2 ndc = placement.ndc;
                const bool offScreen = placement.offScreen;

                const sw::Vec4 color = offScreen ? sw::Vec4{1.0f, 0.62f, 0.20f, 0.85f}
                                                 : sw::Vec4{1.0f, 0.78f, 0.28f, 0.95f};

                // ---- the reticle: an open square, four thin bars ---------
                const sw::f32 half = (offScreen ? 0.020f : 0.028f);
                const sw::f32 halfX = half / aspect;
                constexpr sw::f32 kThick = 0.006f;
                const sw::f32 thickX = kThick / aspect;
                const sw::f32 arm = half * 0.55f;
                const sw::f32 armX = arm / aspect;
                // Corners only (an open reticle does not hide the thing it
                // is pointing at).
                for (const sw::f32 sx : {-1.0f, 1.0f})
                {
                    for (const sw::f32 sy : {-1.0f, 1.0f})
                    {
                        const sw::f32 cx = ndc.x + sx * halfX;
                        const sw::f32 cy = ndc.y + sy * half;
                        bar(cx - (sx < 0.0f ? 0.0f : armX), cy - kThick * 0.5f,
                            cx + (sx < 0.0f ? armX : 0.0f), cy + kThick * 0.5f, color);
                        bar(cx - thickX * 0.5f, cy - (sy < 0.0f ? 0.0f : arm),
                            cx + thickX * 0.5f, cy + (sy < 0.0f ? arm : 0.0f), color);
                    }
                }

                // ---- name, then the distance UNDER it --------------------
                // Glyphs are anchored at their TOP and grow downward, so
                // `textY` is the top of the block. Near the bottom border
                // there is no room under the reticle and the block flips
                // above it rather than falling off the screen.
                const sw::f32 textHeight = offScreen ? 0.030f : 0.036f;
                const bool showLabel = !offScreen;
                const sw::f32 lineStep = textHeight * 1.35f;
                const sw::f32 blockHeight = showLabel ? lineStep + textHeight : textHeight;
                sw::f32 textY = ndc.y + half + 0.016f;
                if (textY + blockHeight > 0.97f)
                {
                    textY = ndc.y - half - 0.016f - blockHeight;
                }
                const std::string_view label =
                    (beacon.label[0] != '\0') ? std::string_view{beacon.label}
                                              : std::string_view{"BEACON"};
                if (showLabel)
                {
                    centredText(label, ndc.x, textY, textHeight, color);
                    textY += lineStep;
                }
                const std::string distanceText =
                    (distance >= 10000.0)
                        ? std::format("{:.1f} KM", distance / 1000.0)
                        : ((distance >= 1000.0)
                               ? std::format("{:.2f} KM", distance / 1000.0)
                               : std::format("{:.0f} M", distance));
                centredText(distanceText, ndc.x, textY, textHeight,
                            {color.r, color.g, color.b, color.a * 0.9f});
            });
    }

    void StarWorksGame::collectHud()
    {
        constexpr sw::f32 kLine = 0.052f;
        constexpr sw::f32 kX = -0.98f;
        sw::f32 y = -0.97f;
        const sw::Vec4 main{0.65f, 0.95f, 0.75f, 0.95f};
        const sw::Vec4 dim{0.55f, 0.75f, 0.85f, 0.9f};

        // ---- mode ----------------------------------------------------------------
        const char* mode = m_mapView ? "MAP" : (m_evaMode ? "EVA" : "NAV");
        hudText(std::format("{} {}", mode, m_shipMode || m_mapView ? "" : "CAM LIBRE"), kX,
                y, kLine, main);
        y += kLine * 1.3f;

        // ---- speed & altitude, relative to the current SOI PRIMARY ---------------
        const sw::WorldVec3 velocity = controlledVelocity();
        const sw::WorldVec3 position =
            m_world.getComponent<TransformComponent>(controlledEntity()).position;

        const sw::f64 time = m_physicsLane->presentSeconds();
        const sw::i32 primaryIndex = controlledPrimaryIndex();
        const char* primaryName = "-";
        sw::WorldVec3 primaryPosition{0.0};
        sw::WorldVec3 primaryVelocity{0.0};
        sw::WorldVec3 primaryAngularVelocity{0.0};
        sw::f64 primaryRadius = 0.0;
        if (primaryIndex >= 0)
        {
            const auto& primary = m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
            primaryName = primary.name;
            primaryRadius = primary.bodyRadius;
            m_celestialIndex.stateAt(primaryIndex, time, primaryPosition,
                                     &primaryVelocity);
            if (const auto* source =
                    m_world.tryGetComponent<sw::phys::GravitySourceComponent>(
                        primary.entity))
            {
                primaryAngularVelocity = source->angularVelocity;
            }
        }

        sw::f64 speed = 0.0;
        if (m_speedSurfaceRelative)
        {
            // Surface velocity: the primary's own motion + its spin at this
            // position — works above any body, not just Terra.
            const sw::WorldVec3 surfaceVelocity =
                primaryVelocity +
                glm::cross(primaryAngularVelocity, position - primaryPosition);
            speed = glm::length(velocity - surfaceVelocity);
        }
        else
        {
            speed = glm::length(velocity - primaryVelocity); // orbital speed
        }
        hudText(std::format("SPD {} {:.1f} M/S", m_speedSurfaceRelative ? "SRF" : "ORB",
                            speed),
                kX, y, kLine, main);
        y += kLine * 1.3f;

        // ---- altitude above the primary -------------------------------------------
        const sw::f64 altitude =
            glm::length(position - primaryPosition) - primaryRadius;
        hudText(std::format("ALT {} {:.1f} KM", primaryName, altitude / 1000.0), kX, y,
                kLine, main);
        y += kLine * 1.3f;

        // ---- the air, while there is any -----------------------------------------
        //
        // Three numbers and a verdict. Dynamic pressure is what the airframe
        // feels and what a gravity turn is flown around; Mach is where the
        // drag lives; and the STABILITY MARGIN — how far the centre of
        // pressure sits behind the centre of mass, in vehicle diameters — is
        // the one number that says whether this rocket will fly straight or
        // swap ends. Positive is stable. It is shown because it is the thing
        // the player can actually fix, by moving fins or moving mass.
        const sw::ecs::Entity flown = controlledEntity();
        const auto* vesselFlown = m_world.tryGetComponent<sw::parts::VesselComponent>(flown);
        if (const auto* air = m_world.tryGetComponent<sw::aero::AeroStateComponent>(flown);
            air != nullptr && vesselFlown != nullptr && air->inAtmosphere != 0)
        {
            const sw::Vec4 warn{1.0f, 0.55f, 0.2f, 1.0f};
            const sw::f64 pressureKpa = air->dynamicPressurePa / 1000.0;
            hudText(std::format("Q {:.0f} KPA  M {:.2f}  AOA {:.0f}", pressureKpa,
                                air->machNumber,
                                air->angleOfAttackRad * 180.0 / 3.14159265358979),
                    kX, y, kLine, (pressureKpa > 45.0) ? warn : main);
            y += kLine * 1.15f;

            // +Z is the tail, so a pressure centre BEHIND the balance point
            // has the larger z. The margin is quoted in calibres — vehicle
            // widths — because that is the form the number is meaningful in:
            // one calibre of margin flies, a tenth of one is a coin toss.
            const sw::f32 calibre =
                std::max(0.5f, 2.0f * std::max(vesselFlown->halfExtents.x, 0.25f));
            const sw::f32 margin =
                (air->centreOfPressure.z - air->centreOfMass.z) / calibre;
            hudText(std::format("DRAG {:.0f} KN  MGN {:+.2f}{}", air->dragN / 1000.0,
                                margin, (margin > 0.05f) ? "" : " UNSTABLE"),
                    kX, y, kLine, (margin > 0.05f) ? main : warn);
            y += kLine * 1.3f;
        }

        // ---- current orbit around the primary: APO / PER / period ----------------
        auto formatEta = [](sw::f64 seconds) {
            return (seconds >= 3600.0) ? std::format("{:.1f} H", seconds / 3600.0)
                                       : std::format("{:.0f} S", seconds);
        };
        // A closest approach can be eight hundred metres or eight hundred
        // million kilometres, and both have to be readable at a glance.
        auto formatDistance = [](sw::f64 metres) {
            if (metres >= 1.0e9)
            {
                return std::format("{:.3f} GM", metres / 1.0e9);
            }
            if (metres >= 1000.0)
            {
                return std::format("{:.1f} KM", metres / 1000.0);
            }
            return std::format("{:.0f} M", metres);
        };
        const auto* controlledBody =
            m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(controlledEntity());
        const bool grounded = controlledBody != nullptr && controlledBody->isGrounded != 0;
        sw::phys::KeplerOrbit currentOrbit{};
        if (primaryIndex >= 0 && !grounded &&
            sw::phys::kepler::fromStateVectors(
                m_celestialIndex.body(static_cast<sw::usize>(primaryIndex)).mu,
                position - primaryPosition, velocity - primaryVelocity, time,
                currentOrbit, /*allowHyperbolic=*/true))
        {
            constexpr sw::f64 kTwoPi = 6.283185307179586;
            const sw::f64 periapsisAltitude =
                sw::phys::kepler::periapsis(currentOrbit) - primaryRadius;
            if (!currentOrbit.isHyperbolic())
            {
                const sw::f64 meanAnomaly = currentOrbit.meanAnomalyAtEpoch;
                const sw::f64 timeToPeriapsis =
                    (kTwoPi - meanAnomaly) / currentOrbit.meanMotion;
                const sw::f64 timeToApoapsis =
                    std::fmod(3.0 * 3.14159265358979 - meanAnomaly, kTwoPi) /
                    currentOrbit.meanMotion;
                const sw::f64 apoapsisAltitude =
                    sw::phys::kepler::apoapsis(currentOrbit) - primaryRadius;
                hudText(std::format("APO {:.1f} KM T-{}", apoapsisAltitude / 1000.0,
                                    formatEta(timeToApoapsis)),
                        kX, y, kLine, dim);
                y += kLine * 1.3f;
                hudText(std::format("PER {:.1f} KM T-{}", periapsisAltitude / 1000.0,
                                    formatEta(timeToPeriapsis)),
                        kX, y, kLine, dim);
                y += kLine * 1.3f;
                hudText(std::format("ORB {}", formatEta(sw::phys::kepler::period(
                                        currentOrbit))),
                        kX, y, kLine, dim);
                y += kLine * 1.3f;
            }
            else // escape trajectory: periapsis (if ahead), no apoapsis
            {
                const sw::f64 meanAnomaly = currentOrbit.meanAnomalyAtEpoch;
                if (meanAnomaly < 0.0) // still inbound toward periapsis
                {
                    hudText(std::format("PER {:.1f} KM T-{}  ESC",
                                        periapsisAltitude / 1000.0,
                                        formatEta(-meanAnomaly /
                                                  currentOrbit.meanMotion)),
                            kX, y, kLine, dim);
                }
                else
                {
                    hudText("ESCAPE TRAJECTORY", kX, y, kLine, dim);
                }
                y += kLine * 1.3f;
            }
        }

        // ---- throttle + warp -----------------------------------------------------------
        // On foot there is no throttle, and there may be no vessel at all.
        const auto* ship = m_shipEntity.isNull()
                               ? nullptr
                               : m_world.tryGetComponent<ShipComponent>(m_shipEntity);
        const std::string warpLabel = m_simulation.isPaused()
                                          ? std::string("0")
                                          : warpText(kWarpLadder[m_warpIndex]);
        hudText((ship != nullptr && !m_evaMode)
                    ? std::format("THR {:.0f}%  WARP X{}", ship->throttle * 100.0f,
                                  warpLabel)
                    : std::format("WARP X{}", warpLabel),
                kX, y, kLine, dim);
        y += kLine * 1.3f;
        // STANDING ON SOMETHING: how far over it is leaning, and whether
        // the ground is still holding it. A rocket resting at eight degrees
        // is not a broken rocket — a body inside its own support polygon
        // does not stand itself back up — but there was no way to tell that
        // from a rocket that had stopped being simulated.
        if (m_flight.grounded)
        {
            hudText(std::format("LANDED  LEAN {:.0f} DEG  {}", m_flight.leanDegrees,
                                m_flight.tipping ? "TIPPING" : "RESTING"),
                    kX, y, kLine * 0.85f,
                    m_flight.tipping ? sw::Vec4{1.0f, 0.65f, 0.25f, 1.0f} : dim);
            y += kLine * 1.15f;
        }
        // WHY THE WARP KEY IS DOING NOTHING. The gate is a rule about the
        // situation, not about the key, so it has to say which situation.
        if (!warpAllowed())
        {
            hudText(std::format("WARP LOCKED  {}", warpBlockReason()), kX, y, kLine * 0.8f,
                    sw::Vec4{0.95f, 0.45f, 0.35f, 1.0f});
            y += kLine * 1.1f;
        }

        // ---- vessel resources (parts carry them as real cargo) -----------------
        sw::f64 fuelUnits = 0.0;
        sw::f64 chargeUnits = 0.0;
        m_world.forEach<sw::parts::PartComponent, sw::factory::InventoryComponent>(
            [&](sw::ecs::Entity, sw::parts::PartComponent& part,
                sw::factory::InventoryComponent& inventory) {
                if (part.vessel != m_shipEntity)
                {
                    return;
                }
                fuelUnits +=
                    sw::factory::inventoryCount(inventory, sw::res::Resource::Fuel);
                chargeUnits += sw::factory::inventoryCount(
                    inventory, sw::res::Resource::ElectricCharge);
            });
        const sw::Vec4 fuelColor = (fuelUnits > 3000.0)
                                       ? dim
                                       : sw::Vec4{1.0f, 0.45f, 0.3f, 0.95f};
        hudText(std::format("FUEL {:.0f} KG  ELEC {:.0f} KJ", fuelUnits, chargeUnits),
                kX, y, kLine, fuelColor);
        y += kLine * 1.3f;

        // ---- flight-plan events (KSP style): the first upcoming transition --------
        for (const sw::space::TrajectorySegment& segment : m_prediction)
        {
            const char* label = nullptr;
            sw::Vec4 color = dim;
            switch (segment.endReason)
            {
            case sw::space::SegmentEnd::Encounter:
                label = "ENC";
                color = {0.4f, 1.0f, 0.9f, 0.95f};
                break;
            case sw::space::SegmentEnd::Impact:
                label = "IMPACT";
                color = {1.0f, 0.35f, 0.3f, 0.95f};
                break;
            case sw::space::SegmentEnd::SoiExit:
                label = "EXIT TO";
                color = {1.0f, 0.85f, 0.4f, 0.95f};
                break;
            default:
                break;
            }
            if (label == nullptr || segment.eventBodyIndex < 0)
            {
                continue;
            }
            const auto& eventBody =
                m_celestialIndex.body(static_cast<sw::usize>(segment.eventBodyIndex));
            const sw::f64 eta = segment.endTime - time;
            hudText(std::format("{} {} T-{:.0f} S", label, eventBody.name,
                                std::max(eta, 0.0)),
                    kX, y, kLine, color);
            y += kLine * 1.3f;
        }

        // ---- the target, and the closest approach to it -----------------------
        if (m_targetIndex >= 0 &&
            static_cast<sw::usize>(m_targetIndex) < m_celestialIndex.size())
        {
            const auto& target =
                m_celestialIndex.body(static_cast<sw::usize>(m_targetIndex));
            constexpr sw::Vec4 kTargetColor{1.0f, 0.55f, 0.88f, 0.95f};
            const sw::f64 distanceNow =
                glm::length(m_celestialIndex.positionAt(m_targetIndex, time) - position);
            hudText(std::format("TGT {}  {}", target.name, formatDistance(distanceNow)),
                    kX, y, kLine, kTargetColor);
            y += kLine * 1.3f;

            auto approachLine = [&](const sw::space::ClosestApproach& approach,
                                    const char* label, const sw::Vec4& color) {
                if (!approach.valid)
                {
                    return;
                }
                // ALTITUDE, not centre distance, once the pass is close
                // enough to be about the surface: 380 km above Luna and
                // 2 117 km from its centre are the same fact, and only one
                // of them tells you whether you hit it.
                const sw::f64 altitude = approach.distanceM - target.bodyRadius;
                const bool hits = altitude <= 0.0;
                hudText(std::format("{} {}  T-{}  REL {:.0f} M/S", label,
                                    hits ? std::string("IMPACT")
                                         : formatDistance(approach.distanceM),
                                    formatEta(std::max(approach.timeSeconds - time, 0.0)),
                                    approach.relativeSpeedMps),
                        kX, y, kLine, hits ? sw::Vec4{1.0f, 0.35f, 0.3f, 0.95f} : color);
                y += kLine * 1.3f;
            };
            approachLine(m_approach, "APPROACH", kTargetColor);
            approachLine(m_nodeApproach, "AFTER BURN",
                         sw::Vec4{0.85f, 0.75f, 1.0f, 0.95f});
        }

        // ---- maneuver node status --------------------------------------------
        if (m_nodeActive)
        {
            // Far from the node: show the PLANNED dv. Once the plan is
            // LOCKED — two minutes out — the LIVE remaining vector, which
            // now genuinely counts down as the engine burns.
            const sw::f64 timeToNode = m_nodeTime - time;
            const sw::f64 plannedDv =
                std::sqrt(m_nodePrograde * m_nodePrograde +
                          m_nodeNormal * m_nodeNormal + m_nodeRadial * m_nodeRadial);
            const sw::f64 remainingDv = glm::length(remainingBurnVector());
            const bool burnWindow = m_burnLocked;
            const sw::Vec4 nodeColor{0.8f, 0.55f, 1.0f, 0.95f};
            hudText(std::format("NODE T-{} DV {:.1f} M/S{}",
                                formatEta(std::max(timeToNode, 0.0)),
                                burnWindow ? remainingDv : plannedDv,
                                burnWindow ? " BURN" : ""),
                    kX, y, kLine, nodeColor);
            y += kLine * 1.3f;
            if (m_mapView)
            {
                hudText(std::format("PGD {:+.1f} NRM {:+.1f} RAD {:+.1f}",
                                    m_nodePrograde, m_nodeNormal, m_nodeRadial),
                        kX, y, kLine, nodeColor);
                y += kLine * 1.3f;
                // WHICH STEP IS ARMED. A ladder the player cannot see is a
                // ladder they have to remember; this line changes under
                // their thumb as they hold the modifier, which is the only
                // documentation a control like this needs.
                const bool shift = input().isKeyDown(sw::KeyCode::LeftShift) ||
                                   input().isKeyDown(sw::KeyCode::RightShift);
                const bool control = input().isKeyDown(sw::KeyCode::LeftControl) ||
                                     input().isKeyDown(sw::KeyCode::RightControl);
                const bool alt = input().isKeyDown(sw::KeyCode::LeftAlt) ||
                                 input().isKeyDown(sw::KeyCode::RightAlt);
                const sw::space::ManeuverStep step =
                    sw::space::maneuverStep(shift, control, alt);
                const char* held = (control && shift) ? "CTRL+SHIFT"
                                   : alt              ? "ALT"
                                   : shift            ? "SHIFT"
                                   : control          ? "CTRL"
                                                      : "-";
                hudText(std::format("STEP {:g} M/S  {:g} S   [{}]  {}", step.deltaVMps,
                                    step.seconds, held,
                                    m_nodeDragging ? "SLIDING" : "DRAG TO SLIDE"),
                        kX, y, kLine,
                        m_nodeDragging ? sw::Vec4{1.0f, 0.85f, 0.45f, 0.95f}
                        : (step.deltaVMps > 1.0)
                            ? sw::Vec4{1.0f, 0.78f, 0.30f, 0.95f}
                            : nodeColor);
                y += kLine * 1.3f;
            }
        }

        // ---- F2: what the ground cursor is about to do -------------------
        if (!m_mapView && !m_editorMode && m_evaMode && !m_buildMenu)
        {
            const auto* held = sw::parts::findDefinition(m_heldBuilding);
            auto nameOf = [&](sw::ecs::Entity entity) -> std::string {
                const auto* building =
                    m_world.tryGetComponent<sw::factory::BuildingComponent>(entity);
                const auto* definition =
                    (building != nullptr)
                        ? sw::parts::findDefinition(building->definitionId)
                        : nullptr;
                return (definition != nullptr) ? definition->name : std::string("?");
            };
            const bool beltMode =
                held != nullptr &&
                held->building.category == sw::factory::BuildingCategory::Conveyor;
            const bool cableMode =
                held != nullptr &&
                held->building.category == sw::factory::BuildingCategory::Cable;

            if (beltMode)
            {
                // Two clicks, and the HUD says which one you are on.
                if (m_beltSource.isNull())
                {
                    hudText("BELT  PICK AN OUTPUT", -0.36f, 0.70f, 0.038f, hud::kTitle);
                    hudText(m_buildCursor.target.isNull()
                                ? "LOOK AT A MACHINE"
                                : std::format("LCLICK  FROM {}",
                                              nameOf(m_buildCursor.target)),
                            -0.36f, 0.75f, 0.030f, hud::kTextDim);
                }
                else
                {
                    const bool ok = m_beltVerdict == sw::build::Verdict::Ok &&
                                    !m_beltPreview.empty();
                    hudText(std::format("BELT  FROM {}", nameOf(m_beltSource)), -0.36f,
                            0.70f, 0.038f, ok ? hud::kOk : hud::kBad);
                    hudText(m_beltPreview.empty()
                                ? "LOOK AT AN INPUT   R CANCEL"
                                : (ok ? std::format("LCLICK  {} SEGMENTS TO {}   R CANCEL",
                                                    m_beltPreview.size(),
                                                    nameOf(m_buildCursor.target))
                                      : std::string(sw::build::verdictText(m_beltVerdict))),
                            -0.36f, 0.75f, 0.030f, ok ? hud::kTextDim : hud::kBad);
                }
            }
            else if (cableMode)
            {
                if (m_cableSource.isNull())
                {
                    hudText("CABLE  PICK A CONNECTION", -0.36f, 0.70f, 0.038f,
                            hud::kTitle);
                    hudText(m_buildCursor.target.isNull()
                                ? std::string("LOOK AT A BUILDING OR A POLE")
                                : std::format("LCLICK  FROM {}   R CUT ITS CABLES",
                                              nameOf(m_buildCursor.target)),
                            -0.36f, 0.75f, 0.030f, hud::kTextDim);
                }
                else
                {
                    const bool ok = m_cableVerdict == sw::factory::CableVerdict::Ok;
                    hudText(std::format("CABLE  FROM {}", nameOf(m_cableSource)), -0.36f,
                            0.70f, 0.038f, ok ? hud::kOk : hud::kBad);
                    hudText(m_buildCursor.target.isNull()
                                ? std::string("LOOK AT THE OTHER END   R CANCEL")
                                : (ok ? std::format("LCLICK  WIRE TO {}   R CANCEL",
                                                    nameOf(m_buildCursor.target))
                                      : std::string(sw::factory::cableVerdictText(
                                            m_cableVerdict))),
                            -0.36f, 0.75f, 0.030f, ok ? hud::kTextDim : hud::kBad);
                }
            }
            else if (held != nullptr)
            {
                const bool ok = m_buildCursor.verdict == sw::build::Verdict::Ok;
                hudText(std::format("BUILD {}  {:.0f} M", held->name,
                                    m_buildCursor.rangeM),
                        -0.36f, 0.70f, 0.038f, ok ? hud::kOk : hud::kBad);
                hudText(ok ? "LCLICK BUILD   WHEEL ROTATE   F MENU"
                           : sw::build::verdictText(m_buildCursor.verdict),
                        -0.36f, 0.75f, 0.030f, ok ? hud::kTextDim : hud::kBad);
            }
            // F3: the machine panel. Only worth advertising when there IS a
            // machine within arm's reach, which is also exactly when E works.
            if (held == nullptr && m_configTarget.isNull() &&
                !m_buildCursor.target.isNull() &&
                m_buildCursor.rangeM <= kConfigRangeM)
            {
                hudText(std::format("E  CONFIGURE {}", nameOf(m_buildCursor.target)),
                        -0.36f, 0.70f, 0.034f, hud::kText);
            }
            if (!beltMode && !cableMode && !m_buildCursor.target.isNull())
            {
                hudText(std::format("R  DEMOLISH {}", nameOf(m_buildCursor.target)),
                        -0.36f, 0.80f, 0.030f, hud::kWarn);
            }
        }

        if (m_buildMenu && !m_editorMode)
        {
            // The catalogue takes the clickable UI over while it is open —
            // it owns m_hudButtons, so nothing behind it can be clicked
            // through.
            collectBuildMenu();
        }
        else if (!m_configTarget.isNull() && !m_editorMode && !m_mapView)
        {
            collectConfigMenu();
        }
        else if (!m_mapView && !m_editorMode)
        {
            collectNavball();
            collectSasButtons();
            // ...and the warp-to-node button in the cockpit too: the burn is
            // planned on the map but it is FLOWN here, and being sent back
            // to the map to skip four hours is a trip for nothing.
            collectWarpToNodeButton();
        }
        else if (m_mapView && !m_editorMode)
        {
            collectMapButtons();
            collectWarpToNodeButton(); // appends: collectMapButtons clears
        }
        // The multiplayer panel lives on the RIGHT and appends after
        // whichever collector above cleared the list, so it coexists with the
        // flight HUD, the map and even the build catalogue rather than
        // fighting any of them for the button table.
        if (m_netPanel && !m_editorMode)
        {
            collectNetPanel();
        }
        // (Hangar UI is not collected here: the hangar renders through its
        // own path — collectHangarItems -> collectEditorUi.)
    }

    // ========================= THE HANGAR (B) ==============================
    void StarWorksGame::enterEditor()
    {
        m_editorMode = true;
        m_pausedBeforeEditor = m_simulation.isPaused();
        m_simulation.setPaused(true);
        m_heldDefinition = 0;
        m_heldSubtree.clear();
        m_blueprintBackup.clear();
        m_heldRotation = {1.0f, 0.0f, 0.0f, 0.0f};
        m_ghost = {};
        // Open on the CURRENT ship, loaded as an editable blueprint.
        m_hangarSource = {};
        m_blueprint.clear();
        hangarLoadNextVessel();
        if (m_blueprint.empty())
        {
            hangarNewBlueprint();
        }
        SW_LOG_INFO("Game", "HANGAR: open");
    }

    void StarWorksGame::exitEditor()
    {
        m_editorMode = false;
        m_simulation.setPaused(m_pausedBeforeEditor);
        SW_LOG_INFO("Game", "HANGAR: closed");
    }

    void StarWorksGame::hangarNewBlueprint()
    {
        m_blueprint.clear();
        m_hangarSource = {};
        BlueprintPart core{};
        core.definitionId = sw::parts::kPartCoreStructural;
        m_blueprint.push_back(core); // every design starts from a command core
        m_heldDefinition = 0;
        m_heldSubtree.clear();
        m_blueprintBackup.clear();
    }

    void StarWorksGame::hangarLoadNextVessel()
    {
        // Cycle through the world's part-built vessels, loading each into
        // the blueprint for modification.
        std::vector<sw::ecs::Entity> vessels;
        m_world.forEach<sw::parts::VesselComponent>(
            [&](sw::ecs::Entity entity, sw::parts::VesselComponent& vessel) {
                if (vessel.partCount > 0)
                {
                    vessels.push_back(entity);
                }
            });
        if (vessels.empty())
        {
            return;
        }
        sw::usize next = 0;
        for (sw::usize i = 0; i < vessels.size(); ++i)
        {
            if (vessels[i] == m_hangarSource)
            {
                next = (i + 1) % vessels.size();
            }
        }
        m_hangarSource = vessels[next];

        // Parts -> blueprint (indices), joints -> parent links.
        m_blueprint.clear();
        std::vector<sw::ecs::Entity> partEntities;
        m_world.forEach<sw::parts::PartComponent>(
            [&](sw::ecs::Entity entity, sw::parts::PartComponent& part) {
                if (part.vessel != m_hangarSource)
                {
                    return;
                }
                BlueprintPart bp{};
                bp.definitionId = part.definitionId;
                bp.localPosition = part.localPosition;
                bp.localRotation = part.localRotation;
                m_blueprint.push_back(bp);
                partEntities.push_back(entity);
            });
        m_world.forEach<sw::parts::JointComponent>(
            [&](sw::ecs::Entity, sw::parts::JointComponent& jointComponent) {
                sw::i32 a = -1;
                sw::i32 b = -1;
                for (sw::usize i = 0; i < partEntities.size(); ++i)
                {
                    if (partEntities[i] == jointComponent.partA) { a = static_cast<sw::i32>(i); }
                    if (partEntities[i] == jointComponent.partB) { b = static_cast<sw::i32>(i); }
                }
                if (a >= 0 && b >= 0)
                {
                    m_blueprint[static_cast<sw::usize>(b)].parentIndex = a;
                    m_blueprint[static_cast<sw::usize>(b)].parentPoint =
                        jointComponent.attachPointA;
                    m_blueprint[static_cast<sw::usize>(b)].childPoint =
                        jointComponent.attachPointB;
                }
            });
        m_heldDefinition = 0;
        m_heldSubtree.clear();
        m_blueprintBackup.clear();
        SW_LOG_INFO("Game", "HANGAR: loaded vessel ({} parts)", m_blueprint.size());
    }

    namespace
    {
        /// The hangar shows the NOSE (-Z) up: +90 deg about X maps +Z to -Y.
        const sw::Quat kHangarDisplay = glm::angleAxis(1.5707963f, sw::Vec3{1, 0, 0});

        /// Shortest-arc rotation taking `from` onto `to` (both normalized).
        [[nodiscard]] sw::Quat rotationBetween(const sw::Vec3& from, const sw::Vec3& to)
        {
            const sw::f32 cosine = glm::dot(from, to);
            if (cosine > 0.9999f)
            {
                return {1.0f, 0.0f, 0.0f, 0.0f};
            }
            if (cosine < -0.9999f)
            {
                const sw::Vec3 seed =
                    std::abs(from.x) < 0.9f ? sw::Vec3{1, 0, 0} : sw::Vec3{0, 1, 0};
                return glm::angleAxis(3.14159265f,
                                      glm::normalize(glm::cross(from, seed)));
            }
            return glm::angleAxis(std::acos(std::clamp(cosine, -1.0f, 1.0f)),
                                  glm::normalize(glm::cross(from, to)));
        }

        constexpr sw::u32 kSymmetryOptions[6] = {1, 2, 3, 4, 6, 8};
    } // namespace

    std::vector<StarWorksGame::OpenAttachPoint> StarWorksGame::openAttachPoints()
    {
        // Open = STACK nodes of blueprint parts not consumed by any link
        // (radial attachment is surface-based and never blocks a node).
        std::vector<OpenAttachPoint> open;
        for (sw::usize i = 0; i < m_blueprint.size(); ++i)
        {
            const auto* definition =
                sw::parts::findDefinition(m_blueprint[i].definitionId);
            if (definition == nullptr)
            {
                continue;
            }
            for (sw::u8 p = 0; p < static_cast<sw::u8>(definition->nodes.size()); ++p)
            {
                if (definition->nodes[p].type != sw::parts::NodeType::Stack)
                {
                    continue;
                }
                bool occupied = false;
                for (sw::usize j = 0; j < m_blueprint.size(); ++j)
                {
                    const auto& other = m_blueprint[j];
                    if ((other.parentIndex == static_cast<sw::i32>(i) &&
                         other.parentPoint == p) ||
                        (j == i && other.parentIndex >= 0 && other.childPoint == p))
                    {
                        occupied = true;
                        break;
                    }
                }
                if (occupied)
                {
                    continue;
                }
                OpenAttachPoint point{};
                point.partIndex = static_cast<sw::i32>(i);
                point.pointIndex = p;
                point.vesselPosition =
                    m_blueprint[i].localPosition +
                    m_blueprint[i].localRotation * definition->nodes[p].position;
                point.vesselDirection =
                    m_blueprint[i].localRotation * definition->nodes[p].direction;
                point.size = definition->nodes[p].size;
                open.push_back(point);
            }
        }
        return open;
    }

    void StarWorksGame::editorCursorRay(sw::Vec3& outOrigin, sw::Vec3& outDirection)
    {
        sw::u32 width = 0;
        sw::u32 height = 0;
        window().framebufferSize(width, height);
        const sw::f32 ndcX =
            input().mouseX() / static_cast<sw::f32>(std::max(width, 1u)) * 2.0f - 1.0f;
        const sw::f32 ndcY =
            input().mouseY() / static_cast<sw::f32>(std::max(height, 1u)) * 2.0f - 1.0f;
        // Unproject two depths (reverse-Z friendly), then undo the display
        // rotation so the ray lives in the BLUEPRINT frame.
        const sw::Mat4 inverse =
            glm::inverse(m_hangarCamera.viewProjectionCameraRelative());
        const sw::Vec4 nearPoint = inverse * sw::Vec4{ndcX, ndcY, 0.9f, 1.0f};
        const sw::Vec4 farPoint = inverse * sw::Vec4{ndcX, ndcY, 0.1f, 1.0f};
        const sw::Vec3 a = sw::Vec3(nearPoint) / nearPoint.w;
        const sw::Vec3 b = sw::Vec3(farPoint) / farPoint.w;
        const sw::Quat undo = glm::inverse(kHangarDisplay);
        outOrigin = undo * sw::Vec3(m_hangarCamera.position());
        outDirection = undo * glm::normalize(b - a);
    }

    sw::f64 StarWorksGame::partWetMassKg(sw::u32 definitionId) const
    {
        const auto* definition = sw::parts::findDefinition(definitionId);
        if (definition == nullptr)
        {
            return 0.0;
        }
        sw::f64 mass = definition->dryMassKg;
        for (const auto& capacity : definition->capacities)
        {
            if (capacity.resource != sw::res::Resource::Count)
            {
                mass += capacity.units *
                        sw::res::definition(capacity.resource).massPerUnitKg;
            }
        }
        return mass;
    }

    void StarWorksGame::computeGhost()
    {
        m_ghost = {};
        if (m_heldDefinition == 0)
        {
            return;
        }
        const auto* held = sw::parts::findDefinition(m_heldDefinition);
        if (held == nullptr)
        {
            return;
        }
        sw::Vec3 origin{};
        sw::Vec3 direction{};
        editorCursorRay(origin, direction);

        // ---- 1. STACK MAGNET: nearest open node whose ray distance is small ----
        sw::f32 bestAlong = 1.0e30f;
        for (const OpenAttachPoint& node : openAttachPoints())
        {
            sw::i32 childPoint = -1;
            for (sw::u8 c = 0; c < static_cast<sw::u8>(held->nodes.size()); ++c)
            {
                if (held->nodes[c].type != sw::parts::NodeType::Stack)
                {
                    continue;
                }
                if (glm::dot(m_heldRotation * held->nodes[c].direction,
                             -node.vesselDirection) > 0.98f)
                {
                    childPoint = c;
                    break;
                }
            }
            if (childPoint < 0)
            {
                continue;
            }
            const sw::Vec3 toNode = node.vesselPosition - origin;
            const sw::f32 along = glm::dot(toNode, direction);
            if (along <= 0.0f)
            {
                continue;
            }
            const sw::f32 distance = glm::length(toNode - direction * along);
            if (distance < std::max(0.9f, node.size * 1.1f) && along < bestAlong)
            {
                bestAlong = along;
                m_ghost.active = true;
                m_ghost.rotation = m_heldRotation;
                m_ghost.position =
                    node.vesselPosition -
                    m_heldRotation * held->nodes[childPoint].position;
                m_ghost.parentIndex = node.partIndex;
                m_ghost.parentPoint = node.pointIndex;
                m_ghost.childPoint = static_cast<sw::u8>(childPoint);
            }
        }

        // ---- 2. RADIAL SURFACE: glue onto the collider under the cursor --------
        if (!m_ghost.active)
        {
            sw::i32 radialChild = -1;
            for (sw::u8 c = 0; c < static_cast<sw::u8>(held->nodes.size()); ++c)
            {
                if (held->nodes[c].type == sw::parts::NodeType::Radial)
                {
                    radialChild = c;
                    break;
                }
            }
            if (radialChild >= 0)
            {
                sw::f32 bestT = 1.0e30f;
                sw::i32 hitPart = -1;
                sw::Vec3 hitPoint{};
                sw::Vec3 hitNormal{0.0f, 0.0f, 1.0f};
                for (sw::usize i = 0; i < m_blueprint.size(); ++i)
                {
                    const auto* def =
                        sw::parts::findDefinition(m_blueprint[i].definitionId);
                    if (def == nullptr)
                    {
                        continue;
                    }
                    const sw::Quat inverseRot =
                        glm::inverse(m_blueprint[i].localRotation);
                    const sw::Vec3 localOrigin =
                        inverseRot * (origin - m_blueprint[i].localPosition);
                    const sw::Vec3 localDirection = inverseRot * direction;
                    sw::parts::PartRayHit hit{};
                    if (sw::parts::raycastPart(*def, localOrigin, localDirection,
                                               500.0f, hit) &&
                        hit.t < bestT)
                    {
                        bestT = hit.t;
                        hitPart = static_cast<sw::i32>(i);
                        hitPoint = m_blueprint[i].localPosition +
                                   m_blueprint[i].localRotation *
                                       (localOrigin + localDirection * hit.t);
                        hitNormal = m_blueprint[i].localRotation * hit.normal;
                    }
                }
                if (hitPart >= 0)
                {
                    const sw::Vec3 glueDirection =
                        m_heldRotation * held->nodes[radialChild].direction;
                    const sw::Quat align = rotationBetween(glueDirection, -hitNormal);
                    m_ghost.active = true;
                    m_ghost.rotation = align * m_heldRotation;
                    m_ghost.position =
                        hitPoint -
                        m_ghost.rotation * held->nodes[radialChild].position;
                    m_ghost.parentIndex = hitPart;
                    m_ghost.parentPoint = 255; // surface attachment
                    m_ghost.childPoint = static_cast<sw::u8>(radialChild);
                }
            }
        }

        if (!m_ghost.active)
        {
            // Free-floating red ghost: nothing under the cursor to attach to.
            m_ghost.position = origin + direction * (m_hangarDistance * 0.7f);
            m_ghost.rotation = m_heldRotation;
            return;
        }

        // ---- validation: real compound-collider overlap + joint load -----------
        // The candidate set = held root (+ its grabbed subtree) (+ symmetry
        // clones for radial placement). Every candidate is tested against
        // every placed part with the SAME OBB test the game trusts.
        struct Candidate
        {
            const sw::parts::PartDefinition* definition;
            sw::Vec3 position;
            sw::Quat rotation;
        };
        std::vector<Candidate> candidates;
        const bool surface = m_ghost.parentPoint == 255;
        const sw::u32 cloneCount =
            (surface && m_heldSubtree.empty()) ? m_symmetryCount : 1;
        for (sw::u32 k = 0; k < cloneCount; ++k)
        {
            const sw::f32 angle =
                2.0f * 3.14159265f * static_cast<sw::f32>(k) / cloneCount;
            const sw::Quat spin = glm::angleAxis(angle, sw::Vec3{0, 0, 1});
            candidates.push_back(
                {held, spin * m_ghost.position, spin * m_ghost.rotation});
            for (const BlueprintPart& rel : m_heldSubtree)
            {
                const auto* relDef = sw::parts::findDefinition(rel.definitionId);
                if (relDef != nullptr)
                {
                    candidates.push_back(
                        {relDef,
                         spin * (m_ghost.position + m_ghost.rotation * rel.localPosition),
                         spin * (m_ghost.rotation * rel.localRotation)});
                }
            }
        }
        bool collides = false;
        for (const Candidate& candidate : candidates)
        {
            for (const BlueprintPart& placed : m_blueprint)
            {
                const auto* placedDef = sw::parts::findDefinition(placed.definitionId);
                if (placedDef == nullptr)
                {
                    continue;
                }
                if (sw::parts::partsOverlap(*candidate.definition, candidate.position,
                                            candidate.rotation, *placedDef,
                                            placed.localPosition, placed.localRotation,
                                            0.05f))
                {
                    collides = true;
                    break;
                }
            }
            if (collides)
            {
                break;
            }
        }

        sw::f64 childMass = partWetMassKg(m_heldDefinition);
        for (const BlueprintPart& rel : m_heldSubtree)
        {
            childMass += partWetMassKg(rel.definitionId);
        }
        const auto* parentDef = sw::parts::findDefinition(
            m_blueprint[static_cast<sw::usize>(m_ghost.parentIndex)].definitionId);
        const bool overloaded =
            childMass * 12.0 >
            std::min(held->breakingForceN, parentDef->breakingForceN);
        m_ghost.valid = !collides && !overloaded;
    }

    void StarWorksGame::commitGhost()
    {
        if (!m_ghost.active || !m_ghost.valid || m_heldDefinition == 0)
        {
            return;
        }
        const bool surface = m_ghost.parentPoint == 255;
        const sw::u32 cloneCount =
            (surface && m_heldSubtree.empty()) ? m_symmetryCount : 1;
        const sw::i32 group =
            cloneCount > 1 ? m_symmetryNextGroup++ : -1;
        for (sw::u32 k = 0; k < cloneCount; ++k)
        {
            const sw::f32 angle =
                2.0f * 3.14159265f * static_cast<sw::f32>(k) / cloneCount;
            const sw::Quat spin = glm::angleAxis(angle, sw::Vec3{0, 0, 1});
            BlueprintPart part{};
            part.definitionId = m_heldDefinition;
            part.localPosition = spin * m_ghost.position;
            part.localRotation = spin * m_ghost.rotation;
            part.parentIndex = m_ghost.parentIndex;
            part.parentPoint = m_ghost.parentPoint;
            part.childPoint = m_ghost.childPoint;
            part.symmetryGroup = group;
            m_blueprint.push_back(part);
            const sw::i32 rootIndex = static_cast<sw::i32>(m_blueprint.size()) - 1;
            const sw::i32 subBase = rootIndex + 1;
            for (const BlueprintPart& rel : m_heldSubtree)
            {
                BlueprintPart absolute = rel;
                absolute.localPosition =
                    spin * (m_ghost.position + m_ghost.rotation * rel.localPosition);
                absolute.localRotation =
                    spin * (m_ghost.rotation * rel.localRotation);
                absolute.parentIndex =
                    rel.parentIndex < 0 ? rootIndex : subBase + rel.parentIndex;
                absolute.symmetryGroup = -1;
                m_blueprint.push_back(absolute);
            }
        }
        m_heldDefinition = 0;
        m_heldSubtree.clear();
        m_blueprintBackup.clear();
        m_ghost = {};
    }

    void StarWorksGame::grabPartAt(sw::usize index)
    {
        if (index >= m_blueprint.size() || m_blueprint[index].parentIndex < 0)
        {
            return; // never grab a root part
        }
        m_blueprintBackup = m_blueprint; // ESC puts everything back

        // Subtree = the part and everything below it (children recurse).
        std::vector<sw::usize> subtree{index};
        for (sw::usize scan = 0; scan < subtree.size(); ++scan)
        {
            for (sw::usize j = 0; j < m_blueprint.size(); ++j)
            {
                if (m_blueprint[j].parentIndex ==
                    static_cast<sw::i32>(subtree[scan]))
                {
                    subtree.push_back(j);
                }
            }
        }

        const BlueprintPart root = m_blueprint[index];
        m_heldDefinition = root.definitionId;
        m_heldRotation = root.localRotation;
        const sw::Quat inverseRoot = glm::inverse(root.localRotation);

        // Relative copies, parents remapped into the subtree (-1 = the root).
        std::vector<sw::i32> toSubtree(m_blueprint.size(), -2);
        toSubtree[index] = -1;
        m_heldSubtree.clear();
        for (sw::usize s = 1; s < subtree.size(); ++s)
        {
            BlueprintPart rel = m_blueprint[subtree[s]];
            rel.localPosition = inverseRoot * (rel.localPosition - root.localPosition);
            rel.localRotation = inverseRoot * rel.localRotation;
            rel.parentIndex = toSubtree[static_cast<sw::usize>(rel.parentIndex)];
            rel.symmetryGroup = -1;
            toSubtree[subtree[s]] = static_cast<sw::i32>(m_heldSubtree.size());
            m_heldSubtree.push_back(rel);
        }

        // The root's symmetry siblings become independent parts.
        if (root.symmetryGroup >= 0)
        {
            for (BlueprintPart& bp : m_blueprint)
            {
                bp.symmetryGroup =
                    bp.symmetryGroup == root.symmetryGroup ? -1 : bp.symmetryGroup;
            }
        }

        // Remove the subtree, remapping the survivors' parent indices.
        std::vector<bool> removed(m_blueprint.size(), false);
        for (const sw::usize s : subtree)
        {
            removed[s] = true;
        }
        std::vector<sw::i32> newIndex(m_blueprint.size(), -1);
        sw::i32 next = 0;
        for (sw::usize i = 0; i < m_blueprint.size(); ++i)
        {
            if (!removed[i])
            {
                newIndex[i] = next++;
            }
        }
        std::vector<BlueprintPart> remaining;
        remaining.reserve(m_blueprint.size() - subtree.size());
        for (sw::usize i = 0; i < m_blueprint.size(); ++i)
        {
            if (removed[i])
            {
                continue;
            }
            BlueprintPart bp = m_blueprint[i];
            if (bp.parentIndex >= 0)
            {
                bp.parentIndex = newIndex[static_cast<sw::usize>(bp.parentIndex)];
            }
            remaining.push_back(bp);
        }
        m_blueprint = std::move(remaining);
        SW_LOG_INFO("Game", "HANGAR: grabbed a subtree of {} part(s)",
                    subtree.size());
    }

    sw::ecs::Entity StarWorksGame::instantiateBlueprint(sw::ecs::Entity existingRoot,
                                                        sw::ecs::Entity pad)
    {
        sw::ecs::Entity root = existingRoot;

        // THE VESSEL'S GROUND HULL, straight from the blueprint: the box its
        // collider shapes fill in vessel space. The pad uses it to stand the
        // rocket ON its engine bells instead of at a guessed offset, and
        // VesselAssemblySystem recomputes the very same box every tick from
        // the live parts — so the spawn pose and the resting pose agree by
        // construction rather than by a constant somebody has to maintain.
        sw::phys::GroundHullComponent hull{};
        {
            constexpr sw::f32 kHuge = 1.0e9f;
            sw::Vec3 low{kHuge, kHuge, kHuge};
            sw::Vec3 high{-kHuge, -kHuge, -kHuge};
            for (const BlueprintPart& bp : m_blueprint)
            {
                if (const auto* definition = sw::parts::findDefinition(bp.definitionId))
                {
                    sw::parts::expandPartHullBounds(*definition, bp.localPosition,
                                                        bp.localRotation, low, high);
                }
            }
            if (low.x <= high.x)
            {
                hull.centre = (low + high) * 0.5f;
                hull.halfExtents = (high - low) * 0.5f;
            }
        }

        if (root.isNull())
        {
            // NEW vessel: born on a LAUNCH PAD, standing on the ground,
            // co-rotating with the planet, nose to the sky.
            //
            // WHICH pad is the F5 question. A real LP-1 the player built has
            // an anchor — a body and a body-frame position — and that is all
            // this needs; everything else below is the same arithmetic it
            // always was. With no pad (the hangar's BUILD shortcut) it falls
            // back to the surveyed place 120 m east of the outpost hub.
            sw::ecs::Entity bodyEntity = m_terraEntity;
            sw::Vec3 padDir{0.0f, 1.0f, 0.0f};
            sw::f64 groundRadius = 0.0;
            const auto* padAnchor =
                m_world.tryGetComponent<sw::phys::SurfaceAnchorComponent>(pad);
            if (padAnchor != nullptr &&
                m_world.hasComponent<TransformComponent>(padAnchor->body))
            {
                bodyEntity = padAnchor->body;
                const sw::f64 radius = glm::length(padAnchor->localPosition);
                padDir = (radius > 1.0) ? sw::Vec3(padAnchor->localPosition / radius)
                                        : sw::Vec3{0.0f, 1.0f, 0.0f};
                // ...and the rocket stands on the pad's DECK, not on the dirt
                // the pad is bolted to.
                //
                // The deck is the top of the hull boxes that lie UNDER THE
                // PAD'S AXIS, not the top of the whole hull: LP-1 carries a
                // service tower 18 m tall in one corner, and taking the
                // bounding box's ceiling stood the rocket up there in the
                // air beside it. Reading it off the boxes means redrawing
                // LP-1 thicker in Part Studio still moves the rocket with
                // it, which is the property worth keeping.
                sw::f64 deck = 0.0;
                if (const auto* building =
                        m_world.tryGetComponent<sw::factory::BuildingComponent>(pad))
                {
                    if (const auto* padDefinition =
                            sw::parts::findDefinition(building->definitionId))
                    {
                        for (const sw::parts::HitBox& box :
                             sw::parts::effectiveHull(*padDefinition))
                        {
                            if (std::abs(box.center.x) <= std::abs(box.halfExtents.x) &&
                                std::abs(box.center.z) <= std::abs(box.halfExtents.z))
                            {
                                deck = std::max(deck, static_cast<sw::f64>(
                                                          box.center.y +
                                                          std::abs(box.halfExtents.y)));
                            }
                        }
                    }
                }
                groundRadius = radius + deck;
            }
            else
            {
                // 120 m east of the site hub: the same surveyed ground, so
                // the pad is on land and the factory is walking distance
                // away.
                const sw::Vec3 siteDir = terraStartSite();
                const sw::Vec3 padEast = glm::normalize(glm::cross(
                    (std::abs(siteDir.y) < 0.9f) ? sw::Vec3{0.0f, 1.0f, 0.0f}
                                                 : sw::Vec3{1.0f, 0.0f, 0.0f},
                    siteDir));
                padDir = glm::normalize(
                    siteDir + padEast * (120.0f / static_cast<sw::f32>(kTerraRadius)));
                groundRadius =
                    kTerraRadius + sw::planet::terrainElevation(presetTerra(), padDir);
            }

            const auto& terra = m_world.getComponent<TransformComponent>(bodyEntity);
            const auto& gravity =
                m_world.getComponent<sw::phys::GravitySourceComponent>(bodyEntity);
            // A new rocket stands TAIL DOWN, so its model +Z is the axis
            // pointing at the ground: the clearance is exactly how far the
            // hull reaches along +Z. (The 11 m constant this replaces was a
            // guess at one particular rocket's half length.)
            const sw::f64 clearance =
                static_cast<sw::f64>(hull.centre.z + hull.halfExtents.z);
            const sw::WorldVec3 padLocal =
                sw::WorldVec3(padDir) * (groundRadius + clearance);
            // Same rule as every surface anchor: a planet-radius offset is
            // rotated with the f64 spin, or the rocket lands a metre from
            // the pad it was supposed to be standing on.
            const glm::dquat terraRotation = sw::phys::spinRotation(gravity);
            const sw::WorldVec3 position = terra.position + terraRotation * padLocal;
            const sw::WorldVec3 radial = position - terra.position;
            const sw::Vec3 up = sw::Vec3(glm::normalize(radial));
            const sw::Vec3 zAxis = -up; // rocket +Z (tail) points down
            const sw::Vec3 reference =
                (std::abs(zAxis.y) < 0.99f) ? sw::Vec3{0, 1, 0} : sw::Vec3{1, 0, 0};
            const sw::Vec3 xAxis = glm::normalize(glm::cross(reference, zAxis));
            const sw::Vec3 yAxis = glm::cross(zAxis, xAxis);

            root = m_world.createEntity();
            TransformComponent transform{};
            transform.position = position;
            transform.rotation = glm::quat_cast(sw::Mat3{xAxis, yAxis, zAxis});
            m_world.addComponent(root, transform);
            m_world.addComponent(root, PreviousTransformComponent{transform.position,
                                                                  transform.rotation});
            m_world.addComponent(root, BoundsComponent{0.1f});
            m_world.addComponent(root, MapMarkerComponent{{0.4f, 0.9f, 1.0f, 1.0f}});
            m_world.addComponent(root, ShipComponent{});
            m_world.addComponent(root, ShipControlsComponent{});
            m_world.addComponent(root, SasComponent{});
            m_world.addComponent(root, sw::parts::VesselComponent{});
            // The air's answer, refreshed every tick. Its PRESENCE is
            // also the switch that turns the old isotropic drag off for
            // this vessel: a part-built craft is flown by its tables.
            m_world.addComponent(root, sw::aero::AeroStateComponent{});
            sw::phys::DynamicBodyComponent body{};
            body.velocity = gravity.worldVelocity +
                            glm::cross(gravity.angularVelocity, radial);
            body.mass = 1.0e4;
            m_world.addComponent(root, body);
            m_world.addComponent(root, hull);
        }
        else
        {
            // APPLY to the loaded vessel: tear out its old parts & joints.
            std::vector<sw::ecs::Entity> stale;
            m_world.forEach<sw::parts::PartComponent>(
                [&](sw::ecs::Entity entity, sw::parts::PartComponent& part) {
                    if (part.vessel == root)
                    {
                        stale.push_back(entity);
                    }
                });
            m_world.forEach<sw::parts::JointComponent>(
                [&](sw::ecs::Entity entity, sw::parts::JointComponent& joint) {
                    for (const sw::ecs::Entity part : stale)
                    {
                        if (joint.partA == part || joint.partB == part)
                        {
                            stale.push_back(entity);
                            return;
                        }
                    }
                });
            for (const sw::ecs::Entity entity : stale)
            {
                m_world.destroyEntity(entity);
            }
        }

        // Blueprint -> live part entities + joints (poses AND rotations).
        const auto& rootTransform = m_world.getComponent<TransformComponent>(root);
        std::vector<sw::ecs::Entity> spawned;
        for (const BlueprintPart& bp : m_blueprint)
        {
            const auto* definition = sw::parts::findDefinition(bp.definitionId);
            const sw::ecs::Entity part = m_world.createEntity();
            TransformComponent transform{};
            transform.position =
                rootTransform.position +
                sw::WorldVec3(rootTransform.rotation * bp.localPosition);
            transform.rotation = rootTransform.rotation * bp.localRotation;
            m_world.addComponent(part, transform);
            m_world.addComponent(part, PreviousTransformComponent{transform.position,
                                                                  transform.rotation});
            m_world.addComponent(part, BoundsComponent{
                                           sw::parts::partBoundsRadius(*definition)});
            m_world.addComponent(part, MeshComponent{m_partMeshIds.at(bp.definitionId)});
            sw::parts::PartComponent component{};
            component.definitionId = bp.definitionId;
            component.vessel = root;
            component.localPosition = bp.localPosition;
            component.localRotation = bp.localRotation;
            m_world.addComponent(part, component);
            // Rocket parts are solid too: you cannot walk through a fuel
            // tank, and a landed booster is furniture like anything else.
            {
                sw::phys::HullComponent hull{};
                if (hullFor(*definition, hull))
                {
                    m_world.addComponent(part, hull);
                }
            }
            if (definition->capacities[0].resource != sw::res::Resource::Count)
            {
                sw::factory::InventoryComponent inventory{};
                const auto resource = definition->capacities[0].resource;
                inventory.volumeCapacityM3 =
                    definition->capacities[0].units *
                    sw::res::definition(resource).volumePerUnitM3 * 1.02;
                if (definition->type == sw::parts::PartType::FuelTank)
                {
                    sw::factory::inventoryAdd(inventory, resource,
                                              definition->capacities[0].units);
                }
                m_world.addComponent(part, inventory);
            }
            spawned.push_back(part);
        }
        for (sw::usize i = 0; i < m_blueprint.size(); ++i)
        {
            const BlueprintPart& bp = m_blueprint[i];
            if (bp.parentIndex < 0)
            {
                continue;
            }
            const auto* a = sw::parts::findDefinition(
                m_blueprint[static_cast<sw::usize>(bp.parentIndex)].definitionId);
            const auto* b = sw::parts::findDefinition(bp.definitionId);
            const sw::f64 force = std::min(a->breakingForceN, b->breakingForceN);
            // Surface attachments (parentPoint 255) are radial by nature;
            // node attachments take the joint type of the parent node.
            const bool radial =
                bp.parentPoint == 255 ||
                (bp.parentPoint < a->nodes.size() &&
                 a->nodes[bp.parentPoint].type == sw::parts::NodeType::Radial);
            sw::parts::connectParts(
                m_world, spawned[static_cast<sw::usize>(bp.parentIndex)], spawned[i],
                bp.parentPoint, bp.childPoint,
                radial ? sw::parts::JointType::Radial : sw::parts::JointType::Stack,
                force, force);
        }
        return root;
    }

    // ------------------------------------------------------------------------
    // F5 — A DESIGN IS A FILE, AND A FILE IS A ROCKET
    //
    // The hangar's working list and the saved `.swship` record are the same
    // data seen twice: the editor's list carries the parent/child joint the
    // ghost snapped to, and the file carries it too. Convert both ways and a
    // design survives a restart with its structure intact — which matters,
    // because the joints are what a decoupler cuts and what breaks under
    // load. A pile of parts flying in formation is not a rocket.
    // ------------------------------------------------------------------------
    std::vector<StarWorksGame::BlueprintPart> StarWorksGame::partsFromDesign(
        const sw::parts::ShipBlueprint& design)
    {
        std::vector<BlueprintPart> parts;
        parts.reserve(design.parts.size());
        for (const sw::parts::BlueprintPartRecord& record : design.parts)
        {
            BlueprintPart part{};
            part.definitionId = record.definitionId;
            part.localPosition = record.localPosition;
            part.localRotation = record.localRotation;
            part.parentIndex = record.parentIndex;
            part.parentPoint = record.parentPoint;
            part.childPoint = record.childPoint;
            part.symmetryGroup = record.symmetryGroup;
            parts.push_back(part);
        }
        return parts;
    }

    sw::parts::ShipBlueprint StarWorksGame::designFromParts(std::string_view name) const
    {
        sw::parts::ShipBlueprint design{};
        design.name = std::string(name);
        design.parts.reserve(m_blueprint.size());
        for (const BlueprintPart& part : m_blueprint)
        {
            sw::parts::BlueprintPartRecord record{};
            record.definitionId = part.definitionId;
            record.localPosition = part.localPosition;
            record.localRotation = part.localRotation;
            record.parentIndex = part.parentIndex;
            record.parentPoint = part.parentPoint;
            record.childPoint = part.childPoint;
            record.symmetryGroup = part.symmetryGroup;
            design.parts.push_back(record);
        }
        return design;
    }

    std::string StarWorksGame::hangarSaveShip()
    {
        if (m_blueprint.empty())
        {
            SW_LOG_WARN("Game", "HANGAR: nothing to save");
            return {};
        }
        // The name is the one thing the hangar has no field for yet, so it
        // is derived: DESIGN 1, DESIGN 2... A rename UI is a text box, and a
        // text box is a whole input mode; the file is the important half and
        // it is on disk, editable, from this milestone on.
        std::string name;
        for (sw::u32 i = 1; i < 100; ++i)
        {
            name = std::format("DESIGN {}", i);
            if (sw::parts::findBlueprint(name) == nullptr)
            {
                break;
            }
        }
        const sw::parts::ShipBlueprint design = designFromParts(name);

        const std::filesystem::path directory =
            sw::FileSystem::executableDirectory() / "Assets" / "Ships";
        std::error_code error{};
        std::filesystem::create_directories(directory, error);
        std::string file;
        for (const char c : name)
        {
            file += (c == ' ') ? '_' : static_cast<char>(std::tolower(c));
        }
        const std::filesystem::path path = directory / (file + ".swship");
        if (!sw::parts::saveBlueprintFile(design, path))
        {
            SW_LOG_ERROR("Game", "HANGAR: could not save '{}'", path.string());
            return {};
        }
        // Registered as well as written: the point of saving is that you can
        // walk to the VAB and order it, now, without restarting the game.
        sw::parts::registerBlueprint(design);
        const sw::parts::BillOfMaterials bill = sw::parts::blueprintCost(design);
        SW_LOG_INFO("Game",
                    "HANGAR: saved '{}' ({} parts, {:.0f} kg iron, {:.0f} kg copper)",
                    name, design.parts.size(), bill.ironKg, bill.copperKg);
        return name;
    }

    void StarWorksGame::orderVehicle(sw::ecs::Entity hall,
                                     const sw::parts::ShipBlueprint& design)
    {
        auto* assembly = m_world.tryGetComponent<sw::factory::AssemblyComponent>(hall);
        if (assembly == nullptr)
        {
            return;
        }
        const sw::parts::BillOfMaterials bill = sw::parts::blueprintCost(design);
        sw::factory::assemblyOrder(*assembly, design.name, bill.ironKg, bill.copperKg);
        SW_LOG_INFO("Game", "VAB: ordered '{}' — {:.0f} kg iron, {:.0f} kg copper",
                    design.name, bill.ironKg, bill.copperKg);
    }

    sw::f64 StarWorksGame::fuelVessel(sw::ecs::Entity vessel, sw::f64 availableUnits)
    {
        if (availableUnits <= 0.0)
        {
            return 0.0;
        }
        sw::f64 poured = 0.0;
        m_world.forEach<sw::parts::PartComponent, sw::factory::InventoryComponent>(
            [&](sw::ecs::Entity, sw::parts::PartComponent& part,
                sw::factory::InventoryComponent& inventory) {
                if (part.vessel != vessel || poured >= availableUnits)
                {
                    return;
                }
                const auto* definition = sw::parts::findDefinition(part.definitionId);
                if (definition == nullptr ||
                    definition->capacities[0].resource != sw::res::Resource::Fuel)
                {
                    return;
                }
                poured += sw::factory::inventoryAdd(inventory, sw::res::Resource::Fuel,
                                                    availableUnits - poured);
            });
        return poured;
    }

    // ------------------------------------------------------------------------
    // THE PAD
    //
    // A crate of vehicle arrives on the belt like any other good, and the
    // pad's job is to turn it back into a thing: pop the design's name off
    // the hall's queue — reached through the belt's own link channel, which
    // already records which machine is at the far end — build it standing on
    // the deck, and pour whatever fuel the pad has been stockpiling into it.
    //
    // It runs in the GAME, not the factory lane, because it makes entities:
    // parts, joints, a vessel. The factory layer's contract is that it moves
    // matter and nothing else.
    // ------------------------------------------------------------------------
    // ------------------------------------------------------------------------
    // IS SOMETHING STANDING ON THE PAD?
    //
    // Asked of the VESSELS, not of the hulls: a rocket is a root entity with
    // a swarm of parts hanging off it, and testing the root against the
    // deck's own footprint is one distance per vessel — cheap enough to ask
    // every frame, and it answers the question a launch director would ask.
    //
    // The radius comes from the pad's own deck boxes, so a wider LP-1
    // redrawn in Part Studio guards a wider deck. The vertical reach is
    // deliberately generous: a rocket fifty metres up is still ON the pad as
    // far as dropping another one there is concerned.
    // ------------------------------------------------------------------------
    bool StarWorksGame::padIsOccupied(sw::ecs::Entity pad)
    {
        const auto* padTransform = m_world.tryGetComponent<TransformComponent>(pad);
        if (padTransform == nullptr)
        {
            return false;
        }
        sw::f64 deckRadius = 12.0;
        if (const auto* building =
                m_world.tryGetComponent<sw::factory::BuildingComponent>(pad))
        {
            if (const auto* definition =
                    sw::parts::findDefinition(building->definitionId))
            {
                for (const sw::parts::HitBox& box :
                     sw::parts::effectiveHull(*definition))
                {
                    if (std::abs(box.center.x) <= std::abs(box.halfExtents.x) &&
                        std::abs(box.center.z) <= std::abs(box.halfExtents.z))
                    {
                        deckRadius = std::max(
                            deckRadius, static_cast<sw::f64>(std::max(
                                            std::abs(box.halfExtents.x),
                                            std::abs(box.halfExtents.z))));
                    }
                }
            }
        }
        // ...plus the height a launching rocket has to clear before the pad
        // counts as free again.
        constexpr sw::f64 kClearanceM = 60.0;
        const sw::f64 reach = deckRadius + kClearanceM;

        bool occupied = false;
        m_world.forEach<sw::parts::VesselComponent, TransformComponent>(
            [&](sw::ecs::Entity, sw::parts::VesselComponent&,
                TransformComponent& transform) {
                if (occupied)
                {
                    return;
                }
                occupied = glm::length(transform.position - padTransform->position) <
                           reach;
            });
        return occupied;
    }

    void StarWorksGame::updateLaunchPads()
    {
        std::vector<sw::ecs::Entity> pads;
        m_world.forEach<sw::factory::BuildingComponent, sw::factory::InventoryComponent>(
            [&](sw::ecs::Entity entity, sw::factory::BuildingComponent& building,
                sw::factory::InventoryComponent& inventory) {
                if (building.category == sw::factory::BuildingCategory::Pad &&
                    sw::factory::inventoryCount(inventory, sw::res::Resource::Vehicle) >=
                        1.0)
                {
                    pads.push_back(entity);
                }
            });

        for (const sw::ecs::Entity pad : pads)
        {
            // IS THERE ALREADY A ROCKET ON IT?
            //
            // A pad holds one vehicle. Unpacking a second one on top of the
            // first put two rockets in the same cubic metre — which the hull
            // solver then resolved by throwing one of them off the pad. The
            // crate simply waits: it is on the pad's own belt, it is not
            // going anywhere, and the panel says the deck is occupied.
            if (padIsOccupied(pad))
            {
                continue;
            }

            // WHICH design is in the crate. The belt that brought it names
            // its source, and the source is the hall that built it.
            std::string name;
            sw::ecs::Entity hall{};
            if (const auto* link =
                    m_world.tryGetComponent<sw::factory::ItemLinkComponent>(pad))
            {
                for (const sw::factory::LinkChannel& channel : link->channels)
                {
                    if (channel.resource != sw::res::Resource::Vehicle)
                    {
                        continue;
                    }
                    if (auto* queue =
                            m_world.tryGetComponent<sw::factory::VehicleQueueComponent>(
                                channel.source))
                    {
                        const std::string_view front =
                            sw::factory::vehicleQueueFront(*queue);
                        if (!front.empty())
                        {
                            name = std::string(front);
                            hall = channel.source;
                            break;
                        }
                    }
                }
            }
            const sw::parts::ShipBlueprint* design =
                name.empty() ? nullptr : sw::parts::findBlueprint(name);
            if (design == nullptr || !sw::parts::blueprintIsBuildable(*design))
            {
                // An unidentified crate is not destroyed and not unpacked:
                // it sits on the pad, and the panel shows it sitting there.
                // Silently deleting a rocket would be the worse answer.
                continue;
            }

            // ---- unpack it ------------------------------------------------
            std::vector<BlueprintPart> saved;
            saved.swap(m_blueprint);
            m_blueprint = partsFromDesign(*design);
            const sw::ecs::Entity vessel = instantiateBlueprint({}, pad);
            m_blueprint.swap(saved);
            if (vessel.isNull())
            {
                continue;
            }

            auto& inventory =
                m_world.getComponent<sw::factory::InventoryComponent>(pad);
            sw::factory::inventoryRemove(inventory, sw::res::Resource::Vehicle, 1.0);
            if (auto* queue =
                    m_world.tryGetComponent<sw::factory::VehicleQueueComponent>(hall))
            {
                sw::factory::vehicleQueuePop(*queue);
            }

            // ...and fuel it from the pad's own tanks. A rocket that arrives
            // dry is a rocket you have to feed by hand; the pad's second
            // conveyor mouth exists precisely so you do not.
            const sw::f64 fuel =
                sw::factory::inventoryCount(inventory, sw::res::Resource::Fuel);
            const sw::f64 poured = fuelVessel(vessel, fuel);
            if (poured > 0.0)
            {
                sw::factory::inventoryRemove(inventory, sw::res::Resource::Fuel, poured);
            }
            rebuildHulls();
            SW_LOG_INFO("Game", "PAD: '{}' rolled out — {:.0f} kg of fuel aboard", name,
                        poured);
        }
    }

    void StarWorksGame::cyclePilotedVessel()
    {
        std::vector<sw::ecs::Entity> pilotable;
        m_world.forEach<ShipComponent>([&](sw::ecs::Entity entity, ShipComponent&) {
            pilotable.push_back(entity);
        });
        if (pilotable.empty())
        {
            SW_LOG_INFO("Game", "No vessel exists yet — order one at the VAB");
            return;
        }

        // ON FOOT, `P` BOARDS THE NEAREST ONE. Cycling through a list is the
        // right verb when you are already flying and want the other rocket;
        // it is the wrong one when you are standing next to exactly one.
        if (m_evaMode || m_shipEntity.isNull() || !m_world.isAlive(m_shipEntity))
        {
            const sw::WorldVec3 here =
                m_world.getComponent<TransformComponent>(controlledEntity()).position;
            sw::ecs::Entity best{};
            sw::f64 bestDistance = 0.0;
            for (const sw::ecs::Entity candidate : pilotable)
            {
                const auto* transform =
                    m_world.tryGetComponent<TransformComponent>(candidate);
                if (transform == nullptr)
                {
                    continue;
                }
                const sw::f64 distance = glm::length(transform->position - here);
                if (best.isNull() || distance < bestDistance)
                {
                    best = candidate;
                    bestDistance = distance;
                }
            }
            if (best.isNull())
            {
                return;
            }
            m_shipEntity = best;
            m_evaMode = false;
            m_sasMode = 0;
            SW_LOG_INFO("Game", "Boarded vessel {} ({:.0f} m away)", m_shipEntity.index,
                        bestDistance);
            return;
        }

        if (pilotable.size() < 2)
        {
            return;
        }
        sw::usize next = 0;
        for (sw::usize i = 0; i < pilotable.size(); ++i)
        {
            if (pilotable[i] == m_shipEntity)
            {
                next = (i + 1) % pilotable.size();
            }
        }
        m_shipEntity = pilotable[next];
        m_sasMode = 0;
        SW_LOG_INFO("Game", "PILOTING vessel {}", m_shipEntity.index);
    }

    void StarWorksGame::updateEditor()
    {
        // Hangar camera: right-drag orbits, wheel zooms.
        if (input().isMouseButtonDown(sw::MouseButton::Right))
        {
            m_hangarYaw -= input().mouseDeltaX() * 0.005f;
            m_hangarPitch = std::clamp(m_hangarPitch - input().mouseDeltaY() * 0.005f,
                                       -1.2f, 1.4f);
        }
        if (const sw::f32 scroll = input().scrollDeltaY(); scroll != 0.0f)
        {
            m_hangarDistance =
                std::clamp(m_hangarDistance * std::pow(1.15f, -scroll), 6.0f, 90.0f);
        }
        const sw::f32 cosPitch = std::cos(m_hangarPitch);
        const sw::Vec3 offset{cosPitch * std::sin(m_hangarYaw) * m_hangarDistance,
                              std::sin(m_hangarPitch) * m_hangarDistance,
                              cosPitch * std::cos(m_hangarYaw) * m_hangarDistance};
        m_hangarCamera.setPosition(sw::WorldVec3(offset));
        const sw::Vec3 forward = glm::normalize(-offset);
        const sw::Vec3 right = glm::normalize(glm::cross(forward, sw::Vec3{0, 1, 0}));
        const sw::Vec3 up = glm::cross(right, forward);
        m_hangarCamera.setOrientation(glm::quat_cast(sw::Mat3{right, up, -forward}));
        m_hangarCamera.setAspectRatio(renderer().aspectRatio());

        // ---- the hand -----------------------------------------------------------
        if (m_heldDefinition != 0)
        {
            // Rotate the held part in 90-degree steps (blueprint axes):
            // W/S pitch (X), A/D yaw (Y), Q/E roll (Z, the stack axis).
            const struct
            {
                sw::KeyCode key;
                sw::Vec3 axis;
                sw::f32 angle;
            } rotations[] = {
                {sw::KeyCode::W, {1, 0, 0}, 1.5707963f},
                {sw::KeyCode::S, {1, 0, 0}, -1.5707963f},
                {sw::KeyCode::A, {0, 1, 0}, 1.5707963f},
                {sw::KeyCode::D, {0, 1, 0}, -1.5707963f},
                {sw::KeyCode::Q, {0, 0, 1}, 1.5707963f},
                {sw::KeyCode::E, {0, 0, 1}, -1.5707963f},
            };
            for (const auto& rotation : rotations)
            {
                if (input().wasKeyPressed(rotation.key))
                {
                    m_heldRotation =
                        glm::angleAxis(rotation.angle, rotation.axis) * m_heldRotation;
                }
            }
            if (input().wasKeyPressed(sw::KeyCode::Escape))
            {
                // Put a grabbed subtree back exactly where it was; a fresh
                // palette part simply vanishes.
                if (!m_blueprintBackup.empty())
                {
                    m_blueprint = m_blueprintBackup;
                }
                m_heldDefinition = 0;
                m_heldSubtree.clear();
                m_blueprintBackup.clear();
            }
            if (input().wasKeyPressed(sw::KeyCode::Delete))
            {
                m_heldDefinition = 0; // discard (grab included: backup dropped)
                m_heldSubtree.clear();
                m_blueprintBackup.clear();
            }
        }
        if (input().wasKeyPressed(sw::KeyCode::X))
        {
            for (sw::usize i = 0; i < 6; ++i)
            {
                if (kSymmetryOptions[i] == m_symmetryCount)
                {
                    m_symmetryCount = kSymmetryOptions[(i + 1) % 6];
                    break;
                }
            }
        }
        if (input().wasKeyPressed(sw::KeyCode::C))
        {
            m_showCenters = !m_showCenters;
        }

        computeGhost();
    }

    void StarWorksGame::collectHangarItems()
    {
        m_drawItems.clear();
        const sw::WorldVec3 cameraPosition = m_hangarCamera.position();
        const sw::Quat display = kHangarDisplay;
        auto toWorld = [&](const sw::Vec3& local) {
            return sw::WorldVec3(display * local);
        };
        auto pushPart = [&](sw::u32 definitionId, const sw::Vec3& position,
                            const sw::Quat& rotation, const sw::Vec4& tint,
                            bool transparent) {
            const auto* definition = sw::parts::findDefinition(definitionId);
            const auto meshIt = m_partMeshIds.find(definitionId);
            if (definition == nullptr || meshIt == m_partMeshIds.end())
            {
                return;
            }
            const sw::Vec3 relative = sw::Vec3(toWorld(position) - cameraPosition);
            sw::DrawItem item{};
            item.mesh = &m_meshes[meshIt->second];
            item.transform = glm::translate(sw::Mat4{1.0f}, relative) *
                             glm::mat4_cast(display * rotation);
            item.boundsCenter = relative;
            item.boundsRadius = sw::parts::partBoundsRadius(*definition) + 0.5f;
            item.tint = tint;
            item.transparent = transparent;
            m_drawItems.push_back(item);
        };

        // Floor grid: the hangar deck.
        {
            sw::DrawItem floor{};
            floor.mesh = &m_meshes[m_hangarFloorMeshIndex];
            floor.transform = glm::translate(
                sw::Mat4{1.0f}, sw::Vec3(sw::WorldVec3{0.0, -14.0, 0.0} - cameraPosition));
            floor.boundsCenter = sw::Vec3(sw::WorldVec3{0.0, -14.0, 0.0} - cameraPosition);
            floor.boundsRadius = 60.0f;
            m_drawItems.push_back(floor);
        }

        // Placed parts.
        for (const BlueprintPart& bp : m_blueprint)
        {
            pushPart(bp.definitionId, bp.localPosition, bp.localRotation,
                     {1.0f, 1.0f, 1.0f, 1.0f}, false);
        }

        // Open STACK nodes: cyan diamonds (the magnet targets).
        for (const OpenAttachPoint& node : openAttachPoints())
        {
            const sw::Vec3 relative =
                sw::Vec3(toWorld(node.vesselPosition) - cameraPosition);
            sw::DrawItem marker{};
            marker.mesh = &m_meshes[m_markerMeshIndex];
            marker.transform = glm::translate(sw::Mat4{1.0f}, relative) *
                               glm::scale(sw::Mat4{1.0f}, sw::Vec3{0.30f});
            marker.boundsCenter = relative;
            marker.boundsRadius = 0.4f;
            marker.tint = {0.3f, 0.9f, 1.0f, 2.0f};
            m_drawItems.push_back(marker);
        }

        // The hand: ghost(s) of the held part (+ subtree, + symmetry clones).
        if (m_heldDefinition != 0)
        {
            const sw::Vec4 ghostTint =
                !m_ghost.active ? sw::Vec4{0.75f, 0.8f, 0.9f, 0.4f}
                : m_ghost.valid ? sw::Vec4{0.35f, 1.0f, 0.45f, 0.45f}
                                : sw::Vec4{1.0f, 0.25f, 0.2f, 0.5f};
            const bool surface = m_ghost.active && m_ghost.parentPoint == 255;
            const sw::u32 cloneCount =
                (surface && m_heldSubtree.empty()) ? m_symmetryCount : 1;
            for (sw::u32 k = 0; k < cloneCount; ++k)
            {
                const sw::f32 angle =
                    2.0f * 3.14159265f * static_cast<sw::f32>(k) / cloneCount;
                const sw::Quat spin = glm::angleAxis(angle, sw::Vec3{0, 0, 1});
                pushPart(m_heldDefinition, spin * m_ghost.position,
                         spin * m_ghost.rotation, ghostTint, true);
                for (const BlueprintPart& rel : m_heldSubtree)
                {
                    pushPart(rel.definitionId,
                             spin * (m_ghost.position +
                                     m_ghost.rotation * rel.localPosition),
                             spin * (m_ghost.rotation * rel.localRotation), ghostTint,
                             true);
                }
            }
        }

        // Center of mass (yellow) and thrust centroid (violet, engines).
        if (m_showCenters && !m_blueprint.empty())
        {
            sw::f64 totalMass = 0.0;
            sw::Vec3 massMoment{0.0f};
            sw::f64 totalThrust = 0.0;
            sw::Vec3 thrustMoment{0.0f};
            for (const BlueprintPart& bp : m_blueprint)
            {
                const sw::f64 mass = partWetMassKg(bp.definitionId);
                totalMass += mass;
                massMoment += bp.localPosition * static_cast<sw::f32>(mass);
                const auto* definition = sw::parts::findDefinition(bp.definitionId);
                if (definition != nullptr && definition->thrustNewtons > 0.0)
                {
                    totalThrust += definition->thrustNewtons;
                    thrustMoment += bp.localPosition *
                                    static_cast<sw::f32>(definition->thrustNewtons);
                }
            }
            // Flags OUTSIDE the hull (a marker inside a tank would be depth-
            // hidden): diamond at x = sideX, thin pointer line toward the axis.
            auto pushCenter = [&](const sw::Vec3& position, const sw::Vec4& color,
                                  sw::f32 scale, sw::f32 sideX) {
                const sw::Vec3 flag{sideX, position.y, position.z};
                const sw::Vec3 relative = sw::Vec3(toWorld(flag) - cameraPosition);
                sw::DrawItem marker{};
                marker.mesh = &m_meshes[m_markerMeshIndex];
                marker.transform = glm::translate(sw::Mat4{1.0f}, relative) *
                                   glm::scale(sw::Mat4{1.0f}, sw::Vec3{scale});
                marker.boundsCenter = relative;
                marker.boundsRadius = scale * 1.5f;
                marker.tint = color;
                m_drawItems.push_back(marker);
                // Pointer line from the flag toward the exact point.
                const sw::Vec3 lineCenter = (flag + position) * 0.5f;
                const sw::Vec3 lineRelative =
                    sw::Vec3(toWorld(lineCenter) - cameraPosition);
                const sw::f32 halfLength = glm::length(flag - position) * 0.5f;
                sw::DrawItem line{};
                line.mesh = &m_meshes[m_navLineMeshIndex];
                line.transform =
                    glm::translate(sw::Mat4{1.0f}, lineRelative) *
                    glm::mat4_cast(display) *
                    glm::scale(sw::Mat4{1.0f},
                               sw::Vec3{std::max(halfLength, 0.1f), 0.03f, 0.03f});
                line.boundsCenter = lineRelative;
                line.boundsRadius = halfLength + 0.2f;
                line.tint = color * sw::Vec4{1.0f, 1.0f, 1.0f, 0.4f};
                line.transparent = true;
                m_drawItems.push_back(line);
            };
            if (totalMass > 0.0)
            {
                pushCenter(massMoment / static_cast<sw::f32>(totalMass),
                           {1.0f, 0.85f, 0.2f, 2.0f}, 0.5f, 4.2f);
            }
            if (totalThrust > 0.0)
            {
                pushCenter(thrustMoment / static_cast<sw::f32>(totalThrust),
                           {0.8f, 0.4f, 1.0f, 2.0f}, 0.42f, 5.0f);
            }
        }

        collectEditorUi();
    }

    // ---- hangar UI: title, clickable part palette, action row, stats ------
    void StarWorksGame::collectEditorUi()
    {
        // The hangar owns the whole button set for the frame (the SAS row
        // is a flight instrument and stays out of the hangar).
        m_hudButtons.clear();

        const sw::Vec4 titleColor{1.0f, 0.85f, 0.35f, 1.0f};
        const sw::Vec4 textColor{0.8f, 0.9f, 1.0f, 0.95f};
        hudText("HANGAR", -0.97f, -0.96f, 0.048f, titleColor);
        hudText(m_hangarSource.isNull() ? "MODE: NEW BUILD -> LAUNCH PAD"
                                        : "MODE: MODIFYING LOADED VESSEL",
                -0.97f, -0.885f, 0.036f, textColor);
        if (m_heldDefinition != 0)
        {
            const auto* held = sw::parts::findDefinition(m_heldDefinition);
            hudText(std::format("IN HAND: {}{}", held != nullptr ? held->name : "?",
                                m_heldSubtree.empty()
                                    ? ""
                                    : std::format(" +{} PARTS", m_heldSubtree.size())),
                    -0.97f, -0.825f, 0.032f, {0.6f, 1.0f, 0.7f, 1.0f});
            hudText("LCLICK PLACE  W/S/A/D/Q/E ROTATE  ESC PUT BACK  DEL DISCARD",
                    -0.97f, -0.77f, 0.026f, sw::Vec4{0.6f, 0.72f, 0.82f, 0.85f});
        }
        else
        {
            hudText("CLICK THE PALETTE FOR A NEW PART - CLICK A PLACED PART TO "
                    "GRAB ITS SUBTREE",
                    -0.97f, -0.825f, 0.026f, sw::Vec4{0.6f, 0.72f, 0.82f, 0.85f});
            hudText("B = EXIT WITHOUT BUILDING   X = SYMMETRY   C = CENTERS",
                    -0.97f, -0.775f, 0.026f, sw::Vec4{0.6f, 0.72f, 0.82f, 0.85f});
        }

        auto panel = [&](sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1,
                         const sw::Vec4& color) {
            sw::DrawItem item{};
            item.mesh = &m_meshes[m_navLineMeshIndex]; // unit quad
            item.transform =
                glm::translate(sw::Mat4{1.0f},
                               {(x0 + x1) * 0.5f, (y0 + y1) * 0.5f, 0.0f}) *
                glm::scale(sw::Mat4{1.0f},
                           {(x1 - x0) * 0.5f, (y1 - y0) * 0.5f, 1.0f});
            item.screenSpace = true;
            item.tint = color;
            m_drawItems.push_back(item);
        };

        // ---- part palette: one clickable row per ROCKET catalog entry ------
        // Buildings share the catalogue since F1; they are placed on the
        // ground, not stacked in the VAB, so they are filtered out here.
        const auto partCatalog = rocketPartPalette();
        constexpr sw::f32 kRowStride = 0.082f;
        constexpr sw::f32 kRowHeight = 0.068f;
        constexpr sw::f32 kRowWidth = 0.40f;
        sw::f32 rowY = -0.70f;
        for (sw::usize i = 0; i < partCatalog.size() && i < 14; ++i)
        {
            const bool held = partCatalog[i]->id == m_heldDefinition;
            panel(-0.98f, rowY, -0.98f + kRowWidth, rowY + kRowHeight,
                  held ? sw::Vec4{0.20f, 0.52f, 0.30f, 0.85f}
                       : sw::Vec4{0.13f, 0.19f, 0.28f, 0.60f});
            hudText(partCatalog[i]->name, -0.962f, rowY + 0.017f, 0.030f,
                    held ? sw::Vec4{0.9f, 1.0f, 0.9f, 1.0f}
                         : sw::Vec4{0.68f, 0.78f, 0.88f, 0.9f});
            m_hudButtons.push_back({-0.98f, rowY, -0.98f + kRowWidth,
                                    rowY + kRowHeight,
                                    100u + static_cast<sw::u32>(i)});
            rowY += kRowStride;
        }

        // ---- action row (bottom-center) -------------------------------------
        struct Action
        {
            const char* label;
            sw::u32 id;
            bool strong;
        };
        const std::string symLabel = std::format("SYM {}", m_symmetryCount);
        const Action actions[] = {
            {"UNDO", 201, false},          {"NEW", 202, false},
            {"LOAD", 203, false},          {symLabel.c_str(), 205, m_symmetryCount > 1},
            {m_showCenters ? "CG:ON" : "CG:OFF", 206, m_showCenters},
            // SAVE is the ONLY thing this room does to the world, and it
            // does it once, on this press — not on every part placed. What
            // it produces is a DESIGN: a `.swship` on disk, registered so a
            // VAB can be told to build it. The hangar itself has not made a
            // rocket since F9; the button that used to is gone, because a
            // drawing office that can also manufacture makes the factory
            // beside it decorative.
            {"SAVE", 207, true},
        };
        constexpr sw::f32 kButtonWidth = 0.135f;
        constexpr sw::f32 kButtonHeight = 0.072f;
        constexpr sw::f32 kButtonGap = 0.016f;
        sw::f32 buttonX = -0.54f;
        const sw::f32 buttonY = 0.86f;
        for (const Action& action : actions)
        {
            const sw::f32 x1 = buttonX + kButtonWidth;
            panel(buttonX, buttonY, x1, buttonY + kButtonHeight,
                  action.strong ? sw::Vec4{0.60f, 0.38f, 0.10f, 0.9f}
                                : sw::Vec4{0.16f, 0.24f, 0.34f, 0.75f});
            hudText(action.label, buttonX + 0.016f, buttonY + 0.019f, 0.032f,
                    sw::Vec4{0.92f, 0.96f, 1.0f, 1.0f});
            m_hudButtons.push_back({buttonX, buttonY, x1,
                                    buttonY + kButtonHeight, action.id});
            buttonX = x1 + kButtonGap;
        }

        // ---- blueprint stats (top-right) -------------------------------------
        sw::f64 wetMassKg = 0.0;
        sw::f64 costCredits = 0.0;
        for (const BlueprintPart& blueprintPart : m_blueprint)
        {
            wetMassKg += partWetMassKg(blueprintPart.definitionId);
            const auto* definition =
                sw::parts::findDefinition(blueprintPart.definitionId);
            costCredits += definition != nullptr ? definition->costCredits : 0.0;
        }
        hudText(std::format("WET MASS {:.1f} T  COST {:.0f}  PARTS {}",
                            wetMassKg / 1000.0, costCredits, m_blueprint.size()),
                0.16f, -0.95f, 0.034f, textColor);
    }

    // ========================================================================
    // MULTIPLAYER
    // ========================================================================

    bool StarWorksGame::keyPressed(sw::KeyCode key)
    {
        return !m_netAddressFocused && input().wasKeyPressed(key);
    }

    void StarWorksGame::refreshFlightState()
    {
        m_flight = FlightState{};

        const sw::ecs::Entity flown = controlledEntity();
        if (flown.isNull() || !m_world.isAlive(flown))
        {
            return;
        }

        // STANDING ON SOMETHING. Two spellings of the same fact: a live body
        // resting on terrain, or — once rails warp has already converted it —
        // a surface anchor, which has no DynamicBodyComponent at all. Testing
        // only the first would make the gate slam shut the instant the warp
        // it permitted took effect.
        if (const auto* body = m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(flown))
        {
            m_flight.grounded = body->isGrounded != 0;
        }
        if (m_world.tryGetComponent<sw::phys::SurfaceAnchorComponent>(flown) != nullptr)
        {
            m_flight.grounded = true;
        }

        m_flight.primaryIndex = controlledPrimaryIndex();
        if (m_flight.primaryIndex < 0)
        {
            return;
        }

        const auto& primary =
            m_celestialIndex.body(static_cast<sw::usize>(m_flight.primaryIndex));
        const sw::f64 time = m_physicsLane->presentSeconds();
        sw::WorldVec3 primaryPosition{0.0};
        sw::WorldVec3 primaryVelocity{0.0};
        m_celestialIndex.stateAt(m_flight.primaryIndex, time, primaryPosition,
                                 &primaryVelocity);

        const sw::WorldVec3 position =
            m_world.getComponent<TransformComponent>(flown).position;
        m_flight.altitude = glm::length(position - primaryPosition) - primary.bodyRadius;

        if (const auto* air =
                m_world.tryGetComponent<sw::phys::AtmosphereComponent>(primary.entity))
        {
            m_flight.atmosphereTop = air->topAltitude;
        }

        if (m_flight.grounded)
        {
            // LEANING, AND WHETHER THAT IS A PROBLEM. Measured against the
            // same statics the ground contact uses, so the readout and the
            // physics can never disagree.
            const sw::Vec3 up = glm::normalize(sw::Vec3(position - primaryPosition));
            const auto& transform = m_world.getComponent<TransformComponent>(flown);
            const sw::Vec3 nose = transform.rotation * sw::Vec3{0.0f, 0.0f, -1.0f};
            m_flight.leanDegrees = static_cast<sw::f32>(
                std::acos(std::clamp(static_cast<sw::f64>(glm::dot(nose, up)), -1.0, 1.0)) *
                180.0 / 3.14159265358979);

            const auto* hull = m_world.tryGetComponent<sw::phys::GroundHullComponent>(flown);
            const auto* vessel = m_world.tryGetComponent<sw::parts::VesselComponent>(flown);
            const auto* body = m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(flown);
            if (hull != nullptr && vessel != nullptr && body != nullptr)
            {
                const sw::Vec3 topple = sw::phys::topplingAcceleration(
                    *hull, transform.rotation, up, vessel->centreOfMass,
                    vessel->inertiaKgM2, body->mass,
                    primary.mu / (primary.bodyRadius * primary.bodyRadius));
                m_flight.tipping = glm::length(topple) > 1.0e-6f;
            }
            return; // no orbit to speak of, and none needed
        }

        sw::phys::KeplerOrbit orbit{};
        if (!sw::phys::kepler::fromStateVectors(primary.mu, position - primaryPosition,
                                                controlledVelocity() - primaryVelocity,
                                                time, orbit, /*allowHyperbolic=*/true))
        {
            return;
        }
        m_flight.periapsisAltitude = sw::phys::kepler::periapsis(orbit) - primary.bodyRadius;
        m_flight.closedOrbit = !orbit.isHyperbolic();
        if (m_flight.closedOrbit)
        {
            m_flight.apoapsisAltitude =
                sw::phys::kepler::apoapsis(orbit) - primary.bodyRadius;
        }
    }

    bool StarWorksGame::warpAllowed() const
    {
        // The rule itself lives in the engine (phys::warpPermitted) so it can
        // be tested without a window; this only feeds it this frame's state.
        return sw::phys::warpPermitted(m_flight.grounded, m_flight.closedOrbit,
                                       m_flight.periapsisAltitude,
                                       m_flight.atmosphereTop);
    }

    const char* StarWorksGame::warpBlockReason() const
    {
        if (warpAllowed())
        {
            return "";
        }
        if (!m_flight.closedOrbit)
        {
            return "NO CLOSED ORBIT";
        }
        return "PERIAPSIS IN AIR";
    }

    std::vector<sw::net::PlayerView> StarWorksGame::netRoster() const
    {
        if (m_netHost != nullptr)
        {
            return m_netHost->roster();
        }
        if (m_netClient != nullptr)
        {
            return m_netClient->roster();
        }
        return {};
    }

    void StarWorksGame::netHost()
    {
        netLeave();
        try
        {
            sw::net::ReplicationSet set;
            set.include("sw.Transform")
                .include("phys.DynamicBody")
                .include("phys.OnRails")
                .include("parts.Part")
                .include("parts.Vessel")
                .include("game.Ship");

            sw::net::PeerAddress bind{};
            const bool parsed = sw::net::PeerAddress::parse(m_netAddress, bind);
            const sw::u16 port = parsed && bind.port != 0 ? bind.port : sw::u16{7777};

            sw::net::Host::Config config;
            config.hostName = "host";
            m_netHost = std::make_unique<sw::net::Host>(
                std::make_unique<sw::net::UdpSocket>(port), m_saveSchema, set, config);
            // Show the address the OTHER machine has to type, not the port
            // on its own. Guessing your own LAN address out of ipconfig is
            // the first thing that goes wrong, and the socket already knows.
            const sw::net::PeerAddress lan = sw::net::localAddress(m_netHost->port());
            m_netStatus = std::format("HOSTING ON {}", lan.toString());
            SW_LOG_INFO("Game", "Multiplayer: hosting on UDP port {} — others join with {}",
                        m_netHost->port(), lan.toString());

            // THE FIREWALL, HERE AND NOWHERE ELSE. Hosting is the only thing
            // in the game that needs an unsolicited inbound datagram, and
            // pressing HOST is the only moment at which asking for it is
            // both expected and explicable. The call is a no-op — no prompt,
            // no process — when the rule already exists, so a player who
            // accepted once never sees it again.
            const sw::platform::FirewallRequest firewall = sw::platform::allowInboundUdp();
            m_netFirewall = firewall;
            m_netPublicNetwork = sw::platform::onPublicNetwork();
            if (m_netPublicNetwork)
            {
                SW_LOG_WARN("Game",
                            "Multiplayer: this machine is on a PUBLIC network profile. The "
                            "inbound rule covers Private and Domain only, so it is inert: "
                            "Windows will drop the connection attempt however many rules "
                            "exist. Set-NetConnectionProfile -NetworkCategory Private.");
            }
            switch (firewall)
            {
                case sw::platform::FirewallRequest::Added:
                    SW_LOG_INFO("Game", "Multiplayer: firewall rule added for this executable");
                    break;
                case sw::platform::FirewallRequest::Declined:
                    // Hosting continues. Refusing administrator rights is a
                    // legitimate answer, the session is perfectly playable
                    // from this machine, and someone on the same network may
                    // still get through if a rule exists by another route.
                    // The verdict goes on its OWN line in the panel, not on
                    // the end of the status: measured, "HOSTING ON
                    // 192.168.1.61:7777 - FIREWALL REFUSED" is 0.725 NDC
                    // wide against 0.494 of usable width.
                    SW_LOG_WARN("Game",
                                "Multiplayer: administrator rights refused, so no inbound rule "
                                "was added. Players on other machines will most likely see "
                                "NO REPLY. Run firewall.ps1 as administrator to add it later.");
                    break;
                case sw::platform::FirewallRequest::Failed:
                    SW_LOG_ERROR("Game",
                                 "Multiplayer: the inbound rule could not be added. "
                                 "Run firewall.ps1 as administrator.");
                    break;
                case sw::platform::FirewallRequest::AlreadyAllowed:
                case sw::platform::FirewallRequest::Unsupported:
                    break;
            }
        }
        catch (const sw::Exception& e)
        {
            m_netHost.reset();
            m_netStatus = "HOST FAILED";
            SW_LOG_ERROR("Game", "Could not host: {}", e.what());
        }
    }

    void StarWorksGame::netJoin()
    {
        netLeave();
        sw::net::PeerAddress address{};
        if (!sw::net::PeerAddress::parse(m_netAddress, address) || address.port == 0)
        {
            m_netStatus = "BAD ADDRESS";
            return;
        }
        try
        {
            m_netMirror.clearForRestore();
            m_netClient = std::make_unique<sw::net::Client>(
                std::make_unique<sw::net::UdpSocket>(0), m_saveSchema, "pilot");
            m_netClient->connect(address, clock().totalSeconds());
            m_netStatus = std::format("JOINING {}", address.toString());
            SW_LOG_INFO("Game", "Multiplayer: {}", m_netStatus);
        }
        catch (const sw::Exception& e)
        {
            m_netClient.reset();
            m_netStatus = "JOIN FAILED";
            SW_LOG_ERROR("Game", "Could not join: {}", e.what());
        }
    }

    void StarWorksGame::netLeave()
    {
        const sw::f64 now = clock().totalSeconds();
        if (m_netClient != nullptr)
        {
            m_netClient->disconnect(now);
            m_netClient.reset();
        }
        if (m_netHost != nullptr)
        {
            m_netHost->shutdown(now);
            m_netHost.reset();
        }
        m_syncWarpTo = 0.0;
        m_syncWarpPlayer = 0;
        m_netStatus.clear();
        m_netTimeoutLogged = false;
        m_netFirewall = sw::platform::FirewallRequest::Unsupported;
        m_netPublicNetwork = false;
    }

    void StarWorksGame::netSyncTo(sw::u32 playerId, sw::f64 targetSeconds)
    {
        // The catch-up warp is still a warp: it puts the world on rails and
        // stops integrating, so the same rule applies. What it does bypass is
        // the ALTITUDE ladder — a player parked in a 200 km orbit would
        // otherwise be capped at x100 and never close a three-hour gap.
        if (!warpAllowed())
        {
            m_netStatus = std::format("CANNOT SYNC - {}", warpBlockReason());
            return;
        }
        if (targetSeconds <= m_physicsLane->presentSeconds())
        {
            m_netStatus = "ALREADY THERE";
            return;
        }
        m_syncWarpTo = targetSeconds;
        m_syncWarpPlayer = playerId;
        m_warpToSeconds = 0.0; // one destination at a time
        m_simulation.setPaused(false);
        SW_LOG_INFO("Game", "Sync warp to player {} at t={:.0f}", playerId, targetSeconds);
    }

    void StarWorksGame::updateTextField()
    {
        if (!m_netAddressFocused)
        {
            return;
        }
        // Only what an address is made of. The HUD font has no lowercase and
        // no underscore, so a hostname could not be displayed even if it
        // could be typed; digits, dots and a colon are the whole alphabet.
        for (const sw::u32 codepoint : input().charsTyped())
        {
            if (m_netAddress.size() >= 21)
            {
                break;
            }
            if ((codepoint >= '0' && codepoint <= '9') || codepoint == '.' ||
                codepoint == ':')
            {
                m_netAddress.push_back(static_cast<char>(codepoint));
            }
        }
        if (input().wasKeyPressed(sw::KeyCode::Backspace) && !m_netAddress.empty())
        {
            m_netAddress.pop_back();
        }
        if (input().wasKeyPressed(sw::KeyCode::Enter) ||
            input().wasKeyPressed(sw::KeyCode::Escape))
        {
            m_netAddressFocused = false;
        }
    }

    void StarWorksGame::updateNetwork(sw::f32 deltaSeconds)
    {
        (void)deltaSeconds;
        if (!netActive())
        {
            return;
        }

        const sw::f64 wall = clock().totalSeconds();
        const sw::f64 simulated = m_physicsLane->presentSeconds();

        if (m_netHost != nullptr)
        {
            m_netHost->update(wall, m_world, simulated);
        }
        if (m_netClient != nullptr)
        {
            m_netClient->update(wall, m_netMirror, simulated);
            if (m_netClient->state() == sw::net::ClientState::Connected)
            {
                m_netStatus = std::format("JOINED AS {}", m_netClient->clientId());
            }
            else if (m_netClient->state() == sw::net::ClientState::Rejected)
            {
                m_netStatus = "REJECTED";
            }
            else if (m_netClient->state() == sw::net::ClientState::TimedOut)
            {
                // A timeout has two completely different causes and the cure
                // for one is useless against the other, so say WHICH. If not
                // one datagram ever came back, nothing on the far side ever
                // answered: the packets die before the host's process sees
                // them — a firewall, a wrong address, or nobody hosting. If
                // some arrived and then stopped, the link was alive and died.
                const bool everHeard = m_netClient->stats().datagramsReceived > 0;
                m_netStatus = everHeard ? "TIMED OUT - LINK LOST" : "NO REPLY - CHECK FIREWALL";
                if (!m_netTimeoutLogged)
                {
                    m_netTimeoutLogged = true;
                    if (everHeard)
                    {
                        SW_LOG_WARN("Game",
                                    "Multiplayer: the host stopped answering after {} datagrams",
                                    m_netClient->stats().datagramsReceived);
                    }
                    else
                    {
                        SW_LOG_WARN(
                            "Game",
                            "Multiplayer: sent {} datagrams to {} and got NOTHING back. Nothing "
                            "at that address is answering: the host is not listening, the "
                            "address is wrong, or inbound UDP is blocked. On the HOSTING "
                            "machine run firewall.ps1 as administrator, and check that its "
                            "network profile is Private and not Public.",
                            m_netClient->stats().datagramsSent, m_netAddress);
                    }
                }
            }
        }

        // A beacon: where this player's craft is, stamped with the instant it
        // was there. Peers behind us hold it until their own clock reaches
        // that instant — which is the whole rule, exercised on real traffic
        // rather than only in a test.
        if (m_netLastBeaconAt < 0.0 || wall - m_netLastBeaconAt >= 0.5)
        {
            m_netLastBeaconAt = wall;
            const sw::ecs::Entity flown = controlledEntity();
            if (!flown.isNull() && m_world.isAlive(flown))
            {
                const sw::WorldVec3 position =
                    m_world.getComponent<TransformComponent>(flown).position;
                sw::ser::BinaryWriter writer;
                writer.write(position.x);
                writer.write(position.y);
                writer.write(position.z);
                if (m_netHost != nullptr)
                {
                    m_netHost->broadcastEvent(simulated, kNetEventBeacon, writer.bytes());
                }
                else if (m_netClient != nullptr)
                {
                    m_netClient->sendEvent(simulated, kNetEventBeacon, writer.bytes());
                }
            }
        }

        // Release whatever the timeline says is due AT OUR CLOCK.
        sw::net::Timeline* timeline = (m_netHost != nullptr) ? &m_netHost->timeline()
                                                             : &m_netClient->timeline();
        m_netEventsApplied += timeline->advance(simulated).size();
    }

    void StarWorksGame::collectNetPanel()
    {
        constexpr sw::f32 kRight = 0.97f;
        constexpr sw::f32 kLeft = 0.44f;
        constexpr sw::f32 kTop = -0.95f;
        constexpr sw::f32 kPad = 0.018f;
        constexpr sw::f32 kHeaderH = 0.086f;
        constexpr sw::f32 kRowH = 0.062f;
        constexpr sw::f32 kGap = 0.008f;

        sw::f32 cursorX = -2.0f;
        sw::f32 cursorY = -2.0f;
        const bool haveCursor = hudCursor(cursorX, cursorY);
        auto hovering = [&](sw::f32 x0, sw::f32 x1, sw::f32 y) {
            return haveCursor && cursorX >= x0 && cursorX <= x1 && cursorY >= y &&
                   cursorY <= y + kRowH;
        };

        const std::vector<sw::net::PlayerView> roster = netRoster();
        const sw::u32 selfId =
            (m_netClient != nullptr) ? m_netClient->clientId() : sw::u32{0};
        const sw::f64 selfClock = m_physicsLane->presentSeconds();

        // Height: address, buttons, status, the PILOTS label, one row per
        // player, and the footer verdict — five fixed rows plus the roster.
        // Counting four put the footer outside the panel, which a mock of the
        // layout showed before the code ever ran.
        // Plus one row while hosting, for what has actually reached the
        // socket. That row is the only place either machine can see the
        // difference between "the packets never arrived" and "the packets
        // arrived and were thrown away".
        const sw::f32 bodyRows = 5.0f + (m_netHost != nullptr ? 1.0f : 0.0f) +
                                 static_cast<sw::f32>(roster.size());
        const sw::f32 bottom =
            kTop + kHeaderH + bodyRows * (kRowH + kGap) + kPad * 2.0f;

        hudPanel(kLeft, kTop, kRight, bottom, hud::kPanel);
        hudQuad(kLeft, kTop, kRight, kTop + kHeaderH - 0.006f, hud::kHeader);
        hudText("MULTIPLAYER", kLeft + 0.022f, kTop + 0.026f, 0.048f, hud::kTitle);
        hudText("F3", kRight - 0.075f, kTop + 0.030f, 0.036f, hud::kTextDim);

        sw::f32 y = kTop + kHeaderH + kPad;

        // ---- the address, typed -------------------------------------------
        {
            const bool hot = hovering(kLeft + kPad, kRight - kPad, y);
            const sw::Vec4 fill = m_netAddressFocused ? hud::kRowOn
                                  : hot                ? hud::kRowHover
                                                       : hud::kRow;
            hudQuad(kLeft + kPad, y, kRight - kPad, y + kRowH, fill);
            // A caret rather than a cursor: the field only ever appends or
            // deletes at the end, so there is nothing to move.
            const std::string shown =
                m_netAddressFocused ? m_netAddress + "-" : m_netAddress;
            hudText(shown, kLeft + kPad + 0.014f, y + 0.014f, 0.034f, hud::kText);
            m_hudButtons.push_back({kLeft + kPad, y, kRight - kPad, y + kRowH, 1003u});
            y += kRowH + kGap;
        }

        // ---- host / join / leave -------------------------------------------
        {
            const sw::f32 mid = (kLeft + kRight) * 0.5f;
            if (!netActive())
            {
                const bool hotHost = hovering(kLeft + kPad, mid - 0.004f, y);
                hudQuad(kLeft + kPad, y, mid - 0.004f, y + kRowH,
                        hotHost ? hud::kRowHover : hud::kRow);
                hudText("HOST", kLeft + kPad + 0.030f, y + 0.014f, 0.034f, hud::kText);
                m_hudButtons.push_back({kLeft + kPad, y, mid - 0.004f, y + kRowH, 1000u});

                const bool hotJoin = hovering(mid + 0.004f, kRight - kPad, y);
                hudQuad(mid + 0.004f, y, kRight - kPad, y + kRowH,
                        hotJoin ? hud::kRowHover : hud::kRow);
                hudText("JOIN", mid + 0.034f, y + 0.014f, 0.034f, hud::kText);
                m_hudButtons.push_back({mid + 0.004f, y, kRight - kPad, y + kRowH, 1001u});
            }
            else
            {
                const bool hot = hovering(kLeft + kPad, kRight - kPad, y);
                hudQuad(kLeft + kPad, y, kRight - kPad, y + kRowH,
                        hot ? hud::kRowHover : hud::kRowStop);
                hudText("LEAVE SESSION", kLeft + kPad + 0.030f, y + 0.014f, 0.034f,
                        hud::kText);
                m_hudButtons.push_back({kLeft + kPad, y, kRight - kPad, y + kRowH, 1002u});
            }
            y += kRowH + kGap;
        }

        // ---- status ---------------------------------------------------------
        hudText(m_netStatus.empty() ? "OFFLINE" : hud::caps(m_netStatus),
                kLeft + kPad, y + 0.012f, 0.032f, hud::kTextDim);
        // The firewall verdict rides in the same row's spare height rather
        // than on the end of the status line: the panel is 0.494 NDC wide
        // and the concatenated string measured 0.725. Only shown when there
        // is something to act on — a rule that already exists is not news.
        if (m_netHost != nullptr)
        {
            if (m_netFirewall == sw::platform::FirewallRequest::Declined)
            {
                hudText("FIREWALL: RIGHTS REFUSED", kLeft + kPad, y + 0.044f, 0.024f,
                        hud::kBad);
            }
            else if (m_netFirewall == sw::platform::FirewallRequest::Failed)
            {
                hudText("FIREWALL: RULE FAILED", kLeft + kPad, y + 0.044f, 0.024f, hud::kBad);
            }
            else if (m_netPublicNetwork)
            {
                // The rule covers Private and Domain. On a Public network it
                // exists, lists as present in every tool that shows rules,
                // and blocks the packet anyway — the single most confusing
                // state this whole feature can be in.
                hudText("NETWORK IS PUBLIC - RULE INACTIVE", kLeft + kPad, y + 0.044f, 0.024f,
                        hud::kBad);
            }
            else if (m_netFirewall == sw::platform::FirewallRequest::Added)
            {
                hudText("FIREWALL: RULE ADDED", kLeft + kPad, y + 0.044f, 0.024f, hud::kOk);
            }
        }
        y += kRowH + kGap;

        // ---- what has actually reached the socket ---------------------------
        // The host is the only machine that can tell the three causes of a
        // client-side timeout apart, and until now it never said.
        if (m_netHost != nullptr)
        {
            const sw::net::Host::Reception& rx = m_netHost->reception();
            // Clamped for display. At 20 snapshots a second these counters
            // reach seven digits in an hour, and the row is 34 characters
            // wide — the exact total stopped being the point after the
            // first one arrived anyway.
            auto shortCount = [](sw::u64 value) {
                return (value > 99999u) ? std::string("99999+") : std::format("{}", value);
            };
            hudText(std::format("RX {}  REFUSED {}", shortCount(rx.arrived),
                                shortCount(rx.refused)),
                    kLeft + kPad, y + 0.006f, 0.030f,
                    rx.arrived == 0 ? hud::kTextDim : hud::kOk);

            std::string verdict;
            sw::Vec4 colour = hud::kTextDim;
            if (rx.wrongVersion > 0)
            {
                verdict = std::format("PEER SPEAKS V{} - REBUILD IT", rx.lastForeignVersion);
                colour = hud::kBad;
            }
            else if (rx.arrived == 0)
            {
                verdict = "NOTHING HAS REACHED THIS PC";
                colour = hud::kWarn;
            }
            else if (rx.notOurs > 0 && rx.notOurs == rx.fromStrangers)
            {
                verdict = "TRAFFIC ARRIVES BUT IS NOT OURS";
                colour = hud::kWarn;
            }
            else
            {
                verdict = "PACKETS ARE GETTING THROUGH";
                colour = hud::kOk;
            }
            hudText(verdict, kLeft + kPad, y + 0.040f, 0.024f, colour);
            y += kRowH + kGap;
        }

        // ---- the players, and how far apart their clocks are ---------------
        hudText(std::format("PILOTS {}", roster.size()), kLeft + kPad, y + 0.012f, 0.032f,
                hud::kTextDim);
        y += kRowH + kGap;

        for (sw::usize i = 0; i < roster.size(); ++i)
        {
            const sw::net::PlayerView& player = roster[i];
            const bool self = (player.id == selfId);
            const sw::f64 offset = player.simulatedSeconds - selfClock;
            // Only a real gap is worth a button. Half a second is network
            // jitter, not a temporality.
            const bool ahead = !self && offset > 1.0;

            const sw::f32 syncX0 = kRight - kPad - 0.15f;
            const sw::f32 rowRight = ahead ? syncX0 - 0.008f : kRight - kPad;
            const bool hot = hovering(kLeft + kPad, rowRight, y);
            hudQuad(kLeft + kPad, y, rowRight, y + kRowH,
                    self ? hud::kRowOn : (hot ? hud::kRowHover
                                              : ((i % 2 == 0) ? hud::kRow : hud::kRowAlt)));
            hudText(hud::caps(player.name.empty() ? std::string("PILOT") : player.name),
                    kLeft + kPad + 0.016f, y + 0.008f, 0.032f, hud::kText);

            const std::string when =
                self ? std::string("NOW") : hud::signedDuration(offset);
            hudText(when, kLeft + kPad + 0.016f, y + 0.036f, 0.024f,
                    (std::abs(offset) < 1.0 || self) ? hud::kOk
                    : (offset > 0.0)                 ? hud::kWarn
                                                     : hud::kTextDim);

            if (ahead)
            {
                const bool hotSync = hovering(syncX0, kRight - kPad, y);
                const bool can = warpAllowed();
                hudQuad(syncX0, y, kRight - kPad, y + kRowH,
                        !can       ? sw::Vec4{0.12f, 0.14f, 0.18f, 0.90f}
                        : hotSync  ? hud::kRowOnHover
                                   : hud::kRowOn);
                hudText("SYNC", syncX0 + 0.028f, y + 0.020f, 0.030f,
                        can ? hud::kText : hud::kTextDim);
                if (can)
                {
                    m_hudButtons.push_back(
                        {syncX0, y, kRight - kPad, y + kRowH,
                         1100u + static_cast<sw::u32>(i)});
                }
            }
            y += kRowH + kGap;
        }

        // ---- footer: why you can or cannot warp ----------------------------
        if (m_syncWarpTo > 0.0)
        {
            const sw::f64 remaining = m_syncWarpTo - selfClock;
            hudText(std::format("SYNCING  T-{}", hud::signedDuration(remaining)),
                    kLeft + kPad, y + 0.010f, 0.032f, hud::kOk);
        }
        else if (warpAllowed())
        {
            hudText(m_flight.grounded ? "WARP READY - LANDED" : "WARP READY - ORBIT",
                    kLeft + kPad, y + 0.010f, 0.030f, hud::kOk);
        }
        else
        {
            hudText(std::format("WARP LOCKED - {}", warpBlockReason()), kLeft + kPad,
                    y + 0.010f, 0.030f, hud::kBad);
        }
    }

    void StarWorksGame::collectSasButtons()
    {
        // First clickable UI: three buttons above the bottom-left corner.
        m_hudButtons.clear();
        constexpr sw::f32 kHeight = 0.062f;
        constexpr sw::f32 kWidth = 0.115f;
        constexpr sw::f32 kGap = 0.018f;
        const sw::f32 y0 = 0.87f;
        sw::f32 x0 = -0.97f;

        // NODE is the fourth: a burn is almost never prograde — a plane
        // change is normal, a circularisation is prograde only by accident
        // — and flying one by eye means chasing a marker across the navball
        // with the throttle already open. It greys out with no node up.
        // SAS is a MODE now, not the absence of one: it holds the craft
        // still. Every button toggles — clicking the lit one switches the
        // autopilot off — so there is no longer a button whose only job is
        // to mean "none of the others".
        const char* labels[4] = {"SAS", "PGD", "RTG", "NODE"};
        const sw::u32 modes[4] = {SasComponent::kStability, SasComponent::kPrograde,
                                  SasComponent::kRetrograde, SasComponent::kNode};
        for (sw::u32 slot = 0; slot < 4; ++slot)
        {
            const sw::u32 id = modes[slot];
            const sw::f32 x1 = x0 + kWidth;
            const bool available = (id != SasComponent::kNode) || m_nodeActive;
            const bool active = (m_sasMode == id);
            const sw::Vec4 background =
                active      ? sw::Vec4{0.15f, 0.55f, 0.30f, 0.85f}
                : available ? sw::Vec4{0.16f, 0.22f, 0.30f, 0.65f}
                            : sw::Vec4{0.12f, 0.14f, 0.18f, 0.55f};

            sw::DrawItem panel{};
            panel.mesh = &m_meshes[m_navLineMeshIndex]; // unit quad
            panel.transform =
                glm::translate(sw::Mat4{1.0f},
                               {(x0 + x1) * 0.5f, y0 + kHeight * 0.5f, 0.0f}) *
                glm::scale(sw::Mat4{1.0f},
                           {kWidth * 0.5f, kHeight * 0.5f, 1.0f});
            panel.screenSpace = true;
            panel.tint = background;
            m_drawItems.push_back(panel);

            hudText(labels[slot], x0 + 0.022f, y0 + 0.015f, 0.036f,
                    active      ? sw::Vec4{0.9f, 1.0f, 0.9f, 1.0f}
                    : available ? sw::Vec4{0.7f, 0.8f, 0.9f, 0.9f}
                                : sw::Vec4{0.42f, 0.48f, 0.56f, 0.8f});

            if (available)
            {
                m_hudButtons.push_back({x0, y0, x1, y0 + kHeight, id});
            }
            x0 = x1 + kGap;
        }

        // WHICH PROGRADE. Under the row, because the two buttons above it
        // mean different directions depending on a toggle three metres away
        // on the other side of the screen — and on the way down they are
        // tens of degrees apart. `V` swaps it, the same key that swaps the
        // speed readout and the navball markers.
        hudText(std::format("{} FRAME (V)", m_speedSurfaceRelative ? "SRF" : "ORB"),
                -0.97f, y0 - 0.045f, 0.032f,
                m_speedSurfaceRelative ? sw::Vec4{0.55f, 0.85f, 0.65f, 0.9f}
                                       : sw::Vec4{0.6f, 0.72f, 0.9f, 0.9f});
    }


    // ------------------------------------------------------------------------
    // PLACING A BUILDING
    //
    // ONE function. The scene builder lays the starting outpost with it and
    // the player's build cursor commits with it, so a machine you put down
    // and a machine the game put down are the same object, made the same
    // way — there is no "scripted" variant that quietly differs.
    //
    // `direction` is a UNIT vector in the body's rotating frame; `yaw` spins
    // the building about its own local vertical. Everything else — the
    // footprint, the power, the storage, the recipes it may run — is read
    // from the .swpart.
    // ------------------------------------------------------------------------
    sw::ecs::Entity StarWorksGame::placeBuilding(sw::u32 definitionId,
                                                 sw::ecs::Entity body,
                                                 const sw::Vec3& direction,
                                                 sw::f32 yawRadians, sw::u32 recipeId,
                                                 sw::ecs::Entity site,
                                                 const sw::Vec4& marker)
    {
        const sw::parts::PartDefinition* definition =
            sw::parts::findDefinition(definitionId);
        if (definition == nullptr || !sw::parts::isBuilding(*definition))
        {
            SW_LOG_WARN("Game", "Building definition {} missing from the catalog",
                        definitionId);
            return sw::ecs::Entity::null();
        }
        const auto* terrain = m_world.tryGetComponent<sw::planet::TerrainComponent>(body);
        const auto* gravity =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(body);
        if (terrain == nullptr || gravity == nullptr)
        {
            SW_LOG_WARN("Game", "Cannot build on a body with no ground");
            return sw::ecs::Entity::null();
        }
        const auto* deposits = m_world.tryGetComponent<sw::planet::DepositComponent>(body);

        const sw::parts::BuildingSpec& spec = definition->building;
        const sw::Vec3 up = glm::normalize(direction);
        const sw::f64 elevation = sw::planet::terrainElevation(*terrain, up);

        // Stand the model upright: its +Y onto the local vertical, then the
        // requested yaw about that same axis (applied in MODEL space, which
        // is why it is a spin on the spot and not a tilt).
        const sw::Quat standUp = standUpFor(up);

        const sw::ecs::Entity e = m_world.createEntity();
        m_world.addComponent(e, TransformComponent{}); // metres: parts are life-size
        m_world.addComponent(e, PreviousTransformComponent{});
        m_world.addComponent(e, BoundsComponent{static_cast<sw::f32>(
                                    std::max(spec.footprintM[0], spec.footprintM[1]))});
        m_world.addComponent(e, MeshComponent{m_partMeshIds.at(definitionId)});
        if (marker.a > 0.0f)
        {
            m_world.addComponent(e, MapMarkerComponent{marker});
        }

        // SOLID. Straight from the .swpart's hitboxes — belts and cables
        // excepted, which is why you can walk a factory floor at all.
        sw::phys::HullComponent hull{};
        if (hullFor(*definition, hull))
        {
            m_world.addComponent(e, hull);
        }

        sw::phys::SurfaceAnchorComponent anchor{};
        anchor.body = body;
        anchor.localPosition = sw::WorldVec3(up) * (gravity->bodyRadius + elevation);
        anchor.localRotation =
            standUp * glm::angleAxis(yawRadians, sw::Vec3{0.0f, 1.0f, 0.0f});
        m_world.addComponent(e, anchor);

        sw::factory::BuildingComponent building{};
        building.definitionId = definitionId;
        building.site = site;
        building.category = spec.category;
        building.groundDensity =
            (deposits != nullptr)
                ? sw::planet::oreDensity(*deposits, up, sw::res::Resource::IronOre)
                : 0.0f;
        m_world.addComponent(e, building);

        sw::factory::RecipeStateComponent state{};
        state.recipeId = recipeId;
        m_world.addComponent(e, state);

        sw::factory::PowerComponent power{};
        power.producedKw = std::max(0.0, spec.powerKw);
        power.consumedKw = std::max(0.0, -spec.powerKw);
        if (const auto* recipe = sw::factory::findRecipe(recipeId))
        {
            power.consumedKw += recipe->powerKw;
        }
        // Who the grid drops first when the sun goes down. A default, not a
        // law: the E panel will let the player promote their electrolyser.
        power.priority = sw::factory::defaultPowerPriority(spec.category);
        m_world.addComponent(e, power);

        // A battery bank is a building that happens to hold joules. Giving
        // it the component here (rather than a flag in the .swpart) keeps
        // the part file about GEOMETRY, which is what the Part Studio edits.
        if (spec.category == sw::factory::BuildingCategory::Battery)
        {
            sw::factory::BatteryComponent battery{};
            m_world.addComponent(e, battery);
        }

        // ...and an assembly hall is a building that happens to hold an
        // ORDER, plus the little queue of names that leaves with its crates.
        // Same reasoning: the .swpart says what it looks like and what
        // category it is; what that category implies lives here.
        if (spec.category == sw::factory::BuildingCategory::Assembly)
        {
            m_world.addComponent(e, sw::factory::AssemblyComponent{});
            m_world.addComponent(e, sw::factory::VehicleQueueComponent{});
        }

        if (spec.inventoryVolumeM3 > 0.0)
        {
            sw::factory::InventoryComponent inventory{};
            inventory.volumeCapacityM3 = spec.inventoryVolumeM3;
            m_world.addComponent(e, inventory);
        }
        return e;
    }




    // ------------------------------------------------------------------------
    // A BODY'S POSE AS IT IS BEING DRAWN
    //
    // Anything positioned relative to a planet has to use the pose that
    // planet is RENDERED at, not the one it was last simulated at. The
    // difference is a physics step of orbital motion — 595 m for Terra — and
    // it resets every tick, so the symptom is always the same: a thing that
    // should be nailed to the ground swings hundreds of metres and snaps
    // back. It cost us the conveyor cargo once already; the build ghost is
    // the second time, so it is a function now.
    // ------------------------------------------------------------------------
    void StarWorksGame::bodyRenderPose(sw::ecs::Entity body, sw::WorldVec3& outPosition,
                                       glm::dquat& outRotation)
    {
        outPosition = sw::WorldVec3{0.0};
        outRotation = glm::dquat{1.0, 0.0, 0.0, 0.0};
        const auto* transform = m_world.tryGetComponent<TransformComponent>(body);
        if (transform == nullptr)
        {
            return;
        }
        const sw::f64 alpha = static_cast<sw::f64>(m_physicsLane->alpha());
        outPosition = transform->position;
        if (const auto* previous =
                m_world.tryGetComponent<PreviousTransformComponent>(body))
        {
            outPosition = glm::mix(previous->position, transform->position, alpha);
        }
        if (const auto* gravity =
                m_world.tryGetComponent<sw::phys::GravitySourceComponent>(body))
        {
            outRotation = sw::phys::spinRotationAt(*gravity, alpha);
        }
        else
        {
            outRotation = glm::dquat(transform->rotation);
        }
    }


    bool StarWorksGame::conveyorPortOf(sw::ecs::Entity entity, sw::parts::NodeType type,
                                       sw::WorldVec3& outLocal)
    {
        return conveyorPortOf(entity, type, 0, outLocal);
    }

    bool StarWorksGame::conveyorPortOf(sw::ecs::Entity entity, sw::parts::NodeType type,
                                       sw::u32 index, sw::WorldVec3& outLocal)
    {
        const auto* building =
            m_world.tryGetComponent<sw::factory::BuildingComponent>(entity);
        const auto* anchor =
            m_world.tryGetComponent<sw::phys::SurfaceAnchorComponent>(entity);
        if (building == nullptr || anchor == nullptr)
        {
            return false;
        }
        const auto* definition = sw::parts::findDefinition(building->definitionId);
        if (definition == nullptr)
        {
            return false;
        }
        const std::vector<const sw::parts::AttachNode*> ports =
            sw::parts::conveyorNodes(*definition, type);
        if (index >= ports.size())
        {
            return false;
        }
        outLocal = anchor->localPosition +
                   sw::WorldVec3(anchor->localRotation * ports[index]->position);
        return true;
    }

    // WHICH MOUTH DID YOU MEAN?
    //
    // A machine with two out ports needs a port picked, and the least
    // intrusive way to ask is not to ask: the player is already aiming at
    // the machine, so take the mouth nearest their aim. It reads as "click
    // the side you want", which is what you would do with real plumbing.
    //
    // Mouths that already have a belt on them are skipped — that is what
    // makes the SECOND port reachable at all once the first is wired, and
    // it means clicking the same machine twice lays two different runs
    // rather than refusing the second.
    sw::u32 StarWorksGame::chooseConveyorPort(sw::ecs::Entity entity,
                                              sw::parts::NodeType type,
                                              const sw::WorldVec3& aimLocal, bool& outAny)
    {
        outAny = false;
        sw::u32 best = 0;
        sw::f64 bestDistance = 1.0e30;
        for (sw::u32 index = 0; index < sw::factory::kMaxMachinePorts; ++index)
        {
            sw::WorldVec3 port{};
            if (!conveyorPortOf(entity, type, index, port))
            {
                break;
            }
            // Taken? A belt mouth within the snap radius already owns it.
            bool taken = false;
            m_world.forEach<sw::factory::BuildingComponent,
                            sw::phys::SurfaceAnchorComponent>(
                [&](sw::ecs::Entity other, sw::factory::BuildingComponent& building,
                    sw::phys::SurfaceAnchorComponent&) {
                    if (taken || other == entity ||
                        building.category != sw::factory::BuildingCategory::Conveyor)
                    {
                        return;
                    }
                    // A belt's mouth of the OPPOSITE kind is what would meet
                    // this one: our out port is met by a belt's in port.
                    const sw::parts::NodeType facing =
                        (type == sw::parts::NodeType::ConveyorOut)
                            ? sw::parts::NodeType::ConveyorIn
                            : sw::parts::NodeType::ConveyorOut;
                    sw::WorldVec3 beltPort{};
                    if (conveyorPortOf(other, facing, 0, beltPort) &&
                        glm::length(beltPort - port) < sw::factory::kConveyorPortSnapM)
                    {
                        taken = true;
                    }
                });
            if (taken)
            {
                continue;
            }
            const sw::f64 distance = glm::length(port - aimLocal);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                best = index;
                outAny = true;
            }
        }
        return best;
    }

    // SOLIDITY, from the .swpart. A definition's authored hitboxes become
    // the entity's HullComponent at spawn, so collision never goes back to
    // the catalogue and an entity's solidity is a fact about the entity.
    //
    // Belts and cables get none, on purpose: you step over a conveyor deck
    // and duck under a wire. Making them solid would turn a factory floor
    // into an obstacle course, and they are the two things a player walks
    // among most.
    bool StarWorksGame::hullFor(const sw::parts::PartDefinition& definition,
                                sw::phys::HullComponent& outHull)
    {
        if (!sw::parts::isSolid(definition))
        {
            return false;
        }
        const std::vector<sw::parts::HitBox> boxes = sw::parts::effectiveHull(definition);
        if (boxes.empty())
        {
            return false;
        }
        outHull = sw::phys::HullComponent{};
        for (const sw::parts::HitBox& box : boxes)
        {
            if (outHull.count >= sw::phys::kMaxHullBoxes)
            {
                break;
            }
            outHull.boxes[outHull.count++] = {box.center, glm::abs(box.halfExtents)};
            outHull.radius = std::max(outHull.radius,
                                      sw::phys::obbRadius(box.center, box.halfExtents));
        }
        return outHull.count > 0;
    }

    // Hulls are DERIVED from the .swpart, so they are not saved — a hull in
    // a save file would be a second copy of the model's own answer, free to
    // drift the moment a part is redrawn. They are rebuilt after a load
    // instead, exactly like the conveyor chains and the power grids. Without
    // this a loaded world had no solid objects at all and E hit nothing.
    void StarWorksGame::rebuildHulls()
    {
        std::vector<std::pair<sw::ecs::Entity, sw::phys::HullComponent>> hulls;
        auto collect = [&](sw::ecs::Entity entity, sw::u32 definitionId) {
            const auto* definition = sw::parts::findDefinition(definitionId);
            sw::phys::HullComponent hull{};
            if (definition != nullptr && hullFor(*definition, hull))
            {
                hulls.emplace_back(entity, hull);
            }
        };
        m_world.forEach<sw::factory::BuildingComponent>(
            [&](sw::ecs::Entity entity, sw::factory::BuildingComponent& building) {
                collect(entity, building.definitionId);
            });
        m_world.forEach<sw::parts::PartComponent>(
            [&](sw::ecs::Entity entity, sw::parts::PartComponent& part) {
                collect(entity, part.definitionId);
            });
        for (const auto& [entity, hull] : hulls)
        {
            if (m_world.hasComponent<sw::phys::HullComponent>(entity))
            {
                m_world.getComponent<sw::phys::HullComponent>(entity) = hull;
            }
            else
            {
                m_world.addComponent(entity, hull);
            }
        }
        // ...and the player, who is the one thing that gets pushed.
        if (!m_capsuleEntity.isNull())
        {
            if (const auto* suit = sw::parts::findDefinition(sw::parts::kPropEvaSuit))
            {
                sw::phys::HullComponent hull{};
                if (hullFor(*suit, hull))
                {
                    if (m_world.hasComponent<sw::phys::HullComponent>(m_capsuleEntity))
                    {
                        m_world.getComponent<sw::phys::HullComponent>(m_capsuleEntity) =
                            hull;
                    }
                    else
                    {
                        m_world.addComponent(m_capsuleEntity, hull);
                    }
                    if (!m_world.hasComponent<sw::phys::HullMoverComponent>(
                            m_capsuleEntity))
                    {
                        m_world.addComponent(m_capsuleEntity,
                                             sw::phys::HullMoverComponent{});
                    }
                }
            }
        }
        SW_LOG_INFO("Game", "Hulls rebuilt: {} solid objects", hulls.size());
    }

    // WHAT AM I LOOKING AT? A ray from the eye against the solid hulls —
    // which is exactly the question "near enough, and in front of me", and
    // exactly what a distance-to-centre check could not answer. A 16 m solar
    // field whose centre is 20 m away is still right there in front of you;
    // a silo behind your shoulder is not, however close its centre is.
    sw::ecs::Entity StarWorksGame::hullUnderCrosshair(sw::f64 maxDistanceM)
    {
        // THE CAMERA IS IN THE RENDERED WORLD, so the boxes must be too.
        //
        // The first version read each building's TransformComponent raw —
        // its TICK pose — and cast an 18 m ray at it from a camera sitting
        // in the interpolated world. One physics step of Terra's orbit is
        // 595 m, so the buildings were never anywhere near the ray and E
        // simply stopped working: not "sometimes wrong", never right.
        //
        // Same interpolation as the mesh pass, the hull overlay, the belt
        // cargo and the build ghost. FIFTH time. On this project, anything
        // that compares a camera-space quantity against a body on a moving
        // planet goes through `mix(previous, current, alpha)`, full stop.
        const sw::f32 alpha = m_physicsLane->alpha();
        const sw::f64 alpha64 = static_cast<sw::f64>(alpha);
        const sw::WorldVec3 eye = m_camera.position();
        const sw::Vec3 forward = m_camera.forward();

        sw::ecs::Entity best{};
        sw::f32 bestT = static_cast<sw::f32>(maxDistanceM);
        // ONE RAY, TWO KINDS OF THING. `E` asks "what am I looking at", and
        // the answer is a machine or a vessel; casting twice and comparing
        // afterwards would be two answers to one question. The caller sorts
        // out what to DO with whatever it hit.
        auto cast = [&](sw::ecs::Entity entity, const TransformComponent& transform,
                        const PreviousTransformComponent& previous,
                        const sw::phys::HullComponent& hull) {
                const sw::WorldVec3 position =
                    glm::mix(previous.position, transform.position, alpha64);
                const sw::Quat rotation =
                    glm::slerp(previous.rotation, transform.rotation, alpha);

                // Broad phase first, same as the collision system: one f64
                // subtraction rejects everything but the neighbours.
                const sw::WorldVec3 offset = position - eye;
                const sw::f64 reach = maxDistanceM + static_cast<sw::f64>(hull.radius);
                if (glm::dot(offset, offset) > reach * reach)
                {
                    return;
                }
                const sw::Vec3 relative = sw::Vec3(offset);
                for (sw::u32 i = 0; i < hull.count; ++i)
                {
                    const sw::phys::Obb box =
                        sw::phys::makeObb(relative + rotation * hull.boxes[i].centre,
                                          hull.boxes[i].halfExtents, rotation);
                    sw::f32 t = 0.0f;
                    sw::Vec3 normal{};
                    if (sw::phys::rayObb(sw::Vec3{0.0f}, forward, box, bestT, t, normal) &&
                        t < bestT)
                    {
                        bestT = t;
                        best = entity;
                    }
                }
        };
        m_world.forEach<TransformComponent, PreviousTransformComponent,
                        sw::phys::HullComponent, sw::factory::BuildingComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                PreviousTransformComponent& previous, sw::phys::HullComponent& hull,
                sw::factory::BuildingComponent&) { cast(entity, transform, previous, hull); });
        m_world.forEach<TransformComponent, PreviousTransformComponent,
                        sw::phys::HullComponent, sw::parts::PartComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                PreviousTransformComponent& previous, sw::phys::HullComponent& hull,
                sw::parts::PartComponent&) { cast(entity, transform, previous, hull); });
        return best;
    }

    // Where a cable hooks onto this entity, in the body's rotating frame.
    // Same shape as `conveyorPortOf` and for the same reason: the node is
    // authored on the geometry, so there is exactly one way to ask where it
    // ended up once the building was stood on a sphere.
    bool StarWorksGame::powerNodeOf(sw::ecs::Entity entity, sw::WorldVec3& outLocal)
    {
        const auto* building =
            m_world.tryGetComponent<sw::factory::BuildingComponent>(entity);
        const auto* anchor =
            m_world.tryGetComponent<sw::phys::SurfaceAnchorComponent>(entity);
        if (building == nullptr || anchor == nullptr)
        {
            return false;
        }
        const auto* definition = sw::parts::findDefinition(building->definitionId);
        if (definition == nullptr)
        {
            return false;
        }
        const sw::parts::AttachNode* node = sw::parts::findPowerNode(*definition);
        if (node == nullptr)
        {
            return false;
        }
        outLocal =
            anchor->localPosition + sw::WorldVec3(anchor->localRotation * node->position);
        return true;
    }

    // ------------------------------------------------------------------------
    // THE GRID, DERIVED FROM THE CABLES
    //
    // Run after every build and every demolition, exactly like the conveyor
    // network — and for the same reason: there must not be a second copy of
    // "who is connected to whom" that can fall out of step with the objects
    // the player can see.
    //
    // What it does, in order:
    //   1. drops cables whose endpoints have gone (demolished under the wire);
    //   2. numbers every building, unions the ones a cable joins, and writes
    //      the resulting component id into each PowerComponent;
    //   3. re-hangs every surviving cable's curve from its endpoints' CURRENT
    //      power nodes, so a span can never be left pointing at where a
    //      machine used to be.
    // ------------------------------------------------------------------------
    void StarWorksGame::rebuildPowerNetwork()
    {
        // ---- 1. every building, in a stable order ------------------------
        std::vector<sw::ecs::Entity> nodes;
        std::unordered_map<sw::ecs::Entity, sw::u32> indexOf;
        m_world.forEach<sw::factory::BuildingComponent, sw::factory::PowerComponent>(
            [&](sw::ecs::Entity entity, sw::factory::BuildingComponent&,
                sw::factory::PowerComponent&) {
                indexOf[entity] = static_cast<sw::u32>(nodes.size());
                nodes.push_back(entity);
            });

        // ---- 2. the cables, minus the ones left dangling -----------------
        std::vector<sw::factory::PowerLink> links;
        std::vector<sw::ecs::Entity> cables;
        std::vector<sw::ecs::Entity> orphans;
        m_world.forEach<sw::factory::PowerLinkComponent>(
            [&](sw::ecs::Entity entity, sw::factory::PowerLinkComponent& link) {
                const auto endA = indexOf.find(link.a);
                const auto endB = indexOf.find(link.b);
                if (endA == indexOf.end() || endB == indexOf.end())
                {
                    orphans.push_back(entity); // one end was demolished
                    return;
                }
                links.push_back({endA->second, endB->second});
                cables.push_back(entity);
            });
        for (const sw::ecs::Entity entity : orphans)
        {
            m_world.destroyEntity(entity);
        }

        // ---- 3. the components, into the components ----------------------
        const std::vector<sw::u32> grid =
            sw::factory::traceGrids(nodes.size(), links);
        for (sw::usize i = 0; i < nodes.size(); ++i)
        {
            if (auto* power =
                    m_world.tryGetComponent<sw::factory::PowerComponent>(nodes[i]))
            {
                power->gridId = grid[i];
            }
        }

        // ---- 4. re-hang every span ---------------------------------------
        for (sw::usize i = 0; i < cables.size(); ++i)
        {
            const auto& link =
                m_world.getComponent<sw::factory::PowerLinkComponent>(cables[i]);
            auto* cable = m_world.tryGetComponent<CableComponent>(cables[i]);
            if (cable == nullptr)
            {
                continue;
            }
            sw::WorldVec3 from{};
            sw::WorldVec3 to{};
            if (!powerNodeOf(link.a, from) || !powerNodeOf(link.b, to))
            {
                continue;
            }
            hangCable(*cable, from, to);
        }
        SW_LOG_INFO("Game", "Power network: {} buildings, {} cables", nodes.size(),
                    cables.size());
    }

    // The sagging curve, sampled into the component. `up` is the local
    // vertical at the middle of the span — on a 6,371 km sphere the two ends
    // of a 40 m cable have verticals 0.0004 degrees apart, so one is enough,
    // and using the midpoint's keeps the sag symmetric.
    void StarWorksGame::hangCable(CableComponent& cable, const sw::WorldVec3& from,
                                  const sw::WorldVec3& to)
    {
        const sw::Vec3 up = sw::Vec3(glm::normalize((from + to) * 0.5));
        cable.pointCount = CableComponent::kMaxPoints;
        sw::f64 length = 0.0;
        for (sw::u32 i = 0; i < cable.pointCount; ++i)
        {
            const sw::f64 t = static_cast<sw::f64>(i) /
                              static_cast<sw::f64>(cable.pointCount - 1);
            cable.points[i] =
                sw::factory::cablePointAt(from, to, up, kCableSagFraction, t);
            if (i > 0)
            {
                length += glm::length(cable.points[i] - cable.points[i - 1]);
            }
        }
        cable.lengthM = static_cast<sw::f32>(length);
    }

    // The whole answer to "may this cable be laid", for the preview and for
    // the commit. One function, so the green line you were shown and the
    // wire you get cannot disagree.
    sw::factory::CableVerdict StarWorksGame::planCable(sw::ecs::Entity from,
                                                       sw::ecs::Entity to,
                                                       sw::WorldVec3& outFrom,
                                                       sw::WorldVec3& outTo)
    {
        if (from.isNull() || to.isNull() || from == to)
        {
            return sw::factory::CableVerdict::SameNode;
        }
        if (!powerNodeOf(from, outFrom) || !powerNodeOf(to, outTo))
        {
            return sw::factory::CableVerdict::NoPowerNode;
        }
        const auto* buildingA =
            m_world.tryGetComponent<sw::factory::BuildingComponent>(from);
        const auto* buildingB =
            m_world.tryGetComponent<sw::factory::BuildingComponent>(to);
        const auto* powerA = m_world.tryGetComponent<sw::factory::PowerComponent>(from);
        const auto* powerB = m_world.tryGetComponent<sw::factory::PowerComponent>(to);
        if (buildingA == nullptr || buildingB == nullptr || powerA == nullptr ||
            powerB == nullptr)
        {
            return sw::factory::CableVerdict::NoPowerNode;
        }

        // How many wires already meet at each end.
        sw::u32 onA = 0;
        sw::u32 onB = 0;
        m_world.forEach<sw::factory::PowerLinkComponent>(
            [&](sw::ecs::Entity, sw::factory::PowerLinkComponent& link) {
                if (link.a == from || link.b == from) { onA += 1; }
                if (link.a == to || link.b == to) { onB += 1; }
            });

        return sw::factory::validateCable(
            true, true, buildingA->category, buildingB->category, onA, onB,
            powerA->gridId, powerB->gridId, glm::length(outTo - outFrom),
            kMaxCableLengthM);
    }

    void StarWorksGame::layCable(sw::ecs::Entity body, sw::ecs::Entity from,
                                 sw::ecs::Entity to)
    {
        sw::WorldVec3 fromNode{};
        sw::WorldVec3 toNode{};
        if (planCable(from, to, fromNode, toNode) != sw::factory::CableVerdict::Ok)
        {
            return;
        }
        const sw::ecs::Entity entity = m_world.createEntity();
        m_world.addComponent(entity, TransformComponent{});
        m_world.addComponent(entity, PreviousTransformComponent{});
        m_world.addComponent(entity, sw::factory::PowerLinkComponent{from, to});
        CableComponent cable{};
        cable.body = body;
        hangCable(cable, fromNode, toNode);
        m_world.addComponent(entity, cable);
        // ...and only NOW is the grid what the cables say it is.
        rebuildPowerNetwork();
    }

    // ------------------------------------------------------------------------
    // PLANNING A BELT
    //
    // The player's operation is "feed THIS from THAT", not "put a tile here,
    // then another". So the tool takes two machines and produces the run
    // between their mouths — and what it produces is ordinary CV-1 buildings,
    // so afterwards there is nothing special about a belt the tool laid
    // versus one placed by hand. The network is still derived from where the
    // ports ended up.
    //
    // Same routine for the preview and the commit: a run you were shown in
    // green cannot come out different when you click.
    // ------------------------------------------------------------------------
    sw::build::Verdict StarWorksGame::planBelt(sw::ecs::Entity body, sw::ecs::Entity from,
                                               sw::ecs::Entity to,
                                               std::vector<BeltTile>& outTiles)
    {
        return planBelt(body, from, 0, to, 0, outTiles);
    }

    sw::build::Verdict StarWorksGame::planBelt(sw::ecs::Entity body, sw::ecs::Entity from,
                                               sw::u32 fromPortIndex, sw::ecs::Entity to,
                                               sw::u32 toPortIndex,
                                               std::vector<BeltTile>& outTiles)
    {
        outTiles.clear();
        const auto* terrain = m_world.tryGetComponent<sw::planet::TerrainComponent>(body);
        const auto* gravity =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(body);
        const auto* segment = sw::parts::findDefinition(sw::parts::kBuildingConveyor);
        if (terrain == nullptr || gravity == nullptr || segment == nullptr ||
            from == to || from.isNull() || to.isNull())
        {
            return sw::build::Verdict::NoDefinition;
        }

        sw::WorldVec3 fromPort{};
        sw::WorldVec3 toPort{};
        if (!conveyorPortOf(from, sw::parts::NodeType::ConveyorOut, fromPortIndex,
                            fromPort) ||
            !conveyorPortOf(to, sw::parts::NodeType::ConveyorIn, toPortIndex, toPort))
        {
            return sw::build::Verdict::NoDefinition; // one of them has no mouth
        }

        sw::WorldVec3 path[sw::factory::kMaxConveyorPoints]{};
        sw::u32 count = sw::factory::kMaxConveyorPoints;
        const sw::f64 length = sw::factory::buildConveyorPath(
            *terrain, gravity->bodyRadius, fromPort, toPort, 0.0, path, count);
        if (length > kMaxBeltLengthM)
        {
            return sw::build::Verdict::OutOfRange;
        }

        const sw::f64 span = static_cast<sw::f64>(m_conveyorSegmentM);
        const sw::i32 tiles =
            std::max(1, static_cast<sw::i32>(std::lround(length / span)));
        const std::vector<sw::build::Footprint> occupied = footprintsOn(body);
        static const sw::planet::DepositComponent kNoDeposits{};
        const auto* deposits = m_world.tryGetComponent<sw::planet::DepositComponent>(body);

        sw::build::Verdict worst = sw::build::Verdict::Ok;
        for (sw::i32 i = 0; i < tiles; ++i)
        {
            sw::WorldVec3 local{};
            sw::Vec3 heading{};
            sw::factory::conveyorPointAt(
                path, count, (static_cast<sw::f64>(i) + 0.5) * (length / tiles), local,
                heading);
            const sw::Vec3 up = sw::Vec3(glm::normalize(local));
            // Model -Z is the direction of travel: aim it down the run.
            outTiles.push_back({up, yawToFace(up, sw::Vec3{0.0f, 0.0f, -1.0f}, heading)});

            const sw::build::Verdict verdict = sw::build::validatePlacement(
                *terrain, (deposits != nullptr) ? *deposits : kNoDeposits,
                gravity->bodyRadius, *segment, up, occupied);
            if (verdict != sw::build::Verdict::Ok && worst == sw::build::Verdict::Ok)
            {
                worst = verdict; // the FIRST reason the run cannot be laid
            }
        }
        return worst;
    }

    sw::u32 StarWorksGame::defaultRecipeFor(sw::factory::BuildingCategory category)
    {
        const std::vector<sw::u32> recipes = sw::factory::recipesForCategory(category);
        return recipes.empty() ? 0u : recipes.front();
    }

    // ------------------------------------------------------------------------
    // THE CONVEYOR NETWORK, DERIVED FROM GEOMETRY
    //
    // A belt segment is an ordinary building: the player places them one at a
    // time, like a smelter. What turns a ROW of them into a working link is
    // not an intention the game recorded — it is that their conveyor-out and
    // conveyor-in ports MEET. So the network is not stored, it is derived,
    // after every build and every demolition, from where things are.
    //
    // That is the same choice the deposits and the orbits already made, and
    // it buys the same thing: there is no second copy of the truth to fall
    // out of step. Demolish a segment in the middle of a run and the chain
    // simply is not there next frame, because the ports no longer meet.
    // ------------------------------------------------------------------------
    void StarWorksGame::rebuildConveyorNetwork()
    {

        // Everything standing, with its ports resolved into the body frame.
        std::vector<sw::factory::PortNode> nodes;
        std::vector<sw::ecs::Entity> bodies;
        m_world.forEach<sw::factory::BuildingComponent,
                        sw::phys::SurfaceAnchorComponent>(
            [&](sw::ecs::Entity entity, sw::factory::BuildingComponent& building,
                sw::phys::SurfaceAnchorComponent& anchor) {
                const auto* definition = sw::parts::findDefinition(building.definitionId);
                if (definition == nullptr)
                {
                    return;
                }
                sw::factory::PortNode node{};
                node.entity = entity;
                node.isBelt =
                    building.category == sw::factory::BuildingCategory::Conveyor;
                node.centre = anchor.localPosition;
                // EVERY mouth, in authored order — the order is the
                // contract: out mouth i ships the recipe's product i.
                for (const sw::parts::AttachNode* port : sw::parts::conveyorNodes(
                         *definition, sw::parts::NodeType::ConveyorOut))
                {
                    if (node.outCount >= sw::factory::kMaxMachinePorts) { break; }
                    node.outPorts[node.outCount++] =
                        anchor.localPosition +
                        sw::WorldVec3(anchor.localRotation * port->position);
                }
                for (const sw::parts::AttachNode* port : sw::parts::conveyorNodes(
                         *definition, sw::parts::NodeType::ConveyorIn))
                {
                    if (node.inCount >= sw::factory::kMaxMachinePorts) { break; }
                    node.inPorts[node.inCount++] =
                        anchor.localPosition +
                        sw::WorldVec3(anchor.localRotation * port->position);
                }
                nodes.push_back(node);
                bodies.push_back(anchor.body);
            });

        // Old conveyors and their links go first: the graph below is the only
        // author of both, so anything left over is a ghost of a demolished run.
        std::vector<sw::ecs::Entity> stale;
        m_world.forEach<ConveyorComponent>(
            [&](sw::ecs::Entity entity, ConveyorComponent&) { stale.push_back(entity); });
        for (const sw::ecs::Entity entity : stale)
        {
            m_world.destroyEntity(entity);
        }
        for (const sw::factory::PortNode& node : nodes)
        {
            if (m_world.hasComponent<sw::factory::ItemLinkComponent>(node.entity))
            {
                m_world.removeComponent<sw::factory::ItemLinkComponent>(node.entity);
            }
        }

        for (const sw::factory::Chain& chain :
             sw::factory::traceConveyorChains(nodes, sw::factory::kConveyorPortSnapM))
        {
            const sw::factory::PortNode& source = nodes[chain.source];
            const sw::factory::PortNode& destination = nodes[chain.destination];
            if (bodies[chain.source] != bodies[chain.destination])
            {
                continue; // two different worlds: not a belt, a coincidence
            }

            // The cargo path: out of the source, along every deck, into the
            // destination.
            std::vector<sw::WorldVec3> path;
            path.push_back(source.outPorts[chain.sourcePort]);
            for (const sw::u32 belt : chain.belts)
            {
                path.push_back(nodes[belt].centre);
            }
            path.push_back(destination.inPorts[chain.destinationPort]);

            // WHAT does it carry? EVERYTHING the source makes — a belt out
            // of an electrolyser carries the hydrogen and the oxygen, because
            // ONE MOUTH, EVERYTHING; SEVERAL MOUTHS, ONE PRODUCT EACH.
            //
            // A machine with a single out port has nowhere else to put its
            // products, so its belt carries all of them — that is what made
            // the fuel chain buildable at all. A machine with SEVERAL ports
            // is making a different statement: mouth i ships product i, so
            // hydrogen leaves by one belt and oxygen by the other, and the
            // player decides where each goes. Nothing to say means nothing
            // to move.
            sw::res::Resource carried[sw::factory::kMaxRecipeIngredients]{
                sw::res::Resource::Count, sw::res::Resource::Count,
                sw::res::Resource::Count, sw::res::Resource::Count};
            sw::usize carriedCount = 0;
            const bool splitByPort = source.outCount > 1;
            if (const auto* state =
                    m_world.tryGetComponent<sw::factory::RecipeStateComponent>(
                        source.entity))
            {
                if (const auto* recipe = sw::factory::findRecipe(state->recipeId))
                {
                    sw::u32 productIndex = 0;
                    for (const sw::factory::Ingredient& output : recipe->outputs)
                    {
                        if (output.resource == sw::res::Resource::Count ||
                            output.unitsPerSecond <= 0.0)
                        {
                            continue;
                        }
                        if (splitByPort)
                        {
                            // Mouth i ships product i, and a mouth past the
                            // last product ships nothing — an unused port is
                            // an empty belt, which is the honest picture.
                            if (productIndex == chain.sourcePort)
                            {
                                carried[carriedCount++] = output.resource;
                            }
                        }
                        else
                        {
                            carried[carriedCount++] = output.resource;
                        }
                        ++productIndex;
                    }
                }
            }
            if (carriedCount == 0 &&
                m_world.hasComponent<sw::factory::AssemblyComponent>(source.entity))
            {
                // AN ASSEMBLY HALL SHIPS ROCKETS. It has no recipe, and the
                // silo rule below would have it export the iron it is
                // standing on — the belt to the pad would run backwards,
                // carrying the metal away from the machine that needs it.
                // A hall's product is the one thing it makes.
                carried[carriedCount++] = sw::res::Resource::Vehicle;
            }
            if (carriedCount == 0)
            {
                // A silo ships what it is holding.
                if (const auto* inventory =
                        m_world.tryGetComponent<sw::factory::InventoryComponent>(
                            source.entity))
                {
                    for (const sw::factory::InventorySlot& slot : inventory->slots)
                    {
                        if (slot.resource != sw::res::Resource::Count && slot.units > 0.0)
                        {
                            carried[carriedCount++] = slot.resource;
                            break;
                        }
                    }
                }
            }
            if (carriedCount == 0 ||
                !m_world.hasComponent<sw::factory::InventoryComponent>(
                    destination.entity))
            {
                continue; // nothing to carry, or nowhere to put it
            }
            const sw::res::Resource resource = carried[0]; // what it looks like

            // THE LINK, on the destination — one channel per good. Several
            // belts may arrive at the same machine, so the component may
            // already be there: add to it rather than replacing it.
            if (!m_world.hasComponent<sw::factory::ItemLinkComponent>(destination.entity))
            {
                m_world.addComponent(destination.entity,
                                     sw::factory::ItemLinkComponent{});
            }
            {
                auto& link =
                    m_world.getComponent<sw::factory::ItemLinkComponent>(
                        destination.entity);
                for (sw::usize i = 0; i < carriedCount; ++i)
                {
                    sw::factory::linkAddChannel(link, source.entity, carried[i],
                                                kConveyorRateUnitsPerSecond);
                }
            }

            // ...and the cargo path, subsampled if the run is longer than the
            // component can hold: the crates are a depiction, and sixteen
            // waypoints depict a belt of any length perfectly well.
            ConveyorComponent conveyor{};
            conveyor.body = bodies[chain.source];
            conveyor.link = destination.entity;
            conveyor.source = source.entity;
            conveyor.cargoColor = resourceCargoColor(resource);
            conveyor.cargoMesh = (resource == sw::res::Resource::Vehicle)
                                     ? m_vehicleCargoMeshIndex
                                     : m_cargoMeshIndex;
            const sw::usize count =
                std::min<sw::usize>(path.size(), ConveyorComponent::kMaxPoints);
            conveyor.pointCount = static_cast<sw::u32>(count);
            for (sw::usize i = 0; i < count; ++i)
            {
                const sw::usize pick =
                    (count == 1) ? 0 : (i * (path.size() - 1)) / (count - 1);
                conveyor.points[i] = path[pick];
            }
            sw::f64 length = 0.0;
            for (sw::u32 i = 0; i + 1 < conveyor.pointCount; ++i)
            {
                length += glm::length(conveyor.points[i + 1] - conveyor.points[i]);
            }
            conveyor.lengthM = static_cast<sw::f32>(length);
            if (conveyor.lengthM < 0.5f)
            {
                continue;
            }

            const sw::ecs::Entity e = m_world.createEntity();
            m_world.addComponent(e, TransformComponent{});
            m_world.addComponent(e, PreviousTransformComponent{});
            sw::phys::SurfaceAnchorComponent anchor{};
            anchor.body = bodies[chain.source];

            anchor.localPosition = conveyor.points[0];
            m_world.addComponent(e, anchor);
            m_world.addComponent(e, conveyor);
        }
    }

    // ------------------------------------------------------------------------
    // F2 — THE GROUND BUILD CURSOR
    //
    // Arm a building in the F menu, walk to where you want it, look at the
    // ground. The ghost lands where your gaze meets the heightfield — the
    // real one, marched, not a flat plane at sea level — inside a reach you
    // have to walk to extend. The wheel spins it. Click builds. R razes what
    // you are looking at.
    //
    // The green/red is not a second opinion: it is exactly the verdict
    // `placeBuilding` will be handed, from exactly the .swpart fields, so
    // the ghost cannot promise something the commit refuses.
    // ------------------------------------------------------------------------
    std::vector<sw::build::Footprint> StarWorksGame::footprintsOn(
        sw::ecs::Entity body)
    {
        std::vector<sw::build::Footprint> footprints;
        m_world.forEach<sw::factory::BuildingComponent,
                        sw::phys::SurfaceAnchorComponent>(
            [&](sw::ecs::Entity, sw::factory::BuildingComponent& building,
                sw::phys::SurfaceAnchorComponent& anchor) {
                if (anchor.body != body)
                {
                    return;
                }
                const auto* definition = sw::parts::findDefinition(building.definitionId);
                if (definition == nullptr ||
                    building.category == sw::factory::BuildingCategory::Conveyor)
                {
                    return; // belts do not block: see validatePlacement
                }
                footprints.push_back(
                    {sw::Vec3(glm::normalize(anchor.localPosition)),
                     sw::build::footprintRadius(definition->building)});
            });
        return footprints;
    }

    sw::ecs::Entity StarWorksGame::siteNear(sw::ecs::Entity body,
                                            const sw::Vec3& direction)
    {
        const auto* gravity =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(body);
        if (gravity == nullptr)
        {
            return {};
        }
        sw::ecs::Entity best{};
        sw::f64 bestDistance = 400.0; // a site is a PLACE: 400 m across, no more
        m_world.forEach<sw::factory::SiteComponent, sw::phys::SurfaceAnchorComponent>(
            [&](sw::ecs::Entity entity, sw::factory::SiteComponent& site,
                sw::phys::SurfaceAnchorComponent& anchor) {
                if (site.body != body)
                {
                    return;
                }
                const sw::f64 distance = sw::build::groundDistance(
                    direction, sw::Vec3(glm::normalize(anchor.localPosition)),
                    gravity->bodyRadius);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    best = entity;
                }
            });
        return best;
    }

    void StarWorksGame::updateBuildCursor()
    {
        m_buildCursor = {};
        m_beltPreview.clear();
        m_beltVerdict = sw::build::Verdict::NoGround;
        if (!m_evaMode || m_mapView || m_editorMode || m_buildMenu ||
            !m_configTarget.isNull() || m_capsuleEntity.isNull())
        {
            m_beltSource = {};
            m_cableSource = {};
            return;
        }

        // What are we standing on? The SOI primary, if it has ground.
        const sw::i32 primaryIndex = controlledPrimaryIndex();
        if (primaryIndex < 0)
        {
            return;
        }
        const sw::ecs::Entity body =
            m_celestialIndex.body(static_cast<sw::usize>(primaryIndex)).entity;
        const auto* terrain = m_world.tryGetComponent<sw::planet::TerrainComponent>(body);
        const auto* gravity =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(body);
        const auto* bodyTransform = m_world.tryGetComponent<TransformComponent>(body);
        if (terrain == nullptr || gravity == nullptr || bodyTransform == nullptr)
        {
            return;
        }
        m_buildCursor.body = body;

        // Into the body's ROTATING frame, through the pose the body is being
        // DRAWN at. The camera sits in the rendered world; transforming it
        // with the raw tick pose would start the ray up to 595 m from where
        // the player actually is.
        sw::WorldVec3 bodyPosition{};
        glm::dquat bodyRotation{};
        bodyRenderPose(body, bodyPosition, bodyRotation);
        const glm::dquat toBody = glm::inverse(bodyRotation);
        const sw::WorldVec3 eyeLocal = toBody * (m_camera.position() - bodyPosition);
        const sw::WorldVec3 aimLocal = toBody * glm::dvec3(m_camera.forward());

        sw::WorldVec3 hitLocal{};
        if (!sw::build::raycastTerrain(*terrain, gravity->bodyRadius, eyeLocal, aimLocal,
                                       kBuildRangeM, hitLocal))
        {
            m_buildCursor.verdict = sw::build::Verdict::NoGround;
            return;
        }
        m_buildCursor.active = true;
        m_buildCursor.direction = sw::Vec3(glm::normalize(hitLocal));
        m_buildCursor.rangeM = glm::length(hitLocal - eyeLocal);
        m_buildCursor.yawRadians = m_buildYaw;

        // ---- what is under the cursor, for R ----------------------------
        {
            sw::f64 nearest = 1.0e9;
            m_world.forEach<sw::factory::BuildingComponent,
                            sw::phys::SurfaceAnchorComponent>(
                [&](sw::ecs::Entity entity, sw::factory::BuildingComponent& building,
                    sw::phys::SurfaceAnchorComponent& anchor) {
                    if (anchor.body != body)
                    {
                        return;
                    }
                    const auto* definition =
                        sw::parts::findDefinition(building.definitionId);
                    if (definition == nullptr)
                    {
                        return;
                    }
                    const sw::f64 distance = sw::build::groundDistance(
                        m_buildCursor.direction,
                        sw::Vec3(glm::normalize(anchor.localPosition)),
                        gravity->bodyRadius);
                    if (distance <
                            static_cast<sw::f64>(
                                sw::build::footprintRadius(definition->building)) &&
                        distance < nearest)
                    {
                        nearest = distance;
                        m_buildCursor.target = entity;
                    }
                });
        }

        const auto* held = sw::parts::findDefinition(m_heldBuilding);

        // ---- BELT MODE: pick an output, then an input --------------------
        // A conveyor is not placed tile by tile. The player's operation is
        // "feed this from that", so the tool takes the two machines and the
        // run between their mouths is the RESULT. It starts moving goods the
        // instant it lands, because the network is derived from where the
        // ports ended up and nothing else has to be told.
        const bool beltMode =
            held != nullptr &&
            held->building.category == sw::factory::BuildingCategory::Conveyor;
        if (beltMode)
        {
            if (!m_world.isAlive(m_beltSource))
            {
                m_beltSource = {};
            }
            // The aim point on the ground, in the body frame: which mouth
            // the player means is answered by which one they are nearest.
            const sw::WorldVec3 aimLocal =
                sw::WorldVec3(m_buildCursor.direction) *
                (gravity->bodyRadius +
                 sw::planet::terrainElevation(*terrain, m_buildCursor.direction));
            if (!m_beltSource.isNull() && !m_buildCursor.target.isNull())
            {
                bool anyIn = false;
                m_beltDestinationPort = chooseConveyorPort(
                    m_buildCursor.target, sw::parts::NodeType::ConveyorIn, aimLocal,
                    anyIn);
                m_beltVerdict =
                    anyIn ? planBelt(body, m_beltSource, m_beltSourcePort,
                                     m_buildCursor.target, m_beltDestinationPort,
                                     m_beltPreview)
                          : sw::build::Verdict::NoDefinition;
                if (!anyIn)
                {
                    m_beltPreview.clear();
                }
            }

            if (input().wasKeyPressed(sw::KeyCode::R) && !m_beltSource.isNull())
            {
                m_beltSource = {}; // R cancels a pending pick
                m_beltPreview.clear();
                return;
            }
            if (input().wasMouseButtonPressed(sw::MouseButton::Left) &&
                !m_buildCursor.target.isNull())
            {
                if (m_beltSource.isNull())
                {
                    bool anyOut = false;
                    const sw::u32 port = chooseConveyorPort(
                        m_buildCursor.target, sw::parts::NodeType::ConveyorOut, aimLocal,
                        anyOut);
                    if (anyOut)
                    {
                        m_beltSource = m_buildCursor.target;
                        m_beltSourcePort = port;
                    }
                    else
                    {
                        SW_LOG_INFO("Game",
                                    "That machine has no free output port");
                    }
                }
                else if (m_beltVerdict == sw::build::Verdict::Ok &&
                         !m_beltPreview.empty())
                {
                    for (const BeltTile& tile : m_beltPreview)
                    {
                        placeBuilding(sw::parts::kBuildingConveyor, body, tile.direction,
                                      tile.yawRadians, 0u,
                                      siteNear(body, tile.direction), {});
                    }
                    SW_LOG_INFO("Game", "BELT laid: {} segments, port {} -> port {}",
                                m_beltPreview.size(), m_beltSourcePort,
                                m_beltDestinationPort);
                    m_beltSource = {};
                    m_beltPreview.clear();
                    // ...and it is carrying goods from this frame on.
                    rebuildConveyorNetwork();
                }
            }
            return;
        }
        m_beltSource = {};

        // ---- CABLE MODE: pick two power nodes ----------------------------
        // The same two clicks as the belt, asking a different question: not
        // "feed this from that" but "put these on the same grid". A cable is
        // ONE entity rather than a row of tiles, so there is nothing to walk
        // along the ground — the span hangs between the two nodes.
        const bool cableMode =
            held != nullptr &&
            held->building.category == sw::factory::BuildingCategory::Cable;
        if (cableMode)
        {
            if (!m_world.isAlive(m_cableSource))
            {
                m_cableSource = {};
            }
            m_cableVerdict = sw::factory::CableVerdict::NoPowerNode;
            if (!m_cableSource.isNull() && !m_buildCursor.target.isNull())
            {
                sw::WorldVec3 from{};
                sw::WorldVec3 to{};
                m_cableVerdict = planCable(m_cableSource, m_buildCursor.target, from, to);
            }

            if (input().wasKeyPressed(sw::KeyCode::R))
            {
                if (!m_cableSource.isNull())
                {
                    m_cableSource = {}; // R cancels a pending pick
                    return;
                }
                // ...and with nothing pending, R CUTS. A wire has no
                // footprint to look at, so the thing you aim at is the
                // building it is tied to — which is also how you think
                // about it ("unplug the smelter").
                if (!m_buildCursor.target.isNull())
                {
                    std::vector<sw::ecs::Entity> cut;
                    const sw::ecs::Entity target = m_buildCursor.target;
                    m_world.forEach<sw::factory::PowerLinkComponent>(
                        [&](sw::ecs::Entity entity,
                            sw::factory::PowerLinkComponent& link) {
                            if (link.a == target || link.b == target)
                            {
                                cut.push_back(entity);
                            }
                        });
                    for (const sw::ecs::Entity entity : cut)
                    {
                        m_world.destroyEntity(entity);
                    }
                    if (!cut.empty())
                    {
                        SW_LOG_INFO("Game", "CUT {} cable(s)", cut.size());
                        rebuildPowerNetwork();
                    }
                }
                return;
            }
            if (input().wasMouseButtonPressed(sw::MouseButton::Left) &&
                !m_buildCursor.target.isNull())
            {
                if (m_cableSource.isNull())
                {
                    sw::WorldVec3 unused{};
                    if (powerNodeOf(m_buildCursor.target, unused))
                    {
                        m_cableSource = m_buildCursor.target;
                    }
                    else
                    {
                        SW_LOG_INFO("Game", "That building has no power connection");
                    }
                }
                else if (m_cableVerdict == sw::factory::CableVerdict::Ok)
                {
                    layCable(body, m_cableSource, m_buildCursor.target);
                    SW_LOG_INFO("Game", "CABLE laid");
                    m_cableSource = {};
                }
            }
            return;
        }
        m_cableSource = {};

        // ---- the verdict, for the armed building -------------------------
        if (held == nullptr)
        {
            m_buildCursor.verdict = sw::build::Verdict::NoDefinition;
        }
        else
        {
            static const sw::planet::DepositComponent kNoDeposits{};
            const auto* deposits =
                m_world.tryGetComponent<sw::planet::DepositComponent>(body);
            m_buildCursor.verdict = sw::build::validatePlacement(
                *terrain, (deposits != nullptr) ? *deposits : kNoDeposits,
                gravity->bodyRadius, *held, m_buildCursor.direction,
                footprintsOn(body));
        }

        // ---- input -------------------------------------------------------
        // The wheel has nothing else to do on foot (first person has no zoom),
        // so it spins the building.
        if (const sw::f32 scroll = input().scrollDeltaY(); scroll != 0.0f)
        {
            m_buildYaw += scroll * 0.19634954f; // 11.25 degrees a notch
            m_buildCursor.yawRadians = m_buildYaw;
        }

        if (input().wasKeyPressed(sw::KeyCode::R) &&
            !m_buildCursor.target.isNull())
        {
            const auto& building = m_world.getComponent<sw::factory::BuildingComponent>(
                m_buildCursor.target);
            const auto* definition = sw::parts::findDefinition(building.definitionId);
            SW_LOG_INFO("Game", "DEMOLISHED {}",
                        (definition != nullptr) ? definition->name : "building");
            m_world.destroyEntity(m_buildCursor.target);
            m_buildCursor.target = {};
            rebuildConveyorNetwork();
            // ...and the GRID, which may just have been cut in two. Any
            // cable left with a dead end is dropped in there, so a wire can
            // never outlive the thing it was tied to.
            rebuildPowerNetwork();
            return;
        }

        if (held != nullptr && m_buildCursor.verdict == sw::build::Verdict::Ok &&
            input().wasMouseButtonPressed(sw::MouseButton::Left))
        {
            // A HUB founds its own site; everything else joins the nearest.
            const sw::ecs::Entity entity = placeBuilding(
                m_heldBuilding, body, m_buildCursor.direction, m_buildYaw,
                defaultRecipeFor(held->building.category),
                siteNear(body, m_buildCursor.direction),
                (held->building.category == sw::factory::BuildingCategory::Beacon)
                    ? sw::Vec4{1.0f, 0.78f, 0.28f, 1.0f}
                    : sw::Vec4{});
            if (!entity.isNull())
            {
                if (held->building.category == sw::factory::BuildingCategory::Hub)
                {
                    sw::factory::SiteComponent site{};
                    std::snprintf(site.name, sizeof(site.name), "SITE %u", entity.index);
                    site.body = body;
                    m_world.addComponent(entity, site);
                    m_world.getComponent<sw::factory::BuildingComponent>(entity).site =
                        entity;
                }
                if (held->building.category == sw::factory::BuildingCategory::Beacon)
                {
                    sw::factory::BeaconComponent beacon{};
                    std::snprintf(beacon.label, sizeof(beacon.label), "BEACON %u",
                                  entity.index);
                    m_world.addComponent(entity, beacon);
                }
                SW_LOG_INFO("Game", "BUILT {} at {:.0f} m", held->name,
                            m_buildCursor.rangeM);
                rebuildConveyorNetwork();
                rebuildPowerNetwork(); // a new building is its own grid of one
            }
        }
    }

    // The cable preview: the span you are about to get, hung exactly as
    // `hangCable` will hang it, in the colour of the verdict. Same curve
    // function as the real thing, so what you are shown IS what you build.
    void StarWorksGame::collectCableGhost(const sw::Camera& activeCamera)
    {
        if (m_cableSource.isNull() || m_buildCursor.target.isNull() ||
            m_cableMeshIndex == 0xFFFFFFFFu)
        {
            return;
        }
        sw::WorldVec3 from{};
        sw::WorldVec3 to{};
        const sw::factory::CableVerdict verdict =
            planCable(m_cableSource, m_buildCursor.target, from, to);
        if (glm::length(to - from) < 1.0e-6)
        {
            return;
        }
        CableComponent preview{};
        preview.body = m_buildCursor.body;
        hangCable(preview, from, to);

        sw::WorldVec3 bodyPosition{};
        glm::dquat bodyRotation{};
        bodyRenderPose(m_buildCursor.body, bodyPosition, bodyRotation);
        const sw::Vec4 tint = (verdict == sw::factory::CableVerdict::Ok)
                                  ? sw::Vec4{0.35f, 1.0f, 0.45f, 0.55f}
                                  : sw::Vec4{1.0f, 0.35f, 0.30f, 0.45f};

        for (sw::u32 i = 0; i + 1 < preview.pointCount; ++i)
        {
            const sw::WorldVec3 a = bodyPosition + bodyRotation * preview.points[i];
            const sw::WorldVec3 b = bodyPosition + bodyRotation * preview.points[i + 1];
            const sw::WorldVec3 delta = b - a;
            const sw::f64 length = glm::length(delta);
            if (length < 1.0e-3)
            {
                continue;
            }
            const sw::Vec3 forward = sw::Vec3(delta / length);
            const sw::Vec3 hint = sw::Vec3(
                glm::normalize(bodyRotation * glm::normalize(a - bodyPosition)));
            sw::Vec3 right = glm::cross(forward, hint);
            if (glm::length(right) < 1.0e-4f)
            {
                right = glm::cross(forward, sw::Vec3{0.0f, 0.0f, 1.0f});
            }
            right = glm::normalize(right);
            const sw::Vec3 realUp = glm::cross(right, forward);

            const sw::Vec3 relative = sw::Vec3((a + b) * 0.5 - activeCamera.position());
            sw::DrawItem item{};
            item.mesh = &m_meshes[m_cableMeshIndex];
            item.transform =
                glm::translate(sw::Mat4{1.0f}, relative) *
                glm::mat4_cast(glm::quat_cast(sw::Mat3{right, realUp, -forward})) *
                glm::scale(sw::Mat4{1.0f},
                           sw::Vec3{2.2f, 2.2f,
                                    static_cast<sw::f32>(length) / m_cableSegmentM});
            item.boundsCenter = relative;
            item.boundsRadius = static_cast<sw::f32>(length);
            item.tint = tint;
            item.transparent = true;
            m_drawItems.push_back(item);
        }
    }

    void StarWorksGame::collectBuildGhost(const sw::Camera& activeCamera)
    {
        if (!m_buildCursor.active || m_heldBuilding == 0)
        {
            return;
        }
        const auto* held = sw::parts::findDefinition(m_heldBuilding);
        const auto meshIt = m_partMeshIds.find(m_heldBuilding);
        if (held == nullptr || meshIt == m_partMeshIds.end())
        {
            return;
        }
        const bool beltMode =
            held->building.category == sw::factory::BuildingCategory::Conveyor;
        if (beltMode && m_beltPreview.empty())
        {
            return; // nothing picked yet, or nothing to show
        }
        // A CABLE has no ground ghost to draw: it is not placed on a spot,
        // it is strung between two nodes. Its preview is the span itself,
        // drawn below once both ends are known.
        if (held->building.category == sw::factory::BuildingCategory::Cable)
        {
            collectCableGhost(activeCamera);
            return;
        }
        const auto* gravity =
            m_world.tryGetComponent<sw::phys::GravitySourceComponent>(m_buildCursor.body);
        const auto* terrain =
            m_world.tryGetComponent<sw::planet::TerrainComponent>(m_buildCursor.body);
        const auto* bodyTransform =
            m_world.tryGetComponent<TransformComponent>(m_buildCursor.body);
        if (gravity == nullptr || terrain == nullptr || bodyTransform == nullptr)
        {
            return;
        }

        // The RENDERED pose — the ghost has to sit on the ground the player
        // can see, which is the interpolated one.
        sw::WorldVec3 bodyPosition{};
        glm::dquat spin{};
        bodyRenderPose(m_buildCursor.body, bodyPosition, spin);

        const bool ok = beltMode ? (m_beltVerdict == sw::build::Verdict::Ok)
                                 : (m_buildCursor.verdict == sw::build::Verdict::Ok);
        const sw::Vec4 tint = ok ? sw::Vec4{0.35f, 1.0f, 0.45f, 0.45f}
                                 : sw::Vec4{1.0f, 0.35f, 0.30f, 0.40f};

        auto pushGhost = [&](const sw::Vec3& up, sw::f32 yaw) {
            const sw::f64 elevation = sw::planet::terrainElevation(*terrain, up);
            const sw::WorldVec3 world =
                bodyPosition +
                spin * (sw::WorldVec3(up) * (gravity->bodyRadius + elevation));
            const sw::Vec3 relative = sw::Vec3(world - activeCamera.position());
            sw::DrawItem item{};
            item.mesh = &m_meshes[meshIt->second];
            item.transform =
                glm::translate(sw::Mat4{1.0f}, relative) *
                glm::mat4_cast(sw::Quat(spin) * standUpFor(up) *
                               glm::angleAxis(yaw, sw::Vec3{0.0f, 1.0f, 0.0f}));
            item.boundsCenter = relative;
            item.boundsRadius = sw::parts::partBoundsRadius(*held) + 0.5f;
            item.tint = tint;
            item.transparent = true;
            m_drawItems.push_back(item);
        };

        if (beltMode)
        {
            // The WHOLE RUN, previewed. What you are shown in green is what
            // planBelt will hand to placeBuilding — the same tiles, from the
            // same call — so a belt cannot come out different when you click.
            for (const BeltTile& tile : m_beltPreview)
            {
                pushGhost(tile.direction, tile.yawRadians);
            }
            return;
        }
        pushGhost(m_buildCursor.direction, m_buildCursor.yawRadians);
    }

    // ------------------------------------------------------------------------
    // CARGO ON THE BELTS
    //
    // No item entities, no second simulation: the crates are a closed-form
    // function of the lane's present time and the link's MEASURED
    // throughput. That is the same discipline the orbits and the planet
    // spin already follow, and it buys the same three things — it is exact
    // under time warp, it costs nothing when nobody is looking, and it
    // cannot drift away from the matter it depicts.
    //
    // Spacing IS the flow: crates/second = flow / unitsPerCrate, so a belt
    // fed by a starving mine visibly thins out and a stopped one empties.
    // ------------------------------------------------------------------------
    // ------------------------------------------------------------------------
    // THE CABLES
    //
    // A span is one entity, not a row of them — you do not demolish a metre
    // of wire — so unlike the belt deck there is no per-tile entity for the
    // world pass to draw, and the curve is stroked here.
    //
    // It goes through `bodyRenderPose` for the same reason the build ghost
    // and the belt cargo do: the endpoints are in the body's ROTATING frame,
    // and transforming them with the planet's tick pose instead of the pose
    // it is being DRAWN at would hang every wire up to 595 m off its poles,
    // resetting every tick. That rule has now cost three features; it is
    // cheaper to obey it than to rediscover it.
    // ------------------------------------------------------------------------
    // F2 — WHAT YOU ACTUALLY BUMP INTO.
    //
    // Every solid hull, drawn as the boxes it is. Green for the things that
    // stand still, amber for the things that get pushed out of them, so the
    // player's own box is never confused with the world's. Belts and cables
    // draw nothing, because they have no hull — which is itself the fastest
    // way to confirm they are walk-through.
    void StarWorksGame::collectHullOverlay(const sw::Camera& activeCamera)
    {
        if (!m_showHitboxes || m_hullBoxMeshIndex == 0xFFFFFFFFu)
        {
            return;
        }
        const sw::WorldVec3 cameraPosition = activeCamera.position();
        // THE RENDERED POSE, not the tick pose. A building's TransformComponent
        // is where it was at the last physics step; the mesh over it is drawn
        // at the interpolated pose, and on Terra one step of orbital motion is
        // 595 m. Reading the raw transform here drew every box swimming beside
        // its own machine and snapping back every tick — the same mistake that
        // has now cost the conveyor cargo, the build ghost and the cables, so
        // this uses the identical mix the mesh pass uses rather than something
        // that merely looks equivalent.
        const sw::f32 alpha = m_physicsLane->alpha();
        const sw::f64 alpha64 = static_cast<sw::f64>(alpha);

        m_world.forEach<TransformComponent, PreviousTransformComponent,
                        sw::phys::HullComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                PreviousTransformComponent& previous, sw::phys::HullComponent& hull) {
                const sw::WorldVec3 position =
                    glm::mix(previous.position, transform.position, alpha64);
                const sw::Quat rotation =
                    glm::slerp(previous.rotation, transform.rotation, alpha);

                const sw::WorldVec3 offset = position - cameraPosition;
                // Same broad phase as the collision itself: a box a
                // kilometre away is not a box you are inspecting.
                if (glm::dot(offset, offset) > 1.0e6)
                {
                    return;
                }
                const bool mover =
                    m_world.hasComponent<sw::phys::HullMoverComponent>(entity);
                const sw::Vec4 tint = mover ? sw::Vec4{1.0f, 0.72f, 0.20f, 0.30f}
                                            : sw::Vec4{0.30f, 1.0f, 0.55f, 0.22f};
                for (sw::u32 i = 0; i < hull.count; ++i)
                {
                    const sw::Vec3 relative =
                        sw::Vec3(offset) + rotation * hull.boxes[i].centre;
                    sw::DrawItem item{};
                    item.mesh = &m_meshes[m_hullBoxMeshIndex];
                    item.transform =
                        glm::translate(sw::Mat4{1.0f}, relative) *
                        glm::mat4_cast(rotation) *
                        // The cube is 1 m across, so its half extent is 0.5:
                        // scaling by the full extent is what makes the drawn
                        // box the same size as the one being tested.
                        glm::scale(sw::Mat4{1.0f},
                                   glm::max(hull.boxes[i].halfExtents * 2.0f,
                                            sw::Vec3{0.02f}));
                    item.boundsCenter = relative;
                    item.boundsRadius = glm::length(hull.boxes[i].halfExtents) + 0.5f;
                    item.tint = tint;
                    item.transparent = true;
                    m_drawItems.push_back(item);
                }
            });
    }

    void StarWorksGame::collectCables(const sw::Camera& activeCamera)
    {
        if (m_cableMeshIndex == 0xFFFFFFFFu)
        {
            return;
        }
        const sw::WorldVec3 cameraPosition = activeCamera.position();

        m_world.forEach<sw::factory::PowerLinkComponent, CableComponent>(
            [&](sw::ecs::Entity, sw::factory::PowerLinkComponent& link,
                CableComponent& cable) {
            if (cable.pointCount < 2 || cable.body.isNull())
            {
                return;
            }
            // Is this wire's grid short? Read it off either end — they are
            // on the same grid by construction, that being what a cable is.
            if (const auto* power =
                    m_world.tryGetComponent<sw::factory::PowerComponent>(link.a))
            {
                cable.starved = power->gridProducedKw + 1.0e-9 < power->gridConsumedKw;
            }
            sw::WorldVec3 bodyPosition{};
            glm::dquat bodyRotation{};
            bodyRenderPose(cable.body, bodyPosition, bodyRotation);

            // A wire is 11 cm across. Past a couple of kilometres it is not
            // a pixel, and stroking twelve segments of it is pure cost.
            const sw::WorldVec3 anchor = bodyPosition + bodyRotation * cable.points[0];
            if (glm::length(anchor - cameraPosition) > 3000.0)
            {
                return;
            }

            // A browning-out grid dims its own wires. It is the cheapest
            // possible answer to "why is my smelter stopped" that does not
            // involve opening a panel.
            const sw::Vec4 tint = cable.starved ? sw::Vec4{0.62f, 0.32f, 0.26f, 1.0f}
                                                : sw::Vec4{1.0f, 1.0f, 1.0f, 1.0f};

            for (sw::u32 i = 0; i + 1 < cable.pointCount; ++i)
            {
                const sw::WorldVec3 a = bodyPosition + bodyRotation * cable.points[i];
                const sw::WorldVec3 b = bodyPosition + bodyRotation * cable.points[i + 1];
                const sw::WorldVec3 delta = b - a;
                const sw::f64 length = glm::length(delta);
                if (length < 1.0e-3)
                {
                    continue;
                }
                // The part's cylinder runs along its own Z, and model -Z is
                // "forward" everywhere in this codebase, so the segment is
                // aimed the way a belt tile is and stretched to fit.
                const sw::Vec3 forward = sw::Vec3(delta / length);
                const sw::Vec3 hint = sw::Vec3(
                    glm::normalize(bodyRotation * glm::normalize(a - bodyPosition)));
                sw::Vec3 right = glm::cross(forward, hint);
                if (glm::length(right) < 1.0e-4f)
                {
                    right = glm::cross(forward, sw::Vec3{0.0f, 0.0f, 1.0f});
                }
                right = glm::normalize(right);
                const sw::Vec3 realUp = glm::cross(right, forward);

                const sw::Vec3 relative = sw::Vec3((a + b) * 0.5 - cameraPosition);
                sw::DrawItem item{};
                item.mesh = &m_meshes[m_cableMeshIndex];
                item.transform =
                    glm::translate(sw::Mat4{1.0f}, relative) *
                    glm::mat4_cast(glm::quat_cast(sw::Mat3{right, realUp, -forward})) *
                    glm::scale(sw::Mat4{1.0f},
                               sw::Vec3{1.0f, 1.0f,
                                        static_cast<sw::f32>(length) / m_cableSegmentM});
                item.boundsCenter = relative;
                item.boundsRadius = static_cast<sw::f32>(length);
                item.tint = tint;
                m_drawItems.push_back(item);
            }
        });
    }

    void StarWorksGame::collectConveyors(const sw::Camera& activeCamera)
    {
        const sw::WorldVec3 cameraPosition = activeCamera.position();
        // EVERY POSE ON SCREEN IS INTERPOLATED, and cargo is no exception.
        // The first version read the belt's TICK pose while the deck under
        // it was drawn at the interpolated one, and the two are a full
        // physics step apart — which for a planet moving 30 km/s around its
        // star is up to 595 metres. The crates were not floating up: they
        // were being drawn where the belt had been (or would be) one tick
        // away, in whatever direction Terra's orbit happened to point.
        const sw::f32 alpha = m_physicsLane->alpha();
        const sw::f64 alpha64 = static_cast<sw::f64>(alpha);
        // The cargo's own phase gets the same treatment: `presentSeconds` is
        // quantised to the tick, so using it raw makes crates advance in
        // 5 cm hops instead of gliding.
        const sw::f64 now = m_physicsLane->presentSeconds() +
                            alpha64 * static_cast<sw::f64>(m_physicsLane->stepSeconds());

        m_world.forEach<TransformComponent, PreviousTransformComponent,
                        ConveyorComponent>(
            [&](sw::ecs::Entity, TransformComponent& transform,
                PreviousTransformComponent& previous, ConveyorComponent& conveyor) {
                if (conveyor.pointCount < 2 || conveyor.lengthM < 1.0f)
                {
                    return;
                }
                // The belt's pose THIS FRAME — the same mix the deck mesh is
                // drawn with, so the cargo can only ever be on the deck.
                const sw::WorldVec3 beltPosition =
                    glm::mix(previous.position, transform.position, alpha64);
                const glm::dquat beltRotation{
                    glm::slerp(previous.rotation, transform.rotation, alpha)};

                // Belts are metres wide: past a few kilometres they are not
                // even a pixel, and their cargo certainly is not.
                const sw::f64 distance = glm::length(beltPosition - cameraPosition);
                if (distance > 4000.0)
                {
                    return;
                }

                // The path in world space, rebuilt from the belt's own pose:
                // its anchor already carries the body's f64 rotation, so the
                // cargo rides exactly where the deck was drawn.
                const sw::WorldVec3 origin = conveyor.points[0];

                // Model -Z is the direction of travel and +Y is up: one
                // frame for the deck tiles and the crates alike.
                auto placeAlong = [&](sw::f64 arcLength, sw::f32 rideHeight,
                                      sw::f32 stretchZ, sw::u32 meshIndex,
                                      const sw::Vec4& tint, sw::f32 boundsRadius) {
                    sw::WorldVec3 local{};
                    sw::Vec3 heading{};
                    sw::factory::conveyorPointAt(conveyor.points, conveyor.pointCount,
                                                 arcLength, local, heading);
                    // `local` and `up` are BODY-FRAME; the ride height is
                    // added there, before the rotation, or the part is
                    // lifted along a world axis instead of the local one.
                    const glm::dvec3 up = glm::normalize(local);
                    const sw::WorldVec3 world =
                        beltPosition +
                        beltRotation * (local - origin + up * static_cast<sw::f64>(
                                                                  rideHeight));
                    const sw::Vec3 relative = sw::Vec3(world - cameraPosition);
                    const sw::Vec3 forward =
                        sw::Vec3(glm::normalize(beltRotation * glm::dvec3(heading)));
                    const sw::Vec3 worldUp =
                        sw::Vec3(glm::normalize(beltRotation * up));
                    const sw::Vec3 right = glm::normalize(glm::cross(forward, worldUp));
                    const sw::Vec3 realUp = glm::cross(right, forward);

                    sw::DrawItem item{};
                    item.mesh = &m_meshes[meshIndex];
                    item.transform =
                        glm::translate(sw::Mat4{1.0f}, relative) *
                        glm::mat4_cast(
                            glm::quat_cast(sw::Mat3{right, realUp, -forward})) *
                        glm::scale(sw::Mat4{1.0f}, sw::Vec3{1.0f, 1.0f, stretchZ});
                    item.boundsCenter = relative;
                    item.boundsRadius = boundsRadius;
                    item.tint = tint;
                    m_drawItems.push_back(item);
                };

                // The DECK is not drawn here any more: since F2 a belt is a
                // row of ordinary CV-1 building entities, and they are drawn
                // by the same pass as every other mesh in the world. What is
                // left for this function is the one thing no entity holds —
                // the cargo, which is a function of time and flow.
                // ---- and only NOW, what is riding it ---------------------
                // The deck above is drawn UNCONDITIONALLY, because a belt is
                // a structure: it exists whether or not goods are on it.
                // That was the bug — the whole conveyor sat behind the flow
                // gate below, so every time the link's source ran dry for a
                // tick the belt itself blinked out of the world.
                const sw::u32 cargoMesh = (conveyor.cargoMesh != 0xFFFFFFFFu)
                                              ? conveyor.cargoMesh
                                              : m_cargoMeshIndex;
                if (cargoMesh == 0xFFFFFFFFu)
                {
                    return;
                }
                const auto* link =
                    m_world.tryGetComponent<sw::factory::ItemLinkComponent>(
                        conveyor.link);
                // Everything this belt's source is putting on it, whatever
                // that is: two gases down one run are two streams of crates.
                const sw::f64 flow =
                    (link != nullptr)
                        ? sw::factory::linkFlowFrom(*link, conveyor.source)
                        : 0.0;
                if (flow <= 1.0e-6)
                {
                    return; // stopped: an empty belt is the honest picture
                }
                const sw::f64 cratesPerSecond =
                    flow / std::max(static_cast<sw::f64>(conveyor.unitsPerCrate), 1.0e-6);
                const sw::f64 spacing =
                    static_cast<sw::f64>(conveyor.speedMps) / cratesPerSecond;
                if (spacing < 0.35)
                {
                    return; // shoulder to shoulder: draw the deck, not confetti
                }
                const sw::i32 crates =
                    std::min(32, static_cast<sw::i32>(conveyor.lengthM / spacing));
                if (crates <= 0)
                {
                    return;
                }
                const sw::f64 travelled =
                    std::fmod(now * static_cast<sw::f64>(conveyor.speedMps),
                              static_cast<sw::f64>(conveyor.lengthM));

                // ---- THE CARGO: the CR-1 prop, tinted per resource -------
                // It rides ON the deck, so its height is the deck's own top
                // surface — read off the CV-1's collider box, not guessed.
                for (sw::i32 c = 0; c < crates; ++c)
                {
                    const sw::f64 s =
                        std::fmod(travelled + static_cast<sw::f64>(c) * spacing,
                                  static_cast<sw::f64>(conveyor.lengthM));
                    placeAlong(s, m_conveyorDeckHeightM, 1.0f, cargoMesh,
                               {conveyor.cargoColor.r, conveyor.cargoColor.g,
                                conveyor.cargoColor.b, 1.0f},
                               (cargoMesh == m_vehicleCargoMeshIndex) ? 4.4f : 0.6f);
                }
            });
    }

    // ------------------------------------------------------------------------
    // THE BUILD MENU (F)
    //
    // Satisfactory's lesson, applied: the catalogue of things you can put on
    // the ground is a first-class screen, not a submenu of a vehicle editor.
    // The VAB (B) assembles ROCKETS out of parts; this assembles a FACTORY
    // out of buildings, and the two never share a palette because a refinery
    // is not something you bolt to a fuel tank.
    //
    // What it does today is arm a definition — `m_heldBuilding` — and show
    // what that building costs and needs. F2 turns that armed id into a
    // ghost on the terrain and a placement; nothing here will have to move
    // when it does, because the id IS the contract.
    // ------------------------------------------------------------------------
    void StarWorksGame::collectBuildMenu()
    {
        m_hudButtons.clear();

        std::vector<const sw::parts::PartDefinition*> buildings;
        for (const sw::parts::PartDefinition& definition : sw::parts::catalog())
        {
            if (sw::parts::isBuilding(definition))
            {
                buildings.push_back(&definition);
            }
        }

        constexpr sw::f32 kLeft = -0.62f;
        constexpr sw::f32 kRight = 0.62f;
        constexpr sw::f32 kTop = -0.56f;
        constexpr sw::f32 kRowHeight = 0.076f;
        constexpr sw::f32 kRowGap = 0.008f;
        constexpr sw::f32 kHeaderH = 0.090f;
        constexpr sw::f32 kFooterH = 0.115f;
        constexpr sw::f32 kPad = 0.018f;

        sw::f32 cursorX = -2.0f;
        sw::f32 cursorY = -2.0f;
        const bool haveCursor = hudCursor(cursorX, cursorY);
        auto hovering = [&](sw::f32 y) {
            return haveCursor && cursorX >= kLeft + kPad && cursorX <= kRight - kPad &&
                   cursorY >= y && cursorY <= y + kRowHeight;
        };

        const sw::f32 listTop = kTop + kHeaderH;
        const sw::f32 bottom =
            listTop + static_cast<sw::f32>(buildings.size()) * (kRowHeight + kRowGap) +
            kFooterH;
        hudPanel(kLeft, kTop, kRight, bottom, hud::kPanel);
        hudQuad(kLeft, kTop, kRight, listTop - 0.006f, hud::kHeader);
        hudText("BUILD", kLeft + 0.025f, kTop + 0.028f, 0.052f, hud::kTitle);
        hudText("CLICK TO ARM   F CLOSE", kRight - 0.34f, kTop + 0.040f, 0.028f,
                hud::kTextDim);

        sw::f32 rowY = listTop;
        for (sw::usize i = 0; i < buildings.size(); ++i)
        {
            const sw::parts::PartDefinition& definition = *buildings[i];
            const sw::parts::BuildingSpec& spec = definition.building;
            const bool armed = definition.id == m_heldBuilding;
            const bool hot = hovering(rowY);

            const sw::Vec4 fill = armed ? (hot ? hud::kRowOnHover : hud::kRowOn)
                                        : (hot ? hud::kRowHover
                                               : ((i % 2 == 0) ? hud::kRow : hud::kRowAlt));
            hudQuad(kLeft + kPad, rowY, kRight - kPad, rowY + kRowHeight, fill);
            // The category chip: a colour band down the left edge of the row.
            hudQuad(kLeft + kPad, rowY, kLeft + kPad + 0.012f, rowY + kRowHeight,
                    hud::categoryColor(spec.category));

            hudText(hud::caps(definition.name), kLeft + kPad + 0.028f, rowY + 0.012f,
                    0.036f, armed ? hud::kTitle : hud::kText);
            hudText(hud::caps(std::string(sw::factory::categoryName(spec.category))),
                    kLeft + kPad + 0.028f, rowY + 0.050f, 0.024f,
                    hud::categoryColor(spec.category));

            // The spec, read straight off the .swpart: how much ground it
            // needs, and what it does to the grid.
            std::string summary =
                std::format("{:.0f}X{:.0f} M   {}", spec.footprintM[0], spec.footprintM[1],
                            hud::powerText(spec.powerKw));
            if (spec.inventoryVolumeM3 > 0.0)
            {
                summary += std::format("   {:.0f} M3", spec.inventoryVolumeM3);
            }
            if (spec.minOreDensity > 0.0)
            {
                summary += std::format("   ORE {:.2f}", spec.minOreDensity);
            }
            hudText(summary, kRight - 0.44f, rowY + 0.020f, 0.028f,
                    armed ? hud::kText : hud::kTextDim);
            const sw::usize recipes =
                sw::factory::recipesForCategory(spec.category).size();
            if (recipes > 0)
            {
                hudText(std::format("{} RECIPES   SLOPE {:.2f}", recipes,
                                    spec.maxSlopeTangent),
                        kRight - 0.44f, rowY + 0.052f, 0.024f, hud::kTextDim);
            }

            m_hudButtons.push_back({kLeft + kPad, rowY, kRight - kPad, rowY + kRowHeight,
                                    400u + static_cast<sw::u32>(definition.id)});
            rowY += kRowHeight + kRowGap;
        }

        if (buildings.empty())
        {
            hudText("NO BUILDINGS IN THE CATALOG", kLeft + 0.04f, rowY + 0.02f, 0.034f,
                    hud::kBad);
            return;
        }

        // The footer: what is in your hand, and what it wants next.
        rowY += 0.012f;
        hudQuad(kLeft + kPad, rowY, kRight - kPad, bottom - 0.014f, hud::kHeader);
        if (const auto* held = sw::parts::findDefinition(m_heldBuilding);
            held != nullptr && sw::parts::isBuilding(*held))
        {
            const bool belt = held->building.category ==
                              sw::factory::BuildingCategory::Conveyor;
            const bool cable =
                held->building.category == sw::factory::BuildingCategory::Cable;
            hudText(std::format("ARMED   {}", hud::caps(held->name)), kLeft + 0.04f,
                    rowY + 0.014f, 0.036f, hud::kOk);
            hudText(belt    ? "PICK AN OUTPUT, THEN AN INPUT"
                    : cable ? "PICK TWO POWER NODES"
                            : "LOOK AT THE GROUND AND CLICK   WHEEL ROTATES",
                    kLeft + 0.04f, rowY + 0.058f, 0.028f, hud::kTextDim);
        }
        else
        {
            hudText("NOTHING ARMED", kLeft + 0.04f, rowY + 0.014f, 0.036f, hud::kTextDim);
            hudText("PICK A BUILDING ABOVE", kLeft + 0.04f, rowY + 0.058f, 0.028f,
                    hud::kTextDim);
        }
    }

    // ======================= THE MACHINE PANEL (E) =========================
    //
    // F3's premise is that a building is a GENERIC executor and its recipe is
    // a choice. Somewhere that choice has to be made, and the honest place is
    // standing in front of the machine: E, near it, and the panel is the
    // machine's own front plate — what it is doing, why it is not, what the
    // grid is giving it, what is in the bin, and the list of jobs its
    // category knows how to do.
    //
    // Everything on it is READ from the components, nothing is cached. A
    // panel that agreed with the simulation only at the moment it opened
    // would be worse than no panel.
    // ------------------------------------------------------------------------
    void StarWorksGame::toggleConfigMenu()
    {
        if (!m_configTarget.isNull())
        {
            m_configTarget = {};
            return;
        }
        // On foot, near a building, nothing else in the way.
        if (!m_evaMode || m_mapView || m_editorMode || m_buildMenu ||
            m_capsuleEntity.isNull())
        {
            return;
        }
        const auto* player = m_world.tryGetComponent<TransformComponent>(m_capsuleEntity);
        if (player == nullptr)
        {
            return;
        }

        // NEAR ENOUGH, AND IN FRONT OF YOU — asked of the machine's actual
        // HULL rather than of its centre. The old rule took the nearest
        // centre inside 18 m, which is wrong twice over: a 16 m solar field
        // you are standing on has a centre 8 m away and loses to a silo
        // behind your shoulder, and a machine you are touching can have its
        // centre out of range entirely. A ray from the eye against the
        // hitboxes answers the real question exactly, and reuses the very
        // boxes you cannot walk through.
        (void)player;
        const sw::ecs::Entity best = hullUnderCrosshair(kConfigRangeM);
        if (best.isNull())
        {
            SW_LOG_INFO("Game", "E: nothing in front of you within {:.0f} m",
                        kConfigRangeM);
            m_configTarget = {};
            return;
        }

        // A ROCKET IS SOMETHING YOU LOOK AT AND PRESS E ON, exactly like a
        // machine. `P` still cycles between vessels once you are flying, but
        // getting IN should not be a key that means something else — you are
        // standing in front of the thing, which is the whole gesture.
        if (const auto* part = m_world.tryGetComponent<sw::parts::PartComponent>(best))
        {
            const sw::ecs::Entity vessel = part->vessel;
            if (!vessel.isNull() && m_world.isAlive(vessel) &&
                m_world.hasComponent<ShipComponent>(vessel))
            {
                m_shipEntity = vessel;
                m_evaMode = false;
                m_sasMode = 0;
                m_configTarget = {};
                SW_LOG_INFO("Game", "Boarded vessel {}", vessel.index);
                return;
            }
            m_configTarget = {}; // a part of something that is not a craft
            return;
        }
        m_configTarget = best;
    }

    void StarWorksGame::collectConfigMenu()
    {
        m_hudButtons.clear();

        const auto* building =
            m_world.tryGetComponent<sw::factory::BuildingComponent>(m_configTarget);
        const auto* definition =
            (building != nullptr) ? sw::parts::findDefinition(building->definitionId)
                                  : nullptr;
        if (building == nullptr || definition == nullptr)
        {
            m_configTarget = {}; // demolished under our feet
            return;
        }
        auto* state =
            m_world.tryGetComponent<sw::factory::RecipeStateComponent>(m_configTarget);
        auto* power = m_world.tryGetComponent<sw::factory::PowerComponent>(m_configTarget);
        const auto* inventory =
            m_world.tryGetComponent<sw::factory::InventoryComponent>(m_configTarget);

        const std::vector<sw::u32> recipes =
            sw::factory::recipesForCategory(building->category);

        // THE VAB'S PANEL IS THIS PANEL. An assembly hall runs no recipes;
        // what it offers instead is the list of designs on disk, priced.
        // Everything else — the state tab, the grid figures, the bin, the
        // priority control — is the same machine panel it always was, which
        // is the whole reason the VAB needed no screen of its own.
        auto* assembly =
            m_world.tryGetComponent<sw::factory::AssemblyComponent>(m_configTarget);
        const bool hall = (assembly != nullptr);
        const std::span<const sw::parts::ShipBlueprint> designs =
            hall ? sw::parts::blueprintCatalog()
                 : std::span<const sw::parts::ShipBlueprint>{};
        // A panel is only a screen tall. Eight designs is already a taller
        // list than any other panel in the game; past that the list is
        // capped and SAYS it is capped, rather than growing off the bottom
        // of the window where the rows cannot be clicked.
        constexpr sw::usize kMaxDesignRows = 8;
        const sw::usize shownDesigns = std::min(designs.size(), kMaxDesignRows);
        const sw::usize listRows = hall ? shownDesigns : recipes.size();
        const bool listEmpty = (listRows == 0);

        constexpr sw::f32 kLeft = -0.68f;
        constexpr sw::f32 kRight = 0.68f;
        constexpr sw::f32 kTop = -0.62f;
        constexpr sw::f32 kRowHeight = 0.072f;
        constexpr sw::f32 kRowGap = 0.008f;
        constexpr sw::f32 kPad = 0.018f;
        constexpr sw::f32 kHeaderH = 0.086f;
        // A hall and a pad each carry one line no other machine has — what
        // is on the slipway, and whether the deck is clear — so their stats
        // block is taller by exactly that line. Leaving it at 0.168 drew
        // that line OUTSIDE its own background, which is the same class of
        // fault as the RECIPE label through the STOP row.
        const bool extraStatsLine =
            hall || building->category == sw::factory::BuildingCategory::Pad;
        const sw::f32 kStatsH = extraStatsLine ? 0.208f : 0.168f;
        constexpr sw::f32 kFooterH = 0.098f;

        sw::f32 cursorX = -2.0f;
        sw::f32 cursorY = -2.0f;
        const bool haveCursor = hudCursor(cursorX, cursorY);
        auto hovering = [&](sw::f32 y, sw::f32 x1) {
            return haveCursor && cursorX >= kLeft + kPad && cursorX <= x1 &&
                   cursorY >= y && cursorY <= y + kRowHeight;
        };

        const sw::f32 listTop = kTop + kHeaderH + kStatsH;
        // The list's own header eats `kLabelH` before the first row: at
        // 0.032 the word RECIPE was drawn straight through the STOP row
        // under it, because a label needs its own height AND the gap.
        constexpr sw::f32 kLabelH = 0.050f;
        // THE CATALOGUE COLUMN. A hall's panel is split: the list of saved
        // designs on the left, and on the right the one that is selected —
        // drawn, priced and weighed — with the button that actually builds
        // it. A name and a tonne figure is not enough to choose a rocket by.
        constexpr sw::f32 kCatalogueSplit = 0.10f;   // where the column starts
        constexpr sw::f32 kPreviewH = 0.40f;         // the 3D box
        constexpr sw::f32 kStatLine = 0.044f;
        // Includes the label strip the column starts below, which the list
        // also pays for — leaving it out put the PRODUCE button through the
        // footer, which a mock of the layout showed before the code ran.
        const sw::f32 columnHeight =
            kLabelH + kPreviewH + 0.014f + 5.0f * kStatLine + 0.014f + kRowHeight;

        const sw::f32 listHeight =
            listEmpty ? 0.06f
                      : (kLabelH +
                         static_cast<sw::f32>(listRows + 1) * (kRowHeight + kRowGap));
        const sw::f32 bodyHeight = hall ? std::max(listHeight, columnHeight) : listHeight;
        const sw::f32 bottom = listTop + bodyHeight + kFooterH;
        hudPanel(kLeft, kTop, kRight, bottom, hud::kPanel);

        // ---- HEADER: who this is -------------------------------------------
        hudQuad(kLeft, kTop, kRight, kTop + kHeaderH - 0.006f, hud::kHeader);
        hudQuad(kLeft, kTop, kLeft + 0.014f, kTop + kHeaderH - 0.006f,
                hud::categoryColor(building->category));
        hudText(hud::caps(definition->name), kLeft + 0.032f, kTop + 0.026f, 0.048f,
                hud::kTitle);
        hudText(hud::caps(std::string(sw::factory::categoryName(building->category))),
                kRight - 0.34f, kTop + 0.020f, 0.026f,
                hud::categoryColor(building->category));
        hudText("E CLOSE", kRight - 0.34f, kTop + 0.052f, 0.026f, hud::kTextDim);

        // ---- STATS: what it is doing, and on what power ---------------------
        sw::f32 rowY = kTop + kHeaderH + 0.006f;
        hudQuad(kLeft + kPad, rowY, kRight - kPad, rowY + kStatsH - 0.020f,
                hud::kRow);
        rowY += 0.016f;

        const char* stateText = "IDLE";
        sw::Vec4 stateColor = hud::kTextDim;
        // A hall reports its OWN state: it is not running a recipe, it is
        // paying for a rocket, and the two are never both true.
        const sw::u32 shownState = hall            ? assembly->state
                                   : (state != nullptr) ? state->state
                                                        : 0u;
        if (hall || state != nullptr)
        {
            switch (shownState)
            {
            case sw::factory::RecipeStateComponent::kRunning:
                stateText = "RUNNING";
                stateColor = hud::kOk;
                break;
            case sw::factory::RecipeStateComponent::kStarved:
                stateText = "STARVED";
                stateColor = hud::kWarn;
                break;
            case sw::factory::RecipeStateComponent::kBlocked:
                stateText = "BLOCKED";
                stateColor = hud::kWarn;
                break;
            case sw::factory::RecipeStateComponent::kNoPower:
                stateText = "NO POWER";
                stateColor = hud::kBad;
                break;
            default:
                break;
            }
        }
        // The state gets its own coloured tab: it is the one thing on this
        // panel you should be able to read from across the room.
        hudQuad(kLeft + kPad, rowY - 0.006f, kLeft + kPad + 0.008f, rowY + 0.040f,
                stateColor);
        hudText(stateText, kLeft + kPad + 0.020f, rowY, 0.042f, stateColor);

        if (power != nullptr)
        {
            const bool short_ = power->satisfaction < 0.999;
            hudText(std::format("SUPPLY {:.0f}%", power->satisfaction * 100.0),
                    kLeft + 0.32f, rowY + 0.004f, 0.034f,
                    short_ ? hud::kBad : hud::kOk);
            hudText(std::format("PRIORITY {}", power->priority), kRight - 0.30f,
                    rowY + 0.004f, 0.034f, hud::kText);
        }
        rowY += 0.050f;

        if (power != nullptr)
        {
            // Two labelled columns, so 0 reads as a zero rather than as the
            // word PASSIVE (which belongs on a catalogue row showing ONE
            // number, not on a line that already says which side is which).
            hudText(std::format("THIS  MAKES {:.0f} KW  DRAWS {:.0f} KW",
                                power->actualProducedKw, power->consumedKw),
                    kLeft + kPad + 0.014f, rowY, 0.028f, hud::kTextDim);
            // The second column starts where the FIRST one can no longer
            // reach: a 180 kW draw is three digits, and at 0.32 the two
            // sentences were printed through each other.
            hudText(std::format("GRID {}  MAKES {:.0f} KW  DRAWS {:.0f} KW",
                                power->gridId, power->gridProducedKw,
                                power->gridConsumedKw),
                    kLeft + 0.62f, rowY, 0.028f,
                    (power->gridProducedKw + 1.0e-9 < power->gridConsumedKw)
                        ? hud::kWarn
                        : hud::kTextDim);
        }
        rowY += 0.034f;

        if (inventory != nullptr)
        {
            std::string stock;
            for (const sw::factory::InventorySlot& slot : inventory->slots)
            {
                if (slot.resource == sw::res::Resource::Count || slot.units <= 0.0)
                {
                    continue;
                }
                if (!stock.empty()) { stock += "   "; }
                stock += std::format("{} {:.0f}",
                                     hud::caps(std::string(
                                         sw::res::definition(slot.resource).name)),
                                     slot.units);
            }
            const sw::f64 used = sw::factory::inventoryVolume(*inventory);
            hudText(stock.empty()
                        ? std::format("BIN EMPTY   0 / {:.0f} M3",
                                      inventory->volumeCapacityM3)
                        : std::format("{}   {:.1f} / {:.0f} M3", stock, used,
                                      inventory->volumeCapacityM3),
                    kLeft + kPad + 0.014f, rowY, 0.028f, hud::kText);
        }

        // ---- THE ORDER, at a hall --------------------------------------------
        // The one line that is not on any other machine's panel: what is on
        // the slipway and how much of its metal has arrived. It goes where
        // the recipe's throughput would be, because it is the same question.
        if (hall)
        {
            rowY += 0.032f;
            if (assembly->blueprint[0] == '\0')
            {
                hudText("NO ORDER   PICK A DESIGN BELOW", kLeft + kPad + 0.014f, rowY,
                        0.028f, hud::kTextDim);
            }
            else
            {
                const sw::f64 progress = sw::factory::assemblyProgress(*assembly);
                hudText(std::format("BUILDING {}   {:.0f}%   IRON {:.0f}/{:.0f}   "
                                    "COPPER {:.0f}/{:.0f}   DONE {}",
                                    hud::caps(std::string(assembly->blueprint)),
                                    progress * 100.0, assembly->ironPaidKg,
                                    assembly->ironNeededKg, assembly->copperPaidKg,
                                    assembly->copperNeededKg, assembly->completed),
                        kLeft + kPad + 0.014f, rowY, 0.028f, hud::kText);
            }
        }

        // ---- THE DECK, at a pad ---------------------------------------------
        // The one thing a pad can tell you that no other building can: why
        // the crate sitting in its bin has not become a rocket.
        if (building->category == sw::factory::BuildingCategory::Pad)
        {
            rowY += 0.032f;
            const sw::f64 crates =
                (inventory != nullptr)
                    ? sw::factory::inventoryCount(*inventory, sw::res::Resource::Vehicle)
                    : 0.0;
            const bool occupied = padIsOccupied(m_configTarget);
            hudText(occupied ? std::format("DECK OCCUPIED   {:.0f} WAITING   LAUNCH OR "
                                           "MOVE THE VESSEL",
                                           crates)
                    : (crates >= 1.0)
                        ? std::format("DECK CLEAR   {:.0f} WAITING   ROLLING OUT", crates)
                        : std::string("DECK CLEAR   NOTHING WAITING"),
                    kLeft + kPad + 0.014f, rowY, 0.028f,
                    occupied ? hud::kWarn : hud::kOk);
        }

        // ---- THE JOB LIST ----------------------------------------------------
        rowY = listTop;
        if (listEmpty)
        {
            hudText(hall ? "NO DESIGNS SAVED   BUILD ONE IN THE HANGAR (B)"
                         : "THIS BUILDING RUNS NO RECIPES",
                    kLeft + kPad + 0.014f, rowY + 0.030f, 0.032f, hud::kTextDim);
        }
        else if (hall)
        {
            hudText(shownDesigns < designs.size()
                        ? std::format("DESIGN   SHOWING {} OF {}", shownDesigns,
                                      designs.size())
                        : std::string("DESIGN"),
                    kLeft + kPad + 0.004f, rowY + 0.006f, 0.026f, hud::kTextDim);
            rowY += kLabelH;

            const std::string_view current{assembly->blueprint};

            // CLEAR, first, for the same reason STOP is: a hall with no order
            // draws its idle load and nothing else, and on a battery night
            // that is a choice worth having.
            const sw::f32 listRight = kCatalogueSplit - 0.010f;
            {
                const bool selected = current.empty();
                const bool hot = hovering(rowY, listRight);
                hudQuad(kLeft + kPad, rowY, listRight, rowY + kRowHeight,
                        selected ? hud::kRowStop
                                 : (hot ? hud::kRowHover : hud::kRow));
                hudText("CLEAR ORDER", kLeft + kPad + 0.018f, rowY + 0.018f, 0.034f,
                        selected ? hud::kTitle : hud::kText);
                m_hudButtons.push_back(
                    {kLeft + kPad, rowY, listRight, rowY + kRowHeight, 899u});
                rowY += kRowHeight + kRowGap;
            }

            for (sw::usize i = 0; i < shownDesigns; ++i)
            {
                const sw::parts::ShipBlueprint& design = designs[i];
                const sw::parts::BillOfMaterials bill =
                    sw::parts::blueprintCost(design);
                const bool buildable = sw::parts::blueprintIsBuildable(design);
                const bool building_ = (current == design.name);
                const bool picked = (m_vabSelection == static_cast<sw::i32>(i));
                const bool hot = hovering(rowY, listRight);
                // TWO DIFFERENT STATES, two different colours: the one being
                // BUILT (green) and the one you are LOOKING at (highlight).
                // They are usually the same row and must not be assumed to
                // be — reading a catalogue is not placing an order.
                hudQuad(kLeft + kPad, rowY, listRight, rowY + kRowHeight,
                        building_ ? (hot ? hud::kRowOnHover : hud::kRowOn)
                        : picked  ? hud::kRowHover
                        : hot     ? hud::kRowHover
                                  : ((i % 2 == 0) ? hud::kRow : hud::kRowAlt));
                if (picked && !building_)
                {
                    hudQuad(kLeft + kPad, rowY, kLeft + kPad + 0.010f, rowY + kRowHeight,
                            hud::kTitle);
                }
                hudText(hud::caps(design.name), kLeft + kPad + 0.022f, rowY + 0.010f,
                        0.032f, (building_ || picked) ? hud::kTitle : hud::kText);
                hudText(buildable ? std::format("{:.1f} T", bill.totalKg() / 1000.0)
                                  : "NO PARTS",
                        kLeft + kPad + 0.022f, rowY + 0.042f, 0.024f,
                        buildable ? hud::kTextDim : hud::kBad);
                m_hudButtons.push_back({kLeft + kPad, rowY, listRight,
                                        rowY + kRowHeight,
                                        900u + static_cast<sw::u32>(i)});
                rowY += kRowHeight + kRowGap;
            }

            // ---- THE CATALOGUE ENTRY: what you are about to build --------
            {
                if (m_vabSelection < 0 ||
                    m_vabSelection >= static_cast<sw::i32>(shownDesigns))
                {
                    m_vabSelection = shownDesigns > 0 ? 0 : -1;
                }
                const sw::f32 columnLeft = kCatalogueSplit;
                const sw::f32 columnRight = kRight - kPad;
                sw::f32 columnY = listTop + kLabelH;

                hudQuad(columnLeft, columnY, columnRight, columnY + kPreviewH,
                        sw::Vec4{0.06f, 0.10f, 0.15f, 0.98f});

                if (m_vabSelection >= 0)
                {
                    const sw::parts::ShipBlueprint& picked =
                        designs[static_cast<sw::usize>(m_vabSelection)];
                    // Wall clock, not simulation time: the model keeps
                    // turning while the game is paused, which is exactly when
                    // somebody is reading this panel.
                    hudDesignPreview(picked, columnLeft + 0.010f, columnY + 0.010f,
                                     columnRight - 0.010f, columnY + kPreviewH - 0.010f,
                                     static_cast<sw::f32>(clock().totalSeconds()) * 0.55f);
                    columnY += kPreviewH + 0.014f;

                    const sw::parts::BillOfMaterials bill =
                        sw::parts::blueprintCost(picked);
                    const bool buildable = sw::parts::blueprintIsBuildable(picked);
                    const sw::f64 iron =
                        (inventory != nullptr)
                            ? sw::factory::inventoryCount(*inventory,
                                                          sw::res::Resource::Iron)
                            : 0.0;
                    const sw::f64 copper =
                        (inventory != nullptr)
                            ? sw::factory::inventoryCount(*inventory,
                                                          sw::res::Resource::Copper)
                            : 0.0;

                    hudText(hud::caps(picked.name), columnLeft + 0.006f, columnY, 0.040f,
                            hud::kTitle);
                    columnY += kStatLine;
                    hudText(std::format("{} PARTS   {:.1f} T DRY", picked.parts.size(),
                                        bill.totalKg() / 1000.0),
                            columnLeft + 0.006f, columnY, 0.030f, hud::kText);
                    columnY += kStatLine;
                    // Each metal against what is IN THE BIN, because that is
                    // the question the player is actually asking.
                    hudText(std::format("IRON    {:.0f} KG   HAVE {:.0f}", bill.ironKg,
                                        iron),
                            columnLeft + 0.006f, columnY, 0.030f,
                            (iron >= bill.ironKg) ? hud::kOk : hud::kWarn);
                    columnY += kStatLine;
                    hudText(std::format("COPPER  {:.0f} KG   HAVE {:.0f}", bill.copperKg,
                                        copper),
                            columnLeft + 0.006f, columnY, 0.030f,
                            (copper >= bill.copperKg) ? hud::kOk : hud::kWarn);
                    columnY += kStatLine;
                    const sw::f64 seconds =
                        (assembly->buildRateKgPerSecond > 0.0)
                            ? bill.totalKg() / assembly->buildRateKgPerSecond
                            : 0.0;
                    hudText(std::format("BUILD   {:.0f} S AT FULL POWER", seconds),
                            columnLeft + 0.006f, columnY, 0.030f, hud::kTextDim);
                    columnY += kStatLine + 0.014f;

                    // ---- PRODUCE -------------------------------------------
                    const bool hot = haveCursor && cursorX >= columnLeft &&
                                     cursorX <= columnRight && cursorY >= columnY &&
                                     cursorY <= columnY + kRowHeight;
                    hudQuad(columnLeft, columnY, columnRight, columnY + kRowHeight,
                            !buildable ? sw::Vec4{0.12f, 0.14f, 0.18f, 0.95f}
                            : hot      ? hud::kRowOnHover
                                       : hud::kRowOn);
                    hudText(buildable ? "PRODUCE" : "PARTS MISSING",
                            columnLeft + 0.026f, columnY + 0.018f, 0.036f,
                            buildable ? hud::kTitle : hud::kBad);
                    if (buildable)
                    {
                        m_hudButtons.push_back({columnLeft, columnY, columnRight,
                                                columnY + kRowHeight, 898u});
                    }
                }
                else
                {
                    hudText("PICK A DESIGN", columnLeft + 0.020f, columnY + 0.18f, 0.034f,
                            hud::kTextDim);
                }
            }
        }
        else
        {
            hudText("RECIPE", kLeft + kPad + 0.004f, rowY + 0.006f, 0.026f,
                    hud::kTextDim);
            rowY += kLabelH;

            const sw::u32 current = (state != nullptr) ? state->recipeId : 0u;

            // STOP is a job like any other: an idle machine still pays its
            // idle draw, and on a battery night that is a choice worth having.
            {
                const bool selected = (sw::factory::findRecipe(current) == nullptr);
                const bool hot = hovering(rowY, kRight - kPad);
                hudQuad(kLeft + kPad, rowY, kRight - kPad, rowY + kRowHeight,
                        selected ? hud::kRowStop
                                 : (hot ? hud::kRowHover : hud::kRow));
                hudText("STOP", kLeft + kPad + 0.018f, rowY + 0.018f, 0.036f,
                        selected ? hud::kTitle : hud::kText);
                hudText("IDLE DRAW ONLY", kLeft + 0.30f, rowY + 0.022f, 0.026f,
                        hud::kTextDim);
                m_hudButtons.push_back(
                    {kLeft + kPad, rowY, kRight - kPad, rowY + kRowHeight, 600u});
                rowY += kRowHeight + kRowGap;
            }

            sw::usize index = 0;
            for (const sw::u32 id : recipes)
            {
                const sw::factory::RecipeDefinition* recipe = sw::factory::findRecipe(id);
                if (recipe == nullptr)
                {
                    continue;
                }
                const bool selected = (id == current);
                const bool hot = hovering(rowY, kRight - kPad);
                hudQuad(kLeft + kPad, rowY, kRight - kPad, rowY + kRowHeight,
                        selected ? (hot ? hud::kRowOnHover : hud::kRowOn)
                                 : (hot ? hud::kRowHover
                                        : ((index % 2 == 0) ? hud::kRow : hud::kRowAlt)));
                hudText(hud::caps(recipe->name), kLeft + kPad + 0.018f, rowY + 0.018f,
                        0.036f, selected ? hud::kTitle : hud::kText);

                // The recipe read as a sentence: what goes in, what comes out,
                // what it costs. The whole fuel chain is legible from these.
                std::string flow;
                for (const sw::factory::Ingredient& in : recipe->inputs)
                {
                    if (in.resource == sw::res::Resource::Count ||
                        in.unitsPerSecond <= 0.0)
                    {
                        continue;
                    }
                    if (!flow.empty()) { flow += " + "; }
                    flow += std::format("{:.2f} {}", in.unitsPerSecond,
                                        hud::caps(std::string(
                                            sw::res::definition(in.resource).name)));
                }
                if (flow.empty()) { flow = "GROUND"; }
                flow += " > ";
                bool firstOut = true;
                for (const sw::factory::Ingredient& out : recipe->outputs)
                {
                    if (out.resource == sw::res::Resource::Count ||
                        out.unitsPerSecond <= 0.0)
                    {
                        continue;
                    }
                    if (!firstOut) { flow += " + "; }
                    firstOut = false;
                    flow += std::format("{:.2f} {}", out.unitsPerSecond,
                                        hud::caps(std::string(
                                            sw::res::definition(out.resource).name)));
                }
                hudText(flow, kLeft + 0.26f, rowY + 0.024f, 0.026f,
                        selected ? hud::kText : hud::kTextDim);
                hudText(std::format("{:.0f} KW", recipe->powerKw), kRight - 0.13f,
                        rowY + 0.024f, 0.028f,
                        (recipe->powerKw >= 400.0) ? hud::kWarn : hud::kTextDim);

                m_hudButtons.push_back({kLeft + kPad, rowY, kRight - kPad,
                                        rowY + kRowHeight, 610u + id});
                rowY += kRowHeight + kRowGap;
                ++index;
            }
        }

        // ---- FOOTER: the priority control -----------------------------------
        if (power != nullptr)
        {
            rowY += 0.012f;
            const sw::f32 buttonRight = kLeft + 0.40f;
            const bool hot = hovering(rowY, buttonRight);
            hudQuad(kLeft + kPad, rowY, buttonRight, rowY + kRowHeight,
                    hot ? hud::kRowHover : hud::kRowAlt);
            hudText(std::format("GRID PRIORITY  {}", power->priority),
                    kLeft + kPad + 0.018f, rowY + 0.020f, 0.032f, hud::kText);
            m_hudButtons.push_back(
                {kLeft + kPad, rowY, buttonRight, rowY + kRowHeight, 601u});
            hudText("CLICK TO CYCLE. LOWER IS SERVED FIRST", kLeft + 0.43f,
                    rowY + 0.022f, 0.026f, hud::kTextDim);
        }
    }

    // Choosing a recipe changes three things at once, and all three have to
    // move together or the site lies about itself: what the machine runs,
    // what it draws from the grid, and what its outgoing belt is carrying.
    void StarWorksGame::applyRecipeChoice(sw::ecs::Entity entity, sw::u32 recipeId)
    {
        auto* state = m_world.tryGetComponent<sw::factory::RecipeStateComponent>(entity);
        if (state == nullptr)
        {
            return;
        }
        state->recipeId = recipeId;
        state->state = sw::factory::RecipeStateComponent::kIdle;

        if (auto* power = m_world.tryGetComponent<sw::factory::PowerComponent>(entity))
        {
            const auto* building =
                m_world.tryGetComponent<sw::factory::BuildingComponent>(entity);
            const auto* definition =
                (building != nullptr)
                    ? sw::parts::findDefinition(building->definitionId)
                    : nullptr;
            const sw::f64 idleKw =
                (definition != nullptr) ? std::max(0.0, -definition->building.powerKw)
                                        : 0.0;
            power->consumedKw = idleKw;
            if (const auto* recipe = sw::factory::findRecipe(recipeId))
            {
                power->consumedKw += recipe->powerKw;
            }
        }
        // The belt out of this machine carries whatever it now makes.
        rebuildConveyorNetwork();
    }

    // The map is where you look at your fleet, so it is where you should be
    // able to change which of it you are flying. `P` already cycled; this is
    // the same action with a surface you can find without knowing it exists.
    void StarWorksGame::collectMapButtons()
    {
        m_hudButtons.clear();

        std::vector<sw::ecs::Entity> pilotable;
        m_world.forEach<ShipComponent>([&](sw::ecs::Entity entity, ShipComponent&) {
            pilotable.push_back(entity);
        });
        if (pilotable.size() < 2)
        {
            return; // one ship: nothing to cycle between
        }
        sw::usize current = 0;
        for (sw::usize i = 0; i < pilotable.size(); ++i)
        {
            if (pilotable[i] == m_shipEntity)
            {
                current = i;
            }
        }

        constexpr sw::f32 kHeight = 0.062f;
        constexpr sw::f32 kWidth = 0.235f;
        const sw::f32 x0 = -0.97f;
        const sw::f32 y0 = 0.87f;
        const sw::f32 x1 = x0 + kWidth;

        sw::DrawItem panel{};
        panel.mesh = &m_meshes[m_navLineMeshIndex]; // unit quad
        panel.transform =
            glm::translate(sw::Mat4{1.0f}, {(x0 + x1) * 0.5f, y0 + kHeight * 0.5f, 0.0f}) *
            glm::scale(sw::Mat4{1.0f}, {kWidth * 0.5f, kHeight * 0.5f, 1.0f});
        panel.screenSpace = true;
        panel.tint = {0.16f, 0.22f, 0.30f, 0.65f};
        m_drawItems.push_back(panel);

        hudText("NEXT SHIP", x0 + 0.022f, y0 + 0.015f, 0.036f,
                {0.7f, 0.8f, 0.9f, 0.95f});
        hudText(std::format("SHIP {}/{}", current + 1, pilotable.size()), x0 + 0.022f,
                y0 - 0.052f, 0.034f, {0.55f, 0.72f, 0.88f, 0.9f});

        m_hudButtons.push_back({x0, y0, x1, y0 + kHeight, 300u});
    }

    // WARP TO THE NODE. A burn planned four hours out is four hours of
    // holding the warp key and watching for the moment to let go — and
    // overshooting by one rung of the ladder costs the whole orbit. This
    // stops one minute short, which is where a pilot wants to be anyway:
    // aligned, throttle hand ready, nothing to do but wait a little.
    void StarWorksGame::collectWarpToNodeButton()
    {
        if (!m_nodeActive)
        {
            return;
        }
        constexpr sw::f32 kHeight = 0.062f;
        // Wide enough for the LONGER of the two labels: a button whose text
        // runs off its own edge when you press it is a button that looks
        // broken at the exact moment you used it.
        constexpr sw::f32 kWidth = 0.40f;
        const sw::f32 x0 = -0.97f;
        const sw::f32 y0 = 0.78f;
        const sw::f32 x1 = x0 + kWidth;
        const bool running = m_warpToSeconds > 0.0;

        sw::DrawItem panel{};
        panel.mesh = &m_meshes[m_navLineMeshIndex]; // unit quad
        panel.transform =
            glm::translate(sw::Mat4{1.0f},
                           {(x0 + x1) * 0.5f, y0 + kHeight * 0.5f, 0.0f}) *
            glm::scale(sw::Mat4{1.0f}, {kWidth * 0.5f, kHeight * 0.5f, 1.0f});
        panel.screenSpace = true;
        panel.tint = running ? sw::Vec4{0.60f, 0.42f, 0.12f, 0.85f}
                             : sw::Vec4{0.16f, 0.22f, 0.30f, 0.65f};
        m_drawItems.push_back(panel);

        hudText(running ? "WARPING   CLICK TO STOP" : "WARP TO NODE -1 MIN",
                x0 + 0.020f, y0 + 0.016f, 0.034f,
                running ? sw::Vec4{1.0f, 0.92f, 0.75f, 1.0f}
                        : sw::Vec4{0.7f, 0.8f, 0.9f, 0.95f});
        m_hudButtons.push_back({x0, y0, x1, y0 + kHeight, 301u});
    }

    void StarWorksGame::handleHudClicks()
    {
        if (!input().wasMouseButtonPressed(sw::MouseButton::Left))
        {
            return;
        }
        sw::u32 width = 0;
        sw::u32 height = 0;
        window().framebufferSize(width, height);
        if (width == 0 || height == 0)
        {
            return;
        }
        const sw::f32 ndcX = input().mouseX() / static_cast<sw::f32>(width) * 2.0f - 1.0f;
        const sw::f32 ndcY = input().mouseY() / static_cast<sw::f32>(height) * 2.0f - 1.0f;
        for (const HudButton& button : m_hudButtons)
        {
            if (ndcX >= button.x0 && ndcX <= button.x1 && ndcY >= button.y0 &&
                ndcY <= button.y1)
            {
                // The multiplayer panel owns 1000+, tested first because
                // every other range below is open-ended upward.
                if (button.id >= 1100)
                {
                    const auto index = static_cast<sw::usize>(button.id - 1100u);
                    const std::vector<sw::net::PlayerView> roster = netRoster();
                    if (index < roster.size())
                    {
                        netSyncTo(roster[index].id, roster[index].simulatedSeconds);
                    }
                    break;
                }
                if (button.id == 1000)
                {
                    netHost();
                    break;
                }
                if (button.id == 1001)
                {
                    netJoin();
                    break;
                }
                if (button.id == 1002)
                {
                    netLeave();
                    break;
                }
                if (button.id == 1003)
                {
                    m_netAddressFocused = !m_netAddressFocused;
                    break;
                }
                // ---- the machine panel (E) ----------------------------------
                // Checked FIRST: its ids sit above the build menu's, and it
                // is the panel actually on screen when they are live.
                // The VAB's rows sit ABOVE the recipe ids, and are therefore
                // tested first: a recipe id is an arbitrary small number and
                // 610+id would happily swallow 900.
                if (button.id >= 900 && !m_configTarget.isNull())
                {
                    const std::span<const sw::parts::ShipBlueprint> designs =
                        sw::parts::blueprintCatalog();
                    const sw::usize index = button.id - 900u;
                    if (index < designs.size())
                    {
                        // A row SELECTS. Ordering is one deliberate press of
                        // PRODUCE, next to the price and the picture — not a
                        // side effect of reading the catalogue.
                        m_vabSelection = static_cast<sw::i32>(index);
                    }
                    return;
                }
                if (button.id == 898 && !m_configTarget.isNull())
                {
                    const std::span<const sw::parts::ShipBlueprint> catalogue =
                        sw::parts::blueprintCatalog();
                    if (m_vabSelection >= 0 &&
                        static_cast<sw::usize>(m_vabSelection) < catalogue.size())
                    {
                        orderVehicle(m_configTarget,
                                     catalogue[static_cast<sw::usize>(m_vabSelection)]);
                    }
                    break;
                }
                if (button.id == 899 && !m_configTarget.isNull())
                {
                    if (auto* assembly =
                            m_world.tryGetComponent<sw::factory::AssemblyComponent>(
                                m_configTarget))
                    {
                        sw::factory::assemblyOrder(*assembly, {}, 0.0, 0.0);
                    }
                    return;
                }
                if (button.id >= 610 && !m_configTarget.isNull())
                {
                    applyRecipeChoice(m_configTarget, button.id - 610u);
                    return;
                }
                if (button.id == 600 && !m_configTarget.isNull())
                {
                    applyRecipeChoice(m_configTarget, 0u); // STOP
                    return;
                }
                if (button.id == 601 && !m_configTarget.isNull())
                {
                    if (auto* power = m_world.tryGetComponent<sw::factory::PowerComponent>(
                            m_configTarget))
                    {
                        power->priority = (power->priority + 1u) % 5u;
                    }
                    return;
                }
                if (button.id >= 400) // build menu: arm this building
                {
                    const sw::u32 definitionId = button.id - 400u;
                    m_heldBuilding =
                        (m_heldBuilding == definitionId) ? 0u : definitionId;
                    return;
                }
                if (button.id == 300) // map: fly the next vessel
                {
                    cyclePilotedVessel();
                    return;
                }
                if (button.id == 301) // map: warp to one minute before the node
                {
                    if (m_warpToSeconds > 0.0)
                    {
                        m_warpToSeconds = 0.0;
                        m_warpIndex = 0;
                        SW_LOG_INFO("Game", "Warp to node cancelled");
                    }
                    else if (m_nodeActive)
                    {
                        m_warpToSeconds = m_nodeTime - 60.0;
                        m_simulation.setPaused(false);
                        SW_LOG_INFO("Game", "Warping to T-60 s on the node");
                    }
                    return;
                }
                if (m_mapView)
                {
                    return; // the map owns only its own buttons
                }
                // ---- hangar actions ------------------------------------------
                if (button.id >= 200)
                {
                    if (button.id == 201 && m_blueprint.size() > 1) // UNDO
                    {
                        // Remove the last placement: the trailing part plus
                        // any trailing symmetry siblings placed with it.
                        const sw::i32 group = m_blueprint.back().symmetryGroup;
                        m_blueprint.pop_back();
                        while (group >= 0 && m_blueprint.size() > 1 &&
                               m_blueprint.back().symmetryGroup == group)
                        {
                            m_blueprint.pop_back();
                        }
                    }
                    else if (button.id == 202) { hangarNewBlueprint(); }
                    else if (button.id == 203) { hangarLoadNextVessel(); }
                            else if (button.id == 205) // symmetry cycle
                    {
                        const sw::u32 options[6] = {1, 2, 3, 4, 6, 8};
                        for (sw::usize i = 0; i < 6; ++i)
                        {
                            if (options[i] == m_symmetryCount)
                            {
                                m_symmetryCount = options[(i + 1) % 6];
                                break;
                            }
                        }
                    }
                    else if (button.id == 206) { m_showCenters = !m_showCenters; }
                    else if (button.id == 207) { hangarSaveShip(); }
                    return;
                }
                if (button.id >= 100) // palette: take the part IN HAND
                {
                    const auto partCatalog = rocketPartPalette();
                    const sw::usize index = button.id - 100;
                    if (index < partCatalog.size())
                    {
                        if (!m_blueprintBackup.empty())
                        {
                            m_blueprint = m_blueprintBackup; // drop a pending grab
                            m_blueprintBackup.clear();
                        }
                        m_heldDefinition = partCatalog[index]->id;
                        m_heldSubtree.clear();
                        m_heldRotation = {1.0f, 0.0f, 0.0f, 0.0f};
                    }
                    return;
                }
                // EVERY autopilot button toggles: clicking the lit one puts
                // the autopilot back to OFF. There is no longer a button
                // that only means "none of the others" — SAS is a mode.
                m_sasMode = (m_sasMode == button.id) ? SasComponent::kOff : button.id;
                SW_LOG_INFO("Game", "SAS mode: {}", sasModeName(m_sasMode));
                break;
            }
        }

        // ---- hangar 3D click (no button consumed it) ----------------------------
        if (m_editorMode)
        {
            if (m_heldDefinition != 0)
            {
                commitGhost(); // no-op while the ghost is red/inactive
                return;
            }
            // Empty hand: ray-pick a placed part and grab its subtree.
            sw::Vec3 origin{};
            sw::Vec3 direction{};
            editorCursorRay(origin, direction);
            sw::f32 bestT = 1.0e30f;
            sw::i32 bestPart = -1;
            for (sw::usize i = 0; i < m_blueprint.size(); ++i)
            {
                const auto* definition =
                    sw::parts::findDefinition(m_blueprint[i].definitionId);
                if (definition == nullptr)
                {
                    continue;
                }
                const sw::Quat inverseRot = glm::inverse(m_blueprint[i].localRotation);
                const sw::Vec3 localOrigin =
                    inverseRot * (origin - m_blueprint[i].localPosition);
                const sw::Vec3 localDirection = inverseRot * direction;
                sw::parts::PartRayHit hit{};
                if (sw::parts::raycastPart(*definition, localOrigin, localDirection,
                                           500.0f, hit) &&
                    hit.t < bestT)
                {
                    bestT = hit.t;
                    bestPart = static_cast<sw::i32>(i);
                }
            }
            if (bestPart >= 0)
            {
                grabPartAt(static_cast<sw::usize>(bestPart));
            }
        }
    }

    void StarWorksGame::collectNavball()
    {
        const sw::i32 primaryIndex = controlledPrimaryIndex();
        if (primaryIndex < 0)
        {
            return; // no reference vertical in deep space
        }
        const sw::ecs::Entity entity = controlledEntity();
        const auto& transform = m_world.getComponent<TransformComponent>(entity);
        const sw::f64 time = m_physicsLane->presentSeconds();

        sw::WorldVec3 primaryPosition{};
        sw::WorldVec3 primaryVelocity{};
        m_celestialIndex.stateAt(primaryIndex, time, primaryPosition, &primaryVelocity);
        const sw::WorldVec3 radial = transform.position - primaryPosition;
        const sw::f64 distance = glm::length(radial);
        if (distance <= 1.0)
        {
            return;
        }
        const sw::Vec3 up = sw::Vec3(radial / distance);

        // ---- attitude vs the local horizon --------------------------------------
        const sw::Quat rotation = transform.rotation;
        const sw::Vec3 forward = rotation * sw::math::kWorldForward;
        const sw::Vec3 rightWing = rotation * sw::Vec3{1.0f, 0.0f, 0.0f};
        const sw::Vec3 shipUp = rotation * sw::Vec3{0.0f, 1.0f, 0.0f};
        const sw::f32 pitch =
            std::asin(std::clamp(glm::dot(forward, up), -1.0f, 1.0f));
        const sw::f32 roll = std::atan2(glm::dot(rightWing, up), glm::dot(shipUp, up));

        const sw::f32 aspect = renderer().aspectRatio();
        const sw::f32 ballRadius = kNavballRadius;

        // Isotropic instrument space: rotate/offset there, compress X by the
        // aspect ratio last (same convention as hudText).
        const sw::Mat4 base =
            glm::translate(sw::Mat4{1.0f}, {0.0f, kNavballCenterY, 0.0f}) *
            glm::scale(sw::Mat4{1.0f}, {1.0f / aspect, 1.0f, 1.0f});

        auto pushNav = [&](sw::u32 meshIndex, const sw::Mat4& local,
                           const sw::Vec4& color) {
            sw::DrawItem item{};
            item.mesh = &m_meshes[meshIndex];
            item.transform = base * local;
            item.screenSpace = true;
            item.tint = color;
            m_drawItems.push_back(item);
        };

        const sw::Vec4 frameColor{0.65f, 0.85f, 0.9f, 0.55f};
        const sw::Vec4 horizonColor{0.55f, 0.95f, 1.0f, 0.9f};
        const sw::Vec4 referenceColor{1.0f, 0.62f, 0.15f, 0.95f};

        // Outer ring.
        pushNav(m_navRingMeshIndex,
                glm::scale(sw::Mat4{1.0f}, sw::Vec3{ballRadius}), frameColor);

        // Horizon line: rotated by roll, shifted by pitch (nose up -> the
        // horizon drops on screen; screen Y grows downward). Each line is
        // clipped to the CHORD of the ball at its offset, so the instrument
        // never bleeds outside its ring (and degrades gracefully at the
        // straight-up/straight-down gimbal poles, where the chord vanishes).
        const sw::Mat4 rollRotation =
            glm::rotate(sw::Mat4{1.0f}, roll, {0.0f, 0.0f, 1.0f});
        auto pushHorizonLine = [&](sw::f32 angleFromHorizon, sw::f32 widthFactor,
                                   sw::f32 thickness, sw::f32 alpha) {
            const sw::f32 normalized =
                std::clamp((pitch + angleFromHorizon) / kHalfPi, -1.0f, 1.0f);
            const sw::f32 offset = normalized * ballRadius * 0.92f;
            const sw::f32 chord = std::sqrt(std::max(
                0.0f, 1.0f - normalized * normalized * 0.85f)); // 0 at the poles
            if (chord < 0.05f)
            {
                return;
            }
            pushNav(m_navLineMeshIndex,
                    rollRotation *
                        glm::translate(sw::Mat4{1.0f}, {0.0f, offset, 0.0f}) *
                        glm::scale(sw::Mat4{1.0f},
                                   {ballRadius * widthFactor * chord,
                                    ballRadius * thickness, 1.0f}),
                    {horizonColor.r, horizonColor.g, horizonColor.b, alpha});
        };
        pushHorizonLine(0.0f, 0.86f, 0.012f, 0.9f);
        // Short pitch ticks at +/- 30 degrees from the horizon.
        pushHorizonLine(-0.5235988f, 0.34f, 0.007f, 0.5f);
        pushHorizonLine(0.5235988f, 0.34f, 0.007f, 0.5f);

        // Fixed craft reference: center diamond + stub wings.
        pushNav(m_navDiamondMeshIndex,
                glm::scale(sw::Mat4{1.0f}, sw::Vec3{ballRadius * 0.05f}),
                referenceColor);
        for (const sw::f32 side : {-1.0f, 1.0f})
        {
            pushNav(m_navLineMeshIndex,
                    glm::translate(sw::Mat4{1.0f},
                                   {side * ballRadius * 0.17f, 0.0f, 0.0f}) *
                        glm::scale(sw::Mat4{1.0f},
                                   {ballRadius * 0.09f, ballRadius * 0.012f, 1.0f}),
                    referenceColor);
        }

        // ---- prograde / retrograde markers ---------------------------------------
        // Velocity in the HUD's current reference frame (ORB or SRF, like
        // the speed readout), projected into the craft's body frame.
        sw::WorldVec3 referenceVelocity = primaryVelocity;
        if (m_speedSurfaceRelative)
        {
            if (const auto* source =
                    m_world.tryGetComponent<sw::phys::GravitySourceComponent>(
                        m_celestialIndex.body(static_cast<sw::usize>(primaryIndex))
                            .entity))
            {
                referenceVelocity += glm::cross(source->angularVelocity, radial);
            }
        }
        const sw::WorldVec3 relativeVelocity =
            controlledVelocity() - referenceVelocity;
        const sw::f64 relativeSpeed = glm::length(relativeVelocity);
        if (relativeSpeed > 0.5)
        {
            const sw::Vec3 direction = sw::Vec3(relativeVelocity / relativeSpeed);
            const sw::Quat inverseRotation = glm::inverse(rotation);
            for (const sw::f32 sign : {1.0f, -1.0f}) // prograde, retrograde
            {
                const sw::Vec3 local = inverseRotation * (direction * sign);
                if (local.z >= 0.0f)
                {
                    continue; // behind the craft: not on the front hemisphere
                }
                const sw::Vec2 ballPosition{local.x * ballRadius,
                                            -local.y * ballRadius};
                const sw::Mat4 place = glm::translate(
                    sw::Mat4{1.0f}, {ballPosition.x, ballPosition.y, 0.0f});
                if (sign > 0.0f) // prograde: filled diamond
                {
                    pushNav(m_navDiamondMeshIndex,
                            place * glm::scale(sw::Mat4{1.0f},
                                               sw::Vec3{ballRadius * 0.085f}),
                            {0.55f, 1.0f, 0.35f, 0.95f});
                }
                else // retrograde: hollow ring
                {
                    pushNav(m_navRingMeshIndex,
                            place * glm::scale(sw::Mat4{1.0f},
                                               sw::Vec3{ballRadius * 0.085f}),
                            {1.0f, 0.5f, 0.25f, 0.95f});
                }
            }
        }

        // ---- maneuver burn marker: point the nose at it, burn, watch DV --
        if (m_nodeActive)
        {
            // The SAME vector the readout counts down and the SAS points
            // at: one answer to "where do I aim and how much is left",
            // never three that can disagree.
            const sw::WorldVec3 burnVector = remainingBurnVector();
            const sw::f64 burnLength = glm::length(burnVector);
            if (burnLength > 0.05)
            {
                const sw::Vec3 local = glm::inverse(rotation) *
                                       sw::Vec3(burnVector / burnLength);
                if (local.z < 0.0f) // front hemisphere
                {
                    const sw::Mat4 place = glm::translate(
                        sw::Mat4{1.0f},
                        {local.x * ballRadius, -local.y * ballRadius, 0.0f});
                    pushNav(m_navDiamondMeshIndex,
                            place * glm::scale(sw::Mat4{1.0f},
                                               sw::Vec3{ballRadius * 0.11f}),
                            {0.75f, 0.4f, 1.0f, 0.95f});
                    pushNav(m_navRingMeshIndex,
                            place * glm::scale(sw::Mat4{1.0f},
                                               sw::Vec3{ballRadius * 0.13f}),
                            {0.75f, 0.4f, 1.0f, 0.7f});
                }
            }
        }
    }

    void StarWorksGame::collectMapTrajectories(const sw::Camera& activeCamera)
    {
        const sw::WorldVec3 cameraPosition = activeCamera.position();
        const sw::f32 markerFactor =
            kMarkerScreenFraction * 2.0f * std::tan(activeCamera.verticalFov() * 0.5f);
        const sw::f64 time = m_physicsLane->presentSeconds();

        // One dot of the dotted trajectory (emissive: tint alpha 2.0).
        auto plotDot = [&](const sw::WorldVec3& point, const sw::Vec4& color,
                           sw::f32 sizeMultiplier) {
            const sw::Vec3 relative = sw::Vec3(point - cameraPosition);
            const sw::f32 scale =
                glm::length(relative) * markerFactor * 0.22f * sizeMultiplier;
            sw::DrawItem item{};
            item.mesh = &m_meshes[m_markerMeshIndex];
            item.transform = glm::translate(sw::Mat4{1.0f}, relative) *
                             glm::scale(sw::Mat4{1.0f}, sw::Vec3{scale});
            item.boundsCenter = relative;
            item.boundsRadius = scale;
            item.tint = {color.r, color.g, color.b, 2.0f};
            m_drawItems.push_back(item);
        };

        // ONE STRAIGHT PIECE OF LINE between two world points.
        //
        // A dotted trajectory hides the one thing the map is for: you
        // cannot tell a plan that ENDS from a plan whose dots have spread
        // out, and at map zoom they always spread out. A stretched box
        // between consecutive samples costs the same draw item and gives a
        // line whose end means something.
        // A LINE IS A BOX, AND A BOX HAS ONE WIDTH.
        //
        // That is the whole difficulty. The width has to follow distance, or
        // the line changes thickness as you zoom; but ONE box spanning a
        // chord of Terra's own orbit is four million kilometres long, and
        // when the camera sits on that orbit — which it does, because the
        // camera is at Terra — one end of the box is billions of kilometres
        // away and the other end is in your eye. Sized for the far end it is
        // three thousand kilometres wide where it passes you, which is the
        // grey band across the planet.
        //
        // So a piece is SPLIT until its near and far ends are within a
        // factor of two of each other, and each piece is then sized from its
        // own closest approach. Almost every chord passes first try; only the
        // handful actually near the camera subdivide, so the cost is a few
        // extra boxes rather than a uniformly denser line.
        auto plotLine = [&](const sw::WorldVec3& a, const sw::WorldVec3& b,
                            const sw::Vec4& color, sw::f32 widthMultiplier) {
            struct Piece
            {
                sw::Vec3 a;
                sw::Vec3 b;
                sw::u32 depth;
            };
            constexpr sw::u32 kMaxSplitDepth = 5; // at most 32 pieces per chord
            Piece pending[2 * kMaxSplitDepth + 2];
            sw::u32 count = 0;
            pending[count++] = {sw::Vec3(a - cameraPosition),
                                sw::Vec3(b - cameraPosition), 0u};

            while (count > 0)
            {
                const Piece piece = pending[--count];
                const sw::Vec3 delta = piece.b - piece.a;
                const sw::f32 length = glm::length(delta);
                if (!(length > 1.0e-4f))
                {
                    continue;
                }

                // Closest approach of THIS piece to the eye, and its far end.
                const sw::f32 t = glm::clamp(
                    -glm::dot(piece.a, delta) / glm::dot(delta, delta), 0.0f, 1.0f);
                // (Not called `near`/`far`: those are macros in the Windows
                // headers this also builds against.)
                const sw::f32 closest = glm::length(piece.a + delta * t);
                const sw::f32 furthest =
                    std::max(glm::length(piece.a), glm::length(piece.b));
                if (piece.depth < kMaxSplitDepth &&
                    furthest > 2.0f * std::max(closest, 1.0f) &&
                    count + 2u <= static_cast<sw::u32>(std::size(pending)))
                {
                    const sw::Vec3 middle = piece.a + delta * 0.5f;
                    pending[count++] = {piece.a, middle, piece.depth + 1};
                    pending[count++] = {middle, piece.b, piece.depth + 1};
                    continue;
                }

                // 0.10 of the marker's own screen fraction is about 1.7
                // pixels at 1080p — a line, not a hairline that aliases into
                // dashes. Measured at the CLOSEST point, which is where a
                // width that is wrong is most obvious.
                const sw::f32 width = closest * markerFactor * 0.10f * widthMultiplier;
                const sw::Vec3 centre = (piece.a + piece.b) * 0.5f;
                const sw::Vec3 forward = delta / length;
                const sw::Vec3 reference = (std::abs(forward.y) < 0.99f)
                                               ? sw::Vec3{0.0f, 1.0f, 0.0f}
                                               : sw::Vec3{1.0f, 0.0f, 0.0f};
                const sw::Vec3 right = glm::normalize(glm::cross(reference, forward));
                const sw::Vec3 up = glm::cross(forward, right);

                sw::DrawItem item{};
                item.mesh = &m_meshes[m_orbitLineMeshIndex];
                item.transform =
                    glm::translate(sw::Mat4{1.0f}, centre) *
                    glm::mat4_cast(glm::quat_cast(sw::Mat3{right, up, forward})) *
                    // A hair longer than the gap so consecutive pieces
                    // overlap rather than leaving a seam on a tight curve.
                    glm::scale(sw::Mat4{1.0f}, sw::Vec3{width, width, length * 1.02f});
                item.boundsCenter = centre;
                item.boundsRadius = length * 0.51f + width;
                item.tint = {color.r, color.g, color.b, 2.0f};
                m_drawItems.push_back(item);
            }
        };

        // Samples a conic around a primary's CURRENT world position over
        // [t0, t1] — KSP map convention: patches are drawn in the frame of
        // where their primary is NOW — and joins the samples up.
        auto plotConic = [&](const sw::phys::KeplerOrbit& orbit,
                             const sw::WorldVec3& primaryPosition, const sw::Vec4& color,
                             sw::f64 t0, sw::f64 t1, sw::u32 samples,
                             sw::f32 widthMultiplier) {
            sw::WorldVec3 previous{};
            bool havePrevious = false;
            for (sw::u32 sample = 0; sample <= samples; ++sample)
            {
                // Spaced by ANOMALY, not by time — see the note on
                // kepler::timeAtArcFraction. Samples spaced evenly in time
                // leave one chord cutting thousands of kilometres off the
                // periapsis of a transfer ellipse: a line drawn straight
                // through the planet the arc goes round.
                const sw::f64 ts = sw::phys::kepler::timeAtArcFraction(
                    orbit, t0, t1, static_cast<sw::f64>(sample) / samples);
                sw::WorldVec3 relativePoint{};
                sw::phys::kepler::evaluate(orbit, ts, relativePoint);
                const sw::WorldVec3 point = primaryPosition + relativePoint;
                if (havePrevious)
                {
                    plotLine(previous, point, color, widthMultiplier);
                }
                previous = point;
                havePrevious = true;
            }
        };

        // Full closed orbit (elliptic only). One period exactly, so the
        // last chord closes the ring.
        auto plotFullOrbit = [&](const sw::phys::KeplerOrbit& orbit,
                                 const sw::WorldVec3& primaryPosition,
                                 const sw::Vec4& color) {
            if (!orbit.isHyperbolic())
            {
                plotConic(orbit, primaryPosition, color, time,
                          time + sw::phys::kepler::period(orbit), kTrajectorySamples,
                          0.6f);
            }
        };

        // ---- celestial orbits: each body around its parent's current position --
        for (sw::usize i = 0; i < m_celestialIndex.size(); ++i)
        {
            const auto& body = m_celestialIndex.body(i);
            if (body.hasOrbit == 0 || body.parentIndex < 0)
            {
                continue;
            }
            sw::Vec4 color{0.5f, 0.5f, 0.55f, 1.0f};
            if (const auto* marker = m_world.tryGetComponent<MapMarkerComponent>(
                    body.entity))
            {
                color = marker->color * 0.6f;
            }
            plotFullOrbit(body.orbit,
                          m_celestialIndex.positionAt(body.parentIndex, time), color);
        }

        // ---- generic rails objects (station modules...) --------------------------
        m_world.forEach<sw::phys::OnRailsComponent, MapMarkerComponent>(
            [&](sw::ecs::Entity entity, sw::phys::OnRailsComponent& rails,
                MapMarkerComponent& marker) {
                if (entity == controlledEntity())
                {
                    return; // the controlled craft gets the full flight plan
                }
                sw::WorldVec3 primaryPosition{0.0};
                if (const auto* primaryTransform =
                        m_world.tryGetComponent<TransformComponent>(rails.primary))
                {
                    primaryPosition = primaryTransform->position;
                }
                plotFullOrbit(rails.orbit, primaryPosition, marker.color * 0.6f);
            });

        // ---- other dynamic objects: single conic around their SOI primary --------
        m_world.forEach<TransformComponent, sw::phys::DynamicBodyComponent,
                        MapMarkerComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                sw::phys::DynamicBodyComponent& body, MapMarkerComponent& marker) {
                if (entity == controlledEntity() || m_celestialIndex.size() == 0)
                {
                    return;
                }
                const sw::i32 primaryIndex =
                    m_celestialIndex.soiPrimaryAt(transform.position, time);
                if (primaryIndex < 0)
                {
                    return;
                }
                const auto& primary =
                    m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
                sw::WorldVec3 primaryPosition{};
                sw::WorldVec3 primaryVelocity{};
                m_celestialIndex.stateAt(primaryIndex, time, primaryPosition,
                                         &primaryVelocity);
                sw::phys::KeplerOrbit orbit{};
                if (sw::phys::kepler::fromStateVectors(
                        primary.mu, transform.position - primaryPosition,
                        body.velocity - primaryVelocity, time, orbit))
                {
                    plotFullOrbit(orbit, primaryPosition, marker.color * 0.6f);
                }
            });

        // ---- THE FLIGHT PLAN: patched-conics prediction of the controlled craft --
        auto plotPlan = [&](const std::vector<sw::space::TrajectorySegment>& plan,
                            bool nodePlan) {
            for (sw::usize segmentIndex = 0; segmentIndex < plan.size(); ++segmentIndex)
            {
                const sw::space::TrajectorySegment& segment = plan[segmentIndex];
                if (segment.primaryIndex < 0 ||
                    segment.endReason == sw::space::SegmentEnd::Lost)
                {
                    continue;
                }
                // The node plan glows white-violet so it never reads as the
                // current trajectory.
                const sw::Vec4 color =
                    nodePlan
                        ? sw::Vec4{0.85f, 0.75f + 0.25f * (segmentIndex == 0), 1.0f,
                                   1.0f}
                        : kPatchColors[std::min(segmentIndex,
                                                std::size(kPatchColors) - 1)];
                const sw::WorldVec3 primaryPosition =
                    m_celestialIndex.positionAt(segment.primaryIndex, time);

                // Every patch draws exactly its [start, end] arc — and the
                // predictor now ends a patch where something HAPPENS to it,
                // so that arc is the whole story: a closed orbit comes back
                // to its own start, a transfer runs to its encounter, an
                // escape runs to the edge of the sphere of influence.
                plotConic(segment.orbit, primaryPosition, color, segment.startTime,
                          segment.endTime, kPredictionDisplaySamples, 1.0f);

                // Event marker at the patch hand-off point.
                sw::Vec4 eventColor{};
                bool hasEvent = true;
                switch (segment.endReason)
                {
                case sw::space::SegmentEnd::Encounter:
                    eventColor = {0.4f, 1.0f, 0.9f, 1.0f};
                    break;
                case sw::space::SegmentEnd::Impact:
                    eventColor = {1.0f, 0.25f, 0.2f, 1.0f};
                    break;
                case sw::space::SegmentEnd::SoiExit:
                    eventColor = {1.0f, 0.85f, 0.4f, 1.0f};
                    break;
                default:
                    hasEvent = false;
                    break;
                }
                if (hasEvent)
                {
                    sw::WorldVec3 eventRelative{};
                    sw::phys::kepler::evaluate(segment.orbit, segment.endTime,
                                               eventRelative);
                    plotDot(primaryPosition + eventRelative, eventColor, 3.0f);
                }
            }
        };
        plotPlan(m_prediction, false);
        plotPlan(m_nodePrediction, true);

        // The maneuver node itself: a large violet diamond at the burn point,
        // drawn around its primary's CURRENT position like every patch.
        if (m_nodeActive && m_nodePrimaryIndex >= 0)
        {
            // Bigger and hotter while it is being dragged: the one moment
            // the player needs to be sure the map heard them.
            plotDot(m_celestialIndex.positionAt(m_nodePrimaryIndex, time) +
                        m_nodeRelativePosition,
                    m_nodeDragging ? sw::Vec4{1.0f, 0.85f, 0.45f, 1.0f}
                                   : sw::Vec4{0.75f, 0.4f, 1.0f, 1.0f},
                    m_nodeDragging ? 5.5f : 4.0f);
        }

        // ---- THE TARGET, AND WHERE IT WILL BE --------------------------------
        //
        // Three things, and the third is the one that makes a transfer
        // possible to fly: the body you picked, WHERE IT WILL HAVE MOVED TO
        // by closest approach, and where you will be when it does. Every
        // one of them is placed in the frame the orbits are drawn in, so
        // the future position sits on the ring rather than out in the world
        // where the body will really be.
        if (m_targetIndex >= 0 &&
            static_cast<sw::usize>(m_targetIndex) < m_celestialIndex.size())
        {
            constexpr sw::Vec4 kTargetColor{1.0f, 0.45f, 0.85f, 1.0f};
            plotDot(m_celestialIndex.positionAt(m_targetIndex, time), kTargetColor, 5.0f);

            auto framePosition = [&](sw::i32 primaryIndex,
                                     const sw::WorldVec3& relative) {
                return (primaryIndex >= 0)
                           ? m_celestialIndex.positionAt(primaryIndex, time) + relative
                           : relative;
            };
            for (const auto& [approach, marker] :
                 {std::pair{&m_approach, false}, std::pair{&m_nodeApproach, true}})
            {
                if (!approach->valid)
                {
                    continue;
                }
                const sw::Vec4 color =
                    marker ? sw::Vec4{0.85f, 0.75f, 1.0f, 1.0f} : kTargetColor;
                const sw::WorldVec3 theirs = framePosition(
                    approach->targetPrimaryIndex, approach->targetRelativePosition);
                const sw::WorldVec3 ours =
                    framePosition(approach->primaryIndex, approach->relativePosition);
                plotDot(theirs, color, 3.5f);
                plotDot(ours, color, 3.0f);
                // The gap itself, drawn: the separation is the point, and a
                // pair of dots leaves the eye to guess which two.
                plotLine(ours, theirs, color * sw::Vec4{1.0f, 1.0f, 1.0f, 0.6f}, 0.7f);
            }
        }
    }

    void StarWorksGame::collectDrawItems(const sw::Camera& activeCamera, bool mapView)
    {
        m_drawItems.clear();
        m_drawItems.reserve(m_world.aliveCount() + 512);

        // Static star dome: CAMERA-CENTERED (no translation), so the stars
        // are parallax-free — an infinitely distant, never-changing sky to
        // orient by. One emissive mesh, one draw call.
        {
            sw::DrawItem stars{};
            stars.mesh = &m_meshes[m_starfieldMeshIndex];
            stars.transform = glm::scale(sw::Mat4{1.0f}, sw::Vec3{kStarDomeRadius});
            stars.boundsCenter = {0.0f, 0.0f, 0.0f};
            stars.boundsRadius = kStarDomeRadius;
            // Daylight washes the stars out: the emissive opacity is
            // (vertexAlpha * tintAlpha - 1), so tint 2.0 = full night sky
            // and tint 1.0 = fully invisible at noon on the pad.
            const sw::f32 nightFactor = 1.0f - 0.96f * m_skyDayFactor;
            stars.tint = {1.0f, 1.0f, 1.0f, 1.0f + nightFactor};
            m_drawItems.push_back(stars);
        }

        // The sun's soft glow: two emissive radial-falloff discs, always
        // facing the camera, drawn in the transparent pass.
        if (const auto* sol = m_world.tryGetComponent<TransformComponent>(m_solEntity))
        {
            const sw::Vec3 toSol = sw::Vec3(sol->position - activeCamera.position());
            const sw::f32 distance = glm::length(toSol);
            if (distance > static_cast<sw::f32>(kSolRadius) * 3.0f)
            {
                const sw::Vec3 z = -toSol / distance; // disc normal, toward camera
                const sw::Vec3 reference =
                    (std::abs(z.y) < 0.99f) ? sw::Vec3{0, 1, 0} : sw::Vec3{1, 0, 0};
                const sw::Vec3 x = glm::normalize(glm::cross(reference, z));
                const sw::Vec3 yAxis = glm::cross(z, x);
                const sw::Mat4 basis{sw::Vec4(x, 0.0f), sw::Vec4(yAxis, 0.0f),
                                     sw::Vec4(z, 0.0f), sw::Vec4(toSol, 1.0f)};

                auto pushGlow = [&](sw::u32 meshIndex, sw::f32 radiusFactor) {
                    const sw::f32 radius =
                        static_cast<sw::f32>(kSolRadius) * radiusFactor;
                    sw::DrawItem glow{};
                    glow.mesh = &m_meshes[meshIndex];
                    glow.transform =
                        basis * glm::scale(sw::Mat4{1.0f}, sw::Vec3{radius});
                    glow.boundsCenter = toSol;
                    glow.boundsRadius = radius;
                    glow.tint = {1.0f, 1.0f, 1.0f, 1.0f};
                    glow.transparent = true;
                    m_drawItems.push_back(glow);
                };
                pushGlow(m_sunHaloMeshIndex, 7.5f);
                pushGlow(m_sunCoreMeshIndex, 2.1f);

                // ---- LENS FLARE: screen-space ghosts along the sun axis ------
                // Only when the sun is on screen and not behind a planet.
                if (!mapView)
                {
                    bool occluded = false;
                    for (sw::usize i = 0; i < m_celestialIndex.size(); ++i)
                    {
                        const auto& body = m_celestialIndex.body(i);
                        if (body.entity == m_solEntity)
                        {
                            continue;
                        }
                        if (const auto* bodyTransform =
                                m_world.tryGetComponent<TransformComponent>(body.entity))
                        {
                            const sw::Vec3 center =
                                sw::Vec3(bodyTransform->position -
                                         activeCamera.position());
                            const sw::Vec3 lightDir = toSol / distance;
                            const sw::f32 along = glm::dot(center, lightDir);
                            if (along > 0.0f && along < distance)
                            {
                                const sw::f32 miss =
                                    glm::length(center - lightDir * along);
                                if (miss < static_cast<sw::f32>(body.bodyRadius))
                                {
                                    occluded = true;
                                    break;
                                }
                            }
                        }
                    }
                    const sw::Vec4 clip =
                        activeCamera.viewProjectionCameraRelative() *
                        sw::Vec4(toSol, 1.0f);
                    // The chain only exists while the sun CORE is on
                    // screen AND in front of the camera (w>0 alone lets
                    // ghosts of an off-screen/behind sun float in deep
                    // space as stray colored circles).
                    const bool sunInFront =
                        clip.w > 0.0f && glm::dot(activeCamera.forward(), z) < 0.0f;
                    if (!occluded && sunInFront)
                    {
                        const sw::Vec2 sunNdc{clip.x / clip.w, clip.y / clip.w};
                        if (std::abs(sunNdc.x) < 0.98f && std::abs(sunNdc.y) < 0.98f)
                        {
                            // Fade toward the screen edge; ghosts mirror
                            // through the center (anamorphic-ish chain).
                            const sw::f32 edgeFade =
                                glm::clamp(1.05f - glm::length(sunNdc), 0.0f, 1.0f);
                            struct FlareGhost
                            {
                                sw::f32 t;      // position along sun->center axis
                                sw::f32 scale;  // NDC radius
                                sw::Vec3 color;
                                sw::f32 alpha;
                            };
                            const FlareGhost ghosts[] = {
                                {0.35f, 0.055f, {1.0f, 0.80f, 0.45f}, 0.16f},
                                {0.62f, 0.028f, {0.55f, 0.85f, 0.60f}, 0.14f},
                                {0.95f, 0.090f, {0.45f, 0.60f, 1.00f}, 0.10f},
                                {1.28f, 0.045f, {1.00f, 0.55f, 0.40f}, 0.12f},
                                {1.60f, 0.130f, {0.55f, 0.45f, 0.95f}, 0.07f},
                            };
                            const sw::f32 aspect = renderer().aspectRatio();
                            for (const FlareGhost& ghost : ghosts)
                            {
                                const sw::Vec2 position =
                                    sunNdc * (1.0f - ghost.t); // toward/past center
                                sw::DrawItem item{};
                                item.mesh = &m_meshes[m_flareMeshIndex];
                                item.transform =
                                    glm::translate(sw::Mat4{1.0f},
                                                   {position.x, position.y, 0.0f}) *
                                    glm::scale(sw::Mat4{1.0f},
                                               {ghost.scale / aspect, ghost.scale, 1.0f});
                                item.screenSpace = true;
                                item.tint = {ghost.color.r, ghost.color.g,
                                             ghost.color.b,
                                             ghost.alpha * edgeFade};
                                m_drawItems.push_back(item);
                            }
                        }
                    }
                }
            }
        }

        const sw::f32 alpha = m_physicsLane->alpha();
        const sw::f64 alpha64 = static_cast<sw::f64>(alpha);
        const sw::WorldVec3 cameraPosition = activeCamera.position();

        auto makeTransform = [&](const TransformComponent& transform,
                                 const PreviousTransformComponent& previous,
                                 sw::Vec3& outRelative) {
            const sw::WorldVec3 position =
                glm::mix(previous.position, transform.position, alpha64);
            const sw::Quat rotation = glm::slerp(previous.rotation, transform.rotation, alpha);
            outRelative = sw::Vec3(position - cameraPosition);
            return glm::translate(sw::Mat4{1.0f}, outRelative) * glm::mat4_cast(rotation) *
                   glm::scale(sw::Mat4{1.0f}, sw::Vec3{transform.uniformScale});
        };

        m_world.forEach<TransformComponent, PreviousTransformComponent, BoundsComponent,
                        MeshComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                PreviousTransformComponent& previous, BoundsComponent& bounds,
                MeshComponent& mesh) {
                // You are INSIDE the suit on EVA: drawing it would fill the
                // screen with the back of your own helmet. The map still
                // shows it — there you are looking at the world, not out of
                // your own eyes.
                if (!mapView && m_evaMode && entity == m_capsuleEntity)
                {
                    return;
                }
                sw::Vec3 relative{};
                const sw::Mat4 model = makeTransform(transform, previous, relative);
                sw::DrawItem item{&m_meshes[mesh.meshIndex], model, relative,
                                  bounds.localRadius * transform.uniformScale};
                item.transparent = mesh.transparent != MeshComponent::kOpaque;
                if (item.transparent)
                {
                    // Shell materials in Mesh.frag: 3.0 = atmosphere (fresnel
                    // limb), 3.2 = cloud deck (per-fragment weather).
                    const sw::f32 shell =
                        (mesh.transparent == MeshComponent::kCloudDeck) ? 3.2f : 3.0f;
                    item.tint = {1.0f, 1.0f, 1.0f, shell};
                }

                // Reentry glow: the craft reddens with heating and turns
                // emissive (self-lit plasma sheath) when it gets severe.
                sw::f32 heat = 0.0f;
                if (entity == m_shipEntity) { heat = m_shipHeat; }
                else if (entity == m_capsuleEntity) { heat = m_capsuleHeat; }
                else if (const auto* part =
                             m_world.tryGetComponent<sw::parts::PartComponent>(entity);
                         part != nullptr && part->vessel == m_shipEntity)
                {
                    heat = m_shipHeat; // the whole rocket glows
                }
                if (heat > 0.0f)
                {
                    const sw::Vec3 glow =
                        glm::mix(sw::Vec3{1.0f, 1.0f, 1.0f},
                                 sw::Vec3{1.0f, 0.30f, 0.12f}, heat);
                    item.tint = {glow.r, glow.g, glow.b, heat > 0.55f ? 2.0f : 1.0f};
                }
                m_drawItems.push_back(item);
            });

        m_world.forEach<TransformComponent, PreviousTransformComponent, BoundsComponent,
                        CelestialLodComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                PreviousTransformComponent& previous, BoundsComponent& bounds,
                CelestialLodComponent& lod) {
                sw::Vec3 relative{};
                const sw::Mat4 model = makeTransform(transform, previous, relative);
                const sw::f64 worldRadius = static_cast<sw::f64>(transform.uniformScale);
                const sw::f64 distance =
                    glm::length(transform.position - cameraPosition);
                const sw::u32 level = selectLodLevel(distance, worldRadius);
                sw::DrawItem item{&m_meshes[lod.meshIndex[level]], model, relative,
                                  bounds.localRadius * transform.uniformScale};
                // CLOSE ORBIT (M23): per-vertex colors blur when the globe
                // fills the screen — hand the surface to the PER-FRAGMENT
                // procedural path (tint alpha 3.6 + style/10 routes it in
                // Mesh.frag; the shader samples the exact same fbm as the
                // CPU terrain, so coastlines stay collision-true).
                // The threshold used to be 4 radii — 25,000 km on Terra, from
                // where the globe is a small disc that the vertex path draws
                // just as well for a fraction of the cost. At 1.6 radii the
                // expensive path only runs when the planet actually fills a
                // meaningful part of the screen, which is the only place its
                // sharpness is visible.
                if (lod.surfaceStyle >= 0 && distance < worldRadius * 1.6)
                {
                    item.tint = {1.0f, 1.0f, 1.0f,
                                 3.6f + 0.1f * static_cast<sw::f32>(lod.surfaceStyle)};
                }
                if (entity == m_solEntity)
                {
                    // The star is self-lit (emissive tint convention).
                    item.tint = {1.0f, 0.96f, 0.82f, 2.0f};
                }
                m_drawItems.push_back(item);
            });

        // ---- procedural terrain patch (near the ground, not in map view) ------
        if (!mapView && m_terrainVisible && m_terrainMeshSlot != 0xFFFFFFFFu)
        {
            if (const auto* body =
                    m_world.tryGetComponent<TransformComponent>(m_terrainBody))
            {
                sw::WorldVec3 bodyPosition = body->position;
                sw::Quat bodyRotation = body->rotation;
                if (const auto* previous =
                        m_world.tryGetComponent<PreviousTransformComponent>(
                            m_terrainBody))
                {
                    bodyPosition = glm::mix(previous->position, body->position, alpha64);
                    bodyRotation =
                        glm::slerp(previous->rotation, body->rotation, alpha);
                }
                // POSITION from the f64 spin (m_terrainOriginLocal is a full
                // planet radius long), ORIENTATION from the f32 quaternion
                // above — over the patch's own few kilometres that is a tenth
                // of a millimetre.
                const auto* spin =
                    m_world.tryGetComponent<sw::phys::GravitySourceComponent>(
                        m_terrainBody);
                const glm::dquat originRotation =
                    (spin != nullptr) ? sw::phys::spinRotationAt(*spin, alpha64)
                                      : glm::dquat(bodyRotation);
                const sw::WorldVec3 originWorld =
                    bodyPosition + originRotation * m_terrainOriginLocal;
                const sw::Vec3 relative = sw::Vec3(originWorld - cameraPosition);
                sw::DrawItem item{};
                item.mesh = &m_meshes[m_terrainMeshSlot];
                // The mesh is oriented by the SAME rotation that placed its
                // origin, cast down: two rotations a ten-millionth of a radian
                // apart would shear the patch away from its own anchor point.
                item.transform = glm::translate(sw::Mat4{1.0f}, relative) *
                                 glm::mat4_cast(sw::Quat(originRotation));
                item.boundsCenter = relative;
                item.boundsRadius = static_cast<sw::f32>(m_terrainExtent * 1.8);
                m_drawItems.push_back(item);

                // THE FIELD, in its own chunks. Same origin, same rotation —
                // it was built in the patch's chart and must be drawn in it,
                // or every blade would slide off the ground it stands on.
                if (m_grassLiveCount != 0 && m_grassBody == m_terrainBody)
                {
                    const sw::WorldVec3 grassOrigin =
                        bodyPosition + originRotation * m_grassOriginLocal;
                    const sw::Vec3 grassRelative = sw::Vec3(grassOrigin - cameraPosition);
                    for (sw::u32 chunk = 0; chunk < m_grassLiveCount; ++chunk)
                    {
                        const sw::u32 slot = m_grassSlots[m_grassSet][chunk];
                        if (!m_grassChunkValid[m_grassSet][chunk] ||
                            slot == 0xFFFFFFFFu || slot >= m_meshes.size())
                        {
                            continue;
                        }
                        sw::DrawItem blades{};
                        blades.mesh = &m_meshes[slot];
                        blades.transform =
                            glm::translate(sw::Mat4{1.0f}, grassRelative) *
                            glm::mat4_cast(sw::Quat(originRotation));
                        blades.boundsCenter = grassRelative;
                        blades.boundsRadius = 320.0f;
                        m_drawItems.push_back(blades);
                    }
                }
            }
        }

        if (mapView)
        {
            const sw::f32 markerFactor =
                kMarkerScreenFraction * 2.0f * std::tan(activeCamera.verticalFov() * 0.5f);

            m_world.forEach<TransformComponent, BoundsComponent, MapMarkerComponent>(
                [&](sw::ecs::Entity, TransformComponent& transform, BoundsComponent& bounds,
                    MapMarkerComponent& marker) {
                    const sw::WorldVec3 toCamera = cameraPosition - transform.position;
                    const sw::f64 distance = glm::length(toCamera);
                    if (distance < 1.0)
                    {
                        return;
                    }
                    const sw::f64 surfaceOffset =
                        static_cast<sw::f64>(bounds.localRadius * transform.uniformScale) *
                        1.05;
                    const sw::WorldVec3 beaconPosition =
                        transform.position + (toCamera / distance) * surfaceOffset;

                    const sw::f32 scale =
                        static_cast<sw::f32>(glm::length(beaconPosition - cameraPosition)) *
                        markerFactor;
                    const sw::Vec3 relative = sw::Vec3(beaconPosition - cameraPosition);
                    const sw::Mat4 model = glm::translate(sw::Mat4{1.0f}, relative) *
                                           glm::scale(sw::Mat4{1.0f}, sw::Vec3{scale});
                    m_drawItems.push_back(
                        {&m_meshes[m_markerMeshIndex], model, relative, scale,
                         {marker.color.r, marker.color.g, marker.color.b, 2.0f}});
                });

            collectMapTrajectories(activeCamera);
        }
        else
        {
            collectParticles(activeCamera);
        }

        if (!mapView)
        {
            collectConveyors(activeCamera);
            collectCables(activeCamera);
            collectHullOverlay(activeCamera);
            collectBuildGhost(activeCamera);
        }
        // Beacons overlay both views; the HUD is drawn last so its panels
        // stay on top of them.
        collectBeacons(activeCamera, mapView);
        collectHud();
    }

    void StarWorksGame::onRender()
    {
        if (m_editorMode)
        {
            // The hangar is its own little world: fixed light, no eclipse,
            // soft studio ambient (no fog).
            renderer().setSunPosition({60.0f, 90.0f, 40.0f});
            renderer().setShadowSpheres({});
            renderer().setAtmosphere({0.0f, 0.0f, 0.0f}, 0.0f,
                                     {0.055f, 0.06f, 0.07f});
            collectHangarItems();
            renderer().renderFrame(m_hangarCamera, m_drawItems);
            return;
        }
        const sw::Camera& activeCamera = m_mapView ? m_mapCamera : m_camera;
        const sw::WorldVec3 cameraPosition = activeCamera.position();

        // Light comes from Sol's actual position (camera-relative), and
        // every celestial body except the star casts an analytic shadow —
        // no sunlight behind a planet.
        if (const auto* sol = m_world.tryGetComponent<TransformComponent>(m_solEntity))
        {
            renderer().setSunPosition(sw::Vec3(sol->position - cameraPosition));
        }
        std::array<sw::Renderer::ShadowSphere, sw::Renderer::kMaxShadowSpheres>
            occluders{};
        sw::u32 occluderCount = 0;
        for (sw::usize i = 0;
             i < m_celestialIndex.size() &&
             occluderCount < sw::Renderer::kMaxShadowSpheres;
             ++i)
        {
            const auto& body = m_celestialIndex.body(i);
            if (body.entity == m_solEntity)
            {
                continue;
            }
            if (const auto* bodyTransform =
                    m_world.tryGetComponent<TransformComponent>(body.entity))
            {
                occluders[occluderCount++] = {
                    sw::Vec3(bodyTransform->position - cameraPosition),
                    static_cast<sw::f32>(body.bodyRadius)};
            }
        }
        renderer().setShadowSpheres(
            std::span<const sw::Renderer::ShadowSphere>(occluders.data(),
                                                        occluderCount));

        // ---- AERIAL PERSPECTIVE (M21): fog + sky light from the camera's ------
        // position inside an atmosphere. Color tracks the local sun
        // elevation — blue at noon, amber at the terminator, black at night.
        sw::Vec3 fogColor{0.0f};
        sw::f32 fogDensity = 0.0f;
        sw::Vec3 skyAmbient{0.0f};
        m_skyDayFactor = 0.0f;

        // ---- M29: WHICH AIR are we looking through? ---------------------------
        // The nearest body that has an atmosphere, always — not just when the
        // camera is inside it. From orbit that same body supplies the limb;
        // from the ground, the sky and the aerial perspective. One integral,
        // three uses, so the descent never switches models.
        if (m_mapView)
        {
            // The star map is a schematic: no air, no haze over the orbits.
            renderer().setAtmosphereBody(sw::Vec3{0.0f}, 0.0f, 0);
        }
        else
        {
            sw::f32 bestRadius = 0.0f;
            sw::Vec3 bestCentre{0.0f};
            sw::i32 bestStyle = 0;
            sw::f64 bestDistance = std::numeric_limits<sw::f64>::max();
            for (sw::usize i = 0; i < m_celestialIndex.size(); ++i)
            {
                const auto& body = m_celestialIndex.body(i);
                if (m_world.tryGetComponent<sw::phys::AtmosphereComponent>(
                        body.entity) == nullptr)
                {
                    continue;
                }
                const auto* bodyTransform =
                    m_world.tryGetComponent<TransformComponent>(body.entity);
                if (bodyTransform == nullptr)
                {
                    continue;
                }
                const sw::f64 distance =
                    glm::length(bodyTransform->position - cameraPosition);
                if (distance >= bestDistance)
                {
                    continue;
                }
                bestDistance = distance;
                bestCentre = sw::Vec3(bodyTransform->position - cameraPosition);
                bestRadius = static_cast<sw::f32>(body.bodyRadius);
                bestStyle = 0;
                if (const auto* lod =
                        m_world.tryGetComponent<CelestialLodComponent>(body.entity);
                    lod != nullptr && lod->surfaceStyle >= 0)
                {
                    bestStyle = lod->surfaceStyle;
                }
            }
            renderer().setAtmosphereBody(bestCentre, bestRadius, bestStyle);
        }

        if (!m_mapView)
        {
            for (sw::usize i = 0; i < m_celestialIndex.size(); ++i)
            {
                const auto& body = m_celestialIndex.body(i);
                const auto* atmosphereComponent =
                    m_world.tryGetComponent<sw::phys::AtmosphereComponent>(body.entity);
                const auto* bodyTransform =
                    m_world.tryGetComponent<TransformComponent>(body.entity);
                if (atmosphereComponent == nullptr || bodyTransform == nullptr)
                {
                    continue;
                }
                const sw::WorldVec3 radial = cameraPosition - bodyTransform->position;
                const sw::f64 altitude = glm::length(radial) - body.bodyRadius;
                if (altitude > atmosphereComponent->topAltitude)
                {
                    continue;
                }
                const sw::f32 density = static_cast<sw::f32>(
                    std::exp(-std::max(altitude, 0.0) /
                             atmosphereComponent->scaleHeight));
                const sw::Vec3 up = sw::Vec3(glm::normalize(radial));
                sw::Vec3 sunDir{0.0f, 1.0f, 0.0f};
                if (const auto* sol =
                        m_world.tryGetComponent<TransformComponent>(m_solEntity))
                {
                    sunDir = sw::Vec3(glm::normalize(sol->position - cameraPosition));
                }
                const sw::f32 elevation = glm::dot(up, sunDir);
                const sw::f32 day =
                    glm::clamp((elevation + 0.06f) / 0.30f, 0.0f, 1.0f);
                const sw::Vec3 daySky{0.28f, 0.46f, 0.78f};
                const sw::Vec3 sunsetSky{0.82f, 0.40f, 0.16f};
                const sw::Vec3 nightSky{0.010f, 0.012f, 0.022f};
                sw::Vec3 sky = glm::mix(sunsetSky, daySky,
                                        glm::clamp((elevation - 0.03f) / 0.30f,
                                                   0.0f, 1.0f));
                sky = glm::mix(nightSky, sky, day);
                fogColor = sky;
                fogDensity = density * 1.2e-4f;
                skyAmbient = sky * (0.38f * density * day);
                m_skyDayFactor = day * density;
                break;
            }
        }
        renderer().setAtmosphere(fogColor, fogDensity, skyAmbient);
        // World clock for animated shading (cloud advection, waves). It is
        // the SIMULATION's clock, not the wall clock: the deck drifts over a
        // ground that now turns analytically with the same time, so warping
        // speeds the weather up exactly as it speeds the day up. Wrapped to
        // a day so f32 keeps millisecond resolution after a long session.
        renderer().setTimeSeconds(static_cast<sw::f32>(
            std::fmod(m_physicsLane->presentSeconds(), 86400.0)));

        collectDrawItems(activeCamera, m_mapView);
        renderer().renderFrame(activeCamera, m_drawItems);
    }
} // namespace game
