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

        constexpr sw::u32 kStationModuleCount = 8;

        constexpr sw::f64 kBubbleEnterRadius = 1.0e4; // 10 km
        constexpr sw::f64 kBubbleExitRadius = 1.5e4;  // 15 km (hysteresis)

        // Star map: constant on-screen marker size and zoom limits (up to
        // the full Sol system — Mars orbit is 2.28e11 m).
        constexpr sw::f32 kMarkerScreenFraction = 0.016f;
        constexpr sw::f64 kMapMinHeight = 2.0e7;
        constexpr sw::f64 kMapMaxHeight = 8.0e11;
        constexpr sw::u32 kTrajectorySamples = 72;
        constexpr sw::u32 kPredictionDisplaySamples = 160; // dots per patch
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
        constexpr sw::f32 kWarpLadder[] = {1.0f,    2.0f,     5.0f,     10.0f,   50.0f,
                                           100.0f, 1000.0f, 10000.0f, 100000.0f};
        constexpr sw::u32 kWarpSteps = static_cast<sw::u32>(std::size(kWarpLadder));

        /// PHYSICS WARP: the world stays fully simulated (drag, thrust,
        /// collisions) up to this time scale; beyond it everything rides
        /// analytic rails. Integration at 50 Hz stays stable to x5.
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
            return 100000.0f;
        }

        /// Sphere LOD resolutions, most to least detailed (rings, segments).
        /// LOD0 is dense enough for readable per-vertex continents.
        constexpr sw::u32 kLodRings[CelestialLodComponent::kLodLevels] = {150, 80, 32, 14, 8};
        constexpr sw::u32 kLodSegments[CelestialLodComponent::kLodLevels] = {225, 120, 48, 21, 12};
        constexpr sw::f32 kLodScreenFractions[CelestialLodComponent::kLodLevels - 1] = {
            0.5f, 0.15f, 0.04f, 0.008f};

        constexpr sw::f32 kCubeBoundsRadius = 0.8660254f;
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
                if (!sw::parts::isBuilding(definition))
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
        // DATA-DRIVEN INDUSTRY (F1): the production chains are .swrecipe
        // files on the same contract — stable ids, a built-in fallback, and
        // a loader that refuses any recipe which would create matter.
        sw::factory::loadRecipeCatalog(sw::FileSystem::executableDirectory() / "Assets" /
                                       "Recipes");

        buildScene();
        buildGlyphMeshes();
        buildNavballMeshes();
        m_hangarFloorMeshIndex = registerMesh(renderer().createMesh(
            sw::PrimitiveFactory::makeGridPlane(40.0f, 20, {0.2f, 0.3f, 0.38f, 1.0f})));
        m_hangarCamera.setPerspective(sw::math::toRadians(55.0f), 0.2f, 500.0f);
        buildSaveSchema();
        m_celestialIndex.rebuild(m_world);

        const sw::WorldVec3 stationCenter =
            m_world.getComponent<TransformComponent>(m_terraEntity).position +
            sw::WorldVec3{0.0, 0.0, kStationOrbitRadius};
        m_cameraController.setPose(stationCenter + sw::WorldVec3{0.0, 250.0, 2000.0}, 0.0f,
                                   -0.08f);

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

        auto& automation = m_simulation.findLane("Automation")->scheduler();
        automation.addSystem(std::make_unique<SolarChargeSystem>());
        // The generic recipe executor: ONE system for every building the
        // player will ever place. The two below stay for the asteroid rig
        // and the orbital station, which are craft, not buildings.
        automation.addSystem(std::make_unique<sw::factory::ProductionSystem>());
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
                    "Controls: Tab pilot/free | G EVA capsule | M map (wheel zoom) | V "
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

        sw::u32 asteroidMeshId = 0;
        sw::f32 asteroidBoundsRadius = 2.5f;
        try
        {
            asteroidMeshId = registerMesh(renderer().createMesh(sw::GltfLoader::loadMesh(
                sw::FileSystem::resolve("Assets/Models/asteroid.glb"))));
        }
        catch (const sw::Exception& e)
        {
            SW_LOG_WARN("Game", "Asteroid asset unavailable ({}); using procedural sphere",
                        e.message());
            asteroidMeshId = registerMesh(renderer().createMesh(
                sw::PrimitiveFactory::makeUvSphere(1.0f, 24, 32, {0.45f, 0.41f, 0.38f, 1.0f})));
            asteroidBoundsRadius = 1.0f;
        }
        const sw::u32 moduleMeshId = registerMesh(renderer().createMesh(
            sw::PrimitiveFactory::makeCube(1.0f, {0.75f, 0.78f, 0.82f, 1.0f})));
        // Part meshes, indexed by catalog id (small ids: direct table).
        for (const sw::parts::PartDefinition& definition : sw::parts::catalog())
        {
            m_partMeshIds[definition.id] = registerMesh(
                renderer().createMesh(sw::parts::buildPartMesh(definition)));
        }
        const auto& partMeshIds = m_partMeshIds;
        m_capsuleMeshIndex = registerMesh(renderer().createMesh(
            sw::PrimitiveFactory::makeCapsule(0.5f, 0.5f, 12, 16,
                                              {0.9f, 0.6f, 0.2f, 1.0f})));
        m_markerMeshIndex = registerMesh(renderer().createMesh(buildMarkerMesh()));

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
            const sw::Quat standUp = [&] {
                const sw::Vec3 from{0.0f, 1.0f, 0.0f};
                const sw::f32 alignment = glm::dot(from, siteUp);
                if (alignment > 0.99999f)
                {
                    return sw::Quat{1.0f, 0.0f, 0.0f, 0.0f};
                }
                if (alignment < -0.99999f)
                {
                    return sw::Quat{0.0f, 1.0f, 0.0f, 0.0f}; // 180 deg about X
                }
                const sw::Vec3 axis = glm::cross(from, siteUp);
                return glm::normalize(
                    sw::Quat{1.0f + alignment, axis.x, axis.y, axis.z});
            }();

            sw::ecs::Entity hubEntity{};
            // Spawns one building from its catalogue definition: geometry,
            // power, storage and siting rules all come from the .swpart.
            auto spawnBuilding = [&](sw::u32 definitionId, sw::f32 eastMetres,
                                     sw::f32 northMetres, sw::u32 recipeId,
                                     const sw::Vec4& marker) {
                const sw::parts::PartDefinition* definition =
                    sw::parts::findDefinition(definitionId);
                if (definition == nullptr || !sw::parts::isBuilding(*definition))
                {
                    SW_LOG_WARN("Game", "Building definition {} missing from the catalog",
                                definitionId);
                    return sw::ecs::Entity::null();
                }
                const sw::parts::BuildingSpec& spec = definition->building;

                // Offsets are metres on the tangent plane, re-normalised so
                // every building sits on the sphere (and on the heightfield
                // the collision code reads).
                const sw::Vec3 direction = glm::normalize(
                    siteUp + siteEast * (eastMetres / static_cast<sw::f32>(kTerraRadius)) +
                    siteNorth * (northMetres / static_cast<sw::f32>(kTerraRadius)));
                const sw::f64 elevation =
                    sw::planet::terrainElevation(terrain, direction);

                const sw::ecs::Entity e = m_world.createEntity();
                TransformComponent transform{};
                m_world.addComponent(e, transform); // metres: parts are life-size
                m_world.addComponent(e, PreviousTransformComponent{});
                m_world.addComponent(e, BoundsComponent{
                                            static_cast<sw::f32>(std::max(
                                                spec.footprintM[0], spec.footprintM[1]))});
                m_world.addComponent(e, MeshComponent{m_partMeshIds.at(definitionId)});
                // Only the BEACON marks the site on the star map. Six
                // markers a few metres apart are six overlapping dots at
                // map scale, which is noise, not information — the site has
                // one pointer, and it is the building whose job that is.
                if (marker.a > 0.0f)
                {
                    m_world.addComponent(e, MapMarkerComponent{marker});
                }
                sw::phys::SurfaceAnchorComponent anchor{};
                anchor.body = terraEntity;
                anchor.localPosition =
                    sw::WorldVec3(direction) * (kTerraRadius + elevation);
                anchor.localRotation = standUp;
                m_world.addComponent(e, anchor);

                sw::factory::BuildingComponent building{};
                building.definitionId = definitionId;
                building.site = hubEntity; // the hub spawns first
                building.category = spec.category;
                building.groundDensity = sw::planet::oreDensity(
                    deposits, direction, sw::res::Resource::IronOre);
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
                m_world.addComponent(e, power);

                if (spec.inventoryVolumeM3 > 0.0)
                {
                    sw::factory::InventoryComponent inventory{};
                    inventory.volumeCapacityM3 = spec.inventoryVolumeM3;
                    m_world.addComponent(e, inventory);
                }
                return e;
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

            const sw::ecs::Entity minerEntity =
                spawnBuilding(sw::parts::kBuildingMiner, 34.0f, 0.0f,
                              sw::factory::kRecipeMineIronOre, {});
            const sw::ecs::Entity refineryEntity =
                spawnBuilding(sw::parts::kBuildingRefinery, 34.0f, -30.0f,
                              sw::factory::kRecipeSmeltIron, {});
            const sw::ecs::Entity storageEntity =
                spawnBuilding(sw::parts::kBuildingStorage, 0.0f, -30.0f, 0u, {});
            spawnBuilding(sw::parts::kBuildingSolarFarm, -34.0f, -15.0f, 0u, {});

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

            // The chain: ore walks to the smelter, iron walks to the silo.
            // Still the F0 link abstraction — F6 replaces it with conveyors.
            if (!minerEntity.isNull() && !refineryEntity.isNull())
            {
                m_world.addComponent(refineryEntity,
                                     sw::factory::ItemLinkComponent{
                                         minerEntity, sw::res::Resource::IronOre, 3.0});
            }
            if (!refineryEntity.isNull() && !storageEntity.isNull())
            {
                m_world.addComponent(storageEntity,
                                     sw::factory::ItemLinkComponent{
                                         refineryEntity, sw::res::Resource::Iron, 3.0});
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

        // ---- asteroid: dynamic body + mining site ---------------------------------
        sw::ecs::Entity asteroidEntity{};
        {
            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            transform.position = stationCenter + sw::WorldVec3{650.0, 120.0, -300.0};
            transform.uniformScale = 120.0f;
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{asteroidBoundsRadius});
            m_world.addComponent(e, MeshComponent{asteroidMeshId});
            m_world.addComponent(e, SpinComponent{{0.2f, 1.0f, 0.1f}, 0.05f});
            m_world.addComponent(e, MapMarkerComponent{{0.85f, 0.65f, 0.35f, 1.0f}});

            const sw::f64 radius = glm::length(transform.position - terraPos0);
            const sw::f64 speed = sw::phys::kepler::circularOrbitSpeed(kMuTerra, radius);
            m_world.addComponent(e, sw::phys::DynamicBodyComponent{
                                        terraVel0 + sw::WorldVec3{speed, 0.0, 0.0}, 5.0e8});

            sw::factory::InventoryComponent hopper{};
            hopper.volumeCapacityM3 = 40.0;
            m_world.addComponent(e, hopper);
            m_world.addComponent(e, sw::factory::MinerComponent{
                                        sw::res::Resource::IronOre, 2.0, 0.0});
            asteroidEntity = e;
        }

        // ---- THE PLAYER ROCKET: a vessel assembled from catalog parts -----------
        // The root entity is the rigid body + controls; every part is its
        // own entity riding the root (entity-per-part is what will make
        // staging, docking and damage cheap later).
        {
            const sw::ecs::Entity root = m_world.createEntity();
            TransformComponent transform{};
            transform.position = stationCenter + sw::WorldVec3{-250.0, 60.0, 400.0};
            m_world.addComponent(root, transform);
            m_world.addComponent(root, snapshotOf(transform));
            m_world.addComponent(root, BoundsComponent{0.1f}); // parts render, not the root
            m_world.addComponent(root, MapMarkerComponent{{0.3f, 1.0f, 0.5f, 1.0f}});
            m_world.addComponent(root, ShipComponent{});
            m_world.addComponent(root, ShipControlsComponent{});
            m_world.addComponent(root, SasComponent{});
            m_world.addComponent(root, sw::parts::VesselComponent{});

            const sw::f64 radius = glm::length(transform.position - terraPos0);
            const sw::f64 speed = sw::phys::kepler::circularOrbitSpeed(kMuTerra, radius);
            // Mass/drag are overwritten by the VesselAssemblySystem each tick.
            m_world.addComponent(root, sw::phys::DynamicBodyComponent{
                                           terraVel0 + sw::WorldVec3{speed, 0.0, 0.0},
                                           4.0e4});
            m_shipEntity = root;

            auto spawnPart = [&](sw::u32 definitionId, const sw::Vec3& localPosition,
                                 const sw::Quat& localRotation = {1, 0, 0, 0},
                                 sw::f32 boundsRadius = 3.0f) {
                const sw::ecs::Entity part = m_world.createEntity();
                TransformComponent partTransform{};
                partTransform.position =
                    transform.position + sw::WorldVec3(localPosition);
                partTransform.rotation = localRotation;
                m_world.addComponent(part, partTransform);
                m_world.addComponent(part, snapshotOf(partTransform));
                m_world.addComponent(part, BoundsComponent{boundsRadius});
                m_world.addComponent(part, MeshComponent{partMeshIds.at(definitionId)});
                sw::parts::PartComponent partComponent{};
                partComponent.definitionId = definitionId;
                partComponent.vessel = root;
                partComponent.localPosition = localPosition;
                partComponent.localRotation = localRotation;
                m_world.addComponent(part, partComponent);
                return part;
            };

            // Stack, nose (-Z) to tail (+Z): every one of the 9 part types,
            // wired together by REAL JOINT ENTITIES (each with its own
            // strength/break force — the structural truth of the vessel).
            auto joint = [&](sw::ecs::Entity a, sw::ecs::Entity b, sw::u8 pointA,
                             sw::u8 pointB, sw::parts::JointType type) {
                const sw::f64 force = std::min(
                    sw::parts::findDefinition(
                        m_world.getComponent<sw::parts::PartComponent>(a).definitionId)
                        ->breakingForceN,
                    sw::parts::findDefinition(
                        m_world.getComponent<sw::parts::PartComponent>(b).definitionId)
                        ->breakingForceN);
                sw::parts::connectParts(m_world, a, b, pointA, pointB, type, force,
                                        force);
            };
            using JT = sw::parts::JointType;
            const auto dock =
                spawnPart(sw::parts::kPartDockingRing, {0.0f, 0.0f, -9.0f}, {1, 0, 0, 0}, 1.1f);
            const auto core =
                spawnPart(sw::parts::kPartCoreStructural, {0.0f, 0.0f, -7.3f}, {1, 0, 0, 0}, 1.8f);
            const auto cargo =
                spawnPart(sw::parts::kPartCargoBaySmall, {0.0f, 0.0f, -4.9f}, {1, 0, 0, 0}, 1.8f);
            const auto decoupler =
                spawnPart(sw::parts::kPartDecouplerFlat, {0.0f, 0.0f, -3.55f}, {1, 0, 0, 0}, 1.2f);
            const sw::ecs::Entity tankA =
                spawnPart(sw::parts::kPartFuelTankMedium, {0.0f, 0.0f, -1.2f},
                          {1, 0, 0, 0}, 2.5f);
            const sw::ecs::Entity tankB =
                spawnPart(sw::parts::kPartFuelTankMedium, {0.0f, 0.0f, 3.0f},
                          {1, 0, 0, 0}, 2.5f);
            const auto engine =
                spawnPart(sw::parts::kPartEngineVector, {0.0f, 0.0f, 6.2f}, {1, 0, 0, 0}, 1.5f);
            // Radial parts sit ON the collider surfaces now (node-accurate);
            // the -X twins carry a 180-degree yaw so their mount faces the hull.
            const sw::Quat flip180{0.0f, 0.0f, 0.0f, 1.0f}; // 180 deg about Z
            const auto finR =
                spawnPart(sw::parts::kPartFinBasic, {1.22f, 0.0f, 3.7f}, {1, 0, 0, 0}, 1.6f);
            const auto finL =
                spawnPart(sw::parts::kPartFinBasic, {-1.22f, 0.0f, 3.7f}, flip180, 1.6f);
            const sw::ecs::Entity battery =
                spawnPart(sw::parts::kPartBatteryPack, {1.25f, 0.0f, -4.9f},
                          {1, 0, 0, 0}, 0.7f);
            const auto solar =
                spawnPart(sw::parts::kPartSolarWing, {-1.25f, 0.0f, -4.9f}, {1, 0, 0, 0}, 3.1f);
            joint(dock, core, 1, 0, JT::Stack); // dock node 1 = bottom
            joint(core, cargo, 1, 0, JT::Stack);
            joint(cargo, decoupler, 1, 0, JT::Stack);
            joint(decoupler, tankA, 1, 0, JT::Stack);
            joint(tankA, tankB, 1, 0, JT::Stack);
            joint(tankB, engine, 1, 0, JT::Stack);
            joint(cargo, battery, 2, 0, JT::Radial);
            joint(cargo, solar, 3, 0, JT::Radial);
            joint(tankB, finR, 2, 0, JT::Radial);
            joint(tankB, finL, 3, 0, JT::Radial);

            // Fill the tanks and half-charge the battery (capacities from
            // the catalog; resources are ordinary volume-bounded cargo).
            for (const sw::ecs::Entity tank : {tankA, tankB})
            {
                sw::factory::InventoryComponent inventory{};
                inventory.volumeCapacityM3 = 21.0;
                sw::factory::inventoryAdd(inventory, sw::res::Resource::Fuel, 16000.0);
                m_world.addComponent(tank, inventory);
            }
            {
                sw::factory::InventoryComponent inventory{};
                inventory.volumeCapacityM3 = 0.12;
                sw::factory::inventoryAdd(inventory,
                                          sw::res::Resource::ElectricCharge, 400.0);
                m_world.addComponent(battery, inventory);
            }
        }

        // ---- station modules (rails around TERRA) + refinery / depot ---------------
        sw::ecs::Entity refineryEntity{};
        for (sw::u32 i = 0; i < kStationModuleCount; ++i)
        {
            const sw::f64 offset =
                (static_cast<sw::f64>(i) / kStationModuleCount - 0.5) * 1.0e-4;
            const sw::phys::KeplerOrbit orbit = sw::phys::kepler::fromElements(
                kMuTerra, kStationOrbitRadius + (static_cast<sw::f64>(i) - 3.5) * 12.0,
                0.0, 0.0, 0.0, 0.0, kStationPhase + offset, 0.0);

            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            sw::phys::kepler::evaluate(orbit, 0.0, transform.position);
            transform.position += terraPos0;
            transform.uniformScale = 30.0f;
            m_world.addComponent(e, transform);
            m_world.addComponent(e, snapshotOf(transform));
            m_world.addComponent(e, BoundsComponent{kCubeBoundsRadius});
            m_world.addComponent(e, MeshComponent{moduleMeshId});
            sw::phys::OnRailsComponent rails{};
            rails.orbit = orbit;
            rails.primary = terraEntity;
            rails.dynamicMass = 2.0e5; // a 200 t module, if ever de-railed
            m_world.addComponent(e, rails);
            m_world.addComponent(e, SpinComponent{{0.3f, 1.0f, 0.2f},
                                                  0.1f + hash01(i) * 0.1f});
            if (i == 0)
            {
                m_world.addComponent(e, MapMarkerComponent{{1.0f, 1.0f, 1.0f, 1.0f}});
                sw::factory::InventoryComponent tanks{};
                tanks.volumeCapacityM3 = 15.0;
                m_world.addComponent(e, tanks);
                m_world.addComponent(e, sw::factory::RefineryComponent{
                                            sw::res::Resource::IronOre,
                                            sw::res::Resource::Iron, 1.0, 0.9, 0.0});
                m_world.addComponent(e, sw::factory::ItemLinkComponent{
                                            asteroidEntity, sw::res::Resource::IronOre,
                                            1.5});
                refineryEntity = e;
            }
            else if (i == 1)
            {
                sw::factory::InventoryComponent silo{};
                silo.volumeCapacityM3 = 60.0;
                m_world.addComponent(e, silo);
                m_world.addComponent(e, sw::factory::ItemLinkComponent{
                                            refineryEntity, sw::res::Resource::Iron, 1.0});
            }
        }
    }

    void StarWorksGame::buildSaveSchema()
    {
        // Stable names + versions. Bump a version whenever the struct layout
        // changes; the loader refuses mismatches instead of guessing.
        m_saveSchema.registerComponent<sw::TransformComponent>("sw.Transform", 1);
        m_saveSchema.registerComponent<sw::PreviousTransformComponent>(
            "sw.PreviousTransform", 1);
        m_saveSchema.registerComponent<sw::phys::DynamicBodyComponent>("phys.DynamicBody",
                                                                       1);
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
        m_saveSchema.registerComponent<SasComponent>("game.Sas", 1);
        m_saveSchema.registerComponent<sw::parts::PartComponent>("parts.Part", 1);
        m_saveSchema.registerComponent<sw::parts::VesselComponent>("parts.Vessel", 1);
        m_saveSchema.registerComponent<sw::parts::JointComponent>("parts.Joint", 1);
        m_saveSchema.registerComponent<sw::factory::InventoryComponent>("factory.Inventory",
                                                                        1);
        m_saveSchema.registerComponent<sw::factory::MinerComponent>("factory.Miner", 1);
        m_saveSchema.registerComponent<sw::factory::RefineryComponent>("factory.Refinery",
                                                                       1);
        m_saveSchema.registerComponent<sw::factory::ItemLinkComponent>("factory.ItemLink",
                                                                       1);
        // F1 — the data-driven industry.
        m_saveSchema.registerComponent<sw::factory::BuildingComponent>("factory.Building",
                                                                       1);
        m_saveSchema.registerComponent<sw::factory::RecipeStateComponent>(
            "factory.RecipeState", 1);
        m_saveSchema.registerComponent<sw::factory::PowerComponent>("factory.Power", 1);
        m_saveSchema.registerComponent<sw::factory::SiteComponent>("factory.Site", 1);
        // v2: + nearRangeM (the pointer steps aside once you have arrived).
        m_saveSchema.registerComponent<sw::factory::BeaconComponent>("factory.Beacon", 2);
        m_saveSchema.registerComponent<BoundsComponent>("game.Bounds", 1);
        m_saveSchema.registerComponent<SpinComponent>("game.Spin", 1);
        m_saveSchema.registerComponent<MeshComponent>("game.Mesh", 2); // v2: transparent
        m_saveSchema.registerComponent<CloudLayerComponent>("game.CloudLayer", 1);
        m_saveSchema.registerComponent<CelestialLodComponent>("game.CelestialLod",
                                                              2); // v2: surfaceStyle
        m_saveSchema.registerComponent<ShipComponent>("game.Ship", 1);
        // v2: + strafeAxis (the EVA sidestep).
        m_saveSchema.registerComponent<ShipControlsComponent>("game.ShipControls", 2);
        m_saveSchema.registerComponent<CapsuleComponent>("game.Capsule", 1);
        m_saveSchema.registerComponent<MapMarkerComponent>("game.MapMarker", 1);
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

        SW_LOG_INFO("Game", "Loaded '{}': {} entities, t={:.1f}s, warp x{:g}",
                    path.string(), m_world.aliveCount(), m_simulation.simulatedSeconds(),
                    kWarpLadder[m_warpIndex]);
    }

    sw::ecs::Entity StarWorksGame::controlledEntity() const
    {
        return (m_evaMode && !m_capsuleEntity.isNull()) ? m_capsuleEntity : m_shipEntity;
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
        // Down to 1.5 km at landing: with 96 cells that is a 31 m grid, fine
        // enough for the gullies and benches the 16-octave heightfield now
        // carries. (It used to bottom out at 4 km / 125 m cells, which could
        // not show anything smaller than a hill.)
        const sw::f64 extent =
            std::clamp(std::max(seaAltitude, 250.0) * 6.0, 1.5e3, 4.0e5);

        const sw::f64 now = clock().totalSeconds();
        const bool moved =
            static_cast<sw::f64>(glm::distance(centerDir, m_terrainCenterDir)) *
                primary.bodyRadius >
            extent * 0.30;
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

        // Everything the build needs is captured BY VALUE: the job never
        // touches the world, the renderer or any component.
        const sw::planet::TerrainComponent terrainCopy = *terrain;
        const sw::f64 radius = primary.bodyRadius;
        m_terrainPendingBody = primary.entity;
        m_terrainJob.store(TerrainJob::Running, std::memory_order_release);
        threadPool().submit([this, terrainCopy, surfaceStyle, centerDir, extent,
                             radius]() {
            buildTerrainPatch(terrainCopy, surfaceStyle, centerDir, extent, radius);
            m_terrainJob.store(TerrainJob::Ready, std::memory_order_release);
        });
    }

    void StarWorksGame::buildTerrainPatch(const sw::planet::TerrainComponent& terrain,
                                          sw::i32 surfaceStyle,
                                          const sw::Vec3& centerDir, sw::f64 extent,
                                          sw::f64 radius)
    {
        // ---- the grid: a tangent plane projected onto the sphere ----------
        constexpr sw::u32 kCells = 96;
        constexpr sw::u32 kVerts = kCells + 1;
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
            }
        }

        sw::MeshData mesh;
        mesh.vertices.resize(kVerts * kVerts);
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

        m_terrainPendingMesh = std::move(mesh);
        m_terrainPendingOrigin = origin;
        m_terrainPendingCenterDir = centerDir;
        m_terrainPendingExtent = extent;
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

        sw::f64 step = 1.0;
        if (input().isKeyDown(sw::KeyCode::LeftShift)) { step *= 10.0; }
        if (input().isKeyDown(sw::KeyCode::LeftControl)) { step *= 0.1; }

        bool edited = false;
        auto adjust = [&](sw::KeyCode plus, sw::KeyCode minus, sw::f64& value,
                          sw::f64 scale) {
            if (input().wasKeyPressed(plus)) { value += step * scale; edited = true; }
            if (input().wasKeyPressed(minus)) { value -= step * scale; edited = true; }
        };
        adjust(sw::KeyCode::L, sw::KeyCode::J, m_nodeTime, 10.0);  // 10 s per tap
        adjust(sw::KeyCode::I, sw::KeyCode::K, m_nodePrograde, 1.0);
        adjust(sw::KeyCode::O, sw::KeyCode::U, m_nodeNormal, 1.0);
        adjust(sw::KeyCode::Y, sw::KeyCode::H, m_nodeRadial, 1.0);
        m_nodeTime = std::max(m_nodeTime, m_physicsLane->presentSeconds() + 5.0);
        if (edited)
        {
            m_lastPredictionSeconds = -1.0e9; // instant visual feedback
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
            return;
        }

        const sw::f64 startTime = m_physicsLane->presentSeconds();
        sw::space::PredictionSettings settings{};
        if (m_nodeActive)
        {
            // Pre-burn plan stops AT the node; the node plan takes over.
            settings.horizonSeconds = std::max(m_nodeTime - startTime, 1.0);
        }
        sw::space::predictTrajectory(m_celestialIndex, transform.position,
                                     controlledVelocity(), startTime, settings,
                                     m_prediction);

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
        m_nodePostBurnVelocity = nodeVelocity + dv;
        m_nodePrimaryIndex = nodePrimary;
        m_nodeRelativePosition = relativePosition;

        sw::space::PredictionSettings nodeSettings{}; // full horizon again
        sw::space::predictTrajectory(m_celestialIndex, nodePosition,
                                     m_nodePostBurnVelocity, m_nodeTime, nodeSettings,
                                     m_nodePrediction);
    }

    void StarWorksGame::toggleEva()
    {
        if (!m_evaMode && m_capsuleEntity.isNull())
        {
            // First EVA: spawn the capsule beside the ship, co-moving.
            const auto& shipTransform =
                m_world.getComponent<TransformComponent>(m_shipEntity);
            const sw::WorldVec3 shipVelocity = controlledVelocity();

            const sw::ecs::Entity e = m_world.createEntity();
            TransformComponent transform{};
            transform.position = shipTransform.position +
                                 sw::WorldVec3(shipTransform.rotation *
                                               sw::Vec3{12.0f, 0.0f, 0.0f});
            m_world.addComponent(
                e, PreviousTransformComponent{transform.position, transform.rotation});
            m_world.addComponent(e, transform);
            m_world.addComponent(e, BoundsComponent{1.2f});
            m_world.addComponent(e, MeshComponent{m_capsuleMeshIndex});
            m_world.addComponent(e, MapMarkerComponent{{1.0f, 0.8f, 0.2f, 1.0f}});
            // The suit STANDS on the ground rather than being buried to the
            // waist: the capsule mesh is a 1 m radius, 1 m barrel centred on
            // the origin, so it reaches 1 m below its own transform.
            m_world.addComponent(e, sw::phys::GroundHullComponent{
                                        {0.0f, 0.0f, 0.0f}, {0.5f, 1.0f, 0.5f}});
            m_world.addComponent(e, CapsuleComponent{});
            m_world.addComponent(e, ShipControlsComponent{});
            sw::phys::DynamicBodyComponent body{};
            body.velocity = shipVelocity;
            body.mass = 120.0;
            body.ballisticFactor = 0.01; // draggier than the ship
            m_world.addComponent(e, body);
            m_capsuleEntity = e;
        }
        m_evaMode = !m_evaMode;
        SW_LOG_INFO("Game", "{}", m_evaMode ? "EVA: controlling capsule"
                                            : "Back aboard: controlling ship");
    }

    void StarWorksGame::updateWarp()
    {
        if (input().wasKeyPressed(sw::KeyCode::Period) && m_warpIndex + 1 < kWarpSteps)
        {
            ++m_warpIndex;
        }
        if (input().wasKeyPressed(sw::KeyCode::Comma) && m_warpIndex > 0)
        {
            --m_warpIndex;
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

        const sw::f32 maxAllowed = maxWarpForAltitude(minAltitude);
        while (m_warpIndex > 0 && kWarpLadder[m_warpIndex] > maxAllowed)
        {
            --m_warpIndex;
            SW_LOG_INFO("Game", "Warp limited to x{:g} (altitude {:.0f} km)",
                        kWarpLadder[m_warpIndex], minAltitude / 1000.0);
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
        // Idle both control blocks, then feed the controlled one.
        auto& shipControls = m_world.getComponent<ShipControlsComponent>(m_shipEntity);
        shipControls = {};
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

        ShipControlsComponent& controls =
            (m_evaMode && capsuleControls != nullptr) ? *capsuleControls : shipControls;

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

        // ---- ground lock: when the trajectory is no longer an orbit ----------
        // (impact predicted, or resting on the surface) AND the craft is low
        // relative to the BODY'S OWN SIZE, the camera stops tumbling with
        // the craft and levels itself on the local horizon instead.
        bool wantGroundLock = false;
        sw::Vec3 radialUp{0.0f, 1.0f, 0.0f};
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
                const sw::f64 altitude = distance - primary.bodyRadius;
                const bool low = altitude < 0.03 * primary.bodyRadius;

                bool suborbital = false;
                if (!m_prediction.empty())
                {
                    suborbital = m_prediction[0].endReason ==
                                 sw::space::SegmentEnd::Impact;
                }
                const auto* dynamicBody =
                    m_world.tryGetComponent<sw::phys::DynamicBodyComponent>(target);
                const bool grounded =
                    dynamicBody != nullptr && dynamicBody->isGrounded != 0;
                wantGroundLock = low && (suborbital || grounded);
            }
        }
        const sw::f32 blendTarget = wantGroundLock ? 1.0f : 0.0f;
        m_groundCamBlend +=
            (blendTarget - m_groundCamBlend) * std::min(1.0f, deltaSeconds * 2.0f);
        const sw::f32 blend = m_groundCamBlend;

        // Offset frame: the craft's own rotation, blended toward a leveled
        // "ground frame" (craft heading projected on the horizon plane).
        sw::Quat offsetRotation = rotation;
        if (blend > 0.001f)
        {
            const sw::Vec3 forwardShip = rotation * sw::math::kWorldForward;
            sw::Vec3 horizontal =
                forwardShip - radialUp * glm::dot(forwardShip, radialUp);
            const sw::f32 horizontalLength = glm::length(horizontal);
            if (horizontalLength > 1.0e-3f)
            {
                horizontal /= horizontalLength;
                const sw::Vec3 right =
                    glm::normalize(glm::cross(horizontal, radialUp));
                const sw::Quat groundRotation =
                    glm::quat_cast(sw::Mat3{right, radialUp, -horizontal});
                offsetRotation = glm::slerp(rotation, groundRotation, blend);
            }
        }

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

        const sw::Vec3 chaseOffset =
            (m_evaMode ? sw::Vec3{0.0f, 2.0f, 9.0f} : sw::Vec3{0.0f, 12.0f, 42.0f}) *
            m_chaseZoom;
        const sw::WorldVec3 cameraPosition =
            position + sw::WorldVec3(offsetRotation * (userOrbit * chaseOffset));
        m_camera.setPosition(cameraPosition);

        const sw::Vec3 forward = glm::normalize(sw::Vec3(position - cameraPosition));
        // Camera up: craft-up in orbit, local vertical near the ground.
        const sw::Vec3 shipUp = rotation * sw::Vec3{0.0f, 1.0f, 0.0f};
        const sw::Vec3 targetUp = glm::normalize(glm::mix(shipUp, radialUp, blend));
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
        // ESC quits the game — except in the hangar, where it belongs to
        // the editor (drop / put back the held part).
        if (input().wasKeyPressed(sw::KeyCode::Escape) && !m_editorMode)
        {
            window().requestClose();
            return;
        }

        // --- mode toggles -------------------------------------------------------
        if (input().wasKeyPressed(sw::KeyCode::M))
        {
            m_mapView = !m_mapView;
            SW_LOG_INFO("Game", "Star map {}", m_mapView ? "opened" : "closed");
        }
        if (input().wasKeyPressed(sw::KeyCode::G) && !m_mapView)
        {
            toggleEva();
        }
        if (input().wasKeyPressed(sw::KeyCode::V))
        {
            m_speedSurfaceRelative = !m_speedSurfaceRelative;
        }
        if (input().wasKeyPressed(sw::KeyCode::Tab))
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
        if (input().wasKeyPressed(sw::KeyCode::Space))
        {
            m_simulation.setPaused(!m_simulation.isPaused());
            SW_LOG_INFO("Game", "Simulation {}", m_simulation.isPaused() ? "paused" : "resumed");
        }
        if (m_mapView)
        {
            updateManeuverNodeInput();
        }

        // ---- vessel editor (B) + staging (Z) -----------------------------------
        if (input().wasKeyPressed(sw::KeyCode::B) && !m_mapView && !m_evaMode)
        {
            if (m_editorMode) { exitEditor(); }
            else { enterEditor(); }
        }
        if (m_editorMode)
        {
            updateEditor();
        }
        if (input().wasKeyPressed(sw::KeyCode::Z) && !m_editorMode && !m_mapView)
        {
            // Staging: fire the first decoupler on the ship (nose-most last).
            sw::ecs::Entity decoupler{};
            m_world.forEach<sw::parts::PartComponent>(
                [&](sw::ecs::Entity entity, sw::parts::PartComponent& part) {
                    if (part.vessel != m_shipEntity || !decoupler.isNull())
                    {
                        return;
                    }
                    const auto* definition =
                        sw::parts::findDefinition(part.definitionId);
                    if (definition != nullptr &&
                        definition->type == sw::parts::PartType::Decoupler)
                    {
                        decoupler = entity;
                    }
                });
            if (!decoupler.isNull())
            {
                const sw::ecs::Entity separated =
                    sw::parts::decoupleAt(m_world, decoupler);
                if (!separated.isNull())
                {
                    // splitVessel already gave it Transform/Previous/body.
                    m_world.addComponent(separated, BoundsComponent{0.1f});
                    m_world.addComponent(
                        separated, MapMarkerComponent{{0.6f, 0.6f, 0.6f, 1.0f}});
                    SW_LOG_INFO("Game", "STAGING: decoupler fired, stage separated");
                }
            }
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

        // ---- SAS: T cycles OFF -> PGD -> RTG; clickable buttons too -----------
        if (input().wasKeyPressed(sw::KeyCode::T))
        {
            m_sasMode = (m_sasMode + 1) % 3;
        }
        if (input().wasKeyPressed(sw::KeyCode::P) && !m_editorMode)
        {
            cyclePilotedVessel(); // fly any built vessel
        }
        handleHudClicks();
        if (auto* sas = m_world.tryGetComponent<SasComponent>(m_shipEntity))
        {
            sas->mode = m_sasMode;
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

        m_simulation.advance(m_world, deltaSeconds, &threadPool());
        m_commands.playback(m_world);

        // Fresh hierarchy snapshot for the map, HUD and flight plan.
        m_celestialIndex.rebuild(m_world);
        refreshPrediction();
        updateTerrainPatch();

        updateReentryEffects(deltaSeconds);

        if (m_shipMode && !m_mapView)
        {
            updateChaseCamera(deltaSeconds);
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

        // ---- current orbit around the primary: APO / PER / period ----------------
        auto formatEta = [](sw::f64 seconds) {
            return (seconds >= 3600.0) ? std::format("{:.1f} H", seconds / 3600.0)
                                       : std::format("{:.0f} S", seconds);
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
        const auto& ship = m_world.getComponent<ShipComponent>(m_shipEntity);
        hudText(std::format("THR {:.0f}%  WARP X{:g}", ship.throttle * 100.0f,
                            kWarpLadder[m_warpIndex]),
                kX, y, kLine, dim);
        y += kLine * 1.3f;

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

        // ---- maneuver node status --------------------------------------------
        if (m_nodeActive)
        {
            // Far from the node: show the PLANNED dv. Within 30 s (or past
            // it): the LIVE remaining vector — burn until it hits zero.
            const sw::f64 timeToNode = m_nodeTime - time;
            const sw::f64 plannedDv =
                std::sqrt(m_nodePrograde * m_nodePrograde +
                          m_nodeNormal * m_nodeNormal + m_nodeRadial * m_nodeRadial);
            const sw::f64 remainingDv =
                glm::length(m_nodePostBurnVelocity - controlledVelocity());
            const bool burnWindow = timeToNode < 30.0;
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
            }
        }

        if (!m_mapView && !m_editorMode)
        {
            collectNavball();
            collectSasButtons();
        }
        else if (m_mapView && !m_editorMode)
        {
            collectMapButtons();
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

    sw::ecs::Entity StarWorksGame::instantiateBlueprint(sw::ecs::Entity existingRoot)
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
                    sw::parts::expandPartColliderBounds(*definition, bp.localPosition,
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
            // NEW vessel: born on the LAUNCH PAD, standing on Terra's
            // terrain next to the mining outpost, co-rotating with the
            // planet, nose to the sky.
            const auto& terra = m_world.getComponent<TransformComponent>(m_terraEntity);
            const auto& gravity =
                m_world.getComponent<sw::phys::GravitySourceComponent>(m_terraEntity);
            // 120 m east of the site hub: the same surveyed ground, so the
            // pad is on land and the factory is walking distance away.
            const sw::Vec3 siteDir = terraStartSite();
            const sw::Vec3 padEast = glm::normalize(glm::cross(
                (std::abs(siteDir.y) < 0.9f) ? sw::Vec3{0.0f, 1.0f, 0.0f}
                                             : sw::Vec3{1.0f, 0.0f, 0.0f},
                siteDir));
            const sw::Vec3 padDir = glm::normalize(
                siteDir + padEast * (120.0f / static_cast<sw::f32>(kTerraRadius)));
            const sw::f64 elevation =
                sw::planet::terrainElevation(presetTerra(), padDir);
            // A new rocket stands TAIL DOWN, so its model +Z is the axis
            // pointing at the ground: the clearance is exactly how far the
            // hull reaches along +Z. (The 11 m constant this replaces was a
            // guess at one particular rocket's half length.)
            const sw::f64 clearance =
                static_cast<sw::f64>(hull.centre.z + hull.halfExtents.z);
            const sw::WorldVec3 padLocal =
                sw::WorldVec3(padDir) * (kTerraRadius + elevation + clearance);
            // Same rule: a planet-radius offset is rotated with the f64
            // spin, or the pad lands a metre from where it was surveyed.
            const auto* terraSpin =
                m_world.tryGetComponent<sw::phys::GravitySourceComponent>(m_terraEntity);
            const glm::dquat terraRotation =
                (terraSpin != nullptr) ? sw::phys::spinRotation(*terraSpin)
                                       : glm::dquat(terra.rotation);
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

    void StarWorksGame::hangarBuild()
    {
        if (m_blueprint.empty())
        {
            return;
        }
        const sw::ecs::Entity vessel = instantiateBlueprint(m_hangarSource);
        if (m_hangarSource.isNull())
        {
            m_shipEntity = vessel; // fly the new build from the pad
            m_sasMode = 0;
            SW_LOG_INFO("Game", "HANGAR: new vessel BUILT on the launch pad");
        }
        else
        {
            SW_LOG_INFO("Game", "HANGAR: modifications applied to loaded vessel");
        }
        exitEditor();
    }

    void StarWorksGame::cyclePilotedVessel()
    {
        std::vector<sw::ecs::Entity> pilotable;
        m_world.forEach<ShipComponent>([&](sw::ecs::Entity entity, ShipComponent&) {
            pilotable.push_back(entity);
        });
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
        m_evaMode = false;
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
            {"BUILD", 204, true},
        };
        constexpr sw::f32 kButtonWidth = 0.135f;
        constexpr sw::f32 kButtonHeight = 0.072f;
        constexpr sw::f32 kButtonGap = 0.016f;
        sw::f32 buttonX = -0.46f;
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

    void StarWorksGame::collectSasButtons()
    {
        // First clickable UI: three buttons above the bottom-left corner.
        m_hudButtons.clear();
        constexpr sw::f32 kHeight = 0.062f;
        constexpr sw::f32 kWidth = 0.115f;
        constexpr sw::f32 kGap = 0.018f;
        const sw::f32 y0 = 0.87f;
        sw::f32 x0 = -0.97f;

        const char* labels[3] = {"SAS", "PGD", "RTG"};
        for (sw::u32 id = 0; id < 3; ++id)
        {
            const sw::f32 x1 = x0 + kWidth;
            const bool active = (id == 0) ? (m_sasMode == 0) : (m_sasMode == id);
            const sw::Vec4 background =
                active ? sw::Vec4{0.15f, 0.55f, 0.30f, 0.85f}
                       : sw::Vec4{0.16f, 0.22f, 0.30f, 0.65f};

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

            hudText(labels[id], x0 + 0.022f, y0 + 0.015f, 0.036f,
                    active ? sw::Vec4{0.9f, 1.0f, 0.9f, 1.0f}
                           : sw::Vec4{0.7f, 0.8f, 0.9f, 0.9f});

            m_hudButtons.push_back({x0, y0, x1, y0 + kHeight, id});
            x0 = x1 + kGap;
        }
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
                if (button.id == 300) // map: fly the next vessel
                {
                    cyclePilotedVessel();
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
                    else if (button.id == 204) { hangarBuild(); }
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
                // SAS button = off; PGD/RTG toggle their mode.
                if (button.id == 0) { m_sasMode = 0; }
                else { m_sasMode = (m_sasMode == button.id) ? 0 : button.id; }
                SW_LOG_INFO("Game", "SAS mode: {}",
                            m_sasMode == 0 ? "OFF"
                                           : (m_sasMode == 1 ? "PROGRADE"
                                                             : "RETROGRADE"));
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
            const sw::WorldVec3 burnVector =
                m_nodePostBurnVelocity - controlledVelocity();
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

        // Samples a conic around a primary's CURRENT world position over
        // [t0, t1] — KSP map convention: patches are drawn in the frame of
        // where their primary is NOW.
        auto plotConic = [&](const sw::phys::KeplerOrbit& orbit,
                             const sw::WorldVec3& primaryPosition, const sw::Vec4& color,
                             sw::f64 t0, sw::f64 t1, sw::u32 samples) {
            for (sw::u32 sample = 0; sample < samples; ++sample)
            {
                const sw::f64 ts =
                    t0 + (t1 - t0) * static_cast<sw::f64>(sample) / samples;
                sw::WorldVec3 relativePoint{};
                sw::phys::kepler::evaluate(orbit, ts, relativePoint);
                plotDot(primaryPosition + relativePoint, color, 1.0f);
            }
        };

        // Full closed orbit (elliptic only).
        auto plotFullOrbit = [&](const sw::phys::KeplerOrbit& orbit,
                                 const sw::WorldVec3& primaryPosition,
                                 const sw::Vec4& color) {
            if (!orbit.isHyperbolic())
            {
                plotConic(orbit, primaryPosition, color, time,
                          time + sw::phys::kepler::period(orbit), kTrajectorySamples);
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

                // A closed, event-free orbit draws one full revolution;
                // every other patch draws exactly its [start, end] arc.
                sw::f64 t1 = segment.endTime;
                if (segment.endReason == sw::space::SegmentEnd::Horizon &&
                    !segment.orbit.isHyperbolic())
                {
                    t1 = std::min(segment.endTime,
                                  segment.startTime +
                                      sw::phys::kepler::period(segment.orbit));
                }
                plotConic(segment.orbit, primaryPosition, color, segment.startTime, t1,
                          kPredictionDisplaySamples);

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
            plotDot(m_celestialIndex.positionAt(m_nodePrimaryIndex, time) +
                        m_nodeRelativePosition,
                    {0.75f, 0.4f, 1.0f, 1.0f}, 4.0f);
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
