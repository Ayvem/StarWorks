#pragma once

// ============================================================================
// GameInternal.hpp — helpers shared by the StarWorksGame translation units.
//
// This is everything that used to live in StarWorksGame.cpp's big anonymous
// namespace: physical constants, HUD palette, mesh builders, small pure
// helpers. It is internal to the Game target — the engine never includes it.
// Functions are inline and constants inline constexpr so that any Game*.cpp
// may include this header without ODR trouble. A helper used by a single
// theme belongs in that file's own anonymous namespace instead.
// ============================================================================

#include "StarWorksGame.hpp"

#include <algorithm>
#include <chrono>
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
            case SasComponent::kRadialOut: return "RADIAL OUT";
            case SasComponent::kRadialIn: return "RADIAL IN";
            case SasComponent::kNormal: return "NORMAL";
            case SasComponent::kAntiNormal: return "ANTI-NORMAL";
            default: return "OFF";
            }
        }

        // ---- real-world dimensions and gravity (meters, m^3/s^2) --------------
        // The named constants below are the bodies the game logic refers to
        // by name (spawn site, menu backdrop, SOI checks). The REST of the
        // solar system lives in the table in GameScene.cpp. All values real.
        inline constexpr sw::f64 kMuSol = 1.32712440018e20;
        // IAU 2015 Resolution B3 nominal solar radius. It was 6.9634e8, an
        // older figure and 0.09% high — inside anything anyone could see, and
        // corrected anyway because Tools/solar_scale/check_scale.py is now
        // watching and a checker that is allowed one exception is a checker.
        inline constexpr sw::f64 kSolRadius = 6.957e8;
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

        // ---- the rest of the solar system (real values) --------------------
        // SOI radii computed as r = a * (mu / mu_parent)^(2/5), like above.
        inline constexpr sw::f64 kMercuryRadius = 2.4397e6;
        inline constexpr sw::f64 kMuMercury = 2.2032e13;
        inline constexpr sw::f64 kMercurySma = 5.791e10;
        inline constexpr sw::f64 kMercurySoi = 1.12e8;
        inline constexpr sw::f64 kVenusRadius = 6.0518e6;
        inline constexpr sw::f64 kMuVenus = 3.24859e14;
        inline constexpr sw::f64 kVenusSma = 1.0821e11;
        inline constexpr sw::f64 kVenusSoi = 6.16e8;
        inline constexpr sw::f64 kJupiterRadius = 6.9911e7;
        inline constexpr sw::f64 kMuJupiter = 1.26687e17;
        inline constexpr sw::f64 kJupiterSma = 7.7857e11;
        inline constexpr sw::f64 kJupiterSoi = 4.82e10;
        inline constexpr sw::f64 kSaturnRadius = 5.8232e7;
        inline constexpr sw::f64 kMuSaturn = 3.7931e16;
        inline constexpr sw::f64 kSaturnSma = 1.4335e12;
        inline constexpr sw::f64 kSaturnSoi = 5.46e10;
        inline constexpr sw::f64 kUranusRadius = 2.5362e7;
        inline constexpr sw::f64 kMuUranus = 5.7940e15;
        inline constexpr sw::f64 kUranusSma = 2.8725e12;
        inline constexpr sw::f64 kUranusSoi = 5.18e10;
        inline constexpr sw::f64 kNeptuneRadius = 2.4622e7;
        inline constexpr sw::f64 kMuNeptune = 6.8365e15;
        inline constexpr sw::f64 kNeptuneSma = 4.4951e12;
        inline constexpr sw::f64 kNeptuneSoi = 8.66e10;

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
        // Neptune orbits at 4.5e12 m and Tau Ceti is at 1.13e17: the map has
        // to be able to frame the solar system AND the neighbourhood, so the
        // zoom range now spans ten orders of magnitude.
        inline constexpr sw::f64 kMapMaxHeight = 1.6e17;
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

        // ---- where a frame went ------------------------------------------------
        //
        // A HITCH IS NOT A FRAME RATE. "It freezes but the counter says 200
        // fps" is not a contradiction and not a mistake by the player: an
        // average over a second hides a single frame that took a third of one.
        // So the frame has to be able to say which PART of it was slow, and
        // the only honest way to know that is to time the parts.
        //
        // The timer ACCUMULATES rather than assigns, because a phase can run
        // more than once in a frame and because two of them nest.
        struct PhaseTimer
        {
            sw::f64& sink;
            std::chrono::steady_clock::time_point start =
                std::chrono::steady_clock::now();
            PhaseTimer(const PhaseTimer&) = delete;
            PhaseTimer& operator=(const PhaseTimer&) = delete;
            explicit PhaseTimer(sw::f64& target) : sink(target) {}
            ~PhaseTimer()
            {
                sink += std::chrono::duration<sw::f64, std::milli>(
                            std::chrono::steady_clock::now() - start)
                            .count();
            }
        };

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
        //
        // AND THE TOP THREE ARE FOR LEAVING THE SYSTEM. Proxima is 4.0e16
        // metres away; at ten million a crossing is a fortnight of real
        // sitting, at a billion it is seven minutes and at ten billion it is
        // forty seconds. Each unlocks at its own DISTANCE FROM THE STAR — five,
        // fifty and a thousand billion kilometres — because the sphere of
        // influence, which is what they were gated on first, is two
        // light-years wide and would have opened the top rung after the part
        // of the journey that needed it. See phys::maxWarpForSpace.
        inline constexpr sw::f32 kWarpLadder[] = {
            1.0f,         2.0f,          5.0f,         10.0f,      50.0f,
            100.0f,       1000.0f,       10000.0f,     100000.0f,  1000000.0f,
            10000000.0f,  100000000.0f,  1000000000.0f, 10000000000.0f};
        inline constexpr sw::u32 kWarpSteps = static_cast<sw::u32>(std::size(kWarpLadder));

        /// The warp rate as a pilot reads it. "1E+07" is a number a compiler
        /// prints; X10M is a number a person reads.
        [[nodiscard]] inline std::string warpText(sw::f32 rate)
        {
            // A BILLION NEEDS ITS OWN LETTER. Without this branch the top rung
            // reads "X1000M", which is not wrong so much as unreadable — the
            // eye has to count zeroes to tell it from the rung below it, and
            // the rung below it is the one you must drop to before you can
            // enter a system.
            if (rate >= 1.0e9f)
            {
                return std::format("{:.0f}B", rate / 1.0e9f);
            }
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

        /// The one event kind this build speaks: "my craft was here, at this
    /// instant". Everything else a player does will join it here, and the
    /// Timeline treats them all the same way.
    inline constexpr sw::u32 kNetEventBeacon = 1;

    /// PHYSICS WARP: the world stays fully simulated — drag, thrust,
    /// collisions, contact — up to this time scale; beyond it everything
    /// rides analytic rails and the engines go out.
    ///
    /// x100 (F16), and the old x5 was never a stability limit. The Physics
    /// lane integrates a FIXED 1/50 s step at every time scale: warp changes
    /// how many steps run per rendered frame, never how long one is. So the
    /// integrator at x100 is doing exactly the arithmetic it does at x1,
    /// a hundred times over — same truncation error per step, same contact
    /// solver, same everything. What x5 really encoded was the lane's
    /// catch-up budget of sixteen ticks a frame, and that is now raised on
    /// demand (SimulationLane::setMaxTicksPerFrame).
    ///
    /// The reason to want it is ION PROPULSION. A plasma engine pushing
    /// 500 tonnes at 88 kN is 0.18 m/s^2: four hours of real burning for a
    /// 2.5 km/s plane change, and rails warp cannot help because rails have
    /// no engines. At x100 that is two and a half minutes, with every
    /// newton of it actually integrated.
    inline constexpr sw::f32 kMaxPhysicsWarp = 100.0f;

    /// ...except in the air, where the limit is not arithmetic but human.
    /// Flying is flying: an aerodynamic vehicle at x100 travels a kilometre
    /// between two of your heartbeats, and no reflex closes that loop. The
    /// step down happens exactly at the top of the atmosphere — entering the
    /// air is the one boundary a pilot can feel, so it is the one the
    /// ceiling should be tied to.
    inline constexpr sw::f32 kMaxAtmosphericWarp = 5.0f;

        /// The ceiling where this craft is, given the body it is near. The
        /// rule itself lives in the engine (phys::maxWarpForAltitude) so it
        /// can be tested without a window; this only supplies the policy.
        [[nodiscard]] inline sw::f32 maxWarpForAltitude(sw::f64 altitudeMeters,
                                                        sw::f64 atmosphereTopMeters,
                                                        sw::f64 bodyRadiusMeters)
        {
            return static_cast<sw::f32>(sw::phys::maxWarpForAltitude(
                altitudeMeters, atmosphereTopMeters, bodyRadiusMeters,
                static_cast<sw::f64>(kMaxAtmosphericWarp),
                static_cast<sw::f64>(kMaxPhysicsWarp)));
        }

        /// How many catch-up ticks a 50 Hz lane needs to actually hold this
        /// time scale, sized for a 60 FPS frame and bounded so a pathological
        /// rung cannot ask for a frame that never ends. Below that, physics
        /// warp still runs — just slower than the number on the HUD says,
        /// which is why the HUD now says both.
        [[nodiscard]] inline sw::u32 physicsTickBudget(sw::f32 rate)
        {
            constexpr sw::u32 kRestingTicks = 16;   // one hitch frame at x1
            constexpr sw::u32 kCeilingTicks = 128;  // x100 at 60 FPS, with slack
            const sw::f32 wanted = std::ceil(rate * 50.0f / 60.0f);
            return std::clamp(static_cast<sw::u32>(wanted), kRestingTicks,
                              kCeilingTicks);
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

        /// Density of whatever `recipeId` extracts, under `up`. 0 when the
        /// body has no geology, or when the recipe digs for nothing.
        ///
        /// Shared rather than private to the placement code because the
        /// machine's own panel prints the same number beside every recipe it
        /// could run. Two implementations of "how rich is this ground" is how
        /// a mine ends up displaying one figure and being paid another.
        [[nodiscard]] inline sw::f32 depositDensityFor(
            const sw::planet::DepositComponent* deposits, const sw::Vec3& up,
            sw::u32 recipeId)
        {
            const sw::res::Resource resource = sw::factory::minedResource(recipeId);
            if (deposits == nullptr || resource == sw::res::Resource::Count)
            {
                return 0.0f;
            }
            return sw::planet::oreDensity(*deposits, up, resource);
        }

        /// The body-fixed direction a surface building stands on, and the
        /// deposits of the body it stands on. False for anything not anchored
        /// to a world — a machine on a vessel has no ground to sample.
        [[nodiscard]] inline bool buildingGround(const sw::ecs::World& world,
                                                 sw::ecs::Entity entity,
                                                 const sw::planet::DepositComponent*& outDeposits,
                                                 sw::Vec3& outUp)
        {
            auto& mutableWorld = const_cast<sw::ecs::World&>(world);
            const auto* anchor =
                mutableWorld.tryGetComponent<sw::phys::SurfaceAnchorComponent>(entity);
            if (anchor == nullptr || !(glm::length(anchor->localPosition) > 1.0))
            {
                return false;
            }
            outUp = sw::Vec3(glm::normalize(anchor->localPosition));
            outDeposits =
                mutableWorld.tryGetComponent<sw::planet::DepositComponent>(anchor->body);
            return true;
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
            // YEARS, because interstellar arrived. Four light-years at a
            // hundred kilometres a second is twelve thousand of them, and
            // "108000000.0 H" is not a number anybody reads — it was the first
            // thing the interstellar guidance panel printed.
            if (magnitude >= 3.15576e7)
            {
                return std::format("{}{:.1f} YR", sign, magnitude / 3.15576e7);
            }
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
            Terra = 0,
            Luna = 1,
            Mars = 2,
            // The landable solar system (terrain presets 3-13, see
            // Planet/Terrain.hpp and its GLSL mirror).
            Mercury = 3,
            Io = 4,
            Europa = 5,
            Ganymede = 6,
            Callisto = 7,
            Titan = 8,
            Enceladus = 9,
            Rhea = 10,
            Titania = 11,
            Oberon = 12,
            Triton = 13,
            // Gas worlds (style >= 20): no ground, the albedo is a banded
            // cloud deck. 24 is Venus — a rocky planet whose visible surface
            // is nonetheless a cloud deck nobody sees through.
            Jupiter = 20,
            Saturn = 21,
            Uranus = 22,
            Neptune = 23,
            Venus = 24,
        };

        [[nodiscard]] inline bool isGasStyle(sw::i32 style) { return style >= 20; }

        /// CPU TWIN of gasGiantAlbedo() in Shaders/PlanetSurface.glsl — the
        /// vertex-coloured LODs are all anyone sees beyond close orbit, so
        /// they must agree with the fragment path to the digit.
        [[nodiscard]] inline sw::Vec3 gasGiantAlbedo(sw::i32 style, const sw::Vec3& dir,
                                                     sw::f32 footprint)
        {
            using sw::math::fbm3;
            using sw::math::smoothstepf;
            sw::Vec3 zone{};
            sw::Vec3 belt{};
            sw::f32 bands = 0.0f;   // base frequency of the band field
            sw::f32 edge = 0.0f;    // how sharp a belt's edge is
            sw::f32 shear = 0.0f;   // how hard the zonal flow bends it
            sw::f32 contrast = 0.0f;
            sw::f32 churnAmount = 0.0f;
            if (style == 20) // Jupiter: the loudest banding in the system
            {
                zone = {1.00f, 0.96f, 0.86f}; belt = {0.42f, 0.24f, 0.14f};
                bands = 6.0f; edge = 0.07f; shear = 0.90f; contrast = 1.00f;
                churnAmount = 0.16f;
            }
            else if (style == 21) // Saturn: the same machinery under a haze
            {
                zone = {1.00f, 0.94f, 0.76f}; belt = {0.56f, 0.42f, 0.24f};
                bands = 5.0f; edge = 0.11f; shear = 0.50f; contrast = 0.85f;
                churnAmount = 0.10f;
            }
            else if (style == 22) // Uranus: almost featureless, and that is the point
            {
                zone = {0.72f, 0.90f, 0.90f}; belt = {0.50f, 0.72f, 0.80f};
                bands = 3.0f; edge = 0.26f; shear = 0.25f; contrast = 0.35f;
                churnAmount = 0.05f;
            }
            else if (style == 23) // Neptune: dark blue with bright methane cirrus
            {
                zone = {0.38f, 0.54f, 0.94f}; belt = {0.09f, 0.16f, 0.50f};
                bands = 4.0f; edge = 0.13f; shear = 0.70f; contrast = 0.95f;
                churnAmount = 0.14f;
            }
            else // Venus: a cloud deck, not a banded atmosphere
            {
                zone = {1.00f, 0.95f, 0.78f}; belt = {0.74f, 0.64f, 0.44f};
                bands = 3.0f; edge = 0.24f; shear = 0.60f; contrast = 0.45f;
                churnAmount = 0.08f;
            }

            // ZONAL SHEAR: six times finer in latitude than in longitude,
            // which is what turns round eddies into ribbons. See the GLSL.
            const sw::Vec3 flow{dir.x, dir.y * 6.0f, dir.z};
            const sw::f32 swirl =
                fbm3(flow * 1.7f, 4, 2020u + static_cast<sw::u32>(style)) - 0.5f;
            const sw::f32 eddy =
                fbm3(flow * 5.3f, 3, 4040u + static_cast<sw::u32>(style)) - 0.5f;

            const sw::f32 lat = dir.y + (swirl + eddy * 0.4f) * shear * 0.05f;

            // THE BAND FIELD, IN FIVE OCTAVES — major belts with minor ones
            // inside them, no two the same width. One sine wave draws a
            // beach ball; this is why it is a field. See the GLSL.
            const sw::f32 field =
                fbm3(sw::Vec3{0.37f, lat * bands, 0.71f}, 5,
                     3030u + static_cast<sw::u32>(style));
            const sw::f32 blend = smoothstepf(0.5f - edge, 0.5f + edge, field);
            const sw::f32 depth =
                0.60f + 0.85f * fbm3(sw::Vec3{1.7f, lat * bands * 0.21f, 4.3f}, 2,
                                     7070u + static_cast<sw::u32>(style));
            sw::Vec3 albedo =
                glm::mix(zone, belt, glm::clamp(blend * depth * contrast, 0.0f, 1.0f));

            const sw::f32 equator = 1.0f - smoothstepf(0.03f, 0.17f, std::abs(dir.y));
            albedo = glm::mix(albedo, zone * 1.04f, equator * 0.45f);

            const sw::f32 coarseFade = 1.0f - smoothstepf(0.002f, 0.008f, footprint);
            if (coarseFade > 0.0f)
            {
                const sw::f32 churn =
                    fbm3(flow * 26.0f, 3, 5050u + static_cast<sw::u32>(style)) - 0.5f;
                albedo *= 1.0f + churn * churnAmount * 1.8f * coarseFade;
            }
            const sw::f32 fineFade = 1.0f - smoothstepf(0.0006f, 0.0025f, footprint);
            if (fineFade > 0.0f)
            {
                const sw::f32 ripple =
                    fbm3(flow * 60.0f, 2, 6060u + static_cast<sw::u32>(style)) - 0.5f;
                albedo *= 1.0f + ripple * churnAmount * 1.1f * fineFade;
            }

            const sw::f32 hood = smoothstepf(0.62f, 0.94f, std::abs(dir.y));
            albedo = glm::mix(albedo,
                              glm::mix(albedo, zone, 0.45f) *
                                  sw::Vec3{0.74f, 0.79f, 0.90f},
                              hood);

            if (style == 20) // the Great Red Spot, fixed in the body frame
            {
                const sw::f32 lxz =
                    std::max(std::sqrt(dir.x * dir.x + dir.z * dir.z), 1.0e-5f);
                const sw::f32 cl = (dir.x * (-0.42f) + dir.z * 0.91f) / lxz;
                const sw::f32 dl = std::acos(glm::clamp(cl, -1.0f, 1.0f));
                const sw::f32 radial =
                    glm::length(sw::Vec2{dl * 0.55f, (dir.y + 0.34f) * 2.2f});
                const sw::f32 spot = 1.0f - smoothstepf(0.10f, 0.17f, radial);
                const sw::f32 collar = smoothstepf(0.10f, 0.15f, radial) *
                                       (1.0f - smoothstepf(0.17f, 0.26f, radial));
                albedo = glm::mix(albedo, sw::Vec3{0.72f, 0.33f, 0.22f}, spot * 0.85f);
                albedo = glm::mix(albedo, sw::Vec3{0.93f, 0.86f, 0.74f}, collar * 0.35f);
            }
            if (style == 23) // Neptune's bright methane streaks
            {
                const sw::f32 streak = smoothstepf(
                    0.60f, 0.72f,
                    fbm3(flow * sw::Vec3{3.0f, 1.4f, 3.0f}, 3, 2323u));
                albedo = glm::mix(albedo, sw::Vec3{0.88f, 0.93f, 0.99f}, streak * 0.55f);
            }
            return albedo;
        }

        /// CPU TWIN of detailFade() in Shaders/PlanetSurface.glsl. How much
        /// of a feature carried by noise at `frequency` a sample of angular
        /// size `footprint` can still resolve: full while a wavelength covers
        /// about sixteen samples, gone by four. The tectonic terrains are thin
        /// lanes inside wide cells and are the first thing in the palette to
        /// alias, so they get the same treatment as the crater size classes.
        [[nodiscard]] inline sw::f32 detailFade(sw::f32 frequency, sw::f32 footprint)
        {
            return 1.0f - sw::math::smoothstepf(0.060f, 0.250f, footprint * frequency);
        }

        /// CPU TWIN of paintCraters() in Shaders/PlanetSurface.glsl — same
        /// constants, same order. See the shader for the reasoning; the short
        /// version is that a crater floor multiplies the ground colour DOWN
        /// while a rim, an ejecta blanket and a ray all lift it and blend it
        /// toward whatever the impact dug through, which on an icy moon is
        /// clean white ice and on a rock world is grey regolith.
        [[nodiscard]] inline sw::Vec3 paintCraters(const sw::Vec3& ground,
                                                   sw::f32 field, sw::f32 rays,
                                                   sw::f32 contrast, sw::f32 floorGain,
                                                   const sw::Vec3& fresh,
                                                   sw::f32 freshAmount,
                                                   sw::f32 rayAmount)
        {
            const sw::f32 signedField = field * contrast;
            const sw::f32 shadowed =
                glm::clamp(signedField * floorGain, -0.72f, 0.0f);
            const sw::f32 excavated = glm::clamp(signedField, 0.0f, 1.40f);
            sw::Vec3 albedo = ground * (1.0f + shadowed + excavated * 0.80f);
            albedo = glm::mix(albedo, fresh,
                              glm::clamp(excavated * freshAmount, 0.0f, 0.85f));
            return glm::mix(albedo, fresh, glm::clamp(rays * rayAmount, 0.0f, 0.80f));
        }

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
            // THE FOOTPRINT THIS MESH SEES. craterTerrain3 fades every size
            // class against the angular size of one sample; the fragment path
            // passes its pixel footprint, and the reciprocal of this mesh's
            // Nyquist frequency is exactly the same quantity — the angle below
            // which this tessellation can no longer tell two things apart. So
            // the two paths run the same ladder with the same constants, and a
            // crater class simply is not drawn on a mesh too coarse to hold
            // it. That is deliberate: Gouraud smearing one random number per
            // vertex across a 400 km triangle is what made these bodies look
            // like they had weather in the first place.
            const sw::f32 footprint = 1.0f / std::max(frequencyLimit, 1.0e-6f);
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
                    const sw::Vec3 abyss{0.004f, 0.024f, 0.115f};
                    const sw::Vec3 open{0.012f, 0.095f, 0.290f};
                    const sw::Vec3 shallows{0.050f, 0.375f, 0.470f};
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

                const sw::Vec3 desert{0.680f, 0.535f, 0.290f};
                const sw::Vec3 steppe{0.505f, 0.440f, 0.205f};
                const sw::Vec3 grass{0.215f, 0.385f, 0.125f};
                const sw::Vec3 forest{0.075f, 0.235f, 0.080f};
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
                //
                // The mask runs 0 over the mare basalt and 1 over the
                // highlands — it used to be called `maria`, which was exactly
                // backwards, and that only stopped being a naming quibble
                // when the crater density started hanging off it.
                const sw::f32 m =
                    fbm3(dir * 3.1f + sw::Vec3{2.9f, 8.1f, 0.4f}, 4, 4242u);
                const sw::f32 highland =
                    smoothstepf(0.435f, 0.515f, m) * resolve(3.1f) + 0.5f *
                                                                     (1.0f -
                                                                      resolve(3.1f));
                const sw::f32 fine =
                    (fbm3(dir * 11.0f, 3, 4343u) - 0.5f) * resolve(11.0f);
                sw::f32 g = glm::mix(0.235f, 0.415f, highland) + 0.10f * fine +
                            detail * 0.08f + relief * 0.10f;
                g = glm::mix(g, g * 1.18f + 0.03f, rock);
                albedo = {g, g, g * 1.04f};

                // The crater record, discounted over the maria: basalt floods
                // three billion years younger than the ground around them
                // carry about a third of the craters per square kilometre.
                sw::f32 rays = 0.0f;
                const sw::f32 craters =
                    sw::math::craterTerrain3(dir, 7.0f, 4242u, footprint, 6, rays);
                const sw::f32 density = glm::mix(0.34f, 1.0f, highland);
                albedo = paintCraters(albedo, craters * density,
                                      rays * glm::mix(0.80f, 1.0f, highland), 1.15f,
                                      1.00f, sw::Vec3{0.62f, 0.62f, 0.64f}, 0.38f,
                                      0.62f);
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
            case SurfaceStyle::Mercury:
            {
                const sw::f32 m =
                    fbm3(dir * 3.4f + sw::Vec3{11.2f, 4.4f, 7.7f}, 4, 7171u);
                const sw::f32 plains = smoothstepf(0.44f, 0.52f, m) * resolve(3.4f) +
                                       0.5f * (1.0f - resolve(3.4f));
                const sw::f32 fine =
                    (fbm3(dir * 12.0f, 3, 7272u) - 0.5f) * resolve(12.0f);
                sw::f32 g = glm::mix(0.245f, 0.385f, plains) + 0.09f * fine +
                            detail * 0.08f + relief * 0.12f;
                g = glm::mix(g, g * 1.16f + 0.03f, rock);
                albedo = {g * 1.11f, g * 0.99f, g * 0.85f};

                // The most heavily battered ground in the inner system: no
                // mare flooding ever erased it, and no atmosphere, water or
                // volcanism ever softened an edge. Only the smooth plains get
                // the discount the lunar maria get.
                sw::f32 rays = 0.0f;
                const sw::f32 craters =
                    sw::math::craterTerrain3(dir, 9.5f, 7171u, footprint, 6, rays);
                const sw::f32 density = glm::mix(1.0f, 0.65f, plains);
                albedo = paintCraters(albedo, craters * density, rays, 0.95f, 0.95f,
                                      sw::Vec3{0.60f, 0.55f, 0.46f}, 0.26f, 0.70f);
                break;
            }
            case SurfaceStyle::Io:
            {
                // The one body here with NO craters, and that is not an
                // omission: tidal heating repaves Io with a centimetre of
                // sulfur a year and nothing older than about a million years
                // survives on it. Voyager looked for impact structures and
                // found zero. What it has instead is eruption centres.
                const sw::f32 p =
                    fbm3(dir * 3.0f + sw::Vec3{3.1f, 9.2f, 5.5f}, 4, 4411u);
                const sw::Vec3 sulfur{0.78f, 0.68f, 0.30f};
                const sw::Vec3 burnt{0.55f, 0.30f, 0.14f};
                const sw::Vec3 frost{0.90f, 0.88f, 0.78f};
                albedo = glm::mix(sulfur, burnt,
                                  smoothstepf(0.55f, 0.75f, p) * resolve(3.0f));
                albedo = glm::mix(albedo, frost,
                                  (1.0f - smoothstepf(0.30f, 0.42f, p)) * resolve(3.0f));
                const sw::f32 e = fbm3(dir * 7.0f, 3, 4422u);
                const sw::f32 vents = smoothstepf(0.66f, 0.74f, e) * resolve(7.0f);
                albedo = glm::mix(albedo, sw::Vec3{0.16f, 0.12f, 0.08f}, vents);

                // Plume deposits: the ring of fallout around a vent, white
                // where it is SO2 frost and red where it is short-chain
                // sulfur. Read off the same field as the vents, thresholded
                // to the annulus around one instead of its floor, so a
                // deposit can only exist where something is erupting.
                const sw::f32 apron = smoothstepf(0.56f, 0.64f, e) *
                                      (1.0f - smoothstepf(0.64f, 0.70f, e)) *
                                      resolve(7.0f);
                const sw::f32 sulfurRed =
                    fbm3(dir * 5.0f + sw::Vec3{6.1f, 1.7f, 8.3f}, 3, 4433u);
                const sw::Vec3 plume =
                    glm::mix(sw::Vec3{0.94f, 0.93f, 0.88f}, sw::Vec3{0.72f, 0.17f, 0.13f},
                             smoothstepf(0.42f, 0.58f, sulfurRed));
                albedo = glm::mix(albedo, plume, apron * 0.75f);

                // The poles are the cold trap: SO2 frost collects where the
                // ground never gets warm enough to sublime it away again.
                albedo = glm::mix(albedo, sw::Vec3{0.88f, 0.90f, 0.92f},
                                  smoothstepf(0.62f, 0.92f, latitude) * 0.45f);

                // Flow fields: overlapping lava flows tens of kilometres
                // long, the newest of them dark because the sulfur has not
                // weathered yet. Without it Io is a flat wash from exactly
                // the altitude a ship in a low orbit sits at.
                const sw::f32 flowFade = detailFade(46.0f, footprint);
                if (flowFade > 0.0f)
                {
                    const sw::f32 flow =
                        fbm3(dir * 46.0f + sw::Vec3{2.3f, 7.9f, 4.1f}, 3, 4455u);
                    albedo *= 1.0f + (flow - 0.5f) * 0.34f * flowFade;
                    albedo = glm::mix(albedo, sw::Vec3{0.40f, 0.22f, 0.11f},
                                      smoothstepf(0.60f, 0.74f, flow) * flowFade *
                                          0.55f);
                }
                albedo *= (1.0f + detail * 0.20f);
                break;
            }
            case SurfaceStyle::Europa:
            case SurfaceStyle::Enceladus:
            case SurfaceStyle::Triton:
            {
                const bool enceladus = style == SurfaceStyle::Enceladus;
                const bool triton = style == SurfaceStyle::Triton;
                const bool europa = style == SurfaceStyle::Europa;
                const sw::Vec3 base = enceladus ? sw::Vec3{0.90f, 0.94f, 0.99f}
                                     : triton   ? sw::Vec3{0.80f, 0.72f, 0.68f}
                                                : sw::Vec3{0.80f, 0.84f, 0.91f};
                const sw::f32 c =
                    std::abs(fbm3(dir * 4.5f + terrain.noiseOffset, 4,
                                  terrain.seed + 17u) -
                             0.5f) *
                    2.0f;
                const sw::f32 lineae =
                    (1.0f - smoothstepf(0.03f, 0.11f, c)) * resolve(4.5f);
                const sw::Vec3 crackTint = enceladus ? sw::Vec3{0.55f, 0.72f, 0.86f}
                                           : triton  ? sw::Vec3{0.42f, 0.36f, 0.34f}
                                                     : sw::Vec3{0.52f, 0.38f, 0.26f};
                const sw::f32 where =
                    enceladus ? smoothstepf(0.45f, 0.75f, -dir.y) : 1.0f;
                albedo = base * (1.0f + detail * 0.10f + relief * 0.10f);
                // Europa keeps this isotropic network at less than half
                // strength; its real lineae are the stretched bands below.
                const sw::f32 crackAmount = europa ? 0.34f : 0.80f;
                albedo = glm::mix(albedo, crackTint, lineae * where * crackAmount);

                if (europa)
                {
                    // THE LINEAE. Hundreds of kilometres long and tens wide,
                    // which no isotropic noise can draw: sampled seven times
                    // finer across than along, a cell becomes a ribbon. Dark
                    // rust core, pale margin — the profile of a triple band.
                    const sw::Vec3 alongX{dir.x * 21.0f, dir.y * 3.0f, dir.z * 3.2f};
                    const sw::Vec3 alongZ{dir.x * 3.2f, dir.y * 3.1f, dir.z * 19.0f};
                    const sw::f32 bandA =
                        std::abs(fbm3(alongX + terrain.noiseOffset, 3,
                                      terrain.seed + 61u) -
                                 0.5f) *
                        2.0f;
                    const sw::f32 bandB =
                        std::abs(fbm3(alongZ + terrain.noiseOffset, 3,
                                      terrain.seed + 83u) -
                                 0.5f) *
                        2.0f;
                    const sw::f32 band = detailFade(21.0f, footprint);
                    const sw::f32 core =
                        std::max(1.0f - smoothstepf(0.012f, 0.055f, bandA),
                                 1.0f - smoothstepf(0.012f, 0.055f, bandB)) *
                        band;
                    const sw::f32 margin =
                        std::max(smoothstepf(0.035f, 0.075f, bandA) *
                                     (1.0f - smoothstepf(0.075f, 0.150f, bandA)),
                                 smoothstepf(0.035f, 0.075f, bandB) *
                                     (1.0f - smoothstepf(0.075f, 0.150f, bandB))) *
                        band;
                    albedo = glm::mix(albedo, sw::Vec3{0.86f, 0.88f, 0.92f},
                                      margin * 0.55f);
                    albedo = glm::mix(albedo, sw::Vec3{0.46f, 0.30f, 0.19f},
                                      core * 0.75f);

                    // Ridged plains, five times finer, for the approach: a
                    // weave of double ridges a few kilometres apart, which is
                    // what is actually under a hundred kilometres of altitude
                    // where the great bands are one smear across the frame.
                    const sw::f32 weaveFade = detailFade(96.0f, footprint);
                    if (weaveFade > 0.0f)
                    {
                        const sw::f32 fineA =
                            std::abs(fbm3(sw::Vec3{dir.x * 96.0f, dir.y * 14.0f,
                                                   dir.z * 15.0f},
                                          3, terrain.seed + 109u) -
                                     0.5f) *
                            2.0f;
                        const sw::f32 fineB =
                            std::abs(fbm3(sw::Vec3{dir.x * 15.0f, dir.y * 13.0f,
                                                   dir.z * 88.0f},
                                          3, terrain.seed + 127u) -
                                     0.5f) *
                            2.0f;
                        const sw::f32 crest =
                            std::max(smoothstepf(0.03f, 0.08f, fineA) *
                                         (1.0f - smoothstepf(0.08f, 0.16f, fineA)),
                                     smoothstepf(0.03f, 0.08f, fineB) *
                                         (1.0f - smoothstepf(0.08f, 0.16f, fineB)));
                        const sw::f32 groove =
                            std::max(1.0f - smoothstepf(0.008f, 0.030f, fineA),
                                     1.0f - smoothstepf(0.008f, 0.030f, fineB));
                        albedo = glm::mix(albedo, sw::Vec3{0.93f, 0.95f, 0.98f},
                                          crest * weaveFade * 0.40f);
                        albedo = glm::mix(albedo, sw::Vec3{0.58f, 0.46f, 0.36f},
                                          groove * weaveFade * 0.45f);
                    }
                }
                if (enceladus)
                {
                    // THE TIGER STRIPES: four roughly parallel sulci inside
                    // the south polar terrain, each flanked by the coarse
                    // blue-green ice the plumes drop back on.
                    const sw::f32 south = smoothstepf(0.35f, 0.78f, -dir.y);
                    const sw::Vec3 alongStripe{dir.x * 2.6f, dir.y * 2.4f,
                                               dir.z * 34.0f};
                    const sw::f32 stripe =
                        std::abs(fbm3(alongStripe + terrain.noiseOffset, 2,
                                      terrain.seed + 97u) -
                                 0.5f) *
                        2.0f;
                    const sw::f32 lane = detailFade(34.0f, footprint);
                    const sw::f32 flank = smoothstepf(0.04f, 0.14f, stripe) *
                                          (1.0f - smoothstepf(0.14f, 0.30f, stripe)) *
                                          lane;
                    const sw::f32 fracture =
                        (1.0f - smoothstepf(0.015f, 0.055f, stripe)) * lane;
                    albedo = glm::mix(albedo, sw::Vec3{0.62f, 0.80f, 0.90f},
                                      flank * south * 0.60f);
                    albedo = glm::mix(albedo, sw::Vec3{0.34f, 0.52f, 0.64f},
                                      fracture * south * 0.85f);
                }
                if (triton)
                {
                    // Cantaloupe terrain: 30 km dimples with no rims and no
                    // ejecta, which is the crater field with only its floor
                    // term left — melt, not bombardment.
                    sw::f32 dimpleRays = 0.0f;
                    const sw::f32 dimples = sw::math::craterTerrain3(
                        dir, 26.0f, terrain.seed + 151u, footprint, 2, dimpleRays);
                    albedo *= 1.0f + glm::clamp(dimples, -0.7f, 0.0f) * 0.42f;
                    // The southern nitrogen-methane frost cap: the brightest
                    // large surface in the system, pink from tholins.
                    albedo = glm::mix(albedo, sw::Vec3{0.94f, 0.86f, 0.80f},
                                      smoothstepf(-0.10f, 0.55f, -dir.y) * 0.50f);
                }
                albedo = glm::mix(albedo, albedo * 1.12f, rock);

                // What little bombardment a young ice world has kept —
                // Enceladus everywhere except its resurfaced south polar
                // terrain, which is measurably crater-free.
                sw::f32 rays = 0.0f;
                const sw::f32 youngIce = sw::math::craterTerrain3(
                    dir, enceladus ? 5.0f : 9.0f, terrain.seed + 733u, footprint, 3,
                    rays);
                const sw::f32 density =
                    enceladus ? (1.0f - smoothstepf(0.20f, 0.70f, -dir.y))
                    : europa  ? 0.30f
                              : 0.45f;
                albedo = paintCraters(albedo, youngIce * density, rays * density,
                                      europa ? 0.30f : 0.55f, 0.70f,
                                      sw::Vec3{0.96f, 0.98f, 1.00f}, 0.40f, 0.50f);
                material = {0.35f, 0.55f};
                break;
            }
            case SurfaceStyle::Ganymede:
            case SurfaceStyle::Callisto:
            case SurfaceStyle::Rhea:
            case SurfaceStyle::Titania:
            case SurfaceStyle::Oberon:
            {
                const sw::f32 m = fbm3(dir * 3.4f + terrain.noiseOffset, 4,
                                       terrain.seed + 29u);
                // Not const: Ganymede's sulci below fold themselves into
                // this mask, because a groove belt IS bright terrain as far
                // as everything downstream is concerned.
                sw::f32 brightTerrain = smoothstepf(0.42f, 0.58f, m) * resolve(3.4f) +
                                        0.5f * (1.0f - resolve(3.4f));
                sw::Vec3 dark{0.27f, 0.25f, 0.23f};
                sw::Vec3 light{0.55f, 0.54f, 0.52f};
                if (style == SurfaceStyle::Callisto) { dark *= 0.62f; light *= 0.72f; }
                if (style == SurfaceStyle::Rhea) { dark *= 1.12f; light *= 1.10f; }
                if (style == SurfaceStyle::Oberon)
                {
                    dark *= sw::Vec3{1.05f, 0.95f, 0.87f};
                    light *= sw::Vec3{1.05f, 0.96f, 0.88f};
                }
                albedo = glm::mix(dark, light, brightTerrain);
                if (style == SurfaceStyle::Ganymede)
                {
                    // GROOVED TERRAIN. Sulci are belts of PARALLEL furrows,
                    // and parallel is exactly what an isotropic fBm cannot
                    // draw — the old `abs(fbm - 0.5)` network drew crazing.
                    // Sampled fourteen times finer across the furrows than
                    // along them a cell becomes a lane; three sets, one of
                    // them oblique so the belts do not point at the body
                    // axes, each behind its own patch mask.
                    const sw::f32 setA =
                        std::abs(fbm3(sw::Vec3{dir.x * 62.0f, dir.y * 4.4f,
                                               dir.z * 4.4f},
                                      3, 6611u) -
                                 0.5f) *
                        2.0f;
                    const sw::f32 setB =
                        std::abs(fbm3(sw::Vec3{dir.x * 4.6f, dir.y * 58.0f,
                                               dir.z * 5.0f},
                                      3, 6612u) -
                                 0.5f) *
                        2.0f;
                    const sw::f32 su = (dir.x + dir.z) * 0.70710678f;
                    const sw::f32 sv = (dir.z - dir.x) * 0.70710678f;
                    const sw::f32 setC =
                        std::abs(fbm3(sw::Vec3{su * 55.0f, dir.y * 4.8f, sv * 4.6f}, 3,
                                      6615u) -
                                 0.5f) *
                        2.0f;
                    const sw::f32 patchA =
                        smoothstepf(0.38f, 0.55f, fbm3(dir * 2.4f, 3, 6613u));
                    const sw::f32 patchB =
                        smoothstepf(0.40f, 0.57f, fbm3(dir * 2.7f, 3, 6614u));
                    const sw::f32 patchC =
                        smoothstepf(0.39f, 0.56f, fbm3(dir * 2.9f, 3, 6616u));
                    const sw::f32 lane = detailFade(62.0f, footprint);
                    const sw::f32 grooves =
                        std::max(std::max((1.0f - smoothstepf(0.05f, 0.17f, setA)) *
                                              patchA,
                                          (1.0f - smoothstepf(0.05f, 0.17f, setB)) *
                                              patchB),
                                 (1.0f - smoothstepf(0.05f, 0.17f, setC)) * patchC) *
                        brightTerrain * lane;
                    albedo = glm::mix(albedo, light * sw::Vec3{1.42f, 1.45f, 1.50f},
                                      grooves * 0.90f);
                    const sw::f32 ribs =
                        std::max(smoothstepf(0.06f, 0.15f, setA) *
                                     (1.0f - smoothstepf(0.15f, 0.26f, setA)) * patchA,
                                 smoothstepf(0.06f, 0.15f, setC) *
                                     (1.0f - smoothstepf(0.15f, 0.26f, setC)) *
                                     patchC) *
                        lane;
                    albedo = glm::mix(albedo, light * 0.66f,
                                      ribs * brightTerrain * 0.50f);
                    brightTerrain = std::max(brightTerrain, grooves);
                }
                const sw::f32 fine =
                    (fbm3(dir * 11.0f, 3, terrain.seed + 43u) - 0.5f) * resolve(11.0f);
                albedo *= (1.0f + fine * 0.16f + detail * 0.10f + relief * 0.10f);
                albedo = glm::mix(albedo, albedo * 1.18f, rock);

                // THE OLDEST SURFACES IN THE SYSTEM. On an icy body an impact
                // is the only thing that ever digs through the dark lag of
                // dust and processed organics four billion years leaves on
                // top, so its crater is not a grey ring on grey ground, it is
                // a WHITE hole punched in soot — and the dark worlds keep
                // their floors shallow, because a 2% albedo has nothing left
                // to give.
                sw::f32 rays = 0.0f;
                sw::f32 density = 1.0f;
                sw::f32 contrast = 0.85f;
                sw::f32 floorGain = 0.62f;
                sw::f32 freshAmount = 0.50f;
                sw::Vec3 freshIce{0.88f, 0.90f, 0.92f};
                if (style == SurfaceStyle::Callisto)
                {
                    contrast = 1.30f;
                    floorGain = 0.20f;
                    freshAmount = 0.62f;
                    freshIce = {0.86f, 0.87f, 0.88f};
                }
                if (style == SurfaceStyle::Ganymede)
                {
                    density = glm::mix(1.0f, 0.40f, brightTerrain);
                    contrast = 0.85f;
                    floorGain = 0.48f;
                }
                if (style == SurfaceStyle::Rhea) { freshIce = {0.94f, 0.96f, 0.98f}; }
                if (style == SurfaceStyle::Oberon) { freshIce = {0.90f, 0.87f, 0.82f}; }
                const sw::f32 craters = sw::math::craterTerrain3(
                    dir, 6.5f, terrain.seed + 29u, footprint, 6, rays);
                albedo = paintCraters(albedo, craters * density, rays * density,
                                      contrast, floorGain, freshIce, freshAmount,
                                      0.60f);
                material = {0.18f, 0.35f};
                break;
            }
            case SurfaceStyle::Titan:
            {
                const sw::Vec3 dunes{0.34f, 0.24f, 0.13f};
                const sw::Vec3 beltDark{0.19f, 0.13f, 0.07f};
                const sw::Vec3 upland{0.58f, 0.47f, 0.29f};
                const sw::f32 equator = 1.0f - smoothstepf(0.25f, 0.55f, latitude);
                albedo = glm::mix(dunes, beltDark, equator);
                albedo = glm::mix(albedo, upland, smoothstepf(0.30f, 0.75f, relief));
                albedo *= (1.0f + detail * 0.14f);

                // Titan has craters and almost none of them: a 1.5 bar
                // atmosphere burns up the small impactors and methane rain
                // fills in what gets through. Two classes, a quarter of
                // Callisto's contrast.
                sw::f32 rays = 0.0f;
                const sw::f32 craters =
                    sw::math::craterTerrain3(dir, 5.0f, 8811u, footprint, 2, rays);
                albedo = paintCraters(albedo, craters, rays * 0.15f, 0.26f, 0.80f,
                                      sw::Vec3{0.56f, 0.46f, 0.30f}, 0.20f, 0.15f);
                // ...and then several hundred kilometres of photochemical
                // smog puts all of it behind a curtain. This is the one soft
                // body in a system of hard edges.
                albedo = glm::mix(albedo, sw::Vec3{0.46f, 0.33f, 0.17f}, 0.38f);
                material = {0.10f, 0.25f};
                break;
            }
            case SurfaceStyle::Jupiter:
            case SurfaceStyle::Saturn:
            case SurfaceStyle::Uranus:
            case SurfaceStyle::Neptune:
            case SurfaceStyle::Venus:
            {
                // 0.03 rad: the angle between two LOD0 vertices. Detail
                // finer than the quad it would be interpolated across is
                // not detail, it is aliasing — so the churn switches off
                // and the vertices carry the bands alone.
                albedo = gasGiantAlbedo(static_cast<sw::i32>(style), dir, 0.03f);
                material = {0.06f, 0.30f};
                break;
            }
            }

            vertex.color = {albedo.r, albedo.g, albedo.b, 1.0f};
            vertex.uv = material;
        }

        // ---- static starfield: parallax-free orientation reference ------------
        //
        // F20: a SKY, measured, rather than a scatter of dots.
        //
        // What was there before was 3400 grains: a cubed uniform for
        // brightness, a "warm" slider for colour, and 42% of them pushed
        // toward a great circle that lay 20 degrees off the ecliptic — i.e.
        // through every planet in the game. Its dimmest star reached sRGB 100
        // and its brightest 230, so the whole sky sat inside a factor of five
        // of itself. The one it stands for spans a factor of sixteen hundred.
        //
        // Three measurements fix it, and none of them is a taste call.
        //
        // 1. THE MAGNITUDE LAW. Star counts grow by about a factor of three
        //    per magnitude: 15 stars brighter than 1st, 48 brighter than 2nd,
        //    171 brighter than 3rd, 1600 brighter than 5th, and some 9100
        //    down to the naked-eye limit at 6.5. That is N proportional to
        //    10^(0.5 m), and a law that simple inverts: draw u uniform on
        //    (0, 1] and m = 6.5 + 2 log10(u) IS the distribution. Nine
        //    thousand stars on it come out with eight brighter than magnitude
        //    0.5 and one near -1.4 without being told to. The real sky has
        //    eight, and Sirius.
        //
        // 2. COLOUR, AND WHERE THE EYE CAN SEE IT. Spectral class sets the
        //    temperature and the temperature sets the colour, blue-white
        //    through white to orange and red — but a star's colour is only
        //    VISIBLE if there is enough light in it to work the cones, and
        //    fainter than about fourth magnitude there is not: the sky goes
        //    monochrome. So the class colour is faded toward white as the
        //    magnitude climbs, which is why a real sky reads as a grey-white
        //    spray with a dozen coloured lamps in it rather than as confetti.
        //
        // 3. THE MILKY WAY, which is the part a uniform scatter cannot fake:
        //    a band of light too fine to resolve, brightest and widest toward
        //    the galactic centre, split lengthwise by the dust of the Great
        //    Rift. It arrives here in two halves — a rise in star DENSITY
        //    along the band, and under it a grain of dim motes for the light
        //    that never resolves into stars at all.
        //
        // Geometry note: the dome is drawn CAMERA-CENTRED and never rotated
        // (see collectDrawItems), so a quad built perpendicular to its own
        // direction is a billboard that faces the camera exactly, for every
        // star, for free, for ever. That is what pays for the counts below:
        // four vertices and two triangles each instead of the six and eight an
        // octahedron cost, so nine thousand stars are 18 000 triangles where
        // the old 3 400 were 27 200. The band's grain is what costs — 48 000
        // more — and it buys the one thing a scatter of points cannot fake.
        // 66 000 static triangles in one draw call, built once at startup.
        inline constexpr sw::f32 kStarDomeRadius = 1.0e12f; // inside the 1e13 far plane
        inline constexpr sw::u32 kStarCount = 9000;         // ...to magnitude 6.5
        inline constexpr sw::u32 kStarHazeCount = 24000;    // the unresolved band

        /// The north galactic pole and the direction of the galactic centre,
        /// in the game's world frame — Y is the ecliptic pole (every planet in
        /// GameScene.cpp orbits about it and every SpinComponent points along
        /// it) and X, Z span the ecliptic. In ecliptic coordinates the pole
        /// sits at longitude 180.0, latitude +29.8 and the centre at 266.8,
        /// -5.5. The one fact those two numbers carry is the one that matters:
        /// the Milky Way crosses the ecliptic at SIXTY degrees. Lay the band
        /// near the ecliptic instead and it runs down the same line as the
        /// planets, which is what the old dome did.
        inline const sw::Vec3 kGalacticPole{-0.8678f, 0.4971f, -0.0003f};
        inline const sw::Vec3 kGalacticCentre{-0.0549f, -0.0965f, -0.9938f};

        /// How much unresolved light the galactic disc puts in a direction,
        /// normalized to about 1 on the brightest part of the band and 0 well
        /// off it. Used twice: to decide where the faint stars go, and how
        /// bright the haze under them is.
        ///
        /// Four terms, all of them things you can point at in the sky:
        ///   - the DISC, exponential in galactic latitude, with a scale height
        ///     that is widest toward the centre (the band is some 15 degrees
        ///     across in Sagittarius and a narrow thread in Auriga);
        ///   - the BULGE around the centre itself, which is the brightest
        ///     patch of sky there is and the reason a summer Milky Way looks
        ///     nothing like a winter one;
        ///   - the GREAT RIFT: not a gap in the stars but dust in front of
        ///     them, running from Cygnus down through Aquila into Sagittarius
        ///     a couple of degrees south of the plane, and taking most of the
        ///     light out of the band's middle for a third of its length;
        ///   - and mottling, because the last thing a real band is is smooth.
        [[nodiscard]] inline sw::f32 milkyWayBrightness(const sw::Vec3& dir)
        {
            using sw::math::smoothstepf;
            const sw::Vec3 east = glm::cross(kGalacticPole, kGalacticCentre);
            const sw::f32 latitude = std::asin(
                glm::clamp(glm::dot(dir, kGalacticPole), -1.0f, 1.0f));
            const sw::f32 longitude = std::atan2(glm::dot(dir, east),
                                                 glm::dot(dir, kGalacticCentre));
            const sw::f32 scaleHeight = 0.100f + 0.040f * std::cos(longitude);
            const sw::f32 along =
                0.30f + 0.70f * (0.5f + 0.5f * std::cos(longitude));
            sw::f32 brightness = along * std::exp(-std::abs(latitude) / scaleHeight);

            const sw::f32 fromCentre = std::acos(
                glm::clamp(glm::dot(dir, kGalacticCentre), -1.0f, 1.0f));
            brightness += 0.85f * std::exp(-fromCentre * fromCentre / (2.0f * 0.09f));

            const sw::f32 riftLatitude = latitude + 0.030f;
            const sw::f32 rift =
                std::exp(-riftLatitude * riftLatitude / (2.0f * 0.045f * 0.045f)) *
                smoothstepf(-0.40f, 0.05f, longitude) *
                (1.0f - smoothstepf(1.15f, 1.65f, longitude));
            brightness *= 1.0f - 0.75f * rift;

            brightness *= 0.52f + 0.95f * fbm3(dir * 5.5f, 4, 90210u);
            return glm::clamp(brightness, 0.0f, 1.0f);
        }

        /// The linear colour of one star. `pick` chooses its spectral class
        /// and `spread` its place inside that class; both are uniform on
        /// [0, 1).
        ///
        /// The ladder is a blackbody at each class's effective temperature —
        /// Blackbody colour from an effective temperature. It lives in the
        /// ENGINE now (sw::space::blackbodyColor) because it is a statement
        /// about physics rather than about this game, and because a function
        /// the whole star catalogue is judged by has to be reachable from a
        /// test.
        using sw::space::blackbodyColor;

        [[nodiscard]] inline sw::Vec3 starClassColor(sw::f32 pick, sw::f32 spread)
        {
            // THE SAME LADDER, READ BY CLASS. These six are blackbodyColor
            // evaluated at each class's representative temperature, so the
            // nine thousand anonymous stars and the thirty-six catalogued ones
            // are coloured by one rule. They carried the same sRGB-encoded
            // values the catalogue ladder did, and a dome corrected while the
            // catalogue was not — or the reverse — would put Sirius in a sky
            // whose neighbours obey different physics, which is the one thing
            // the eye finds instantly.
            const sw::Vec3 ladder[6] = {
                {0.334f, 0.501f, 1.000f}, // O/B  ~20000 K
                {0.554f, 0.692f, 1.000f}, // A     ~9000 K
                {0.745f, 0.832f, 1.000f}, // F     ~7000 K
                {1.000f, 0.992f, 0.979f}, // G     ~5700 K
                {1.000f, 0.832f, 0.613f}, // K     ~4500 K
                {1.000f, 0.615f, 0.263f}, // M     ~3300 K
            };
            // Cumulative: O+B 20%, A 24%, F 12%, G 12%, K 24%, M 8%.
            const sw::f32 edges[7] = {0.0f,  0.20f, 0.44f, 0.56f,
                                      0.68f, 0.92f, 1.0f};
            sw::i32 spectralClass = 5;
            for (sw::i32 i = 0; i < 6; ++i)
            {
                if (pick < edges[i + 1])
                {
                    spectralClass = i;
                    break;
                }
            }
            // A class is a RANGE of temperatures, not one — B0 and B9 are
            // 20 000 K apart — so each star is blurred half a class either
            // way and the six rungs read as the continuum they stand for.
            const sw::f32 position = glm::clamp(
                static_cast<sw::f32>(spectralClass) + spread - 0.5f, 0.0f, 5.0f);
            const sw::i32 step = glm::min(static_cast<sw::i32>(position), 4);
            return glm::mix(ladder[step], ladder[step + 1],
                            position - static_cast<sw::f32>(step));
        }

        [[nodiscard]] inline sw::MeshData buildStarfieldMesh()
        {
            sw::MeshData mesh;
            mesh.vertices.reserve(kStarHazeCount * 5 + kStarCount * 20);
            mesh.indices.reserve(kStarHazeCount * 12 + kStarCount * 60);
            sw::u32 seed = 0xC0FFEEu; // FIXED seed: the sky never changes

            // ---- F21: WHY THIS MESH IS BUILT THE WAY IT IS -------------------
            //
            // The dome used to be drawn OPAQUE, with the star's brightness in
            // its RGB and its alpha pinned at 1. Two consequences, and the
            // second one is why the Milky Way had to be rebuilt from scratch:
            //
            //   - opaque means the grade runs on every fragment, and the grade
            //     lifts blacks to sRGB 37 against a sky of 13. THE DIMMEST
            //     THING AN OPAQUE DOME CAN DRAW IS A LIGHT GREY. There is no
            //     faint in it — a mote worth a hundredth of the band's core
            //     and the core itself come out within a few levels of each
            //     other;
            //   - and every quad is therefore a STEP. Twenty-four thousand
            //     grey squares, each one visible, is not a galaxy: it is
            //     static. That is exactly what it looked like.
            //
            // So the dome is TRANSLUCENT now (GameMap.cpp sets it), the
            // brightness rides the ALPHA, and the RGB carries hue alone. A
            // mote at alpha 0.01 adds one grey level, which is what a mote is
            // worth; a first-magnitude star at alpha 1 is the full colour.
            // The day fade falls out of the same arithmetic for free — the
            // instance tint multiplies the whole alpha, and opacity is
            // (alpha - 1), so a brightening sky erases the faint stars first
            // and Sirius last, which is what dawn does.
            //
            // AND THE BLOBS ARE SOFT. A quad has four corners and one colour:
            // it cannot fade out at its own edge, so at any alpha you can
            // actually see it is a rectangle. Five vertices — the centre at
            // full opacity, four rim points at zero — and four triangles make
            // a bilinear pyramid with NO edge at all. It doubles the triangle
            // count and it is the whole difference between a band of light and
            // a field of confetti. It also gives every star a soft core, which
            // is what an eye and a lens both actually do to a point source.
            // `sides` is the fan's rim count. FOUR for the haze, which is a
            // diamond and does not matter because forty of them overlap; EIGHT
            // for a star, which does — a bilinear pyramid on four points has
            // diamond-shaped iso-lines, and a sky of grey diamonds is its own
            // kind of wrong. Eight is where the polygon stops reading as a
            // polygon at the two-to-ten pixels a star is ever drawn at.
            const auto pushSoft = [&mesh](const sw::Vec3& dir, sw::f32 size,
                                          sw::f32 roll, sw::f32 aspect,
                                          sw::f32 shell, const sw::Vec3& hue,
                                          sw::f32 intensity, sw::u32 sides) {
                const sw::Vec3 reference =
                    (std::abs(dir.y) < 0.95f) ? sw::Vec3{0, 1, 0} : sw::Vec3{1, 0, 0};
                const sw::Vec3 tangent = glm::normalize(glm::cross(reference, dir));
                const sw::Vec3 bitangent = glm::cross(dir, tangent);
                const sw::Vec3 axis = tangent * std::cos(roll) + bitangent * std::sin(roll);
                const sw::Vec3 u = axis * (size * aspect);
                const sw::Vec3 v = glm::cross(dir, axis) * (size / aspect);
                const sw::u32 base = static_cast<sw::u32>(mesh.vertices.size());
                // Emissive convention (Mesh.frag): opacity = alpha - 1, so 1.0
                // is transparent and 2.0 is solid.
                const sw::Vec4 core{hue.r, hue.g, hue.b,
                                    1.0f + glm::clamp(intensity, 0.0f, 1.0f)};
                const sw::Vec4 rim{hue.r, hue.g, hue.b, 1.0f};
                const sw::Vec3 centre = dir * shell;
                mesh.vertices.push_back({centre, -dir, core, {}});
                for (sw::u32 i = 0; i < sides; ++i)
                {
                    const sw::f32 angle =
                        6.2831853f * static_cast<sw::f32>(i) / static_cast<sw::f32>(sides);
                    mesh.vertices.push_back(
                        {centre + u * std::cos(angle) + v * std::sin(angle), -dir, rim, {}});
                }
                for (sw::u32 i = 0; i < sides; ++i)
                {
                    mesh.indices.push_back(base);
                    mesh.indices.push_back(base + 1 + i);
                    mesh.indices.push_back(base + 1 + ((i + 1) % sides));
                }
            };
            const auto randomDirection = [&seed]() {
                // Uniform on the sphere: z uniform, longitude uniform. TWO
                // STATEMENTS, DELIBERATELY — `f(seed++), f(seed++)` in one
                // expression leaves the order of the two increments to the
                // compiler, so MSVC and GCC would generate different skies
                // from the same save.
                const sw::f32 z = 2.0f * hash01(seed++) - 1.0f;
                const sw::f32 phi = 6.2831853f * hash01(seed++);
                const sw::f32 r = std::sqrt(std::max(0.0f, 1.0f - z * z));
                return sw::Vec3{r * std::cos(phi), z, r * std::sin(phi)};
            };
            // A direction drawn from the galactic light profile, by rejection
            // against a uniform sphere. Two gates: galactic latitude first,
            // which throws away three quarters of the sky for the price of a
            // dot product (30 degrees off the plane the profile is under 0.04,
            // which is below anything this dome can draw), and then the
            // profile itself.
            //
            // THE CAP IS NOT A FORMALITY. At 40 tries one draw in thirty came
            // back unaccepted and got used anyway, and 24 000 motes with a 6%
            // failure rate is fourteen hundred grey specks strewn over a sky
            // that is supposed to be empty — which is exactly what the first
            // build of this looked like at the galactic pole. The loops that
            // consume it drop whatever still comes back off the band.
            const auto galacticDirection = [&seed, &randomDirection]() {
                sw::Vec3 dir = randomDirection();
                for (sw::i32 attempt = 0; attempt < 200; ++attempt)
                {
                    if (std::abs(glm::dot(dir, kGalacticPole)) < 0.50f &&
                        hash01(seed++) < milkyWayBrightness(dir))
                    {
                        break;
                    }
                    dir = randomDirection();
                }
                return dir;
            };

            // ---- the unresolved band, first so the stars draw over it ------
            //
            // The Milky Way is LIGHT, and now that the dome can draw faint
            // things it is drawn as light: overlapping soft pyramids, each one
            // worth a grey level or two, piling up to about six deep in the
            // core and thinning to nothing at the edges. No mote is visible
            // on its own — which is the test, and the previous version failed
            // it so plainly that the player reported it as a bug.
            //
            // The SIZES are twenty times the old ones (0.4 to 1.4 degrees
            // against a fourteenth of a degree) and that is the trade the
            // softness pays for: a hard quad has to be small enough to read as
            // a speck, a soft one can be large enough to overlap its
            // neighbours and blend with them. Fewer, bigger, softer, fainter.
            //
            // The mottling that makes it look like a galaxy rather than a
            // painted stripe is in milkyWayBrightness (fbm, the rift, the
            // bulge) and in the scatter itself — where the profile is dim,
            // the rejection sampler simply puts fewer blobs.
            for (sw::u32 speck = 0; speck < kStarHazeCount; ++speck)
            {
                const sw::Vec3 dir = galacticDirection();
                const sw::f32 brightness = milkyWayBrightness(dir);
                if (brightness < 0.04f)
                {
                    continue; // a draw that never landed on the band: no mote
                }
                const sw::f32 size = 0.0070f + 0.0180f * brightness;
                const sw::f32 roll = 6.2831853f * hash01(seed++);
                // A random roll and a random ASPECT. Softness kills the edges
                // but not the SHAPE: a field of identically-proportioned blobs
                // still reads as a texture, and stretching them at random is
                // what turns the core into cloud.
                const sw::f32 aspect = 0.50f + 1.30f * hash01(seed++);
                // The integrated light of a galactic disc is the light of its
                // K giants: a warm white, not a blue one. The hue is
                // normalized — brightness lives in the alpha now.
                const sw::f32 intensity = 0.0009f + 0.0060f * brightness * brightness;
                pushSoft(dir, size, roll, aspect, 1.0f,
                         sw::Vec3{1.00f, 0.94f, 0.85f}, intensity, 4);
            }

            // ---- the stars ------------------------------------------------
            for (sw::u32 star = 0; star < kStarCount; ++star)
            {
                // Magnitude straight off the count law, then the flux it
                // stands for: five magnitudes are a factor of a hundred, so
                // the sky this draws spans 1600 to 1 in light.
                const sw::f32 uniform = std::max(hash01(seed++), 1.0e-5f);
                const sw::f32 magnitude = 6.5f + 2.0f * std::log10(uniform);
                const sw::f32 flux = std::pow(10.0f, -0.4f * (magnitude - 6.5f));

                // WHERE. Bright stars are near ones — a few hundred parsecs
                // at most — and near ones are scattered all round us; the
                // faint end of a naked-eye sky is looking down the length of
                // the disc. So the chance of a star belonging to the band
                // climbs with magnitude, which is what makes the Milky Way
                // appear out of the fine grain instead of being painted on.
                const sw::f32 bandChance = 0.10f + 0.35f * glm::clamp(
                                                        (magnitude - 2.0f) / 4.0f,
                                                        0.0f, 1.0f);
                const sw::Vec3 dir = (hash01(seed++) < bandChance)
                                         ? galacticDirection()
                                         : randomDirection();

                // BRIGHTNESS AND SIZE are two different compressions of the
                // same flux, and they have to be: the sky spans sixteen hundred
                // to one in light and an 8-bit frame has about two hundred
                // useful levels of it. Brightness now means OPACITY, and the
                // range it has is the full 0..1 rather than the twenty levels
                // the opaque dome left above its own black floor — so the
                // exponent could come down from 0.67 to 0.55, which spreads
                // the faint end out instead of bunching it.
                //
                // 0.030 puts a magnitude 6.5 star at the very edge of visible
                // (one grey level over the sky, which is what sixth magnitude
                // IS) and the brightest of nine thousand draws at full colour.
                const sw::f32 intensity = 0.105f * std::pow(flux, 0.55f);
                // SIZE, and the floor is not a look, it is the rasteriser: a
                // blob narrower than a pixel is hit or missed depending on
                // where the pixel centre lands, so the faintest three quarters
                // of the sky would twinkle as the camera turned. The soft
                // pyramid needs more of it than the old hard quad did — its
                // peak is one vertex, so a two-pixel blob puts its full value
                // nowhere. Three pixels at 1080p and a 60 degree field, and
                // eleven for the brightest, where the falloff itself is what
                // an eye does to a bright point.
                const sw::f32 size = 0.00220f * std::pow(flux, 0.20f);

                // COLOUR: the spectral class, and then its temperature inside
                // that class. Two draws, two statements — see randomDirection.
                const sw::f32 classPick = hash01(seed++);
                const sw::f32 classSpread = hash01(seed++);
                const sw::Vec3 hue = starClassColor(classPick, classSpread);
                // ...and then most of that colour is thrown away again,
                // because the eye's cones need light the sky does not give
                // them. Antares and Rigel and Betelgeuse have a colour; a
                // fourth-magnitude star of the same class is grey, and the
                // sky in between is a ramp. This one line is most of the
                // difference between a night sky and a bag of confetti.
                const sw::f32 chroma =
                    glm::clamp((4.0f - magnitude) / 3.0f, 0.10f, 1.0f);
                const sw::Vec3 tinted =
                    glm::mix(sw::Vec3{1.0f, 1.0f, 1.0f}, hue, chroma);
                pushSoft(dir, size, 6.2831853f * hash01(seed++), 1.0f, 0.994f,
                         tinted, intensity, 8);
                // THE HALO, on the brightest one star in fifty. A first
                // magnitude star does not end at its own disc: the eye's own
                // optics and every lens ever made spread it, and that spread
                // is most of what makes a bright star READ as bright rather
                // than as a big dot. Four times the width at a fifteenth of
                // the opacity, and it costs nothing because almost no star
                // qualifies — a thousandth of the sky's flux draw does.
                if (flux > 22.0f)
                {
                    pushSoft(dir, size * 4.2f, 6.2831853f * hash01(seed++), 1.0f,
                             0.993f, tinted,
                             glm::min(0.16f, 0.065f * std::pow(flux, 0.28f)), 8);
                }
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

        // ---- F28: THE SUN'S GLARE ----------------------------------------------
        /// A billboard whose RADIANCE follows a power law from the centre out,
        /// in `kRings` radial steps rather than one linear ramp between a
        /// centre vertex and a rim.
        ///
        /// Two things were wrong with the old two-vertex disc, and both are the
        /// same mistake: the sun was authored as a COLOUR when it is a
        /// BRIGHTNESS.
        ///
        ///   - the grade maps a radiance of 1.0 to 0.43 and only reaches white
        ///     at 4.0, so a disc authored at (1, 0.99, 0.94) could not come out
        ///     brighter than sRGB 176 no matter what. The sun measured 190
        ///     against a background star at 140: thirty-six per cent brighter
        ///     than a speck. Feed it a radiance of fifteen and the tone curve
        ///     clips it to white, which is what a star does to a camera;
        ///   - and a LINEAR ramp is not what glare looks like. Light spread by
        ///     an optic (or by an eye) falls off as an inverse power, steeply
        ///     at first and then with a long tail. One exponent and twenty
        ///     rings buy that for eighty vertices.
        ///
        /// `coverage` (the alpha) is tied to the radiance rather than authored
        /// separately: where the glare is bright it is OPAQUE, which is why a
        /// star disappears when it drifts near the sun, and where it is faint
        /// it barely tints what is behind it.
        [[nodiscard]] inline sw::MeshData buildGlareDiscMesh(const sw::Vec3& hue,
                                                            sw::f32 radiance,
                                                            sw::f32 peakCoverage,
                                                            sw::f32 falloff,
                                                            sw::f32 plateau = 0.0f,
                                                            sw::f32 skirtWeight = 0.045f,
                                                            sw::f32 skirtRate = 30.0f)
        {
            sw::MeshData mesh;
            const sw::Vec3 normal{0.0f, 0.0f, 1.0f};
            constexpr sw::u32 kSegments = 72;
            constexpr sw::u32 kRings = 22;
            // ONE COLOUR PER LAYER, AND THE FALLOFF IN THE ALPHA. Both were in
            // the colour on the first attempt and the sun came out as a flat
            // cream BALL with a hard edge, because a src-alpha blend is not an
            // add: a layer whose alpha is high REPLACES what is behind it. A
            // faint outer glare with a high alpha is therefore an opaque disc
            // that happens to be dim, which is exactly what it looked like.
            //
            // Written this way each layer composites like an ADD over the sky:
            // colour C at coverage a contributes C*a and leaves (1-a) of the
            // stars showing. The core is the one layer allowed to reach a = 1,
            // because a star really does hide what is behind it.
            //
            // The colour is a RADIANCE and goes through the grade, so the core
            // (30) clips to white while the aureole (1.15) keeps its amber —
            // which is also physically the way it works. Bright things look
            // white; only the faint edge of a glare has a colour.
            const auto push = [&](sw::f32 radius, sw::f32 angle) {
                const sw::f32 t = radius;
                // A FLAT PLATEAU FIRST, and it is what a clipped source needs
                // rather than what looks tidy on a graph. From Saturn the
                // core disc is three pixels across, and with a bare pow() the
                // alpha had fallen to 0.45 by the time it reached the pixel
                // NEXT TO the centre — so the sun came out at sRGB 181, a grey
                // dot, dimmer than the background stars. A source that clips
                // has to clip over an area, not at a point.
                const sw::f32 u =
                    glm::clamp((t - plateau) / std::max(1.0f - plateau, 1.0e-4f),
                               0.0f, 1.0f);
                // A tight core plus a wide skirt: a plain pow() dies too
                // abruptly to read as glare, and a real point spread function
                // is the sum of exactly these two things.
                //
                // THE SKIRT IS THE WHOLE LAYER for the wide one, and it has to
                // be tunable per layer or the outer glare is a bump rather
                // than a tail. Fixed at 0.045/(1+30t^2) the aureole had fallen
                // to a thousandth of its peak a third of the way out, so the
                // sun from Saturn reached the sky floor at twenty-five pixels
                // while a BACKGROUND STAR's halo was still above it at forty.
                // The sun was measurably the smaller object in the frame.
                // A near-1/t^2 wing (rate 1.8) holds the aureole up to its own
                // rim, which is what a lens actually does with a source eight
                // orders of magnitude over the sensor's full scale.
                const sw::f32 core = std::pow(1.0f - u, falloff);
                const sw::f32 skirt = skirtWeight / (1.0f + skirtRate * t * t);
                // ...and every layer closes smoothly at its own rim, whatever
                // its exponent. A disc that still has a percent of alpha where
                // its geometry stops draws a visible circle in the sky — and
                // it did, a dotted one, because the rim is a 72-gon and the
                // seam between two triangles blends twice where they meet.
                // The taper reaches zero a tenth of the radius INSIDE the
                // geometry, so the last band of triangles carries nothing at
                // all and there is no seam left to see.
                const sw::f32 taper = sw::math::smoothstepf(0.90f, 0.62f, t);
                const sw::f32 coverage =
                    glm::clamp(peakCoverage * (core + skirt) * taper, 0.0f, 1.0f);
                mesh.vertices.push_back(
                    {{std::cos(angle) * radius, std::sin(angle) * radius, 0.0f},
                     normal,
                     {hue.r * radiance, hue.g * radiance, hue.b * radiance,
                      1.0f + coverage},
                     {}});
            };
            push(0.0f, 0.0f);
            for (sw::u32 ring = 1; ring <= kRings; ++ring)
            {
                // Rings bunched toward the centre, where all the structure is.
                const sw::f32 t = static_cast<sw::f32>(ring) / kRings;
                const sw::f32 radius = t * t;
                for (sw::u32 s = 0; s <= kSegments; ++s)
                {
                    push(radius, 6.2831853f * static_cast<sw::f32>(s) / kSegments);
                }
            }
            const sw::u32 stride = kSegments + 1;
            for (sw::u32 s = 0; s < kSegments; ++s)
            {
                mesh.indices.insert(mesh.indices.end(), {0u, 1u + s, 2u + s});
            }
            for (sw::u32 ring = 1; ring < kRings; ++ring)
            {
                const sw::u32 inner = 1u + (ring - 1u) * stride;
                const sw::u32 outer = 1u + ring * stride;
                for (sw::u32 s = 0; s < kSegments; ++s)
                {
                    mesh.indices.insert(mesh.indices.end(),
                                        {inner + s, outer + s, outer + s + 1,
                                         inner + s, outer + s + 1, inner + s + 1});
                }
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

        /// Saturn's rings: an annulus from 1.24 R to 2.27 R in the plane
        /// perpendicular to the spin axis, on the UNIT sphere (the instance
        /// scale is the planet's radius). Band brightness is radial noise,
        /// the Cassini division is an alpha gap, and the tilt is BAKED into
        /// the vertices: the ring rides a CloudLayer glued to the planet,
        /// whose rotation is about this same axis — a rotation an annulus
        /// about its own axis cannot show.
        /// CPU TWIN of ringOpacity() in Shaders/PlanetSurface.glsl. The mesh
        /// below is authored from it and the planet's shading casts a shadow
        /// from it, so the gaps you can see and the gaps in the shadow are
        /// the same gaps. `r` is in body radii.
        [[nodiscard]] inline sw::f32 ringOpacity(sw::f32 r)
        {
            using sw::math::fbm3;
            using sw::math::smoothstepf;
            const sw::f32 band = fbm3(sw::Vec3{r * 7.3f, 0.5f, 0.5f}, 3, 21210u);
            const sw::f32 fine = fbm3(sw::Vec3{r * 31.0f, 2.5f, 4.5f}, 2, 21211u);
            sw::f32 opacity = 0.62f * (0.55f + 0.60f * band) * (0.82f + 0.36f * fine);
            opacity *= smoothstepf(1.24f, 1.30f, r);
            opacity *= 1.0f - smoothstepf(2.17f, 2.27f, r);
            const sw::f32 cassini = smoothstepf(1.92f, 1.95f, r) *
                                    (1.0f - smoothstepf(1.99f, 2.03f, r));
            opacity *= 1.0f - 0.94f * cassini;
            const sw::f32 encke = smoothstepf(2.20f, 2.215f, r) *
                                  (1.0f - smoothstepf(2.225f, 2.24f, r));
            opacity *= 1.0f - 0.85f * encke;
            return glm::clamp(opacity, 0.0f, 1.0f);
        }

        [[nodiscard]] inline sw::MeshData buildRingMesh(const sw::Vec3& axis)
        {
            sw::MeshData mesh;
            const sw::Vec3 up = glm::normalize(axis);
            const sw::Vec3 seed =
                std::abs(up.x) < 0.9f ? sw::Vec3{1, 0, 0} : sw::Vec3{0, 1, 0};
            const sw::Vec3 e1 = glm::normalize(glm::cross(up, seed));
            const sw::Vec3 e2 = glm::cross(up, e1);

            // 160 rings, not 40. The radial direction is where ALL of a ring
            // system's structure lives — a ringlet is a few hundred
            // kilometres wide on a 74 000 km annulus — and forty steps could
            // not resolve the Cassini division's edge, let alone anything
            // finer. The segment count is untouched: nothing varies along a
            // circle.
            constexpr sw::u32 kSegments = 180;
            constexpr sw::u32 kRings = 160;
            constexpr sw::f32 kInner = 1.24f;
            constexpr sw::f32 kOuter = 2.27f;
            for (sw::u32 ring = 0; ring <= kRings; ++ring)
            {
                const sw::f32 t = static_cast<sw::f32>(ring) / kRings;
                const sw::f32 r = kInner + (kOuter - kInner) * t;
                const sw::f32 alpha = ringOpacity(r);
                // THE THREE RINGS ARE NOT THE SAME COLOUR. C is thin, dark
                // and slightly blue-grey; B is the bright one and the most
                // strongly reddened by whatever dirties the ice; A sits
                // between them. Cassini imaged this and it is most of what
                // makes a ring system read as a structure with a history
                // rather than as a grey disc.
                const sw::f32 toB = sw::math::smoothstepf(1.50f, 1.58f, r);
                const sw::f32 toA = sw::math::smoothstepf(2.00f, 2.05f, r);
                sw::Vec3 tone = glm::mix(sw::Vec3{0.52f, 0.53f, 0.58f},
                                         sw::Vec3{0.86f, 0.76f, 0.58f}, toB);
                tone = glm::mix(tone, sw::Vec3{0.72f, 0.67f, 0.58f}, toA);
                // Brightness follows the same structure the opacity does: a
                // dense ringlet is both more opaque and more reflective.
                tone *= 0.55f + 0.75f * alpha;
                for (sw::u32 s = 0; s <= kSegments; ++s)
                {
                    const sw::f32 angle =
                        2.0f * 3.14159265f * static_cast<sw::f32>(s) / kSegments;
                    const sw::Vec3 position =
                        (e1 * std::cos(angle) + e2 * std::sin(angle)) * r;
                    mesh.vertices.push_back(
                        {position, up, {tone.x, tone.y, tone.z, alpha}, {0.2f, 0.4f}});
                }
            }
            for (sw::u32 ring = 0; ring < kRings; ++ring)
            {
                for (sw::u32 s = 0; s < kSegments; ++s)
                {
                    const sw::u32 a = ring * (kSegments + 1) + s;
                    const sw::u32 b = a + kSegments + 1;
                    mesh.indices.insert(mesh.indices.end(),
                                        {a, b, a + 1, a + 1, b, b + 1});
                }
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
