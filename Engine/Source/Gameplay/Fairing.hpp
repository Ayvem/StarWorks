#pragma once

// ============================================================================
// Gameplay/Fairing.hpp
// A PAYLOAD SHELL THE PLAYER DRAWS.
//
// Every other part in this game is a file: its shapes, its nodes and its drag
// table are authored once and every instance of it is identical. A fairing
// cannot be, because its whole purpose is to be the shape of whatever is
// inside it — and what is inside it is a design that did not exist when the
// part was made.
//
// So a fairing is a base part plus a PROFILE: a short list of (height, radius)
// rings the player places with the cursor, exactly as KSP does it — each click
// puts a ring where the cursor is, the wall follows the mouse between clicks,
// and bringing the radius back to the axis closes the nose. Everything else
// here is derived from that list:
//
//   * the MESH, lathed into `sides` flat panels per band — which is what a
//     real fairing is, and the reason the aerodynamics below can be honest
//     without a solver;
//   * the MASS and the COST, from the wetted area, because a bigger shell is
//     more aluminium and there is nothing else it could be;
//   * WHAT IS INSIDE, which is the whole point: a part within the profile is
//     shielded from the airflow completely;
//   * the DRAG, summed over those panels with the same three-term law the
//     offline forge uses on every other part, so a fairing and a nose cone
//     are priced by the same physics.
//
// The profile is fixed-size and trivially copyable because it rides on an
// entity and the save is a column memcpy. Twelve rings is a shape with eleven
// bends in it; past that the player is drawing rather than building.
// ============================================================================

#include "Core/Types.hpp"
#include "Math/Math.hpp"

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace sw::parts
{
    struct FairingComponent
    {
        static constexpr u32 kMaxRings = 12;
        static constexpr u32 kMinSides = 3;
        static constexpr u32 kMaxSides = 24;

        /// x = height above the base's mounting face, metres, strictly
        /// increasing; y = radius there, metres. Ring 0 is the base's own
        /// rim, so a profile is only a shell once it has two of them.
        Vec2 rings[kMaxRings]{};
        u32 ringCount = 0;
        /// How many flat panels the lathe cuts it into — and, when it is
        /// thrown away, how many pieces come off.
        u32 sides = 8;
        /// 1 once the nose has been closed. An open profile is a draft: it
        /// shields nothing and it is not drawn in flight.
        u32 closed = 0;
        /// 1 once it has been thrown away. The shell stops existing for the
        /// drag, the shielding and the renderer alike, and the base stays on
        /// the rocket — which is what is left of a real one too.
        u32 jettisoned = 0;
    };
    static_assert(std::is_trivially_copyable_v<FairingComponent>);

    /// The radius of the shell at a height above the mounting face, or 0
    /// outside the profile. Linear between rings, because the panels are
    /// straight and this is the same line they are cut along.
    [[nodiscard]] inline f32 fairingRadiusAt(const FairingComponent& fairing, f32 height)
    {
        if (fairing.ringCount < 2 || height < fairing.rings[0].x ||
            height > fairing.rings[fairing.ringCount - 1].x)
        {
            return 0.0f;
        }
        for (u32 i = 1; i < fairing.ringCount; ++i)
        {
            const Vec2 lower = fairing.rings[i - 1];
            const Vec2 upper = fairing.rings[i];
            if (height <= upper.x)
            {
                const f32 span = upper.x - lower.x;
                const f32 t = (span > 1.0e-6f) ? (height - lower.x) / span : 0.0f;
                return glm::mix(lower.y, upper.y, t);
            }
        }
        return fairing.rings[fairing.ringCount - 1].y;
    }

    /// Is a point inside the shell? `point` is in the FAIRING PART's frame,
    /// with the profile measured along -Z — the nose direction every part in
    /// this game is built along.
    ///
    /// Note what this does NOT do: it takes a point, not a bounding box, and
    /// it takes it whole. A part half in and half out of a fairing is a build
    /// nobody should be flying, and pretending to shield half of it would be
    /// a number invented to paper over that.
    [[nodiscard]] inline bool fairingEncloses(const FairingComponent& fairing,
                                              const Vec3& point)
    {
        if (fairing.closed == 0 || fairing.jettisoned != 0 || fairing.ringCount < 2)
        {
            return false;
        }
        const f32 height = -point.z; // -Z is up the stack
        const f32 radius = fairingRadiusAt(fairing, height);
        if (!(radius > 0.0f))
        {
            return false;
        }
        return glm::length(Vec2{point.x, point.y}) <= radius;
    }

    /// Total wetted area of the shell, m^2 — the lateral area of the stack of
    /// truncated cones the profile describes.
    [[nodiscard]] inline f64 fairingAreaM2(const FairingComponent& fairing)
    {
        constexpr f64 kPi = 3.14159265358979323846;
        f64 area = 0.0;
        for (u32 i = 1; i < fairing.ringCount; ++i)
        {
            const Vec2 lower = fairing.rings[i - 1];
            const Vec2 upper = fairing.rings[i];
            const f64 dz = static_cast<f64>(upper.x - lower.x);
            const f64 dr = static_cast<f64>(upper.y - lower.y);
            const f64 slant = std::sqrt(dz * dz + dr * dr);
            area += kPi * (static_cast<f64>(lower.y) + static_cast<f64>(upper.y)) * slant;
        }
        return area;
    }

    /// Aluminium panel, ribs and separation hardware, per square metre of
    /// shell. A three-metre shroud over a two-metre payload lands near a
    /// hundred kilograms, which is the right order for the real thing.
    inline constexpr f64 kFairingMassPerM2 = 11.0;
    inline constexpr f64 kFairingCostPerM2 = 130.0;

    [[nodiscard]] inline f64 fairingMassKg(const FairingComponent& fairing)
    {
        return fairingAreaM2(fairing) * kFairingMassPerM2;
    }

    [[nodiscard]] inline f64 fairingCostCredits(const FairingComponent& fairing)
    {
        return fairingAreaM2(fairing) * kFairingCostPerM2;
    }

    /// How far up the stack the shell's mass sits, metres above the mounting
    /// face, area-weighted. It matters: a half-tonne shroud is metres AHEAD of
    /// the base it bolts to, so weighing it at the base would put the balance
    /// point of every shrouded rocket in the wrong place — and moving it back
    /// again is most of what throwing the thing away is for.
    [[nodiscard]] inline f32 fairingCentroidHeight(const FairingComponent& fairing)
    {
        constexpr f64 kPi = 3.14159265358979323846;
        f64 weighted = 0.0;
        f64 total = 0.0;
        for (u32 i = 1; i < fairing.ringCount; ++i)
        {
            const Vec2 lower = fairing.rings[i - 1];
            const Vec2 upper = fairing.rings[i];
            const f64 dz = static_cast<f64>(upper.x - lower.x);
            const f64 dr = static_cast<f64>(upper.y - lower.y);
            const f64 area = kPi * (static_cast<f64>(lower.y) + static_cast<f64>(upper.y)) *
                             std::sqrt(dz * dz + dr * dr);
            weighted += area * static_cast<f64>(lower.x + upper.x) * 0.5;
            total += area;
        }
        return (total > 1.0e-9) ? static_cast<f32>(weighted / total) : 0.0f;
    }

    /// Is this profile a shell that flies — closed, still attached, and with
    /// enough rings to have a surface? The three questions are always asked
    /// together, and asking two of them was a bug twice.
    [[nodiscard]] inline bool fairingIsFlying(const FairingComponent& fairing)
    {
        return fairing.closed != 0 && fairing.jettisoned == 0 && fairing.ringCount >= 2;
    }

    /// One panel's corner ring, as the mesh and the aerodynamics both see it:
    /// side `s` of `sides` spans the angles [s, s+1] * 2pi/sides.
    [[nodiscard]] inline Vec3 fairingCorner(const FairingComponent& fairing, u32 ring,
                                            u32 corner)
    {
        constexpr f32 kTwoPi = 6.28318530717958647692f;
        const u32 sides = std::clamp(fairing.sides, FairingComponent::kMinSides,
                                     FairingComponent::kMaxSides);
        const f32 angle = kTwoPi * static_cast<f32>(corner % sides) /
                          static_cast<f32>(sides);
        const Vec2 profile = fairing.rings[std::min(ring, fairing.ringCount - 1)];
        return Vec3{profile.y * std::cos(angle), profile.y * std::sin(angle),
                    -profile.x};
    }

    // ------------------------------------------------------------------------
    // THE AERODYNAMICS OF A BOX OF FLAT PLATES
    //
    // A fairing has no `.aero.json` and cannot have one: the offline forge
    // photographs a part's silhouette from four hundred directions, and this
    // part's silhouette is drawn by the player five minutes before it flies.
    //
    // It does not need one. Every other part is solved by pointing a depth
    // buffer at an arbitrary mesh because an arbitrary mesh is what it is; a
    // fairing is a stack of truncated cones cut into flat quads, and the same
    // three-term law the forge integrates per pixel can be integrated per
    // PANEL in closed form. Same constants, same physics, one loop:
    //
    //   IMPACT     Cp = Cp_max cos^2(theta) on every panel the flow can see,
    //   BASE       Cp = -0.20 on the ones it cannot,
    //   FRICTION   Cf = 0.004 along the surface, over the wetted area.
    //
    // Returned per unit of dynamic pressure, in the same m^2 / m^3 units an
    // AeroSample carries, so the caller multiplies by q exactly as it does
    // for a tabulated part.
    // ------------------------------------------------------------------------
    inline constexpr f64 kFairingStagnationCp = 1.0;
    inline constexpr f64 kFairingBaseCp = -0.20;
    inline constexpr f64 kFairingSkinCf = 0.004;

    inline void fairingAero(const FairingComponent& fairing,
                            const Vec3& flowDirectionPartFrame, Vec3& outForceM2,
                            Vec3& outMomentM3)
    {
        outForceM2 = Vec3{0.0f};
        outMomentM3 = Vec3{0.0f};
        if (fairing.closed == 0 || fairing.jettisoned != 0 || fairing.ringCount < 2)
        {
            return;
        }
        const f32 flowLength = glm::length(flowDirectionPartFrame);
        if (!(flowLength > 1.0e-6f))
        {
            return;
        }
        const Vec3 flow = flowDirectionPartFrame / flowLength;
        const u32 sides = std::clamp(fairing.sides, FairingComponent::kMinSides,
                                     FairingComponent::kMaxSides);
        glm::dvec3 force{0.0};
        glm::dvec3 moment{0.0};
        for (u32 ring = 1; ring < fairing.ringCount; ++ring)
        {
            for (u32 side = 0; side < sides; ++side)
            {
                // The quad, in order round its edge.
                const Vec3 a = fairingCorner(fairing, ring - 1, side);
                const Vec3 b = fairingCorner(fairing, ring - 1, side + 1);
                const Vec3 c = fairingCorner(fairing, ring, side + 1);
                const Vec3 d = fairingCorner(fairing, ring, side);
                const Vec3 centre = (a + b + c + d) * 0.25f;
                // Newell's normal: correct for a quad that is not quite
                // planar, which the top band of a closed nose never is.
                Vec3 normal = glm::cross(c - a, d - b) * 0.5f;
                const f32 area = glm::length(normal);
                if (!(area > 1.0e-9f))
                {
                    continue;
                }
                normal /= area;
                // OUTWARD. The lathe winds one way and a sign error here
                // would push the shell inside out.
                if (glm::dot(normal, Vec3{centre.x, centre.y, 0.0f}) < 0.0f)
                {
                    normal = -normal;
                }

                const f64 facing = static_cast<f64>(glm::dot(normal, -flow));
                const f64 pressureCp = (facing > 0.0)
                                           ? kFairingStagnationCp * facing * facing
                                           : kFairingBaseCp;
                const glm::dvec3 pressureForce =
                    glm::dvec3(-normal) * (pressureCp * static_cast<f64>(area));
                // Shear along the surface, in the direction the air slides.
                const glm::dvec3 tangent =
                    glm::dvec3(flow) - glm::dvec3(normal) * static_cast<f64>(
                                                                glm::dot(flow, normal));
                const f64 tangentLength = glm::length(tangent);
                const glm::dvec3 shear =
                    (tangentLength > 1.0e-9)
                        ? (tangent / tangentLength) * (kFairingSkinCf *
                                                       static_cast<f64>(area))
                        : glm::dvec3{0.0};
                const glm::dvec3 panelForce = pressureForce + shear;
                force += panelForce;
                moment += glm::cross(glm::dvec3(centre), panelForce);
            }
        }
        outForceM2 = Vec3(force);
        outMomentM3 = Vec3(moment);
    }
} // namespace sw::parts
