#pragma once

// ============================================================================
// Physics/HullCollision.hpp
// SOLID OBJECTS.
//
// Parts now AUTHOR their collision hull as a handful of boxes (see
// parts::HitBox). This is what those boxes are FOR: you cannot walk through
// a refinery, a fuel tank or a power pole, and the game finds that out with
// a few dozen dot products rather than by testing every triangle it drew.
//
// Three things live here, all pure — no world, no components, no clock — so
// the arithmetic that decides where the player ends up can be tested without
// a scene, which is the only way it ever gets tested at all.
//
//   * `obbOverlap`   : do two oriented boxes touch (separating-axis, 15 axes)
//   * `obbPenetration`: ...and if so, the SHORTEST push that separates them.
//     Shortest matters: pushing along the deepest axis instead would shove a
//     player standing next to a wall out through the roof.
//   * `rayObb`       : what am I looking at? Used by the E panel, which asks
//     "near enough and in front of me" — a question a ray answers exactly
//     and a distance-to-centre check only approximates.
//
// EVERYTHING IS RELATIVE. The caller subtracts the mover's f64 position from
// every box centre before calling in, so these functions work in f32 metres
// around the player and never see a 6,371 km planet radius. That is not an
// optimisation, it is the only way f32 survives here at all.
// ============================================================================

#include "Math/Math.hpp"

#include <algorithm>
#include <cmath>
#include <span>

namespace sw::phys
{
    /// An oriented box: centre, half extents, and its own axes as columns.
    struct Obb
    {
        Vec3 centre{0.0f};
        Vec3 halfExtents{0.5f};
        Mat3 axes{1.0f};
    };

    [[nodiscard]] inline Obb makeObb(const Vec3& centre, const Vec3& halfExtents,
                                     const Quat& rotation)
    {
        Obb box{};
        box.centre = centre;
        box.halfExtents = glm::abs(halfExtents);
        box.axes = glm::mat3_cast(rotation);
        return box;
    }

    /// Bounding-sphere radius of a box — the broad phase's whole vocabulary.
    [[nodiscard]] inline f32 obbRadius(const Vec3& centre, const Vec3& halfExtents)
    {
        return glm::length(centre) + glm::length(glm::abs(halfExtents));
    }

    /// The SHORTEST translation that separates `a` from `b`, applied to `a`.
    ///
    /// Returns false when they are already apart. On true, `outAxis` is a
    /// unit vector pointing away from `b` and `outDepth` is how far along it
    /// `a` has to move — so `a.centre + outAxis * outDepth` just touches.
    ///
    /// Fifteen candidate axes: three of each box, and the nine cross
    /// products. Dropping the cross products is a classic shortcut and a
    /// classic bug — two boxes can miss on all six face axes and still
    /// overlap edge to edge, which is exactly a player wedged into the
    /// corner of a diagonal building.
    [[nodiscard]] inline bool obbPenetration(const Obb& a, const Obb& b, Vec3& outAxis,
                                             f32& outDepth)
    {
        constexpr f32 kEpsilon = 1.0e-6f;
        const Vec3 offset = b.centre - a.centre;

        f32 bestDepth = 1.0e30f;
        Vec3 bestAxis{0.0f};

        const auto test = [&](const Vec3& axis) {
            const f32 lengthSquared = glm::dot(axis, axis);
            if (lengthSquared < kEpsilon)
            {
                return true; // degenerate cross product: not a separating axis
            }
            const Vec3 unit = axis / std::sqrt(lengthSquared);
            const f32 distance = std::abs(glm::dot(offset, unit));
            f32 reach = 0.0f;
            for (i32 i = 0; i < 3; ++i)
            {
                reach += a.halfExtents[i] * std::abs(glm::dot(unit, Vec3(a.axes[i])));
                reach += b.halfExtents[i] * std::abs(glm::dot(unit, Vec3(b.axes[i])));
            }
            const f32 depth = reach - distance;
            if (depth <= 0.0f)
            {
                return false; // a separating axis: they do not touch
            }
            if (depth < bestDepth)
            {
                bestDepth = depth;
                // Point AWAY from b, so the caller can simply add it.
                bestAxis = (glm::dot(offset, unit) > 0.0f) ? -unit : unit;
            }
            return true;
        };

        for (i32 i = 0; i < 3; ++i)
        {
            if (!test(Vec3(a.axes[i]))) { return false; }
            if (!test(Vec3(b.axes[i]))) { return false; }
        }
        for (i32 i = 0; i < 3; ++i)
        {
            for (i32 j = 0; j < 3; ++j)
            {
                if (!test(glm::cross(Vec3(a.axes[i]), Vec3(b.axes[j])))) { return false; }
            }
        }

        outAxis = bestAxis;
        outDepth = bestDepth;
        return true;
    }

    [[nodiscard]] inline bool obbOverlap(const Obb& a, const Obb& b)
    {
        Vec3 axis{0.0f};
        f32 depth = 0.0f;
        return obbPenetration(a, b, axis, depth);
    }

    /// Ray against an oriented box, in the box's own frame (slab test).
    /// `direction` need not be normalised; `outT` is in units of it.
    [[nodiscard]] inline bool rayObb(const Vec3& origin, const Vec3& direction,
                                     const Obb& box, f32 maxT, f32& outT, Vec3& outNormal)
    {
        const Vec3 delta = origin - box.centre;
        // Into the box's frame: its axes are orthonormal, so the transpose
        // IS the inverse and there is nothing to invert.
        Vec3 localOrigin{};
        Vec3 localDirection{};
        for (i32 i = 0; i < 3; ++i)
        {
            localOrigin[i] = glm::dot(delta, Vec3(box.axes[i]));
            localDirection[i] = glm::dot(direction, Vec3(box.axes[i]));
        }

        f32 near = 0.0f;
        f32 far = maxT;
        i32 hitAxis = -1;
        f32 hitSign = 1.0f;
        for (i32 i = 0; i < 3; ++i)
        {
            const f32 half = std::abs(box.halfExtents[i]);
            if (std::abs(localDirection[i]) < 1.0e-8f)
            {
                // Parallel to this slab: either inside it forever, or never.
                if (std::abs(localOrigin[i]) > half)
                {
                    return false;
                }
                continue;
            }
            const f32 inverse = 1.0f / localDirection[i];
            f32 t0 = (-half - localOrigin[i]) * inverse;
            f32 t1 = (half - localOrigin[i]) * inverse;
            f32 sign = -1.0f;
            if (t0 > t1)
            {
                std::swap(t0, t1);
                sign = 1.0f;
            }
            if (t0 > near)
            {
                near = t0;
                hitAxis = i;
                hitSign = sign;
            }
            far = std::min(far, t1);
            if (near > far)
            {
                return false;
            }
        }
        outT = near;
        outNormal = (hitAxis >= 0) ? Vec3(box.axes[hitAxis]) * hitSign
                                   : Vec3{0.0f, 1.0f, 0.0f};
        return true;
    }
} // namespace sw::phys
