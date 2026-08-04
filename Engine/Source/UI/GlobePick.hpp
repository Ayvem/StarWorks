#pragma once

// ============================================================================
// UI/GlobePick.hpp
// CLICKING A SPHERE, and orbiting a camera around one.
//
// Two pure functions, here rather than inside the screen that uses them,
// because both are the kind of arithmetic that is wrong in a way no
// screenshot shows: a ray-sphere solve that takes the FAR root picks the point
// on the back of the globe — which is a plausible-looking latitude, on the
// wrong side of the planet, and the beacon it drops is simply somewhere else.
// Pure and out here, a test can hold them.
// ============================================================================

#include "Core/Types.hpp"
#include "Math/Math.hpp"

#include <cmath>

namespace sw::ui
{
    /// Where a camera orbiting the origin sits, for a yaw, a pitch and a
    /// distance. The one definition of the convention: yaw turns about +Y from
    /// +Z, pitch lifts toward +Y.
    [[nodiscard]] inline Vec3 orbitCameraOffset(f32 yaw, f32 pitch, f32 distance)
    {
        const f32 cosPitch = std::cos(pitch);
        return Vec3{cosPitch * std::sin(yaw) * distance, std::sin(pitch) * distance,
                    cosPitch * std::cos(yaw) * distance};
    }

    /// The yaw and pitch that put that camera in front of `direction` — the
    /// inverse of the above, used to frame a mark that is on the far side.
    inline void orbitCameraAim(const Vec3& direction, f32& outYaw, f32& outPitch)
    {
        outYaw = std::atan2(direction.x, direction.z);
        outPitch = std::asin(glm::clamp(direction.y, -1.0f, 1.0f));
    }

    /// First intersection of a ray with the UNIT SPHERE at the origin, as a
    /// unit direction. False when the ray misses, or when the sphere is
    /// entirely behind the eye.
    ///
    /// NEAR ROOT, always: the far one is the point on the back of the globe,
    /// and a pick that returns it looks like a working pick until someone
    /// checks which hemisphere the mark landed in.
    [[nodiscard]] inline bool pickUnitSphere(const Vec3& eye, const Vec3& rayDirection,
                                             Vec3& outDirection)
    {
        const f32 lengthSq = glm::dot(rayDirection, rayDirection);
        if (!(lengthSq > 0.0f))
        {
            return false;
        }
        const Vec3 direction = rayDirection / std::sqrt(lengthSq);
        const f32 along = -glm::dot(eye, direction);
        const f32 missSq = glm::dot(eye, eye) - along * along;
        if (missSq > 1.0f)
        {
            return false; // the ray passes beside the globe
        }
        const f32 half = std::sqrt(std::max(1.0f - missSq, 0.0f));
        const f32 hit = along - half;
        if (hit <= 0.0f)
        {
            return false; // behind the eye, or the eye is inside the sphere
        }
        outDirection = glm::normalize(eye + direction * hit);
        return true;
    }
} // namespace sw::ui
