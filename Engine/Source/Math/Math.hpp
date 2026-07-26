#pragma once

// ============================================================================
// Math/Math.hpp
// Single entry point for engine mathematics.
//
// GLM is an implementation detail of this module: the rest of the engine
// uses the sw:: aliases below, never glm:: directly. This is the seam that
// will let us migrate to an in-house math library (or to f64/large-world
// types for interplanetary coordinates) without touching every module.
//
// Conventions:
//  - Right-handed world space, +Y up, -Z forward (camera looks down -Z).
//  - Depth range [0, 1] (GLM_FORCE_DEPTH_ZERO_TO_ONE, set by the build).
//  - Angles in radians everywhere (GLM_FORCE_RADIANS).
// ============================================================================

#include "Core/Types.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace sw
{
    using Vec2 = glm::vec2;
    using Vec3 = glm::vec3;
    using Vec4 = glm::vec4;
    using Mat3 = glm::mat3;
    using Mat4 = glm::mat4;
    using Quat = glm::quat;

    /// World-space positions use f64: the game runs at REAL astronomical
    /// scale (planet radii in the millions of meters, moons hundreds of
    /// millions away) where f32 precision collapses (~0.5 m ULP at Earth
    /// radius). Rule: POSITIONS in world space are WorldVec3; everything
    /// the GPU sees is f32 *camera-relative* (subtract the camera position
    /// in f64 first, then narrow) — precision is then highest exactly where
    /// the player is looking.
    using WorldVec3 = glm::dvec3;

    namespace math
    {
        inline constexpr f32 kPi = 3.14159265358979323846f;
        inline constexpr f32 kTwoPi = 2.0f * kPi;
        inline constexpr f32 kHalfPi = 0.5f * kPi;

        [[nodiscard]] inline constexpr f32 toRadians(f32 degrees)
        {
            return degrees * (kPi / 180.0f);
        }

        [[nodiscard]] inline constexpr f32 toDegrees(f32 radians)
        {
            return radians * (180.0f / kPi);
        }

        // Canonical world axes.
        inline constexpr Vec3 kWorldRight{1.0f, 0.0f, 0.0f};
        inline constexpr Vec3 kWorldUp{0.0f, 1.0f, 0.0f};
        inline constexpr Vec3 kWorldForward{0.0f, 0.0f, -1.0f};
    } // namespace math
} // namespace sw
