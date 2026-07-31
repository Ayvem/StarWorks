#pragma once

// ============================================================================
// Math/Frustum.hpp
// View-frustum extraction (Gribb-Hartmann) and bounding-sphere tests, used
// for CPU culling. Works on camera-relative matrices, so f32 is precise
// exactly where it matters. Convention-agnostic: planes are read from the
// actual view-projection matrix, so reverse-Z and the Vulkan Y-flip are
// handled automatically.
// ============================================================================

#include "Math/Math.hpp"

#include <array>

namespace sw
{
    class Frustum
    {
    public:
        /// Extracts the six planes from a (projection * view) matrix.
        /// The view matrix must be camera-relative (no world translation).
        [[nodiscard]] static Frustum fromViewProjection(const Mat4& viewProjection);

        /// True if a sphere (center in the same camera-relative space)
        /// intersects the frustum. Conservative: never culls anything
        /// visible; may keep borderline invisible spheres.
        [[nodiscard]] bool intersectsSphere(const Vec3& center, f32 radius) const;

    private:
        /// Planes as (normal.xyz, d); inside satisfies dot(n, p) + d >= 0.
        std::array<Vec4, 6> m_planes{};
    };
} // namespace sw
