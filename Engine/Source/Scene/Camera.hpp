#pragma once

// ============================================================================
// Scene/Camera.hpp
// Perspective camera.
//
// The camera is pure data + math: it does not read input and does not know
// about windows. Controllers (e.g. FreeCameraController) drive it; the
// renderer only consumes its matrices. The projection matrix is built for
// Vulkan conventions: depth [0, 1] (reverse-Z) and Y flipped vs OpenGL.
//
// Large-world rule: the camera position is f64 (WorldVec3). Rendering is
// camera-relative — viewRotationMatrix() carries no translation; callers
// subtract position() from world positions in f64 before narrowing to f32.
// ============================================================================

#include "Math/Math.hpp"

namespace sw
{
    class Camera
    {
    public:
        Camera();

        void setPerspective(f32 verticalFovRadians, f32 nearPlane, f32 farPlane);
        void setAspectRatio(f32 aspect);

        void setPosition(const WorldVec3& position);
        void setOrientation(const Quat& orientation);

        /// Shifts the eye without touching where it is looking. Exists for the
        /// floating origin: when the world's origin moves to another star the
        /// camera has to move with it in the SAME frame, or the one frame in
        /// between is rendered from four light-years away.
        void translate(const WorldVec3& delta) { m_position += delta; }

        [[nodiscard]] const WorldVec3& position() const { return m_position; }
        [[nodiscard]] const Quat& orientation() const { return m_orientation; }
        [[nodiscard]] f32 verticalFov() const { return m_verticalFov; }
        [[nodiscard]] f32 nearPlane() const { return m_nearPlane; }
        [[nodiscard]] f32 farPlane() const { return m_farPlane; }

        // Basis vectors in world space.
        [[nodiscard]] Vec3 forward() const;
        [[nodiscard]] Vec3 right() const;
        [[nodiscard]] Vec3 up() const;

        /// Rotation-only view matrix (camera-relative rendering).
        [[nodiscard]] Mat4 viewRotationMatrix() const;
        [[nodiscard]] const Mat4& projectionMatrix() const;
        /// projection * viewRotation — apply to CAMERA-RELATIVE positions.
        [[nodiscard]] Mat4 viewProjectionCameraRelative() const;

        /// Narrows a world position to f32 camera-relative space.
        [[nodiscard]] Vec3 worldToCameraRelative(const WorldVec3& worldPosition) const
        {
            return Vec3(worldPosition - m_position);
        }

    private:
        void rebuildProjection();

        WorldVec3 m_position{0.0, 0.0, 0.0};
        Quat m_orientation{1.0f, 0.0f, 0.0f, 0.0f}; // identity (w, x, y, z)

        f32 m_verticalFov = math::toRadians(60.0f);
        f32 m_aspectRatio = 16.0f / 9.0f;
        f32 m_nearPlane = 0.1f;
        f32 m_farPlane = 1.0e9f; // reverse-Z keeps precision usable this far

        Mat4 m_projection{1.0f};
    };
} // namespace sw
