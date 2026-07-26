#include "Scene/Camera.hpp"

namespace sw
{
    Camera::Camera()
    {
        rebuildProjection();
    }

    void Camera::setPerspective(f32 verticalFovRadians, f32 nearPlane, f32 farPlane)
    {
        m_verticalFov = verticalFovRadians;
        m_nearPlane = nearPlane;
        m_farPlane = farPlane;
        rebuildProjection();
    }

    void Camera::setAspectRatio(f32 aspect)
    {
        if (aspect > 0.0f && aspect != m_aspectRatio)
        {
            m_aspectRatio = aspect;
            rebuildProjection();
        }
    }

    void Camera::setPosition(const WorldVec3& position)
    {
        m_position = position;
    }

    void Camera::setOrientation(const Quat& orientation)
    {
        m_orientation = glm::normalize(orientation);
    }

    Vec3 Camera::forward() const
    {
        return m_orientation * math::kWorldForward;
    }

    Vec3 Camera::right() const
    {
        return m_orientation * math::kWorldRight;
    }

    Vec3 Camera::up() const
    {
        return m_orientation * math::kWorldUp;
    }

    Mat4 Camera::viewRotationMatrix() const
    {
        // No translation: rendering happens in camera-relative space, where
        // the camera sits at the origin by construction.
        return glm::mat4_cast(glm::conjugate(m_orientation));
    }

    const Mat4& Camera::projectionMatrix() const
    {
        return m_projection;
    }

    Mat4 Camera::viewProjectionCameraRelative() const
    {
        return m_projection * viewRotationMatrix();
    }

    void Camera::rebuildProjection()
    {
        // Reverse-Z: passing (far, near) to the [0,1]-depth perspective maps
        // the near plane to depth 1 and the far plane to depth 0. Combined
        // with GREATER_OR_EQUAL depth tests and a 0.0 depth clear, this gives
        // vastly better f32 depth precision at large distances — a hard
        // requirement for planetary/orbital scales.
        m_projection = glm::perspective(m_verticalFov, m_aspectRatio, m_farPlane, m_nearPlane);
        // GLM builds OpenGL-style projections (+Y up in NDC); Vulkan's NDC has
        // +Y pointing down, so flip the Y axis here, once, at the source.
        m_projection[1][1] *= -1.0f;
    }
} // namespace sw
