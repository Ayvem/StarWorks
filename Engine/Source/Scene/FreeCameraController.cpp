#include "Scene/FreeCameraController.hpp"

#include "Input/Input.hpp"
#include "Platform/Window.hpp"
#include "Scene/Camera.hpp"

#include <algorithm>
#include <cmath>

namespace sw
{
    FreeCameraController::FreeCameraController(Camera& camera)
        : m_camera(camera)
    {
        applyToCamera();
    }

    void FreeCameraController::setPose(const WorldVec3& position, f32 yawRadians,
                                       f32 pitchRadians)
    {
        m_position = position;
        m_yaw = yawRadians;
        m_pitch = pitchRadians;
        applyToCamera();
    }

    void FreeCameraController::update(Input& input, Window& window, f32 deltaSeconds)
    {
        // --- look (only while right mouse button is held) -------------------
        const bool looking = input.isMouseButtonDown(MouseButton::Right);
        if (looking != window.isCursorCaptured())
        {
            window.setCursorCaptured(looking);
            input.resetMouseDelta(); // avoid a spike from the cursor jump
        }

        if (looking)
        {
            m_yaw -= input.mouseDeltaX() * m_settings.mouseSensitivity;
            m_pitch -= input.mouseDeltaY() * m_settings.mouseSensitivity;

            constexpr f32 kPitchLimit = math::kHalfPi - 0.01f;
            m_pitch = std::clamp(m_pitch, -kPitchLimit, kPitchLimit);

            // Keep yaw in a sane numeric range.
            if (m_yaw > math::kPi) { m_yaw -= math::kTwoPi; }
            if (m_yaw < -math::kPi) { m_yaw += math::kTwoPi; }
        }

        // --- speed adjustment via mouse wheel --------------------------------
        const f32 scroll = input.scrollDeltaY();
        if (scroll != 0.0f)
        {
            const f32 factor = std::pow(1.2f, scroll);
            m_settings.moveSpeed = std::clamp(m_settings.moveSpeed * factor,
                                              m_settings.minSpeed, m_settings.maxSpeed);
        }

        // --- translation ------------------------------------------------------
        // Yaw around world up, then pitch around the resulting local right axis.
        const Quat orientation = glm::angleAxis(m_yaw, math::kWorldUp) *
                                 glm::angleAxis(m_pitch, math::kWorldRight);
        const Vec3 forward = orientation * math::kWorldForward;
        const Vec3 right = orientation * math::kWorldRight;

        Vec3 move{0.0f};
        if (input.isKeyDown(KeyCode::W)) { move += forward; }
        if (input.isKeyDown(KeyCode::S)) { move -= forward; }
        if (input.isKeyDown(KeyCode::D)) { move += right; }
        if (input.isKeyDown(KeyCode::A)) { move -= right; }
        if (input.isKeyDown(KeyCode::E)) { move += math::kWorldUp; }
        if (input.isKeyDown(KeyCode::Q)) { move -= math::kWorldUp; }

        if (glm::dot(move, move) > 0.0f)
        {
            f32 speed = m_settings.moveSpeed;
            if (input.isKeyDown(KeyCode::LeftShift))
            {
                speed *= m_settings.boostMultiplier;
            }
            // Accumulate in f64 world space: at planetary distances an f32
            // position would quantize into visible jumps.
            m_position += WorldVec3(glm::normalize(move)) *
                          static_cast<f64>(speed * deltaSeconds);
        }

        applyToCamera();
    }

    void FreeCameraController::applyToCamera()
    {
        m_camera.setPosition(m_position);
        m_camera.setOrientation(glm::angleAxis(m_yaw, math::kWorldUp) *
                                glm::angleAxis(m_pitch, math::kWorldRight));
    }
} // namespace sw
