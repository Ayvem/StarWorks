#pragma once

// ============================================================================
// Scene/FreeCameraController.hpp
// Editor-style free-fly camera controller.
//
// Controls:
//  - Hold right mouse button: capture cursor and look around.
//  - W/A/S/D: move forward/left/back/right, Q/E: down/up.
//  - Left Shift: speed boost. Mouse wheel: adjust base speed.
//
// The controller owns the yaw/pitch state and writes position/orientation
// into a Camera it does not own. It reads Input and toggles cursor capture
// on the Window — it is the only place those three systems meet.
// ============================================================================

#include "Math/Math.hpp"

namespace sw
{
    class Camera;
    class Input;
    class Window;

    class FreeCameraController
    {
    public:
        struct Settings
        {
            // Speeds in meters/second. The range is astronomical on purpose:
            // the wheel scales exponentially from inspecting a machine part
            // to crossing a planetary system.
            f32 moveSpeed = 100.0f;
            f32 boostMultiplier = 50.0f;    // while Left Shift is held
            f32 mouseSensitivity = 0.0025f; // radians per pixel
            f32 minSpeed = 0.5f;
            f32 maxSpeed = 2.0e7f;
        };

        explicit FreeCameraController(Camera& camera);

        /// Snaps the controller to the given pose and syncs the camera.
        void setPose(const WorldVec3& position, f32 yawRadians, f32 pitchRadians);

        void update(Input& input, Window& window, f32 deltaSeconds);

        [[nodiscard]] Settings& settings() { return m_settings; }

    private:
        void applyToCamera();

        Camera& m_camera;
        Settings m_settings{};

        WorldVec3 m_position{0.0, 0.0, 5.0};
        f32 m_yaw = 0.0f;   // radians, around world +Y
        f32 m_pitch = 0.0f; // radians, clamped to avoid gimbal flip
    };
} // namespace sw
