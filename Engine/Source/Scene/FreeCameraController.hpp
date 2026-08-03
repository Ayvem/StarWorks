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

        /// Shifts the controller's own copy of the eye position, and ONLY
        /// that. It keeps a copy SEPARATE from the Camera's and pushes it down
        /// every input frame, so a floating-origin shift that moved only the
        /// Camera would be silently undone the moment the player touched a
        /// key. It deliberately does not touch the Camera: in flight the
        /// camera is driven by the craft, not by this, and writing to it here
        /// would teleport the view to wherever the free camera was last left.
        void translate(const WorldVec3& delta) { m_position += delta; }

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
