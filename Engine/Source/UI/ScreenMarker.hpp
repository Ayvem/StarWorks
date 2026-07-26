#pragma once

// ============================================================================
// UI/ScreenMarker.hpp
// Where on the screen does a THING IN THE WORLD get its label?
//
// Every world-anchored overlay the game will ever draw — navigation beacons
// today, mine/site/vessel tags in F4, target and node markers — needs the
// same answer, and the same answer to the awkward half of the question:
// what do you draw when the thing is BEHIND you, or past the edge of the
// screen? Nothing is the wrong answer. You are looking for the beacon
// precisely because you cannot see it; a pointer that disappears exactly
// then is a pointer that never helps.
//
// So this is one pure function, unit-tested, with two rules:
//
//   * IN FRONT AND ON SCREEN — the honest perspective projection.
//   * OTHERWISE — clamped to the border, in the direction you would have to
//     turn to find it, and flagged `offScreen` so the caller can draw it
//     differently (an arrow, not a box).
//
// The behind-camera case cannot use the perspective divide at all: with
// w < 0 it mirrors the point through the origin and points you exactly the
// wrong way — a bug worth naming, because it looks plausible on screen and
// sends the pilot 180 degrees off course. The direction is rebuilt from the
// camera basis instead.
//
// Convention: NDC x right, y DOWN (Vulkan), both in [-1, 1].
// ============================================================================

#include "Math/Math.hpp"

namespace sw::ui
{
    struct MarkerPlacement
    {
        Vec2 ndc{0.0f, 0.0f};
        /// True when the marker was pushed to the border because the target
        /// is behind the camera or outside the visible rectangle.
        bool offScreen = false;
    };

    /// `cameraRelativePosition` is the target in the same camera-relative
    /// f32 space the renderer uses (world - camera). `edge` is the border
    /// the clamped marker sits on, in NDC (0.93 leaves room for its label).
    [[nodiscard]] inline MarkerPlacement placeScreenMarker(
        const Mat4& viewProjectionCameraRelative, const Vec3& cameraRelativePosition,
        const Vec3& cameraRight, const Vec3& cameraUp, const Vec3& cameraForward,
        f32 edge = 0.93f)
    {
        MarkerPlacement placement{};
        const Vec4 clip =
            viewProjectionCameraRelative * Vec4(cameraRelativePosition, 1.0f);

        if (clip.w > 0.0f)
        {
            placement.ndc = Vec2{clip.x / clip.w, clip.y / clip.w};
            placement.offScreen =
                std::abs(placement.ndc.x) > edge || std::abs(placement.ndc.y) > edge;
            if (!placement.offScreen)
            {
                return placement;
            }
        }
        else
        {
            placement.offScreen = true;
            const f32 length = glm::length(cameraRelativePosition);
            const Vec3 toTarget =
                (length > 1.0e-3f) ? cameraRelativePosition / length : cameraForward;
            // Screen Y grows downward, hence the negated up component.
            placement.ndc = Vec2{glm::dot(toTarget, cameraRight),
                                 -glm::dot(toTarget, cameraUp)};
        }

        // Push out to the border along the marker's own direction, so the
        // pointer sits where you would turn to find the target.
        Vec2 direction = placement.ndc;
        if (glm::length(direction) < 1.0e-4f)
        {
            direction = Vec2{0.0f, 1.0f}; // dead astern: send it straight down
        }
        direction = glm::normalize(direction);
        const f32 reach = std::max(std::abs(direction.x), std::abs(direction.y));
        placement.ndc = direction * (edge / std::max(reach, 1.0e-4f));
        return placement;
    }
} // namespace sw::ui
