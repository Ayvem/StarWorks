// ============================================================================
// ScreenMarkerTests.cpp — where a world-anchored label lands on screen.
//
// The interesting cases are not the easy one. A marker straight ahead is
// arithmetic anyone gets right; the two that matter are BEHIND THE CAMERA
// (where the perspective divide silently points you 180 degrees the wrong
// way) and PAST THE EDGE (where doing nothing hides the pointer exactly
// when the pilot is hunting for it). Both are pinned here.
// ============================================================================

#include "TestFramework.hpp"

#include <Scene/Camera.hpp>
#include <UI/ScreenMarker.hpp>

#include <cmath>

using namespace sw;

namespace
{
    /// A camera at the origin looking down -Z (the engine's identity pose),
    /// with a 16:9 60-degree perspective.
    Camera identityCamera()
    {
        Camera camera;
        camera.setPerspective(math::toRadians(60.0f), 0.5f, 1.0e9f);
        camera.setAspectRatio(16.0f / 9.0f);
        return camera;
    }

    ui::MarkerPlacement place(const Camera& camera, const Vec3& cameraRelative)
    {
        return ui::placeScreenMarker(camera.viewProjectionCameraRelative(), cameraRelative,
                                     camera.right(), camera.up(), camera.forward());
    }
} // namespace

SW_TEST(ScreenMarkerCentresATargetDeadAhead)
{
    const Camera camera = identityCamera();
    const ui::MarkerPlacement placement = place(camera, camera.forward() * 5000.0f);

    SW_CHECK(!placement.offScreen);
    SW_CHECK(std::abs(placement.ndc.x) < 1.0e-4f);
    SW_CHECK(std::abs(placement.ndc.y) < 1.0e-4f);
}

SW_TEST(ScreenMarkerPutsRightAndUpOnTheRightSides)
{
    const Camera camera = identityCamera();

    // A target to the camera's right lands on the right half of the screen.
    const ui::MarkerPlacement right =
        place(camera, camera.forward() * 1000.0f + camera.right() * 60.0f);
    SW_CHECK(!right.offScreen);
    SW_CHECK(right.ndc.x > 0.0f);

    // A target ABOVE the camera lands on the upper half — and screen Y grows
    // DOWNWARD, so "upper" means a NEGATIVE y. Getting this backwards would
    // put every label on the wrong side of the horizon.
    const ui::MarkerPlacement above =
        place(camera, camera.forward() * 1000.0f + camera.up() * 60.0f);
    SW_CHECK(!above.offScreen);
    SW_CHECK(above.ndc.y < 0.0f);
}

SW_TEST(ScreenMarkerBehindTheCameraPointsBackwardsNotForwards)
{
    const Camera camera = identityCamera();

    // Behind and to the RIGHT. The perspective divide with w < 0 would
    // mirror this through the origin and report "left" — sending the pilot
    // the wrong way round. The placement must say right, and must say it is
    // off screen.
    const Vec3 target = -camera.forward() * 4000.0f + camera.right() * 1000.0f;
    const ui::MarkerPlacement placement = place(camera, target);

    SW_CHECK(placement.offScreen);
    SW_CHECK(placement.ndc.x > 0.0f);

    // Mirroring the target mirrors the pointer: no dead zone, no flip.
    const ui::MarkerPlacement mirrored =
        place(camera, -camera.forward() * 4000.0f - camera.right() * 1000.0f);
    SW_CHECK(mirrored.offScreen);
    SW_CHECK(mirrored.ndc.x < 0.0f);
}

SW_TEST(ScreenMarkerClampsToTheBorderAndStaysOnIt)
{
    const Camera camera = identityCamera();

    // Far off to one side, in front and behind: every clamped pointer must
    // land ON the border box (its larger coordinate exactly at the edge)
    // and never outside it, or it would be drawn off the screen.
    constexpr f32 kEdge = 0.93f;
    const Vec3 targets[] = {
        camera.forward() * 100.0f + camera.right() * 900.0f,
        camera.forward() * 100.0f - camera.right() * 900.0f,
        camera.forward() * 100.0f + camera.up() * 900.0f,
        -camera.forward() * 100.0f + camera.right() * 900.0f,
        -camera.forward() * 100.0f - camera.up() * 900.0f,
        -camera.forward() * 900.0f, // dead astern: the degenerate case
    };
    for (const Vec3& target : targets)
    {
        const ui::MarkerPlacement placement = place(camera, target);
        SW_CHECK(placement.offScreen);
        const f32 reach =
            std::max(std::abs(placement.ndc.x), std::abs(placement.ndc.y));
        SW_CHECK(std::abs(reach - kEdge) < 1.0e-3f);
        SW_CHECK(std::abs(placement.ndc.x) <= kEdge + 1.0e-3f);
        SW_CHECK(std::abs(placement.ndc.y) <= kEdge + 1.0e-3f);
    }
}

SW_TEST(ScreenMarkerIsStableAsATargetCrossesTheEdge)
{
    // Sweeping a target from straight ahead round to straight behind must
    // never produce a NaN and never leave the border box: a pointer that
    // jumps to the far corner for one frame reads as a bug in the beacon.
    const Camera camera = identityCamera();
    constexpr f32 kEdge = 0.93f;
    for (i32 step = 0; step <= 180; ++step)
    {
        const f32 angle = math::toRadians(static_cast<f32>(step));
        const Vec3 target =
            camera.forward() * (std::cos(angle) * 2000.0f) +
            camera.right() * (std::sin(angle) * 2000.0f);
        const ui::MarkerPlacement placement = place(camera, target);
        SW_CHECK(std::isfinite(placement.ndc.x));
        SW_CHECK(std::isfinite(placement.ndc.y));
        SW_CHECK(std::abs(placement.ndc.x) <= kEdge + 1.0e-3f);
        SW_CHECK(std::abs(placement.ndc.y) <= kEdge + 1.0e-3f);
        // Everything from 90 degrees out is off screen at this FOV.
        if (step >= 90)
        {
            SW_CHECK(placement.offScreen);
        }
    }
}
