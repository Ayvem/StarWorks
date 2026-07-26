// ============================================================================
// FrustumTests.cpp — culling correctness against the engine's actual camera
// conventions (reverse-Z projection, Vulkan Y flip, camera-relative view).
// ============================================================================

#include "TestFramework.hpp"

#include <Math/Frustum.hpp>
#include <Scene/Camera.hpp>

namespace
{
    /// Camera at origin, default orientation (looking down -Z), 60° FOV.
    sw::Frustum makeTestFrustum(sw::f32 nearPlane = 0.1f, sw::f32 farPlane = 500.0f)
    {
        sw::Camera camera;
        camera.setPerspective(sw::math::toRadians(60.0f), nearPlane, farPlane);
        camera.setAspectRatio(16.0f / 9.0f);
        return sw::Frustum::fromViewProjection(camera.viewProjectionCameraRelative());
    }
} // namespace

SW_TEST(FrustumAcceptsVisibleSpheres)
{
    const sw::Frustum frustum = makeTestFrustum();

    SW_CHECK(frustum.intersectsSphere({0.0f, 0.0f, -10.0f}, 1.0f));  // dead center
    SW_CHECK(frustum.intersectsSphere({0.0f, 0.0f, -499.0f}, 2.0f)); // near the far plane
    SW_CHECK(frustum.intersectsSphere({0.0f, 0.0f, 0.0f}, 5.0f));    // surrounds the camera
    // Straddles the near plane: partially visible, must be kept.
    SW_CHECK(frustum.intersectsSphere({0.0f, 0.0f, -0.05f}, 1.0f));
}

SW_TEST(FrustumRejectsInvisibleSpheres)
{
    const sw::Frustum frustum = makeTestFrustum();

    SW_CHECK(!frustum.intersectsSphere({0.0f, 0.0f, 10.0f}, 1.0f));    // behind the camera
    SW_CHECK(!frustum.intersectsSphere({0.0f, 0.0f, -1000.0f}, 1.0f)); // beyond far plane
    SW_CHECK(!frustum.intersectsSphere({-500.0f, 0.0f, -10.0f}, 1.0f)); // far left
    SW_CHECK(!frustum.intersectsSphere({500.0f, 0.0f, -10.0f}, 1.0f));  // far right
    SW_CHECK(!frustum.intersectsSphere({0.0f, 500.0f, -10.0f}, 1.0f));  // far above
    SW_CHECK(!frustum.intersectsSphere({0.0f, -500.0f, -10.0f}, 1.0f)); // far below
}

SW_TEST(FrustumHandlesPlanetScaleRadii)
{
    // Planet-sized bounding sphere at planet distance (camera-relative
    // values as they occur in the real scene at 400 km altitude).
    const sw::Frustum frustum = makeTestFrustum(0.5f, 1.0e9f);

    // Terra: center ~6.771e6 m "below" the camera, radius 6.371e6 m.
    SW_CHECK(frustum.intersectsSphere({0.0f, 0.0f, -6.771e6f}, 6.371e6f));
    // Luna-sized sphere far to the side but inside the far plane: visible
    // when centered ahead, culled when far off-axis.
    SW_CHECK(frustum.intersectsSphere({0.0f, 0.0f, -3.844e8f}, 1.737e6f));
    SW_CHECK(!frustum.intersectsSphere({3.844e8f, 0.0f, -10.0f}, 1.737e6f));
}
