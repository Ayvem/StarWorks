#include "Math/Frustum.hpp"

namespace sw
{
    namespace
    {
        /// Row i of a column-major glm matrix.
        Vec4 row(const Mat4& m, int i)
        {
            return {m[0][i], m[1][i], m[2][i], m[3][i]};
        }

        Vec4 normalizePlane(const Vec4& plane)
        {
            const f32 length = glm::length(Vec3(plane));
            return (length > 0.0f) ? plane / length : plane;
        }
    } // namespace

    Frustum Frustum::fromViewProjection(const Mat4& vp)
    {
        const Vec4 r0 = row(vp, 0);
        const Vec4 r1 = row(vp, 1);
        const Vec4 r2 = row(vp, 2);
        const Vec4 r3 = row(vp, 3);

        Frustum frustum;
        frustum.m_planes[0] = normalizePlane(r3 + r0); // left
        frustum.m_planes[1] = normalizePlane(r3 - r0); // right
        frustum.m_planes[2] = normalizePlane(r3 + r1); // bottom
        frustum.m_planes[3] = normalizePlane(r3 - r1); // top
        frustum.m_planes[4] = normalizePlane(r2);      // z >= 0  (near or far under reverse-Z)
        frustum.m_planes[5] = normalizePlane(r3 - r2); // z <= w  (the other depth bound)
        return frustum;
    }

    bool Frustum::intersectsSphere(const Vec3& center, f32 radius) const
    {
        for (const Vec4& plane : m_planes)
        {
            if (glm::dot(Vec3(plane), center) + plane.w < -radius)
            {
                return false; // fully outside one plane
            }
        }
        return true;
    }
} // namespace sw
