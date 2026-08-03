#include "Gameplay/PartGeometry.hpp"

#include <algorithm>
#include <cmath>

namespace sw::parts
{
    namespace
    {
        constexpr f32 kPi = 3.14159265358979f;

        [[nodiscard]] u32 clampSegments(u32 segments)
        {
            return std::clamp(segments, 3u, 64u);
        }

        void pushVertex(MeshData& mesh, const Vec3& position, const Vec3& normal,
                        const Vec4& color)
        {
            Vertex vertex{};
            vertex.position = position;
            vertex.normal = normal;
            vertex.color = color;
            mesh.vertices.push_back(vertex);
        }

        void pushTriangle(MeshData& mesh, u32 a, u32 b, u32 c)
        {
            mesh.indices.push_back(a);
            mesh.indices.push_back(b);
            mesh.indices.push_back(c);
        }

        /// Box centered at origin, half extents `h`, flat-face normals.
        void buildBox(MeshData& mesh, const Vec3& h, const Vec4& color)
        {
            const Vec3 normals[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                                     {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
            for (const Vec3& normal : normals)
            {
                // Build a tangent basis for the face.
                const Vec3 up = std::abs(normal.z) > 0.9f ? Vec3{0, 1, 0} : Vec3{0, 0, 1};
                const Vec3 tangent = glm::normalize(glm::cross(up, normal));
                const Vec3 bitangent = glm::cross(normal, tangent);
                const Vec3 center = normal * h;
                const Vec3 extentT = tangent * glm::dot(glm::abs(tangent), h);
                const Vec3 extentB = bitangent * glm::dot(glm::abs(bitangent), h);
                const u32 base = static_cast<u32>(mesh.vertices.size());
                pushVertex(mesh, center - extentT - extentB, normal, color);
                pushVertex(mesh, center + extentT - extentB, normal, color);
                pushVertex(mesh, center + extentT + extentB, normal, color);
                pushVertex(mesh, center - extentT + extentB, normal, color);
                pushTriangle(mesh, base, base + 1, base + 2);
                pushTriangle(mesh, base, base + 2, base + 3);
            }
        }

        /// Cone frustum along Z: radius `radiusA` at -Z, `radiusB` at +Z,
        /// half length `halfLength`. Degenerate radii (0) produce a point.
        /// Also serves cylinders (radiusA == radiusB).
        void buildFrustum(MeshData& mesh, f32 radiusA, f32 radiusB, f32 halfLength,
                          u32 segments, const Vec4& color)
        {
            segments = clampSegments(segments);
            const f32 slope = (radiusB - radiusA) / (2.0f * halfLength);

            // Side.
            const u32 sideBase = static_cast<u32>(mesh.vertices.size());
            for (u32 i = 0; i <= segments; ++i)
            {
                const f32 angle = (2.0f * kPi * static_cast<f32>(i)) /
                                  static_cast<f32>(segments);
                const f32 cosA = std::cos(angle);
                const f32 sinA = std::sin(angle);
                const Vec3 normal =
                    glm::normalize(Vec3{cosA, sinA, -slope});
                pushVertex(mesh, {radiusA * cosA, radiusA * sinA, -halfLength}, normal,
                           color);
                pushVertex(mesh, {radiusB * cosA, radiusB * sinA, halfLength}, normal,
                           color);
            }
            for (u32 i = 0; i < segments; ++i)
            {
                const u32 a = sideBase + i * 2;
                // Winding: outward faces counter-clockwise (engine convention).
                pushTriangle(mesh, a, a + 2, a + 1);
                pushTriangle(mesh, a + 1, a + 2, a + 3);
            }

            // Caps.
            const auto buildCap = [&](f32 radius, f32 z, f32 normalZ) {
                if (radius <= 1.0e-5f)
                {
                    return;
                }
                const u32 center = static_cast<u32>(mesh.vertices.size());
                pushVertex(mesh, {0.0f, 0.0f, z}, {0.0f, 0.0f, normalZ}, color);
                for (u32 i = 0; i <= segments; ++i)
                {
                    const f32 angle = (2.0f * kPi * static_cast<f32>(i)) /
                                      static_cast<f32>(segments);
                    pushVertex(mesh, {radius * std::cos(angle), radius * std::sin(angle), z},
                               {0.0f, 0.0f, normalZ}, color);
                }
                for (u32 i = 0; i < segments; ++i)
                {
                    if (normalZ > 0.0f)
                    {
                        pushTriangle(mesh, center, center + 1 + i, center + 2 + i);
                    }
                    else
                    {
                        pushTriangle(mesh, center, center + 2 + i, center + 1 + i);
                    }
                }
            };
            buildCap(radiusA, -halfLength, -1.0f);
            buildCap(radiusB, halfLength, 1.0f);
        }

        /// Ellipsoid with per-axis radii.
        void buildEllipsoid(MeshData& mesh, const Vec3& radii, u32 segments,
                            const Vec4& color)
        {
            segments = clampSegments(segments);
            const u32 rings = std::max(3u, segments / 2);
            const u32 base = static_cast<u32>(mesh.vertices.size());
            for (u32 ring = 0; ring <= rings; ++ring)
            {
                const f32 phi = kPi * static_cast<f32>(ring) / static_cast<f32>(rings);
                const f32 z = std::cos(phi);
                const f32 planar = std::sin(phi);
                for (u32 i = 0; i <= segments; ++i)
                {
                    const f32 angle = (2.0f * kPi * static_cast<f32>(i)) /
                                      static_cast<f32>(segments);
                    const Vec3 unit{planar * std::cos(angle), planar * std::sin(angle), z};
                    const Vec3 position = unit * radii;
                    // Ellipsoid normal: gradient of the implicit surface.
                    const Vec3 normal = glm::normalize(Vec3{
                        unit.x / std::max(radii.x, 1.0e-5f),
                        unit.y / std::max(radii.y, 1.0e-5f),
                        unit.z / std::max(radii.z, 1.0e-5f)});
                    pushVertex(mesh, position, normal, color);
                }
            }
            const u32 stride = segments + 1;
            for (u32 ring = 0; ring < rings; ++ring)
            {
                for (u32 i = 0; i < segments; ++i)
                {
                    const u32 a = base + ring * stride + i;
                    const u32 b = a + stride;
                    // Outward winding (same convention note as makeUvSphere).
                    pushTriangle(mesh, a, b, a + 1);
                    pushTriangle(mesh, a + 1, b, b + 1);
                }
            }
        }

        /// Ring along Z: outer radius, inner radius, half length.
        void buildTube(MeshData& mesh, f32 outer, f32 inner, f32 halfLength, u32 segments,
                       const Vec4& color)
        {
            segments = clampSegments(segments);
            inner = std::clamp(inner, 0.0f, outer - 1.0e-4f);

            const auto ring = [&](f32 radius, f32 normalSign) {
                // Side wall at `radius`; normals point out (+1) or in (-1).
                const u32 base = static_cast<u32>(mesh.vertices.size());
                for (u32 i = 0; i <= segments; ++i)
                {
                    const f32 angle = (2.0f * kPi * static_cast<f32>(i)) /
                                      static_cast<f32>(segments);
                    const Vec3 radialDir{std::cos(angle), std::sin(angle), 0.0f};
                    pushVertex(mesh, radialDir * radius + Vec3{0, 0, -halfLength},
                               radialDir * normalSign, color);
                    pushVertex(mesh, radialDir * radius + Vec3{0, 0, halfLength},
                               radialDir * normalSign, color);
                }
                for (u32 i = 0; i < segments; ++i)
                {
                    const u32 a = base + i * 2;
                    if (normalSign > 0.0f)
                    {
                        pushTriangle(mesh, a, a + 2, a + 1);
                        pushTriangle(mesh, a + 1, a + 2, a + 3);
                    }
                    else
                    {
                        pushTriangle(mesh, a, a + 1, a + 2);
                        pushTriangle(mesh, a + 1, a + 3, a + 2);
                    }
                }
            };
            ring(outer, 1.0f);
            ring(inner, -1.0f);

            // Flat ring caps at +-Z.
            const auto cap = [&](f32 z, f32 normalZ) {
                const u32 base = static_cast<u32>(mesh.vertices.size());
                for (u32 i = 0; i <= segments; ++i)
                {
                    const f32 angle = (2.0f * kPi * static_cast<f32>(i)) /
                                      static_cast<f32>(segments);
                    const Vec3 radialDir{std::cos(angle), std::sin(angle), 0.0f};
                    pushVertex(mesh, radialDir * inner + Vec3{0, 0, z},
                               {0, 0, normalZ}, color);
                    pushVertex(mesh, radialDir * outer + Vec3{0, 0, z},
                               {0, 0, normalZ}, color);
                }
                for (u32 i = 0; i < segments; ++i)
                {
                    const u32 a = base + i * 2;
                    if (normalZ > 0.0f)
                    {
                        pushTriangle(mesh, a, a + 1, a + 2);
                        pushTriangle(mesh, a + 1, a + 3, a + 2);
                    }
                    else
                    {
                        pushTriangle(mesh, a, a + 2, a + 1);
                        pushTriangle(mesh, a + 1, a + 2, a + 3);
                    }
                }
            };
            cap(halfLength, 1.0f);
            cap(-halfLength, -1.0f);
        }

        /// The oriented-box stand-in used by overlap tests.
        [[nodiscard]] Vec3 shapeBoxHalfExtents(const PartShape& shape)
        {
            switch (shape.kind)
            {
            case ShapeKind::Box:
                return shape.size;
            case ShapeKind::Cylinder:
                return {shape.size.x, shape.size.x, shape.size.y};
            case ShapeKind::Cone:
            {
                const f32 radius = std::max(shape.size.x, shape.size.z);
                return {radius, radius, shape.size.y};
            }
            case ShapeKind::Sphere:
                return shape.size;
            case ShapeKind::Tube:
                return {shape.size.x, shape.size.x, shape.size.y};
            }
            return shape.size;
        }

        struct ObbData
        {
            Vec3 center;
            Vec3 halfExtents;
            Mat3 axes; // columns = local axes in the common frame
        };

        /// Standard 15-axis OBB separating-axis test.
        [[nodiscard]] bool obbOverlap(const ObbData& a, const ObbData& b)
        {
            constexpr f32 kEpsilon = 1.0e-5f;
            const Mat3 rotation = glm::transpose(a.axes) * b.axes;
            Mat3 absRotation{};
            for (i32 i = 0; i < 3; ++i)
            {
                for (i32 j = 0; j < 3; ++j)
                {
                    absRotation[i][j] = std::abs(rotation[i][j]) + kEpsilon;
                }
            }
            const Vec3 translation = glm::transpose(a.axes) * (b.center - a.center);

            // A's axes.
            for (i32 i = 0; i < 3; ++i)
            {
                const f32 ra = a.halfExtents[i];
                const f32 rb = b.halfExtents[0] * absRotation[0][i] +
                               b.halfExtents[1] * absRotation[1][i] +
                               b.halfExtents[2] * absRotation[2][i];
                if (std::abs(translation[i]) > ra + rb)
                {
                    return false;
                }
            }
            // B's axes.
            for (i32 i = 0; i < 3; ++i)
            {
                const f32 ra = a.halfExtents[0] * absRotation[i][0] +
                               a.halfExtents[1] * absRotation[i][1] +
                               a.halfExtents[2] * absRotation[i][2];
                const f32 rb = b.halfExtents[i];
                const f32 distance = std::abs(translation[0] * rotation[i][0] +
                                              translation[1] * rotation[i][1] +
                                              translation[2] * rotation[i][2]);
                if (distance > ra + rb)
                {
                    return false;
                }
            }
            // Cross products of axes.
            for (i32 i = 0; i < 3; ++i)
            {
                for (i32 j = 0; j < 3; ++j)
                {
                    const i32 i1 = (i + 1) % 3;
                    const i32 i2 = (i + 2) % 3;
                    const i32 j1 = (j + 1) % 3;
                    const i32 j2 = (j + 2) % 3;
                    const f32 ra = a.halfExtents[i1] * absRotation[j][i2] +
                                   a.halfExtents[i2] * absRotation[j][i1];
                    const f32 rb = b.halfExtents[j1] * absRotation[j2][i] +
                                   b.halfExtents[j2] * absRotation[j1][i];
                    const f32 distance =
                        std::abs(translation[i2] * rotation[j][i1] -
                                 translation[i1] * rotation[j][i2]);
                    if (distance > ra + rb)
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        /// Ray vs finite Z-cylinder (radius, halfLength) at the origin.
        [[nodiscard]] bool rayCylinder(const Vec3& origin, const Vec3& direction,
                                       f32 radius, f32 halfLength, f32& outT,
                                       Vec3& outNormal)
        {
            bool hit = false;
            f32 bestT = 1.0e30f;

            // Side: project on XY.
            const f32 a = direction.x * direction.x + direction.y * direction.y;
            if (a > 1.0e-10f)
            {
                const f32 b = 2.0f * (origin.x * direction.x + origin.y * direction.y);
                const f32 c = origin.x * origin.x + origin.y * origin.y - radius * radius;
                const f32 discriminant = b * b - 4.0f * a * c;
                if (discriminant >= 0.0f)
                {
                    const f32 sqrtDisc = std::sqrt(discriminant);
                    for (const f32 t : {(-b - sqrtDisc) / (2.0f * a),
                                        (-b + sqrtDisc) / (2.0f * a)})
                    {
                        if (t > 0.0f && t < bestT)
                        {
                            const f32 z = origin.z + direction.z * t;
                            if (std::abs(z) <= halfLength)
                            {
                                bestT = t;
                                const Vec3 point = origin + direction * t;
                                outNormal = glm::normalize(Vec3{point.x, point.y, 0.0f});
                                hit = true;
                            }
                        }
                    }
                }
            }
            // Caps.
            if (std::abs(direction.z) > 1.0e-10f)
            {
                for (const f32 capZ : {-halfLength, halfLength})
                {
                    const f32 t = (capZ - origin.z) / direction.z;
                    if (t > 0.0f && t < bestT)
                    {
                        const Vec3 point = origin + direction * t;
                        if (point.x * point.x + point.y * point.y <= radius * radius)
                        {
                            bestT = t;
                            outNormal = {0.0f, 0.0f, capZ > 0.0f ? 1.0f : -1.0f};
                            hit = true;
                        }
                    }
                }
            }
            outT = bestT;
            return hit;
        }

        /// Ray vs axis-aligned box (slab method), local frame.
        [[nodiscard]] bool rayBox(const Vec3& origin, const Vec3& direction, const Vec3& h,
                                  f32& outT, Vec3& outNormal)
        {
            f32 tMin = -1.0e30f;
            f32 tMax = 1.0e30f;
            i32 hitAxis = -1;
            f32 hitSign = 1.0f;
            for (i32 axis = 0; axis < 3; ++axis)
            {
                if (std::abs(direction[axis]) < 1.0e-10f)
                {
                    if (std::abs(origin[axis]) > h[axis])
                    {
                        return false;
                    }
                    continue;
                }
                f32 t1 = (-h[axis] - origin[axis]) / direction[axis];
                f32 t2 = (h[axis] - origin[axis]) / direction[axis];
                f32 sign = -1.0f;
                if (t1 > t2)
                {
                    std::swap(t1, t2);
                    sign = 1.0f;
                }
                if (t1 > tMin)
                {
                    tMin = t1;
                    hitAxis = axis;
                    hitSign = sign;
                }
                tMax = std::min(tMax, t2);
                if (tMin > tMax)
                {
                    return false;
                }
            }
            if (tMin <= 0.0f || hitAxis < 0)
            {
                return false; // inside or behind: not a surface hit we use
            }
            outT = tMin;
            outNormal = Vec3{0.0f};
            outNormal[hitAxis] = hitSign;
            return true;
        }

        /// Ray vs ellipsoid: squash space to the unit sphere.
        [[nodiscard]] bool rayEllipsoid(const Vec3& origin, const Vec3& direction,
                                        const Vec3& radii, f32& outT, Vec3& outNormal)
        {
            const Vec3 safeRadii = glm::max(radii, Vec3{1.0e-5f});
            const Vec3 o = origin / safeRadii;
            const Vec3 d = direction / safeRadii;
            const f32 a = glm::dot(d, d);
            const f32 b = 2.0f * glm::dot(o, d);
            const f32 c = glm::dot(o, o) - 1.0f;
            const f32 discriminant = b * b - 4.0f * a * c;
            if (discriminant < 0.0f)
            {
                return false;
            }
            const f32 t = (-b - std::sqrt(discriminant)) / (2.0f * a);
            if (t <= 0.0f)
            {
                return false;
            }
            outT = t;
            const Vec3 point = origin + direction * t;
            outNormal = glm::normalize(point / (safeRadii * safeRadii));
            return true;
        }
    } // namespace

    Quat shapeRotation(const PartShape& shape)
    {
        return Quat(glm::radians(shape.rotationDeg));
    }

    MeshData buildShapeMesh(const PartShape& shape)
    {
        MeshData local;
        // Emissive convention: alpha in (1, 2] = self-lit (see renderer).
        const Vec4 color{shape.color,
                         shape.emissive > 0.0f ? 1.0f + std::min(shape.emissive, 1.0f)
                                               : 1.0f};
        switch (shape.kind)
        {
        case ShapeKind::Box:
            buildBox(local, shape.size, color);
            break;
        case ShapeKind::Cylinder:
            buildFrustum(local, shape.size.x, shape.size.x, shape.size.y, shape.segments,
                         color);
            break;
        case ShapeKind::Cone:
            buildFrustum(local, shape.size.x, shape.size.z, shape.size.y, shape.segments,
                         color);
            break;
        case ShapeKind::Sphere:
            buildEllipsoid(local, shape.size, shape.segments, color);
            break;
        case ShapeKind::Tube:
            buildTube(local, shape.size.x, shape.size.z, shape.size.y, shape.segments,
                      color);
            break;
        }

        // Apply the shape pose + the material channel (uv.x = specular
        // strength, uv.y = gloss — read by Mesh.frag). Emissive shapes are
        // light sources, not reflectors.
        const Quat rotation = shapeRotation(shape);
        const Vec2 material = shape.emissive > 0.0f
                                  ? Vec2{0.0f, 0.0f}
                                  : Vec2{shape.specular, shape.gloss};
        for (Vertex& vertex : local.vertices)
        {
            vertex.position = rotation * vertex.position + shape.position;
            vertex.normal = rotation * vertex.normal;
            vertex.uv = material;
        }
        return local;
    }

    MeshData buildPartMeshGroup(const PartDefinition& definition, i32 group)
    {
        MeshData mesh;
        for (const PartShape& shape : definition.shapes)
        {
            if (!shape.visible || shape.animation != group)
            {
                continue;
            }
            const MeshData shapeMesh = buildShapeMesh(shape);
            const u32 base = static_cast<u32>(mesh.vertices.size());
            mesh.vertices.insert(mesh.vertices.end(), shapeMesh.vertices.begin(),
                                 shapeMesh.vertices.end());
            for (const u32 index : shapeMesh.indices)
            {
                mesh.indices.push_back(base + index);
            }
        }
        return mesh;
    }

    MeshData buildPartMesh(const PartDefinition& definition)
    {
        MeshData mesh;
        for (const PartShape& shape : definition.shapes)
        {
            if (!shape.visible)
            {
                continue;
            }
            const MeshData shapeMesh = buildShapeMesh(shape);
            const u32 base = static_cast<u32>(mesh.vertices.size());
            mesh.vertices.insert(mesh.vertices.end(), shapeMesh.vertices.begin(),
                                 shapeMesh.vertices.end());
            for (const u32 index : shapeMesh.indices)
            {
                mesh.indices.push_back(base + index);
            }
        }
        return mesh;
    }

    bool hasHitbox(const PartDefinition& definition)
    {
        return !definition.hitboxes.empty();
    }

    std::vector<HitBox> hitboxesFromColliders(const PartDefinition& definition)
    {
        bool anyCollider = false;
        for (const PartShape& shape : definition.shapes)
        {
            anyCollider = anyCollider || shape.collider;
        }

        std::vector<HitBox> boxes;
        for (const PartShape& shape : definition.shapes)
        {
            if (anyCollider ? !shape.collider : !shape.visible)
            {
                continue;
            }
            // The shape's own oriented box, projected onto the PART's axes:
            // an AABB around a rotated primitive, which is what a hitbox is.
            const Vec3 halfExtents = shapeBoxHalfExtents(shape);
            const Mat3 axes = Mat3(shapeRotation(shape));
            const Vec3 extent{
                std::abs(axes[0][0]) * halfExtents.x + std::abs(axes[1][0]) * halfExtents.y +
                    std::abs(axes[2][0]) * halfExtents.z,
                std::abs(axes[0][1]) * halfExtents.x + std::abs(axes[1][1]) * halfExtents.y +
                    std::abs(axes[2][1]) * halfExtents.z,
                std::abs(axes[0][2]) * halfExtents.x + std::abs(axes[1][2]) * halfExtents.y +
                    std::abs(axes[2][2]) * halfExtents.z};
            boxes.push_back({shape.position, extent});
        }
        return boxes;
    }

    std::vector<HitBox> effectiveHull(const PartDefinition& definition)
    {
        return definition.hitboxes.empty() ? hitboxesFromColliders(definition)
                                           : definition.hitboxes;
    }

    void expandPartHullBounds(const PartDefinition& definition, const Vec3& position,
                              const Quat& rotation, Vec3& outMin, Vec3& outMax)
    {
        // THE AUTHORED HULL WINS. A part that declares hitboxes is telling
        // you what it bumps into; only a part that declares none is asking
        // to have it guessed from what it looks like.
        if (!definition.hitboxes.empty())
        {
            const Mat3 axes = Mat3(rotation);
            for (const HitBox& box : definition.hitboxes)
            {
                const Vec3 h = glm::abs(box.halfExtents);
                const Vec3 extent{
                    std::abs(axes[0][0]) * h.x + std::abs(axes[1][0]) * h.y +
                        std::abs(axes[2][0]) * h.z,
                    std::abs(axes[0][1]) * h.x + std::abs(axes[1][1]) * h.y +
                        std::abs(axes[2][1]) * h.z,
                    std::abs(axes[0][2]) * h.x + std::abs(axes[1][2]) * h.y +
                        std::abs(axes[2][2]) * h.z};
                const Vec3 centre = position + rotation * box.center;
                outMin = glm::min(outMin, centre - extent);
                outMax = glm::max(outMax, centre + extent);
            }
            return;
        }

        bool anyCollider = false;
        for (const PartShape& shape : definition.shapes)
        {
            anyCollider = anyCollider || shape.collider;
        }

        for (const PartShape& shape : definition.shapes)
        {
            if (anyCollider ? !shape.collider : !shape.visible)
            {
                continue;
            }
            const Vec3 halfExtents = shapeBoxHalfExtents(shape);
            const Mat3 axes = Mat3(rotation * shapeRotation(shape));
            // The box, projected onto the caller frame's axes: the extent
            // along axis i is the sum over the box's own axes of
            // |axis_j . e_i| * halfExtent_j.
            const Vec3 extent{
                std::abs(axes[0][0]) * halfExtents.x + std::abs(axes[1][0]) * halfExtents.y +
                    std::abs(axes[2][0]) * halfExtents.z,
                std::abs(axes[0][1]) * halfExtents.x + std::abs(axes[1][1]) * halfExtents.y +
                    std::abs(axes[2][1]) * halfExtents.z,
                std::abs(axes[0][2]) * halfExtents.x + std::abs(axes[1][2]) * halfExtents.y +
                    std::abs(axes[2][2]) * halfExtents.z};
            const Vec3 centre = position + rotation * shape.position;
            outMin = glm::min(outMin, centre - extent);
            outMax = glm::max(outMax, centre + extent);
        }
    }

    f32 partBoundsRadius(const PartDefinition& definition)
    {
        f32 radius = 0.5f;
        for (const PartShape& shape : definition.shapes)
        {
            const Vec3 halfExtents = shapeBoxHalfExtents(shape);
            radius = std::max(radius,
                              glm::length(shape.position) + glm::length(halfExtents));
        }
        // A hull may legitimately stick out past the geometry — a hitbox
        // around a landing gear's sweep, say — and a culling sphere that did
        // not contain it would pop the part out of view while it was still
        // being stood on.
        for (const HitBox& box : definition.hitboxes)
        {
            radius = std::max(radius, glm::length(box.center) +
                                          glm::length(glm::abs(box.halfExtents)));
        }
        // AND THE DEPLOYED POSE. A solar wing folded against the hull is a
        // metre across and four metres across with its arrays out; a radius
        // measured on the stowed pose alone would cull the part the moment the
        // player looked slightly away from a deployed panel, which is the one
        // moment they are looking AT it.
        for (const PartShape& shape : definition.shapes)
        {
            if (shape.animation < 0)
            {
                continue;
            }
            const Vec3 halfExtents = shapeBoxHalfExtents(shape);
            radius = std::max(radius,
                              glm::length(shape.endPosition) + glm::length(halfExtents));
        }
        return radius;
    }

    f32 rayEntersSphere(const Vec3& toCentre, const Vec3& direction, f32 radius)
    {
        const f32 alongAxis = glm::dot(toCentre, direction);
        const f32 centreDistanceSq = glm::dot(toCentre, toCentre);
        if (centreDistanceSq <= radius * radius)
        {
            return 0.0f; // the eye is already inside it
        }
        if (alongAxis <= 0.0f)
        {
            return -1.0f; // wholly behind
        }
        const f32 offAxisSq = centreDistanceSq - alongAxis * alongAxis;
        const f32 radiusSq = radius * radius;
        if (offAxisSq > radiusSq)
        {
            return -1.0f;
        }
        return alongAxis - std::sqrt(radiusSq - offAxisSq);
    }

    bool raycastPart(const PartDefinition& definition, const Vec3& origin,
                     const Vec3& direction, f32 maxDistance, PartRayHit& outHit)
    {
        bool anyCollider = false;
        for (const PartShape& shape : definition.shapes)
        {
            if (shape.collider)
            {
                anyCollider = true;
                break;
            }
        }

        bool hit = false;
        outHit.t = maxDistance;
        for (usize index = 0; index < definition.shapes.size(); ++index)
        {
            const PartShape& shape = definition.shapes[index];
            if (anyCollider ? !shape.collider : !shape.visible)
            {
                continue;
            }
            // Into the shape's local frame.
            const Quat rotation = shapeRotation(shape);
            const Quat inverse = glm::inverse(rotation);
            const Vec3 localOrigin = inverse * (origin - shape.position);
            const Vec3 localDirection = inverse * direction;

            f32 t = 0.0f;
            Vec3 normal{0.0f, 0.0f, 1.0f};
            bool shapeHit = false;
            switch (shape.kind)
            {
            case ShapeKind::Box:
                shapeHit = rayBox(localOrigin, localDirection, shape.size, t, normal);
                break;
            case ShapeKind::Cylinder:
                shapeHit = rayCylinder(localOrigin, localDirection, shape.size.x,
                                       shape.size.y, t, normal);
                break;
            case ShapeKind::Cone:
                shapeHit = rayCylinder(localOrigin, localDirection,
                                       std::max(shape.size.x, shape.size.z),
                                       shape.size.y, t, normal);
                break;
            case ShapeKind::Sphere:
                shapeHit = rayEllipsoid(localOrigin, localDirection, shape.size, t, normal);
                break;
            case ShapeKind::Tube:
                shapeHit = rayCylinder(localOrigin, localDirection, shape.size.x,
                                       shape.size.y, t, normal);
                break;
            }
            if (shapeHit && t < outHit.t)
            {
                outHit.t = t;
                outHit.normal = rotation * normal;
                outHit.shapeIndex = static_cast<i32>(index);
                hit = true;
            }
        }
        return hit;
    }

    bool partsOverlap(const PartDefinition& definitionA, const Vec3& positionA,
                      const Quat& rotationA, const PartDefinition& definitionB,
                      const Vec3& positionB, const Quat& rotationB, f32 margin)
    {
        const auto collectBoxes = [margin](const PartDefinition& definition,
                                           const Vec3& position, const Quat& rotation,
                                           std::vector<ObbData>& out) {
            // The authored hull first, same rule as the bounds: a part that
            // says what it bumps into is not second-guessed.
            if (!definition.hitboxes.empty())
            {
                for (const HitBox& hit : definition.hitboxes)
                {
                    ObbData box{};
                    box.center = position + rotation * hit.center;
                    box.halfExtents =
                        glm::max(glm::abs(hit.halfExtents) - Vec3{margin}, Vec3{0.01f});
                    box.axes = glm::mat3_cast(rotation);
                    out.push_back(box);
                }
                return;
            }
            bool anyCollider = false;
            for (const PartShape& shape : definition.shapes)
            {
                anyCollider = anyCollider || shape.collider;
            }
            for (const PartShape& shape : definition.shapes)
            {
                if (anyCollider ? !shape.collider : !shape.visible)
                {
                    continue;
                }
                ObbData box{};
                const Quat worldRotation = rotation * shapeRotation(shape);
                box.center = position + rotation * shape.position;
                box.halfExtents =
                    glm::max(shapeBoxHalfExtents(shape) - Vec3{margin}, Vec3{0.01f});
                box.axes = glm::mat3_cast(worldRotation);
                out.push_back(box);
            }
        };

        std::vector<ObbData> boxesA;
        std::vector<ObbData> boxesB;
        collectBoxes(definitionA, positionA, rotationA, boxesA);
        collectBoxes(definitionB, positionB, rotationB, boxesB);
        for (const ObbData& a : boxesA)
        {
            for (const ObbData& b : boxesB)
            {
                if (obbOverlap(a, b))
                {
                    return true;
                }
            }
        }
        return false;
    }
} // namespace sw::parts
