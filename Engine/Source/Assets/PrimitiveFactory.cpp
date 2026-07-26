#include "Assets/PrimitiveFactory.hpp"

#include <cmath>

namespace sw
{
    MeshData PrimitiveFactory::makeCube(f32 size, const Vec4& color)
    {
        return makeBox(Vec3{size * 0.5f}, color);
    }

    MeshData PrimitiveFactory::makeBox(const Vec3& halfExtents, const Vec4& color)
    {
        const f32 hx = halfExtents.x;
        const f32 hy = halfExtents.y;
        const f32 hz = halfExtents.z;

        // 6 faces * 4 vertices, flat normals; CCW when viewed from outside.
        struct Face
        {
            Vec3 normal;
            Vec3 corners[4]; // CCW order
        };
        const Face faces[6] = {
            {{0, 0, 1},  {{-hx, -hy, hz}, {hx, -hy, hz}, {hx, hy, hz}, {-hx, hy, hz}}},     // +Z
            {{0, 0, -1}, {{hx, -hy, -hz}, {-hx, -hy, -hz}, {-hx, hy, -hz}, {hx, hy, -hz}}}, // -Z
            {{1, 0, 0},  {{hx, -hy, hz}, {hx, -hy, -hz}, {hx, hy, -hz}, {hx, hy, hz}}},     // +X
            {{-1, 0, 0}, {{-hx, -hy, -hz}, {-hx, -hy, hz}, {-hx, hy, hz}, {-hx, hy, -hz}}}, // -X
            {{0, 1, 0},  {{-hx, hy, hz}, {hx, hy, hz}, {hx, hy, -hz}, {-hx, hy, -hz}}},     // +Y
            {{0, -1, 0}, {{-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, -hy, hz}, {-hx, -hy, hz}}}, // -Y
        };
        const Vec2 uvs[4] = {{0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}, {0.0f, 0.0f}};

        MeshData mesh;
        mesh.vertices.reserve(24);
        mesh.indices.reserve(36);
        for (const Face& face : faces)
        {
            const u32 base = static_cast<u32>(mesh.vertices.size());
            for (u32 i = 0; i < 4; ++i)
            {
                mesh.vertices.push_back({face.corners[i], face.normal, color, uvs[i]});
            }
            mesh.indices.insert(mesh.indices.end(),
                                {base, base + 1, base + 2, base, base + 2, base + 3});
        }
        return mesh;
    }

    MeshData PrimitiveFactory::makeOctahedron(f32 radius, const Vec4& color)
    {
        MeshData mesh;
        const Vec3 vertices[6] = {
            {radius, 0, 0},  {-radius, 0, 0}, {0, radius, 0},
            {0, -radius, 0}, {0, 0, radius},  {0, 0, -radius},
        };
        mesh.vertices.reserve(6);
        for (const Vec3& position : vertices)
        {
            mesh.vertices.push_back({position, glm::normalize(position), color, {0, 0}});
        }
        // 8 faces, CCW from outside. Vertex order: +X -X +Y -Y +Z -Z.
        mesh.indices = {
            2, 4, 0, 2, 0, 5, 2, 5, 1, 2, 1, 4, // top pyramid
            3, 0, 4, 3, 5, 0, 3, 1, 5, 3, 4, 1, // bottom pyramid
        };
        return mesh;
    }

    void PrimitiveFactory::append(MeshData& target, const MeshData& source, const Vec3& offset)
    {
        const u32 base = static_cast<u32>(target.vertices.size());
        target.vertices.reserve(target.vertices.size() + source.vertices.size());
        for (Vertex vertex : source.vertices)
        {
            vertex.position += offset;
            target.vertices.push_back(vertex);
        }
        target.indices.reserve(target.indices.size() + source.indices.size());
        for (const u32 index : source.indices)
        {
            target.indices.push_back(base + index);
        }
    }

    MeshData PrimitiveFactory::makeUvSphere(f32 radius, u32 rings, u32 segments,
                                            const Vec4& color)
    {
        MeshData mesh;
        mesh.vertices.reserve(static_cast<usize>(rings + 1) * (segments + 1));

        for (u32 ring = 0; ring <= rings; ++ring)
        {
            const f32 v = static_cast<f32>(ring) / static_cast<f32>(rings);
            const f32 phi = v * math::kPi; // 0 (north pole) .. pi (south pole)
            const f32 sinPhi = std::sin(phi);
            const f32 cosPhi = std::cos(phi);

            for (u32 segment = 0; segment <= segments; ++segment)
            {
                const f32 u = static_cast<f32>(segment) / static_cast<f32>(segments);
                const f32 theta = u * math::kTwoPi;

                const Vec3 normal{sinPhi * std::cos(theta), cosPhi,
                                  sinPhi * std::sin(theta)};
                mesh.vertices.push_back({normal * radius, normal, color, {u, v}});
            }
        }

        for (u32 ring = 0; ring < rings; ++ring)
        {
            for (u32 segment = 0; segment < segments; ++segment)
            {
                const u32 a = ring * (segments + 1) + segment;
                const u32 b = a + segments + 1;
                // OUTWARD winding (right-hand rule normal = radial). The
                // engine's front-face reference is the cube: the cross
                // product of the listed edges must point OUT. The previous
                // order was inverted — every UV sphere rendered its
                // INTERIOR (from orbit you could see BOTH polar caps at
                // once: the M15 "pole glitch", finally explained).
                mesh.indices.insert(mesh.indices.end(), {a, a + 1, b, a + 1, b + 1, b});
            }
        }
        return mesh;
    }

    MeshData PrimitiveFactory::makeCapsule(f32 radius, f32 cylinderHalfHeight, u32 rings,
                                           u32 segments, const Vec4& color)
    {
        // Start from a UV sphere, then push the upper/lower hemispheres
        // apart along Y; normals are recomputed against the shifted centers
        // so the cylindrical band shades correctly.
        MeshData mesh = makeUvSphere(radius, rings, segments, color);
        for (Vertex& vertex : mesh.vertices)
        {
            const f32 offset = (vertex.position.y >= 0.0f) ? cylinderHalfHeight
                                                           : -cylinderHalfHeight;
            vertex.position.y += offset;
            const Vec3 capCenter{0.0f, offset, 0.0f};
            vertex.normal = glm::normalize(vertex.position - capCenter);
        }
        return mesh;
    }

    MeshData PrimitiveFactory::makeGridPlane(f32 extent, u32 cells, const Vec4& color)
    {
        MeshData mesh;
        const u32 verts = cells + 1;
        mesh.vertices.reserve(static_cast<usize>(verts) * verts);

        for (u32 z = 0; z < verts; ++z)
        {
            for (u32 x = 0; x < verts; ++x)
            {
                const f32 fx = (static_cast<f32>(x) / cells - 0.5f) * 2.0f * extent;
                const f32 fz = (static_cast<f32>(z) / cells - 0.5f) * 2.0f * extent;
                mesh.vertices.push_back({{fx, 0.0f, fz},
                                         {0.0f, 1.0f, 0.0f},
                                         color,
                                         {static_cast<f32>(x), static_cast<f32>(z)}});
            }
        }
        for (u32 z = 0; z < cells; ++z)
        {
            for (u32 x = 0; x < cells; ++x)
            {
                const u32 a = z * verts + x;
                const u32 b = a + verts;
                mesh.indices.insert(mesh.indices.end(), {a, b, a + 1, a + 1, b, b + 1});
            }
        }
        return mesh;
    }
} // namespace sw
