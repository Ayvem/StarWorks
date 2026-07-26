#pragma once

// ============================================================================
// Assets/PrimitiveFactory.hpp
// Procedural generation of basic meshes. Used for engine testing, debug
// visualization, and as placeholder geometry until real assets exist.
// All primitives are indexed, counter-clockwise (engine convention), with
// correct normals.
// ============================================================================

#include "Assets/MeshData.hpp"

namespace sw
{
    class PrimitiveFactory
    {
    public:
        PrimitiveFactory() = delete;

        /// Axis-aligned cube centered at the origin (24 vertices, flat faces).
        [[nodiscard]] static MeshData makeCube(f32 size, const Vec4& color);

        /// Axis-aligned box centered at the origin with per-axis half extents.
        [[nodiscard]] static MeshData makeBox(const Vec3& halfExtents, const Vec4& color);

        /// Octahedron (6 vertices, 8 faces) — cheap marker/beacon shape.
        [[nodiscard]] static MeshData makeOctahedron(f32 radius, const Vec4& color);

        /// Appends `source` into `target`, translated by `offset` (indices
        /// are rebased). Used to assemble compound meshes (e.g. ships) from
        /// primitive parts before GPU upload.
        static void append(MeshData& target, const MeshData& source, const Vec3& offset);

        /// UV sphere centered at the origin.
        [[nodiscard]] static MeshData makeUvSphere(f32 radius, u32 rings, u32 segments,
                                                   const Vec4& color);

        /// Vertical capsule centered at the origin: cylinder of half-height
        /// `cylinderHalfHeight` with hemispherical caps of `radius`.
        [[nodiscard]] static MeshData makeCapsule(f32 radius, f32 cylinderHalfHeight,
                                                  u32 rings, u32 segments, const Vec4& color);

        /// Flat grid on the XZ plane (for spatial orientation while flying).
        [[nodiscard]] static MeshData makeGridPlane(f32 extent, u32 cells, const Vec4& color);
    };
} // namespace sw
