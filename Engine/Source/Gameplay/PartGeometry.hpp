#pragma once

// ============================================================================
// Gameplay/PartGeometry.hpp
// Geometry services for composed-primitive parts: mesh generation for the
// renderer, ray casting and overlap tests against the COLLIDER shapes.
//
// One source of truth: the same PartShape list drives what you SEE (visible
// shapes -> triangles, per-vertex colors) and what you TOUCH (collider
// shapes -> ray hits for mouse picking / radial surface attach, OBB
// overlap for placement validation). This is what fixes "nodes inside the
// part": Part Studio authors nodes ON these collider surfaces and the VAB
// validates against exactly the same data.
//
// Conventions: everything here works in the PART-LOCAL frame (callers
// transform rays/poses); -Z is the stack nose.
// ============================================================================

#include "Assets/MeshData.hpp"
#include "Gameplay/Parts.hpp"

namespace sw::parts
{
    /// Shape pose rotation from the authored euler angles (degrees, XYZ).
    [[nodiscard]] Quat shapeRotation(const PartShape& shape);

    /// Triangulates ONE shape (pose applied). Round shapes honor
    /// shape.segments (clamped to [3, 64]).
    [[nodiscard]] MeshData buildShapeMesh(const PartShape& shape);

    /// Triangulates every VISIBLE shape of the definition into one mesh.
    [[nodiscard]] MeshData buildPartMesh(const PartDefinition& definition);

    /// Conservative bounding-sphere radius around the part origin
    /// (all shapes, visible or not).
    [[nodiscard]] f32 partBoundsRadius(const PartDefinition& definition);

    /// Axis-aligned bounds of the part's COLLIDER shapes, posed at
    /// `position`/`rotation` and projected onto the axes of THAT frame
    /// (vessel space, typically). Each collider shape contributes the same
    /// oriented box the overlap test uses — so what a vessel RESTS on is
    /// derived from what it COLLIDES with, not from a second description
    /// that could drift.
    ///
    /// `outMin`/`outMax` are EXPANDED, never reset: call it once per part
    /// over a running box. Falls back to the visible shapes when a part has
    /// no collider at all.
    void expandPartColliderBounds(const PartDefinition& definition, const Vec3& position,
                                  const Quat& rotation, Vec3& outMin, Vec3& outMax);

    struct PartRayHit
    {
        f32 t = 0.0f;          // distance along the (normalized) ray
        Vec3 normal{0.0f, 0.0f, 1.0f}; // part-local surface normal
        i32 shapeIndex = -1;   // which shape was struck
    };

    /// Ray vs the part's COLLIDER shapes (falls back to visible shapes when
    /// the part has no collider — never authored, but never crash).
    /// `origin`/`direction` in part-local space, direction normalized.
    /// Round shapes use their exact quadric where cheap (sphere, cylinder
    /// side/caps); cones and tubes are treated as their bounding cylinder.
    [[nodiscard]] bool raycastPart(const PartDefinition& definition, const Vec3& origin,
                                   const Vec3& direction, f32 maxDistance,
                                   PartRayHit& outHit);

    /// Compound collider overlap between two POSED parts (positions in the
    /// same frame, e.g. vessel-local). Each collider shape is tested as an
    /// oriented box (separating-axis theorem); `margin` shrinks every box
    /// on all sides — pass a small positive margin so flush stack contact
    /// does not read as a collision.
    [[nodiscard]] bool partsOverlap(const PartDefinition& definitionA, const Vec3& positionA,
                                    const Quat& rotationA, const PartDefinition& definitionB,
                                    const Vec3& positionB, const Quat& rotationB, f32 margin);
} // namespace sw::parts
