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
#include "Gameplay/Fairing.hpp"
#include "Gameplay/Parts.hpp"

#include <span>

namespace sw::parts
{
    /// Shape pose rotation from the authored euler angles (degrees, XYZ).
    [[nodiscard]] Quat shapeRotation(const PartShape& shape);

    /// Triangulates ONE shape (pose applied). Round shapes honor
    /// shape.segments (clamped to [3, 64]).
    [[nodiscard]] MeshData buildShapeMesh(const PartShape& shape);

    /// Triangulates every VISIBLE shape of the definition into one mesh.
    /// One mesh for one ANIMATION GROUP of a part.
    ///
    /// `group` is -1 for the shapes that never move — the body of the part,
    /// which is what almost every part is entirely made of — and otherwise the
    /// index of an animation, collecting the shapes that animation moves.
    ///
    /// A part used to be ONE welded mesh with every shape's pose baked into
    /// its vertices, uploaded once at boot and shared by every instance. That
    /// is still exactly what the static group is, and it is why animation had
    /// to be a splitting rather than a rewrite: the vertices of a moving group
    /// are baked at its REST pose, the group is drawn with an extra transform
    /// in front of the part's own, and nothing about the vertex format, the
    /// draw item or the shader had to learn what an animation is.
    [[nodiscard]] MeshData buildPartMeshGroup(const PartDefinition& definition, i32 group);

    // ------------------------------------------------------------------------
    // ONE MESH PER MOTION, NOT ONE PER ANIMATION
    //
    // An animation used to be a single rigid body: every shape it moved was
    // welded into one mesh and carried by the hinge of the FIRST of them.
    // That is right for a panel and the struts holding it, and wrong the
    // moment an author gives one animation two motions — a telescoping array
    // whose four segments deploy to four different places, say. The editor
    // previews each shape on its own hinge, so it looked perfect there and
    // came out in game as one segment with three stowaways.
    //
    // So the grouping is DERIVED rather than declared: shapes belong to the
    // same motion when the same animation drives them AND the rigid transform
    // from their rest pose to their deployed pose is the same. A panel and its
    // struts still weld into one mesh and one draw call, because they really
    // do share a hinge; four segments going four places become four, because
    // they really are four motions. Nothing about the format changed and no
    // part had to be re-authored — the old solar wing's five shapes still
    // group into exactly one motion.
    // ------------------------------------------------------------------------
    struct PartMotionGroup
    {
        i32 animation = -1;   // which animation drives the phase
        u32 driver = 0;       // index into definition.shapes: the pose to follow
        std::vector<u32> shapes;
    };

    /// Every distinct rigid motion in the definition, in shape order. Empty
    /// for a part that animates nothing.
    [[nodiscard]] std::vector<PartMotionGroup> partMotionGroups(
        const PartDefinition& definition);

    /// The mesh for one motion, its vertices baked at the REST pose exactly
    /// as buildPartMeshGroup bakes an animation's.
    [[nodiscard]] MeshData buildPartMotionMesh(const PartDefinition& definition,
                                               const PartMotionGroup& motion);

    /// Every visible shape, whatever it animates. Kept for the tools and for
    /// anything that wants a part's whole silhouette in one buffer.
    [[nodiscard]] MeshData buildPartMesh(const PartDefinition& definition);

    /// Conservative bounding-sphere radius around the part origin
    /// (all shapes, visible or not).
    [[nodiscard]] f32 partBoundsRadius(const PartDefinition& definition);

    /// THE SHELL THE PLAYER DREW, lathed into flat panels.
    ///
    /// One quad per (band, side), plus a fan closing the nose when the last
    /// ring has a radius. `side` selects ONE panel column instead of the whole
    /// shell — which is what a jettisoned fairing breaks into, and the reason
    /// the two share a builder rather than agreeing by hand about where the
    /// seams are.
    [[nodiscard]] MeshData buildFairingMesh(const FairingComponent& fairing,
                                            const Vec4& color);
    [[nodiscard]] MeshData buildFairingPanelMesh(const FairingComponent& fairing,
                                                 u32 side, const Vec4& color);

    /// True when the definition AUTHORS its collision hull as hitboxes.
    /// False means "derive it from the collider shapes", which is what every
    /// part did before hitboxes existed and what any .swpart written without
    /// them still does.
    [[nodiscard]] bool hasHitbox(const PartDefinition& definition);

    /// One box per COLLIDER shape, each the axis-aligned bounds of that
    /// shape in the part's own frame — a starting hull good enough to edit
    /// from rather than to draw from scratch. Part Studio's FIT button and
    /// the offline seeding of the shipped catalogue are the same call, so a
    /// generated hull and an authored one are the same kind of object.
    [[nodiscard]] std::vector<HitBox> hitboxesFromColliders(
        const PartDefinition& definition);

    /// The boxes a part actually collides with: its authored hitboxes if it
    /// has any, the ones its collider shapes imply otherwise. One call, so
    /// no caller has to remember the fallback rule.
    [[nodiscard]] std::vector<HitBox> effectiveHull(const PartDefinition& definition);

    /// Axis-aligned bounds of the part's HULL — its hitboxes if it has any,
    /// its collider shapes otherwise — posed at
    /// `position`/`rotation` and projected onto the axes of THAT frame
    /// (vessel space, typically). Each collider shape contributes the same
    /// oriented box the overlap test uses — so what a vessel RESTS on is
    /// derived from what it COLLIDES with, not from a second description
    /// that could drift.
    ///
    /// `outMin`/`outMax` are EXPANDED, never reset: call it once per part
    /// over a running box. Falls back to the visible shapes when a part has
    /// no collider at all.
    void expandPartHullBounds(const PartDefinition& definition, const Vec3& position,
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

    /// How far along a unit ray from the origin it first enters a sphere of
    /// `radius` whose centre is at `toCentre`, or -1 if it never does. A ray
    /// starting INSIDE the sphere returns 0 — it is already there.
    ///
    /// THIS IS WHAT PICKING A PART TO OPERATE USES, and `raycastPart` above is
    /// not, which is worth writing down because the exact one is the obvious
    /// choice and it is wrong twice over.
    ///
    /// It is wrong once because it tests the part's COLLIDER boxes, which sit
    /// at the shapes' REST poses — so a solar wing that is drawn swung out has
    /// its collider still folded against the hull, and clicking the panel you
    /// can see misses it entirely while clicking empty space beside the tank
    /// hits it.
    ///
    /// It is wrong twice because exact is the wrong AMBITION. A wing seen
    /// edge-on from twenty metres is two pixels of collider; a switch that
    /// demands two pixels is a switch nobody can work. The bounding sphere
    /// already covers the deployed pose (see partBoundsRadius) and turns the
    /// same click into a couple of degrees of tolerance.
    [[nodiscard]] f32 rayEntersSphere(const Vec3& toCentre, const Vec3& direction,
                                      f32 radius);

    /// Compound HULL overlap between two POSED parts (positions in the
    /// same frame, e.g. vessel-local). Each hull box is tested as an
    /// oriented box (separating-axis theorem); `margin` shrinks every box
    /// on all sides — pass a small positive margin so flush stack contact
    /// does not read as a collision.
    [[nodiscard]] bool partsOverlap(const PartDefinition& definitionA, const Vec3& positionA,
                                    const Quat& rotationA, const PartDefinition& definitionB,
                                    const Vec3& positionB, const Quat& rotationB, f32 margin);

    // ------------------------------------------------------------------------
    // WHETHER A PLACEMENT FITS
    //
    // The editor's red/green light, as a function rather than as a hundred
    // lines inside the ghost. It is here and not in the game because it was
    // WRONG in a way only a test could have caught: a radial part was tested
    // for overlap against the very part it was being bolted to, and the hull
    // it was tested against is a SQUARE prism circumscribing a round tank. So
    // a battery glued anywhere except the four compass points was inside its
    // parent's corner, and read red. Measured on the shipped catalogue: 28% of
    // azimuths legal at symmetry 1, 2 and 4 — and NONE AT ALL at 3, 6 and 8,
    // because those put a clone at 120°, 60° or 45°, which is exactly where
    // the square's corners are.
    //
    // A joint IS contact. Asking whether a bolt is inside the plate it is
    // bolted to is not a question with a useful answer, so the part being
    // attached to is not something the placement can collide with.
    // ------------------------------------------------------------------------

    /// One piece of a judgement: a definition and its pose in the design frame.
    struct PlacementPiece
    {
        const PartDefinition* definition = nullptr;
        Vec3 position{0.0f};
        Quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    };

    /// The k-th radial-symmetry copy of a pose, turned about the design's own
    /// axis (+Z). One place, because the editor needs the same arithmetic to
    /// judge a placement, to commit it and to draw it, and three copies of it
    /// is three chances to disagree.
    [[nodiscard]] PlacementPiece symmetryClone(const PlacementPiece& piece, u32 index,
                                               u32 count);

    /// Does this placement (the held part, its grabbed subtree and every
    /// symmetry copy) intersect anything already on the design? `attachedTo`
    /// holds the indices in `placed` of the parts it is being bolted to —
    /// those are skipped, because a joint means they touch. There is more
    /// than one because a ring of boosters lands on a ring of decouplers:
    /// every copy has its own parent, and excluding only the one under the
    /// cursor makes every other copy read as a collision with the very thing
    /// it is being mounted on.
    /// `outBlockedBy`, when given, receives the index of the first placed
    /// part that stood in the way — which is the one thing a red ghost cannot
    /// tell you and the first thing you want to know.
    [[nodiscard]] bool placementCollides(std::span<const PlacementPiece> candidates,
                                         std::span<const PlacementPiece> placed,
                                         std::span<const i32> attachedTo, f32 margin,
                                         i32* outBlockedBy = nullptr);
} // namespace sw::parts
