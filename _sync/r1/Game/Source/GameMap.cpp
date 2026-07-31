// ============================================================================
// GameMap.cpp — The map view: trajectories, markers, and the per-frame draw item list.
// Split out of StarWorksGame.cpp; same class, one theme per translation unit.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"
#include "Systems.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <format>
#include <limits>

namespace game
{

    void StarWorksGame::collectMapTrajectories(const sw::Camera& activeCamera)
    {
        const sw::WorldVec3 cameraPosition = activeCamera.position();
        const sw::f32 markerFactor =
            kMarkerScreenFraction * 2.0f * std::tan(activeCamera.verticalFov() * 0.5f);
        const sw::f64 time = m_physicsLane->presentSeconds();

        // One dot of the dotted trajectory (emissive: tint alpha 2.0).
        auto plotDot = [&](const sw::WorldVec3& point, const sw::Vec4& color,
                           sw::f32 sizeMultiplier) {
            const sw::Vec3 relative = sw::Vec3(point - cameraPosition);
            const sw::f32 scale =
                glm::length(relative) * markerFactor * 0.22f * sizeMultiplier;
            sw::DrawItem item{};
            item.mesh = &m_meshes[m_markerMeshIndex];
            item.transform = glm::translate(sw::Mat4{1.0f}, relative) *
                             glm::scale(sw::Mat4{1.0f}, sw::Vec3{scale});
            item.boundsCenter = relative;
            item.boundsRadius = scale;
            item.tint = {color.r, color.g, color.b, 2.0f};
            m_drawItems.push_back(item);
        };

        // ONE STRAIGHT PIECE OF LINE between two world points.
        //
        // A dotted trajectory hides the one thing the map is for: you
        // cannot tell a plan that ENDS from a plan whose dots have spread
        // out, and at map zoom they always spread out. A stretched box
        // between consecutive samples costs the same draw item and gives a
        // line whose end means something.
        // A LINE IS A BOX, AND A BOX HAS ONE WIDTH.
        //
        // That is the whole difficulty. The width has to follow distance, or
        // the line changes thickness as you zoom; but ONE box spanning a
        // chord of Terra's own orbit is four million kilometres long, and
        // when the camera sits on that orbit — which it does, because the
        // camera is at Terra — one end of the box is billions of kilometres
        // away and the other end is in your eye. Sized for the far end it is
        // three thousand kilometres wide where it passes you, which is the
        // grey band across the planet.
        //
        // So a piece is SPLIT until its near and far ends are within a
        // factor of two of each other, and each piece is then sized from its
        // own closest approach. Almost every chord passes first try; only the
        // handful actually near the camera subdivide, so the cost is a few
        // extra boxes rather than a uniformly denser line.
        auto plotLine = [&](const sw::WorldVec3& a, const sw::WorldVec3& b,
                            const sw::Vec4& color, sw::f32 widthMultiplier) {
            struct Piece
            {
                sw::Vec3 a;
                sw::Vec3 b;
                sw::u32 depth;
            };
            constexpr sw::u32 kMaxSplitDepth = 5; // at most 32 pieces per chord
            Piece pending[2 * kMaxSplitDepth + 2];
            sw::u32 count = 0;
            pending[count++] = {sw::Vec3(a - cameraPosition),
                                sw::Vec3(b - cameraPosition), 0u};

            while (count > 0)
            {
                const Piece piece = pending[--count];
                const sw::Vec3 delta = piece.b - piece.a;
                const sw::f32 length = glm::length(delta);
                if (!(length > 1.0e-4f))
                {
                    continue;
                }

                // Closest approach of THIS piece to the eye, and its far end.
                const sw::f32 t = glm::clamp(
                    -glm::dot(piece.a, delta) / glm::dot(delta, delta), 0.0f, 1.0f);
                // (Not called `near`/`far`: those are macros in the Windows
                // headers this also builds against.)
                const sw::f32 closest = glm::length(piece.a + delta * t);
                const sw::f32 furthest =
                    std::max(glm::length(piece.a), glm::length(piece.b));
                if (piece.depth < kMaxSplitDepth &&
                    furthest > 2.0f * std::max(closest, 1.0f) &&
                    count + 2u <= static_cast<sw::u32>(std::size(pending)))
                {
                    const sw::Vec3 middle = piece.a + delta * 0.5f;
                    pending[count++] = {piece.a, middle, piece.depth + 1};
                    pending[count++] = {middle, piece.b, piece.depth + 1};
                    continue;
                }

                // 0.10 of the marker's own screen fraction is about 1.7
                // pixels at 1080p — a line, not a hairline that aliases into
                // dashes. Measured at the CLOSEST point, which is where a
                // width that is wrong is most obvious.
                const sw::f32 width = closest * markerFactor * 0.10f * widthMultiplier;
                const sw::Vec3 centre = (piece.a + piece.b) * 0.5f;
                const sw::Vec3 forward = delta / length;
                const sw::Vec3 reference = (std::abs(forward.y) < 0.99f)
                                               ? sw::Vec3{0.0f, 1.0f, 0.0f}
                                               : sw::Vec3{1.0f, 0.0f, 0.0f};
                const sw::Vec3 right = glm::normalize(glm::cross(reference, forward));
                const sw::Vec3 up = glm::cross(forward, right);

                sw::DrawItem item{};
                item.mesh = &m_meshes[m_orbitLineMeshIndex];
                item.transform =
                    glm::translate(sw::Mat4{1.0f}, centre) *
                    glm::mat4_cast(glm::quat_cast(sw::Mat3{right, up, forward})) *
                    // A hair longer than the gap so consecutive pieces
                    // overlap rather than leaving a seam on a tight curve.
                    glm::scale(sw::Mat4{1.0f}, sw::Vec3{width, width, length * 1.02f});
                item.boundsCenter = centre;
                item.boundsRadius = length * 0.51f + width;
                item.tint = {color.r, color.g, color.b, 2.0f};
                m_drawItems.push_back(item);
            }
        };

        // Samples a conic around a primary's CURRENT world position over
        // [t0, t1] — KSP map convention: patches are drawn in the frame of
        // where their primary is NOW — and joins the samples up.
        auto plotConic = [&](const sw::phys::KeplerOrbit& orbit,
                             const sw::WorldVec3& primaryPosition, const sw::Vec4& color,
                             sw::f64 t0, sw::f64 t1, sw::u32 samples,
                             sw::f32 widthMultiplier) {
            sw::WorldVec3 previous{};
            bool havePrevious = false;
            for (sw::u32 sample = 0; sample <= samples; ++sample)
            {
                // Spaced by ANOMALY, not by time — see the note on
                // kepler::timeAtArcFraction. Samples spaced evenly in time
                // leave one chord cutting thousands of kilometres off the
                // periapsis of a transfer ellipse: a line drawn straight
                // through the planet the arc goes round.
                const sw::f64 ts = sw::phys::kepler::timeAtArcFraction(
                    orbit, t0, t1, static_cast<sw::f64>(sample) / samples);
                sw::WorldVec3 relativePoint{};
                sw::phys::kepler::evaluate(orbit, ts, relativePoint);
                const sw::WorldVec3 point = primaryPosition + relativePoint;
                if (havePrevious)
                {
                    plotLine(previous, point, color, widthMultiplier);
                }
                previous = point;
                havePrevious = true;
            }
        };

        // Full closed orbit (elliptic only). One period exactly, so the
        // last chord closes the ring.
        auto plotFullOrbit = [&](const sw::phys::KeplerOrbit& orbit,
                                 const sw::WorldVec3& primaryPosition,
                                 const sw::Vec4& color) {
            if (!orbit.isHyperbolic())
            {
                plotConic(orbit, primaryPosition, color, time,
                          time + sw::phys::kepler::period(orbit), kTrajectorySamples,
                          0.6f);
            }
        };

        // ---- celestial orbits: each body around its parent's current position --
        for (sw::usize i = 0; i < m_celestialIndex.size(); ++i)
        {
            const auto& body = m_celestialIndex.body(i);
            if (body.hasOrbit == 0 || body.parentIndex < 0)
            {
                continue;
            }
            sw::Vec4 color{0.5f, 0.5f, 0.55f, 1.0f};
            if (const auto* marker = m_world.tryGetComponent<MapMarkerComponent>(
                    body.entity))
            {
                color = marker->color * 0.6f;
            }
            plotFullOrbit(body.orbit,
                          m_celestialIndex.positionAt(body.parentIndex, time), color);
        }

        // ---- generic rails objects (station modules...) --------------------------
        m_world.forEach<sw::phys::OnRailsComponent, MapMarkerComponent>(
            [&](sw::ecs::Entity entity, sw::phys::OnRailsComponent& rails,
                MapMarkerComponent& marker) {
                if (entity == controlledEntity())
                {
                    return; // the controlled craft gets the full flight plan
                }
                sw::WorldVec3 primaryPosition{0.0};
                if (const auto* primaryTransform =
                        m_world.tryGetComponent<TransformComponent>(rails.primary))
                {
                    primaryPosition = primaryTransform->position;
                }
                plotFullOrbit(rails.orbit, primaryPosition, marker.color * 0.6f);
            });

        // ---- other dynamic objects: single conic around their SOI primary --------
        m_world.forEach<TransformComponent, sw::phys::DynamicBodyComponent,
                        MapMarkerComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                sw::phys::DynamicBodyComponent& body, MapMarkerComponent& marker) {
                if (entity == controlledEntity() || m_celestialIndex.size() == 0)
                {
                    return;
                }
                const sw::i32 primaryIndex =
                    m_celestialIndex.soiPrimaryAt(transform.position, time);
                if (primaryIndex < 0)
                {
                    return;
                }
                const auto& primary =
                    m_celestialIndex.body(static_cast<sw::usize>(primaryIndex));
                sw::WorldVec3 primaryPosition{};
                sw::WorldVec3 primaryVelocity{};
                m_celestialIndex.stateAt(primaryIndex, time, primaryPosition,
                                         &primaryVelocity);
                sw::phys::KeplerOrbit orbit{};
                if (sw::phys::kepler::fromStateVectors(
                        primary.mu, transform.position - primaryPosition,
                        body.velocity - primaryVelocity, time, orbit))
                {
                    plotFullOrbit(orbit, primaryPosition, marker.color * 0.6f);
                }
            });

        // ---- THE FLIGHT PLAN: patched-conics prediction of the controlled craft --
        auto plotPlan = [&](const std::vector<sw::space::TrajectorySegment>& plan,
                            bool nodePlan) {
            for (sw::usize segmentIndex = 0; segmentIndex < plan.size(); ++segmentIndex)
            {
                const sw::space::TrajectorySegment& segment = plan[segmentIndex];
                if (segment.primaryIndex < 0 ||
                    segment.endReason == sw::space::SegmentEnd::Lost)
                {
                    continue;
                }
                // The node plan glows white-violet so it never reads as the
                // current trajectory.
                const sw::Vec4 color =
                    nodePlan
                        ? sw::Vec4{0.85f, 0.75f + 0.25f * (segmentIndex == 0), 1.0f,
                                   1.0f}
                        : kPatchColors[std::min(segmentIndex,
                                                std::size(kPatchColors) - 1)];
                const sw::WorldVec3 primaryPosition =
                    m_celestialIndex.positionAt(segment.primaryIndex, time);

                // Every patch draws exactly its [start, end] arc — and the
                // predictor now ends a patch where something HAPPENS to it,
                // so that arc is the whole story: a closed orbit comes back
                // to its own start, a transfer runs to its encounter, an
                // escape runs to the edge of the sphere of influence.
                plotConic(segment.orbit, primaryPosition, color, segment.startTime,
                          segment.endTime, kPredictionDisplaySamples, 1.0f);

                // Event marker at the patch hand-off point.
                sw::Vec4 eventColor{};
                bool hasEvent = true;
                switch (segment.endReason)
                {
                case sw::space::SegmentEnd::Encounter:
                    eventColor = {0.4f, 1.0f, 0.9f, 1.0f};
                    break;
                case sw::space::SegmentEnd::Impact:
                    eventColor = {1.0f, 0.25f, 0.2f, 1.0f};
                    break;
                case sw::space::SegmentEnd::SoiExit:
                    eventColor = {1.0f, 0.85f, 0.4f, 1.0f};
                    break;
                default:
                    hasEvent = false;
                    break;
                }
                if (hasEvent)
                {
                    sw::WorldVec3 eventRelative{};
                    sw::phys::kepler::evaluate(segment.orbit, segment.endTime,
                                               eventRelative);
                    plotDot(primaryPosition + eventRelative, eventColor, 3.0f);
                }
            }
        };
        plotPlan(m_prediction, false);
        plotPlan(m_nodePrediction, true);

        // The maneuver node itself: a large violet diamond at the burn point,
        // drawn around its primary's CURRENT position like every patch.
        if (m_nodeActive && m_nodePrimaryIndex >= 0)
        {
            // Bigger and hotter while it is being dragged: the one moment
            // the player needs to be sure the map heard them.
            plotDot(m_celestialIndex.positionAt(m_nodePrimaryIndex, time) +
                        m_nodeRelativePosition,
                    m_nodeDragging ? sw::Vec4{1.0f, 0.85f, 0.45f, 1.0f}
                                   : sw::Vec4{0.75f, 0.4f, 1.0f, 1.0f},
                    m_nodeDragging ? 5.5f : 4.0f);
        }

        // ---- THE TARGET, AND WHERE IT WILL BE --------------------------------
        //
        // Three things, and the third is the one that makes a transfer
        // possible to fly: the body you picked, WHERE IT WILL HAVE MOVED TO
        // by closest approach, and where you will be when it does. Every
        // one of them is placed in the frame the orbits are drawn in, so
        // the future position sits on the ring rather than out in the world
        // where the body will really be.
        if (m_targetIndex >= 0 &&
            static_cast<sw::usize>(m_targetIndex) < m_celestialIndex.size())
        {
            constexpr sw::Vec4 kTargetColor{1.0f, 0.45f, 0.85f, 1.0f};
            plotDot(m_celestialIndex.positionAt(m_targetIndex, time), kTargetColor, 5.0f);

            auto framePosition = [&](sw::i32 primaryIndex,
                                     const sw::WorldVec3& relative) {
                return (primaryIndex >= 0)
                           ? m_celestialIndex.positionAt(primaryIndex, time) + relative
                           : relative;
            };
            for (const auto& [approach, marker] :
                 {std::pair{&m_approach, false}, std::pair{&m_nodeApproach, true}})
            {
                if (!approach->valid)
                {
                    continue;
                }
                const sw::Vec4 color =
                    marker ? sw::Vec4{0.85f, 0.75f, 1.0f, 1.0f} : kTargetColor;
                const sw::WorldVec3 theirs = framePosition(
                    approach->targetPrimaryIndex, approach->targetRelativePosition);
                const sw::WorldVec3 ours =
                    framePosition(approach->primaryIndex, approach->relativePosition);
                plotDot(theirs, color, 3.5f);
                plotDot(ours, color, 3.0f);
                // The gap itself, drawn: the separation is the point, and a
                // pair of dots leaves the eye to guess which two.
                plotLine(ours, theirs, color * sw::Vec4{1.0f, 1.0f, 1.0f, 0.6f}, 0.7f);
            }
        }
    }

    void StarWorksGame::collectDrawItems(const sw::Camera& activeCamera, bool mapView)
    {
        m_drawItems.clear();
        m_drawItems.reserve(m_world.aliveCount() + 512);

        // Static star dome: CAMERA-CENTERED (no translation), so the stars
        // are parallax-free — an infinitely distant, never-changing sky to
        // orient by. One emissive mesh, one draw call.
        {
            sw::DrawItem stars{};
            stars.mesh = &m_meshes[m_starfieldMeshIndex];
            stars.transform = glm::scale(sw::Mat4{1.0f}, sw::Vec3{kStarDomeRadius});
            stars.boundsCenter = {0.0f, 0.0f, 0.0f};
            stars.boundsRadius = kStarDomeRadius;
            // Daylight washes the stars out: the emissive opacity is
            // (vertexAlpha * tintAlpha - 1), so tint 2.0 = full night sky
            // and tint 1.0 = fully invisible at noon on the pad.
            const sw::f32 nightFactor = 1.0f - 0.96f * m_skyDayFactor;
            stars.tint = {1.0f, 1.0f, 1.0f, 1.0f + nightFactor};
            m_drawItems.push_back(stars);
        }

        // The sun's soft glow: two emissive radial-falloff discs, always
        // facing the camera, drawn in the transparent pass.
        if (const auto* sol = m_world.tryGetComponent<TransformComponent>(m_solEntity))
        {
            const sw::Vec3 toSol = sw::Vec3(sol->position - activeCamera.position());
            const sw::f32 distance = glm::length(toSol);
            if (distance > static_cast<sw::f32>(kSolRadius) * 3.0f)
            {
                const sw::Vec3 z = -toSol / distance; // disc normal, toward camera
                const sw::Vec3 reference =
                    (std::abs(z.y) < 0.99f) ? sw::Vec3{0, 1, 0} : sw::Vec3{1, 0, 0};
                const sw::Vec3 x = glm::normalize(glm::cross(reference, z));
                const sw::Vec3 yAxis = glm::cross(z, x);
                const sw::Mat4 basis{sw::Vec4(x, 0.0f), sw::Vec4(yAxis, 0.0f),
                                     sw::Vec4(z, 0.0f), sw::Vec4(toSol, 1.0f)};

                auto pushGlow = [&](sw::u32 meshIndex, sw::f32 radiusFactor) {
                    const sw::f32 radius =
                        static_cast<sw::f32>(kSolRadius) * radiusFactor;
                    sw::DrawItem glow{};
                    glow.mesh = &m_meshes[meshIndex];
                    glow.transform =
                        basis * glm::scale(sw::Mat4{1.0f}, sw::Vec3{radius});
                    glow.boundsCenter = toSol;
                    glow.boundsRadius = radius;
                    glow.tint = {1.0f, 1.0f, 1.0f, 1.0f};
                    glow.transparent = true;
                    m_drawItems.push_back(glow);
                };
                pushGlow(m_sunHaloMeshIndex, 7.5f);
                pushGlow(m_sunCoreMeshIndex, 2.1f);

                // ---- LENS FLARE: screen-space ghosts along the sun axis ------
                // Only when the sun is on screen and not behind a planet.
                if (!mapView)
                {
                    bool occluded = false;
                    for (sw::usize i = 0; i < m_celestialIndex.size(); ++i)
                    {
                        const auto& body = m_celestialIndex.body(i);
                        if (body.entity == m_solEntity)
                        {
                            continue;
                        }
                        if (const auto* bodyTransform =
                                m_world.tryGetComponent<TransformComponent>(body.entity))
                        {
                            const sw::Vec3 center =
                                sw::Vec3(bodyTransform->position -
                                         activeCamera.position());
                            const sw::Vec3 lightDir = toSol / distance;
                            const sw::f32 along = glm::dot(center, lightDir);
                            if (along > 0.0f && along < distance)
                            {
                                const sw::f32 miss =
                                    glm::length(center - lightDir * along);
                                if (miss < static_cast<sw::f32>(body.bodyRadius))
                                {
                                    occluded = true;
                                    break;
                                }
                            }
                        }
                    }
                    const sw::Vec4 clip =
                        activeCamera.viewProjectionCameraRelative() *
                        sw::Vec4(toSol, 1.0f);
                    // The chain only exists while the sun CORE is on
                    // screen AND in front of the camera (w>0 alone lets
                    // ghosts of an off-screen/behind sun float in deep
                    // space as stray colored circles).
                    const bool sunInFront =
                        clip.w > 0.0f && glm::dot(activeCamera.forward(), z) < 0.0f;
                    if (!occluded && sunInFront)
                    {
                        const sw::Vec2 sunNdc{clip.x / clip.w, clip.y / clip.w};
                        if (std::abs(sunNdc.x) < 0.98f && std::abs(sunNdc.y) < 0.98f)
                        {
                            // Fade toward the screen edge; ghosts mirror
                            // through the center (anamorphic-ish chain).
                            const sw::f32 edgeFade =
                                glm::clamp(1.05f - glm::length(sunNdc), 0.0f, 1.0f);
                            struct FlareGhost
                            {
                                sw::f32 t;      // position along sun->center axis
                                sw::f32 scale;  // NDC radius
                                sw::Vec3 color;
                                sw::f32 alpha;
                            };
                            const FlareGhost ghosts[] = {
                                {0.35f, 0.055f, {1.0f, 0.80f, 0.45f}, 0.16f},
                                {0.62f, 0.028f, {0.55f, 0.85f, 0.60f}, 0.14f},
                                {0.95f, 0.090f, {0.45f, 0.60f, 1.00f}, 0.10f},
                                {1.28f, 0.045f, {1.00f, 0.55f, 0.40f}, 0.12f},
                                {1.60f, 0.130f, {0.55f, 0.45f, 0.95f}, 0.07f},
                            };
                            const sw::f32 aspect = renderer().aspectRatio();
                            for (const FlareGhost& ghost : ghosts)
                            {
                                const sw::Vec2 position =
                                    sunNdc * (1.0f - ghost.t); // toward/past center
                                sw::DrawItem item{};
                                item.mesh = &m_meshes[m_flareMeshIndex];
                                item.transform =
                                    glm::translate(sw::Mat4{1.0f},
                                                   {position.x, position.y, 0.0f}) *
                                    glm::scale(sw::Mat4{1.0f},
                                               {ghost.scale / aspect, ghost.scale, 1.0f});
                                item.screenSpace = true;
                                item.tint = {ghost.color.r, ghost.color.g,
                                             ghost.color.b,
                                             ghost.alpha * edgeFade};
                                m_drawItems.push_back(item);
                            }
                        }
                    }
                }
            }
        }

        const sw::f32 alpha = m_physicsLane->alpha();
        const sw::f64 alpha64 = static_cast<sw::f64>(alpha);
        const sw::WorldVec3 cameraPosition = activeCamera.position();

        auto makeTransform = [&](const TransformComponent& transform,
                                 const PreviousTransformComponent& previous,
                                 sw::Vec3& outRelative) {
            const sw::WorldVec3 position =
                glm::mix(previous.position, transform.position, alpha64);
            const sw::Quat rotation = glm::slerp(previous.rotation, transform.rotation, alpha);
            outRelative = sw::Vec3(position - cameraPosition);
            return glm::translate(sw::Mat4{1.0f}, outRelative) * glm::mat4_cast(rotation) *
                   glm::scale(sw::Mat4{1.0f}, sw::Vec3{transform.uniformScale});
        };

        m_world.forEach<TransformComponent, PreviousTransformComponent, BoundsComponent,
                        MeshComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                PreviousTransformComponent& previous, BoundsComponent& bounds,
                MeshComponent& mesh) {
                // You are INSIDE the suit on EVA: drawing it would fill the
                // screen with the back of your own helmet. The map still
                // shows it — there you are looking at the world, not out of
                // your own eyes.
                if (!mapView && m_evaMode && entity == m_capsuleEntity)
                {
                    return;
                }
                sw::Vec3 relative{};
                const sw::Mat4 model = makeTransform(transform, previous, relative);
                sw::DrawItem item{&m_meshes[mesh.meshIndex], model, relative,
                                  bounds.localRadius * transform.uniformScale};
                item.transparent = mesh.transparent != MeshComponent::kOpaque;
                if (item.transparent)
                {
                    // Shell materials in Mesh.frag: 3.0 = atmosphere (fresnel
                    // limb), 3.2 = cloud deck (per-fragment weather).
                    const sw::f32 shell =
                        (mesh.transparent == MeshComponent::kCloudDeck) ? 3.2f : 3.0f;
                    item.tint = {1.0f, 1.0f, 1.0f, shell};
                }

                // Reentry glow: the craft reddens with heating and turns
                // emissive (self-lit plasma sheath) when it gets severe.
                sw::f32 heat = 0.0f;
                if (entity == m_shipEntity) { heat = m_shipHeat; }
                else if (entity == m_capsuleEntity) { heat = m_capsuleHeat; }
                else if (const auto* part =
                             m_world.tryGetComponent<sw::parts::PartComponent>(entity);
                         part != nullptr && part->vessel == m_shipEntity)
                {
                    heat = m_shipHeat; // the whole rocket glows
                }
                if (heat > 0.0f)
                {
                    const sw::Vec3 glow =
                        glm::mix(sw::Vec3{1.0f, 1.0f, 1.0f},
                                 sw::Vec3{1.0f, 0.30f, 0.12f}, heat);
                    item.tint = {glow.r, glow.g, glow.b, heat > 0.55f ? 2.0f : 1.0f};
                }
                m_drawItems.push_back(item);
            });

        m_world.forEach<TransformComponent, PreviousTransformComponent, BoundsComponent,
                        CelestialLodComponent>(
            [&](sw::ecs::Entity entity, TransformComponent& transform,
                PreviousTransformComponent& previous, BoundsComponent& bounds,
                CelestialLodComponent& lod) {
                sw::Vec3 relative{};
                const sw::Mat4 model = makeTransform(transform, previous, relative);
                const sw::f64 worldRadius = static_cast<sw::f64>(transform.uniformScale);
                const sw::f64 distance =
                    glm::length(transform.position - cameraPosition);
                const sw::u32 level = selectLodLevel(distance, worldRadius);
                sw::DrawItem item{&m_meshes[lod.meshIndex[level]], model, relative,
                                  bounds.localRadius * transform.uniformScale};
                // CLOSE ORBIT (M23): per-vertex colors blur when the globe
                // fills the screen — hand the surface to the PER-FRAGMENT
                // procedural path (tint alpha 3.6 + style/10 routes it in
                // Mesh.frag; the shader samples the exact same fbm as the
                // CPU terrain, so coastlines stay collision-true).
                // The threshold used to be 4 radii — 25,000 km on Terra, from
                // where the globe is a small disc that the vertex path draws
                // just as well for a fraction of the cost. At 1.6 radii the
                // expensive path only runs when the planet actually fills a
                // meaningful part of the screen, which is the only place its
                // sharpness is visible.
                if (lod.surfaceStyle >= 0 && distance < worldRadius * 1.6)
                {
                    item.tint = {1.0f, 1.0f, 1.0f,
                                 3.6f + 0.1f * static_cast<sw::f32>(lod.surfaceStyle)};
                }
                if (entity == m_solEntity)
                {
                    // The star is self-lit (emissive tint convention).
                    item.tint = {1.0f, 0.96f, 0.82f, 2.0f};
                }
                m_drawItems.push_back(item);
            });

        // ---- procedural terrain patch (near the ground, not in map view) ------
        if (!mapView && m_terrainVisible && m_terrainMeshSlot != 0xFFFFFFFFu)
        {
            if (const auto* body =
                    m_world.tryGetComponent<TransformComponent>(m_terrainBody))
            {
                sw::WorldVec3 bodyPosition = body->position;
                sw::Quat bodyRotation = body->rotation;
                if (const auto* previous =
                        m_world.tryGetComponent<PreviousTransformComponent>(
                            m_terrainBody))
                {
                    bodyPosition = glm::mix(previous->position, body->position, alpha64);
                    bodyRotation =
                        glm::slerp(previous->rotation, body->rotation, alpha);
                }
                // POSITION from the f64 spin (m_terrainOriginLocal is a full
                // planet radius long), ORIENTATION from the f32 quaternion
                // above — over the patch's own few kilometres that is a tenth
                // of a millimetre.
                const auto* spin =
                    m_world.tryGetComponent<sw::phys::GravitySourceComponent>(
                        m_terrainBody);
                const glm::dquat originRotation =
                    (spin != nullptr) ? sw::phys::spinRotationAt(*spin, alpha64)
                                      : glm::dquat(bodyRotation);
                const sw::WorldVec3 originWorld =
                    bodyPosition + originRotation * m_terrainOriginLocal;
                const sw::Vec3 relative = sw::Vec3(originWorld - cameraPosition);
                sw::DrawItem item{};
                item.mesh = &m_meshes[m_terrainMeshSlot];
                // The mesh is oriented by the SAME rotation that placed its
                // origin, cast down: two rotations a ten-millionth of a radian
                // apart would shear the patch away from its own anchor point.
                item.transform = glm::translate(sw::Mat4{1.0f}, relative) *
                                 glm::mat4_cast(sw::Quat(originRotation));
                item.boundsCenter = relative;
                item.boundsRadius = static_cast<sw::f32>(m_terrainExtent * 1.8);
                m_drawItems.push_back(item);

                // THE FIELD, in its own chunks. Same origin, same rotation —
                // it was built in the patch's chart and must be drawn in it,
                // or every blade would slide off the ground it stands on.
                if (m_grassLiveCount != 0 && m_grassBody == m_terrainBody)
                {
                    const sw::WorldVec3 grassOrigin =
                        bodyPosition + originRotation * m_grassOriginLocal;
                    const sw::Vec3 grassRelative = sw::Vec3(grassOrigin - cameraPosition);
                    for (sw::u32 chunk = 0; chunk < m_grassLiveCount; ++chunk)
                    {
                        const sw::u32 slot = m_grassSlots[m_grassSet][chunk];
                        if (!m_grassChunkValid[m_grassSet][chunk] ||
                            slot == 0xFFFFFFFFu || slot >= m_meshes.size())
                        {
                            continue;
                        }
                        sw::DrawItem blades{};
                        blades.mesh = &m_meshes[slot];
                        blades.transform =
                            glm::translate(sw::Mat4{1.0f}, grassRelative) *
                            glm::mat4_cast(sw::Quat(originRotation));
                        blades.boundsCenter = grassRelative;
                        blades.boundsRadius = 320.0f;
                        m_drawItems.push_back(blades);
                    }
                }
            }
        }

        if (mapView)
        {
            const sw::f32 markerFactor =
                kMarkerScreenFraction * 2.0f * std::tan(activeCamera.verticalFov() * 0.5f);

            m_world.forEach<TransformComponent, BoundsComponent, MapMarkerComponent>(
                [&](sw::ecs::Entity, TransformComponent& transform, BoundsComponent& bounds,
                    MapMarkerComponent& marker) {
                    const sw::WorldVec3 toCamera = cameraPosition - transform.position;
                    const sw::f64 distance = glm::length(toCamera);
                    if (distance < 1.0)
                    {
                        return;
                    }
                    const sw::f64 surfaceOffset =
                        static_cast<sw::f64>(bounds.localRadius * transform.uniformScale) *
                        1.05;
                    const sw::WorldVec3 beaconPosition =
                        transform.position + (toCamera / distance) * surfaceOffset;

                    const sw::f32 scale =
                        static_cast<sw::f32>(glm::length(beaconPosition - cameraPosition)) *
                        markerFactor;
                    const sw::Vec3 relative = sw::Vec3(beaconPosition - cameraPosition);
                    const sw::Mat4 model = glm::translate(sw::Mat4{1.0f}, relative) *
                                           glm::scale(sw::Mat4{1.0f}, sw::Vec3{scale});
                    m_drawItems.push_back(
                        {&m_meshes[m_markerMeshIndex], model, relative, scale,
                         {marker.color.r, marker.color.g, marker.color.b, 2.0f}});
                });

            collectMapTrajectories(activeCamera);
        }
        else
        {
            collectParticles(activeCamera);
        }

        if (!mapView)
        {
            collectConveyors(activeCamera);
            collectCables(activeCamera);
            collectHullOverlay(activeCamera);
            collectBuildGhost(activeCamera);
        }
        // Beacons overlay both views; the HUD is drawn last so its panels
        // stay on top of them.
        collectBeacons(activeCamera, mapView);
        collectHud();
    }
} // namespace game
