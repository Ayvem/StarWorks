// ============================================================================
// GameHangar.cpp — The hangar (design office): blueprint editing, ghosts, symmetry, .swship files.
// Split out of StarWorksGame.cpp; same class, one theme per translation unit.
// ============================================================================

#include "StarWorksGame.hpp"

#include "GameInternal.hpp"
#include "Systems.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <format>
#include <limits>

namespace game
{

    // ========================= THE HANGAR (B) ==============================
    void StarWorksGame::enterEditor()
    {
        m_editorMode = true;
        m_pausedBeforeEditor = m_simulation.isPaused();
        m_simulation.setPaused(true);
        m_heldDefinition = 0;
        m_heldSubtree.clear();
        m_blueprintBackup.clear();
        m_heldRotation = {1.0f, 0.0f, 0.0f, 0.0f};
        m_ghost = {};
        // Open on the CURRENT ship, loaded as an editable blueprint.
        m_hangarSource = {};
        m_blueprint.clear();
        hangarLoadNextVessel();
        if (m_blueprint.empty())
        {
            hangarNewBlueprint();
        }
        SW_LOG_INFO("Game", "HANGAR: open");
    }

    void StarWorksGame::exitEditor()
    {
        m_editorMode = false;
        m_simulation.setPaused(m_pausedBeforeEditor);
        SW_LOG_INFO("Game", "HANGAR: closed");
    }

    void StarWorksGame::hangarNewBlueprint()
    {
        m_blueprint.clear();
        m_hangarSource = {};
        BlueprintPart core{};
        core.definitionId = sw::parts::kPartCoreStructural;
        m_blueprint.push_back(core); // every design starts from a command core
        m_heldDefinition = 0;
        m_heldSubtree.clear();
        m_blueprintBackup.clear();
    }

    void StarWorksGame::hangarLoadNextVessel()
    {
        // Cycle through the world's part-built vessels, loading each into
        // the blueprint for modification.
        std::vector<sw::ecs::Entity> vessels;
        m_world.forEach<sw::parts::VesselComponent>(
            [&](sw::ecs::Entity entity, sw::parts::VesselComponent& vessel) {
                if (vessel.partCount > 0)
                {
                    vessels.push_back(entity);
                }
            });
        if (vessels.empty())
        {
            return;
        }
        sw::usize next = 0;
        for (sw::usize i = 0; i < vessels.size(); ++i)
        {
            if (vessels[i] == m_hangarSource)
            {
                next = (i + 1) % vessels.size();
            }
        }
        m_hangarSource = vessels[next];

        // Parts -> blueprint (indices), joints -> parent links.
        m_blueprint.clear();
        std::vector<sw::ecs::Entity> partEntities;
        m_world.forEach<sw::parts::PartComponent>(
            [&](sw::ecs::Entity entity, sw::parts::PartComponent& part) {
                if (part.vessel != m_hangarSource)
                {
                    return;
                }
                BlueprintPart bp{};
                bp.definitionId = part.definitionId;
                bp.localPosition = part.localPosition;
                bp.localRotation = part.localRotation;
                m_blueprint.push_back(bp);
                partEntities.push_back(entity);
            });
        m_world.forEach<sw::parts::JointComponent>(
            [&](sw::ecs::Entity, sw::parts::JointComponent& jointComponent) {
                sw::i32 a = -1;
                sw::i32 b = -1;
                for (sw::usize i = 0; i < partEntities.size(); ++i)
                {
                    if (partEntities[i] == jointComponent.partA) { a = static_cast<sw::i32>(i); }
                    if (partEntities[i] == jointComponent.partB) { b = static_cast<sw::i32>(i); }
                }
                if (a >= 0 && b >= 0)
                {
                    m_blueprint[static_cast<sw::usize>(b)].parentIndex = a;
                    m_blueprint[static_cast<sw::usize>(b)].parentPoint =
                        jointComponent.attachPointA;
                    m_blueprint[static_cast<sw::usize>(b)].childPoint =
                        jointComponent.attachPointB;
                }
            });
        m_heldDefinition = 0;
        m_heldSubtree.clear();
        m_blueprintBackup.clear();
        SW_LOG_INFO("Game", "HANGAR: loaded vessel ({} parts)", m_blueprint.size());
    }

    namespace
    {
        /// The hangar shows the NOSE (-Z) up: +90 deg about X maps +Z to -Y.
        const sw::Quat kHangarDisplay = glm::angleAxis(1.5707963f, sw::Vec3{1, 0, 0});

        /// Shortest-arc rotation taking `from` onto `to` (both normalized).
        [[nodiscard]] sw::Quat rotationBetween(const sw::Vec3& from, const sw::Vec3& to)
        {
            const sw::f32 cosine = glm::dot(from, to);
            if (cosine > 0.9999f)
            {
                return {1.0f, 0.0f, 0.0f, 0.0f};
            }
            if (cosine < -0.9999f)
            {
                const sw::Vec3 seed =
                    std::abs(from.x) < 0.9f ? sw::Vec3{1, 0, 0} : sw::Vec3{0, 1, 0};
                return glm::angleAxis(3.14159265f,
                                      glm::normalize(glm::cross(from, seed)));
            }
            return glm::angleAxis(std::acos(std::clamp(cosine, -1.0f, 1.0f)),
                                  glm::normalize(glm::cross(from, to)));
        }

        constexpr sw::u32 kSymmetryOptions[6] = {1, 2, 3, 4, 6, 8};
    } // namespace

    std::vector<StarWorksGame::OpenAttachPoint> StarWorksGame::openAttachPoints()
    {
        // Open = STACK nodes of blueprint parts not consumed by any link
        // (radial attachment is surface-based and never blocks a node).
        std::vector<OpenAttachPoint> open;
        for (sw::usize i = 0; i < m_blueprint.size(); ++i)
        {
            const auto* definition =
                sw::parts::findDefinition(m_blueprint[i].definitionId);
            if (definition == nullptr)
            {
                continue;
            }
            for (sw::u8 p = 0; p < static_cast<sw::u8>(definition->nodes.size()); ++p)
            {
                if (definition->nodes[p].type != sw::parts::NodeType::Stack)
                {
                    continue;
                }
                bool occupied = false;
                for (sw::usize j = 0; j < m_blueprint.size(); ++j)
                {
                    const auto& other = m_blueprint[j];
                    if ((other.parentIndex == static_cast<sw::i32>(i) &&
                         other.parentPoint == p) ||
                        (j == i && other.parentIndex >= 0 && other.childPoint == p))
                    {
                        occupied = true;
                        break;
                    }
                }
                if (occupied)
                {
                    continue;
                }
                OpenAttachPoint point{};
                point.partIndex = static_cast<sw::i32>(i);
                point.pointIndex = p;
                point.vesselPosition =
                    m_blueprint[i].localPosition +
                    m_blueprint[i].localRotation * definition->nodes[p].position;
                point.vesselDirection =
                    m_blueprint[i].localRotation * definition->nodes[p].direction;
                point.size = definition->nodes[p].size;
                open.push_back(point);
            }
        }
        return open;
    }

    void StarWorksGame::editorCursorRay(sw::Vec3& outOrigin, sw::Vec3& outDirection)
    {
        // A CAPTURE HAS NO MOUSE, and the editor is nothing but a mouse. The
        // hooks that photograph it aim this ray instead — everything
        // downstream, the ghost, the symmetry, the red or green and the
        // commit, is the code a click runs.
        if (m_editorRayScripted)
        {
            outOrigin = m_editorRayOrigin;
            outDirection = m_editorRayDirection;
            return;
        }
        sw::u32 width = 0;
        sw::u32 height = 0;
        window().framebufferSize(width, height);
        const sw::f32 ndcX =
            input().mouseX() / static_cast<sw::f32>(std::max(width, 1u)) * 2.0f - 1.0f;
        const sw::f32 ndcY =
            input().mouseY() / static_cast<sw::f32>(std::max(height, 1u)) * 2.0f - 1.0f;
        // Unproject two depths (reverse-Z friendly), then undo the display
        // rotation so the ray lives in the BLUEPRINT frame.
        const sw::Mat4 inverse =
            glm::inverse(m_hangarCamera.viewProjectionCameraRelative());
        const sw::Vec4 nearPoint = inverse * sw::Vec4{ndcX, ndcY, 0.9f, 1.0f};
        const sw::Vec4 farPoint = inverse * sw::Vec4{ndcX, ndcY, 0.1f, 1.0f};
        const sw::Vec3 a = sw::Vec3(nearPoint) / nearPoint.w;
        const sw::Vec3 b = sw::Vec3(farPoint) / farPoint.w;
        const sw::Quat undo = glm::inverse(kHangarDisplay);
        outOrigin = undo * sw::Vec3(m_hangarCamera.position());
        outDirection = undo * glm::normalize(b - a);
    }

    void StarWorksGame::debugPlacePart(const char* spec)
    {
        if (spec == nullptr || !m_editorMode)
        {
            SW_LOG_ERROR("Game", "SW_PLACE: not in the hangar — set SW_HANGAR too");
            return;
        }
        // <definitionId>[,<symmetry>[,<azimuthDeg>[,<height>]]]
        char* cursor = nullptr;
        const sw::u32 definitionId =
            static_cast<sw::u32>(std::strtoul(spec, &cursor, 10));
        const sw::u32 symmetry =
            (*cursor == ',') ? static_cast<sw::u32>(std::strtoul(cursor + 1, &cursor, 10))
                             : 1u;
        const sw::f32 azimuth =
            (*cursor == ',') ? std::strtof(cursor + 1, &cursor) : 0.0f;
        const sw::f32 height = (*cursor == ',') ? std::strtof(cursor + 1, &cursor) : 0.0f;
        if (sw::parts::findDefinition(definitionId) == nullptr)
        {
            SW_LOG_ERROR("Game", "SW_PLACE: no part with id {}", definitionId);
            return;
        }
        m_heldDefinition = definitionId;
        m_heldSubtree.clear();
        m_heldRotation = sw::Quat{1.0f, 0.0f, 0.0f, 0.0f};
        m_symmetryCount = std::max(symmetry, 1u);

        // AIM AT THE STACK FROM OUTSIDE IT, along the azimuth asked for: the
        // ray a player's cursor would cast standing on that side of the deck.
        constexpr sw::f32 kDegrees = 3.14159265f / 180.0f;
        const sw::Vec3 inward{-std::cos(azimuth * kDegrees), -std::sin(azimuth * kDegrees),
                              0.0f};
        m_editorRayScripted = true;
        m_editorRayDirection = glm::normalize(inward);
        m_editorRayOrigin = -inward * 40.0f + sw::Vec3{0.0f, 0.0f, height};
        computeGhost();
        const bool active = m_ghost.active;
        const bool valid = m_ghost.valid;
        commitGhost();
        m_editorRayScripted = false;
        if (!active || !valid)
        {
            SW_LOG_ERROR("Game",
                         "SW_PLACE: part {} at {:.0f} deg x{} REFUSED (active {}, "
                         "collides {}, overloaded {}, parent {}, blocked by {})",
                         definitionId, static_cast<double>(azimuth), m_symmetryCount,
                         active, m_ghostCollides, m_ghostOverloaded, m_ghost.parentIndex,
                         m_ghostBlockedBy);
            return;
        }
        SW_LOG_INFO("Game", "SW_PLACE: part {} placed at {:.0f} deg, symmetry x{} -> {} parts",
                    definitionId, static_cast<double>(azimuth), m_symmetryCount,
                    m_blueprint.size());
    }

    sw::f64 StarWorksGame::partWetMassKg(sw::u32 definitionId) const
    {
        const auto* definition = sw::parts::findDefinition(definitionId);
        if (definition == nullptr)
        {
            return 0.0;
        }
        sw::f64 mass = definition->dryMassKg;
        for (const auto& capacity : definition->capacities)
        {
            if (capacity.resource != sw::res::Resource::Count)
            {
                mass += capacity.units *
                        sw::res::definition(capacity.resource).massPerUnitKg;
            }
        }
        return mass;
    }

    void StarWorksGame::computeGhost()
    {
        m_ghost = {};
        if (m_heldDefinition == 0)
        {
            return;
        }
        const auto* held = sw::parts::findDefinition(m_heldDefinition);
        if (held == nullptr)
        {
            return;
        }
        sw::Vec3 origin{};
        sw::Vec3 direction{};
        editorCursorRay(origin, direction);

        // ---- 1. STACK MAGNET: nearest open node whose ray distance is small ----
        sw::f32 bestAlong = 1.0e30f;
        for (const OpenAttachPoint& node : openAttachPoints())
        {
            sw::i32 childPoint = -1;
            for (sw::u8 c = 0; c < static_cast<sw::u8>(held->nodes.size()); ++c)
            {
                if (held->nodes[c].type != sw::parts::NodeType::Stack)
                {
                    continue;
                }
                if (glm::dot(m_heldRotation * held->nodes[c].direction,
                             -node.vesselDirection) > 0.98f)
                {
                    childPoint = c;
                    break;
                }
            }
            if (childPoint < 0)
            {
                continue;
            }
            const sw::Vec3 toNode = node.vesselPosition - origin;
            const sw::f32 along = glm::dot(toNode, direction);
            if (along <= 0.0f)
            {
                continue;
            }
            const sw::f32 distance = glm::length(toNode - direction * along);
            if (distance < std::max(0.9f, node.size * 1.1f) && along < bestAlong)
            {
                bestAlong = along;
                m_ghost.active = true;
                m_ghost.rotation = m_heldRotation;
                m_ghost.position =
                    node.vesselPosition -
                    m_heldRotation * held->nodes[childPoint].position;
                m_ghost.parentIndex = node.partIndex;
                m_ghost.parentPoint = node.pointIndex;
                m_ghost.childPoint = static_cast<sw::u8>(childPoint);
            }
        }

        // ---- 2. RADIAL SURFACE: glue onto the collider under the cursor --------
        if (!m_ghost.active)
        {
            sw::i32 radialChild = -1;
            for (sw::u8 c = 0; c < static_cast<sw::u8>(held->nodes.size()); ++c)
            {
                if (held->nodes[c].type == sw::parts::NodeType::Radial)
                {
                    radialChild = c;
                    break;
                }
            }
            if (radialChild >= 0)
            {
                sw::f32 bestT = 1.0e30f;
                sw::i32 hitPart = -1;
                sw::Vec3 hitPoint{};
                sw::Vec3 hitNormal{0.0f, 0.0f, 1.0f};
                for (sw::usize i = 0; i < m_blueprint.size(); ++i)
                {
                    const auto* def =
                        sw::parts::findDefinition(m_blueprint[i].definitionId);
                    if (def == nullptr)
                    {
                        continue;
                    }
                    const sw::Quat inverseRot =
                        glm::inverse(m_blueprint[i].localRotation);
                    const sw::Vec3 localOrigin =
                        inverseRot * (origin - m_blueprint[i].localPosition);
                    const sw::Vec3 localDirection = inverseRot * direction;
                    sw::parts::PartRayHit hit{};
                    if (sw::parts::raycastPart(*def, localOrigin, localDirection,
                                               500.0f, hit) &&
                        hit.t < bestT)
                    {
                        bestT = hit.t;
                        hitPart = static_cast<sw::i32>(i);
                        hitPoint = m_blueprint[i].localPosition +
                                   m_blueprint[i].localRotation *
                                       (localOrigin + localDirection * hit.t);
                        hitNormal = m_blueprint[i].localRotation * hit.normal;
                    }
                }
                if (hitPart >= 0)
                {
                    const sw::Vec3 glueDirection =
                        m_heldRotation * held->nodes[radialChild].direction;
                    const sw::Quat align = rotationBetween(glueDirection, -hitNormal);
                    m_ghost.active = true;
                    m_ghost.rotation = align * m_heldRotation;
                    m_ghost.position =
                        hitPoint -
                        m_ghost.rotation * held->nodes[radialChild].position;
                    m_ghost.parentIndex = hitPart;
                    m_ghost.parentPoint = 255; // surface attachment
                    m_ghost.childPoint = static_cast<sw::u8>(radialChild);
                }
            }
        }

        if (!m_ghost.active)
        {
            // Free-floating red ghost: nothing under the cursor to attach to.
            m_ghost.position = origin + direction * (m_hangarDistance * 0.7f);
            m_ghost.rotation = m_heldRotation;
            return;
        }

        // ---- validation: real compound-collider overlap + joint load -----------
        // The candidate set = held root (+ its grabbed subtree) (+ symmetry
        // clones for radial placement), tested by the engine's placement rule
        // — which knows that the part you are bolting ON to is not something
        // you can collide with.
        std::vector<sw::parts::PlacementPiece> candidates;
        std::vector<sw::parts::PlacementPiece> placed;
        placed.reserve(m_blueprint.size());
        for (const BlueprintPart& part : m_blueprint)
        {
            placed.push_back({sw::parts::findDefinition(part.definitionId),
                              part.localPosition, part.localRotation});
        }
        for (sw::u32 k = 0; k < ghostCloneCount(); ++k)
        {
            candidates.push_back(sw::parts::symmetryClone(
                {held, m_ghost.position, m_ghost.rotation}, k, ghostCloneCount()));
            for (const BlueprintPart& rel : m_heldSubtree)
            {
                const auto* relDef = sw::parts::findDefinition(rel.definitionId);
                if (relDef != nullptr)
                {
                    candidates.push_back(sw::parts::symmetryClone(
                        {relDef, m_ghost.position + m_ghost.rotation * rel.localPosition,
                         m_ghost.rotation * rel.localRotation},
                        k, ghostCloneCount()));
                }
            }
        }
        const std::vector<sw::i32> parents = ghostParents(ghostCloneCount());
        m_ghostBlockedBy = -1;
        const bool collides = sw::parts::placementCollides(candidates, placed, parents,
                                                           0.05f, &m_ghostBlockedBy);
        m_ghostCollides = collides;

        sw::f64 childMass = partWetMassKg(m_heldDefinition);
        for (const BlueprintPart& rel : m_heldSubtree)
        {
            childMass += partWetMassKg(rel.definitionId);
        }
        const auto* parentDef = sw::parts::findDefinition(
            m_blueprint[static_cast<sw::usize>(m_ghost.parentIndex)].definitionId);
        const bool overloaded =
            parentDef != nullptr &&
            childMass * 12.0 >
                std::min(held->breakingForceN, parentDef->breakingForceN);
        m_ghostOverloaded = overloaded;
        m_ghost.valid = !collides && !overloaded;
    }

    /// How many copies this placement makes. Symmetry is a RADIAL idea — a
    /// ring of boosters round a core — so it applies to surface attachment
    /// only, and only to a part taken fresh from the palette: a grabbed
    /// subtree already has a shape of its own to keep.
    sw::u32 StarWorksGame::ghostCloneCount() const
    {
        return (m_ghost.parentPoint == 255 && m_heldSubtree.empty()) ? m_symmetryCount
                                                                     : 1u;
    }

    /// WHAT EACH COPY BOLTS TO. Three decouplers round a core and then three
    /// boosters on them is the shape this room exists to build, and hanging
    /// all three boosters off the ONE decoupler the cursor touched makes a
    /// rocket that looks right and stages wrong — firing that decoupler would
    /// drop a booster from the far side. When the parent is itself part of a
    /// ring of the same size, copy k goes on ring member k, counted from the
    /// one under the cursor so the piece you aimed at is the piece you get.
    ///
    /// The validity test needs the same answer as the commit: every copy
    /// touches ITS parent by construction, and excluding only the first makes
    /// every other copy read as a collision with what it is mounted on.
    std::vector<sw::i32> StarWorksGame::ghostParents(sw::u32 cloneCount) const
    {
        std::vector<sw::i32> parents(std::max(cloneCount, 1u), m_ghost.parentIndex);
        if (cloneCount < 2 || m_ghost.parentIndex < 0)
        {
            return parents;
        }
        const sw::i32 parentGroup =
            m_blueprint[static_cast<sw::usize>(m_ghost.parentIndex)].symmetryGroup;
        if (parentGroup < 0)
        {
            return parents;
        }
        std::vector<sw::i32> ring;
        for (sw::usize i = 0; i < m_blueprint.size(); ++i)
        {
            if (m_blueprint[i].symmetryGroup == parentGroup)
            {
                ring.push_back(static_cast<sw::i32>(i));
            }
        }
        if (ring.size() != cloneCount)
        {
            return parents; // a ring of a different size: no sensible pairing
        }
        sw::usize first = 0;
        for (sw::usize i = 0; i < ring.size(); ++i)
        {
            if (ring[i] == m_ghost.parentIndex) { first = i; }
        }
        for (sw::u32 k = 0; k < cloneCount; ++k)
        {
            parents[k] = ring[(first + k) % ring.size()];
        }
        return parents;
    }

    void StarWorksGame::commitGhost()
    {
        if (!m_ghost.active || !m_ghost.valid || m_heldDefinition == 0)
        {
            return;
        }
        const sw::u32 cloneCount = ghostCloneCount();
        const sw::i32 group =
            cloneCount > 1 ? m_symmetryNextGroup++ : -1;
        const std::vector<sw::i32> parents = ghostParents(cloneCount);
        for (sw::u32 k = 0; k < cloneCount; ++k)
        {
            const sw::parts::PlacementPiece clone = sw::parts::symmetryClone(
                {nullptr, m_ghost.position, m_ghost.rotation}, k, cloneCount);
            BlueprintPart part{};
            part.definitionId = m_heldDefinition;
            part.localPosition = clone.position;
            part.localRotation = clone.rotation;
            part.parentIndex = parents[k];
            part.parentPoint = m_ghost.parentPoint;
            part.childPoint = m_ghost.childPoint;
            part.symmetryGroup = group;
            m_blueprint.push_back(part);
            const sw::i32 rootIndex = static_cast<sw::i32>(m_blueprint.size()) - 1;
            const sw::i32 subBase = rootIndex + 1;
            for (const BlueprintPart& rel : m_heldSubtree)
            {
                const sw::parts::PlacementPiece relClone = sw::parts::symmetryClone(
                    {nullptr, m_ghost.position + m_ghost.rotation * rel.localPosition,
                     m_ghost.rotation * rel.localRotation},
                    k, cloneCount);
                BlueprintPart absolute = rel;
                absolute.localPosition = relClone.position;
                absolute.localRotation = relClone.rotation;
                absolute.parentIndex =
                    rel.parentIndex < 0 ? rootIndex : subBase + rel.parentIndex;
                absolute.symmetryGroup = -1;
                m_blueprint.push_back(absolute);
            }
        }
        const sw::u32 placed = m_heldDefinition;
        m_heldDefinition = 0;
        m_heldSubtree.clear();
        m_blueprintBackup.clear();
        m_ghost = {};
        // PLACING THE BASE PUTS YOU IN THE TOOL, which is how KSP does it and
        // is right for the same reason: a fairing base on its own is not a
        // part anybody wants, it is the first half of one gesture.
        if (const auto* definition = sw::parts::findDefinition(placed);
            definition != nullptr && definition->type == sw::parts::PartType::Fairing &&
            !m_blueprint.empty())
        {
            beginFairing(m_blueprint.size() - 1);
        }
    }

    void StarWorksGame::grabPartAt(sw::usize index)
    {
        if (index >= m_blueprint.size() || m_blueprint[index].parentIndex < 0)
        {
            return; // never grab a root part
        }
        m_blueprintBackup = m_blueprint; // ESC puts everything back

        // Subtree = the part and everything below it (children recurse).
        std::vector<sw::usize> subtree{index};
        for (sw::usize scan = 0; scan < subtree.size(); ++scan)
        {
            for (sw::usize j = 0; j < m_blueprint.size(); ++j)
            {
                if (m_blueprint[j].parentIndex ==
                    static_cast<sw::i32>(subtree[scan]))
                {
                    subtree.push_back(j);
                }
            }
        }

        const BlueprintPart root = m_blueprint[index];
        m_heldDefinition = root.definitionId;
        m_heldRotation = root.localRotation;
        const sw::Quat inverseRoot = glm::inverse(root.localRotation);

        // Relative copies, parents remapped into the subtree (-1 = the root).
        std::vector<sw::i32> toSubtree(m_blueprint.size(), -2);
        toSubtree[index] = -1;
        m_heldSubtree.clear();
        for (sw::usize s = 1; s < subtree.size(); ++s)
        {
            BlueprintPart rel = m_blueprint[subtree[s]];
            rel.localPosition = inverseRoot * (rel.localPosition - root.localPosition);
            rel.localRotation = inverseRoot * rel.localRotation;
            rel.parentIndex = toSubtree[static_cast<sw::usize>(rel.parentIndex)];
            rel.symmetryGroup = -1;
            toSubtree[subtree[s]] = static_cast<sw::i32>(m_heldSubtree.size());
            m_heldSubtree.push_back(rel);
        }

        // The root's symmetry siblings become independent parts.
        if (root.symmetryGroup >= 0)
        {
            for (BlueprintPart& bp : m_blueprint)
            {
                bp.symmetryGroup =
                    bp.symmetryGroup == root.symmetryGroup ? -1 : bp.symmetryGroup;
            }
        }

        // Remove the subtree, remapping the survivors' parent indices.
        std::vector<bool> removed(m_blueprint.size(), false);
        for (const sw::usize s : subtree)
        {
            removed[s] = true;
        }
        std::vector<sw::i32> newIndex(m_blueprint.size(), -1);
        sw::i32 next = 0;
        for (sw::usize i = 0; i < m_blueprint.size(); ++i)
        {
            if (!removed[i])
            {
                newIndex[i] = next++;
            }
        }
        std::vector<BlueprintPart> remaining;
        remaining.reserve(m_blueprint.size() - subtree.size());
        for (sw::usize i = 0; i < m_blueprint.size(); ++i)
        {
            if (removed[i])
            {
                continue;
            }
            BlueprintPart bp = m_blueprint[i];
            if (bp.parentIndex >= 0)
            {
                bp.parentIndex = newIndex[static_cast<sw::usize>(bp.parentIndex)];
            }
            remaining.push_back(bp);
        }
        m_blueprint = std::move(remaining);
        SW_LOG_INFO("Game", "HANGAR: grabbed a subtree of {} part(s)",
                    subtree.size());
    }

    sw::ecs::Entity StarWorksGame::instantiateBlueprint(sw::ecs::Entity existingRoot,
                                                        sw::ecs::Entity pad)
    {
        sw::ecs::Entity root = existingRoot;

        // THE VESSEL'S GROUND HULL, straight from the blueprint: the box its
        // collider shapes fill in vessel space. The pad uses it to stand the
        // rocket ON its engine bells instead of at a guessed offset, and
        // VesselAssemblySystem recomputes the very same box every tick from
        // the live parts — so the spawn pose and the resting pose agree by
        // construction rather than by a constant somebody has to maintain.
        sw::phys::GroundHullComponent hull{};
        {
            constexpr sw::f32 kHuge = 1.0e9f;
            sw::Vec3 low{kHuge, kHuge, kHuge};
            sw::Vec3 high{-kHuge, -kHuge, -kHuge};
            for (const BlueprintPart& bp : m_blueprint)
            {
                if (const auto* definition = sw::parts::findDefinition(bp.definitionId))
                {
                    sw::parts::expandPartHullBounds(*definition, bp.localPosition,
                                                        bp.localRotation, low, high);
                }
            }
            if (low.x <= high.x)
            {
                hull.centre = (low + high) * 0.5f;
                hull.halfExtents = (high - low) * 0.5f;
            }
        }

        if (root.isNull())
        {
            // NEW vessel: born on a LAUNCH PAD, standing on the ground,
            // co-rotating with the planet, nose to the sky.
            //
            // WHICH pad is the F5 question. A real LP-1 the player built has
            // an anchor — a body and a body-frame position — and that is all
            // this needs; everything else below is the same arithmetic it
            // always was. With no pad (the hangar's BUILD shortcut) it falls
            // back to the surveyed place 120 m east of the outpost hub.
            sw::ecs::Entity bodyEntity = m_terraEntity;
            sw::Vec3 padDir{0.0f, 1.0f, 0.0f};
            sw::f64 groundRadius = 0.0;
            const auto* padAnchor =
                m_world.tryGetComponent<sw::phys::SurfaceAnchorComponent>(pad);
            if (padAnchor != nullptr &&
                m_world.hasComponent<TransformComponent>(padAnchor->body))
            {
                bodyEntity = padAnchor->body;
                const sw::f64 radius = glm::length(padAnchor->localPosition);
                padDir = (radius > 1.0) ? sw::Vec3(padAnchor->localPosition / radius)
                                        : sw::Vec3{0.0f, 1.0f, 0.0f};
                // ...and the rocket stands on the pad's DECK, not on the dirt
                // the pad is bolted to.
                //
                // The deck is the top of the hull boxes that lie UNDER THE
                // PAD'S AXIS, not the top of the whole hull: LP-1 carries a
                // service tower 18 m tall in one corner, and taking the
                // bounding box's ceiling stood the rocket up there in the
                // air beside it. Reading it off the boxes means redrawing
                // LP-1 thicker in Part Studio still moves the rocket with
                // it, which is the property worth keeping.
                sw::f64 deck = 0.0;
                if (const auto* building =
                        m_world.tryGetComponent<sw::factory::BuildingComponent>(pad))
                {
                    if (const auto* padDefinition =
                            sw::parts::findDefinition(building->definitionId))
                    {
                        for (const sw::parts::HitBox& box :
                             sw::parts::effectiveHull(*padDefinition))
                        {
                            if (std::abs(box.center.x) <= std::abs(box.halfExtents.x) &&
                                std::abs(box.center.z) <= std::abs(box.halfExtents.z))
                            {
                                deck = std::max(deck, static_cast<sw::f64>(
                                                          box.center.y +
                                                          std::abs(box.halfExtents.y)));
                            }
                        }
                    }
                }
                groundRadius = radius + deck;
            }
            else
            {
                // 120 m east of the site hub: the same surveyed ground, so
                // the pad is on land and the factory is walking distance
                // away.
                const sw::Vec3 siteDir = terraStartSite();
                const sw::Vec3 padEast = glm::normalize(glm::cross(
                    (std::abs(siteDir.y) < 0.9f) ? sw::Vec3{0.0f, 1.0f, 0.0f}
                                                 : sw::Vec3{1.0f, 0.0f, 0.0f},
                    siteDir));
                padDir = glm::normalize(
                    siteDir + padEast * (120.0f / static_cast<sw::f32>(kTerraRadius)));
                groundRadius =
                    kTerraRadius + sw::planet::terrainElevation(presetTerra(), padDir);
            }

            const auto& terra = m_world.getComponent<TransformComponent>(bodyEntity);
            const auto& gravity =
                m_world.getComponent<sw::phys::GravitySourceComponent>(bodyEntity);
            // A new rocket stands TAIL DOWN, so its model +Z is the axis
            // pointing at the ground: the clearance is exactly how far the
            // hull reaches along +Z. (The 11 m constant this replaces was a
            // guess at one particular rocket's half length.)
            const sw::f64 clearance =
                static_cast<sw::f64>(hull.centre.z + hull.halfExtents.z);
            const sw::WorldVec3 padLocal =
                sw::WorldVec3(padDir) * (groundRadius + clearance);
            // Same rule as every surface anchor: a planet-radius offset is
            // rotated with the f64 spin, or the rocket lands a metre from
            // the pad it was supposed to be standing on.
            const glm::dquat terraRotation = sw::phys::spinRotation(gravity);
            const sw::WorldVec3 position = terra.position + terraRotation * padLocal;
            const sw::WorldVec3 radial = position - terra.position;
            const sw::Vec3 up = sw::Vec3(glm::normalize(radial));
            const sw::Vec3 zAxis = -up; // rocket +Z (tail) points down
            const sw::Vec3 reference =
                (std::abs(zAxis.y) < 0.99f) ? sw::Vec3{0, 1, 0} : sw::Vec3{1, 0, 0};
            const sw::Vec3 xAxis = glm::normalize(glm::cross(reference, zAxis));
            const sw::Vec3 yAxis = glm::cross(zAxis, xAxis);

            root = m_world.createEntity();
            TransformComponent transform{};
            transform.position = position;
            transform.rotation = glm::quat_cast(sw::Mat3{xAxis, yAxis, zAxis});
            m_world.addComponent(root, transform);
            m_world.addComponent(root, PreviousTransformComponent{transform.position,
                                                                  transform.rotation});
            m_world.addComponent(root, BoundsComponent{0.1f});
            m_world.addComponent(root, MapMarkerComponent{{0.4f, 0.9f, 1.0f, 1.0f}});
            m_world.addComponent(root, ShipComponent{});
            m_world.addComponent(root, ShipControlsComponent{});
            m_world.addComponent(root, SasComponent{});
            m_world.addComponent(root, sw::parts::VesselComponent{});
            // The air's answer, refreshed every tick. Its PRESENCE is
            // also the switch that turns the old isotropic drag off for
            // this vessel: a part-built craft is flown by its tables.
            m_world.addComponent(root, sw::aero::AeroStateComponent{});
            sw::phys::DynamicBodyComponent body{};
            body.velocity = gravity.worldVelocity +
                            glm::cross(gravity.angularVelocity, radial);
            body.mass = 1.0e4;
            m_world.addComponent(root, body);
            m_world.addComponent(root, hull);
        }
        else
        {
            // APPLY to the loaded vessel: tear out its old parts & joints.
            std::vector<sw::ecs::Entity> stale;
            m_world.forEach<sw::parts::PartComponent>(
                [&](sw::ecs::Entity entity, sw::parts::PartComponent& part) {
                    if (part.vessel == root)
                    {
                        stale.push_back(entity);
                    }
                });
            m_world.forEach<sw::parts::JointComponent>(
                [&](sw::ecs::Entity entity, sw::parts::JointComponent& joint) {
                    for (const sw::ecs::Entity part : stale)
                    {
                        if (joint.partA == part || joint.partB == part)
                        {
                            stale.push_back(entity);
                            return;
                        }
                    }
                });
            for (const sw::ecs::Entity entity : stale)
            {
                m_world.destroyEntity(entity);
            }
        }

        // Blueprint -> live part entities + joints (poses AND rotations).
        const auto& rootTransform = m_world.getComponent<TransformComponent>(root);
        std::vector<sw::ecs::Entity> spawned;
        for (const BlueprintPart& bp : m_blueprint)
        {
            const auto* definition = sw::parts::findDefinition(bp.definitionId);
            const sw::ecs::Entity part = m_world.createEntity();
            TransformComponent transform{};
            transform.position =
                rootTransform.position +
                sw::WorldVec3(rootTransform.rotation * bp.localPosition);
            transform.rotation = rootTransform.rotation * bp.localRotation;
            m_world.addComponent(part, transform);
            m_world.addComponent(part, PreviousTransformComponent{transform.position,
                                                                  transform.rotation});
            m_world.addComponent(part, BoundsComponent{
                                           sw::parts::partBoundsRadius(*definition)});
            m_world.addComponent(part, MeshComponent{m_partMeshIds.at(bp.definitionId)});
            attachPartAnimation(part, bp.definitionId);
            // THE SHELL IS GEOMETRY PER INSTANCE, which nothing else in this
            // game is: two fairing bases off the same catalogue entry carry
            // two different shells, so the mesh cannot come from the
            // definition's table and is built and registered right here.
            if (sw::parts::fairingIsFlying(bp.fairing))
            {
                m_world.addComponent(part, bp.fairing);
            }
            // A PART THAT CAN BEND CARRIES SOMEWHERE TO PUT THE BEND. Only
            // the ones whose author gave them a stiffness: everything else is
            // rigid, which is what every part in the game was before this
            // existed and what most of them should stay.
            if (definition->flexStiffnessNmPerRad > 0.0)
            {
                m_world.addComponent(part, sw::parts::PartFlexComponent{});
            }
            sw::parts::PartComponent component{};
            component.definitionId = bp.definitionId;
            component.vessel = root;
            component.localPosition = bp.localPosition;
            component.localRotation = bp.localRotation;
            m_world.addComponent(part, component);
            // ...AND THE SHELL IS WELDED ON AFTER THAT, because the mesh
            // builder asks the part which definition it is and a part that
            // does not have its PartComponent yet cannot answer. It crashed
            // exactly once, on the first design that shipped with a shell.
            if (sw::parts::fairingIsFlying(bp.fairing))
            {
                buildFairingShellMesh(part, bp.fairing);
            }
            // Rocket parts are solid too: you cannot walk through a fuel
            // tank, and a landed booster is furniture like anything else.
            {
                sw::phys::HullComponent hull{};
                if (hullFor(*definition, hull))
                {
                    m_world.addComponent(part, hull);
                }
            }
            if (definition->capacities[0].resource != sw::res::Resource::Count)
            {
                sw::factory::InventoryComponent inventory{};
                const auto resource = definition->capacities[0].resource;
                inventory.volumeCapacityM3 =
                    definition->capacities[0].units *
                    sw::res::definition(resource).volumePerUnitM3 * 1.02;
                if (definition->type == sw::parts::PartType::FuelTank)
                {
                    sw::factory::inventoryAdd(inventory, resource,
                                              definition->capacities[0].units);
                }
                m_world.addComponent(part, inventory);
            }
            spawned.push_back(part);
        }
        for (sw::usize i = 0; i < m_blueprint.size(); ++i)
        {
            const BlueprintPart& bp = m_blueprint[i];
            if (bp.parentIndex < 0)
            {
                continue;
            }
            const auto* a = sw::parts::findDefinition(
                m_blueprint[static_cast<sw::usize>(bp.parentIndex)].definitionId);
            const auto* b = sw::parts::findDefinition(bp.definitionId);
            const sw::f64 force = std::min(a->breakingForceN, b->breakingForceN);
            // Surface attachments (parentPoint 255) are radial by nature;
            // node attachments take the joint type of the parent node.
            const bool radial =
                bp.parentPoint == 255 ||
                (bp.parentPoint < a->nodes.size() &&
                 a->nodes[bp.parentPoint].type == sw::parts::NodeType::Radial);
            sw::parts::connectParts(
                m_world, spawned[static_cast<sw::usize>(bp.parentIndex)], spawned[i],
                bp.parentPoint, bp.childPoint,
                radial ? sw::parts::JointType::Radial : sw::parts::JointType::Stack,
                force, force);
        }
        return root;
    }

    // ------------------------------------------------------------------------
    // F5 — A DESIGN IS A FILE, AND A FILE IS A ROCKET
    //
    // The hangar's working list and the saved `.swship` record are the same
    // data seen twice: the editor's list carries the parent/child joint the
    // ghost snapped to, and the file carries it too. Convert both ways and a
    // design survives a restart with its structure intact — which matters,
    // because the joints are what a decoupler cuts and what breaks under
    // load. A pile of parts flying in formation is not a rocket.
    // ------------------------------------------------------------------------
    std::vector<StarWorksGame::BlueprintPart> StarWorksGame::partsFromDesign(
        const sw::parts::ShipBlueprint& design)
    {
        std::vector<BlueprintPart> parts;
        parts.reserve(design.parts.size());
        for (const sw::parts::BlueprintPartRecord& record : design.parts)
        {
            BlueprintPart part{};
            part.definitionId = record.definitionId;
            part.localPosition = record.localPosition;
            part.localRotation = record.localRotation;
            part.parentIndex = record.parentIndex;
            part.parentPoint = record.parentPoint;
            part.childPoint = record.childPoint;
            part.symmetryGroup = record.symmetryGroup;
            part.fairing = record.fairing;
            parts.push_back(part);
        }
        return parts;
    }

    sw::parts::ShipBlueprint StarWorksGame::designFromParts(std::string_view name) const
    {
        sw::parts::ShipBlueprint design{};
        design.name = std::string(name);
        design.parts.reserve(m_blueprint.size());
        for (const BlueprintPart& part : m_blueprint)
        {
            sw::parts::BlueprintPartRecord record{};
            record.definitionId = part.definitionId;
            record.localPosition = part.localPosition;
            record.localRotation = part.localRotation;
            record.parentIndex = part.parentIndex;
            record.parentPoint = part.parentPoint;
            record.childPoint = part.childPoint;
            record.symmetryGroup = part.symmetryGroup;
            record.fairing = part.fairing;
            design.parts.push_back(record);
        }
        return design;
    }

    std::string StarWorksGame::hangarSaveShip()
    {
        if (m_blueprint.empty())
        {
            SW_LOG_WARN("Game", "HANGAR: nothing to save");
            return {};
        }
        // The name is the one thing the hangar has no field for yet, so it
        // is derived: DESIGN 1, DESIGN 2... A rename UI is a text box, and a
        // text box is a whole input mode; the file is the important half and
        // it is on disk, editable, from this milestone on.
        std::string name;
        for (sw::u32 i = 1; i < 100; ++i)
        {
            name = std::format("DESIGN {}", i);
            if (sw::parts::findBlueprint(name) == nullptr)
            {
                break;
            }
        }
        const sw::parts::ShipBlueprint design = designFromParts(name);

        const std::filesystem::path directory =
            sw::FileSystem::executableDirectory() / "Assets" / "Ships";
        std::error_code error{};
        std::filesystem::create_directories(directory, error);
        std::string file;
        for (const char c : name)
        {
            file += (c == ' ') ? '_' : static_cast<char>(std::tolower(c));
        }
        const std::filesystem::path path = directory / (file + ".swship");
        if (!sw::parts::saveBlueprintFile(design, path))
        {
            SW_LOG_ERROR("Game", "HANGAR: could not save '{}'", path.string());
            return {};
        }
        // Registered as well as written: the point of saving is that you can
        // walk to the VAB and order it, now, without restarting the game.
        sw::parts::registerBlueprint(design);
        const sw::parts::BillOfMaterials bill = sw::parts::blueprintCost(design);
        SW_LOG_INFO("Game",
                    "HANGAR: saved '{}' ({} parts, {:.0f} kg iron, {:.0f} kg copper)",
                    name, design.parts.size(), bill.ironKg, bill.copperKg);
        return name;
    }

    void StarWorksGame::updateEditor()
    {
        // THE FAIRING TOOL TAKES THE FRAME. It owns the cursor, the left
        // button and the right one, and leaving the ghost running underneath
        // would put a part in your hand while you are drawing a shell with it.
        // The camera still orbits, because you draw a shroud by looking at it
        // from more than one side.
        updateFairing();
        // Hangar camera: right-drag orbits, wheel zooms.
        if (input().isMouseButtonDown(sw::MouseButton::Right))
        {
            m_hangarYaw -= input().mouseDeltaX() * 0.005f;
            m_hangarPitch = std::clamp(m_hangarPitch - input().mouseDeltaY() * 0.005f,
                                       -1.2f, 1.4f);
        }
        if (const sw::f32 scroll = input().scrollDeltaY(); scroll != 0.0f)
        {
            m_hangarDistance =
                std::clamp(m_hangarDistance * std::pow(1.15f, -scroll), 6.0f, 90.0f);
        }
        const sw::f32 cosPitch = std::cos(m_hangarPitch);
        const sw::Vec3 offset{cosPitch * std::sin(m_hangarYaw) * m_hangarDistance,
                              std::sin(m_hangarPitch) * m_hangarDistance,
                              cosPitch * std::cos(m_hangarYaw) * m_hangarDistance};
        m_hangarCamera.setPosition(sw::WorldVec3(offset));
        const sw::Vec3 forward = glm::normalize(-offset);
        const sw::Vec3 right = glm::normalize(glm::cross(forward, sw::Vec3{0, 1, 0}));
        const sw::Vec3 up = glm::cross(right, forward);
        m_hangarCamera.setOrientation(glm::quat_cast(sw::Mat3{right, up, -forward}));
        m_hangarCamera.setAspectRatio(renderer().aspectRatio());

        // ---- the hand -----------------------------------------------------------
        if (m_heldDefinition != 0)
        {
            // Rotate the held part in 90-degree steps (blueprint axes):
            // W/S pitch (X), A/D yaw (Y), Q/E roll (Z, the stack axis).
            const struct
            {
                sw::KeyCode key;
                sw::Vec3 axis;
                sw::f32 angle;
            } rotations[] = {
                {sw::KeyCode::W, {1, 0, 0}, 1.5707963f},
                {sw::KeyCode::S, {1, 0, 0}, -1.5707963f},
                {sw::KeyCode::A, {0, 1, 0}, 1.5707963f},
                {sw::KeyCode::D, {0, 1, 0}, -1.5707963f},
                {sw::KeyCode::Q, {0, 0, 1}, 1.5707963f},
                {sw::KeyCode::E, {0, 0, 1}, -1.5707963f},
            };
            for (const auto& rotation : rotations)
            {
                if (input().wasKeyPressed(rotation.key))
                {
                    m_heldRotation =
                        glm::angleAxis(rotation.angle, rotation.axis) * m_heldRotation;
                }
            }
            if (input().wasKeyPressed(sw::KeyCode::Escape))
            {
                // Put a grabbed subtree back exactly where it was; a fresh
                // palette part simply vanishes.
                if (!m_blueprintBackup.empty())
                {
                    m_blueprint = m_blueprintBackup;
                }
                m_heldDefinition = 0;
                m_heldSubtree.clear();
                m_blueprintBackup.clear();
            }
            if (input().wasKeyPressed(sw::KeyCode::Delete))
            {
                m_heldDefinition = 0; // discard (grab included: backup dropped)
                m_heldSubtree.clear();
                m_blueprintBackup.clear();
            }
        }
        if (input().wasKeyPressed(sw::KeyCode::X))
        {
            for (sw::usize i = 0; i < 6; ++i)
            {
                if (kSymmetryOptions[i] == m_symmetryCount)
                {
                    m_symmetryCount = kSymmetryOptions[(i + 1) % 6];
                    break;
                }
            }
        }
        if (input().wasKeyPressed(sw::KeyCode::C))
        {
            m_showCenters = !m_showCenters;
        }

        if (m_fairingPreviewDirty)
        {
            // OUTSIDE the draw-item collection: registering a mesh reallocates
            // m_meshes, and every DrawItem already collected holds a raw
            // pointer into it.
            rebuildFairingPreview();
        }
        if (m_fairingDrawing)
        {
            m_ghost = {};
            return;
        }
        computeGhost();
    }

    void StarWorksGame::collectHangarItems()
    {
        m_drawItems.clear();
        const sw::WorldVec3 cameraPosition = m_hangarCamera.position();
        const sw::Quat display = kHangarDisplay;
        auto toWorld = [&](const sw::Vec3& local) {
            return sw::WorldVec3(display * local);
        };
        auto pushPart = [&](sw::u32 definitionId, const sw::Vec3& position,
                            const sw::Quat& rotation, const sw::Vec4& tint,
                            bool transparent) {
            const auto* definition = sw::parts::findDefinition(definitionId);
            const auto meshIt = m_partMeshIds.find(definitionId);
            if (definition == nullptr || meshIt == m_partMeshIds.end())
            {
                return;
            }
            const sw::Vec3 relative = sw::Vec3(toWorld(position) - cameraPosition);
            sw::DrawItem item{};
            item.mesh = &m_meshes[meshIt->second];
            item.transform = glm::translate(sw::Mat4{1.0f}, relative) *
                             glm::mat4_cast(display * rotation);
            item.boundsCenter = relative;
            item.boundsRadius = sw::parts::partBoundsRadius(*definition) + 0.5f;
            item.tint = tint;
            item.transparent = transparent;
            m_drawItems.push_back(item);
            // ...AND EVERYTHING THAT MOVES, at its rest pose.
            //
            // The design office drew the STATIC mesh and nothing else, so an
            // animated part appeared here as whatever it has that does not
            // move. On a solar wing that is the twenty-centimetre mount block
            // and nothing at all of the array — you place a panel and see a
            // stub. It was invisible as a bug for the same reason it is
            // obvious as a symptom: the two shipped animated parts before now
            // were the Endurance's, which cannot be built in a hangar, and
            // the V-400, whose only moving shape is a glow cone that is dark
            // until the engine lights.
            //
            // The rest pose is the right pose: a part is BUILT stowed — a
            // wing folded in its fairing, a gear down on the pad — and the
            // motion transform at phase zero is the identity, so this is the
            // same matrix the body uses.
            if (const auto motions = m_partMotions.find(definitionId);
                motions != m_partMotions.end())
            {
                for (sw::u32 index = 0; index < motions->second.size(); ++index)
                {
                    const auto slot =
                        m_partGroupMeshIds.find(partGroupKey(definitionId, index));
                    if (slot == m_partGroupMeshIds.end())
                    {
                        continue;
                    }
                    sw::DrawItem moving = item;
                    moving.mesh = &m_meshes[slot->second];
                    m_drawItems.push_back(moving);
                }
            }
        };

        // Floor grid: the hangar deck.
        {
            sw::DrawItem floor{};
            floor.mesh = &m_meshes[m_hangarFloorMeshIndex];
            floor.transform = glm::translate(
                sw::Mat4{1.0f}, sw::Vec3(sw::WorldVec3{0.0, -14.0, 0.0} - cameraPosition));
            floor.boundsCenter = sw::Vec3(sw::WorldVec3{0.0, -14.0, 0.0} - cameraPosition);
            floor.boundsRadius = 60.0f;
            m_drawItems.push_back(floor);
        }

        // Placed parts.
        for (const BlueprintPart& bp : m_blueprint)
        {
            pushPart(bp.definitionId, bp.localPosition, bp.localRotation,
                     {1.0f, 1.0f, 1.0f, 1.0f}, false);
        }

        // Open STACK nodes: cyan diamonds (the magnet targets).
        for (const OpenAttachPoint& node : openAttachPoints())
        {
            const sw::Vec3 relative =
                sw::Vec3(toWorld(node.vesselPosition) - cameraPosition);
            sw::DrawItem marker{};
            marker.mesh = &m_meshes[m_markerMeshIndex];
            marker.transform = glm::translate(sw::Mat4{1.0f}, relative) *
                               glm::scale(sw::Mat4{1.0f}, sw::Vec3{0.30f});
            marker.boundsCenter = relative;
            marker.boundsRadius = 0.4f;
            marker.tint = {0.3f, 0.9f, 1.0f, 2.0f};
            m_drawItems.push_back(marker);
        }

        // The hand: ghost(s) of the held part (+ subtree, + symmetry clones).
        if (m_heldDefinition != 0)
        {
            const sw::Vec4 ghostTint =
                !m_ghost.active ? sw::Vec4{0.75f, 0.8f, 0.9f, 0.4f}
                : m_ghost.valid ? sw::Vec4{0.35f, 1.0f, 0.45f, 0.45f}
                                : sw::Vec4{1.0f, 0.25f, 0.2f, 0.5f};
            const sw::u32 cloneCount = m_ghost.active ? ghostCloneCount() : 1u;
            for (sw::u32 k = 0; k < cloneCount; ++k)
            {
                const sw::parts::PlacementPiece clone = sw::parts::symmetryClone(
                    {nullptr, m_ghost.position, m_ghost.rotation}, k, cloneCount);
                pushPart(m_heldDefinition, clone.position, clone.rotation, ghostTint,
                         true);
                for (const BlueprintPart& rel : m_heldSubtree)
                {
                    const sw::parts::PlacementPiece relClone = sw::parts::symmetryClone(
                        {nullptr, m_ghost.position + m_ghost.rotation * rel.localPosition,
                         m_ghost.rotation * rel.localRotation},
                        k, cloneCount);
                    pushPart(rel.definitionId, relClone.position, relClone.rotation,
                             ghostTint, true);
                }
            }
        }

        // Center of mass (yellow) and thrust centroid (violet, engines).
        if (m_showCenters && !m_blueprint.empty())
        {
            sw::f64 totalMass = 0.0;
            sw::Vec3 massMoment{0.0f};
            sw::f64 totalThrust = 0.0;
            sw::Vec3 thrustMoment{0.0f};
            for (const BlueprintPart& bp : m_blueprint)
            {
                const sw::f64 mass = partWetMassKg(bp.definitionId);
                totalMass += mass;
                massMoment += bp.localPosition * static_cast<sw::f32>(mass);
                const auto* definition = sw::parts::findDefinition(bp.definitionId);
                if (definition != nullptr && definition->thrustNewtons > 0.0)
                {
                    totalThrust += definition->thrustNewtons;
                    thrustMoment += bp.localPosition *
                                    static_cast<sw::f32>(definition->thrustNewtons);
                }
            }
            // Flags OUTSIDE the hull (a marker inside a tank would be depth-
            // hidden): diamond at x = sideX, thin pointer line toward the axis.
            auto pushCenter = [&](const sw::Vec3& position, const sw::Vec4& color,
                                  sw::f32 scale, sw::f32 sideX) {
                const sw::Vec3 flag{sideX, position.y, position.z};
                const sw::Vec3 relative = sw::Vec3(toWorld(flag) - cameraPosition);
                sw::DrawItem marker{};
                marker.mesh = &m_meshes[m_markerMeshIndex];
                marker.transform = glm::translate(sw::Mat4{1.0f}, relative) *
                                   glm::scale(sw::Mat4{1.0f}, sw::Vec3{scale});
                marker.boundsCenter = relative;
                marker.boundsRadius = scale * 1.5f;
                marker.tint = color;
                m_drawItems.push_back(marker);
                // Pointer line from the flag toward the exact point.
                const sw::Vec3 lineCenter = (flag + position) * 0.5f;
                const sw::Vec3 lineRelative =
                    sw::Vec3(toWorld(lineCenter) - cameraPosition);
                const sw::f32 halfLength = glm::length(flag - position) * 0.5f;
                sw::DrawItem line{};
                line.mesh = &m_meshes[m_navLineMeshIndex];
                line.transform =
                    glm::translate(sw::Mat4{1.0f}, lineRelative) *
                    glm::mat4_cast(display) *
                    glm::scale(sw::Mat4{1.0f},
                               sw::Vec3{std::max(halfLength, 0.1f), 0.03f, 0.03f});
                line.boundsCenter = lineRelative;
                line.boundsRadius = halfLength + 0.2f;
                line.tint = color * sw::Vec4{1.0f, 1.0f, 1.0f, 0.4f};
                line.transparent = true;
                m_drawItems.push_back(line);
            };
            if (totalMass > 0.0)
            {
                pushCenter(massMoment / static_cast<sw::f32>(totalMass),
                           {1.0f, 0.85f, 0.2f, 2.0f}, 0.5f, 4.2f);
            }
            if (totalThrust > 0.0)
            {
                pushCenter(thrustMoment / static_cast<sw::f32>(totalThrust),
                           {0.8f, 0.4f, 1.0f, 2.0f}, 0.42f, 5.0f);
            }
        }

        collectFairingPreview(cameraPosition, display);
        collectEditorUi();
        collectFairingUi();
    }

    // ---- hangar UI: title, clickable part palette, action row, stats ------
    void StarWorksGame::collectEditorUi()
    {
        // The hangar owns the whole button set for the frame (the SAS row
        // is a flight instrument and stays out of the hangar).
        hudBeginButtons(); // the hangar renders through its own path

        const sw::Vec4 titleColor{1.0f, 0.85f, 0.35f, 1.0f};
        const sw::Vec4 textColor{0.8f, 0.9f, 1.0f, 0.95f};
        hudText("HANGAR", -0.97f, -0.96f, 0.048f, titleColor);
        hudText(m_hangarSource.isNull() ? "MODE: NEW BUILD -> LAUNCH PAD"
                                        : "MODE: MODIFYING LOADED VESSEL",
                -0.97f, -0.885f, 0.036f, textColor);
        if (m_heldDefinition != 0)
        {
            const auto* held = sw::parts::findDefinition(m_heldDefinition);
            hudText(std::format("IN HAND: {}{}", held != nullptr ? held->name : "?",
                                m_heldSubtree.empty()
                                    ? ""
                                    : std::format(" +{} PARTS", m_heldSubtree.size())),
                    -0.97f, -0.825f, 0.032f, {0.6f, 1.0f, 0.7f, 1.0f});
            hudText("LCLICK PLACE  W/S/A/D/Q/E ROTATE  ESC PUT BACK  DEL DISCARD",
                    -0.97f, -0.77f, 0.026f, sw::Vec4{0.6f, 0.72f, 0.82f, 0.85f});
        }
        else
        {
            hudText("CLICK THE PALETTE FOR A NEW PART - CLICK A PLACED PART TO "
                    "GRAB ITS SUBTREE",
                    -0.97f, -0.825f, 0.026f, sw::Vec4{0.6f, 0.72f, 0.82f, 0.85f});
            hudText("B = EXIT WITHOUT BUILDING   X = SYMMETRY   C = CENTERS",
                    -0.97f, -0.775f, 0.026f, sw::Vec4{0.6f, 0.72f, 0.82f, 0.85f});
        }

        auto panel = [&](sw::f32 x0, sw::f32 y0, sw::f32 x1, sw::f32 y1,
                         const sw::Vec4& color) {
            sw::DrawItem item{};
            item.mesh = &m_meshes[m_navLineMeshIndex]; // unit quad
            item.transform =
                glm::translate(sw::Mat4{1.0f},
                               {(x0 + x1) * 0.5f, (y0 + y1) * 0.5f, 0.0f}) *
                glm::scale(sw::Mat4{1.0f},
                           {(x1 - x0) * 0.5f, (y1 - y0) * 0.5f, 1.0f});
            item.screenSpace = true;
            item.tint = color;
            m_drawItems.push_back(item);
        };

        // ---- part palette: sections, one open at a time ---------------------
        // Buildings share the catalogue since F1; they are placed on the
        // ground, not stacked in the VAB, so they are filtered out here. What
        // is left is sorted into shelves — press a header to open its shelf —
        // because a flat list of twenty-one parts was both hard to read and,
        // past the sixteenth row, impossible to click.
        const auto rows = paletteRows(m_paletteGroup);
        constexpr sw::f32 kRowStride = 0.070f;
        constexpr sw::f32 kRowHeight = 0.058f;
        constexpr sw::f32 kRowWidth = 0.40f;
        sw::f32 rowY = -0.70f;
        for (sw::usize i = 0; i < rows.size(); ++i)
        {
            const bool open = rows[i].header && rows[i].group == m_paletteGroup;
            const bool held = !rows[i].header && rows[i].part->id == m_heldDefinition;
            const sw::f32 x0 = rows[i].header ? -0.98f : -0.955f;
            panel(x0, rowY, -0.98f + kRowWidth, rowY + kRowHeight,
                  held  ? sw::Vec4{0.20f, 0.52f, 0.30f, 0.85f}
                  : rows[i].header
                      ? (open ? sw::Vec4{0.22f, 0.30f, 0.42f, 0.90f}
                              : sw::Vec4{0.10f, 0.14f, 0.20f, 0.75f})
                      : sw::Vec4{0.13f, 0.19f, 0.28f, 0.60f});
            const std::string label =
                rows[i].header
                    ? std::format("{} {}", open ? "-" : "+",
                                  kPaletteGroupNames[static_cast<sw::usize>(
                                      rows[i].group)])
                    : std::string(rows[i].part->name);
            hudText(label, x0 + 0.018f, rowY + 0.014f, rows[i].header ? 0.032f : 0.028f,
                    held         ? sw::Vec4{0.9f, 1.0f, 0.9f, 1.0f}
                    : rows[i].header ? sw::Vec4{0.85f, 0.92f, 1.0f, 1.0f}
                                     : sw::Vec4{0.68f, 0.78f, 0.88f, 0.9f});
            m_hudButtons.push_back({x0, rowY, -0.98f + kRowWidth, rowY + kRowHeight,
                                    100u + static_cast<sw::u32>(i)});
            rowY += kRowStride;
        }

        // ---- action row (bottom-center) -------------------------------------
        struct Action
        {
            const char* label;
            sw::u32 id;
            bool strong;
        };
        const std::string symLabel = std::format("SYM {}", m_symmetryCount);
        const Action actions[] = {
            {"UNDO", 201, false},          {"NEW", 202, false},
            {"LOAD", 203, false},          {symLabel.c_str(), 205, m_symmetryCount > 1},
            {m_showCenters ? "CG:ON" : "CG:OFF", 206, m_showCenters},
            // SAVE is the ONLY thing this room does to the world, and it
            // does it once, on this press — not on every part placed. What
            // it produces is a DESIGN: a `.swship` on disk, registered so a
            // VAB can be told to build it. The hangar itself has not made a
            // rocket since F9; the button that used to is gone, because a
            // drawing office that can also manufacture makes the factory
            // beside it decorative.
            {"SAVE", 207, true},
        };
        constexpr sw::f32 kButtonWidth = 0.135f;
        constexpr sw::f32 kButtonHeight = 0.072f;
        constexpr sw::f32 kButtonGap = 0.016f;
        sw::f32 buttonX = -0.54f;
        const sw::f32 buttonY = 0.86f;
        for (const Action& action : actions)
        {
            const sw::f32 x1 = buttonX + kButtonWidth;
            panel(buttonX, buttonY, x1, buttonY + kButtonHeight,
                  action.strong ? sw::Vec4{0.60f, 0.38f, 0.10f, 0.9f}
                                : sw::Vec4{0.16f, 0.24f, 0.34f, 0.75f});
            hudText(action.label, buttonX + 0.016f, buttonY + 0.019f, 0.032f,
                    sw::Vec4{0.92f, 0.96f, 1.0f, 1.0f});
            m_hudButtons.push_back({buttonX, buttonY, x1,
                                    buttonY + kButtonHeight, action.id});
            buttonX = x1 + kButtonGap;
        }

        // ---- blueprint stats (top-right) -------------------------------------
        sw::f64 wetMassKg = 0.0;
        sw::f64 costCredits = 0.0;
        for (const BlueprintPart& blueprintPart : m_blueprint)
        {
            wetMassKg += partWetMassKg(blueprintPart.definitionId);
            const auto* definition =
                sw::parts::findDefinition(blueprintPart.definitionId);
            costCredits += definition != nullptr ? definition->costCredits : 0.0;
            // A SHELL IS AS HEAVY AS IT IS BIG, and the whole point of drawing
            // one by hand is that the player can see that trade while drawing.
            if (sw::parts::fairingIsFlying(blueprintPart.fairing))
            {
                wetMassKg += sw::parts::fairingMassKg(blueprintPart.fairing);
                costCredits += sw::parts::fairingCostCredits(blueprintPart.fairing);
            }
        }
        hudText(std::format("WET MASS {:.1f} T  COST {:.0f}  PARTS {}",
                            wetMassKg / 1000.0, costCredits, m_blueprint.size()),
                0.16f, -0.95f, 0.034f, textColor);
    }
} // namespace game
