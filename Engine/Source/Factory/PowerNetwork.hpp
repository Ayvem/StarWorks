#pragma once

// ============================================================================
// Factory/PowerNetwork.hpp
// WHO IS ON THE SAME GRID AS WHOM.
//
// F3 shipped with one grid per SITE, which was a placeholder and read like
// one: every building you planted anywhere near the hub was silently wired
// to it. Electricity is a network, and a network you cannot see or route is
// not a mechanic — it is a rule.
//
// So the grid is now the CABLES. A cable is a real object between two power
// nodes; a grid is a connected component of that graph; and a solar farm you
// forgot to hook up powers nothing, visibly, because there is no wire.
//
// Two rules give the network its shape, and they are the whole design:
//
//   * A BUILDING takes ONE cable. That is what makes routing a decision —
//     you cannot daisy-chain your factory, you have to distribute it.
//   * A POLE takes as many as you like. That is what makes the pole worth
//     building, and it puts the layout of your grid in the player's hands
//     rather than in an adjacency rule they cannot see.
//
// Everything here is pure integer work over indices: no world, no
// components, no entities. The union-find below is what `rebuildPowerNetwork`
// runs after every build and every demolition, and what the tests exercise
// without a scene.
// ============================================================================

#include "Core/Types.hpp"
#include "Factory/Recipes.hpp"

#include <algorithm>
#include <span>
#include <vector>

namespace sw::factory
{
    /// A cable, as the graph sees it: two node indices.
    struct PowerLink
    {
        u32 a = 0;
        u32 b = 0;
    };

    /// How many cables may meet at one building of this category.
    ///
    /// A pole is the ONLY thing that branches. Everything else gets one wire,
    /// which is why a factory needs a distribution layout instead of a chain.
    [[nodiscard]] inline u32 maxCablesFor(BuildingCategory category)
    {
        return (category == BuildingCategory::Pole) ? 0xFFFFFFFFu : 1u;
    }

    [[nodiscard]] inline bool isPowerHub(BuildingCategory category)
    {
        return category == BuildingCategory::Pole;
    }

    /// Assigns every node a GRID id: nodes joined by cables share one, and a
    /// node with no cable at all gets an id of its own.
    ///
    /// Ids are the smallest node index in each component, so they are stable
    /// against link ORDER — rebuild the network after demolishing an
    /// unrelated cable on the far side of the base and the ids you were
    /// already looking at do not shuffle.
    ///
    /// Links referring to nodes that do not exist are ignored rather than
    /// trusted: the caller builds this list from entities, and an entity can
    /// die between one rebuild and the next.
    [[nodiscard]] inline std::vector<u32> traceGrids(usize nodeCount,
                                                     std::span<const PowerLink> links)
    {
        std::vector<u32> parent(nodeCount);
        for (usize i = 0; i < nodeCount; ++i)
        {
            parent[i] = static_cast<u32>(i);
        }
        // Iterative find with full path compression: a base is a few hundred
        // nodes and this runs on a build click, but recursion here would be
        // a stack depth proportional to the player's cable run.
        auto find = [&parent](u32 node) {
            u32 root = node;
            while (parent[root] != root)
            {
                root = parent[root];
            }
            while (parent[node] != root)
            {
                const u32 next = parent[node];
                parent[node] = root;
                node = next;
            }
            return root;
        };

        for (const PowerLink& link : links)
        {
            if (link.a >= nodeCount || link.b >= nodeCount || link.a == link.b)
            {
                continue;
            }
            const u32 rootA = find(link.a);
            const u32 rootB = find(link.b);
            if (rootA == rootB)
            {
                continue;
            }
            // Union by SMALLEST index, which is what makes the id stable.
            if (rootA < rootB)
            {
                parent[rootB] = rootA;
            }
            else
            {
                parent[rootA] = rootB;
            }
        }

        std::vector<u32> grid(nodeCount, 0);
        for (usize i = 0; i < nodeCount; ++i)
        {
            grid[i] = find(static_cast<u32>(i));
        }
        return grid;
    }

    /// How many cables already meet at each node.
    [[nodiscard]] inline std::vector<u32> countCables(usize nodeCount,
                                                      std::span<const PowerLink> links)
    {
        std::vector<u32> degree(nodeCount, 0);
        for (const PowerLink& link : links)
        {
            if (link.a >= nodeCount || link.b >= nodeCount || link.a == link.b)
            {
                continue;
            }
            degree[link.a] += 1;
            degree[link.b] += 1;
        }
        return degree;
    }

    /// Why a cable may not be laid. `Ok` means lay it.
    enum class CableVerdict : u8
    {
        Ok = 0,
        SameNode,     // both ends on the same thing
        AlreadyWired, // ...and already on the same grid: the cable would do nothing
        EndpointFull, // a building already has its one cable
        NoPowerNode,  // this thing has nothing to hook a wire to
        TooLong,
        Count
    };

    [[nodiscard]] inline std::string_view cableVerdictText(CableVerdict verdict)
    {
        switch (verdict)
        {
        case CableVerdict::Ok:
            return "OK";
        case CableVerdict::SameNode:
            return "SAME BUILDING";
        case CableVerdict::AlreadyWired:
            return "ALREADY ON THIS GRID";
        case CableVerdict::EndpointFull:
            return "ONE CABLE PER BUILDING - USE A POLE";
        case CableVerdict::NoPowerNode:
            return "NO POWER CONNECTION";
        case CableVerdict::TooLong:
            return "CABLE TOO LONG";
        default:
            return "REFUSED";
        }
    }

    /// The whole placement rule for one cable, as an answer the HUD can show
    /// and the commit can trust — the same function for both, so the green
    /// you see and the cable you get cannot disagree.
    [[nodiscard]] inline CableVerdict validateCable(bool hasNodeA, bool hasNodeB,
                                                    BuildingCategory categoryA,
                                                    BuildingCategory categoryB,
                                                    u32 cablesOnA, u32 cablesOnB,
                                                    u32 gridA, u32 gridB, f64 lengthM,
                                                    f64 maxLengthM)
    {
        if (!hasNodeA || !hasNodeB)
        {
            return CableVerdict::NoPowerNode;
        }
        if (gridA == gridB && cablesOnA > 0 && cablesOnB > 0)
        {
            // Already connected THROUGH something. A second path adds
            // nothing but a wire to trip over.
            return CableVerdict::AlreadyWired;
        }
        if (cablesOnA >= maxCablesFor(categoryA) ||
            cablesOnB >= maxCablesFor(categoryB))
        {
            return CableVerdict::EndpointFull;
        }
        if (lengthM > maxLengthM)
        {
            return CableVerdict::TooLong;
        }
        return CableVerdict::Ok;
    }

    /// Where a hanging cable is at fraction `t` of the way across.
    ///
    /// A straight line between two poles reads as a girder, not a wire. Real
    /// spans hang, and the sag is what makes the shape legible at a glance —
    /// so the curve is a parabola (the small-sag limit of a catenary, and
    /// indistinguishable from one at the sags a base uses) with a droop
    /// proportional to the span.
    [[nodiscard]] inline WorldVec3 cablePointAt(const WorldVec3& from,
                                                const WorldVec3& to, const Vec3& up,
                                                f64 sagFraction, f64 t)
    {
        const f64 clamped = std::clamp(t, 0.0, 1.0);
        const WorldVec3 straight = from + (to - from) * clamped;
        const f64 span = glm::length(to - from);
        // 4t(1-t) peaks at 1 in the middle and is 0 at both ends, so the
        // curve meets the two nodes exactly wherever they are.
        const f64 droop = 4.0 * clamped * (1.0 - clamped) * span * sagFraction;
        return straight - WorldVec3(glm::normalize(up)) * droop;
    }
} // namespace sw::factory
