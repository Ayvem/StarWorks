#pragma once

// ============================================================================
// Factory/Conveyor.hpp
// The GEOMETRY of a belt: where a conveyor's deck runs between two points on
// a body's surface.
//
// A belt is not a straight line in space. It is a line on a SPHERE whose
// height at every step comes from the same analytic heightfield the collider
// and the renderer already share — otherwise a belt laid across a rise
// disappears into it, and one laid across a dip hangs in the air. Sampling
// the terrain is the whole job; the rest is arc length.
//
// It lives in the engine, and not in the scene code that happens to call it
// first, because F2 lets the player draw these by hand and F6 turns them
// into real transport. Both need the same answer to "where does the deck
// go", and a second implementation would be a belt that renders somewhere
// its items are not.
//
// Points come back in the body's ROTATING frame, metres from the body
// centre — the same convention as SurfaceAnchorComponent, so a belt is
// anchored and saved exactly like the buildings it joins.
// ============================================================================

#include "Core/Types.hpp"
#include "ECS/Entity.hpp"
#include "Planet/Terrain.hpp"

#include <algorithm>
#include <span>
#include <vector>

namespace sw::factory
{
    inline constexpr u32 kMaxConveyorPoints = 16;

    /// How close two conveyor mouths must be to count as joined, in metres.
    ///
    /// It has to be generous — these are placed by eye on rough ground, and a
    /// belt that refuses to connect for want of twenty centimetres reads as
    /// broken rather than as strict. But it MUST stay under the length of one
    /// segment, or demolishing a tile out of the middle of a run leaves a gap
    /// the snap bridges anyway, and the run keeps conducting through a hole
    /// you can see straight through. 1.5 m against the shipped 2 m CV-1.
    inline constexpr f64 kConveyorPortSnapM = 1.5;

    /// Lays a deck from `fromLocal` to `toLocal` (both body-frame positions,
    /// metres from the centre) across `count` points, each riding
    /// `clearanceM` above the ground under it.
    ///
    /// Returns the deck's total length. `count` is clamped to
    /// [2, kMaxConveyorPoints]; `outPoints` must hold that many.
    [[nodiscard]] inline f64 buildConveyorPath(const planet::TerrainComponent& terrain,
                                               f64 bodyRadiusM,
                                               const WorldVec3& fromLocal,
                                               const WorldVec3& toLocal, f64 clearanceM,
                                               WorldVec3* outPoints, u32& count)
    {
        count = std::clamp(count, 2u, kMaxConveyorPoints);
        const Vec3 from = glm::normalize(Vec3(fromLocal));
        const Vec3 to = glm::normalize(Vec3(toLocal));

        for (u32 i = 0; i < count; ++i)
        {
            const f32 t = static_cast<f32>(i) / static_cast<f32>(count - 1);
            // Normalising the lerp walks the great circle without a slerp:
            // over the tens of metres a belt spans the two are identical to
            // far below a millimetre, and this one cannot go singular.
            const Vec3 direction = glm::normalize(glm::mix(from, to, t));
            outPoints[i] = WorldVec3(direction) *
                           (bodyRadiusM + planet::terrainElevation(terrain, direction) +
                            clearanceM);
        }

        f64 length = 0.0;
        for (u32 i = 0; i + 1 < count; ++i)
        {
            length += glm::length(outPoints[i + 1] - outPoints[i]);
        }
        return length;
    }

    // ---- the network, derived from where the ports are ---------------------
    //
    // A belt segment is an ordinary building. What turns a ROW of them into a
    // working link is not an intention the game recorded — it is that their
    // conveyor-out and conveyor-in ports MEET. So the network is derived,
    // never stored: demolish a segment in the middle of a run and the chain
    // simply is not there next time, because the ports no longer meet.
    //
    // Pure graph work, in the body frame, so it can be tested without a
    // world, a renderer or a planet.

    /// One machine's conveyor mouths, resolved into the body frame.
    /// Most mouths a machine may have of one kind. Four, to match
    /// kMaxRecipeIngredients: a machine never needs more mouths than its
    /// recipe has products.
    inline constexpr u32 kMaxMachinePorts = 4;

    struct PortNode
    {
        ecs::Entity entity{};
        bool isBelt = false; // a segment, i.e. a link in a chain
        WorldVec3 centre{0.0};
        /// EVERY mouth, in authored order. A machine with two outputs can
        /// put hydrogen on one belt and oxygen on another, which is the
        /// difference between a fuel chain you can lay out and one where
        /// both gases are stuck sharing a single run.
        WorldVec3 outPorts[kMaxMachinePorts]{};
        WorldVec3 inPorts[kMaxMachinePorts]{};
        u32 outCount = 0;
        u32 inCount = 0;
    };

    /// A complete run: a machine's OUT MOUTH, some number of belts, another
    /// machine's IN MOUTH. The mouth indices matter — which product leaves
    /// by which belt is decided by which port it left from.
    struct Chain
    {
        u32 source = 0;      // index into the node list
        u32 sourcePort = 0;  // which of the source's out mouths
        u32 destination = 0;
        u32 destinationPort = 0;
        std::vector<u32> belts; // in travel order, may be empty
    };

    /// Every complete chain in the layout. A chain must START and END at a
    /// non-belt machine; stubs, runs into nothing, and loops are dropped.
    ///
    /// `snapM` is how close two mouths must be to count as joined. It is
    /// generous on purpose: these are placed by eye on rough ground, and a
    /// belt that refuses to connect for want of twenty centimetres reads as
    /// broken rather than as strict.
    [[nodiscard]] inline std::vector<Chain> traceConveyorChains(
        std::span<const PortNode> nodes, f64 snapM)
    {
        /// One end of a hop: which node, and which of its in mouths.
        struct PortLink
        {
            i32 node = -1;
            u32 port = 0;
        };

        // An IN MOUTH TAKES ONE BELT. Two runs arriving at the same mouth
        // would be two links into one hole; claiming as we trace is also
        // what makes a machine's second in mouth USEFUL, because the second
        // run is pushed onto it instead of piling onto the first.
        std::vector<u8> claimed(nodes.size() * kMaxMachinePorts, 0);
        auto claimIndex = [](u32 node, u32 port) { return node * kMaxMachinePorts + port; };

        // Which IN mouth does this OUT mouth feed? The nearest free one.
        auto feeds = [&](u32 from, u32 fromPort) -> PortLink {
            if (fromPort >= nodes[from].outCount)
            {
                return {};
            }
            PortLink best{};
            f64 bestDistance = snapM;
            for (u32 i = 0; i < nodes.size(); ++i)
            {
                if (i == from)
                {
                    continue;
                }
                for (u32 port = 0; port < nodes[i].inCount; ++port)
                {
                    if (claimed[claimIndex(i, port)] != 0)
                    {
                        continue;
                    }
                    const f64 distance =
                        glm::length(nodes[i].inPorts[port] - nodes[from].outPorts[fromPort]);
                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        best = {static_cast<i32>(i), port};
                    }
                }
            }
            return best;
        };

        std::vector<Chain> chains;
        for (u32 start = 0; start < nodes.size(); ++start)
        {
            if (nodes[start].isBelt)
            {
                continue;
            }
            for (u32 startPort = 0; startPort < nodes[start].outCount; ++startPort)
            {
                Chain chain{};
                chain.source = start;
                chain.sourcePort = startPort;

                std::vector<u32> taken; // rolled back if the run leads nowhere
                PortLink cursor = feeds(start, startPort);
                // The guard is a cycle breaker: a ring of segments feeding
                // each other is a legal thing to BUILD and must not be an
                // infinite loop to trace.
                u32 guard = 0;
                bool looped = false;
                while (cursor.node >= 0 && nodes[static_cast<u32>(cursor.node)].isBelt)
                {
                    const u32 belt = static_cast<u32>(cursor.node);
                    if (std::find(chain.belts.begin(), chain.belts.end(), belt) !=
                            chain.belts.end() ||
                        guard++ > 4096)
                    {
                        looped = true;
                        break;
                    }
                    chain.belts.push_back(belt);
                    taken.push_back(claimIndex(belt, cursor.port));
                    claimed[claimIndex(belt, cursor.port)] = 1;
                    cursor = feeds(belt, 0); // a belt has one mouth each way
                }
                if (looped || cursor.node < 0 ||
                    nodes[static_cast<u32>(cursor.node)].isBelt ||
                    static_cast<u32>(cursor.node) == start)
                {
                    // A stub, a ring, a run that leads nowhere, or a machine
                    // feeding itself. Give the mouths back.
                    for (const u32 index : taken)
                    {
                        claimed[index] = 0;
                    }
                    continue;
                }
                chain.destination = static_cast<u32>(cursor.node);
                chain.destinationPort = cursor.port;
                claimed[claimIndex(chain.destination, chain.destinationPort)] = 1;
                chains.push_back(std::move(chain));
            }
        }
        return chains;
    }

    /// Position and direction at `arcLength` metres along a deck, wrapping
    /// at the end. This is how cargo is placed: a pure function of distance
    /// travelled, so it is exact under time warp and costs nothing when
    /// nobody is looking at the belt.
    inline void conveyorPointAt(const WorldVec3* points, u32 count, f64 arcLength,
                                WorldVec3& outPosition, Vec3& outHeading)
    {
        outPosition = (count > 0) ? points[0] : WorldVec3{0.0};
        outHeading = Vec3{0.0f, 0.0f, 1.0f};
        if (count < 2)
        {
            return;
        }
        f64 remaining = std::max(0.0, arcLength);
        for (u32 i = 0; i + 1 < count; ++i)
        {
            const WorldVec3 step = points[i + 1] - points[i];
            const f64 segment = glm::length(step);
            if (remaining <= segment || i + 2 == count)
            {
                const f64 t =
                    (segment > 1.0e-9) ? std::clamp(remaining / segment, 0.0, 1.0) : 0.0;
                outPosition = points[i] + step * t;
                outHeading = Vec3(step / std::max(segment, 1.0e-9));
                return;
            }
            remaining -= segment;
        }
    }
} // namespace sw::factory
