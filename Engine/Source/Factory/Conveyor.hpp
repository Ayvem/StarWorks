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
#include "Planet/Terrain.hpp"

#include <algorithm>

namespace sw::factory
{
    inline constexpr u32 kMaxConveyorPoints = 16;

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
