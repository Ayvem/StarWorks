#pragma once

// ============================================================================
// Scene/TransformComponents.hpp
// Engine-level spatial components, shared by Physics, Scene and game code.
// (Moved from the game layer when the Physics module arrived: engine systems
// integrate positions, so the engine must own the transform type.)
//
// Positions are f64 world space (real astronomical scale); rotation stays
// f32 (orientation needs no astronomical precision); scale is uniform.
// ============================================================================

#include "Math/Math.hpp"

#include <type_traits>

namespace sw
{
    struct TransformComponent
    {
        WorldVec3 position{0.0};
        Quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        /// Uniform scale in meters (meshes are authored at unit size).
        f32 uniformScale = 1.0f;
    };

    /// Snapshot of the transform at the previous simulation step, used by
    /// rendering to interpolate. For non-interpolated (on-rails) entities it
    /// simply mirrors the current transform.
    struct PreviousTransformComponent
    {
        WorldVec3 position{0.0};
        Quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    };

    static_assert(std::is_trivially_copyable_v<TransformComponent>);
    static_assert(std::is_trivially_copyable_v<PreviousTransformComponent>);
} // namespace sw
