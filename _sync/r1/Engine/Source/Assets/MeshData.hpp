#pragma once

// ============================================================================
// Assets/MeshData.hpp
// CPU-side mesh representation. This is the exchange format between asset
// loaders / procedural generators and the renderer; it knows nothing about
// Vulkan. The GPU-side counterpart is Renderer/Mesh.
// ============================================================================

#include "Core/Types.hpp"
#include "Math/Math.hpp"

#include <vector>

namespace sw
{
    /// Interleaved vertex. Layout is mirrored by Renderer/Vertex.hpp and by
    /// the Mesh.vert shader — change all three together.
    struct Vertex
    {
        Vec3 position{0.0f};
        Vec3 normal{0.0f, 1.0f, 0.0f};
        Vec4 color{1.0f};
        Vec2 uv{0.0f};
    };

    struct MeshData
    {
        std::vector<Vertex> vertices;
        std::vector<u32> indices;

        [[nodiscard]] bool empty() const { return vertices.empty() || indices.empty(); }
    };
} // namespace sw
