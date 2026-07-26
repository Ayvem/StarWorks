#pragma once

// ============================================================================
// Renderer/Vertex.hpp
// Vulkan vertex-input description for the engine's interleaved Vertex
// (defined CPU-side in Assets/MeshData.hpp). Kept next to the pipelines that
// consume it; asset code never sees Vulkan types.
// ============================================================================

#include "Assets/MeshData.hpp"

#include <vulkan/vulkan.h>

#include <array>

namespace sw::vulkan
{
    struct VertexInput
    {
        [[nodiscard]] static VkVertexInputBindingDescription binding()
        {
            VkVertexInputBindingDescription description{};
            description.binding = 0;
            description.stride = sizeof(Vertex);
            description.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
            return description;
        }

        [[nodiscard]] static std::array<VkVertexInputAttributeDescription, 4> attributes()
        {
            std::array<VkVertexInputAttributeDescription, 4> attrs{};
            attrs[0] = {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, position)};
            attrs[1] = {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, normal)};
            attrs[2] = {2, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(Vertex, color)};
            attrs[3] = {3, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(Vertex, uv)};
            return attrs;
        }
    };
} // namespace sw::vulkan
