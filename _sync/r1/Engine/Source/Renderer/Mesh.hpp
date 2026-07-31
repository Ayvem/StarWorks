#pragma once

// ============================================================================
// Renderer/Mesh.hpp
// GPU-resident mesh: a vertex buffer + an index buffer, uploaded once from
// a MeshData. Move-only; buffers are freed automatically. Created through
// Renderer::createMesh so game code never touches the memory layer.
// ============================================================================

#include "Assets/MeshData.hpp"
#include "Renderer/Vulkan/VulkanMemory.hpp"

namespace sw
{
    class Mesh
    {
    public:
        Mesh() = default;
        Mesh(vulkan::VulkanMemory& memory, const MeshData& data);

        Mesh(const Mesh&) = delete;
        Mesh& operator=(const Mesh&) = delete;
        Mesh(Mesh&&) noexcept = default;
        Mesh& operator=(Mesh&&) noexcept = default;

        [[nodiscard]] VkBuffer vertexBuffer() const { return m_vertexBuffer.handle(); }
        [[nodiscard]] VkBuffer indexBuffer() const { return m_indexBuffer.handle(); }
        [[nodiscard]] u32 indexCount() const { return m_indexCount; }
        [[nodiscard]] bool valid() const { return m_vertexBuffer.valid(); }

    private:
        vulkan::VulkanBuffer m_vertexBuffer;
        vulkan::VulkanBuffer m_indexBuffer;
        u32 m_indexCount = 0;
    };

    /// One draw request: a mesh and its world transform.
    /// (Will become an ECS component pair once the ECS module lands.)
    struct RenderObject
    {
        const Mesh* mesh = nullptr;
        Mat4 transform{1.0f};
    };
} // namespace sw
