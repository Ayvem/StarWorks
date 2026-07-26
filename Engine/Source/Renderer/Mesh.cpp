#include "Renderer/Mesh.hpp"

#include "Core/Error.hpp"

namespace sw
{
    Mesh::Mesh(vulkan::VulkanMemory& memory, const MeshData& data)
    {
        if (data.empty())
        {
            SW_THROW("Cannot create a GPU mesh from empty MeshData");
        }

        const VkDeviceSize vertexBytes = data.vertices.size() * sizeof(Vertex);
        const VkDeviceSize indexBytes = data.indices.size() * sizeof(u32);

        m_vertexBuffer = memory.createDeviceBuffer(vertexBytes,
                                                   VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
        m_indexBuffer = memory.createDeviceBuffer(indexBytes, VK_BUFFER_USAGE_INDEX_BUFFER_BIT);

        memory.uploadToBuffer(m_vertexBuffer, data.vertices.data(), vertexBytes);
        memory.uploadToBuffer(m_indexBuffer, data.indices.data(), indexBytes);

        m_indexCount = static_cast<u32>(data.indices.size());
    }
} // namespace sw
