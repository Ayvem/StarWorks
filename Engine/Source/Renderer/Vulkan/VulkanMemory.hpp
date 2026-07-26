#pragma once

// ============================================================================
// Renderer/Vulkan/VulkanMemory.hpp
// GPU memory layer built on VulkanMemoryAllocator (VMA).
//
//  - VulkanMemory owns the VmaAllocator and is the only entry point for
//    creating buffers/images. Everything it hands out is RAII.
//  - VulkanBuffer / VulkanImage are move-only owners of (handle, allocation).
//  - uploadToBuffer() performs the classic staging-buffer copy through a
//    one-shot command buffer. Batched/async uploads come with the Assets
//    streaming milestone; the call sites will not change.
// ============================================================================

#include "Core/Types.hpp"

#include <vulkan/vulkan.h>

// Forward declarations from VMA (vk_mem_alloc.h is included in .cpp only,
// to keep this header light and compile times sane).
VK_DEFINE_HANDLE(VmaAllocator)
VK_DEFINE_HANDLE(VmaAllocation)

namespace sw::vulkan
{
    class VulkanDevice;

    /// Move-only owning wrapper around a VkBuffer + its VMA allocation.
    class VulkanBuffer
    {
    public:
        VulkanBuffer() = default;
        VulkanBuffer(VmaAllocator allocator, VkBuffer buffer, VmaAllocation allocation,
                     VkDeviceSize size, void* mappedData);
        ~VulkanBuffer();

        VulkanBuffer(const VulkanBuffer&) = delete;
        VulkanBuffer& operator=(const VulkanBuffer&) = delete;
        VulkanBuffer(VulkanBuffer&& other) noexcept;
        VulkanBuffer& operator=(VulkanBuffer&& other) noexcept;

        [[nodiscard]] VkBuffer handle() const { return m_buffer; }
        [[nodiscard]] VkDeviceSize size() const { return m_size; }
        [[nodiscard]] bool valid() const { return m_buffer != VK_NULL_HANDLE; }

        /// Non-null only for persistently mapped (host-visible) buffers.
        [[nodiscard]] void* mappedData() const { return m_mappedData; }

        void reset();

    private:
        VmaAllocator m_allocator = nullptr;
        VkBuffer m_buffer = VK_NULL_HANDLE;
        VmaAllocation m_allocation = nullptr;
        VkDeviceSize m_size = 0;
        void* m_mappedData = nullptr;
    };

    /// Move-only owning wrapper around a VkImage + view + its VMA allocation.
    class VulkanImage
    {
    public:
        VulkanImage() = default;
        VulkanImage(VmaAllocator allocator, VkDevice device, VkImage image, VkImageView view,
                    VmaAllocation allocation, VkFormat format, VkExtent2D extent);
        ~VulkanImage();

        VulkanImage(const VulkanImage&) = delete;
        VulkanImage& operator=(const VulkanImage&) = delete;
        VulkanImage(VulkanImage&& other) noexcept;
        VulkanImage& operator=(VulkanImage&& other) noexcept;

        [[nodiscard]] VkImage handle() const { return m_image; }
        [[nodiscard]] VkImageView view() const { return m_view; }
        [[nodiscard]] VkFormat format() const { return m_format; }
        [[nodiscard]] VkExtent2D extent() const { return m_extent; }
        [[nodiscard]] bool valid() const { return m_image != VK_NULL_HANDLE; }

        void reset();

    private:
        VmaAllocator m_allocator = nullptr;
        VkDevice m_device = VK_NULL_HANDLE;
        VkImage m_image = VK_NULL_HANDLE;
        VkImageView m_view = VK_NULL_HANDLE;
        VmaAllocation m_allocation = nullptr;
        VkFormat m_format = VK_FORMAT_UNDEFINED;
        VkExtent2D m_extent{};
    };

    class VulkanMemory
    {
    public:
        VulkanMemory(VkInstance instance, const VulkanDevice& device);
        ~VulkanMemory();

        VulkanMemory(const VulkanMemory&) = delete;
        VulkanMemory& operator=(const VulkanMemory&) = delete;
        VulkanMemory(VulkanMemory&&) = delete;
        VulkanMemory& operator=(VulkanMemory&&) = delete;

        /// Device-local buffer (vertex/index/storage). Fill via uploadToBuffer.
        [[nodiscard]] VulkanBuffer createDeviceBuffer(VkDeviceSize size,
                                                      VkBufferUsageFlags usage);

        /// Host-visible, persistently mapped buffer (uniforms, staging).
        [[nodiscard]] VulkanBuffer createHostVisibleBuffer(VkDeviceSize size,
                                                           VkBufferUsageFlags usage);

        /// 2D attachment image (depth or color render target).
        [[nodiscard]] VulkanImage createAttachmentImage(VkExtent2D extent, VkFormat format,
                                                        VkImageUsageFlags usage,
                                                        VkImageAspectFlags aspect);

        /// Synchronous staging upload into a device-local buffer.
        void uploadToBuffer(const VulkanBuffer& target, const void* data, VkDeviceSize size);

    private:
        const VulkanDevice& m_device;
        VmaAllocator m_allocator = nullptr;
        /// Pool + fence for one-shot upload command buffers.
        VkCommandPool m_uploadPool = VK_NULL_HANDLE;
        VkFence m_uploadFence = VK_NULL_HANDLE;
    };
} // namespace sw::vulkan
