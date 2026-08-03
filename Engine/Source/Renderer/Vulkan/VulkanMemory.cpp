#include "Renderer/Vulkan/VulkanMemory.hpp"

#include "Core/Assert.hpp"
#include "Core/Log.hpp"
#include "Renderer/Vulkan/VulkanCommon.hpp"
#include "Renderer/Vulkan/VulkanDevice.hpp"

#if defined(__GNUC__) && !defined(__clang__)
    // VMA's header defines small helpers not used by every including TU.
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include <vk_mem_alloc.h>
#if defined(__GNUC__) && !defined(__clang__)
    #pragma GCC diagnostic pop
#endif

#include <cstring>
#include <utility>

namespace sw::vulkan
{
    namespace
    {
        constexpr const char* kLogCat = "VulkanMemory";
    } // namespace

    // ------------------------------------------------------------------------
    // VulkanBuffer
    // ------------------------------------------------------------------------
    VulkanBuffer::VulkanBuffer(VmaAllocator allocator, VkBuffer buffer, VmaAllocation allocation,
                               VkDeviceSize size, void* mappedData)
        : m_allocator(allocator)
        , m_buffer(buffer)
        , m_allocation(allocation)
        , m_size(size)
        , m_mappedData(mappedData)
    {
    }

    VulkanBuffer::~VulkanBuffer()
    {
        reset();
    }

    VulkanBuffer::VulkanBuffer(VulkanBuffer&& other) noexcept
        : m_allocator(std::exchange(other.m_allocator, nullptr))
        , m_buffer(std::exchange(other.m_buffer, VK_NULL_HANDLE))
        , m_allocation(std::exchange(other.m_allocation, nullptr))
        , m_size(std::exchange(other.m_size, 0))
        , m_mappedData(std::exchange(other.m_mappedData, nullptr))
    {
    }

    VulkanBuffer& VulkanBuffer::operator=(VulkanBuffer&& other) noexcept
    {
        if (this != &other)
        {
            reset();
            m_allocator = std::exchange(other.m_allocator, nullptr);
            m_buffer = std::exchange(other.m_buffer, VK_NULL_HANDLE);
            m_allocation = std::exchange(other.m_allocation, nullptr);
            m_size = std::exchange(other.m_size, 0);
            m_mappedData = std::exchange(other.m_mappedData, nullptr);
        }
        return *this;
    }

    void VulkanBuffer::reset()
    {
        if (m_buffer != VK_NULL_HANDLE)
        {
            vmaDestroyBuffer(m_allocator, m_buffer, m_allocation);
            m_buffer = VK_NULL_HANDLE;
            m_allocation = nullptr;
            m_size = 0;
            m_mappedData = nullptr;
        }
    }

    // ------------------------------------------------------------------------
    // VulkanImage
    // ------------------------------------------------------------------------
    VulkanImage::VulkanImage(VmaAllocator allocator, VkDevice device, VkImage image,
                             VkImageView view, VmaAllocation allocation, VkFormat format,
                             VkExtent2D extent)
        : m_allocator(allocator)
        , m_device(device)
        , m_image(image)
        , m_view(view)
        , m_allocation(allocation)
        , m_format(format)
        , m_extent(extent)
    {
    }

    VulkanImage::~VulkanImage()
    {
        reset();
    }

    VulkanImage::VulkanImage(VulkanImage&& other) noexcept
        : m_allocator(std::exchange(other.m_allocator, nullptr))
        , m_device(std::exchange(other.m_device, VK_NULL_HANDLE))
        , m_image(std::exchange(other.m_image, VK_NULL_HANDLE))
        , m_view(std::exchange(other.m_view, VK_NULL_HANDLE))
        , m_allocation(std::exchange(other.m_allocation, nullptr))
        , m_format(std::exchange(other.m_format, VK_FORMAT_UNDEFINED))
        , m_extent(std::exchange(other.m_extent, VkExtent2D{}))
    {
    }

    VulkanImage& VulkanImage::operator=(VulkanImage&& other) noexcept
    {
        if (this != &other)
        {
            reset();
            m_allocator = std::exchange(other.m_allocator, nullptr);
            m_device = std::exchange(other.m_device, VK_NULL_HANDLE);
            m_image = std::exchange(other.m_image, VK_NULL_HANDLE);
            m_view = std::exchange(other.m_view, VK_NULL_HANDLE);
            m_allocation = std::exchange(other.m_allocation, nullptr);
            m_format = std::exchange(other.m_format, VK_FORMAT_UNDEFINED);
            m_extent = std::exchange(other.m_extent, VkExtent2D{});
        }
        return *this;
    }

    void VulkanImage::reset()
    {
        if (m_view != VK_NULL_HANDLE)
        {
            vkDestroyImageView(m_device, m_view, nullptr);
            m_view = VK_NULL_HANDLE;
        }
        if (m_image != VK_NULL_HANDLE)
        {
            vmaDestroyImage(m_allocator, m_image, m_allocation);
            m_image = VK_NULL_HANDLE;
            m_allocation = nullptr;
        }
    }

    // ------------------------------------------------------------------------
    // VulkanMemory
    // ------------------------------------------------------------------------
    VulkanMemory::VulkanMemory(VkInstance instance, const VulkanDevice& device)
        : m_device(device)
    {
        VmaAllocatorCreateInfo allocatorInfo{};
        allocatorInfo.instance = instance;
        allocatorInfo.physicalDevice = device.physicalHandle();
        allocatorInfo.device = device.handle();
        allocatorInfo.vulkanApiVersion = VK_API_VERSION_1_3;
        SW_VK_CHECK(vmaCreateAllocator(&allocatorInfo, &m_allocator));

        VkCommandPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        poolInfo.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
        poolInfo.queueFamilyIndex = device.graphicsFamilyIndex();
        SW_VK_CHECK(vkCreateCommandPool(device.handle(), &poolInfo, nullptr, &m_uploadPool));

        VkFenceCreateInfo fenceInfo{};
        fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        SW_VK_CHECK(vkCreateFence(device.handle(), &fenceInfo, nullptr, &m_uploadFence));

        SW_LOG_INFO(kLogCat, "VMA allocator created");
    }

    VulkanMemory::~VulkanMemory()
    {
        if (m_uploadFence != VK_NULL_HANDLE)
        {
            vkDestroyFence(m_device.handle(), m_uploadFence, nullptr);
        }
        if (m_uploadPool != VK_NULL_HANDLE)
        {
            vkDestroyCommandPool(m_device.handle(), m_uploadPool, nullptr);
        }
        if (m_allocator != nullptr)
        {
            vmaDestroyAllocator(m_allocator);
            m_allocator = nullptr;
        }
    }

    VulkanBuffer VulkanMemory::createDeviceBuffer(VkDeviceSize size, VkBufferUsageFlags usage)
    {
        VkBufferCreateInfo bufferInfo{};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        VmaAllocationCreateInfo allocInfo{};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

        VkBuffer buffer = VK_NULL_HANDLE;
        VmaAllocation allocation = nullptr;
        SW_VK_CHECK(vmaCreateBuffer(m_allocator, &bufferInfo, &allocInfo, &buffer, &allocation,
                                    nullptr));
        return {m_allocator, buffer, allocation, size, nullptr};
    }

    VulkanBuffer VulkanMemory::createHostVisibleBuffer(VkDeviceSize size,
                                                       VkBufferUsageFlags usage)
    {
        VkBufferCreateInfo bufferInfo{};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = usage;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        VmaAllocationCreateInfo allocInfo{};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                          VMA_ALLOCATION_CREATE_MAPPED_BIT;

        VkBuffer buffer = VK_NULL_HANDLE;
        VmaAllocation allocation = nullptr;
        VmaAllocationInfo resultInfo{};
        SW_VK_CHECK(vmaCreateBuffer(m_allocator, &bufferInfo, &allocInfo, &buffer, &allocation,
                                    &resultInfo));
        return {m_allocator, buffer, allocation, size, resultInfo.pMappedData};
    }

    VulkanImage VulkanMemory::createAttachmentImage(VkExtent2D extent, VkFormat format,
                                                    VkImageUsageFlags usage,
                                                    VkImageAspectFlags aspect)
    {
        VkImageCreateInfo imageInfo{};
        imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.format = format;
        imageInfo.extent = {extent.width, extent.height, 1};
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.usage = usage;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

        VmaAllocationCreateInfo allocInfo{};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
        allocInfo.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT; // attachments

        VkImage image = VK_NULL_HANDLE;
        VmaAllocation allocation = nullptr;
        SW_VK_CHECK(
            vmaCreateImage(m_allocator, &imageInfo, &allocInfo, &image, &allocation, nullptr));

        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = image;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = format;
        viewInfo.subresourceRange.aspectMask = aspect;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.layerCount = 1;

        VkImageView view = VK_NULL_HANDLE;
        SW_VK_CHECK(vkCreateImageView(m_device.handle(), &viewInfo, nullptr, &view));

        return {m_allocator, m_device.handle(), image, view, allocation, format, extent};
    }

    void VulkanMemory::uploadToBuffer(const VulkanBuffer& target, const void* data,
                                      VkDeviceSize size)
    {
        SW_ASSERT(size <= target.size(), "Upload of {} bytes into a {}-byte buffer", size,
                  target.size());

        // ---- staging buffer -------------------------------------------------
        VulkanBuffer staging =
            createHostVisibleBuffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
        std::memcpy(staging.mappedData(), data, static_cast<usize>(size));

        // ---- one-shot copy ---------------------------------------------------
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = m_uploadPool;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = 1;

        VkCommandBuffer cmd = VK_NULL_HANDLE;
        SW_VK_CHECK(vkAllocateCommandBuffers(m_device.handle(), &allocInfo, &cmd));

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        SW_VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo));

        VkBufferCopy copyRegion{};
        copyRegion.size = size;
        vkCmdCopyBuffer(cmd, staging.handle(), target.handle(), 1, &copyRegion);

        SW_VK_CHECK(vkEndCommandBuffer(cmd));

        VkCommandBufferSubmitInfo cmdInfo{};
        cmdInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
        cmdInfo.commandBuffer = cmd;

        VkSubmitInfo2 submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
        submitInfo.commandBufferInfoCount = 1;
        submitInfo.pCommandBufferInfos = &cmdInfo;

        SW_VK_CHECK(vkQueueSubmit2(m_device.graphicsQueue(), 1, &submitInfo, m_uploadFence));
        SW_VK_CHECK(vkWaitForFences(m_device.handle(), 1, &m_uploadFence, VK_TRUE, UINT64_MAX));
        SW_VK_CHECK(vkResetFences(m_device.handle(), 1, &m_uploadFence));

        vkFreeCommandBuffers(m_device.handle(), m_uploadPool, 1, &cmd);
        // staging is destroyed here (RAII) — safe: the copy has completed.
    }

    std::vector<u8> VulkanMemory::readImageToHost(VkImage image, VkExtent2D extent,
                                                  VkImageLayout currentLayout)
    {
        const VkDeviceSize bytes =
            static_cast<VkDeviceSize>(extent.width) * extent.height * 4;
        // A BUFFER, not a linear image. An image copy has to agree with the
        // driver about tiling and row pitch; a buffer copy is tightly packed
        // by definition and works the same on every implementation, which
        // matters here because this path exists to run on llvmpipe.
        VulkanBuffer readback =
            createHostVisibleBuffer(bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT);

        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = m_uploadPool;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = 1;
        VkCommandBuffer cmd = VK_NULL_HANDLE;
        SW_VK_CHECK(vkAllocateCommandBuffers(m_device.handle(), &allocInfo, &cmd));

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        SW_VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo));

        const auto barrier = [&](VkImageLayout from, VkImageLayout to,
                                 VkAccessFlags2 srcAccess, VkAccessFlags2 dstAccess) {
            VkImageMemoryBarrier2 imageBarrier{};
            imageBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
            imageBarrier.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
            imageBarrier.srcAccessMask = srcAccess;
            imageBarrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
            imageBarrier.dstAccessMask = dstAccess;
            imageBarrier.oldLayout = from;
            imageBarrier.newLayout = to;
            imageBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            imageBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            imageBarrier.image = image;
            imageBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            imageBarrier.subresourceRange.levelCount = 1;
            imageBarrier.subresourceRange.layerCount = 1;
            VkDependencyInfo dependency{};
            dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
            dependency.imageMemoryBarrierCount = 1;
            dependency.pImageMemoryBarriers = &imageBarrier;
            vkCmdPipelineBarrier2(cmd, &dependency);
        };

        barrier(currentLayout, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                VK_ACCESS_2_MEMORY_WRITE_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);

        VkBufferImageCopy region{};
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.layerCount = 1;
        region.imageExtent = {extent.width, extent.height, 1};
        vkCmdCopyImageToBuffer(cmd, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                               readback.handle(), 1, &region);

        // Back where it was found: the swapchain image is still owned by the
        // presentation engine's state machine.
        barrier(VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, currentLayout,
                VK_ACCESS_2_TRANSFER_READ_BIT, VK_ACCESS_2_MEMORY_READ_BIT);

        SW_VK_CHECK(vkEndCommandBuffer(cmd));

        VkCommandBufferSubmitInfo cmdInfo{};
        cmdInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
        cmdInfo.commandBuffer = cmd;
        VkSubmitInfo2 submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
        submitInfo.commandBufferInfoCount = 1;
        submitInfo.pCommandBufferInfos = &cmdInfo;
        SW_VK_CHECK(vkQueueSubmit2(m_device.graphicsQueue(), 1, &submitInfo, m_uploadFence));
        SW_VK_CHECK(vkWaitForFences(m_device.handle(), 1, &m_uploadFence, VK_TRUE, UINT64_MAX));
        SW_VK_CHECK(vkResetFences(m_device.handle(), 1, &m_uploadFence));
        vkFreeCommandBuffers(m_device.handle(), m_uploadPool, 1, &cmd);

        std::vector<u8> pixels(static_cast<usize>(bytes));
        std::memcpy(pixels.data(), readback.mappedData(), static_cast<usize>(bytes));
        return pixels;
    }
} // namespace sw::vulkan
