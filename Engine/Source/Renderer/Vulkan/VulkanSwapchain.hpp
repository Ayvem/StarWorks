#pragma once

// ============================================================================
// Renderer/Vulkan/VulkanSwapchain.hpp
// Swapchain ownership: surface format / present mode selection, image views,
// acquire/present, and in-place recreation on resize.
//
// Preferences: B8G8R8A8_SRGB + SRGB_NONLINEAR (gamma handled by the display
// engine), MAILBOX present mode when available (low latency without tearing),
// FIFO otherwise (always available, vsync).
// ============================================================================

#include "Core/Types.hpp"

#include <vulkan/vulkan.h>

#include <vector>

namespace sw::vulkan
{
    class VulkanDevice;

    class VulkanSwapchain
    {
    public:
        VulkanSwapchain(const VulkanDevice& device, VkSurfaceKHR surface,
                        u32 desiredWidth, u32 desiredHeight);
        ~VulkanSwapchain();

        VulkanSwapchain(const VulkanSwapchain&) = delete;
        VulkanSwapchain& operator=(const VulkanSwapchain&) = delete;
        VulkanSwapchain(VulkanSwapchain&&) = delete;
        VulkanSwapchain& operator=(VulkanSwapchain&&) = delete;

        /// Rebuilds the swapchain for a new framebuffer size. The caller must
        /// ensure the GPU is idle first (Renderer does a waitIdle).
        void recreate(u32 desiredWidth, u32 desiredHeight);

        /// May return VK_ERROR_OUT_OF_DATE_KHR / VK_SUBOPTIMAL_KHR — the
        /// renderer decides when to recreate.
        [[nodiscard]] VkResult acquireNextImage(VkSemaphore signalSemaphore, u32& outImageIndex);

        [[nodiscard]] VkResult present(VkQueue presentQueue, VkSemaphore waitSemaphore,
                                       u32 imageIndex);

        [[nodiscard]] VkSwapchainKHR handle() const { return m_swapchain; }
        [[nodiscard]] VkFormat imageFormat() const { return m_imageFormat; }
        [[nodiscard]] VkExtent2D extent() const { return m_extent; }
        [[nodiscard]] u32 imageCount() const { return static_cast<u32>(m_images.size()); }
        [[nodiscard]] VkImage image(u32 index) const { return m_images[index]; }
        [[nodiscard]] VkImageView imageView(u32 index) const { return m_imageViews[index]; }

    private:
        void create(u32 desiredWidth, u32 desiredHeight, VkSwapchainKHR oldSwapchain);
        void destroyImageViews();

        const VulkanDevice& m_device;
        VkSurfaceKHR m_surface = VK_NULL_HANDLE;

        VkSwapchainKHR m_swapchain = VK_NULL_HANDLE;
        VkFormat m_imageFormat = VK_FORMAT_UNDEFINED;
        VkExtent2D m_extent{};
        std::vector<VkImage> m_images;
        std::vector<VkImageView> m_imageViews;
    };
} // namespace sw::vulkan
