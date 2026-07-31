#include "Renderer/Vulkan/VulkanSwapchain.hpp"

#include "Core/Log.hpp"
#include "Renderer/Vulkan/VulkanCommon.hpp"
#include "Renderer/Vulkan/VulkanDevice.hpp"

#include <algorithm>
#include <limits>

namespace sw::vulkan
{
    namespace
    {
        constexpr const char* kLogCat = "Vulkan";

        VkSurfaceFormatKHR chooseSurfaceFormat(VkPhysicalDevice physical, VkSurfaceKHR surface)
        {
            u32 count = 0;
            vkGetPhysicalDeviceSurfaceFormatsKHR(physical, surface, &count, nullptr);
            std::vector<VkSurfaceFormatKHR> formats(count);
            vkGetPhysicalDeviceSurfaceFormatsKHR(physical, surface, &count, formats.data());

            for (const VkSurfaceFormatKHR& format : formats)
            {
                if (format.format == VK_FORMAT_B8G8R8A8_SRGB &&
                    format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
                {
                    return format;
                }
            }
            // Fallback: first reported format (spec guarantees at least one).
            return formats.at(0);
        }

        VkPresentModeKHR choosePresentMode(VkPhysicalDevice physical, VkSurfaceKHR surface)
        {
            u32 count = 0;
            vkGetPhysicalDeviceSurfacePresentModesKHR(physical, surface, &count, nullptr);
            std::vector<VkPresentModeKHR> modes(count);
            vkGetPhysicalDeviceSurfacePresentModesKHR(physical, surface, &count, modes.data());

            for (VkPresentModeKHR mode : modes)
            {
                if (mode == VK_PRESENT_MODE_MAILBOX_KHR)
                {
                    return mode;
                }
            }
            return VK_PRESENT_MODE_FIFO_KHR; // guaranteed by the spec
        }

        VkExtent2D chooseExtent(const VkSurfaceCapabilitiesKHR& caps, u32 desiredWidth,
                                u32 desiredHeight)
        {
            if (caps.currentExtent.width != std::numeric_limits<u32>::max())
            {
                return caps.currentExtent; // surface dictates the size
            }
            VkExtent2D extent{desiredWidth, desiredHeight};
            extent.width = std::clamp(extent.width, caps.minImageExtent.width,
                                      caps.maxImageExtent.width);
            extent.height = std::clamp(extent.height, caps.minImageExtent.height,
                                       caps.maxImageExtent.height);
            return extent;
        }
    } // namespace

    VulkanSwapchain::VulkanSwapchain(const VulkanDevice& device, VkSurfaceKHR surface,
                                     u32 desiredWidth, u32 desiredHeight)
        : m_device(device)
        , m_surface(surface)
    {
        create(desiredWidth, desiredHeight, VK_NULL_HANDLE);
    }

    VulkanSwapchain::~VulkanSwapchain()
    {
        destroyImageViews();
        if (m_swapchain != VK_NULL_HANDLE)
        {
            vkDestroySwapchainKHR(m_device.handle(), m_swapchain, nullptr);
            m_swapchain = VK_NULL_HANDLE;
        }
    }

    void VulkanSwapchain::recreate(u32 desiredWidth, u32 desiredHeight)
    {
        destroyImageViews();
        VkSwapchainKHR old = m_swapchain;
        create(desiredWidth, desiredHeight, old);
        if (old != VK_NULL_HANDLE)
        {
            vkDestroySwapchainKHR(m_device.handle(), old, nullptr);
        }
    }

    VkResult VulkanSwapchain::acquireNextImage(VkSemaphore signalSemaphore, u32& outImageIndex)
    {
        return vkAcquireNextImageKHR(m_device.handle(), m_swapchain,
                                     std::numeric_limits<u64>::max(), signalSemaphore,
                                     VK_NULL_HANDLE, &outImageIndex);
    }

    VkResult VulkanSwapchain::present(VkQueue presentQueue, VkSemaphore waitSemaphore,
                                      u32 imageIndex)
    {
        VkPresentInfoKHR presentInfo{};
        presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
        presentInfo.waitSemaphoreCount = 1;
        presentInfo.pWaitSemaphores = &waitSemaphore;
        presentInfo.swapchainCount = 1;
        presentInfo.pSwapchains = &m_swapchain;
        presentInfo.pImageIndices = &imageIndex;
        return vkQueuePresentKHR(presentQueue, &presentInfo);
    }

    void VulkanSwapchain::create(u32 desiredWidth, u32 desiredHeight,
                                 VkSwapchainKHR oldSwapchain)
    {
        const VkPhysicalDevice physical = m_device.physicalHandle();

        VkSurfaceCapabilitiesKHR caps{};
        SW_VK_CHECK(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physical, m_surface, &caps));

        const VkSurfaceFormatKHR surfaceFormat = chooseSurfaceFormat(physical, m_surface);
        const VkPresentModeKHR presentMode = choosePresentMode(physical, m_surface);
        const VkExtent2D extent = chooseExtent(caps, desiredWidth, desiredHeight);

        // One more than the minimum to avoid stalling on the driver;
        // clamped to the maximum (0 = unlimited).
        u32 imageCount = caps.minImageCount + 1;
        if (caps.maxImageCount > 0 && imageCount > caps.maxImageCount)
        {
            imageCount = caps.maxImageCount;
        }

        VkSwapchainCreateInfoKHR createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
        createInfo.surface = m_surface;
        createInfo.minImageCount = imageCount;
        createInfo.imageFormat = surfaceFormat.format;
        createInfo.imageColorSpace = surfaceFormat.colorSpace;
        createInfo.imageExtent = extent;
        createInfo.imageArrayLayers = 1;
        createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        createInfo.preTransform = caps.currentTransform;
        createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        createInfo.presentMode = presentMode;
        createInfo.clipped = VK_TRUE;
        createInfo.oldSwapchain = oldSwapchain;

        const u32 familyIndices[] = {m_device.graphicsFamilyIndex(),
                                     m_device.presentFamilyIndex()};
        if (m_device.graphicsFamilyIndex() != m_device.presentFamilyIndex())
        {
            // Simplicity over throughput for the rare split-family case.
            createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
            createInfo.queueFamilyIndexCount = 2;
            createInfo.pQueueFamilyIndices = familyIndices;
        }
        else
        {
            createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        }

        SW_VK_CHECK(vkCreateSwapchainKHR(m_device.handle(), &createInfo, nullptr, &m_swapchain));

        m_imageFormat = surfaceFormat.format;
        m_extent = extent;

        u32 actualCount = 0;
        vkGetSwapchainImagesKHR(m_device.handle(), m_swapchain, &actualCount, nullptr);
        m_images.resize(actualCount);
        vkGetSwapchainImagesKHR(m_device.handle(), m_swapchain, &actualCount, m_images.data());

        m_imageViews.resize(actualCount);
        for (u32 i = 0; i < actualCount; ++i)
        {
            VkImageViewCreateInfo viewInfo{};
            viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
            viewInfo.image = m_images[i];
            viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            viewInfo.format = m_imageFormat;
            viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            viewInfo.subresourceRange.baseMipLevel = 0;
            viewInfo.subresourceRange.levelCount = 1;
            viewInfo.subresourceRange.baseArrayLayer = 0;
            viewInfo.subresourceRange.layerCount = 1;
            SW_VK_CHECK(
                vkCreateImageView(m_device.handle(), &viewInfo, nullptr, &m_imageViews[i]));
        }

        SW_LOG_INFO(kLogCat, "Swapchain created: {}x{}, {} images, format {}, present mode {}",
                    m_extent.width, m_extent.height, actualCount,
                    static_cast<i32>(m_imageFormat), static_cast<i32>(presentMode));
    }

    void VulkanSwapchain::destroyImageViews()
    {
        for (VkImageView view : m_imageViews)
        {
            vkDestroyImageView(m_device.handle(), view, nullptr);
        }
        m_imageViews.clear();
        m_images.clear();
    }
} // namespace sw::vulkan
