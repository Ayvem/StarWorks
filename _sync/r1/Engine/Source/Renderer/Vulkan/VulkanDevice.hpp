#pragma once

// ============================================================================
// Renderer/Vulkan/VulkanDevice.hpp
// Physical device selection and logical device / queue ownership.
//
// Requirements enforced at selection time:
//  - Vulkan 1.3 with dynamicRendering + synchronization2 (the engine's
//    render-graph-to-be emits its own barriers; no legacy render passes).
//  - A graphics queue family and a present-capable queue family
//    (usually the same; both cases are handled).
//  - VK_KHR_swapchain.
// Discrete GPUs are preferred over integrated/software implementations.
// ============================================================================

#include "Core/Types.hpp"

#include <vulkan/vulkan.h>

namespace sw::vulkan
{
    class VulkanDevice
    {
    public:
        struct Options
        {
            /// Invert type scoring so software/CPU implementations win.
            bool preferCpuDevice = false;
        };

        VulkanDevice(VkInstance instance, VkSurfaceKHR surface, const Options& options);
        ~VulkanDevice();

        VulkanDevice(const VulkanDevice&) = delete;
        VulkanDevice& operator=(const VulkanDevice&) = delete;
        VulkanDevice(VulkanDevice&&) = delete;
        VulkanDevice& operator=(VulkanDevice&&) = delete;

        [[nodiscard]] VkDevice handle() const { return m_device; }
        [[nodiscard]] VkPhysicalDevice physicalHandle() const { return m_physicalDevice; }

        [[nodiscard]] u32 graphicsFamilyIndex() const { return m_graphicsFamily; }
        [[nodiscard]] u32 presentFamilyIndex() const { return m_presentFamily; }
        [[nodiscard]] VkQueue graphicsQueue() const { return m_graphicsQueue; }
        [[nodiscard]] VkQueue presentQueue() const { return m_presentQueue; }

        [[nodiscard]] const VkPhysicalDeviceProperties& properties() const
        {
            return m_properties;
        }

        /// Blocks until the GPU is idle. Used before destruction/recreation.
        void waitIdle() const;

    private:
        struct Candidate
        {
            VkPhysicalDevice device = VK_NULL_HANDLE;
            u32 graphicsFamily = 0;
            u32 presentFamily = 0;
            i32 score = -1;
        };

        [[nodiscard]] static Candidate evaluate(VkPhysicalDevice device, VkSurfaceKHR surface,
                                                const Options& options);

        VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
        VkDevice m_device = VK_NULL_HANDLE;
        VkQueue m_graphicsQueue = VK_NULL_HANDLE;
        VkQueue m_presentQueue = VK_NULL_HANDLE;
        u32 m_graphicsFamily = 0;
        u32 m_presentFamily = 0;
        VkPhysicalDeviceProperties m_properties{};
    };
} // namespace sw::vulkan
