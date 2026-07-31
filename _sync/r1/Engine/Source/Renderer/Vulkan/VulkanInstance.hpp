#pragma once

// ============================================================================
// Renderer/Vulkan/VulkanInstance.hpp
// VkInstance ownership: API version negotiation, instance extensions,
// validation layers and the debug messenger (routed into the engine log).
// ============================================================================

#include "Core/Types.hpp"

#include <vulkan/vulkan.h>

#include <string>
#include <vector>

namespace sw::vulkan
{
    class VulkanInstance
    {
    public:
        struct Config
        {
            std::string applicationName = "StarWorks";
            std::string engineName = "StarWorks Engine";
            /// Extensions required by the platform layer (from GLFW).
            std::vector<const char*> requiredExtensions;
            /// Request VK_LAYER_KHRONOS_validation + debug messenger.
            /// Silently downgraded (with a warning) if the layer is absent.
            bool enableValidation = false;
        };

        explicit VulkanInstance(const Config& config);
        ~VulkanInstance();

        VulkanInstance(const VulkanInstance&) = delete;
        VulkanInstance& operator=(const VulkanInstance&) = delete;
        VulkanInstance(VulkanInstance&&) = delete;
        VulkanInstance& operator=(VulkanInstance&&) = delete;

        [[nodiscard]] VkInstance handle() const { return m_instance; }
        [[nodiscard]] bool validationEnabled() const { return m_validationEnabled; }

    private:
        void createDebugMessenger();

        VkInstance m_instance = VK_NULL_HANDLE;
        VkDebugUtilsMessengerEXT m_debugMessenger = VK_NULL_HANDLE;
        bool m_validationEnabled = false;
    };
} // namespace sw::vulkan
