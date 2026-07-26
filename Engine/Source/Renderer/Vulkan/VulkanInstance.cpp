#include "Renderer/Vulkan/VulkanInstance.hpp"

#include "Core/Log.hpp"
#include "Renderer/Vulkan/VulkanCommon.hpp"

#include <cstring>

namespace sw::vulkan
{
    namespace
    {
        constexpr const char* kLogCat = "Vulkan";
        constexpr const char* kValidationLayerName = "VK_LAYER_KHRONOS_validation";

        bool isLayerAvailable(const char* layerName)
        {
            u32 count = 0;
            vkEnumerateInstanceLayerProperties(&count, nullptr);
            std::vector<VkLayerProperties> layers(count);
            vkEnumerateInstanceLayerProperties(&count, layers.data());

            for (const VkLayerProperties& layer : layers)
            {
                if (std::strcmp(layer.layerName, layerName) == 0)
                {
                    return true;
                }
            }
            return false;
        }

        VKAPI_ATTR VkBool32 VKAPI_CALL debugMessengerCallback(
            VkDebugUtilsMessageSeverityFlagBitsEXT severity,
            VkDebugUtilsMessageTypeFlagsEXT /*types*/,
            const VkDebugUtilsMessengerCallbackDataEXT* callbackData,
            void* /*userData*/)
        {
            const char* message = (callbackData != nullptr && callbackData->pMessage != nullptr)
                                      ? callbackData->pMessage
                                      : "<no message>";

            if (severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT)
            {
                SW_LOG_ERROR("VulkanValidation", "{}", message);
            }
            else if (severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)
            {
                SW_LOG_WARN("VulkanValidation", "{}", message);
            }
            else
            {
                SW_LOG_TRACE("VulkanValidation", "{}", message);
            }
            return VK_FALSE; // never abort the offending call
        }

        VkDebugUtilsMessengerCreateInfoEXT makeDebugMessengerCreateInfo()
        {
            VkDebugUtilsMessengerCreateInfoEXT info{};
            info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
            info.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                                   VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
            info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                               VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                               VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
            info.pfnUserCallback = debugMessengerCallback;
            return info;
        }
    } // namespace

    VulkanInstance::VulkanInstance(const Config& config)
    {
        // ---- validation availability ---------------------------------------
        m_validationEnabled = config.enableValidation;
        if (m_validationEnabled && !isLayerAvailable(kValidationLayerName))
        {
            SW_LOG_WARN(kLogCat,
                        "Validation requested but {} is not installed — continuing without it "
                        "(install the Vulkan SDK to enable validation)",
                        kValidationLayerName);
            m_validationEnabled = false;
        }

        // ---- application / engine identity ---------------------------------
        VkApplicationInfo appInfo{};
        appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        appInfo.pApplicationName = config.applicationName.c_str();
        appInfo.applicationVersion = VK_MAKE_VERSION(0, 1, 0);
        appInfo.pEngineName = config.engineName.c_str();
        appInfo.engineVersion = VK_MAKE_VERSION(0, 1, 0);
        appInfo.apiVersion = VK_API_VERSION_1_3;

        // ---- extensions and layers -----------------------------------------
        std::vector<const char*> extensions = config.requiredExtensions;
        std::vector<const char*> layers;
        if (m_validationEnabled)
        {
            extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
            layers.push_back(kValidationLayerName);
        }

        VkInstanceCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        createInfo.pApplicationInfo = &appInfo;
        createInfo.enabledExtensionCount = static_cast<u32>(extensions.size());
        createInfo.ppEnabledExtensionNames = extensions.data();
        createInfo.enabledLayerCount = static_cast<u32>(layers.size());
        createInfo.ppEnabledLayerNames = layers.empty() ? nullptr : layers.data();

        // Catch issues during vkCreateInstance/vkDestroyInstance themselves.
        VkDebugUtilsMessengerCreateInfoEXT debugInfo = makeDebugMessengerCreateInfo();
        if (m_validationEnabled)
        {
            createInfo.pNext = &debugInfo;
        }

        SW_VK_CHECK(vkCreateInstance(&createInfo, nullptr, &m_instance));

        u32 instanceVersion = VK_API_VERSION_1_0;
        vkEnumerateInstanceVersion(&instanceVersion);
        SW_LOG_INFO(kLogCat, "Instance created (loader API {}.{}.{}, validation: {})",
                    VK_API_VERSION_MAJOR(instanceVersion), VK_API_VERSION_MINOR(instanceVersion),
                    VK_API_VERSION_PATCH(instanceVersion), m_validationEnabled ? "on" : "off");

        if (m_validationEnabled)
        {
            createDebugMessenger();
        }
    }

    VulkanInstance::~VulkanInstance()
    {
        if (m_debugMessenger != VK_NULL_HANDLE)
        {
            auto destroyFn = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
                vkGetInstanceProcAddr(m_instance, "vkDestroyDebugUtilsMessengerEXT"));
            if (destroyFn != nullptr)
            {
                destroyFn(m_instance, m_debugMessenger, nullptr);
            }
            m_debugMessenger = VK_NULL_HANDLE;
        }
        if (m_instance != VK_NULL_HANDLE)
        {
            vkDestroyInstance(m_instance, nullptr);
            m_instance = VK_NULL_HANDLE;
        }
    }

    void VulkanInstance::createDebugMessenger()
    {
        auto createFn = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(m_instance, "vkCreateDebugUtilsMessengerEXT"));
        if (createFn == nullptr)
        {
            SW_LOG_WARN(kLogCat, "vkCreateDebugUtilsMessengerEXT unavailable — "
                                 "no validation output will be captured");
            return;
        }

        const VkDebugUtilsMessengerCreateInfoEXT info = makeDebugMessengerCreateInfo();
        SW_VK_CHECK(createFn(m_instance, &info, nullptr, &m_debugMessenger));
    }
} // namespace sw::vulkan
