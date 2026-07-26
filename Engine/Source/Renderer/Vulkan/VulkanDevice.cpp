#include "Renderer/Vulkan/VulkanDevice.hpp"

#include "Core/Log.hpp"
#include "Renderer/Vulkan/VulkanCommon.hpp"

#include <cstring>
#include <set>
#include <vector>

namespace sw::vulkan
{
    namespace
    {
        constexpr const char* kLogCat = "Vulkan";

        constexpr const char* kRequiredDeviceExtensions[] = {
            VK_KHR_SWAPCHAIN_EXTENSION_NAME,
        };

        bool supportsRequiredExtensions(VkPhysicalDevice device)
        {
            u32 count = 0;
            vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr);
            std::vector<VkExtensionProperties> available(count);
            vkEnumerateDeviceExtensionProperties(device, nullptr, &count, available.data());

            for (const char* required : kRequiredDeviceExtensions)
            {
                bool found = false;
                for (const VkExtensionProperties& ext : available)
                {
                    if (std::strcmp(ext.extensionName, required) == 0)
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    return false;
                }
            }
            return true;
        }

        bool supportsRequired13Features(VkPhysicalDevice device)
        {
            VkPhysicalDeviceVulkan13Features features13{};
            features13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;

            VkPhysicalDeviceFeatures2 features2{};
            features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
            features2.pNext = &features13;

            vkGetPhysicalDeviceFeatures2(device, &features2);
            return features13.dynamicRendering == VK_TRUE &&
                   features13.synchronization2 == VK_TRUE;
        }
    } // namespace

    VulkanDevice::Candidate VulkanDevice::evaluate(VkPhysicalDevice device, VkSurfaceKHR surface,
                                                   const Options& options)
    {
        Candidate candidate{};
        candidate.device = device;

        VkPhysicalDeviceProperties props{};
        vkGetPhysicalDeviceProperties(device, &props);

        if (props.apiVersion < VK_API_VERSION_1_3 || !supportsRequiredExtensions(device) ||
            !supportsRequired13Features(device))
        {
            return candidate; // score stays -1 (rejected)
        }

        // ---- queue families --------------------------------------------------
        u32 familyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(device, &familyCount, nullptr);
        std::vector<VkQueueFamilyProperties> families(familyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(device, &familyCount, families.data());

        bool graphicsFound = false;
        bool presentFound = false;
        for (u32 i = 0; i < familyCount; ++i)
        {
            const bool isGraphics = (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0;

            VkBool32 presentSupport = VK_FALSE;
            vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface, &presentSupport);

            // Prefer a single family that does both graphics and present.
            if (isGraphics && presentSupport == VK_TRUE)
            {
                candidate.graphicsFamily = i;
                candidate.presentFamily = i;
                graphicsFound = true;
                presentFound = true;
                break;
            }
            if (isGraphics && !graphicsFound)
            {
                candidate.graphicsFamily = i;
                graphicsFound = true;
            }
            if (presentSupport == VK_TRUE && !presentFound)
            {
                candidate.presentFamily = i;
                presentFound = true;
            }
        }

        if (!graphicsFound || !presentFound)
        {
            return candidate; // rejected
        }

        // ---- scoring ---------------------------------------------------------
        // Default: fastest hardware wins. preferCpuDevice inverts the type
        // ranking so a software implementation (llvmpipe/SwiftShader) is
        // chosen when present — rendering code is unaffected either way.
        i32 score = 0;
        if (options.preferCpuDevice)
        {
            switch (props.deviceType)
            {
            case VK_PHYSICAL_DEVICE_TYPE_CPU:            score += 1000; break;
            case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU:    score += 500;  break;
            case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU: score += 250;  break;
            case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:   score += 100;  break;
            default:                                     score += 50;   break;
            }
        }
        else
        {
            switch (props.deviceType)
            {
            case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:   score += 1000; break;
            case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU: score += 500;  break;
            case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU:    score += 250;  break;
            case VK_PHYSICAL_DEVICE_TYPE_CPU:            score += 100;  break;
            default:                                     score += 50;   break;
            }
        }
        if (candidate.graphicsFamily == candidate.presentFamily)
        {
            score += 50; // avoids ownership transfers between queues
        }
        candidate.score = score;
        return candidate;
    }

    VulkanDevice::VulkanDevice(VkInstance instance, VkSurfaceKHR surface, const Options& options)
    {
        // ---- pick the best physical device ----------------------------------
        u32 deviceCount = 0;
        SW_VK_CHECK(vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr));
        if (deviceCount == 0)
        {
            SW_THROW("No Vulkan-capable GPU found");
        }
        std::vector<VkPhysicalDevice> devices(deviceCount);
        SW_VK_CHECK(vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data()));

        Candidate best{};
        for (VkPhysicalDevice device : devices)
        {
            const Candidate candidate = evaluate(device, surface, options);
            if (candidate.score > best.score)
            {
                best = candidate;
            }
        }
        if (best.score < 0)
        {
            SW_THROW("No GPU satisfies the engine requirements "
                     "(Vulkan 1.3, dynamicRendering, synchronization2, swapchain, present)");
        }

        m_physicalDevice = best.device;
        m_graphicsFamily = best.graphicsFamily;
        m_presentFamily = best.presentFamily;
        vkGetPhysicalDeviceProperties(m_physicalDevice, &m_properties);

        SW_LOG_INFO(kLogCat, "Selected GPU: {} (driver {}.{}.{}, API {}.{}.{})",
                    m_properties.deviceName,
                    VK_API_VERSION_MAJOR(m_properties.driverVersion),
                    VK_API_VERSION_MINOR(m_properties.driverVersion),
                    VK_API_VERSION_PATCH(m_properties.driverVersion),
                    VK_API_VERSION_MAJOR(m_properties.apiVersion),
                    VK_API_VERSION_MINOR(m_properties.apiVersion),
                    VK_API_VERSION_PATCH(m_properties.apiVersion));

        // ---- logical device ---------------------------------------------------
        const std::set<u32> uniqueFamilies{m_graphicsFamily, m_presentFamily};
        const f32 queuePriority = 1.0f;
        std::vector<VkDeviceQueueCreateInfo> queueInfos;
        for (u32 family : uniqueFamilies)
        {
            VkDeviceQueueCreateInfo queueInfo{};
            queueInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
            queueInfo.queueFamilyIndex = family;
            queueInfo.queueCount = 1;
            queueInfo.pQueuePriorities = &queuePriority;
            queueInfos.push_back(queueInfo);
        }

        VkPhysicalDeviceVulkan13Features features13{};
        features13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
        features13.dynamicRendering = VK_TRUE;
        features13.synchronization2 = VK_TRUE;

        VkPhysicalDeviceFeatures2 features2{};
        features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
        features2.pNext = &features13;

        VkDeviceCreateInfo deviceInfo{};
        deviceInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        deviceInfo.pNext = &features2; // features passed via pNext, not pEnabledFeatures
        deviceInfo.queueCreateInfoCount = static_cast<u32>(queueInfos.size());
        deviceInfo.pQueueCreateInfos = queueInfos.data();
        deviceInfo.enabledExtensionCount =
            static_cast<u32>(std::size(kRequiredDeviceExtensions));
        deviceInfo.ppEnabledExtensionNames = kRequiredDeviceExtensions;

        SW_VK_CHECK(vkCreateDevice(m_physicalDevice, &deviceInfo, nullptr, &m_device));

        vkGetDeviceQueue(m_device, m_graphicsFamily, 0, &m_graphicsQueue);
        vkGetDeviceQueue(m_device, m_presentFamily, 0, &m_presentQueue);

        SW_LOG_INFO(kLogCat, "Logical device created (graphics family {}, present family {})",
                    m_graphicsFamily, m_presentFamily);
    }

    VulkanDevice::~VulkanDevice()
    {
        if (m_device != VK_NULL_HANDLE)
        {
            vkDestroyDevice(m_device, nullptr);
            m_device = VK_NULL_HANDLE;
        }
    }

    void VulkanDevice::waitIdle() const
    {
        if (m_device != VK_NULL_HANDLE)
        {
            vkDeviceWaitIdle(m_device);
        }
    }
} // namespace sw::vulkan
