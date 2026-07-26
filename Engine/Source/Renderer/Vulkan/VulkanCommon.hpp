#pragma once

// ============================================================================
// Renderer/Vulkan/VulkanCommon.hpp
// Shared Vulkan utilities: error checking and result stringification.
//
// Policy: SW_VK_CHECK is used on calls whose failure is unrecoverable
// (initialization, resource creation). Per-frame calls that have legitimate
// non-success results (acquire/present returning OUT_OF_DATE) are handled
// explicitly by the renderer instead.
// ============================================================================

#include "Core/Error.hpp"

#include <vulkan/vulkan.h>

namespace sw::vulkan
{
    /// Human-readable name for a VkResult (falls back to the numeric value).
    [[nodiscard]] const char* toString(VkResult result);
} // namespace sw::vulkan

#define SW_VK_CHECK(expr)                                                          \
    do                                                                             \
    {                                                                              \
        const VkResult swVkResult_ = (expr);                                       \
        if (swVkResult_ != VK_SUCCESS)                                             \
        {                                                                          \
            SW_THROW("Vulkan call failed: {} -> {}", #expr,                        \
                     ::sw::vulkan::toString(swVkResult_));                         \
        }                                                                          \
    } while (false)
