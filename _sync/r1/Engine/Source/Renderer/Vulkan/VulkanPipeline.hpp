#pragma once

// ============================================================================
// Renderer/Vulkan/VulkanPipeline.hpp
// Graphics pipeline + pipeline layout, built for dynamic rendering
// (VkPipelineRenderingCreateInfo instead of a VkRenderPass).
//
// Viewport and scissor are dynamic states so window resizes never require a
// pipeline rebuild. This class will later be superseded by a pipeline cache
// keyed on shader/state hashes; its interface is kept deliberately small.
// ============================================================================

#include "Core/Types.hpp"

#include <vulkan/vulkan.h>

#include <vector>

namespace sw::vulkan
{
    class VulkanPipeline
    {
    public:
        struct Config
        {
            std::vector<u8> vertexSpirv;
            std::vector<u8> fragmentSpirv;
            VkFormat colorAttachmentFormat = VK_FORMAT_UNDEFINED;
            /// VK_FORMAT_UNDEFINED = no depth attachment.
            VkFormat depthAttachmentFormat = VK_FORMAT_UNDEFINED;
            bool depthTest = false;
            bool depthWrite = false;
            /// Engine uses reverse-Z: near plane maps to depth 1, far to 0,
            /// hence GREATER comparisons (far better f32 depth distribution —
            /// essential at planetary scales).
            VkCompareOp depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;

            /// Standard alpha blending on the color attachment (HUD).
            bool enableAlphaBlend = false;

            /// Empty = no vertex input (fullscreen / generated geometry).
            std::vector<VkVertexInputBindingDescription> vertexBindings;
            std::vector<VkVertexInputAttributeDescription> vertexAttributes;

            /// Descriptor set layouts for the pipeline layout (set 0..N).
            std::vector<VkDescriptorSetLayout> descriptorSetLayouts;

            /// Size in bytes of the push-constant block visible to the
            /// vertex stage (0 = no push constants).
            u32 pushConstantSize = 0;
            VkPrimitiveTopology topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
            VkPolygonMode polygonMode = VK_POLYGON_MODE_FILL;
            VkCullModeFlags cullMode = VK_CULL_MODE_BACK_BIT;
            /// Engine convention: geometry is authored counter-clockwise in
            /// world space (+Y up). Verified empirically against the
            /// projection Y-flip in Camera — do not change one without the
            /// other.
            VkFrontFace frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        };

        VulkanPipeline(VkDevice device, const Config& config);
        ~VulkanPipeline();

        VulkanPipeline(const VulkanPipeline&) = delete;
        VulkanPipeline& operator=(const VulkanPipeline&) = delete;
        VulkanPipeline(VulkanPipeline&&) = delete;
        VulkanPipeline& operator=(VulkanPipeline&&) = delete;

        [[nodiscard]] VkPipeline handle() const { return m_pipeline; }
        [[nodiscard]] VkPipelineLayout layout() const { return m_layout; }

    private:
        [[nodiscard]] VkShaderModule createShaderModule(const std::vector<u8>& spirv) const;

        VkDevice m_device = VK_NULL_HANDLE;
        VkPipelineLayout m_layout = VK_NULL_HANDLE;
        VkPipeline m_pipeline = VK_NULL_HANDLE;
    };
} // namespace sw::vulkan
