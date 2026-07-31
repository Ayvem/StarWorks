#include "Renderer/Vulkan/VulkanPipeline.hpp"

#include "Renderer/Vulkan/VulkanCommon.hpp"

#include <array>

namespace sw::vulkan
{
    VulkanPipeline::VulkanPipeline(VkDevice device, const Config& config)
        : m_device(device)
    {
        // ---- pipeline layout -------------------------------------------------
        VkPushConstantRange pushRange{};
        pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        pushRange.offset = 0;
        pushRange.size = config.pushConstantSize;

        VkPipelineLayoutCreateInfo layoutInfo{};
        layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        layoutInfo.setLayoutCount = static_cast<u32>(config.descriptorSetLayouts.size());
        layoutInfo.pSetLayouts =
            config.descriptorSetLayouts.empty() ? nullptr : config.descriptorSetLayouts.data();
        if (config.pushConstantSize > 0)
        {
            layoutInfo.pushConstantRangeCount = 1;
            layoutInfo.pPushConstantRanges = &pushRange;
        }
        SW_VK_CHECK(vkCreatePipelineLayout(m_device, &layoutInfo, nullptr, &m_layout));

        // ---- shader stages -----------------------------------------------------
        const VkShaderModule vertModule = createShaderModule(config.vertexSpirv);
        const VkShaderModule fragModule = createShaderModule(config.fragmentSpirv);

        std::array<VkPipelineShaderStageCreateInfo, 2> stages{};
        stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = vertModule;
        stages[0].pName = "main";
        stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = fragModule;
        stages[1].pName = "main";

        // ---- fixed function state ----------------------------------------------
        VkPipelineVertexInputStateCreateInfo vertexInput{};
        vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        vertexInput.vertexBindingDescriptionCount =
            static_cast<u32>(config.vertexBindings.size());
        vertexInput.pVertexBindingDescriptions =
            config.vertexBindings.empty() ? nullptr : config.vertexBindings.data();
        vertexInput.vertexAttributeDescriptionCount =
            static_cast<u32>(config.vertexAttributes.size());
        vertexInput.pVertexAttributeDescriptions =
            config.vertexAttributes.empty() ? nullptr : config.vertexAttributes.data();

        VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
        inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        inputAssembly.topology = config.topology;

        VkPipelineViewportStateCreateInfo viewportState{};
        viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        viewportState.viewportCount = 1;
        viewportState.scissorCount = 1;

        VkPipelineRasterizationStateCreateInfo rasterization{};
        rasterization.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rasterization.polygonMode = config.polygonMode;
        rasterization.cullMode = config.cullMode;
        rasterization.frontFace = config.frontFace;
        rasterization.lineWidth = 1.0f;

        VkPipelineMultisampleStateCreateInfo multisample{};
        multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineDepthStencilStateCreateInfo depthStencil{};
        depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        depthStencil.depthTestEnable = config.depthTest ? VK_TRUE : VK_FALSE;
        depthStencil.depthWriteEnable = config.depthWrite ? VK_TRUE : VK_FALSE;
        depthStencil.depthCompareOp = config.depthCompareOp;

        VkPipelineColorBlendAttachmentState blendAttachment{};
        blendAttachment.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
            VK_COLOR_COMPONENT_A_BIT;
        if (config.enableAlphaBlend)
        {
            blendAttachment.blendEnable = VK_TRUE;
            blendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
            blendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
            blendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
            blendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
            blendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
            blendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
        }
        else
        {
            blendAttachment.blendEnable = VK_FALSE;
        }

        VkPipelineColorBlendStateCreateInfo colorBlend{};
        colorBlend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        colorBlend.attachmentCount = 1;
        colorBlend.pAttachments = &blendAttachment;

        constexpr std::array<VkDynamicState, 2> dynamicStates = {
            VK_DYNAMIC_STATE_VIEWPORT,
            VK_DYNAMIC_STATE_SCISSOR,
        };
        VkPipelineDynamicStateCreateInfo dynamicState{};
        dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamicState.dynamicStateCount = static_cast<u32>(dynamicStates.size());
        dynamicState.pDynamicStates = dynamicStates.data();

        // ---- dynamic rendering target description --------------------------------
        VkPipelineRenderingCreateInfo renderingInfo{};
        renderingInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
        renderingInfo.colorAttachmentCount = 1;
        renderingInfo.pColorAttachmentFormats = &config.colorAttachmentFormat;
        renderingInfo.depthAttachmentFormat = config.depthAttachmentFormat;

        VkGraphicsPipelineCreateInfo pipelineInfo{};
        pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipelineInfo.pNext = &renderingInfo;
        pipelineInfo.stageCount = static_cast<u32>(stages.size());
        pipelineInfo.pStages = stages.data();
        pipelineInfo.pVertexInputState = &vertexInput;
        pipelineInfo.pInputAssemblyState = &inputAssembly;
        pipelineInfo.pViewportState = &viewportState;
        pipelineInfo.pRasterizationState = &rasterization;
        pipelineInfo.pMultisampleState = &multisample;
        pipelineInfo.pDepthStencilState = &depthStencil;
        pipelineInfo.pColorBlendState = &colorBlend;
        pipelineInfo.pDynamicState = &dynamicState;
        pipelineInfo.layout = m_layout;
        pipelineInfo.renderPass = VK_NULL_HANDLE; // dynamic rendering

        const VkResult result = vkCreateGraphicsPipelines(m_device, VK_NULL_HANDLE, 1,
                                                          &pipelineInfo, nullptr, &m_pipeline);

        // Shader modules can be destroyed as soon as the pipeline exists.
        vkDestroyShaderModule(m_device, vertModule, nullptr);
        vkDestroyShaderModule(m_device, fragModule, nullptr);

        if (result != VK_SUCCESS)
        {
            SW_THROW("vkCreateGraphicsPipelines failed: {}", toString(result));
        }
    }

    VulkanPipeline::~VulkanPipeline()
    {
        if (m_pipeline != VK_NULL_HANDLE)
        {
            vkDestroyPipeline(m_device, m_pipeline, nullptr);
        }
        if (m_layout != VK_NULL_HANDLE)
        {
            vkDestroyPipelineLayout(m_device, m_layout, nullptr);
        }
    }

    VkShaderModule VulkanPipeline::createShaderModule(const std::vector<u8>& spirv) const
    {
        if (spirv.empty() || (spirv.size() % 4) != 0)
        {
            SW_THROW("Invalid SPIR-V blob (size {} bytes)", spirv.size());
        }

        VkShaderModuleCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = spirv.size();
        createInfo.pCode = reinterpret_cast<const u32*>(spirv.data());

        VkShaderModule module = VK_NULL_HANDLE;
        SW_VK_CHECK(vkCreateShaderModule(m_device, &createInfo, nullptr, &module));
        return module;
    }
} // namespace sw::vulkan
