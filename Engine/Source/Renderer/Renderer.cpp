#include "Renderer/Renderer.hpp"

#include "Core/FileSystem.hpp"
#include "Core/Log.hpp"
#include "Platform/Window.hpp"
#include "Renderer/Vertex.hpp"
#include "Renderer/Vulkan/VulkanCommon.hpp"
#include "Renderer/Vulkan/VulkanDevice.hpp"
#include "Renderer/Vulkan/VulkanInstance.hpp"
#include "Renderer/Vulkan/VulkanPipeline.hpp"
#include "Renderer/Vulkan/VulkanSwapchain.hpp"
#include "Scene/Camera.hpp"
#include "UI/HudOrder.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>

namespace sw
{
    namespace
    {
        constexpr const char* kLogCat = "Renderer";
        constexpr u32 kInitialInstanceCapacity = 8192;

        using ProfileClock = std::chrono::steady_clock;

        f32 millisecondsSince(ProfileClock::time_point start)
        {
            return std::chrono::duration<f32, std::milli>(ProfileClock::now() - start).count();
        }
    } // namespace

    Renderer::Renderer(Window& window, const RendererConfig& config)
        : m_window(window)
    {
        // ---- instance + surface + device -------------------------------------
        vulkan::VulkanInstance::Config instanceConfig{};
        instanceConfig.applicationName = config.applicationName;
        instanceConfig.requiredExtensions = Window::requiredVulkanInstanceExtensions();
        instanceConfig.enableValidation = config.enableValidation;
        m_instance = std::make_unique<vulkan::VulkanInstance>(instanceConfig);

        m_surface = m_window.createVulkanSurface(m_instance->handle());
        vulkan::VulkanDevice::Options deviceOptions{};
        deviceOptions.preferCpuDevice = config.preferCpuDevice;
        m_device = std::make_unique<vulkan::VulkanDevice>(m_instance->handle(), m_surface,
                                                          deviceOptions);

        // ---- memory + swapchain + depth ---------------------------------------
        m_memory = std::make_unique<vulkan::VulkanMemory>(m_instance->handle(), *m_device);

        u32 width = 0;
        u32 height = 0;
        m_window.framebufferSize(width, height);
        m_swapchain =
            std::make_unique<vulkan::VulkanSwapchain>(*m_device, m_surface, width, height);
        createDepthResources();

        // ---- descriptors + per-frame resources ---------------------------------
        createDescriptorResources();
        createFrameResources();
        createSwapchainSemaphores();

        // ---- mesh pipeline (instanced, no push constants) ------------------------
        vulkan::VulkanPipeline::Config pipelineConfig{};
        pipelineConfig.vertexSpirv =
            FileSystem::readBinaryFile(FileSystem::resolve("Shaders/Mesh.vert.spv"));
        pipelineConfig.fragmentSpirv =
            FileSystem::readBinaryFile(FileSystem::resolve("Shaders/Mesh.frag.spv"));
        pipelineConfig.colorAttachmentFormat = m_swapchain->imageFormat();
        pipelineConfig.depthAttachmentFormat = m_depthImage.format();
        pipelineConfig.depthTest = true;
        pipelineConfig.depthWrite = true;
        pipelineConfig.vertexBindings = {vulkan::VertexInput::binding()};
        const auto attrs = vulkan::VertexInput::attributes();
        pipelineConfig.vertexAttributes.assign(attrs.begin(), attrs.end());
        pipelineConfig.descriptorSetLayouts = {m_descriptorSetLayout};
        m_meshPipeline =
            std::make_unique<vulkan::VulkanPipeline>(m_device->handle(), pipelineConfig);

        // ---- transparent pipeline: same shaders, blended, depth-read-only,
        // no culling (atmosphere/cloud shells are seen from inside too) -----
        vulkan::VulkanPipeline::Config transparentConfig = pipelineConfig;
        transparentConfig.depthWrite = false;
        transparentConfig.enableAlphaBlend = true;
        transparentConfig.cullMode = VK_CULL_MODE_NONE;
        m_transparentPipeline = std::make_unique<vulkan::VulkanPipeline>(
            m_device->handle(), transparentConfig);

        // ---- HUD pipeline: screen-space, unlit, alpha-blended, no depth -----
        vulkan::VulkanPipeline::Config hudConfig{};
        hudConfig.vertexSpirv =
            FileSystem::readBinaryFile(FileSystem::resolve("Shaders/Hud.vert.spv"));
        hudConfig.fragmentSpirv =
            FileSystem::readBinaryFile(FileSystem::resolve("Shaders/Hud.frag.spv"));
        hudConfig.colorAttachmentFormat = m_swapchain->imageFormat();
        hudConfig.depthAttachmentFormat = m_depthImage.format(); // same pass
        hudConfig.depthTest = false;
        hudConfig.depthWrite = false;
        hudConfig.cullMode = VK_CULL_MODE_NONE;
        hudConfig.enableAlphaBlend = true;
        hudConfig.vertexBindings = {vulkan::VertexInput::binding()};
        hudConfig.vertexAttributes.assign(attrs.begin(), attrs.end());
        hudConfig.descriptorSetLayouts = {m_descriptorSetLayout};
        m_hudPipeline =
            std::make_unique<vulkan::VulkanPipeline>(m_device->handle(), hudConfig);

        SW_LOG_INFO(kLogCat,
                    "Renderer initialized ({} frames in flight, reverse-Z, instanced)",
                    kFramesInFlight);
    }

    Renderer::~Renderer()
    {
        if (m_device)
        {
            m_device->waitIdle();
        }

        destroySwapchainSemaphores();
        destroyFrameResources();
        destroyDescriptorResources();
        m_hudPipeline.reset();
        m_meshPipeline.reset();
        m_depthImage.reset();
        m_swapchain.reset();
        m_memory.reset();
        m_device.reset();

        if (m_surface != VK_NULL_HANDLE && m_instance)
        {
            vkDestroySurfaceKHR(m_instance->handle(), m_surface, nullptr);
            m_surface = VK_NULL_HANDLE;
        }
        m_instance.reset();
    }

    Mesh Renderer::createMesh(const MeshData& data)
    {
        return Mesh(*m_memory, data);
    }

    void Renderer::onFramebufferResized(u32 width, u32 height)
    {
        m_framebufferResized = true;
        m_pendingWidth = width;
        m_pendingHeight = height;
    }

    void Renderer::setSunPosition(const Vec3& cameraRelativePosition)
    {
        m_sunPosition = cameraRelativePosition;
    }

    void Renderer::setAtmosphere(const Vec3& fogColor, f32 fogDensity,
                                 const Vec3& skyAmbient)
    {
        m_fogColor = fogColor;
        m_fogDensity = fogDensity;
        m_skyAmbient = skyAmbient;
    }

    void Renderer::setQuality(u32 level)
    {
        m_quality = static_cast<f32>(std::min(level, 2u));
    }

    void Renderer::setTimeSeconds(f32 seconds)
    {
        m_timeSeconds = seconds;
    }

    void Renderer::setAtmosphereBody(const Vec3& cameraRelativeCentre, f32 radius,
                                     i32 style)
    {
        m_atmosphereCentre = cameraRelativeCentre;
        m_atmosphereRadius = radius;
        m_atmosphereStyle = static_cast<f32>(style);
    }

    void Renderer::setShadowSpheres(std::span<const ShadowSphere> spheres)
    {
        m_shadowSphereCount =
            std::min(static_cast<u32>(spheres.size()), kMaxShadowSpheres);
        for (u32 i = 0; i < m_shadowSphereCount; ++i)
        {
            m_shadowSpheres[i] = spheres[i];
        }
    }

    void Renderer::waitIdle() const
    {
        if (m_device)
        {
            m_device->waitIdle();
        }
    }

    f32 Renderer::aspectRatio() const
    {
        const VkExtent2D extent = m_swapchain->extent();
        if (extent.height == 0)
        {
            return 16.0f / 9.0f;
        }
        return static_cast<f32>(extent.width) / static_cast<f32>(extent.height);
    }

    void Renderer::prepareBatches(FrameResources& frame, const Camera& camera,
                                  std::span<const DrawItem> items)
    {
        m_stats = {};
        m_stats.itemsSubmitted = static_cast<u32>(items.size());

        // ---- frustum culling ---------------------------------------------------
        const Frustum frustum =
            Frustum::fromViewProjection(camera.viewProjectionCameraRelative());

        m_visibleIndices.clear();
        m_visibleIndices.reserve(items.size());
        m_transparentIndices.clear();
        m_hudIndices.clear();
        for (u32 i = 0; i < items.size(); ++i)
        {
            const DrawItem& item = items[i];
            if (item.mesh == nullptr || !item.mesh->valid())
            {
                continue;
            }
            if (item.screenSpace)
            {
                m_hudIndices.push_back(i); // never culled, submission order
            }
            else if (frustum.intersectsSphere(item.boundsCenter, item.boundsRadius))
            {
                (item.transparent ? m_transparentIndices : m_visibleIndices)
                    .push_back(i);
            }
        }
        m_stats.itemsCulled =
            m_stats.itemsSubmitted - static_cast<u32>(m_visibleIndices.size()) -
            static_cast<u32>(m_transparentIndices.size()) -
            static_cast<u32>(m_hudIndices.size());

        // ---- group by mesh (sort keeps batches contiguous) -----------------------
        std::sort(m_visibleIndices.begin(), m_visibleIndices.end(),
                  [&items](u32 a, u32 b) { return items[a].mesh < items[b].mesh; });
        // Transparents sort BACK-TO-FRONT (correct blending), never by mesh.
        std::sort(m_transparentIndices.begin(), m_transparentIndices.end(),
                  [&items](u32 a, u32 b) {
                      return glm::dot(items[a].boundsCenter, items[a].boundsCenter) >
                             glm::dot(items[b].boundsCenter, items[b].boundsCenter);
                  });
        // THE HUD DRAWS IN THE ORDER IT WAS SUBMITTED, by layer. This used
        // to be a plain sort by mesh pointer — "their draw order has no
        // meaning" — and it cost a blank, flickering build menu: a panel and
        // the rows on it share one unit-quad mesh, so the sort scrambled
        // them against each other, and the glyph meshes sit at unrelated
        // addresses, so text landed above or below its panel depending on
        // where the mesh table happened to put them. The rule now lives in
        // one pure, tested function (UI/HudOrder.hpp) instead of in a
        // comparator's side effects.
        {
            std::vector<ui::HudItemKey> keys;
            keys.reserve(m_hudIndices.size());
            for (const u32 index : m_hudIndices)
            {
                keys.push_back({items[index].hudLayer, items[index].mesh});
            }
            const std::vector<u32> order = ui::hudDrawOrder(keys);
            std::vector<u32> reordered;
            reordered.reserve(order.size());
            for (const u32 slot : order)
            {
                reordered.push_back(m_hudIndices[slot]);
            }
            m_hudIndices.swap(reordered);
        }

        const u32 visibleCount = static_cast<u32>(m_visibleIndices.size());
        const u32 transparentCount = static_cast<u32>(m_transparentIndices.size());
        const u32 hudCount = static_cast<u32>(m_hudIndices.size());
        ensureInstanceCapacity(frame, visibleCount + transparentCount + hudCount);

        auto* instances = static_cast<InstanceData*>(frame.instanceBuffer.mappedData());
        m_batches.clear();
        m_transparentBatches.clear();
        m_hudBatches.clear();

        auto appendBatches = [&](const std::vector<u32>& indices, u32 firstSlot,
                                 std::vector<DrawBatch>& batches) {
            for (u32 i = 0; i < indices.size(); ++i)
            {
                const u32 slot = firstSlot + i;
                const DrawItem& item = items[indices[i]];
                instances[slot].model = item.transform;
                instances[slot].tint = item.tint;

                const f32 distanceSquared =
                    glm::dot(item.boundsCenter, item.boundsCenter);
                if (batches.empty() || batches.back().mesh != item.mesh)
                {
                    batches.push_back({item.mesh, slot, 1, distanceSquared});
                }
                else
                {
                    ++batches.back().instanceCount;
                    batches.back().nearDistanceSquared =
                        std::min(batches.back().nearDistanceSquared, distanceSquared);
                }
            }
        };
        appendBatches(m_visibleIndices, 0, m_batches);
        appendBatches(m_transparentIndices, visibleCount, m_transparentBatches);
        appendBatches(m_hudIndices, visibleCount + transparentCount, m_hudBatches);

        // ---- opaque draws go FRONT TO BACK --------------------------------
        // Sorting the BATCHES (not the instances) keeps every batch's slice
        // of the instance buffer exactly where it was written, so this costs
        // one sort of a few dozen entries. What it buys: the terrain patch
        // and the craft are drawn before the planet behind them, and with
        // the early depth test declared in Mesh.frag every hidden fragment
        // of ground is rejected BEFORE it evaluates a single noise octave.
        // Standing on a planet, that is most of the screen.
        std::sort(m_batches.begin(), m_batches.end(),
                  [](const DrawBatch& a, const DrawBatch& b) {
                      return a.nearDistanceSquared < b.nearDistanceSquared;
                  });

        m_stats.instancesDrawn = visibleCount + transparentCount + hudCount;
        m_stats.drawCalls = static_cast<u32>(m_batches.size() +
                                             m_transparentBatches.size() +
                                             m_hudBatches.size());
    }

    void Renderer::renderFrame(const Camera& camera, std::span<const DrawItem> items)
    {
        if (m_window.isMinimized())
        {
            return;
        }

        FrameResources& frame = m_frames[m_currentFrame];
        const VkDevice device = m_device->handle();
        const ProfileClock::time_point frameStart = ProfileClock::now();

        // ---- wait until this frame slot is free -------------------------------
        vkWaitForFences(device, 1, &frame.inFlight, VK_TRUE, UINT64_MAX);
        const f32 fenceWaitMs = millisecondsSince(frameStart);

        if (m_framebufferResized)
        {
            recreateSwapchain();
            m_framebufferResized = false;
        }

        // ---- acquire ------------------------------------------------------------
        u32 imageIndex = 0;
        const VkResult acquireResult =
            m_swapchain->acquireNextImage(frame.imageAvailable, imageIndex);
        if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR)
        {
            recreateSwapchain();
            return;
        }
        if (acquireResult != VK_SUCCESS && acquireResult != VK_SUBOPTIMAL_KHR)
        {
            SW_LOG_ERROR(kLogCat, "vkAcquireNextImageKHR failed: {}",
                         vulkan::toString(acquireResult));
            return;
        }

        vkResetFences(device, 1, &frame.inFlight);

        // ---- CPU frame preparation (fence guarantees buffers are free) ----------
        CameraUniforms uniforms{};
        uniforms.viewProjection = camera.viewProjectionCameraRelative();
        uniforms.cameraPosition = Vec4(0.0f, 0.0f, 0.0f, 1.0f);
        uniforms.sunPosition =
            Vec4(m_sunPosition, static_cast<f32>(m_shadowSphereCount));
        for (u32 i = 0; i < m_shadowSphereCount; ++i)
        {
            uniforms.shadowSpheres[i] =
                Vec4(m_shadowSpheres[i].center, m_shadowSpheres[i].radius);
        }
        uniforms.fogColorDensity = Vec4(m_fogColor, m_fogDensity);
        uniforms.skyAmbient = Vec4(m_skyAmbient, 0.0f);
        uniforms.qualityTime =
            Vec4(m_quality, m_timeSeconds, m_atmosphereStyle, 0.0f);
        uniforms.atmosphereBody = Vec4(m_atmosphereCentre, m_atmosphereRadius);
        std::memcpy(frame.cameraUbo.mappedData(), &uniforms, sizeof(uniforms));

        const ProfileClock::time_point prepareStart = ProfileClock::now();
        prepareBatches(frame, camera, items);
        const f32 prepareMs = millisecondsSince(prepareStart);

        // ---- record --------------------------------------------------------------
        vkResetCommandPool(device, frame.commandPool, 0);

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        SW_VK_CHECK(vkBeginCommandBuffer(frame.commandBuffer, &beginInfo));
        recordCommands(frame.commandBuffer, imageIndex, frame);
        SW_VK_CHECK(vkEndCommandBuffer(frame.commandBuffer));

        // ---- submit (synchronization2) --------------------------------------------
        VkSemaphoreSubmitInfo waitInfo{};
        waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
        waitInfo.semaphore = frame.imageAvailable;
        waitInfo.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;

        VkSemaphoreSubmitInfo signalInfo{};
        signalInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
        signalInfo.semaphore = m_renderFinished[imageIndex];
        signalInfo.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;

        VkCommandBufferSubmitInfo cmdInfo{};
        cmdInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
        cmdInfo.commandBuffer = frame.commandBuffer;

        VkSubmitInfo2 submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
        submitInfo.waitSemaphoreInfoCount = 1;
        submitInfo.pWaitSemaphoreInfos = &waitInfo;
        submitInfo.commandBufferInfoCount = 1;
        submitInfo.pCommandBufferInfos = &cmdInfo;
        submitInfo.signalSemaphoreInfoCount = 1;
        submitInfo.pSignalSemaphoreInfos = &signalInfo;

        SW_VK_CHECK(vkQueueSubmit2(m_device->graphicsQueue(), 1, &submitInfo, frame.inFlight));

        // ---- present ---------------------------------------------------------------
        const VkResult presentResult = m_swapchain->present(
            m_device->presentQueue(), m_renderFinished[imageIndex], imageIndex);
        if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR)
        {
            recreateSwapchain();
        }
        else if (presentResult != VK_SUCCESS)
        {
            SW_LOG_ERROR(kLogCat, "vkQueuePresentKHR failed: {}",
                         vulkan::toString(presentResult));
        }

        m_currentFrame = (m_currentFrame + 1) % kFramesInFlight;

        m_stats.cpuFenceWaitMs = fenceWaitMs;
        m_stats.cpuPrepareMs = prepareMs;
        m_stats.cpuTotalMs = millisecondsSince(frameStart);
    }

    void Renderer::recordCommands(VkCommandBuffer cmd, u32 imageIndex,
                                  const FrameResources& frame) const
    {
        const VkExtent2D extent = m_swapchain->extent();
        const VkImage swapImage = m_swapchain->image(imageIndex);

        VkImageSubresourceRange colorRange{};
        colorRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        colorRange.levelCount = 1;
        colorRange.layerCount = 1;

        VkImageSubresourceRange depthRange = colorRange;
        depthRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;

        // ---- transitions ---------------------------------------------------------
        {
            std::array<VkImageMemoryBarrier2, 2> barriers{};

            barriers[0].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
            barriers[0].srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
            barriers[0].srcAccessMask = VK_ACCESS_2_NONE;
            barriers[0].dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
            barriers[0].dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
            barriers[0].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            barriers[0].newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            barriers[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barriers[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barriers[0].image = swapImage;
            barriers[0].subresourceRange = colorRange;

            barriers[1].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
            barriers[1].srcStageMask = VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
            barriers[1].srcAccessMask = VK_ACCESS_2_NONE;
            barriers[1].dstStageMask = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT |
                                       VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
            barriers[1].dstAccessMask = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                                        VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
            barriers[1].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            barriers[1].newLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
            barriers[1].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barriers[1].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barriers[1].image = m_depthImage.handle();
            barriers[1].subresourceRange = depthRange;

            VkDependencyInfo dependency{};
            dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
            dependency.imageMemoryBarrierCount = static_cast<u32>(barriers.size());
            dependency.pImageMemoryBarriers = barriers.data();
            vkCmdPipelineBarrier2(cmd, &dependency);
        }

        // ---- dynamic rendering pass -------------------------------------------
        VkClearValue clearColor{};
        clearColor.color = {{0.004f, 0.006f, 0.015f, 1.0f}};

        VkRenderingAttachmentInfo colorAttachment{};
        colorAttachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
        colorAttachment.imageView = m_swapchain->imageView(imageIndex);
        colorAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.clearValue = clearColor;

        VkRenderingAttachmentInfo depthAttachment{};
        depthAttachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
        depthAttachment.imageView = m_depthImage.view();
        depthAttachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
        depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthAttachment.clearValue.depthStencil = {0.0f, 0}; // reverse-Z: far = 0

        VkRenderingInfo renderingInfo{};
        renderingInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        renderingInfo.renderArea = {{0, 0}, extent};
        renderingInfo.layerCount = 1;
        renderingInfo.colorAttachmentCount = 1;
        renderingInfo.pColorAttachments = &colorAttachment;
        renderingInfo.pDepthAttachment = &depthAttachment;

        vkCmdBeginRendering(cmd, &renderingInfo);

        VkViewport viewport{};
        viewport.width = static_cast<f32>(extent.width);
        viewport.height = static_cast<f32>(extent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        vkCmdSetViewport(cmd, 0, 1, &viewport);

        VkRect2D scissor{{0, 0}, extent};
        vkCmdSetScissor(cmd, 0, 1, &scissor);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_meshPipeline->handle());
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                m_meshPipeline->layout(), 0, 1, &frame.descriptorSet, 0,
                                nullptr);

        // ---- one indexed draw per mesh batch --------------------------------------
        auto drawBatches = [cmd](const std::vector<DrawBatch>& batches) {
            for (const DrawBatch& batch : batches)
            {
                const VkBuffer vertexBuffer = batch.mesh->vertexBuffer();
                const VkDeviceSize offset = 0;
                vkCmdBindVertexBuffers(cmd, 0, 1, &vertexBuffer, &offset);
                vkCmdBindIndexBuffer(cmd, batch.mesh->indexBuffer(), 0,
                                     VK_INDEX_TYPE_UINT32);
                // gl_InstanceIndex starts at firstInstance: the shader
                // indexes the instance SSBO directly.
                vkCmdDrawIndexed(cmd, batch.mesh->indexCount(), batch.instanceCount, 0, 0,
                                 batch.firstInstance);
            }
        };
        drawBatches(m_batches);

        // ---- transparent pass: back-to-front, blended, depth-read-only ------------
        if (!m_transparentBatches.empty())
        {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                              m_transparentPipeline->handle());
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                    m_transparentPipeline->layout(), 0, 1,
                                    &frame.descriptorSet, 0, nullptr);
            drawBatches(m_transparentBatches);
        }

        // ---- HUD pass: same rendering scope, screen-space pipeline ----------------
        if (!m_hudBatches.empty())
        {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_hudPipeline->handle());
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                    m_hudPipeline->layout(), 0, 1, &frame.descriptorSet, 0,
                                    nullptr);
            drawBatches(m_hudBatches);
        }

        vkCmdEndRendering(cmd);

        // ---- transition: COLOR_ATTACHMENT_OPTIMAL -> PRESENT_SRC ---------------
        {
            VkImageMemoryBarrier2 barrier{};
            barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
            barrier.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
            barrier.srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
            barrier.dstStageMask = VK_PIPELINE_STAGE_2_NONE;
            barrier.dstAccessMask = VK_ACCESS_2_NONE;
            barrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            barrier.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.image = swapImage;
            barrier.subresourceRange = colorRange;

            VkDependencyInfo dependency{};
            dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
            dependency.imageMemoryBarrierCount = 1;
            dependency.pImageMemoryBarriers = &barrier;
            vkCmdPipelineBarrier2(cmd, &dependency);
        }
    }

    void Renderer::recreateSwapchain()
    {
        u32 width = 0;
        u32 height = 0;
        m_window.framebufferSize(width, height);
        if (width == 0 || height == 0)
        {
            return;
        }

        m_device->waitIdle();
        const u32 previousImageCount = m_swapchain->imageCount();
        m_swapchain->recreate(width, height);
        createDepthResources();

        if (m_swapchain->imageCount() != previousImageCount)
        {
            destroySwapchainSemaphores();
            createSwapchainSemaphores();
        }

        SW_LOG_DEBUG(kLogCat, "Swapchain recreated: {}x{}", width, height);
    }

    VkFormat Renderer::findDepthFormat() const
    {
        constexpr std::array<VkFormat, 3> candidates = {
            VK_FORMAT_D32_SFLOAT,
            VK_FORMAT_D32_SFLOAT_S8_UINT,
            VK_FORMAT_D24_UNORM_S8_UINT,
        };
        for (VkFormat format : candidates)
        {
            VkFormatProperties props{};
            vkGetPhysicalDeviceFormatProperties(m_device->physicalHandle(), format, &props);
            if (props.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT)
            {
                return format;
            }
        }
        SW_THROW("No supported depth attachment format found");
    }

    void Renderer::createDepthResources()
    {
        m_depthImage.reset();
        m_depthImage = m_memory->createAttachmentImage(
            m_swapchain->extent(), findDepthFormat(),
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VK_IMAGE_ASPECT_DEPTH_BIT);
    }

    void Renderer::createDescriptorResources()
    {
        const VkDevice device = m_device->handle();

        // ---- layout: camera UBO (b0) + instance SSBO (b1) ----------------------
        std::array<VkDescriptorSetLayoutBinding, 2> bindings{};
        bindings[0].binding = 0;
        bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        bindings[0].descriptorCount = 1;
        bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        bindings[1].binding = 1;
        bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[1].descriptorCount = 1;
        bindings[1].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

        VkDescriptorSetLayoutCreateInfo layoutInfo{};
        layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        layoutInfo.bindingCount = static_cast<u32>(bindings.size());
        layoutInfo.pBindings = bindings.data();
        SW_VK_CHECK(
            vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &m_descriptorSetLayout));

        std::array<VkDescriptorPoolSize, 2> poolSizes{};
        poolSizes[0] = {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, kFramesInFlight};
        poolSizes[1] = {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, kFramesInFlight};

        VkDescriptorPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        poolInfo.maxSets = kFramesInFlight;
        poolInfo.poolSizeCount = static_cast<u32>(poolSizes.size());
        poolInfo.pPoolSizes = poolSizes.data();
        SW_VK_CHECK(vkCreateDescriptorPool(device, &poolInfo, nullptr, &m_descriptorPool));
    }

    void Renderer::destroyDescriptorResources()
    {
        const VkDevice device = m_device ? m_device->handle() : VK_NULL_HANDLE;
        if (device == VK_NULL_HANDLE)
        {
            return;
        }
        if (m_descriptorPool != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorPool(device, m_descriptorPool, nullptr);
            m_descriptorPool = VK_NULL_HANDLE;
        }
        if (m_descriptorSetLayout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device, m_descriptorSetLayout, nullptr);
            m_descriptorSetLayout = VK_NULL_HANDLE;
        }
    }

    void Renderer::ensureInstanceCapacity(FrameResources& frame, u32 required)
    {
        if (required <= frame.instanceCapacity)
        {
            return;
        }
        u32 capacity = std::max(frame.instanceCapacity, kInitialInstanceCapacity);
        while (capacity < required)
        {
            capacity *= 2;
        }

        // Safe to replace immediately: the frame's fence was waited on.
        frame.instanceBuffer = m_memory->createHostVisibleBuffer(
            static_cast<VkDeviceSize>(capacity) * sizeof(InstanceData),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        frame.instanceCapacity = capacity;
        writeFrameDescriptorSet(frame);

        SW_LOG_DEBUG(kLogCat, "Instance buffer grown to {} instances", capacity);
    }

    void Renderer::writeFrameDescriptorSet(FrameResources& frame) const
    {
        VkDescriptorBufferInfo uboInfo{};
        uboInfo.buffer = frame.cameraUbo.handle();
        uboInfo.offset = 0;
        uboInfo.range = sizeof(CameraUniforms);

        VkDescriptorBufferInfo ssboInfo{};
        ssboInfo.buffer = frame.instanceBuffer.handle();
        ssboInfo.offset = 0;
        ssboInfo.range = VK_WHOLE_SIZE;

        std::array<VkWriteDescriptorSet, 2> writes{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = frame.descriptorSet;
        writes[0].dstBinding = 0;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writes[0].descriptorCount = 1;
        writes[0].pBufferInfo = &uboInfo;
        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = frame.descriptorSet;
        writes[1].dstBinding = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[1].descriptorCount = 1;
        writes[1].pBufferInfo = &ssboInfo;

        vkUpdateDescriptorSets(m_device->handle(), static_cast<u32>(writes.size()),
                               writes.data(), 0, nullptr);
    }

    void Renderer::createFrameResources()
    {
        const VkDevice device = m_device->handle();

        for (FrameResources& frame : m_frames)
        {
            VkCommandPoolCreateInfo poolInfo{};
            poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
            poolInfo.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
            poolInfo.queueFamilyIndex = m_device->graphicsFamilyIndex();
            SW_VK_CHECK(vkCreateCommandPool(device, &poolInfo, nullptr, &frame.commandPool));

            VkCommandBufferAllocateInfo allocInfo{};
            allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
            allocInfo.commandPool = frame.commandPool;
            allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            allocInfo.commandBufferCount = 1;
            SW_VK_CHECK(vkAllocateCommandBuffers(device, &allocInfo, &frame.commandBuffer));

            VkSemaphoreCreateInfo semaphoreInfo{};
            semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
            SW_VK_CHECK(
                vkCreateSemaphore(device, &semaphoreInfo, nullptr, &frame.imageAvailable));

            VkFenceCreateInfo fenceInfo{};
            fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
            fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
            SW_VK_CHECK(vkCreateFence(device, &fenceInfo, nullptr, &frame.inFlight));

            // ---- camera UBO + instance SSBO + descriptor set -----------------
            frame.cameraUbo = m_memory->createHostVisibleBuffer(
                sizeof(CameraUniforms), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
            frame.instanceBuffer = m_memory->createHostVisibleBuffer(
                static_cast<VkDeviceSize>(kInitialInstanceCapacity) * sizeof(InstanceData),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            frame.instanceCapacity = kInitialInstanceCapacity;

            VkDescriptorSetAllocateInfo setAllocInfo{};
            setAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
            setAllocInfo.descriptorPool = m_descriptorPool;
            setAllocInfo.descriptorSetCount = 1;
            setAllocInfo.pSetLayouts = &m_descriptorSetLayout;
            SW_VK_CHECK(vkAllocateDescriptorSets(device, &setAllocInfo, &frame.descriptorSet));

            writeFrameDescriptorSet(frame);
        }
    }

    void Renderer::destroyFrameResources()
    {
        const VkDevice device = m_device ? m_device->handle() : VK_NULL_HANDLE;
        if (device == VK_NULL_HANDLE)
        {
            return;
        }
        for (FrameResources& frame : m_frames)
        {
            if (frame.inFlight != VK_NULL_HANDLE)
            {
                vkDestroyFence(device, frame.inFlight, nullptr);
            }
            if (frame.imageAvailable != VK_NULL_HANDLE)
            {
                vkDestroySemaphore(device, frame.imageAvailable, nullptr);
            }
            if (frame.commandPool != VK_NULL_HANDLE)
            {
                vkDestroyCommandPool(device, frame.commandPool, nullptr);
            }
            frame.cameraUbo.reset();
            frame.instanceBuffer.reset();
            frame = {};
        }
    }

    void Renderer::createSwapchainSemaphores()
    {
        const VkDevice device = m_device->handle();
        m_renderFinished.resize(m_swapchain->imageCount(), VK_NULL_HANDLE);
        for (VkSemaphore& semaphore : m_renderFinished)
        {
            VkSemaphoreCreateInfo semaphoreInfo{};
            semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
            SW_VK_CHECK(vkCreateSemaphore(device, &semaphoreInfo, nullptr, &semaphore));
        }
    }

    void Renderer::destroySwapchainSemaphores()
    {
        const VkDevice device = m_device ? m_device->handle() : VK_NULL_HANDLE;
        if (device == VK_NULL_HANDLE)
        {
            return;
        }
        for (VkSemaphore semaphore : m_renderFinished)
        {
            if (semaphore != VK_NULL_HANDLE)
            {
                vkDestroySemaphore(device, semaphore, nullptr);
            }
        }
        m_renderFinished.clear();
    }
} // namespace sw
