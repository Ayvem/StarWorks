#pragma once

// ============================================================================
// Renderer/Renderer.hpp
// Renderer front-end: owns the Vulkan objects and the frame loop's GPU side.
//
// Mass-rendering pipeline (Milestone 5):
//  1. The game submits DrawItems (mesh + camera-relative transform + bounding
//     sphere) — positions were narrowed from f64 world space by the caller.
//  2. CPU frustum culling rejects everything outside the view.
//  3. Survivors are sorted by mesh and written to a per-frame storage buffer
//     of instance transforms; ONE indexed draw is issued per distinct mesh
//     (vertex shader reads its matrix via gl_InstanceIndex).
// GPU culling / indirect draws will slot in behind this same DrawItem API.
//
// Frame model: kFramesInFlight (2) frames in a ring; per frame: command
// pool + buffer, acquire semaphore, fence, camera UBO, instance SSBO and a
// descriptor set. Per swapchain image: a render-finished semaphore.
// Depth: reverse-Z (clear 0, GREATER) — mandatory at planetary scales.
// ============================================================================

#include "Core/Types.hpp"
#include "Math/Frustum.hpp"
#include "Math/Math.hpp"
#include "Renderer/Mesh.hpp"
#include "Renderer/Vulkan/VulkanMemory.hpp"

#include <vulkan/vulkan.h>

#include <array>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace sw
{
    class Window;
    class Camera;
} // namespace sw

namespace sw::vulkan
{
    class VulkanInstance;
    class VulkanDevice;
    class VulkanSwapchain;
    class VulkanPipeline;
} // namespace sw::vulkan

namespace sw
{
    struct RendererConfig
    {
        std::string applicationName = "StarWorks";
        bool enableValidation =
#if defined(SW_DEBUG)
            true;
#else
            false;
#endif
        /// Prefer a CPU/software Vulkan implementation (llvmpipe, SwiftShader)
        /// over hardware GPUs during device selection. Rendering code is
        /// identical either way — this only inverts the selection scoring.
        bool preferCpuDevice = false;
    };

    /// One draw request, in CAMERA-RELATIVE space (see Camera).
    struct DrawItem
    {
        const Mesh* mesh = nullptr;
        Mat4 transform{1.0f};
        /// Bounding sphere for frustum culling, camera-relative center.
        Vec3 boundsCenter{0.0f};
        f32 boundsRadius = 0.0f;
        /// Per-instance color multiplier (markers, highlights, damage...).
        Vec4 tint{1.0f, 1.0f, 1.0f, 1.0f};
        /// Screen-space (HUD) item: transform maps mesh space directly to
        /// Vulkan NDC; never culled, drawn unlit and alpha-blended on top
        /// of the world after the 3D pass.
        bool screenSpace = false;
        /// World-space TRANSLUCENT item (atmosphere shells, cloud layers):
        /// drawn after all opaques, back-to-front, alpha-blended, depth-
        /// tested but not depth-written, no face culling (visible from
        /// inside — the sky IS the far side of the shell).
        bool transparent = false;
        /// HUD PAINTER'S LAYER (see UI/HudOrder.hpp). Higher is drawn later.
        /// Backgrounds are 0 and text is 1, so a glyph can never end up
        /// underneath the panel it belongs to. Ignored unless `screenSpace`.
        u8 hudLayer = 0;
        /// A SOLID screen-space item: real geometry drawn inside a panel —
        /// the vehicle preview in the VAB. Same pipeline family as the rest
        /// of the HUD (unlit, no depth buffer, submission order) with ONE
        /// difference that makes 3D readable without a depth test: BACK
        /// FACES ARE CULLED. For a convex part that alone is correct — the
        /// front faces of a convex solid never overlap each other — and the
        /// caller sorts the parts back-to-front among themselves.
        ///
        /// The transform must have the SAME HANDEDNESS as the camera's, or
        /// the culling keeps precisely the faces it should throw away. The
        /// camera negates Y once, at the source (Camera.cpp: `m_projection
        /// [1][1] *= -1`), and the counter-clockwise front-face convention
        /// was settled against that. So a preview transform is a proper
        /// rotation times a scale with a NEGATIVE Y — the same single flip,
        /// not zero and not two. Ignored unless `screenSpace`.
        bool hudSolid = false;
    };

    struct RenderStats
    {
        u32 itemsSubmitted = 0;
        u32 itemsCulled = 0;
        u32 instancesDrawn = 0;
        u32 drawCalls = 0;
        /// CPU milliseconds spent in renderFrame (fence wait included /
        /// excluded) — the difference exposes GPU-bound frames.
        f32 cpuFenceWaitMs = 0.0f;
        f32 cpuPrepareMs = 0.0f; // culling + sort + instance upload
        f32 cpuTotalMs = 0.0f;
    };

    class Renderer
    {
    public:
        static constexpr u32 kFramesInFlight = 2;

        Renderer(Window& window, const RendererConfig& config);
        ~Renderer();

        Renderer(const Renderer&) = delete;
        Renderer& operator=(const Renderer&) = delete;
        Renderer(Renderer&&) = delete;
        Renderer& operator=(Renderer&&) = delete;

        /// Uploads a CPU mesh to the GPU. Valid for the renderer's lifetime.
        [[nodiscard]] Mesh createMesh(const MeshData& data);

        /// Culls, batches and submits one frame. Handles swapchain
        /// recreation internally; never throws on recoverable paths.
        void renderFrame(const Camera& camera, std::span<const DrawItem> items);

        /// Statistics of the most recently submitted frame.
        [[nodiscard]] const RenderStats& stats() const { return m_stats; }

        /// Called by the Application when the framebuffer size changes.
        void onFramebufferResized(u32 width, u32 height);

        /// A sphere that blocks sunlight (planet/moon), camera-relative.
        struct ShadowSphere
        {
            Vec3 center{0.0f};
            f32 radius = 0.0f;
        };
        static constexpr u32 kMaxShadowSpheres = 8;

        /// CAMERA-RELATIVE position of the star. Lighting is computed per
        /// fragment from this point (correct direction everywhere in the
        /// system — Terra and Mars are lit from different directions).
        void setSunPosition(const Vec3& cameraRelativePosition);

        /// Camera-relative occluder spheres for analytic shadows: any
        /// fragment whose ray to the sun crosses one is in eclipse — no
        /// light behind a planet. Extra entries beyond kMaxShadowSpheres
        /// are ignored.
        void setShadowSpheres(std::span<const ShadowSphere> spheres);

        /// Aerial perspective, set per frame by the game from the camera's
        /// position in an atmosphere. `fogColor` is the horizon tint (linear),
        /// `fogDensity` the exponential extinction (0 = space, no fog),
        /// `skyAmbient` extra ambient light scattered onto the scene.
        void setAtmosphere(const Vec3& fogColor, f32 fogDensity, const Vec3& skyAmbient);

        /// Shading quality tier (0 LOW / 1 MEDIUM / 2 HIGH) and the world
        /// clock the shaders animate with (cloud advection, waves). Both
        /// travel in the per-frame camera UBO; nothing about them touches
        /// the simulation.
        void setQuality(u32 level);
        void setTimeSeconds(f32 seconds);

        /// The body whose AIR the camera is looking through this frame
        /// (M29): camera-relative centre, surface radius, and the style id
        /// that selects its scattering coefficients (0 Terra, 2 Mars).
        /// A radius of 0 disables the physical atmosphere entirely.
        void setAtmosphereBody(const Vec3& cameraRelativeCentre, f32 radius,
                               i32 style);

        /// Blocks until the GPU has finished all submitted work.
        void waitIdle() const;

        [[nodiscard]] f32 aspectRatio() const;

    private:
        /// Layout must match the Camera uniform block in Shaders/Mesh.vert.
        struct CameraUniforms
        {
            Mat4 viewProjection{1.0f};
            Vec4 cameraPosition{0.0f}; // (0,0,0): rendering is camera-relative
            /// xyz: camera-relative sun position; w: shadow-sphere count.
            Vec4 sunPosition{0.0f, 1.0e12f, 0.0f, 0.0f};
            /// xyz: camera-relative center; w: radius.
            Vec4 shadowSpheres[kMaxShadowSpheres]{};
            /// xyz: horizon/fog color (linear); w: exponential fog density.
            Vec4 fogColorDensity{0.0f, 0.0f, 0.0f, 0.0f};
            /// xyz: sky-scattered ambient added to every lit fragment.
            Vec4 skyAmbient{0.0f, 0.0f, 0.0f, 0.0f};
            /// x: quality tier (0/1/2); y: world time in seconds (animation);
            /// z: atmosphere style id; w: reserved.
            Vec4 qualityTime{2.0f, 0.0f, 0.0f, 0.0f};
            /// xyz: camera-relative centre of the atmospheric body;
            /// w: its surface radius (0 = no atmosphere this frame).
            Vec4 atmosphereBody{0.0f, 0.0f, 0.0f, 0.0f};
        };

        /// Per-instance GPU data (std430 layout in Shaders/Mesh.vert).
        struct InstanceData
        {
            Mat4 model{1.0f};
            Vec4 tint{1.0f};
        };
        static_assert(sizeof(InstanceData) == 80, "Must match Mesh.vert std430 stride");

        struct FrameResources
        {
            VkCommandPool commandPool = VK_NULL_HANDLE;
            VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
            VkSemaphore imageAvailable = VK_NULL_HANDLE;
            VkFence inFlight = VK_NULL_HANDLE;
            vulkan::VulkanBuffer cameraUbo;     // persistently mapped
            vulkan::VulkanBuffer instanceBuffer; // persistently mapped SSBO
            u32 instanceCapacity = 0;
            VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
        };

        struct DrawBatch
        {
            const Mesh* mesh = nullptr;
            u32 firstInstance = 0;
            u32 instanceCount = 0;
            /// HUD only: this batch needs the back-face-culled pipeline.
            bool solid = false;
            /// Distance to the nearest instance in this batch. Opaque
            /// batches are submitted in increasing order of it — the depth
            /// buffer then rejects everything they hide.
            f32 nearDistanceSquared = 0.0f;
        };

        void createDepthResources();
        void createDescriptorResources();
        void destroyDescriptorResources();
        void createFrameResources();
        void destroyFrameResources();
        void createSwapchainSemaphores();
        void destroySwapchainSemaphores();
        void recreateSwapchain();
        void ensureInstanceCapacity(FrameResources& frame, u32 required);
        void writeFrameDescriptorSet(FrameResources& frame) const;

        /// Culling + mesh sort + instance upload; fills m_batches.
        void prepareBatches(FrameResources& frame, const Camera& camera,
                            std::span<const DrawItem> items);
        void recordCommands(VkCommandBuffer cmd, u32 imageIndex,
                            const FrameResources& frame) const;
        [[nodiscard]] VkFormat findDepthFormat() const;

        Window& m_window;

        std::unique_ptr<vulkan::VulkanInstance> m_instance;
        VkSurfaceKHR m_surface = VK_NULL_HANDLE;
        std::unique_ptr<vulkan::VulkanDevice> m_device;
        // m_memory must outlive every buffer/image below (declaration order
        // drives destruction order — do not reorder).
        std::unique_ptr<vulkan::VulkanMemory> m_memory;
        std::unique_ptr<vulkan::VulkanSwapchain> m_swapchain;
        vulkan::VulkanImage m_depthImage;
        std::unique_ptr<vulkan::VulkanPipeline> m_meshPipeline;
        std::unique_ptr<vulkan::VulkanPipeline> m_transparentPipeline;
        std::unique_ptr<vulkan::VulkanPipeline> m_hudPipeline;
        /// Same, back faces culled: solid geometry inside a HUD panel.
        std::unique_ptr<vulkan::VulkanPipeline> m_hudSolidPipeline;

        VkDescriptorSetLayout m_descriptorSetLayout = VK_NULL_HANDLE;
        VkDescriptorPool m_descriptorPool = VK_NULL_HANDLE;

        std::array<FrameResources, kFramesInFlight> m_frames{};
        /// One per swapchain image, signaled when rendering to it completes.
        std::vector<VkSemaphore> m_renderFinished;

        // Per-frame scratch (persistent to avoid reallocation).
        std::vector<u32> m_visibleIndices;
        std::vector<u32> m_transparentIndices;
        std::vector<u32> m_hudIndices;
        std::vector<DrawBatch> m_batches;
        std::vector<DrawBatch> m_transparentBatches;
        std::vector<DrawBatch> m_hudBatches;
        RenderStats m_stats{};

        Vec3 m_sunPosition{0.0f, 1.0e12f, 0.0f}; // camera-relative
        Vec3 m_fogColor{0.0f};
        f32 m_fogDensity = 0.0f;
        Vec3 m_skyAmbient{0.0f};
        f32 m_quality = 2.0f;
        f32 m_timeSeconds = 0.0f;
        Vec3 m_atmosphereCentre{0.0f};
        f32 m_atmosphereRadius = 0.0f;
        f32 m_atmosphereStyle = 0.0f;
        std::array<ShadowSphere, kMaxShadowSpheres> m_shadowSpheres{};
        u32 m_shadowSphereCount = 0;
        u32 m_currentFrame = 0;
        bool m_framebufferResized = false;
        u32 m_pendingWidth = 0;
        u32 m_pendingHeight = 0;
    };
} // namespace sw
