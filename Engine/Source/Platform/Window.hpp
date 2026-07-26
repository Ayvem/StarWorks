#pragma once

// ============================================================================
// Platform/Window.hpp
// GLFW-backed OS window.
//
// Design:
//  - The window knows nothing about the renderer or input systems; it only
//    exposes raw callbacks (std::function) that the Application wires up.
//    This keeps Platform decoupled from Renderer/Input (low coupling).
//  - Vulkan appears in this interface only for surface creation, which is
//    intrinsically a windowing concern (GLFW owns the platform surface
//    extensions).
//  - GLFW global init/terminate is reference counted, so multiple windows
//    (editor, tools) are safe later.
// ============================================================================

#include "Core/Types.hpp"

#include <vulkan/vulkan_core.h>

#include <functional>
#include <string>
#include <vector>

struct GLFWwindow; // avoid leaking GLFW/glfw3.h into every consumer

namespace sw
{
    struct WindowConfig
    {
        std::string title = "StarWorks";
        u32 width = 1600;
        u32 height = 900;
        bool resizable = true;
    };

    /// Raw window events. All callbacks are optional.
    struct WindowCallbacks
    {
        std::function<void(i32 key, i32 scancode, i32 action, i32 mods)> onKey;
        std::function<void(i32 button, i32 action, i32 mods)> onMouseButton;
        std::function<void(f64 x, f64 y)> onCursorPos;
        std::function<void(f64 xOffset, f64 yOffset)> onScroll;
        std::function<void(u32 width, u32 height)> onFramebufferResize;
    };

    class Window
    {
    public:
        explicit Window(const WindowConfig& config);
        ~Window();

        Window(const Window&) = delete;
        Window& operator=(const Window&) = delete;
        Window(Window&&) = delete;
        Window& operator=(Window&&) = delete;

        /// Processes pending OS events for all windows.
        static void pollEvents();
        /// Blocks until at least one event arrives (used while minimized).
        static void waitEvents();

        /// Instance extensions GLFW requires for surface creation.
        [[nodiscard]] static std::vector<const char*> requiredVulkanInstanceExtensions();

        [[nodiscard]] bool shouldClose() const;
        void requestClose();

        void setCallbacks(WindowCallbacks callbacks);

        /// Size of the framebuffer in pixels (may differ from window size on
        /// high-DPI displays). Zero while minimized.
        void framebufferSize(u32& outWidth, u32& outHeight) const;
        [[nodiscard]] bool isMinimized() const;

        void setTitle(const std::string& title);

        /// Captures (hides + locks) or releases the mouse cursor.
        void setCursorCaptured(bool captured);
        [[nodiscard]] bool isCursorCaptured() const { return m_cursorCaptured; }

        /// Creates the Vulkan presentation surface for this window.
        /// Throws sw::Exception on failure. The caller owns the surface.
        [[nodiscard]] VkSurfaceKHR createVulkanSurface(VkInstance instance) const;

        [[nodiscard]] GLFWwindow* nativeHandle() const { return m_handle; }

    private:
        static void installGlfwCallbacks(GLFWwindow* handle);

        GLFWwindow* m_handle = nullptr;
        WindowCallbacks m_callbacks{};
        bool m_cursorCaptured = false;
    };
} // namespace sw
