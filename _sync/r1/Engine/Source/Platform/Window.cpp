#include "Platform/Window.hpp"

#include "Core/Error.hpp"
#include "Core/Log.hpp"

#include <GLFW/glfw3.h>

namespace sw
{
    namespace
    {
        constexpr const char* kLogCat = "Platform";

        i32 g_glfwRefCount = 0;

        void glfwErrorCallback(int errorCode, const char* description)
        {
            SW_LOG_ERROR("GLFW", "Error {:#x}: {}", errorCode, description ? description : "?");
        }

        void acquireGlfw()
        {
            if (g_glfwRefCount == 0)
            {
                glfwSetErrorCallback(glfwErrorCallback);
                if (glfwInit() != GLFW_TRUE)
                {
                    SW_THROW("glfwInit() failed");
                }
                SW_LOG_INFO(kLogCat, "GLFW initialized (version: {})", glfwGetVersionString());
            }
            ++g_glfwRefCount;
        }

        void releaseGlfw()
        {
            --g_glfwRefCount;
            if (g_glfwRefCount == 0)
            {
                glfwTerminate();
                SW_LOG_INFO(kLogCat, "GLFW terminated");
            }
        }
    } // namespace

    Window::Window(const WindowConfig& config)
    {
        acquireGlfw();

        if (glfwVulkanSupported() != GLFW_TRUE)
        {
            releaseGlfw();
            SW_THROW("Vulkan is not supported on this system (no loader / ICD found)");
        }

        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API); // Vulkan: no GL context
        glfwWindowHint(GLFW_RESIZABLE, config.resizable ? GLFW_TRUE : GLFW_FALSE);

        m_handle = glfwCreateWindow(static_cast<int>(config.width),
                                    static_cast<int>(config.height),
                                    config.title.c_str(), nullptr, nullptr);
        if (m_handle == nullptr)
        {
            releaseGlfw();
            SW_THROW("Failed to create window '{}' ({}x{})", config.title, config.width,
                     config.height);
        }

        glfwSetWindowUserPointer(m_handle, this);
        installGlfwCallbacks(m_handle);

        // Raw mouse motion gives cleaner camera control when captured.
        if (glfwRawMouseMotionSupported() == GLFW_TRUE)
        {
            glfwSetInputMode(m_handle, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
        }

        SW_LOG_INFO(kLogCat, "Window created: '{}' {}x{}", config.title, config.width,
                    config.height);
    }

    Window::~Window()
    {
        if (m_handle != nullptr)
        {
            glfwDestroyWindow(m_handle);
            m_handle = nullptr;
        }
        releaseGlfw();
    }

    void Window::pollEvents()
    {
        glfwPollEvents();
    }

    void Window::waitEvents()
    {
        glfwWaitEvents();
    }

    std::vector<const char*> Window::requiredVulkanInstanceExtensions()
    {
        u32 count = 0;
        const char** extensions = glfwGetRequiredInstanceExtensions(&count);
        if (extensions == nullptr || count == 0)
        {
            SW_THROW("glfwGetRequiredInstanceExtensions() returned nothing — "
                     "Vulkan presentation is unavailable");
        }
        return {extensions, extensions + count};
    }

    bool Window::shouldClose() const
    {
        return glfwWindowShouldClose(m_handle) == GLFW_TRUE;
    }

    void Window::requestClose()
    {
        glfwSetWindowShouldClose(m_handle, GLFW_TRUE);
    }

    void Window::setCallbacks(WindowCallbacks callbacks)
    {
        m_callbacks = std::move(callbacks);
    }

    void Window::framebufferSize(u32& outWidth, u32& outHeight) const
    {
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(m_handle, &width, &height);
        outWidth = static_cast<u32>(width);
        outHeight = static_cast<u32>(height);
    }

    bool Window::isMinimized() const
    {
        u32 width = 0;
        u32 height = 0;
        framebufferSize(width, height);
        return width == 0 || height == 0;
    }

    void Window::setTitle(const std::string& title)
    {
        glfwSetWindowTitle(m_handle, title.c_str());
    }

    void Window::setCursorCaptured(bool captured)
    {
        if (m_cursorCaptured == captured)
        {
            return;
        }
        m_cursorCaptured = captured;
        glfwSetInputMode(m_handle, GLFW_CURSOR,
                         captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
    }

    VkSurfaceKHR Window::createVulkanSurface(VkInstance instance) const
    {
        VkSurfaceKHR surface = VK_NULL_HANDLE;
        const VkResult result = glfwCreateWindowSurface(instance, m_handle, nullptr, &surface);
        if (result != VK_SUCCESS)
        {
            SW_THROW("glfwCreateWindowSurface() failed (VkResult {})",
                     static_cast<i32>(result));
        }
        return surface;
    }

    void Window::installGlfwCallbacks(GLFWwindow* handle)
    {
        glfwSetKeyCallback(handle, [](GLFWwindow* w, int key, int scancode, int action, int mods) {
            auto* self = static_cast<Window*>(glfwGetWindowUserPointer(w));
            if (self != nullptr && self->m_callbacks.onKey)
            {
                self->m_callbacks.onKey(key, scancode, action, mods);
            }
        });

        glfwSetCharCallback(handle, [](GLFWwindow* w, unsigned int codepoint) {
            auto* self = static_cast<Window*>(glfwGetWindowUserPointer(w));
            if (self != nullptr && self->m_callbacks.onChar)
            {
                self->m_callbacks.onChar(static_cast<u32>(codepoint));
            }
        });

        glfwSetMouseButtonCallback(handle, [](GLFWwindow* w, int button, int action, int mods) {
            auto* self = static_cast<Window*>(glfwGetWindowUserPointer(w));
            if (self != nullptr && self->m_callbacks.onMouseButton)
            {
                self->m_callbacks.onMouseButton(button, action, mods);
            }
        });

        glfwSetCursorPosCallback(handle, [](GLFWwindow* w, double x, double y) {
            auto* self = static_cast<Window*>(glfwGetWindowUserPointer(w));
            if (self != nullptr && self->m_callbacks.onCursorPos)
            {
                self->m_callbacks.onCursorPos(x, y);
            }
        });

        glfwSetScrollCallback(handle, [](GLFWwindow* w, double xOffset, double yOffset) {
            auto* self = static_cast<Window*>(glfwGetWindowUserPointer(w));
            if (self != nullptr && self->m_callbacks.onScroll)
            {
                self->m_callbacks.onScroll(xOffset, yOffset);
            }
        });

        glfwSetFramebufferSizeCallback(handle, [](GLFWwindow* w, int width, int height) {
            auto* self = static_cast<Window*>(glfwGetWindowUserPointer(w));
            if (self != nullptr && self->m_callbacks.onFramebufferResize)
            {
                self->m_callbacks.onFramebufferResize(static_cast<u32>(width),
                                                      static_cast<u32>(height));
            }
        });
    }
} // namespace sw
