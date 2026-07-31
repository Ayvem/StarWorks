#include "Core/Application.hpp"

#include <algorithm>
#include <format>

namespace sw
{
    namespace
    {
        constexpr const char* kLogCat = "Core";
    } // namespace

    Application::Application(const ApplicationConfig& config)
        : m_config(config)
    {
        Log::initialize(config.log);
        SW_LOG_INFO(kLogCat, "==== {} starting ====", config.name);

        m_threadPool = std::make_unique<ThreadPool>();

        WindowConfig windowConfig = config.window;
        if (windowConfig.title == "StarWorks" && config.name != "StarWorks")
        {
            windowConfig.title = config.name;
        }
        m_window = std::make_unique<Window>(windowConfig);

        RendererConfig rendererConfig{};
        rendererConfig.applicationName = config.name;
        rendererConfig.preferCpuDevice = config.preferCpuDevice;
        m_renderer = std::make_unique<Renderer>(*m_window, rendererConfig);

        wireWindowCallbacks();
    }

    Application::~Application()
    {
        // Renderer must go before the window (it owns the surface).
        m_renderer.reset();
        m_window.reset();
        m_threadPool.reset();
        SW_LOG_INFO(kLogCat, "==== {} shut down ====", m_config.name);
        Log::shutdown();
    }

    void Application::run()
    {
        m_clock.reset();
        SW_LOG_INFO(kLogCat, "Entering main loop");

        while (!m_window->shouldClose())
        {
            // Promote input snapshots *before* polling so callbacks write
            // into the current frame's state.
            m_input.newFrame();
            Window::pollEvents();
            m_clock.tick();

            // While minimized: don't burn CPU, wait for events.
            if (m_window->isMinimized())
            {
                Window::waitEvents();
                continue;
            }

            onUpdate(m_clock.deltaSeconds());
            onRender();

            // Window title doubles as a lightweight FPS display (dev builds).
            const f64 now = m_clock.totalSeconds();
            if (now - m_lastTitleUpdateSeconds > 0.5)
            {
                m_lastTitleUpdateSeconds = now;
                m_window->setTitle(std::format("{} — {:.0f} FPS", m_config.name,
                                               m_clock.smoothedFps()));
            }

            if (m_config.maxFrames > 0 && m_clock.frameIndex() >= m_config.maxFrames)
            {
                SW_LOG_INFO(kLogCat, "maxFrames ({}) reached, exiting loop",
                            m_config.maxFrames);
                m_window->requestClose();
            }
        }

        SW_LOG_INFO(kLogCat, "Main loop exited after {} frames ({:.1f} s, avg {:.0f} FPS)",
                    m_clock.frameIndex(), m_clock.totalSeconds(),
                    m_clock.frameIndex() / std::max(m_clock.totalSeconds(), 0.001));

        // Let in-flight GPU work drain before members start destructing.
        m_renderer->waitIdle();
    }

    void Application::wireWindowCallbacks()
    {
        WindowCallbacks callbacks{};
        callbacks.onKey = [this](i32 key, i32 /*scancode*/, i32 action, i32 /*mods*/) {
            m_input.handleKey(key, action);
        };
        callbacks.onChar = [this](u32 codepoint) { m_input.handleChar(codepoint); };
        callbacks.onMouseButton = [this](i32 button, i32 action, i32 /*mods*/) {
            m_input.handleMouseButton(button, action);
        };
        callbacks.onCursorPos = [this](f64 x, f64 y) { m_input.handleCursorPos(x, y); };
        callbacks.onScroll = [this](f64 x, f64 y) { m_input.handleScroll(x, y); };
        callbacks.onFramebufferResize = [this](u32 width, u32 height) {
            m_renderer->onFramebufferResized(width, height);
        };
        m_window->setCallbacks(std::move(callbacks));
    }
} // namespace sw
