#pragma once

// ============================================================================
// Core/Application.hpp
// Engine entry object: owns the Window, Input, Clock and Renderer, wires
// them together and runs the main loop. Game code derives from Application
// and overrides the on*() hooks; it never touches GLFW or Vulkan directly.
//
// Loop shape (variable-rate for now):
//   newFrame -> poll events -> tick clock -> onUpdate(dt) -> onRender()
//
// The future Simulation module will insert fixed-step accumulators between
// tick and onUpdate; rendering will remain an observer of simulation state,
// never its driver.
// ============================================================================

#include "Core/Clock.hpp"
#include "Core/Log.hpp"
#include "Core/ThreadPool.hpp"
#include "Core/Types.hpp"
#include "Input/Input.hpp"
#include "Platform/Window.hpp"
#include "Renderer/Renderer.hpp"

#include <memory>
#include <string>

namespace sw
{
    struct ApplicationConfig
    {
        std::string name = "StarWorks";
        WindowConfig window{};
        Log::Config log{};
        /// If > 0, the loop exits after this many frames (soak tests / CI).
        u64 maxFrames = 0;
        /// Prefer a software (CPU) Vulkan implementation for rendering.
        bool preferCpuDevice = false;
        /// Shading quality: 0 = LOW (software rasterizers / CI), 1 = MEDIUM,
        /// 2 = HIGH. Drives the per-fragment planet path's octave budget,
        /// terrain self-shadowing and cloud shadows.
        u32 renderQuality = 2;
        /// If set (with maxFrames), the last frame is written here as a PNG.
        /// See Renderer::requestCapture for why this exists.
        std::string capturePath;
    };

    class Application
    {
    public:
        explicit Application(const ApplicationConfig& config);
        virtual ~Application();

        Application(const Application&) = delete;
        Application& operator=(const Application&) = delete;

        /// Runs the main loop until the window closes (or maxFrames is hit).
        void run();

    protected:
        /// Per-frame logic (input handling, camera, gameplay).
        virtual void onUpdate(f32 deltaSeconds) = 0;
        /// Per-frame rendering; typically calls renderer().renderFrame(...).
        virtual void onRender() = 0;

        [[nodiscard]] Window& window() { return *m_window; }
        [[nodiscard]] Input& input() { return m_input; }
        [[nodiscard]] Renderer& renderer() { return *m_renderer; }
        [[nodiscard]] const Clock& clock() const { return m_clock; }
        [[nodiscard]] ThreadPool& threadPool() { return *m_threadPool; }

    private:
        void wireWindowCallbacks();

        ApplicationConfig m_config;
        // Order matters: window must outlive the renderer; the thread pool
        // must outlive everything that may submit work to it.
        std::unique_ptr<ThreadPool> m_threadPool;
        std::unique_ptr<Window> m_window;
        std::unique_ptr<Renderer> m_renderer;
        Input m_input;
        Clock m_clock;
        f64 m_lastTitleUpdateSeconds = 0.0;
    };
} // namespace sw
