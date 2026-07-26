#pragma once

// ============================================================================
// Input/Input.hpp
// Polled keyboard/mouse state.
//
// Design:
//  - The Input system stores per-frame snapshots (current + previous) so
//    gameplay code can query "down", "just pressed" and "just released"
//    without owning any callbacks.
//  - It is fed by the Application, which forwards raw Window callbacks into
//    handle*() methods. Input has no dependency on GLFW or the Window; the
//    KeyCode values mirror GLFW's keycodes so forwarding is a no-op cast.
// ============================================================================

#include "Core/Types.hpp"

#include <array>

namespace sw
{
    /// Values intentionally mirror GLFW keycodes (see glfw3.h); the platform
    /// layer forwards them without translation.
    enum class KeyCode : i16
    {
        Unknown = -1,
        Space = 32,
        Apostrophe = 39,
        Comma = 44,
        Minus = 45,
        Period = 46,
        Slash = 47,
        Num0 = 48, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
        Semicolon = 59,
        Equal = 61,
        A = 65, B, C, D, E, F, G, H, I, J, K, L, M,
        N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
        Escape = 256,
        Enter = 257,
        Tab = 258,
        Backspace = 259,
        Insert = 260,
        Delete = 261,
        Right = 262,
        Left = 263,
        Down = 264,
        Up = 265,
        PageUp = 266,
        PageDown = 267,
        Home = 268,
        End = 269,
        CapsLock = 280,
        F1 = 290, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12,
        LeftShift = 340,
        LeftControl = 341,
        LeftAlt = 342,
        RightShift = 344,
        RightControl = 345,
        RightAlt = 346,
    };

    /// Values mirror GLFW mouse button ids.
    enum class MouseButton : u8
    {
        Left = 0,
        Right = 1,
        Middle = 2,
    };

    class Input
    {
    public:
        static constexpr usize kMaxKeys = 512;
        static constexpr usize kMaxMouseButtons = 8;

        /// Must be called once per frame *before* event polling: promotes the
        /// current snapshot to "previous" and clears per-frame deltas.
        void newFrame();

        // --- queries -------------------------------------------------------
        [[nodiscard]] bool isKeyDown(KeyCode key) const;
        [[nodiscard]] bool wasKeyPressed(KeyCode key) const;  // this frame
        [[nodiscard]] bool wasKeyReleased(KeyCode key) const; // this frame

        [[nodiscard]] bool isMouseButtonDown(MouseButton button) const;
        [[nodiscard]] bool wasMouseButtonPressed(MouseButton button) const;
        [[nodiscard]] bool wasMouseButtonReleased(MouseButton button) const;

        [[nodiscard]] f32 mouseX() const { return m_mouseX; }
        [[nodiscard]] f32 mouseY() const { return m_mouseY; }
        [[nodiscard]] f32 mouseDeltaX() const { return m_mouseDeltaX; }
        [[nodiscard]] f32 mouseDeltaY() const { return m_mouseDeltaY; }
        [[nodiscard]] f32 scrollDeltaY() const { return m_scrollDeltaY; }

        // --- event feed (called by Application from Window callbacks) ------
        void handleKey(i32 key, i32 action);
        void handleMouseButton(i32 button, i32 action);
        void handleCursorPos(f64 x, f64 y);
        void handleScroll(f64 xOffset, f64 yOffset);

        /// Drops the mouse-delta reference point; call after capturing or
        /// releasing the cursor to avoid a large fake delta.
        void resetMouseDelta();

    private:
        std::array<bool, kMaxKeys> m_keys{};
        std::array<bool, kMaxKeys> m_keysPrevious{};
        /// Latched press events: set by handleKey, cleared by newFrame.
        /// Guarantees that a press+release arriving within a single frame
        /// (fast tap, low frame rate, injected input) is never lost.
        std::array<bool, kMaxKeys> m_keyPressedEvents{};
        std::array<bool, kMaxMouseButtons> m_mouseButtons{};
        std::array<bool, kMaxMouseButtons> m_mouseButtonsPrevious{};
        std::array<bool, kMaxMouseButtons> m_mousePressedEvents{};

        f32 m_mouseX = 0.0f;
        f32 m_mouseY = 0.0f;
        f32 m_mouseDeltaX = 0.0f;
        f32 m_mouseDeltaY = 0.0f;
        f32 m_scrollDeltaY = 0.0f;
        bool m_hasMouseReference = false;
    };
} // namespace sw
