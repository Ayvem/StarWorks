#include "Input/Input.hpp"

namespace sw
{
    namespace
    {
        // GLFW action values, duplicated locally to keep Input GLFW-free.
        constexpr i32 kActionRelease = 0;
        constexpr i32 kActionPress = 1;
        // kActionRepeat (2) intentionally ignored: repeats are a text-input
        // concern, not a game-input one.
    } // namespace

    void Input::newFrame()
    {
        m_keysPrevious = m_keys;
        m_mouseButtonsPrevious = m_mouseButtons;
        m_keyPressedEvents.fill(false);
        m_mousePressedEvents.fill(false);
        m_mouseDeltaX = 0.0f;
        m_mouseDeltaY = 0.0f;
        m_scrollDeltaY = 0.0f;
    }

    bool Input::isKeyDown(KeyCode key) const
    {
        const auto index = static_cast<i32>(key);
        if (index < 0 || index >= static_cast<i32>(kMaxKeys))
        {
            return false;
        }
        return m_keys[static_cast<usize>(index)];
    }

    bool Input::wasKeyPressed(KeyCode key) const
    {
        const auto index = static_cast<i32>(key);
        if (index < 0 || index >= static_cast<i32>(kMaxKeys))
        {
            return false;
        }
        const auto i = static_cast<usize>(index);
        // The latched event catches sub-frame taps that the edge detect
        // (down now, up before) would miss.
        return m_keyPressedEvents[i] || (m_keys[i] && !m_keysPrevious[i]);
    }

    bool Input::wasKeyReleased(KeyCode key) const
    {
        const auto index = static_cast<i32>(key);
        if (index < 0 || index >= static_cast<i32>(kMaxKeys))
        {
            return false;
        }
        const auto i = static_cast<usize>(index);
        return !m_keys[i] && m_keysPrevious[i];
    }

    bool Input::isMouseButtonDown(MouseButton button) const
    {
        return m_mouseButtons[static_cast<usize>(button)];
    }

    bool Input::wasMouseButtonPressed(MouseButton button) const
    {
        const auto i = static_cast<usize>(button);
        return m_mousePressedEvents[i] || (m_mouseButtons[i] && !m_mouseButtonsPrevious[i]);
    }

    bool Input::wasMouseButtonReleased(MouseButton button) const
    {
        const auto i = static_cast<usize>(button);
        return !m_mouseButtons[i] && m_mouseButtonsPrevious[i];
    }

    void Input::handleKey(i32 key, i32 action)
    {
        if (key < 0 || key >= static_cast<i32>(kMaxKeys))
        {
            return;
        }
        if (action == kActionPress)
        {
            m_keys[static_cast<usize>(key)] = true;
            m_keyPressedEvents[static_cast<usize>(key)] = true;
        }
        else if (action == kActionRelease)
        {
            m_keys[static_cast<usize>(key)] = false;
        }
    }

    void Input::handleMouseButton(i32 button, i32 action)
    {
        if (button < 0 || button >= static_cast<i32>(kMaxMouseButtons))
        {
            return;
        }
        if (action == kActionPress)
        {
            m_mouseButtons[static_cast<usize>(button)] = true;
            m_mousePressedEvents[static_cast<usize>(button)] = true;
        }
        else if (action == kActionRelease)
        {
            m_mouseButtons[static_cast<usize>(button)] = false;
        }
    }

    void Input::handleCursorPos(f64 x, f64 y)
    {
        const auto newX = static_cast<f32>(x);
        const auto newY = static_cast<f32>(y);

        if (m_hasMouseReference)
        {
            // Accumulate: several cursor events can arrive within one frame.
            m_mouseDeltaX += newX - m_mouseX;
            m_mouseDeltaY += newY - m_mouseY;
        }

        m_mouseX = newX;
        m_mouseY = newY;
        m_hasMouseReference = true;
    }

    void Input::handleScroll(f64 /*xOffset*/, f64 yOffset)
    {
        m_scrollDeltaY += static_cast<f32>(yOffset);
    }

    void Input::resetMouseDelta()
    {
        m_mouseDeltaX = 0.0f;
        m_mouseDeltaY = 0.0f;
        m_hasMouseReference = false;
    }
} // namespace sw
