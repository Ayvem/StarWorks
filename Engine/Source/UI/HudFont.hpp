#pragma once

// ============================================================================
// UI/HudFont.hpp
// Procedural 5x7 bitmap font for the HUD — zero assets, zero textures.
// Each glyph becomes ONE mesh (its lit pixels as quads in a [0,1]x[0,1]
// cell, Y down to match Vulkan NDC), so a text line is one instanced draw
// item per character through the renderer's screen-space path.
//
// This is the debug/flight HUD tier of the UI module; the full retained UI
// with proper font atlases arrives much later and does not replace this —
// an engine always needs a dependency-free text path.
// ============================================================================

#include "Assets/MeshData.hpp"

namespace sw::ui
{
    /// Supported characters: A-Z, 0-9, space, . , - + / % : x(as letter).
    /// Unsupported characters render as space.
    [[nodiscard]] bool isGlyphSupported(char character);

    /// Builds the pixel-quad mesh of one glyph in a unit cell (Y down).
    /// White vertices; use the DrawItem tint for color.
    [[nodiscard]] MeshData buildGlyphMesh(char character);

    /// Glyph advance width relative to cell height (5x7 grid + 1px gap).
    inline constexpr f32 kGlyphAdvance = 6.0f / 7.0f;
} // namespace sw::ui
