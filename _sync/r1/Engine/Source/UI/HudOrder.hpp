#pragma once

// ============================================================================
// UI/HudOrder.hpp
// THE PAINTER'S ALGORITHM, WRITTEN DOWN.
//
// The HUD has no depth buffer. What is drawn later is on top, and that is the
// only rule there is — so the order the game SUBMITS its panels, rows and
// glyphs in is not a hint, it is the whole layout.
//
// The renderer used to sort HUD items by mesh POINTER, to batch them, on the
// grounds that "their draw order has no meaning". That was true when the HUD
// was glyphs plus a handful of isolated markers. It stopped being true the
// day panels got rows: a panel and the rows on it are the same unit-quad
// mesh, and the glyphs are other meshes again — so whether text landed above
// or below its own panel depended on where those meshes happened to sit in
// the mesh table. Adding two parts to the catalogue moved them, and the build
// menu went blank. `std::sort` being unstable, the panel-versus-row order was
// also decided differently from frame to frame: the menu flickered.
//
// So the ordering is decided here, once, by a pure function that can be
// tested without a GPU — because the failure was invisible in review and
// glaring on screen, which is exactly the kind of bug a test should own.
//
// Two rules, and the reason each one is safe:
//
//   * A HIGHER LAYER IS ALWAYS DRAWN LATER. Text is a layer above
//     backgrounds, so a glyph can never be painted over by a panel, whatever
//     the mesh table looks like today.
//   * BACKGROUNDS KEEP THEIR SUBMISSION ORDER, exactly. Panels, rows and
//     chips overlap each other on purpose, so their order IS the design and
//     nothing may reorder it. Glyphs, on the other hand, never overlap each
//     other — text advances across the screen — so the TEXT layer is free to
//     group by mesh, which is where the batching actually mattered (one draw
//     call per distinct letter instead of one per character).
// ============================================================================

#include "Core/Types.hpp"

#include <algorithm>
#include <span>
#include <vector>

namespace sw::ui
{
    /// HUD layers, low to high. Anything added later goes BETWEEN these,
    /// never above Text: the point of the top layer is that it is the top.
    enum class HudLayer : u8
    {
        Background = 0, // panels, rows, chips, bars, markers
        Text = 1,       // glyphs: readable by construction
        Count
    };

    /// What ordering needs to know about one HUD item.
    struct HudItemKey
    {
        u8 layer = 0;
        /// Opaque batching key — the mesh pointer, in the renderer. Compared
        /// only for EQUALITY, never for order: comparing mesh addresses for
        /// order is the bug this header exists to make impossible.
        const void* mesh = nullptr;
    };

    /// The order to draw `keys` in, as indices into it.
    [[nodiscard]] inline std::vector<u32> hudDrawOrder(std::span<const HudItemKey> keys)
    {
        std::vector<u32> order(keys.size());
        for (u32 i = 0; i < keys.size(); ++i)
        {
            order[i] = i;
        }

        // Layer first, submission order within it. Stable, so a background
        // panel submitted before its rows still precedes them.
        std::stable_sort(order.begin(), order.end(),
                         [keys](u32 a, u32 b) { return keys[a].layer < keys[b].layer; });

        // ...then group the TEXT layer by mesh. Safe only because glyphs do
        // not overlap one another; do not extend this to Background.
        const auto textBegin =
            std::find_if(order.begin(), order.end(), [keys](u32 i) {
                return keys[i].layer >= static_cast<u8>(HudLayer::Text);
            });
        std::stable_sort(textBegin, order.end(), [keys](u32 a, u32 b) {
            if (keys[a].layer != keys[b].layer)
            {
                return keys[a].layer < keys[b].layer;
            }
            return std::less<const void*>{}(keys[a].mesh, keys[b].mesh);
        });
        return order;
    }
} // namespace sw::ui
