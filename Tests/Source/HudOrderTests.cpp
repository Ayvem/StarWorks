// ============================================================================
// HudOrderTests.cpp — the HUD is a painter's algorithm, and this is the paint
// order.
//
// The bug this file exists for: the renderer sorted HUD items by MESH
// POINTER to batch them. A panel and the rows on it are the same unit-quad
// mesh, and glyphs are other meshes at unrelated addresses — so whether text
// landed above or below its own panel depended on where those meshes happened
// to sit in the mesh table. Two parts were added to the catalogue, the table
// moved, and the build menu rendered as a blank box. `std::sort` being
// unstable, the panel-versus-row order was also decided afresh every frame:
// it flickered.
//
// Nothing about that was visible in code review, and it was glaring on
// screen. So the rule lives in a pure function and the assertions below own
// it. Every case here is written to FAIL against the old comparator.
// ============================================================================

#include "TestFramework.hpp"

#include <UI/HudOrder.hpp>

#include <algorithm>
#include <vector>

using namespace sw;
using namespace sw::ui;

namespace
{
    /// Fake mesh addresses. Their ORDER is the whole point: `quad` sits at a
    /// higher address than every glyph, which is precisely the arrangement
    /// that made the real menu go blank.
    const char kStorage[8] = {};
    const void* const kGlyphA = &kStorage[0];
    const void* const kGlyphB = &kStorage[1];
    const void* const kLine = &kStorage[2];
    const void* const kQuad = &kStorage[7];

    [[nodiscard]] HudItemKey background(const void* mesh)
    {
        return {static_cast<u8>(HudLayer::Background), mesh};
    }
    [[nodiscard]] HudItemKey text(const void* mesh)
    {
        return {static_cast<u8>(HudLayer::Text), mesh};
    }

    /// Position of submission index `item` in the draw order.
    [[nodiscard]] usize slotOf(const std::vector<u32>& order, u32 item)
    {
        const auto it = std::find(order.begin(), order.end(), item);
        return static_cast<usize>(it - order.begin());
    }
} // namespace

SW_TEST(HudTextIsNeverPaintedOverByItsOwnPanel)
{
    // A panel, then a row on it, then the row's label — the exact shape of
    // one line of the build menu. The quad mesh deliberately sits ABOVE the
    // glyph meshes in memory.
    const HudItemKey keys[] = {background(kQuad), background(kQuad), text(kGlyphA),
                               text(kGlyphB)};
    const std::vector<u32> order = hudDrawOrder(keys);

    SW_CHECK_EQ(order.size(), static_cast<usize>(4));
    // Both glyphs are drawn after both quads. Under the old mesh-pointer
    // sort this was exactly backwards, and the menu had no text.
    SW_CHECK(slotOf(order, 2) > slotOf(order, 0));
    SW_CHECK(slotOf(order, 2) > slotOf(order, 1));
    SW_CHECK(slotOf(order, 3) > slotOf(order, 0));
    SW_CHECK(slotOf(order, 3) > slotOf(order, 1));

    // ...and it does not depend on the addresses. Swap the roles so the
    // glyph mesh is the HIGH one and nothing changes.
    const HudItemKey flipped[] = {background(kGlyphA), text(kQuad)};
    const std::vector<u32> flippedOrder = hudDrawOrder(flipped);
    SW_CHECK(slotOf(flippedOrder, 1) > slotOf(flippedOrder, 0));
}

SW_TEST(HudBackgroundsKeepTheOrderTheyWereSubmittedIn)
{
    // Panel, row, chip, row, chip — all one mesh, and every one of them
    // overlaps the last. Their order IS the layout; nothing may touch it.
    const HudItemKey keys[] = {background(kQuad), background(kQuad), background(kQuad),
                               background(kQuad), background(kQuad)};
    const std::vector<u32> order = hudDrawOrder(keys);
    for (u32 i = 0; i < 5; ++i)
    {
        SW_CHECK_EQ(order[i], i);
    }

    // Backgrounds of DIFFERENT meshes must not be regrouped either: a
    // navball line drawn over a panel must stay over it. This is the case
    // batching would have been tempting for, and the one that would put a
    // marker underneath the box it labels.
    const HudItemKey mixed[] = {background(kQuad), background(kLine), background(kQuad)};
    const std::vector<u32> mixedOrder = hudDrawOrder(mixed);
    SW_CHECK_EQ(mixedOrder[0], 0u);
    SW_CHECK_EQ(mixedOrder[1], 1u);
    SW_CHECK_EQ(mixedOrder[2], 2u);
}

SW_TEST(HudOrderIsDeterministicAndBatchesTheGlyphs)
{
    // A realistic frame: a panel, ten rows, and a hundred glyphs alternating
    // between two letters. Run it twice — the old unstable sort gave a
    // different answer on different inputs, which is what made the menu
    // flicker rather than simply being wrong.
    std::vector<HudItemKey> keys;
    keys.push_back(background(kQuad));
    for (u32 i = 0; i < 10; ++i)
    {
        keys.push_back(background(kQuad));
    }
    for (u32 i = 0; i < 100; ++i)
    {
        keys.push_back(text((i % 2 == 0) ? kGlyphA : kGlyphB));
    }
    const std::vector<u32> first = hudDrawOrder(keys);
    SW_CHECK(first == hudDrawOrder(keys));

    // Every background comes before every glyph.
    SW_CHECK_EQ(first.size(), keys.size());
    for (usize i = 0; i < 11; ++i)
    {
        SW_CHECK(keys[first[i]].layer == static_cast<u8>(HudLayer::Background));
    }
    for (usize i = 11; i < first.size(); ++i)
    {
        SW_CHECK(keys[first[i]].layer == static_cast<u8>(HudLayer::Text));
    }

    // The glyphs ARE grouped by mesh — that is the batching the sort was
    // there for in the first place, and it is safe in the text layer because
    // glyphs do not overlap one another. Two letters, two runs, two draw
    // calls instead of a hundred.
    usize runs = 1;
    for (usize i = 12; i < first.size(); ++i)
    {
        if (keys[first[i]].mesh != keys[first[i - 1]].mesh)
        {
            runs += 1;
        }
    }
    SW_CHECK_EQ(runs, static_cast<usize>(2));

    // Adding one more glyph must not reshuffle the panel and its rows —
    // this is the flicker, reproduced: the frame's content changes every
    // time a distance readout ticks over.
    keys.push_back(text(kGlyphA));
    const std::vector<u32> second = hudDrawOrder(keys);
    for (usize i = 0; i < 11; ++i)
    {
        SW_CHECK_EQ(second[i], first[i]);
    }
}

SW_TEST(HudOrderHandlesTheDegenerateFrames)
{
    SW_CHECK(hudDrawOrder({}).empty());

    const HudItemKey one[] = {text(kGlyphA)};
    SW_CHECK_EQ(hudDrawOrder(one).size(), static_cast<usize>(1));

    // Text only, backgrounds only, and text submitted BEFORE the background
    // it belongs to — the layer wins over submission order, which is what
    // makes the rule usable: a caller cannot get it wrong by drawing in the
    // wrong sequence.
    const HudItemKey inverted[] = {text(kGlyphA), background(kQuad)};
    const std::vector<u32> order = hudDrawOrder(inverted);
    SW_CHECK_EQ(order[0], 1u); // the background, though submitted second
    SW_CHECK_EQ(order[1], 0u);
}
