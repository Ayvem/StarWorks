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

#include <Input/Input.hpp>
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

// ============================================================================
// A QUICK CLICK, TOLD APART FROM A DRAG
//
// The right button turns the camera when held and works a part's switches when
// tapped, and the only thing separating the two is how long it was down and how
// far the mouse went. The first attempt used six pixels of travel and no time
// bound at all — and never fired once, because six pixels is inside the slop of
// an ordinary click on an ordinary mouse.
// ============================================================================
SW_TEST(AQuickClickIsShortAndStillAndADragIsNeither)
{
    using sw::Input;
    // A tap: a few hundredths of a second, a few pixels of shake.
    SW_CHECK(Input::isQuickClick(0.04f, 0.0f));
    SW_CHECK(Input::isQuickClick(0.12f, 9.0f));
    // ...and the shake is allowed to be real. This is the case the six-pixel
    // bound rejected, which is why the feature appeared to do nothing at all.
    SW_CHECK(Input::isQuickClick(0.10f, 30.0f));

    // A SLOW, CAREFUL DRAG covers almost no distance and is still a drag.
    SW_CHECK(!Input::isQuickClick(1.50f, 4.0f));
    // A FAST FLICK covers a lot in very little time and is also a drag.
    SW_CHECK(!Input::isQuickClick(0.08f, 400.0f));
    // Both bounds are needed: neither of the two above is caught by the other.
    SW_CHECK(!Input::isQuickClick(0.9f, 300.0f));

    // Exactly on the boundary counts as a click — a threshold that excluded
    // its own value would make the documented limit a lie by one epsilon.
    SW_CHECK(Input::isQuickClick(Input::kQuickClickSeconds, Input::kQuickClickPixels));
    SW_CHECK(!Input::isQuickClick(Input::kQuickClickSeconds + 0.01f, 0.0f));
    SW_CHECK(!Input::isQuickClick(0.0f, Input::kQuickClickPixels + 1.0f));
}

// ============================================================================
// WHICH PANEL A CLICK BELONGS TO
//
// The second HUD rule that was invisible in review and infuriating on screen.
// Every clickable rectangle carries a number, the ranges are open-ended upward
// so each panel has room to grow, and a chain of `if (id >= N)` tests is then
// only correct written in DESCENDING order of N.
//
// It was not. The part menu's ids start at 900 and its test sat BELOW the build
// menu's `id >= 400`, so every press of OPEN or TURN OFF on a solar array was
// read as "arm building number 500" and silently did nothing. The menu opened,
// the rows highlighted under the cursor, and the panel never moved.
//
// Every case below fails against that ordering.
// ============================================================================

#include <UI/HudRoute.hpp>

SW_TEST(TheBuildMenuDoesNotSwallowThePartMenusButtons)
{
    // THE BUG, EXACTLY. 900 is above 400, so a chain that tests 400 first
    // answers BuildArm for every row of the part menu.
    for (u32 slot = 0; slot < ui::kHudPartAnimationSlots; ++slot)
    {
        const ui::HudRoute route = ui::routeHudClick(900u + slot, false, false);
        SW_CHECK(route.action == ui::HudAction::PartAnimation);
        SW_CHECK_EQ(route.index, slot);
    }

    // ...and the build menu still owns its own range, which is the half of
    // the fix that is easy to break while making the other half work.
    const ui::HudRoute build = ui::routeHudClick(400u + 17u, false, false);
    SW_CHECK(build.action == ui::HudAction::BuildArm);
    SW_CHECK_EQ(build.index, 17u);
}

SW_TEST(NineHundredMeansTheAssemblyCatalogueWhileItsPanelIsOpen)
{
    // 900+ is the one range two panels genuinely share, and the machine's
    // configuration panel being open is what separates them: the part menu is
    // only ever collected while it is closed.
    const ui::HudRoute vab = ui::routeHudClick(903u, true, false);
    SW_CHECK(vab.action == ui::HudAction::VabSelect);
    SW_CHECK_EQ(vab.index, 3u);

    const ui::HudRoute part = ui::routeHudClick(903u, false, false);
    SW_CHECK(part.action == ui::HudAction::PartAnimation);

    // The part menu's range is BOUNDED at the number of animations a part can
    // carry. The very next id is the docked port's release — one more row on
    // the same panel — and the one after THAT still falls through, which is
    // the property that matters: the range did not grow to fill the gap.
    const ui::HudRoute release =
        ui::routeHudClick(900u + ui::kHudPartAnimationSlots, false, false);
    SW_CHECK(release.action == ui::HudAction::PartUndock);
    const ui::HudRoute beyond =
        ui::routeHudClick(902u + ui::kHudPartAnimationSlots, false, false);
    SW_CHECK(beyond.action == ui::HudAction::BuildArm);
}

SW_TEST(HudRoutingIsOrderedByDescendingRange)
{
    // THE INVARIANT, checked over every id any panel can emit: an id must
    // route to the OWNER of its range and never to a lower, wider one. Written
    // as a table so adding a range means adding a row here.
    struct Case
    {
        u32 id;
        bool configTarget;
        bool mapView;
        ui::HudAction expected;
    };
    const Case cases[] = {
        {2000, false, false, ui::HudAction::Shell},
        {2431, true, true, ui::HudAction::Shell},   // even over an open panel
        {1500, false, false, ui::HudAction::Geology},
        {1523, true, true, ui::HudAction::Geology}, // its own screen, over both
        {1999, false, false, ui::HudAction::Geology},
        {1100, false, false, ui::HudAction::NetSyncTo},
        {1499, false, false, ui::HudAction::NetSyncTo}, // ...and not one over
        {1000, false, false, ui::HudAction::NetHost},
        {1001, false, false, ui::HudAction::NetJoin},
        {1002, false, false, ui::HudAction::NetLeave},
        {1003, false, false, ui::HudAction::NetAddress},
        {900, true, false, ui::HudAction::VabSelect},
        {900, false, false, ui::HudAction::PartAnimation},
        {904, false, false, ui::HudAction::PartUndock},
        {905, false, false, ui::HudAction::PartJettison},
        {904, true, false, ui::HudAction::VabSelect}, // ...unless a hall is open
        {899, true, false, ui::HudAction::VabCancel},
        {898, true, false, ui::HudAction::VabProduce},
        {617, true, false, ui::HudAction::RecipeChoice},
        {601, true, false, ui::HudAction::PowerPriority},
        {600, true, false, ui::HudAction::RecipeStop},
        {400, false, false, ui::HudAction::BuildArm},
        {301, false, true, ui::HudAction::MapWarpToNode},
        {300, false, true, ui::HudAction::MapCycleVessel},
        {207, false, false, ui::HudAction::HangarAction},
        {100, false, false, ui::HudAction::PalettePart},
        {2, false, false, ui::HudAction::SasMode},
    };
    for (const Case& c : cases)
    {
        SW_CHECK(ui::routeHudClick(c.id, c.configTarget, c.mapView).action == c.expected);
    }

    // THE MAP OWNS ONLY ITS OWN BUTTONS. A hangar or cockpit id pressed while
    // the map is up must do nothing at all — not the nearest thing below it.
    SW_CHECK(ui::routeHudClick(207u, false, true).action == ui::HudAction::None);
    SW_CHECK(ui::routeHudClick(100u, false, true).action == ui::HudAction::None);
    SW_CHECK(ui::routeHudClick(2u, false, true).action == ui::HudAction::None);
    // ...but the map's own two, and everything above them, still work there.
    SW_CHECK(ui::routeHudClick(300u, false, true).action == ui::HudAction::MapCycleVessel);
    SW_CHECK(ui::routeHudClick(901u, false, true).action == ui::HudAction::PartAnimation);
}
