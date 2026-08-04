#pragma once

// ============================================================================
// UI/HudRoute.hpp
// WHICH PANEL A CLICK BELONGS TO, WRITTEN DOWN.
//
// Every clickable thing on the HUD is a rectangle with a NUMBER on it, and
// the number says what pressing it means. The ranges are open-ended upward —
// "400 and above arms a building", "100 and above takes a part in hand" —
// because each panel wants room to grow, and a chain of `if (id >= N)` tests
// is then only correct if it is written in DESCENDING order of N.
//
// It was not. The part menu's ids start at 900 and its branch sat BELOW the
// build menu's `id >= 400`, so every press of OPEN or TURN OFF was read as
// "arm building number 500", returned, and did nothing the pilot could see.
// The menu appeared, the buttons highlighted under the cursor, and the panel
// never moved: a feature that looked implemented and was unreachable.
//
// That is the third time an open-ended range has swallowed one above it, and
// the comments in the dispatcher warn about it twice in the code that then
// got it wrong. So the decision is no longer expressed as the order of a
// hundred and forty lines of side effects. It is this pure function, the
// dispatcher switches on what it returns, and the ordering is a thing a test
// can hold — because a click that silently does the wrong thing is invisible
// in review and infuriating on screen.
//
// ADDING A RANGE: put it in descending order of its lower bound and add a
// case to HudRoutingIsOrderedByDescendingRange. If a new range would sit
// inside an existing open-ended one, bound the inner range explicitly the way
// PartAnimation is bounded below.
// ============================================================================

#include "Core/Types.hpp"

namespace sw::ui
{
    /// What a HUD button press means. The dispatcher owns the side effects;
    /// this is only the decision about WHICH one.
    enum class HudAction : u8
    {
        None = 0,      // consumed by nothing (the map, off-panel)
        Shell,         // the pause/main menu, id passed through
        Geology,       // the survey screen, id passed through: close, channel, body
        NetSyncTo,     // index: a row of the multiplayer roster
        NetHost,
        NetJoin,
        NetLeave,
        NetAddress,
        VabSelect,     // index: a design in the vehicle assembly catalogue
        VabProduce,
        VabCancel,
        RecipeChoice,  // index: a recipe on the configured machine
        RecipeStop,
        PowerPriority,
        PartAnimation, // index: an animation on the part whose menu is open
        PartUndock,    // the release button on a docked port's own menu
        PartJettison,  // the release button on a closed fairing's own menu
        BuildArm,      // index: a building definition to place
        MapCycleVessel,
        MapWarpToNode,
        HangarAction,  // id passed through: undo, new, load, symmetry, save
        PalettePart,   // index: a part in the hangar palette
        SasMode,       // id passed through: the autopilot mode
    };

    struct HudRoute
    {
        HudAction action = HudAction::None;
        /// The action's argument: an index into whatever list it names, or
        /// the raw id for the actions that switch on it themselves.
        u32 index = 0;
    };

    /// The highest number of animations one part can carry. Kept here rather
    /// than reached for from the parts module so this header stays free of
    /// gameplay: the routing table only needs to know how WIDE the part
    /// menu's range is, and a range that is wider than the menu would eat the
    /// build menu's ids all over again.
    inline constexpr u32 kHudPartAnimationSlots = 4;

    /// Decide what a press on `id` means. `hasConfigTarget` is true while a
    /// machine's configuration panel is open — the one piece of context two
    /// panels genuinely share a range over — and `mapView` is true on the
    /// orbital map, which owns only its own buttons and swallows the rest.
    [[nodiscard]] constexpr HudRoute routeHudClick(u32 id, bool hasConfigTarget,
                                                   bool mapView)
    {
        // THE SHELL OWNS 2000+, tested before everything else because while a
        // menu is up nothing behind it is clickable.
        if (id >= 2000) { return {HudAction::Shell, id}; }
        // THE GEOLOGY SCREEN owns 1500-1999, and it sits here rather than
        // below the multiplayer panel because 1100+ is open-ended upward: a
        // range under it would be swallowed exactly the way the part menu was.
        // Like the shell and the hangar it takes the raw id, because its three
        // controls — close, pick a channel, pick a body — are one panel's
        // business and splitting them into three actions buys nothing.
        if (id >= 1500) { return {HudAction::Geology, id}; }
        // The multiplayer panel owns 1000+.
        if (id >= 1100) { return {HudAction::NetSyncTo, id - 1100u}; }
        if (id == 1000) { return {HudAction::NetHost, id}; }
        if (id == 1001) { return {HudAction::NetJoin, id}; }
        if (id == 1002) { return {HudAction::NetLeave, id}; }
        if (id == 1003) { return {HudAction::NetAddress, id}; }
        // 900+ IS SHARED, and `hasConfigTarget` is what separates the two
        // owners: the assembly catalogue's rows are only live while that panel
        // is open, and the part menu is only collected while it is not.
        if (id >= 900 && hasConfigTarget) { return {HudAction::VabSelect, id - 900u}; }
        // ...and the part menu's range is BOUNDED, so the ids between it and
        // the multiplayer panel keep falling through exactly as they did.
        if (id >= 900 && id < 900u + kHudPartAnimationSlots)
        {
            return {HudAction::PartAnimation, id - 900u};
        }
        // ...and the row after the last animation slot is the docked port's
        // release. It sits INSIDE the part menu's block rather than in a range
        // of its own because it is one more row on that same panel, and it is
        // bounded above for the same reason the animations are.
        if (id == 900u + kHudPartAnimationSlots)
        {
            return {HudAction::PartUndock, id};
        }
        if (id == 901u + kHudPartAnimationSlots)
        {
            return {HudAction::PartJettison, id};
        }
        if (id == 898 && hasConfigTarget) { return {HudAction::VabProduce, id}; }
        if (id == 899 && hasConfigTarget) { return {HudAction::VabCancel, id}; }
        if (id >= 610 && hasConfigTarget) { return {HudAction::RecipeChoice, id - 610u}; }
        if (id == 600 && hasConfigTarget) { return {HudAction::RecipeStop, id}; }
        if (id == 601 && hasConfigTarget) { return {HudAction::PowerPriority, id}; }
        if (id >= 400) { return {HudAction::BuildArm, id - 400u}; }
        if (id == 300) { return {HudAction::MapCycleVessel, id}; }
        if (id == 301) { return {HudAction::MapWarpToNode, id}; }
        // The map owns only its own buttons; everything below belongs to the
        // hangar or the cockpit and must not fire from the map.
        if (mapView) { return {HudAction::None, id}; }
        if (id >= 200) { return {HudAction::HangarAction, id}; }
        if (id >= 100) { return {HudAction::PalettePart, id - 100u}; }
        return {HudAction::SasMode, id};
    }
} // namespace sw::ui
