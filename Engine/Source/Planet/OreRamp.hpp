#pragma once

// ============================================================================
// Planet/OreRamp.hpp
// HOW MUCH ORE, AS A COLOUR — one hue per resource, lightness for magnitude.
//
// It lives in the engine rather than beside the screen that draws it because
// three places now have to agree: the geology globe, its legend, and the map
// marks over a surveyed body. Three copies of a colour ramp is three chances
// for the legend to describe a picture that is drawn from different numbers.
//
// THE SHAPE IS NOT A CHOICE. Sequential data — a magnitude, here a density in
// [0,1] — takes ONE hue whose lightness moves with the value. A rainbow ramp
// reads as five categories and nobody can say whether green is more than
// yellow without going back to the legend every time. On a black screen the
// anchor flips from the paper convention: near-zero recedes into the surface
// and the rich end is the bright one, so the eye lands on the deposit rather
// than on the emptiness around it.
//
// THE HUES WERE MEASURED, NOT PICKED. Rust, malachite and ice-blue clear the
// normal-vision separation floor in all pairs (worst 22.8 in OKLab x100) and
// all three sit above 3:1 contrast on the screen's own black. Iron and copper
// are close under deuteranopia (6.2, inside the floor band), which is legal
// here and only here: one channel is displayed at a time and the live one is
// named in a lit button, so the colour never carries the identity alone.
//
// Copper is GREEN and not a second orange for the same reason — malachite is
// what copper ore actually looks like, and it puts a whole hue between the
// two metals a player is most likely to confuse.
// ============================================================================

#include "Core/Types.hpp"
#include "Math/Math.hpp"
#include "Resources/ResourceTypes.hpp"

namespace sw::planet
{
    /// The colour a density in [0,1] reads as on `resource`'s own ramp.
    /// Monotone in lightness by construction: three stops, each brighter than
    /// the last.
    [[nodiscard]] inline Vec3 oreRampColor(res::Resource resource, f32 density)
    {
        struct Ramp
        {
            Vec3 foot; // 0.0 — barely there
            Vec3 mid;  // 0.5
            Vec3 head; // 1.0 — a core worth flying to
        };
        constexpr Ramp kIron{{0.14f, 0.07f, 0.05f},
                             {0.83f, 0.33f, 0.23f},
                             {1.00f, 0.80f, 0.68f}};
        constexpr Ramp kCopper{{0.05f, 0.12f, 0.08f},
                               {0.18f, 0.64f, 0.41f},
                               {0.76f, 0.98f, 0.84f}};
        constexpr Ramp kIce{{0.05f, 0.09f, 0.16f},
                            {0.30f, 0.56f, 0.94f},
                            {0.80f, 0.90f, 1.00f}};
        const Ramp& ramp = (resource == res::Resource::IronOre)     ? kIron
                           : (resource == res::Resource::CopperOre) ? kCopper
                                                                    : kIce;
        const f32 t = glm::clamp(density, 0.0f, 1.0f);
        return (t < 0.5f) ? glm::mix(ramp.foot, ramp.mid, t * 2.0f)
                          : glm::mix(ramp.mid, ramp.head, (t - 0.5f) * 2.0f);
    }

    /// Relative luminance, the quantity the ramp is monotone in. Rec. 709
    /// weights: the eye reads green as most of a colour's brightness, and a
    /// ramp that rises in red while falling in green is not a ramp.
    [[nodiscard]] inline f32 rampLuminance(const Vec3& color)
    {
        return 0.2126f * color.r + 0.7152f * color.g + 0.0722f * color.b;
    }
} // namespace sw::planet
