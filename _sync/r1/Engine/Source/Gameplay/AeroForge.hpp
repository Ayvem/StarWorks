#pragma once

// ============================================================================
// Gameplay/AeroForge.hpp
// THE WIND TUNNEL — offline. Turns a part's geometry into its `.aero.json`.
//
// This is the expensive half of the aerodynamics architecture, and it never
// runs while anybody is playing. For each of a few hundred wind directions
// it integrates the pressure and the shear over the part's REAL triangulated
// surface and records the resulting force and moment. Those numbers, divided
// by the dynamic pressure, are the table; the game only ever reads it.
//
// THE SOLVER is a surface-impact method — the standard engineering tool for
// exactly this job, and the reason it fits here is that it is a RASTERISER.
// Looking along the wind, the surface elements that receive air are precisely
// the ones a depth buffer keeps, so the self-shadowing that makes a part's
// far side contribute nothing comes out of the algorithm rather than being
// bolted onto it. Three terms are integrated per visible element:
//
//   IMPACT PRESSURE   Cp = Cp_max * cos^2(theta), theta measured from the
//       surface normal to the flow: the air's momentum normal to the surface
//       is given up to it. This is what makes a cone cheap and a flat plate
//       expensive, out of the geometry alone, with no coefficient typed by
//       anybody.
//   BASE PRESSURE     the same law with a NEGATIVE coefficient on the
//       rear-facing elements the depth buffer sees LAST: the wake does not
//       push, it pulls. A blunt tail collects all of it, a boat-tail almost
//       none — again from the shape.
//   SKIN FRICTION     a flat-plate shear along the local tangent over the
//       wetted area. Small on a capsule; not small on a booster.
//
// Validation is in the tests, against textbook numbers: a flat plate lands
// at Cd 1.2 (measured 1.17), a sphere at 0.6 (0.47 subcritical, 0.92
// hypersonic), a 15-degree cone at 0.27 (0.25 measured with base drag).
// That is the accuracy this buys — right shape, right order, right trends,
// for zero cost at runtime.
// ============================================================================

#include "Gameplay/Parts.hpp"
#include "Physics/Aerodynamics.hpp"

namespace sw::aero
{
    struct ForgeSettings
    {
        /// Wind-direction grid. theta from the +Z axis (nose-on is 0),
        /// phi around it. 19 x 18 is 10-degree pitch steps and 20-degree
        /// roll steps — fine enough that a fin's edge-on notch survives
        /// interpolation, coarse enough to keep a table around 20 kB.
        u32 thetaCount = 19;
        u32 phiCount = 18;
        /// Depth-buffer resolution per side. The pixel is the integration
        /// element, so this is the solver's accuracy knob: 192 puts about
        /// 30 000 elements on a part that fills the frame.
        u32 resolution = 192;
        /// Stagnation pressure coefficient. 1.0 is incompressible; the
        /// hypersonic limit of the modified-Newtonian form is 1.84, which
        /// the runtime's Mach factor stands in for.
        f64 stagnationCp = 1.0;
        /// Base (wake) pressure coefficient. Negative: suction.
        f64 basePressureCp = -0.20;
        /// Flat-plate skin-friction coefficient.
        f64 skinFrictionCf = 0.004;

        // ---- the lifting term -------------------------------------------
        //
        // IMPACT THEORY ALONE MAKES FINS USELESS, and it took a probe to
        // find out: solved with pressure only, a fin at ten degrees produced
        // a thirtieth of the force it should, and a rocket with a full set
        // of them still flipped. The reason is not a bug, it is the theory's
        // own boundary. Newtonian pressure counts only the momentum the air
        // gives up normal to the surface — which is right for a blunt nose
        // behind a detached shock and badly wrong for a thin surface at a
        // shallow angle, where nearly all the force comes from CIRCULATION:
        // the flow bending round the leading edge and pulling on the far
        // side. Impact theory has no far side.
        //
        // So the solver carries the linear (potential-flow) term as well,
        // and applies it where linear theory is the valid one: on surfaces
        // lying nearly ALONG the flow, front and back alike. Cp = k sin(d)
        // with d the local inclination — compression on the windward face,
        // suction on the leeward one, both pushing the surface the same way.
        // A flat plate collects it twice and lifts like a wing; a sphere's
        // stagnation point never sees it at all.
        /// IS THIS A WING OR IS IT A BODY? The linear term belongs only to
        /// the first, and applying it to the second is not a small error: a
        /// nose cone's flank is inclined fifteen degrees to the flow, and
        /// charging it a lifting surface's pressure would treble the drag of
        /// every rocket in the game. So the forge decides ONCE per part, from
        /// its proportions, and `solveDirection` is simply told the answer —
        /// which is also how a designer thinks about it, and what lets a
        /// test ask each question on its own.
        bool liftingSurface = false;
        /// A part is a lifting surface when its thinnest dimension is under
        /// this fraction of its longest. A fin is a twelfth; a fuel tank is
        /// well over half.
        f64 plateRatio = 0.20;
        /// Half the lift-curve slope of a moderate-aspect-ratio surface:
        /// 2 pi AR / (AR + 2) is 3.2 per radian at AR 2, and a plate picks
        /// the term up on both faces. Per-surface aspect ratio is the next
        /// refinement; this is the honest average for a fin.
        f64 liftSlope = 1.6;
        /// Where attached flow ends. Below this |sin(d)| the linear term is
        /// at full strength, above `separatedSine` it is gone and impact
        /// theory has the surface to itself. The transition IS the stall:
        /// a plate's force peaks near twelve degrees and falls away after,
        /// which is what a plate does.
        f64 attachedSine = 0.20;   // ~11.5 degrees
        f64 separatedSine = 0.45;  // ~26.7 degrees
    };

    /// Every triangle the air can touch, in part-local metres.
    /// Visible shapes if the part has any (the skin is what you SEE),
    /// collider shapes otherwise.
    struct SkinTriangle
    {
        Vec3 a{0.0f};
        Vec3 b{0.0f};
        Vec3 c{0.0f};
        Vec3 normal{0.0f, 0.0f, 1.0f}; // outward
    };

    [[nodiscard]] std::vector<SkinTriangle> partSkin(const parts::PartDefinition& definition);

    /// One wind direction solved. `flowDirection` is the way the air
    /// TRAVELS, part frame, unit length.
    struct SolvedDirection
    {
        Vec3 forceM2{0.0f};   // force / q
        Vec3 momentM3{0.0f};  // moment / q about the part origin
        f64 projectedAreaM2 = 0.0;
    };

    [[nodiscard]] SolvedDirection solveDirection(std::span<const SkinTriangle> skin,
                                                 const Vec3& flowDirection,
                                                 const ForgeSettings& settings);

    /// The whole table. Deterministic: same geometry in, same numbers out,
    /// on every machine — a generated asset that differs between two
    /// developers is an asset nobody can review.
    [[nodiscard]] AeroTable forgePart(const parts::PartDefinition& definition,
                                      const ForgeSettings& settings = {});

    /// `<part>.aero.json` next to `<part>.swpart`.
    [[nodiscard]] std::filesystem::path aeroPathFor(const std::filesystem::path& partPath);
} // namespace sw::aero
