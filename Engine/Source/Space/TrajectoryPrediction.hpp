#pragma once

// ============================================================================
// Space/TrajectoryPrediction.hpp
// PATCHED-CONICS trajectory prediction — the KSP-style flight plan.
//
// Starting from a world-frame state (position, velocity, time), the
// predictor:
//   1. finds the current SOI primary and fits a conic (elliptic OR
//      hyperbolic) around it;
//   2. scans forward in time for the FIRST event on that conic:
//        - IMPACT     r < primary body radius,
//        - SOI EXIT   r > primary SOI radius,
//        - ENCOUNTER  distance to a CHILD body (evaluated on ITS orbit at
//                     the same future time) drops below the child's SOI;
//      the event time is refined by bisection to sub-second precision;
//   3. transforms the state into the new primary's frame and repeats,
//      up to maxSegments patches or the time horizon.
//
// Every segment is primary-RELATIVE: the caller renders it around the
// primary's CURRENT world position (KSP map style), so a Luna flyby arc is
// drawn around where Luna is NOW — which is exactly what a pilot needs.
//
// Pure function of the CelestialIndex — no ECS access, trivially testable.
// ============================================================================

#include "Space/CelestialIndex.hpp"

#include <vector>

namespace sw::space
{
    enum class SegmentEnd : u32
    {
        Horizon = 0,  // ran to the prediction horizon without an event
        SoiExit,      // leaves the primary's sphere of influence
        Encounter,    // enters a child body's sphere of influence
        Impact,       // intersects the primary's surface
        Lost,         // state not representable as a conic (near-parabolic)
        /// CAME BACK TO WHERE IT STARTED. A closed orbit that meets nothing
        /// in one revolution has said everything it is going to say: the
        /// line joins up, and a second revolution would be drawn exactly on
        /// top of the first. This is the difference between "the plan ends
        /// here" and "the plan ran out of time", and the map draws the two
        /// very differently.
        Closed,
        /// RAN OUT OF ROOM RATHER THAN OUT OF TIME. A hyperbolic escape from
        /// the outermost body has no sphere of influence to leave and no
        /// revolution to close, so before this existed it was drawn to the
        /// twenty-year horizon — a line reaching a hundred billion kilometres
        /// with nothing on it, sampled thousands of times and then split into
        /// thousands of screen-space boxes because every chord of it spans
        /// four orders of magnitude of camera distance. Placing a node on an
        /// escape trajectory visibly hitched the frame. The plan now stops at
        /// a range the pilot can actually reason about.
        RangeLimit,
    };

    struct TrajectorySegment
    {
        i32 primaryIndex = -1;     // CelestialIndex body the orbit is around
        phys::KeplerOrbit orbit{}; // primary-relative conic
        f64 startTime = 0.0;
        f64 endTime = 0.0;
        SegmentEnd endReason = SegmentEnd::Horizon;
        /// Encounter: the child entered. SoiExit: the parent returned to.
        /// Impact: the primary hit. -1 otherwise.
        i32 eventBodyIndex = -1;
    };

    struct PredictionSettings
    {
        /// A HARD CAP, not the plan's normal length.
        ///
        /// The scan used to stop after a fixed six days, which is why a
        /// heliocentric transfer was a stub of a line hanging in space:
        /// six days of a two-year orbit is one and a half degrees of arc.
        /// A segment now runs until something HAPPENS to it — an encounter,
        /// an escape, an impact, or a full revolution that meets none of
        /// them — and this number only stops a trajectory that would
        /// otherwise coast forever (a hyperbolic escape from the outermost
        /// body, which has no sphere of influence to leave).
        ///
        /// It is also how the maneuver planner asks for a plan that stops
        /// AT the node: set it to the time remaining.
        f64 horizonSeconds = 20.0 * 365.25 * 86400.0; // 20 years
        u32 maxSegments = 5;
        /// Event-scan resolution, in samples PER REVOLUTION of the orbit
        /// being scanned — not per horizon. That is the property worth
        /// having: a 90-minute parking orbit and a two-year transfer are
        /// scanned at the same angular resolution, so a small sphere of
        /// influence cannot slip between two samples of a big orbit.
        /// Bisection then refines the event itself to ~1 ms.
        u32 samplesPerRevolution = 4096;
        /// The most probes one segment may cost before the step is
        /// stretched to fit. Only an unbounded escape reaches this; it
        /// exists so a prediction can never become an unbounded loop.
        u32 maxSamplesPerSegment = 200000;
        /// How far from the primary a plan is worth drawing. Ten billion
        /// kilometres is past every planet in the solar system and two orders
        /// of magnitude short of the nearest star, which is the right place to
        /// stop: beyond it a heliocentric conic says nothing a pilot can use,
        /// and an interstellar crossing is steered by heading rather than by
        /// looking at an arc. Zero disables the cap.
        f64 maxRangeMeters = 1.0e13;
    };

    // ------------------------------------------------------------------------
    // PLANNING A BURN: how much is one tap?
    //
    // A maneuver node spans five orders of magnitude. Trimming a rendezvous
    // is a tenth of a metre per second; leaving Terra for Mars is three and
    // a half kilometres of it. One key with one step size can do exactly one
    // of those jobs, so the step is a LADDER on the modifier keys — and the
    // same ladder moves the node in time, because a burn a hundred times
    // bigger is one you are planning a hundred times further out.
    // ------------------------------------------------------------------------
    struct ManeuverStep
    {
        f64 deltaVMps = 1.0;  // per tap of prograde / normal / radial
        f64 seconds = 10.0;   // per tap of the node's time
    };

    /// The step for the modifiers currently held.
    ///
    /// PRECEDENCE MATTERS AND IS NOT ALPHABETICAL. Control alone is the FINE
    /// step and Control+Shift is the COARSEST one, so the combination has to
    /// be tested before either key on its own — asked the other way round, a
    /// player reaching for a kilometre per second would get a tenth of one.
    ///
    ///   ctrl+shift  x1000   1000 m/s   10000 s
    ///   alt         x100     100 m/s    1000 s
    ///   shift       x10       10 m/s     100 s
    ///   (none)      x1         1 m/s      10 s
    ///   ctrl        x0.1     0.1 m/s       1 s
    [[nodiscard]] constexpr ManeuverStep maneuverStep(bool shift, bool control, bool alt)
    {
        f64 scale = 1.0;
        if (control && shift) { scale = 1000.0; }
        else if (alt)         { scale = 100.0; }
        else if (shift)       { scale = 10.0; }
        else if (control)     { scale = 0.1; }
        return ManeuverStep{scale, 10.0 * scale};
    }

    /// WHAT THE PLAN COST TO FIND, in the only unit that is the same on
    /// every machine. The cost of a prediction is dominated by the event
    /// scan, and the event scan is `samples` positions on our conic with
    /// every child body of the primary evaluated at each — so this number,
    /// not a stopwatch, is what a regression test can hold on to.
    ///
    /// It exists because the difference between a plan that costs 8 ms and
    /// one that costs 125 ms was invisible from the outside: both returned
    /// the same segments, ending at the same place, for the same reason.
    struct PredictionStats
    {
        u32 segments = 0;
        u32 samples = 0;
    };

    /// Predicts from a WORLD-frame state. Returns at least one segment
    /// (unless the index is empty).
    void predictTrajectory(const CelestialIndex& index, const WorldVec3& worldPosition,
                           const WorldVec3& worldVelocity, f64 startTime,
                           const PredictionSettings& settings,
                           std::vector<TrajectorySegment>& outSegments,
                           PredictionStats* outStats = nullptr);

    // ------------------------------------------------------------------------
    // HOW CLOSE DO I GET?
    //
    // The one number that turns a trajectory into a transfer. A plan that
    // passes 400 000 km from Luna and one that passes 4 000 km look
    // identical on a map; the difference is the whole mission.
    //
    // Both sides are analytic — our conic and the target's orbit are exact
    // functions of time — so this is a minimisation, not a simulation: scan
    // the plan for the smallest separation, then refine it by golden
    // section until the answer stops moving.
    // ------------------------------------------------------------------------
    struct ClosestApproach
    {
        bool valid = false;
        f64 timeSeconds = 0.0;
        f64 distanceM = 0.0;
        f64 relativeSpeedMps = 0.0;
        /// WHERE, in the frame the MAP draws in.
        ///
        /// Not world positions: a rendezvous three days out happens 7.8
        /// million kilometres along Terra's own orbit, and a marker at that
        /// world position would sit in empty space far off the side of a
        /// map centred on Terra now. Each position is given RELATIVE to a
        /// primary, exactly as the drawn orbits are, so the caller adds
        /// that primary's current position and the marker lands on the ring
        /// the player is looking at. A primary index of -1 means the
        /// position is already absolute.
        i32 primaryIndex = -1;
        WorldVec3 relativePosition{0.0};
        i32 targetPrimaryIndex = -1;
        WorldVec3 targetRelativePosition{0.0};
    };

    /// Closest approach between a predicted trajectory and a celestial body.
    ///
    /// `samplesPerSegment` sets only the COARSE scan — its job is to find
    /// the right valley, not the bottom of it, which the golden section
    /// then locates exactly. Measured across encounter phases from a 0.5 km
    /// grazing pass to a 130 000 km miss, 256 samples and 4096 return the
    /// same answer to the metre for sixteen times less work. Samples are
    /// spaced by anomaly (as the map draws), which puts them where the
    /// craft moves fastest — where a close pass is easiest to step over.
    [[nodiscard]] ClosestApproach closestApproachToBody(
        const CelestialIndex& index, const std::vector<TrajectorySegment>& segments,
        i32 targetIndex, u32 samplesPerSegment = 256);

    // ------------------------------------------------------------------------
    // ...AND THE TARGET DOES NOT HAVE TO BE A WORLD (F49)
    //
    // A rendezvous is with a CRAFT far more often than with a planet, and the
    // arithmetic above never cared which: all it asks of a target is "where
    // are you at time t", and it asks that a few hundred times across the
    // whole plan. A body answers from its own conic about its parent; a craft
    // answers from a conic taken off its state vectors — which is exactly the
    // orbit the map already draws round it, so the marker and the number
    // cannot disagree.
    //
    // What a craft's answer is NOT is a promise: it is where the target would
    // be if it coasted, and a target under thrust will not be there. That is
    // the same caveat the player's own dotted line carries, and it is why this
    // is a conic rather than a second integration.
    // ------------------------------------------------------------------------
    struct TargetPath
    {
        /// Whose frame `orbit` is in, or -1 when `staticPosition` is already
        /// a world position.
        i32 primaryIndex = -1;
        bool hasOrbit = false;
        phys::KeplerOrbit orbit{};
        WorldVec3 staticPosition{0.0};
        /// Radius of the thing, for the "is that an impact?" test. Zero for a
        /// craft, which you can hit but not crash into.
        f64 bodyRadius = 0.0;
    };

    /// How a catalogued body moves — the path `closestApproachToBody` uses.
    [[nodiscard]] TargetPath bodyTargetPath(const CelestialIndex& index, i32 targetIndex);

    /// How a coasting craft moves, from where it is and how fast it is going.
    /// `primaryIndex` is the body it is bound to (its SOI); a state that has
    /// no conic (radial, parabolic) comes back as a fixed point, which is
    /// honest for the second or two before it does.
    [[nodiscard]] TargetPath craftTargetPath(const CelestialIndex& index, i32 primaryIndex,
                                             const WorldVec3& worldPosition,
                                             const WorldVec3& worldVelocity,
                                             f64 nowSeconds);

    [[nodiscard]] ClosestApproach closestApproachToPath(
        const CelestialIndex& index, const std::vector<TrajectorySegment>& segments,
        const TargetPath& target, u32 samplesPerSegment = 256);

    /// THE BURN STILL TO FLY.
    ///
    /// What is left of a planned dv while the engine is lit — the number a
    /// pilot burns down to zero. The subtlety is what "applied so far"
    /// means, and getting it wrong makes the readout useless in two
    /// different ways:
    ///
    ///   * Recompute the plan from the CURRENT state each frame and the
    ///     target moves with the ship: the readout sits at the full dv
    ///     forever and never tells you to stop. (That was the bug.)
    ///   * Compare against the velocity at ignition and GRAVITY counts as
    ///     thrust: a two-minute burn in low orbit picks up a kilometre per
    ///     second of it, and the readout hits zero with the burn half done.
    ///
    /// So it is measured against the COASTING velocity — where the ship
    /// would be, and how fast, on the frozen pre-burn plan at this same
    /// moment. The difference is exactly the velocity the engine added.
    /// `coast` must be the plan as it was BEFORE the burn started.
    [[nodiscard]] WorldVec3 remainingBurn(const CelestialIndex& index,
                                          const std::vector<TrajectorySegment>& coast,
                                          const WorldVec3& plannedDv,
                                          const WorldVec3& currentVelocity,
                                          f64 nowSeconds);

    /// GRABBING THE PLAN WITH THE MOUSE.
    ///
    /// Walks the drawn line and answers: which MOMENT of this trajectory is
    /// nearest that point on the screen? That single question is what makes
    /// a maneuver node draggable — the player pulls the marker along the
    /// orbit and the node's TIME follows the pixel under the cursor.
    ///
    /// It samples exactly as the map draws (spaced by anomaly, not by time),
    /// so the time it returns belongs to the pixel the player is actually
    /// pointing at, and it skips anything behind the camera — where the
    /// perspective divide would fold the far side of the orbit onto the
    /// near side and hand back a moment on the wrong half of the plan.
    ///
    /// Segment poses are taken around each primary's position at
    /// `renderTime`, which MUST be the same time the map drew them at.
    /// Returns false when nothing on the plan is in front of the camera.
    [[nodiscard]] bool timeNearestScreenPoint(
        const CelestialIndex& index, const std::vector<TrajectorySegment>& segments,
        const Mat4& viewProjectionCameraRelative, const WorldVec3& cameraPosition,
        f64 renderTime, const Vec2& targetNdc, u32 samplesPerSegment, f64& outTime,
        f32& outDistanceNdc);

    /// WORLD-frame state at time t ON an existing prediction: locates the
    /// segment containing t (clamping to the first/last — conics evaluate
    /// fine slightly outside their bounds) and composes its primary's world
    /// state with the relative conic. The foundation of maneuver nodes:
    /// "where will I be, and how fast, at the moment of the burn?"
    /// Returns false when the prediction is empty or degenerate.
    bool stateOnPrediction(const CelestialIndex& index,
                           const std::vector<TrajectorySegment>& segments,
                           f64 timeSeconds, WorldVec3& outPosition,
                           WorldVec3& outVelocity);
} // namespace sw::space
