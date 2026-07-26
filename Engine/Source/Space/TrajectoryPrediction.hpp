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
        f64 horizonSeconds = 6.0 * 86400.0; // 6 days: covers a lunar transfer
        u32 maxSegments = 5;
        /// Event-scan resolution per segment. 16384 over 6 days is a 32 s
        /// step — far finer than any SOI crossing; bisection then refines
        /// the event to ~1 ms.
        u32 samplesPerSegment = 16384;
    };

    /// Predicts from a WORLD-frame state. Returns at least one segment
    /// (unless the index is empty).
    void predictTrajectory(const CelestialIndex& index, const WorldVec3& worldPosition,
                           const WorldVec3& worldVelocity, f64 startTime,
                           const PredictionSettings& settings,
                           std::vector<TrajectorySegment>& outSegments);

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
