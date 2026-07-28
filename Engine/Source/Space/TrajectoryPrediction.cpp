#include "Space/TrajectoryPrediction.hpp"

#include <algorithm>
#include <cmath>

namespace sw::space
{
    namespace
    {
        /// What (if anything) terminates the conic at a given time.
        struct EventProbe
        {
            SegmentEnd type = SegmentEnd::Horizon;
            i32 eventBody = -1;
        };

        /// Evaluates the termination predicates on `orbit` at time t.
        /// Returns Horizon when the trajectory is still freely coasting.
        [[nodiscard]] EventProbe probeEvents(const CelestialIndex& index,
                                             i32 primaryIndex,
                                             const phys::KeplerOrbit& orbit, f64 t)
        {
            const CelestialIndex::Body& primary =
                index.body(static_cast<usize>(primaryIndex));

            WorldVec3 relative{};
            phys::kepler::evaluate(orbit, t, relative);
            const f64 radius = glm::length(relative);

            if (radius < primary.bodyRadius)
            {
                return {SegmentEnd::Impact, primaryIndex};
            }
            if (primary.parentIndex >= 0 && radius > primary.soiRadius)
            {
                return {SegmentEnd::SoiExit, primary.parentIndex};
            }
            for (const i32 childIndex : index.childrenOf(static_cast<usize>(primaryIndex)))
            {
                const CelestialIndex::Body& child =
                    index.body(static_cast<usize>(childIndex));
                if (child.hasOrbit == 0)
                {
                    continue;
                }
                WorldVec3 childRelative{};
                phys::kepler::evaluate(child.orbit, t, childRelative);
                const WorldVec3 delta = relative - childRelative;
                if (glm::dot(delta, delta) < child.soiRadius * child.soiRadius)
                {
                    return {SegmentEnd::Encounter, childIndex};
                }
            }
            return {SegmentEnd::Horizon, -1};
        }

        /// Bisects the first event between tQuiet (no event) and tActive
        /// (event) down to millisecond precision.
        [[nodiscard]] f64 refineEventTime(const CelestialIndex& index, i32 primaryIndex,
                                          const phys::KeplerOrbit& orbit, f64 tQuiet,
                                          f64 tActive)
        {
            for (int iteration = 0; iteration < 64 && (tActive - tQuiet) > 1.0e-3;
                 ++iteration)
            {
                const f64 mid = 0.5 * (tQuiet + tActive);
                if (probeEvents(index, primaryIndex, orbit, mid).type ==
                    SegmentEnd::Horizon)
                {
                    tQuiet = mid;
                }
                else
                {
                    tActive = mid;
                }
            }
            return tActive;
        }
    } // namespace

    void predictTrajectory(const CelestialIndex& index, const WorldVec3& worldPosition,
                           const WorldVec3& worldVelocity, f64 startTime,
                           const PredictionSettings& settings,
                           std::vector<TrajectorySegment>& outSegments)
    {
        outSegments.clear();
        if (index.size() == 0)
        {
            return;
        }

        const f64 horizon = startTime + settings.horizonSeconds;

        // Initial frame: the SOI primary of the starting point.
        i32 primaryIndex = index.soiPrimaryAt(worldPosition, startTime);
        if (primaryIndex < 0)
        {
            return;
        }
        WorldVec3 primaryPosition{};
        WorldVec3 primaryVelocity{};
        index.stateAt(primaryIndex, startTime, primaryPosition, &primaryVelocity);
        WorldVec3 relative = worldPosition - primaryPosition;
        WorldVec3 relativeVelocity = worldVelocity - primaryVelocity;
        f64 segmentStart = startTime;

        for (u32 segmentCount = 0;
             segmentCount < settings.maxSegments && segmentStart < horizon;
             ++segmentCount)
        {
            const CelestialIndex::Body& primary =
                index.body(static_cast<usize>(primaryIndex));

            TrajectorySegment segment{};
            segment.primaryIndex = primaryIndex;
            segment.startTime = segmentStart;

            if (!phys::kepler::fromStateVectors(primary.mu, relative, relativeVelocity,
                                                segmentStart, segment.orbit,
                                                /*allowHyperbolic=*/true))
            {
                segment.endTime = segmentStart;
                segment.endReason = SegmentEnd::Lost;
                outSegments.push_back(segment);
                return;
            }

            // ---- how far, and how finely, to scan ------------------------
            //
            // HOW FAR is the whole point of this function. A closed orbit
            // says everything it has to say in one revolution: if nothing
            // happens in that turn, nothing ever will, and the line joins
            // up. An open one runs until it leaves — and only a hyperbola
            // around a body with no sphere of influence to leave can run
            // past the caller's horizon.
            //
            // HOW FINELY follows from the same idea. The step is a fraction
            // of the orbit's OWN period, never of the horizon, so a lunar
            // parking orbit and a Mars transfer are scanned at the same
            // angular resolution. Sampling a two-year transfer at the step
            // that suited six days is exactly what used to make the line
            // stop in the middle of nowhere.
            const f64 samplesPerRevolution =
                static_cast<f64>(std::max(settings.samplesPerRevolution, 16u));
            bool closes = false;
            f64 windowEnd = horizon;
            f64 step = 0.0;
            if (!segment.orbit.isHyperbolic())
            {
                const f64 period = phys::kepler::period(segment.orbit);
                if (period > 0.0)
                {
                    step = period / samplesPerRevolution;
                    if (segmentStart + period <= horizon)
                    {
                        windowEnd = segmentStart + period;
                        closes = true;
                    }
                }
            }
            else
            {
                // A hyperbola has no period; its natural clock is the same
                // sqrt(|a|^3/mu) the mean motion is built from.
                const f64 timeScale =
                    (segment.orbit.meanMotion > 0.0) ? (1.0 / segment.orbit.meanMotion)
                                                     : 0.0;
                constexpr f64 kTwoPi = 6.283185307179586;
                step = timeScale * (kTwoPi / samplesPerRevolution);
            }
            if (!(step > 0.0))
            {
                step = (windowEnd - segmentStart) / samplesPerRevolution;
            }
            u32 sampleCount = static_cast<u32>(
                std::min<f64>(std::ceil((windowEnd - segmentStart) / step),
                              static_cast<f64>(std::max(settings.maxSamplesPerSegment, 16u))));
            sampleCount = std::max(sampleCount, 16u);
            step = (windowEnd - segmentStart) / static_cast<f64>(sampleCount);

            segment.endTime = windowEnd;
            segment.endReason = closes ? SegmentEnd::Closed : SegmentEnd::Horizon;

            f64 previousTime = segmentStart;
            for (u32 sample = 1; sample <= sampleCount; ++sample)
            {
                const f64 sampleTime = segmentStart + step * static_cast<f64>(sample);
                const EventProbe probe =
                    probeEvents(index, primaryIndex, segment.orbit, sampleTime);
                if (probe.type != SegmentEnd::Horizon)
                {
                    const f64 eventTime = refineEventTime(
                        index, primaryIndex, segment.orbit, previousTime, sampleTime);
                    // Re-probe AT the refined time (the first-triggering
                    // event may differ from the one seen a full step later).
                    const EventProbe refined =
                        probeEvents(index, primaryIndex, segment.orbit, eventTime);
                    segment.endTime = eventTime;
                    segment.endReason = (refined.type != SegmentEnd::Horizon)
                                            ? refined.type
                                            : probe.type;
                    segment.eventBodyIndex = (refined.type != SegmentEnd::Horizon)
                                                 ? refined.eventBody
                                                 : probe.eventBody;
                    break;
                }
                previousTime = sampleTime;
            }

            outSegments.push_back(segment);

            if (segment.endReason == SegmentEnd::Horizon ||
                segment.endReason == SegmentEnd::Closed ||
                segment.endReason == SegmentEnd::Impact)
            {
                return;
            }

            // ---- frame hand-off into the next patch ----------------------
            const f64 eventTime = segment.endTime;
            WorldVec3 position{};
            WorldVec3 velocity{};
            phys::kepler::evaluate(segment.orbit, eventTime, position, &velocity);

            if (segment.endReason == SegmentEnd::Encounter)
            {
                const CelestialIndex::Body& child =
                    index.body(static_cast<usize>(segment.eventBodyIndex));
                WorldVec3 childPosition{};
                WorldVec3 childVelocity{};
                phys::kepler::evaluate(child.orbit, eventTime, childPosition,
                                       &childVelocity);
                relative = position - childPosition;
                relativeVelocity = velocity - childVelocity;
                primaryIndex = segment.eventBodyIndex;
            }
            else // SoiExit: climb to the primary's own parent
            {
                if (primary.hasOrbit == 0 || primary.parentIndex < 0)
                {
                    return; // a static root has no exit (defensive)
                }
                WorldVec3 primaryRelative{};
                WorldVec3 primaryRelativeVelocity{};
                phys::kepler::evaluate(primary.orbit, eventTime, primaryRelative,
                                       &primaryRelativeVelocity);
                relative = position + primaryRelative;
                relativeVelocity = velocity + primaryRelativeVelocity;
                primaryIndex = primary.parentIndex;
            }
            segmentStart = eventTime;
        }
    }

    ClosestApproach closestApproachToBody(const CelestialIndex& index,
                                          const std::vector<TrajectorySegment>& segments,
                                          i32 targetIndex, u32 samplesPerSegment)
    {
        ClosestApproach result{};
        if (targetIndex < 0 || static_cast<usize>(targetIndex) >= index.size())
        {
            return result;
        }
        const u32 samples = std::max(samplesPerSegment, 16u);

        // Separation at an absolute time, on a given segment's conic.
        auto separationAt = [&](const TrajectorySegment& segment, f64 time) {
            WorldVec3 relative{};
            phys::kepler::evaluate(segment.orbit, time, relative);
            const WorldVec3 ours = index.positionAt(segment.primaryIndex, time) + relative;
            return glm::length(index.positionAt(targetIndex, time) - ours);
        };

        const TrajectorySegment* bestSegment = nullptr;
        f64 bestTime = 0.0;
        f64 bestDistance = 0.0;
        f64 bracketLow = 0.0;
        f64 bracketHigh = 0.0;

        for (const TrajectorySegment& segment : segments)
        {
            if (segment.primaryIndex < 0 || segment.endReason == SegmentEnd::Lost ||
                segment.endTime <= segment.startTime)
            {
                continue;
            }
            f64 previousTime = segment.startTime;
            for (u32 sample = 0; sample <= samples; ++sample)
            {
                const f64 time = phys::kepler::timeAtArcFraction(
                    segment.orbit, segment.startTime, segment.endTime,
                    static_cast<f64>(sample) / static_cast<f64>(samples));
                const f64 distance = separationAt(segment, time);
                if (bestSegment == nullptr || distance < bestDistance)
                {
                    bestSegment = &segment;
                    bestTime = time;
                    bestDistance = distance;
                    // The bracket is the neighbouring samples: the minimum
                    // cannot be outside them, and inside them the function
                    // is smooth enough for a golden section.
                    bracketLow = previousTime;
                    bracketHigh = std::min(
                        segment.endTime,
                        phys::kepler::timeAtArcFraction(
                            segment.orbit, segment.startTime, segment.endTime,
                            static_cast<f64>(std::min(sample + 1, samples)) /
                                static_cast<f64>(samples)));
                }
                previousTime = time;
            }
        }
        if (bestSegment == nullptr)
        {
            return result;
        }

        // ---- refine: golden section on the bracket ------------------------
        // A coarse scan lands within one sample of the minimum; the number
        // the player reads is the minimum itself, so it is worth the sixty
        // evaluations it takes to find it.
        {
            constexpr f64 kInvPhi = 0.6180339887498949;
            f64 low = std::max(bracketLow, bestSegment->startTime);
            f64 high = std::min(bracketHigh, bestSegment->endTime);
            if (high > low)
            {
                f64 c = high - (high - low) * kInvPhi;
                f64 d = low + (high - low) * kInvPhi;
                f64 fc = separationAt(*bestSegment, c);
                f64 fd = separationAt(*bestSegment, d);
                for (int iteration = 0; iteration < 60 && (high - low) > 1.0e-3;
                     ++iteration)
                {
                    if (fc < fd)
                    {
                        high = d;
                        d = c;
                        fd = fc;
                        c = high - (high - low) * kInvPhi;
                        fc = separationAt(*bestSegment, c);
                    }
                    else
                    {
                        low = c;
                        c = d;
                        fc = fd;
                        d = low + (high - low) * kInvPhi;
                        fd = separationAt(*bestSegment, d);
                    }
                }
                const f64 refined = 0.5 * (low + high);
                const f64 refinedDistance = separationAt(*bestSegment, refined);
                if (refinedDistance < bestDistance)
                {
                    bestTime = refined;
                    bestDistance = refinedDistance;
                }
            }
        }

        // ---- and what it looks like at that moment ------------------------
        WorldVec3 relative{};
        WorldVec3 relativeVelocity{};
        phys::kepler::evaluate(bestSegment->orbit, bestTime, relative,
                               &relativeVelocity);
        WorldVec3 primaryPosition{};
        WorldVec3 primaryVelocity{};
        index.stateAt(bestSegment->primaryIndex, bestTime, primaryPosition,
                      &primaryVelocity);
        WorldVec3 targetPosition{};
        WorldVec3 targetVelocity{};
        index.stateAt(targetIndex, bestTime, targetPosition, &targetVelocity);

        result.valid = true;
        result.timeSeconds = bestTime;
        result.distanceM = bestDistance;
        result.relativeSpeedMps =
            glm::length((primaryVelocity + relativeVelocity) - targetVelocity);
        result.primaryIndex = bestSegment->primaryIndex;
        result.relativePosition = relative;

        // The target, in ITS OWN orbit's frame, so the marker lands on the
        // ring the map has drawn rather than out in the world.
        const CelestialIndex::Body& target = index.body(static_cast<usize>(targetIndex));
        if (target.hasOrbit != 0 && target.parentIndex >= 0)
        {
            result.targetPrimaryIndex = target.parentIndex;
            result.targetRelativePosition =
                targetPosition - index.positionAt(target.parentIndex, bestTime);
        }
        else
        {
            result.targetPrimaryIndex = -1; // a static root: already absolute
            result.targetRelativePosition = targetPosition;
        }
        return result;
    }

    WorldVec3 remainingBurn(const CelestialIndex& index,
                            const std::vector<TrajectorySegment>& coast,
                            const WorldVec3& plannedDv, const WorldVec3& currentVelocity,
                            f64 nowSeconds)
    {
        WorldVec3 coastPosition{};
        WorldVec3 coastVelocity{};
        if (!stateOnPrediction(index, coast, nowSeconds, coastPosition, coastVelocity))
        {
            return plannedDv; // no frozen plan: nothing has been applied yet
        }
        return plannedDv - (currentVelocity - coastVelocity);
    }

    bool timeNearestScreenPoint(const CelestialIndex& index,
                                const std::vector<TrajectorySegment>& segments,
                                const Mat4& viewProjectionCameraRelative,
                                const WorldVec3& cameraPosition, f64 renderTime,
                                const Vec2& targetNdc, u32 samplesPerSegment,
                                f64& outTime, f32& outDistanceNdc)
    {
        const u32 samples = std::max(samplesPerSegment, 8u);
        bool found = false;
        f32 best = 0.0f;
        for (const TrajectorySegment& segment : segments)
        {
            if (segment.primaryIndex < 0 || segment.endReason == SegmentEnd::Lost ||
                segment.endTime <= segment.startTime)
            {
                continue;
            }
            const WorldVec3 primaryPosition =
                index.positionAt(segment.primaryIndex, renderTime);
            for (u32 sample = 0; sample <= samples; ++sample)
            {
                const f64 time = phys::kepler::timeAtArcFraction(
                    segment.orbit, segment.startTime, segment.endTime,
                    static_cast<f64>(sample) / static_cast<f64>(samples));
                WorldVec3 relative{};
                phys::kepler::evaluate(segment.orbit, time, relative);
                const Vec3 cameraRelative =
                    Vec3((primaryPosition + relative) - cameraPosition);
                const Vec4 clip =
                    viewProjectionCameraRelative * Vec4(cameraRelative, 1.0f);
                if (clip.w <= 0.0f)
                {
                    continue; // behind the camera: no honest screen position
                }
                const Vec2 ndc{clip.x / clip.w, clip.y / clip.w};
                const f32 distance = glm::length(ndc - targetNdc);
                if (!found || distance < best)
                {
                    found = true;
                    best = distance;
                    outTime = time;
                }
            }
        }
        if (found)
        {
            outDistanceNdc = best;
        }
        return found;
    }

    bool stateOnPrediction(const CelestialIndex& index,
                           const std::vector<TrajectorySegment>& segments,
                           f64 timeSeconds, WorldVec3& outPosition,
                           WorldVec3& outVelocity)
    {
        const TrajectorySegment* chosen = nullptr;
        for (const TrajectorySegment& segment : segments)
        {
            if (segment.endReason == SegmentEnd::Lost || segment.primaryIndex < 0)
            {
                continue;
            }
            chosen = &segment; // last segment whose start precedes t wins
            if (timeSeconds < segment.endTime)
            {
                break;
            }
        }
        if (chosen == nullptr)
        {
            return false;
        }

        WorldVec3 relative{};
        WorldVec3 relativeVelocity{};
        phys::kepler::evaluate(chosen->orbit, timeSeconds, relative, &relativeVelocity);
        WorldVec3 primaryPosition{};
        WorldVec3 primaryVelocity{};
        index.stateAt(chosen->primaryIndex, timeSeconds, primaryPosition,
                      &primaryVelocity);
        outPosition = primaryPosition + relative;
        outVelocity = primaryVelocity + relativeVelocity;
        return true;
    }
} // namespace sw::space
