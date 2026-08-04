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
                                             const phys::KeplerOrbit& orbit, f64 t,
                                             f64 maxRange)
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
            // Checked AFTER the sphere of influence, so a real escape is still
            // reported as an escape and hands off to the parent; the range cap
            // only catches the case where there is no parent to hand off to.
            if (maxRange > 0.0 && radius > maxRange)
            {
                return {SegmentEnd::RangeLimit, -1};
            }
            for (const i32 childIndex : index.childrenOf(static_cast<usize>(primaryIndex)))
            {
                const CelestialIndex::Body& child =
                    index.body(static_cast<usize>(childIndex));
                if (child.hasOrbit == 0)
                {
                    continue;
                }
                // A PLANET YOU ARE NOWHERE NEAR THE ORBIT OF NEEDS NO KEPLER
                // SOLVE. Both radii are measured from the same primary, so a
                // child whose whole annulus — periapsis to apoapsis, widened
                // by its own sphere of influence — excludes our current radius
                // cannot possibly contain us, and the comparison costs three
                // flops against the Newton iteration it replaces.
                //
                // This is most of the scan on the trajectory that matters: an
                // escape sweeps from one au to sixty-seven, and at any single
                // point along it at most one planet's annulus is in play,
                // while the loop used to solve all nine at every one of four
                // thousand samples.
                if (child.orbit.eccentricity < 1.0 && child.orbit.semiMajorAxis > 0.0)
                {
                    const f64 span = child.orbit.semiMajorAxis * child.orbit.eccentricity;
                    if (radius < child.orbit.semiMajorAxis - span - child.soiRadius ||
                        radius > child.orbit.semiMajorAxis + span + child.soiRadius)
                    {
                        continue;
                    }
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
                                          f64 tActive, f64 maxRange)
        {
            for (int iteration = 0; iteration < 64 && (tActive - tQuiet) > 1.0e-3;
                 ++iteration)
            {
                const f64 mid = 0.5 * (tQuiet + tActive);
                if (probeEvents(index, primaryIndex, orbit, mid, maxRange).type ==
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
                           std::vector<TrajectorySegment>& outSegments,
                           PredictionStats* outStats)
    {
        outSegments.clear();
        if (outStats != nullptr)
        {
            *outStats = PredictionStats{};
        }
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
            // AN OPEN ARC IS SAMPLED BY ANOMALY, NOT BY TIME, and that is what
            // turned a hitch into nothing. The step for a hyperbola is a
            // fraction of its own time scale — about an hour for a solar
            // escape — while the arc it has to cover is eight years of it, so
            // uniform-in-time meant seventy thousand samples with eight child
            // bodies probed at each. Measured: 341 ms per call, four times a
            // second, for a plan a player creates by pressing one key.
            //
            // Spacing the samples evenly in HYPERBOLIC ANOMALY instead puts
            // them where the geometry changes: dense at periapsis, spreading
            // out as the arc straightens, so four thousand of them cover the
            // whole escape with a resolution that still cannot step over a
            // planet's sphere of influence. Same coverage, seventeen times
            // fewer probes.
            bool walkAnomaly = false;
            f64 anomalyNow = 0.0;
            f64 anomalyAtBound = 0.0;
            const f64 startRadius = glm::length(relative);
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

                // AND THEN SOLVE FOR WHEN IT LEAVES, rather than sampling
                // until it has. This is the difference between a hitch and no
                // hitch, and the arithmetic is four lines.
                //
                // A hyperbola's window used to be the caller's whole horizon —
                // twenty years — while its STEP is a fraction of its own time
                // scale, which for a solar escape is about an hour. Seventy
                // thousand samples, eight child bodies probed at each, half a
                // million Kepler evaluations, four times a second. Measured at
                // 341 ms per call for an escape from Terra's orbit, and 138 ms
                // even after the range cap was added, because the cap stops the
                // scan at the right PLACE but the scan still has to walk there
                // one step at a time.
                //
                // The radius on a hyperbola is r = a(1 - e cosh H) with a
                // negative, so the anomaly at any given radius is a closed
                // form, and so is the time: Kepler's equation for the
                // hyperbolic case is M = e sinh H - H, and M is linear in t.
                // The window becomes the arc that actually gets drawn.
                const f64 outerBound =
                    (primary.parentIndex >= 0)
                        ? ((settings.maxRangeMeters > 0.0)
                               ? std::min(primary.soiRadius, settings.maxRangeMeters)
                               : primary.soiRadius)
                        : settings.maxRangeMeters;
                const f64 a = segment.orbit.semiMajorAxis; // negative
                const f64 e = segment.orbit.eccentricity;
                if (outerBound > 0.0 && a < 0.0 && e > 1.0 &&
                    segment.orbit.meanMotion > 0.0)
                {
                    const f64 coshBound = (1.0 - outerBound / a) / e;
                    const f64 coshNow = (1.0 - startRadius / a) / e;
                    if (coshBound > 1.0 && coshNow >= 1.0 && coshBound > coshNow)
                    {
                        anomalyAtBound = std::acosh(coshBound);
                        anomalyNow = std::acosh(coshNow);
                        const f64 meanNow =
                            segment.orbit.meanAnomalyAtEpoch +
                            segment.orbit.meanMotion *
                                (segmentStart - segment.orbit.epochSeconds);
                        // WHICH SIDE OF PERIAPSIS, and this is the whole bug.
                        //
                        // acosh has one branch, so the anomaly it returns is
                        // positive whether the craft is coming in or going
                        // out. The sign lives in the mean anomaly, which is
                        // odd in H — so an inbound arc has to carry it, and
                        // its window then runs from a NEGATIVE anomaly,
                        // through periapsis, out to the bound.
                        //
                        // Refusing the inbound case (which is what this used
                        // to do) is not a small conservatism: it drops the
                        // segment back on the uniform-in-time walk, whose
                        // window is then the caller's whole twenty-year
                        // horizon at an hour-and-a-half step. Measured leaving
                        // Terra at +50 km/s — a heliocentric hyperbola that
                        // starts a few degrees before periapsis, which is what
                        // ANY burn made on the day side gives you — that is
                        // sixty-six thousand samples with nine planets probed
                        // at each: 125 ms, four times a second, on the main
                        // thread. It is exactly the freeze that was reported,
                        // and the frame counter never saw it because the
                        // frames either side of it were as fast as ever.
                        if (meanNow < 0.0)
                        {
                            anomalyNow = -anomalyNow;
                        }
                        // ONE STEP PAST THE BOUND, so the bound is INSIDE the
                        // window rather than on its edge.
                        //
                        // The last sample used to land exactly on the sphere
                        // of influence, where the scan's test is `radius >
                        // soiRadius` — a strict inequality against a number
                        // the sample was computed to equal. Whether the exit
                        // was seen came down to the last bit of a cosh, so it
                        // was seen for some burns and not others, and the ones
                        // where it was missed lost every patch after the
                        // first: a Terra escape that should hand off to Sol
                        // drew one arc and stopped there.
                        anomalyAtBound +=
                            (anomalyAtBound - anomalyNow) / samplesPerRevolution;
                        const f64 meanAtBound =
                            e * std::sinh(anomalyAtBound) - anomalyAtBound;
                        if (meanAtBound > meanNow)
                        {
                            const f64 timeToBound =
                                segmentStart +
                                (meanAtBound - meanNow) / segment.orbit.meanMotion;
                            windowEnd = std::min(windowEnd, timeToBound);
                            walkAnomaly = windowEnd < horizon;
                        }
                    }
                }
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
            if (walkAnomaly)
            {
                sampleCount = static_cast<u32>(samplesPerRevolution);
            }
            // The time of sample `i`: linear in time for a closed orbit, linear
            // in hyperbolic anomaly for an open one that has a bound to reach.
            const f64 eccentricity = segment.orbit.eccentricity;
            const f64 meanMotion = segment.orbit.meanMotion;
            const f64 epoch = segment.orbit.epochSeconds;
            const f64 meanAtEpoch = segment.orbit.meanAnomalyAtEpoch;
            const auto timeOfSample = [&](u32 sample) {
                if (!walkAnomaly)
                {
                    return segmentStart + step * static_cast<f64>(sample);
                }
                const f64 fraction =
                    static_cast<f64>(sample) / static_cast<f64>(sampleCount);
                const f64 anomaly =
                    anomalyNow + (anomalyAtBound - anomalyNow) * fraction;
                const f64 mean = eccentricity * std::sinh(anomaly) - anomaly;
                return epoch + (mean - meanAtEpoch) / meanMotion;
            };

            segment.endTime = windowEnd;
            segment.endReason = closes ? SegmentEnd::Closed : SegmentEnd::Horizon;

            f64 previousTime = segmentStart;
            for (u32 sample = 1; sample <= sampleCount; ++sample)
            {
                const f64 sampleTime = timeOfSample(sample);
                const EventProbe probe = probeEvents(index, primaryIndex, segment.orbit,
                                                     sampleTime, settings.maxRangeMeters);
                if (probe.type != SegmentEnd::Horizon)
                {
                    const f64 eventTime =
                        refineEventTime(index, primaryIndex, segment.orbit, previousTime,
                                        sampleTime, settings.maxRangeMeters);
                    // Re-probe AT the refined time (the first-triggering
                    // event may differ from the one seen a full step later).
                    const EventProbe refined =
                        probeEvents(index, primaryIndex, segment.orbit, eventTime,
                                    settings.maxRangeMeters);
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
                if (outStats != nullptr)
                {
                    ++outStats->samples;
                }
            }

            outSegments.push_back(segment);
            if (outStats != nullptr)
            {
                ++outStats->segments;
                // The sample that FOUND the event is counted too: the loop
                // breaks before the tally at the bottom, and a scan that
                // stopped on its first probe still cost one probe.
                if (segment.endReason != SegmentEnd::Horizon &&
                    segment.endReason != SegmentEnd::Closed)
                {
                    ++outStats->samples;
                }
            }

            if (segment.endReason == SegmentEnd::Horizon ||
                segment.endReason == SegmentEnd::Closed ||
                segment.endReason == SegmentEnd::RangeLimit ||
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

    TargetPath bodyTargetPath(const CelestialIndex& index, i32 targetIndex)
    {
        TargetPath path{};
        if (targetIndex < 0 || static_cast<usize>(targetIndex) >= index.size())
        {
            return path;
        }
        const CelestialIndex::Body& body = index.body(static_cast<usize>(targetIndex));
        path.bodyRadius = body.bodyRadius;
        if (body.hasOrbit != 0 && body.parentIndex >= 0)
        {
            path.primaryIndex = body.parentIndex;
            path.hasOrbit = true;
            path.orbit = body.orbit;
        }
        else
        {
            path.staticPosition = body.staticPosition;
        }
        return path;
    }

    TargetPath craftTargetPath(const CelestialIndex& index, i32 primaryIndex,
                               const WorldVec3& worldPosition,
                               const WorldVec3& worldVelocity, f64 nowSeconds)
    {
        TargetPath path{};
        if (primaryIndex < 0 || static_cast<usize>(primaryIndex) >= index.size())
        {
            path.staticPosition = worldPosition;
            return path;
        }
        WorldVec3 primaryPosition{};
        WorldVec3 primaryVelocity{};
        index.stateAt(primaryIndex, nowSeconds, primaryPosition, &primaryVelocity);
        path.primaryIndex = primaryIndex;
        phys::KeplerOrbit orbit{};
        if (phys::kepler::fromStateVectors(index.body(static_cast<usize>(primaryIndex)).mu,
                                           worldPosition - primaryPosition,
                                           worldVelocity - primaryVelocity, nowSeconds,
                                           orbit, true))
        {
            path.hasOrbit = true;
            path.orbit = orbit;
        }
        else
        {
            // No conic in that state — straight up, straight down, or exactly
            // parabolic. Pinning it where it is beats inventing an ellipse.
            path.staticPosition = worldPosition - primaryPosition;
        }
        return path;
    }

    namespace
    {
        /// Where a target is, in the world, at an absolute time.
        [[nodiscard]] WorldVec3 pathPositionAt(const CelestialIndex& index,
                                               const TargetPath& target, f64 time)
        {
            WorldVec3 local = target.staticPosition;
            if (target.hasOrbit)
            {
                phys::kepler::evaluate(target.orbit, time, local);
            }
            return (target.primaryIndex >= 0)
                       ? index.positionAt(target.primaryIndex, time) + local
                       : local;
        }

        void pathStateAt(const CelestialIndex& index, const TargetPath& target, f64 time,
                         WorldVec3& outPosition, WorldVec3& outVelocity)
        {
            WorldVec3 local = target.staticPosition;
            WorldVec3 localVelocity{0.0};
            if (target.hasOrbit)
            {
                phys::kepler::evaluate(target.orbit, time, local, &localVelocity);
            }
            if (target.primaryIndex >= 0)
            {
                WorldVec3 primaryPosition{};
                WorldVec3 primaryVelocity{};
                index.stateAt(target.primaryIndex, time, primaryPosition,
                              &primaryVelocity);
                outPosition = primaryPosition + local;
                outVelocity = primaryVelocity + localVelocity;
                return;
            }
            outPosition = local;
            outVelocity = localVelocity;
        }
    } // namespace

    ClosestApproach closestApproachToBody(const CelestialIndex& index,
                                          const std::vector<TrajectorySegment>& segments,
                                          i32 targetIndex, u32 samplesPerSegment)
    {
        if (targetIndex < 0 || static_cast<usize>(targetIndex) >= index.size())
        {
            return {};
        }
        return closestApproachToPath(index, segments, bodyTargetPath(index, targetIndex),
                                     samplesPerSegment);
    }

    ClosestApproach closestApproachToPath(const CelestialIndex& index,
                                          const std::vector<TrajectorySegment>& segments,
                                          const TargetPath& target, u32 samplesPerSegment)
    {
        ClosestApproach result{};
        const u32 samples = std::max(samplesPerSegment, 16u);

        // Separation at an absolute time, on a given segment's conic.
        auto separationAt = [&](const TrajectorySegment& segment, f64 time) {
            WorldVec3 relative{};
            phys::kepler::evaluate(segment.orbit, time, relative);
            const WorldVec3 ours = index.positionAt(segment.primaryIndex, time) + relative;
            return glm::length(pathPositionAt(index, target, time) - ours);
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
        pathStateAt(index, target, bestTime, targetPosition, targetVelocity);

        result.valid = true;
        result.timeSeconds = bestTime;
        result.distanceM = bestDistance;
        result.relativeSpeedMps =
            glm::length((primaryVelocity + relativeVelocity) - targetVelocity);
        result.primaryIndex = bestSegment->primaryIndex;
        result.relativePosition = relative;

        // The target, in ITS OWN orbit's frame, so the marker lands on the
        // ring the map has drawn rather than out in the world.
        if (target.primaryIndex >= 0)
        {
            result.targetPrimaryIndex = target.primaryIndex;
            result.targetRelativePosition =
                targetPosition - index.positionAt(target.primaryIndex, bestTime);
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
