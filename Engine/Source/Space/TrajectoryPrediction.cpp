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

            // ---- coarse forward scan for the first event -----------------
            const f64 window = horizon - segmentStart;
            const f64 step = window / static_cast<f64>(settings.samplesPerSegment);
            segment.endTime = horizon;
            segment.endReason = SegmentEnd::Horizon;

            f64 previousTime = segmentStart;
            for (u32 sample = 1; sample <= settings.samplesPerSegment; ++sample)
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
