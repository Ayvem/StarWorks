#pragma once

// ============================================================================
// Network/Timeline.hpp
// THE RULE THAT MAKES TIME WARP SURVIVE MULTIPLAYER.
//
// In this game every player owns their own clock. One of them engages warp
// and walks off into the future; nobody else is dragged along, because
// nobody else asked to be. That is the whole point of warp — you use it
// because YOU are waiting for an apoapsis, and the person landing a rocket
// two hundred kilometres away is not.
//
// So the worlds do not share an instant. What they share is a TIMELINE.
//
// An action does not happen "now", it happens AT AN INSTANT, and it carries
// that instant with it. A player three hours ahead who separates a stage
// stamps the event with the simulation second it occurred at, and every
// other player holds it — unopened — until their own clock reaches that
// second. Then it happens, for them, at exactly the moment it happened for
// him. Nobody's world is rewritten and nobody's world is ahead of itself.
//
// This is what makes the model coherent rather than merely permissive: warp
// is not a licence to desynchronise, it is a licence to run AHEAD along a
// line everyone else will walk later.
//
// The one case the rule cannot cover is an event that arrives already
// stamped in the local past — a player BEHIND you acting, or a very late
// packet. There is no honest answer there short of rewinding the
// simulation, so the Timeline releases it at once and counts it. The count
// is the diagnostic: if it is large, players are interacting across a gap
// they should have closed first.
// ============================================================================

#include "Core/Types.hpp"

#include <limits>
#include <span>
#include <vector>

namespace sw::net
{
    /// A stamped action. `kind` is a game-level vocabulary; the Timeline
    /// never looks inside `payload`.
    struct TimelineEvent
    {
        /// The SIMULATION second the action happened at, on the session's
        /// shared clock — not wall-clock, and not the receiver's clock.
        f64 stampSeconds = 0.0;
        u32 originClientId = 0;
        u32 kind = 0;
        std::vector<u8> payload;
    };

    class Timeline
    {
    public:
        /// Takes an event. Order of arrival does not matter.
        void push(TimelineEvent event);

        /// Releases everything stamped at or before `nowSeconds`, in stamp
        /// order, and returns it. The result is valid until the next call.
        [[nodiscard]] std::span<const TimelineEvent> advance(f64 nowSeconds);

        /// How many events are still waiting for their instant.
        [[nodiscard]] usize pendingCount() const { return m_pending.size(); }
        /// The instant of the earliest event still waiting; +infinity when
        /// there is none. A UI can turn this into "next event in 4 min".
        [[nodiscard]] f64 nextStampSeconds() const;

        /// Events that arrived already stamped in the local past and were
        /// therefore released immediately. See the header note.
        [[nodiscard]] u64 lateCount() const { return m_late; }
        [[nodiscard]] u64 releasedCount() const { return m_released; }

        void clear();

    private:
        /// Kept sorted by stamp, EARLIEST LAST, so releasing is a pop_back
        /// loop rather than an erase from the front. Insertion is a linear
        /// scan, which is right for a queue that holds a handful of events
        /// and is written far less often than it is read.
        std::vector<TimelineEvent> m_pending;
        std::vector<TimelineEvent> m_due;
        f64 m_now = -std::numeric_limits<f64>::infinity();
        u64 m_late = 0;
        u64 m_released = 0;
    };
} // namespace sw::net
