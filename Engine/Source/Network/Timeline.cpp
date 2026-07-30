#include "Network/Timeline.hpp"

#include <algorithm>
#include <limits>

namespace sw::net
{
    void Timeline::push(TimelineEvent event)
    {
        if (event.stampSeconds < m_now)
        {
            // Already in our past. Nothing short of rewinding the simulation
            // could place it correctly, so it lands now and is counted.
            ++m_late;
        }

        // Descending by stamp: the next one due sits at the back.
        const auto position =
            std::lower_bound(m_pending.begin(), m_pending.end(), event.stampSeconds,
                             [](const TimelineEvent& entry, f64 stamp) {
                                 return entry.stampSeconds > stamp;
                             });
        m_pending.insert(position, std::move(event));
    }

    std::span<const TimelineEvent> Timeline::advance(f64 nowSeconds)
    {
        m_now = std::max(m_now, nowSeconds);
        m_due.clear();
        while (!m_pending.empty() && m_pending.back().stampSeconds <= nowSeconds)
        {
            m_due.push_back(std::move(m_pending.back()));
            m_pending.pop_back();
            ++m_released;
        }
        return m_due;
    }

    f64 Timeline::nextStampSeconds() const
    {
        return m_pending.empty() ? std::numeric_limits<f64>::infinity()
                                 : m_pending.back().stampSeconds;
    }

    void Timeline::clear()
    {
        m_pending.clear();
        m_due.clear();
        m_now = -std::numeric_limits<f64>::infinity();
    }
} // namespace sw::net
