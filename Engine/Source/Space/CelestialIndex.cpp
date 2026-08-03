#include "Space/CelestialIndex.hpp"

#include "ECS/World.hpp"
#include "Physics/PhysicsComponents.hpp"
#include "Scene/TransformComponents.hpp"

#include <cstring>

namespace sw::space
{
    void CelestialIndex::rebuild(ecs::World& world)
    {
        m_bodies.clear();
        m_children.clear();

        // Gather raw records first (unsorted).
        struct Record
        {
            ecs::Entity entity;
            ecs::Entity parent;
            Body body;
        };
        std::vector<Record> records;
        world.forEach<TransformComponent, phys::GravitySourceComponent,
                      CelestialBodyComponent>(
            [&records](ecs::Entity entity, TransformComponent& transform,
                       phys::GravitySourceComponent& source,
                       CelestialBodyComponent& celestial) {
                Record record{};
                record.entity = entity;
                record.parent = celestial.parent;
                record.body.entity = entity;
                record.body.mu = source.mu;
                record.body.bodyRadius = source.bodyRadius;
                record.body.soiRadius = source.soiRadius;
                record.body.hasOrbit = celestial.hasOrbit;
                record.body.orbit = celestial.orbit;
                record.body.staticPosition = transform.position;
                std::memcpy(record.body.name, celestial.name, sizeof(record.body.name));
                records.push_back(record);
            });

        // Topological insertion: parents before children. A body whose
        // parent never resolves (destroyed, or not a celestial) degrades to
        // a static root at its current position — the simulation keeps
        // running instead of dereferencing a dead handle.
        std::vector<bool> placed(records.size(), false);
        usize placedCount = 0;
        while (placedCount < records.size())
        {
            bool progress = false;
            for (usize i = 0; i < records.size(); ++i)
            {
                if (placed[i])
                {
                    continue;
                }
                i32 parentIndex = -1;
                if (!records[i].parent.isNull())
                {
                    parentIndex = indexOf(records[i].parent);
                    if (parentIndex < 0)
                    {
                        continue; // parent not placed yet (or missing)
                    }
                }
                Body body = records[i].body;
                body.parentIndex = parentIndex;
                m_bodies.push_back(body);
                placed[i] = true;
                ++placedCount;
                progress = true;
            }
            if (!progress)
            {
                // Remaining bodies have unresolvable parents: root them.
                for (usize i = 0; i < records.size(); ++i)
                {
                    if (!placed[i])
                    {
                        Body body = records[i].body;
                        body.parentIndex = -1;
                        body.hasOrbit = 0;
                        m_bodies.push_back(body);
                        placed[i] = true;
                        ++placedCount;
                    }
                }
            }
        }

        m_children.resize(m_bodies.size());
        for (usize i = 0; i < m_bodies.size(); ++i)
        {
            if (m_bodies[i].parentIndex >= 0)
            {
                m_children[static_cast<usize>(m_bodies[i].parentIndex)].push_back(
                    static_cast<i32>(i));
            }
        }
    }

    i32 CelestialIndex::indexOf(ecs::Entity entity) const
    {
        for (usize i = 0; i < m_bodies.size(); ++i)
        {
            if (m_bodies[i].entity == entity)
            {
                return static_cast<i32>(i);
            }
        }
        return -1;
    }

    void CelestialIndex::stateAt(i32 index, f64 timeSeconds, WorldVec3& outPosition,
                                 WorldVec3* outVelocity) const
    {
        const Body& body = m_bodies[static_cast<usize>(index)];
        if (body.hasOrbit == 0)
        {
            outPosition = body.staticPosition;
            if (outVelocity != nullptr)
            {
                *outVelocity = {0.0, 0.0, 0.0};
            }
            return;
        }

        WorldVec3 parentPosition{0.0};
        WorldVec3 parentVelocity{0.0};
        if (body.parentIndex >= 0)
        {
            stateAt(body.parentIndex, timeSeconds, parentPosition,
                    (outVelocity != nullptr) ? &parentVelocity : nullptr);
        }

        WorldVec3 relativePosition{};
        WorldVec3 relativeVelocity{};
        phys::kepler::evaluate(body.orbit, timeSeconds, relativePosition,
                               (outVelocity != nullptr) ? &relativeVelocity : nullptr);
        outPosition = parentPosition + relativePosition;
        if (outVelocity != nullptr)
        {
            *outVelocity = parentVelocity + relativeVelocity;
        }
    }

    void CelestialIndex::stateAtSplit(i32 index, f64 wholeSeconds, f64 fraction,
                                      WorldVec3& outPosition,
                                      WorldVec3* outVelocity) const
    {
        // THE WHOLE CHAIN AT FULL PRECISION, and it has to be the whole chain:
        // Luna's position is Terra's plus its own, so evaluating the moon
        // exactly on top of a parent that was evaluated at a rounded time
        // gives the moon the parent's noise. The recursion carries the split
        // down to every link.
        const Body& body = m_bodies[static_cast<usize>(index)];
        if (body.hasOrbit == 0)
        {
            outPosition = body.staticPosition;
            if (outVelocity != nullptr)
            {
                *outVelocity = {0.0, 0.0, 0.0};
            }
            return;
        }

        WorldVec3 parentPosition{0.0};
        WorldVec3 parentVelocity{0.0};
        if (body.parentIndex >= 0)
        {
            stateAtSplit(body.parentIndex, wholeSeconds, fraction, parentPosition,
                         (outVelocity != nullptr) ? &parentVelocity : nullptr);
        }

        WorldVec3 relativePosition{};
        WorldVec3 relativeVelocity{};
        phys::kepler::evaluateSplit(body.orbit, wholeSeconds, fraction, relativePosition,
                                    (outVelocity != nullptr) ? &relativeVelocity : nullptr);
        outPosition = parentPosition + relativePosition;
        if (outVelocity != nullptr)
        {
            *outVelocity = parentVelocity + relativeVelocity;
        }
    }

    WorldVec3 CelestialIndex::positionAt(i32 index, f64 timeSeconds) const
    {
        WorldVec3 position{};
        stateAt(index, timeSeconds, position);
        return position;
    }

    i32 CelestialIndex::soiPrimaryAt(const WorldVec3& worldPosition,
                                     f64 timeSeconds) const
    {
        i32 best = -1;
        f64 bestSoi = 0.0;
        for (usize i = 0; i < m_bodies.size(); ++i)
        {
            const Body& body = m_bodies[i];
            const WorldVec3 delta = worldPosition - positionAt(static_cast<i32>(i),
                                                               timeSeconds);
            const f64 distanceSq = glm::dot(delta, delta);
            if (distanceSq < body.soiRadius * body.soiRadius &&
                (best < 0 || body.soiRadius < bestSoi))
            {
                best = static_cast<i32>(i);
                bestSoi = body.soiRadius;
            }
        }
        return best;
    }
} // namespace sw::space
