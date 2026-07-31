#include "Space/CelestialSystems.hpp"

#include "ECS/World.hpp"
#include "Simulation/Simulation.hpp"

namespace sw::space
{
    void CelestialMotionSystem::update(ecs::World& world, f32 /*deltaSeconds*/)
    {
        m_index.rebuild(world);
        const f64 time = m_timebase.presentSeconds();

        world.forEach<TransformComponent, PreviousTransformComponent,
                      phys::GravitySourceComponent, CelestialBodyComponent>(
            [this, time](ecs::Entity entity, TransformComponent& transform,
                         PreviousTransformComponent& previous,
                         phys::GravitySourceComponent& source, CelestialBodyComponent&) {
                const i32 index = m_index.indexOf(entity);
                if (index < 0)
                {
                    return;
                }

                WorldVec3 position{};
                WorldVec3 velocity{};
                m_index.stateAt(index, time, position, &velocity);

                // Real interpolation: previous = last tick's position. The
                // rotation snapshot belongs to the spin system.
                previous.position = transform.position;
                transform.position = position;
                source.worldVelocity = velocity;
            });
    }
} // namespace sw::space
